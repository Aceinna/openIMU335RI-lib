/*****************************************************************************
 * @file can.c the definitions of basic functions
 * @Author Feng
 * @date   May 2017
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#if defined(DBC_FILE) || defined(SAE_J1939)
#include <stdlib.h>

#include "GlobalConstants.h"
#include "can.h"
#include "stm32f4xx.h"
#include "boardDefinition.h"
#include "osapi.h"
#include "ecu.h"
#include "halAPI.h"

static void (*gCANTxCompleteCallback)(void);    // callback function pointer of CAN transmit
static void (*gCANRxCompleteCallback)(void);    // callback function pointer of CAN receive

static          CAN_FilterInitTypeDef FILTER_InitStructure;
static uint8_t      lec     = 0U;
static uint32_t     canStat = 0U;
static CAN_TypeDef  *pCAN1;
static uint32_t     filterNum  = 1U;

/** ***************************************************************************
 * @name CAN_Set_BTR() Set up bit timing
 * @brief Provides a function to set bit timing according to the expected baud rate
 *
 * @param br [in] baud rate
 * @param CAN_InitStructure [in] CAN_InitStructure CAN init structure definition
 *
 ******************************************************************************/
static void CAN_Set_BTR(ECU_BAUD_RATE const br, CAN_InitTypeDef* const CAN_InitStructure)
{
    canStat++;

    switch (br)
    {
      // 500Kbps, time quantum 10, segment1 4, segment2 1.
    case ECU_BAUD_500K:
        CAN_InitStructure->CAN_Prescaler = 6U;
        CAN_InitStructure->CAN_BS1 = CAN_BS1_6tq;
        CAN_InitStructure->CAN_BS2 = CAN_BS2_3tq;
        break;
      // 250Kbps, time quantum 24, segment1 3, segment2 1.
    case ECU_BAUD_250K:
        CAN_InitStructure->CAN_Prescaler = 12U;
        CAN_InitStructure->CAN_BS1 = CAN_BS1_6tq;
        CAN_InitStructure->CAN_BS2 = CAN_BS2_3tq;
        break;
      // 125Kbps, time quantum 24, segment1 7, segment2 2.
    case ECU_BAUD_125K:
        CAN_InitStructure->CAN_Prescaler = 24U;
        CAN_InitStructure->CAN_BS1 = CAN_BS1_6tq;
        CAN_InitStructure->CAN_BS2 = CAN_BS2_3tq;
        break;
      // 1000Kbps, time quantum 2, segment1 7, segment2 2.
    case ECU_BAUD_1000K:
        CAN_InitStructure->CAN_Prescaler = 3U;
        CAN_InitStructure->CAN_BS1 = CAN_BS1_6tq;
        CAN_InitStructure->CAN_BS2 = CAN_BS2_3tq;
        break;
      default:
        break;
  }

  return;
}

/**********************************************
* @brief
*
* @param mode --
* @param baudRate --
***********************************************/
static void CAN_InitEngine(uint8_t const mode, ECU_BAUD_RATE const baudRate)
{
    CAN_InitTypeDef  CAN_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    CAN_DeInit(pCAN1);

    CAN_StructInit(&CAN_InitStructure);

    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_AWUM = ENABLE;
    CAN_InitStructure.CAN_TXFP = ENABLE;
    CAN_InitStructure.CAN_Mode = mode;

    CAN_InitStructure.CAN_SJW = 0U;
    // set to default value, 250kbps, if out of supported range
    if (baudRate > ECU_BAUD_1000K){
        CAN_Set_BTR(ECU_BAUD_250K, &CAN_InitStructure);
    }
    else{
        CAN_Set_BTR(baudRate, &CAN_InitStructure);
    }

        // initialize CAN1 clock
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

        // initialize CAN1 tx/rx clock
        RCC_AHB1PeriphClockCmd(CAN1_RX_GPIO_CLK, ENABLE);
        RCC_AHB1PeriphClockCmd(CAN1_TX_GPIO_CLK, ENABLE);

        // configure CAN1 tx/rx pins
        GPIO_PinAFConfig(PORT_B, CAN1_RX_SOURCE, GPIO_AF_CAN1);
        GPIO_PinAFConfig(PORT_B, CAN1_TX_SOURCE, GPIO_AF_CAN1);

        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

        GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN;
        GPIO_Init(PORT_B, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = CAN1_TX_PIN;
        GPIO_Init(PORT_B, &GPIO_InitStructure);

        canStat += CAN_Init(pCAN1, &CAN_InitStructure);
         /// Enable the CAN1 global interrupt, CAN1_TX_IRQn
         NVIC_InitStructure.NVIC_IRQChannel                   = (uint8_t)CAN1_TX_IRQn;
         NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x9U;
         NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0U;
         NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

         /// Enable the CAN1 global interrupt, CAN1_RX0_IRQn
         NVIC_InitStructure.NVIC_IRQChannel                   = (uint8_t)CAN1_RX0_IRQn;
         NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xAU;
         NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x1U;
         NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

         // Enable the CAN1 global interrupt, CAN2_RX1_IRQn
         NVIC_InitStructure.NVIC_IRQChannel                   = (uint8_t)CAN1_RX1_IRQn;
         NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xAU;
         NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x2U;
         NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        CAN_DBGFreeze(pCAN1, DISABLE);

}

/*******************************************
 * @brief
 *
********************************************/
static void CAN_Init_Filter_Config_Structure(void)
{
  // reinitialize filter number if unit engine restarts   
  filterNum  = 1U;
  // initialize common parameters
  FILTER_InitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
  FILTER_InitStructure.CAN_FilterMode           = CAN_FilterMode_IdMask;
  FILTER_InitStructure.CAN_FilterScale          = CAN_FilterScale_32bit;
  FILTER_InitStructure.CAN_FilterActivation     = ENABLE;
}

/** ***************************************************************************
 * @name _CAN_Init_IT() interrupt initialization
 * @brief Set interrupts' bits on CAN interface
 *
 * @param baseID [in]
 * @param baseMask [in]
 *
 ******************************************************************************/
void CAN_ConfigureMessageFilter(uint32_t const baseID, uint32_t const baseMask, BOOL isExtended)
{
    if(filterNum == 28){
		while(1);
    }

    uint32_t mask;
    uint32_t filtr;
    uint32_t filtr0;
    uint32_t filtr1;
  	// initialize filter ECU ID
	  uint32_t ExtID = isExtended ? USER_CAN_IDE : 0U;
    if(!isExtended){
      FILTER_InitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
      FILTER_InitStructure.CAN_FilterScale = CAN_FilterScale_16bit;
      filtr0 = ((baseID & 0xFFFFU) << 5U) | ((uint32_t)ExtID << 3U) | ((uint32_t)USER_CAN_RTR << 4U);
      filtr1 = (((baseID & 0xFFFF0000U)>>16) << 5U) | ((uint32_t)ExtID << 3U) | ((uint32_t)USER_CAN_RTR << 4U);
      FILTER_InitStructure.CAN_FilterIdLow = (uint16_t)(filtr0 & 0xFFFFU);
      FILTER_InitStructure.CAN_FilterIdHigh = (uint16_t)(filtr1 & 0xFFFFU);
      FILTER_InitStructure.CAN_FilterMaskIdLow = (uint16_t)(0x0000);
      FILTER_InitStructure.CAN_FilterMaskIdHigh = (uint16_t)(0x0000);
      // mask  = ((baseMask & 0xFFFF) << 5U) | ((uint32_t)ExtID << 3U) | ((uint32_t)USER_CAN_RTR << 4U);
    }else{
      filtr = (baseID << 3U) | ((uint32_t)ExtID << 2U) | ((uint32_t)USER_CAN_RTR << 1U);
      mask  = (baseMask << 3U) | ((uint32_t)ExtID << 2U) | ((uint32_t)USER_CAN_RTR << 1U);
      FILTER_InitStructure.CAN_FilterIdHigh     =  (uint16_t)(filtr >> 16U);
      FILTER_InitStructure.CAN_FilterIdLow      =  (uint16_t)(filtr & 0xFFFFU);
      FILTER_InitStructure.CAN_FilterMaskIdHigh =  (uint16_t)(mask >> 16U);
      FILTER_InitStructure.CAN_FilterMaskIdLow  =  (uint16_t)(mask & 0xFFFFU);
    }
	  FILTER_InitStructure.CAN_FilterNumber     =  (uint8_t)filterNum;
    filterNum++;
  CAN_FilterInit(&FILTER_InitStructure);
}

/** ***************************************************************************
 * @name _CAN_Init_IT() interrupt initialization
 * @brief Set interrupts' bits on CAN interface
 *
 *
 ******************************************************************************/
static void CAN_Activate_Filters()
{
  CAN_SlaveStartBank(28U);
}


/** ***************************************************************************
 * @name _CAN_Init_IT() interrupt initialization
 * @brief Set interrupts' bits on CAN interface
 *
 * @param CANx [in] , where x can be 1 or 2 to select the CAN peripheral.
 *
 ******************************************************************************/
static void CAN_Init_IT(CAN_TypeDef* const CANx)
{
  uint32_t int_bits = 0U;

  int_bits = CAN_IT_TME | CAN_IT_FMP0 | CAN_IT_FF0 | CAN_IT_FOV0 | CAN_IT_FMP1 | CAN_IT_FF1 | CAN_IT_FOV1;

  CAN_ITConfig(CANx, int_bits, ENABLE);

}

/** ***************************************************************************
 * @name CAN1_TX_IRQHandler() CAN1 transmit IRQ handler
 * @brief Handle transmitting interrupt of CAN1
 *
 *
 ******************************************************************************/
void CAN1_TX_IRQHandler(void)
{
  ITStatus ItRslt;

  OSEnterISR();

  ItRslt = CAN_GetITStatus(pCAN1, CAN_IT_TME);

    if (ItRslt == SET)
    {
    gCANTxCompleteCallback();
    CAN_ClearITPendingBit(pCAN1, CAN_IT_TME);
  }

  OSExitISR();

  return;
}

/** ***************************************************************************
 * @name CAN1_RX0_IRQHandler CAN1 receive buffer 0 IRQ handler
 * @brief Handle receiving interrupt of CAN1 buffer 0
 *
 *
 ******************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
  ITStatus ItRslt;
	OSEnterISR();

  ItRslt =  CAN_GetITStatus(pCAN1, CAN_IT_FMP0);
    if (ItRslt == SET)
    {
        uint8_t fifoPending = CAN_MessagePending(pCAN1, 0U);
        while (fifoPending > 0U)
        {
            struct ecu_rx_desc *desc = ecu_get_rx_descriptor();
            lec = CAN_GetLastErrorCode(pCAN1);
            CAN_Receive(pCAN1, 0U, &(desc->rx_buffer));
            CAN_FIFORelease(pCAN1, 0U);
        gCANRxCompleteCallback();
        fifoPending--;
       };

        CAN_ClearITPendingBit(pCAN1, CAN_IT_FF0);
  }

	OSExitISR();

  return;
}

/** ***************************************************************************
 * @name CAN1_RX1_IRQHandler() CAN1 receive buffer 1 IRQ handler
 * @brief Handle receiving interrupt of CAN1 buffer 1
 *
 *
 ******************************************************************************/
void CAN1_RX1_IRQHandler(void)
{
#ifdef SAE_J1939
  ITStatus ItRslt;

	OSEnterISR();

    ItRslt =  CAN_GetITStatus(pCAN1, CAN_IT_FMP1);
    if (ItRslt == SET)
    {
        uint8_t fifoPending = CAN_MessagePending(pCAN1, 1U);
        while (fifoPending > 0U)
        {
            struct ecu_rx_desc *desc = ecu_get_rx_descriptor();
            lec = CAN_GetLastErrorCode(pCAN1);
            CAN_Receive(pCAN1, 1U, &(desc->rx_buffer));
            CAN_FIFORelease(pCAN1, 1U);
        gCANRxCompleteCallback();
        fifoPending--;
        } ;

        CAN_ClearITPendingBit(pCAN1, CAN_IT_FF1);
  }

	OSExitISR();
#endif

  return;
}

/** ***************************************************************************
 * @name InitCommunication_UserCAN() user interface, CAN, initialization
 * @brief Provide an API of CAN initialization to system
 *        initialize CAN1 of version 3rd PCB
 * @param baudRate [in]
 *
 ******************************************************************************/
void CAN_InitCommunication(int32_t const baudRate)
{
    pCAN1 = CAN_GetInstancePtr(CAN_1);


    HW_ConfigureCANInterface();

    CAN_InitEngine(CAN_Mode_Normal, (ECU_BAUD_RATE)baudRate);

    CAN_Init_Filter_Config_Structure();

    // configure CAN controller for selective reception of CAN messages
    ecu_configure_CAN_message_filters();

    CAN_Activate_Filters();

   CAN_Init_IT(pCAN1);

  return;
}


/**********************************************
* @brief
*
* @param txCallback --
* @param rxCallback --
***********************************************/
void CAN_ConfigureCallbacks(void (*txCallback)(void), void (*rxCallback)(void))
{
    gCANTxCompleteCallback = txCallback;
    gCANRxCompleteCallback = rxCallback;

    return;
}


/*******************************************
 * @brief
 *
 * @param enable ==
********************************************/
void CAN_EnableDummyFilter(BOOL const enable)
{
  CAN_FilterInitTypeDef FILTER_InitStruct;

  FILTER_InitStruct.CAN_FilterIdHigh         = 0x0000U;
  FILTER_InitStruct.CAN_FilterIdLow          = 0x0000U;
  FILTER_InitStruct.CAN_FilterMaskIdHigh     = 0x0000U;
  FILTER_InitStruct.CAN_FilterMaskIdLow      = 0x0000U;
  FILTER_InitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
  FILTER_InitStruct.CAN_FilterMode           = CAN_FilterMode_IdMask;
  FILTER_InitStruct.CAN_FilterScale          = CAN_FilterScale_32bit;
  if(enable){
    FILTER_InitStruct.CAN_FilterActivation   = ENABLE;
  }else{
    FILTER_InitStruct.CAN_FilterActivation     = DISABLE;
  }
  FILTER_InitStruct.CAN_FilterNumber         = filterNum;   // should be last one

  CAN_FilterInit(&FILTER_InitStruct);

  CAN_SlaveStartBank(28U);

  return;
}

/** ***************************************************************************
 * @brief
 * @param rate [in]
 * @return counter
 ******************************************************************************/
uint8_t CAN_SwitchToNewBaudrate(uint8_t rate)
{
    rate += 1U;
    if(rate == (uint8_t)ECU_BAUD_125K){
        rate += 1U;
    }

    if(rate == (uint8_t)ECU_BAUD_ERROR){
        rate = (uint8_t)ECU_BAUD_500K;
    }

    CAN_InitCommunication(rate);
    CAN_EnableDummyFilter(TRUE);

    return rate;
}

//#define DEBUG_CAN_ERRORS 1
/*******************************************
 * @brief
 *
 * @return uint8_t
********************************************/
uint8_t   CAN_GetDetectedError()
{
    return  CAN_GetLastErrorCode(pCAN1);
}

/** ***************************************************************************
 * @brief
 * @return TRUE id detected
 ******************************************************************************/
BOOL    CAN_IsErrorDetected()
{

#ifdef DEBUG_CAN_ERRORS
    static volatile uint8_t errTab[1024];
    static volatile uint8_t errCnt[1024];
    static volatile int32_t errTabIdx = 0U;
#endif
    volatile uint8_t const cntrRx  = CAN_GetReceiveErrorCounter(pCAN1);

    if(cntrRx){
        static uint8_t incCount = 0U;
        static uint8_t prevCnt = 0U;
        if(cntrRx != prevCnt){
            if(cntrRx > prevCnt){
                incCount++;
            }
#ifdef DEBUG_CAN_ERRORS
            uint8_t lec       = CAN_GetLastErrorCode(pCAN1);
            errTab[errTabIdx] = lec;
            errCnt[errTabIdx] = cntrRx;
            errTabIdx ++;
            errTabIdx &= 0x3FF;
#endif
            prevCnt    = cntrRx;
        }
        
        if((cntrRx > 20U) || (incCount > 3U)){
            prevCnt  = 0U;
            incCount = 0U;
            return TRUE;
        }
    }

    return FALSE;

}

/** ***************************************************************************
 * @brief
 * @return counter
 ******************************************************************************/
uint8_t CAN_GetLastError()
{
    return (uint8_t)(lec & ~CAN_ErrorCode_ACKErr);
}


/** ***************************************************************************
 * @brief
 * @return counter
 ******************************************************************************/
uint8_t CAN_GetTxErrorCounter()
{
    return CAN_GetLSBTransmitErrorCounter(pCAN1);
}


/**********************************************
* @brief
*
* @return uint8_t
***********************************************/
uint8_t CAN_GetRxErrorCounter()
{
    return CAN_GetReceiveErrorCounter(pCAN1);
}

/*******************************************
 * @brief
 *
 * @param desc ==
 * @return BOOL
********************************************/
BOOL CAN_SendPacket(struct ecu_tx_desc* const desc)
{
//  struct ecu_tx_desc* const desc = (struct ecu_tx_desc *)pDesc;

  CanTxMsg* TxMsg;
  uint8_t   result = 0U;

  // assign tx buffer
  TxMsg = &(desc->tx_buffer);

  // call transmitting API
  result = CAN_Transmit(pCAN1, TxMsg);

  if(result != CAN_TxStatus_NoMailBox){
      return TRUE;
  }

  return FALSE;

}

/**********************************************
* @brief
*
* @return uint8_t
***********************************************/
uint8_t CAN_GetExtId()
{
    return (uint8_t)CAN_ID_EXT;
}

/**********************************************
* @brief
*
* @return uint8_t
***********************************************/
uint8_t CAN_GetDataRtr()
{
    return (uint8_t)CAN_RTR_Data;
}


#endif
