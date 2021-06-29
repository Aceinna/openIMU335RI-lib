/** ***************************************************************************
 * @file hwAPI.h
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *****************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/

#ifndef HAL_API_H
#define HAL_API_H

#ifdef __cplusplus
 extern "C" {
#endif


#include "stdint.h"
#include "GlobalConstants.h"
#include "ecu.h"

/**********************************************
* @brief
*
***********************************************/
void    HW_Init();

// GPIO - related fucntions
/**********************************************
* @brief
*
***********************************************/
void    HW_TP1_Toggle();

/**********************************************
* @brief
*
***********************************************/
void    HW_TP2_Toggle();

/**********************************************
* @brief
*
***********************************************/
void    HW_TP1_On();

/**********************************************
* @brief
*
***********************************************/
void    HW_TP2_On();

/**********************************************
* @brief
*
***********************************************/
void    HW_TP1_Off();

/**********************************************
* @brief
*
***********************************************/
void    HW_TP2_Off();

/**********************************************
* @brief
*
***********************************************/
void    HW_EnableCANTransceiver();

/**********************************************
* @brief
*
***********************************************/
void    HW_DisableCANTransceiver();

/**********************************************
* @brief
*
***********************************************/
void    HW_ConfigureCANInterface();

/**********************************************
* @brief
*
***********************************************/
void    HW_InitWatchdod();

/**********************************************
* @brief
*
***********************************************/
void    HW_FeedWatchdog();


// UART - related functions
/**********************************************
* @brief
*
* @param channel --
* @param baudrate --
* @return int32_t
***********************************************/
int32_t     UART_Init(int32_t channel, uint32_t baudrate);

/**********************************************
* @brief
*
* @param channel --
* @param data --
* @param len --
* @return int32_t
***********************************************/
int32_t     UART_Read(int32_t channel, uint8_t *data, uint32_t len);

/**********************************************
* @brief
*
* @param channel --
* @param data --
* @param len --
* @return int32_t
***********************************************/
int32_t     UART_Write(int32_t channel, uint8_t *data, uint32_t  len);

// CAN - related functions
/**********************************************
* @brief
*
* @param baudRate --
***********************************************/
void    CAN_InitCommunication(int32_t baudRate);

/**********************************************
* @brief
*
* @param txCallback --
* @param rxCallback --
***********************************************/
void    CAN_ConfigureCallbacks(void (*txCallback)(void), void (*rxCallback)(void));

/**********************************************
* @brief
*
* @param baseID --
* @param baseMask --
***********************************************/
void    CAN_ConfigureMessageFilter(uint32_t baseID, uint32_t baseMask, BOOL isExtended);

/**********************************************
* @brief
*
* @param enable ---
***********************************************/
void    CAN_EnableDummyFilter(BOOL enable);

/**********************************************
* @brief
*
* @param rate --
* @return uint8_t
***********************************************/
uint8_t     CAN_SwitchToNewBaudrate(uint8_t rate);

/**********************************************
* @brief
*
* @return BOOL
***********************************************/
BOOL    CAN_IsErrorDetected();

/**********************************************
* @brief
*
* @return uint8_t
***********************************************/
uint8_t CAN_GetLastError();

/**********************************************
* @brief
*
* @return uint8_t
***********************************************/
uint8_t CAN_GetTxErrorCounter();

/**********************************************
* @brief
*
* @return uint8_t
***********************************************/
uint8_t CAN_GetRxErrorCounter();

/**********************************************
* @brief
*
* @param desc --
* @return BOOL
***********************************************/
BOOL    CAN_SendPacket(struct ecu_tx_desc* desc);

/**********************************************
* @brief
*
* @return uint8_t
***********************************************/
uint8_t CAN_GetExtId();

/**********************************************
* @brief
*
* @return uint8_t
***********************************************/
uint8_t CAN_GetDataRtr();


/**********************************************
* @brief
*
* @return uint8_t
***********************************************/
uint8_t   CAN_GetDetectedError();


// Timers -related functions
/**********************************************
* @brief
*
* @return BOOL
***********************************************/
BOOL        TIMER_IsDacqOverrun();

/**********************************************
* @brief
*
* @return int32_t
***********************************************/
int32_t     TIMER_WaitForNewDacqTick();


/**********************************************
* @brief
*
* @return uint64_t
***********************************************/
uint64_t TIMER_GetCurrTimeStamp();

/**********************************************
* @brief
*
* @return uint64_t
***********************************************/
uint64_t TIMER_GetDacqTimeStamp();

/**********************************************
* @brief
*
* @param interval --
***********************************************/
void     TIMER_SetDacqInterval(uint32_t interval);

/**********************************************
* @brief
*
***********************************************/
void     TIMER_InitOSTick();

/**********************************************
* @brief
*
* @param Delay --
***********************************************/
void     TIMER_DelayMs(uint32_t Delay);


/**********************************************
* @brief
*
* @return uint16_t
***********************************************/
uint16_t TIMER_GetRollingCount();

/*******************************************
 * @brief
 *
********************************************/
void  TIMER_Init();


/*******************************************
 * @brief 
 * 
********************************************/
void    TIMER_DacqStarted();

/*******************************************
 * @brief 
 * 
********************************************/
void TIMER_GetSampleCounts();



// system related functions
/**********************************************
* @brief
*
***********************************************/
void    HW_SystemReset(void);


/**********************************************
* @brief
*
* @param SA --
* @param addr --
* @param baudrate --
* @param proceed --
***********************************************/
void    HW_EnforceCanBootMode(uint8_t SA, uint8_t addr, uint8_t baudrate, uint8_t proceed);

/**********************************************
* @brief
*
***********************************************/
void    HW_EnforceSerialBootMode();


/**********************************************
* @brief
*
* @return uint32_t
***********************************************/
uint32_t HW_GetResetRootCause(void);

/**********************************************
* @brief
*
***********************************************/
void    HW_FillStackPattern(void);

/*******************************************
 * @brief 
 * 
 * @return uint8_t 
********************************************/
uint8_t HW_GetEcuFunctionInstance(void);


/**********************************************
* @brief
*
* @return BOOL
***********************************************/
BOOL    HW_IsStackOverflow(void);

/**********************************************
* @brief
*
* @param cause ---
***********************************************/
void    HW_SetResetRootCause(uint8_t cause);

enum {
    NUM_SERIAL_PORTS = 1
};

/*******************************************
 * @brief
 *
********************************************/
void HW_InitADC();

/*******************************************
 * @brief
 *
********************************************/
void HW_StartADC();


/*******************************************
 * @brief
 *
 * @return BOOL
********************************************/
BOOL HW_IsADCConversionComplete();

/*******************************************
 * @brief
 *
 * @param voltages ==
********************************************/
void    HW_GetADCReadings(float32_t voltages[]);

/*******************************************
 * @brief
 *
 * @return uint8_t
********************************************/
uint8_t HW_ReadUnitHwConfiguration( void );


/*******************************************
 * @brief
 *
 * @param i ==
 * @param voltAvg ==
********************************************/
void    ADC_SetVoltCounts(uint16_t i, float32_t voltAvg);

/*******************************************
 * @brief
 *
 * @param channel ==
 * @return int16_t
********************************************/
int16_t     ADC_GetChannelReadings(int32_t channel);



static int32_t const USER_SERIAL_PORT    =  0 ;
static int32_t const INVALID_SERIAL_PORT = -1 ;

extern int16_t ucbPort;
extern int32_t debugPort;
extern uint32_t	const _ebss;
extern uint32_t const _Min_Heap_Size;
extern uint32_t const _estack;

enum{
    ADC_CHANNEL_1 = 0,
    ADC_CHANNEL_2 = 1,
    ADC_CHANNEL_3 = 2,
    ADC_CHANNEL_4 = 3,
    ADC_CHANNEL_5 = 4,
    ADC_CHANNEL_6 = 5,
    ADC_CHANNEL_7 = 6,
};


#ifdef __cplusplus
}
#endif


#endif //__UART_H
