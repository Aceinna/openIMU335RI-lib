/** ***************************************************************************
 * @file   configureGPIO.c
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * Initialize the GPIO pins to output for the 380 board
 ******************************************************************************/
#include "configureGPIO.h"
#include "halAPI.h"


/**********************************************
* @brief
*
***********************************************/
static void InitUnitConfigurationPins( void )
{
    InitPin_GPIO( CONFIG_0_CLK, PORT_A, CONFIG_0_PIN, GPIO_INPUT );
    InitPin_GPIO( CONFIG_1_CLK, PORT_A, CONFIG_1_PIN, GPIO_INPUT );
}

/**********************************************
* @brief
*
***********************************************/
static void InitUnitTestPins( void )
{
    InitPin_GPIO( TP1_CLK, PORT_A, TP1_PIN, GPIO_OUTPUT );
    InitPin_GPIO( TP2_CLK, PORT_A, TP2_PIN, GPIO_OUTPUT );
}

#ifdef CAN_BUS_COMM

/**********************************************
* @brief
*
***********************************************/
void HW_DisableCANTransceiver()
{
#ifndef MTLT335_V1
    GPIO_WriteBit(PORT_B, CAN_EN_CTRL_PIN, Bit_SET);
#else
    GPIO_WriteBit(PORT_A, CAN_AB_CTRL_PIN, Bit_SET);
#endif
}

/**********************************************
* @brief
*
***********************************************/
void HW_EnableCANTransceiver()
{
#ifndef MTLT335_V1
    GPIO_WriteBit(PORT_B, CAN_EN_CTRL_PIN, Bit_RESET);
#else
    GPIO_WriteBit(PORT_A, CAN_AB_CTRL_PIN, Bit_RESET);
#endif
}

/**********************************************
* @brief
*
***********************************************/
void HW_ConfigureCANInterface()
{
    
#ifndef MTLT335_V1
    InitPin_GPIO( CAN_EN_CTRL_CLK,
                  PORT_B,
                  CAN_EN_CTRL_PIN,
                  GPIO_OUTPUT );

    GPIO_WriteBit(PORT_B, CAN_EN_CTRL_PIN, Bit_RESET);
#else
    InitPin_GPIO( CAN_AB_CTRL_CLK,
                  PORT_A,
                  CAN_AB_CTRL_PIN,
                  GPIO_OUTPUT );

    GPIO_WriteBit(PORT_A, CAN_AB_CTRL_PIN, Bit_RESET);
#endif

}
#endif

/**********************************************
* @brief
*
***********************************************/
void InitBoardConfiguration_GPIO()
{
    InitUnitConfigurationPins(); // as inputs (logic levels on pins read later)
    InitUnitTestPins(); // as inputs (logic levels on pins read later)
}

/**********************************************
* @brief
*
* @return uint8_t --
***********************************************/
uint8_t HW_ReadUnitHwConfiguration( void )
{
    uint8_t tmp = 0x00U;
    uint8_t bit0;
    uint8_t bit1;

    bit0 = GPIO_ReadInputDataBit( PORT_A, CONFIG_0_PIN );
    bit1 = GPIO_ReadInputDataBit( PORT_B, CONFIG_1_PIN );

    tmp = (uint8_t)(bit0 << 0U) | (uint8_t)(bit1 << 1U);

    return tmp;
}

/**********************************************
* @brief
*
* @param PeriphClock --
* @param GPIO_Port --
* @param GPIO_Pin --
* @param inputOutputSelector --
***********************************************/
void InitPin_GPIO( uint32_t    const  PeriphClock,
                   GPIO_TypeDef* const GPIO_Port,
                   uint32_t     const GPIO_Pin,
                   uint8_t      const inputOutputSelector )
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Set up the peripheral clocks
    RCC_AHB1PeriphClockCmd( PeriphClock, ENABLE );

    if( inputOutputSelector == (uint8_t)GPIO_INPUT ) {
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;  /// INPUT/output/alt func/analog
    } else {
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT; /// input/OUTPUT/alt func/analog
    }

    /// Configure the pins as Push-Pull with a Pull-Up resistor
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    /// PUSH-PULL or open-drain
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;     /// UP/Down/NoPull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; /// low/med/fast/high speed

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
    GPIO_Init( GPIO_Port, &GPIO_InitStructure);
}

/*******************************************
 * @brief 
 *
 * @param PeriphClock ==
 * @param GPIO_Port ==
 * @param GPIO_Pin ==
********************************************/
void InitPin_GPIO_ANALOG( uint32_t      const PeriphClock,
                          GPIO_TypeDef* const GPIO_Port,
                          uint32_t      const GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Set up the peripheral clocks
    RCC_AHB1PeriphClockCmd( PeriphClock, ENABLE );

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;        /// analog mode

    /// Configure the pins as Push-Pull with a Pull-Up resistor
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;       /// PUSH-PULL or open-drain
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;      /// UP/Down/NoPull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    /// low/med/fast/high speed

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
    GPIO_Init( GPIO_Port, &GPIO_InitStructure);
}



/**********************************************
* @brief
*
***********************************************/
void ControlPortInit(void)
{
   	GPIO_InitTypeDef GPIO_InitStructure;
    // Set up the peripheral clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    // PD2 for 120Ohm switch control
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PORT_D, &GPIO_InitStructure);
}


/**********************************************
* @brief
*
***********************************************/
void GPIO_InitSpiBitBangInterface()
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStruct.GPIO_Pin   = (uint32_t)SPI_BITBANG_MISO_PINS;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PORT_A, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin   = (uint32_t)(SPI_BITBANG_MOSI_PIN | SPI_BITBANG_SCK_PIN);
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PORT_B, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin   = (uint32_t)SPI_BITBANG_NSS_PINS;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PORT_B, &GPIO_InitStruct);
}

/**********************************************
* @brief
*
***********************************************/
void HW_TP1_On()
{
    GPIO_WriteBit(PORT_A, TP1_PIN, Bit_SET);
}

/**********************************************
* @brief
*
***********************************************/
void HW_TP1_Off()
{
    GPIO_WriteBit(PORT_A, TP1_PIN, Bit_RESET);
}

/**********************************************
* @brief
*
***********************************************/
void HW_TP2_On()
{
    GPIO_WriteBit(PORT_A, TP2_PIN, Bit_SET);
}

/**********************************************
* @brief
*
***********************************************/
void HW_TP2_Off()
{
    GPIO_WriteBit(PORT_A, TP2_PIN, Bit_RESET);
}

/**********************************************
* @brief
*
***********************************************/
void HW_TP1_Toggle()
{
    GPIO_ToggleBits(PORT_A, TP1_PIN);
}

/**********************************************
* @brief
*
***********************************************/
void HW_TP2_Toggle()
{
    GPIO_ToggleBits(PORT_A, TP2_PIN);
}
