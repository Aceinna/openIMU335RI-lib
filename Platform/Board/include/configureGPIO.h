/** ***************************************************************************
 * @file configureGPIO.h BSP call to set up GPIO pins
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef CONFIGURE_IO_H
#define CONFIGURE_IO_H

#ifdef __cplusplus
 extern "C" {
#endif


#include <stdint.h>

#include "stm32f4xx.h"
#include "boardDefinition.h"

enum {
    GPIO_INPUT  = 0,
    GPIO_OUTPUT = 1,
};

// Define local functions
/*******************************************
 * @brief 
 * 
 * @param PeriphClock ==
 * @param GPIO_Port ==
 * @param GPIO_Pin ==
 * @param inputOutputSelector ==
********************************************/
void 		InitPin_GPIO( uint32_t      PeriphClock,
                   GPIO_TypeDef* GPIO_Port,
                   uint32_t      GPIO_Pin,
                   uint8_t       inputOutputSelector );

/*******************************************
 * @brief 
 * 
 * @param PeriphClock ==
 * @param GPIO_Port ==
 * @param GPIO_Pin ==
********************************************/
void InitPin_GPIO_ANALOG( uint32_t     PeriphClock,
                          GPIO_TypeDef *GPIO_Port,
                          uint32_t      GPIO_Pin);


//  from main() to set up the GPIO pins and determine the
//   board configuration based on three GPIOs.
/*******************************************
 * @brief 
 * 
********************************************/
void 		InitBoardConfiguration_GPIO(void);

/*******************************************
 * @brief 
 * 
********************************************/
void    ControlPortInit(void);

/*******************************************
 * @brief 
 * 
********************************************/
void    GPIO_InitSpiBitBangInterface();


#ifdef __cplusplus
}
#endif


#endif
