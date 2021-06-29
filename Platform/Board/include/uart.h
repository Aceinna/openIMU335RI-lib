/** ***************************************************************************
 * @file   uart.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *  UART driver for the DMU380's two user serial ports
 *	transmitting and receive of serial data and then number of bytes remaining in
 *	the circular buffer. There is no FIFO on the uart as there is in the 525
 *
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

#ifndef UART_H
#define UART_H

#ifdef __cplusplus
extern "C" {
#endif    

#include <stdint.h>
#include "stm32f4xx.h"

struct sPinConfig {
    GPIO_TypeDef *port;
    uint16_t     pin;
    uint16_t     source;
    uint8_t      altFun;
};

struct sUartConfig {
    int32_t               idx;
    USART_TypeDef     *uart;
    uint32_t          ahb1ClockEnable;
    uint32_t          apb1ClockEnable;
    uint32_t          ahb2ClockEnable;
    uint32_t          apb2ClockEnable;
    struct sPinConfig tx;
    struct sPinConfig rx;
    uint16_t          irqChannel;
    uint8_t           preemptPriority;
    uint8_t           subPriority;
    uint32_t          dmaTxChannel;
    uint32_t          dmaRxChannel;
    DMA_TypeDef*      Dma;
    DMA_Stream_TypeDef *DMA_TX_Stream;
    DMA_Stream_TypeDef *DMA_RX_Stream;
    IRQn_Type         dmaTxIRQn;
    IRQn_Type         dmaRxIRQn;
    uint32_t          dmaTxFlags;
    uint32_t          dmaRxFlags;
};


/*******************************************
 * @brief 
 * 
 * @param channel ==
 * @param baudrate ==
 * @return int32_t 
********************************************/
extern int32_t          uart_init(int32_t channel, uint32_t baudrate);

/*******************************************
 * @brief 
 * 
 * @param channel ==
 * @param data ==
 * @param len ==
 * @return int32_t 
********************************************/
extern int32_t          uart_write(int32_t channel, uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif    

#endif
