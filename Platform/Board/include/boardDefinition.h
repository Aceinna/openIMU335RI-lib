/** ***************************************************************************
 * @file boarDefinition.h DMU380 ARM Cortex M0 I/O pins and interrupts
 * @brief Settings for the DMU380 board (STM32F205RE).
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************///
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
#ifndef BOARD_DEFINITION_H
#define BOARD_DEFINITION_H

#include "stm32f4xx_conf.h"
/// == set up the device's DEBUG-serial as USART ==
/// USART (serial interface) defines: TX - A9
///                                   Rx - A10

///**********************************************************************
/// == user-communications protocol as UART ==
/// The User pins can be SPI (MOSI, MISO, CLK, Select or two UARTS
///   (4 and 5) or a USART and a UART(3 and 5)

static uint32_t const kUserA_UART   =      0U; // where it is in the uart.c gUartConfig structure
#define USER_A_UART                        USART3
#define USER_A_UART_CLK                    RCC_APB1Periph_USART3
#define USER_A_UART_TX_PIN                 GPIO_Pin_10
#define USER_A_UART_TX_GPIO_PORT           GPIOC
#define USER_A_UART_TX_GPIO_CLK            RCC_AHB1Periph_GPIOC
#define USER_A_UART_TX_SOURCE              GPIO_PinSource10
#define USER_A_UART_TX_AF                  GPIO_AF_USART3
#define USER_A_UART_RX_PIN                 GPIO_Pin_11
#define USER_A_UART_RX_GPIO_PORT           GPIOC
#define USER_A_UART_RX_GPIO_CLK            RCC_AHB1Periph_GPIOC
#define USER_A_UART_RX_SOURCE              GPIO_PinSource11
#define USER_A_UART_RX_AF                  GPIO_AF_USART3
#define USER_A_UART_IRQn                   USART3_IRQn
#define USER_A_UART_IRQ                    USART3_IRQHandler

#define USER_A_UART_DMA                    DMA1
#define USER_A_UART_DMA_CLK                RCC_AHB1Periph_DMA1
#define USER_A_UART_DMA_CHANNEL            DMA_Channel_4
#define USER_A_UART_DMA_RX_STREAM          DMA1_Stream1
#define USER_A_UART_DMA_TX_STREAM          DMA1_Stream3
#define USER_A_UART_DMA_RX_STREAM_IRQ      DMA1_Stream1_IRQn
#define USER_A_UART_DMA_TX_STREAM_IRQ      DMA1_Stream3_IRQn
#define USER_A_UART_DMA_RX_IRQHandler      DMA1_Stream1_IRQHandler
#define USER_A_UART_DMA_TX_IRQHandler      DMA1_Stream3_IRQHandler

#define USER_A_UART_DMA_TX_FLAGS          (DMA_FLAG_FEIF3  | \
                                           DMA_FLAG_DMEIF3 | \
                                           DMA_FLAG_TEIF3  | \
                                           DMA_FLAG_HTIF3  | \
                                           DMA_FLAG_TCIF3 )

#define USER_A_UART_DMA_RX_FLAGS          (DMA_FLAG_FEIF1  | \
                                           DMA_FLAG_DMEIF1 | \
                                           DMA_FLAG_TEIF1  | \
                                           DMA_FLAG_HTIF1  | \
                                           DMA_FLAG_TCIF1    )





// ========= set up the Unit Configuration Pins =========
#define CONFIG_0_PIN                GPIO_Pin_9
#define CONFIG_0_CLK                RCC_AHB1Periph_GPIOA
#define CONFIG_0_SOURCE             GPIO_PinSource9

#define CONFIG_1_PIN                GPIO_Pin_4
#define CONFIG_1_CLK                RCC_AHB1Periph_GPIOB
#define CONFIG_1_SOURCE             GPIO_PinSource4

// ========= set up the Unit TP Pins =========
#define TP1_PIN                     GPIO_Pin_11
#define TP1_CLK                     RCC_AHB1Periph_GPIOA
#define TP1_SOURCE                  GPIO_PinSource11

#define TP2_PIN                     GPIO_Pin_12
#define TP2_CLK                     RCC_AHB1Periph_GPIOA
#define TP2_SOURCE                  GPIO_PinSource12


// ADC pins
#define ADC1_PIN                    GPIO_Pin_1              // ADC123 in 11 V Input First 
#define ADC1_PORT                   GPIOC
#define ADC1_PORT_CLK               RCC_AHB1Periph_GPIOC

#define ADC2_PIN                    GPIO_Pin_0              // ADC123 in 10 V Input Second
#define ADC2_PORT                   GPIOC
#define ADC2_PORT_CLK               RCC_AHB1Periph_GPIOC

#define ADC3_PIN                    GPIO_Pin_4              // ADC12 in 14 5V Input First
#define ADC3_PORT                   GPIOC
#define ADC3_PORT_CLK               RCC_AHB1Periph_GPIOC

#define ADC4_PIN                    GPIO_Pin_5              // ADC12 in 155V Input Second
#define ADC4_PORT                   GPIOC
#define ADC4_PORT_CLK               RCC_AHB1Periph_GPIOC

#define ADC5_PIN                    GPIO_Pin_2              // ADC123 IN 12 3.3 V Input First
#define ADC5_PORT                   GPIOC
#define ADC5_PORT_CLK               RCC_AHB1Periph_GPIOC

#define ADC6_PIN                    GPIO_Pin_3              // ADC 123 in 13 .3 V Input Second
#define ADC6_PORT                   GPIOC
#define ADC6_PORT_CLK               RCC_AHB1Periph_GPIOC

#define ADC1_CLK                    RCC_APB2Periph_ADC1
#define ADC2_CLK                    RCC_APB2Periph_ADC2
#define ADC3_CLK                    RCC_APB2Periph_ADC3

#define ACCEL_DATA_READY_EXTI_LINE           EXTI_Line8
#define MAG_DATA_READY_EXTI_LINE             EXTI_Line5

#define ADC1_DMA_CLK                RCC_AHB1Periph_DMA2

//CAN bus 
#define CAN1_RX_PIN                     GPIO_Pin_8
#define CAN1_RX_GPIO_CLK                RCC_AHB1Periph_GPIOB
#define CAN1_RX_SOURCE                  GPIO_PinSource8
#define CAN1_TX_PIN                     GPIO_Pin_9
#define CAN1_TX_GPIO_CLK                RCC_AHB1Periph_GPIOB
#define CAN1_TX_SOURCE                  GPIO_PinSource9


// for V1 board
#define CAN_AB_CTRL_PIN                 GPIO_Pin_15
#define CAN_AB_CTRL_CLK                 RCC_AHB1Periph_GPIOA
#define CAN_AB_CTRL_SOURCE              GPIO_PinSource15


// for V2 board
#define CAN_EN_CTRL_PIN                 GPIO_Pin_15
#define CAN_EN_CTRL_PORT                GPIOB
#define CAN_EN_CTRL_CLK                 RCC_AHB1Periph_GPIOB
#define CAN_EN_CTRL_SOURCE              GPIO_PinSource15


#ifndef MTLT335_V1

#define SPI_BITBANG_SCK_PIN             GPIO_Pin_3  // PB3         
#define SPI_BITBANG_MOSI_PIN            GPIO_Pin_5  // PB5         

#define SPI_BITBANG_MISO1_PIN           GPIO_Pin_2  // PA2  
#define SPI_BITBANG_MISO2_PIN           GPIO_Pin_3  // PA3  
#define SPI_BITBANG_MISO3_PIN           GPIO_Pin_6  // PA6  

#define SPI_BITBANG_MISO_PINS           (SPI_BITBANG_MISO1_PIN | SPI_BITBANG_MISO2_PIN | \
                                         SPI_BITBANG_MISO3_PIN )

#define SPI_BITBANG_NSS1_PIN            GPIO_Pin_0  // PB0  
#define SPI_BITBANG_NSS2_PIN            GPIO_Pin_1  // PB1  
#define SPI_BITBANG_NSS3_PIN            GPIO_Pin_2  // PB2  

#define SPI_BITBANG_NSS_PINS            (SPI_BITBANG_NSS1_PIN | SPI_BITBANG_NSS2_PIN | \
                                         SPI_BITBANG_NSS3_PIN )    
#else

#define SPI_BITBANG_SCK_PIN             GPIO_Pin_10   // PB10               
#define SPI_BITBANG_MOSI_PIN            GPIO_Pin_11   // PB11               

#define SPI_BITBANG_MISO1_PIN           GPIO_Pin_2    // PA2
#define SPI_BITBANG_MISO2_PIN           GPIO_Pin_3    // PA3    
#define SPI_BITBANG_MISO3_PIN           GPIO_Pin_4    // PA4


#define SPI_BITBANG_MISO_PINS           (SPI_BITBANG_MISO1_PIN | SPI_BITBANG_MISO2_PIN | \
                                         SPI_BITBANG_MISO3_PIN)

#define SPI_BITBANG_NSS1_PIN            GPIO_Pin_0    // PB0
#define SPI_BITBANG_NSS2_PIN            GPIO_Pin_1    // PB1
#define SPI_BITBANG_NSS3_PIN            GPIO_Pin_2    // PB2

#define SPI_BITBANG_NSS_PINS            (SPI_BITBANG_NSS1_PIN | SPI_BITBANG_NSS2_PIN | \
                                         SPI_BITBANG_NSS3_PIN)    
#endif

// Sensors interrupt pins

#define RATE1_INT1_PIN                    GPIO_Pin_15
#define RATE1_INT1_PORT                   GPIOA
#define RATE1_INT1_CLK                    RCC_AHB1Periph_GPIOA
#define RATE1_INT1_SOURCE                 GPIO_PinSource15
#define RATE1_INT1_EXTI_LINE              EXTI_Line15

#define RATE1_INT2_PIN                    GPIO_Pin_10
#define RATE1_INT2_PORT                   GPIOA
#define RATE1_INT2_CLK                    RCC_AHB1Periph_GPIOA
#define RATE1_INT2_SOURCE                 GPIO_PinSource10
#define RATE1_INT2_EXTI_LINE              EXTI_Line10
#define RATE1_INT2_PORT_SOURCE            EXTI_PortSourceGPIOA

#define RATE2_INT1_PIN                    GPIO_Pin_7
#define RATE2_INT1_PORT                   GPIOA
#define RATE2_INT1_CLK                    RCC_AHB1Periph_GPIOA
#define RATE2_INT1_SOURCE                 GPIO_PinSource7
#define RATE2_INT1_EXTI_LINE              EXTI_Line7
#define RATE2_INT1_PORT_SOURCE            EXTI_PortSourceGPIOA

#define RATE2_INT2_PIN                    GPIO_Pin_7
#define RATE2_INT2_PORT                   GPIOC
#define RATE2_INT2_CLK                    RCC_AHB1Periph_GPIOC
#define RATE2_INT2_SOURCE                 GPIO_PinSource7
#define RATE2_INT2_EXTI_LINE              EXTI_Line7
#define RATE2_INT2_PORT_SOURCE            EXTI_PortSourceGPIOC

#define RATE3_INT1_PIN                    GPIO_Pin_8
#define RATE3_INT1_PORT                   GPIOA
#define RATE3_INT1_CLK                    RCC_AHB1Periph_GPIOA
#define RATE3_INT1_SOURCE                 GPIO_PinSource8
#define RATE3_INT1_EXTI_LINE              EXTI_Line8
#define RATE3_INT1_PORT_SOURCE            EXTI_PortSourceGPIOA

#define RATE3_INT2_PIN                    GPIO_Pin_9
#define RATE3_INT2_PORT                   GPIOC
#define RATE3_INT2_CLK                    RCC_AHB1Periph_GPIOC
#define RATE3_INT2_SOURCE                 GPIO_PinSource9
#define RATE3_INT2_EXTI_LINE              EXTI_Line9
#define RATE3_INT2_PORT_SOURCE            EXTI_PortSourceGPIOC


// ADC- RElared


extern GPIO_TypeDef* PORT_A;
extern GPIO_TypeDef* PORT_B;
extern GPIO_TypeDef* PORT_D;
extern GPIO_TypeDef* PORT_C;


#endif
