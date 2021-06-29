/** ***************************************************************************
 * @file   uart.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
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

#include <stdint.h>

#include "serial_port_def.h"
#include "uart.h"
#include "comm_buffers.h"
#include "osapi.h"
#include "boardDefinition.h"
#include "halAPI.h"


//extern port_struct gPort[], gPort0, gPort1, gPort2;
static port_struct gPort[3];
/// port-associated software rx and tx circular buffers
static uint16_t udummy;


// 0 - user com, 1 GPS
static struct sUartConfig gUartConfig[NUM_SERIAL_PORTS] = {
    {
        .idx             = 0,
        .uart            = USER_A_UART,             // USART3
        .ahb1ClockEnable = (USER_A_UART_TX_GPIO_CLK | USER_A_UART_TX_GPIO_CLK | USER_A_UART_DMA_CLK),
        .apb1ClockEnable = USER_A_UART_CLK,
        .ahb2ClockEnable = 0,
        .apb2ClockEnable = 0,
        .tx = {
            .pin    = USER_A_UART_TX_PIN,
            .port   = USER_A_UART_TX_GPIO_PORT,
            .source = USER_A_UART_TX_SOURCE,
            .altFun = USER_A_UART_TX_AF
        },
        .rx = {
            .pin    = USER_A_UART_RX_PIN,
            .port   = USER_A_UART_RX_GPIO_PORT,
            .source = USER_A_UART_RX_SOURCE,
            .altFun = USER_A_UART_RX_AF
        },
        .irqChannel = USER_A_UART_IRQn,
        .preemptPriority = 8,                           // low
        .subPriority     = 1,
        .dmaTxChannel  = USER_A_UART_DMA_CHANNEL,       // Channel 4
        .dmaRxChannel  = USER_A_UART_DMA_CHANNEL,       // Channel 4
        .DMA_TX_Stream = USER_A_UART_DMA_TX_STREAM,     // DMA1_Stream3
        .DMA_RX_Stream = USER_A_UART_DMA_RX_STREAM,     // DMA1_Stream1
        .dmaTxIRQn     = USER_A_UART_DMA_TX_STREAM_IRQ, // DMA1_Stream3_IRQn 
        .dmaRxIRQn     = USER_A_UART_DMA_RX_STREAM_IRQ, // DMA1_Stream1_IRQn
        .Dma           = DMA1,
        .dmaTxFlags    = USER_A_UART_DMA_TX_FLAGS,
        .dmaRxFlags    = USER_A_UART_DMA_RX_FLAGS,
    }
};

/** ****************************************************************************
 * @name uart_buf_init
 * @brief initalizes the buffer related pointers, indexs buffer size. it also
 *        sets datatype and tx_int_flg to zero. These items are
 *        the only ones that are appropriate for this routine to initialize all
 *        others should be done by the subroutines which are related to them.
 *        The data type and hw_int_flg are set here to inhibit functions from
 *        running if not initialized.
 * Trace:
 * [SDD_COM_BUF_INIT_01 <-- SRC_COM_BUF_INIT]
 * [SDD_COM_BUF_INIT_02 <-- SRC_COM_BUF_INIT]
 * [SDD_COM_BUF_INIT_03 <-- SRC_COM_BUF_INIT]
 *
 * @param port [in] - pointer to the circular buffer structure
 * @param tx_buf [in]  - pointer to the transmit buffer.
 * @param rx_buf [in] - pointer to the receive buffer.
 * @param rx_size [in] - buffer size
 * @param tx_size [in] - buffer size
 * Return value: N/A
 ******************************************************************************/
static void uart_buf_init(port_struct* const port,
                  uint8_t* const tx_buf,
                  uint8_t* const rx_buf,
                  uint32_t  const rx_size,
                  uint32_t  const tx_size)
{
	port->rec_buf.buf_add    = rx_buf;
	port->rec_buf.buf_size   = rx_size;
	port->rec_buf.buf_inptr  = 0U;
	port->rec_buf.buf_outptr = 0U;
    port->rec_buf.dma_bytes_to_rx = 0U;
    port->rec_buf.dma_bytes_to_tx = 0U;

	port->xmit_buf.buf_add    = tx_buf;
	port->xmit_buf.buf_size   = tx_size;
	port->xmit_buf.buf_inptr  = 0U;
	port->xmit_buf.buf_outptr = 0U;
    port->xmit_buf.dma_bytes_to_rx = 0U;
    port->xmit_buf.dma_bytes_to_tx = 0U;
    port->txBusy = 0;
    port->rxBusy = 0;

}  /*end of COM_buf_init*/


/** ****************************************************************************
 * @name uart_init
 * @brief initializes all channels of the UART peripheral
 *
 * @param channel [in] - 0- USER_UART, port 1 - GPS UART, 2 - DEBUG UART
 * @param baudrate [in]  - uart_hw data structure containing fifo trigger levels
 *						and the baudrate for this port
 * @return -1 if invalid
 * ******************************************************************************/
int32_t uart_init(int32_t const channel, uint32_t const baudrate)
{
    static uint8_t port_rx_buf[NUM_SERIAL_PORTS][512];
    static uint8_t port_tx_buf[NUM_SERIAL_PORTS][512];

    struct sUartConfig* const uartConfig = &(gUartConfig[channel]);
    
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;

    if(channel != USER_SERIAL_PORT){
        return -1;
    }

    /// initialize software circular buffers comm_buffers.c User com
    uart_buf_init(&gPort[channel], 
                 port_tx_buf[channel],
                 port_rx_buf[channel],    
                 512U,
                 512U);

    RCC_AHB1PeriphClockCmd(uartConfig->ahb1ClockEnable, ENABLE);
    RCC_APB1PeriphClockCmd(uartConfig->apb1ClockEnable, ENABLE);
    RCC_AHB2PeriphClockCmd(uartConfig->ahb2ClockEnable, ENABLE);
    RCC_APB2PeriphClockCmd(uartConfig->apb2ClockEnable, ENABLE);

    /// configure COM TX Pins
    GPIO_InitStructure.GPIO_Pin   = uartConfig->tx.pin;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(uartConfig->tx.port, &GPIO_InitStructure);
    GPIO_PinAFConfig(uartConfig->tx.port, uartConfig->tx.source, uartConfig->tx.altFun);

    /// configure COM RX Pins
    GPIO_InitStructure.GPIO_Pin = uartConfig->rx.pin;
    GPIO_Init(uartConfig->rx.port, &GPIO_InitStructure);
    GPIO_PinAFConfig(uartConfig->rx.port, uartConfig->rx.source, uartConfig->rx.altFun);

    /** USARTx configured as follow:
    - BaudRate = from port_hw
    - Word Length = 8 Bits, one stop bit, o parity, no flow control
    - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate            = baudrate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(uartConfig->uart, &USART_InitStructure);
    // initialize TX DMA stream 
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_Channel            = uartConfig->dmaTxChannel;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(uartConfig->uart->DR);
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
    DMA_InitStructure.DMA_BufferSize         = 0U;
    DMA_InitStructure.DMA_Memory0BaseAddr    = 0U;              // will be asssigned later in transaction
    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
    DMA_Init(uartConfig->DMA_TX_Stream, &DMA_InitStructure);
    /// Enable TX DMA interrupt
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = uartConfig->preemptPriority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = uartConfig->subPriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannel                   = (uint8_t)uartConfig->dmaTxIRQn;
    NVIC_Init(&NVIC_InitStructure);


    USART_Cmd(uartConfig->uart, ENABLE);

    /// clear anything in the rx
    if (USART_GetFlagStatus(uartConfig->uart, USART_FLAG_RXNE)) {
        udummy = USART_ReceiveData(uartConfig->uart);
    }

    /// now for interrupts, only turn rx interrupts on
    USART_ITConfig( uartConfig->uart, USART_IT_RXNE, ENABLE );
	NVIC_InitStructure.NVIC_IRQChannel                   = (uint8_t)uartConfig->irqChannel;
    int8_t const tmp = (int8_t)(0x8 + channel);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (uint8_t)tmp; // low
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0U;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init( &NVIC_InitStructure );

    return 0;

} /* end function uart_init */

/** ****************************************************************************
 * @name uart_init
 * @brief initializes all channels of the UART peripheral
 *
 * @param channel [in] - 0- USER_UART, port 1 - GPS UART, 2 - DEBUG UART
 * @param baudrate [in]  - uart_hw data structure containing fifo trigger levels
 *						   and the baudrate for this port
 * @return -1 if invalid
 * ******************************************************************************/
int32_t UART_Init(int32_t const channel, uint32_t const baudrate)
{
    return uart_init(channel, baudrate);
}


/** ****************************************************************************
 * @name uart_init
 * @brief initializes all channels of the UART peripheral
 *
 * @param channel [in] - 0- USER_UART, port 1 - GPS UART, 2 - DEBUG UART
 * @param data [in]  - uart_hw data structure containing fifo trigger levels
 * @param len [in]  - length
 * @return -1 if invalid
 * ******************************************************************************/
int32_t uart_write(int32_t const channel, uint8_t* const data, uint32_t const len)
{
    if(channel == INVALID_SERIAL_PORT){
        return 0;
    }

    ENTER_CRITICAL();
    int32_t const written = COM_buf_add(&gPort[channel].xmit_buf, data, len);
    if(!gPort[channel].txBusy){
        gPort[channel].txBusy = 1;
        USART_ITConfig( gUartConfig[channel].uart, USART_IT_TXE, ENABLE);
    }
    EXIT_CRITICAL();
    return written;
} /* end function uart_write */

/** ****************************************************************************
 * @name uart_init
 * @brief initializes all channels of the UART peripheral
 *
 * @param channel [in] - 0- USER_UART, port 1 - GPS UART, 2 - DEBUG UART
 * @param data [in]  - uart_hw data structure containing fifo trigger levels
 * @param len [in]  - length
 * @return -1 if invalid
 * ******************************************************************************/
int32_t UART_Write(int32_t const channel, uint8_t* const data, uint32_t const len)
{
    return uart_write(channel, data, len);
}



/** ****************************************************************************
 * @brief 
 *
 * @param channel [in] - 0- USER_UART, port 1 - GPS UART, 2 - DEBUG UART
 * @param data [in]  - uart_hw data structure containing fifo trigger levels
 * @param len [in]  - length
 * @return -1 if invalid
 * ******************************************************************************/

int32_t UART_Read(int32_t const channel, uint8_t* const data, uint32_t const len)
{
    if(channel == INVALID_SERIAL_PORT){
        return 0;
    }
    return COM_buf_get(&gPort[channel].rec_buf, data, len);
}  // end function uart_read


/** ****************************************************************************
 * @brief 
 *
 * @param config [in] - 0- USER_UART, port 1 - GPS UART, 2 - DEBUG UART
 * @param data [in]  - uart_hw data structure containing fifo trigger levels
 * @param length [in]  - length
 * @return -1 if invalid
 * ******************************************************************************/
static uint8_t uart_dma_transmit(struct sUartConfig*  const config, uint8_t* const data, uint16_t const length )
{
    if(config->idx == 0){
        config->Dma->LIFCR = config->dmaTxFlags;
    }else{
    config->Dma->HIFCR = config->dmaTxFlags;
    }

    /// Configure the memory address: DMA_MemoryTargetConfig()
    config->DMA_TX_Stream->M0AR = (uint32_t)data;
    /// number of bytes to be transferred on the DMA
    config->DMA_TX_Stream->NDTR = (uint16_t)length;
    /// Enable the DMA interrupt with the transfer complete interrupt mask
    DMA_ITConfig(config->DMA_TX_Stream, DMA_IT_TC, ENABLE);
    /// enable DMA support on USART
    USART_DMACmd(config->uart, USART_DMAReq_Tx, ENABLE);
    /// Enable the DMA Stream
    config->DMA_TX_Stream->CR |= (uint32_t)DMA_SxCR_EN;

    return 0U;
}


/** ****************************************************************************
 * @brief 
 *
 * @param channel [in] - 0- USER_UART, port 1 - GPS UART, 2 - DEBUG UART
 * @param port [in]  - uart_hw data structure containing fifo trigger levels
 * ******************************************************************************/
static void uart_prepare_tx(uint32_t const channel, port_struct* const port)
{

    struct sUartConfig* const uartConfig = &(gUartConfig[channel]);
    if(channel == 0U){
        uartConfig->Dma->LIFCR   = uartConfig->dmaTxFlags;   
    }else{
        uartConfig->Dma->HIFCR   = uartConfig->dmaTxFlags;   
    }

    COM_buf_delete_isr(&port->xmit_buf, 0U); // will delete bytes for previous DMA transaction

    if(COM_buf_bytes_available(&port->xmit_buf)){
       uint8_t  *data;
       uint32_t const bytesToTx  = COM_buf_prepare_dma_tx_transaction (&port->xmit_buf, &data);
       udummy = uart_dma_transmit(uartConfig, data, (uint16_t)bytesToTx);
       port->txBusy = 1;
    }else {
       port->txBusy = 0;
    }
}


/** ****************************************************************************
 * @name uart_isr - common callback for the port specific handlers below
 * @brief  this routine will read the UART status register and if there was an
 *  interrupt generated from this UART port it will check for a transmit fifo level
 *  interrupt. If no interrupt was generated this routine will exit.  If there was
 *  another interrupt source the interrupt enable register will be cleared except
 *  for the FIFO level interrupt.  If a valid transmit fifo level interrupt has
 *  occured the fifo level will be read from the UART and compared to the number
 *  of bytes in the transmit buffer.  If there is more than will fit in the UART
 *  FIFO then only what will fit is move to a temporary buffer and then put in the
 *  FIFO, the interrupt remains active.  If there is less bytes in the buffer than
 *  space in the FIFO the buffer is emptied and the interrupt disabled.
 *
 * @param channel [in] - selects UART channel to read from, must be < MAX_CHANNEL
 * @param port [in] *  - pointer to the data structure for this UART port
 *              uses xmit buffer and the tx_int_flg
 *  
 ******************************************************************************/
static void uart_isr (uint32_t const channel, port_struct* const port)
{

    USART_TypeDef* const uart = gUartConfig[channel].uart;

    // receive data
    if (USART_GetFlagStatus(uart, USART_FLAG_RXNE)) {
        uint8_t ch = (uint8_t)uart->DR;
        COM_buf_add(&(port->rec_buf), &ch, 1U);
    }

    // transmit data
    if (USART_GetFlagStatus(uart, USART_FLAG_TXE)) {
        USART_ITConfig( uart, USART_IT_TXE, DISABLE);
        uart_prepare_tx(channel, port);
    }

} /* end function uart_isr */

/** ****************************************************************************
 * @brief handle
 *  
 ******************************************************************************/
void USER_A_UART_IRQ()
{
    uint32_t channel;

    OSEnterISR();

    channel           = (uint32_t)USER_SERIAL_PORT;
    port_struct* const port = &gPort[0];

    uart_isr (channel, port);

    OSExitISR();
}

void USER_A_UART_DMA_TX_IRQHandler()
{
    OSEnterISR();
    uint32_t const channel  = kUserA_UART;
    port_struct* const port = &gPort[0];
    uart_prepare_tx(channel, port);
    OSExitISR();
}
