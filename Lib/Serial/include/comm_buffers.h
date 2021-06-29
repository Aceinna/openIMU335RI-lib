/** ***************************************************************************
 * @file comm_buffers.h for the uart driver(extern_port), for use by the system
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
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

#ifndef COMM_BUFFERS_H
#define COMM_BUFFERS_H

#ifdef __cplusplus
extern "C" {
#endif    

typedef struct {
    uint8_t 	*buf_add;	  ///< pointer to (address) comm buffer
    uint32_t 	buf_size;	  ///< buffer size
    uint32_t    buf_inptr;  ///< buffer input pointer - modified in UART ISR
	uint32_t    buf_outptr; ///< buffer output pointer - modified in UART ISR
    uint32_t    bytes_in_buffer; ///< ammount of buffer used
    uint32_t    dma_bytes_to_tx;  ///< amount of bytes to transmit by dma in single transaction
    uint32_t    dma_bytes_to_rx;  ///< amount of bytes to receive by dma in single transaction
} cir_buf_t;

// cir_buf defined in port_def.h
/*******************************************
 * @brief 
 * 
 * @param circBuf ==
 * @return uint32_t 
********************************************/
extern uint32_t COM_buf_bytes_available (cir_buf_t* const circBuf);

/*******************************************
 * @brief 
 * 
 * @param circBuf ==
 * @param delCnt ==
 * @return uint32_t 
********************************************/
extern uint32_t COM_buf_delete_isr (cir_buf_t* const circBuf, uint32_t const delCnt);

/*******************************************
 * @brief 
 * 
 * @param circBuf ==
 * @param buf ==
 * @param cnt ==
 * @return int32_t 
********************************************/
extern int32_t  COM_buf_add(cir_buf_t* const circBuf, uint8_t* buf, uint32_t cnt);

/*******************************************
 * @brief 
 * 
 * @param circBuf ==
 * @param buf ==
 * @param cnt ==
 * @return int32_t 
********************************************/
extern int32_t  COM_buf_get(cir_buf_t* const circBuf, uint8_t* const buf, uint32_t cnt);

/*******************************************
 * @brief 
 * 
 * @param circBuf ==
 * @param dataBufPtr ==
 * @return uint32_t 
********************************************/
extern uint32_t COM_buf_prepare_dma_tx_transaction (cir_buf_t* const circBuf, uint8_t ** const dataBufPtr);
#ifdef __cplusplus
}
#endif    

#endif
