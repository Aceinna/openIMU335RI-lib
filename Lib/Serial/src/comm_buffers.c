/** ***************************************************************************
 * @file comm_buffers.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
 * This is a set of routines to move data in and out of the communication
 * circular buffers. They will also report back the byte used and bytes available
 * in the buffer passed. These are common routines used on both bootloader and DUP
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

#include "comm_buffers.h"
#include "osapi.h"


/**********************************************
* @brief 
* 
* @param circBuf 
* @return uint32_t 
***********************************************/
uint32_t COM_buf_bytes_available (cir_buf_t* const circBuf)
{
    return circBuf->bytes_in_buffer;
}   /* end of COM_buf_bytes_available */

/**********************************************
* @brief 
* 
* @param circBuf 
* @return uint32_t 
***********************************************/
static uint32_t COM_buf_headroom (cir_buf_t* const circBuf)
{
	return (circBuf->buf_size - circBuf->bytes_in_buffer);
}   /* end of COM_buf_headroom */


/**********************************************
* @brief 
* 
* @param circBuf 
* @param delCnt 
* @return uint32_t 
***********************************************/
uint32_t COM_buf_delete_isr (cir_buf_t* const circBuf, uint32_t delCnt)
{

	if(circBuf->dma_bytes_to_tx != 0U){
        // assuming that all bytes got transmitted
        delCnt = circBuf->dma_bytes_to_tx;
        circBuf->dma_bytes_to_tx = 0U;
    }else if(delCnt > circBuf->bytes_in_buffer){
        delCnt = circBuf->bytes_in_buffer;
    }else{
		// nothing to do
	}

	if (circBuf->bytes_in_buffer && delCnt)  {
		if (circBuf->bytes_in_buffer < delCnt)  {
			delCnt = circBuf->bytes_in_buffer;	// delete all
		}
		circBuf->bytes_in_buffer -= delCnt;
		circBuf->buf_outptr      += delCnt;
		circBuf->buf_outptr      &= circBuf->buf_size-1U;	// assuming size if power of 2
    }else{
		delCnt = 0U;
	} 

	return delCnt;
}  /* end of COM_buf_delete */


/**********************************************
* @brief 
* 
* @param circBuf 
* @param buf 
* @param cnt 
* @return int32_t 
***********************************************/
int32_t COM_buf_add(cir_buf_t* const circBuf, uint8_t buf[], uint32_t cnt)
{
	uint32_t const headroom   = COM_buf_headroom(circBuf); 

	if(headroom < cnt){
		cnt = headroom;
	}

	for (uint32_t i = 0U; i < cnt; i++)
	{
		*(circBuf->buf_add + (circBuf->buf_inptr)) = buf[i];
		circBuf->buf_inptr++;
		// carefull here - buffer considered to be power of 2 length
		circBuf->buf_inptr &= circBuf->buf_size - 1U;
	}
	circBuf->bytes_in_buffer += cnt;
	
	return (int32_t)cnt;
}   /* end of COM_buf_add */

/**********************************************
* @brief 
* 
* @param circBuf 
* @param buf 
* @param cnt 
* @return int32_t 
***********************************************/
int32_t COM_buf_get(cir_buf_t* const circBuf, uint8_t buf[], uint32_t  cnt)
{
	uint32_t num;
	num = circBuf->bytes_in_buffer;

    if(cnt > num){
	    cnt = num;
    }

    for (uint32_t i = 0U; i < cnt; i++) {
        buf[i] = *(circBuf->buf_add + (circBuf->buf_outptr));
        circBuf->buf_outptr++;
        circBuf->buf_outptr &= circBuf->buf_size - 1U;	// size is power of 2
    }
    ENTER_CRITICAL();
	circBuf->bytes_in_buffer -= cnt;
    EXIT_CRITICAL();
	
	return (int32_t)cnt;
}       /*end of COM_buf_get*/

/**********************************************
* @brief 
* 
* @param circBuf 
* @param dataBufPtr 
* @return uint32_t 
***********************************************/
uint32_t COM_buf_prepare_dma_tx_transaction (cir_buf_t* const circBuf, uint8_t** const dataBufPtr)
{
    circBuf->dma_bytes_to_tx = circBuf->buf_size - circBuf->buf_outptr;
    if(circBuf->dma_bytes_to_tx > circBuf->bytes_in_buffer){
        circBuf->dma_bytes_to_tx = circBuf->bytes_in_buffer;
    }
    *dataBufPtr = circBuf->buf_add + circBuf->buf_outptr; 
    return circBuf->dma_bytes_to_tx;
}   /* end of COM_buf_bytes_available */
