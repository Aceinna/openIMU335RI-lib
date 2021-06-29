/** ***************************************************************************
 * @file   Butterworth_filterFP.h
 * @Author
 * @date   March, 2017
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * generic accelerometer interface, it should be implemented
 * by whichever accelerometer is in use
 *****************************************************************************/
#ifndef _BTW_FILTER_FP
#define _BTW_FILTER_FP
#include <stdint.h>
//======================data struct definitions======================
	typedef struct _buffer
	{
		float32_t     *d;      // data storage, each column represents a set of data
		int32_t        m;      // number of rows    (input array size)
		int32_t        n;      // number of columns (filter memory size = order + 1)
		int32_t        i;      // index for data being put into the buffer, -1 means buffer is empty
		int32_t        full;   // 1 means buffer is full, 0 not
		int32_t        num;    // number of samples in the buffer 
	} Buffer;


int32_t Butterworth_filterInitFP(int32_t chipNum);
int32_t Butterworth_filterFP(int32_t chipNum, uint8_t fAccel, int32_t freq, float32_t *dataIn, float32_t *dataOut);
int32_t Butterworth_filterFP2(int32_t chipNum, uint8_t fAccel, int32_t freq, float32_t *dataIn, float32_t *dataOut);
     
     
#endif

