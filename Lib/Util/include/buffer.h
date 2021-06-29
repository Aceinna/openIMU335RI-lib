/******************************************************************************
 * @file buffer.h
 * @author Xiaoguang Dong (xgdong@aceinna.com)
 * @brief Buffer operation.
 * @version 1.0
 * @date 2020-09-03
 * 
 * @copyright Copyright (c) 2020
 * 
 *****************************************************************************/


#ifndef BUFFER_H_INCLUDED
#define BUFFER_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "GlobalConstants.h"


//======================data struct definitions======================
typedef struct _buffer
{
    real *d;        // data storage, each column represents a set of data
    int32_t m;      //row number
    int32_t n;      //column number
    int32_t i;      //index for data being put into the buffer, -1 means buffer is empty
    int32_t full;   //1 means buffer is full, 0 not
    int32_t num;    //data number in the buffer
} Buffer;

/******************************************************************************
 * @brief Initialize a buffer.
 * 
 * @param bf    [in]    pointer to the buffer.
 * @param d     [in]    pointer to the memory for data storage.
 * @param m     [in]    row number(dimension of data).
 * @param n     [in]    column number(number of sets of data).
 *  
 *****************************************************************************/
void bfNew(Buffer* const bf, real* const d, int32_t const m, int32_t const n);

/******************************************************************************
 * @brief Put data into the buffer.
 * 
 * @param bf    [in]    pointer to the buffer.
 * @param d     [in]    pointer to an array being put into the buffer.
 *****************************************************************************/
void bfPut(Buffer* const bf, real d[]);

/******************************************************************************
 * @brief Read data from the buffer.
 * 
 * @param bf    [in]    pointer to the buffer.
 * @param d     [in]    pointer to an array where the data read from the buffer
 *                      is put.
 * @param idx   [in]    index of the data in the buffer to be read.
 *                      idx=0 means the latest data,
 *                      idx=1 means data before the latest...
 * @return 1 menas read OK, 0 menas idx out of bound .
 *****************************************************************************/
int32_t bfGet(const Buffer* const bf, real d[], int32_t idx);

/******************************************************************************
 * @brief Clear the buffer.
 * 
 * @param bf    [in]    pointer to the buffer.
 *  
 *****************************************************************************/
void bfClear(Buffer* const bf);

#ifdef __cplusplus
}
#endif

#endif /* BUFFER_H_INCLUDED */
