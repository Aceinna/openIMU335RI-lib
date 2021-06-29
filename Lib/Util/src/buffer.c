/******************************************************************************
 * @file buffer.c
 * @author Xiaoguang Dong (xgdong@aceinna.com)
 * @brief Buffer operation.
 * @version 1.0
 * @date 2020-09-03
 * 
 * @copyright Copyright (c) 2020
 * 
 *****************************************************************************/

#include "buffer.h"

 /******************************************************************************
  * @brief Initialize a buffer.
  *
  * @param bf    [in]    pointer to the buffer.
  * @param d     [in]    pointer to the memory for data storage.
  * @param m     [in]    row number(dimension of data).
  * @param n     [in]    column number(number of sets of data).
  *  
  *****************************************************************************/
void bfNew(Buffer* const bf, real* const d, int32_t const m, int32_t const n)
{
    bf->d = d;
    bf->m = m;
    bf->n = n;
    bf->i = -1;
    bf->full = 0;
    bf->num = 0;
}

/******************************************************************************
 * @brief Put data into the buffer.
 *
 * @param bf    [in]    pointer to the buffer.
 * @param d     [in]    pointer to an array being put into the buffer.
 *****************************************************************************/
void bfPut(Buffer* const bf, real d[])
{
    int32_t i;
    int32_t const m = bf->m;              // row number
    int32_t const n = bf->n;              // column number

    bf->i += 1;                 // move to the next position
    if(bf->full == 0)           // buffer is not full
    {
        bf->num = bf->i + 1;
        if( bf->i == (n-1) )        // buffer become full
        {
            bf->full = 1;
        }
    }
    if(bf->i==n)                //buffer is full, overwrite the oldest data
    {
        bf->i = 0;
    }
    // put data into buffer
    for(i=0;i<m;i++)
    {
        bf->d[(i*n) + bf->i] = d[i];
    }
}

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
int32_t bfGet(const Buffer* const bf, real d[], int32_t idx)
{
    int32_t i;
    int32_t m;
    int32_t n;
    m = bf->m;
    n = bf->n;
    if(idx>=n)  // idx exceeds the max column number
    {
        for (i = 0; i < m; i++)
        {
            d[i] = 0.0F;
        }
        return 0;
    }
    if(bf->full)// buffer is full
    {
        idx = bf->i - idx;
        if (idx < 0)
        {
            idx += n;
        }
        for (i = 0; i < m; i++)
        {
            d[i] = bf->d[(i * n) + idx];
        }
    }
    else//buffer is not full
    {
        idx = bf->i - idx;
        if(idx<0)// idx exceeds the storage range
        {
            for (i = 0; i < m; i++)
            {
                d[i] = 0.0F;
            }
            return 0;
        }
        else// idx within the storage range
        {
            for (i = 0; i < m; i++)
            {
                d[i] = bf->d[(i * n) + idx];
            }
        }
    }
    return 1;
}

/******************************************************************************
 * @brief Clear the buffer.
 *
 * @param bf    [in]    pointer to the buffer.
 *  
 *****************************************************************************/
void bfClear(Buffer* const bf)
{
    bf->i = -1;
    bf->full = 0;
    bf->num = 0;
}
