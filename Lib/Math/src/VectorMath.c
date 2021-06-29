/* 
 * File:   VectorMath.cpp
 * Author: joemotyka
 * 
 * Created on May 7, 2016, 12:50 AM
 */

#include "VectorMath.h"

#include <math.h>

/******************************************************************************
 * @brief Compute the variance of a vector
 * 
 * @param a     [in]    pointer to the vector.
 * @param am    [in]    mean value of the vector.
 * @param n     [in]    number of elements in the vector.
 * @return  variance of the vector
 *****************************************************************************/
real vecVar(real * const a, real const am, int32_t const n)
{
    int32_t i;
    real sum = 0.0F;
    for (i = 0; i < n; i++)
    {
        real const d = a[i] - am;
        sum += d * d;
    }
    return sum / (real)n;
}

/******************************************************************************
 * @brief vector coross product
 *
 * @param a [in]    3D vector
 * @param b [in]    3D vector
 * @param axb [out]   c = axb
 *****************************************************************************/
void cross(real a[], real b[], real axb[])
{
    axb[0] = ( a[1] * b[2] ) - ( a[2] * b[1] );
    axb[1] = ( a[2] * b[0] ) - ( a[0] * b[2] );
    axb[2] = ( a[0] * b[1] ) - ( a[1] * b[0] );
}
