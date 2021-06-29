/*
* File:   VectorMath.h
* Author: joemotyka
*
* Created on May 7, 2016, 12:50 AM
*/

#ifndef VECTORMATH_H
#define VECTORMATH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "GlobalConstants.h"

/******************************************************************************
 * @brief Compute the variance of a vector
 * 
 * @param a     [in]    pointer to the vector.
 * @param am    [in]    mean value of the vector.
 * @param n     [in]    number of elements in the vector.
 * @return  variance of the vector
 *****************************************************************************/
real vecVar(real * const a, real const am, int32_t const n);

/******************************************************************************
 * @brief vector coross product
 * 
 * @param a [in]    3D vector
 * @param b [in]    3D vector
 * @param axb [out]   c = axb
 *****************************************************************************/
void cross(real a[], real b[], real axb[]);

#ifdef __cplusplus
}
#endif

#endif /* VECTORMATH_H */
