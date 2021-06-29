/******************************************************************************
* @file matrixMath.h real precision Linear algebra calculation functions
* @author
* @date   September, 2008
* @copyright (c) 2013, 2014 All Rights Reserved.
* @section LICENSE
* THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
* PARTICULAR PURPOSE.
* @brief
******************************************************************************/

#ifndef MATRIXMATH_H
#define MATRIXMATH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>  // uint8_t, etc.
#include "GlobalConstants.h"  // for real

/******************************************************************************
 * @brief Matrix A is multiplied by matrix B to get matrix C.
 *        C = A * B.
 * 
 * @param A         [in]    Matrix A.
 * @param B         [in]    Matrix B.
 * @param rowsInA   [in]    Number of rows in matrix A.
 * @param colsInA   [in]    NUmber of columns in matrix A.
 * @param colsInB   [in]    NUmber of columns in matrix B.
 * @param C         [out]   Matrix C.
 *  
 *****************************************************************************/
void AxB(const real* const A,
         const real* const B,
         uint8_t const rowsInA,
         uint8_t const colsInA,
         uint8_t const colsInB,
         real* const C);

/******************************************************************************
 * @brief Matrix A is multiplied by a vector V to get vector C.
 * 
 * @param A         [in]    Matrix A.
 * @param V         [in]    Vector V.
 * @param rowsInA   [in]    Number of rows in matrix A.
 * @param colsInA   [in]    NUmber of columns in matrix A.
 * @param C         [out]   Vector C.
 *  
 *****************************************************************************/
void AxV(const real* const A,
         const real* const V,
         uint8_t const rowsInA,
         uint8_t const colsInA,
         real* const C);

/******************************************************************************
 * @brief Matrix A minus matrix B to get matrix C.
 *        C = A - B.
 * 
 * @param A         [in]    Matrix A.
 * @param B         [in]    Matrix B.
 * @param rowsInA   [in]    Number of rows in matrix A.
 * @param colsInA   [in]    NUmber of columns in matrix A.
 * @param C         [out]   Matrix C.
 *  
 *****************************************************************************/
void AMinusB(const real* const A,
             const real* const B,
             uint8_t const rowsInA,
             uint8_t const colsInA,
             real* const C);

/******************************************************************************
 * @brief Inverse of a 3-by-3 matrix A 
 * 
 * @param A         [in]    Matrix A.
 * @param AInverse  [in]    Inverse of matrix A.
 *  
 *****************************************************************************/
void matrixInverse_3x3(real A[3][3], real AInverse[3][3]);

#ifdef __cplusplus
}
#endif

#endif /* MATRIXMATH_H */
