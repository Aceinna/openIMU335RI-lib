/*****************************************************************************
* @file matrixMath.c real precision Linear algebra calculation functions
* @author
* @date   September, 2008
* @copyright (c) 2013, 2014 All Rights Reserved.
* @section LICENSE
* THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
* PARTICULAR PURPOSE.
* @details
* NOTE these count on the matrices being defined per the inputs.
* They use pointer math instead of array indexing so you can't pass in a
* 7x3 matrix and attempt to calculate using the first two columns
*****************************************************************************/

#include "MatrixMath.h"

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
void AxB( const real* const A,
             const real* const B,
             uint8_t const rowsInA,
             uint8_t const colsInA,
             uint8_t const colsInB,
             real* const C )
{
    uint8_t rowNum;
    uint8_t colNum;
    uint8_t multIndex;

    // Compute A * B
    for (rowNum = 0U; rowNum < rowsInA; rowNum++) {
        for (colNum = 0U; colNum < colsInB; colNum++) {
            C[(rowNum*colsInB) + colNum] = 0.0F;
            for (multIndex = 0U; multIndex < colsInA; multIndex++) {
                C[(rowNum*colsInB) + colNum] += A[(rowNum*colsInA) + multIndex] * B[colNum + (colsInB*multIndex)];
            }
        }
    }
}

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
void AxV( const real* const A,
          const real* const V,
          uint8_t const rowsInA,
          uint8_t const colsInA,
          real* const C )
{
    uint8_t rowNum;
    uint8_t multIndex;

    for (rowNum = 0U; rowNum < rowsInA; rowNum++) {
        C[rowNum] = 0.0F;
        for (multIndex = 0U; multIndex < colsInA; multIndex++) {
            C[rowNum] += A[(rowNum*colsInA) + multIndex] * V[multIndex];
        }
    }
}

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
void AMinusB( const real* const A,
                 const real* const B,
                 uint8_t const rowsInA,
                 uint8_t const colsInA,
                 real* const C )
{
    uint8_t rowNum;
    uint8_t colNum;

    for (rowNum = 0U; rowNum < rowsInA; rowNum++) {
        for (colNum = 0U; colNum < colsInA; colNum++) {
            uint32_t const idx = ((uint32_t)rowNum * (uint32_t)colsInA) + (uint32_t)colNum;
            C[idx] = A[idx] - B[idx];
        }
    }
}

/******************************************************************************
 * @brief Inverse of a 3-by-3 matrix A
 *
 * @param A         [in]    Matrix A.
 * @param AInverse  [in]    Inverse of matrix A.
 *  
 *****************************************************************************/
void matrixInverse_3x3( real A[3][3], real AInverse[3][3] )
{
    real temp[3];
    real detInv;

    temp[0] =  (A[2][2] * A[1][1]) - (A[2][1] * A[1][2]);
    temp[1] = -(A[2][2] * A[0][1]) + (A[2][1] * A[0][2]);
    temp[2] =  (A[1][2] * A[0][1]) - (A[1][1] * A[0][2]);
    detInv = 1.0F / ( (A[0][0] * temp[0]) + (A[1][0] * temp[1]) + (A[2][0] * temp[2]) );

    AInverse[0][0] = temp[0] * detInv;
    AInverse[0][1] = temp[1] * detInv;
    AInverse[0][2] = temp[2] * detInv;

    temp[0] = -(A[2][2] * A[1][0]) + (A[2][0] * A[1][2]);
    temp[1] =  (A[2][2] * A[0][0]) - (A[2][0] * A[0][2]);
    temp[2] = -(A[1][2] * A[0][0]) + (A[1][0] * A[0][2]);
    AInverse[1][0] = temp[0] * detInv;
    AInverse[1][1] = temp[1] * detInv;
    AInverse[1][2] = temp[2] * detInv;

    temp[0] =  (A[2][1] * A[1][0]) - (A[2][0] * A[1][1]);
    temp[1] = -(A[2][1] * A[0][0]) + (A[2][0] * A[0][1]);
    temp[2] =  (A[1][1] * A[0][0]) - (A[1][0] * A[0][1]);
    AInverse[2][0] = temp[0] * detInv;
    AInverse[2][1] = temp[1] * detInv;
    AInverse[2][2] = temp[2] * detInv;
}

