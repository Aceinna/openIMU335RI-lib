/*
 * File:   algoAPI.h
 *
 * Created on Jul 22, 2020, 12:23 AM
 */

#ifndef ALGO_API_H
#define ALGO_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "GlobalConstants.h"

/******************************************************************************
 * @brief Get the linear acceleration detection (LAD) mode in the algorithm.
 * 
 * @return TRUE if using raw accel for LAD, FALSe if using filtered accel.
 *****************************************************************************/
BOOL   GetAlgorithmLinAccelDetectMode();

/******************************************************************************
 * @brief Get the acceleration prediction (AP) mode in the algorithm.
 * 
 * @return TRUE if using raw rate for AP, FALSE if using corrected rate.
 *****************************************************************************/
BOOL   GetAlgorithmAccelPredictMode();

/******************************************************************************
 * @brief Get the coefficient to reduce state covariance of the gyro bias.
 * 
 * @return The coefficient.
 *****************************************************************************/
float32_t  GetAlgorithmCoefOfReduceQ();

/******************************************************************************
 * @brief Get the delay to switch to stationary mode when no linear acceleration
 *        is detected.
 * 
 * @return the dealy. [sec].
 *****************************************************************************/
float32_t  GetAlgorithmAccelSwitchDelay();

/******************************************************************************
 * @brief Rate is used to porpgate accel to a period of time. After this period
 *        of time, the propagation is reset.
 * 
 * @return The period of time fro propagation. [sec].
 *****************************************************************************/
float32_t  GetAlgorithmRateIntegrationTime();

#ifdef __cplusplus
}
#endif

#endif /* ALGO_API_H */

