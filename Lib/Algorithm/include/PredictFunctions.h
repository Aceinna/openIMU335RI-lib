/******************************************************************************
 * @file PredictFunctions.h
 * @author Joe Motyka
 * @brief Prediction stage of the EKF.
 * @version 1.0
 * @date 2020-09-09
 * 
 * @copyright Copyright (c) 2020
 * 
 *****************************************************************************/

#ifndef PREDICT_FUNCTIONS_H
#define PREDICT_FUNCTIONS_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * @brief Prediction stage of the Kalman filter.
 * 
 * @param filteredAccel [in]    Acceleration after low-pass filter in the
 *                              algorithm, [m/s/s]
 *  
 *****************************************************************************/
void EKF_PredictionStage(real filteredAccel[]);

/******************************************************************************
 * @brief Initialize the process covariance matrix Q.
 * 
 *  
 *****************************************************************************/
void GenerateProcessCovariance(void);

/******************************************************************************
 * @brief Initialize the process Jacobian.
 * 
 *  
 *****************************************************************************/
void GenerateProcessJacobian(void);

#ifdef UNIT_TEST
#include "wrappers.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* PREDICT_FUNCTIONS_H */
