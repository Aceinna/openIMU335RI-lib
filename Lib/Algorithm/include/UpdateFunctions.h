/******************************************************************************
 * @file UpdateFunctions.h
 * @author Joe Motyka
 * @brief Update stage of the EKF.
 * @version 1.0
 * @date 2020-09-09
 * 
 * @copyright Copyright (c) 2020
 * 
 *****************************************************************************/

#ifndef UPDATEFUNCTIONS_H
#define UPDATEFUNCTIONS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>    // for uint8_t, ...
#include "GlobalConstants.h"

// Update rates
enum
{
    TEN_HERTZ_UPDATE = 10U,
    TWENTY_HERTZ_UPDATE = 20U,
    TWENTY_FIVE_HERTZ_UPDATE = 25U,
    FIFTY_HERTZ_UPDATE = 50U,
    ONE_HUNDRED_HERTZ_UPDATE = 100U
};

/******************************************************************************
 * @brief Measurement update stage of the Kalman filter.
 *
 *  
 *****************************************************************************/
void EKF_UpdateStage(void);

/******************************************************************************
 * @brief Kalman filter update stage using GPS position.
 * 
 *  
 *****************************************************************************/
void Update_Pos(void);

/******************************************************************************
 * @brief Kalman filter udpate stage using GPS velocity.
 * 
 *  
 *****************************************************************************/
void Update_Vel(void);

/******************************************************************************
 * @brief Kalman filter update stage using roll and pitch from accel and
 *        (optionally) yaw from either magnetometer or GPS.
 * 
 *  
 *****************************************************************************/
void Update_Att(void);

/******************************************************************************
 * @brief Compute attitude innovation.
 * 
 *  
 *****************************************************************************/
void ComputeSystemInnovation_Att(void);

/******************************************************************************
 * @brief Compute position innovation.
 * 
 *  
 *****************************************************************************/
void ComputeSystemInnovation_Pos(void);

/******************************************************************************
 * @brief Compute velocity innovation.
 * 
 *  
 *****************************************************************************/
void ComputeSystemInnovation_Vel(void);

/******************************************************************************
 * @brief Compuate observation Jacobian.
 * 
 *  
 *****************************************************************************/
void GenerateObservationJacobian_AHRS(void);

/******************************************************************************
 * @brief Compute the covariance matrix of observations in the VG/AHRS mode.
 * 
 *  
 *****************************************************************************/
void GenerateObservationCovariance_AHRS(void);

/******************************************************************************
 * @brief Compute the covariance matrix of observations in the INS mode.
 * 
 *  
 *****************************************************************************/
void GenerateObservationCovariance_INS(void);

#ifdef UNIT_TEST
#include "wrappers.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* UPDATEFUNCTIONS_H */

