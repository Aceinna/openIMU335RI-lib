/*
 * File:   ekfAPI.h
 * Author: joemotyka
 *
 * Created on May 8, 2016, 12:23 AM
 */

#ifndef EKF_API_H
#define EKF_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "GlobalConstants.h"
#include "gpsAPI.h" 
#include "odoAPI.h"

/******************************************************************************
 * @brief Initialize the EKF algorithm.
 * 
 * @param dacqFreq  [in]    Calling frequency. In units of [Hz]. For now,
 *                          only 100Hz and 200Hz are supported.
 * @param imuType   [in]    Specify the IMU type to choose different set of
 *                          IMU specifications.
 *  
 *****************************************************************************/
void EKF_Initialize(uint32_t const dacqFreq, enumIMUType const imuType);

/******************************************************************************
 * @brief Get the algorithm timer.
 * 
 * @return uint32_t 
 *****************************************************************************/
uint32_t EKF_GetTimer();

/******************************************************************************
 * @brief Get the algorithm counter.
 * 
 * @return uint16_t 
 *****************************************************************************/
uint16_t EKF_GetCounter();

/******************************************************************************
 * @brief Get the execution frequency of the algorithm.
 * 
 * @return uint32_t 
 *****************************************************************************/
uint32_t EKF_GetCallingFreq();

/******************************************************************************
 * @brief Main routine of the EKF algorithm.
 * 
 *  
 *****************************************************************************/
void EKF_Algorithm(void);

/******************************************************************************
 * @brief Get the attitude (expressed as Euler angles in roll, pitch and yaw
 *        order) of the body-frame in the NED-frame.
 * 
 * @param EulerAngles   [out]   Euler angles in roll, pitch and yaw order. [deg].
 *  
 *****************************************************************************/
void EKF_GetAttitude_EA(real EulerAngles[]);

/******************************************************************************
 * @brief Get the attitude (expressed as Euler angles in roll, pitch and yaw
 *        order) of the body-frame in the NED-frame.
 * 
 * @param EulerAngles   [out]   Euler angles in roll, pitch and yaw order. [rad].
 *  
 *****************************************************************************/
void EKF_GetAttitude_EA_RAD(real EulerAngles[]);

/******************************************************************************
 * @brief Get the attitude quaternion of the body-frame in the NED-frame.
 * 
 * @param Quaternions   [out]   Attitude quaternion with scalar first.
 *  
 *****************************************************************************/
void EKF_GetAttitude_Q(real Quaternions[]);

/******************************************************************************
 * @brief Get angular rate corrected for estimated rate bias.
 * 
 * @param CorrAngRates_B    [out]   Corrected angular rate. [dps].
 *  
 *****************************************************************************/
void EKF_GetCorrectedAngRates(real CorrAngRates_B[]);

/******************************************************************************
 * @brief Get accleration corrected for estimated accleratio bias.
 * 
 * @param CorrAccels_B  [out]   Corrected acceleration. [m/s^2]l
 *  
 *****************************************************************************/
void EKF_GetCorrectedAccels(real CorrAccels_B[]);

/******************************************************************************
 * @brief Get estimated gyro bias.
 * 
 * @param AngRateBias_B [out]   Estimated gyro bias. [dps].
 *  
 *****************************************************************************/
void EKF_GetEstimatedAngRateBias(real AngRateBias_B[]);

/******************************************************************************
 * @brief Get estimated accel bias.
 * 
 * @param AccelBias_B   [out]   Estimated accel bias. [m/s^2].
 *  
 *****************************************************************************/
void EKF_GetEstimatedAccelBias(real AccelBias_B[]);

/******************************************************************************
 * @brief Get the estimated NED position.
 * 
 * @param Position_N    [out]   NED position. [m].
 *  
 *****************************************************************************/
void EKF_GetEstimatedPosition(real Position_N[]);

/******************************************************************************
 * @brief Get the estimated NED velocity.
 * 
 * @param Velocity_N    [out]   NED velocity. [m/s].
 *  
 *****************************************************************************/
void EKF_GetEstimatedVelocity(real Velocity_N[]);

/******************************************************************************
 * @brief Get the estimated latitude, longitude and altitude.
 * 
 * @param LLA   [out]   [latitude, longitude, altitude]. [deg, deg, rad].
 *  
 *****************************************************************************/
void EKF_GetEstimatedLLA(float64_t LLA[]);

/******************************************************************************
 * @brief Get operation mode of the algorithm.
 *          0: Stabilize
 *          1: Initialize
 *          2: High-Gain VG/AHRS mode
 *          3: Low-Gain VG/AHRS mode
 *          4: INS operation
 * @param EKF_OperMode  [out]   Operation mode.
 *  
 *****************************************************************************/
void EKF_GetOperationalMode(uint8_t* const EKF_OperMode);

/******************************************************************************
 * @brief Get the linear-acceleration and turn-switch flags
 * 
 * @param EKF_LinAccelSwitch    [out]   Linear acceleration switch flag.
 * @param EKF_TurnSwitch        [out]   Turn swith flag.
 *  
 *****************************************************************************/
void EKF_GetOperationalSwitches(uint8_t* const EKF_LinAccelSwitch,
                                uint8_t* const EKF_TurnSwitch);

/******************************************************************************
 * @brief Get angular rate corrected for estimated rate bias.
 * 
 * @param corrRates_B   [out]   Corrected angular rate. [dps].
 *   
 *****************************************************************************/
void EKF_GetCorrectedRates_B(real corrRates_B[]);

// Setter functions
/******************************************************************************
 * @brief Feed sensor measurement to the algorithm.
 * 
 * @param accels        [in]    Accel measurement. [g].
 * @param rates         [in]    Gyro measurement. [rad/s].
 * @param mags          [in]    Magnetometer measurements. [G].
 * @param gps           [in]    GPS measurements.
 * @param odo           [in]    Odometer measurements.
 * @param ppsDetected   [in]    True if PPS is just detected, FALSE if not. 
 *  
 *****************************************************************************/
void EKF_SetInputStruct(float64_t accels[],
                        float64_t rates[],
                        float64_t mags[],
                        const gpsDataStruct_t* const gps,
                        const odoDataStruct_t* const odo,
                        BOOL const ppsDetected);

/******************************************************************************
 * @brief Update algorithm output.
 * 
 *  
 *****************************************************************************/
void EKF_SetOutputStruct(void);

/******************************************************************************
 * @brief Update algorithm configurations by user configurations that are 
 *        read from EEPROM.
 * 
 *****************************************************************************/
 void UpdateUserAlgoCfg();

#ifdef UNIT_TEST
#include "wrappers.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* EKF_API_H */

