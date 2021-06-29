/******************************************************************************
 * @file SensorNoiseParameters.h
 * @author Joe Motyka
 * @brief Default IMU and GPS specs.
 * @version 1.0
 * @date 2020-09-09
 * 
 * @copyright Copyright (c) 2020
 * 
 *****************************************************************************/

#ifndef SENSOR_NOISE_PARAMETERS_H
#define SENSOR_NOISE_PARAMETERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "GlobalConstants.h"

// IMU spec
static real const RW_ODR    = 100.0F;   /* [Hz], The data sampling rate when calculate ARW and VRW.
                                         * ARW = sigma * sqrt(dt) = sigma * sqrt(1/ODR)
                                         */
static real const ARW_300ZA = 8.73e-5F; /* [rad/sqrt(s)], gyro angular random walk, sampled at 100Hz
                                         * 0.3deg/sqrt(Hr) = 0.3 / 60 * D2R = 8.72664625997165e-05rad/sqrt(s)
                                         */
static real const BIW_300ZA = 2.91e-5F; /* [rad/s], gyro bias instability
                                         * 6.0deg/Hr = 6.0 / 3600 * D2R = 2.90888208665722e-05rad/s
                                         */
static real const MAX_BW    = 8.73e-3F; /* [rad/s], max possible gyro bias
                                         * 0.5deg/s = 0.5 * D2R = 0.00872664625997165rad/s
                                         */
static real const VRW_300ZA = 1.0e-3F;  /* [m/s/sqrt(s)], accel velocity random walk, sampled at 100Hz
                                         * 0.06m/s/sqrt(Hr) = 0.06 / 60 = 0.001m/s/sqrt(s)
                                         */
#define BIA_300ZA   10.0e-6 * GRAVITY   /* [m/s/s], accel bias instability
                                         * 10ug = 10.0e-6g * GRAVITY
                                         */
#define MAX_BA      3.0e-3 * GRAVITY    /* [m/s/s], max possible accel bias
                                         * 3mg = 3.0e-3g * GRAVITY
                                         */

// GNSS spec
static real const  R_VALS_GPS_POS_X = 5.0F;
static real const  R_VALS_GPS_POS_Y = 5.0F;
static real const  R_VALS_GPS_POS_Z = 7.5F;

static real const  R_VALS_GPS_VEL_X = 0.025F;
static real const  R_VALS_GPS_VEL_Y = 0.025F;
static real const  R_VALS_GPS_VEL_Z = 0.025F;

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_NOISE_PARAMETERS_H */
