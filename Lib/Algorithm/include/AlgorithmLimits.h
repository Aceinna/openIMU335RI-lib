/******************************************************************************
 * @file AlgorithmLimits.h
 * @author Xiaoguang Dong (xgdong@aceinna.com)
 * @brief 
 * @version 0.1
 * @date 2020-09-09
 * 
 * @copyright Copyright (c) 2020
 * 
 *****************************************************************************/


#ifndef ALGORITHM_LIMITS_H
#define ALGORITHM_LIMITS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "GlobalConstants.h"

static real const INIT_P_Q    = 1.0e-5F;
static real const INIT_P_WB   = 1.0e-5F;
static real const INIT_P_INS  = 1.0e-3F;

// Declare the limits
static real const LIMIT_P     = 500.0F;
static real const LIMIT_MIN_GPS_VELOCITY_HEADING  = 0.45F;        //0.45 m/s ~= 1.0 mph
static real const RELIABLE_GPS_VELOCITY_HEADING   = 1.0F;         // velocity of 1.0m/s should provide reliable GNSS heading

// The following times are compared against ITOW (units in [msec])
static uint32_t const LIMIT_MAX_GPS_DROP_TIME                    = 300U;    // [sec]
static uint32_t const LIMIT_RELIABLE_DR_TIME                     = 10U;     // [sec]
static uint32_t const LIMIT_MAX_REST_TIME_BEFORE_HEADING_INVALID = 120000U; // 120sec, heading drifts much slower than pos

// The following is compared against a counter (in units of the calling frequency of the EKF)
static uint8_t const LIMIT_FREE_INTEGRATION_CNTR   = 60U;                // 60 [ sec ]
static real const LIMIT_QUASI_STATIC_STARTUP_RATE  = 0.087266462599716F; // (5.0 * ONE_DEGREE_IN_RAD)

#ifdef __cplusplus
}
#endif

#endif /* ALGORITHM_LIMITS_H */
