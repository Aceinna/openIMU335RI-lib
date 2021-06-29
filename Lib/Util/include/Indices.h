/** ***************************************************************************
 * @file Indices.h
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *****************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/

#ifndef INDICES_H
#define INDICES_H

#ifdef __cplusplus
extern "C" {
#endif

enum {
    X_AXIS   =  0U,
    Y_AXIS   =  1U,
    Z_AXIS   =  2U,
    NUM_AXIS =  3U
};

enum {
    ROLL     =  0U,
    PITCH    =  1U,
    YAW      =  2U
};

enum {
    DUMMY_SENSOR = 0U,
    ACCEL_SENSOR = 1U,
    RATE_SENSOR  = 2U,
    MAG_SENSOR   = 3U,
};

/* raw sensor order  (from xbowsp_algorithm.h) */
enum {
    XACCEL     = 0U,
    YACCEL     = 1U,
    ZACCEL     = 2U,
    XRATE      = 3U,
    YRATE      = 4U,
    ZRATE      = 5U,
    XMAG       = 6U,
    CHIP_TEMP  = 6U,
    YMAG       = 7U,
    ZMAG       = 8U,
    XATEMP     = 9U,
    YATEMP     = 10U,
    ZATEMP     = 11U,
    XRTEMP     = 12U,
    YRTEMP     = 13U,
    ZRTEMP     = 14U,
    BTEMP      = 15U,
    N_RAW_SENS = 16U // size
};

/// raw sensor order (from dmu.h)
enum {
    ACCEL_START         = 0U,
    RATE_START          = 3U,
    MAG_START           = 6U,
    NUM_SENSOR_IN_AXIS  = 9U,
    TEMP_START          = 9U,
    GYRO_TEMP           = 9U,
    TEMP_SENSOR         = 10U,
    NUM_SENSOR_READINGS = 11U
};

enum {
    NUM_SENSOR_CHIPS    = 3U,
    NUM_SENSORS         = 3U,
};

enum {
    Q0 = 0U,
    Q1 = 1U,
    Q2 = 2U,
    Q3 = 3U,
};


enum {
    LAT = 0U,
    LON = 1U,
    ALT = 2U,
};

enum {
  GPS_NORTH = 0U,
  GPS_EAST  = 1U,
  GPS_DOWN  = 2U
};

// Size of EKF matrices
enum {
    ROWS_IN_P  = 16U,
    COLS_IN_P  = ROWS_IN_P,
    ROWS_IN_F  = 16U,
    COLS_IN_F  = ROWS_IN_F,
    ROWS_IN_H  = 3U,
    COLS_IN_H  = 16U,
    ROWS_IN_R  = 3U,
    COLS_IN_R  = ROWS_IN_R,
    ROWS_IN_K  = 16U
};

#ifdef __cplusplus
}
#endif


#endif /* INDICES_H */
