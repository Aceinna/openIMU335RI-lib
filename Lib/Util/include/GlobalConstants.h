/*******************************************************************************
 * File:   GlobalConstants.h
 *******************************************************************************/
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


#ifndef GLOBALCONSTANTS_H
#define GLOBALCONSTANTS_H

#ifdef __cplusplus
 extern "C" {
#endif



#include <stdint.h>


/**
 * @brief
 *
 */
typedef int8_t BOOL;

enum {
	FALSE = 0,
	TRUE  = 1,
};

/**
 * @brief
 *
 * @param x  ==
 * @return float64_t
 */
float64_t SQUARE(float64_t x);

/**
 * @brief
 *
 */
typedef  float32_t real;

// Force Q0 positive by flipping the sign on all quaternion-elements when q0 < 0.  This
//   seems to reduce the errors in the system although (in theory) it shouldn't affect
//   the result.
#define FORCE_Q0_POSITIVE


// Constants
extern float64_t const RAD_TO_DEG;
extern float64_t const DEG_TO_RAD;

extern uint32_t const ROLL_INCIDENCE_LIMIT;
extern uint32_t const PITCH_INCIDENCE_LIMIT;
extern uint32_t const HARD_IRON_LIMIT; // 0.25 G
extern uint32_t const SOFT_IRON_LIMIT; // 20%


// The following is the acceleration due to gravity at the calibration location
extern float64_t const  GRAVITY;

extern uint32_t const MAX_ITOW;    // ms in a week (assuming exactly 24 hours in a day)

// PI and related values
extern  float64_t const  TWO_PI;
extern  float64_t const  PI;

// Specify constants used to limit variables in the algorithm
extern float64_t const ONE_DEGREE_IN_RAD ;
extern float64_t const TWO_DEGREES_IN_RAD;
extern float64_t const TWO_PT_FIVE_DEGREES_IN_RAD;
extern float64_t const THREE_DEGREES_IN_RAD;
extern float64_t const FIVE_DEGREES_IN_RAD;
extern float64_t const SIX_DEGREES_IN_RAD;
extern float64_t const TEN_DEGREES_IN_RAD;
extern float64_t const TWENTY_DEGREES_IN_RAD;
extern float64_t const THREE_HUNDRED_EIGHTY_DEGREES_IN_RAD;

extern float64_t const MIN_TO_MILLISECONDS;

/// Specify the data acquisition task rate of the system in Hz. Due to the way data is collected,
/// this is different than the sampling rate of the sensors.  Note: must be 100 or 200.
enum {
  DACQ_100_HZ      =  100,
  DACQ_200_HZ      =  200,
  DACQ_RATE_INVALID = 0,
};

/// Specify the algorithm execution frequency in Hz.
/// So far only 100 and 200
enum {
  FREQ_50_HZ      =   50,
  FREQ_100_HZ     =   100,
  FREQ_200_HZ     =   200,
  FREQ_INVALID    =   0,
};

// Choices for user communication interface
enum {
  UART_COMM    =   0,
  SPI_COMM     =   1,
  CAN_BUS      =   2,
};


// Choices for GPS protocol type
enum{
    AUTODETECT              = -1,
	UBLOX_BINARY            =  0,
	NOVATEL_BINARY          =  1,
	NOVATEL_ASCII           =  2,
	NMEA_TEXT               =  3,
    DEFAULT_SEARCH_PROTOCOL =  NMEA_TEXT, // 3
	SIRF_BINARY             =  4,
	INIT_SEARCH_PROTOCOL    =  SIRF_BINARY, ///< 4 max value, goes through each until we hit AUTODETECT
	UNKNOWN                 = 0xFF
};


// Algorithm specifiers
enum
{
	IMU = 0,
	AHRS = 1,
	VG = 2,
	INS = 3,
};



enum
{
	USER_PACKET_OK = 0,
	USER_PACKET_ERROR = 2,
};

extern uint32_t const MAXUINT32; ///< max unsigned 32 bit int32_t=> ((2^32)-1)
extern int32_t const MAXINT32; ///< max signed 32 bit int32_t
extern int32_t const MININT32; ///< max negative signed 32 bit int32_t

extern uint16_t const MAXUINT16;  ///< max unsigned 16 bit int32_t=> ((2^16)-1)
extern int16_t const MAXINT16;  ///< max signed 16 bit int32_t=> ((2^15)-1)
extern int16_t const MININT16; ///< max negative signed 16 bit int32_t=> (-(2^15))
extern int16_t const INT16_LIMIT;

// Choices for IMU type
/*******************************************
 * @brief
 *
********************************************/
typedef enum{
	OpenIMU300RI            =  0,
	OpenIMU300ZI            =  1,
	OpenIMU330              =  2,
	OpenIMU335RI            =  3,
	MTLT335                 =  4
} enumIMUType;

extern void *null;

#ifdef __cplusplus
}
#endif

#endif /* GLOBALCONSTANTS_H */
