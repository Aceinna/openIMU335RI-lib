/** ***************************************************************************
 * @file BITStatus.h 
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
#ifndef BIT_STATUS_H
#define BIT_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


struct ALGO_STATUS_BIT_BITS {                // bits
    uint16_t algoInitError  : 1;  //   0  algorithm did not initialize in time
    uint16_t algoInitStatus : 1;  //   1  algorithm initializing
    uint16_t turnSwitch     : 1;  //   2  turn switch
    uint16_t algoMode       : 1;  //   3  algorithm mode (eg. high gain mode or low gain mode)
    uint16_t algoError      : 1;  //   4  other algorithm error
    uint16_t rsvd           : 11; //   5:15
};

union ALGO_STATUS_BIT{
    uint16_t                       all;
    struct ALGO_STATUS_BIT_BITS    bit;
};


struct COM_STATUS_BIT_BITS {            //      scope
    uint16_t txQueueOverflow    : 1;    // 0              CAN transmit queue overflow
    uint16_t txQueueOverflowError   : 1;    // 1              CAN transmit queue overflow persisted more than 3 consecutive cycles
    uint16_t rxQueueOverflow        : 1;    // 2              CAN receive queue overflow
    uint16_t rxQueueOverflowError   : 1;    // 3              CAN receive queue overflow persisted more than 3 consecutive cycles
    uint16_t sensorCommStatus       : 3;    // 4:6  chips     inertial sensor communication status
    uint16_t rsvd                   : 9;    // 7:15
};

union COM_STATUS_BIT {
    uint16_t                    all;
    struct COM_STATUS_BIT_BITS  bit;
};


struct SENSOR_STATUS_BITS { // bits                 scope 
    uint32_t accelOverRange         : 3; // 0:2     axes    accel data over-range
    uint32_t rateOverRange          : 3; // 3:5     axes    rate  data over-range
    uint32_t accelDisagreement      : 3; // 6:8     axes    accelerometer data disagreement 
    uint32_t rateDisagreement       : 3; // 9:11    axes    rate sensor data disagreement
    uint32_t accelSensorStatus      : 3; // 12:14   chips   available acceleromenter sensors (HW or SW faults) 
    uint32_t rsvd0                  : 1; // 15
    uint32_t rateSensorStatus       : 3; // 16:18   chips   available rate sensors (HW or SW faults)
    uint32_t accelDegradationError  : 3; // 19:21   axes
    uint32_t rateDegradationError   : 3; // 22:24   axes
    uint32_t accelOverRangeError    : 3; // 25:27   axes
    uint32_t rateOverRangeError     : 3; // 28:30   axes
    uint32_t rsvd1                  : 1; // 31  
};

union SENSOR_STATUS {
    uint32_t                  all;
    struct SENSOR_STATUS_BITS bit;
};

// ************************* //
// *** CRITICAL FAILURES *** //
// ************************* //

// All of these are critical error, any of the bit set in 
// this word, sets HW Error bit in Master BIT
struct HW_ERROR_BITS {	                    // bits   scope
    uint16_t cpuOverTemp            : 1;    // 0                cpu over temperature
    uint16_t cpuOverTempError       : 1;    // 1
    uint16_t sensorOverTemp         : 3;    // 2 - 4  chips     sensor chip over temperature. Checked at on raw data
    uint16_t sensorOverTempError    : 1;    // 5
    uint16_t sensorComm             : 3;    // 6 - 8  chips     communication with this sensor chip is busted
    uint16_t unlockedEEPROM         : 1;    // 9                unlocked EEPROM status
    uint16_t lastResetCause         : 3;    // 10 - 12            last reset cause
    uint16_t powerConsumptionError  : 1;    // 13               power consumption error
    uint16_t extPowerSupError       : 1;    // 14               external power supply error
    uint16_t intPowerSupError       : 1;    // 15               internal power supply error
};

union HW_STATUS {
    uint16_t              all;
    struct HW_ERROR_BITS bit;
};

// All of these are critical error, any of the bit set 
// in this word, sets SW Error bit in Master BIT
struct SW_ERROR_BITS {           // bits       scope
    uint16_t stackOverflowError     : 1;  // 0          stack overflow error
    uint16_t configurationError     : 1;  // 2              configuration critical error
    uint16_t calibrationError       : 3;  // 3:5    chips
    uint16_t overRunStatus          : 1;  // 6
    uint16_t overRunError           : 1;  // 7
    uint16_t applicationCRCError    : 1;  // 8
    uint16_t rsvd                   : 7; // 9:15
};

union SW_STATUS {
    uint16_t                  all;
    struct SW_ERROR_BITS bit;
};

// ************************* //

// ************************* //


//BIT and Status structure
typedef struct {
    union COM_STATUS_BIT    comStatus;         // communicxation status  
    union ALGO_STATUS_BIT   algoStatus;        // algorithm status
    union HW_STATUS         hwStatus;          // hardware  status
    union SW_STATUS         swStatus;          // software  status
    union SENSOR_STATUS     sensorStatus;      // sensor    status
} BITStatusStruct;

typedef struct{
    uint16_t masterStatus           : 1;    // 0    fatal error
    uint16_t hardwareStatus         : 1;    // 1    fatal HW error
    uint16_t softwareStatus         : 1;    // 2    fatal SW error
    uint16_t configurationStatus    : 1;    // 3    configuration struct corrupted
    uint16_t calibrationStatus      : 1;    // 4    calibration partitions corrupted
    uint16_t accelDegradation       : 1;    // 5    only one accel sensor working OR one accel sensor voted out and disagreement between remaining two sensors
    uint16_t rateDegradation        : 1;    // 6    only one rate sensor working OR one rate sensor voted out and disagreement between remaining two sensors
    uint16_t forcedReset            : 1;    // 7    last reset due to WatchDog or Brown Out Detect
    uint16_t applicationCRC         : 1;    // 8    bad application CRC
    uint16_t txOverFlowError        : 1;    // 9    transmit overflow occured more than 3 times
    uint16_t rsvd                   : 6;    // 10:15
}master_status_word_bits;


typedef union{
    uint16_t            all;
    master_status_word_bits  bit;
}master_status_word; 

typedef struct{
    uint32_t stackOverflowError     : 1;    // 0 
    uint32_t algoError              : 1;    // 1
    uint32_t algoInitStatus         : 1;    // 2
    uint32_t rsvd0                  : 1;    // 3
    uint32_t accelOverRangeStatus   : 3;    // 4:6
    uint32_t rateOverRangeStatus    : 3;    // 7:9
    uint32_t configurationError     : 1;    // 10
    uint32_t calibrationDataStatus  : 3;    // 11:13
    uint32_t accelSensorStatus      : 3;    // 14:16
    uint32_t rateSensorStatus       : 3;    // 17:19
    uint32_t accelDisagreement      : 1;    // 20
    uint32_t rateDisagreement       : 1;    // 21
    uint32_t lastResetStatus        : 3;    // 22:24
    uint32_t dataProcessingOverRunStatus : 1;// 25
    uint32_t turnSwitchStatus       : 1;    // 26
    uint32_t algoMode               : 1;    // 27
    uint32_t txQueueOverflowStatus  : 1;    // 28
    uint32_t rsvd1                  : 3;    // 29:31
}sw_status_word_bits;

typedef union{
    uint32_t        all;
    sw_status_word_bits  bit;
}sw_status_word; 

typedef struct{
    uint16_t powerConsumptionError      : 1;    // 0 = normal, 1 = unit consumes exessive amount of power
    uint16_t extPowerSupError           : 1;    // 0 = normal, 1 = external power supply voltage is not within specified threshold
    uint16_t intPowerSupError           : 1;    // 0 = normal, 1 = over/under voltage in internal power supply
    uint16_t cpuOverTemp                : 1;    // 0 = normal, 1 = MCU temp exceeds allowed limit
    uint16_t chipOverTemp               : 3;    // 0 = normal, 1 = sensor temp exceeds allowed limit         (chips)
    uint16_t sensorCommStatus           : 3;    // 0 = normal, 1 = communication with sensor chip[bit] faild (chips)
    uint16_t rsvd0                      : 6;
}hw_status_word_bits;


typedef union{
    uint16_t         all;
    hw_status_word_bits  bit;
}hw_status_word; 


enum
{
    // NORMAL RESETS
    POWER_ON_RESET      =   0U,
    SOFTWARE_RESET      =   1U,
    RESERVED0           =   2U,
    RESERVED1           =   3U,

    // FORCED RESETS
    WATCHDOG_RESET      =   4U,
    BROWNOUT_RESET      =   5U,
    TXOVERFLOW_RESET    =   6U,
    MAX_RESET           =   7U
};

enum {
    EXT_POWER_DTC_MASK         = 0x0001U,
    EXT_VOLTAGE_DTC_MASK       = 0x0002U,
    INT_POWER_DTC_MASK         = 0x0004U,
    CPU_OVER_TEMP_DTC_MASK     = 0x0008U,
    COMM_ERROR_DTC_MASK        = 0x0010U,   // sticky - cannot be reset
    ALGO_ERROR_DTC_MASK        = 0x0020U,
    STACK_OVERFLOW_DTC_MASK    = 0x0040U,   // sticky - cannot be reset
    SENSOR_OVER_TEMP_DTC_MASK  = 0x0080U,
    CONFIG_ERROR_DTC_MASK      = 0x0100U,   // sticky - cannot be reset
    APP_ERROR_DTC_MASK         = 0x0200U,   // sticky - cannot be reset
};

enum {
    EXT_VOLTAGE_MAX  = 0,
    INT_VOLTAGE_1    = 2,
    INT_VOLTAGE_2    = 3,
    INT_VOLTAGE_3    = 4,
    INT_VOLTAGE_4    = 5,
    CPU_TEMP         = 6,
};

static uint16_t  const  BIT_OUT_PERIOD                  =   8U;                 // so far 10 second
static uint16_t  const  BIT_CAL_TEST_PERIOD             =   1U ;                // so far every cycle
static uint16_t  const  BIT_CONFIG_TEST_PERIOD          =   200U;               // so far every second
static uint16_t  const  BIT_STACK_TEST_PERIOD           =   10U;                // so far every 50 ms
static uint32_t  const  BIT_CAN_OVERFLOW_THRESHOLD      =   (uint32_t)((uint32_t)200U * (uint32_t)20U);       // 20 seconds
static uint32_t  const  BIT_CAN_TX_OVERFLOW_THRESHOLD   =   3U;                 // Can we afford to loose some message?
static uint32_t  const  BIT_CAN_RX_OVERFLOW_THRESHOLD   =   5U;                 // Less severe than transmit - host can retry
static uint32_t  const  BIT_TEMP_HI_THRESHOLD           =   (uint32_t)((uint32_t)200U * (uint32_t)60U * (uint32_t)5U);  // 5 minutes
static uint32_t  const  BIT_TEMP_LOW_THRESHOLD          =   (uint32_t)((uint32_t)200U * (uint32_t)60U * (uint32_t)1U);  // 1 minute
static uint16_t  const  BIT_ALGO_INIT_THRESHOLD         =   (uint16_t)((uint16_t)200U * (uint16_t)3U);        // 3 seconds
static uint16_t  const  BIT_MAX_OVERRANGE_LIMIT         =   4U;                 // 4 data processing cycles
static uint16_t  const  BIT_MIN_OVERRANGE_LIMIT         =   2U;                 // 2 data processing cycles
static uint16_t  const  DATA_OVERRUN_MAX                =   5U;                 // data processing overrun threshold
static uint16_t  const  BIT_TEMP_CPU_THRESHOLD          =   300U;               // checked every second - 5 minutes
static uint16_t  const  FIFO_RESET_CAUSE_EEPROM         =   0x0001U;
static uint16_t  const  POWER_OUTAGE_TRIGGER_TIME       =   60U;                // checked every second - 1 minute

static uint16_t  const  BIT_FMI_CODE_OK                 =   0U ; 
static uint16_t  const  BIT_FMI_CODE_NOTIFY             =   1U ; 
static uint16_t  const  BIT_FMI_CODE_WARNING            =   2U ; 
static uint16_t  const  BIT_FMI_CODE_FAILURE            =   3U ; 
static uint16_t  const  BIT_DM1_PERIOD                  =   2000U ;             // 10 seconds with 200Hz DACQ task cycle 

#ifdef __cplusplus
}
#endif


#endif

