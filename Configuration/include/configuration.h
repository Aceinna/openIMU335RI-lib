/** ***************************************************************************
 * @file   configuration.h  Configuration data structure
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 ******************************************************************************/
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

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#ifdef __cplusplus
 extern "C" {
#endif


#include <stdint.h>

/// constants for port type configuration fields (see DMU specification)
   enum
   {
     BAUD_CODE_UNDEFINED = -1,
     BAUD_CODE_9600 = 0,
     BAUD_CODE_19200 = 1,
     BAUD_CODE_38400 = 2,
     BAUD_CODE_57600 = 3,
     BAUD_CODE_4800 = 4,
     BAUD_CODE_115200 = 5,
     BAUD_CODE_230400 = 6,
     NUM_BAUD_RATES = 8,
   };

 enum
   {
      BAUD_RATE_4800   = 4800,
      BAUD_RATE_9600   = 9600,
      BAUD_RATE_19200  = 19200,
      BAUD_RATE_38400  = 38400,
      BAUD_RATE_57600  = 57600,
      BAUD_RATE_115200 = 115200,
      BAUD_RATE_230400 = 230400,
   };


// xbowsp_fields.c (set default at startup to 20 Hz, AU pckt at 57.6 kbps)

static uint16_t const    DEFAULT_PACKET_RATE_DIVIDER	=  0U;
static uint16_t const    DEFAULT_PACKET_CODE          =  0x5331U;    // S1
static uint16_t const    DEFAULT_BAUD_RATE              =  (uint16_t)BAUD_CODE_115200;
static int16_t const     MSEC_IN_DACQ_PERIOD          =  5;  // 200 Hz
static uint32_t const    SERIAL_TX_ROUTINE_FREQUENCY  = 100U; ///< Hz

// here is definition for packet rate divider
// considering that data acquisition task runs at 200 Hz
enum {
    PACKET_RATE_DIV_INVALID = -1,
    PACKET_RATE_DIV_QUIET   = 0,      // quiet mode
    PACKET_RATE_DIV_200HZ   = 200,    // packet rate 200 Hz
    PACKET_RATE_DIV_100HZ   = 1,      // packet rate 100 Hz
    PACKET_RATE_DIV_50HZ    = 2,      // packet rate 50 Hz
    PACKET_RATE_DIV_25HZ    = 4,      // packet rate 25 Hz
    PACKET_RATE_DIV_20HZ    = 5,     // packet rate 20 Hz
    PACKET_RATE_DIV_10HZ    = 10,     // packet rate 10 Hz
    PACKET_RATE_DIV_5HZ     = 20,     // packet rate 5  Hz
    PACKET_RATE_DIV_2HZ     = 50,    // packet rate 2  Hz
    PACKET_RATE_DIV_1HZ     = 100,    // packet rate 1  Hz
};



/// EEPROM Data Structure: configuration data (NOTE: Refer to the DMU Serial Interface
///                        spec before changing this structure).  All variables must
///                        fit in the space allocated.
typedef struct {
    uint16_t           calibrationCRC;    /// CRC on the calibration structure  0x0000
    uint16_t           packetRateDivider; /// continuous packet rate divider    0x0001
    uint16_t           baudRateUser;      /// user port                         0x0002
    uint16_t           packetCode;		  /// continuous packet 2 bytes code    0x0003

    uint16_t           analogFilterClocks[3];                               //  0x0004, 0x0005, 0x0006

    uint16_t           orientation; 	  /// user defined axis orientation     0x0007

    uint16_t           userBehavior;       // uint16_t                          0x0008
    int16_t            hardIronBias[2];    ///< [-1,1) Gauss                    0x0009, 0x000a
    uint16_t           softIronScaleRatio; ///< [0,2), [0-200%)                 0x000b
    uint16_t           headingTrackOffset;                                  //  0x000c
    uint16_t           turnSwitchThreshold; // 0, 0.4, 10 driving, 1 flying [deg/sec]   0x000d
    int16_t            softIronAngle;                                       //  0x000e
    int16_t            testFreq;                                            //  0x000f

    uint16_t           hardwareStatusEnable;                                //  0x0010
    uint16_t           comStatusEnable;                                     //  0x0011
    uint16_t           softwareStatusEnable;                                //  0x0012
    uint16_t           sensorStatusEnable;                                  //  0x0013
    int16_t            baudRateGPS; // enum in driverGPS.h                      0x0014
    int16_t            protocolGPS; // enum enumGPSProtocol in driverGPS.h      0x0015
    int16_t            baroCorrection;                                      //  0x0016
    int16_t            OffsetAnglesExtMag[2];                               //  0x0017, 0x0018
    int16_t            OffsetAnglesAlign[3];                                //  0x0019, 0x001a, 0x001b
    int16_t            hardIronBiasExt[2];                                  //  0x001c, 0x001d
    uint16_t           softIronScaleRatioExt;                               //  0x001e
    int16_t            softIronAngleExt;                                    //  0x001f
    uint16_t           orientationExt; // uint16_t 32(128)?                     0x0020
    // 33 16-bit spaces used

    // The MTLT uses twelve 16-bit fields
    int16_t            portOneUsage;                                       // 0x0021
    int16_t            portTwoUsage;                                       // 0x0022
    int16_t            portThreeUsage;                                     // 0x0023
    int16_t            portFourUsage;                                      // 0x0024
    int16_t            portOneBaudRate;                                    // 0x0025
    int16_t            portTwoBaudRate;                                    // 0x0026
    int16_t            portThreeBaudRate;                                  // 0x0027
    int16_t            portFourBaudRate;                                   // 0x0028

    int16_t	           rollUpperAlarmLimit;                                // 0x0029
    int16_t	           rollLowerAlarmLimit;                                // 0x002a
    int16_t	           pitchUpperAlarmLimit;                               // 0x002b
    int16_t	           pitchLowerAlarmLimit;                               // 0x002c
    uint16_t 	       rollHysteresis; 		//Hysteresis for Roll alarm 0x002d
    uint16_t	       pitchHysteresis; 	//Hysteresis for Pitch alarm 0x002e
    uint16_t	       alarmSelector;		//Cone/Independent axis alarm selector 0x002f
    uint16_t	       coneAngleLimit;		//angle = acos( cos( phi ) * cos( theta ) ) * 180/pi 0x0030
    uint16_t	       coneAngleHysteresis; //Hysteresis for cone angle alarm  0x0031

    uint16_t           ecuAddress;                                              // 0x0032
    uint16_t           ecuBaudRate;                                             // 0x0033
    uint16_t           algResetSaveCfgPs;                                       // 0x0034
    uint16_t           HardSoftBitPs;                                           // 0x0035
    uint16_t           statusPrPs;                                              // 0x0036
    uint16_t           PtDfPs;                                                  // 0x0037
    uint16_t           OrienUserBehvPs;                                         // 0x0038
    uint16_t           AngConeAlarmPs;                                          // 0x0039
    int16_t            CanBaudRateDetectEnable;                                 // 0x003a
    int16_t            CanTermResistorEnable;                                   // 0x003b
    int16_t            CanOdr;                                                  // 0x003c
    uint16_t           canPacketType;                                           // 0x003d

    uint16_t           spiDataReadyConfig;                                      // 0x003e
    uint16_t           spiDataRate;                                             // 0x003f
    uint16_t           spiRateDynamicRange;                                     // 0x0040
    uint16_t           spiLpfType;                                              // 0x0041

    uint16_t           activeChips;                                             // 0x0042
    uint16_t           usedChips;                                               // 0x0043

    uint16_t           usedSensors[16];                                         // 0x44 - 0x53
    uint16_t           accelRange;                                              // 0x54
    uint16_t           gyroRange;                                               // 0x55
    int16_t            accelCheckImprobabilityPeriod;                           // 0x0056   miliseconds min 250
    int16_t            accelSignalConsistencyPeriod;                            // 0x0057   miliseconds min 100
    int16_t            ratesSignalConsistencyPeriod;                            // 0x0058   miliseconds min 100

    uint16_t           rsvd1[3];                                                // 0x0059, 5a, 5B
    int16_t            accelSignalConsistencyThreshold;                         // 0x005C   min 100  (mG)
    int16_t            ratesSignalConsistencyThreshold;                         // 0x005D   min 1000 (deg/s)

    uint16_t           rsvd2[3];                                                // 0x005E, 5f, 60
    uint16_t           accelConsistencyCheckEnable;                             // 0x0061
    uint16_t           ratesConsistencyCheckEnable;                             // 0x0062
    uint16_t           sfCorrection;                                            // 0x63
    uint16_t           ecuUnitBehavior;                                         // 0x064
    uint16_t           gyroSamplingCountsThreshold;                             // 0x065
    uint16_t           accelSamplingCountsThreshold;                            // 0x066
    uint16_t           rsvd3[24];                                               // 0x067 - 0x07E
    uint16_t           crc;
} FactoryConfigurationStruct;



/*******************************************
 * @brief 
 * 
 * @param baudEnum ==
 * @return int32_t 
********************************************/
int32_t baudEnumToBaudRate(int32_t baudEnum);

enum {
  FORWARD =  0,
  RIGHT   =  1,
  DOWN    =  2,
};

enum{
  PLUS_X  =  0x582B,
  PLUS_Y  =  0x592B,
  PLUS_Z   = 0x5A2B,
  MINUS_X  = 0x582D,
  MINUS_Y =  0x592D,
  MINUS_Z =  0x5A2D,
};


enum{
  FWD_X_PLUS_MASK	=   0x00000000,
  FWD_X_MINUS_MASK =  0x00000001,
  FWD_Y_PLUS_MASK	=   0x00000002,
  FWD_Y_MINUS_MASK =  0x00000003,
  FWD_Z_PLUS_MASK	=   0x00000004,
  FWD_Z_MINUS_MASK =  0x00000005,

  RIGHT_X_PLUS_MASK = 0x00000020,
  RIGHT_X_MINUS_MASK =0x00000028,
  RIGHT_Y_PLUS_MASK = 0x00000000,
  RIGHT_Y_MINUS_MASK =0x00000008,
  RIGHT_Z_PLUS_MASK = 0x00000010,
  RIGHT_Z_MINUS_MASK =0x00000018,

  DOWN_X_PLUS_MASK  = 0x00000080,
  DOWN_X_MINUS_MASK = 0x000000C0,
  DOWN_Y_PLUS_MASK  = 0x00000100,
  DOWN_Y_MINUS_MASK = 0x00000140,
  DOWN_Z_PLUS_MASK  = 0x00000000,
  DOWN_Z_MINUS_MASK = 0x00000040,
};

enum{
  USED_ACCELS_ON_CHIP_MASK=0x0007,
  USED_GYROS_ON_CHIP_MASK = 0x0038,
};

enum{
      PACKET_RATE_200HZ  = 200,
      PACKET_RATE_100HZ  = 100,
      PACKET_RATE_50HZ   = 50,
      PACKET_RATE_25HZ   = 25,
      PACKET_RATE_20HZ   = 20,
      PACKET_RATE_10HZ   = 10,
      PACKET_RATE_5HZ    = 5,
      PACKET_RATE_2HZ    = 2,
      PACKET_RATE_QUIET  = 0,
};

enum {
    ORIENTATION_0     =  0      ,
    ORIENTATION_9     =  9      ,
    ORIENTATION_35    =  35     ,
    ORIENTATION_42    =  42     ,
    ORIENTATION_65    =  65     ,
    ORIENTATION_72    =  72     ,
    ORIENTATION_98    =  98     ,
    ORIENTATION_107   =  107    ,
    ORIENTATION_133   =  133    ,
    ORIENTATION_140   =  140    ,
    ORIENTATION_146   =  146    ,
    ORIENTATION_155   =  155    ,
    ORIENTATION_196   =  196    ,
    ORIENTATION_205   =  205    ,
    ORIENTATION_211   =  211    ,
    ORIENTATION_218   =  218    ,
    ORIENTATION_273   =  273    ,
    ORIENTATION_280   =  280    ,
    ORIENTATION_292   =  292    ,
    ORIENTATION_301   =  301    ,
    ORIENTATION_336   =  336    ,
    ORIENTATION_345   =  345    ,
    ORIENTATION_357   =  357    ,
    ORIENTATION_364   =  364    ,
};


#ifdef __cplusplus
}
#endif


#endif
