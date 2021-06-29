/** ***************************************************************************
 * @file aceinna_sae_j1939.h definitions of SAE J1939 standard
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef SAE_J1939_H
#define SAE_J1939_H
#include "ecu.h"

#ifdef __cplusplus
extern "C"{
#endif

// J1939 Name definition
enum  {
    ACEINNA_SAE_J1939_VEHICLE_SYSTEM               = 0,
    ACEINNA_SAE_J1939_VEHICLE_SYSTEM_INSTANCE      = 0,
    ACEINNA_SAE_J1939_FUNCTION                     = 131,
    ACEINNA_SAE_J1939_FUNCTION_INSTANCE            = 0,
    ACEINNA_SAE_J1939_ECU                          = 0,
    ACEINNA_SAE_J1939_MANUFACTURER_CODE            = 823,
};


// J1939 PF definition, see protocol spec
// Broadcast PFs - 240 to 255
enum {
    SAE_J1939_PDU_FORMAT_DATA           =        240,
    SAE_J1939_PDU_FORMAT_ECU            =        253,
    SAE_J1939_PDU_FORMAT_254            =        254,
    SAE_J1939_PDU_FORMAT_240            =        240,
    SAE_J1939_PDU_FORMAT_239            =        239,
    SAE_J1939_PDU_FORMAT_251            =        251,
    SAE_J1939_PDU_FORMAT_GLOBAL         =        255,
    SAE_J1939_PDU_FORMAT_ACK            =        232,
    SAE_J1939_PDU_FORMAT_REQUEST        =        234,
    SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM  =        238,
    SAE_J1939_PDU_FORMAT_STARTSTOP      =        223,
    SAE_J1939_PDU_FORMAT_MA_RESPONSE    =        216,
    SAE_J1939_PDU_FORMAT_MA_REQUEST     =        217,

    // Below PS numbers are not j1939 standard PS
    // They are there to accomodate StandardIDs

    // Generic for id below 256, add specific PF for id above 255
    STANDARD_ID_BELOW_256               =        0,
    STANDARD_ID_COROLA_GEAR             =        3,
};



// J1939 PS definition, see protocol spec
enum {
    SAE_J1939_PDU_SPECIFIC_25                    = 25,
    SAE_J1939_PDU_SPECIFIC_243                   = 243,	// 65267 Vehicle Position 1
    SAE_J1939_PDU_SPECIFIC_191                   = 191,	// 65215 Wheel Speed Information
    SAE_J1939_PDU_SPECIFIC_232                   = 232,	// 65256 Vehicle Direction/Speed
	SAE_J1939_PDU_SPECIFIC_0 					 = 0,	// 126720 JD Machine Speed
    SAE_J1939_PDU_SPECIFIC_1                     = 1,
    SAE_J1939_PDU_SPECIFIC_2                     = 2,
    SAE_J1939_PDU_SPECIFIC_246                   = 246,	// 654502 Vehicle GNSS DOP
    SAE_J1939_PDU_SPECIFIC_110                   = 110,	// 65390  CUSTOM_GNSS_TIME
    SAE_J1939_PDU_SPECIFIC_111                   = 111,	// 65391  CUSTOM_GNSS_FIX
    SAE_J1939_PDU_SPECIFIC_112                   = 112,	// 65392  CUSTOM_GNSS_DOP
    SAE_J1939_PDU_SPECIFIC_211                   = 211,	// 65235  DM11
    SAE_J1939_PDU_SPECIFIC_DM1                   = 202,	// 65226  DM1

    STANDARD_ID_COROLA_WS_170                    = 170, // Corola Wheelspeed StdId 170
    STANDARD_ID_COROLA_GEAR_188                  = 188, // Corola Gear StdId 956
};

enum {
    SAE_J1939_ACK  = 0,
    SAE_J1939_NACK = 1,
};

// J1939 PS definition
enum {
    SAE_J1939_GROUP_EXTENSION_ECU                = 197,
    SAE_J1939_GROUP_EXTENSION_SOFTWARE_VERSION   = 218,
    SAE_J1939_GROUP_EXTENSION_DM1                = 202,
    SAE_J1939_GROUP_EXTENSION_ALGORITHM_RESET    = 80,
    SAE_J1939_GROUP_EXTENSION_SAVE_CONFIGURATION = 81,
    SAE_J1939_GROUP_EXTENSION_TEST_HARDWARE      = 82,
    SAE_J1939_GROUP_EXTENSION_TEST_SOFTWARE      = 83,
    SAE_J1939_GROUP_EXTENSION_TEST_STATUS        = 84,
    SAE_J1939_GROUP_EXTENSION_PACKET_RATE        = 85,
    SAE_J1939_GROUP_EXTENSION_PACKET_TYPE        = 86,
    SAE_J1939_GROUP_EXTENSION_DIGITAL_FILTER     = 87,
    SAE_J1939_GROUP_EXTENSION_ORIENTATION        = 88,
    SAE_J1939_GROUP_EXTENSION_USER_BEHAVIOR      = 89,
    SAE_J1939_GROUP_EXTENSION_DM1_CONFIG         = 90,
    SAE_J1939_GROUP_EXTENSION_ALGO_CTRL          = 91,
    SAE_J1939_GROUP_EXTENSION_AIDING_SRC_CFG     = 95,
    SAE_J1939_GROUP_EXTENSION_AIDING_SRC_LEVER_ARM   = 96,
    SAE_J1939_GROUP_EXTENSION_BANK0              = 240,
    SAE_J1939_GROUP_EXTENSION_BANK1              = 241,
    SAE_J1939_GROUP_EXTENSION_BANK2              = 242,
    SAE_J1939_GROUP_EXTENSION_ADDR               = 254,
    SAE_J1939_GROUP_EXTENSION_ACK                = 255,
    SAE_J1939_GROUP_EXTENSION_VP                 = 243,
    SAE_J1939_GROUP_EXTENSION_SLOPE_SENSOR2      = 41,
    SAE_J1939_GROUP_EXTENSION_SLOPE_SENSOR       = 19 ,
    SAE_J1939_GROUP_EXTENSION_ANGULAR_RATE       = 42,
    SAE_J1939_GROUP_EXTENSION_ACCELERATION       = 45,
    SAE_J1939_GROUP_EXTENSION_ANGULAR_RATE_HR    = 107,
    SAE_J1939_GROUP_EXTENSION_ACCELERATION_HR    = 109,
    SAE_J1939_GROUP_EXTENSION_TRN_CONTROL        = 5,
    SAE_J1939_GROUP_EXTENSION_WHEEL_SPEED        = 191,
    SAE_J1939_GROUP_EXTENSION_CCONTROL           = 241,
};

// J1939 message priority
enum {
    SAE_J1939_CONTROL_PRIORITY                   = 6,
    SAE_J1939_REQUEST_PRIORITY                   = 6,
    SAE_J1939_ACCELERATION_PRIORITY              = 2,
    SAE_J1939_RATE_PRIORITY                      = 3,
    SAE_J1939_SLOPE_PRIORITY                     = 3,
    SAE_J1939_TP_DATA_PRIORITY                   = 7,
};

// J1939 message page number, currently uses 0
enum {
    SAE_J1939_DATA_PAGE  = 0
};

// Bootloader - related section
enum {
     SAE_1939J_MA_COMMAND_ERASE 		    = 0,
     SAE_1939J_MA_COMMAND_READ   		    = 1,
     SAE_1939J_MA_COMMAND_WRITE   		    = 2,
     SAE_1939J_MA_COMMAND_STATUS_REQUEST    = 3,
     SAE_1939J_MA_COMMAND_OP_CPLT       	= 4,
     SAE_1939J_MA_COMMAND_OP_FAILED    	    = 5,
     SAE_1939J_MA_COMMAND_BOOT_LOAD 	    = 6,
     SAE_1939J_MA_COMMAND_EDCP_GEN    	    = 7,
};

enum {
     SAE_1939J_MR_STATUS_PROCEED 		    = 0,
     SAE_1939J_MR_STATUS_COMPLETED 		    = 4,
     SAE_1939J_MR_STATUS_FAILED 		    = 5,
};

// definition of custom PS numbers ramge
enum {
         ACEINNA_SAE_J1939_PS_MIN    =        0x40,
         ACEINNA_SAE_J1939_PS_MAX    =        0x6C
};

// definition of boot protocol errors
enum
{
    SAE_1939J_MR_NO_ERROR = 0x000000,
    SAE_1939J_MR_ERROR_UDEFINED = 0x000001,
    SAE_1939J_MR_ERROR_BOOT_LOAD = 0x000016,
    SAE_1939J_MR_ERROR_ERASE = 0x000016,
    SAE_1939J_MR_ERROR_WRITE = 0x000016,
    SAE_1939J_MR_ERROR_SECURITY_GENERAL = 0x001000,
    SAE_1939J_MR_ERROR_SECURITY_PASSWORD = 0x001001,
    SAE_1939J_MR_ERROR_SECURITY_USER = 0x001002,
    SAE_1939J_MR_ERROR_SECURITY_KEY = 0x001003,
    SAE_1939J_MR_ERROR_ERASE_NEEDED = 0x000105,
    SAE_1939J_MR_ERROR_INVALID_LENGTH = 0x000102,
    SAE_1939J_MR_ERROR_INVALID_ADDRESS = 0x000101,
    SAE_1939J_MR_ERROR_BUSY_WRITE = 0x000012,
    SAE_1939J_MR_ERROR_BUSY_EDCP = 0x000017,
    SAE_1939J_MR_ERROR_BUSY_UNSPECIFIED = 0x00001F,
    SAE_1939J_MR_ERROR_TRANSPORT_FAILED = 0x010003,
    SAE_1939J_MR_ADDRESS_ERROR = 0x000100,
    SAE_1939J_MR_ERROR_INTERNAL_FAILURE = 0x000024,
    SAE_1939J_MR_ERROR_BUSY_OTHER = 0x000002,
    SAE_1939J_MR_ERROR_BOOT_ADDRESS = 0x000108,
    SAE_1939J_MR_ERROR_UNAVAILABLE = 0xFFFFFF,
    SAE_1939J_MR_ERROR_FLASH_WRITE = 0x000022

};

static uint16_t const   SAE_1939J_MR_KEY_NOT_REQUIRED = 0xFFFFU;



// MTLT message type upon J1939, from protocol spec
enum {
    ACEINNA_SAE_J1939_VERSION                    = 1,
    ACEINNA_SAE_J1939_ECU_ID                     = 2,
    ACEINNA_SAE_J1939_ALGORITHM_RESET            = 3,
    ACEINNA_SAE_J1939_CONFIG_SAVE                = 4,
    ACEINNA_SAE_J1939_ACK                        = 5,
    ACEINNA_SAE_J1939_HARDWARE_BITS              = 6,
    ACEINNA_SAE_J1939_SOFTWARE_BITS              = 7,
    ACEINNA_SAE_J1939_STATUS                     = 8,
    ACEINNA_SAE_J1939_RATE_DIVIDER               = 9,
    ACEINNA_SAE_J1939_PACKET_TYPE                = 10,
    ACEINNA_SAE_J1939_DIGITAL_FILTER             = 11,
    ACEINNA_SAE_J1939_ORIENTATION                = 12,
    ACEINNA_SAE_J1939_USER_BEHAVIOR              = 13,
    ACEINNA_SAE_J1939_BUILT_IN_TEST              = 14,
    ACEINNA_SAE_J1939_ACCELERATION_PARAMETERS    = 15,
    ACEINNA_SAE_J1939_PS_BANK0                   = 16,
    ACEINNA_SAE_J1939_PS_BANK1                   = 17,
    ACEINNA_SAE_J1939_PS_BANK2                   = 18,
};

// MTLT message's length, see protocol spec
enum
{
    SAE_J1939_PAYLOAD_LEN_2_BYTES = 2,
    SAE_J1939_PAYLOAD_LEN_3_BYTES = 3,
    SAE_J1939_PAYLOAD_LEN_4_BYTES = 4,
    SAE_J1939_PAYLOAD_LEN_5_BYTES = 5,
    SAE_J1939_PAYLOAD_LEN_7_BYTES = 7,
    SAE_J1939_PAYLOAD_LEN_8_BYTES = 8,
};

// MTLT's operation
enum
{
    ACEINNA_SAE_J1939_REQUEST = 0,
    ACEINNA_SAE_J1939_RESPONSE = 1,
    ACEINNA_SAE_J1939_RESET = 2,
};

// MTLT's address pool
enum
{
    ACEINNA_SAE_J1939_ADDRESS_MIN = 128,
    ACEINNA_SAE_J1939_ADDRESS_MAX = 247
};

// MTLT's ODR on CAN
enum {
  ACEINNA_SAE_J1939_PACKET_RATE_0           =           0,   //quiet
  ACEINNA_SAE_J1939_PACKET_RATE_2           =           2,   // 2Hz
  ACEINNA_SAE_J1939_PACKET_RATE_5           =           5,   // 5Hz
  ACEINNA_SAE_J1939_PACKET_RATE_10          =           10,  // 10Hz
  ACEINNA_SAE_J1939_PACKET_RATE_20          =           20,  // 20Hz
  ACEINNA_SAE_J1939_PACKET_RATE_25          =           25,  // 25Hz
  ACEINNA_SAE_J1939_PACKET_RATE_50          =           50,  // 50Hz
  ACEINNA_SAE_J1939_PACKET_RATE_100         =           100  // 100Hz
};


// MTLT's packet types
enum {
  ACEINNA_SAE_J1939_PACKET_SLOPE_SENSOR2    =           0x01,   // slope sensor 2
  ACEINNA_SAE_J1939_PACKET_ANGULAR_RATE     =           0x02,   // angular rate
  ACEINNA_SAE_J1939_PACKET_ACCELERATION     =           0x04,   // acceleration
  ACEINNA_SAE_J1939_PACKET_ANGULAR_RATE_HR  =           0x08,   // high resolution raw angular rate
  ACEINNA_SAE_J1939_PACKET_ACCELERATION_HR  =           0x10,   // high resolution raw acceleration
  ACEINNA_SAE_J1939_PACKET_SLOPE_SENSOR     =           0x20,   // slope sensor
};



// MTLT's control packet types
enum {
  SAE_J1939_REQUEST_PACKET            =    1,       // request packet
  SAE_J1939_ACK_PACKET                =    2,       // ack packet
  SAE_J1939_RESPONSE_PACKET           =    3,       // response packet
  SAE_J1939_SET_PACKET                =    4,       // set packet
  SAE_J1939_DATA_PACKET               =    5        // data packet
};


// bit status definition, coming from MTLT's user's guide
enum {
	ACEINNA_SAE_J1939_BIT_HW     =    1,
	ACEINNA_SAE_J1939_BIT_SW     =    2,
	ACEINNA_SAE_J1939_BIT_MASTER =    4,
};


typedef struct {
    uint8_t  dest_address;
    uint8_t  rsvd;
    uint16_t limitRateIntegrationTime;  // 1 ms precision   2000 (2 seconds) 0.01 resolution
    uint16_t limitAccelSwitchDelay;     // 1 ms precision   2000 (2 seconds) 0.01 resolution
    uint16_t coefOfReduceQ;             // value/10000      0.001
} ALGO_CONTROL_PAYLOAD;

typedef struct {
    uint8_t  dest_address;
    uint8_t  lvrArmXLSB;
    uint8_t  lvrArmXMSB;
    uint8_t  lvrArmYLSB;
    uint8_t  lvrArmYMSB;
    uint8_t  lvrArmZLSB;
    uint8_t  lvrArmZMSB;
} AIDING_SOURCE_LEVER_ARM_CTRL_PAYLOAD;

typedef struct {
    uint8_t  dest_address;
    uint8_t  signalSource;
    uint8_t  aidingMsgPF;
    uint8_t  aidingMsgPS;
    uint8_t  aidingMsgRate;
    uint8_t  drivingDirMsgPF;
    uint8_t  drivingDirMsgPS;
    uint8_t  odoCfgSwitch;
} AIDING_SOURCE_CONFIG_CTRL_PAYLOAD;

typedef struct {
    uint8_t  currDataLink;
    uint8_t  j1939Net2;
    uint8_t  j1939Net3;
    uint8_t  holdSignal;
    uint8_t  suspendTimeL;
    uint8_t  suspendTimeH;
    uint8_t  rsvd[2];
} STARTSTOP_PAYLOAD;

// Bank0 set payload format
typedef struct {
        uint8_t dest_address;              // destination address
        uint8_t alg_reset_ps;              // new ps value of alg reset
        uint8_t save_cfg_ps;               // new ps value of config save
        uint8_t hw_status_ps;              // new ps value of status message
        uint8_t sw_status_ps;              // new ps value of status message
        uint8_t master_status_ps;          // new ps value of status message
        uint8_t hr_rate_ps;                // new ps hr rate message
        uint8_t hr_accel_ps;               // new ps hr accel message
        ECU_IDENTIFIER_FIELD  bank0_pgn;
} BANK0_PS_PAYLOAD;


// Bank1 set payload format
typedef struct {
        uint8_t dest_address;              // destination address
        uint8_t packet_rate_ps;            // new ps value of packet rate
        uint8_t packet_type_ps;            // new ps value of packet type
        uint8_t digital_filter_ps;         // new ps value of lpf
        uint8_t orientation_ps;            // new ps value of orientation
        uint8_t user_behavior_ps;          // new ps value of user behavior
        uint8_t algo_control_ps;           // new ps value of algrithm control
        uint8_t rsvd1;                     // reserved
        ECU_IDENTIFIER_FIELD  bank1_pgn;
} BANK1_PS_PAYLOAD;


// Bank2 set payload format
typedef struct {
        uint8_t dest_address;              // destination address
		uint8_t aid_config_ps;             // new ps value of aiding config message
        uint8_t aid_lvarm_ps;              // new ps value of aiding lvarm data message
        uint8_t dm1_config_ps;             // new ps value of dm1 config message
        uint8_t reserved[4];
        ECU_IDENTIFIER_FIELD  bank2_pgn;
} BANK2_PS_PAYLOAD;


// slope sensor data payload format
typedef struct {
    uint64_t pitch                :       24;       // pitch
    uint64_t roll                 :       24;       // roll
    uint64_t pitch_compensation   :       2;        // pitch compensation
    uint64_t pitch_merit          :       2;        // pitch merit
    uint64_t roll_compensation    :       2;        // roll compensation
    uint64_t roll_merit           :       2;        // roll merit
    uint64_t measure_latency      :       8;        // latency
} SLOPE_SENSOR_2;

// slope sensor data payload format
typedef struct {
    uint64_t pitch                :       16;       // pitch
    uint64_t roll                 :       16;       // roll
    uint64_t pitch_rate           :       16;       // pitch compensation
    uint64_t pitch_merit          :       2;        // pitch merit
    uint64_t roll_merit           :       2;        // roll merit
    uint64_t rate_merit           :       2;        // roll merit
    uint64_t compensation         :       2;        // pitch and roll compensation
    uint64_t measurement_latency  :       8;        // latency
} SLOPE_SENSOR_1;

// angular rate data payload format
typedef struct {
    uint16_t  pitch_rate;                          // pitch rate
    uint16_t roll_rate;                             // roll  rate
    uint16_t yaw_rate;                              // yaw   rate
    uint16_t  pitch_merit          :       2;      // pitch rate merit
    uint16_t  roll_merit           :       2;      // roll  rate merit
    uint16_t  yaw_merit            :       2;      // yaw  rate merit
    uint16_t  mode                 :       2;      // rsvd
    uint16_t  measurement_latency  :       8;      // latency
} ANGULAR_RATE;

// angular rate data payload format
typedef struct {
    uint64_t  pitch_rate           :       19;       // pitch rate
    uint64_t roll_rate            :       19;       // roll  rate
    uint64_t yaw_rate             :       19;       // yaw   rate
    uint64_t  pitch_merit         :       2;        // pitch rate merit
    uint64_t  roll_merit          :       2;        // roll  rate merit
    uint64_t  yaw_merit           :       2;        // yaw  rate merit
} ANGULAR_RATE_RAW;


// accleration data payload format
typedef struct {
    uint16_t   acceleration_x;                      // x-axis acceleration
    uint16_t   acceleration_y;                      // y-axis acceleration
    uint16_t   acceleration_z;                      // z-axis acceleration
    uint16_t    lateral_merit        :       2;     // laterar acc merit
    uint16_t    longitudinal_merit   :       2;     // longitudinal merit
    uint16_t    vertical_merit       :       2;     // vertical merit
    uint16_t    transmit_rate        :       2;     // repetition rate
    uint16_t    measurement_latency  :       8;     // latency
} ACCELERATION_SENSOR;

// accleration data payload format
typedef struct {
    uint64_t   acceleration_x       :       19;     // x-axis acceleration
    uint64_t   acceleration_y       :       19;     // y-axis acceleration
    uint64_t   acceleration_z       :       19;     // z-axis acceleration
    uint64_t   lateral_merit        :       2;      // laterar acc merit
    uint64_t   longitudinal_merit   :       2;      // longitudinal merit
    uint64_t   vertical_merit       :       2;      // vertical merit
} ACCELERATION_SENSOR_RAW;


// DM1 diagnostics message
typedef struct {
    uint64_t   lamp_status          :       8;      // Protect Lamp Status,
                                                    // Amber Warning Lamp Status
                                                    // Red Stop Lamp Status
                                                    // Malfunction Indicator Lamp Status

    uint64_t   flash_status         :       8;      // Flash Protect Lamp Status
                                                    // Flash Amber Warning Lamp Status
                                                    // Flash Red Stop Lamp Status
                                                    // Flash Malfunction Indicator Lamp Status

    uint64_t   SPN_LOW              :       16;     // Suspect Parameter Number
    uint64_t   FMI                  :       5;      // Failure Mode Identifier
    uint64_t   SPN_HI               :       3;     // Suspect Parameter Number

    uint64_t   occurence_count      :       7;      // Occurrence Count
    uint64_t   SPN_Conversion       :       1;      // SPN Conversion Method
    uint64_t   rsvd1                :       8;      // reserved
    uint64_t   rsvd2                :       8;      // reserved
} DM1_MESSAGE_PAYLOAD;

typedef struct {
    uint8_t           dest_address;         // destination address
    uint8_t           lamp_status;          // status of warning indicators
    uint8_t           flash_status;         // flash control of warning indicators
    uint8_t           SPN_LO;               // SPN of message DM1
    uint8_t           SPN_MED;              // SPN of message DM1
    uint8_t           SPN_HIGH;             // SPN of message DM1
    uint8_t           FMI1;                 // Failure Mode Identifier for DTC1
    uint8_t           FMI2;                 // Failure Mode Identifier for DTC2
} DM1_CONFIG_PAYLOAD;

enum {
    LAMP_STATUS_OFF          = 0,
    FLASH_STATUS_UNAVAILABLE = 3,       // 0x03
    DM1_SPN_UNAVAILABLE      = 524287,  // 0x7FFFF
    DM1_FMI_UNAVAILABLE       = 31,     // 0x1F
    OCCURENCE_COUNT_UNAVAILABLE = 127,  // 0x7F
    CONVERSION_METHOD_UNAVAILABLE = 1,  // 0x01
};

// ODR set packet type
typedef struct {
    uint8_t dest_address;              // target's address
    uint8_t div;                       // packet rate divider
} RATE_CONFIG_PAYLOAD;

// data packet payload
typedef  struct {
    uint8_t     dest_address;             // targer address
    uint8_t    type_low;
    uint8_t    type_high;
    uint8_t    prioVal;
    uint8_t    prioMask;
} PACKET_TYPE_PAYLOAD;


// LPF set payload format
typedef struct {
    uint8_t dest_address;                  // target's address
    uint8_t rate_cutoff;                   // LPF of rate sensor
    uint8_t accel_cutoff;                  // LPF of XL sensor
} DIGITAL_FILTER_PAYLOAD;


// orientation set payload format
typedef struct {
    uint8_t dest_address;                  // target's address
    uint8_t orien_bits[2];                 // orientation setting
} ORIENTATION_SETTING;

// user behavior set payload format

typedef struct {
    uint8_t  dest_address;                 // target's address
    uint8_t  set_option_bits[2];
    uint8_t  reset_option_bits[2];
    uint8_t  new_unit_address;                       // reprogram_unit_address
} USER_BEHAVIOR_PAYLOAD;


// CAN's configuration parameters
typedef struct {
  uint8_t   version[5];                      // software version
  uint16_t  packet_rate_div;                 // odr
  uint16_t  packet_type;                     // packet type
  uint16_t  accel_cut_off;                   // Xl's lpf
  uint16_t  rate_cut_off;                    // rate's lpf
  uint16_t  orien_bits;                      // orientation
  uint16_t  user_behavior;                   // restart on over range
  uint8_t   config_changed;                  // confiugration changed
  uint8_t   alg_reset_ps;                     // new ps value of alg reset
  uint8_t   save_cfg_ps;                      // new ps value of config save
  uint8_t   master_status_ps;                 // new ps value of status message
  uint8_t   packet_rate_ps;                   // new ps value of packet rate
  uint8_t   packet_type_ps;                   // new ps value of packet type
  uint8_t   digital_filter_ps;                // new ps value of lpf
  uint8_t   orientation_ps;                   // new ps value of orientation
  uint8_t   user_behavior_ps;                 // new ps value of user behavior
  uint8_t   sw_status_ps;                     // new ps value of status message
  uint8_t   hw_status_ps;                     // new ps value of status message
  uint8_t   algo_control_ps;                  // new ps value of algorithm control message
  uint8_t   hr_accel_ps;                      // new ps value of hr accel message
  uint8_t   hr_rate_ps;                       // new ps value of hr rate message
  uint8_t   aid_lvarm_ps;                     // new ps value of aiding lever arm
  uint8_t   aid_config_ps;                    // new ps value of aiding configuration
  uint8_t   dm1_config_ps;                    // new ps value of DM1 message configuration
  uint16_t  limitRateIntegrationTime;         // 1 ms precision   2000 (2 seconds) 0.01 resolution
  uint16_t  limitAccelSwitchDelay;            // 1 ms precision   2000 (2 seconds) 0.01 resolution
  uint16_t  coefOfReduceQ;                    // value/10000      0.001
  uint8_t   ecuAddress;
  uint8_t   ecuBaudrate;
 //
  int16_t   odoLeverArmX;
  int16_t   odoLeverArmY;
  int16_t   odoLeverArmZ;
//
  uint8_t   lamp_status;          // status of warning indicators
  uint8_t   flash_status;         // flash control of warning indicators
  uint32_t  SPN;                  // SPN of message DM1
  uint8_t   FMI1;                 // Failure Mode Identifier for DTC1
  uint8_t   FMI2;                 // Failure Mode Identifier for DTC2
//
   uint8_t   signalSource;
   uint8_t   aidingPF;
   uint8_t   aidingPS;
   uint8_t   aidingMsgRate;
   uint8_t   drivingDirPF;
   uint8_t   drivingDirPS;
   uint8_t   odoCfgSwitch;
//
   uint8_t   ariPriority;       // priority of the ARI  packet
   uint8_t   accsPriority;      // priority of the ACCS packet
   uint8_t   ssi2Priority;      // priority of the SSI2 packet

} EcuConfigurationStruct;

extern EcuConfigurationStruct *gEcuConfigPtr;

// command types

typedef enum eAceinnaGetType{
    ACEINNA_J1939_INVALID_IDENTIFIER         =     -1,    // invalid indentifier
    ACEINNA_J1939_IGNORE                     =      0,    // ignore
    ACEINNA_J1939_SOFTWARE_VERSION           =      2,    // sofware version packet
    ACEINNA_J1939_ECU_ID                     =      3,    // ecu id packet
    ACEINNA_J1939_ALG_RST                    =      4,    // alg reset packet
    ACEINNA_J1939_CFG_SAVE                   =      5,    // config save packet
    ACEINNA_J1939_HARDWARE_TEST              =      6,    // hardware bit packet
    ACEINNA_J1939_SOFTWARE_TEST              =      7,    // software bit packet
    ACEINNA_J1939_STATUS_TEST                =      8,    // status packet
    ACEINNA_J1939_DATA                       =      10,   // data packet
    ACEINNA_J1939_ADDRESS_CLAIM              =      11,   // address claim packet
    ACEINNA_J1939_REQUEST_PACKET             =      12,   // request packet
    ACEINNA_J1939_CONFIG                     =      13,   // config packet
    ACEINNA_J1939_NETWORK                    =      14,    // netwwork packet
    ACEINNA_J1939_MA_REQUEST                 =      15,   // ma request
    ACEINNA_J1939_DM11                       =      16,   // DM11 packet
} ACEINNA_J1939_PACKET_TYPE;

enum {
    BOOT_STATE_INIT = 0,
    BOOT_STATE_READY           = 1,     // after first key exchange & after ack
    BOOT_STATE_WRITE_REQUEST   = 2,     // write request
    BOOT_STATE_TP_INIT         = 3,     // rts/cts
    BOOT_STATE_COMPLETE        = 5,     // message received
    BOOT_STATE_CHECK_BOOT_KEY  = 6,     // Check auth key
    BOOT_STATE_CHECK_ERASE_KEY = 7,     // Check auth key
    BOOT_STATE_CPLT            = 8,     // EDCP copmlete
    BOOT_STATE_FINISH          = 9,     // boot load
    BOOT_STATE_AUTH            = 10,    // authentication
    BOOT_STATE_WAIT_ERASE      = 11,    // wait erase request
    BOOT_STATE_WAIT_BOOT_CPLT  = 12,    // wait comp-lete command
    BOOT_STATE_WAIT_ERASE_CPLT = 13,    // wait comp-lete command
};


// angle data mode
enum {
    ANGLE_DATA_MODE_CORERCTED_SWAPPED   = 0,
    ANGLE_DATA_MODE_UNCORERCTED_SWAPPED = 1,
    ANGLE_DATA_MODE_CORERCTED_NORMAL    = 2,
    ANGLE_DATA_MODE_UNCORERCTED_NORMAL  = 3
};

// edcp extension
enum
{
    EDCP_EXTENSION_ERROR = 0x06,
    EDCP_EXTENSION_UNDEFINED = 0xFF
};

static uint16_t const GENERIC_MA_KEY = 0x0007U;
static uint32_t const BOOT_START_ADDR = 0x08000000U;


// set command type
typedef struct {
    uint8_t request;                              // request or response
    uint8_t dest_address;                         // target's address
    uint8_t success;                              // successful or failure
} COMMAND_SET_PAYLOAD;

typedef struct{
  	int32_t 	pkt_type;
	int32_t 	priority;
	uint8_t PF;
	uint8_t PS;
	int32_t len;
	uint8_t source;
    uint8_t data_page;
    uint8_t ext_page;
}msg_params_t;

typedef struct {
    uint8_t reserved            :  1;
    uint8_t command             :  3;
    uint8_t pointer_type       	:  1;	// = 0 address from app start, 1 - block/item #
    uint8_t lennum_requested_hi  :  3;
}ma_req_second_byte;

typedef struct {
    uint8_t len_num_requested_lo;			    // 0 to 2047 blocks length 128
    ma_req_second_byte second_byte;
    uint8_t pointer_0;	// 	lsb
    uint8_t pointer_1;
    uint8_t pointer_2;
    uint8_t pointer_3;	// 	msb
    uint8_t key_user0;	// 	0xff
    uint8_t key_user1;	//	0xff
}ma_request_payload_t;


typedef struct
{
    uint8_t reserved1           :  1;
    uint8_t status              :  3;
    uint8_t reserved2           :  1;
    uint8_t lennum_requested_hi :  3;
}ma_resp_2nd_byte_t;

typedef struct {
    uint8_t     len_num_requested_lo;			 // 0 to 2047 blocks length 128
    ma_resp_2nd_byte_t second_byte;
     uint8_t EDC_0;		  //lsb
     uint8_t EDC_1;
     uint8_t EDC_2;		  // msb
     uint8_t EDC_EXT;	  //  0xFF in case EDC parameter
     uint8_t seed0;	    // 	0xff
     uint8_t seed1;	    //	0xff
}ma_response_payload_t;

typedef struct {
    uint8_t  buf[2048];
    BOOL         ma_session_established;
    int32_t      numPacketsInSegment;
    int32_t      segmentLen;
    int32_t      totalFlashOffset;       // offset to start programming from
    int32_t      totalSegmentNum;        // number of memory segments which were processed (equals to number of write requests of TP sessions)
    int32_t      currentDataSegment;
    int32_t      currentDataLen;
    int32_t      lastOp;
    BOOL         imageVerified;
    int32_t      state;
    int32_t      tp_state;
    BOOL         erased;
    uint32_t     segmentAddr;
    uint32_t     negotiatedLen;
}boot_data_transfer_ctrl_t;


// PGN 65215 Wheel speed information msg
typedef struct {
    uint8_t frontAccelSpeedLsb;     // 1/256 kph/bit, 0 - 251 kph
    uint8_t frontAccelSpeedMsb;
    uint8_t relativeSpeedFrontAxleLeftWheel;    // 1/16 kph/bit 0ffset -7.8125, -7.8125 to 7.8125 km/h
    uint8_t relativeSpeedFrontAxleRightWheel;   // 1/16 kph/bit 0ffset -7.8125, -7.8125 to 7.8125 km/h
    uint8_t relativeSpeedRearAxle1LeftWheel;    // 1/16 kph/bit 0ffset -7.8125, -7.8125 to 7.8125 km/h
    uint8_t relativeSpeedRearAxle1RightWheel;   // 1/16 kph/bit 0ffset -7.8125, -7.8125 to 7.8125 km/h
    uint8_t relativeSpeedRearAxle2LeftWheel;    // 1/16 kph/bit 0ffset -7.8125, -7.8125 to 7.8125 km/h
    uint8_t relativeSpeedRearAxle2RightWheel;   // 1/16 kph/bit 0ffset -7.8125, -7.8125 to 7.8125 km/h
}wheel_speed_msg_t;

// PGN 65265 Wheel speed information msg
typedef struct {
    uint8_t switchMask1;
    uint8_t wheelBasedSpeedLsb;                // 1/256 kph/bit, 0 - 251 kph
    uint8_t wheelBasedSpeedMsb;
    uint8_t switchMask2;
    uint8_t switchMask3;
    uint8_t cruiseSetSpeed;                    // 1 kph/bit, 0 - 250 kph
    uint8_t switchMask4;
    uint8_t switchMask5;
}cruise_ctrl_vehicle_speed_msg_t;

// PGN 61445 ETC2
typedef struct {
    uint8_t selectedGear;                      // 1 gear/bit offset -125, -125 - 125 range
    uint8_t actualGearRatioLsb;                // 1/256 kph/bit, 0 - 251 kph
    uint8_t actualGearRatioMsb;                // 0.001/bit, offset 0,  0 - 64355
    uint8_t currentGear;                       // 1 gear/bit offset -125, -125 - 125 range
    uint8_t requestedRange1;                   // ASCII
    uint8_t requestedRange2;                   // ASCII
    uint8_t currentRange1;                     // ASCII
    uint8_t currentRange2;                     // ASCII
}transmission_controller_msg_t;

// PGN 126720 JD Specific Machine Acceleration msg
typedef struct {
    uint8_t cmd_byte1;
    uint8_t latAxcelExtRangeLsb;                // Offset: -320, Range: -320 to 322.55 m/s^2, Resolution: 0.01 m/s^2 (0.001019716 g)
    uint8_t latAxcelExtRangeMsb;
    uint8_t lonAxcelExtRangeLsb;                // Offset: -320, Range: -320 to 322.55 m/s^2, Resolution: 0.01 m/s^2 (0.001019716 g)
    uint8_t lonAxcelExtRangeMsb;
    uint8_t verAxcelExtRangeLsb;                // Offset: -320, Range: -320 to 322.55 m/s^2, Resolution: 0.01 m/s^2 (0.001019716 g)
    uint8_t verAxcelExtRangeMsb;
    uint8_t machineStationary;
}jd_machine_accel_msg_t;

// STD_ID 170 Corola WheelSpeed
typedef struct {
    uint8_t wheelSpeedFrontRightMsb;
    uint8_t wheelSpeedFrontRightLsb;
    uint8_t wheelSpeedFrontLeftMsb;
    uint8_t wheelSpeedFrontLeftLsb;
    uint8_t wheelSpeedRearRightMsb;
    uint8_t wheelSpeedRearRightLsb;
    uint8_t wheelSpeedRearLeftMsb;
    uint8_t wheelSpeedRearLeftLsb;
}corola_wheel_speed_msg_t;

// STD_ID 956 Corola Gear
//-------------
// Gear | Value
//-------------
//  P     32
//  R     16
//  N     8
//  D     0
// dbc: MSB, unsigned
//             SG_ Name : StartBit | Length @ ByteOrder SignedFlag (Factor,Offset) [Minimum | Maximum] "Unit" Receiver1,Receiver2
//             BO_ 956 GEAR_PACKET: 8 XXX
//             SG_ GEAR : 13|6@0+ (1,0) [0|63] "" XXX
//             SG_ SPORT_ON : 3|1@0+ (1,0) [0|1] "" XXX

typedef struct {
    uint8_t cmd_byte1;
    uint8_t gear;
    uint8_t cmd_byte2;
    uint8_t cmd_byte3;
    uint8_t cmd_byte4;
    uint8_t cmd_byte5;
}corola_gear_msg_t;


extern EcuConfigurationStruct *gEcuConfigPtr;

/*******************************************
 * @brief
 *
********************************************/
extern void ecu_process(void);

/*******************************************
 * @brief
 *
********************************************/
extern void ecu_transmit(void);

/*******************************************
 * @brief
 *
********************************************/

/*******************************************
 * @brief
 *
********************************************/
extern void    aceinna_j1939_initialize_config();

/*******************************************
 * @brief
 *
 * @param status_type ==
 * @param statusWord ==
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_send_status_packet(int32_t status_type, void * statusWord);

/*******************************************
 * @brief
 *
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_send_software_version(void);

/*******************************************
 * @brief
 *
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_send_ecu_id(void);

/*******************************************
 * @brief
 *
 * @param data ==
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_send_slope_sensor2(SLOPE_SENSOR_2 * data);

/*******************************************
 * @brief
 *
 * @param data ==
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_send_acceleration(ACCELERATION_SENSOR * data);

/*******************************************
 * @brief
 *
 * @param data ==
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_send_angular_rate(ANGULAR_RATE * data);

/*******************************************
 * @brief
 *
 * @param address ==
 * @param alg_rst ==
 * @param success ==
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_send_algrst_cfgsave(uint8_t address, uint8_t alg_rst, uint8_t success);

/*******************************************
 * @brief
 *
 * @param odr ==
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_send_packet_rate(uint8_t odr);

/*******************************************
 * @brief
 *
 * @param type ==
 * @param priorityMask ==
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_send_packet_type(uint16_t type, uint8_t const priorityMask);

/*******************************************
 * @brief
 *
 * @param accel_cutoff ==
 * @param rate_cutoff ==
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_send_digital_filter(uint8_t accel_cutoff, uint8_t rate_cutoff);

/*******************************************
 * @brief
 *
 * @param aidingCfdPld ==
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_send_aiding_sig_config(AIDING_SOURCE_CONFIG_CTRL_PAYLOAD * const aidingCfdPld);

/*******************************************
 * @brief
 *
 * @param lvrArmCfgPld ==
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_send_aiding_leverArm_config(AIDING_SOURCE_LEVER_ARM_CTRL_PAYLOAD * const lvrArmCfgPld);

/*******************************************
 * @brief
 *
 * @param orien ==
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_send_orientation( uint8_t *orien);

/*******************************************
 * @brief
 *
 * @param payload ==
 * @param params ==
 * @return uint8_t
********************************************/

/*******************************************
 * @brief
 *
 * @param payload ==
 * @param params ==
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_build_msg(void* const payload, msg_params_t* const params);

/*******************************************
 * @brief
 *
 * @param data ==
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_send_acceleration_raw(ACCELERATION_SENSOR_RAW * data);

/*******************************************
 * @brief
 *
 * @param data ==
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_send_angular_rate_raw(ANGULAR_RATE_RAW * data);

/*******************************************
 * @brief
 *
 * @param behavior ==
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_send_user_behavior(uint16_t behavior);

/*******************************************
 * @brief
 *
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_send_address_claim();

/*******************************************
 * @brief
 *
 * @param data ==
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_send_algo_control_packet(void *data);

/*******************************************
 * @brief
 *
 * @param data ==
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_send_slope_sensor(SLOPE_SENSOR_1 * data);

/*******************************************
 * @brief
 *
 * @param pld ==
 * @return uint8_t
********************************************/
extern uint8_t aceinna_j1939_send_ma_response(ma_response_payload_t *pld);

/*******************************************
 * @brief
 *
 * @param desc ==
 * @return BOOL
********************************************/
extern BOOL    aceinna_j1939_processRequest(struct ecu_rx_desc * const desc);

/*******************************************
 * @brief
 *
 * @param desc ==
********************************************/
extern void    aceinna_j1939_processCommands(struct ecu_rx_desc * const desc);

/*******************************************
 * @brief
 *
 * @param desc ==
********************************************/
extern void    aceinna_j1939_processNetworkRequest(struct ecu_rx_desc * const desc);

/*******************************************
 * @brief
 *
 * @param desc ==
********************************************/
extern void    aceinna_j1939_process_ma_request(struct ecu_rx_desc * const desc);

/*******************************************
 * @brief
 *
 * @param input ==
 * @return uint8_t
********************************************/
uint8_t find_tx_desc(struct ecu_tx_desc** const input);


/*******************************************
 * @brief
 *
********************************************/
void aceinna_j1939_processAidingDataPackets(struct ecu_rx_desc* const desc);

/*******************************************
 * @brief
 *
 * @param ident ==
 * @return ACEINNA_J1939_PACKET_TYPE
********************************************/
ACEINNA_J1939_PACKET_TYPE is_aceinna_data_packet(ECU_IDENTIFIER_FIELD *ident);


/*******************************************
 * @brief
 *
 * @param pld ==
 * @return uint8_t
********************************************/
uint8_t aceinna_j1939_send_dm1_config(DM1_CONFIG_PAYLOAD * const pld);

/*******************************************
 * @brief
 *
 * @param pld ==
 * @return uint8_t
********************************************/
uint8_t aceinna_j1939_send_DM1_message(DM1_MESSAGE_PAYLOAD *pld);

/*******************************************
 * @brief
 *
 * @param destAddr ==
 * @param ackCode ==
 * @param ackPF ==
 * @param ackPS ==
 * @param ackPage ==
 * @return uint8_t
********************************************/
uint8_t aceinna_j1939_send_ack(uint8_t destAddr, uint8_t ackCode, uint8_t ackPF, uint8_t ackPS, uint8_t ackPage);

/*******************************************
 * @brief
 *
 * @param desc ==
********************************************/
void aceinna_j1939_process_DM11_Message(struct  ecu_rx_desc* const desc);


#ifdef __cplusplus
}
#endif

#endif // SAE_J1939_H
