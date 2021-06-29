/** ***************************************************************************
 * @file ucb_packet.h 
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

#ifndef UCB_PACKET_H
#define UCB_PACKET_H
#ifdef __cplusplus
 extern "C" {
#endif
 
 
#include "crc.h"
#include "GlobalConstants.h"
#include "SerialAPI.h"

typedef enum {
    UCB_PING,               //  0 PK 0x504B input packets
    UCB_ECHO,               //  1 CH 0x4348
    UCB_GET_PACKET,         //  2 GP 0x4750
    UCB_SET_FIELDS,         //  3 SF 0x5346
    UCB_GET_FIELDS,         //  4 GF 0x4746
    UCB_READ_FIELDS,        //  5 RF 0x4246
    UCB_WRITE_FIELDS,       //  6 WF 0x5746
    UCB_UNLOCK_EEPROM,      //  7 UE 0x5545
    UCB_READ_EEPROM,        //  8 RE 0x5245
    UCB_WRITE_EEPROM,       //  9 WE 0x4745
    UCB_PROGRAM_RESET,      // 10 PR 0x5052
    UCB_SOFTWARE_RESET,     // 11 SR 0x5352
    UCB_WRITE_CAL,          // 13 WC 0x5743
    UCB_IDENTIFICATION,     // 14 ID 0x4944 output packets
    UCB_VERSION_DATA,       // 15 VR 0x4652
    UCB_VERSION_ALL_DATA,   // 16 VA 0x5641
    UCB_ANGLE_1,            // 17 A1 0x4131
    UCB_SCALED_0,           // 22 S0 0x5330
    UCB_SCALED_1,           // 23 S1 0x5331
    UCB_SCALED_M,           // 23 S1 0x534D
    UCB_SCALED_N,           // 23 S1 0x534E
    UCB_TEST_0,             // 24 T0 0x5430
    UCB_FACTORY_1,          // 26 F1 0x4631
    UCB_FACTORY_2,          // 27 F2 0x4632
    UCB_FACTORY_M,          // 28 F3 0x464D
    UCB_MAG_CAL_1_COMPLETE, // 29 CB 0x4342
    UCB_MAG_CAL_3_COMPLETE, // 30 CD 0x4344
    UCB_READ_CAL,           //  RC 0x5243
    UCB_WRITE_APP,          //    WA 0x5743
    UCB_J2BOOT,             //    JB 0x4A42
    UCB_J2APP,              //    JA 0x4A41
    UCB_J2IAP,              //    JI 0x4A49
    UCB_LOCK_EEPROM,        //    LE 0x4C45
    UCB_ANGLE_2,            //    A1 0x4132
    UCB_ANGLE_3,            //    A1 0x4132

    UCB_NAK,                //  37
    UCB_ERROR_TIMEOUT,      //  39 timeout reached before entire packet was received
    UCB_ERROR_CRC_FAIL,     //  40
    
    UCB_INPUT_PACKET_MAX, 
    UCB_PACKET_LAST 
} UcbPacketType;

typedef struct{
    int32_t   ptype;
    uint32_t  psync;
}ucbPacketTableEntry_t;

extern ucbPacketTableEntry_t ucbPackets[34];

enum
{
    UCB_IDENTIFICATION_LENGTH = 69,
    UCB_VERSION_DATA_LENGTH = 5,
    UCB_VERSION_ALL_DATA_LENGTH = 15,
    UCB_ANGLE_1_LENGTH = 32,
    UCB_ANGLE_2_LENGTH = 30,
    UCB_ANGLE_3_LENGTH = 30,
    UCB_ANGLE_4_LENGTH = 42,
    UCB_ANGLE_5_LENGTH = 62,
    UCB_ANGLE_U_LENGTH = 42,
    UCB_SCALED_0_LENGTH = 30,
    UCB_SCALED_1_LENGTH = 24,
    UCB_SCALED_M_LENGTH = 62,
    UCB_SCALED_N_LENGTH = 68,
    UCB_TEST_0_LENGTH = 28,
    UCB_TEST_1_LENGTH = 32,
    UCB_FACTORY_1_LENGTH = 54,
    UCB_FACTORY_2_LENGTH = 85,
    UCB_FACTORY_4_LENGTH = 54,
    UCB_FACTORY_5_LENGTH = 70,
    UCB_FACTORY_6_LENGTH = 66,
    UCB_FACTORY_7_LENGTH = 134,
    UCB_FACTORY_M_LENGTH = 85,
    UCB_MAG_CAL_1_COMPLETE_LENGTH = 4,
    UCB_MAG_CAL_3_COMPLETE_LENGTH = 10,
    UCB_NAV_0_LENGTH = 32,
    UCB_NAV_1_LENGTH = 42,
    UCB_NAV_2_LENGTH = 46, // with ITOW
    UCB_KT_LENGTH = 18,
};

enum {
    INPUT_STATE_0 = 0,
    INPUT_STATE_1 = 1,
    INPUT_STATE_2 = 2,
    INPUT_STATE_3 = 3,
    INPUT_STATE_4 = 4,
    INPUT_STATE_5 = 5,
};

/// UCB packet-specific utility functions ucb_packet.c

/**
 * @brief 
 * 
 * @param type 
 * @return BOOL 
 */
extern BOOL              UcbPacketIsAnOutputPacket     (UcbPacketType type);

/**
 * @brief 
 * 
 * @param bytes 
 * @param fGet 
 * @return UcbPacketType 
 */
extern UcbPacketType     UcbPacketBytesToPacketType    (uint8_t *bytes, BOOL fGet);

/**
 * @brief 
 * 
 * @param port 
 * @param ptrUcbPacket 
 */
extern void SendUcbPacket   (uint16_t port, UcbPacketStruct *ptrUcbPacket);

/**
 * @brief 
 * 
 * @param port 
 * @param ptrUcbPacket 
 */
extern void HandleUcbPacket (uint16_t port, UcbPacketStruct *ptrUcbPacket);

/**
 * @brief 
 * 
 * @param port 
 * @param ptrUcbPacket 
 */
extern void HandleUcbTx (uint16_t  port, UcbPacketStruct *ptrUcbPacket);

enum {
    PRIMARY_UCB_PORT = 0U,
};


#ifdef __cplusplus
}
#endif

#endif
