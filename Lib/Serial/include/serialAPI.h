/** ******************************************************************************
 * @file commAPI.h API for referencing serial communication functions 
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
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


#ifndef SERIAL_API_H
#define SERIAL_API_H

enum
{
    UCB_MAX_PAYLOAD_LENGTH = 255,
    UCB_USER_IN = 200,
    UCB_USER_OUT = 201,
    UCB_ERROR_INVALID_TYPE = 202,
    UCB_PACKET_TYPE_LENGTH	=	2,
};


typedef struct {
     uint8_t       packetType;      // 0
     uint8_t       systemType;      // 1
     uint8_t       spiAddress;      // 2
     uint8_t       sync_MSB;        // 3
     uint8_t       sync_LSB;        // 4
     uint8_t       code_MSB;        // 5
     uint8_t       code_LSB;        // 6
     uint8_t	   payloadLength;   // 7
     uint8_t       payload[UCB_MAX_PAYLOAD_LENGTH + 3]; // aligned to 4 bytes
} UcbPacketStruct;


/**********************************************
* @brief 
* 
***********************************************/
void  ProcessUserCommands();

/*******************************************
 * @brief 
 * 
 * @param dacqRate ==
********************************************/
void  SendContinuousPacket(int32_t dacqRate);

/*******************************************
 * @brief 
 * 
 * @param ptrUcbPacket ==
 * @return int32_t 
********************************************/
int32_t HandleUserInputPacket(UcbPacketStruct *ptrUcbPacket);

/*******************************************
 * @brief 
 * 
 * @param receivedCode ==
 * @return int32_t 
********************************************/
int32_t CheckUserPacketType(uint16_t receivedCode);

/*******************************************
 * @brief 
 * 
 * @param bytes ==
********************************************/
void   UserPacketTypeToBytes(uint8_t bytes[]);

/*******************************************
 * @brief Set the User Packet Type object
 * 
 * @param type ==
 * @param fApply ==
 * @return BOOL 
********************************************/
BOOL   SetUserPacketType(uint8_t* type, BOOL fApply);

/*******************************************
 * @brief 
 * 
 * @param payload ==
 * @param payloadLen ==
 * @return BOOL 
********************************************/
BOOL HandleUserOutputPacket(uint8_t *payload, uint8_t *payloadLen);

/*******************************************
 * @brief Get the User Payload Length object
 * 
 * @return int32_t 
********************************************/
int32_t GetUserPayloadLength(void);


#endif
