/** ***************************************************************************
 * @file input_msg.c 
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

#include "ucb_packet.h"
#include "crc16.h"
#include "halAPI.h"

int16_t ucbPort   =  0;   
int32_t debugPort = -1; 



/// List of allowed input packet codes 
ucbPacketTableEntry_t ucbPackets[] = {		//       
    {UCB_PING,               0x5555504B},   //  "PK" 
    {UCB_ECHO,               0x55554348},   //  "CH" 
    {UCB_GET_PACKET,         0x55554750},   //  "GP" 
    {UCB_SET_FIELDS,         0x55555346},   //  "SF" 
    {UCB_GET_FIELDS,         0x55554746},   //  "GF" 
    {UCB_READ_FIELDS,        0x55555246},   //  "RF" 
    {UCB_WRITE_FIELDS,       0x55555746},   //  "WF" 
    {UCB_UNLOCK_EEPROM,      0x55555545},   //  "UE" 
    {UCB_READ_EEPROM,        0x55555245},   //  "RE" 
    {UCB_WRITE_EEPROM,       0x55555745},   //  "WE" 
    {UCB_SOFTWARE_RESET,     0x55555352},   //  "SR" 
    {UCB_WRITE_CAL,          0x55555743},   //  "WC" 
    {UCB_READ_CAL,           0x55555243},   //  "RC" 
    {UCB_WRITE_APP,          0x55555741},   //  "WA" 
    {UCB_J2BOOT,             0x55554A42},   //  "JB" 
    {UCB_J2IAP,              0x55554A49},   //  "JI" 
    {UCB_J2APP,              0x55554A41},   //  "JA" 
    {UCB_LOCK_EEPROM,        0x55554C45},   //  "LE" 
    {UCB_INPUT_PACKET_MAX,   0x00000000},    //  "  "

    {UCB_IDENTIFICATION,     0x55554944},   //  "ID" 
    {UCB_VERSION_DATA,       0x55555652},   //  "VR" 
    {UCB_VERSION_ALL_DATA,   0x55555641},   //  "VA" 
    {UCB_SCALED_0,           0x55555330},   //  "S0" 
    {UCB_SCALED_1,           0x55555331},   //  "S1" 
    {UCB_SCALED_M,           0x5555534D},   //  "SM" 
    {UCB_SCALED_N,           0x5555534E},   //  "SN" 
    {UCB_FACTORY_1,          0x55554631},   //  "F1" 
    {UCB_FACTORY_2,          0x55554632},   //  "F2" 
    {UCB_FACTORY_M,          0x5555464D},   //  "FM" 
    {UCB_ANGLE_1,            0x55554131},   //  "A1" 
    {UCB_TEST_0,             0x55555430},   //  "T0" 
    {UCB_ANGLE_2,            0x55554132},   //  "A2" 
    {UCB_ANGLE_3,            0x55554133},   //  "A3" 
    {UCB_PACKET_LAST,        0x00000000},    //  "  "
};



/**********************************************
* @brief Get the Ucb Packet object
* 
* @param uartPort [ ]
* @return int32_t 
***********************************************/
static int32_t GetUcbPacket(int32_t const uartPort)
{
    static UcbPacketStruct ucbPacket;
    static uint8_t dataBuffer[512];
    static int32_t bytesInBuffer = 0;
    static int32_t state = 0;
    static int32_t len = 0;
    static uint8_t *ptr = &ucbPacket.payload[0];
    static uint16_t crcMsg = 0U;
	static uint32_t sync = 0U;
    uint8_t  tmp;
	static int32_t  pos    = 0;
	static int32_t  synced = 0;
	static uint32_t type;
	uint16_t crcCalc;
    static int32_t crcError = 0;
    ucbPacketTableEntry_t *syncTable;
	
    
	while(1){
        if(!bytesInBuffer){
            bytesInBuffer = UART_Read(uartPort, dataBuffer, sizeof (dataBuffer));
            if(!bytesInBuffer){
                return 0; // nothing to do
            }
            pos = 0; 
        }
        tmp = dataBuffer[pos];
        pos++;
        bytesInBuffer--;
        sync = (sync << 8U) | tmp;
        syncTable = ucbPackets;
        if((sync & 0xFFFF0000U) == 0x55550000U){
            while(syncTable->ptype != UCB_INPUT_PACKET_MAX){
                if(syncTable->psync == sync){
                    type = (uint32_t)syncTable->ptype;
                    synced       = 1;
                    break;
                }
                if(synced){
                    break;
                }
                syncTable++;
            }
            if(!synced){
                uint16_t const code = (uint16_t)(sync & 0xFFFFU); 
                type = (uint32_t)CheckUserPacketType(code);
                if(type != (uint32_t)UCB_ERROR_INVALID_TYPE){
                    synced = 1;
                }
            }
        }
        if(synced){
            ucbPacket.packetType    = (uint8_t)type;
            ucbPacket.payloadLength = 0U;
            ucbPacket.code_MSB      = (uint8_t)((sync >> 8U) & 0xFFU);
            ucbPacket.code_LSB      = (uint8_t)(sync & 0xFFU);
	        state	                = INPUT_STATE_1;
			len                     = 0;
            synced = 0;
            continue;
        }
        switch(state){
        case INPUT_STATE_0:
            break;
        case INPUT_STATE_1:
            ucbPacket.payloadLength = tmp;
            if(tmp == 0U){
                state = INPUT_STATE_3;  // crc next
            }else{
                state = INPUT_STATE_2;  // data next
                len   = 0;
            }
            ptr   = ucbPacket.payload;
            break;
        case INPUT_STATE_2:
            if(len > UCB_MAX_PAYLOAD_LENGTH){
                state = 0;
                break;
            }
            
            *ptr = tmp;
            ptr++;
            len++;

            if((uint8_t)len == ucbPacket.payloadLength){
                //crc next
                state  = INPUT_STATE_3;
                crcMsg = 0U; 
            }
            break;
        case INPUT_STATE_3:
            crcMsg = tmp;
            *ptr = tmp;   
            ptr++;   
            state = INPUT_STATE_4;
            break;
        case INPUT_STATE_4:
            state   = INPUT_STATE_0;
            crcMsg  = (uint16_t)(crcMsg | ((uint16_t)tmp << 8U));
            *ptr  = tmp;   
            ptr++;   
            crcCalc = CalculateCRC((uint8_t*)&ucbPacket.code_MSB, len + 3);
            if(crcMsg != crcCalc){
                crcError++;
            }else {
                // process message here
               HandleUcbPacket ((uint16_t)ucbPort, &ucbPacket);
               debugPort = INVALID_SERIAL_PORT;
               return 0;   // will come back later
            }
            break;
        default:
            while(1){}; // should not be here
        }
    }
}

/**********************************************
* @brief 
* 
***********************************************/
void  ProcessUserCommands()
{
    GetUcbPacket(ucbPort);
}
