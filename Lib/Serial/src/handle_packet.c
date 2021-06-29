/** ***************************************************************************
 * @file handle_packet.c functions for handling serial UCB packets and CRM
 *       packets
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

//********************************
#include <stdint.h>
#include "ucb_packet.h"
#include "parameters.h"
#include "eepromAPI.h"
#include "osapi.h"
#include "calibrationAPI.h"
#include "configurationAPI.h"
#include "halAPI.h"
#include "bitAPI.h"

static uint32_t handStat = 0U;

/**********************************************
* @brief Set the Nak object
* 
* @param port --
* @param ptrUcbPacket --
***********************************************/
static void SetNak (uint16_t const port, UcbPacketStruct* const ptrUcbPacket)
{
    handStat += port;
	/// return NAK, requested packet type placed in data field by external port
	ptrUcbPacket->packetType 	= (uint8_t)UCB_NAK;
    ptrUcbPacket->code_MSB      = 0x15U;
    ptrUcbPacket->code_LSB      = 0x15U;
	ptrUcbPacket->payloadLength = (uint8_t)(uint16_t)UCB_PACKET_TYPE_LENGTH;
}



/**********************************************
* @brief 
* 
* @param bytes --
* @param fGet --
* @return UcbPacketType 
***********************************************/
UcbPacketType UcbPacketBytesToPacketType(uint8_t bytes[], BOOL const fGet)
{
    uint8_t ref[2];
    uint16_t code;
    ucbPacketTableEntry_t *packetEntry = &ucbPackets[0];
 
     if(!fGet){
        code = bytes[0];
        code <<= 8U;
        code |= bytes[1]; 
    }else {
        code = bytes[1]; 
        code <<= 8U; 
        code |= bytes[0]; 
    }
   
    while(packetEntry->ptype != UCB_PACKET_LAST){
        ref[0] = (uint8_t)((packetEntry->psync >> 8U) & 0xFFU);
        ref[1] = (uint8_t)(packetEntry->psync & 0xFFU);
        if((ref[0] == bytes[1]) && (ref[1] == bytes[0])){
            return (UcbPacketType)packetEntry->ptype;
        }
        packetEntry++;
    }
    
    return (UcbPacketType)CheckUserPacketType(code);
}

/**********************************************
* @brief 
* 
* @param type 
* @return BOOL 
***********************************************/
BOOL UcbPacketIsAnOutputPacket (UcbPacketType const type)
{
	BOOL isAnOutputPacket;
    int32_t const tt = type;
	switch (tt) {
        case UCB_IDENTIFICATION:
        case UCB_VERSION_DATA:
        case UCB_VERSION_ALL_DATA:
        case UCB_SCALED_0:
        case UCB_SCALED_1:
        case UCB_SCALED_M:
        case UCB_SCALED_N:
        case UCB_TEST_0:
        case UCB_FACTORY_1:
        case UCB_FACTORY_2:
        case UCB_FACTORY_M:
        case UCB_PING:
        case UCB_ANGLE_1:
        case UCB_ANGLE_2:
        case UCB_ANGLE_3:
        case UCB_USER_OUT:
            isAnOutputPacket = TRUE;
            break;
		default:
          isAnOutputPacket = FALSE;
	}
	return isAnOutputPacket;
}

/**********************************************
* @brief 
* 
* @param port --
* @param ptrUcbPacket --
***********************************************/
static void UcbPing (uint16_t const port,
                      UcbPacketStruct * const ptrUcbPacket)
{
	ptrUcbPacket->payloadLength = 0U; /// return ping acknowledgement
	HandleUcbTx(port, ptrUcbPacket);
}

/**********************************************
* @brief 
* 
* @param port --
* @param ptrUcbPacket --
***********************************************/
static void UcbEcho (uint16_t const port,
                      UcbPacketStruct* const ptrUcbPacket)
{
	HandleUcbTx(port, ptrUcbPacket);
}

/**********************************************
* @brief 
* 
* @param port 
* @param ptrUcbPacket 
***********************************************/
static void UcbGetPacket (uint16_t const port,
                           UcbPacketStruct* const ptrUcbPacket)
{
    
	if (ptrUcbPacket->payloadLength == 2U) {
        uint8_t bytes[2];
        bytes[0] = ptrUcbPacket->payload[1];
        bytes[1] = ptrUcbPacket->payload[0];
        UcbPacketType const requestedType = UcbPacketBytesToPacketType(bytes, TRUE);

		if (UcbPacketIsAnOutputPacket(requestedType) == TRUE) {
			ptrUcbPacket->packetType = (uint8_t)requestedType; ///< response packet type
		 	SendUcbPacket(port, ptrUcbPacket); ///< generic response packet handler
            return;
		} else {
            SetNak(port, ptrUcbPacket);
		}
	} else {
        SetNak(port, ptrUcbPacket);
	}
	HandleUcbTx(port, ptrUcbPacket);
}

/**********************************************
* @brief 
* 
* @param port 
* @param ptrUcbPacket 
***********************************************/
static void UcbSetFields (uint16_t const port,
                           UcbPacketStruct* const ptrUcbPacket)

{
	uint8_t const numFields = ptrUcbPacket->payload[0];

	/// verify that the packet length matches packet specification
    if ((numFields > 0U) &&
    	(ptrUcbPacket->payloadLength == (1U + (numFields * 4U))))
    {
	    uint8_t fieldCount;
	    uint8_t validFieldCount;
         /// some fields need to be set together, so collect all field ID's and data n one set of arrays 
	    uint16_t fieldId   [UCB_MAX_PAYLOAD_LENGTH / 4];
        /// array sizes are based on maximum number of fields to change
	    uint16_t fieldData [UCB_MAX_PAYLOAD_LENGTH / 4];
	    /// loop through all fields and data specified in set fields request
	    for (fieldCount = 0U; fieldCount < numFields; ++fieldCount) {
	    	/// read field ID and field data from packet into usable arrays
            fieldId[fieldCount]   = (uint16_t)(((uint16_t)ptrUcbPacket->payload[(fieldCount * 4U) + 1U] << 8U) |
                                                (uint16_t)ptrUcbPacket->payload[(fieldCount * 4U) + 2U]);
            fieldData[fieldCount] = (uint16_t)(((uint16_t)ptrUcbPacket->payload[(fieldCount * 4U) + 3U] << 8U) |
                                                (uint16_t)ptrUcbPacket->payload[(fieldCount * 4U) + 4U]);
	    }

	    validFieldCount = config_CheckRamFieldData(numFields, fieldId, fieldData, fieldId);
		if (validFieldCount > 0U) {	/// all or some requested field changes valid?
			/// build and send positive acknowledgement packet
			ptrUcbPacket->payloadLength = (uint8_t)(1U + (validFieldCount * 2U));
	    	ptrUcbPacket->payload[0]    = validFieldCount; /// number of valid fields

			/// place valid field ID's in payload
			for (fieldCount = 0U; fieldCount < validFieldCount; ++fieldCount) {
				ptrUcbPacket->payload[(fieldCount * 2U) + 1U] = (uint8_t)((fieldId[fieldCount] >> 8U) & 0xFFU);
				ptrUcbPacket->payload[(fieldCount * 2U) + 2U] = (uint8_t)( fieldId[fieldCount]       & 0xFFU);
			}
	        HandleUcbTx(port, ptrUcbPacket); ///< send acknowledgement
		}

		/// any invalid requested field changes?
		if (validFieldCount < numFields) {
            SetNak(port, ptrUcbPacket);
     	    HandleUcbTx(port, ptrUcbPacket);
		}

		if (validFieldCount > 0U) { /// apply any changes
			config_SetFieldData();  
		}
	} else {
        SetNak(port, ptrUcbPacket);
	    HandleUcbTx(port, ptrUcbPacket);
	}
}

/**********************************************
* @brief 
* 
* @param port 
* @param ptrUcbPacket 
***********************************************/
static void UcbGetFields (uint16_t const port,
                           UcbPacketStruct * const ptrUcbPacket)
{
	uint8_t  const numFields = ptrUcbPacket->payload[0];

	/// verify that the packet length matches packet specification
    if ((numFields > 0U) &&
    	(ptrUcbPacket->payloadLength == (1U + (numFields * 2U)))) {

	    uint8_t  fieldCount;
	    uint8_t  validFieldCount = 0U;
	    uint16_t fieldId [UCB_MAX_PAYLOAD_LENGTH / 4];

        /// read all fields specified in get fields request
        for (fieldCount = 0U; fieldCount < numFields; ++fieldCount) {
            /// read field ID from packet into usable array
            fieldId[validFieldCount] = (uint16_t)(((uint16_t)ptrUcbPacket->payload[(fieldCount * 2U) + 1U] << 8U) |
                                                   (uint16_t)ptrUcbPacket->payload[(fieldCount * 2U) + 2U]);

            /// check get field address bounds
            if (((fieldId[validFieldCount] >= (uint16_t)LOWER_CONFIG_ADDR_BOUND) &&
                 (fieldId[validFieldCount] <= (uint16_t)UPPER_CONFIG_ADDR_BOUND)) ||
                 (fieldId[validFieldCount] == (uint16_t)PRODUCT_CONFIGURATION_FIELD_ID)) {
                ++validFieldCount;
            }
        }

        if (validFieldCount > 0U) {	/// all or some requested get field addresses valid?
            /// build and return valid get fields with data packet
            ptrUcbPacket->payloadLength = (uint8_t)(1U + (validFieldCount * 4U));

            /// number of fields being returned
            ptrUcbPacket->payload[0] = validFieldCount;

            /// retrieve all fields specified in get fields request
            for (fieldCount = 0U; fieldCount < validFieldCount; ++fieldCount) {
                /** product configuration field is out of normal
                    configuration address range, needs to be fetched from
                    calibration structure */
                if (fieldId[fieldCount] == (uint16_t)PRODUCT_CONFIGURATION_FIELD_ID) {
                    uint16_t const cfg = cal_GetProductConfiguration();

                    ptrUcbPacket->payload[(fieldCount * 4U) + 1U] = (uint8_t)((fieldId[fieldCount] >> 8U) & 0xFFU);
                    ptrUcbPacket->payload[(fieldCount * 4U) + 2U] = (uint8_t)( fieldId[fieldCount]       & 0xFFU);
                    ptrUcbPacket->payload[(fieldCount * 4U) + 3U] = (uint8_t)((cfg >> 8U) & 0xFFU);
                    ptrUcbPacket->payload[(fieldCount * 4U) + 4U] = (uint8_t)( cfg       & 0xFFU);
                }
                else {	/// normal field, exists in configuration structure
                    uint16_t const param = config_GetParam((uint8_t)fieldId[fieldCount]);

                    ptrUcbPacket->payload[(fieldCount * 4U) + 1U] = (uint8_t)((fieldId[fieldCount] >> 8U) & 0xFFU);
                    ptrUcbPacket->payload[(fieldCount * 4U) + 2U] = (uint8_t)( fieldId[fieldCount]       & 0xFFU);
                    ptrUcbPacket->payload[(fieldCount * 4U) + 3U] = (uint8_t)((param >> 8U) & 0xFFU);
                    ptrUcbPacket->payload[(fieldCount * 4U) + 4U] = (uint8_t)( param  & 0xFFU);
                }
            }
            HandleUcbTx(port, ptrUcbPacket);
        }

        /// any invalid get fields addresses?
        if (validFieldCount < numFields) {
            SetNak(port, ptrUcbPacket);
            HandleUcbTx(port, ptrUcbPacket);
        }
    } else {
        SetNak(port, ptrUcbPacket);
        HandleUcbTx(port, ptrUcbPacket);
    }
}

/**********************************************
* @brief 
* 
* @param port 
* @param ptrUcbPacket 
***********************************************/
static void UcbReadFields (uint16_t const port,
                            UcbPacketStruct * const ptrUcbPacket)
{
	uint8_t  const numFields = ptrUcbPacket->payload[0];

    /// verify that the packet length matches packet specification
    if ((numFields > 0U) &&
    (ptrUcbPacket->payloadLength == (1U + (numFields * 2U)))) {
	    
        uint8_t  fieldCount;
	    uint8_t  validFieldCount = 0U;
	    uint16_t fieldId [UCB_MAX_PAYLOAD_LENGTH / 4];
        uint16_t idTmp;

        for (fieldCount = 0U; fieldCount < numFields; ++fieldCount) {
            /// read field ID from packet into usable array
            idTmp   = ptrUcbPacket->payload[(fieldCount * 2U) + 1U];
            idTmp <<= 8U;
            idTmp  |= ptrUcbPacket->payload[(fieldCount * 2U) + 2U];
            fieldId[validFieldCount] = idTmp;

            /// check read field address bounds
            if (((fieldId[validFieldCount] >= (uint16_t)LOWER_CONFIG_ADDR_BOUND) &&
                 (fieldId[validFieldCount] <= (uint16_t)UPPER_CONFIG_ADDR_BOUND)) ||
                 (fieldId[validFieldCount] == (uint16_t)PRODUCT_CONFIGURATION_FIELD_ID)) {
                ++validFieldCount;
            }
        }

        if (validFieldCount > 0U) { /// all or some requested addresses valid?
            /// build and return valid get fields with data packet
            ptrUcbPacket->payloadLength = (uint8_t)(1U + (validFieldCount * 4U));

            ptrUcbPacket->payload[0] = validFieldCount; ///< # being returned

            /// retrieve all fields specified in get fields request
            for (fieldCount = 0U; fieldCount < validFieldCount; ++fieldCount) {
                /** product configuration field is out of normal configuration
                    address range, needs to be fetched from calibration
                    structure */
                uint16_t fieldData;
                if (fieldId[fieldCount] == (uint16_t)PRODUCT_CONFIGURATION_FIELD_ID) {
                    ptrUcbPacket->payload[(fieldCount * 4U) + 1U] = (uint8_t)((fieldId[fieldCount] >> 8U) & 0xFFU);
                    ptrUcbPacket->payload[(fieldCount * 4U) + 2U] = (uint8_t)( fieldId[fieldCount]       & 0xFFU);

                    fieldData = cal_GetProductConfiguration();
                    ptrUcbPacket->payload[(fieldCount * 4U) + 3U] = (uint8_t)((fieldData >> 8U) & 0xFFU);
                    ptrUcbPacket->payload[(fieldCount * 4U) + 4U] = (uint8_t)( fieldData       & 0xFFU);
                } else {	/// normal field, exists in configuration structure
                    ptrUcbPacket->payload[(fieldCount * 4U) + 1U] = (uint8_t)((fieldId[fieldCount] >> 8U) & 0xFFU);
                    ptrUcbPacket->payload[(fieldCount * 4U) + 2U] = (uint8_t)( fieldId[fieldCount]       & 0xFFU);
                    /// read field from EEPROM
                    EEPROM_ReadByte(fieldId[fieldCount], sizeof(fieldData), (uint8_t*)&fieldData);
                    ptrUcbPacket->payload[(fieldCount * 4U) + 3U] = (uint8_t)((fieldData >> 8U) & 0xFFU);
                    ptrUcbPacket->payload[(fieldCount * 4U) + 4U] = (uint8_t)( fieldData       & 0xFFU);
                }
            }
            HandleUcbTx(port, ptrUcbPacket);
        }

        /// invalid get fields addresses?
        if (validFieldCount < numFields) {
            SetNak(port, ptrUcbPacket);
            HandleUcbTx(port, ptrUcbPacket);
        }
    } else {
        SetNak(port, ptrUcbPacket);
        HandleUcbTx(port, ptrUcbPacket);
    }

}

/**********************************************
* @brief 
* 
* @param port ---
* @param ptrUcbPacket ---
***********************************************/
static void UcbWriteFields (uint16_t const port,
                             UcbPacketStruct * const ptrUcbPacket)
{
    uint8_t  const numFields = ptrUcbPacket->payload[0];

    /// verify that the packet length matches packet specification
    if( ( numFields > 0U ) &&
        ( ptrUcbPacket->payloadLength == (1U + (numFields * 4U)) ) )
    {
        uint8_t  fieldCount;
        uint8_t  validFieldCount;
        uint16_t fieldId   [UCB_MAX_PAYLOAD_LENGTH / 4];
        uint16_t fieldData [UCB_MAX_PAYLOAD_LENGTH / 4];

        /// loop through all fields and data specified in set fields request
        for (fieldCount = 0U; fieldCount < numFields; ++fieldCount) {
            /// read field ID and field data from packet into usable arrays
            fieldId[fieldCount]   = (uint16_t)(((uint16_t)ptrUcbPacket->payload[(fieldCount * 4U) + 1U] << 8U) |
                                                (uint16_t)ptrUcbPacket->payload[(fieldCount * 4U) + 2U]);
            fieldData[fieldCount] = (uint16_t)(((uint16_t)ptrUcbPacket->payload[(fieldCount * 4U) + 3U] << 8U) |
                                                (uint16_t)ptrUcbPacket->payload[(fieldCount * 4U) + 4U]);
        }

        /// check if data to set is valid xbowsp_fields.c
        validFieldCount = config_CheckEepromFieldData(numFields,
                                               fieldId,
                                               fieldData,
                                               fieldId);
// there is no check for corect number of changed fields only that something has changed
        if (validFieldCount > 0U) { ///< all or some requested field changes valid?
            /// apply any changes
            if (config_WriteFieldData() == TRUE) { // xbowsp_fields.c
                /// build and send positive acknowledgement packet
                ptrUcbPacket->payloadLength = (uint8_t)(1U + (validFieldCount * 2U));

                /// number of valid fields
                ptrUcbPacket->payload[0] = validFieldCount;

                /// place valid field ID's in payload
                for (fieldCount = 0U; fieldCount < validFieldCount; ++fieldCount) {
                    ptrUcbPacket->payload[(fieldCount * 2U) + 1U] = (uint8_t)((fieldId[fieldCount] >> 8U) & 0xFFU);
                    ptrUcbPacket->payload[(fieldCount * 2U) + 2U] = (uint8_t)( fieldId[fieldCount]       & 0xFFU);
                }
                HandleUcbTx(port, ptrUcbPacket);
            } else {
                SetNak(port, ptrUcbPacket);
                HandleUcbTx(port, ptrUcbPacket);
            }
        }

        /// any invalid requested field changes?
        if (validFieldCount < numFields) {
            SetNak(port, ptrUcbPacket);
            HandleUcbTx(port, ptrUcbPacket);
        }
    } else {
        SetNak(port, ptrUcbPacket);
        HandleUcbTx(port, ptrUcbPacket);
    }
}

/**********************************************
* @brief 
* 
* @param port 
* @param ptrUcbPacket 
***********************************************/
static void UcbUnlockEeprom (uint16_t const port,
                              UcbPacketStruct * const ptrUcbPacket)
{
    BOOL const res = EEPROM_UnlockCalSectors(ptrUcbPacket->payload);

 	if (res == TRUE) 
    { ///< correct unlock code?
        BIT_SetEepromLockStatus(TRUE);
	    ptrUcbPacket->payloadLength = 0U;
    }else {
        SetNak(port, ptrUcbPacket);
	}
	HandleUcbTx(port, ptrUcbPacket);
} /* end HandleUcbUnlockEeprom() */
    
/**********************************************
* @brief 
* 
* @param port 
* @param ptrUcbPacket 
***********************************************/
static void UcbLockEeprom (uint16_t const port,
                              UcbPacketStruct * const ptrUcbPacket)
{
    BOOL const res = EEPROM_LockCalSectors();

 	if (res == TRUE) 
    { ///< correct unlock code?
        BIT_SetEepromLockStatus(FALSE);
	    ptrUcbPacket->payloadLength = 0U;
    }else {
        SetNak(port, ptrUcbPacket);
    }
	HandleUcbTx(port, ptrUcbPacket);
} /* end HandleUcbUnlockEeprom() */


/**********************************************
* @brief 
* 
* @param port 
* @param ptrUcbPacket 
***********************************************/
static void UcbReadEeprom (uint16_t const port, UcbPacketStruct * const ptrUcbPacket)
{
    uint32_t startAddress;
    uint8_t  wordsToRead;
    uint8_t  bytesToRead;

    startAddress = ((uint32_t)ptrUcbPacket->payload[0] << 8U) | ptrUcbPacket->payload[1];
    wordsToRead  = ptrUcbPacket->payload[2];
    bytesToRead  = (uint8_t)(wordsToRead * 2U);

    /// verify that the packet length matches packet specification
    if (ptrUcbPacket->payloadLength == 3U) {
        ptrUcbPacket->payloadLength = (uint8_t)(ptrUcbPacket->payloadLength + bytesToRead);
        EEPROM_ReadByte((uint16_t)startAddress, bytesToRead, &(ptrUcbPacket->payload[3]));
	} else {
        SetNak(port, ptrUcbPacket);
    }
	HandleUcbTx(port, ptrUcbPacket);

}

/**********************************************
* @brief 
* 
* @param port 
* @param ptrUcbPacket 
***********************************************/
static void UcbReadCal (uint16_t const port, UcbPacketStruct* const ptrUcbPacket)
{
    uint32_t startAddress;
    uint8_t  wordsToRead;
    uint8_t  bytesToRead;

    startAddress  = ((uint32_t)ptrUcbPacket->payload[0] << 8U) | ptrUcbPacket->payload[1];
    startAddress *= 2U;
    wordsToRead  = ptrUcbPacket->payload[2];
    bytesToRead  = (uint8_t)(wordsToRead * 2U);

    /// verify that the packet length matches packet specification
    if (ptrUcbPacket->payloadLength == 3U) {
        ptrUcbPacket->payloadLength = (uint8_t)(ptrUcbPacket->payloadLength + bytesToRead);
        handStat += (uint8_t)EEPROM_ReadFromCalPartition((uint16_t)startAddress, bytesToRead, &(ptrUcbPacket->payload[3]));
	} else {
        SetNak(port, ptrUcbPacket);
    }
	HandleUcbTx(port, ptrUcbPacket);
}


/**********************************************
* @brief 
* 
* @param port 
* @param ptrUcbPacket 
***********************************************/
static void UcbWriteEeprom (uint16_t const port,
                             UcbPacketStruct* const ptrUcbPacket)
{
    uint32_t startAddress;
    uint8_t  wordsToWrite;
    uint16_t bytesToWrite;

    startAddress = ((uint32_t)ptrUcbPacket->payload[0] << 8U) | ptrUcbPacket->payload[1];
    wordsToWrite = ptrUcbPacket->payload[2];
    bytesToWrite = (uint16_t)wordsToWrite * 2U;


    /// verify that the packet length matches packet specification
    if (ptrUcbPacket->payloadLength == (bytesToWrite + 3U)){
        /// 0 means no errors
        if (EEPROM_WriteWords((uint16_t)startAddress,
                             wordsToWrite,
                             &(ptrUcbPacket->payload[3])) == 0) {
            ptrUcbPacket->payloadLength = 3U;
        } else {
            SetNak(port, ptrUcbPacket);
        }
    } else {
        SetNak(port, ptrUcbPacket);
    }

    HandleUcbTx(port, ptrUcbPacket);

}

/**********************************************
* @brief 
* 
* @param port 
* @param ptrUcbPacket 
***********************************************/
static void UcbWriteCal (uint16_t const port,
                          UcbPacketStruct * const ptrUcbPacket)
{
    uint32_t    startAddress;
    uint8_t     wordsToWrite;
    uint16_t    bytesToWrite;

    startAddress = ((uint32_t)ptrUcbPacket->payload[0] << 8U) | ptrUcbPacket->payload[1];
    startAddress *= 2U;
    wordsToWrite  = ptrUcbPacket->payload[2];
    bytesToWrite  = (uint16_t)wordsToWrite * 2U;


    /// verify that the packet length matches packet specification
    if (ptrUcbPacket->payloadLength == (bytesToWrite + 3U)) {
        /// 0 means no errors
        if (EEPROM_WriteToCalPartition((uint16_t)startAddress, bytesToWrite,
                             &(ptrUcbPacket->payload[3])) != 0) {
            ptrUcbPacket->payloadLength = 3U;
        } else {
            SetNak(port, ptrUcbPacket);
        }
    } else {
        SetNak(port, ptrUcbPacket);
    }

    HandleUcbTx(port, ptrUcbPacket);

}

/**********************************************
* @brief 
* 
* @param port 
* @param ptrUcbPacket 
***********************************************/
static void UcbJump2BOOT (uint16_t const port, UcbPacketStruct* const ptrUcbPacket)
{

	//set need update app flag
    HW_EnforceSerialBootMode();

	HandleUcbTx(port, ptrUcbPacket);

    OS_Delay(10);

    HW_SystemReset();

} 

/**********************************************
* @brief 
* 
* @param port 
* @param ptrUcbPacket 
***********************************************/
static void UcbJump2IAP (uint16_t const port, UcbPacketStruct * const ptrUcbPacket)
{

	//set need update app flag
    if(EEPROM_PrepareToEnterBootloader())
    {
		SetNak(port,ptrUcbPacket);
    }

	HandleUcbTx(port, ptrUcbPacket);

    OS_Delay(10);

    HW_SystemReset();

} 

/**********************************************
* @brief 
* 
* @param port 
* @param ptrUcbPacket 
***********************************************/
static void UcbJump2APP (uint16_t const port, UcbPacketStruct* const ptrUcbPacket)
{
	    HandleUcbTx(port, ptrUcbPacket);
        // nothing to do - already there
        return;
} 

/**********************************************
* @brief 
* 
* @param port 
* @param ptrUcbPacket 
***********************************************/
static void UcbSoftwareReset (uint16_t const port,
                                UcbPacketStruct * const ptrUcbPacket)
{
    /// return software reset acknowledgement
	HandleUcbTx(port, ptrUcbPacket);

     OS_Delay(10);
   
    HW_SystemReset();
}

/**********************************************
* @brief 
* 
* @param port 
* @param ptrUcbPacket 
***********************************************/
static void UcbError (uint16_t const port,
                       UcbPacketStruct* const ptrUcbPacket)
{
	/// return NAK, requested packet type placed in data field by external port
	ptrUcbPacket->packetType 	= (uint8_t)UCB_NAK;
	ptrUcbPacket->payloadLength = (uint8_t)(uint16_t)UCB_PACKET_TYPE_LENGTH;
    HandleUcbTx(port, ptrUcbPacket);
}


/**********************************************
* @brief 
* 
* @param port 
* @param ptrUcbPacket 
***********************************************/
void HandleUcbPacket (uint16_t const port, UcbPacketStruct* const ptrUcbPacket)
{
		switch ((int32_t)ptrUcbPacket->packetType) {
            case UCB_PING:
                UcbPing(port, ptrUcbPacket); 
                break;
            case UCB_ECHO:
                UcbEcho(port, ptrUcbPacket); 
                break;
            case UCB_GET_PACKET:
                UcbGetPacket(port, ptrUcbPacket); 
                break;
            case UCB_GET_FIELDS:
                UcbGetFields(port, ptrUcbPacket); 
                break;
            case UCB_J2BOOT:
                UcbJump2BOOT  (port, ptrUcbPacket); 
                break;
            case UCB_J2IAP:
                UcbJump2IAP   (port, ptrUcbPacket); 
                break;
            case UCB_J2APP:
                UcbJump2APP (port, ptrUcbPacket); 
                break;
            case UCB_READ_CAL:
                UcbReadCal(port, ptrUcbPacket); 
                break;
            case UCB_SOFTWARE_RESET:
                UcbSoftwareReset(port, ptrUcbPacket); 
                break;
            case UCB_SET_FIELDS:
                UcbSetFields(port, ptrUcbPacket); 
                break;
            case UCB_READ_FIELDS:
                UcbReadFields(port, ptrUcbPacket); 
                break;
            case UCB_WRITE_FIELDS:
                UcbWriteFields(port, ptrUcbPacket); 
                break;
            case UCB_UNLOCK_EEPROM:
                UcbUnlockEeprom(port, ptrUcbPacket); 
                break;
            case UCB_LOCK_EEPROM:
                UcbLockEeprom(port, ptrUcbPacket); 
                break;
            case UCB_READ_EEPROM:
                UcbReadEeprom(port, ptrUcbPacket); 
                break;
            case UCB_WRITE_EEPROM:
                UcbWriteEeprom(port, ptrUcbPacket); 
                break;
            case UCB_WRITE_CAL:
                UcbWriteCal(port, ptrUcbPacket); 
                break;
            case UCB_USER_IN:
                {
                    int32_t const result = HandleUserInputPacket(ptrUcbPacket);
                    if(result != USER_PACKET_OK){
	            SetNak(port,ptrUcbPacket);
                    }
                HandleUcbTx(port, ptrUcbPacket);
                }
                break;
            default:
                UcbError(port, ptrUcbPacket); 
                break; /// default handler - unknown send NAK
		}
}
/* end HandleUcbPacket() */

