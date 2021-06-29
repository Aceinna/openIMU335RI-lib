/** ***************************************************************************
 * @file parameters.h 
  *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
  * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
  * PARTICULAR PURPOSE.
  *****************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License"),    
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/
#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "ucb_packet.h"
#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif
 
/**********************************************
* @brief 
* 
* @param portBaudRate [ ]
* @return BOOL 
***********************************************/
extern BOOL   	CheckPortBaudRate 		    (uint16_t portBaudRate);    

/**********************************************
* @brief 
* 
* @param packetRateDivider [ ]
* @return BOOL 
***********************************************/
extern BOOL   	CheckPacketRateDivider	  (uint16_t packetRateDivider);    

/**********************************************
* @brief 
* 
* @param outputPacket [ ]
* @param baudRate [ ]
* @param packetRateDivider [ ]
* @return BOOL 
***********************************************/
extern BOOL		  CheckContPacketRate       (UcbPacketType outputPacket, uint16_t baudRate, uint16_t packetRateDivider);    

enum{

    LOWER_CONFIG_ADDR_BOUND				=      0x0001 ,     ///< lower configuration address boundary
    PACKET_RATE_DIVIDER_FIELD_ID  	=	   0x0001,    	///< continuous packet rate divider
    BAUD_RATE_USER_ID             	=  	 0x0002,    	///< continuous packet rate divider
    PACKET_TYPE_FIELD_ID  				  =    0x0003,    	///< continuous packet type
    ORIENTATION_FIELD_ID  			 =   0x0007,    	/// !! < user defined axis orientation
    USER_BEHAVIOR_FIELD_ID				   =   0x0008,    	///< user behaviour switches
    ECU_ADDRESS_FIELD_ID             =   0x0032,        // ++ !!
    ECU_BAUD_RATE_FIELD_ID           =   0x0033,        // ++ !!
    ECU_BEHAVIOR_FIELD_ID            =   0x0064,        // ++ !! 
    ECU_ODR_FIELD_ID                 =   0x003C,        // ++ !!
    ACCEL_LPF_FIELD_ID               =   0x0005,        // ++
    RATE_LPF_FIELD_ID                =   0x0006,        // ++
    ECU_PACKET_TYPE_FIELD_ID         =   0x003D,        // ++ !!
    UPPER_CONFIG_ADDR_BOUND			 =   0x007F,    ///< upper configuration address boundary
    PRODUCT_CONFIGURATION_FIELD_ID	 =	 0x071C,    	///< outside of configuration, but needs to be read as a field
};    

/**********************************************
* @brief 
* 
* @param orientation [ ]
* @return BOOL 
***********************************************/
extern BOOL CheckOrientation (uint16_t orientation) ;

/**********************************************
* @brief 
* 
* @param response [ ]
* @param index [ ]
* @return uint16_t 
***********************************************/
extern uint16_t appendAttitudeTrue (uint8_t *response, uint16_t index);

/**********************************************
* @brief 
* 
* @param response [ ]
* @param index [ ]
* @return uint16_t 
***********************************************/
extern uint16_t appendCorrectedRates (uint8_t *response, uint16_t index);    

/**********************************************
* @brief 
* 
* @param response [ ]
* @param index [ ]
* @return uint16_t 
***********************************************/
extern uint16_t appendAccels (uint8_t *response, uint16_t index);

/**********************************************
* @brief 
* 
* @param response [ ]
* @param index [ ]
* @param chipId [ ]
* @return uint16_t 
***********************************************/
extern uint16_t appendChipAccels (uint8_t *response, uint16_t index, int32_t chipId);

/**********************************************
* @brief 
* 
* @param response [ ]
* @param index [ ]
* @return uint16_t 
***********************************************/
extern uint16_t appendRates (uint8_t *response, uint16_t index);

/**********************************************
* @brief 
* 
* @param response [ ]
* @param index [ ]
* @param chipId [ ]
* @return uint16_t 
***********************************************/
extern uint16_t appendChipRates (uint8_t *response, uint16_t index, int32_t chipId);

/**********************************************
* @brief 
* 
* @param response [ ]
* @param index [ ]
* @return uint16_t 
***********************************************/
extern uint16_t appendMagReadings (uint8_t *response, uint16_t index);

/**********************************************
* @brief 
* 
* @param response [ ]
* @param index [ ]
* @return uint16_t 
***********************************************/
extern uint16_t appendTemps (uint8_t *response, uint16_t index);

/**********************************************
* @brief 
* 
* @param response [ ]
* @param index [ ]
* @param chipId [ ]
* @return uint16_t 
***********************************************/
extern uint16_t appendChipTemps (uint8_t *response, uint16_t index, int32_t chipId);

/**********************************************
* @brief 
* 
* @param response [ ]
* @param index [ ]
* @return uint16_t 
***********************************************/
extern uint16_t appendInertialCounts (uint8_t *response, uint16_t index);


/**********************************************
* @brief 
* 
* @param response [ ]
* @param index [ ]
* @return uint16_t 
***********************************************/
extern uint16_t appendAllTempCounts (uint8_t *response, uint16_t index);

/**********************************************
* @brief 
* 
* @param response [ ]
* @param index [ ]
* @return uint16_t 
***********************************************/
extern uint16_t appendRateTemp (uint8_t  *response, uint16_t index);

/**********************************************
* @brief 
* 
* @param response [ ]
* @param index [ ]
* @return uint16_t 
***********************************************/
uint16_t        appendTemp (uint8_t  *response, uint16_t index);


// API fcns to load words and shorts into the byte buffers
/**********************************************
* @brief 
* 
* @param buffer [ ]
* @param index [ ]
* @param inWord [ ]
* @return uint16_t 
***********************************************/
extern uint16_t uint32ToBuffer(uint8_t *buffer, uint16_t index, uint32_t inWord);    

/**********************************************
* @brief 
* 
* @param buffer [ ]
* @param index [ ]
* @param inWord [ ]
* @return uint16_t 
***********************************************/
extern uint16_t uint16ToBuffer(uint8_t *buffer, uint16_t index, uint16_t inWord);    

#ifdef __cplusplus
}
#endif

#endif