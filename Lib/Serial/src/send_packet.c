/** ***************************************************************************
 * @file send_packet.c UCB callbacks for assembling then sending serial packets
 * to NAV-VIEW.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * @brief this is the serial send handler for the Nav View data link
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

//****************************
#include "ucb_packet.h"
#include "partNumber.h"
#include "parameters.h"
#include "halAPI.h"
#include "ekfAPI.h"
#include "sensorsAPI.h"
#include "crc16.h"
#include "configurationAPI.h"
#include "calibrationAPI.h"
#include "bitAPI.h"
#include "Indices.h"
#include "appVersion.h"

static uint32_t funCount = 0U;


/*******************************************
 * @brief 
 * 
 * @param port ==
 * @param ptrUcbPacket ==
********************************************/
static void UcbIdentification (uint16_t const port,
                         UcbPacketStruct* const ptrUcbPacket)

{
    uint16_t packetIndex = 0U;
    int32_t  stringIndex = 0;
    static uint32_t  sernum = 0U;
    static uint8_t *versionString;

    sernum        = cal_GetUnitSerialNum();
    versionString = cal_GetUnitVersion();

    /// serial number
    packetIndex = (uint8_t)uint32ToBuffer(ptrUcbPacket->payload,
                                         packetIndex,
                                         sernum);
	/// model string
	while ((stringIndex < N_VERSION_STR) && (versionString[stringIndex] != 0U))
	{
		ptrUcbPacket->payload[packetIndex] = (uint8_t)versionString[stringIndex];
        packetIndex++;
        stringIndex++;
    }

	/// space between
	ptrUcbPacket->payload[packetIndex] = 0x20U;
	packetIndex++;
    stringIndex = 0;

	/// software part number
	while ((stringIndex < SOFTWARE_PART_LEN) && ((uint8_t)PART_NUMBER_STRING[stringIndex] != 0U)) {
		ptrUcbPacket->payload[packetIndex] = (uint8_t)PART_NUMBER_STRING[stringIndex];
		packetIndex++;
        stringIndex++;
	}

    stringIndex = 0;
	while ((uint8_t)APP_VERSION_STRING[stringIndex] != 0U) {
		ptrUcbPacket->payload[packetIndex] = (uint8_t)APP_VERSION_STRING[stringIndex];
		packetIndex++;
        stringIndex++;
	}

	ptrUcbPacket->payload[packetIndex] = 0U;  ///< zero delimiter
	packetIndex++;  
	ptrUcbPacket->payloadLength          = (uint8_t)packetIndex; ///< return packet length
	HandleUcbTx(port, ptrUcbPacket); ///< send identification packet
}

/*******************************************
 * @brief 
 * 
 * @param port ==
 * @param ptrUcbPacket ==
********************************************/
static void UcbVersionData (uint16_t const port,
                      UcbPacketStruct* const ptrUcbPacket)
{
	/// return packet length
	ptrUcbPacket->payloadLength = (uint8_t)UCB_VERSION_DATA_LENGTH;

    /// 525 digital processor DUP version data - append
	ptrUcbPacket->payload[0] = (uint8_t)VERSION_MAJOR;
	ptrUcbPacket->payload[1] = (uint8_t)VERSION_MINOR;
	ptrUcbPacket->payload[2] = (uint8_t)VERSION_PATCH;
	ptrUcbPacket->payload[3] = (uint8_t)VERSION_STAGE;
	ptrUcbPacket->payload[4] = (uint8_t)VERSION_BUILD;
	HandleUcbTx(port, ptrUcbPacket); /// send version data packet
}

/*******************************************
 * @brief 
 * 
 * @param port ==
 * @param ptrUcbPacket ==
********************************************/
static void UcbVersionAllData (uint16_t const port,
                         UcbPacketStruct* const ptrUcbPacket)
{
    uint8_t packetIndex = 0U;
    uint8_t bootVersion[6];

    config_GetBootloaderVersion(bootVersion);
    // DUP IOUP are here to allow compatibility with NavView
	/// 525 digital processor DUP version data
	ptrUcbPacket->payload[packetIndex] = (uint8_t)VERSION_MAJOR;
    packetIndex++;
	ptrUcbPacket->payload[packetIndex] = (uint8_t)VERSION_MINOR;
    packetIndex++;
	ptrUcbPacket->payload[packetIndex] = (uint8_t)VERSION_PATCH;
    packetIndex++;
	ptrUcbPacket->payload[packetIndex] = (uint8_t)VERSION_STAGE;
    packetIndex++;
	ptrUcbPacket->payload[packetIndex] = (uint8_t)VERSION_BUILD;
    packetIndex++;
    /// 525 input output processor IOUP version data
	ptrUcbPacket->payload[packetIndex] = (uint8_t)VERSION_MAJOR;
    packetIndex++;
	ptrUcbPacket->payload[packetIndex] = (uint8_t)VERSION_MINOR;
    packetIndex++;
	ptrUcbPacket->payload[packetIndex] = (uint8_t)VERSION_PATCH;
    packetIndex++;
	ptrUcbPacket->payload[packetIndex] = (uint8_t)VERSION_STAGE;
    packetIndex++;
	ptrUcbPacket->payload[packetIndex] = (uint8_t)VERSION_BUILD;
    packetIndex++;
    /// boot version data
	ptrUcbPacket->payload[packetIndex] = bootVersion[0];
    packetIndex++;
	ptrUcbPacket->payload[packetIndex] = bootVersion[1];
    packetIndex++;
	ptrUcbPacket->payload[packetIndex] = bootVersion[2];
    packetIndex++;
	ptrUcbPacket->payload[packetIndex] = bootVersion[3];
    packetIndex++;
	ptrUcbPacket->payload[packetIndex] = bootVersion[4];
    packetIndex++;

	ptrUcbPacket->payloadLength = packetIndex; ///< return packet length
	HandleUcbTx(port, ptrUcbPacket); 	///< send version all data packet
}

/*******************************************
 * @brief 
 * 
 * @param port ==
 * @param ptrUcbPacket ==
********************************************/
static void UcbAngle1 (uint16_t const port,
                 UcbPacketStruct* const ptrUcbPacket)
{
	uint16_t  packetIndex = 0U;
	Crc32Type payloadCrc;

	ptrUcbPacket->payloadLength = (uint8_t)UCB_ANGLE_1_LENGTH; /// set packet length
    // xbowsp_fields.c
	/// roll angle, pitch angle, magnetometer yaw angle
	packetIndex = appendAttitudeTrue(ptrUcbPacket->payload, packetIndex);

        /// X-angular rate, Y, Z
    packetIndex = appendCorrectedRates(ptrUcbPacket->payload, packetIndex);

        /// X-accelerometer, Y, Z (according to the serial-interface spec, the
        ///   accelerometer signal is uncorrected).
    packetIndex = appendAccels(ptrUcbPacket->payload, packetIndex);

    /// magnetometer X, Y, Z
    packetIndex = appendMagReadings(ptrUcbPacket->payload, packetIndex);

    /// X rate temp
    packetIndex = appendRateTemp(ptrUcbPacket->payload, packetIndex);

    packetIndex = uint32ToBuffer(ptrUcbPacket->payload, /// timer
                                 packetIndex,
                                 EKF_GetTimer());

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                  packetIndex,
                                  BIT_GetMasterStatusWord());
	/// compute Universal payload CRC
	payloadCrc = Crc32(ptrUcbPacket->payload,
                       (UCB_ANGLE_U_LENGTH - 4),
                       CRC_32_INITIAL_SEED);
	/// Universal payload CRC
	Crc32TypeToBytes (payloadCrc, &(ptrUcbPacket->payload[packetIndex]));

    HandleUcbTx(port, ptrUcbPacket); /// send Angle 1 packet

}

/*******************************************
 * @brief 
 * 
 * @param port ==
 * @param ptrUcbPacket ==
********************************************/
static void UcbAngle2 (uint16_t const port,
                 UcbPacketStruct* const ptrUcbPacket)
{
	uint16_t  packetIndex = 0U;
	Crc32Type payloadCrc;

	ptrUcbPacket->payloadLength = (uint8_t)UCB_ANGLE_2_LENGTH; /// set packet length
    // xbowsp_fields.c
	/// roll angle, pitch angle, magnetometer yaw angle
	packetIndex = appendAttitudeTrue(ptrUcbPacket->payload, packetIndex);
	/// X-angular rate, Y, Z
	packetIndex = appendCorrectedRates(ptrUcbPacket->payload, packetIndex);
	/// X-accelerometer, Y, Z
	packetIndex = appendAccels(ptrUcbPacket->payload, packetIndex);

    /// X,Y,Z rate temp (do not include board temp per Serial Interface Spec, i.e. do not
    ///   use appendTemps)
    packetIndex = appendRateTemp(ptrUcbPacket->payload, packetIndex);
    packetIndex = appendRateTemp(ptrUcbPacket->payload, packetIndex);
    packetIndex = appendRateTemp(ptrUcbPacket->payload, packetIndex);

    packetIndex = uint32ToBuffer(ptrUcbPacket->payload, /// timer
                                 packetIndex,
                                 EKF_GetTimer());

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                  packetIndex,
                                  BIT_GetMasterStatusWord());
	/// compute Universal payload CRC
	payloadCrc = Crc32(ptrUcbPacket->payload,
                       (UCB_ANGLE_U_LENGTH - 4),
                       CRC_32_INITIAL_SEED);
	/// Universal payload CRC
	Crc32TypeToBytes (payloadCrc, &(ptrUcbPacket->payload[packetIndex]));
	HandleUcbTx(port, ptrUcbPacket); /// send Angle 2 packet
}

/*******************************************
 * @brief 
 * 
 * @param port ==
 * @param ptrUcbPacket ==
********************************************/
static void UcbAngle3 (uint16_t const port,
                 UcbPacketStruct* const ptrUcbPacket)
{
	uint16_t  packetIndex = 0U;
	Crc32Type payloadCrc;

	ptrUcbPacket->payloadLength = (uint8_t)UCB_ANGLE_3_LENGTH; /// set packet length
    // xbowsp_fields.c
	/// roll angle, pitch angle, magnetometer yaw angle
	packetIndex = appendAttitudeTrue(ptrUcbPacket->payload, packetIndex);
	/// X-angular rate, Y, Z
	packetIndex = appendRates(ptrUcbPacket->payload, packetIndex);
	/// X-accelerometer, Y, Z
	packetIndex = appendAccels(ptrUcbPacket->payload, packetIndex);

    /// X,Y,Z rate temp (do not include board temp per Serial Interface Spec, i.e. do not
    ///   use appendTemps)
    packetIndex = appendRateTemp(ptrUcbPacket->payload, packetIndex);
    packetIndex = appendRateTemp(ptrUcbPacket->payload, packetIndex);
    packetIndex = appendRateTemp(ptrUcbPacket->payload, packetIndex);

    packetIndex = uint32ToBuffer(ptrUcbPacket->payload, /// timer
                                 packetIndex,
                                 EKF_GetTimer());

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                  packetIndex,
                                  BIT_GetMasterStatusWord());
	/// compute Universal payload CRC
	payloadCrc = Crc32(ptrUcbPacket->payload,
                       (UCB_ANGLE_U_LENGTH - 4),
                       CRC_32_INITIAL_SEED);
	/// Universal payload CRC
	Crc32TypeToBytes (payloadCrc, &(ptrUcbPacket->payload[packetIndex]));
	HandleUcbTx(port, ptrUcbPacket); /// send Angle 3 packet
}

/*******************************************
 * @brief 
 * 
 * @param port ==
 * @param ptrUcbPacket ==
********************************************/
static void UcbScaled0 (uint16_t const port,
                  UcbPacketStruct* const ptrUcbPacket)
{
	uint16_t packetIndex = 0U;

	/// set packet length
	ptrUcbPacket->payloadLength = (uint8_t)UCB_SCALED_0_LENGTH;
    /// X-accelerometer, Y, Z
	packetIndex = appendAccels(ptrUcbPacket->payload, packetIndex);
	/// X-angular, Y, Z rate
	packetIndex = appendRates(ptrUcbPacket->payload, packetIndex);
	/// X-magnetometer, Y, Z
	packetIndex = appendMagReadings(ptrUcbPacket->payload, packetIndex);
	/// rate and board temperature
	packetIndex = appendTemps(ptrUcbPacket->payload, packetIndex);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, // itow???
                                  packetIndex,
                                  TIMER_GetRollingCount());

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                  packetIndex,
                                  BIT_GetMasterStatusWord());
    funCount += packetIndex;

    HandleUcbTx(port, ptrUcbPacket); /// send Scaled 0 packet
}

/*******************************************
 * @brief 
 * 
 * @param port ==
 * @param ptrUcbPacket ==
********************************************/
static void UcbScaled1 (uint16_t const port,
                  UcbPacketStruct * const ptrUcbPacket)
{
	uint16_t packetIndex = 0U;

	ptrUcbPacket->payloadLength = (uint8_t)UCB_SCALED_1_LENGTH;
    /// X-accelerometer, Y, Z
	packetIndex = appendAccels(ptrUcbPacket->payload, packetIndex);
	/// X-angular rate, Y, Z
	packetIndex = appendRates(ptrUcbPacket->payload, packetIndex);
	/// rate and board temperature
	packetIndex = appendTemps(ptrUcbPacket->payload, packetIndex);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// packet counter
                                 packetIndex,
                                 TIMER_GetRollingCount());

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                 packetIndex,
                                 BIT_GetMasterStatusWord());
    funCount += packetIndex;

    HandleUcbTx(port, ptrUcbPacket); /// send Scaled 1 packet
}

/*******************************************
 * @brief 
 * 
 * @param port ==
 * @param ptrUcbPacket ==
********************************************/
static void UcbScaledM (uint16_t const port, UcbPacketStruct* const ptrUcbPacket)
{
	static uint16_t sampleIdx = 0U;
	static uint16_t const sampleSubset = 0U;
    uint16_t packetIndex = 0U;

	ptrUcbPacket->payloadLength = (uint8_t)UCB_SCALED_M_LENGTH;

    for(int32_t i = 0; i < NUM_SENSOR_CHIPS; i++){
        /// X-accelerometer, Y, Z
	    packetIndex = appendChipAccels(ptrUcbPacket->payload, packetIndex, i);
	    /// X-angular rate, Y, Z
	    packetIndex = appendChipRates(ptrUcbPacket->payload, packetIndex, i);
	    /// rate temperature
	    packetIndex = appendChipTemps(ptrUcbPacket->payload, packetIndex, i);
    }

    /// X-accelerometer, Y, Z
    packetIndex = appendAccels(ptrUcbPacket->payload, packetIndex);
    /// X-angular rate, Y, Z
    packetIndex = appendRates(ptrUcbPacket->payload, packetIndex);
    /// rate temperature
    packetIndex = appendTemp(ptrUcbPacket->payload, packetIndex);


    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// sensors subset
                                 packetIndex,
                                sampleSubset );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// sampleIdx
                                 packetIndex,
                                 sampleIdx);
    sampleIdx++;
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                 packetIndex,
                                 BIT_GetMasterStatusWord());
    funCount += packetIndex;

    HandleUcbTx(port, ptrUcbPacket); /// send Scaled 1 packet
}

/*******************************************
 * @brief 
 * 
 * @param port ==
 * @param ptrUcbPacket ==
********************************************/
static void UcbTest0 (uint16_t const port, UcbPacketStruct* const ptrUcbPacket)
{
	uint16_t packetIndex = 0U;

	/// set packet length
	ptrUcbPacket->payloadLength = (uint8_t)UCB_TEST_0_LENGTH;
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT
                                 packetIndex,
                                 BIT_GetMasterStatusWord() );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// Hardware BIT
                                 packetIndex,
                                 BIT_GetHWStatusWord());
    
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// Software BIT MSB
                                 packetIndex,
                                 (uint16_t)(BIT_GetSWStatusWord()>>16U)); 
	
    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// Software BIT LSB
                                  packetIndex,
                                  (uint16_t)(BIT_GetSWStatusWord() & 0xFFFFU));

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// com BIT
                                 packetIndex,
                                 BIT_GetCommStatusWord() );

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// com serial A BIT
                                 packetIndex,
                                 ADC_GetChannelReadings(ADC_CHANNEL_1));

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// com serial b BIT
                                 packetIndex,
                                 ADC_GetChannelReadings(ADC_CHANNEL_2));

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// software BIT
                                 packetIndex,
                                 ADC_GetChannelReadings(ADC_CHANNEL_3));

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// software algorithm BIT
                                 packetIndex,
                                 BIT_GetAlgoStatusWord());

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// software data BIT
                                 packetIndex,
                                 ADC_GetChannelReadings(ADC_CHANNEL_4));

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// hardware status
                                 packetIndex,
                                 ADC_GetChannelReadings(ADC_CHANNEL_5));

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// com status
                                 packetIndex,
                                 ADC_GetChannelReadings(ADC_CHANNEL_6));

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// SENSOR status
                                 packetIndex,
                                 BIT_GetSensorStatusWord_MSB());

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// SENSOR status
                                 packetIndex,
                                 BIT_GetSensorStatusWord_LSB());

    funCount += packetIndex;

    HandleUcbTx(port, ptrUcbPacket); /// send Test 0 packet
}

/*******************************************
 * @brief 
 * 
 * @param port ==
 * @param ptrUcbPacket ==
********************************************/
static void UcbFactory1 (uint16_t const port, UcbPacketStruct* const ptrUcbPacket)
{
	uint16_t packetIndex = 0U;

	ptrUcbPacket->payloadLength = (uint8_t)UCB_FACTORY_1_LENGTH;
	packetIndex = appendInertialCounts(ptrUcbPacket->payload, packetIndex);
	packetIndex = appendAllTempCounts(ptrUcbPacket->payload, packetIndex);

    packetIndex = uint16ToBuffer(ptrUcbPacket->payload, /// BIT status
                                  packetIndex,
                                  BIT_GetMasterStatusWord());

    funCount += packetIndex;

    HandleUcbTx(port, ptrUcbPacket); /// send Factory 1 packet
}

/*******************************************
 * @brief 
 * 
 * @param port ==
 * @param ptrUcbPacket ==
********************************************/
static void UcbFactoryM (uint16_t const port, UcbPacketStruct* const ptrUcbPacket)
{
    ptrUcbPacket->packetType    = (uint8_t)UCB_FACTORY_M;
	ptrUcbPacket->payloadLength = (uint8_t)sens_FillRawPayload(ptrUcbPacket->payload);
	HandleUcbTx(port, ptrUcbPacket); /// send Factory 2 packet
}

/*******************************************
 * @brief 
 * 
 * @param port ==
 * @param ptrUcbPacket ==
********************************************/
void SendUcbPacket (uint16_t const port, UcbPacketStruct* const ptrUcbPacket)
{
        // FIXME could be a lookup table using a search
		switch ((int32_t)ptrUcbPacket->packetType) {
            case UCB_IDENTIFICATION:   // ID 0x4944
                UcbIdentification(port, ptrUcbPacket);
                break;
            case UCB_VERSION_DATA:     // VR 0x5652
                UcbVersionData(port, ptrUcbPacket);
                break;
            case UCB_VERSION_ALL_DATA: // VA 0x5641
                UcbVersionAllData(port, ptrUcbPacket);
                break;
            case UCB_ANGLE_1:          // A1 0x4131
                UcbAngle1(port, ptrUcbPacket);
                break;
            case UCB_ANGLE_2:          // A2 0x4132
                UcbAngle2(port, ptrUcbPacket);
                break;
            case UCB_ANGLE_3:          // A3 0x4133
                UcbAngle3(port, ptrUcbPacket);
                break;
            case UCB_SCALED_0:         // S0 0x5330
                UcbScaled0(port, ptrUcbPacket);
                break;
            case UCB_SCALED_1:         // S1 0x5331
                UcbScaled1(port, ptrUcbPacket);
                break;
            case UCB_SCALED_M:         // SM 0x534D
                UcbScaledM(port, ptrUcbPacket);
                break;
            case UCB_TEST_0:           // T0 0x5430
                UcbTest0(port, ptrUcbPacket);
                break;
            case UCB_FACTORY_1:        // F1 0x4631
                UcbFactory1(port, ptrUcbPacket);
                break;
            case UCB_FACTORY_M:        // F2 0x464D
                UcbFactoryM(port, ptrUcbPacket);
                break;
            case (uint8_t)UCB_USER_OUT:
                {
                    BOOL const result = HandleUserOutputPacket(ptrUcbPacket->payload, &ptrUcbPacket->payloadLength);
                    if(result != FALSE) {
                HandleUcbTx(port, ptrUcbPacket);   /// send user packet
                    }
                }
                break;
			default:
              break; /// default handler?
		}
}

/*******************************************
 * @brief 
 * 
 * @param port ==
 * @param ptrUcbPacket ==
********************************************/
void HandleUcbTx (uint16_t const port, UcbPacketStruct* const ptrUcbPacket)

{
    uint16_t crc;
    BOOL found = 0;

    ptrUcbPacket->sync_MSB = 0x55U;
    ptrUcbPacket->sync_LSB = 0x55U;

    if(ptrUcbPacket->packetType == (uint8_t)UCB_USER_OUT){
        uint8_t bytes[2] = {0U,0U};
        UserPacketTypeToBytes(bytes);
        ptrUcbPacket->code_MSB = bytes[0];
        ptrUcbPacket->code_LSB = bytes[1];
        found = 1;
    }else {
        ucbPacketTableEntry_t *packetEntry = &ucbPackets[0];

        while (packetEntry->ptype != UCB_PACKET_LAST)
        {
            if (packetEntry->ptype == (int32_t)ptrUcbPacket->packetType)
            {
                // put packet code in place
                ptrUcbPacket->code_MSB = (uint8_t)((packetEntry->psync >> 8U) & 0xFFU);
                ptrUcbPacket->code_LSB = (uint8_t)(packetEntry->psync & 0xFFU);
                found = 1;
                break;
            }
            packetEntry++;
        }
    }


    if(!found){
        ptrUcbPacket->payloadLength = 2U;
        ptrUcbPacket->payload[0]    =  ptrUcbPacket->code_MSB;
        ptrUcbPacket->payload[1]    =  ptrUcbPacket->code_LSB;
        ptrUcbPacket->code_MSB = 0x15U; // NAK
        ptrUcbPacket->code_LSB = 0x15U;
    }


    crc = CalculateCRC ((uint8_t *)&ptrUcbPacket->code_MSB, ptrUcbPacket->payloadLength + 3U);
    ptrUcbPacket->payload[ptrUcbPacket->payloadLength+1U]   = (uint8_t)((crc >> 8U) & 0xFFU);
    ptrUcbPacket->payload[ptrUcbPacket->payloadLength]      = (uint8_t)(crc  & 0xFFU);

    UART_Write(port, (uint8_t*)&ptrUcbPacket->sync_MSB, ptrUcbPacket->payloadLength + 7U);
}
/* end HandleUcbTx */

/*******************************************
 * @brief 
 * 
 * @param dacqRate ==
********************************************/
void SendContinuousPacket(int32_t const dacqRate)
{
    // here we come at 200 Hz
    static int32_t numCycles = 0;

    uint32_t  const div  = config_GetPacketRateDivider();
    if(div){
        static UcbPacketStruct contPacket;
        uint16_t   code = config_GetOutputPacketCode();
        contPacket.packetType = (uint8_t)UcbPacketBytesToPacketType((uint8_t *)&code, TRUE);
        numCycles++;
        if(numCycles >= (int32_t)div){
            numCycles = 0;
            if(contPacket.packetType != (uint8_t)UCB_ERROR_INVALID_TYPE){
                SendUcbPacket((uint16_t)ucbPort, &contPacket);
            }
         }
    }else{
        numCycles = 0;
    }

    funCount += (uint32_t)dacqRate;

}
