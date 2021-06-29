/** ***************************************************************************
 * @file parameters.c
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

//***************************
#include <math.h>
#include <stdint.h>
#include "parameters.h"
#include "Indices.h"
#include "sensorsAPI.h"
#include "configuration.h"

#ifdef USE_ALGORITHM
#include "ekfAPI.h"
#endif


/*******************************************
 * @brief 
 * 
 * @param value ==
 * @return float32_t 
********************************************/
static float32_t SCALE_BY_2POW16_OVER_2PI(float32_t const value)
{
	    return value * 10430.37835047045F;
}

/*******************************************
 * @brief 
 * 
 * @param value ==
 * @return float32_t 
********************************************/
static float32_t SCALE_BY_2POW16_OVER_7PI(float32_t const value)
{
    return value * 2980.108100134415F;
}

/*******************************************
 * @brief 
 * 
 * @param value ==
 * @return float32_t 
********************************************/
static float32_t SCALE_BY_2POW16_OVER_200(float32_t const value)
{
    return value * 327.68F;
}


/*******************************************
 * @brief 
 * 
 * @param packetRateDivider ==
 * @return BOOL 
********************************************/
BOOL CheckPacketRateDivider (uint16_t const packetRateDivider)
{
    switch ((int16_t)packetRateDivider) {
     case PACKET_RATE_DIV_QUIET:
     case PACKET_RATE_DIV_200HZ:
     case PACKET_RATE_DIV_100HZ:
     case PACKET_RATE_DIV_50HZ :
     case PACKET_RATE_DIV_25HZ :
     case PACKET_RATE_DIV_20HZ :
     case PACKET_RATE_DIV_10HZ :
     case PACKET_RATE_DIV_5HZ  :
     case PACKET_RATE_DIV_2HZ  :
     case PACKET_RATE_DIV_1HZ  :
        return TRUE;
    default:
        return FALSE;
    }
}
/* end CheckPacketRateDivider */

/*******************************************
 * @brief 
 * 
 * @param outputPacket ==
 * @param baudRate ==
 * @param packetRateDivider ==
 * @return BOOL 
********************************************/
BOOL CheckContPacketRate (UcbPacketType const outputPacket,
                          uint16_t      const baudRate,
                          uint16_t      const packetRateDivider)
{

    if (packetRateDivider != 0U) {
        uint16_t bytesPerPacket = 5U; // UCB_OVERHEAD_LENGTH crc + len +header

        switch (outputPacket) {
            case UCB_IDENTIFICATION:
                bytesPerPacket += (uint16_t)UCB_IDENTIFICATION_LENGTH;
                break;
            case UCB_TEST_0:
                bytesPerPacket += (uint16_t)UCB_TEST_0_LENGTH;
                break;
            case UCB_FACTORY_1:
                bytesPerPacket += (uint16_t)UCB_FACTORY_1_LENGTH;
                break;
            case UCB_FACTORY_2:
                bytesPerPacket += (uint16_t)UCB_FACTORY_2_LENGTH;
                break;
            case UCB_FACTORY_M:
                bytesPerPacket += (uint16_t)UCB_FACTORY_M_LENGTH;
                break;
            case UCB_ANGLE_1:
                bytesPerPacket += (uint16_t)UCB_ANGLE_1_LENGTH;
                break;
            case UCB_ANGLE_2:
                bytesPerPacket += (uint16_t)UCB_ANGLE_2_LENGTH;
                break;
            case UCB_ANGLE_3:
                bytesPerPacket += (uint16_t)UCB_ANGLE_3_LENGTH;
                break;
            case UCB_VERSION_DATA:
                bytesPerPacket += (uint16_t)UCB_VERSION_DATA_LENGTH;
                break;
            case UCB_VERSION_ALL_DATA:
                bytesPerPacket += (uint16_t)UCB_VERSION_ALL_DATA_LENGTH;
                break;
            case UCB_SCALED_0:
                bytesPerPacket += (uint16_t)UCB_SCALED_0_LENGTH;
                break;
            case UCB_SCALED_1:
                bytesPerPacket += (uint16_t)UCB_SCALED_1_LENGTH;
                break;
            case UCB_SCALED_M:
                bytesPerPacket += (uint16_t)UCB_SCALED_M_LENGTH;
                break;
            case UCB_SCALED_N:
                bytesPerPacket += (uint16_t)UCB_SCALED_N_LENGTH;
                break;
            default:
                break;
        }

        if((int32_t)outputPacket == UCB_USER_OUT){
            int32_t const pldLen  = GetUserPayloadLength();
            if(pldLen == 0){
                return FALSE;
            }
            bytesPerPacket += (uint16_t)(uint32_t)pldLen;
        }

        uint16_t bytesPerSecond;
        if(packetRateDivider == 200U){
            bytesPerSecond = bytesPerPacket * 200U;
        }else {
        bytesPerSecond = bytesPerPacket * (100U / packetRateDivider);
        }

        // For a message with 10 bits/byte (data, start, and stop-bits) and a
        //   safety-factor of 80%, determine if the baud-rate can support the
        //   message and output data rate (ODR)
        float32_t numm;
        real const dutyCycle = 0.85F;
        switch ((int16_t)baudRate) {
            case BAUD_CODE_9600:
                numm = 960.0F * dutyCycle;
                break;
            case BAUD_CODE_19200:
                numm = 1920.0F * dutyCycle;
                break;
            case BAUD_CODE_38400:
                numm = 3840.0F * dutyCycle;
                break;
            case BAUD_CODE_57600:
                numm = 5760.0F * dutyCycle;
                break;
            case BAUD_CODE_115200:
                numm = 11520.0F * dutyCycle;
                break;
            case BAUD_CODE_230400:
                numm = 23040.0F * dutyCycle;
                break;
            default:
                return FALSE;
        }

        int32_t const aa = (int32_t)bytesPerSecond;

        if((float32_t)aa < numm){
            return TRUE;
    }
    }


    return TRUE;
} /* end CheckContPacketRate */

/*******************************************
 * @brief 
 * 
 * @param portBaudRate ==
 * @return BOOL 
********************************************/
BOOL CheckPortBaudRate (uint16_t const portBaudRate)
{
    BOOL valid = TRUE;

    if (portBaudRate >= (uint16_t)NUM_BAUD_RATES) {
        valid = FALSE;
    }
    return valid;
}

/*******************************************
 * @brief 
 * 
 * @param orientation ==
 * @return BOOL 
********************************************/
BOOL CheckOrientation (uint16_t const orientation)
{
    switch ( (int16_t)orientation ) {
    case ORIENTATION_0:
    case ORIENTATION_9:
    case ORIENTATION_35:
    case ORIENTATION_42:
    case ORIENTATION_65:
    case ORIENTATION_72:
    case ORIENTATION_98:
    case ORIENTATION_107:
    case ORIENTATION_133:
    case ORIENTATION_140:
    case ORIENTATION_146:
    case ORIENTATION_155:
    case ORIENTATION_196:
    case ORIENTATION_205:
    case ORIENTATION_211:
    case ORIENTATION_218:
    case ORIENTATION_273:
    case ORIENTATION_280:
    case ORIENTATION_292:
    case ORIENTATION_301:
    case ORIENTATION_336:
    case ORIENTATION_345:
    case ORIENTATION_357:
    case ORIENTATION_364:
        return TRUE;
    default:
        return FALSE;
    }
}  /*end CheckOrientation */


#ifdef USE_ALGORITHM
/*******************************************
 * @brief 
 * 
 * @param response ==
 * @param index ==
 * @return uint16_t 
********************************************/
uint16_t appendCorrectedRates (uint8_t  response[],
                               uint16_t index)
{
    int16_t tmp;

    real corrRates[3];
    EKF_GetCorrectedRates_B(corrRates);


    /// i = 0
    tmp = (int16_t)(int32_t)(float32_t)SCALE_BY_2POW16_OVER_7PI(corrRates[X_AXIS]);
    index = uint16ToBuffer(response,
                           index,
                           (uint16_t)tmp);
    /// i = 1
    tmp = (int16_t)(int32_t)(float32_t) SCALE_BY_2POW16_OVER_7PI(corrRates[Y_AXIS]);
    index = uint16ToBuffer(response,
                           index,
                           (uint16_t)tmp);
    /// i = 2
    tmp = (int16_t)(int32_t)(float32_t) SCALE_BY_2POW16_OVER_7PI(corrRates[Z_AXIS]);
    index = uint16ToBuffer(response,
                           index,
                           (uint16_t)tmp);
    return index;
}
/* end appendCorrectedRates */
#endif // USE_ALGORITHM

/*******************************************
 * @brief 
 * 
 * @param response ==
 * @param index ==
 * @return uint16_t 
********************************************/
uint16_t appendRates (uint8_t  response[],
                      uint16_t index)
{
    float32_t ftmp;
    int32_t tmp;
    float64_t rates[3];

    sens_GetRateData_degPerSec(rates);

    /// X-Axis
    ftmp  = (float32_t)((rates[0]*32768.0)/630.0);
    tmp   = (int32_t)ftmp;
    index = uint16ToBuffer(response, index, (uint16_t)tmp);
    /// Y-Axis
    ftmp  = (float32_t)((rates[1]*32768.0)/630.0);
    tmp   = (int32_t)ftmp;
    index = uint16ToBuffer(response, index, (uint16_t)tmp);
    /// Z-Axis
    ftmp  = (float32_t)((rates[2]*32768.0)/630.0);
    tmp   = (int32_t)ftmp;
    index = uint16ToBuffer(response, index, (uint16_t)tmp);

    return index;
} /* end appendRates */

/*******************************************
 * @brief 
 * 
 * @param response ==
 * @param index ==
 * @param chipId ==
 * @return uint16_t 
********************************************/
uint16_t appendChipRates (uint8_t response[],
                      uint16_t index, int32_t const chipId)
{
    float32_t ftmp;
    int32_t tmp;
    float32_t rates[3];

    sens_GetChipRateData_degPerSec(chipId, rates);
    /// X-Axis
    ftmp  = (rates[0]*32768.0F)/630.0F;
    tmp   = (int32_t)ftmp;
    index = uint16ToBuffer(response, index, (uint16_t)tmp);
    /// Y-Axis
    ftmp  = (rates[1]*32768.0F)/630.0F;
    tmp   = (int32_t)ftmp;
    index = uint16ToBuffer(response, index, (uint16_t)tmp);
    /// Z-Axis
    ftmp  = (rates[2]*32768.0F)/630.0F;
    tmp   = (int32_t)ftmp;
    index = uint16ToBuffer(response, index, (uint16_t)tmp);

    return index;
} /* end appendRates */


/*******************************************
 * @brief 
 * 
 * @param response ==
 * @param index ==
 * @return uint16_t 
********************************************/
uint16_t appendMagReadings( uint8_t  response[],
                            uint16_t index )
{
    uint16_t const tmp = 0U;

    /// Split the 16-bit integer into two 8-bit numbers and place it into the buffer
    index = uint16ToBuffer(response, index, tmp);
    index = uint16ToBuffer(response, index, tmp);
    index = uint16ToBuffer(response, index, tmp);

    return index;
}

/* end appendMagReadings */

/*******************************************
 * @brief 
 * 
 * @param response ==
 * @param index ==
 * @return uint16_t 
********************************************/
uint16_t appendAccels (uint8_t  response[],
                       uint16_t index)
{
    float32_t   ftmp;
    int32_t     tmp;
    float64_t   accels[3];

    sens_GetAccelData_g(accels);

    /// X-Axis
    ftmp  = (float32_t)(accels[XACCEL]*3276.8);
    tmp   = (int32_t)ftmp;
    index = uint16ToBuffer(response, index, (uint16_t)tmp);
    /// Y-Axis
    ftmp  = (float32_t)(accels[YACCEL]*3276.8);
    tmp   = (int32_t)ftmp;
    index = uint16ToBuffer(response, index, (uint16_t)tmp);
    /// Z-Axis
    ftmp  = (float32_t)(accels[ZACCEL]*3276.8);
    tmp   = (int32_t)ftmp;
    index = uint16ToBuffer(response, index, (uint16_t)tmp);
    return index;
} /* end appendAccels */

/*******************************************
 * @brief 
 * 
 * @param response ==
 * @param index ==
 * @param chipId ==
 * @return uint16_t 
********************************************/
uint16_t appendChipAccels (uint8_t response[],
                          uint16_t index, int32_t const chipId)
{
    float32_t    tmp;
    float32_t    accels[3];
    int16_t      itmp;
    sens_GetChipAccelData_g(chipId, accels);

    /// X-Axis
    tmp   = accels[XACCEL]*3276.8F;
    itmp   = (int16_t)tmp;
    index  = uint16ToBuffer(response, index, (uint16_t)itmp);
    /// Y-Axis
    tmp   = accels[YACCEL]*3276.8F;
    itmp   = (int16_t)tmp;
    index = uint16ToBuffer(response, index, (uint16_t)itmp);
    /// Z-Axis
    tmp   = accels[ZACCEL]*3276.8F;
    itmp   = (int16_t)tmp;
    index = uint16ToBuffer(response, index, (uint16_t)itmp);
    return index;
} /* end appendAccels */


/*******************************************
 * @brief 
 * 
 * @param response ==
 * @param index ==
 * @return uint16_t 
********************************************/
uint16_t appendRateTemp (uint8_t response[],
                         uint16_t index)
{
    int16_t tmp;
    float32_t   tmpF;
    float32_t   const MAX_TEMP_4_SENSOR_PACKET = 99.9F;

    tmpF = sens_GetUnitTemp();


    if (tmpF >= MAX_TEMP_4_SENSOR_PACKET) {
        tmpF =  MAX_TEMP_4_SENSOR_PACKET;
    }
    if (tmpF <= -MAX_TEMP_4_SENSOR_PACKET) {
        tmpF =  -MAX_TEMP_4_SENSOR_PACKET;
    }

    tmp   = (int16_t)(int32_t)(SCALE_BY_2POW16_OVER_200(tmpF));

    index = uint16ToBuffer(response,
                           index,
                           (uint16_t)tmp);
    return index;

} /*end appendTemps */


/*******************************************
 * @brief 
 * 
 * @param response ==
 * @param index ==
 * @param chipId ==
 * @return uint16_t 
********************************************/
uint16_t appendChipTemps (uint8_t response[],
                         uint16_t index, int32_t const chipId)
{
    int16_t  tmp;
    float32_t    ftmp;

    ftmp   = sens_GetChipTemp(chipId);
    ftmp   *= 327.68F;
    tmp    = (int16_t)ftmp;
    index  = uint16ToBuffer(response, index, (uint16_t)tmp);

    return index;

} /*end appendTemps */

/*******************************************
 * @brief 
 * 
 * @param response ==
 * @param index ==
 * @return uint16_t 
********************************************/
uint16_t appendTemps (uint8_t response[], uint16_t index)
{
    int16_t tmp;
    float32_t   ftmp;

    // Convert to scaled output T { degC ] * ( 2^16/200 )
    ftmp    = sens_GetUnitTemp();
    ftmp   *= 327.68F;
    tmp    = (int16_t)ftmp;

    for(int32_t i = 0; i < 4; i++){
        index = uint16ToBuffer(response, index, (uint16_t)tmp);
    }

    return index;
} /*end appendTemps */

/*******************************************
 * @brief 
 * 
 * @param response ==
 * @param index ==
 * @return uint16_t 
********************************************/
uint16_t appendTemp (uint8_t response[], uint16_t index)
{
    int16_t tmp;
    float32_t   ftmp;

    // Convert to scaled output T { degC ] * ( 2^16/200 )
    ftmp  = sens_GetUnitTemp();
    ftmp  *= 327.68F;
    tmp   = (int16_t)ftmp;
    index = uint16ToBuffer(response, index, (uint16_t)tmp);

    return index;
} /*end appendTemps */


#ifdef USE_ALGORITHM

/*******************************************
 * @brief 
 * 
 * @param response ==
 * @param index ==
 * @return uint16_t 
********************************************/
uint16_t appendAttitudeTrue (uint8_t  response[],
                             uint16_t index)
{
    int16_t tmp;

    real  eulerAngles[3];
    EKF_GetAttitude_EA_RAD(eulerAngles);


    // X-Axis (angle in radians
    tmp = (int16_t) SCALE_BY_2POW16_OVER_2PI(eulerAngles[ROLL]);
    index = uint16ToBuffer(response,
                           index,
                           (uint16_t)tmp);
    /// Y-Axis
    tmp = (int16_t) SCALE_BY_2POW16_OVER_2PI(eulerAngles[PITCH]);
    index = uint16ToBuffer(response,
                           index,
                           (uint16_t)tmp);
    /// Z-Axis
    tmp = (int16_t) SCALE_BY_2POW16_OVER_2PI(eulerAngles[YAW]);
    index = uint16ToBuffer(response,
                           index,
                           (uint16_t)tmp);
    return index;
}
#endif // USE_ALGORITHM

/* end appendAttitudeTrue */

/*******************************************
 * @brief 
 * 
 * @param response ==
 * @param index ==
 * @return uint16_t 
********************************************/
uint16_t appendInertialCounts (uint8_t  response[],
                               uint16_t index)
{
    static    int32_t   rawData[N_RAW_SENS];

    sens_GetRawData(rawData);

    /// X-Axis Accelerometer
    index = uint32ToBuffer(response,
                           index,
                           (uint32_t)rawData[XACCEL]);
    /// Y-Axis Accelerometer
    index = uint32ToBuffer(response,
                           index,
                           (uint32_t)rawData[YACCEL]);

    /// Z-Axis Accelerometer
    index = uint32ToBuffer(response,
                           index,
                           (uint32_t)rawData[ZACCEL]);

    /// X-Axis Rate-Sensor
    index = uint32ToBuffer(response,
                           index,
                           (uint32_t)rawData[XRATE]);

    /// Y-Axis Rate-Sensor
    index = uint32ToBuffer(response,
                           index,
                           (uint32_t)rawData[YRATE]);

    /// Z-Axis Rate-Sensor
    index = uint32ToBuffer(response,
                           index,
                           (uint32_t)rawData[ZRATE]);

    return index;
}

/* end appendInertialCounts */

/*******************************************
 * @brief 
 * 
 * @param response ==
 * @param index ==
 * @return uint16_t 
********************************************/
uint16_t appendAllTempCounts (uint8_t  response[],
                              uint16_t index)
{
    uint32_t temp;

    /// @brief
    /// The older device had a temperature sensor on each of the sensors (prior
    /// to tri-axial device). The DMU380 only has a temperature sensor on the
    /// rate sensor and the boad.  To preserve the packet definition used
    /// previously (and to make the packet work with NavView), the first element
    /// of the group of temperature sensors has data while the rest are zero.

    /// accelerometer temperature sensors
    temp  = (uint32_t)sens_GetRawTempCounts();

   /// X-Axis accelerometer temperature
    index = uint32ToBuffer(response,
                           index,
                           temp);

    /// Y-Axis accelerometer temperature (same as x-axis)
    index = uint32ToBuffer(response,
                           index,
                           temp);

    /// Z-Axis accelerometer temperature (same as x-axis)
    index = uint32ToBuffer(response,
                           index,
                           temp);

    /// X-Axis rate-sensor temperature
    index = uint32ToBuffer(response,
                           index,
                           temp);

    /// Y-Axis rate-sensor temperature (same as x-axis)
    index = uint32ToBuffer(response,
                           index,
                           temp);

    /// Z-Axis rate-sensor temperature (same as x-axis)
    index = uint32ToBuffer(response,
                           index,
                           temp);

    /// The last element of the packet contains the board temperature.
    index = uint32ToBuffer(response,
                           index,
                           temp);
    return index;
} /* end appendAllTempCounts */

/*******************************************
 * @brief 
 * 
 * @param buffer ==
 * @param index ==
 * @param inWord ==
 * @return uint16_t 
********************************************/
uint16_t uint32ToBuffer(uint8_t buffer[],
               uint16_t index,
               uint32_t const inWord)
{
    buffer[index] = (uint8_t)((inWord >> 24U) & 0xFFU);
    index++;
    buffer[index] = (uint8_t)((inWord >> 16U) & 0xFFU);
    index++;
    buffer[index] = (uint8_t)((inWord >>  8U) & 0xFFU);
    index++;
    buffer[index] = (uint8_t)(inWord & 0xFFU);
    index++;

  return index;
}

/*******************************************
 * @brief 
 * 
 * @param buffer ==
 * @param index ==
 * @param inWord ==
 * @return uint16_t 
********************************************/
uint16_t uint16ToBuffer(uint8_t  buffer[],
               uint16_t index,
               uint16_t const inWord)
{
    buffer[index] = (uint8_t)((inWord >> 8U) & 0xFFU);
    index++;
    buffer[index] = (uint8_t)(inWord & 0xFFU);
    index++;

  return index;
}
