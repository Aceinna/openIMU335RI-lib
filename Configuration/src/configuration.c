/** ***************************************************************************
 * @file configuration.c
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
#include <string.h>
#include <stdint.h>

//*****************************
#include "eepromAPI.h"
#include "configurationAPI.h"
#include "configuration.h"
#include "parameters.h"
#include "Indices.h"
#include "FaultDetection.h"
#include "userAPI.h"
#include "EcuSettings.h"
#include "appVersion.h"
#include "partNumber.h"
#include "filterAPI.h"

static FactoryConfigurationStruct gConfiguration;
/// proposed configurations
static FactoryConfigurationStruct proposedRamConfiguration;
static FactoryConfigurationStruct proposedEepromConfiguration;
static BOOL fWrite = FALSE;
// placholders for Nav_view compatibility
static uint8_t bootVersion[5] = {0,0,0,0,0};
static BOOL disableChipFilter = FALSE;
static int32_t uartBaudCode   = 0;
/*******************************************
 * @brief 
 * 
 * @return BOOL 
********************************************/
BOOL config_IsChipFilterDisabled()
{
    return disableChipFilter;
}

/*******************************************
 * @brief 
 * 
 * @param version ==
********************************************/
void config_GetBootloaderVersion(uint8_t version[])
{
    for(int i = 0; i < 5; i++){
        version[i] = bootVersion[i];
    } 
}

/** ****************************************************************************
 * @name ValidPortConfiguration
 * @brief Check output packet configuration members for sanity
 * @param proposedConfiguration [in] - configuratione
 * @return 	boolean, TRUE if the packet output is possible, FALSE if not
 ******************************************************************************/
static BOOL ValidPortConfiguration (FactoryConfigurationStruct *proposedConfiguration)
{
    /// working packet type byte buffer
    UcbPacketType continuousPacketType;
    uint8_t     valid = 1U;

    /// check packet rate divider
    valid &= (uint8_t)(CheckPacketRateDivider(proposedConfiguration->packetRateDivider));


    continuousPacketType = UcbPacketBytesToPacketType((uint8_t *)&proposedConfiguration->packetCode, TRUE);

    /// check that a valid continuous output packet has been selected
    valid &= (uint8_t)UcbPacketIsAnOutputPacket(continuousPacketType);

    /// check continuous packet rate
    valid &= (uint8_t)CheckContPacketRate( continuousPacketType,
                                  proposedConfiguration->baudRateUser,
                                  proposedConfiguration->packetRateDivider );
    /// check port baud rates
    valid &= (uint8_t)CheckPortBaudRate(proposedConfiguration->baudRateUser);

    return (BOOL)valid;
}
/* end ValidPortConfiguration */

/******************************************************************************
 * @brief
 * @return used chips
 ******************************************************************************/
uint16_t config_GetUsedChips(void)
{
    return (gConfiguration.usedChips & gConfiguration.activeChips);
}

/******************************************************************************
 * @brief
 * @return active chips
 ******************************************************************************/
uint16_t config_GetActiveChips(void)
{
    return gConfiguration.activeChips;
}

/******************************************************************************
 * @brief
 * @param chipIdx  [in]
 * @return used chips
 ******************************************************************************/
uint16_t config_GetUsedSensors(int32_t const chipIdx)
{
    return gConfiguration.usedSensors[chipIdx];
}

/******************************************************************************
 * @brief readConfigIntoMem LOCAL Read configuration from EEPROM into RAM
 *  
 ******************************************************************************/
static void getBootloaderVersion()
{
    EEPROM_GetBootVersion(bootVersion);
}

/** ****************************************************************************
 * @name CheckFieldData
 * @brief checks if field data has valid values.
 * @param currentConfiguration [in]  - current configuration to modify
 * @param numFields [in]  - number of fields to check
 * @param fieldId [in]  - number of fields to check
 * @param fieldData [in]  - number of fields to check
 * @param validFields [out] [] - array of fieldId that have been validated.
 * @return boolean
 ******************************************************************************/
static uint8_t config_CheckFieldData (FactoryConfigurationStruct* const currentConfiguration,
                               uint8_t    const    numFields,
                               uint16_t            fieldId [],
                               uint16_t            fieldData [],
                               uint16_t            validFields [])
{
    BOOL                packetTypeChanged        = FALSE;
    BOOL                packetRateDividerChanged = FALSE;
    BOOL                userBaudChanged          = FALSE;
    BOOL                success                  = FALSE;
    /// index for stepping through proposed configuration fields
    uint8_t             fieldIndex      = 0U;
    uint8_t             validFieldIndex = 0U; ///< index for building valid return array
    uint8_t             type [UCB_PACKET_TYPE_LENGTH]; ///< working packet type byte buffer
    UcbPacketType       continuousPacketType;
    FactoryConfigurationStruct proposedPortConfig;

    /// copy current configuration - for testing validity of port configuration only
    proposedPortConfig = *currentConfiguration;

    /// update new field settings in proposed configuration */
    for (fieldIndex = 0U; fieldIndex < numFields; ++fieldIndex) {
        if ((fieldId[fieldIndex] >= (uint16_t)LOWER_CONFIG_ADDR_BOUND) &&
            (fieldId[fieldIndex] <= (uint16_t)UPPER_CONFIG_ADDR_BOUND)) {
            /// parse field ID and, if applicable, check using respective
            /// function (not all fields require this)
            switch ((int16_t)fieldId[fieldIndex]) {
                case PACKET_TYPE_FIELD_ID:
                    /// get enum for requested continuous packet type
                    type[1] = (uint8_t)((fieldData[fieldIndex] >> 8U) & 0xFFU);
                    type[0] = (uint8_t)(fieldData[fieldIndex] & 0xFFU);

                    continuousPacketType = UcbPacketBytesToPacketType(type, FALSE);

                    /// check that a valid continuous output packet has been selected
                    if (UcbPacketIsAnOutputPacket(continuousPacketType) == TRUE) {
                        packetTypeChanged             = TRUE;
                        proposedPortConfig.packetCode = fieldData[fieldIndex];
                    }
                    break;
                case PACKET_RATE_DIVIDER_FIELD_ID:
                    packetRateDividerChanged             = TRUE;
                    proposedPortConfig.packetRateDivider = fieldData[fieldIndex];
                    break;
                case BAUD_RATE_USER_ID:
                    userBaudChanged = TRUE;
                    uartBaudCode    = fieldData[fieldIndex];
                    proposedPortConfig.baudRateUser = fieldData[fieldIndex];
                    break;
                case ORIENTATION_FIELD_ID:
                    if (CheckOrientation(fieldData[fieldIndex]) == TRUE) {
                        /// update proposed configuration
                        currentConfiguration->orientation = fieldData[fieldIndex];
                        /// add to valid list
                        validFields[validFieldIndex]        = fieldId[fieldIndex];
                        validFieldIndex++;
                        // Set the flags to RESTART the algorithm
                        InitUserAlgorithm();
                        if(fWrite){
                            SetEcuOrientation(fieldData[fieldIndex]);
                        }    
                    }
                    break;
                case ECU_ADDRESS_FIELD_ID:
                        if(fWrite){
                            SetEcuAddress(fieldData[fieldIndex]);
                        }
                        ((uint16_t *)currentConfiguration)[(fieldId[fieldIndex])] = fieldData[fieldIndex];
                        validFields[validFieldIndex] = fieldId[fieldIndex];
                        validFieldIndex++;
                        break;
                case ECU_BAUD_RATE_FIELD_ID:
                        if(fWrite){
                            SetEcuBaudrate(fieldData[fieldIndex]);
                        }
                        ((uint16_t *)currentConfiguration)[(fieldId[fieldIndex])] = fieldData[fieldIndex];
                        validFields[validFieldIndex] = fieldId[fieldIndex];
                        validFieldIndex++;
                        break;
                case ECU_BEHAVIOR_FIELD_ID:
                        if(fWrite){
                            SetEcuBehavior(fieldData[fieldIndex]);
                        }    
                        ((uint16_t *)currentConfiguration)[(fieldId[fieldIndex])] = fieldData[fieldIndex];
                        validFields[validFieldIndex] = fieldId[fieldIndex];
                        validFieldIndex++;
                        break;
                case ECU_ODR_FIELD_ID:
                        if(fWrite){
                            success = SetEcuPacketRate(fieldData[fieldIndex]);
                            if(success){
                                ((uint16_t *)currentConfiguration)[(fieldId[fieldIndex])] = fieldData[fieldIndex];
                                validFields[validFieldIndex] = fieldId[fieldIndex];
                                validFieldIndex++;
                            }
                        }    
                        break;
                case ACCEL_LPF_FIELD_ID:
                    if(fWrite){
                        int32_t freq = config_GetFilterFreq(DUMMY_SENSOR, fieldData[fieldIndex]);
                        SetEcuAccelFilter((uint16_t)freq);
                    }
                    ((uint16_t *)currentConfiguration)[(fieldId[fieldIndex])] = fieldData[fieldIndex];
                    validFields[validFieldIndex] = fieldId[fieldIndex];
                    validFieldIndex++;
                    break;
                case RATE_LPF_FIELD_ID:
                    if(fWrite){
                        int32_t freq = config_GetFilterFreq(DUMMY_SENSOR, fieldData[fieldIndex]);
                        SetEcuRateFilter((uint16_t)freq);
                    }
                    ((uint16_t *)currentConfiguration)[(fieldId[fieldIndex])] = fieldData[fieldIndex];
                    validFields[validFieldIndex] = fieldId[fieldIndex];
                    validFieldIndex++;
                    break;
                case ECU_PACKET_TYPE_FIELD_ID:
                    if(fWrite){
                        SetEcuPacketType(fieldData[fieldIndex]);
                        }    
                        ((uint16_t *)currentConfiguration)[(fieldId[fieldIndex])] = fieldData[fieldIndex];
                        validFields[validFieldIndex] = fieldId[fieldIndex];
                        validFieldIndex++;
                        break;
                default:
                    ((uint16_t *)currentConfiguration)[(fieldId[fieldIndex])] = fieldData[fieldIndex];
                    validFields[validFieldIndex] = fieldId[fieldIndex];
                    validFieldIndex++;
                    break;
            }
        }
    }

    /// check proposed port configuration field settings (order/priority matters!)
    if (userBaudChanged == TRUE) {
        if (ValidPortConfiguration(&proposedPortConfig) == TRUE) {

            /// add configuration changes to proposed configuration and add all
            /// relevant fields to valid list
            if (packetTypeChanged == TRUE) {
                currentConfiguration->packetCode = proposedPortConfig.packetCode;
                validFields[validFieldIndex]   = (uint16_t)PACKET_TYPE_FIELD_ID;
                validFieldIndex++;
            }

            if (packetRateDividerChanged == TRUE) {
                currentConfiguration->packetRateDivider = proposedPortConfig.packetRateDivider;
                validFields[validFieldIndex]          = (uint16_t)PACKET_RATE_DIVIDER_FIELD_ID;
                validFieldIndex++;
            }

            if (userBaudChanged == TRUE) {
                currentConfiguration->baudRateUser = proposedPortConfig.baudRateUser;
                validFields[validFieldIndex]     = (uint16_t)BAUD_RATE_USER_ID;
                validFieldIndex++;
                if(fWrite){
                    SetUserUartBaudRate(baudEnumToBaudRate(uartBaudCode));
                }
            }
        }
    }
    if ((packetTypeChanged == TRUE) ||  (packetRateDividerChanged == TRUE)) {
        /// port usage or baud settings haven't changed, DON'T indicate port
        /// configuration change
        proposedPortConfig.baudRateUser = currentConfiguration->baudRateUser;

        if (ValidPortConfiguration(&proposedPortConfig) == TRUE) {
            if (packetTypeChanged == TRUE) {
                currentConfiguration->packetCode = proposedPortConfig.packetCode;
                validFields[validFieldIndex]   = (uint16_t)PACKET_TYPE_FIELD_ID;
                validFieldIndex++;
            }

            if (packetRateDividerChanged == TRUE) {
                currentConfiguration->packetRateDivider = proposedPortConfig.packetRateDivider;
                validFields[validFieldIndex]          = (uint16_t)PACKET_RATE_DIVIDER_FIELD_ID;
                validFieldIndex++;
            }
        }
    }
    return validFieldIndex;
} /* end CheckFieldData */

/*******************************************
 * @brief 
 * 
 * @param numFields ==
 * @param fieldId ==
 * @param fieldData ==
 * @param validFields ==
 * @return uint8_t 
********************************************/
uint8_t config_CheckEepromFieldData (uint8_t  const numFields,
                              uint16_t fieldId [],
                              uint16_t fieldData [],
                              uint16_t validFields [])
{   /// copy current EEPROM configuration
    EEPROM_ReadFactoryConfiguration(&proposedEepromConfiguration);

    fWrite = TRUE;
    return config_CheckFieldData(&proposedEepromConfiguration,
                          numFields,
                          fieldId,
                          fieldData,
                          validFields);
}


/** ****************************************************************************
 * @name SetFieldData
 * @brief Perform config changes required by the "SF" command
 *  
 ******************************************************************************/
void config_SetFieldData (void)
{
    gConfiguration = proposedRamConfiguration;
} /* end SetFieldData */

/*******************************************
 * @brief 
 * 
 * @param numFields ==
 * @param fieldId ==
 * @param fieldData ==
 * @param validFields ==
 * @return uint8_t 
********************************************/
uint8_t config_CheckRamFieldData (uint8_t  const numFields,
                           uint16_t fieldId [],
                           uint16_t fieldData [],
                           uint16_t validFields [])
{
    proposedRamConfiguration = gConfiguration; /// current RAM configuration

    return config_CheckFieldData(&proposedRamConfiguration,
                          numFields,
                          fieldId,
                          fieldData,
                          validFields);
}

/** ****************************************************************************
 * @name WriteFieldData
 * @brief write the data from the WF command into eeprom
 * @return status TRUE, FALSE
 ******************************************************************************/
BOOL config_WriteFieldData (void)
{
    int32_t const  NUM_CONFIG_FIELDS = (int32_t)UPPER_CONFIG_ADDR_BOUND - (int32_t)LOWER_CONFIG_ADDR_BOUND - 2;

    BOOL     success;
    // ConfigurationStruct xbowsp_generaldrivers.h
    uint16_t *ptr = (uint16_t*) &proposedEepromConfiguration;

    ptr = ptr + 1; ///< get past CRC at top

    /// write entire proposed configuration back to EEPROM
    if (EEPROM_WriteByte((uint16_t)LOWER_CONFIG_ADDR_BOUND, 
                        (uint16_t)NUM_CONFIG_FIELDS * 2U,
                        (void *)ptr) == 0) {
        success = TRUE;
    } else {
        success = FALSE;
    }

    return success;
} /* end WriteFieldData */


/** ****************************************************************************
 * @brief Set initial ports:
 *  
 ******************************************************************************/
void config_ApplyDefaultSerialPortSettings (void)
{
  gConfiguration.packetRateDivider = (uint16_t)DEFAULT_PACKET_RATE_DIVIDER;
  gConfiguration.packetCode        = (uint16_t)DEFAULT_PACKET_CODE;
  gConfiguration.baudRateUser      = (uint16_t)DEFAULT_BAUD_RATE;
} /* end DefaultPortConfiguration */


/*******************************************
 * @brief 
 * 
 * @param baudEnum ==
 * @return int32_t 
********************************************/
int32_t baudEnumToBaudRate(int32_t const baudEnum)
{   // -1 (invalid) can be sent in. Passed through to gGpsDataPtr->GPSConfigureOK
    int32_t baudRate = baudEnum;

    switch (baudEnum) {
        case BAUD_CODE_9600: // 0
            baudRate =    9600;
            break;
        case BAUD_CODE_19200: // 1
            baudRate =   19200;
            break;
        case BAUD_CODE_38400: // 2
            baudRate =	 38400;
            break;
        case BAUD_CODE_4800:  // 4
            baudRate =	  4800;
            break;
        case BAUD_CODE_115200: // 5
            baudRate =	115200;
            break;
        case BAUD_CODE_230400: // 6
            baudRate =  230400;
            break;
        case BAUD_CODE_57600:  // 3
        default:
            baudRate =	 57600;
            break;
    }

    return baudRate;
}


/******************************************************************************
 * @brief readConfigIntoMem LOCAL Read configuration from EEPROM into RAM
 *  
 ******************************************************************************/
static void readConfigIntoMem ()
{
    EEPROM_ReadFactoryConfiguration(&gConfiguration); // s_eeprom.c
}

/** ****************************************************************************
 * @brief initConfigureUnit initializes the data structures and configurations
 *  
 ******************************************************************************/
void ApplyFactoryConfiguration(void)
{
    uint16_t const chipsMask = (1U << (uint16_t)NUM_SENSOR_CHIPS) - 1U;

    readConfigIntoMem();
    getBootloaderVersion();

    if(gConfiguration.accelSamplingCountsThreshold == 0xFFFFU){
        gConfiguration.accelSamplingCountsThreshold = 1000U;
    }

    if(gConfiguration.gyroSamplingCountsThreshold == 0xFFFFU){
        gConfiguration.gyroSamplingCountsThreshold = 1000U;
    }


    /// check user orientation field for validity and set defaults based on com
    //  type if not valid xbow_fields.c
    if (CheckOrientation(gConfiguration.orientation) == FALSE) {
        gConfiguration.orientation = 0U;
    }

    /// check port configuration fields against rules
	// xbow_fields.c
    if (ValidPortConfiguration(&gConfiguration) == FALSE) {
        config_ApplyDefaultSerialPortSettings();
    }


    if(gConfiguration.usedChips > chipsMask){
        gConfiguration.usedChips = chipsMask;
    }

    if(gConfiguration.activeChips > chipsMask){
        gConfiguration.activeChips = chipsMask;
    }

    if((gConfiguration.accelSignalConsistencyPeriod == -1) ||
       (gConfiguration.accelSignalConsistencyPeriod < ACCEL_CONSISTENCY_DETECT_PERIOD_MIN)){
        gConfiguration.accelSignalConsistencyPeriod = ACCEL_CONSISTENCY_DETECT_PERIOD_DEFAULT;
    };

    if((gConfiguration.ratesSignalConsistencyPeriod == -1) ||
       (gConfiguration.ratesSignalConsistencyPeriod < RATE_CONSISTENCY_DETECT_PERIOD_MIN)){
        gConfiguration.ratesSignalConsistencyPeriod = RATE_CONSISTENCY_DETECT_PERIOD_DEFAULT;
    };

//********************************************************

    if((gConfiguration.accelSignalConsistencyThreshold == -1) ||
      (gConfiguration.accelSignalConsistencyThreshold < ACCEL_CONSISTENCY_DETECT_THRESHOLD_MIN)){
        gConfiguration.accelSignalConsistencyThreshold = ACCEL_CONSISTENCY_DETECT_THRESHOLD_DEFAULT;
    };

    if( (gConfiguration.ratesSignalConsistencyThreshold == -1 )||
       (gConfiguration.ratesSignalConsistencyThreshold < RATE_CONSISTENCY_DETECT_THRESHOLD_MIN)){
        gConfiguration.ratesSignalConsistencyThreshold = RATE_CONSISTENCY_DETECT_THRESHOLD_DEFAULT;
    };

//********************************************************

    if(gConfiguration.accelConsistencyCheckEnable > 1U){
        gConfiguration.accelConsistencyCheckEnable = 1U;
    };

    if(gConfiguration.ratesConsistencyCheckEnable > 1U){
        gConfiguration.ratesConsistencyCheckEnable = 1U;
    };


}


/*******************************************
 * @brief
 * 
 * @param orientation ==
 * @param fApply ==
 * @return BOOL 
********************************************/
BOOL config_ApplyOrientation(uint16_t const orientation, BOOL const fApply)
{
    if(CheckOrientation(orientation)){
        if(fApply){
        gConfiguration.orientation = orientation;
        }
        return TRUE;
    }
    return FALSE;
}

/*******************************************
 * @brief 
 * 
 * @param code ==
 * @param fApply ==
 * @return BOOL 
********************************************/
BOOL  config_SetOutputPacketCode(uint16_t const code, BOOL const fApply)
{
    uint16_t const tmp =  gConfiguration.packetCode;
    BOOL res;

    gConfiguration.packetCode = code;
    res = ValidPortConfiguration(&gConfiguration);

    if ((res == FALSE) || (!fApply)) {
        gConfiguration.packetCode =  tmp;
    }

    return res;
}

/*******************************************
 * @brief 
 * 
 * @param baudRate ==
 * @param fApply ==
 * @return BOOL 
********************************************/
BOOL config_SetBaudRate(int32_t const baudRate, BOOL const fApply)
{
    BOOL res;
    int16_t code;

    switch (baudRate){
        case BAUD_RATE_9600:	    
            code = BAUD_CODE_9600;  
            break;
        case BAUD_RATE_19200:       
            code = BAUD_CODE_19200;  
            break;
        case BAUD_RATE_38400:       
            code = BAUD_CODE_38400;  
            break;
        case BAUD_RATE_57600:       
            code = BAUD_CODE_57600;  
            break;
        case BAUD_RATE_4800:	    
            code = BAUD_CODE_4800;   
            break;
        case BAUD_RATE_115200:      
            code = BAUD_CODE_115200; 
            break;
        case BAUD_RATE_230400:      
            code = BAUD_CODE_230400; 
            break;
        default:
            return FALSE;
    }

    uint16_t const tmp = gConfiguration.baudRateUser;
    gConfiguration.baudRateUser =  (uint16_t)code;
    res = ValidPortConfiguration(&gConfiguration);

    if ((res == FALSE) || (!fApply)) {
        gConfiguration.baudRateUser = tmp;
    }

    return res;

}

/*******************************************
 * @brief 
 * 
 * @param rate ==
 * @param fApply ==
 * @return BOOL 
********************************************/
BOOL config_SetPacketRate(int32_t const rate, BOOL const fApply)
{
    BOOL res;
    int16_t divider;

    switch(rate){
        case PACKET_RATE_200HZ:
            divider = PACKET_RATE_DIV_200HZ;
            break;
        case PACKET_RATE_100HZ:
            divider = PACKET_RATE_DIV_100HZ;
            break;
        case  PACKET_RATE_50HZ:
            divider = PACKET_RATE_DIV_50HZ;
            break;
        case PACKET_RATE_25HZ:
            divider = PACKET_RATE_DIV_25HZ;
            break;
        case PACKET_RATE_20HZ:
             divider = PACKET_RATE_DIV_20HZ;
            break;
        case PACKET_RATE_10HZ:
            divider = PACKET_RATE_DIV_10HZ;
            break;
        case PACKET_RATE_5HZ:
            divider = PACKET_RATE_DIV_5HZ;
            break;
        case PACKET_RATE_2HZ:
            divider = PACKET_RATE_DIV_2HZ;
            break;
        case PACKET_RATE_QUIET:
            divider = PACKET_RATE_DIV_QUIET;
            break;
        default:
            return FALSE;

    }
    
    uint16_t const tmp = gConfiguration.packetRateDivider;

    gConfiguration.packetRateDivider =  (uint16_t)divider;      // validation uses 100Hz based criteria

    res = ValidPortConfiguration(&gConfiguration); 

    if ((res == FALSE) || (!fApply)) {
        gConfiguration.packetRateDivider =  tmp;
    }

    return res;
}




/******************************************************************************
 * @brief
 * @param type [in]  - chip num
 * @return counts
 ******************************************************************************/
static uint16_t GetFilterCounts(uint32_t const type)
{
    switch((int32_t)type){
        case LPF_02HZ:
            return 26785U;
        case LPF_05HZ:
            return 10713U;
        case LPF_10HZ:
            return 5356U;
        case LPF_20HZ:
            return 2678U;
        case LPF_25HZ:
            return 2142U;
        case LPF_40HZ:
            return 1338U;
        case LPF_UNFILTERED:
            return 0U;
        case LPF_50HZ:
        default:
            return 1070U;
    }

}

/******************************************************************************
 * @brief
 * @param idx [in]  - chip num
 * @return param
 ******************************************************************************/
uint16_t config_GetParam(uint8_t idx)
{
    idx &= 0x7FU;

    return ((uint16_t *)&gConfiguration)[idx];
}


/******************************************************************************
 * @brief tSelect_LP_filter API function for selecting LP filter type
 * @param sensor [in]  - sensor type to apply filter to 1 - accel, 2 - rate
 * @param cutoffFreq [in]  type   - new value selection for LP filter type
 * @param fApply [in]
 * @return boolean  TRUE if success FALSE otherwise
 ******************************************************************************/
BOOL config_SelectUserLPFilter(int32_t const sensor, int32_t const cutoffFreq, BOOL const fApply)
{
    BOOL res = TRUE;
    int32_t  type;

    switch(cutoffFreq){
        case CUTOFF_FREQ_UNFILTERED:
            type = LPF_UNFILTERED;
            break;
        case  CUTOFF_FREQ_2HZ:
            type = LPF_02HZ;
            break;
        case  CUTOFF_FREQ_5HZ:
                type = LPF_05HZ;
            break;
        case  CUTOFF_FREQ_10HZ:
            type = LPF_10HZ;
            break;
        case  CUTOFF_FREQ_20HZ:
            type = LPF_20HZ;
            break;
        case CUTOFF_FREQ_40HZ:
            type = LPF_40HZ;
            break;
        case  CUTOFF_FREQ_25HZ:
            type = LPF_25HZ;
            break;
        case  CUTOFF_FREQ_50HZ:
            type = LPF_50HZ;
            break;
        default:
            res = FALSE;
            break;
    }

    if((!fApply) || (res == FALSE)){
        return res;
    }

    gConfiguration.analogFilterClocks[sensor] = GetFilterCounts((uint32_t)type);

    return TRUE;
}



/*******************************************
 * @brief 
 * 
 * @param sensor ==
 * @return int32_t 
********************************************/
int32_t  config_GetFilterFreq(int32_t const sensor, uint16_t inCounts)
{
    uint16_t counts;

    if(sensor == ACCEL_SENSOR){
        counts = gConfiguration.analogFilterClocks[1];
    }else if(sensor == RATE_SENSOR){
        counts = gConfiguration.analogFilterClocks[2];
    }else{
        counts = inCounts;
    }
    
    if (counts > 18749U) {
        return CUTOFF_FREQ_2HZ;
    } else if (counts > 8034U) {
        return CUTOFF_FREQ_5HZ;
    } else if (counts > 4017U) {
        return CUTOFF_FREQ_10HZ;
    } else if (counts > 2410U) {
        return CUTOFF_FREQ_20HZ;
    } else if (counts > 1740U) {
        return CUTOFF_FREQ_25HZ;
    } else if (counts > 1204U) {
        return CUTOFF_FREQ_40HZ;
    } else if (counts > 0U) {
        return CUTOFF_FREQ_50HZ;
    } else {
        return CUTOFF_FREQ_UNFILTERED;
    } 

}



/******************************************************************************
 * @brief
 * @return rate
 ******************************************************************************/
int32_t      config_GetBaudRate(void)
{
    return baudEnumToBaudRate(gConfiguration.baudRateUser);
}

/******************************************************************************
 * @brief
 * @return divider
 ******************************************************************************/
uint32_t    config_GetPacketRateDivider()
{
    if(gConfiguration.packetRateDivider == 200U){
        return 1U;
    }
        return gConfiguration.packetRateDivider * 2U;
}


/******************************************************************************
 * @brief
 * @return divider
 ******************************************************************************/
uint32_t    config_GetUartPacketRate()
{
    if(gConfiguration.packetRateDivider == 200U){
        return 200U;
    }

    if(gConfiguration.packetRateDivider == 0U){
        return 0U;
    }
    
    return 100U/gConfiguration.packetRateDivider;
}



/******************************************************************************
 * @brief
 * @return code
 ******************************************************************************/
uint16_t config_GetOutputPacketCode()
{
        return gConfiguration.packetCode;
}


/******************************************************************************
 * @brief
 * @return freq
 ******************************************************************************/
uint16_t config_GetAccelLfpFreq()
{

    return gConfiguration.analogFilterClocks[1];
}

/******************************************************************************
 * @brief
 * @return freq
 ******************************************************************************/
uint16_t config_GetRateLfpFreq()
{

    return gConfiguration.analogFilterClocks[2];
}

/******************************************************************************
 * @brief
 * @return freq
 ******************************************************************************/
uint16_t config_GetPrefilterFreq()
{

    return gConfiguration.analogFilterClocks[0];
}


/******************************************************************************
 * @brief
 * @return orientation
 ******************************************************************************/
uint16_t config_GetOrientation()
{

    return gConfiguration.orientation;
}



/******************************************************************************
 * @brief
 * @return range
 ******************************************************************************/
uint16_t config_GetAccelRange()
{
    return gConfiguration.accelRange;
}

/******************************************************************************
 * @brief
 * @return range
 ******************************************************************************/
uint16_t config_GetGyroRange()
{
    return gConfiguration.gyroRange;
}

/*******************************************
 * @brief
 *  
 * @param versionBytes ==
********************************************/
void    config_GetVersionBytesFromAppVersion(uint8_t versionBytes[])
{
    uint8_t   vChar[3][10];
    uint32_t  idx     = 0U;
    uint32_t  section = 0U;
    uint32_t  offset  = 0U;
    int8_t    tmp;
    uint8_t   num[3] = {0U,0U,0U};
    int8_t    const dot   = 0x2E; // '.'
    int8_t    const space = 0x20; // ' '

    for(int32_t i = 0; i < 5; i++){
        versionBytes[i] = 0U;
    }

    memset(vChar, 0, sizeof(vChar));
    
    for(int32_t i = 0; i < 20; i++){
        tmp = APP_VERSION_STRING[idx]; 
        idx++;
        if(tmp == 0){
            break;
        }
        if(tmp == space){
            continue;
        }
        if(tmp  == dot){
            if(offset == 0U){
                return;
            }
            section++;
            if(section > 2U){
                break;
            }
            offset = 0U;
            continue;
        }
        tmp -= 0x30;
        if((tmp < 0) || (tmp > 9)){
            return;
        }
        vChar[section][offset] = (uint8_t)tmp;
        offset++;
        if(offset > 9U){
            return;
        }
    }
    
    for(int32_t j = 0; j < 3; j++){
        for(int32_t n = 0; n < 2; n++){
            num[j] *= 10U;
            num[j] += vChar[j][n]; 
        }
    }


    versionBytes[0]  = num[0];
    versionBytes[1]  = num[1];
    versionBytes[4]  = num[2];
    versionBytes[5]  = HW_PART_HI;
    versionBytes[6]  = HW_PART_MI;
    versionBytes[7]  = HW_PART_LO;

}

/******************************************************************************
 * @brief
 * @return size N/A
 ******************************************************************************/
uint16_t config_GetStructSize()
{
    return (uint16_t)sizeof(FactoryConfigurationStruct);
}

/******************************************************************************
 * @brief
 * @param period [in] sample new data to push onto the queue
 * @return periodF
 ******************************************************************************/
int16_t  config_GetAccelSignalConsistencyPeriod(uint16_t* const period)
{
    *period = (uint16_t)gConfiguration.accelSignalConsistencyPeriod;
    return (gConfiguration.accelSignalConsistencyPeriod/MSEC_IN_DACQ_PERIOD) + 1;
}

/******************************************************************************
 * @brief
 * @param period [in] sample new data to push onto the queue
 * @return periodF
 ******************************************************************************/
int16_t  config_GetRatesSignalConsistencyPeriod(uint16_t* const period)
{
    *period = (uint16_t)gConfiguration.ratesSignalConsistencyPeriod;
    return (gConfiguration.ratesSignalConsistencyPeriod/MSEC_IN_DACQ_PERIOD) + 1;
}


/******************************************************************************
 * @brief
 * @param trsh [in] sample new data to push onto the queue
 * @return threshold
 ******************************************************************************/
float32_t  config_GetAccelSignalConsistencyThreshold(uint16_t* const trsh)
{
    *trsh = (uint16_t)gConfiguration.accelSignalConsistencyThreshold;
    return (float32_t)gConfiguration.accelSignalConsistencyThreshold/1000.0F;
}

/******************************************************************************
 * @brief
 * @param trsh [in] sample new data to push onto the queue
 * @return threshold
 ******************************************************************************/
float32_t config_GetRatesSignalConsistencyThreshold(uint16_t* const trsh)
{
    *trsh = (uint16_t)gConfiguration.ratesSignalConsistencyThreshold;
    return (float32_t)gConfiguration.ratesSignalConsistencyThreshold/1000.0F;
}


 /******************************************************************************
 * @brief
 * @return boolean
 ******************************************************************************/
BOOL config_AccelConsistencyCheckEnabled()
{
    return (BOOL)gConfiguration.accelConsistencyCheckEnable;
}

 /******************************************************************************
 * @brief
 * @return boolean
 ******************************************************************************/
BOOL config_RateConsistencyCheckEnabled()
{
    return (BOOL)gConfiguration.ratesConsistencyCheckEnable;
}

 /******************************************************************************
 * @brief
 * @return boolean
 ******************************************************************************/
BOOL config_NeedApplySfCorrection()
{
    return gConfiguration.sfCorrection == 1U;
}

/*******************************************
 * @brief 
 * 
 * @param address ==
********************************************/
void    config_ApplyEcuAddress(uint8_t const address)
{
    gConfiguration.ecuAddress = address;
}

/*******************************************
 * @brief 
 * 
 * @return uint8_t 
********************************************/
uint16_t     config_GetEcuAddress()
{
    return (uint8_t)gConfiguration.ecuAddress;
}


/*******************************************
 * @brief 
 * 
 * @param baudrate ==
********************************************/
void    config_ApplyEcuBaudrate(uint8_t const baudrate)
{
    gConfiguration.ecuBaudRate = baudrate;
}

/*******************************************
 * @brief 
 * 
 * @return uint16_t 
********************************************/
uint16_t    config_GetEcuBaudRate()
{
    return gConfiguration.ecuBaudRate;
}


/*******************************************
 * @brief 
 * 
 * @param behavior ==
********************************************/
void    config_ApplyEcuUnitBehavior(uint16_t behavior)
{
    gConfiguration.ecuUnitBehavior = behavior;
}

/*******************************************
 * @brief 
 * 
 * @param rate ==
********************************************/
void    config_ApplyCanPacketRate(uint16_t rate)
{
    if(!rate){
        gConfiguration.CanOdr = 0;
    }else{
        gConfiguration.CanOdr = rate;
    }
}


/*******************************************
 * @brief 
 * 
 * @param type ==
********************************************/
void    config_ApplyCanPacketType(uint16_t type)
{
    gConfiguration.canPacketType = type;
}

/*******************************************
 * @brief 
 * 
 * @param accelLpf ==
 * @param rateLpf ==
********************************************/
void    config_ApplyCanAccelFilter(uint16_t accelLpf)
{
    gConfiguration.analogFilterClocks[1] = accelLpf;
}

/*******************************************
 * @brief 
 * 
 * @param rateLpf ==
********************************************/
void    config_ApplyCanRateFilter(uint16_t rateLpf)
{
    gConfiguration.analogFilterClocks[2] = rateLpf;
}


/*******************************************
 * @brief 
 * 
 * @param orientation ==
********************************************/
void    config_ApplyCanOrientation(uint16_t orientation)
{
    gConfiguration.orientation = orientation;
}


/*******************************************
 * @brief 
 * 
 * @param gyroThreshold ==
 * @param accelThreshold ==
********************************************/
void config_GetSamplingDiffThreshonds(uint16_t* gyroThreshold, uint16_t* accelThreshold)
{
    *gyroThreshold  =  gConfiguration.gyroSamplingCountsThreshold;    
    *accelThreshold =  gConfiguration.accelSamplingCountsThreshold;  
}




#ifdef UNIT_TEST
#include "configuration_wrappers.h"
#endif


/*end void initConfigureUnit(void) */
