/** ******************************************************************************
 * @file configurationAPI.h API functions for Interfacing with unit configurationb parameters
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


#ifndef CONFIG_API_H
#define CONFIG_API_H
#include <stdint.h>
#include "GlobalConstants.h"

// serial port related functions
/**
 * @brief
 *
 * @return int32_t
 */
int32_t  config_GetBaudRate(void);



/**
 * @brief
 *
 * @return uint32_t
 */
uint32_t config_GetPacketRateDivider();


/*******************************************
 * @brief 
 * 
 * @param divider ==
 * @return BOOL 
********************************************/
BOOL config_SetPacketRateDivider(uint32_t divider);


/**
 * @brief
 *
 */
void     config_ApplyDefaultSerialPortSettings (void);

/**
 * @brief
 *
 * @return uint16_t
 */
uint16_t config_GetOutputPacketCode();

// IMU related functions
/**
 * @brief
 *
 * @return uint16_t
 */
uint16_t config_GetOrientation(void);

/**
 * @brief
 *
 * @return uint16_t
 */
uint16_t config_GetAccelLfpFreq();

/**
 * @brief
 *
 * @return uint16_t
 */
uint16_t config_GetRateLfpFreq();

/**
 * @brief
 *
 * @return uint16_t
 */
uint16_t config_GetPrefilterFreq();

/**
 * @brief
 *
 * @param sensor
 * @param cutoffFreq
 * @param fApply
 * @return BOOL
 */
BOOL     config_SelectUserLPFilter(int32_t sensor, int32_t cutoffFreq, BOOL fApply);

/**
 * @brief
 *
 * @param chipIdx
 * @return uint16_t
 */
uint16_t config_GetUsedSensors(int32_t chipIdx);


/**
 * @brief
 *
 * @return uint16_t
 */
uint16_t config_GetUsedChips(void);


/**
 * @brief
 *
 * @return uint16_t
 */
uint16_t config_GetActiveChips(void);

/**
 * @brief
 *
 * @param idx
 * @return uint16_t
 */
uint16_t config_GetParam(uint8_t idx);

// CAN bus related parameters
/**
 * @brief
 *
 */
void     ApplyFactoryConfiguration();

/**
 * @brief
 *
 * @return uint16_t
 */
uint16_t config_GetAccelRange();

/**
 * @brief
 *
 * @return uint16_t
 */
uint16_t config_GetGyroRange();


/**
 * @brief
 *
 * @return uint16_t
 */
uint16_t config_GetStructSize();


// general functions
/**
 * @brief
 *
 * @param numFields
 * @param fieldId
 * @param fieldData
 * @param validFields
 * @return uint8_t
 */
uint8_t  config_CheckRamFieldData (uint8_t  numFields,
                                 uint16_t fieldId [],
                                 uint16_t fieldData [],
                                 uint16_t validFields []);

/**
 * @brief
 *
 * @param numFields
 * @param fieldId
 * @param fieldData
 * @param validFields
 * @return uint8_t
 */
uint8_t  config_CheckEepromFieldData (uint8_t  numFields,
                              uint16_t fieldId [],
                              uint16_t fieldData [],
                              uint16_t validFields []);

/**
 * @brief
 *
 */
void     config_SetFieldData (void);

/**
 * @brief
 *
 * @return BOOL
 */
BOOL     config_WriteFieldData (void);




// fault detection - related
/**
 * @brief
 *
 * @return BOOL
 */
BOOL     config_AccelConsistencyCheckEnabled();

/**
 * @brief
 *
 * @return BOOL
 */
BOOL     config_RateConsistencyCheckEnabled();

/**
 * @brief
 *
 * @param period
 * @return int16_t
 */
int16_t  config_GetAccelSignalConsistencyPeriod(uint16_t* const period);

/**
 * @brief
 *
 * @param period
 * @return int16_t
 */
int16_t  config_GetRatesSignalConsistencyPeriod(uint16_t* const period);

/**
 * @brief
 *
 * @param trsh
 * @return float32_t
 */
float32_t config_GetAccelSignalConsistencyThreshold(uint16_t* const trsh);

/**
 * @brief
 *
 * @param trsh
 * @return float32_t
 */
float32_t config_GetRatesSignalConsistencyThreshold(uint16_t* const trsh);

/**
 * @brief
 *
 * @return BOOL
 */
BOOL     config_NeedApplySfCorrection();

/*******************************************
 * @brief
 *
 * @param orientation ==
 * @param fApply ==
 * @return BOOL
********************************************/
BOOL     config_ApplyOrientation(uint16_t const orientation, BOOL const fApply);

/*******************************************
 * @brief
 *
 * @param address ==
********************************************/
void    config_ApplyEcuAddress(uint8_t address);

/*******************************************
 * @brief
 *
 * @param baudrate ==
********************************************/
void    config_ApplyEcuBaudrate(uint8_t baudrate);


/*******************************************
 * @brief 
 * 
 * @param code ==
 * @param fApply ==
 * @return BOOL 
********************************************/
BOOL     config_SetOutputPacketCode(uint16_t code, BOOL fApply);


/*******************************************
 * @brief 
 * 
 * @param sensor ==
 * @param inCounts ==
 * @return int32_t 
********************************************/
int32_t  config_GetFilterFreq(int32_t sensor, uint16_t inCounts);


/*******************************************
 * @brief 
 * 
 * @return uint32_t 
********************************************/
uint32_t  config_GetUartPacketRate();


/*******************************************
 * @brief 
 * 
 * @param baudRate ==
 * @param fApply ==
 * @return BOOL 
********************************************/
BOOL config_SetBaudRate(int32_t baudRate, BOOL fApply);


/*******************************************
 * @brief 
 * 
 * @param rate ==
 * @param fApply ==
 * @return BOOL 
********************************************/
BOOL config_SetPacketRate(int32_t rate, BOOL fApply);

/*******************************************
 * @brief 
 * 
 * @param versionBytes ==
********************************************/
void config_GetVersionBytesFromAppVersion(uint8_t versionBytes[]);


/*******************************************
 * @brief 
 * 
 * @param fEnable ==
********************************************/
void    config_ApplyEcuAutoBaudMode(BOOL const fEnable);


/*******************************************
 * @brief 
 * 
 * @return uint8_t 
********************************************/
uint16_t     config_GetEcuAddress();

/*******************************************
 * @brief 
 * 
 * @return uint16_t 
********************************************/
uint16_t    config_GetEcuBaudRate();

/*******************************************
 * @brief 
 * 
 * @param behavior ==
********************************************/
void    config_ApplyEcuUnitBehavior(uint16_t behavior);

/*******************************************
 * @brief 
 * 
 * @param rate ==
********************************************/
void    config_ApplyCanPacketRate(uint16_t rate);


/*******************************************
 * @brief 
 * 
 * @param type ==
********************************************/
void    config_ApplyCanPacketType(uint16_t type);

/*******************************************
 * @brief 
 * 
 * @param accelLpf ==
********************************************/
void    config_ApplyCanAccelFilter(uint16_t accelLpf);

/*******************************************
 * @brief 
 * 
 * @param rateLpf ==
********************************************/
void    config_ApplyCanRateFilter(uint16_t rateLpf);


/*******************************************
 * @brief 
 * 
 * @param orientation ==
********************************************/
void    config_ApplyCanOrientation(uint16_t orientation);

/*******************************************
 * @brief 
 * 
 * @param orientation ==
********************************************/
void    config_GetSamplingDiffThreshonds(uint16_t* gyroThreshold, uint16_t* accelThreshold);

/*******************************************
 * @brief 
 * 
********************************************/
void config_GetBootloaderVersion(uint8_t version[]);

/*******************************************
 * @brief 
 * 
 * @return BOOL 
********************************************/
BOOL config_IsChipFilterDisabled();


#ifdef UNIT_TEST
#include "configuration_wrappers_h.h"
#endif

#endif
