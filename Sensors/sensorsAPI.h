/** ***************************************************************************
 * @file sensorsI.h API functions for Magnitometer functionality
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

#ifndef SENSORS_API_H
#define SENSORS_API_H

#include <stdint.h>
#include "GlobalConstants.h"

/*******************************************
 * @brief 
 * 
 * @param data ==
********************************************/
void  sens_GetAccelData_g(float64_t *data);

/*******************************************
 * @brief 
 * 
 * @param idx ==
 * @param data ==
********************************************/
void  sens_GetChipAccelData_g(int32_t idx, float32_t *data);

/*******************************************
 * @brief 
 * 
 * @param idx ==
 * @param axis ==
 * @return float32_t 
********************************************/
float32_t sens_GetChipAccelAxisData_g(int32_t idx, int32_t axis);

/*******************************************
 * @brief 
 * 
 * @param data ==
********************************************/
void  sens_GetAccelData_mPerSecSq(float64_t *data);

/*******************************************
 * @brief 
 * 
 * @param data ==
********************************************/
void  sens_GetRateData_radPerSec(float64_t *data);

/*******************************************
 * @brief 
 * 
 * @param idx ==
 * @param axis ==
 * @return float32_t 
********************************************/
float32_t sens_GetChipRateAxisData_radPerSec(int32_t idx, int32_t axis);

/*******************************************
 * @brief 
 * 
 * @param data ==
********************************************/
void  sens_GetRateData_degPerSec(float64_t *data);

/*******************************************
 * @brief 
 * 
 * @param idx ==
 * @param data ==
********************************************/
void  sens_GetChipRateData_degPerSec(int32_t idx, float32_t *data);

/*******************************************
 * @brief 
 * 
 * @param data ==
********************************************/
void  sens_GetMagData_G(float64_t *data);

/*******************************************
 * @brief 
 * 
 * @param temp ==
********************************************/
void  sens_GetBoardTempData(float64_t *temp);

/*******************************************
 * @brief 
 * 
 * @return float32_t 
********************************************/
float32_t sens_GetUnitTemp();

/*******************************************
 * @brief 
 * 
 * @param chipId ==
 * @return float32_t 
********************************************/
float32_t sens_GetChipTemp(int32_t chipId);

/*******************************************
 * @brief 
 * 
********************************************/
void sens_FillRawData();


/** ****************************************************************************
 * @name CombineSensorsData
 * @brief Combines sensors data after calibration
 *  
 ******************************************************************************/
void sens_CombineScaledData();

/** ****************************************************************************
 * @name initSensorsData
 * @brief Initialized sensors data structures
 *  
 ******************************************************************************/
void sens_InitDataStructures();


/** ****************************************************************************
 * @name SapmleSensorsData
 * @brief Performs sampling of sensors data
 *  
 ******************************************************************************/
void sens_SampleData();


/*******************************************
 * @brief 
 * 
 * @param validMask ==
********************************************/
void sens_Init(uint16_t *validMask);


/*******************************************
 * @brief 
 * 
 * @return int32_t 
********************************************/
int32_t   sens_GetRawTempCounts();

/*******************************************
 * @brief 
 * 
 * @param rawData ==
********************************************/
void      sens_GetRawData(int32_t rawData[]);

/*******************************************
 * @brief 
 * 
 * @param rawData ==
 * @param chipId ==
********************************************/
void      sens_GetRawChipData(int32_t rawData[], int32_t const chipId);

/*******************************************
 * @brief 
 * 
 * @param rawData ==
 * @param sensor ==
 * @param idx ==
********************************************/
void      sens_SetRawChipSensorData(int32_t const rawData, int32_t const sensor, int32_t const idx);

/*******************************************
 * @brief 
 * 
 * @param ptr ==
 * @return uint32_t 
********************************************/
uint32_t  sens_FillRawPayload(void *ptr);

/*******************************************
 * @brief 
 * 
 * @return BOOL 
********************************************/
BOOL sens_SamplingEnabled();

/*******************************************
 * @brief 
 * 
 * @return BOOL 
********************************************/
BOOL    sens_ResetFifo();

/*******************************************
 * @brief 
 * 
 * @param cycleNum ==
 * @param fBuffer ==
********************************************/
void sens_GetNumSamplesInFifo(uint16_t cycleNum, BOOL fBuffer);

#endif
