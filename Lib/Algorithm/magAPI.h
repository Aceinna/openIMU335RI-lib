/** ***************************************************************************
 * @file magAPI.h API functions for Magnitometer functionality
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

#ifndef MAG_API_H
#define MAG_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "GlobalConstants.h"

/******************************************************************************
 * @brief Initialize the parameters for magnetic heading calculation
 *  
 *****************************************************************************/
void MagAlign_Init(void);

// example of user payload structure
typedef struct {
    uint8_t   parameter[8];
} magAlignCmdPayload;

typedef struct{
    real  hardIron_X;
    real  hardIron_Y;
    real  softIron_Ratio;
    real  softIron_Angle;
}magAlignUserParams_t;

/******************************************************************************
 * @brief Initialzie the world magnetic mode.
 *  
 *****************************************************************************/
void WMM_Init();

#ifdef __cplusplus
}
#endif

#endif
