/** ***************************************************************************
 * @file filterAPI.h iir, fir, moving average, debounce digital filters
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

#ifndef FILTER_API_H
#define FILTER_API_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif

enum {
    LPF_UNFILTERED = 0,
    LPF_02HZ       = 1,
    LPF_05HZ       = 2,
    LPF_10HZ       = 3,
    LPF_20HZ       = 4,
    LPF_25HZ       = 5,
    LPF_40HZ       = 6,
    LPF_50HZ       = 7,
};

enum
{
    CUTOFF_FREQ_50HZ        = 50,
    CUTOFF_FREQ_40HZ        = 40,
    CUTOFF_FREQ_25HZ        = 25,
    CUTOFF_FREQ_20HZ        = 20,
    CUTOFF_FREQ_10HZ        = 10,
    CUTOFF_FREQ_5HZ         = 5,
    CUTOFF_FREQ_2HZ         = 2,
    CUTOFF_FREQ_UNFILTERED  = 0,
};


/*******************************************
 * @brief 
 * 
 * @param idx ==
 * @param freq ==
********************************************/
void filter_ApplyToAccelerometerData(int32_t const idx, int32_t inData[], int32_t outData[]);

/*******************************************
 * @brief 
 * 
 * @param idx ==
 * @param freq ==
********************************************/
void filter_ApplyToRateSensorData(int32_t const idx, int32_t inData[], int32_t outData[]);


#ifdef __cplusplus
}
#endif

#endif /* FILTER_H */
