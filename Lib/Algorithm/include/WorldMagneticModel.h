/******************************************************************************
* File:   WorldMagneticModel.h
* Created on May 16, 2016, 8:22 PM
*******************************************************************************/
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


#ifndef WORLDMAGNETICMODEL_H
#define WORLDMAGNETICMODEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "GlobalConstants.h"


// 
typedef struct {
    uint32_t timeOfLastSoln;
    BOOL validSoln;

    float32_t decl_rad;
} WorldMagModelStruct;

extern  WorldMagModelStruct  gWorldMagModel;

#ifdef __cplusplus
}
#endif

#endif /* WORLDMAGNETICMODEL_H */
