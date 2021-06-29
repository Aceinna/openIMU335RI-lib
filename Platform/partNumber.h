/** ***************************************************************************
 * @file platform_version.h
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
#ifndef PART_NUMBER_H
#define PART_NUMBER_H
#include <stdint.h>

static uint8_t const  VERSION_MAJOR = 0U;
static uint8_t const  VERSION_MINOR = 0U;
static uint8_t const  VERSION_PATCH = 0U;
static uint8_t const  VERSION_STAGE = 0U;
static uint8_t const  VERSION_BUILD = 0U;

#ifdef MTLT335_PART

static uint8_t const HW_PART_HI = 33U;
static uint8_t const HW_PART_MI = 27U;
static uint8_t const HW_PART_LO = 01U;

static uint8_t const PART_NUMBER_STRING[] = "5020-3327-01"; 
#else 
// openIMU225RI
static uint8_t const HW_PART_HI = 33U;
static uint8_t const HW_PART_MI = 21U;
static uint8_t const HW_PART_LO = 01U;

static uint8_t const PART_NUMBER_STRING[] = "5020-3321-01"; 
#endif

static int32_t const SOFTWARE_PART_LEN = 50;
static int32_t const N_VERSION_STR   = 128;

#endif
