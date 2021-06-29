/** ***************************************************************************
 * @file crc.h
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

#ifndef CRC_H
#define CRC_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint16_t CrcCcittType;
typedef uint32_t Crc32Type;

static uint32_t const CRC_32_INITIAL_SEED = 0xFFFFFFFFU;
static uint16_t const CRC_CCITT_INITIAL_SEED = 0x1D0FU;

enum{
    CRC_32_LENGTH  = 4,
    CRC_CCITT_LENGTH = 2
};

/**
 * @brief
 *
 * @param data
 * @param length
 * @param seed
 * @return CrcCcittType
 */
extern CrcCcittType CrcCcitt			(uint8_t const data [], uint16_t const length, CrcCcittType const seed);

/**
 * @brief
 *
 * @param data
 * @param length
 * @param seed
 * @return Crc32Type
 */
extern Crc32Type    Crc32				(uint8_t const data [], uint16_t const length, Crc32Type const seed);

/**
 * @brief
 *
 * @param type
 * @param bytes
 */
extern void         CrcCcittTypeToBytes	(CrcCcittType const type, uint8_t bytes []);

/**
 * @brief
 *
 * @param bytes
 * @return CrcCcittType
 */
extern CrcCcittType BytesToCrcCcittType (uint8_t const bytes []);

/**
 * @brief
 *
 * @param type
 * @param bytes
 */
extern void         Crc32TypeToBytes    (Crc32Type const type, uint8_t bytes []);

/**
 * @brief
 *
 * @param bytes
 * @return Crc32Type
 */
extern Crc32Type    BytesToCrc32Type    (uint8_t const bytes []);

/**
 * @brief
 *
 * @param v
 * @param seed
 * @return uint16_t
 */
uint16_t            initCRC_16bit       (uint16_t  const v, uint16_t const seed);

#ifdef __cplusplus
}
#endif

#endif
