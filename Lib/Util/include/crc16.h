/** ***************************************************************************
 * @file crc16.h
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

#ifndef CRC16_H
#define CRC16_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *
 * @param data
 * @param len
 * @return uint16_t
 */
uint16_t CRC16(uint8_t data[], int32_t len);

/**
 * @brief
 *
 * @param buf
 * @param length
 * @return uint16_t
 */
uint16_t CalculateCRC (uint8_t *buf, int32_t  length);

#ifdef __cplusplus
}
#endif

#endif
