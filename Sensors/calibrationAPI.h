/** ***************************************************************************
 * @file calibration.h
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef CALIBRATION_API_H
#define CALIBRATION_API_H
#include <stdint.h>

/**
 * @brief
 *
 */
void     cal_Init(void);

/**
 * @brief
 *
 */
void     cal_Apply(void);

/**
 * @brief
 *
 * @return uint32_t
 */
uint32_t cal_GetUnitSerialNum();

/**
 * @brief
 *
 * @return uint8_t*
 */
uint8_t *cal_GetUnitVersion();

/**
 * @brief
 *
 * @return uint16_t
 */
uint16_t cal_GetProductConfiguration();

/**
 * @brief
 *
 * @param idx
 * @return BOOL
 */
BOOL     checkCalibrationStructCrc(uint8_t idx);

#ifdef UNIT_TEST
#include "wrappers.h"
#endif

#endif
