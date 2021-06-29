/** ***************************************************************************
 * @file BITStatus.h
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

#ifndef BIT_API_H
#define BIT_API_H

#include "GlobalConstants.h"
#include <stdint.h>

/*******************************************
 * @brief 
 * 
********************************************/
void     BIT_Iinitialize();

/*******************************************
 * @brief 
 * 
 * @param chipMask  ==
********************************************/
void     BIT_UpdateSensorSelfTestStatus(uint8_t const chipMask);

/*******************************************
 * @brief 
 * 
 * @param chip  ==
 * @param fAccel  ==
********************************************/
void     BIT_UpdateFaultDetectionStatus(uint8_t const chip, BOOL const fAccel);

/*******************************************
 * @brief 
 * 
 * @param status  ==
********************************************/
void     BIT_UpdateAlgorithmStatus(uint16_t const status);

/*******************************************
 * @brief 
 * 
 * @param chip  ==
 * @param sensorAxis  ==
 * @param fOverRange  ==
********************************************/
void     BIT_UpdateSensorOverRangeStatus(uint8_t const chip, uint8_t sensorAxis, BOOL const fOverRange);

/*******************************************
 * @brief 
 * 
 * @param chip  ==
********************************************/
void     BIT_SetInvalidCalStatus(uint8_t const chip);

/*******************************************
 * @brief 
 * 
********************************************/
void     BIT_SetInvalidConfigStatus();

/*******************************************
 * @brief 
 * 
 * @param fError  ==
********************************************/
void     BIT_UpdateCpuOverTempError(BOOL const fError);

/*******************************************
 * @brief 
 * 
 * @param chip  ==
 * @param fOverTemp  ==
********************************************/
void     BIT_UpdateChipOverTempError(uint8_t const chip, BOOL const fOverTemp);

/*******************************************
 * @brief 
 * 
 * @param fError  ==
********************************************/
void     BIT_UpdateIntPowerSupError(BOOL const fError);

/*******************************************
 * @brief 
 * 
 * @param fError  ==
********************************************/
void     BIT_UpdateExtPowerSupError(BOOL const fError);

/*******************************************
 * @brief 
 * 
 * @param fOverConsumption  ==
********************************************/
void     BIT_UpdatePowerConsumptionError(BOOL const fOverConsumption);

/*******************************************
 * @brief 
 * 
 * @return BOOL 
********************************************/
BOOL     BIT_IsEepromUnlocked();

/*******************************************
 * @brief 
 * 
 * @return uint16_t 
********************************************/
uint16_t BIT_GetMasterStatusWord();

/*******************************************
 * @brief 
 * 
 * @return uint16_t 
********************************************/
uint16_t BIT_GetHWStatusWord();

/*******************************************
 * @brief 
 * 
 * @return uint32_t 
********************************************/
uint32_t BIT_GetSWStatusWord();


/*******************************************
 * @brief 
 * 
 * @return uint16_t 
********************************************/
uint16_t BIT_GetCommStatusWord();

/*******************************************
 * @brief 
 * 
 * @return uint16_t 
********************************************/
uint16_t BIT_GetAlgoStatusWord();

/*******************************************
 * @brief 
 * 
 * @return uint16_t 
********************************************/
uint16_t BIT_GetSensorStatusWord_MSB();

/*******************************************
 * @brief 
 * 
 * @return uint16_t 
********************************************/
uint16_t BIT_GetSensorStatusWord_LSB();

/*******************************************
 * @brief 
 * 
 * @param locked  ==
********************************************/
void     BIT_SetEepromLockStatus(BOOL const locked);

/*******************************************
 * @brief 
 * 
 * @param status  ==
********************************************/
void     BIT_DacqOverrun(BOOL const status);

/*******************************************
 * @brief 
 * 
********************************************/
void     BIT_PerformPeriodicTest();

/*******************************************
 * @brief 
 * 
 * @return BOOL 
********************************************/
BOOL     BIT_NeedToSendStatus();

/*******************************************
 * @brief 
 * 
 * @param calId  ==
********************************************/
void     BIT_TestCalibrationPartition(uint8_t const calId);

/*******************************************
 * @brief 
 * 
********************************************/
void     BIT_TestStack();

/*******************************************
 * @brief 
 * 
 * @param overflowed  ==
 * @return BOOL 
********************************************/
BOOL     BIT_CANTxOverflow(BOOL const overflowed);

/*******************************************
 * @brief 
 * 
 * @param overflowed  ==
 * @return BOOL 
********************************************/
BOOL     BIT_CANRxOverflow(BOOL const overflowed);

/*******************************************
 * @brief 
 * 
 * @param status  ==
********************************************/
void     BIT_UpdateTurnSwitchError(BOOL const status);

/*******************************************
 * @brief 
 * 
 * @param fHighGain  ==
********************************************/
void     BIT_SetHighGainMode(BOOL const fHighGain);

/*******************************************
 * @brief 
 * 
 * @param axisInfo  ==
 * @param fAccel  ==
 * @param fChipFailure  ==
********************************************/
void     BIT_UpdateSensorsDisagreementStatus(uint8_t const * const axisInfo, BOOL const fAccel, BOOL const fChipFailure);

/*******************************************
 * @brief 
 * 
 * @return BOOL 
********************************************/
BOOL     BIT_IsAlgorithmDegraded();

/*******************************************
 * @brief 
 * 
 * @return BOOL 
********************************************/
BOOL     BIT_IsAlgorithmError();

/*******************************************
 * @brief 
 * 
 * @param axis  ==
 * @return BOOL 
********************************************/
BOOL     BIT_IsAccelerationDegraded(int32_t const axis);

/*******************************************
 * @brief 
 * 
 * @param axis  ==
 * @return BOOL 
********************************************/
BOOL     BIT_IsAccelerationError(int32_t const axis);

/*******************************************
 * @brief 
 * 
 * @param axis  ==
 * @return BOOL 
********************************************/
BOOL     BIT_IsRatesDegraded(int32_t const axis);

/*******************************************
 * @brief 
 * 
 * @param axis  ==
 * @return BOOL 
********************************************/
BOOL     BIT_IsRatesError(int32_t axis);

/*******************************************
 * @brief 
 * 
 * @return uint16_t 
********************************************/
uint16_t BIT_GetUsedChips();

/*******************************************
 * @brief 
 * 
********************************************/
void     BIT_UpdateConfiguredSensorChips();

/*******************************************
 * @brief 
 * 
 * @param chip  ==
 * @return uint16_t 
********************************************/
uint16_t BIT_GetUsedSensors(int32_t const chip);

/*******************************************
 * @brief 
 * 
 * @param accels ==
 * @param rates ==
********************************************/
void     BIT_GetUsedSensorsMask(uint8_t accels[], uint8_t rates[]);

/*******************************************
 * @brief 
 * 
********************************************/
void     BIT_UpdateDaqCycleStartFlag();

/*******************************************
 * @brief 
 * 
********************************************/
void     BIT_ClearFifoResetEvents();

/*******************************************
 * @brief 
 * 
 * @return BOOL 
********************************************/
BOOL     BIT_NeedResetFifo();

/*******************************************
 * @brief 
 * 
********************************************/
void     BIT_NotifyEepromWriteEvent();

/*******************************************
 * @brief 
 * 
********************************************/
void     BIT_BegintPowerCheck();

/*******************************************
 * @brief 
 * 
********************************************/
void     BIT_CheckPower();

/*******************************************
 * @brief 
 * 
 * @return uint8_t 
********************************************/
uint8_t  BIT_GetFmiCode();


/*******************************************
 * @brief 
 * 
 * @return BOOL 
********************************************/
BOOL BIT_NeedToSendDM1Message(uint8_t *fmiCode, uint8_t *count);


/*******************************************
 * @brief 
 * 
 * @param FMI1 ==
 * @param FMI2 ==
********************************************/
void BIT_SetFMICodes(uint8_t FMI1, uint8_t FMI2);

/*******************************************
 * @brief 
 * 
********************************************/
void BIT_ResetDTC();

/*
static uint16_t  const  BIT_OUT_PERIOD                  =   200U;               // so far 1 second
static uint16_t  const  BIT_CAL_TEST_PERIOD             =   1U ;                // so far every cycle
static uint16_t  const  BIT_CONFIG_TEST_PERIOD          =   200U;               // so far every second
static uint16_t  const  BIT_STACK_TEST_PERIOD           =   10U;                // so far every 50 ms
static uint32_t  const  BIT_CAN_OVERFLOW_THRESHOLD      =   (uint32_t)((uint32_t)200U * (uint32_t)20U);       // 20 seconds
static uint32_t  const  BIT_CAN_TX_OVERFLOW_THRESHOLD   =   3U;                 // Can we afford to loose some message?
static uint32_t  const  BIT_CAN_RX_OVERFLOW_THRESHOLD   =   5U;                 // Less severe than transmit - host can retry
static uint32_t  const  BIT_TEMP_HI_THRESHOLD           =   (uint32_t)((uint32_t)200U * (uint32_t)60U * (uint32_t)5U);  // 5 minutes
static uint32_t  const  BIT_TEMP_LOW_THRESHOLD          =   (uint32_t)((uint32_t)200U * (uint32_t)60U * (uint32_t)1U);  // 1 minute
static uint16_t  const  BIT_ALGO_INIT_THRESHOLD         =   (uint16_t)((uint16_t)200U * (uint16_t)3U);        // 3 seconds
static uint16_t  const  BIT_MAX_OVERRANGE_LIMIT         =   4U;                 // 4 data processing cycles
static uint16_t  const  BIT_MIN_OVERRANGE_LIMIT         =   2U;                 // 2 data processing cycles
static uint16_t  const  DATA_OVERRUN_MAX                =   5U;                 // data processing overrun threshold
static uint16_t  const  BIT_TEMP_CPU_THRESHOLD          =   300U;               // checked every second - 5 minutes
static uint16_t  const  FIFO_RESET_CAUSE_EEPROM         =   0x0001U;
static uint16_t  const  POWER_OUTAGE_TRIGGER_TIME       =   60U;                 // 10 seconds

static uint16_t  const  BIT_FMI_CODE_OK                 =   0U ; 
static uint16_t  const  BIT_FMI_CODE_NOTIFY             =   1U ; 
static uint16_t  const  BIT_FMI_CODE_WARNING            =   2U ; 
static uint16_t  const  BIT_FMI_CODE_FAILURE            =   3U ; 
static uint16_t  const  BIT_DM1_PERIOD                  =   2000U ;             // 10 seconds with 200Hz DACQ task cycle 
*/

#ifdef UNIT_TEST
#include "bit_wrappers_h.h"
#endif

#endif
