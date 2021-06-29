/******************************************************************************
 * @file TimingVars.h
 * @author Joe Motyka
 * @brief 
 * @version 1.0
 * @date 2020-09-09
 * 
 * @copyright Copyright (c) 2020
 * 
 *****************************************************************************/

#ifndef TIMINGVARS_H
#define TIMINGVARS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>  // uint8_t, etc.
#include "GlobalConstants.h"

#ifdef DISPLAY_DIAGNOSTIC_MSG
#ifdef INS_OFFLINE
#include <iostream>     // std::cin, std::cout
#include <fstream>      // std::ifstream
#include <string>
#include <sstream>
#else
#include "debug.h"
#endif
#endif

#ifdef DISPLAY_DIAGNOSTIC_MSG
/*******************************************
 * @brief 
 * 
 * @param timeStep ==
********************************************/
void TimingVars_DisplayTimerVars(int32_t timeStep);
#ifdef INS_OFFLINE
/*******************************************
 * @brief 
 * 
 * @param msg ==
********************************************/
void TimingVars_DiagnosticMsg(std::string msg);
#else
/*******************************************
 * @brief 
 * 
 * @param msg ==
********************************************/
void TimingVars_DiagnosticMsg(char* msg);
#endif
#endif

//
/*******************************************
 * @brief 
 * 
********************************************/
void TimingVars_Increment(void);

/******************************************************************************
 * @brief Get time in seconds.
 * 
 * @return float32_t 
 *****************************************************************************/
float32_t TimingVars_GetTime(void);

/******************************************************************************
 * @brief Set minutes.
 * 
 * @param tMin  [in]    Input minute.
 *****************************************************************************/
void  TimingVars_SetTMin(float32_t const tMin);

/******************************************************************************
 * @brief Get minutes.
 * 
 * @return float32_t 
 *****************************************************************************/
float32_t TimingVars_GetTMin(void);

/*******************************************
 * @brief 
 * 
 * @param freq ==
********************************************/
void TimingVars_dacqFrequency(uint32_t const freq);

/******************************************************************************
 * @brief Get time step.
 * 
 * @return uint32_t 
 *****************************************************************************/
uint32_t TimingVars_GetTimeStep(void);

typedef struct {
    uint32_t dacqFrequency;
    uint32_t secondCntr;
    uint32_t basicFrameCounter;
    uint8_t  tenHertzCntr;
    int8_t   subFrameCntr;
    // toggles between 0 and 1 at 200 Hz (currently used in firmware)
    uint8_t oneHundredHertzFlag;

    //
    float32_t time;
    float32_t tMin;
} TimingVars;

extern TimingVars timer;

/*******************************************
 * @brief 
 * 
********************************************/
void Initialize_Timing(void);

#ifdef __cplusplus
}
#endif

#endif /* TIMINGVARS_H */

