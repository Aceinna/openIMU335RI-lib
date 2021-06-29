/******************************************************************************
 * File:   TimingVars.c
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


#include "TimingVars.h"

TimingVars           timer;   // for InitTimingVars

/******************************************************************************
 * @brief Increment timing variables that control the operation of the Extended
 * Kalman Filter (in particular, the update stage of the EKF).
 * Output: secondCntr -- increments once per secondCntr (1 Hz); not reset.
 *         tenHertzCntr -- increments ten times per second (10 Hz); reset once
 *                         it reached 10.
 *         subFrameCntr -- increments one hundred times per second (100 Hz);
 *                         reset once it reaches 10.
 *         t -- time generated from the three counters
 *  
 ******************************************************************************/
void TimingVars_Increment(void)
{
    if(timer.dacqFrequency == (uint32_t)DACQ_100_HZ){
        timer.oneHundredHertzFlag = 1U;
    }else{
        timer.oneHundredHertzFlag++;
        timer.oneHundredHertzFlag &= 1U;     // toggle 1Hz flag
    }

    if ( timer.basicFrameCounter >= timer.dacqFrequency) {
        timer.basicFrameCounter = 0U;
    } else {
        timer.basicFrameCounter++;
    }

    if (timer.oneHundredHertzFlag == 1U) {
        timer.subFrameCntr = timer.subFrameCntr + 1;
        if (timer.subFrameCntr >= 10) {
            timer.subFrameCntr = 0;

            timer.tenHertzCntr = timer.tenHertzCntr + 1U;
            if (timer.tenHertzCntr >= 10U) {
                timer.tenHertzCntr = 0U;

                timer.secondCntr = timer.secondCntr + 1U;
            }
        }
    }
}

/******************************************************************************
 * @brief Get time in seconds.
 * 
 * @return float32_t 
 *****************************************************************************/
float32_t TimingVars_GetTime(void)
{
    timer.time = (float32_t)( ( 1.00F*(float32_t)timer.secondCntr ) +
                              ( 0.10F*(float32_t)timer.tenHertzCntr ) +
                              ( 0.01F* (float32_t)timer.subFrameCntr ) );
    //timing.t = timing.t + timing.tMin;

    return(timer.time);
}

/******************************************************************************
 * @brief Set minutes.
 * 
 * @param tMin  [in]    Input minute.
 *****************************************************************************/
void TimingVars_SetTMin(float32_t const tMin)
{
    timer.tMin = tMin;
}

/******************************************************************************
 * @brief Get minutes.
 * 
 * @return float32_t 
 *****************************************************************************/
float32_t TimingVars_GetTMin(void)
{
    return(timer.tMin);
}

#ifdef DISPLAY_DIAGNOSTIC_MSG
#ifdef INS_OFFLINE
void TimingVars_DisplayTimerVars(int32_t timeStep)
{
    std::cout << "Iter " << timeStep << ": " << timer.secondCntr << ", "
                                             << timer.tenHertzCntr << ", "
                                             << timer.subFrameCntr << ", (t = "
                                             << TimingVars_GetTime() << ")\n";
}

void TimingVars_DiagnosticMsg( std::string msg )
{
    std::cout << msg << " (t = " << TimingVars_GetTime()
                     << ", k = " << TimingVars_GetTimeStep()
                     << ")\n";
}
#else
void TimingVars_DisplayTimerVars(int32_t timeStep)
{
    DEBUG_INT("Iter", timeStep);
    DEBUG_INT(",", timer.secondCntr);
    DEBUG_INT(",", timer.tenHertzCntr);
    DEBUG_INT(",", timer.subFrameCntr);
    DEBUG_INT(",(", TimingVars_GetTime());
    DEBUG_STRING(")\r\n,");
}

void TimingVars_DiagnosticMsg( char *msg )
{
    DEBUG_STRING(msg);
    DEBUG_INT(" (t = ", TimingVars_GetTime());
    DEBUG_INT(", k = ", TimingVars_GetTimeStep());
    DEBUG_STRING(")\r\n");
}
#endif
#endif

/******************************************************************************
 * @brief Get time step.
 * 
 * @return uint32_t 
 *****************************************************************************/
uint32_t TimingVars_GetTimeStep(void)
{
    return( ( 100U * timer.secondCntr ) +
            ( 10U * (uint32_t)timer.tenHertzCntr ) +
            ( 1U * (uint32_t)timer.subFrameCntr ) );
}

/******************************************************************************
 * @brief Initialize timing variables
 * 
 *****************************************************************************/
void Initialize_Timing(void)
{
    // InitTimingVars.m
    //
    // Purpose: Initialize the timing variables that control operation of the
    //          Extended Kalman Filter
    //
    // Output: secondCntr -- increments once per simulated second (1 Hz); not reset.
    //         tenHertzCntr -- increments ten times per second (10 Hz); reset
    //                         once it reached 10.
    //         subFrameCntr -- increments one hundred times per second (100 Hz);
    //                         reset once it reaches 10.
    //         time -- time generated from the three counters
    //

    // Initialize timing variables
    timer.secondCntr = 0U;
    timer.tenHertzCntr = 0U;
    timer.subFrameCntr = -1;
    timer.basicFrameCounter = 0U; // increments at 100 or 200 Hz and reset after 1 second

    timer.time = 0.0F;
    timer.tMin = 0.0F;

    // toggles between 0 and 1 at 200 Hz (currently used in firmware)
    timer.oneHundredHertzFlag = 0U;

    // Override execution rate of taskDataAcquisition() based on the configuration
    timer.dacqFrequency = (uint32_t)DACQ_200_HZ;     // default
}

/*******************************************
 * @brief 
 * 
 * @param freq ==
********************************************/
void TimingVars_dacqFrequency(uint32_t const freq)
{
    timer.dacqFrequency = freq;
}
