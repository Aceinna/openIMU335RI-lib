/** ***************************************************************************
 * @file   timer.c
 * @Author
 * @copyright (c) 2019 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * sensor data acquisition task runs at 200Hz, gets the data for each sensor
 * and applies available calibration
 ******************************************************************************/
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

#include <stdlib.h>
#include "stm32f4xx_conf.h"
#include "osapi.h"
#include "halAPI.h"
#include "sensorsAPI.h"


static uint64_t solutionTstampFull = 0U;
static uint32_t NewTick    = 0U;
static float64_t rollingTimer = 0.0;
static BOOL dacqStarted = FALSE;

static TIM_TypeDef* TIM_2;
static TIM_TypeDef* TIM_5;

void    TIMER_DacqStarted()
{
    dacqStarted = TRUE;
}

/*******************************************
 * @brief
 *   
 * @return uint64_t 
********************************************/
uint64_t TIMER_GetCurrTimeStamp()
{
    static uint64_t  currentTstamp = 0U;
    ENTER_CRITICAL();
    uint32_t const cur =  TIM_GetCounter(TIM_5);
    static  uint32_t prevTim5Val  = 0U;
    if(cur < prevTim5Val){
        currentTstamp += 0x0000000100000000ULL;
    }
    currentTstamp &= 0xFFFFFFFF00000000;
    currentTstamp |= cur;
    prevTim5Val   = cur;
    uint64_t const res  = currentTstamp;
    EXIT_CRITICAL();
    return res;
}

// value in microseconds
void  TIMER_Init()
{
    TIM_2 = TIM_GetInstancePtr(TIM_NUM_2);
    TIM_5 = TIM_GetInstancePtr(TIM_NUM_5);
}

/** ***************************************************************************
 * @brief 
 * @return time
 ******************************************************************************/
uint64_t TIMER_GetDacqTimeStamp()
{
    return solutionTstampFull;
}

/** ***************************************************************************
 * @brief After timeout, TIM2_IRQHandler, resets the rate-sensor data buffers.
 *        The individual sensor interrupts populates the data buffers when the
 *        sensors indicate 'data-ready'.
 *
 *        Using 32 bit TIM2. Get a sync signal from GPS or the user on TIM2.
 *        use that to trim the TIM2 reload value to have a timer that is the
 *        same rate as the desired outputDataRate
 *
 * @param interval 
 *  
 ******************************************************************************/
void TIMER_SetDacqInterval(uint32_t const interval)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;
    float64_t period;

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );

    TIM_Cmd(TIM_2, (uint32_t)DISABLE);       ///< TIM disable counter
    TIM_ClearITPendingBit(TIM_2, TIM_IT_Update);
    /// Set the timer interrupt period (counter value at interrupt).  Alter this
    /// to sync with a 1PPS or 1kHz signal. ==> SystemCoreClock = 120MHz
    period = (float64_t)( SystemCoreClock ); // >> 1; ///< period = 120 MHz / 2 = 60 MHz
    period = (0.5 * period) / (float64_t)(float32_t)interval;    ///< = period / ODR ==> 60 MHz / 500 Hz = 120,000

    /// Time base configuration
    TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );
    TIM_TimeBaseStructure.TIM_Period      = (uint32_t)(float32_t)(period+0.5);
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM_2, &TIM_TimeBaseStructure );
    TIM_ARRPreloadConfig( TIM_2, ENABLE );

    /// Enable the TIM2 global interrupt, TIM2_IRQn
    NVIC_InitStructure.NVIC_IRQChannel                   = (uint8_t)TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x3U;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0U;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

    TIM_ClearITPendingBit(TIM_2, TIM_IT_Update);
    TIM_ITConfig( TIM_2, TIM_IT_Update, ENABLE ); ///< TIM Interrupts enable
    /// Ensure an UEV generates an interrupt by deselecting the URS bit in TIM2_CR1
    TIM_UpdateRequestConfig(TIM_2, TIM_UpdateSource_Global);
    TIM_Cmd( TIM_2, ENABLE ); ///< TIM enable counter
}


/** ***************************************************************************
 * @brief
 *  
 ******************************************************************************/
void TIMER_InitOSTick()
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    float64_t period;

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM5, ENABLE );

    TIM_Cmd( TIM_5, DISABLE);       ///< TIM disable counter
    TIM_ClearITPendingBit(TIM_5, TIM_IT_Update);
    TIM_ITConfig( TIM_5, TIM_IT_Update, DISABLE); 

    period = (float64_t)( SystemCoreClock);   // ticks
    period = (0.5 * period) /1000000.0;        // 1 uS

    /// Time base configuration
    TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );
    TIM_TimeBaseStructure.TIM_Prescaler   = (uint16_t)(uint32_t)(float32_t)period;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit( TIM_5, &TIM_TimeBaseStructure );
    TIM_UpdateRequestConfig(TIM_5, TIM_UpdateSource_Global);
    
    TIM_Cmd( TIM_5, ENABLE ); ///< TIM enable counter
}

/** ***************************************************************************
 * @brief
 ******************************************************************************/

uint64_t  tstamps[16];
int tstampIdx = 0;
int sErr = 0;
uint32_t numCycles = 0;

extern BOOL    samplingInProgress;
extern BOOL    tickFired;
static uint32_t divv = 0;

void TIM2_IRQHandler(void)
{

    OSEnterISR();

    numCycles++;

    if(dacqStarted && divv != 4){
        if(!samplingInProgress){
            // read counts here
            sens_GetNumSamplesInFifo(divv, FALSE);
        }else{
            tickFired = TRUE;
        }
    }
    
    divv++;

    if(divv >= 5 ){
        divv = 0;
    NewTick = 1U;
    }
    // reset the interrupt flag
    TIM_ClearITPendingBit(TIM_2, TIM_IT_Update);

    OSExitISR();

}


/** ***************************************************************************
 * @brief
 * @return tick
 ******************************************************************************/
uint16_t TIMER_GetRollingCount()
{ 
    return (uint16_t)(uint32_t)(int32_t)(float32_t)rollingTimer; 
}

/** ***************************************************************************
 * @brief
 * @return tick
 ******************************************************************************/
int32_t TIMER_WaitForNewDacqTick()
{
//     sampleSum[0] = 0;
//     sampleSum[1] = 0;
//     sampleSum[2] = 0;

     while(NewTick == 0U){};
     NewTick = 0U;
     
     solutionTstampFull = TIMER_GetCurrTimeStamp(); // Maitenance of system us timer
     rollingTimer      += 327.68;    // 65536 per second
     return 0;
}


/** ***************************************************************************
 * @brief
 * @return tick
 ******************************************************************************/
BOOL TIMER_IsDacqOverrun()
{
    return NewTick == 1U;
}


/** ***************************************************************************
 * @brief After timeout, TIM2_IRQHandler, resets the rate-sensor data buffers.
 *        The individual sensor interrupts populates the data buffers when the
 *        sensors indicate 'data-ready'.
 *
 *        Using 32 bit TIM2. Get a sync signal from GPS or the user on TIM2.
 *        use that to trim the TIM2 reload value to have a timer that is the
 *        same rate as the desired outputDataRate
 *
 * @param Delay ==  
 *  
 ******************************************************************************/
void TIMER_DelayMs(uint32_t const Delay)
{
    uint64_t currTime;
    uint64_t const endTime   = (TIMER_GetCurrTimeStamp()/1000U) + Delay;    

    while(1){
        currTime = TIMER_GetCurrTimeStamp()/1000U;
        if(currTime >= endTime){
            break;
        }
    }
}



