/******************************************************************************
 * @file SelectState.h
 * @author Joe Motyka
 * @brief 
 * @version 1.0
 * @date 2020-09-09
 * 
 * @copyright Copyright (c) 2020
 * 
 *****************************************************************************/

#ifndef SELECTSTATE_H
#define SELECTSTATE_H

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************
 * @brief Run for a prescribed period to let the sensors settle.
 *  
 *****************************************************************************/
void StabilizeSystem(void);

/******************************************************************************
 * @brief Initialize the algorithm by collecting sensor data for
 *        a prescribed period and averaging it.
 *  
 *****************************************************************************/
void InitializeAttitude(void);

/******************************************************************************
 * @brief Transition from high-gain to low-gain.  Only check
 *        is that the bias isn't greater than 10 deg/sec.
 *  
 *****************************************************************************/
void HG_To_LG_Transition_Test(void);

/******************************************************************************
 * @brief This logic is only called upon transition to INS from LG_AHRS then it
 *        is not called unless the algorithm reverts back to HG_AHRS, which will
 *        cause the system to pass through LG_AHRS on its way to INS.
 *  
 *****************************************************************************/
void LG_To_INS_Transition_Test(void);

/******************************************************************************
 * @brief Drop back to LG AHRS operation if...
 *          1) GPS drops out for more than 3 seconds
 *          2) magnetometer data not available AND at rest too long
 *          3) magnetic alignment being performed
 *  
 *****************************************************************************/
void INS_To_AHRS_Transition_Test(void);

/******************************************************************************
 * @brief Dynamic motion logic:
 *          0) When dynamicMotion is FALSE, remain in high-gain AHRS (do not
 *             decrement counter in 'HG_To_LG_Transition_Test').
 *          1) If dynamicMotion is selected then proceed to other filter states
 *             upon timeout (else, stay in HG mode).
 *          2) When in LG or INS mode... if dynamicMotion is set FALSE then
 *             transition to HG AHRS.
 *          3) Once dynamicMotion is reset TRUE (by user), the system should
 *             begin transition to LG AHRS as if beginning from nominal startup.
 *  
 *****************************************************************************/
void DynamicMotion(void);

#ifdef UNIT_TEST
#include "wrappers.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* SELECTSTATE_H */
