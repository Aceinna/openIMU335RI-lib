/* 
 * File:   QuaternionMath.h
 * Author: joemotyka
 *
 * Created on May 7, 2016, 5:03 PM
 */

#ifndef QUATERNIONMATH_H
#define QUATERNIONMATH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "GlobalConstants.h"

/******************************************************************************
 * @brief Convert Euler angles (roll, pitch and yaw) to quaternions according to
 *        the ZYX rotation sequence.
 * 
 * @param EulerAngles   [in]    Euler angles. [rad].
 * @param Quaternion    [out]   Quaternion.
 *  
 *****************************************************************************/
void EulerAnglesToQuaternion(real EulerAngles[], real Quaternion[]);

/******************************************************************************
 * @brief Normalize a quaternion.
 * 
 * @param Quat  [in/out]    Quaternion.
 *   
 *****************************************************************************/
void QuatNormalize(real Quat[]);

/******************************************************************************
 * @brief Convert quaternion to Euler angles (roll, pitch and yaw) according to
 *        the ZYX rotation sequence.
 * 
 * @param EulerAngles   [out]   Euler angles. [rad].
 * @param Quaternion    [in]    Quaternion.
 *   
 *****************************************************************************/
void QuaternionToEulerAngles(real EulerAngles[], real Quaternion[]);

/******************************************************************************
 * @brief Convert quaternion to direction cosine matrix.
 * 
 * @param Quaternion    [in]    Quaternion.
 * @param R321          [out]   Direction cosine matrix.
 *   
 *****************************************************************************/
void QuaternionToR321(real Quaternion[], real R321[3][3]);

#ifdef __cplusplus
}
#endif

#endif /* QUATERNIONMATH_H */

