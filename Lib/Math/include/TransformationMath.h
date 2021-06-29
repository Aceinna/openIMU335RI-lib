/*
 * File:   TransformationMath.h
 * Author: joemotyka
 *
 * Created on May 8, 2016, 12:35 AM
 */

#ifndef TRANSFORMATIONMATH_H
#define TRANSFORMATIONMATH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "GlobalConstants.h"

static float64_t const E_MAJOR                = 6.378137e+6;             // semi major axis a
static float64_t const E_MINOR                = 6.356752314245216e+6;    // semi minor axis b
static float64_t const E_ECC                  = 0.08181919084255;        // first eccentricity
static float64_t const E_ECC_SQ               = 0.006694379990130;       // (first eccentricity)^2
static float64_t const E_MAJOR_SQ             = 4.068063159076900e+013;  // E_MAJOR^2
static float64_t const E_MINOR_SQ             = 4.040829998466191e+013;  // E_MINOR2
static float64_t const EP_SQ                  = 4.284131151324081e+004;  // (E_MAJOR2 - E_MINOR2)/E_MINOR
static float64_t const E_MINOR_OVER_MAJOR_SQ  = 0.993305620009870;       // E_MAJOR_SQ / E_MINOR_SQ
static float64_t const E_MAJOR_OVER_MINOR_SQ  = 1.006739496742265;       // E_MINOR_SQ / E_MAJOR_SQ
static float64_t const E_MAJOR_OVER_MINOR     = 1.003364089820971;       // E_MAJOR / E_MINOR
static float64_t const E_ECC_SQxE_MAJOR       = 42697.67270710779;

/******************************************************************************
 * @brief Compute unit gravity vector in the body frame from accel measurement.
 *        Accelerometer measurement = -gravity when there is no linear acceleration.
 *        If acceleromter measurement is [0; 0; -1], the unit gravity vector should
 *        be [0; 0; 1].
 * 
 * @param accel             [in]    Accelerometer measurement. [g] or [m/s^2].
 * @param unitGravityVector [out]   Unit gravity vector in the body frame.
 *  
 *****************************************************************************/
void UnitGravity(real accel[], real unitGravityVector[]);

/******************************************************************************
 * @brief Compute pitch and roll angle from unit gravity vector in the body frame.
 *
 * @param unitGravityVector [in]    unit gravity vector in the body frame.
 * @param eulerAngles       [out]   Euler angles in order [roll pitch yaw].
 *                                  roll is put in eulerAngle[0],
 *                                  and pitch in eulerAngle[1].
 *  
 *****************************************************************************/
void UnitGravityToEulerAngles(real unitGravityVector[], real eulerAngles[]);

/******************************************************************************
 * @brief Compute yaw angle from unit gravity vector and magnetic measurement.
 *        The unit gravity vector in the body frame is used to project the magnetic
 *        measurement onto a perpendicular frame. The projected x and y component
 *        of the magnetic measurement in this perpendicular frame are used to compute
 *        the yaw angle.
 * 
 * @param unitGravityVector [in]    Unit gravity vector in the body frame.
 * @param magFieldVector    [in]    Magnetic vector in the body frame.
 * @return Yaw angle. [rad].
 *****************************************************************************/
real UnitGravityAndMagToYaw(real unitGravityVector[], real magFieldVector[]);

/******************************************************************************
 * @brief Compute yaw angle from pitch, roll and magnetic measurement.
 *        The pitch and roll angles are used to project the magneticmeasurement
 *        onto a perpendicular frame. The projected x and y component of the
 *        magnetic measurement in this perpendicular frame are used to compute
 *        the yaw angle.
 * 
 * @param roll              [in]    Roll angle. [rad].
 * @param pitch             [in]    Pitch angle. [rad].
 * @param magFieldVector    [in]    Magnetic vector in the body frame.
 * @return Yaw angle. [rad].
 *****************************************************************************/
real RollPitchAndMagToYaw(real const roll, real const pitch, real magFieldVector[]);

/******************************************************************************
 * @brief Limit angle error to be [-180, 180]deg.
 * 
 * @param aErr  [in]    Angle. [deg].
 * @return aErr within [-180, 180]deg.
 *****************************************************************************/
real AngleErrDeg(real aErr);

/******************************************************************************
 * @brief Limit angle error to be [-PI, PI].
 * 
 * @param aErr  [in]    Angle. [rad].
 * @return aErr within [-2*PI, 2*PI].
 *****************************************************************************/
real AngleErrRad(real aErr);

/******************************************************************************
 * @brief Calculate NED relative position of two ECEF positions.
 * 
 * @param rECEF_Init    [in]    Initial ECEF position. [m]
 * @param rECEF         [in]    Current ECEF position. [m].
 * @param R_NinE        [in]    Rotation matrix from NED to ECEF
 * @param dr_N          [out]   Current position w.r.t the initial position in 
 *                              NED. [m]
 *  
 *****************************************************************************/
void ECEF_To_Base(float64_t rECEF_Init[], float64_t rECEF[],
                  real R_NinE[3][3], real dr_N[]);

/******************************************************************************
 * @brief Given an inital ECEF position and an position relative to the initial
 *        poistion in NED, convert the relative poistion to ECEF position.
 *        rECEF = rECEC_Init + (NED_to_ECEF) * rNED.
 * 
 * @param r_N           [in]    Relative position in NED. [m]
 * @param rECEF_Init    [in]    "Absolute" position in ECEF. [m].
 * @param R_NinE        [in]    Rotation matrix from NED to ECEF.
 * @param rECEF         [out]   r_N plus rECEV_Init in ECEF. [m]
 *  
 *****************************************************************************/
void PosNED_To_PosECEF(real r_N[],
                       float64_t rECEF_Init[], //BaseECEF,
                       real R_NinE[3][3],
                       float64_t rECEF[]);

/******************************************************************************
 * @brief Convert ECEF position to [latitude, longitude, altitude].
 * 
 * @param llaDeg   [out]   [latitude, longitude, altitude] in [deg, deg, m].
 * @param ecef_m    [in]    ECEF position. [m].
 *   
 *****************************************************************************/
void ECEF_To_LLA(float64_t llaDeg[], float64_t ecef_m[]);

#ifdef INS_OFFLINE
void printMtx(const float32_t* const a, int32_t const m, int32_t const n);
void printVec(const float32_t* const v, int32_t const n);
#endif // INS_OFFLINE


#ifdef __cplusplus
}
#endif

#endif /* TRANSFORMATIONMATH_H */



