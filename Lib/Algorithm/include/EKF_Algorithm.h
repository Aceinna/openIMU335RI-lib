/******************************************************************************
 * @file EKF_Algorithm.h
 * @author Xiaoguang Dong (xgdong@aceinna.com)
 * @brief 
 * @version 0.1
 * @date 2020-09-09
 * 
 * @copyright Copyright (c) 2020
 * 
 *****************************************************************************/

#ifndef EKF_ALGORITHM_H
#define EKF_ALGORITHM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "GlobalConstants.h"
#include "StateIndices.h"
#include "gpsAPI.h"   // For gpsDataStruct_t in EKF setter
#include "odoAPI.h"
#include "Indices.h"


// Global Kalman Filter structure
typedef struct {
    // States
    real Velocity_N[NUM_AXIS];
    real Position_N[NUM_AXIS];
    real quaternion[4];
    real quaternion_Past[4];
    real rateBias_B[NUM_AXIS];
    real accelBias_B[NUM_AXIS];

    // Prediction variables: P = FxPxFTranspose + Q
    real F[NUMBER_OF_EKF_STATES][NUMBER_OF_EKF_STATES];
    real P[NUMBER_OF_EKF_STATES][NUMBER_OF_EKF_STATES];
    real Q[NUMBER_OF_EKF_STATES];
    real Qq[6]; /* The process cov matrix of quaternion should be 4x4.
                 * Its 4 diagonal terms are stored in Q. 
                 * Its off-diagonol terms are stored in Qq. Because the matrix
                 * is symmetric, only 6 off-diagonal terms need stored.
                 */

    real correctedRate_B[NUM_AXIS];     // [rad/s]
    real correctedAccel_B[NUM_AXIS];    // [m/s/s]
    real linearAccel_B[NUM_AXIS];       // [m/s/s], linear acceleration in body frame, used to detect drive position

    /* Algorithm results. Velocity states are directly used as results for output.
     * The following two are calculated from state
     */
    real eulerAngles[NUM_AXIS];
    float64_t llaDeg[NUM_AXIS];

    // measurements
    real R_BinN[3][3];                  // convert body to NED
    real Rn2e[3][3];                    // Coordinate tranformation matrix from NED to ECEF
    real measuredEulerAngles[3];        // Euler angles measurements
    real rGPS_N[3];                     // current IMU position w.r.t rGPS0_E in NED.
    float64_t rGPS0_E[3];                  // Initial IMU ECEF position when first entering INS state.
    float64_t rGPS_E[3];                   // current IMU ECEF position

    // Update variables: S = HxPxHTranspose + R
    real nu[9];
    real H[3][NUMBER_OF_EKF_STATES];
    real R[9];
    real K[NUMBER_OF_EKF_STATES][3];
    real stateUpdate[NUMBER_OF_EKF_STATES];
    real deltaP_tmp[NUMBER_OF_EKF_STATES][NUMBER_OF_EKF_STATES];

    // The following two are used in more than one functions, so they are pre-computed.
    real wTrueTimesDtOverTwo[NUM_AXIS];
    real turnSwitchMultiplier;

    // saved states when pps comes in
    real ppsP[NUMBER_OF_EKF_STATES][NUMBER_OF_EKF_STATES];
    real phi[NUMBER_OF_EKF_STATES][NUMBER_OF_EKF_STATES];
    real dQ[NUMBER_OF_EKF_STATES][NUMBER_OF_EKF_STATES];
    real ppsPosition_N[NUM_AXIS];
    real ppsVelocity_N[NUM_AXIS];
    real ppsEulerAngles[NUM_AXIS];
    uint32_t ppsITow;
} KalmanFilterStruct;

extern KalmanFilterStruct gKalmanFilter;

/* Global Algorithm structure  */
typedef struct {
    // Sensor readings in the body-frame (B)
    real accel_B[NUM_AXIS];         // [m/s/s]
    real angRate_B[NUM_AXIS];       // [rad/s]
    real magField_B[NUM_AXIS];      // [G]

    // GPS information
    uint32_t itow;
    float64_t llaAnt[3];               // Antenna Lat, Lon, ellipsoid Altitude, [rad, rad, meter]
    float64_t vNedAnt[NUM_AXIS];       // Antenna NED velocity, [m/s, m/s, m/s]
    float64_t lla[3];                  // IMU Lat, Lon, ellipsoid Altitude, [rad, rad, meter]
    float64_t vNed[3];                 // IMU NED velocity, [m/s, m/s, m/s]
    float32_t geoidAboveEllipsoid;      // [m]
    real trueCourse;                // Antenna heading, [deg]
    real rawGroundSpeed;            // IMU ground speed, calculated from vNed, [m/s]
    float32_t GPSHorizAcc;              // [m]
    float32_t GPSVertAcc;               // [m]
    float32_t HDOP;
    uint8_t gpsFixType;             // Indicate if this GNSS measurement is valid
    uint8_t numSatellites;          /* Num of satellites in this GNSS measurement.
                                     * This is valid only when there is gps udpate.
                                     */
    uint8_t gpsUpdate;                 // Indicate if GNSS measurement is updated.

    // odometer
    odoDataStruct_t odo;

    // 1PPS from GNSS receiver
    BOOL ppsDetected;
} EKF_InputDataStruct;

extern EKF_InputDataStruct gEKFInput;


/* Global Algorithm structure  */
typedef struct {
    // Algorithm states (15 states)
    float64_t            position_N[NUM_AXIS];
    float64_t            velocity_N[NUM_AXIS];
    float64_t            quaternion_BinN[4];
    float64_t            angRateBias_B[NUM_AXIS];
    float64_t            accelBias_B[NUM_AXIS];
    
    float64_t            llaDeg[NUM_AXIS];

    // Derived variables
    float64_t            eulerAngs_BinN[NUM_AXIS];
    float64_t            corrAngRates_B[NUM_AXIS];
    float64_t            corrAccel_B[NUM_AXIS];

    // Operational states
    uint8_t           opMode;
    uint8_t           turnSwitchFlag;
    uint8_t           linAccelSwitch;
} EKF_OutputDataStruct;

extern EKF_OutputDataStruct gEKFOutput;


/******************************************************************************
 * @brief Initialize Kalman filter parameters of the INS app
 * 
 *  
 *****************************************************************************/
void InitINSFilter(void);

#ifdef __cplusplus
}
#endif

#endif /* EKF_ALGORITHM_H */

