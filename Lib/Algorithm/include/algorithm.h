/******************************************************************************
 * @file algorithm.h
 * @author Xiaoguang Dong (xgdong@aceinna.com)
 * @brief 
 * @version 0.1
 * @date 2020-09-09
 * 
 * @copyright Copyright (c) 2020
 * 
 *****************************************************************************/

#ifndef ALGORITHM_H
#define ALGORITHM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "GlobalConstants.h"
#include "ProcessODO.h"

// Define the algorithm states
enum {
    STABILIZE_SYSTEM    = 0,
    INITIALIZE_ATTITUDE = 1,
    HIGH_GAIN_AHRS      = 2,
    LOW_GAIN_AHRS       = 3,
    INS_SOLUTION        = 4
};


// Specify the minimum state times (in seconds)
static uint32_t const STABILIZE_SYSTEM_DURATION    = 360U;    // [msec]
static uint32_t const INITIALIZE_ATTITUDE_DURATION = 640U;    // ( 1.0 - 0.36 ) [msec]
static uint32_t const HIGH_GAIN_AHRS_DURATION      = 30000U;  // [msec]
static uint32_t const LOW_GAIN_AHRS_DURATION       = 30000U;  // [msec]

// Define heading initialization reliability
static uint8_t const HEADING_UNINITIALIZED  = 0U;
static uint8_t const HEADING_MAG            = 1U;
static uint8_t const HEADING_GNSS_LOW       = 2U;
static uint8_t const HEADING_GNSS_HIGH      = 3U;

typedef struct {
	uint32_t Stabilize_System;      // SAMPLING_RATE * 0.36
	uint32_t Initialize_Attitude;   // SAMPLING_RATE * ( 1.0 - 0.36 )
	uint32_t High_Gain_AHRS;        // 60.0 * SAMPLING_RATE
	uint32_t Low_Gain_AHRS;         // 30.0 * SAMPLING_RATE
} DurationStruct;

typedef struct {
	real positionError;
	real velocityError;
	real attitudeError;
} InnovationStruct;

typedef struct {
    uint32_t maxGpsDropTime;     // [msec]
    uint32_t maxReliableDRTime;  /* [msec] When GPS outage duration exceeds this limit,
                                 * the position and velocity will be reinitialized when GPS
                                 * is available again. Otherwise, the fusion algorithm will
                                 * gradually correct the position and velocity to the GPS.
                                 */
	int32_t Max_Rest_Time_Before_Drop_To_AHRS;   // [msec]
	int32_t Declination_Expiration_Time;         // [msec]

	uint32_t Free_Integration_Cntr;              // [count]

	real accelSwitch;
	uint32_t linAccelSwitchDelay;
	uint32_t rateIntegrationTime;
	InnovationStruct       Innov;
} LimitStruct;

/// specifying how the user sets up the device algorithm
struct algoBehavior_BITS {                  // bits   description
    uint32_t freeIntegrate : 1;             // 0
    uint32_t useMag : 1;                    // 1
    uint32_t useGPS : 1;                    // 2
    uint32_t useOdo : 1;                    // 3
    uint32_t enableStationaryLockYaw : 1;   // 4
    uint32_t restartOnOverRange : 1;        // 5
    uint32_t dynamicMotion : 1;             // 6
    uint32_t enableImuStaticDetect : 1;     // 7
    uint32_t rsvd : 24;                     // 8:31
};

union AlgoBehavior
{
    uint32_t                 all;
	struct algoBehavior_BITS bit;
};

// Algorithm states
struct ALGO_STATUS_BITS
{
	uint32_t algorithmInit : 1;         // 0  algorithm initialization
	uint32_t highGain : 1;              // 1  high gain mode
	uint32_t attitudeOnlyAlgorithm : 1; // 2  attitude only algorithm
	uint32_t turnSwitch : 1;            // 3  turn switch
    uint32_t linearAccel : 1;           // 4  Linear acceleration detected by difference from gravity
	uint32_t staticImu : 1;             // 5  zero velocity detected by IMU
	uint32_t gpsHeadingValid : 1;       /* 6  When GPS velocity is above a certain threshold,
                                         * this is set to TRUE, and GPS heading measurement
                                         * is used, otherwise, this is set to FALSE and magnetic
                                         * heading (if available) is used.
                                         */
	uint32_t stationaryYawLock : 1;     // 7  Yaw is locked when unit is static
    uint32_t ppsAvailable : 1;          /* 8.
                                         * This will be initialized as FALSE.
                                         * This will become TRUE when PPS is detected for the first time.
                                         * This will become FALSE when PPS has not been detected for more than 2sec.
                                         * This will become FALSE when GNSS measurement is refrshed and used.
                                         */
	uint32_t rsvd : 7;                  // 9:15
};

typedef union ALGO_STATUS
{
	uint32_t                all;
	struct ALGO_STATUS_BITS bit;
} AlgoStatus;

extern AlgoStatus gAlgoStatus;

typedef struct
{
    real arw;                       // [rad/sqrt(s)], gyro angle random walk
    real sigmaW;                    // [rad/s], gyro noise std
    real biW;                       // [rad/s], gyro bias instability
    real maxBiasW;                  // [rad/s], max possible gyro bias
    real vrw;                       // [m/s/sqrt(s)], accel velocity random walk
    real sigmaA;                    // [m/s/s], accel noise std
    real biA;                       // [m/s/s], accel bias instability
    real maxBiasA;                  // [m/s/s], max possible accel bias
} IMU_SPEC;

typedef struct 
{
    real staticVarGyro;             // [rad/s]^2
    real staticVarAccel;            // [m/s/s]^2
    real maxGyroBias;               // [rad/s]
    real staticGnssVel;             // [m/s]
    real staticNoiseMultiplier[3];  /* Use IMU noise level and gyro output to detect static period.
                                     * The nominal noise level and max gyro bias of an IMU is defined in
                                     * SensorNoiseParameters.h. These parameters are determined by IMU
                                     * output when static and are hightly related to ARW and VRW.
                                     * When IMU is installed on a vehicle, its noise level when
                                     * vehicle is static could be higher than the nominal noise
                                     * level due to vibration. This setting is used to scale
                                     * the nominal noise level and gyro bias for static detection.
                                     * [scale_gyro_var, scale_accel_var, scale_gyro_bias]
                                     */
} STATIC_DETECT_SETTING;


/* Global Algorithm structure  */
typedef struct {
    uint32_t    itow;
    uint32_t    dITOW;

    // control the stage of operation for the algorithms
    uint32_t    stateTimer;
    uint8_t     state;			        // takes values from HARDWARE_STABILIZE to INIT_ATTITUDE to HG_AHRS

    uint8_t insFirstTime;
    uint8_t headingIni;
    uint8_t applyDeclFlag;

    uint32_t timeOfLastSufficientGPSVelocity;
    uint32_t timeOfLastGoodGPSReading;

    real filteredYawRate;				// Yaw-Rate (Turn-Switch) filter

    /* The following variables are used to increase the Kalman filter gain when the
     * acceleration is very close to one (i.e. the system is at rest)
     */
    uint32_t linAccelSwitchCntr;
    uint8_t linAccelSwitch;             // TRUE if unit is stationary, FALSE if moving

    uint8_t linAccelLPFType;
    BOOL useRawAccToDetectLinAccel;
    BOOL useRawRateToPredAccel;
    real coefOfReduceQ;

    uint32_t callingFreq;
    real    dt;
    real    dtOverTwo;
    real    dtSquared;
    real    sqrtDt;

    volatile uint32_t timer;  			// timer since power up (ms)
    volatile uint16_t counter;			// inc. with every continuous mode output packet

    union   AlgoBehavior Behavior;
    real    turnSwitchThreshold;		// 0, 0.4, 10 driving, 1 flying [deg/sec]   0x000d

    real leverArmB[3];					// Antenna position w.r.t IMU in vehicle body frame
    real pointOfInterestB[3];			// Point of interest position w.r.t IMU in vehicle body frame

    BOOL velocityAlwaysAlongBodyX;      // enable zero velocity update

    IMU_SPEC imuSpec;                   // IMU specifications
    STATIC_DETECT_SETTING staticDetectParam;    // params used for static detection         

    DurationStruct    Duration;
    LimitStruct       Limit;

    // Odometer.
    OdoAlgoCfgStruct_t odoCfg;
    OdoStatusStruct    odoStatus;
} AlgorithmStruct;

extern AlgorithmStruct gAlgorithm;

/******************************************************************************
 * @brief Update IMU specs.
 * 
 * @param rwOdr     [in]    At what sampling frequency, the angle random walk and
 *                          the velocity random walk are calculated.
 *                          In units of [Hz].
 * @param arw       [in]    Angle random walk. In units of [rad/sqrt(s)].
 * @param biw       [in]    Gyro bias instability. In units of [rad/s].
 * @param maxBiasW  [in]    Max possible gyro bias. In untis of [rad/s].
 * @param vrw       [in]    Velocity random walk. In untis of [m/s/sqrt(s)].
 * @param bia       [in]    Accelerometer bias instability. In units of [m/s/s].
 * @param maxBiasA  [in]    Max possible accel bias. In units of [m/s/s].
 *  
 *****************************************************************************/
void UpdateImuSpec(real rwOdr, real arw, real biw, real maxBiasW,
                   real vrw, real bia, real maxBiasA);

/******************************************************************************
 * @brief Initialize the algorithm.
 * 
 * @param callingFreq   [in]    Calling frequency. In units of [Hz]. For now,
 *                              only 100Hz and 200Hz are supported.
 * @param imuType       [in]    Specify the IMU type to choose different set of
 *                              IMU specifications.
 *  
 *****************************************************************************/
void InitializeAlgorithmStruct(uint32_t callingFreq, enumIMUType const imuType);

/******************************************************************************
 * @brief Get the status of the algorithm.
 * 
 * @param algoStatus    [out]  Status of the algorithm. 
 *  
 *****************************************************************************/
void GetAlgoStatus(AlgoStatus* const algoStatus);

/******************************************************************************
 * @brief Set the execution frequency of the algorithm.
 * 
 * @param freq  [in]    Execution frequency of the algorithm. In units of [Hz].
 *  
 *****************************************************************************/
void setAlgorithmExeFreq(uint32_t const freq);


/******************************************************************************
 * @brief Set the lever arm of the GNSS antenna w.r.t. the IMU.
 * 
 * @param leverArmBx    [in]    X component of the lever arm in the IMU body frame.
 *                              In units of [m].
 * @param leverArmBy    [in]    Y component of the lever arm in the IMU body frame.
 *                              In units of [m].
 * @param leverArmBz    [in]    Z component of the lever arm in the IMU body frame.
 *                              In units of [m].
 *  
 *****************************************************************************/
void setLeverArm( real leverArmBx, real leverArmBy, real leverArmBz );

/******************************************************************************
 * @brief Set position of the point of interest w.r.t. the IMU.
 * 
 * @param poiBx [in]    X component of the position of the point of interest
 *                      in the IMU body frame. In units of [m].
 * @param poiBy [in]    Y component of the position of the point of interest
 *                      in the IMU body frame. In units of [m].
 * @param poiBz [in]    Z component of the position of the point of interest
 *                      in the IMU body frame. In units of [m].
 *****************************************************************************/
void setPointOfInterest( real poiBx, real poiBy, real poiBz );

/******************************************************************************
 * @brief Enable/disable the magnetometer in the algorithm.
 * 
 * @param enable    [in]    True to enable, and FALSe to disable.
 *  
 *****************************************************************************/
void enableMagInAlgorithm(BOOL enable);

/******************************************************************************
 * @brief Enable/disable GPS in the algorithm.
 * 
 * @param enable    [in]    True to enable, and FALSe to disable.
 *  
 *****************************************************************************/
void enableGpsInAlgorithm(BOOL enable);

/******************************************************************************
 * @brief Enable/disable the odometer in the algorithm.
 * 
 * @param enable    [in]    True to enable, and FALSe to disable.
 *  
 *****************************************************************************/
void enableOdoInAlgorithm(BOOL enable);

/******************************************************************************
 * @brief Enable/disable yaw locking when being stationary in the algorithm.
 * 
 * @param enable    [in]    True to enable, and FALSe to disable.
 *  
 *****************************************************************************/
void enableStationaryLockYaw(BOOL enable);

/******************************************************************************
 * @brief Enable/disable stationary period detection using IMU in the algorithm.
 * 
 * @param enable    [in]    True to enable, and FALSe to disable.
 *  
 *****************************************************************************/
void enableImuStaticDetect(BOOL enable);

/******************************************************************************
 * @brief Enable/disable free integration in the algorithm.
 * 
 * @param enable    [in]    True to enable, and FALSe to disable.
 *  
 *****************************************************************************/
void enableFreeIntegration(BOOL const enable);

/******************************************************************************
 * @brief Check if the magnetometer is enabled in the algorithm.
 * 
 * @return True if enabled, FALSE if disabled.
 *****************************************************************************/
BOOL magUsedInAlgorithm();

/******************************************************************************
 * @brief Check if GPS is enabled in the algorithm.
 * 
 * @return True if enabled, FALSE if disabled.
 *****************************************************************************/
BOOL gpsUsedInAlgorithm(void);

/******************************************************************************
 * @brief Check if the odometer is enabled in the algorithm.
 * 
 * @return True if enabled, FALSE if disabled.
 *****************************************************************************/
BOOL odoUsedInAlgorithm(void);

/******************************************************************************
 * @brief Check if free integration is enabled in the algorithm.
 * 
 * @return True if enabled, FALSE if disabled.
 *****************************************************************************/
BOOL freeIntegrationEnabled();

/******************************************************************************
 * @brief Check if yaw locking when being stationary is enabled in the algorithm.
 * 
 * @return True if enabled, FALSE if disabled.
 *****************************************************************************/
BOOL stationaryLockYawEnabled();

/******************************************************************************
 * @brief Check if stationary period detection using IMU is enabled in the algorithm.
 * 
 * @return True if enabled, FALSE if disabled.
 *****************************************************************************/
BOOL imuStaticDetectEnabled();

/******************************************************************************
 * @brief Get the algorithm timer.
 * 
 * @return uint32_t 
 *****************************************************************************/
uint32_t getAlgorithmTimer();

/******************************************************************************
 * @brief Get the algorithm counter.
 * 
 * @return uint16_t 
 *****************************************************************************/
uint16_t getAlgorithmCounter();

/******************************************************************************
 * @brief Get the execution frequency of the algorithm.
 * 
 * @return uint32_t 
 *****************************************************************************/
uint32_t getAlgorithmFrequency();

/******************************************************************************
 * @brief Get the TOW (time of week) of the algorithm.
 * 
 * @return uint32_t 
 *****************************************************************************/
uint32_t getAlgorithmITOW();

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
void SetAccelSwitchDelay(uint32_t value);
void SetRateIntegrationTime(uint32_t value);
void SetLinAccelDetectMode(BOOL value);
void SetAccelPredictMode(BOOL value);
void SetCoefOfReduceQ(real value);
void SetOdoUserCfg(const OdoCfgStruct_t* const value);

#ifdef __cplusplus
}
#endif

#endif
