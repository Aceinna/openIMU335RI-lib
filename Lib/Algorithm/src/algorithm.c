/******************************************************************************
 * @file algorithm.c
 * @brief Top level algorithm configurations and functions.
 * All top-level algorithm configurations and functions are here, including
 * algorithm state, algorithm configurations, algorithm input and output.
 * @author Dong Xiaoguang
 * @date 2019.05.09
 * @version V1.0.0
 *-----------------------------------------------------------------------------
 * Change History
 * <Date>     | <Version> | <Author>       | <Description>
 * ----------------------------------------------------------------------------
 * 2019.05.09 | v1.0.0    | Dong Xiaoguang | Create file
 * ----------------------------------------------------------------------------
******************************************************************************/

#include <math.h>
#include <string.h>

#include "Indices.h"
#include "SensorNoiseParameters.h"
#include "algorithm.h"
#include "AlgorithmLimits.h"
#include "algorithmAPI.h"

AlgorithmStruct gAlgorithm;
AlgoStatus      gAlgoStatus;

/******************************************************************************
 * @brief Initialize the algorithm.
 *
 * @param callingFreq   [in]    Calling frequency. In units of [Hz]. For now,
 *                              only 100Hz and 200Hz are supported.
 * @param imuType       [in]    Specify the IMU type to choose different set of
 *                              IMU specifications.
 *  
 *****************************************************************************/
void InitializeAlgorithmStruct(uint32_t callingFreq, enumIMUType const imuType)
{
    memset(&gAlgorithm, 0, sizeof(AlgorithmStruct));

    //----------------------------algortihm config-----------------------------
    // The calling frequency drives the execution rate of the EKF and dictates
    //   the algorithm constants
    if(callingFreq == 0U){
        // IMU case
        callingFreq = (uint32_t)FREQ_200_HZ;
    }
    gAlgorithm.callingFreq = callingFreq;

    // Set dt based on the calling frequency of the EKF
    if (gAlgorithm.callingFreq == (uint32_t)FREQ_100_HZ)
    {
        gAlgorithm.dt = 0.01F;
        gAlgorithm.dITOW = 10U;
    }
    else if (gAlgorithm.callingFreq == (uint32_t)FREQ_200_HZ)
    {
        gAlgorithm.dt = 0.005F;
        gAlgorithm.dITOW = 5U;
    }
    else
    {
        while (1) {};
    }

    // Set up other timing variables
    gAlgorithm.dtOverTwo = (real)(0.5) * gAlgorithm.dt;
    gAlgorithm.dtSquared = gAlgorithm.dt * gAlgorithm.dt;
    gAlgorithm.sqrtDt = sqrtf(gAlgorithm.dt);

    // Set the algorithm duration periods
    gAlgorithm.Duration.Stabilize_System = STABILIZE_SYSTEM_DURATION / gAlgorithm.dITOW;
    gAlgorithm.Duration.Initialize_Attitude = INITIALIZE_ATTITUDE_DURATION / gAlgorithm.dITOW;
    gAlgorithm.Duration.High_Gain_AHRS = HIGH_GAIN_AHRS_DURATION / gAlgorithm.dITOW;
    gAlgorithm.Duration.Low_Gain_AHRS = LOW_GAIN_AHRS_DURATION / gAlgorithm.dITOW;

    // Set the initial state of the EKF
    gAlgorithm.state = (uint8_t)STABILIZE_SYSTEM;
    gAlgorithm.stateTimer = gAlgorithm.Duration.Stabilize_System;

    // Turn-switch variable
    gAlgorithm.filteredYawRate = 0.0F;

    // Tell the algorithm to apply the declination correction to the heading
    //  (at startup in AHRS, do not apply.  After INS becomes healthy, apply,
    //  even in AHRS, but this condition shouldn't last forever.  Question:
    //  how long to keep this set TRUE after GPS in invalid?)
    gAlgorithm.applyDeclFlag = (uint8_t)FALSE;

    gAlgorithm.insFirstTime = (uint8_t)TRUE;
    gAlgorithm.headingIni = (uint8_t)HEADING_UNINITIALIZED;

    //gAlgorithm.magAlignUnderway = FALSE; // Set and reset in mag-align code

    // Increment at 100 Hz in EKF_Algorithm; sync with GPS itow when valid.
    gAlgorithm.itow = 0U;

    // Limit is compared to ITOW.  Time must be in [msec].
    gAlgorithm.Limit.maxGpsDropTime = LIMIT_MAX_GPS_DROP_TIME * 1000U;
    gAlgorithm.Limit.maxReliableDRTime = LIMIT_RELIABLE_DR_TIME * 1000U;

    // Limit is compared to count (incremented upon loop through
    //   taskDataAcquisition).  Time must be in [count] based on ODR.
    gAlgorithm.Limit.Free_Integration_Cntr = gAlgorithm.callingFreq * LIMIT_FREE_INTEGRATION_CNTR;

    // Linear acceleration switch limits (level and time)
    gAlgorithm.Limit.accelSwitch = (real)(0.012);   // [g]
    gAlgorithm.Limit.linAccelSwitchDelay = 2U * gAlgorithm.callingFreq;
    gAlgorithm.Limit.rateIntegrationTime = 2U * gAlgorithm.callingFreq;

    // Innovation error limits for EKF states
    gAlgorithm.Limit.Innov.positionError = (real)270.0;
    gAlgorithm.Limit.Innov.velocityError = (real)27.0;
    gAlgorithm.Limit.Innov.attitudeError = (real)SIX_DEGREES_IN_RAD;

    // Five-hertz LPF (corresponding integer value found in PredictFunctions.c)
    // Replace with a function that computes the coefficients.  Value (below) will
    //   then be the cutoff frequency.
    gAlgorithm.linAccelLPFType = 1U;

    // Uing raw accel to detect linear acceleration has lower failure rate in small
    //  and smooth linear acceleration. But on some platform, there is large vibration,
    //  uing raw accel to detect linear acceleration will always detect linear accel.
    gAlgorithm.useRawAccToDetectLinAccel = TRUE;    // TRUE: raw accel, FALSE: filtered accel.

    // The gyro data normally has just a small bias after factory calibration
    // and the accuracy is good enough to detect linear acceleration.
    // However, the gyro_x with a big bias, and the rate integration time default
    // setting of 2 seconds combined to not be accurate enough predicting the
    // next acceleration measurement. So in most of situations,
    // corrected rate should be used to predict next accel.
    gAlgorithm.useRawRateToPredAccel     = FALSE;   // FALSE: corrected rate, TRUE: raw rate.

    // Coefficient of reducing Q.
    gAlgorithm.coefOfReduceQ             = 0.001F;

    // Set the turn-switch threshold to a default value in [deg/sec]
    gAlgorithm.turnSwitchThreshold = 6.0F;

	// default lever arm and point of interest
    gAlgorithm.leverArmB[X_AXIS] = 0.0F;
    gAlgorithm.leverArmB[Y_AXIS] = 0.0F;
    gAlgorithm.leverArmB[Z_AXIS] = 0.0F;
    gAlgorithm.pointOfInterestB[X_AXIS] = 0.0F;
    gAlgorithm.pointOfInterestB[Y_AXIS] = 0.0F;
    gAlgorithm.pointOfInterestB[Z_AXIS] = 0.0F;

    // For most vehicles, the velocity is always along the body x axis
    gAlgorithm.velocityAlwaysAlongBodyX = TRUE;

    // get IMU specifications
    switch (imuType)
    {
    case OpenIMU330:
    case OpenIMU335RI:
    case MTLT335:
        {
            //0.2deg/sqrt(Hr) = 0.2 / 60 * D2R = 5.8177640741e-05rad/sqrt(s)
            gAlgorithm.imuSpec.arw      = (real)5.82e-5; 
            gAlgorithm.imuSpec.sigmaW   = (real)((1.25 * 5.82e-5) / sqrt(1.0/RW_ODR));
            //1.5deg/Hr = 1.5 / 3600 * D2R = 7.272205093e-06rad/s
            gAlgorithm.imuSpec.biW      = (real)7.27e-6;
            gAlgorithm.imuSpec.maxBiasW = (real)MAX_BW;
            //0.04m/s/sqrt(Hr) = 0.04 / 60 = 6.67e-04 m/s/sqrt(s)
            gAlgorithm.imuSpec.vrw      = (real)6.67e-04;
            gAlgorithm.imuSpec.sigmaA   = (real)((1.25 * 6.67e-04) / sqrt(1.0/RW_ODR));
             //20ug = 20.0e-6g * GRAVITY
            gAlgorithm.imuSpec.biA      = (real)(20.0e-6 * GRAVITY);
            gAlgorithm.imuSpec.maxBiasA = (real)(MAX_BA);
        }
        break;
    case OpenIMU300ZI:
    case OpenIMU300RI:
    default:
        {
            gAlgorithm.imuSpec.arw = (real)ARW_300ZA;
            gAlgorithm.imuSpec.sigmaW = (real)((1.25 * ARW_300ZA) / sqrt(1.0/RW_ODR));
            gAlgorithm.imuSpec.biW = (real)BIW_300ZA;
            gAlgorithm.imuSpec.maxBiasW = (real)MAX_BW;
            gAlgorithm.imuSpec.vrw = (real)VRW_300ZA;
            gAlgorithm.imuSpec.sigmaA = (real)((1.25 * VRW_300ZA) / sqrt(1.0/RW_ODR));
            gAlgorithm.imuSpec.biA = (real)(BIA_300ZA);
            gAlgorithm.imuSpec.maxBiasA = (real)(MAX_BA);
        }
        break;
    }

    // default noise level multiplier for static detection
    gAlgorithm.staticDetectParam.staticVarGyro = (real)(gAlgorithm.imuSpec.sigmaW * gAlgorithm.imuSpec.sigmaW);
    gAlgorithm.staticDetectParam.staticVarAccel = (real)(gAlgorithm.imuSpec.sigmaA * gAlgorithm.imuSpec.sigmaA);
    gAlgorithm.staticDetectParam.maxGyroBias = gAlgorithm.imuSpec.maxBiasW;
    gAlgorithm.staticDetectParam.staticGnssVel = 0.2F;
    gAlgorithm.staticDetectParam.staticNoiseMultiplier[0] = 4.0F;
    gAlgorithm.staticDetectParam.staticNoiseMultiplier[1] = 4.0F;
    gAlgorithm.staticDetectParam.staticNoiseMultiplier[2] = 1.0F;

    gAlgorithm.Behavior.bit.dynamicMotion = 1U;

    //----------------------------algorithm states-----------------------------
    memset(&gAlgoStatus, 0, sizeof(gAlgoStatus));

    //----------------- Initialization of adding aiding signal-----------------
    gAlgorithm.odoStatus.bStaticODO      = FALSE;  // Initial status is in motion.
    gAlgorithm.odoStatus.bOverTime       = TRUE;
    gAlgorithm.odoCfg.maxReliableODTime  = 200U; // in [ms]

    gAlgorithm.odoCfg.signalSource       = NO_SOURCE;
    gAlgorithm.odoCfg.msgRate            = 0U;
    gAlgorithm.odoCfg.mountLocation      = STATIC_WRT_BODY;
    gAlgorithm.odoCfg.leverArmB[X_AXIS]  = 0.0F;
    gAlgorithm.odoCfg.leverArmB[Y_AXIS]  = 0.0F;
    gAlgorithm.odoCfg.leverArmB[Z_AXIS]  = 0.0F;
    if (NO_SOURCE != gAlgorithm.odoCfg.signalSource)
    {
        enableOdoInAlgorithm(TRUE);
        if (STATIC_WRT_BODY == gAlgorithm.odoCfg.mountLocation)
        {
            UpdateCoefOfReduceQ(&gAlgorithm.coefOfReduceQ, TRUE);
        }
    }
    else
    {
        enableOdoInAlgorithm(FALSE);
    }
}

/******************************************************************************
 * @brief Get the status of the algorithm.
 *
 * @param algoStatus    [out]  Status of the algorithm.
 *
 *****************************************************************************/
void GetAlgoStatus(AlgoStatus * const algoStatus)
{
    algoStatus->all = gAlgoStatus.all;
}

/******************************************************************************
 * @brief Set the execution frequency of the algorithm.
 *
 * @param freq  [in]    Execution frequency of the algorithm. In units of [Hz].
 *
 *****************************************************************************/
void setAlgorithmExeFreq(uint32_t const freq)
{
    gAlgorithm.callingFreq = freq;
}


/******************************************************************************
 * @brief Get the algorithm timer.
 *
 * @return uint32_t
 *****************************************************************************/
uint32_t getAlgorithmTimer()
{
    return gAlgorithm.timer;
}

/******************************************************************************
 * @brief Get the algorithm counter.
 *
 * @return uint16_t
 *****************************************************************************/
uint16_t getAlgorithmCounter()
{
    return gAlgorithm.counter;
}

/******************************************************************************
 * @brief Get the execution frequency of the algorithm.
 *
 * @return uint32_t
 *****************************************************************************/
uint32_t getAlgorithmFrequency()
{
    return gAlgorithm.callingFreq;
}

/******************************************************************************
 * @brief Get the TOW (time of week) of the algorithm.
 *
 * @return uint32_t
 *****************************************************************************/
uint32_t getAlgorithmITOW()
{
    return gAlgorithm.itow;
}

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
void setLeverArm( real const leverArmBx, real const leverArmBy, real const leverArmBz )
{
    gAlgorithm.leverArmB[0] = leverArmBx;
    gAlgorithm.leverArmB[1] = leverArmBy;
    gAlgorithm.leverArmB[2] = leverArmBz;
}

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
void setPointOfInterest(real const poiBx, real const poiBy, real const poiBz )
{
    gAlgorithm.pointOfInterestB[0] = poiBx;
    gAlgorithm.pointOfInterestB[1] = poiBy;
    gAlgorithm.pointOfInterestB[2] = poiBz;
}

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
void UpdateImuSpec(real const rwOdr, real const arw, real const biw, real const maxBiasW,
                   real const vrw, real const bia, real const maxBiasA)
{
    // Update IMU specifications
    real const tmp = sqrtf(1.0F / rwOdr);
    gAlgorithm.imuSpec.arw = arw;
    gAlgorithm.imuSpec.sigmaW = (real)((1.25 * arw) / tmp);
    gAlgorithm.imuSpec.biW = biw;
    gAlgorithm.imuSpec.maxBiasW = maxBiasW;
    gAlgorithm.imuSpec.vrw = vrw;
    gAlgorithm.imuSpec.sigmaA = (real)((1.25 * vrw) / tmp);
    gAlgorithm.imuSpec.biA = bia;
    gAlgorithm.imuSpec.maxBiasA = maxBiasA;

    // Update affected params related to zero velocity detection
    gAlgorithm.staticDetectParam.staticVarGyro = (real)(gAlgorithm.imuSpec.sigmaW * gAlgorithm.imuSpec.sigmaW);
    gAlgorithm.staticDetectParam.staticVarAccel = (real)(gAlgorithm.imuSpec.sigmaA * gAlgorithm.imuSpec.sigmaA);
    gAlgorithm.staticDetectParam.maxGyroBias = gAlgorithm.imuSpec.maxBiasW;
}

/******************************************************************************
 * @brief Enable/disable the magnetometer in the algorithm.
 *
 * @param enable    [in]    True to enable, and FALSe to disable.
 *
 *****************************************************************************/
void enableMagInAlgorithm(BOOL const enable)
{
    gAlgorithm.Behavior.bit.useMag = (uint32_t)enable;
}

/******************************************************************************
 * @brief Enable/disable GPS in the algorithm.
 *
 * @param enable    [in]    True to enable, and FALSe to disable.
 *
 *****************************************************************************/
void enableGpsInAlgorithm(BOOL const enable)
{
    gAlgorithm.Behavior.bit.useGPS = (uint32_t)enable;
}

/******************************************************************************
 * @brief Enable/disable the odometer in the algorithm.
 *
 * @param enable    [in]    True to enable, and FALSe to disable.
 *
 *****************************************************************************/
void enableOdoInAlgorithm(BOOL const enable)
{
    gAlgorithm.Behavior.bit.useOdo = (uint32_t)enable;
}

/******************************************************************************
 * @brief Check if the magnetometer is enabled in the algorithm.
 *
 * @return True if enabled, FALSE if disabled.
 *****************************************************************************/
BOOL magUsedInAlgorithm()
{
    return gAlgorithm.Behavior.bit.useMag != 0U;
}

/******************************************************************************
 * @brief Check if GPS is enabled in the algorithm.
 *
 * @return True if enabled, FALSE if disabled.
 *****************************************************************************/
BOOL gpsUsedInAlgorithm(void)
{
    return (BOOL)gAlgorithm.Behavior.bit.useGPS;
}

/******************************************************************************
 * @brief Check if the odometer is enabled in the algorithm.
 *
 * @return True if enabled, FALSE if disabled.
 *****************************************************************************/
BOOL odoUsedInAlgorithm(void)
{
    return (BOOL)gAlgorithm.Behavior.bit.useOdo;
}

/******************************************************************************
 * @brief Enable/disable free integration in the algorithm.
 *
 * @param enable    [in]    True to enable, and FALSe to disable.
 *
 *****************************************************************************/
void enableFreeIntegration(BOOL const enable)
{
    gAlgorithm.Behavior.bit.freeIntegrate = (uint32_t)enable;
}

/******************************************************************************
 * @brief Check if free integration is enabled in the algorithm.
 *
 * @return True if enabled, FALSE if disabled.
 *****************************************************************************/
BOOL freeIntegrationEnabled()
{
    return (BOOL)gAlgorithm.Behavior.bit.freeIntegrate;
}

/******************************************************************************
 * @brief Enable/disable yaw locking when being stationary in the algorithm.
 *
 * @param enable    [in]    True to enable, and FALSe to disable.
 *
 *****************************************************************************/
void enableStationaryLockYaw(BOOL const enable)
{
    gAlgorithm.Behavior.bit.enableStationaryLockYaw = (uint32_t)enable;
}

/******************************************************************************
 * @brief Check if yaw locking when being stationary is enabled in the algorithm.
 *
 * @return True if enabled, FALSE if disabled.
 *****************************************************************************/
BOOL stationaryLockYawEnabled()
{
    return (BOOL)gAlgorithm.Behavior.bit.enableStationaryLockYaw;
}

/******************************************************************************
 * @brief Enable/disable stationary period detection using IMU in the algorithm.
 *
 * @param enable    [in]    True to enable, and FALSe to disable.
 *
 *****************************************************************************/
void enableImuStaticDetect(BOOL const enable)
{
    gAlgorithm.Behavior.bit.enableImuStaticDetect = (uint32_t)enable;
}

/******************************************************************************
 * @brief Check if stationary period detection using IMU is enabled in the algorithm.
 *
 * @return True if enabled, FALSE if disabled.
 *****************************************************************************/
BOOL imuStaticDetectEnabled()
{
    return (BOOL)gAlgorithm.Behavior.bit.enableImuStaticDetect;
}

/******************************************************************************
 * @brief Update linAccelSwitchDelay in algorithm structure.
 *
 * @param value    [in]    value of linAccelSwitchDelay.
 *
 *****************************************************************************/
void SetAccelSwitchDelay(uint32_t value)
{
    gAlgorithm.Limit.linAccelSwitchDelay = value * gAlgorithm.callingFreq;
}

/******************************************************************************
 * @brief Update rateIntegrationTime in algorithm structure.
 *
 * @param value    [in]    value of rateIntegrationTime.
 *
 *****************************************************************************/
void SetRateIntegrationTime(uint32_t value)
{
    gAlgorithm.Limit.rateIntegrationTime = value * gAlgorithm.callingFreq;
}

/******************************************************************************
 * @brief Update useRawAccToDetectLinAccel in algorithm structure.
 *
 * @param value    [in]    value of useRawAccToDetectLinAccel.
 *
 *****************************************************************************/
void SetLinAccelDetectMode(BOOL value)
{
    gAlgorithm.useRawAccToDetectLinAccel = value;
}

/******************************************************************************
 * @brief Update useRawRateToPredAccel in algorithm structure.
 *
 * @param value    [in]    value of useRawRateToPredAccel.
 *
 *****************************************************************************/
void SetAccelPredictMode(BOOL value)
{
    gAlgorithm.useRawRateToPredAccel = value;
}

/******************************************************************************
 * @brief Update coefOfReduceQ in algorithm structure.
 *
 * @param value    [in]    value of coefOfReduceQ.
 *
 *****************************************************************************/
void SetCoefOfReduceQ(real value)
{
    gAlgorithm.coefOfReduceQ = value;
}

/******************************************************************************
 * @brief Update user configurations of odometer in algorithm structure.
 *
 * @param value    [in]    User configurations of odometer.
 *
 *****************************************************************************/
void SetOdoUserCfg(const OdoCfgStruct_t* const value)
{
    gAlgorithm.odoCfg.signalSource       = value->signalSource;
    gAlgorithm.odoCfg.msgRate            = value->msgRate;
    gAlgorithm.odoCfg.mountLocation      = (value->odoCfgSwitch & ODO_MOUNT_LOC_MASK);
    memcpy(gAlgorithm.odoCfg.leverArmB, value->leverArmB, NUM_AXIS * sizeof(real));

    if (NO_SOURCE != gAlgorithm.odoCfg.signalSource)
    {
        enableOdoInAlgorithm(TRUE);
    }
    else
    {
        enableOdoInAlgorithm(FALSE);
    }
}
