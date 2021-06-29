/*
 * File:   EKF_Algorithms.c
 * Author: joemotyka
 *
 * Created on May 8, 2016, 12:35 AM
 */
#include <math.h>       // std::abs
#include <stdlib.h>     // EXIT_FAILURE
#include <string.h>   // memset

#include "GlobalConstants.h"   // TRUE, FALSE, etc
#include "Indices.h"    // IND

#include "MagAlign.h"
#include "QuaternionMath.h"
#include "algorithm.h"  // gAlgorithm
#include "AlgorithmLimits.h"
#include "TransformationMath.h"
#include "VectorMath.h"
#include "MotionStatus.h"
#include "SelectState.h"
#include "EKF_Algorithm.h"
#include "PredictFunctions.h"
#include "UpdateFunctions.h"
#include "bitAPI.h"
#include "magAPI.h"
#include "ekfAPI.h"
#include "TimingVars.h"
#include "algorithmAPI.h"

#ifdef INS_OFFLINE
#include <stdio.h>
#else
#ifdef DISPLAY_DIAGNOSTIC_MSG
#include "debug.h"
#endif
#endif

KalmanFilterStruct      gKalmanFilter;
EKF_InputDataStruct     gEKFInput;
EKF_OutputDataStruct    gEKFOutput;
ImuStatsStruct          gImuStats;

/******************************************************************************
 * @brief Remove lever arm in position and velocity.
 *        GNSS measured position is the position of the antenna. For GNSS/INS
 *        integration, the position of IMU is needed. Before using the GNSS
 *        position and velocity, those shold be first converted to the IMU
 *        position and velocity by removing lever arm effects. Lever arm
 *        introduces an offset in position. This offset can be directly canceled
 *        by substracting lever arm. Combined with angular velocity, lever arm
 *        further introduces relative velocity.
 *
 * @param lla       [in/out]    Lat, lon and alt of the antenna, and
 *                              will be converted to LLA of the IMU. [rad, rad, m].
 * @param vNed      [in/out]    NED velocity of the antenna, and will be
 *                              converted to NED velocity of the IMU. [m/s].
 * @param w         [in]        angular velocity of the vehicle relative to
 *                              ECEF in the body frame. [rad/s].
 * @param leverArmB [in]        lever arm in the body frame. [m].
 * @param rn2e      [out]       ransformation matrix from NED to ECEF.
 * @param ecef      [out]       ECEF position without lever arm. [m]
 * ******************************************************************************/
static void RemoveLeverArm(float64_t lla[], float64_t vNed[],
                           real w[], real leverArmB[],
                           real rn2e[3][3], float64_t ecef[]);

/******************************************************************************
 * @brief Check if PPS from GPS reciever is detected or available.
 *
 *  
 *****************************************************************************/
static void HandlePps();

/******************************************************************************
 * @brief Save Kalman filter states once PPS is detected.
 *
 *  
 *****************************************************************************/
static void SaveKfStateAtPps();

//=============================================================================

/******************************************************************************
 * @brief Main routine of the EKF algorithm. This routine is called at either 100
 *        or 200 Hz based upon the system configuration:
 *          -- Unaided soln: 200 Hz
 *          -- Aided soln: 100 Hz
 * 
 *  
 *****************************************************************************/
void EKF_Algorithm(void)
{
    TimingVars_Increment();

    gAlgorithm.timer = gAlgorithm.timer + gAlgorithm.dITOW;
    if (odoUsedInAlgorithm())
    {
        IntegrateOdo(&gEKFInput.odo, &gAlgorithm.odoCfg, gAlgorithm.timer, gAlgorithm.dt,
                    gKalmanFilter.correctedRate_B, gEKFInput.accel_B, &gAlgorithm.odoStatus);
    }

    /* After STABILIZE_SYSTEM, the accel data will first pass a low-pass filter.
     * Stats of the filter accel will then be calculated.
     * According to gAlgorithm.useRawAccToDetectLinAccel,
     * raw or filtered accel is used to detect linear accel.
     */
    if (gAlgorithm.state > (uint8_t)STABILIZE_SYSTEM)
    {
        /* Compute IMU mean/var, filter IMU data (optional), detect static.
         * After STABILIZE_SYSTEM, each IMU sample is pushed into a buffer.
         * Before the buffer is full, results are not accurate should not be used.
         */
        MotionStatusImu(gEKFInput.angRate_B, gEKFInput.accel_B, &gImuStats, FALSE);

        /* It is realiable to detect static by aiding signal, so staticDelay and 
         * rateIntegrationTime can be reduced less than default 2 seconds.
         */
        uint32_t staticDelay = gAlgorithm.Limit.linAccelSwitchDelay;
        uint32_t rateIntegrationTime = gAlgorithm.Limit.rateIntegrationTime;
        if (gAlgorithm.odoStatus.bStaticODO && (STATIC_WRT_BODY == gAlgorithm.odoCfg.mountLocation))
        {
            // Reduce staticDelay and rateIntegrationTime to 0.5 second if static.
            real const temp = 0.5F * (real)gAlgorithm.callingFreq;
            staticDelay = (uint32_t)temp;
            rateIntegrationTime = (uint32_t)temp;
        }

        // estimate accel error
        if (gAlgorithm.useRawAccToDetectLinAccel)
        {
            EstimateAccelError(gEKFInput.accel_B,
                               gEKFInput.angRate_B,
                               gAlgorithm.dt,
                               staticDelay,
                               rateIntegrationTime,
                               gKalmanFilter.rateBias_B,
                               &gImuStats);
        }
        else
        {
            EstimateAccelError(gImuStats.lpfAccel,
                               gEKFInput.angRate_B,
                               gAlgorithm.dt,
                               staticDelay,
                               rateIntegrationTime,
                               gKalmanFilter.rateBias_B,
                               &gImuStats);
        }

        // Detect if the unit is static or dynamic
        DetectMotionFromAccel(gImuStats.accelNorm, 0);
        gAlgoStatus.bit.linearAccel = !gAlgorithm.linAccelSwitch;

        // if zero velocity detection by IMU is not enabled, detection result is always false.
        if (!gAlgorithm.Behavior.bit.enableImuStaticDetect)
        {
            gImuStats.bStaticIMU = FALSE;
        }
        gAlgoStatus.bit.staticImu = (uint32_t)gImuStats.bStaticIMU;
    }

    // Compute the EKF solution if past the stabilization and initialization stages
    if (gAlgorithm.state > (uint8_t)INITIALIZE_ATTITUDE)
    {
        // Increment the algorithm itow
        gAlgorithm.itow = gAlgorithm.itow + gAlgorithm.dITOW;

        // Update .coefOfReduceQ dynamically, in case of aiding signal is over time or IMU is static.
        UpdateCoefOfReduceQ(&gAlgorithm.coefOfReduceQ, FALSE);

        // Perform EKF Prediction
        EKF_PredictionStage(gImuStats.lpfAccel);

        if (gAlgorithm.state == (uint8_t)INS_SOLUTION)
        {
            HandlePps();
        }

        /* Update the predicted states if not freely integrating
         * NOTE: free- integration is not applicable in HG AHRS mode.
         */
        static uint16_t freeIntegrationCounter = 0U;
        if (gAlgorithm.Behavior.bit.freeIntegrate && (gAlgorithm.state > (uint8_t)HIGH_GAIN_AHRS))
        {
            /* Limit the free-integration time before reverting to the complete
             * EKF solution (including updates).
             */
            freeIntegrationCounter = freeIntegrationCounter + 1U;   // [cycles]
            if (freeIntegrationCounter >= gAlgorithm.Limit.Free_Integration_Cntr)
            {
                freeIntegrationCounter = 0U;
                enableFreeIntegration(FALSE);

#ifdef DISPLAY_DIAGNOSTIC_MSG
                // Display the time at the end of the free-integration period
                TimingVars_DiagnosticMsg("Free integration period ended");
#endif
            }
            // Restart the system in LG AHRS after free integration is complete
            gAlgorithm.insFirstTime = (uint8_t)TRUE;
            gAlgorithm.state = (uint8_t)LOW_GAIN_AHRS;
            gAlgorithm.stateTimer = gAlgorithm.Duration.Low_Gain_AHRS;
        }
        else
        {
            enableFreeIntegration(FALSE);
            freeIntegrationCounter = 0U;

            // Perform EKF Update
            EKF_UpdateStage();
        }

        /* Save the past attitude quaternion before updating (for use in the
         * covariance estimation calculations).
         */
        gKalmanFilter.quaternion_Past[Q0] = gKalmanFilter.quaternion[Q0];
        gKalmanFilter.quaternion_Past[Q1] = gKalmanFilter.quaternion[Q1];
        gKalmanFilter.quaternion_Past[Q2] = gKalmanFilter.quaternion[Q2];
        gKalmanFilter.quaternion_Past[Q3] = gKalmanFilter.quaternion[Q3];

        /* Generate the transformation matrix (R_BinN) based on the past value of
         * the attitude quaternion (prior to prediction at the new time-step)
         */
        QuaternionToR321(gKalmanFilter.quaternion, gKalmanFilter.R_BinN);

        /* Euler angels are not calcualted here because it is used as a propagation resutls
         * to calculate system innovation. So, Euler angles are updated in the prediction
         * stage. In theory, Euler angles should be updated after each measurement update.
         * However, after EKF converges, it does not matter.
         */

         // Update LLA
        if ((gAlgorithm.insFirstTime == (uint8_t)FALSE))
        {
            float64_t r_E[NUM_AXIS];
            float32_t pointOfInterestN[3];
            pointOfInterestN[0] = (gKalmanFilter.R_BinN[0][0] * gAlgorithm.pointOfInterestB[0]) +
                (gKalmanFilter.R_BinN[0][1] * gAlgorithm.pointOfInterestB[1]) +
                (gKalmanFilter.R_BinN[0][2] * gAlgorithm.pointOfInterestB[2]);
            pointOfInterestN[1] = (gKalmanFilter.R_BinN[1][0] * gAlgorithm.pointOfInterestB[0]) +
                (gKalmanFilter.R_BinN[1][1] * gAlgorithm.pointOfInterestB[1]) +
                (gKalmanFilter.R_BinN[1][2] * gAlgorithm.pointOfInterestB[2]);
            pointOfInterestN[2] = (gKalmanFilter.R_BinN[2][0] * gAlgorithm.pointOfInterestB[0]) +
                (gKalmanFilter.R_BinN[2][1] * gAlgorithm.pointOfInterestB[1]) +
                (gKalmanFilter.R_BinN[2][2] * gAlgorithm.pointOfInterestB[2]);
            pointOfInterestN[0] += gKalmanFilter.Position_N[0];
            pointOfInterestN[1] += gKalmanFilter.Position_N[1];
            pointOfInterestN[2] += gKalmanFilter.Position_N[2];
            PosNED_To_PosECEF(pointOfInterestN, gKalmanFilter.rGPS0_E, gKalmanFilter.Rn2e, r_E);
            //                 100 Hz           generated once          1 Hz                      100 Hz
            // output variable (ned used for anything else); this is in [ deg, deg, m ]
            ECEF_To_LLA(gKalmanFilter.llaDeg, r_E);
            //          100 Hz                    100 Hz
        }
    }

    /* Select the algorithm state based upon the present state as well as
     * operational conditions (time, sensor health, etc).  Note: This is called
     * after the the above code-block to prevent the transition from occuring
     * until the next time step.
     */
    switch ((int32_t)(gAlgorithm.state))
    {
    case STABILIZE_SYSTEM:
        StabilizeSystem();
        break;
    case INITIALIZE_ATTITUDE:
        InitializeAttitude();
        break;
    case HIGH_GAIN_AHRS:
        HG_To_LG_Transition_Test();
        break;
    case LOW_GAIN_AHRS:
        LG_To_INS_Transition_Test();
        break;
    case INS_SOLUTION:
        INS_To_AHRS_Transition_Test();
        break;
    default:
        return;
    }

    // Dynamic motion logic (to revert back to HG AHRS)
    DynamicMotion();

    AlgoStatus algoStatus;
    GetAlgoStatus(&algoStatus);

    BIT_UpdateAlgorithmStatus((uint16_t)algoStatus.all);
}

/******************************************************************************
 * @brief Check if PPS from GPS reciever is detected or available.
 *
 *  
 *****************************************************************************/
static void HandlePps()
{
    // PPS not detected for a long time?
    uint32_t timeSinceLastPps;
    if (gAlgorithm.itow >= gKalmanFilter.ppsITow)
    {
        timeSinceLastPps = gAlgorithm.itow - gKalmanFilter.ppsITow;
    }
    else
    {
        // a new week
        timeSinceLastPps = (gAlgorithm.itow + MAX_ITOW) - gKalmanFilter.ppsITow;
    }
    if (timeSinceLastPps > 2000U)
    {
        gAlgoStatus.bit.ppsAvailable = (uint32_t)FALSE;
    }
    // PPS detected
    if (gEKFInput.ppsDetected)
    {
        // PPS is available from now.
        gAlgoStatus.bit.ppsAvailable = (uint32_t)TRUE;
        // save states when pps detected
        SaveKfStateAtPps();
    }
}

/******************************************************************************
 * @brief Save Kalman filter states once PPS is detected.
 *
 *  
 *****************************************************************************/
static void SaveKfStateAtPps()
{
    // save time
    gKalmanFilter.ppsITow = gAlgorithm.itow;

    // save states
    gKalmanFilter.ppsPosition_N[0] = gKalmanFilter.Position_N[0];
    gKalmanFilter.ppsPosition_N[1] = gKalmanFilter.Position_N[1];
    gKalmanFilter.ppsPosition_N[2] = gKalmanFilter.Position_N[2];
    gKalmanFilter.ppsVelocity_N[0] = gKalmanFilter.Velocity_N[0];
    gKalmanFilter.ppsVelocity_N[1] = gKalmanFilter.Velocity_N[1];
    gKalmanFilter.ppsVelocity_N[2] = gKalmanFilter.Velocity_N[2];
    gKalmanFilter.ppsEulerAngles[0] = gKalmanFilter.eulerAngles[0];
    gKalmanFilter.ppsEulerAngles[1] = gKalmanFilter.eulerAngles[1];
    gKalmanFilter.ppsEulerAngles[2] = gKalmanFilter.eulerAngles[2];
    // save state covariance matrix
    memcpy(gKalmanFilter.ppsP, &gKalmanFilter.P, sizeof(gKalmanFilter.P));

    /* reset state transition matrix and state covariance matrix increment between pps and update
     * phi is reset to an identity matrix, and dQ is reset to a zero matrix.
     */
    memset(gKalmanFilter.phi, 0, sizeof(gKalmanFilter.phi));
    memset(gKalmanFilter.dQ, 0, sizeof(gKalmanFilter.dQ));
    for (int32_t i = 0; i < NUMBER_OF_EKF_STATES; i++)
    {
        gKalmanFilter.phi[i][i] = 1.0F;
    }
}

/******************************************************************************
 * @brief Get the attitude (expressed as Euler angles in roll, pitch and yaw
 *        order) of the body-frame in the NED-frame.
 *
 * @param EulerAngles   [out]   Euler angles in roll, pitch and yaw order. [deg].
 *  
 *****************************************************************************/
void EKF_GetAttitude_EA(real EulerAngles[])
{
    // Euler-angles in [deg]
    EulerAngles[ROLL] = (real)gEKFOutput.eulerAngs_BinN[ROLL];
    EulerAngles[PITCH] = (real)gEKFOutput.eulerAngs_BinN[PITCH];
    EulerAngles[YAW] = (real)gEKFOutput.eulerAngs_BinN[YAW];
}

/******************************************************************************
 * @brief Get the attitude (expressed as Euler angles in roll, pitch and yaw
 *        order) of the body-frame in the NED-frame.
 *
 * @param EulerAngles   [out]   Euler angles in roll, pitch and yaw order. [rad].
 *  
 *****************************************************************************/
void EKF_GetAttitude_EA_RAD(real EulerAngles[])
{
    // Euler-angles in [rad]
    EulerAngles[ROLL] = (real)gKalmanFilter.eulerAngles[ROLL];
    EulerAngles[PITCH] = (real)gKalmanFilter.eulerAngles[PITCH];
    EulerAngles[YAW] = (real)gKalmanFilter.eulerAngles[YAW];
}

/******************************************************************************
 * @brief Get the attitude quaternion of the body-frame in the NED-frame.
 *
 * @param Quaternions   [out]   Attitude quaternion with scalar first.
 *  
 *****************************************************************************/
void EKF_GetAttitude_Q(real Quaternions[])
{
    Quaternions[Q0] = (real)gEKFOutput.quaternion_BinN[Q0];
    Quaternions[Q1] = (real)gEKFOutput.quaternion_BinN[Q1];
    Quaternions[Q2] = (real)gEKFOutput.quaternion_BinN[Q2];
    Quaternions[Q3] = (real)gEKFOutput.quaternion_BinN[Q3];
}

/******************************************************************************
 * @brief Get angular rate corrected for estimated rate bias.
 *
 * @param CorrAngRates_B    [out]   Corrected angular rate. [dps].
 *  
 *****************************************************************************/
void EKF_GetCorrectedAngRates(real CorrAngRates_B[])
{
    // Angular-rate in [deg/s]
    CorrAngRates_B[X_AXIS] = (real)gEKFOutput.corrAngRates_B[X_AXIS];
    CorrAngRates_B[Y_AXIS] = (real)gEKFOutput.corrAngRates_B[Y_AXIS];
    CorrAngRates_B[Z_AXIS] = (real)gEKFOutput.corrAngRates_B[Z_AXIS];
}

/******************************************************************************
 * @brief Get angular rate corrected for estimated rate bias.
 *
 * @param corrRates_B   [out]   Corrected angular rate. [dps].
 *  
 *****************************************************************************/
void EKF_GetCorrectedRates_B(real corrRates_B[])
{
    // Angular-rate in [rad]
    corrRates_B[X_AXIS] = (real)gKalmanFilter.correctedRate_B[X_AXIS];
    corrRates_B[Y_AXIS] = (real)gKalmanFilter.correctedRate_B[Y_AXIS];
    corrRates_B[Z_AXIS] = (real)gKalmanFilter.correctedRate_B[Z_AXIS];
}

/******************************************************************************
 * @brief Get accleration corrected for estimated accleratio bias.
 *
 * @param CorrAccels_B  [out]   Corrected acceleration. [m/s^2]l
 *  
 *****************************************************************************/
void EKF_GetCorrectedAccels(real CorrAccels_B[])
{
    // Acceleration in [m/s^2]
    CorrAccels_B[X_AXIS] = (real)gEKFOutput.corrAccel_B[X_AXIS];
    CorrAccels_B[Y_AXIS] = (real)gEKFOutput.corrAccel_B[Y_AXIS];
    CorrAccels_B[Z_AXIS] = (real)gEKFOutput.corrAccel_B[Z_AXIS];
}

/******************************************************************************
 * @brief Get estimated gyro bias.
 *
 * @param AngRateBias_B [out]   Estimated gyro bias. [dps].
 *  
 *****************************************************************************/
void EKF_GetEstimatedAngRateBias(real AngRateBias_B[])
{
    // Angular-rate bias in [deg/sec]
    AngRateBias_B[X_AXIS] = (real)gEKFOutput.angRateBias_B[X_AXIS];
    AngRateBias_B[Y_AXIS] = (real)gEKFOutput.angRateBias_B[Y_AXIS];
    AngRateBias_B[Z_AXIS] = (real)gEKFOutput.angRateBias_B[Z_AXIS];
}

/******************************************************************************
 * @brief Get estimated accel bias.
 *
 * @param AccelBias_B   [out]   Estimated accel bias. [m/s^2].
 *  
 *****************************************************************************/
void EKF_GetEstimatedAccelBias(real AccelBias_B[])
{
    // Acceleration-bias in [m/s^2]
    AccelBias_B[X_AXIS] = (real)gEKFOutput.accelBias_B[X_AXIS];
    AccelBias_B[Y_AXIS] = (real)gEKFOutput.accelBias_B[Y_AXIS];
    AccelBias_B[Z_AXIS] = (real)gEKFOutput.accelBias_B[Z_AXIS];
}

/******************************************************************************
 * @brief Get the estimated NED position.
 *
 * @param Position_N    [out]   NED position. [m].
 *  
 *****************************************************************************/
void EKF_GetEstimatedPosition(real Position_N[])
{
    // Position in [m]
    Position_N[X_AXIS] = (real)gEKFOutput.position_N[X_AXIS];
    Position_N[Y_AXIS] = (real)gEKFOutput.position_N[Y_AXIS];
    Position_N[Z_AXIS] = (real)gEKFOutput.position_N[Z_AXIS];
}

/******************************************************************************
 * @brief Get the estimated NED velocity.
 *
 * @param Velocity_N    [out]   NED velocity. [m/s].
 *  
 *****************************************************************************/
void EKF_GetEstimatedVelocity(real Velocity_N[])
{
    // Velocity in [m/s]
    Velocity_N[X_AXIS] = (real)gEKFOutput.velocity_N[X_AXIS];
    Velocity_N[Y_AXIS] = (real)gEKFOutput.velocity_N[Y_AXIS];
    Velocity_N[Z_AXIS] = (real)gEKFOutput.velocity_N[Z_AXIS];
}

/******************************************************************************
 * @brief Get the estimated latitude, longitude and altitude.
 *
 * @param LLA   [out]   [latitude, longitude, altitude]. [deg, deg, rad].
 *  
 *****************************************************************************/
void EKF_GetEstimatedLLA(float64_t LLA[])
{
    // Velocity in [m/s]
    LLA[X_AXIS] = (float64_t)gEKFOutput.llaDeg[X_AXIS];
    LLA[Y_AXIS] = (float64_t)gEKFOutput.llaDeg[Y_AXIS];
    LLA[Z_AXIS] = (float64_t)gEKFOutput.llaDeg[Z_AXIS];
}

/******************************************************************************
 * @brief Get operation mode of the algorithm.
 *          0: Stabilize
 *          1: Initialize
 *          2: High-Gain VG/AHRS mode
 *          3: Low-Gain VG/AHRS mode
 *          4: INS operation
 * @param EKF_OperMode  [out]   Operation mode.
 *  
 *****************************************************************************/
void EKF_GetOperationalMode(uint8_t* const EKF_OperMode)
{
    *EKF_OperMode = gEKFOutput.opMode;
}

/******************************************************************************
 * @brief Get the linear-acceleration and turn-switch flags
 *
 * @param EKF_LinAccelSwitch    [out]   Linear acceleration switch flag.
 * @param EKF_TurnSwitch        [out]   Turn swith flag.
 *  
 *****************************************************************************/
void EKF_GetOperationalSwitches(uint8_t* const EKF_LinAccelSwitch,
                                uint8_t* const EKF_TurnSwitch)
{
    *EKF_LinAccelSwitch = gEKFOutput.linAccelSwitch;
    *EKF_TurnSwitch = gEKFOutput.turnSwitchFlag;
}

/******************************************************************************
 * @brief Feed sensor measurement to the algorithm.
 *
 * @param accels        [in]    Accel measurement. [g].
 * @param rates         [in]    Gyro measurement. [rad/s].
 * @param mags          [in]    Magnetometer measurements. [G].
 * @param gps           [in]    GPS measurements.
 * @param odo           [in]    Odometer measurements.
 * @param ppsDetected   [in]    True if PPS is just detected, FALSE if not.
 *  
 *****************************************************************************/
void EKF_SetInputStruct(float64_t accels[],
                        float64_t rates[],
                        float64_t mags[],
                        const gpsDataStruct_t* const gps,
                        const odoDataStruct_t* const odo,
                        BOOL const ppsDetected)
{
    // Accelerometer signal is in [m/s/s]
    gEKFInput.accel_B[X_AXIS] = (real)(accels[X_AXIS] * GRAVITY);
    gEKFInput.accel_B[Y_AXIS] = (real)(accels[Y_AXIS] * GRAVITY);
    gEKFInput.accel_B[Z_AXIS] = (real)(accels[Z_AXIS] * GRAVITY);

    // Angular-rate signal is in [rad/s]
    gEKFInput.angRate_B[X_AXIS] = (real)rates[X_AXIS];
    gEKFInput.angRate_B[Y_AXIS] = (real)rates[Y_AXIS];
    gEKFInput.angRate_B[Z_AXIS] = (real)rates[Z_AXIS];

    // Magnetometer signal is in [G]
    gEKFInput.magField_B[X_AXIS] = (real)mags[X_AXIS];
    gEKFInput.magField_B[Y_AXIS] = (real)mags[Y_AXIS];
    gEKFInput.magField_B[Z_AXIS] = (real)mags[Z_AXIS];
    real tmp[2];
    tmp[X_AXIS] = gEKFInput.magField_B[X_AXIS] - gMagAlign.hardIronBias[X_AXIS];
    tmp[Y_AXIS] = gEKFInput.magField_B[Y_AXIS] - gMagAlign.hardIronBias[Y_AXIS];
    gEKFInput.magField_B[X_AXIS] = (gMagAlign.SF[0] * tmp[X_AXIS]) + (gMagAlign.SF[1] * tmp[Y_AXIS]);
    gEKFInput.magField_B[Y_AXIS] = (gMagAlign.SF[2] * tmp[X_AXIS]) + (gMagAlign.SF[3] * tmp[Y_AXIS]);

    // ----- Input from the GPS goes here -----
    gEKFInput.gpsUpdate = gps->gpsUpdate;

    if (gEKFInput.gpsUpdate)
    {
        // Validity data
        gEKFInput.gpsFixType = gps->gpsFixType;

        // num of satellites
        gEKFInput.numSatellites = gps->numSatellites;

        // ITOW data
        gEKFInput.itow = gps->itow;

        // Data quality measures
        gEKFInput.GPSHorizAcc = gps->GPSHorizAcc;
        gEKFInput.GPSVertAcc = gps->GPSVertAcc;
        gEKFInput.HDOP = gps->HDOP;

        // Lat/Lon/Alt data
        gEKFInput.llaAnt[LAT] = gps->latitude * DEG_TO_RAD;
        gEKFInput.llaAnt[LON] = gps->longitude * DEG_TO_RAD;
        gEKFInput.llaAnt[ALT] = gps->altitude;
        gEKFInput.geoidAboveEllipsoid = gps->geoidAboveEllipsoid;

        // Velocity data
        gEKFInput.vNedAnt[X_AXIS] = gps->vNed[X_AXIS];
        gEKFInput.vNedAnt[Y_AXIS] = gps->vNed[Y_AXIS];
        gEKFInput.vNedAnt[Z_AXIS] = gps->vNed[Z_AXIS];

        // Course and velocity data
        gEKFInput.rawGroundSpeed = (real)sqrt(SQUARE(gps->vNed[0]) +
                                              SQUARE(gps->vNed[1]));// gps->rawGroundSpeed;
        gEKFInput.trueCourse = (real)gps->trueCourse;

        /* Remove lever arm effects in LLA/Velocity. To do this requires transformation matrix
         * from the body frame to the NED frame. Before heading initialized, lever arm cannot
         * be correctly removed. After heading initialized, there would be position jump if
         * initial heading is different from uninitlized one and the lever arm is large.
         * After heading intialized, the position/velocity could also be reinitialized, and
         * lever arm effects on the position/velocity are not corrected removed.
         * LLA without lever arm is used to update Rn2e/ECEF postion, and calculate relative
         * position in NED
         */
        if (gEKFInput.gpsFixType)
        {
            gEKFInput.lla[LAT] = gEKFInput.llaAnt[LAT];
            gEKFInput.lla[LON] = gEKFInput.llaAnt[LON];
            gEKFInput.lla[ALT] = gEKFInput.llaAnt[ALT];
            gEKFInput.vNed[0] = gEKFInput.vNedAnt[0];
            gEKFInput.vNed[1] = gEKFInput.vNedAnt[1];
            gEKFInput.vNed[2] = gEKFInput.vNedAnt[2];
            /* remove lever arm. Indeed, corrected angular rate should be used. Considering angular
             * bias is small, raw angular rate is used.
             */
            RemoveLeverArm(gEKFInput.lla,
                           gEKFInput.vNed,
                           gEKFInput.angRate_B,
                           gAlgorithm.leverArmB,
                           gKalmanFilter.Rn2e,
                           gKalmanFilter.rGPS_E);

            /* Calculate relative position in the NED frame. The initial position is rGPS0_E which.
             * is determined when the algorithm first enters the INS mode (InitINSFilter).
             */
            ECEF_To_Base(gKalmanFilter.rGPS0_E,
                         gKalmanFilter.rGPS_E,
                         gKalmanFilter.Rn2e,
                         gKalmanFilter.rGPS_N);
        }
    }

    // odometer
    if (odoUsedInAlgorithm())
    {
        memcpy(&gEKFInput.odo, odo, sizeof(odoDataStruct_t));
    }

    // 1PPS signal from GNSS receiver
    gEKFInput.ppsDetected = ppsDetected;
}

/******************************************************************************
 * @brief Update algorithm output.
 *
 *  
 *****************************************************************************/
void EKF_SetOutputStruct(void)
{
    // ------------------ States ------------------

    // Position in [m]
    gEKFOutput.position_N[X_AXIS] = gKalmanFilter.Position_N[X_AXIS];
    gEKFOutput.position_N[Y_AXIS] = gKalmanFilter.Position_N[Y_AXIS];
    gEKFOutput.position_N[Z_AXIS] = gKalmanFilter.Position_N[Z_AXIS];

    // Velocity in [m/s]
    gEKFOutput.velocity_N[X_AXIS] = gKalmanFilter.Velocity_N[X_AXIS];
    gEKFOutput.velocity_N[Y_AXIS] = gKalmanFilter.Velocity_N[Y_AXIS];
    gEKFOutput.velocity_N[Z_AXIS] = gKalmanFilter.Velocity_N[Z_AXIS];

    // Position in [N/A]
    gEKFOutput.quaternion_BinN[Q0] = gKalmanFilter.quaternion[Q0];
    gEKFOutput.quaternion_BinN[Q1] = gKalmanFilter.quaternion[Q1];
    gEKFOutput.quaternion_BinN[Q2] = gKalmanFilter.quaternion[Q2];
    gEKFOutput.quaternion_BinN[Q3] = gKalmanFilter.quaternion[Q3];

    // Angular-rate bias in [deg/sec]
    gEKFOutput.angRateBias_B[X_AXIS] = gKalmanFilter.rateBias_B[X_AXIS] * RAD_TO_DEG;
    gEKFOutput.angRateBias_B[Y_AXIS] = gKalmanFilter.rateBias_B[Y_AXIS] * RAD_TO_DEG;
    gEKFOutput.angRateBias_B[Z_AXIS] = gKalmanFilter.rateBias_B[Z_AXIS] * RAD_TO_DEG;

    // Acceleration-bias in [m/s^2]
    gEKFOutput.accelBias_B[X_AXIS] = gKalmanFilter.accelBias_B[X_AXIS];
    gEKFOutput.accelBias_B[Y_AXIS] = gKalmanFilter.accelBias_B[Y_AXIS];
    gEKFOutput.accelBias_B[Z_AXIS] = gKalmanFilter.accelBias_B[Z_AXIS];

    // ------------------ Derived variables ------------------

    // Euler-angles in [deg]
    gEKFOutput.eulerAngs_BinN[ROLL] = gKalmanFilter.eulerAngles[ROLL] * RAD_TO_DEG;
    gEKFOutput.eulerAngs_BinN[PITCH] = gKalmanFilter.eulerAngles[PITCH] * RAD_TO_DEG;
    gEKFOutput.eulerAngs_BinN[YAW] = gKalmanFilter.eulerAngles[YAW] * RAD_TO_DEG;

    // Angular-rate in [deg/s]
    real tmp;
    tmp = gEKFInput.angRate_B[X_AXIS] - gKalmanFilter.rateBias_B[X_AXIS];
    gEKFOutput.corrAngRates_B[X_AXIS] = (float64_t)tmp * RAD_TO_DEG;
    tmp = gEKFInput.angRate_B[Y_AXIS] - gKalmanFilter.rateBias_B[Y_AXIS];
    gEKFOutput.corrAngRates_B[Y_AXIS] = (float64_t)tmp * RAD_TO_DEG;
    tmp = gEKFInput.angRate_B[Z_AXIS] - gKalmanFilter.rateBias_B[Z_AXIS];
    gEKFOutput.corrAngRates_B[Z_AXIS] = (float64_t)tmp * RAD_TO_DEG;

    // Acceleration in [m/s^2]
    tmp = gEKFInput.accel_B[X_AXIS] - gKalmanFilter.accelBias_B[X_AXIS];
    gEKFOutput.corrAccel_B[X_AXIS] = (float64_t)tmp;
    tmp = gEKFInput.accel_B[Y_AXIS] - gKalmanFilter.accelBias_B[Y_AXIS];
    gEKFOutput.corrAccel_B[Y_AXIS] = (float64_t)tmp;
    tmp = gEKFInput.accel_B[Z_AXIS] - gKalmanFilter.accelBias_B[Z_AXIS];
    gEKFOutput.corrAccel_B[Z_AXIS] = (float64_t)tmp;


    // ------------------ Algorithm flags ------------------
    gEKFOutput.opMode = gAlgorithm.state;
    gEKFOutput.linAccelSwitch = gAlgorithm.linAccelSwitch;
    gEKFOutput.turnSwitchFlag = (uint8_t)gAlgoStatus.bit.turnSwitch;

    // ------------------ Latitude and Longitude Data ------------------
    gEKFOutput.llaDeg[LAT] = gKalmanFilter.llaDeg[LAT];
    gEKFOutput.llaDeg[LON] = gKalmanFilter.llaDeg[LON];
    gEKFOutput.llaDeg[ALT] = gKalmanFilter.llaDeg[ALT];
}

/******************************************************************************
 * @brief Initialize Kalman filter parameters of the INS app
 *
 *  
 *****************************************************************************/
void InitINSFilter(void)
{
    real tmp[7][7];
    int32_t rowNum;
    int32_t colNum;

#ifdef INS_OFFLINE
    printf("reset INS filter.\n");
#endif // INS_OFFLINE

    gAlgorithm.insFirstTime = (uint8_t)FALSE;

    // Sync the algorithm and GPS ITOW
    gAlgorithm.itow = gEKFInput.itow;

    /* We have a good GPS reading now - set this variable so we
     * don't drop into INS right away
     */
    gAlgorithm.timeOfLastGoodGPSReading = gEKFInput.itow;

    /* Upon the first entry into INS, save off the base position and reset the
     * Kalman filter variables.
     */
     // Save off the base ECEF location
    gKalmanFilter.rGPS0_E[X_AXIS] = gKalmanFilter.rGPS_E[X_AXIS];
    gKalmanFilter.rGPS0_E[Y_AXIS] = gKalmanFilter.rGPS_E[Y_AXIS];
    gKalmanFilter.rGPS0_E[Z_AXIS] = gKalmanFilter.rGPS_E[Z_AXIS];

    // Reset the gps position (as position is relative to starting location)
    gKalmanFilter.rGPS_N[X_AXIS] = 0.0F;
    gKalmanFilter.rGPS_N[Y_AXIS] = 0.0F;
    gKalmanFilter.rGPS_N[Z_AXIS] = 0.0F;

    // Reset prediction values. Position_N is also IMU position.
    gKalmanFilter.Position_N[X_AXIS] = (real)0.0;
    gKalmanFilter.Position_N[Y_AXIS] = (real)0.0;
    gKalmanFilter.Position_N[Z_AXIS] = (real)0.0;

    gKalmanFilter.Velocity_N[X_AXIS] = (real)gEKFInput.vNed[X_AXIS];
    gKalmanFilter.Velocity_N[Y_AXIS] = (real)gEKFInput.vNed[Y_AXIS];
    gKalmanFilter.Velocity_N[Z_AXIS] = (real)gEKFInput.vNed[Z_AXIS];

    gKalmanFilter.accelBias_B[X_AXIS] = (real)0.0;
    gKalmanFilter.accelBias_B[Y_AXIS] = (real)0.0;
    gKalmanFilter.accelBias_B[Z_AXIS] = (real)0.0;

    gKalmanFilter.linearAccel_B[X_AXIS] = (real)0.0;

    /* Extract the Quaternion and rate-bias values from the matrix before
     * resetting
     */
     // Save off the quaternion and rate-bias covariance values
    for (rowNum = Q0; rowNum <= (Q3 + Z_AXIS + 1); rowNum++)
    {
        for (colNum = Q0; colNum <= (Q3 + Z_AXIS + 1); colNum++)
        {
            tmp[rowNum][colNum] = gKalmanFilter.P[rowNum + STATE_Q0][colNum + STATE_Q0];
        }
    }

    // Reset P
    memset(gKalmanFilter.P, 0, sizeof(gKalmanFilter.P));
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++)
    {
        gKalmanFilter.P[rowNum][rowNum] = (real)INIT_P_INS;
    }

    // Repopulate the P matrix with the quaternion and rate-bias values
    for (rowNum = Q0; rowNum <= (Q3 + Z_AXIS + 1); rowNum++)
    {
        for (colNum = Q0; colNum <= (Q3 + Z_AXIS + 1); colNum++)
        {
            gKalmanFilter.P[rowNum + STATE_Q0][colNum + STATE_Q0] = tmp[rowNum][colNum];
        }
    }

    /* Use the GPS-provided horizontal and vertical accuracy values to populate
     *   the covariance values.
     */
    gKalmanFilter.P[STATE_RX][STATE_RX] = gEKFInput.GPSHorizAcc * gEKFInput.GPSHorizAcc;
    gKalmanFilter.P[STATE_RY][STATE_RY] = gKalmanFilter.P[STATE_RX][STATE_RX];
    gKalmanFilter.P[STATE_RZ][STATE_RZ] = gEKFInput.GPSVertAcc * gEKFInput.GPSVertAcc;

    /* Scale the best velocity error by HDOP then multiply by the z-axis angular
     * rate PLUS one (to prevent the number from being zero) so the velocity
     * update during high-rate turns is reduced.
     */
    float32_t temp = (real)0.0625 * gEKFInput.HDOP;  // 0.0625 = 0.05 / 0.8
    real const absFilteredYawRate = (real)fabs(gAlgorithm.filteredYawRate);
    if (absFilteredYawRate > TEN_DEGREES_IN_RAD)
    {
        temp *= 1.0F + absFilteredYawRate;
    }
    gKalmanFilter.P[STATE_VX][STATE_VX] = temp;
    gKalmanFilter.P[STATE_VX][STATE_VX] = gKalmanFilter.P[STATE_VX][STATE_VX] * gKalmanFilter.P[STATE_VX][STATE_VX];
    gKalmanFilter.P[STATE_VY][STATE_VY] = gKalmanFilter.P[STATE_VX][STATE_VX];

    // z-axis velocity isn't really a function of yaw-rate and hdop
    gKalmanFilter.P[STATE_VZ][STATE_VZ] = 0.1F * 0.1F;
}

/******************************************************************************
 * @brief Remove lever arm in position and velocity.
 *        GNSS measured position is the position of the antenna. For GNSS/INS
 *        integration, the position of IMU is needed. Before using the GNSS
 *        position and velocity, those shold be first converted to the IMU
 *        position and velocity by removing lever arm effects. Lever arm
 *        introduces an offset in position. This offset can be directly canceled
 *        by substracting lever arm. Combined with angular velocity, lever arm
 *        further introduces relative velocity.
 *
 * @param lla       [in/out]    Lat, lon and alt of the antenna, and
 *                              will be converted to LLA of the IMU. [rad, rad, m].
 * @param vNed      [in/out]    NED velocity of the antenna, and will be
 *                              converted to NED velocity of the IMU. [m/s].
 * @param w         [in]        angular velocity of the vehicle relative to
 *                              ECEF in the body frame. [rad/s].
 * @param leverArmB [in]        lever arm in the body frame. [m].
 * @param rn2e      [out]       ransformation matrix from NED to ECEF.
 * @param ecef      [out]       ECEF position without lever arm. [m]
 *******************************************************************************/
static void RemoveLeverArm(float64_t lla[], float64_t vNed[],
                           real w[], real leverArmB[],
                           real rn2e[3][3], float64_t ecef[])
{
    // Using position with lever arm to calculate rm and rn
    float64_t sinLat = sin(lla[LAT]);
    float64_t cosLat = cos(lla[LAT]);
    float64_t tmp = 1.0 - (E_ECC_SQ * sinLat * sinLat);
    float64_t const sqrtTmp = sqrt(tmp);
    float64_t const rn = E_MAJOR / sqrtTmp; // radius of Curvature [meters]
    float64_t const rm = (rn * (1.0 - E_ECC_SQ)) / tmp;
    // Remove lever arm from position
    real leverArmN[3];  // lever arm in the NED frame
    leverArmN[0] = (gKalmanFilter.R_BinN[0][0] * leverArmB[0]) +
                   (gKalmanFilter.R_BinN[0][1] * leverArmB[1]) +
                   (gKalmanFilter.R_BinN[0][2] * leverArmB[2]);
    leverArmN[1] = (gKalmanFilter.R_BinN[1][0] * leverArmB[0]) +
                   (gKalmanFilter.R_BinN[1][1] * leverArmB[1]) +
                   (gKalmanFilter.R_BinN[1][2] * leverArmB[2]);
    leverArmN[2] = (gKalmanFilter.R_BinN[2][0] * leverArmB[0]) +
                   (gKalmanFilter.R_BinN[2][1] * leverArmB[1]) +
                   (gKalmanFilter.R_BinN[2][2] * leverArmB[2]);
    lla[0] -= leverArmN[0] / rm;
    lla[1] -= leverArmN[1] / rn / cosLat;
    lla[2] += leverArmN[2];     /* Notice: lever arm is now in NED frame while altitude is
                                 * in the opposite direction of the z axis of NED frame.
                                 */

    /* Remove lever arm effects from velocity
    * v_gnss = v_imu + C_b2n * cross(wB, leverArmB)
    */
    cross(w, leverArmB, leverArmN);    // use leverArmN to temporatily hold w x leverArmB in body frame
    real dv;
    dv = (gKalmanFilter.R_BinN[0][0] * leverArmN[0]) +
         (gKalmanFilter.R_BinN[0][1] * leverArmN[1]) +
         (gKalmanFilter.R_BinN[0][2] * leverArmN[2]);
    vNed[0] -= (float64_t)dv;
    dv = (gKalmanFilter.R_BinN[1][0] * leverArmN[0]) +
         (gKalmanFilter.R_BinN[1][1] * leverArmN[1]) +
         (gKalmanFilter.R_BinN[1][2] * leverArmN[2]);
    vNed[1] -= (float64_t)dv;
    dv = (gKalmanFilter.R_BinN[2][0] * leverArmN[0]) +
         (gKalmanFilter.R_BinN[2][1] * leverArmN[1]) +
         (gKalmanFilter.R_BinN[2][2] * leverArmN[2]);
    vNed[2] -= (float64_t)dv;

    // calcualte transfromation matrix from NED to ECEF
    sinLat = sin(lla[LAT]); // recalculate with LLA without lever arm
    cosLat = cos(lla[LAT]);
    float64_t const sinLon = sin(lla[LON]);
    float64_t const cosLon = cos(lla[LON]);

    real const sinLat_r = (real)sinLat;
    real const cosLat_r = (real)cosLat;
    real const sinLon_r = (real)sinLon;
    real const cosLon_r = (real)cosLon;

    // Form the transformation matrix from NED to ECEF 
    // First row
    rn2e[0][0] = -sinLat_r * cosLon_r;
    rn2e[0][1] = -sinLon_r;
    rn2e[0][2] = -cosLat_r * cosLon_r;
    // Second row
    rn2e[1][0] = -sinLat_r * sinLon_r;
    rn2e[1][1] = cosLon_r;
    rn2e[1][2] = -cosLat_r * sinLon_r;
    // Third row
    rn2e[2][0] = cosLat_r;
    rn2e[2][1] = 0.0F;
    rn2e[2][2] = -sinLat_r;

    // calculate ECEF position
    tmp = (rn + lla[ALT]) * cosLat;
    ecef[X_AXIS] = tmp * cosLon;
    ecef[Y_AXIS] = tmp * sinLon;
    ecef[Z_AXIS] = ((E_MINOR_OVER_MAJOR_SQ * (rn)) + lla[ALT]) * sinLat;
}

/******************************************************************************
 * @brief Initialize the EKF algorithm.
 *
 * @param dacqFreq  [in]    Calling frequency. In units of [Hz]. For now,
 *                          only 100Hz and 200Hz are supported.
 * @param imuType   [in]    Specify the IMU type to choose different set of
 *                          IMU specifications.
 *  
 *****************************************************************************/
void EKF_Initialize(uint32_t const dacqFreq, enumIMUType const imuType)
{
    MagAlign_Init();

    // Initialize built-in algorithm structure
    InitializeAlgorithmStruct(dacqFreq, imuType);
    
    // Update algorithm structure by user configurations.
    UpdateUserAlgoCfg();

    // Initialize the timing variables and set the odr based on the system type
    Initialize_Timing();
    TimingVars_dacqFrequency(dacqFreq);
}

/******************************************************************************
 * @brief Update algorithm configurations by user configurations that are 
 *        read from EEPROM.
 * 
 *****************************************************************************/
void UpdateUserAlgoCfg()
{
    real tmp = GetAlgorithmAccelSwitchDelay();
    SetAccelSwitchDelay((uint32_t)round(tmp));

    tmp = GetAlgorithmRateIntegrationTime();
    SetRateIntegrationTime((uint32_t)round(tmp));

    SetLinAccelDetectMode(GetAlgorithmLinAccelDetectMode());

    SetAccelPredictMode(GetAlgorithmAccelPredictMode());

    SetCoefOfReduceQ(GetAlgorithmCoefOfReduceQ());

    SetOdoUserCfg(GetOdometerCfgPtr());

    if (odoUsedInAlgorithm() && (STATIC_WRT_BODY == gAlgorithm.odoCfg.mountLocation))
    {
        UpdateCoefOfReduceQ(&gAlgorithm.coefOfReduceQ, TRUE);
    }
}

/******************************************************************************
 * @brief Get the algorithm timer.
 *
 * @return uint32_t
 *****************************************************************************/
uint32_t EKF_GetTimer()
{
    return getAlgorithmTimer();
}

/******************************************************************************
 * @brief Get the algorithm counter.
 *
 * @return uint16_t
 *****************************************************************************/
uint16_t EKF_GetCounter()
{
    return getAlgorithmCounter();
}

/******************************************************************************
 * @brief Get the execution frequency of the algorithm.
 *
 * @return uint32_t
 *****************************************************************************/
uint32_t EKF_GetCallingFreq()
{
    return getAlgorithmFrequency();
}

#ifdef UNIT_TEST
#include "wrappers_EKF_Algorithm.h"
#endif
