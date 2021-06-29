/******************************************************************************
 * @file MotionStatus.h
 * @author Xiaoguang Dong (xgdong@aceinna.com)
 * @brief Calculate sensor stats, and detect motion status using IMU/ODO/GNSS
 * @version 1.0
 * @date 2019-08-01
 * 
 * @copyright Copyright (c) 2020
 * 
 *****************************************************************************/


#ifndef MOTION_STATUS_H_INCLUDED
#define MOTION_STATUS_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "GlobalConstants.h"
#include "algorithm.h"
#include "buffer.h"


typedef struct
{
    BOOL bValid;        // tell if stats are valid
    BOOL bStaticIMU;    // Static period detected by IMU
    BOOL accelErrLimit; // accelErr is set to max/min limit
    real lpfAccel[3];   // [m/s/s], low-pass filtered accel
    real accelNorm;     // [m/s/s], magnitude of current accel
    real accelMean[3];  // [m/s/s], average of past n accel samples
    real accelVar[3];   // [m/s/s]^2, variance of past n accel samples
    real accelErr[3];   // [m/s/s], estimated accel error
    real lpfGyro[3];    // [rad/s], low-pass filtered gyro
    real gyroMean[3];   // [rad/s], average of past n gyro samples
    real gyroVar[3];    // [rad/s]^2, variance of past n gyro samples
} ImuStatsStruct;

extern ImuStatsStruct          gImuStats;

// Filter variables (Third-Order BWF w/ default 5 Hz Cutoff)
enum{ FILTER_ORDER = 3 }; 

enum {
    CURRENT = 0,
    PASTx1  = 1,
    PASTx2  = 2,
    PASTx3  = 3
};

/* Replace this with a fuction that will compute the coefficients so the
 * input is the cutoff frequency in Hertz
 */
enum {
    NO_LPF              = 0,
    TWO_HZ_LPF          = 1,
    FIVE_HZ_LPF         = 2,
    TEN_HZ_LPF          = 3,
    TWENTY_HZ_LPF       = 4,
    TWENTY_FIVE_HZ_LPF  = 5,
    N_LPF               = 6
};

enum { SAMPLES_FOR_STATS = 20 };    /* 20 samples can give a relative good estimate of var
                                     * This value should not be below FILTER_ORDER.
                                     */

/******************************************************************************
 * @brief Get filter coefficients of a 3rd Butterworth low-pass filter.
 *        For now only a few specific cut-off frequencies are supported.
 * 
 * @param lpfType       [in]    Low-pass filter cut-off frequency.
 * @param callingFreq   [in]    Sampling frequency, only 100Hz and 200Hz are supported.
 * @param b             [out]   coefficients of the numerator of the filter.
 * @param a             [out]   coefficients of the denominator of the filter.
 *  
 *****************************************************************************/
void PopulateFilterCoefficients(uint8_t const lpfType, uint32_t const callingFreq,
                                real b[], real a[]);

/******************************************************************************
 * @brief Process input data through a low-pass Butterworth filter.
 * 
 * @param in        [in]    Input data
 * @param bfIn      [in]    Input data buffer
 * @param bfOut     [in]    Output data buffer
 * @param b         [in]    Numerator coef of the filter 
 * @param a         [in]    Denominator coef of the filter
 * @param filtered  [out]   Filtered IMU data.
 *  
******************************************************************************/
void LowPassFilter(real in[], Buffer* const bfIn, const Buffer* const bfOut,
                   real b[], real a[], real filtered[]);

/******************************************************************************
 * @brief Compute mean and var of the input data.
 *        Calculate mena and var of the latest n samples.
 * 
 * @param bf        [in]        The buffer to hold the latest n samples.
 * @param latest    [in]        The latest sample.
 * @param mean      [in/out]    Mean of samples already in buffer is used as
 *                              input, and mean of latest samples (remove oldest
 *                              and include latest) is returned as output.
 * @param var       [in/out]    Var of samples already in buffer is used as input,
 *                              and var of latest samples (remove oldest and
 *                              include latest) is returned as output.
 *  
******************************************************************************/
void ComputeStats(Buffer* const bf, real latest[], real mean[], real var[]);

/******************************************************************************
 * @brief Detect zero velocity using IMU data.
 * 
 * @param gyroVar   [in]    variance of gyro    [rad/s]^2
 * @param gyroMean  [in]    mean of gyro        [rad/s]
 * @param accelVar  [in]    variance of accel   [m/s/s]^2
 * @param threshold [in]    threshold to detect zero velocity
 * @return TRUE if static, otherwise FALSE.
******************************************************************************/
BOOL DetectStaticIMU(real gyroVar[], real gyroMean[], real accelVar[],
                     const STATIC_DETECT_SETTING* const threshold);

/******************************************************************************
 * @brief Calculate IMU data stats, and detect zero velocity.
 * 
 * @param gyro      [in]    Input gyro rate, [rad/s]
 * @param accel     [in]    Input acceleration, [m/s/s]
 * @param reset     [in]    TRUE to reset this process, FALSE not.
 * @param imuStats  [Out]   Statistics of IMU data.
 *  
******************************************************************************/
void MotionStatusImu(real gyro[], real accel[],
                     ImuStatsStruct* const imuStats, BOOL const reset);

/******************************************************************************
 * @brief Using gyro propagation to estimate accel error.
 *        g_dot = -cross(w, g), g is gravity and w is angular rate.
 * 
 * @param accel                 [in]    Input accel, [m/s/s].
 * @param w                     [in]    Input angular rate, [rad/s].
 * @param dt                    [in]    Sampling interval, sec.
 * @param staticDelay           [in]    A Counter. When static period detected, delay
 * @param rateIntegrationTime   [in]    A Counter. Rate Integration Time for LAD.
 * @param rateBias              [staticDelay] samples before lowering accel error.
 *                              [staticDelay] is also used to reset initial accel
 *                              propagated using gyro to estimate future accel.
 * @param imuStats  [out]       A struct for results storage.
 *  
******************************************************************************/
void EstimateAccelError(real accel[], real w[], real const dt,
                        uint32_t const staticDelay, uint32_t const rateIntegrationTime, 
                        real rateBias[], ImuStatsStruct* const imuStats);

/******************************************************************************
 * @brief Detect motion according to the difference between measured accel.
 *        magnitude and 1g. Set gAlgorithm.linAccelSwitch to be True if being
 *        static for a while.
 * 
 * @param accelNorm [in]    Input accel magnitude, [g].
 * @param iReset    [in]    Reset the procedure.
 *  
******************************************************************************/
void DetectMotionFromAccel(real const accelNorm, int32_t const iReset);

/******************************************************************************
 * @brief Detect zero velocity using GNSS speed.
 * 
 * @param vNED      [in]    NED velocity measured by GNSS, [m/s]
 * @param gnssValid [in]    Indicate if GNSS measurement is valid.
 * @param threshold         If valid, vNED will be used to detect zero velocity.
 *                          If not, detection will be reset and FALSE is always
 *                          returned.
 * @return TRUE if static, otherwise FALSE.
******************************************************************************/
BOOL DetectStaticGnssVelocity(float64_t vNED[], real const threshold,
                              uint8_t const gnssValid);

/******************************************************************************
 * @brief Detect zero velocity using odometer data.
 * 
 * @param odoCfg    [in]    Configuration of odometer.
 * @param odo       [in]    Odometer data.
 * @param odoStatus [out]   Status of odometer.
 * @return BOOL, TRUE if static, other FALSE.
 ******************************************************************************/
BOOL DetectStaticOdo(const OdoAlgoCfgStruct_t* const odoCfg,
                     const odoDataStruct_t* const odo, OdoStatusStruct* const odoStatus);


#ifdef __cplusplus
}
#endif

#endif /* MOTION_STATUS_H_INCLUDED */
