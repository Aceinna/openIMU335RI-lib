/*****************************************************************************
 * @file    ProcessODO.h
 * @brief   Integrating aiding signal in algorithm.
 * @author  Song Yang (ysong@aceinna.com)
 * @version 1.0
 * @date    2020-12-10
 *
 * @copyright Copyright (c) 2020
 *
 *****************************************************************************/


#ifndef PROCESSODO_H
#define PROCESSODO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "GlobalConstants.h"
#include "Indices.h"
#include "odoAPI.h"

typedef struct
{
    BOOL bStaticODO;                 // Flag, used to indicate unit is in motion or not which is detected by odometer
    real vel[NUM_AXIS];              // Velocity of the point of IMU.
    real accel[NUM_AXIS];            // [m/s/s], linear and centripetal acceleration calculated by raw velocity or Vehicle accelerations.
    BOOL bOverTime;                  // TRUE, when odometer outage duration exceeds limit 'maxReliableODTime'
} OdoStatusStruct;

typedef struct  {
    SignalSourceType signalSource;   // Aiding signal type.
    uint8_t  msgRate;                // Message rate of aiding signal.
    real leverArmB[NUM_AXIS];        // [m]
    MountLocation  mountLocation;    // IMU mount location.
    uint16_t maxReliableODTime;      /* [msec], When odometer outage duration exceeds this limit, the 
                                      * Odometer status will be reinitialized when odometer is available again.
                                      */
} OdoAlgoCfgStruct_t;

/******************************************************************************
 * @brief Integrating aiding signal which includes Odometer and Vehicle Accelerations.
 *
 * @param odo        [in]  Odometer or vehicle accelerations data.
 * @param odoCfg     [in]  Configurations for integrating aiding signal.
 * @param t          [in]  internal timestamp, [msec]
 * @param dt         [in]  Sampling interval, [sec].
 * @param rate       [in]  IMU rate data, [rad/s].
 * @param accel      [out] IMU accel data, [m/s^2].
 * @param odoStatus  [out] Status of odometer.
 ******************************************************************************/
void IntegrateOdo(const odoDataStruct_t* const odo, OdoAlgoCfgStruct_t* const odoCfg, uint32_t const t,
                real const dt, real rate[], real accel[], OdoStatusStruct* const odoStatus);

/******************************************************************************
 * @brief Update .coefOfReduceQ dynamically. 
 * 
 * @param coe        [in/out]   Coefficient of reducing rate bias component in Q.
 * @param reset      [in]       TRUE to reset this process, FALSE not.
******************************************************************************/
void UpdateCoefOfReduceQ(real* const coe, BOOL const reset);

#ifdef UNIT_TEST
#include "wrappers.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* PROCESSODO_H */
