/*****************************************************************************
 * @file    ProcessODO.c
 * @brief   Integrating aiding signal in algorithm.
 * @author  Song Yang
 * @version 1.0
 * @date    2020-12-10
 *****************************************************************************/
#include <string.h>
#include <math.h>
#include "ProcessODO.h"
#include "MotionStatus.h"
#include "VectorMath.h"

/******************************************************************************
 * @brief Calculate linear and centripetal acceleration by odometer.
 * 
 * @param odo        [in]   Odometer data.
 * @param odoCfg     [in]   Configurations of algorithm.
 * @param rate       [in]   3D angular-rate signal in the body-frame, [rad/s].
 * @param odoStatus  [out]  Status of odometer.
******************************************************************************/
static void CalAccelByOdo(const odoDataStruct_t* const odo, OdoAlgoCfgStruct_t* const odoCfg,
                        real rate[], OdoStatusStruct* const odoStatus);

/******************************************************************************
 * @brief Calculate linear and centripetal acceleration by vehicle accelerations.
 * 
 * @param odo        [in]   vehicle accelerations data.
 * @param odoCfg     [in]   Configurations of algorithm.
 * @param rate       [in]   3D angular-rate signal in the body-frame, [rad/s].
 * @param odoStatus  [out]  Status of odometer.
******************************************************************************/
static void CalAccelByVehicleAccel(const odoDataStruct_t* const odo, OdoAlgoCfgStruct_t* const odoCfg,
                        real rate[], OdoStatusStruct* const odoStatus);

/******************************************************************************
 * @brief Calculate linear and centripetal acceleration by rotation radius.
 * 
 * @param rate       [in]   3D angular-rate signal in the body-frame, [rad/s].
 * @param leverArmB  [in]   Lever arm of pivot point, [m].
 * @param dt         [in]   Sampling interval, sec.  
 * @param reset      [in]   TRUE to reset this process, FALSE not.
 * @param accel      [out]  3D IMU Accelerometer signal, [m/s^2].
******************************************************************************/
static void CalAccelByLeverArm(real rate[], real leverArmB[], real const dt, BOOL const reset, real accel[]);

/******************************************************************************
 * @brief Remove linear and centripetal acceleration from IMU accel sensor reading.
 * 
 * @param odoStatus  [in]   Status of odometer.
 * @param accel      [out]  3D IMU Accelerometer signal, [m/s^2].
******************************************************************************/
static void RemoveLinearAccel(const OdoStatusStruct* const odoStatus, real* accel);

/******************************************************************************
 * @brief Remove lever arm effects from velocity.
 * 
 * @param odo        [in]   Odometer data.
 * @param rate       [in]   3D angular-rate signal in the body-frame, [rad/s].
 * @param leverArmB  [in]   Lever arm in the body frame, [m].
 * @param vel        [out]  Status of odometer, [m/s].
******************************************************************************/
static void RemoveLeverArmInVel(const odoDataStruct_t* const odo, real rate[], real leverArmB[], real vel[]);

/******************************************************************************
 * @brief Check aiding signal is over time or not.
 * 
 * @param odo        [in]   Aiding signal data.
 * @param odoCfg     [in]   Configurations of algorithm.
 * @param t          [in]   Timestamp, [msec].
 * @param odoStatus  [out]  Status of odometer.
******************************************************************************/
static void IsAidingSignalOverTime(const odoDataStruct_t* const odo, const OdoAlgoCfgStruct_t* const odoCfg,
                                    uint32_t const t, OdoStatusStruct* const odoStatus);

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
                real const dt, real rate[], real accel[], OdoStatusStruct* const odoStatus)
{
    if (NO_SOURCE == odoCfg->signalSource) // No aiding signal.
    {
        return;
    }

    IsAidingSignalOverTime(odo, odoCfg, t, odoStatus);
    odoStatus->bStaticODO = DetectStaticOdo(odoCfg, odo, odoStatus);

    if (STATIC_WRT_BODY == odoCfg->mountLocation)
    {
        if (ODOMETER == odoCfg->signalSource)
        {
            CalAccelByOdo(odo, odoCfg, rate, odoStatus);
        }
        else if (VEHICLE_ACCEL == odoCfg->signalSource)
        {
            CalAccelByVehicleAccel(odo, odoCfg, rate, odoStatus);
        }
        else
        {
        	// Do nothing.
        }
        RemoveLinearAccel(odoStatus, accel);
    }
    else if (ROTATE_WRT_BODY == odoCfg->mountLocation)
    {
        CalAccelByLeverArm(rate, odoCfg->leverArmB, dt, FALSE, odoStatus->accel);
        if (odoStatus->bStaticODO)
        {
            RemoveLinearAccel(odoStatus, accel);
        }
    }
    else
    {
    	// Do nothing.
    }
}

/******************************************************************************
 * @brief Calculate linear and centripetal acceleration by odometer.
 *
 * @param odo        [in]   Odometer data.
 * @param odoCfg     [in]   Configurations of algorithm.
 * @param rate       [in]   3D angular-rate signal in the body-frame, [rad/s].
 * @param odoStatus  [out]  Status of odometer.
******************************************************************************/
static void CalAccelByOdo(const odoDataStruct_t* const odo, OdoAlgoCfgStruct_t* const odoCfg,
                        real rate[], OdoStatusStruct* const odoStatus)
{
    static BOOL bIni = FALSE;

    if(odo->update)
    {
        static real velLast[NUM_AXIS] = {0.0F};     // Last odometer data.

        if(!bIni)
        {
            bIni = TRUE;
            RemoveLeverArmInVel(odo, rate, odoCfg->leverArmB, odoStatus->vel);
            memcpy(velLast, odoStatus->vel, (uint16_t)NUM_AXIS * sizeof(real));
            memset(odoStatus->accel, 0, (uint16_t)NUM_AXIS * sizeof(real));
            return;
        }

        // Remove lever arm effects from velocity
        RemoveLeverArmInVel(odo, rate, odoCfg->leverArmB, odoStatus->vel);

        // Calculate linear and centripetal acceleration.
        cross(rate, odoStatus->vel, odoStatus->accel);
        odoStatus->accel[X_AXIS] += (odoStatus->vel[X_AXIS] - velLast[X_AXIS]) * (real)(odoCfg->msgRate);
        odoStatus->accel[Y_AXIS] += (odoStatus->vel[Y_AXIS] - velLast[Y_AXIS]) * (real)(odoCfg->msgRate);
        odoStatus->accel[Z_AXIS] += (odoStatus->vel[Z_AXIS] - velLast[Z_AXIS]) * (real)(odoCfg->msgRate);

        memcpy(velLast, odoStatus->vel, (uint16_t)NUM_AXIS * sizeof(real));
    }
    else if (odoStatus->bOverTime)
    {
        bIni = FALSE;
        memset(odoStatus->accel, 0, (uint16_t)NUM_AXIS * sizeof(real));
    }
    else
    {
        // Do nothing.
    }
}

/******************************************************************************
 * @brief Calculate linear and centripetal acceleration by vehicle accelerations.
 *
 * @param odo        [in]   vehicle accelerations data.
 * @param odoCfg     [in]   Configurations of algorithm.
 * @param rate       [in]   3D angular-rate signal in the body-frame, [rad/s].
 * @param odoStatus  [out]  Status of odometer.
******************************************************************************/
static void CalAccelByVehicleAccel(const odoDataStruct_t* const odo, OdoAlgoCfgStruct_t* const odoCfg,
                        real rate[], OdoStatusStruct* const odoStatus)
{
    static BOOL bIni = FALSE;

    if(odo->update)
    {
        static real vlLast[NUM_AXIS] = {0.0F}; // Lever arm effects for velocity of last time.
        real vl[NUM_AXIS] = {0.0F};            // Lever arm effects for velocity.
        real al[NUM_AXIS] = {0.0F};            // Lever arm effects for acceleration.
        cross(rate, odoCfg->leverArmB, vl); // w x leverArmB

        if(!bIni)
        {
            bIni = TRUE;
            memcpy(vlLast, vl, (uint16_t)NUM_AXIS * sizeof(real));
            memcpy(odoStatus->accel, odo->vehAccel, (uint16_t)NUM_AXIS * sizeof(real));
            return;
        }

        // a_l = dv_l/dt
        al[X_AXIS] = (vl[X_AXIS] - vlLast[X_AXIS]) * (real)(odoCfg->msgRate);
        al[Y_AXIS] = (vl[Y_AXIS] - vlLast[Y_AXIS]) * (real)(odoCfg->msgRate);
        al[Z_AXIS] = (vl[Z_AXIS] - vlLast[Z_AXIS]) * (real)(odoCfg->msgRate);
        memcpy(vlLast, vl, (uint16_t)NUM_AXIS * sizeof(real));
        
        // Remove lever arm effects from accel at the location of IMU.
        // a_imu = a_veh - a_l
        odoStatus->accel[X_AXIS] = odo->vehAccel[X_AXIS] - al[X_AXIS];
        odoStatus->accel[Y_AXIS] = odo->vehAccel[Y_AXIS] - al[Y_AXIS];
        odoStatus->accel[Z_AXIS] = odo->vehAccel[Z_AXIS] - al[Z_AXIS];
    }
    else if (odoStatus->bOverTime)
    {
        bIni = FALSE;
        memset(odoStatus->accel, 0, (uint16_t)NUM_AXIS * sizeof(real));
    }
    else
    {
        // Do nothing.
    }
}

/******************************************************************************
 * @brief Calculate linear and centripetal acceleration by rotation radius.
 * 
 * @param rate       [in]   3D angular-rate signal in the body-frame, [rad/s].
 * @param leverArmB  [in]   Lever arm of pivot point, [m].
 * @param dt         [in]   Sampling interval, sec.  
 * @param reset      [in]   TRUE to reset this process, FALSE not.
 * @param accel      [out]  3D IMU Accelerometer signal, [m/s^2].
******************************************************************************/
static void CalAccelByLeverArm(real rate[], real leverArmB[], real const dt, BOOL const reset, real accel[])
{
    static BOOL bIni = FALSE;
    static real velLast[NUM_AXIS] = {0.0F};     // Last odometer data.
    real vel[NUM_AXIS] = {0.0F};

    /*
    * leverArmB is a vector which from IMU center to pivot point,
    * while R is a vector which from pivot point to IMU center, so
    * R = -leverArmB
    */
    real rotation_radius[NUM_AXIS] = {0.0F};
    rotation_radius[X_AXIS] = -1.0F * leverArmB[X_AXIS];
    rotation_radius[Y_AXIS] = -1.0F * leverArmB[Y_AXIS];
    rotation_radius[Z_AXIS] = -1.0F * leverArmB[Z_AXIS];

    // reset the routing.
    if (reset)
    {
        bIni = FALSE;
    }

    if(!bIni)
    {
        bIni = TRUE;
        cross(rate, rotation_radius, velLast);
        memset(accel, 0, (uint16_t)NUM_AXIS * sizeof(real));
        return;
    }

    /*
    * v = w x R,   where v is velocity of IMU, 
    *              R is the vector which from pivot point to IMU center.
    * 
    * a = at + an, where 'a' is the resultant acceleration caused by rotation.
    *              'at' is tangential acceleration, 
    *              'an' is centripetal acceleration.
    * 
    * at = dv/dt, an = w x v, 
    */
    cross(rate, rotation_radius, vel); //  v = w x R
    cross(rate, vel, accel);           // an = w x v
    accel[X_AXIS] += (vel[X_AXIS] - velLast[X_AXIS]) / dt; // a = at + an = dv/dt + w x v
    accel[Y_AXIS] += (vel[Y_AXIS] - velLast[Y_AXIS]) / dt;
    accel[Z_AXIS] += (vel[Z_AXIS] - velLast[Z_AXIS]) / dt;

    memcpy(velLast, vel, (uint16_t)NUM_AXIS * sizeof(real));
}

/******************************************************************************
 * @brief Remove linear and centripetal acceleration from IMU accel sensor reading.
 *
 * @param odoStatus  [in]   Status of odometer.
 * @param accel      [out]  3D IMU Accelerometer signal, [m/s^2].
******************************************************************************/
static void RemoveLinearAccel(const OdoStatusStruct* const odoStatus, real accel[])
{
    // Remove Linear and centripetal acceleration from accel sensor reading.
    accel[X_AXIS] -= odoStatus->accel[X_AXIS];
    accel[Y_AXIS] -= odoStatus->accel[Y_AXIS];
    accel[Z_AXIS] -= odoStatus->accel[Z_AXIS];
}

/******************************************************************************
 * @brief Remove lever arm effects from velocity.
 *
 * @param odo        [in]   Odometer data.
 * @param rate       [in]   3D angular-rate signal in the body-frame, [rad/s].
 * @param leverArmB  [in]   Lever arm in the body frame, [m].
 * @param vel        [out]  Status of odometer, [m/s].
******************************************************************************/
static void RemoveLeverArmInVel(const odoDataStruct_t* const odo, real rate[], real leverArmB[], real vel[])
{
    real vl[3]; // Lever arm effects for velocity.
    cross(rate, leverArmB, vl); // v = w x l

    // Remove lever arm effects from velocity.
    vel[X_AXIS] = odo->v - vl[X_AXIS];
    vel[Y_AXIS] = - vl[Y_AXIS];
    vel[Z_AXIS] = - vl[Z_AXIS];
}

/******************************************************************************
 * @brief Check aiding signal is over time or not.
 * 
 * @param odo        [in]   Aiding signal data.
 * @param odoCfg     [in]   Configurations of algorithm.
 * @param t          [in]   Timestamp, [msec].
 * @param odoStatus  [out]  Status of odometer.
******************************************************************************/
static void IsAidingSignalOverTime(const odoDataStruct_t* const odo, const OdoAlgoCfgStruct_t* const odoCfg,
                                    uint32_t const t, OdoStatusStruct* const odoStatus)
{
    static uint32_t timeStampOfUpdate = 0U; // [msec], timestamp of latest vehicle acceleration data.

    if(odo->update)
    {
        timeStampOfUpdate = t;
        odoStatus->bOverTime = FALSE;
    }
    else
    {
        /* If we can't get a new aiding signal data over 'maxReliableODTime',
         * this means the aiding signal data is too old to use and over time. 
         */
        if(fabs(((float64_t)t - (float64_t)timeStampOfUpdate)) > (float64_t)odoCfg->maxReliableODTime)
        {
            odoStatus->bOverTime = TRUE;
        }
    }
}

/******************************************************************************
 * @brief Update .coefOfReduceQ dynamically. 
 * 
 * @param coe        [in/out]   Coefficient of reducing rate bias component in Q.
 * @param reset      [in]       TRUE to reset this process, FALSE not.
******************************************************************************/
void UpdateCoefOfReduceQ(real* const coe, BOOL const reset)
{
    static real userCoefOfReduceQ = 0.0F;
    real const coeOfQwbOdo        = 1.0F;
    real const coeOfQwbStatic     = 0.05F;
    static BOOL bIni              = FALSE;

    // reset the routing.
    if (reset)
    {
        bIni = FALSE;
    }

    if(!bIni)
    {
        bIni = TRUE;
        userCoefOfReduceQ = *coe; // Save .coefOfReduceQ read from EEPROM.
        return;
    }

    if (odoUsedInAlgorithm() && (STATIC_WRT_BODY == gAlgorithm.odoCfg.mountLocation))
    {
        if (FALSE == gAlgorithm.odoStatus.bOverTime)
        {
            *coe = coeOfQwbOdo;
            return;
        }
    }
    else
    {
        if (TRUE == gAlgorithm.linAccelSwitch)
        {
            *coe = coeOfQwbStatic;
            return;
        }
    }

    *coe = userCoefOfReduceQ;
}

#ifdef UNIT_TEST
#include "wrappers_ProcessODO.h"
#endif
