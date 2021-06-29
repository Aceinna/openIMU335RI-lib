/** ***************************************************************************
 * @file odoApi.h Odometer interface.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * @brief This is a generalized odometer interface.
 *****************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/

#ifndef ODO_API_H
#define ODO_API_H

#include <stdint.h>
#include "GlobalConstants.h"
#include "Indices.h"
// Aiding signal source type.
typedef enum {
    NO_SOURCE         = 0,          // There is no Aiding signal for algorithm.
    ODOMETER          = 1,          // Odometer signal.
    VEHICLE_ACCEL     = 2           // Vehicle accelerations signal.
}SignalSourceType;

//OdoCfgSwitch bits
#define ODO_MOUNT_LOC_MASK   (1 << 0)
#define ODO_MOUNT_DIR_MASK   (1 << 2)
// add odoCfgSwitch bits here and update below mask for reserved bits
#define ODO_RESERVED_MASK    (0xFA)

// IMU Mount Location.
typedef enum {
    STATIC_WRT_BODY      = 0,      // Unit cannot rotate relative vehicle body, such as unit is mounted on vehicle body or chassis.
    ROTATE_WRT_BODY      = 1       // Unit can rotate relative vehicle body, such as unit is mounted on backhoe's arm or bucket.
} MountLocation;

typedef enum {
    TRANSMISSION_PREVIOUS    = 0,          // Current transmission state is same as previous
    TRANSMISSION_PARK        = 1,          // Current transmission state is P
    TRANSMISSION_REVERSE     = 2,          // Current transmission state is R
    TRANSMISSION_FORWARD     = 3,          // Current transmission state is D
    TRANSMISSION_NUTRAL      = 4,          // Current transmission state is D
	TRANSMISSION_MAX         = 5
}TransmissionType;

typedef struct  {
    BOOL update;         // TURE if contains new data
    real v;              // velocity measured by odometer, [m/s]
    real vehAccel[3];    // vehicle accelerations, [m/s^2]
} odoDataStruct_t;

typedef struct  {
    SignalSourceType    signalSource;           // Aiding signal type.
    uint8_t             msgRate;                // Message rate of aiding signal.
    real                leverArmB[NUM_AXIS];    // [m]
    uint8_t       		odoCfgSwitch;          	// IMU mount location.
} OdoCfgStruct_t;

void OdoInit();

/*******************************************
 * @brief 
 * 
 * @param leverArm ==
********************************************/
void OdoUpdateLeverArmConfig(int16_t leverArm[3]);

/*******************************************
 * @brief 
 * 
 * @param aidingMsgRate ==
 * @param signalSource ==
 * @param odoCfgSwitch ==
********************************************/
void OdoUpdateConfig(uint8_t aidingMsgRate, uint8_t signalSource, uint8_t odoCfgSwitch);

/*******************************************
 * @brief Process the Transmission Data
 *
 * @param Odo == selectedGear
********************************************/

void OdoUpdateTransmission(TransmissionType CurrTransmission);

/*******************************************
 * @brief Process WheelSpeed/VehicalSpeed Data
 *
 * @param velocity == current calculated delocity
********************************************/

void OdoUpdateVelocity(real velocity);

/*******************************************
 * @brief Process the machine accel Message
 *
 * @param Odo ==
********************************************/
void OdoUpdateVehicalAccel(real vehAccel[NUM_AXIS]);

/*******************************************
 * @brief Get the Odometer Data object
 *
 * @param Odo ==
********************************************/
void GetOdometerData(odoDataStruct_t* const Odo);

/*******************************************
 * @brief Get the Odometer Ptr object
 *
 * @return odoDataStruct_t*
********************************************/
odoDataStruct_t*    GetOdometerPtr();

/*******************************************
 * @brief Get the Odometer Cfg Ptr object
 *
 * @return OdoCfgStruct_t*
********************************************/
OdoCfgStruct_t*    GetOdometerCfgPtr();

#endif /* ODO_API_H */
