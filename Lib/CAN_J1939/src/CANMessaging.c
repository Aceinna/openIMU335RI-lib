/** ***************************************************************************
 * @file   UserConfiguration.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 ******************************************************************************/
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

#include <string.h>
#include <stdint.h>
#include "math.h"
#include "EcuSettings.h"
#include "sensorsAPI.h"
#include "Indices.h"
#include "canJ1939API.h"
#include "ekfAPI.h"
#include "halAPI.h"
#include "osapi.h"
#include "bitAPI.h"
#include "configurationAPI.h"

extern EcuConfigurationStruct         *gEcuConfigPtr;

/** ***************************************************************************
 * @name  ecu_corolaAidingSig_CAN_message_filters
 * @brief Configures CAN filter for corola aiding signal messages
 * In order to configure any filter once, this method checks for a specific combination of aiding/drivingDir PF/PS values
 * Eg.
 * Message          | Filter enable condition
 * -----------------|--------------------------
 * PGN 170 & 956    | Both these message IDs are Corola specific.
 *                  | 170 is aiding message and 965 is driveDir message.
 *                  | If both aiding & driveDir pf/ps matches these pgns respective then enable the filter. (Filter enables both message reception)
 *
******************************************************************************/

void ecu_corolaAidingSig_CAN_message_filters(uint8_t aiding_pf_val, uint8_t aiding_ps_val, uint8_t driveDir_pf_val, uint8_t driveDir_ps_val){
    // Same filter for both Corola messages 956 & 170
    if (((driveDir_pf_val == STANDARD_ID_COROLA_GEAR) && (driveDir_ps_val == STANDARD_ID_COROLA_GEAR_188))
        && ((aiding_pf_val == STANDARD_ID_BELOW_256) && (aiding_ps_val == STANDARD_ID_COROLA_WS_170))){
        // Standard ID filters are Identifier Filter Mode 0pposed to the Mask Filter Mode for Extended ID
        // StandardID: 956 & StandardID: 170
        // Corola Gear & Wheel Speed data messaages
        // Combined two filter values into one, Gear & WheelSpeed Messages
        uint32_t const COROLA_GEAR_WS_DATA_ID_BASE = 0x03BC00AAU;
        uint32_t const COROLA_GEAR_WS_DATA_FILTER_BASE_MASK = 0x00000000U;
        // corola gear info 956
        // initialize Filters for incoming Corola Transmission data message & Corola WheelSpeed data message
	    CAN_ConfigureMessageFilter(COROLA_GEAR_WS_DATA_ID_BASE, COROLA_GEAR_WS_DATA_FILTER_BASE_MASK, FALSE);
    }
}

// configure CAN controller for selective reception of CAN messages
/** ***************************************************************************
 * @name  ConfigureCANMessageFilters
* @brief configure CAN controller for selective reception of CAN messages
******************************************************************************/
void ecu_configure_CAN_message_filters()
{
    // priority 6
    // PF 255
    // PS 240 and 241
    uint32_t const ACEINNA_BANK_ID_BASE = 0x18FFF000U;
    uint32_t const ACEINNA_BANK_FILTER_BASE_MASK = 0x18FFF000U;

    // priority 6
    // PF 238
    // PS 255
    uint32_t const SAE_J1939_ADDRESS_CLAIM_ID_BASE = 0x18EEFF00U;
    uint32_t const SAE_J1939_ADDRESS_CLAIM_FILTER_BASE_MASK = 0x18FFFF00U;

    // priority 6
    // PF 234
    // PS 255  - global requests
    uint32_t const SAE_J1939_REQUEST_ID_BASE = 0x18EAFF00U;
    uint32_t const SAE_J1939_REQUEST_FILTER_BASE_MASK = 0x18EAFF00U;

    // priority 6
    // PF 255
    // PS 50 - 5F
    uint32_t const SAE_J1939_CONTROL1_ID_BASE = 0x18FFFF00U;
    uint32_t const SAE_J1939_CONTROL1_FILTER_BASE_MASK = 0x18FF4000U;

    // priority 6
    // PF 253
    // PS 197
    uint32_t const SAE_J1939_ECU_ID_BASE = 0x18FDC500U;
    uint32_t const SAE_J1939_ECU_FILTER_BASE_MASK = 0x18FDC500U;

    // priority 2
    // PF 217, 235, 236
    // PS all
    uint32_t const VEHICLE_BOOT_ID_BASE = 0x08FF0000U;
    uint32_t const VEHICLE_BOOT_FILTER_BASE_MASK = 0x08C80000U;

    // transmission controller info 61445
    // priority 6
    // PF 240
    // PS 05
    // Transmission Data Message
    uint32_t const SAE_J1939_TRANSMISSION_DATA_ID_BASE = 0x18F00500U;
    uint32_t const SAE_J1939_TRANSMISSION_DATA_FILTER_BASE_MASK = 0x18F00500U;

    // wheel speed info 65215 and cruise control message 65265
    // priority 6
    // PF 254
    // PS 191 & 241
    // Vehical Speed aand Wheel Speed data messaage
    uint32_t const SAE_J1939_SPEED_DATA_ID_BASE = 0x18FEFF00U;
    uint32_t const SAE_J1939_SPEED_DATA_FILTER_BASE_MASK = 0x18FEB100U;

    // transmission controller info 126720
    // priority 6
    // DP 1
    // PF 239
    // PS 00
    // JD specific Machine Acceleration Data
    uint32_t const SAE_J1939_ACCEL_DATA_ID_BASE = 0x19EF0000U;
    uint32_t const SAE_J1939_ACCEL_DATA_FILTER_BASE_MASK = 0x19EF0000U;

    // initialize filter for ECU ID
    CAN_ConfigureMessageFilter(SAE_J1939_ECU_ID_BASE, SAE_J1939_ECU_FILTER_BASE_MASK, TRUE);
    // initialize filter for control messages
    CAN_ConfigureMessageFilter(SAE_J1939_CONTROL1_ID_BASE, SAE_J1939_CONTROL1_FILTER_BASE_MASK, TRUE);
    // initialize filter for requests
    CAN_ConfigureMessageFilter(SAE_J1939_REQUEST_ID_BASE, SAE_J1939_REQUEST_FILTER_BASE_MASK, TRUE);
    // initialize filter for address claim
    CAN_ConfigureMessageFilter(SAE_J1939_ADDRESS_CLAIM_ID_BASE, SAE_J1939_ADDRESS_CLAIM_FILTER_BASE_MASK, TRUE);
    // initialize filter for Bank 1 and Bank 0 commands
    CAN_ConfigureMessageFilter(ACEINNA_BANK_ID_BASE, ACEINNA_BANK_FILTER_BASE_MASK, TRUE);
    // initialize Filter for incoming boot messages
    CAN_ConfigureMessageFilter(VEHICLE_BOOT_ID_BASE, VEHICLE_BOOT_FILTER_BASE_MASK, TRUE);
    // initialize Filter for incoming J1939 Transmission data message
    CAN_ConfigureMessageFilter(SAE_J1939_TRANSMISSION_DATA_ID_BASE, SAE_J1939_TRANSMISSION_DATA_FILTER_BASE_MASK, TRUE);
    // initialize Filter for incoming J1939 WheelSpeed/VehicalSpeed data message
    CAN_ConfigureMessageFilter(SAE_J1939_SPEED_DATA_ID_BASE, SAE_J1939_SPEED_DATA_FILTER_BASE_MASK, TRUE);
    // initialize Filter for incoming JD Machine Acceleration data message
    CAN_ConfigureMessageFilter(SAE_J1939_ACCEL_DATA_ID_BASE, SAE_J1939_ACCEL_DATA_FILTER_BASE_MASK, TRUE);

    // Initialize aiding signal filters for corola messages only when is configured, to avoid combining standard ID message filter with extended id message filters in production
    ecu_corolaAidingSig_CAN_message_filters(gEcuConfigPtr->aidingPF, gEcuConfigPtr->aidingPS, gEcuConfigPtr->drivingDirPF, gEcuConfigPtr->drivingDirPS);
}

/*******************************************
 * @brief
 *
 * @param latency ==
 * @param packetsMask ==
 * @return uint8_t
********************************************/
static uint8_t EnqeuePeriodicDataPackets(uint32_t const latency, uint16_t const packetsMask)
{

  float32_t   tmp;
  uint32_t    utmp;

  uint16_t    packets_to_send = 0U;
  uint8_t     error = 0U;

  if ((packetsMask && gEcuConfigPtr->packet_rate_div) != 0U){
      packets_to_send = gEcuConfigPtr->packet_type;
  }

  packets_to_send |= GetRequestedPackets();

  // ss2 packets supported by 9DOF
  if ((packets_to_send & (uint16_t)ACEINNA_SAE_J1939_PACKET_SLOPE_SENSOR2) && UseAlgorithm()) {
      SLOPE_SENSOR_2 slope_data;
      real eulerAngles[3];

      EKF_GetAttitude_EA_RAD(eulerAngles);

      float64_t  pitch = eulerAngles[PITCH] * 57.296;
      float64_t  roll  = eulerAngles[ROLL]  * 57.296;

      slope_data.pitch_compensation = 0U;
      slope_data.roll_compensation  = 0U;
      slope_data.pitch_merit        = 0U;
      slope_data.roll_merit         = 0U;

      if (BIT_IsAlgorithmDegraded()){
          slope_data.pitch_merit = 1U;
          slope_data.roll_merit  = 1U;
      }

      if (pitch < -250.00){
          pitch = -250.00;
          slope_data.pitch_merit  = 2U;
      }

      if(pitch > 250.00){
          pitch = 250.00;
          slope_data.pitch_merit = 2U;
      }

      if (roll < -250.00){
          roll = -250.00;
          slope_data.roll_merit  = 2U;
      }

      if(roll > 250.00){
          roll = 250.00;
          slope_data.roll_merit  = 2U;
      }

      if (BIT_IsAlgorithmError()){
          slope_data.pitch_merit = 2U;
          slope_data.roll_merit  = 2U;
      }

      tmp                           = (float32_t)(((pitch + 250.00) * 32768.0) + 0.5);
      slope_data.pitch              = (uint32_t)tmp;
      tmp                           = (float32_t)(((roll  + 250.00) * 32768.0) + 0.5);
      slope_data.roll               = (uint32_t)tmp;
      utmp                          = latency/5U;
      slope_data.measure_latency    = utmp;

      error |= aceinna_j1939_send_slope_sensor2(&slope_data);
  }

  if ( (packets_to_send & (uint16_t)ACEINNA_SAE_J1939_PACKET_SLOPE_SENSOR) && UseAlgorithm()){
          // 61459 , 0xF013
          SLOPE_SENSOR_1 slope1_data;
          real  eulerAngles[3];

          slope1_data.pitch_merit         = 0U;    // OK
          slope1_data.roll_merit          = 0U;    // OK
          slope1_data.rate_merit          = 0U;    // OK
          EKF_GetAttitude_EA_RAD(eulerAngles);

          float32_t     pitch = eulerAngles[PITCH] * 57.296F;
          float32_t     roll  = eulerAngles[ROLL]  * 57.296F;

         if (BIT_IsAlgorithmDegraded()){
            slope1_data.pitch_merit = 1U;
            slope1_data.roll_merit  = 1U;
            slope1_data.rate_merit  = 0U;    // OK
         }

          // *********************** pitch ********************
          if(pitch < -64.0F){
              pitch = -64.0F;
              slope1_data.pitch_merit = 2U;        // out of range
          }else if(pitch > 64.0F){
              pitch = 64.0F;
              slope1_data.pitch_merit = 2U;        // out of range
          }else{
              // nothing to do here
          }

          // *********************** roll ********************
          if(roll < -64.0F){
              roll = -64.0F;
              slope1_data.roll_merit = 2U;        // out of range
          }else if(roll > 64.0F){
              roll = 64.0F;
              slope1_data.roll_merit = 2U;        // out of range
          }else {
                // nothing to do here
          }
          // *********************** rate (axis Y) *************
          real corrRates[3];
          EKF_GetCorrectedRates_B(corrRates);

          float32_t rate = corrRates[PITCH] * 57.296F;

          if(rate < -64.0F){
              rate = -64.0F;
              slope1_data.rate_merit = 2U;        // out of range
          }else if(rate > 64.0F){
              rate = 64.0F;
              slope1_data.rate_merit = 2U;        // out of range
          }else{
              // nothing to do
          }

          if (BIT_IsAlgorithmError()){
              slope1_data.pitch_merit = 2U;
              slope1_data.roll_merit  = 2U;
              slope1_data.rate_merit  = 2U;
          }

          tmp = (float32_t)((pitch + 64.0F) * 500.0F);
          slope1_data.pitch               = (uint16_t)tmp;
          tmp = (float32_t)((roll  + 64.0F) * 500.0F);
          slope1_data.roll                = (uint16_t)tmp;
          tmp = (float32_t)((rate  + 64.0F) * 500.0F);
          slope1_data.pitch_rate          = (uint16_t)tmp;
          slope1_data.compensation        = 0U;
          utmp = latency/5U;
          slope1_data.measurement_latency = utmp;    // in 0.5 ms/bit

          error |= aceinna_j1939_send_slope_sensor(&slope1_data);
      }

   // acceleration packets
   if (packets_to_send & (uint16_t)ACEINNA_SAE_J1939_PACKET_ACCELERATION) {
        ACCELERATION_SENSOR accel_data;
        float64_t accelData[3];
        sens_GetAccelData_mPerSecSq(accelData);

        if(SwapAccelFrame()){
            accelData[1] = -accelData[1];
            accelData[2] = -accelData[2];
        }

        tmp = (float32_t)((((accelData[0]) + 320.00) * 100.0) + 0.5);
        accel_data.acceleration_x = (uint16_t)tmp;
        tmp = (float32_t)((((accelData[1]) + 320.00) * 100.0) + 0.5);
        accel_data.acceleration_y = (uint16_t)tmp;
        tmp = (float32_t)((((accelData[2]) + 320.00) * 100.0) + 0.5);
        accel_data.acceleration_z = (uint16_t)tmp;

        if(SwapPitchAndRoll()){
            uint16_t const ttt1 = accel_data.acceleration_x;
            accel_data.acceleration_x = accel_data.acceleration_y;
            accel_data.acceleration_y = ttt1;
        }

        accel_data.lateral_merit        = 0U;
        accel_data.longitudinal_merit   = 0U;
        accel_data.vertical_merit       = 0U;
        accel_data.transmit_rate        = 2U;
        utmp = latency/5U;
        accel_data.measurement_latency  = (uint16_t)utmp;

        if(BIT_IsAccelerationDegraded(XACCEL)){
            accel_data.lateral_merit      = 1U;
        }

        if(BIT_IsAccelerationDegraded(YACCEL)){
            accel_data.longitudinal_merit = 1U;
        }

        if(BIT_IsAccelerationDegraded(ZACCEL)){
            accel_data.vertical_merit     = 1U;
        }

        if(BIT_IsAccelerationError(XACCEL)){
            accel_data.lateral_merit      = 2U;
        }

        if(BIT_IsAccelerationError(YACCEL)){
            accel_data.longitudinal_merit = 2U;
        }

        if(BIT_IsAccelerationError(ZACCEL)){
            accel_data.vertical_merit     = 2U;
        }

        error |= aceinna_j1939_send_acceleration(&accel_data);
   }


   // acceleration packets
   if (packets_to_send & (uint16_t)ACEINNA_SAE_J1939_PACKET_ACCELERATION_HR) {
        ACCELERATION_SENSOR_RAW accel_data;
        float64_t accelData[3];
        sens_GetAccelData_mPerSecSq(accelData);

      if(SwapAccelFrame()){
          accelData[1] = -accelData[1];
          accelData[2] = -accelData[2];
      }


        tmp = (float32_t)(((accelData[0] + 320.00) * 800.0) + 0.5);
        accel_data.acceleration_x = (uint32_t)tmp;
        tmp = (float32_t)(((accelData[1] + 320.00) * 800.0) + 0.5);
        accel_data.acceleration_y = (uint32_t)tmp;
        tmp = (float32_t)(((accelData[2] + 320.00) * 800.0) + 0.5);
        accel_data.acceleration_z = (uint32_t)tmp;



        if(SwapPitchAndRoll()){
            uint32_t const ttt2 = (uint32_t)accel_data.acceleration_x;
            accel_data.acceleration_x = accel_data.acceleration_y;
            accel_data.acceleration_y = ttt2;
        }


        accel_data.lateral_merit        = 0U;
        accel_data.longitudinal_merit   = 0U;
        accel_data.vertical_merit       = 0U;

        if(BIT_IsAccelerationDegraded(XACCEL)){
            accel_data.lateral_merit      = 1U;
        }

        if(BIT_IsAccelerationDegraded(YACCEL)){
            accel_data.longitudinal_merit = 1U;
        }

        if(BIT_IsAccelerationDegraded(ZACCEL)){
            accel_data.vertical_merit     = 1U;
        }

        if(BIT_IsAccelerationError(XACCEL)){
            accel_data.lateral_merit      = 2U;
        }

        if(BIT_IsAccelerationError(YACCEL)){
            accel_data.longitudinal_merit = 2U;
        }

        if(BIT_IsAccelerationError(ZACCEL)){
            accel_data.vertical_merit     = 2U;
        }

        error |= aceinna_j1939_send_acceleration_raw(&accel_data);
   }



  if (packets_to_send & (uint16_t)ACEINNA_SAE_J1939_PACKET_ANGULAR_RATE) {
     float64_t  rate[3];
     uint8_t merit[3] = {0U,0U,0U};
     ANGULAR_RATE   angle_data;
     angle_data.mode  = (uint16_t)ANGLE_DATA_MODE_CORERCTED_SWAPPED;

     if(UseAlgorithm() && (!SendRawRates())) {
        real correctedRate_B[3];
        EKF_GetCorrectedAngRates(correctedRate_B);

        rate[0] = (float64_t)correctedRate_B[0];
        rate[1] = (float64_t)correctedRate_B[1];
        rate[2] = (float64_t)correctedRate_B[2];

        angle_data.mode  = (uint16_t)ANGLE_DATA_MODE_CORERCTED_SWAPPED;
     }
     else {
        float64_t rateData[3];
        sens_GetRateData_degPerSec(rateData);
        rate[0] = rateData[0];
        rate[1] = rateData[1];
        rate[2] = rateData[2];
        angle_data.mode  = (uint16_t)ANGLE_DATA_MODE_UNCORERCTED_SWAPPED;
     }

     if(BIT_IsRatesDegraded(XRATE)){
          merit[0]  = 1U;
     }

     if(BIT_IsRatesDegraded(YRATE)){
          merit[1]  = 1U;
     }

     if(BIT_IsRatesDegraded(ZRATE)){
          merit[2]  = 1U;
     }

     if(BIT_IsRatesError(XRATE)){
          merit[0]  = 2U;
     }

     if(BIT_IsRatesError(YRATE)){
          merit[1]  = 2U;
     }

     if(BIT_IsRatesError(ZRATE)){
          merit[2]  = 2U;
     }


     for (int32_t i = 0; i < 3; i++){
        if (rate[i]  < -250.00){
            rate[i]  = -250.00;
            merit[i] = 2U;
        }
        if (rate[i] > 250.00){
            rate[i] = 250.00;
            merit[i] = 2U;
        }
     }

     if(SwapPitchAndRoll()){
         float64_t const  ttt3  = rate[0];
         uint8_t const mtmp = merit[0];
         rate[0] = rate[1];
         rate[1] = ttt3;
         merit[0] = merit[1];
         merit[1] = mtmp;
     }


     tmp = (float32_t)(((rate[0] + 250.00) * 128.0) + 0.5);
     angle_data.pitch_rate  = (uint16_t)tmp;
     tmp = (float32_t)(((rate[1] + 250.00) * 128.0 )+ 0.5);
     angle_data.roll_rate   = (uint16_t)tmp;
     tmp = (float32_t)(((rate[2] + 250.00) * 128.0) + 0.5);
     angle_data.yaw_rate    = (uint16_t)tmp;
     angle_data.pitch_merit = merit[0];
     angle_data.roll_merit  = merit[1];
     angle_data.yaw_merit   = merit[2];
     utmp = latency/5U;
     angle_data.measurement_latency = (uint16_t)utmp;

     error |= aceinna_j1939_send_angular_rate(&angle_data);
   }


  if (packets_to_send & (uint16_t)ACEINNA_SAE_J1939_PACKET_ANGULAR_RATE_HR) {
     float64_t rate[3];
     uint8_t merit[3] = {0U,0U,0U};
     ANGULAR_RATE_RAW   angle_data;

     sens_GetRateData_degPerSec(rate);

     if(BIT_IsRatesDegraded(XRATE)){
          merit[0]  = 1U;
     }

     if(BIT_IsRatesDegraded(YRATE)){
          merit[1]  = 1U;
     }

     if(BIT_IsRatesDegraded(ZRATE)){
          merit[2]  = 1U;
     }

     if(BIT_IsRatesError(XRATE)){
          merit[0]  = 2U;
     }

     if(BIT_IsRatesError(YRATE)){
          merit[1]  = 2U;
     }

     if(BIT_IsRatesError(ZRATE)){
          merit[2]  = 2U;
     }

     for (int32_t i = 0; i < 3; i++){

        if (rate[i]  < -250.00){
            rate[i]  = -250.00;
            merit[i] = 2U;
        }
        if (rate[i] > 250.00){
            rate[i]  = 250.00;
            merit[i] = 2U;
        }
     }

     if(SwapPitchAndRoll()){
         float64_t const  ttt4 = rate[0];
         uint8_t const mtmp = merit[0];
         rate[0] = rate[1];
         rate[1] = ttt4;
         merit[0] = merit[1];
         merit[1] = mtmp;
     }

     tmp = (float32_t)(((rate[0] + 250.00) * 1024.0) + 0.5);
     angle_data.pitch_rate  = (uint32_t)tmp;
     tmp = (float32_t)(((rate[1] + 250.00) * 1024.0) + 0.5);
     angle_data.roll_rate   = (uint32_t)tmp;
     tmp = (float32_t)(((rate[2] + 250.00) * 1024.0) + 0.5);
     angle_data.yaw_rate    = (uint32_t)tmp;
     angle_data.pitch_merit = merit[0];
     angle_data.roll_merit  = merit[1];
     angle_data.yaw_merit   = merit[2];

     error |= aceinna_j1939_send_angular_rate_raw(&angle_data);
   }

   return error;
}

/**********************************************
* @brief
*
* @return uint8_t
***********************************************/
static uint8_t EnqeueMasterStatusPacket()
{
    uint16_t status = BIT_GetMasterStatusWord();
    return aceinna_j1939_send_status_packet(ACEINNA_SAE_J1939_BIT_MASTER, (void *)&status);
}

/*******************************************
 * @brief
 *
 * @return uint8_t 
********************************************/
static uint8_t EnqeueDM1StatusPacket(uint8_t const fmiCode, uint8_t const occurenceCount)
{
    DM1_MESSAGE_PAYLOAD pld;

    pld.lamp_status     = gEcuConfigPtr->lamp_status;
    pld.flash_status    = gEcuConfigPtr->flash_status;
    pld.SPN_LOW         = gEcuConfigPtr->SPN & 0xFFFFU;
    pld.SPN_HI          = (gEcuConfigPtr->SPN >> 16) & 0x07U;
    pld.FMI             = fmiCode;                          // Failure Mode Identifier
    pld.occurence_count = occurenceCount;                   // Occurrence Count
    pld.SPN_Conversion  = 0;                                // SPN Conversion Method
    pld.rsvd1           = 0xFFU;
    pld.rsvd2           = 0xFFU;

    return aceinna_j1939_send_DM1_message((void *)&pld);

}



/*******************************************
 * @brief
 *
********************************************/
void ProcessCANMessages()
{
    static   uint32_t   baudRate = 0U;
    static   int32_t    packetRateDivider   = 0;   // 0Hz default
    static   uint16_t   sendPeriodicPackets = 0U;
    static   int32_t    PacketRateCounter   = 0;
    static   BOOL       claimSucceded       = FALSE;
    static   BOOL       firstTime           = TRUE;
    static   BOOL       autobaudEnabled     = FALSE ;
    static   uint32_t   cmsgStat = 0U;
    BOOL     detected = FALSE;

    if(firstTime){
        firstTime = FALSE;
        uint8_t const address  = config_GetEcuAddress();
        baudRate = (uint32_t)config_GetEcuBaudRate();
        ecu_initialize((uint16_t)baudRate, address);
        autobaudEnabled = CanBaudRateDetectionEnabled();
        if(autobaudEnabled){
            ecu_set_state(ECU_STATE_BAUDRATE_DETECT);
        }else{
            ecu_set_state(ECU_STATE_WAIT_ADDRESS);
        }
    }

    // Auto-detection
    detected = ecu_detect_baudrate(&baudRate);
    if(!detected){
        return;
    }

    detected = ecu_is_bus_congestion_detected(autobaudEnabled);
    if(detected){
        return;
    }


    // Address claim
    claimSucceded = ecu_claim_address();
    if(!claimSucceded){
        return;
    }

    packetRateDivider  = GetCANPacketRateDivider();
    if(packetRateDivider){
        // at 200 Hz here
        packetRateDivider *=2;
    }

    sendPeriodicPackets = 0U;

    // process incoming messages
    ecu_process();

    uint64_t const dts = OS_GetDacqTimeStamp();
    uint64_t const cts = OS_GetCurrTimeStamp();

    // prepare outgoing data packets
    uint32_t const latency   = (uint32_t)((cts - dts)/100U);    // in 0.1 ms

    if(packetRateDivider){
        PacketRateCounter++;
        if(PacketRateCounter >= packetRateDivider){
            PacketRateCounter   = 0;
            sendPeriodicPackets = 1U;
        }
    }else{
        PacketRateCounter = 0;
    }

    ecu_check_if_pause_required(&sendPeriodicPackets);

    cmsgStat += EnqeuePeriodicDataPackets(latency, sendPeriodicPackets);

    if(BIT_NeedToSendStatus()){
        cmsgStat += EnqeueMasterStatusPacket();
    }
    
    uint8_t fmiCode;
    uint8_t occurenceCount;

    if(BIT_NeedToSendDM1Message(&fmiCode, &occurenceCount)){
        cmsgStat += EnqeueDM1StatusPacket(fmiCode, occurenceCount);
    }

    ecu_transmit();

    if(NeedResetUnit()){
        OS_Delay(50);
        HW_SystemReset();
    }

}
