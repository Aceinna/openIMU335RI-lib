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
#include "userAPI.h"
#include "bitAPI.h"
#include "halAPI.h"
#include "aceinna_sae_j1939.h"
#include "configurationAPI.h"
#include "Indices.h"
#include "crc16.h"
#include "EcuSettings.h"
#include "canJ1939API.h"

static BOOL      fResetUnit = FALSE;
static uint32_t  forced_packets = 0U;
static uint32_t  procStat = 0U;
static uint16_t bootSeed     = 0U;
static uint16_t digestedSeed = 0U;

/**********************************************
* @brief
*
* @param ps
* @return BOOL
***********************************************/
static BOOL    custom_ps_valid(int32_t const ps)
{
    if((ps >= ACEINNA_SAE_J1939_PS_MIN) && (ps <= ACEINNA_SAE_J1939_PS_MAX)){
        return TRUE;
    }

    return FALSE;
}

/**********************************************
* @brief
*
***********************************************/
static void mr_prepare_seed()
{
    bootSeed     = (uint16_t)TIMER_GetCurrTimeStamp();
    digestedSeed = CalculateCRC ((uint8_t *)&bootSeed, 2);
}

/**********************************************
* @brief
*
* @param pld
* @param edcp_val
* @param key
* @param len
***********************************************/
static void aceinna_j1939_fill_ma_response_payload(ma_response_payload_t* const pld, uint32_t const edcp_val, uint16_t const key, uint32_t const len)
{

    pld->second_byte.lennum_requested_hi = (uint8_t)(len >> 8U);
    pld->second_byte.reserved1           = 1U;
    pld->len_num_requested_lo                 = (uint8_t)(len & 0xFFU);
    pld->EDC_0                                = (uint8_t)(edcp_val & 0xFFU);
    pld->EDC_1                                = (uint8_t)((edcp_val >> 8U) & 0xFFU);
    pld->EDC_2                                = (uint8_t)((edcp_val >> 16U) & 0xFFU);
    pld->seed0                                = (uint8_t)(key & 0xFFU);
    pld->seed1                                = (uint8_t)(key >> 8U) & 0xFFU;
}

/*******************************************
 * @brief
 *
 * @param desc ==
 * @return BOOL
********************************************/
BOOL aceinna_j1939_processRequest(struct ecu_rx_desc * const desc)
{
    ECU_IDENTIFIER_FIELD *ident;
    uint8_t pf_val;
    uint8_t req_pf_val;
    uint8_t req_ps_val;
    ///  ** uint8_t req_ext_val; commented for future
    uint8_t *command;

    forced_packets = 0U;

    // check desc
    if (desc == null){
       return FALSE;
    }


    // check receiving buffer
    if ((desc->rx_buffer.RTR) || (!desc->rx_buffer.IDE) || (desc->rx_buffer.DLC != 3U)){
        return FALSE;
    }

    //identifier
    ident = &(desc->rx_identifier);

    //payload
    command = desc->rx_buffer.Data;

    pf_val   = ident->pdu_format;
    gEcu.DA  = ident->source;

    if(SwapBytesInRequest()){
        req_pf_val  = command[1];
        req_ps_val  = command[2];
    }else{
        req_pf_val  = command[1];
        req_ps_val  = command[0];
    }

    if (pf_val != (uint8_t)SAE_J1939_PDU_FORMAT_REQUEST){
        return FALSE;
    }

    // dispatch the requests to the corresponding handlers
    switch ((int32_t)req_pf_val)
    {
    // software version request
    case SAE_J1939_PDU_FORMAT_254:
    {
        if (req_ps_val == (uint8_t)SAE_J1939_GROUP_EXTENSION_SOFTWARE_VERSION)
        {
            procStat += aceinna_j1939_send_software_version();
        }
    }
    break;
    // ecu id request
    case SAE_J1939_PDU_FORMAT_ECU:
    {
        if(req_ps_val == (uint8_t)SAE_J1939_GROUP_EXTENSION_ECU){
            procStat += aceinna_j1939_send_ecu_id();
        }
    }
    break;
    // data packet request
    case SAE_J1939_PDU_FORMAT_DATA:
        if (req_ps_val == (uint8_t)SAE_J1939_GROUP_EXTENSION_SLOPE_SENSOR2)
        {
            forced_packets |= (uint8_t)ACEINNA_SAE_J1939_PACKET_SLOPE_SENSOR2;
        }
        if (req_ps_val == (uint8_t)SAE_J1939_GROUP_EXTENSION_SLOPE_SENSOR)
        {
            forced_packets |= (uint8_t)ACEINNA_SAE_J1939_PACKET_SLOPE_SENSOR;
        }
        if (req_ps_val == (uint8_t)SAE_J1939_GROUP_EXTENSION_ANGULAR_RATE)
        {
            forced_packets |= (uint8_t)ACEINNA_SAE_J1939_PACKET_ANGULAR_RATE;
        }
        if (req_ps_val == (uint8_t)SAE_J1939_GROUP_EXTENSION_ACCELERATION)
        {
            forced_packets |= (uint8_t)ACEINNA_SAE_J1939_PACKET_ACCELERATION;
        }
        break;

    case SAE_J1939_PDU_FORMAT_GLOBAL:
        if (req_ps_val == (uint8_t)SAE_J1939_GROUP_EXTENSION_ACCELERATION_HR)
        {
            forced_packets |= (uint8_t)ACEINNA_SAE_J1939_PACKET_ACCELERATION_HR;
            break;
        }
        if (req_ps_val == (uint8_t)SAE_J1939_GROUP_EXTENSION_ANGULAR_RATE_HR)
        {
            forced_packets |= (uint8_t)ACEINNA_SAE_J1939_PACKET_ANGULAR_RATE_HR;
            break;
        }

        // status request
        if ((req_ps_val == gEcuConfigPtr->master_status_ps))
        {
            uint16_t status = BIT_GetMasterStatusWord();
            procStat += aceinna_j1939_send_status_packet(ACEINNA_SAE_J1939_BIT_MASTER, (void *)&status);
        }
        if ((req_ps_val == gEcuConfigPtr->sw_status_ps))
        {
            uint32_t status = BIT_GetSWStatusWord();
            procStat += aceinna_j1939_send_status_packet(ACEINNA_SAE_J1939_BIT_SW, (void *)&status);
        }
        if ((req_ps_val == gEcuConfigPtr->hw_status_ps))
        {
            uint16_t status = BIT_GetHWStatusWord();
            procStat += aceinna_j1939_send_status_packet(ACEINNA_SAE_J1939_BIT_HW, (void *)&status);
        }
        if ((req_ps_val == gEcuConfigPtr->algo_control_ps))
        {
            ALGO_CONTROL_PAYLOAD  pld;
            pld.dest_address             = ident->source;
            pld.coefOfReduceQ            = gEcuConfigPtr->coefOfReduceQ;
            pld.limitAccelSwitchDelay    = gEcuConfigPtr->limitAccelSwitchDelay;
            pld.limitRateIntegrationTime = gEcuConfigPtr->limitRateIntegrationTime;
            pld.rsvd                     = 0xFFU;
            procStat += aceinna_j1939_send_algo_control_packet((void *)&pld);
        }
        // pasket rate request
        else if ((req_ps_val == gEcuConfigPtr->packet_rate_ps))
        {
            procStat += aceinna_j1939_send_packet_rate((uint8_t)gEcuConfigPtr->packet_rate_div);
        }
        // pasket type request
        else if ((req_ps_val == gEcuConfigPtr->packet_type_ps))
        {
            uint8_t  priorityMask = 0;
            priorityMask |=  gEcuConfigPtr->ariPriority & 0x03U;
            priorityMask |= (gEcuConfigPtr->accsPriority << 2) & 0x0CU;
            priorityMask |= (gEcuConfigPtr->ssi2Priority << 4) & 0x30U;
            procStat += aceinna_j1939_send_packet_type(gEcuConfigPtr->packet_type, priorityMask);
        }
        // filter settings request
        else if ((req_ps_val == gEcuConfigPtr->digital_filter_ps))
        {
            procStat += aceinna_j1939_send_digital_filter((uint8_t)gEcuConfigPtr->accel_cut_off, (uint8_t)gEcuConfigPtr->rate_cut_off);
        }
        else if ((req_ps_val == gEcuConfigPtr->user_behavior_ps))
        {
            procStat += aceinna_j1939_send_user_behavior(gEcuConfigPtr->user_behavior);
        }
        else if ((req_ps_val == gEcuConfigPtr->aid_config_ps))
        {
            AIDING_SOURCE_CONFIG_CTRL_PAYLOAD pld;
            pld.aidingMsgPF    = gEcuConfigPtr->aidingPF;
            pld.aidingMsgPS    = gEcuConfigPtr->aidingPS;
            pld.drivingDirMsgPF= gEcuConfigPtr->drivingDirPF;
            pld.drivingDirMsgPS= gEcuConfigPtr->drivingDirPS;
            pld.aidingMsgRate  = gEcuConfigPtr->aidingMsgRate;
            pld.odoCfgSwitch   = gEcuConfigPtr->odoCfgSwitch;
            pld.signalSource   = gEcuConfigPtr->signalSource;
            procStat += aceinna_j1939_send_aiding_sig_config(&pld);
        }
        else if ((req_ps_val == gEcuConfigPtr->aid_lvarm_ps))
        {
            AIDING_SOURCE_LEVER_ARM_CTRL_PAYLOAD pld;
            uint16_t offset = 32000;
            uint16_t levrArm[NUM_AXIS] = {0};

            levrArm[X_AXIS] = gEcuConfigPtr->odoLeverArmX + offset;
            levrArm[Y_AXIS] = gEcuConfigPtr->odoLeverArmY + offset;
            levrArm[Z_AXIS] = gEcuConfigPtr->odoLeverArmZ + offset;

            pld.lvrArmXMSB = ((levrArm[X_AXIS] >> 8) & 0xFF);
            pld.lvrArmXLSB = (levrArm[X_AXIS] & 0xFF);
            pld.lvrArmYMSB = ((levrArm[Y_AXIS] >> 8) & 0xFF);
            pld.lvrArmYLSB = (levrArm[Y_AXIS] & 0xFF);
            pld.lvrArmZMSB = ((levrArm[Z_AXIS] >> 8) & 0xFF);
            pld.lvrArmZLSB = (levrArm[Z_AXIS] & 0xFF);
            procStat += aceinna_j1939_send_aiding_leverArm_config(&pld);
        }
        else if ((req_ps_val == gEcuConfigPtr->dm1_config_ps))
        {
            DM1_CONFIG_PAYLOAD pld;
            pld.lamp_status  = gEcuConfigPtr->lamp_status;
            pld.flash_status = gEcuConfigPtr->flash_status;
            pld.FMI1         = gEcuConfigPtr->FMI1;
            pld.FMI2         = gEcuConfigPtr->FMI2;
            pld.SPN_LO       = gEcuConfigPtr->SPN & 0xFF;
            pld.SPN_MED      = (gEcuConfigPtr->SPN >> 8) & 0xFF;
            pld.SPN_HIGH     = (gEcuConfigPtr->SPN >> 16) & 0xFF;
            procStat += aceinna_j1939_send_dm1_config(&pld);
        }
        // orientation settings request
        else if ((req_ps_val == gEcuConfigPtr->orientation_ps))
        {
            uint8_t bytes[2];
            bytes[0] = (uint8_t)((gEcuConfigPtr->orien_bits >> 8U) & 0xFFU);
            bytes[1] = (uint8_t)((gEcuConfigPtr->orien_bits) & 0xFFU);
            procStat += aceinna_j1939_send_orientation(bytes);
        }else{
            procStat +=1U;
        }

        break;
        // address claim request
    case SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM:
        procStat += aceinna_j1939_send_address_claim();
        break;
    default:
        break;
    }

    return TRUE;
}

/*******************************************
 * @brief
 *
 * @param desc ==
********************************************/
void aceinna_j1939_processNetworkRequest(struct ecu_rx_desc* const desc)
{

    // check desc
    if (desc == null){
        return;
    }

    // check receiving buffer
    if ((desc->rx_buffer.RTR) || (!desc->rx_buffer.IDE) || (desc->rx_buffer.DLC != 3U)){
        return;
    }

    STARTSTOP_PAYLOAD* const pld = (STARTSTOP_PAYLOAD*)desc->rx_buffer.Data;

    if((pld->currDataLink & 0xC0U) == 0U){
        // current data link suspend
        ecu_stop_periodic_packets();
    }

    if((pld->currDataLink & 0xC0U) == 0x40U){
        // current data link proceed
        ecu_start_periodic_packets();

    }

    if(pld->j1939Net2 == 0x0FU){
        // reset timeout
        ecu_reset_pause();
    }

}
#include "stdio.h"
/*******************************************
 * @brief
 *
 * @param desc ==
********************************************/
void aceinna_j1939_processAidingDataPackets(struct  ecu_rx_desc* const desc)
{
    ECU_IDENTIFIER_FIELD *ident;
    uint8_t data_page = 0;
    uint8_t pf_val;
    uint8_t ps_val;
    uint8_t *command;

    // check desc
    if (desc == null){
        return;
    }

    ident = &(desc->rx_identifier);

    data_page = ident->data_page;

    // check commands
    command = desc->rx_buffer.Data;

    if(SwapBytesInRequest()){
        pf_val  = ident->pdu_format;
        ps_val  = ident->pdu_specific;
    }else{
        pf_val  = ident->pdu_format;
        ps_val  = ident->pdu_specific;
    }

    if((pf_val != gEcuConfigPtr->aidingPF       || ps_val != gEcuConfigPtr->aidingPS) &&
       (pf_val != gEcuConfigPtr->drivingDirPF   || ps_val != gEcuConfigPtr->drivingDirPS)){
        return;
    }

	/// 61445 - 100 mS - Transmission Control Message
    if ((pf_val == SAE_J1939_PDU_FORMAT_DATA) && (ps_val == SAE_J1939_GROUP_EXTENSION_TRN_CONTROL)){
        transmission_controller_msg_t* const pld = (transmission_controller_msg_t *)command;
        int8_t currentGear = (int8_t)(pld->currentGear - 125U);

		// 0xFB is parking state, anything greater is parking as well
		if(currentGear > 125) // >= 0xFB (251)
        {
            OdoUpdateTransmission(TRANSMISSION_PARK);
        }
        else if(currentGear == 0){
            OdoUpdateTransmission(TRANSMISSION_NUTRAL);
        }
        else if(currentGear > 0){
            OdoUpdateTransmission(TRANSMISSION_FORWARD);
        }
        else if(currentGear < 0){
            OdoUpdateTransmission(TRANSMISSION_REVERSE);
        }

        return;
    }

    /// 65215 - 100mS - Wheel Speed Message
    if ((pf_val == SAE_J1939_PDU_FORMAT_254) && (ps_val == SAE_J1939_GROUP_EXTENSION_WHEEL_SPEED)){
		wheel_speed_msg_t* const pld = (wheel_speed_msg_t *)command;

        real offset = -7.8125; //km/h
        real resolution = 1.0/16.0;
        // Front Axle Speed, the average speed of the two front wheels. range:[0, 250.996] km/h.
        real front_axle_speed        = (real)((uint16_t)(pld->frontAccelSpeedMsb << 8U) | pld->frontAccelSpeedLsb) / 256.0;
        // The speed of the front axle, left wheel speed,
        real front_left_wheel_speed  = (real)(pld->relativeSpeedFrontAxleLeftWheel * resolution) + offset + front_axle_speed;
        // The speed of the front axle, right wheel speed,
        real front_right_wheel_speed = (real)(pld->relativeSpeedFrontAxleRightWheel * resolution) + offset + front_axle_speed;
        // The speed of the rear axle #1, left wheel speed,
        real rear_left1_wheel_speed  = (real)(pld->relativeSpeedRearAxle1LeftWheel * resolution) + offset + front_axle_speed;
        // The speed of the rear axle #1, right wheel speed,
        real rear_right1_wheel_speed = (real)(pld->relativeSpeedRearAxle1RightWheel * resolution) + offset + front_axle_speed;
        // The speed of the rear axle #2, left wheel speed,
        real rear_left2_wheel_speed  = (real)(pld->relativeSpeedRearAxle2LeftWheel * resolution) + offset + front_axle_speed;
        // The speed of the rear axle #2, right wheel speed,
        real rear_right2_wheel_speed = (real)(pld->relativeSpeedRearAxle2RightWheel * resolution) + offset + front_axle_speed;
        (void) front_left_wheel_speed;
        (void) front_right_wheel_speed;
        (void) rear_left2_wheel_speed;
        (void) rear_right2_wheel_speed;
        // get velocity and convert unit from km/h to m/s.
        real vel_m_persec = ((rear_left1_wheel_speed + rear_right1_wheel_speed) / 2)/3.6;

        OdoUpdateVelocity(vel_m_persec);

       return;
    }

    /// 65265 - 100mS - Vehical Speed
    if ((pf_val == SAE_J1939_PDU_FORMAT_254) && (ps_val == SAE_J1939_GROUP_EXTENSION_CCONTROL)){
		cruise_ctrl_vehicle_speed_msg_t* const pld = (cruise_ctrl_vehicle_speed_msg_t *)command;

        static real resolution = 1.0/256.0;
        uint16_t vel_unint = (uint16_t)(((uint16_t)pld->wheelBasedSpeedMsb << 8) | pld->wheelBasedSpeedLsb);
        real vel_km_perhr = (real)vel_unint * resolution;
        real vel_m_persec = vel_km_perhr / 3.6;

        OdoUpdateVelocity(vel_m_persec);
        return;
    }

    /// 126720(0x1EF00) - 100mS - JD Accel Message
    if ((pf_val == SAE_J1939_PDU_FORMAT_239) && (data_page == 1U)){
        jd_machine_accel_msg_t* const pld = (jd_machine_accel_msg_t *)command;
        real resolution = 0.01f;
        int16_t offset = -320;
        real vehicalAccel[NUM_AXIS] = {0};

	    vehicalAccel[X_AXIS] = ((real)(((uint16_t)pld->lonAxcelExtRangeMsb << 8) | pld->lonAxcelExtRangeLsb) * resolution) + offset;
	    vehicalAccel[Y_AXIS] = ((real)(((uint16_t)pld->latAxcelExtRangeMsb << 8) | pld->latAxcelExtRangeLsb) * resolution) + offset;
	    vehicalAccel[Z_AXIS] = ((real)(((uint16_t)pld->verAxcelExtRangeMsb << 8) | pld->verAxcelExtRangeLsb) * resolution) + offset;

        OdoUpdateVehicalAccel(vehicalAccel);

        return;
    }

    if ((pf_val == STANDARD_ID_BELOW_256) && (ps_val == STANDARD_ID_COROLA_WS_170)){
        // corola wheel speed info 170
        corola_wheel_speed_msg_t* const pld = (corola_wheel_speed_msg_t *)command;
        real resolution = 0.01f;
        real offset = -67.67f;

        real speedRearLeft = ((real)(((uint16_t)pld->wheelSpeedRearLeftMsb << 8) | pld->wheelSpeedRearLeftLsb) * resolution) + offset;
        real speedRearRight = ((real)(((uint16_t)pld->wheelSpeedRearRightMsb << 8) | pld->wheelSpeedRearRightLsb)  * resolution) + offset;
        // get velocity and convert unit from km/h to m/s.
        real vel_m_persec = ((speedRearLeft + speedRearRight) / 2U)/3.6f;

        OdoUpdateVelocity(vel_m_persec);

       return;
    }

    if ((pf_val == STANDARD_ID_COROLA_GEAR) && (ps_val == STANDARD_ID_COROLA_GEAR_188)){
        // corola gear info 956
        corola_gear_msg_t* const pld = (corola_gear_msg_t *)command;

        switch(pld->gear){
            case 32: OdoUpdateTransmission(TRANSMISSION_PARK); break;
            case 16: OdoUpdateTransmission(TRANSMISSION_REVERSE); break;
            case 8: OdoUpdateTransmission(TRANSMISSION_NUTRAL); break;
            case 0: OdoUpdateTransmission(TRANSMISSION_FORWARD); break;
            default:
                OdoUpdateTransmission(TRANSMISSION_PREVIOUS); break;
        }

        return;
    }
}

/*******************************************
 * @brief
 *
 * @param desc ==
********************************************/
void aceinna_j1939_processCommands(struct  ecu_rx_desc* const desc)
{
    uint8_t addr;
    ECU_IDENTIFIER_FIELD *ident;
    uint8_t pf_val;
    uint8_t ps_val;
    uint8_t *command;

    // check desc
    if (desc == null){
        return;
    }

    ident = &(desc->rx_identifier);

    addr = ident->source;

    // check commands
    command = desc->rx_buffer.Data;

    pf_val  = ident->pdu_format;
    ps_val  = ident->pdu_specific;

    if (pf_val != (uint8_t)SAE_J1939_PDU_FORMAT_GLOBAL){
        return;
    }


    if (ps_val == gEcuConfigPtr->alg_reset_ps)
    {
        COMMAND_SET_PAYLOAD* const pld = (COMMAND_SET_PAYLOAD *)command;
        if (((pld->request == (uint8_t)ACEINNA_SAE_J1939_REQUEST) || (pld->request == (uint8_t)ACEINNA_SAE_J1939_RESET)) && (pld->dest_address == gEcu.addr))
        {
            InitUserAlgorithm();
            procStat += aceinna_j1939_send_algrst_cfgsave(addr, 1U, 1U);
            if (pld->request == (uint8_t)ACEINNA_SAE_J1939_RESET)
            {
                fResetUnit = TRUE;
            }
        }
    }
    else if (ps_val == gEcuConfigPtr->save_cfg_ps)
    {
        // Save configuration command
        COMMAND_SET_PAYLOAD* const pld = (COMMAND_SET_PAYLOAD *)command;
        if (((pld->request == (uint8_t)ACEINNA_SAE_J1939_REQUEST) || (pld->request == (uint8_t)ACEINNA_SAE_J1939_RESET)) && (pld->dest_address == gEcu.addr))
        {
            procStat += (uint8_t)SaveEcuSettings(FALSE, FALSE);
            procStat += aceinna_j1939_send_algrst_cfgsave(addr, 0U, 1U);
            if (pld->request == (uint8_t)ACEINNA_SAE_J1939_RESET)
            {
                fResetUnit = TRUE;
            }
        }
    }
    else if (ps_val == gEcuConfigPtr->packet_rate_ps)
    {
        // Update packet rate command
        RATE_CONFIG_PAYLOAD* const pld = (RATE_CONFIG_PAYLOAD *)command;
        if (pld->dest_address == gEcu.addr)
        {
            gEcuConfigPtr->packet_rate_div = pld->div;
        }
    }
    else if (ps_val == gEcuConfigPtr->packet_type_ps)
    {
        // Update set of transmitted packets
        PACKET_TYPE_PAYLOAD* const pld = (PACKET_TYPE_PAYLOAD *)command;
        if (pld->dest_address == gEcu.addr)
        {
            uint16_t tt1 = pld->type_high;
            tt1 <<=8U;
            tt1 |= pld->type_low;
            gEcuConfigPtr->packet_type = tt1;
            uint8_t pVal  = pld->prioVal;
            uint8_t pMask = pld->prioMask;
            // here ckecking validity of mask
            BOOL maskValid[4]   = {FALSE, FALSE, FALSE, FALSE};
            uint8_t priority[4] = {0, 0, 0, 0};
            if(pMask != 0U){
                for( int32_t i = 0; i < 4; i++){
                    if((pMask & 0x03U) == 0x03U){
                        maskValid[i] = TRUE;
                        priority[i]  = pVal & 0x03U;
                    }
                    pMask >>= 2;
                    pVal  >>= 2;
                }
            }
            // update priorities if valid mask 
            if(maskValid[0]){
                gEcuConfigPtr->ariPriority  = priority[0];
            }
            if(maskValid[1]){
                gEcuConfigPtr->accsPriority = priority[1];
            }
            if(maskValid[2]){
                gEcuConfigPtr->ssi2Priority = priority[2];
            }
        }
    }
    else if (ps_val == gEcuConfigPtr->user_behavior_ps)
    {
        // Update set of transmitted packets
        USER_BEHAVIOR_PAYLOAD* const pld = (USER_BEHAVIOR_PAYLOAD *)command;
        if (pld->dest_address == gEcu.addr)
        {
            uint16_t tmp = pld->set_option_bits[1];
            tmp = (uint16_t)(tmp << 8U);
            tmp |= pld->set_option_bits[0];
            gEcuConfigPtr->user_behavior |= tmp;
            tmp = pld->reset_option_bits[1];
            tmp = (uint16_t)(tmp << 8U);
            tmp |= pld->reset_option_bits[0];
            gEcuConfigPtr->user_behavior &= (uint16_t)~tmp;

            if((pld->new_unit_address >= (uint8_t)ACEINNA_SAE_J1939_ADDRESS_MIN) &&
               (pld->new_unit_address <= (uint8_t)ACEINNA_SAE_J1939_ADDRESS_MAX)){
                    gEcu.newAddr = pld->new_unit_address;
               }
        }
    }
    else if (ps_val == gEcuConfigPtr->digital_filter_ps)
    {
        // Update cutoff frequencies for digital filters
        DIGITAL_FILTER_PAYLOAD* const pld = (DIGITAL_FILTER_PAYLOAD *)command;
        if (pld->dest_address == gEcu.addr)
        {
            gEcuConfigPtr->rate_cut_off = pld->rate_cutoff;
            gEcuConfigPtr->accel_cut_off = pld->accel_cutoff;
            procStat += (uint8_t)config_SelectUserLPFilter(RATE_SENSOR, gEcuConfigPtr->rate_cut_off, TRUE);
            procStat += (uint8_t)config_SelectUserLPFilter(ACCEL_SENSOR, gEcuConfigPtr->accel_cut_off, TRUE);
        }
    }
    else if (ps_val == gEcuConfigPtr->orientation_ps)
    {
        // Update unit orientation
        ORIENTATION_SETTING* const pld = (ORIENTATION_SETTING *)command;
        if (pld->dest_address == gEcu.addr)
        {
            uint16_t orien = pld->orien_bits[0];
            orien <<= 8U;
            orien |= pld->orien_bits[1];
            if (config_ApplyOrientation(orien, TRUE))
            {
                gEcuConfigPtr->orien_bits = orien;
            }
        }
    }
    else if (ps_val == gEcuConfigPtr->algo_control_ps)
    {
        // Update unit algorithm settings
        ALGO_CONTROL_PAYLOAD* const pld = (ALGO_CONTROL_PAYLOAD *)command;
        if (pld->dest_address == gEcu.addr)
        {
            gEcuConfigPtr->limitAccelSwitchDelay    = pld->limitAccelSwitchDelay;
            gEcuConfigPtr->limitRateIntegrationTime = pld->limitRateIntegrationTime;
            gEcuConfigPtr->coefOfReduceQ            = pld->coefOfReduceQ;
        }
    }
    else if (ps_val == gEcuConfigPtr->aid_config_ps)
    {
        // Update unit algorithm settings
        AIDING_SOURCE_CONFIG_CTRL_PAYLOAD* const pld = (AIDING_SOURCE_CONFIG_CTRL_PAYLOAD *)command;
        if (pld->dest_address == gEcu.addr)
        {
            switch (pld->signalSource){
                case 0: // Do not use Odo or Accel
                    gEcuConfigPtr->aidingPF         = 0;
                    gEcuConfigPtr->aidingPS         = 0;
                    gEcuConfigPtr->aidingMsgRate    = 0;
                    gEcuConfigPtr->signalSource     = 0;
                    gEcuConfigPtr->drivingDirPF     = 0;
                    gEcuConfigPtr->drivingDirPS     = 0;
                    gEcuConfigPtr->odoCfgSwitch     = 0;
                    break;
                case 1: // Using Odometer for Aiding Signal
                    gEcuConfigPtr->aidingPF         = pld->aidingMsgPF;
                    gEcuConfigPtr->aidingPS         = pld->aidingMsgPS;
                    gEcuConfigPtr->aidingMsgRate    = pld->aidingMsgRate;
                    gEcuConfigPtr->signalSource     = pld->signalSource;
                    gEcuConfigPtr->drivingDirPF     = pld->drivingDirMsgPF;
                    gEcuConfigPtr->drivingDirPS     = pld->drivingDirMsgPS;
                    gEcuConfigPtr->odoCfgSwitch     = pld->odoCfgSwitch;
                    break;
                case 2: // Using Vehical Accel for Aiding Signal
                    gEcuConfigPtr->aidingPF         = pld->aidingMsgPF;
                    gEcuConfigPtr->aidingPS         = pld->aidingMsgPS;
                    gEcuConfigPtr->aidingMsgRate    = pld->aidingMsgRate;
                    gEcuConfigPtr->signalSource     = pld->signalSource;
                    gEcuConfigPtr->drivingDirPF     = 0;
                    gEcuConfigPtr->drivingDirPS     = 0;
					gEcuConfigPtr->odoCfgSwitch     = pld->odoCfgSwitch;
                    break;
                default:
                    gEcuConfigPtr->aidingPF         = 0;
                    gEcuConfigPtr->aidingPS         = 0;
                    gEcuConfigPtr->aidingMsgRate    = 0;
                    gEcuConfigPtr->signalSource     = 0;
                    gEcuConfigPtr->drivingDirPF     = 0;
                    gEcuConfigPtr->drivingDirPS     = 0;
                    gEcuConfigPtr->odoCfgSwitch    	= 0;
                    break;
            }
            ECU_IDENTIFIER_FIELD aidingID;
            ECU_IDENTIFIER_FIELD drivingDirID;
            aidingID.pdu_format = gEcuConfigPtr->aidingPF;
            aidingID.pdu_specific = gEcuConfigPtr->aidingPS;
            drivingDirID.pdu_format = gEcuConfigPtr->drivingDirPF;
            drivingDirID.pdu_specific = gEcuConfigPtr->drivingDirPS;

            // Check DrivingDirection Messaage ID is valid
            if(pld->signalSource == 1 && is_aceinna_data_packet(&drivingDirID) != ACEINNA_J1939_DATA){
                // return when the DrivingDir Message PGN is in valid
                return;
            }
            // Check aiding Messaage ID is valid
            if(is_aceinna_data_packet(&aidingID) != ACEINNA_J1939_DATA){
                // return when the Aiding Message PGN is in valid
                return;
            }
            // Update Odo configuraion
            OdoUpdateConfig(gEcuConfigPtr->aidingMsgRate, gEcuConfigPtr->signalSource, gEcuConfigPtr->odoCfgSwitch);
        }
    }
    else if (ps_val == gEcuConfigPtr->aid_lvarm_ps)
    {
        // Update unit algorithm settings
        AIDING_SOURCE_LEVER_ARM_CTRL_PAYLOAD* const pld = (AIDING_SOURCE_LEVER_ARM_CTRL_PAYLOAD *)command;
		uint16_t leverArm[NUM_AXIS];
		if (pld->dest_address == gEcu.addr)
        {
            leverArm[X_AXIS] = ((uint16_t)(pld->lvrArmXMSB << 8) | pld->lvrArmXLSB);
            leverArm[Y_AXIS] = ((uint16_t)(pld->lvrArmYMSB << 8) | pld->lvrArmYLSB);
            leverArm[Z_AXIS] = ((uint16_t)(pld->lvrArmZMSB << 8) | pld->lvrArmZLSB);
            
            if(leverArm[X_AXIS] > 64000)
            {
                leverArm[X_AXIS]  = 64000;
            }
            if(leverArm[Y_AXIS] > 64000)
            {
                leverArm[Y_AXIS]  = 64000;
            }
            if(leverArm[Z_AXIS] > 64000)
            {
                leverArm[Z_AXIS]  = 64000;
            }
            
            gEcuConfigPtr->odoLeverArmX = (int16_t)leverArm[X_AXIS] - 32000;   // in mm
            gEcuConfigPtr->odoLeverArmY = (int16_t)leverArm[Y_AXIS] - 32000;   // in mm
            gEcuConfigPtr->odoLeverArmZ = (int16_t)leverArm[Z_AXIS] - 32000;  // in mm

            OdoUpdateLeverArmConfig(&(gEcuConfigPtr->odoLeverArmX));
        }
    }
    else if ((ps_val == gEcuConfigPtr->dm1_config_ps))
    {
        DM1_CONFIG_PAYLOAD *pld = (DM1_CONFIG_PAYLOAD *)command;
		if (pld->dest_address == gEcu.addr)
        {
            gEcuConfigPtr->lamp_status  = pld->lamp_status;
            gEcuConfigPtr->flash_status = pld->flash_status;
            gEcuConfigPtr->FMI1         = pld->FMI1;
            gEcuConfigPtr->FMI2         = pld->FMI2;
            gEcuConfigPtr->SPN          = 0;
            gEcuConfigPtr->SPN         |= (pld->SPN_HIGH << 16U);
            gEcuConfigPtr->SPN         |= (pld->SPN_MED  << 8U);
            gEcuConfigPtr->SPN         |=  pld->SPN_LO;
        }
    }


    else if (ps_val == (uint8_t)SAE_J1939_GROUP_EXTENSION_BANK0)
    {
        BANK0_PS_PAYLOAD* const pld = (BANK0_PS_PAYLOAD *)command;
        if (pld->dest_address == gEcu.addr)
        {
            if(custom_ps_valid(pld->alg_reset_ps)){
                gEcuConfigPtr->alg_reset_ps     = pld->alg_reset_ps;
            }
            if(custom_ps_valid(pld->save_cfg_ps)){
                gEcuConfigPtr->save_cfg_ps      = pld->save_cfg_ps;
            }
            if(custom_ps_valid(pld->master_status_ps)){
                gEcuConfigPtr->master_status_ps = pld->master_status_ps;
            }
            if(custom_ps_valid(pld->hw_status_ps)){
                gEcuConfigPtr->hw_status_ps     = pld->hw_status_ps;
            }
            if(custom_ps_valid(pld->sw_status_ps)){
                gEcuConfigPtr->sw_status_ps     = pld->sw_status_ps;
            }
            if(custom_ps_valid(pld->hr_accel_ps)){
                gEcuConfigPtr->hr_accel_ps      = pld->hr_accel_ps;
            }
            if(custom_ps_valid(pld->hr_rate_ps)){
                gEcuConfigPtr->hr_rate_ps       = pld->hr_rate_ps;
            }
        }
    }
    else if (ps_val == (uint8_t)SAE_J1939_GROUP_EXTENSION_BANK1)
    {
        // Reassign pdu-specific codes
        BANK1_PS_PAYLOAD* const pld = (BANK1_PS_PAYLOAD *)command;
        if (pld->dest_address == gEcu.addr)
        {
            if(custom_ps_valid(pld->packet_rate_ps)){
                gEcuConfigPtr->packet_rate_ps = pld->packet_rate_ps;
            }
            if(custom_ps_valid(pld->packet_type_ps)){
                gEcuConfigPtr->packet_type_ps = pld->packet_type_ps;
            }
            if(custom_ps_valid(pld->digital_filter_ps)){
                gEcuConfigPtr->digital_filter_ps = pld->digital_filter_ps;
            }
            if(custom_ps_valid(pld->orientation_ps)){
                gEcuConfigPtr->orientation_ps = pld->orientation_ps;
            }
            if(custom_ps_valid(pld->user_behavior_ps)){
                gEcuConfigPtr->user_behavior_ps = pld->user_behavior_ps;
            }
            if(custom_ps_valid(pld->algo_control_ps)){
                gEcuConfigPtr->algo_control_ps  = pld->algo_control_ps;
            }
        }
	}
	else if (ps_val == (uint8_t)SAE_J1939_GROUP_EXTENSION_BANK2)
    {
        // Reassign pdu-specific codes
        BANK2_PS_PAYLOAD* const pld = (BANK2_PS_PAYLOAD *)command;
        if (pld->dest_address == gEcu.addr)
        {
            if(custom_ps_valid(pld->aid_config_ps)){
                gEcuConfigPtr->aid_config_ps = pld->aid_config_ps;
            }
            if(custom_ps_valid(pld->aid_lvarm_ps)){
                gEcuConfigPtr->aid_lvarm_ps = pld->aid_lvarm_ps;
            }
            if(custom_ps_valid(pld->dm1_config_ps)){
                gEcuConfigPtr->dm1_config_ps = pld->dm1_config_ps;
            }
        }
    }else{
        procStat +=1U;
    }
}

/*******************************************
 * @brief
 *
 * @param desc ==
********************************************/
void aceinna_j1939_process_DM11_Message(struct  ecu_rx_desc* const desc)
{
    uint8_t addr;
    ECU_IDENTIFIER_FIELD *ident;
    uint8_t pf_val;
    uint8_t ps_val;

    // check desc
    if (desc == null){
        return;
    }

    ident = &(desc->rx_identifier);

    addr  = ident->source;

    pf_val  = ident->pdu_format;
    ps_val  = ident->pdu_specific;

    if (pf_val == (uint8_t)SAE_J1939_PDU_FORMAT_254 && ps_val == SAE_J1939_PDU_SPECIFIC_211){
        // DM11 message
        BIT_ResetDTC();
        procStat += aceinna_j1939_send_ack(addr, SAE_J1939_ACK, SAE_J1939_PDU_FORMAT_254, SAE_J1939_PDU_SPECIFIC_211, 0);
        return;
    }

    procStat += aceinna_j1939_send_ack(addr, SAE_J1939_NACK, SAE_J1939_PDU_FORMAT_254, SAE_J1939_PDU_SPECIFIC_211, 0);
}




/**********************************************
* @brief
*
***********************************************/
void aceinna_j1939_initialize_config()
{
    gEcuConfigPtr->config_changed = 0U;

    // init alg rst ps value
    if (!gEcuConfigPtr->alg_reset_ps){
        gEcuConfigPtr->alg_reset_ps = (uint8_t)SAE_J1939_GROUP_EXTENSION_ALGORITHM_RESET;
    }

    // init config save ps values
    if (!gEcuConfigPtr->save_cfg_ps){
        gEcuConfigPtr->save_cfg_ps = (uint8_t)SAE_J1939_GROUP_EXTENSION_SAVE_CONFIGURATION;
    }

    // init status ps values
    if (!gEcuConfigPtr->master_status_ps){
        gEcuConfigPtr->master_status_ps = (uint8_t)SAE_J1939_GROUP_EXTENSION_TEST_STATUS;
    }

    // init hw status ps values
    if (!gEcuConfigPtr->hw_status_ps){
        gEcuConfigPtr->hw_status_ps = (uint8_t)SAE_J1939_GROUP_EXTENSION_TEST_HARDWARE;
    }

    // init hw status ps values
    if (!gEcuConfigPtr->sw_status_ps){
        gEcuConfigPtr->sw_status_ps = (uint8_t)SAE_J1939_GROUP_EXTENSION_TEST_SOFTWARE;
    }

    // init hw status ps values
    if (!gEcuConfigPtr->algo_control_ps){
        gEcuConfigPtr->algo_control_ps = (uint8_t)SAE_J1939_GROUP_EXTENSION_ALGO_CTRL;
    }

    // init odr ps values
    if (!gEcuConfigPtr->packet_rate_ps){
        gEcuConfigPtr->packet_rate_ps = (uint8_t)SAE_J1939_GROUP_EXTENSION_PACKET_RATE;
    }

    // init packet type ps value
    if (!gEcuConfigPtr->packet_type_ps){
        gEcuConfigPtr->packet_type_ps = (uint8_t)SAE_J1939_GROUP_EXTENSION_PACKET_TYPE;
    }

    // init lpf ps values
    if (!gEcuConfigPtr->digital_filter_ps){
        gEcuConfigPtr->digital_filter_ps = (uint8_t)SAE_J1939_GROUP_EXTENSION_DIGITAL_FILTER;
    }

    // init orientation ps values
    if (!gEcuConfigPtr->orientation_ps){
        gEcuConfigPtr->orientation_ps = (uint8_t)SAE_J1939_GROUP_EXTENSION_ORIENTATION;
    }

    // init user behavior ps values
    if (!gEcuConfigPtr->user_behavior_ps){
        gEcuConfigPtr->user_behavior_ps = (uint8_t)SAE_J1939_GROUP_EXTENSION_USER_BEHAVIOR;
    }

    if (!gEcuConfigPtr->hr_accel_ps){
        gEcuConfigPtr->hr_accel_ps = (uint8_t)SAE_J1939_GROUP_EXTENSION_ACCELERATION_HR;
    }

    if (!gEcuConfigPtr->hr_rate_ps){
        gEcuConfigPtr->hr_rate_ps = (uint8_t)SAE_J1939_GROUP_EXTENSION_ANGULAR_RATE_HR;
    }

    if (!gEcuConfigPtr->aid_lvarm_ps){
        gEcuConfigPtr->aid_lvarm_ps = (uint8_t)SAE_J1939_GROUP_EXTENSION_AIDING_SRC_LEVER_ARM;
    }

    if (!gEcuConfigPtr->aid_config_ps){
        gEcuConfigPtr->aid_config_ps = (uint8_t)SAE_J1939_GROUP_EXTENSION_AIDING_SRC_CFG;
    }

    if (!gEcuConfigPtr->dm1_config_ps){
        gEcuConfigPtr->dm1_config_ps = (uint8_t)SAE_J1939_GROUP_EXTENSION_DM1_CONFIG;
    }

    return;
}


/**********************************************
* @brief
*
* @return int32_t
***********************************************/
int32_t NeedResetUnit()
{
    return fResetUnit;
}

/**********************************************
* @brief Get the Requested Packets object
*
* @return uint16_t
***********************************************/
uint16_t GetRequestedPackets()
{
    uint16_t const res   = (uint16_t)forced_packets;
    forced_packets = 0U;

    return res;
}

/*******************************************
 * @brief
 *
 * @param desc ==
********************************************/
void aceinna_j1939_process_ma_request(struct ecu_rx_desc* const  desc)
{

    ECU_IDENTIFIER_FIELD        *ident;
    uint8_t                      ma_source_addr;
    ma_request_payload_t         ma_req_pld;
    ma_response_payload_t        ma_resp_pld;
    uint32_t                     errorCode = 0U;
    uint8_t                      status;    // undefined
    uint32_t                     resp_len;
    uint16_t                     retKey;
    uint16_t                     usrKey;

    static boot_data_transfer_ctrl_t bootCtrl = {
    .ma_session_established = FALSE,
    .numPacketsInSegment    = 0,
    .segmentLen             = 0,
    .totalFlashOffset       = 0,
    .totalSegmentNum        = 0,
    .currentDataSegment     = 0,
    .currentDataLen         = 0,
    .lastOp                 = 0,
    .imageVerified          = 0,
    .state                  = 0,
    .tp_state               = 0,
    .erased                 = FALSE,
    .buf                    = {0},
    .segmentAddr            = 0,
    .negotiatedLen          = 0,
    };


    if (desc == null){
        return;
    }

    ident = &desc->rx_identifier;

    if(ident->pdu_specific != gEcu.addr){
        // not for us
        return;
    }
    // parse request here
    ma_source_addr = ident->source;

    memcpy(&ma_req_pld, desc->rx_buffer.Data, 8U);

    uint32_t const ptr =((uint32_t)ma_req_pld.pointer_3 << 24U) |
                        ((uint32_t)ma_req_pld.pointer_2 << 16U) |
                        ((uint32_t)ma_req_pld.pointer_1 << 8U)  |
                        ma_req_pld.pointer_0;

    if(gEcu.SA    != ma_source_addr){
        // another unit tries to engage - endulge and resync
        gEcu.SA        = ma_source_addr;
        bootCtrl.state = BOOT_STATE_INIT;
    }

    // precooked values
    usrKey   = ma_req_pld.key_user1;
    usrKey <<= 8U;
    usrKey  |= ma_req_pld.key_user0;

    retKey              = SAE_1939J_MR_KEY_NOT_REQUIRED;
    // error by default
    status              = (uint8_t)SAE_1939J_MR_STATUS_FAILED;
    ma_resp_pld.EDC_EXT = (uint8_t)EDCP_EXTENSION_ERROR;
    errorCode           = (uint8_t)SAE_1939J_MR_ERROR_UDEFINED;
    resp_len            = 0U;

    switch(bootCtrl.state){
        case BOOT_STATE_INIT:  // not authorized yet, only status request allowed for authorization
            mr_prepare_seed();
            if(ma_req_pld.second_byte.command != (uint8_t)SAE_1939J_MA_COMMAND_BOOT_LOAD){
                errorCode = (uint32_t)SAE_1939J_MR_ERROR_BOOT_LOAD;
                break;
            }
            // here we have boot load mesage - try to authenticate
            if(usrKey != GENERIC_MA_KEY){
                // wrong user key
                errorCode = (uint32_t)SAE_1939J_MR_ERROR_SECURITY_USER;
                break;
            }
            if(ptr != BOOT_START_ADDR){
                // wrong user key
                errorCode = (uint32_t)SAE_1939J_MR_ERROR_INVALID_ADDRESS;
                break;
            }
            // seems OK, let's authenticate
            retKey              = bootSeed;
            bootCtrl.state      = BOOT_STATE_CHECK_BOOT_KEY;
            errorCode           = (uint32_t)SAE_1939J_MR_NO_ERROR;
            status              = (uint8_t)SAE_1939J_MR_STATUS_PROCEED;
            break;
        case BOOT_STATE_CHECK_BOOT_KEY:    // Check the key
            if(ma_req_pld.second_byte.command != (uint8_t)SAE_1939J_MA_COMMAND_BOOT_LOAD){
                errorCode      = (uint32_t)SAE_1939J_MR_ERROR_BOOT_LOAD;
                bootCtrl.state = BOOT_STATE_INIT;
                break;
            }
            // here we have boot load mesage - try to authenticate
            if(usrKey != digestedSeed){
                // wrong user key
                errorCode      = (uint32_t)SAE_1939J_MR_ERROR_SECURITY_KEY;
                bootCtrl.state = BOOT_STATE_INIT;
                break;
            }
            if(ptr != BOOT_START_ADDR){
                // wrong user key
                errorCode      = (uint32_t)SAE_1939J_MR_ERROR_INVALID_ADDRESS;
                bootCtrl.state = BOOT_STATE_INIT;
                break;
            }
#ifdef      BOOT_PROCEED_FROM_BOOTLOADER
//          Alternative scenario// seems OK, let's reset for boot loading
            HW_EnforceCanBootMode(gEcu.SA, gEcu.addr, (uint8_t)gEcu.baudrate, 1U);
            HW_SystemReset();
            break;
#else
            // seems OK, let's proceed
            errorCode           = (uint32_t)SAE_1939J_MR_NO_ERROR;
            status              = SAE_1939J_MR_STATUS_PROCEED;
            bootCtrl.state      = BOOT_STATE_WAIT_BOOT_CPLT;
             // send MR proceed response here
            ma_resp_pld.second_byte.bits.status = status;
            aceinna_j1939_fill_ma_response_payload(&ma_resp_pld, errorCode, retKey, resp_len);
            aceinna_j1939_send_ma_response(&ma_resp_pld);
             // prepare MR complete response here
            status              = SAE_1939J_MR_STATUS_COMPLETED;
#endif
            break;
        case BOOT_STATE_WAIT_BOOT_CPLT:   // Check the key
            if(ma_req_pld.second_byte.command != (uint8_t)SAE_1939J_MA_COMMAND_OP_CPLT){
                errorCode      = (uint32_t)SAE_1939J_MR_ERROR_BOOT_LOAD;
                bootCtrl.state = BOOT_STATE_INIT;
                break;
            }
            if(ptr != (uint32_t)BOOT_START_ADDR){
                // wrong address
                errorCode      = (uint32_t)SAE_1939J_MR_ERROR_INVALID_ADDRESS;
                bootCtrl.state = BOOT_STATE_INIT;
                break;
            }
            // seems OK, let's reset for boot loading
            HW_EnforceCanBootMode(gEcu.SA, gEcu.addr, (uint8_t)gEcu.baudrate, 0U);
            HW_SystemReset();
            break;

        default:
             bootCtrl.state = BOOT_STATE_INIT;
            // ignore command in unknown state
            return;
    }

    if(errorCode == (uint32_t)SAE_1939J_MR_NO_ERROR){
        ma_resp_pld.EDC_EXT = (uint8_t)EDCP_EXTENSION_UNDEFINED;
    }

    ma_resp_pld.second_byte.status = status;
    aceinna_j1939_fill_ma_response_payload(&ma_resp_pld, errorCode, retKey, resp_len);
    procStat += aceinna_j1939_send_ma_response(&ma_resp_pld);

}
