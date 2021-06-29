/** ***************************************************************************
 * @file sae_j1939_slave.c the definitions of basic functions
 * @Author Feng
 * @date   May 2017
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifdef SAE_J1939
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "aceinna_sae_j1939.h"
#include "configurationAPI.h"
#include "halAPI.h"
#include "eepromAPI.h"
#include "bitAPI.h"


/**********************************************
* @brief
*
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_software_version(void)
{
  	msg_params_t params;
	uint8_t 	data[10];

  	// build up the header of software version
    params.data_page = 0U;
  	params.ext_page  = 0U;
    params.pkt_type = SAE_J1939_RESPONSE_PACKET;
  	params.len       = SAE_J1939_PAYLOAD_LEN_8_BYTES;
  	params.priority = SAE_J1939_CONTROL_PRIORITY;
  	params.PF       = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_254;
  	params.PS       = (uint8_t)(uint16_t)SAE_J1939_GROUP_EXTENSION_SOFTWARE_VERSION;

  	// build up software version payload

  	config_GetVersionBytesFromAppVersion(data);

	return aceinna_j1939_build_msg((void *)data, &params);

}

/**********************************************
* @brief
*
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_DM1_message(DM1_MESSAGE_PAYLOAD *pld)
{
  	msg_params_t params;

  	// build up the header of software version
    params.data_page = 0U;
  	params.ext_page  = 0U;
    params.pkt_type  = SAE_J1939_RESPONSE_PACKET;
  	params.len       = SAE_J1939_PAYLOAD_LEN_8_BYTES;
  	params.priority  = SAE_J1939_CONTROL_PRIORITY;
  	params.PF        = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_254;
  	params.PS        = (uint8_t)(uint16_t)SAE_J1939_GROUP_EXTENSION_DM1;

	return aceinna_j1939_build_msg((void *)pld, &params);

}

/**********************************************
* @brief
*
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_ack(uint8_t destAddr, uint8_t ackCode, uint8_t ackPF, uint8_t ackPS, uint8_t ackPage)
{
    uint8_t data[8];
  	msg_params_t params;

    data[0] = ackCode;
    data[1] = 0;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = destAddr;
    data[5] = ackPS;
    data[6] = ackPF;
    data[7] = ackPage;


  	// build up the header of software version
    params.data_page = 0U;
  	params.ext_page  = 0U;
    params.pkt_type  = SAE_J1939_RESPONSE_PACKET;
  	params.len       = SAE_J1939_PAYLOAD_LEN_8_BYTES;
  	params.priority  = SAE_J1939_CONTROL_PRIORITY;
  	params.PF        = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_ACK;
  	params.PS        = (uint8_t)(uint16_t)SAE_J1939_GROUP_EXTENSION_ACK;

	return aceinna_j1939_build_msg((void *)data, &params);

}



/**********************************************
* @brief
*
* @param pld -
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_ma_response(ma_response_payload_t* const pld)
{
  	msg_params_t params;
	  uint8_t 	data[8];

  	// build up the header
    params.data_page = 0U;
  	params.ext_page  = 0U;
    params.pkt_type  = SAE_J1939_RESPONSE_PACKET;
  	params.len       = SAE_J1939_PAYLOAD_LEN_8_BYTES;
  	params.priority  = SAE_J1939_CONTROL_PRIORITY;
  	params.PF        = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_MA_RESPONSE;
  	params.PS        = gEcu.SA;

    memcpy(data, pld, SAE_J1939_PAYLOAD_LEN_8_BYTES);

	return aceinna_j1939_build_msg((void *)data, &params);
}

/**********************************************
* @brief
*
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_ecu_id(void)
{
  	msg_params_t params;

  	// build up the header of ecu id
    params.data_page = 0U;
  	params.ext_page = 0U;
  	params.pkt_type = SAE_J1939_RESPONSE_PACKET;
  	params.len      = SAE_J1939_PAYLOAD_LEN_8_BYTES;
  	params.priority = SAE_J1939_CONTROL_PRIORITY;
  	params.PF       = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_ECU;
  	params.PS       = (uint8_t)(uint16_t)SAE_J1939_GROUP_EXTENSION_ECU;

	return aceinna_j1939_build_msg((void *)ecu_get_name(), &params);
}


/**********************************************
* @brief
*
* @param address -
* @param alg_rst -
* @param success -
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_algrst_cfgsave(uint8_t const address, uint8_t const alg_rst, uint8_t const success)
{
  	msg_params_t params;
	uint8_t 	data[8];

  	// build up the header
    params.data_page = 0U;
  	params.ext_page = 0U;
  	params.pkt_type = SAE_J1939_RESPONSE_PACKET;
  	params.len      = SAE_J1939_PAYLOAD_LEN_3_BYTES;
  	params.priority = SAE_J1939_CONTROL_PRIORITY;
  	params.PF       = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_GLOBAL;

	if (alg_rst){
    	params.PS = gEcuConfigPtr->alg_reset_ps;
  	}
  	else{
    	params.PS = gEcuConfigPtr->save_cfg_ps;
	}

  	// build up payload
  	data[0] = (uint8_t)ACEINNA_SAE_J1939_RESPONSE;
  	data[1] = address;
  	data[2] = success;

	return aceinna_j1939_build_msg((void *)data, &params);
}

/**********************************************
* @brief
*
* @param status_type -
* @param statusWord -
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_status_packet(int32_t const status_type, void* const statusWord)
{
  	msg_params_t params;
    uint32_t status  = *(uint32_t *)statusWord;

	params.len       = SAE_J1939_PAYLOAD_LEN_4_BYTES;

  	// build up the header of bit status
    if(status_type == ACEINNA_SAE_J1939_BIT_MASTER){
    	params.PS  = gEcuConfigPtr->master_status_ps;
        status    &= 0x0000FFFFU;
        uint16_t CRC = EEPROM_GetApplicationCRC();
        status   |= (CRC << 16);
	}else if(status_type == ACEINNA_SAE_J1939_BIT_HW){
    	params.PS  = gEcuConfigPtr->hw_status_ps;
		params.len = SAE_J1939_PAYLOAD_LEN_2_BYTES;
        status    &= 0x0000FFFFU;
	}else if(status_type == ACEINNA_SAE_J1939_BIT_SW){
    	params.PS  = gEcuConfigPtr->sw_status_ps;
	}else{
		return 0U;
	}

	params.data_page = 0U;
  	params.ext_page  = 0U;
  	params.pkt_type  = SAE_J1939_RESPONSE_PACKET;
  	params.priority  = SAE_J1939_CONTROL_PRIORITY;
  	params.PF        = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_GLOBAL;


	return aceinna_j1939_build_msg((void *)&status, &params);
}


/**********************************************
* @brief
*
* @param data -
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_algo_control_packet(void* const data)
{
  	msg_params_t params;

  	// build up the header of bit status
    params.data_page = 0U;
  	params.ext_page = 0U;
  	params.pkt_type = SAE_J1939_RESPONSE_PACKET;
  	params.priority = SAE_J1939_CONTROL_PRIORITY;
  	params.PF       = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_GLOBAL;
    params.len      = SAE_J1939_PAYLOAD_LEN_8_BYTES;
    params.PS       = gEcuConfigPtr->algo_control_ps;

	return aceinna_j1939_build_msg((void *)data, &params);
}



/**********************************************
* @brief
*
* @param odr -
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_packet_rate(uint8_t const odr)
{
  	msg_params_t params;
	uint8_t 	data[8];

  	// header message of response
    params.data_page = 0U;
  	params.ext_page = 0U;
  	params.pkt_type = SAE_J1939_RESPONSE_PACKET;
  	params.len      = SAE_J1939_PAYLOAD_LEN_2_BYTES;
  	params.priority = SAE_J1939_CONTROL_PRIORITY;
  	params.PF       = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_GLOBAL;
  	params.PS       = gEcuConfigPtr->packet_rate_ps;

  	// payload of response
  	data[0] = gEcu.DA;
  	data[1] = odr;

	return aceinna_j1939_build_msg((void *)data, &params);
}


/**********************************************
* @brief
*
* @param type -
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_packet_type(uint16_t const type, uint8_t const priorityMask)
{
  	msg_params_t params;
	uint8_t 	data[8];
	uint16_t     tmp;
  // header message of packet type response
    params.data_page = 0U;
  	params.ext_page = 0U;
  	params.pkt_type = SAE_J1939_RESPONSE_PACKET;
  	params.len      = SAE_J1939_PAYLOAD_LEN_4_BYTES;
  	params.priority = SAE_J1939_CONTROL_PRIORITY;
  	params.PF       = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_GLOBAL;
  	params.PS       = gEcuConfigPtr->packet_type_ps;

 	 // payload of packet type response
  	data[0] = gEcu.DA;
  	tmp	= type & 0xFFU;
	data[1] = (uint8_t)tmp;
  	tmp = (type >> 8U) & 0xFFU;
  	data[2] = (uint8_t) tmp;
    data[3] = priorityMask;

	return aceinna_j1939_build_msg((void *)data, &params);
}

/**********************************************
* @brief
*
* @param accel_cutoff -
* @param rate_cutoff -
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_digital_filter(uint8_t const accel_cutoff, uint8_t const rate_cutoff)
{
  	msg_params_t params;
	uint8_t 	data[3];

  	// header message of lpf response
    params.data_page = 0U;
  	params.ext_page = 0U;
  	params.pkt_type = SAE_J1939_RESPONSE_PACKET;
  	params.len      = SAE_J1939_PAYLOAD_LEN_3_BYTES;
  	params.priority = SAE_J1939_CONTROL_PRIORITY;
  	params.PF       = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_GLOBAL;
  	params.PS       = gEcuConfigPtr->digital_filter_ps;

  	// payload of lpf response
  	data[0] = gEcu.DA;
  	data[1] = rate_cutoff;
  	data[2] = accel_cutoff;

	return aceinna_j1939_build_msg((void *)data, &params);
}

/**********************************************
* @brief
*
* @param behavior -
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_user_behavior(uint16_t const behavior)
{
  	msg_params_t params;
	uint8_t 	data[3];
	uint16_t    tmp;
  	// header message of lpf response
    params.data_page = 0U;
  	params.ext_page = 0U;
  	params.pkt_type = SAE_J1939_RESPONSE_PACKET;
  	params.len      = SAE_J1939_PAYLOAD_LEN_3_BYTES;
  	params.priority = SAE_J1939_CONTROL_PRIORITY;
  	params.PF       = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_GLOBAL;
  	params.PS       = gEcuConfigPtr->user_behavior_ps;

  	// payload of lpf response
  	data[0] = gEcu.DA;
  	tmp = behavior & 0xFFU;
  	data[1] = (uint8_t) tmp;
  	tmp = (behavior >> 8U) & 0xFFU;
  	data[2] = (uint8_t) tmp;

	return aceinna_j1939_build_msg((void *)data, &params);
}

/**********************************************
* @brief
*
* @param orien -
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_aiding_sig_config(AIDING_SOURCE_CONFIG_CTRL_PAYLOAD * const aidingCfgPld){
	msg_params_t params;
	uint8_t 	data[8];
	data[0] = gEcu.DA;
	data[1] = aidingCfgPld->signalSource;
	data[2] = aidingCfgPld->aidingMsgPF;
	data[3] = aidingCfgPld->aidingMsgPS;
	data[4] = aidingCfgPld->aidingMsgRate;
	data[5] = aidingCfgPld->drivingDirMsgPF;
	data[6] = aidingCfgPld->drivingDirMsgPS;
	data[7] = aidingCfgPld->odoCfgSwitch;

    params.data_page = 0U;
  	params.ext_page = 0U;
  	params.pkt_type = SAE_J1939_RESPONSE_PACKET;
  	params.len      = SAE_J1939_PAYLOAD_LEN_8_BYTES;
  	params.priority = SAE_J1939_CONTROL_PRIORITY;
  	params.PF       = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_GLOBAL;
  	params.PS       = gEcuConfigPtr->aid_config_ps;

	return aceinna_j1939_build_msg((void *)data, &params);
}

/**********************************************
* @brief
*
* @param LvrArmConfig -
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_aiding_leverArm_config(AIDING_SOURCE_LEVER_ARM_CTRL_PAYLOAD * const lvrArmCfgPld){
	msg_params_t params;
	uint8_t 	data[7];
	data[0] = gEcu.DA;
	data[1] = lvrArmCfgPld->lvrArmXLSB;
	data[2] = lvrArmCfgPld->lvrArmXMSB;
	data[3] = lvrArmCfgPld->lvrArmYLSB;
	data[4] = lvrArmCfgPld->lvrArmYMSB;
	data[5] = lvrArmCfgPld->lvrArmZLSB;
	data[6] = lvrArmCfgPld->lvrArmZMSB;

    params.data_page = 0U;
  	params.ext_page = 0U;
  	params.pkt_type = SAE_J1939_RESPONSE_PACKET;
  	params.len      = SAE_J1939_PAYLOAD_LEN_7_BYTES;
  	params.priority = SAE_J1939_CONTROL_PRIORITY;
  	params.PF       = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_GLOBAL;
  	params.PS       = gEcuConfigPtr->aid_lvarm_ps;

	return aceinna_j1939_build_msg((void *)data, &params);
}

/*******************************************
 * @brief 
 * 
 * @param pld ==
 * @return uint8_t 
********************************************/
uint8_t aceinna_j1939_send_dm1_config(DM1_CONFIG_PAYLOAD * const pld){
	msg_params_t params;

    pld->dest_address = gEcu.DA;

    params.data_page = 0U;
  	params.ext_page  = 0U;
  	params.pkt_type  = SAE_J1939_RESPONSE_PACKET;
  	params.len       = SAE_J1939_PAYLOAD_LEN_8_BYTES;
  	params.priority  = SAE_J1939_CONTROL_PRIORITY;
  	params.PF        = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_GLOBAL;
  	params.PS        = gEcuConfigPtr->dm1_config_ps;

	return aceinna_j1939_build_msg((void *)pld, &params);
}



/**********************************************
* @brief
*
* @param orien -
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_orientation( uint8_t orien[])
{
  	msg_params_t params;
	uint8_t 	data[3];
  	// header message of orientation response
    params.data_page = 0U;
  	params.ext_page = 0U;
  	params.pkt_type = SAE_J1939_RESPONSE_PACKET;
  	params.len      = SAE_J1939_PAYLOAD_LEN_3_BYTES;
  	params.priority = SAE_J1939_CONTROL_PRIORITY;
  	params.PF       = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_GLOBAL;
  	params.PS       = gEcuConfigPtr->orientation_ps;

  	// payload of orientation response
  	data[0] = gEcu.DA;
  	data[1] = orien[0];
  	data[2] = orien[1];

	return aceinna_j1939_build_msg((void *)data, &params);
}


/**********************************************
* @brief
*
* @param data -
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_slope_sensor2(SLOPE_SENSOR_2 *const  data)
{

  	msg_params_t params;
    int32_t priority = (int32_t)gEcuConfigPtr->ssi2Priority;

  // header of ss2 packet
    params.data_page = 0U;
  	params.ext_page = 0U;
  	params.pkt_type = SAE_J1939_DATA_PACKET;
  	params.priority = priority;
  	params.PF       = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_DATA;
  	params.len      = SAE_J1939_PAYLOAD_LEN_8_BYTES;
  	params.PS       = (uint8_t)(uint16_t)SAE_J1939_GROUP_EXTENSION_SLOPE_SENSOR2;

	return aceinna_j1939_build_msg((void *)data, &params);
}

/**********************************************
* @brief
*
* @param data -
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_slope_sensor(SLOPE_SENSOR_1 * const data)
{

  	msg_params_t params;
    int32_t priority = (int32_t)gEcuConfigPtr->ssi2Priority;

  // header of ss2 packet
    params.data_page = 0U;
  	params.ext_page  = 0U;
  	params.pkt_type  = SAE_J1939_DATA_PACKET;
  	params.priority  = priority;
  	params.PF        = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_DATA;
  	params.len       = SAE_J1939_PAYLOAD_LEN_8_BYTES;
  	params.PS        = (uint8_t)(uint16_t)SAE_J1939_GROUP_EXTENSION_SLOPE_SENSOR;

	return aceinna_j1939_build_msg((void *)data, &params);
}

/**********************************************
* @brief
*
* @param data -
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_angular_rate(ANGULAR_RATE* const data)
{
  	msg_params_t params;
    int32_t priority = (int32_t)gEcuConfigPtr->ariPriority;

  	// header of angular rate packet
    params.data_page = 0U;
  	params.ext_page = 0U;
  	params.pkt_type = SAE_J1939_DATA_PACKET;
  	params.priority = priority;
  	params.PF       = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_DATA;
  	params.len      = SAE_J1939_PAYLOAD_LEN_8_BYTES;
  	params.PS       = (uint8_t)(uint16_t)SAE_J1939_GROUP_EXTENSION_ANGULAR_RATE;

	return aceinna_j1939_build_msg((void *)data, &params);
}

/**********************************************
* @brief
*
* @param data -
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_angular_rate_raw(ANGULAR_RATE_RAW* const  data)
{
  	msg_params_t params;
    int32_t priority = (int32_t)gEcuConfigPtr->ariPriority;

  	// header of angular rate packet
    params.data_page = 0U;
  	params.ext_page = 0U;
  	params.pkt_type = SAE_J1939_DATA_PACKET;
  	params.priority = priority;
  	params.PF       = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_GLOBAL;
  	params.len      = SAE_J1939_PAYLOAD_LEN_8_BYTES;
  	params.PS       = gEcuConfigPtr->hr_rate_ps;

	return aceinna_j1939_build_msg((void *)data, &params);
}


/**********************************************
* @brief
*
* @param data -
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_acceleration(ACCELERATION_SENSOR *const  data)
{
  	msg_params_t params;
    int32_t priority = (int32_t)gEcuConfigPtr->accsPriority;

  // header of acceleration packet
    params.data_page = 0U;
  	params.ext_page = 0U;
  	params.pkt_type = SAE_J1939_DATA_PACKET;
  	params.priority = priority;
  	params.PF       = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_DATA;
  	params.len      = SAE_J1939_PAYLOAD_LEN_8_BYTES;
  	params.PS       = (uint8_t)(uint16_t)SAE_J1939_GROUP_EXTENSION_ACCELERATION;

	return aceinna_j1939_build_msg((void *)data, &params);
}

/**********************************************
* @brief
*
* @param data -
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_acceleration_raw(ACCELERATION_SENSOR_RAW* const data)
{
  	msg_params_t params;
    int32_t priority = (int32_t)gEcuConfigPtr->accsPriority;

  // header of acceleration packet
    params.data_page = 0U;
  	params.ext_page  = 0U;
  	params.pkt_type  = SAE_J1939_DATA_PACKET;
  	params.priority  = priority;
  	params.PF        = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_GLOBAL;
  	params.len       = SAE_J1939_PAYLOAD_LEN_8_BYTES;
  	params.PS        = gEcuConfigPtr->hr_accel_ps;

	return aceinna_j1939_build_msg((void *)data, &params);
}

/**********************************************
* @brief
*
* @return uint8_t
***********************************************/
uint8_t aceinna_j1939_send_address_claim()
{
	msg_params_t params;

	params.data_page = 0U;
	params.ext_page  = 0U;
  	params.pkt_type  = ACEINNA_J1939_ADDRESS_CLAIM;
  	params.priority  = SAE_J1939_REQUEST_PRIORITY;
  	params.PF        = (uint8_t)(uint16_t)SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM;
  	params.PS        = (uint8_t)(uint16_t)SAE_J1939_GROUP_EXTENSION_ACK;
  	params.len       = SAE_J1939_PAYLOAD_LEN_8_BYTES;

  	return aceinna_j1939_build_msg((void *)ecu_get_name(), &params);
}


/**********************************************
* @brief
*
* @param payload - message payload
* @param params  - message parameters
* @return uint8_t result
***********************************************/
uint8_t aceinna_j1939_build_msg(void* const payload, msg_params_t* const params)
{
  struct ecu_tx_desc * desc;

  // check state machine
  if (gEcu.state < ECU_STATE_WAIT_ADDRESS){
    	return 0U;
  }

  // get tx desc
  if (find_tx_desc(&desc) == 0U){
    	return 0U;
  }

  // header of acceleration packet
  desc->tx_identifier.priority  	= (uint8_t)(uint16_t)(uint32_t)params->priority;
  desc->tx_identifier.data_page 	= params->data_page;
  desc->tx_identifier.ext_page      = params->ext_page;
  desc->tx_identifier.pdu_format    = params->PF;
  desc->tx_identifier.pdu_specific  = params->PS;
  desc->tx_payload_len              = (uint8_t)(uint32_t)params->len;
  desc->tx_identifier.source        = gEcu.addr;

//  desc->tx_buffer.ExtId =  ((uint32_t)desc->tx_identifier.r << 24U) |
//                           ((uint32_t)desc->tx_identifier.pdu_format << 16U) |
//                           ((uint32_t)desc->tx_identifier.pdu_specific << 8U) |
//                           (desc->tx_identifier.source);
  memcpy(&desc->tx_buffer.ExtId, &desc->tx_identifier, 4U);
//  desc->tx_buffer.ExtId = *(uint32_t*)&desc->tx_identifier;
  desc->tx_buffer.IDE = CAN_GetExtId();
  desc->tx_buffer.RTR = CAN_GetDataRtr();
  desc->tx_buffer.DLC = sizeof(desc->tx_buffer.Data);   // always 8 bytes

  // payload of position packet
  memset((void *)desc->tx_buffer.Data,  0xAA, sizeof(desc->tx_buffer.Data));     // add padding here
  memcpy((void *)desc->tx_buffer.Data, (void *)payload, desc->tx_payload_len);
  desc->tx_pkt_ready  = DESC_OCCUPIED;

  return 1U;
}


#endif
