/** ***************************************************************************
 * @file sae_j1939.c the definitions of basic functions
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
#include "ecu.h"
#include "halAPI.h"
#include "bitAPI.h"
#include "calibrationAPI.h"
#include "aceinna_sae_J1939.h"
#include "EcuSettings.h"
#include "osapi.h"


ECU_INSTANCE                  gEcu;

static int32_t  delayCounter  = 0;
static ECU_ADDR ecu_pool[ECU_ADDRESS_POOL_SIZE];

static uint32_t coreStat = 0U;
static uint32_t claimDelay = 0U;
/**********************************************
* @brief
*
* @param baudRate --
* @param address --
***********************************************/
void ecu_initialize(uint16_t const baudRate, uint8_t const address)
{
  static struct   ecu_tx_desc   ecu_xmit_desc[ECU_MAX_TX_DESC];
  static struct   ecu_rx_desc   ecu_rec_desc[ECU_MAX_RX_DESC];
  int32_t   i;
  uint32_t  utmp;

  aceinna_j1939_initialize_config();


  memset(ecu_xmit_desc, 0, sizeof(ecu_xmit_desc));
  memset(ecu_rec_desc, 0, sizeof(ecu_rec_desc));


  for ( i = 0; i < ECU_MAX_TX_DESC; i++) {
    if (i == (ECU_MAX_TX_DESC - 1))
    {
        ecu_xmit_desc[i].next = &ecu_xmit_desc[0];
    }
    else
    {
        ecu_xmit_desc[i].next = &ecu_xmit_desc[i+1];
    }
  }

  for ( i = 0; i < ECU_MAX_RX_DESC; i++) {
    if (i == (ECU_MAX_RX_DESC - 1))
    {
        ecu_rec_desc[i].next = &ecu_rec_desc[0];
    }
    else
    {
        ecu_rec_desc[i].next = &ecu_rec_desc[i+1];
    }
  }

  for (i = 0; i < ECU_ADDRESS_POOL_SIZE; i++) {
    ecu_pool[i].status = (uint8_t)ECU_ADDR_AVAILABLE;
    ecu_pool[i].addr  = 128U;
    ecu_pool[i].addr  += (uint8_t)i;
    if(ecu_pool[i].addr == address){
        ecu_pool[i].status = (uint8_t)ECU_ADDR_OCCUPIED;
    }
  }


    gEcu.addr                           = address;
    gEcu.oldAddr                        = address;
    gEcu.SA                             = 255U;
    gEcu.newAddr                        = 0U;
    gEcu.baudrate                       = baudRate;
    gEcu.state                          = ECU_STATE_READY;
    gEcu.ecu_name.arbitrary_address     = 1U;
    gEcu.ecu_name.function              = (uint8_t)ACEINNA_SAE_J1939_FUNCTION;
    gEcu.ecu_name.manufacture_code      = (uint16_t)ACEINNA_SAE_J1939_MANUFACTURER_CODE;
    gEcu.ecu_name.function_instance     = HW_GetEcuFunctionInstance();
    utmp                                = (uint32_t)(cal_GetUnitSerialNum() & 0x1FFFFFU);
    gEcu.ecu_name.identity_number       = utmp;
    gEcu.curr_tx_desc                   = &(ecu_xmit_desc[0]);
    gEcu.curr_rx_desc                   = &(ecu_rec_desc[0]);
    gEcu.curr_process_desc              = &(ecu_rec_desc[0]);

    CAN_ConfigureCallbacks(&ecu_transmit_isr, &ecu_receive_isr);
    CAN_InitCommunication(baudRate);
    claimDelay = cal_GetUnitSerialNum() & 0x1FU;
    if(claimDelay == 0){
		claimDelay = 31;
	}
    delayCounter = (int32_t)claimDelay;

    return ;
}


/**********************************************
* @brief
*
* @param peerAddress --
* @return uint8_t
***********************************************/
static uint8_t allocate_ecu_addr(uint8_t const peerAddress)
{
  int32_t i;

  for (i = 0; i < ECU_ADDRESS_POOL_SIZE; i++) {
        if (ecu_pool[i].addr == peerAddress) {
            ecu_pool[i].status = (uint8_t)(uint32_t)ECU_ADDR_OCCUPIED;
            continue;
        }
    if (ecu_pool[i].status == (uint8_t)(uint32_t)ECU_ADDR_AVAILABLE) {
      ecu_pool[i].status = (uint8_t)(uint32_t)ECU_ADDR_OCCUPIED;
      return ecu_pool[i].addr;
    }
  }

  return 247U;
}


/**********************************************
* @brief
*
* @param input -
* @return uint8_t
***********************************************/
uint8_t find_tx_desc(struct ecu_tx_desc** const input)
{
  *input = gEcu.curr_tx_desc;

    while ((*input)->tx_pkt_ready != DESC_IDLE)
    {
        *input =  (*input)->next;
        if (*input == gEcu.curr_tx_desc) {
            coreStat += (uint8_t)BIT_CANTxOverflow(TRUE);
            return 0U;
        }
    }

    coreStat += (uint8_t)BIT_CANTxOverflow(FALSE);

    return 1U;
}

/**********************************************
* @brief
*
* @param desc --
***********************************************/
static void ecu_process_address_claim(struct ecu_rx_desc* const desc)
{
    ECU_IDENTIFIER_FIELD* ident;
    uint8_t source_addr;
    uint64_t peer_name;
    uint64_t my_name;

    if (desc == null){
        return;
    }

    ident = &desc->rx_identifier;

    source_addr = ident->source;
    memcpy((void *)&peer_name, (void *)desc->rx_buffer.Data, 8U);
    memcpy((void *)&my_name, (void *)&gEcu.ecu_name, 8U);
    if (source_addr == gEcu.addr) {
        if (peer_name <= my_name){
            gEcu.oldAddr = gEcu.addr;
            gEcu.addr = allocate_ecu_addr(source_addr);
            delayCounter = (int32_t)claimDelay;
        }else if(delayCounter != 0){
            gEcu.oldAddr = gEcu.addr;
            gEcu.addr    = allocate_ecu_addr(source_addr);
        }else {
        coreStat += aceinna_j1939_send_address_claim(&gEcu);
    }
    }

  gEcu.state = ECU_STATE_READY;

  return;

}


/**********************************************
* @brief
*
* @param ident --
* @return ACEINNA_J1939_PACKET_TYPE
***********************************************/
static ACEINNA_J1939_PACKET_TYPE is_valid_address_claim(ECU_IDENTIFIER_FIELD const * const ident)
{
  uint8_t pf_val;
  uint8_t ps_val;

  if (ident == null){
        return ACEINNA_J1939_INVALID_IDENTIFIER;
  }

  pf_val = ident->pdu_format;
  ps_val = ident->pdu_specific;

  if ((pf_val == (uint8_t)SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM) &&
      (ps_val == (uint8_t)SAE_J1939_GROUP_EXTENSION_ACK)) {
       return ACEINNA_J1939_ADDRESS_CLAIM;
  }

  return ACEINNA_J1939_IGNORE;
}


/**********************************************
* @brief
*
***********************************************/
void ecu_transmit()
{
  struct ecu_tx_desc *tx_desc;
  BOOL result;

  // place outgoing message into queue
  tx_desc = gEcu.curr_tx_desc;

  while (tx_desc->next != gEcu.curr_tx_desc)
  {
    if (tx_desc->tx_pkt_ready == DESC_IDLE) {
      tx_desc = tx_desc->next;
      continue;
    }
    result = CAN_SendPacket(tx_desc);
    if (result) {
      tx_desc->tx_pkt_ready = DESC_IDLE;
      tx_desc = tx_desc->next;
      continue;
    }
    // always end up on busy unsent descriptor
    gEcu.curr_tx_desc = tx_desc;
    return;
  }

}

/**********************************************
* @brief
*
***********************************************/
void ecu_transmit_isr(void)
{
    ecu_transmit();
}

/**********************************************
* @brief
*
***********************************************/
void ecu_receive_isr(void)
{
  struct ecu_rx_desc *desc;

  desc = gEcu.curr_rx_desc;

  if( (gEcu.state == (uint8_t)ECU_STATE_BAUDRATE_DETECT) ||
      (gEcu.state == (uint8_t)ECU_STATE_WAIT_ADDRESS)){
      return;
  }

  // check current rx desc
  if (  (desc == null) ||
        (desc->next == null) ||
        (desc->rx_pkt_ready == DESC_OCCUPIED)){
        coreStat +=  (uint8_t)BIT_CANRxOverflow(TRUE);
        return;
  }

  coreStat += (uint8_t)BIT_CANRxOverflow(FALSE);

  desc->rx_pkt_ready = DESC_OCCUPIED;
  // provide next descriptor regardless if it's empty or not
  gEcu.curr_rx_desc  = gEcu.curr_rx_desc->next;

  return;
}


/**********************************************
* @brief
*
* @param pf_val --
* @return uint8_t
***********************************************/
static uint8_t is_valid_pf(uint8_t const pf_val)
{
  if ((pf_val == (uint8_t)SAE_J1939_PDU_FORMAT_ACK) ||
      (pf_val == (uint8_t)SAE_J1939_PDU_FORMAT_REQUEST) ||
      (pf_val == (uint8_t)SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM) ||
      (pf_val == (uint8_t)SAE_J1939_PDU_FORMAT_ECU) ||
      (pf_val == (uint8_t)SAE_J1939_PDU_FORMAT_254) ||
	  (pf_val == (uint8_t)SAE_J1939_PDU_FORMAT_239) ||
      (pf_val == (uint8_t)SAE_J1939_PDU_FORMAT_240) ||
      (pf_val == (uint8_t)SAE_J1939_PDU_FORMAT_251) ||
      (pf_val == (uint8_t)SAE_J1939_PDU_FORMAT_MA_REQUEST) ||
      (pf_val == (uint8_t)SAE_J1939_PDU_FORMAT_STARTSTOP) ||
      (pf_val == (uint8_t)STANDARD_ID_COROLA_GEAR)  ||
      (pf_val == (uint8_t)STANDARD_ID_BELOW_256)    ||
      (pf_val == (uint8_t)SAE_J1939_PDU_FORMAT_GLOBAL)){
        return 1U;
      }

   return 0U;
}

/**********************************************
* @brief
*
* @param identifier --
* @return uint8_t
***********************************************/
static uint8_t is_valid_j1939_identifier(ECU_IDENTIFIER_FIELD const * const identifier)
{
  // Add data page 1 here
//  if (identifier->data_page){
//        return 0U;
//  }

  if (!is_valid_pf(identifier->pdu_format)){
        return 0U;
  }

  return 1U;
}

/**********************************************
* @brief
*
* @param ident --
* @return ACEINNA_J1939_PACKET_TYPE
***********************************************/
static ACEINNA_J1939_PACKET_TYPE is_valid_config_command(ECU_IDENTIFIER_FIELD const * const ident)
{
   uint8_t pf_val;
   uint8_t ps_val;

   if (ident == null){
     return ACEINNA_J1939_INVALID_IDENTIFIER;
   }

   pf_val = ident->pdu_format;
   ps_val = ident->pdu_specific;

   if (
       (pf_val == (uint8_t)SAE_J1939_PDU_FORMAT_GLOBAL) &&
       (
       (ps_val == (uint8_t)SAE_J1939_GROUP_EXTENSION_BANK0) ||
       (ps_val == (uint8_t)SAE_J1939_GROUP_EXTENSION_BANK1) ||
       (ps_val == (uint8_t)SAE_J1939_GROUP_EXTENSION_BANK2) ||
       (ps_val == gEcuConfigPtr->alg_reset_ps) ||
       (ps_val == gEcuConfigPtr->save_cfg_ps) ||
       (ps_val == gEcuConfigPtr->master_status_ps) ||
       (ps_val == gEcuConfigPtr->sw_status_ps) ||
       (ps_val == gEcuConfigPtr->hw_status_ps) ||
       (ps_val == gEcuConfigPtr->hr_rate_ps) ||
       (ps_val == gEcuConfigPtr->hr_accel_ps) ||
       (ps_val == gEcuConfigPtr->packet_rate_ps) ||
       (ps_val == gEcuConfigPtr->packet_type_ps) ||
       (ps_val == gEcuConfigPtr->digital_filter_ps) ||
       (ps_val == gEcuConfigPtr->orientation_ps) ||
       (ps_val == gEcuConfigPtr->algo_control_ps) ||
       (ps_val == gEcuConfigPtr->aid_lvarm_ps) ||
       (ps_val == gEcuConfigPtr->aid_config_ps) ||
       (ps_val == gEcuConfigPtr->dm1_config_ps) ||
       (ps_val == gEcuConfigPtr->user_behavior_ps)
    )) {
         return ACEINNA_J1939_CONFIG;
   }

   return ACEINNA_J1939_IGNORE;
}

/**********************************************
* @brief
*
* @param ident --
* @return ACEINNA_J1939_PACKET_TYPE
***********************************************/
static ACEINNA_J1939_PACKET_TYPE is_valid_network_command(ECU_IDENTIFIER_FIELD const * const ident)
{
   uint8_t pf_val;
   uint8_t ps_val;

   if (ident == null){
      return ACEINNA_J1939_INVALID_IDENTIFIER;
   }

   pf_val = ident->pdu_format;
   ps_val = ident->pdu_specific;

   if (
       (pf_val == (uint8_t)SAE_J1939_PDU_FORMAT_STARTSTOP) &&
       (
            (ps_val == (uint8_t)SAE_J1939_GROUP_EXTENSION_ACK)
       )
      ) {
         return ACEINNA_J1939_NETWORK;
   }

   return ACEINNA_J1939_IGNORE;
}

/**********************************************
* @brief
*
* @param ident --
* @return ACEINNA_J1939_PACKET_TYPE
***********************************************/
static ACEINNA_J1939_PACKET_TYPE is_valid_DM_command(ECU_IDENTIFIER_FIELD const * const ident)
{
   uint8_t pf_val;
   uint8_t ps_val;

   if (ident == null){
      return ACEINNA_J1939_INVALID_IDENTIFIER;
   }

   pf_val = ident->pdu_format;
   ps_val = ident->pdu_specific;

   if (
       (pf_val == (uint8_t)SAE_J1939_PDU_FORMAT_254) &&
       (
            (ps_val == (uint8_t)SAE_J1939_PDU_SPECIFIC_211)
       )
      ) {
         return ACEINNA_J1939_DM11;
   }

   return ACEINNA_J1939_IGNORE;
}



/*******************************************
 * @brief
 *
 * @param ident ==
 * @return ACEINNA_J1939_PACKET_TYPE
********************************************/
ACEINNA_J1939_PACKET_TYPE is_aceinna_data_packet(ECU_IDENTIFIER_FIELD *ident)
{
  uint8_t pf_val, ps_val;


    pf_val = ident->pdu_format;
    ps_val = ident->pdu_specific;

    if ((pf_val == SAE_J1939_PDU_FORMAT_DATA) && (ps_val == SAE_J1939_GROUP_EXTENSION_TRN_CONTROL)){
        // transmission controller info 61445
       return ACEINNA_J1939_DATA;
    }

    if ((pf_val == SAE_J1939_PDU_FORMAT_254) && (ps_val == SAE_J1939_GROUP_EXTENSION_WHEEL_SPEED)){
        // wheel speed info 65215
       return ACEINNA_J1939_DATA;
    }

    if ((pf_val == SAE_J1939_PDU_FORMAT_254) && (ps_val == SAE_J1939_GROUP_EXTENSION_CCONTROL)){
        // cruise control message 65265
        return ACEINNA_J1939_DATA;
    }

    if ((pf_val == STANDARD_ID_BELOW_256) && (ps_val == STANDARD_ID_COROLA_WS_170)){
        // corola wheel speed info 170
       return ACEINNA_J1939_DATA;
    }

    if ((pf_val == STANDARD_ID_COROLA_GEAR) && (ps_val == STANDARD_ID_COROLA_GEAR_188)){
        // corola gear info 956
       return ACEINNA_J1939_DATA;
    }

	if ((pf_val == SAE_J1939_PDU_FORMAT_239) && (ps_val == SAE_J1939_PDU_SPECIFIC_0)){
        // transmission controller info 61445
       return ACEINNA_J1939_DATA;
    }

  return ACEINNA_J1939_IGNORE;
}


/**********************************************
* @brief
*
* @param ident --
* @return ACEINNA_J1939_PACKET_TYPE
***********************************************/
static ACEINNA_J1939_PACKET_TYPE is_valid_ma_request(ECU_IDENTIFIER_FIELD const * const ident)
{
  uint8_t pf_val;
  uint8_t ps_val;

  if (ident == null){
    return ACEINNA_J1939_INVALID_IDENTIFIER;
  }

  pf_val = ident->pdu_format;
  ps_val = ident->pdu_specific;

  if ((pf_val == (uint8_t)SAE_J1939_PDU_FORMAT_MA_REQUEST) &&
      (ps_val == gEcu.addr)) {
       return ACEINNA_J1939_MA_REQUEST;
  }

  return ACEINNA_J1939_IGNORE;
}

/**********************************************
* @brief
*
* @param ident --
* @return ACEINNA_J1939_PACKET_TYPE
***********************************************/
static ACEINNA_J1939_PACKET_TYPE is_valid_request(ECU_IDENTIFIER_FIELD const * const ident)
{
  uint8_t pf_val;

  if (ident == null){
    return ACEINNA_J1939_INVALID_IDENTIFIER;
  }

  pf_val = ident->pdu_format;

  if (pf_val == (uint8_t)SAE_J1939_PDU_FORMAT_REQUEST){
       return ACEINNA_J1939_REQUEST_PACKET;
  }

  return ACEINNA_J1939_IGNORE;
}


/**********************************************
* @brief
*
* @param rx_desc --
* @return ACEINNA_J1939_PACKET_TYPE
***********************************************/
static ACEINNA_J1939_PACKET_TYPE ecu_is_valid_j1939_rcv(struct ecu_rx_desc* const rx_desc)
{
  ECU_IDENTIFIER_FIELD *ident;
  ACEINNA_J1939_PACKET_TYPE result;

  // check rx desc is NULL
  if (rx_desc == null){
       return ACEINNA_J1939_INVALID_IDENTIFIER;
  }

  ident = &(rx_desc->rx_identifier);

  // valid identifier
  if (!is_valid_j1939_identifier(ident)){
      return ACEINNA_J1939_INVALID_IDENTIFIER;
  }

  // is address claim packet
  result = is_valid_address_claim(ident);
  if (result == ACEINNA_J1939_ADDRESS_CLAIM){
      return result;
  }

  // is SET commands
  result = is_valid_config_command(ident);
  if (result == ACEINNA_J1939_CONFIG){
      return result;
  }

  // is DM commands
  result = is_valid_DM_command(ident);
  if (result == ACEINNA_J1939_DM11){
      return result;
  }

  // is network commands
  result = is_valid_network_command(ident);
  if (result == ACEINNA_J1939_NETWORK){
      return result;
  }

  // is valid data packet
  result = is_aceinna_data_packet(ident);
  if (result == ACEINNA_J1939_DATA){
      return result;
  }

  // is ma request packet
  result = is_valid_ma_request(ident);
  if (result == ACEINNA_J1939_MA_REQUEST){
      return result;
  }

  // is ma request packet
  result = is_valid_request(ident);
  if (result == ACEINNA_J1939_REQUEST_PACKET){
      return result;
  }


  return ACEINNA_J1939_INVALID_IDENTIFIER;
}


/**********************************************
* @brief
*
***********************************************/
void ecu_process(void)
{
  struct ecu_rx_desc *rx_desc = gEcu.curr_process_desc;
  ACEINNA_J1939_PACKET_TYPE incoming_type;
  ECU_ADDRESS_ENTRY ecu_entry;

  memcpy((void *)&ecu_entry.ecu_name, (void *)&gEcu.ecu_name, 8U);
  ecu_entry.address  =  gEcu.addr;
  ecu_entry.status   = ECU_STATUS_NORMAL;
  ecu_entry.category = ECU_MASTER;

  // check rx desc
  if (rx_desc == null){
      return;
  }

  // decode and analyze identifier
  while (rx_desc->rx_pkt_ready == DESC_OCCUPIED) {

    if(rx_desc->rx_buffer.IDE == 0x00000000){
      // Process message with Standard ID
      rx_desc->rx_identifier.priority 	   = (uint8_t)0;
      rx_desc->rx_identifier.data_page 	   = (uint8_t)0;
      rx_desc->rx_identifier.source 		   = (uint8_t)gEcu.addr;
      rx_desc->rx_identifier.pdu_format 	   = (uint8_t)((rx_desc->rx_buffer.StdId >> 8U) & 0xFFU);
      rx_desc->rx_identifier.pdu_specific    = (uint8_t)(rx_desc->rx_buffer.StdId & 0xFFU);

    }else{
      // Process message with Extended ID
      rx_desc->rx_identifier.priority 	   = (uint8_t)((rx_desc->rx_buffer.ExtId >> 26U) & 0x7U);
      rx_desc->rx_identifier.data_page 	   = (uint8_t)((rx_desc->rx_buffer.ExtId >> 24U) & 0x1U);
      rx_desc->rx_identifier.pdu_format 	   = (uint8_t)((rx_desc->rx_buffer.ExtId >> 16U) & 0xFFU);
      rx_desc->rx_identifier.pdu_specific    = (uint8_t)((rx_desc->rx_buffer.ExtId >> 8U)  & 0xFFU);
      rx_desc->rx_identifier.source 		   = (uint8_t)(rx_desc->rx_buffer.ExtId & 0xFFU);
    }
    incoming_type = ecu_is_valid_j1939_rcv(rx_desc);
    if (
        (incoming_type == ACEINNA_J1939_IGNORE) ||
        (incoming_type == ACEINNA_J1939_INVALID_IDENTIFIER))
    {
        rx_desc->rx_pkt_ready = DESC_IDLE;
        rx_desc = rx_desc->next;
        gEcu.curr_process_desc =  rx_desc;
        continue;
    }

    // dispatch to message handler
    switch (incoming_type)
    {
    // address claim handler
    case ACEINNA_J1939_ADDRESS_CLAIM:
      ecu_process_address_claim(rx_desc);
      break;
    // DM11
    case ACEINNA_J1939_DM11:
      aceinna_j1939_process_DM11_Message(rx_desc);
      break;
      // request handler
    case ACEINNA_J1939_REQUEST_PACKET:
      coreStat += (uint8_t)aceinna_j1939_processRequest(rx_desc);
      break;
      // network control packet handler
    case ACEINNA_J1939_NETWORK:
      aceinna_j1939_processNetworkRequest(rx_desc);
      break;
      // aiding data packets handling
    case ACEINNA_J1939_DATA:
      aceinna_j1939_processAidingDataPackets(rx_desc);
      break;
    case ACEINNA_J1939_MA_REQUEST:
        // memory access request
      aceinna_j1939_process_ma_request(rx_desc);
      break;
    default:
      // SET commands handler
      aceinna_j1939_processCommands(rx_desc);
      break;
    }
    
    rx_desc->rx_pkt_ready = DESC_IDLE;
    rx_desc = rx_desc->next;
    gEcu.curr_process_desc =  rx_desc;
  }


  return;
}

/**********************************************
* @brief
*
* @return BOOL
***********************************************/
BOOL ecu_claim_address(void)
{
    if(delayCounter){
        delayCounter--;
        ecu_process();
        if(delayCounter != 0){
            ecu_transmit();
            return FALSE;
        }
        coreStat += aceinna_j1939_send_address_claim(&gEcu);
        gEcu.state    = ECU_STATE_READY;
        if(gEcu.addr != gEcu.oldAddr){
            coreStat += (uint8_t)SaveEcuAddress(gEcu.addr);
        }
    }

    return TRUE;
}

/**********************************************
* @brief
*
* @return struct ecu_rx_desc*
***********************************************/

struct ecu_rx_desc  *ecu_get_rx_descriptor()
{
    return gEcu.curr_rx_desc;
}

/**********************************************
* @brief
*
* @param baudrate -
***********************************************/
void   ecu_set_baudrate(int32_t const baudrate)
{
    gEcu.baudrate = (uint8_t)baudrate;
}

/**********************************************
* @brief
*
* @param state --
***********************************************/
void   ecu_set_state(ECU_STATE const state)
{
    gEcu.state = state;
}

/**********************************************
* @brief
*
* @return uint64_t*
***********************************************/
uint64_t  *ecu_get_name()
{
    return (uint64_t *)&gEcu.ecu_name;
}

/**********************************************
* @brief
*
* @param addr --
***********************************************/
void   ecu_set_address(uint8_t const addr)
{
    gEcu.addr = addr;
}

/**********************************************
* @brief
*
* @return true -
* @return false -
***********************************************/
BOOL ecu_is_good_J1939_message_received()
{
    uint8_t const err = CAN_GetLastError();

    if((gEcu.curr_rx_desc->rx_buffer.IDE != 0U) && (err == 0U)){
        return TRUE;
    }

    return FALSE;
}

/**********************************************
* @brief
*
* @param baudrate -
* @return BOOL
***********************************************/
BOOL    ecu_detect_baudrate(uint32_t* const baudrate)
{
    static uint32_t congErrCnt = 0;
    static BOOL startup = TRUE;
    if(gEcu.state != ECU_STATE_BAUDRATE_DETECT){
        return TRUE;
    }

    BOOL detected = ecu_is_good_J1939_message_received();
    if (detected){
        if (*baudrate != gEcu.baudrate){
            *baudrate = gEcu.baudrate;
            UpdateEcuBaudrate(gEcu.baudrate);
        }
        HW_EnableCANTransceiver();
        CAN_EnableDummyFilter(FALSE);
        gEcu.state = ECU_STATE_WAIT_ADDRESS;
        delayCounter = (int32_t)claimDelay;
        startup      = FALSE;
        congErrCnt  = 0;
        return TRUE;
    }
    
    detected = CAN_IsErrorDetected();
    
    if(detected){
        if(!startup){
            congErrCnt++;
            if(congErrCnt < 20U){
                return FALSE;
            }
        }
        gEcu.baudrate = CAN_SwitchToNewBaudrate((uint8_t)gEcu.baudrate);
        congErrCnt    = 0;
    }
    return FALSE;
}

/**********************************************
* @brief
*
* @param autobaudEnabled --
* @return BOOL
***********************************************/
uint8_t  errBuf[128];
uint32_t errBufIdx = 0;
static uint32_t congErrCnt = 0;
BOOL    ecu_is_bus_congestion_detected(BOOL const autobaudEnabled)
{
    if(!autobaudEnabled){
        return FALSE;
    }

//    volatile uint8_t const cntrTx = CAN_GetTxErrorCounter();
//    volatile uint8_t const cntrRx = CAN_GetRxErrorCounter();
    struct ecu_rx_desc* descr = gEcu.curr_rx_desc;
    descr->rx_buffer.IDE = 0;
    descr=descr->next;

    uint8_t err = CAN_GetDetectedError();

    if(err){
        errBuf[errBufIdx++] = err;
        errBufIdx&=0x7F;
        if(err == 0x30){
            return FALSE;   // stay on transmit
        }
    }else{
        congErrCnt = 0;
        return FALSE;
    }

    congErrCnt++;

    if (congErrCnt > 30){
        congErrCnt = 0;
        // switch to baudrate detection mode 
        ENTER_CRITICAL();
        gEcu.state = ECU_STATE_BAUDRATE_DETECT;
        gEcu.baudrate = (uint16_t)CAN_SwitchToNewBaudrate((uint8_t)gEcu.baudrate);
        while(gEcu.curr_rx_desc != descr){
            descr->rx_buffer.IDE = 0;
            descr = descr->next;
        }
        EXIT_CRITICAL();
        return TRUE;
    }

    return FALSE;
}

static int32_t pauseCnt = 0;

/**********************************************
* @brief
*
* @param sendPeriodicPackets ---
***********************************************/
void    ecu_check_if_pause_required(uint16_t* const sendPeriodicPackets)
{
    if(pauseCnt >= 0){
        pauseCnt--;
        *sendPeriodicPackets = 0U;
    }
}

/**********************************************
* @brief
*
***********************************************/
void    ecu_stop_periodic_packets()
{
    pauseCnt = 1200;    // 6 seconds
}

/**********************************************
* @brief
*
***********************************************/
void    ecu_start_periodic_packets()
{
    pauseCnt = 0;
}

/**********************************************
* @brief
*
***********************************************/
void    ecu_reset_pause()
{
    if(pauseCnt){
        pauseCnt = 1200;    // 6 seconds
    }
}

#ifdef UNIT_TEST
void setDelayCounter(int32_t cnt)
{
  delayCounter = cnt;
}
#endif

#endif //SAEJ1939
