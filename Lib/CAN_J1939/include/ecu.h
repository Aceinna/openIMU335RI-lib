/** ***************************************************************************
 * @file ecu.h definitions of ecu-related parameters
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef ECU_H
#define ECU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "can_messages.h"
#include "GlobalConstants.h"


/**********************************************
* @brief 
* 
***********************************************/
enum {
  GLOBAL,
  ON_HIGHWAY_EQUIPMENT,
  AGRICULTURE_FORESTRY_EQUIPMENT,
  CONSTRUCTION_EQUIPMENT,
  MARINE,
  INDUSTRY_PROCESS,
  RESERVED
} SAE_J1939_INDUSTRY_GROUP;  

enum {
  ARBITRARY_ADDRESS_DISBAlE,
  ARBITRARY_ADDRESS_ENABLE
} ARBITRARY_ADDRESS_CAPABLE;  

// ECU connection status
/**********************************************
* @brief 
* 
***********************************************/
typedef enum {
  ECU_STATUS_IDLE                  =   0,        // ready
  ECU_STATUS_LOST_CONNECTION       =   1,        // connection lost
  ECU_STATUS_NORMAL                =   2,        // normal state
  ECU_STATUS_EMPTY_ADDRESS         =   3,        // no address assigned
  ECU_STATUS_EXPIRED               =   4         // address expires
} ECU_STATUS;

// ECU states
/**********************************************
* @brief 
* 
***********************************************/
typedef enum {
  ECU_STATE_INVALID_NAME          =   -1,       // invalid name
  ECU_STATE_TX_OVERFLOW           =   -2,       // tx queue overflow
  ECU_STATE_RX_OVERFLOW           =   -3,       // rx queue overflow
  ECU_STATE_BAUDRATE_DETECT       =   1,        // baud rate detection
  ECU_STATE_CHECK_ADDRESS         =   2,        // check address conflicted
  ECU_STATE_ALGORITHM_RESET       =   3,        // algo reset
  ECU_STATE_WAIT_ADDRESS          =   4,        // none of address
  ECU_STATE_READY                 =   64,       // ready to be used
  ECU_STATE_WAIT_ID               =   65,       // wait for ecu id packets
  ECU_STATE_WAIT_SOFTWARE_VER     =   66,       // wait for software version packets
  ECU_STATE_WAIT_ALG_RESET        =   68,       // wait for algo reset packet
  ECU_STATE_WAIT_CONFIG_SAVE      =   72,       // wait config save packet
  ECU_STATE_WAIT_BUILTIN_TEST     =   80        // wait for bit status packet
} ECU_STATE;


// MTLT is used as slave or master device
/**********************************************
* @brief 
* 
***********************************************/
typedef enum {
  ECU_MASTER                =   0,        // host device
  ECU_SLAVE                 =   1         // slave device
} ECU_CATEGORY;

// supported baud rate
/**********************************************
* @brief 
* 
***********************************************/
typedef enum {
  ECU_BAUD_500K      =    0,                    // 500kbps
  ECU_BAUD_250K      =    1,                    // 250kbps
  ECU_BAUD_125K      =    2,                    // 125kbps
  ECU_BAUD_1000K     =    3,                    // 1000kbps
  ECU_BAUD_ERROR     =    4,                    // invalid rate
} ECU_BAUD_RATE;

// ODR on CAN
/**********************************************
* @brief 
* 
***********************************************/
enum {
  ECU_PACKET_RATE_0           =           0,   //quiet
  ECU_PACKET_RATE_2           =           2,   // 2Hz
  ECU_PACKET_RATE_5           =           5,   // 5Hz
  ECU_PACKET_RATE_10          =           10,  // 10Hz
  ECU_PACKET_RATE_20          =           20,  // 20Hz
  ECU_PACKET_RATE_25          =           25,  // 25Hz
  ECU_PACKET_RATE_50          =           50,  // 50Hz
  ECU_PACKET_RATE_100         =           100, // 100Hz
  ECU_PACKET_RATE_200         =           200  // 200Hz
}ECU_PACKET_RATE;

// ECU address status
/**********************************************
* @brief 
* 
***********************************************/
enum {
    ECU_ADDR_AVAILABLE        =      0,    // available address value
    ECU_ADDR_OCCUPIED         =      1     // occupied address value
};

// descriptor state
/**********************************************
* @brief 
* 
***********************************************/
typedef enum {
  DESC_IDLE                  =   0,        // ready for being used
  DESC_OCCUPIED              =   1,        // unavailable
  DESC_PENDING               =   2         // in queue
} DESC_STATE;


/**********************************************
* @brief 
* 
***********************************************/
enum {
// desc numbers
  ECU_MAX_TX_DESC           =  32,
  ECU_MAX_RX_DESC           =  32,
  ECU_MAX_ADDR_TABLE_ENTRY  =  128,
  ECU_ADDRESS_POOL_SIZE     =  120
};

// 29-bit identifier

/**********************************************
* @brief 
***********************************************/
typedef struct {
/**********************************************
* @brief 
***********************************************/
    unsigned int source		: 8;               ///!< source address

/**********************************************
* @brief 
***********************************************/
    unsigned int pdu_specific : 8;         ///!< ps value
/**********************************************
* @brief 
***********************************************/
    unsigned int pdu_format : 8;           ///!< pf value

/**********************************************
* @brief 
***********************************************/
    unsigned int data_page  : 1;   /// data page bit

/**********************************************
* @brief 
***********************************************/
    unsigned int ext_page   : 1;   /// extended tata page

/**********************************************
* @brief 
***********************************************/
    unsigned int priority   : 3;   /// priority bits

/**********************************************
* @brief 
***********************************************/
    unsigned int reserved   : 3;
} ECU_IDENTIFIER_FIELD;


/**********************************************
* @brief ECU address structure 
***********************************************/
typedef struct {
  /**********************************************
  * @brief 
  ***********************************************/
  uint8_t      status;                 // status, available or occupied
  /**********************************************
  * @brief 
  ***********************************************/
  uint8_t      addr;                   // address
} ECU_ADDR;


/**********************************************
* @brief ECU receive descriptor 
* 
***********************************************/
struct ecu_rx_desc {

  /**********************************************
  * @brief 
  ***********************************************/
  uint8_t                     rx_pkt_len;         // rx packet length

  /**********************************************
  * @brief 
  ***********************************************/
  DESC_STATE                  rx_pkt_ready;       // rx state

  /**********************************************
  * @brief 
  ***********************************************/
  ECU_IDENTIFIER_FIELD        rx_identifier;      // indentifier of rx packet

  /**********************************************
  * @brief 
  ***********************************************/
  CanRxMsg                    rx_buffer;          // rx buffer

  /**********************************************
  * @brief 
  ***********************************************/
  struct ecu_rx_desc * next;

};


/**********************************************
* @brief ECU transmit descriptor
* 
***********************************************/
struct ecu_tx_desc {

  /**********************************************
  * @brief 
  ***********************************************/
  uint8_t                     tx_payload_len;       // tx payload length

  /**********************************************
  * @brief 
  ***********************************************/
  DESC_STATE                  tx_pkt_ready;         // tx state

  /**********************************************
  * @brief 
  ***********************************************/
  ECU_IDENTIFIER_FIELD        tx_identifier;        // idnetifier of tx packet

  /**********************************************
  * @brief 
  ***********************************************/
  CanTxMsg                    tx_buffer;            // tx buffer

  /**********************************************
  * @brief 
  ***********************************************/
  struct ecu_tx_desc *next;  
};


// 
/**********************************************
* @brief J1939 64-bit name structure
* 
***********************************************/
typedef struct {
  /**********************************************
  * @brief 
  ***********************************************/
    unsigned int  identity_number         : 21;    // id bits

  /**********************************************
  * @brief 
  ***********************************************/
    unsigned int  manufacture_code        : 11;    // manufacture code from SAE, 823 belongs to Aceinna

  /**********************************************
  * @brief 
  ***********************************************/
    unsigned int  ecu                     : 3;     // ecu bits

  /**********************************************
  * @brief 
  ***********************************************/
    unsigned int  function_instance       : 5;     // function instance bits

  /**********************************************
  * @brief 
  ***********************************************/
    unsigned int  function                : 8;     // function bits

  /**********************************************
  * @brief 
  ***********************************************/
    unsigned int  reserved                : 1;

  /**********************************************
  * @brief 
  ***********************************************/
    unsigned int  vehicle_system          : 7;     // system bits

  /**********************************************
  * @brief 
  ***********************************************/
    unsigned int  vehicle_system_instance : 4;     // system instance bits

  /**********************************************
  * @brief 
  ***********************************************/
    unsigned int  industry_group          : 3;     // group bits

  /**********************************************
  * @brief 
  ***********************************************/
    unsigned int  arbitrary_address       : 1;     // arbit bits
} ECU_NAME_FIELD;

/**********************************************
* @brief  address table
* 
***********************************************/
typedef struct {
  /**********************************************
  * @brief 
  ***********************************************/
  	ECU_NAME_FIELD  ecu_name;             	  // ecu's name

  /**********************************************
  * @brief 
  ***********************************************/
  	uint8_t       address;                             // ecu's address

  /**********************************************
  * @brief 
  ***********************************************/
  	ECU_STATUS    status;                        	  // ecu's status

  /**********************************************
  * @brief 
  ***********************************************/
  	ECU_CATEGORY  category;                   	  // ecu's category

  /**********************************************
  * @brief 
  ***********************************************/
  	uint32_t      last_scan_time; // ms                // time of last message sent

  /**********************************************
  * @brief 
  ***********************************************/
  	uint32_t      idle_time;    // ms                  // idle time

  /**********************************************
  * @brief 
  ***********************************************/
  	uint32_t      alive_time;   // second              // alive time

} ECU_ADDRESS_ENTRY;


// ECU structure
/**********************************************
* @brief 
* 
***********************************************/
typedef struct {

  /**********************************************
  * @brief 
  ***********************************************/
  ECU_NAME_FIELD              ecu_name;       // ecu's name
  /**********************************************
  * @brief 
  ***********************************************/
  uint8_t                     addr;           // ecu's address

  /**********************************************
  * @brief 
  ***********************************************/
  uint8_t                    oldAddr;        // ecu's address

  /**********************************************
  * @brief 
  ***********************************************/
  uint8_t                     newAddr;

  /**********************************************
  * @brief 
  ***********************************************/
  uint8_t                     SA;             // source address for boot-initiating unit

  /**********************************************
  * @brief 
  ***********************************************/
  uint8_t                     DA;             // source address for request-initiating unit

  /**********************************************
  * @brief 
  ***********************************************/
  uint16_t                    baudrate;       // baudrate

  /**********************************************
  * @brief 
  ***********************************************/
  ECU_CATEGORY                category;       // ecu's category

  /**********************************************
  * @brief 
  ***********************************************/
  ECU_STATE                   state;          // ecu's state

  /**********************************************
  * @brief 
  ***********************************************/
  ECU_ADDRESS_ENTRY           * addrTbl;      // address table

  /**********************************************
  * @brief 
  ***********************************************/
  struct ecu_tx_desc         * curr_tx_desc;               // current tx desc

  /**********************************************
  * @brief 
  ***********************************************/
  struct ecu_rx_desc         * curr_process_desc;          // current desc processed by hardware

  /**********************************************
  * @brief 
  ***********************************************/
  struct ecu_rx_desc         * curr_rx_desc;               // current rx desc
  

} ECU_INSTANCE;





/**********************************************
* @brief 
* 
* @param baudRate ==
* @param address ==
***********************************************/
void                 ecu_initialize(uint16_t baudRate, uint8_t address);

/**********************************************
* @brief 
* 
* @return struct ecu_rx_desc* 
***********************************************/
struct ecu_rx_desc  *ecu_get_rx_descriptor();

/**********************************************
* @brief 
* 
* @param baudrate == 
***********************************************/
void                 ecu_set_baudrate(int32_t baudrate);


/**********************************************
* @brief 
* 
* @param state == 
***********************************************/
void                 ecu_set_state(ECU_STATE state);

/**********************************************
* @brief 
* 
* @return uint64_t* 
***********************************************/
uint64_t             *ecu_get_name();

/**********************************************
* @brief 
* 
* @param addr ==
***********************************************/
void                 ecu_set_address(uint8_t addr);

/**********************************************
* @brief 
* 
***********************************************/
void                 ecu_transmit_isr(void);

/**********************************************
* @brief 
* 
***********************************************/
void                 ecu_receive_isr(void);

/**********************************************
* @brief 
* 
* @return BOOL 
***********************************************/
BOOL                 ecu_claim_address(void);

/**********************************************
* @brief 
* 
***********************************************/
void                 ecu_configure_CAN_message_filters();

/**********************************************
* @brief
*
***********************************************/
void                 ecu_corolaAidingSig_CAN_message_filters(uint8_t aiding_pf_val, uint8_t aiding_ps_val, uint8_t driveDir_pf_val, uint8_t driveDir_ps_val);

/**********************************************
* @brief 
* 
* @return BOOL 
***********************************************/
BOOL                 ecu_is_good_J1939_message_received();

/**********************************************
* @brief 
* 
* @param baudrate == 
* @return BOOL 
***********************************************/
BOOL                 ecu_detect_baudrate(uint32_t *baudrate);
/**********************************************
* @brief 
* 
* @param autobaudEnabled == 
* @return BOOL 
***********************************************/
BOOL                 ecu_is_bus_congestion_detected(BOOL autobaudEnabled);

/**********************************************
* @brief 
* 
* @param sendPeriodicPackets == 
***********************************************/
void                 ecu_check_if_pause_required(uint16_t* sendPeriodicPackets);

/**********************************************
* @brief 
* 
***********************************************/
void                 ecu_stop_periodic_packets();

/**********************************************
* @brief 
* 
***********************************************/
void                 ecu_start_periodic_packets();

/**********************************************
* @brief 
* 
***********************************************/
void                 ecu_reset_pause();

extern ECU_INSTANCE  gEcu;

#ifdef __cplusplus
}
#endif

#endif //define ECU_H


