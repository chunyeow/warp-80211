/** @file wlan_mac_ipc_util.h
 *  @brief Inter-processor Communication Framework
 *
 *  This contains code common to both CPU_LOW and CPU_HIGH that allows them
 *  to pass messages to one another.
 *
 *  @copyright Copyright 2014, Mango Communications. All rights reserved.
 *          Distributed under the Mango Communications Reference Design License
 *				See LICENSE.txt included in the design archive or
 *				at http://mangocomm.com/802.11/license
 *
 *  @author Chris Hunter (chunter [at] mangocomm.com)
 *  @author Patrick Murphy (murphpo [at] mangocomm.com)
 *  @author Erik Welsh (welsh [at] mangocomm.com)
 */

#ifndef WLAN_MAC_IPC_UTIL_H_

#include "wlan_mac_misc_util.h"

#define WLAN_MAC_IPC_UTIL_H_

#define PKT_BUF_MUTEX_DEVICE_ID		XPAR_MUTEX_0_DEVICE_ID

//FIXME: We need to disambiguate CPU_HIGH and CPU_LOW here. I'm currently using the presence
//of an interrupt controller for that, but there has to be a better way

#ifdef XPAR_INTC_0_DEVICE_ID
#include "xintc.h"
//#define MAILBOX_DEVICE_ID			XPAR_MBOX_1_DEVICE_ID
#define MAILBOX_DEVICE_ID			XPAR_MBOX_0_DEVICE_ID
#else
#define MAILBOX_DEVICE_ID			XPAR_MBOX_0_DEVICE_ID
#endif

#define EXC_MUTEX_TX_FAILURE 1
#define EXC_MUTEX_RX_FAILURE 2

#define PKT_BUF_MUTEX_SUCCESS 				0
#define PKT_BUF_MUTEX_FAIL_INVALID_BUF		-1
#define PKT_BUF_MUTEX_FAIL_ALREADY_LOCKED	-2
#define PKT_BUF_MUTEX_FAIL_NOT_LOCK_OWNER	-3

#define PKT_BUF_MUTEX_TX_BASE	0
#define PKT_BUF_MUTEX_RX_BASE	16

#define IPC_MBOX_MSG_ID_DELIM		0xF000
#define IPC_BUFFER_MAX_NUM_WORDS    100

//IPC Messages
#define IPC_MBOX_RX_MPDU_READY			0
#define IPC_MBOX_TX_MPDU_READY			1
#define IPC_MBOX_TX_MPDU_ACCEPT			2
#define IPC_MBOX_TX_MPDU_DONE			3
#define IPC_MBOX_HW_INFO				4
#define IPC_MBOX_CPU_STATUS				5
#define IPC_MBOX_CONFIG_CHANNEL			6
#define IPC_MBOX_CONFIG_MAC				7
#define IPC_MBOX_CONFIG_PHY_RX			8
#define IPC_MBOX_CONFIG_PHY_TX			9
#define IPC_MBOX_SET_TIME				11
#define IPC_MBOX_DEMO_CONFIG			12
#define IPC_MBOX_CONFIG_RX_ANT_MODE		13
#define IPC_MBOX_CONFIG_TX_CTRL_POW		14
#define IPC_MBOX_CONFIG_RX_FILTER		15
#define IPC_MBOX_MEM_READ_WRITE     	16
#define IPC_MBOX_LOW_PARAM				17
#define IPC_MBOX_LOW_RANDOM_SEED        18

typedef struct{
	u32  baseaddr;
	u32  num_words;
} ipc_reg_read_write;

#define IPC_REG_READ_MODE 0
#define IPC_REG_WRITE_MODE 1

#define DEMO_CONFIG_FLAGS_EN 0x0001

#define IPC_MBOX_MSG_ID(id) (IPC_MBOX_MSG_ID_DELIM | ((id) & 0xFFF))
#define IPC_MBOX_MSG_ID_TO_MSG(id) (id) & 0xFFF

typedef struct{
	u32 slot_config; //TODO: remove
} ipc_config_mac;

#define SLOT_CONFIG_RAND 0xFFFFFFFF

typedef struct{
	u8 reserved[4];
} ipc_config_phy_tx;

typedef struct{
	u8 enable_dsss;
	u8 reserved[3];
} ipc_config_phy_rx;

#define init_ipc_config(x,y,z) {										\
									x = (z*)y;							\
									memset((void*)x, 0xFF, sizeof(z));	\
								}

#define IPC_MBOX_SUCCESS			0
#define IPC_MBOX_INVALID_MSG		-1
#define IPC_MBOX_NO_MSG_AVAIL		-2

typedef struct {
	u16 msg_id;
	u8	num_payload_words;
	u8	arg0;
	u32* payload_ptr;
} wlan_ipc_msg;



// Hardware information struct to share data between the 
//   low and high CPUs

#define WLAN_MAC_FPGA_DNA_LEN         2
#define WLAN_MAC_ETH_ADDR_LEN         6

typedef struct {
    u32   type;
	u32   serial_number;
	u32   fpga_dna[WLAN_MAC_FPGA_DNA_LEN];
    u32   wn_exp_eth_device;
    u8    hw_addr_wn[WLAN_MAC_ETH_ADDR_LEN];
    u8    hw_addr_wlan[WLAN_MAC_ETH_ADDR_LEN];
    
} wlan_mac_hw_info;

///Note: This struct must be padded to be an integer
///number of u32 words.
typedef struct {
	u32   tx_start_delta;
	phy_tx_params phy_params;
	u16   num_slots;
	u16	  cw;
	u8 	  chan_num;
	u8 	  padding[3];
} wlan_mac_low_tx_details;





int wlan_lib_init ();
inline int ipc_mailbox_read_isempty();
inline int wlan_lib_mac_rate_to_mbps (u8 rate);
int lock_pkt_buf_tx(u8 pkt_buf_ind);
int lock_pkt_buf_rx(u8 pkt_buf_ind);
int unlock_pkt_buf_tx(u8 pkt_buf_ind);
int unlock_pkt_buf_rx(u8 pkt_buf_ind);
int status_pkt_buf_tx(u8 pkt_buf_ind, u32* Locked, u32 *Owner);
int status_pkt_buf_rx(u8 pkt_buf_ind, u32* Locked, u32 *Owner);

int ipc_mailbox_read_msg(wlan_ipc_msg* msg);
int ipc_mailbox_write_msg(wlan_ipc_msg* msg);
void nullCallback(void* param);


#ifdef XPAR_INTC_0_DEVICE_ID
int wlan_lib_mailbox_setup_interrupt(XIntc* intc);
void wlan_lib_mailbox_set_rx_callback( function_ptr_t callback );
void MailboxIntrHandler(void *CallbackRef);
#endif


#endif /* WLAN_MAC_IPC_UTIL_H_ */
