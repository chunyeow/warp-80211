/** @file wlan_mac_high.h
 *  @brief Low-level WLAN MAC High Framework
 *
 *  This contains the low-level code for accessing the WLAN MAC Low Framework.
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

#include "xparameters.h"
#include "w3_userio.h"
#include "w3_ad_controller.h"
#include "w3_clock_controller.h"
#include "w3_iic_eeprom.h"
#include "radio_controller.h"
#include "malloc.h"
#include "string.h"
#include "stdlib.h"

#include "wlan_mac_ipc_util.h"
#include "wlan_mac_802_11_defs.h"
#include "wlan_mac_misc_util.h"
#include "wlan_phy_util.h"

#include "wlan_exp.h"
#include "wlan_mac_low.h"


static u32					mac_param_chan; 										///< Current channel of the lower-level MAC
static u8           		mac_param_band;											///< Current band of the lower-level MAC
static s8					mac_param_ctrl_tx_pow;									///< Current transmit power (dBm) for control packets
static u32					mac_param_rx_filter;									///< Current filter applied to packet receptions
static u8   				rx_pkt_buf;												///< Current receive buffer of the lower-level MAC
static u32  				cpu_low_status;											///< Status flags that are reported to upper-level MAC
static wlan_mac_hw_info    	hw_info;												///< Information about the hardware reported to upper-level MAC
static wlan_ipc_msg        	ipc_msg_from_high;										///< Buffer for incoming IPC messages
static u32                 	ipc_msg_from_high_payload[IPC_BUFFER_MAX_NUM_WORDS];	///< Buffer for payload of incoming IPC messages

// Callback function pointers
function_ptr_t     frame_rx_callback;			///< User callback frame receptions
function_ptr_t     frame_tx_callback;			///< User callback frame transmissions


/**
 * @brief Initialize MAC Low Framework
 *
 * This function initializes the MAC Low Framework by setting
 * up the hardware and other subsystems in the framework.
 *
 * @param type
 * 	- lower-level MAC type
 * @return int status
 *  - initialization status (0 = success)
 */
int wlan_mac_low_init(u32 type){
	u32 status;
	rx_frame_info* rx_mpdu;
	wlan_ipc_msg ipc_msg_to_high;

	mac_param_band = RC_24GHZ;
	mac_param_ctrl_tx_pow = 10;
	cpu_low_status = 0;

	mac_param_rx_filter = (RX_FILTER_FCS_ALL | RX_FILTER_HDR_ADDR_MATCH_MPDU);

	frame_rx_callback	= (function_ptr_t)nullCallback;
	frame_tx_callback	= (function_ptr_t)nullCallback;

	status = w3_node_init();

	if(status != 0) {
		xil_printf("Error in w3_node_init()! Exiting\n");
		return -1;
	}

	//wlan_phy_tx_timestamp_ins_start(24);
	//wlan_phy_tx_timestamp_ins_end(31);
	wlan_phy_tx_timestamp_ins_start(1);
	wlan_phy_tx_timestamp_ins_end(0);

	wlan_lib_init();

	//create IPC message to receive into
	ipc_msg_from_high.payload_ptr = &(ipc_msg_from_high_payload[0]);

	//Begin by trying to lock packet buffer 0 for wireless receptions
	rx_pkt_buf = 0;
	if(lock_pkt_buf_rx(rx_pkt_buf) != PKT_BUF_MUTEX_SUCCESS){
		warp_printf(PL_ERROR, "Error: unable to lock pkt_buf %d\n", rx_pkt_buf);
		wlan_mac_low_send_exception(EXC_MUTEX_TX_FAILURE);
		return -1;
	} else {
		rx_mpdu = (rx_frame_info*)RX_PKT_BUF_TO_ADDR(rx_pkt_buf);
		rx_mpdu->state = RX_MPDU_STATE_RX_PENDING;
		wlan_phy_rx_pkt_buf_ofdm(rx_pkt_buf);
		wlan_phy_rx_pkt_buf_dsss(rx_pkt_buf);
	}

	//Move the PHY's starting address into the packet buffers by PHY_XX_PKT_BUF_PHY_HDR_OFFSET.
	//This accounts for the metadata located at the front of every packet buffer (Xx_mpdu_info)
	wlan_phy_rx_pkt_buf_phy_hdr_offset(PHY_RX_PKT_BUF_PHY_HDR_OFFSET);
	wlan_phy_tx_pkt_buf_phy_hdr_offset(PHY_TX_PKT_BUF_PHY_HDR_OFFSET);

	wlan_radio_init();
	wlan_phy_init();
	wlan_mac_low_dcf_init();

	// Initialize the HW info structure
	wlan_mac_low_init_hw_info(type);

	// Send a message to other processor to identify hw info of cpu low
	ipc_msg_to_high.msg_id = IPC_MBOX_MSG_ID(IPC_MBOX_HW_INFO);
	ipc_msg_to_high.num_payload_words = 8;
	ipc_msg_to_high.payload_ptr = (u32 *) &(hw_info);

	ipc_mailbox_write_msg(&ipc_msg_to_high);

	return 0;
}

/**
 * @brief Finish Initializing MAC Low Framework
 *
 * This function finishes the initialization and notifies the upper-level
 * MAC that it has finished booting.
 *
 * @param None
 * @return None
 */
void wlan_mac_low_finish_init(){
	wlan_ipc_msg ipc_msg_to_high;
	u32 ipc_msg_to_high_payload[1];
	cpu_low_status |= CPU_STATUS_INITIALIZED;
	//Send a message to other processor to say that this processor is initialized and ready
	ipc_msg_to_high.msg_id = IPC_MBOX_MSG_ID(IPC_MBOX_CPU_STATUS);
	ipc_msg_to_high.num_payload_words = 1;
	ipc_msg_to_high.payload_ptr = &(ipc_msg_to_high_payload[0]);
	ipc_msg_to_high_payload[0] = cpu_low_status;
	ipc_mailbox_write_msg(&ipc_msg_to_high);
}

/**
 * @brief Initialize the DCF Hardware Core
 *
 * This function initializes the DCF hardware core.
 *
 * @param None
 * @return None
 */
void wlan_mac_low_dcf_init(){
	u16 i;
	rx_frame_info* rx_mpdu;

	//Enable blocking of the Rx PHY following good-FCS reception
	REG_SET_BITS(WLAN_MAC_REG_CONTROL, (WLAN_MAC_CTRL_MASK_RX_PHY_BLOCK_EN | WLAN_MAC_CTRL_MASK_BLOCK_RX_ON_TX ));
	REG_CLEAR_BITS(WLAN_MAC_REG_CONTROL, (WLAN_MAC_CTRL_MASK_DISABLE_NAV | WLAN_MAC_CTRL_MASK_BLOCK_RX_ON_VALID_RXEND));

	//MAC timing parameters are in terms of units of 100 nanoseconds
	wlan_mac_set_slot(T_SLOT*10);
	wlan_mac_set_DIFS((T_DIFS)*10);
	wlan_mac_set_TxDIFS(((T_DIFS)*10) - (TX_PHY_DLY_100NSEC));
	wlan_mac_set_timeout(T_TIMEOUT*10);

	//TODO: NAV adjust needs verification
	//NAV adjust time - signed char (Fix8_0) value
	wlan_mac_set_NAV_adj(0*10);
	wlan_mac_set_EIFS(T_EIFS*10);

	//Clear any stale Rx events
	wlan_mac_dcf_hw_unblock_rx_phy();

	for(i=0;i < NUM_RX_PKT_BUFS; i++){
		rx_mpdu = (rx_frame_info*)RX_PKT_BUF_TO_ADDR(i);
		rx_mpdu->state = RX_MPDU_STATE_EMPTY;
	}

}

/**
 * @brief Send Exception to Upper-Level MAC
 *
 * This function generates an IPC message for the upper-level MAC
 * to tell it that something has gone wrong
 *
 * @param u32 reason
 *  - reason code for the exception
 * @return None
 */
inline void wlan_mac_low_send_exception(u32 reason){
	wlan_ipc_msg ipc_msg_to_high;
	u32 ipc_msg_to_high_payload[2];
	//Send an exception to CPU_HIGH along with a reason
	cpu_low_status |= CPU_STATUS_EXCEPTION;
	ipc_msg_to_high.msg_id = IPC_MBOX_MSG_ID(IPC_MBOX_CPU_STATUS);
	ipc_msg_to_high.num_payload_words = 2;
	ipc_msg_to_high.payload_ptr = &(ipc_msg_to_high_payload[0]);
	ipc_msg_to_high_payload[0] = cpu_low_status;
	ipc_msg_to_high_payload[1] = reason;
	ipc_mailbox_write_msg(&ipc_msg_to_high);

	userio_write_hexdisp_left(USERIO_BASEADDR, reason & 0xF);
	userio_write_hexdisp_right(USERIO_BASEADDR, (reason>>4) & 0xF);

	while(1){
		userio_write_leds_red(USERIO_BASEADDR, 0x5);
		usleep(250000);
		userio_write_leds_red(USERIO_BASEADDR, 0xA);
		usleep(250000);
	}
}

/**
 * @brief Poll for IPC Receptions
 *
 * This function is a non-blocking poll for IPC receptions from the
 * upper-level MAC.
 *
 * @param None
 * @return None
 */
inline void wlan_mac_low_poll_ipc_rx(){
	//Poll mailbox read msg
	if(ipc_mailbox_read_msg(&ipc_msg_from_high) == IPC_MBOX_SUCCESS){
		process_ipc_msg_from_high(&ipc_msg_from_high);
	}
}

/**
 * @brief Process IPC Reception
 *
 * This is an internal function to the WLAN MAC Low framework to process
 * received IPC messages and call the appropriate callback.
 *
 * @param None
 * @return None
 */
void process_ipc_msg_from_high(wlan_ipc_msg* msg){
	u16 tx_pkt_buf;
	u8 rate;
	tx_frame_info* tx_mpdu;
	wlan_ipc_msg ipc_msg_to_high;
	u32 status;
	mac_header_80211* tx_80211_header;
	u16 ACK_N_DBPS;
	u32 isLocked, owner;
	u64 new_timestamp;
	wlan_mac_low_tx_details* low_tx_details;
	u32 low_tx_details_size;
	u32 temp1;
	u32 temp2;
	u32* payload_to_write;

		switch(IPC_MBOX_MSG_ID_TO_MSG(msg->msg_id)){
			case IPC_MBOX_MEM_READ_WRITE:
				switch(msg->arg0){
					case IPC_REG_WRITE_MODE:
						payload_to_write = (u32*)((u8*)ipc_msg_from_high_payload + sizeof(ipc_reg_read_write));

						//IMPORTANT: this memcpy assumes the payload provided by CPU high is ready as-is
						// Any byte swapping (i.e. for payloads that arrive over Ethernet) *must* be performed
						//  before the payload is passed to this function
						memcpy((u8*)(((ipc_reg_read_write*)ipc_msg_from_high_payload)->baseaddr),
 							  (u8*)payload_to_write,
							  sizeof(u32)*((ipc_reg_read_write*)ipc_msg_from_high_payload)->num_words );

					break;
					case IPC_REG_READ_MODE:
						/*
						xil_printf("\nCPU Low Read:\n");
						xil_printf(" Addr: 0x%08x\n", (u32*)((ipc_reg_read_write*)ipc_msg_from_high_payload)->baseaddr);
						xil_printf(" N Wrds: %d\n", ((ipc_reg_read_write*)ipc_msg_from_high_payload)->num_words);

						xil_printf("Mem[0x%08x] = 0x%08x\n",
								(u32*)((ipc_reg_read_write*)ipc_msg_from_high_payload)->baseaddr,
								Xil_In32((u32*)((ipc_reg_read_write*)ipc_msg_from_high_payload)->baseaddr));
 	 	 	 	 	 	 */
						ipc_msg_to_high.msg_id = IPC_MBOX_MSG_ID(IPC_MBOX_MEM_READ_WRITE);
						ipc_msg_to_high.num_payload_words = ((ipc_reg_read_write*)ipc_msg_from_high_payload)->num_words;
						ipc_msg_to_high.payload_ptr = (u32*)((ipc_reg_read_write*)ipc_msg_from_high_payload)->baseaddr;

						ipc_mailbox_write_msg(&ipc_msg_to_high);

					break;
				}

			break;

			case IPC_MBOX_LOW_PARAM:
				switch(ipc_msg_from_high_payload[0]){
					case LOW_PARAM_PHYSICAL_CS_THRESH:

						if(ipc_msg_from_high_payload[1] < 1023){
							wlan_phy_rx_set_cca_thresh(ipc_msg_from_high_payload[1] * PHY_RX_RSSI_SUM_LEN);
						} else {
							wlan_phy_rx_set_cca_thresh(1023 * PHY_RX_RSSI_SUM_LEN);
						}


					break;
				}
			break;

			case IPC_MBOX_CONFIG_CHANNEL:
				mac_param_chan = ipc_msg_from_high_payload[0];
				//TODO: allow mac_param_chan to select 5GHz channels
				radio_controller_setCenterFrequency(RC_BASEADDR, (RC_ALL_RF), mac_param_band, mac_param_chan);
				wlan_mac_reset_NAV_counter();
			break;

			case IPC_MBOX_LOW_RANDOM_SEED:
				srand(ipc_msg_from_high_payload[0]);
			break;

			case IPC_MBOX_CONFIG_TX_CTRL_POW:
				mac_param_ctrl_tx_pow = (s8)ipc_msg_from_high_payload[0];
			break;

			case IPC_MBOX_CONFIG_RX_FILTER:
				temp1 = (u32)ipc_msg_from_high_payload[0];
				temp2 = 0;
				if((temp1 & RX_FILTER_FCS_MASK) == RX_FILTER_FCS_NOCHANGE){
					temp2 |= (mac_param_rx_filter & RX_FILTER_FCS_MASK);
				} else {
					temp2 |= (temp1 & RX_FILTER_FCS_MASK);
				}
				if((temp1 & RX_FILTER_HDR_NOCHANGE) == RX_FILTER_HDR_NOCHANGE){
					temp2 |= (mac_param_rx_filter & RX_FILTER_HDR_NOCHANGE);
				} else {
					temp2 |= (temp1 & RX_FILTER_HDR_NOCHANGE);
				}

				mac_param_rx_filter = temp2;
			break;

			case IPC_MBOX_CONFIG_RX_ANT_MODE:
				wlan_rx_config_ant_mode(ipc_msg_from_high_payload[0]);
			break;

			case IPC_MBOX_CONFIG_MAC:
				process_config_mac((ipc_config_mac*)ipc_msg_from_high_payload);
			break;

			case IPC_MBOX_SET_TIME:
				new_timestamp = *(u64*)ipc_msg_from_high_payload;
				wlan_mac_low_set_time(new_timestamp);
			break;

			case IPC_MBOX_CONFIG_PHY_TX:
				process_config_phy_tx((ipc_config_phy_tx*)ipc_msg_from_high_payload);
			break;

			case IPC_MBOX_CONFIG_PHY_RX:
				process_config_phy_rx((ipc_config_phy_rx*)ipc_msg_from_high_payload);
			break;

			case IPC_MBOX_TX_MPDU_READY:

				//Message is an indication that a Tx Pkt Buf needs processing
				tx_pkt_buf = msg->arg0;
				//TODO: Sanity check tx_pkt_buf so that it's within the number of tx packet bufs


				ipc_msg_to_high.msg_id = IPC_MBOX_MSG_ID(IPC_MBOX_TX_MPDU_ACCEPT);
				ipc_msg_to_high.num_payload_words = 0;
				ipc_msg_to_high.arg0 = tx_pkt_buf;
				ipc_mailbox_write_msg(&ipc_msg_to_high);


				if(lock_pkt_buf_tx(tx_pkt_buf) != PKT_BUF_MUTEX_SUCCESS){
					warp_printf(PL_ERROR, "Error: unable to lock TX pkt_buf %d\n", tx_pkt_buf);

					status_pkt_buf_tx(tx_pkt_buf, &isLocked, &owner);

					warp_printf(PL_ERROR, "	TX pkt_buf %d status: isLocked = %d, owner = %d\n", tx_pkt_buf, isLocked, owner);

				} else {

					tx_mpdu = (tx_frame_info*)TX_PKT_BUF_TO_ADDR(tx_pkt_buf);

					tx_mpdu->delay_accept = (u32)(get_usec_timestamp() - tx_mpdu->timestamp_create);

//					REG_SET_BITS(WLAN_RX_DEBUG_GPIO,0x80);


					//Convert rate index into rate code used in PHY's SIGNAL field
					//ACK_N_DBPS is used to calculate duration of received ACKs.
					//The selection of ACK rates given DATA rates is specified in 9.7.6.5.2 of 802.11-2012
					switch(tx_mpdu->params.phy.rate){
						case WLAN_MAC_RATE_1M:
							warp_printf(PL_ERROR, "Error: DSSS rate was selected for transmission. Only OFDM transmissions are supported.\n");

							//Default to BPSK 1/2 if user requests DSSS Tx (should never happen - CPU High will catch this first)
							rate = WLAN_PHY_RATE_BPSK12;
							ACK_N_DBPS = N_DBPS_R6;
						break;
						case WLAN_MAC_RATE_6M:
							rate = WLAN_PHY_RATE_BPSK12;
							ACK_N_DBPS = N_DBPS_R6;
						break;
						case WLAN_MAC_RATE_9M:
							rate = WLAN_PHY_RATE_BPSK34;
							ACK_N_DBPS = N_DBPS_R6;
						break;
						case WLAN_MAC_RATE_12M:
							rate = WLAN_PHY_RATE_QPSK12;
							ACK_N_DBPS = N_DBPS_R12;
						break;
						case WLAN_MAC_RATE_18M:
							rate = WLAN_PHY_RATE_QPSK34;
							ACK_N_DBPS = N_DBPS_R12;
						break;
						case WLAN_MAC_RATE_24M:
							rate = WLAN_PHY_RATE_16QAM12;
							ACK_N_DBPS = N_DBPS_R24;
						break;
						case WLAN_MAC_RATE_36M:
							rate = WLAN_PHY_RATE_16QAM34;
							ACK_N_DBPS = N_DBPS_R24;
						break;
						case WLAN_MAC_RATE_48M:
							rate = WLAN_PHY_RATE_64QAM23;
							ACK_N_DBPS = N_DBPS_R24;
						break;
						case WLAN_MAC_RATE_54M:
							rate = WLAN_PHY_RATE_64QAM34;
							ACK_N_DBPS = N_DBPS_R24;
						break;
						default:
							//Default to BSPK 1/2 if CPU High requests invalid rate
							rate = WLAN_PHY_RATE_BPSK12;
							ACK_N_DBPS = N_DBPS_R6;

							xil_printf("Invalid rate in Tx MPDU Info: %d\n", tx_mpdu->params.phy.rate);
						break;
					}


					if((tx_mpdu->flags) & TX_MPDU_FLAGS_FILL_DURATION){
						//Get pointer to start of MAC header in packet buffer
						tx_80211_header = (mac_header_80211*)(TX_PKT_BUF_TO_ADDR(tx_pkt_buf)+PHY_TX_PKT_BUF_MPDU_OFFSET);

						//Compute and fill in the duration of any time-on-air following this packet's transmission
						// For DATA Tx, DURATION = T_SIFS + T_ACK, where T_ACK is function of the ACK Tx rate
						tx_80211_header->duration_id = wlan_ofdm_txtime(sizeof(mac_header_80211_ACK)+WLAN_PHY_FCS_NBYTES, ACK_N_DBPS) + T_SIFS;
					}

					if((tx_mpdu->flags) & TX_MPDU_FLAGS_FILL_TIMESTAMP){
						//Some management packets contain the node's local 64-bit microsecond timer value
						// The Tx hardware can insert this value into the outgoing byte stream automatically
						// This ensures the timestamp value is not skewed by any pre-Tx deferrals

						//The macros below set the first and last byte index where the Tx logic should insert
						// the 8-byte timestamp.
						//In the current implementation these indexes must span an 8-byte-aligned
						// region of the packet buffer (i.e. (start_ind % 8)==0 )
						wlan_phy_tx_timestamp_ins_start((24+PHY_TX_PKT_BUF_PHY_HDR_SIZE));
						wlan_phy_tx_timestamp_ins_end((31+PHY_TX_PKT_BUF_PHY_HDR_SIZE));

					} else {
						//When start>end, the Tx logic will not insert any timestamp
						wlan_phy_tx_timestamp_ins_start(1);
						wlan_phy_tx_timestamp_ins_end(0);
					}

					//Allocate memory to store the record of each transmission of this MPDU
					// Allocating dynamically gives flexibility to change num_tx_max per packet, constrained only
					//  by CPU Low's heap size. malloc failures are handled by skipping TX_LOW log data but proceeding
					//  normally with actual MPDU transmission
					low_tx_details_size = sizeof(wlan_mac_low_tx_details)*tx_mpdu->params.mac.num_tx_max;
					low_tx_details = malloc(low_tx_details_size);

					//Submit the MPDU for transmission - this callback will return only when the MPDU Tx is
					// complete (after all re-transmissions, ACK Rx, timeouts, etc.)
					status = frame_tx_callback(tx_pkt_buf, rate, tx_mpdu->length, low_tx_details);

					//Record the total time this MPDU spent in the Tx state machine
					tx_mpdu->delay_done = (u32)(get_usec_timestamp() - (tx_mpdu->timestamp_create + (u64)(tx_mpdu->delay_accept)));

					//REG_CLEAR_BITS(WLAN_RX_DEBUG_GPIO,0x80);

					if(status == 0){
						tx_mpdu->tx_result = TX_MPDU_RESULT_SUCCESS;
					} else {
						tx_mpdu->tx_result = TX_MPDU_RESULT_FAILURE;
					}

					//Revert the state of the packet buffer and return control to CPU High
					tx_mpdu->state = TX_MPDU_STATE_EMPTY;

					if(unlock_pkt_buf_tx(tx_pkt_buf) != PKT_BUF_MUTEX_SUCCESS){
						warp_printf(PL_ERROR, "Error: unable to unlock TX pkt_buf %d\n", tx_pkt_buf);
						wlan_mac_low_send_exception(EXC_MUTEX_TX_FAILURE);
					} else {
						ipc_msg_to_high.msg_id =  IPC_MBOX_MSG_ID(IPC_MBOX_TX_MPDU_DONE);

						//Add the per-Tx-event details to the IPC message so CPU High can add them to the log as TX_LOW entries
						if(low_tx_details != NULL){
							ipc_msg_to_high.payload_ptr = (u32*)low_tx_details;

							//Make sure we don't overfill the IPC mailbox with TX_LOW data; truncate the Tx details if necessary
							if(low_tx_details_size < (IPC_BUFFER_MAX_NUM_WORDS << 2)){
								ipc_msg_to_high.num_payload_words = ( (tx_mpdu->num_tx)*sizeof(wlan_mac_low_tx_details) ) >> 2; // # of u32 words
							} else {
								ipc_msg_to_high.num_payload_words = ( ((IPC_BUFFER_MAX_NUM_WORDS << 2)/sizeof(wlan_mac_low_tx_details)  )*sizeof(wlan_mac_low_tx_details) ) >> 2; // # of u32 words
							}
						} else {
							ipc_msg_to_high.num_payload_words = 0;
							ipc_msg_to_high.payload_ptr = NULL;
						}
						ipc_msg_to_high.arg0 = tx_pkt_buf;
						ipc_mailbox_write_msg(&ipc_msg_to_high);
					}

					free(low_tx_details);
				}
			break;
		}
}

/**
 * @brief Set MAC microsecond timer
 *
 * This function sets the MAC core's microsecond timer
 * The timer starts at 0 at FPGA configuration time and counts up forever.
 * Some 802.11 handshakes require updating the local timer to match a partner
 *  node's timer value (reception of a beacon, for example)
 *
 * @param u64 new_time
 *  - the new base timestamp for the system
 * @return None
 */
void wlan_mac_low_set_time(u64 new_time) {
	Xil_Out32(WLAN_MAC_REG_SET_TIMESTAMP_LSB, (u32)new_time);
	Xil_Out32(WLAN_MAC_REG_SET_TIMESTAMP_MSB, (u32)(new_time>>32));

	Xil_Out32(WLAN_MAC_REG_CONTROL, (Xil_In32(WLAN_MAC_REG_CONTROL) & ~WLAN_MAC_CTRL_MASK_UPDATE_TIMESTAMP));
	Xil_Out32(WLAN_MAC_REG_CONTROL, (Xil_In32(WLAN_MAC_REG_CONTROL) | WLAN_MAC_CTRL_MASK_UPDATE_TIMESTAMP));
	Xil_Out32(WLAN_MAC_REG_CONTROL, (Xil_In32(WLAN_MAC_REG_CONTROL) & ~WLAN_MAC_CTRL_MASK_UPDATE_TIMESTAMP));
}


/**
 * @brief Process MAC Configuration
 *
 * This function processes MAC configurations.
 *
 * @param ipc_config_mac config_mac
 *  - configuration struct
 * @return None
 */
void process_config_mac(ipc_config_mac* config_mac){
	//TODO
}

/**
 * @brief Initialize Hardware Info Struct
 *
 * This function initializes the hardware info struct with values read from the EEPROM.
 *
 * @param None
 * @return None
 */
void wlan_mac_low_init_hw_info( u32 type ) {

	// Initialize the wlan_mac_hw_info structure to all zeros
	//
	memset( (void*)( &hw_info ), 0x0, sizeof( wlan_mac_hw_info ) );

	// Set General Node information
	hw_info.type          = type;
    hw_info.serial_number = w3_eeprom_readSerialNum(EEPROM_BASEADDR);
    hw_info.fpga_dna[1]   = w3_eeprom_read_fpga_dna(EEPROM_BASEADDR, 1);
    hw_info.fpga_dna[0]   = w3_eeprom_read_fpga_dna(EEPROM_BASEADDR, 0);

    // Set HW Addresses
    //   - NOTE:  The w3_eeprom_readEthAddr() function handles the case when the WARP v3
    //     hardware does not have a valid Ethernet address
    //
    // Use address 0 for the WLAN interface, address 1 for the Ethernet interface
	w3_eeprom_readEthAddr(EEPROM_BASEADDR, 0, hw_info.hw_addr_wlan);
	w3_eeprom_readEthAddr(EEPROM_BASEADDR, 1, hw_info.hw_addr_wn);

    // WARPNet will use ethernet device 1 (ETH_B) by default
    hw_info.wn_exp_eth_device = 1;
}

/**
 * @brief Return Hardware Info Struct
 *
 * This function returns the hardware info struct stored in the MAC Low Framework
 *
 * @param None
 * @return None
 */
inline wlan_mac_hw_info* wlan_mac_low_get_hw_info(){
	return &hw_info;
}

/**
 * @brief Return Current Channel Selection
 *
 * This function returns the the current channel.
 *
 * @param None
 * @return None
 */
inline u32 wlan_mac_low_get_active_channel(){
	return mac_param_chan;
}

inline s8 wlan_mac_low_get_current_ctrl_tx_pow(){
	return mac_param_ctrl_tx_pow;
}

inline u32 wlan_mac_low_get_current_rx_filter(){
	return mac_param_rx_filter;
}


/**
 * @brief Calculates Rx Power (in dBm)
 *
 * This function calculates receive power for a given band, RSSI and LNA gain. This
 * provides a reasonable estimate of Rx power, accurate to a few dB for standard waveforms.
 *
 * This function does not use the VGA gain setting or I/Q magnitudes. The PHY should use these
 * to refine its own power measurement if needed.
 *
 * @param None
 * @return None
 */
inline int wlan_mac_low_calculate_rx_power(u16 rssi, u8 lna_gain){
#define RSSI_SLOPE_BITSHIFT		3
#define RSSI_OFFSET_LNA_LOW		(-61)
#define RSSI_OFFSET_LNA_MED		(-76)
#define RSSI_OFFSET_LNA_HIGH	(-92)
	u8 band;
	int power = -100;

	band = mac_param_band;

	if(band == RC_24GHZ){
		switch(lna_gain){
			case 0:
			case 1:
				//Low LNA Gain State
				power = (rssi>>(RSSI_SLOPE_BITSHIFT + PHY_RX_RSSI_SUM_LEN_BITS)) + RSSI_OFFSET_LNA_LOW;
			break;

			case 2:
				//Medium LNA Gain State
				power = (rssi>>(RSSI_SLOPE_BITSHIFT + PHY_RX_RSSI_SUM_LEN_BITS)) + RSSI_OFFSET_LNA_MED;
			break;

			case 3:
				//High LNA Gain State
				power = (rssi>>(RSSI_SLOPE_BITSHIFT + PHY_RX_RSSI_SUM_LEN_BITS)) + RSSI_OFFSET_LNA_HIGH;
			break;

		}
	}
	return power;
}

/**
 * @brief Polls for PHY Rx Start
 *
 * This function polls for PHY receptions and calls the appropriate callback;
 *
 * @param None
 * @return u32
 * 	- status flags about the reception
 */
inline u32 wlan_mac_low_poll_frame_rx(){
	u32 return_status = 0;
	u32 rate, length;

	//Read the MAC/PHY status
	u32 mac_hw_status = wlan_mac_get_status();

	//Check if PHY is currently receiving or has finished receiving
	if(mac_hw_status & (WLAN_MAC_STATUS_MASK_PHY_RX_ACTIVE | WLAN_MAC_STATUS_MASK_RX_PHY_BLOCKED)) {

		return_status |= POLL_MAC_STATUS_RECEIVED_PKT; //We received something in this poll

		length = wlan_mac_get_rx_phy_length() - WLAN_PHY_FCS_NBYTES; //Strip off FCS
		rate =  wlan_mac_get_rx_phy_rate();

		//Translate the PHY's rate code (from the SIGNAL field) into a rate index for use by the MAC
		switch(rate){
			case WLAN_PHY_RATE_DSSS_1M:
				rate = WLAN_MAC_RATE_1M;
			break;
			case WLAN_PHY_RATE_BPSK12:
				rate = WLAN_MAC_RATE_6M;
			break;
			case WLAN_PHY_RATE_BPSK34:
				rate = WLAN_MAC_RATE_9M;
			break;
			case WLAN_PHY_RATE_QPSK12:
				rate = WLAN_MAC_RATE_12M;
			break;
			case WLAN_PHY_RATE_QPSK34:
				rate = WLAN_MAC_RATE_18M;
			break;
			case WLAN_PHY_RATE_16QAM12:
				rate = WLAN_MAC_RATE_24M;
			break;
			case WLAN_PHY_RATE_16QAM34:
				rate = WLAN_MAC_RATE_36M;
			break;
			case WLAN_PHY_RATE_64QAM23:
				rate = WLAN_MAC_RATE_48M;
			break;
			case WLAN_PHY_RATE_64QAM34:
				rate = WLAN_MAC_RATE_54M;
			break;
			default:
				//Assume DSSS if PHY-reported rate was somehow invalid
				rate = WLAN_MAC_RATE_1M;
			break;
		}

		if(wlan_mac_get_rx_phy_sel() == WLAN_RX_PHY_OFDM) {
			//OFDM packet is being received

			if(rate == WLAN_MAC_RATE_1M){
				xil_printf("ERROR: PHY reported DSSS rate, OFDM PHY active\n");	  //DEBUG FIXME
			}

			return_status |= frame_rx_callback(rx_pkt_buf, rate, length);
		} else {
			//DSSS packet is being received

			if(rate != WLAN_MAC_RATE_1M){
				xil_printf("ERROR: PHY reported OFDM rate, DSSS PHY active\n");	  //DEBUG FIXME
			}

			//Strip off extra pre-MAC-header bytes used in DSSS frames; this adjustment allows the next
			// function to treat OFDM and DSSS payloads the same
			length = length-5;

			return_status |= frame_rx_callback(rx_pkt_buf, rate, length);
		}

		//Current frame_rx_callback() always unblocks PHY
		// uncomment this unblock_rx_phy if custom frame_rx_callback does not wait to unblock the PHY
		//wlan_mac_dcf_hw_unblock_rx_phy();
	}

	return return_status;
}


/**
 * @brief Set Frame Reception Callback
 *
 * Tells the framework which function should be called when
 * the PHY begins processing a frame reception
 *
 * @param function_ptr_t callback
 *  - Pointer to callback function
 * @return None
 *
 */
inline void wlan_mac_low_set_frame_rx_callback(function_ptr_t callback){
	frame_rx_callback = callback;
}

/**
 * @brief Set Frame Transmission Callback
 *
 * Tells the framework which function should be called when
 * an MPDU is passed down from the upper-level MAC for
 * wireless transmission
 *
 * @param function_ptr_t callback
 *  - Pointer to callback function
 * @return None
 *
 */
inline void wlan_mac_low_set_frame_tx_callback(function_ptr_t callback){
	frame_tx_callback = callback;
}

/**
 * @brief Notify upper-level MAC of frame reception
 *
 * Sends an IPC message to the upper-level MAC to notify it
 * that a frame has been received and is ready to be processed
 *
 * @param None
 * @return None
 * @note This function assumes it is called in the same context where
 * rx_pkt_buf is still valid.
 */
void wlan_mac_low_frame_ipc_send(){
	wlan_ipc_msg ipc_msg_to_high;

	ipc_msg_to_high.msg_id = IPC_MBOX_MSG_ID(IPC_MBOX_RX_MPDU_READY);
	ipc_msg_to_high.arg0 = rx_pkt_buf;
	ipc_msg_to_high.num_payload_words = 0;
	ipc_mailbox_write_msg(&ipc_msg_to_high);
}

/**
 * @brief Search for and Lock Empty Packet Buffer (Blocking)
 *
 * This is a blocking function for finding and locking an empty rx packet
 * buffer.
 *
 * @param None
 * @return None
 * @note This function assumes it is called in the same context where
 * rx_pkt_buf is still valid.
 */
inline void wlan_mac_low_lock_empty_rx_pkt_buf(){
	//This function blocks until it safely finds a packet buffer for the PHY RX to store a future reception
	rx_frame_info* rx_mpdu;
	u32 i = 1;

	while(1){
		rx_pkt_buf = (rx_pkt_buf+1) % NUM_RX_PKT_BUFS;
		rx_mpdu = (rx_frame_info*) RX_PKT_BUF_TO_ADDR(rx_pkt_buf);
		if((rx_mpdu->state) == RX_MPDU_STATE_EMPTY){
			if(lock_pkt_buf_rx(rx_pkt_buf) == PKT_BUF_MUTEX_SUCCESS){

				//By default Rx pkt buffers are not zeroed out, to save the performance penalty of bzero'ing 2KB
				// However zeroing out the pkt buffer can be helpful when debugging Rx PHY behaviors
				//bzero((void *)(RX_PKT_BUF_TO_ADDR(rx_pkt_buf)), 2048);

				rx_mpdu->state = RX_MPDU_STATE_RX_PENDING;

				//Set the OFDM and DSSS PHYs to use the same Rx pkt buffer
				wlan_phy_rx_pkt_buf_ofdm(rx_pkt_buf);
				wlan_phy_rx_pkt_buf_dsss(rx_pkt_buf);

				return;
			}
		}
		xil_printf("Searching for empty packet buff %d\n", i);
		i++;
	}
}

/**
 * @brief Get the Current Microsecond Timestamp
 *
 * This function returns the current timestamp of the system
 *
 * @param None
 * @return u64
 * - microsecond timestamp
 */
inline u64 get_usec_timestamp(){

	//The MAC core register interface is only 32-bit, so the 64-bit timestamp
	// is read from two 32-bit registers and reconstructed here.

	u32 timestamp_high_u32;
	u32 timestamp_low_u32;
	u64 timestamp_u64;

	timestamp_high_u32 = Xil_In32(WLAN_MAC_REG_TIMESTAMP_MSB);
	timestamp_low_u32 = Xil_In32(WLAN_MAC_REG_TIMESTAMP_LSB);

	//Catch very rare race when 32-LSB of 64-bit value wraps between the two 32-bit reads
	if( (timestamp_high_u32 & 0x1) != (Xil_In32(WLAN_MAC_REG_TIMESTAMP_MSB) & 0x1) ) {
		//32-LSB wrapped - start over
		timestamp_high_u32 = Xil_In32(WLAN_MAC_REG_TIMESTAMP_MSB);
		timestamp_low_u32 = Xil_In32(WLAN_MAC_REG_TIMESTAMP_LSB);
	}

	timestamp_u64 = (((u64)timestamp_high_u32)<<32) + ((u64)timestamp_low_u32);

	return timestamp_u64;
}

/**
 * @brief Get the Rx Start Microsecond Timestamp
 *
 * This function returns the Rx start timestamp of the system
 *
 * @param None
 * @return u64
 * - microsecond timestamp
 */
inline u64 get_rx_start_timestamp() {
	u32 timestamp_high_u32;
	u32 timestamp_low_u32;
	u64 timestamp_u64;

	//RX_START timestamp is captured once per reception - no race condition between 32-bit reads
	timestamp_high_u32 = Xil_In32(WLAN_MAC_REG_RX_TIMESTAMP_MSB);
	timestamp_low_u32 = Xil_In32(WLAN_MAC_REG_RX_TIMESTAMP_LSB);
	timestamp_u64 = (((u64)timestamp_high_u32)<<32) + ((u64)timestamp_low_u32);

	return timestamp_u64;
}


/**
 * @brief Get the Tx Start Microsecond Timestamp
 *
 * This function returns the Tx start timestamp of the system
 *
 * @param None
 * @return u64
 * - microsecond timestamp
 */
inline u64 get_tx_start_timestamp() {

	u32 timestamp_high_u32;
	u32 timestamp_low_u32;
	u64 timestamp_u64;

	//TX_START timestamp is captured once per transmission - no race condition between 32-bit reads
	timestamp_high_u32 = Xil_In32(WLAN_MAC_REG_TX_TIMESTAMP_MSB);
	timestamp_low_u32 = Xil_In32(WLAN_MAC_REG_TX_TIMESTAMP_LSB);
	timestamp_u64 = (((u64)timestamp_high_u32)<<32) + ((u64)timestamp_low_u32);

	return timestamp_u64;
}

/**
 * @brief Unblock the Receive PHY
 *
 * This function unblocks the receive PHY, allowing it to overwrite the currently selected
 * rx packet buffer
 *
 * @param None
 * @return None
 */
void wlan_mac_dcf_hw_unblock_rx_phy() {
	//Posedge on WLAN_MAC_CTRL_MASK_RX_PHY_BLOCK_RESET unblocks PHY (clear then set here to ensure posedge)
	REG_CLEAR_BITS(WLAN_MAC_REG_CONTROL, WLAN_MAC_CTRL_MASK_RX_PHY_BLOCK_RESET);
	REG_SET_BITS(WLAN_MAC_REG_CONTROL, WLAN_MAC_CTRL_MASK_RX_PHY_BLOCK_RESET);
	REG_CLEAR_BITS(WLAN_MAC_REG_CONTROL, WLAN_MAC_CTRL_MASK_RX_PHY_BLOCK_RESET);

	return;
}

/**
 * @brief Finish PHY Reception
 *
 * This function returns the rx start timestamp of the system
 *
 * @param None
 * @return u32
 * - FCS status (RX_MPDU_STATE_FCS_GOOD or RX_MPDU_STATE_FCS_BAD)
 */
inline u32 wlan_mac_dcf_hw_rx_finish(){
	u32 mac_status;
	//Wait for the packet to finish
	do{
		mac_status = wlan_mac_get_status();
	} while(mac_status & WLAN_MAC_STATUS_MASK_PHY_RX_ACTIVE);

	//Check FCS

	if(mac_status & WLAN_MAC_STATUS_MASK_RX_FCS_GOOD) {
		return RX_MPDU_STATE_FCS_GOOD;
	} else {
		//Ensure auto-Tx logic is disabled if FCS was bad
		// Actual MAC code above should do the same thing - no harm disabling twice
		wlan_mac_auto_tx_en(0);
		return RX_MPDU_STATE_FCS_BAD;
	}
}

/**
 * @brief Convert dBm to Tx Gain Target
 *
 * This function maps a transmit power (in dBm) to a radio gain target.
 *
 * @param s8 power
 * @return u8 gain_target
 * - gain target in range of [0,63]
 */
inline u8 wlan_mac_low_dbm_to_gain_target(s8 power){
	s8 power_railed;
	u8 return_value;

	if(power > TX_POWER_MAX_DBM){
		power_railed = TX_POWER_MAX_DBM;
	} else if( power < TX_POWER_MIN_DBM){
		power_railed = TX_POWER_MIN_DBM;
	} else {
		power_railed = power;
	}

	return_value = (u8)(2*power_railed + 25);

	return return_value;
}

/**
 * @brief Force reset backoff counter in MAC hardware
 */
inline void wlan_mac_reset_backoff_counter() {
	Xil_Out32(WLAN_MAC_REG_CONTROL, Xil_In32(WLAN_MAC_REG_CONTROL) | WLAN_MAC_CTRL_MASK_RESET_BACKOFF);
	Xil_Out32(WLAN_MAC_REG_CONTROL, Xil_In32(WLAN_MAC_REG_CONTROL) & ~WLAN_MAC_CTRL_MASK_RESET_BACKOFF);
	return;
}

/**
 * @brief Force reset NAV counter in MAC hardware
 */
inline void wlan_mac_reset_NAV_counter() {
	Xil_Out32(WLAN_MAC_REG_CONTROL, Xil_In32(WLAN_MAC_REG_CONTROL) | WLAN_MAC_CTRL_MASK_RESET_NAV);
	Xil_Out32(WLAN_MAC_REG_CONTROL, Xil_In32(WLAN_MAC_REG_CONTROL) & ~WLAN_MAC_CTRL_MASK_RESET_NAV);
	return;
}
