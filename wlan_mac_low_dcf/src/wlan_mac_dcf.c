////////////////////////////////////////////////////////////////////////////////
// File   : wlan_mac_dcf
// Authors: Patrick Murphy (murphpo [at] mangocomm.com)
//			Chris Hunter (chunter [at] mangocomm.com)
//          Erik Welsh (welsh [at] mangocomm.com)
// License: Copyright 2013, Mango Communications. All rights reserved.
//          Distributed under the Mango Communications Reference Design License
//				See LICENSE.txt included in the design archive or
//				at http://mangocomm.com/802.11/license
////////////////////////////////////////////////////////////////////////////////

//Xilinx SDK includes
#include "xparameters.h"
#include <stdio.h>
#include <stdlib.h>
#include "xtmrctr.h"
#include "xio.h"
#include <string.h>

//WARP includes
#include "w3_userio.h"
#include "w3_ad_controller.h"
#include "w3_clock_controller.h"
#include "w3_iic_eeprom.h"
#include "radio_controller.h"

#include "wlan_mac_ipc_util.h"
#include "wlan_mac_802_11_defs.h"
#include "wlan_mac_misc_util.h"
#include "wlan_phy_util.h"
#include "wlan_mac_dcf.h"


#ifdef _DEBUG_

void wlan_mac_init_hw_info( void );

#endif

static u32 mac_param_chan;
static u32 stationShortRetryCount;
static u32 stationLongRetryCount;
static u32 cw_exp;
static u8 bcast_addr[6];
static u8 rx_pkt_buf;

static u32 cpu_low_status;


wlan_mac_hw_info  hw_info;


#define NUM_LEDS 4
u8 red_led_index;
u8 green_led_index;

wlan_ipc_msg ipc_msg_from_high;
u32 ipc_msg_from_high_payload[10];

u8 mac_param_band;


int main(){
	rx_frame_info* rx_mpdu;
	u32 status;
	wlan_ipc_msg ipc_msg_to_high;
	u32 ipc_msg_to_high_payload[1];

	xil_printf("\f----- wlan_mac_dcf -----\n");
	xil_printf("Compiled %s %s\n", __DATE__, __TIME__);

	mac_param_band = RC_24GHZ;

	cpu_low_status = 0;

	red_led_index = 0;
	red_led_index = 0;

	userio_write_leds_green(USERIO_BASEADDR, (1<<green_led_index));
	userio_write_leds_red(USERIO_BASEADDR, (1<<red_led_index));

	status = w3_node_init();

	if(status != 0) {
		xil_printf("Error in w3_node_init()! Exiting\n");
		return -1;
	}

	wlan_lib_init();

	//TODO: Debug: Wait for CPU_HIGH to set up all its interrupts
	//usleep(5000000);

	//create IPC message to receive into
	ipc_msg_from_high.payload_ptr = &(ipc_msg_from_high_payload[0]);

	//Begin by trying to lock packet buffer 0 for wireless receptions
	rx_pkt_buf = 0;
	if(lock_pkt_buf_rx(rx_pkt_buf) != PKT_BUF_MUTEX_SUCCESS){
		warp_printf(PL_ERROR, "Error: unable to lock pkt_buf %d\n", rx_pkt_buf);
		send_exception(EXC_MUTEX_TX_FAILURE);
		return -1;
	} else {
		rx_mpdu = (rx_frame_info*)RX_PKT_BUF_TO_ADDR(rx_pkt_buf);
		rx_mpdu->state = RX_MPDU_STATE_RX_PENDING;
		wlan_phy_rx_pkt_buf_ofdm(rx_pkt_buf);
		wlan_phy_rx_pkt_buf_dsss(rx_pkt_buf);
	}

	if(lock_pkt_buf_tx(TX_PKT_BUF_ACK) != PKT_BUF_MUTEX_SUCCESS){
		warp_printf(PL_ERROR, "Error: unable to lock ack packet buf %d\n", TX_PKT_BUF_ACK);
		send_exception(EXC_MUTEX_TX_FAILURE);
		return -1;
	}

	//FIXME: Unlock tx packet buffers 0 and 1, just in case a processor reset happened when CPU_LOW owned one
	//unlock_pkt_buf_tx(0);
	//unlock_pkt_buf_tx(1);



	//Move the PHY's starting address into the packet buffers by PHY_XX_PKT_BUF_PHY_HDR_OFFSET.
	//This accounts for the metadata located at the front of every packet buffer (Xx_mpdu_info)
	wlan_phy_rx_pkt_buf_phy_hdr_offset(PHY_RX_PKT_BUF_PHY_HDR_OFFSET);
	wlan_phy_tx_pkt_buf_phy_hdr_offset(PHY_TX_PKT_BUF_PHY_HDR_OFFSET);

	wlan_radio_init();
	wlan_phy_init();
	mac_dcf_init();
	
	cpu_low_status |= CPU_STATUS_INITIALIZED;
	//Send a message to other processor to say that this processor is initialized and ready
	ipc_msg_to_high.msg_id = IPC_MBOX_MSG_ID(IPC_MBOX_CPU_STATUS);
	ipc_msg_to_high.num_payload_words = 1;
	ipc_msg_to_high.payload_ptr = &(ipc_msg_to_high_payload[0]);
	ipc_msg_to_high_payload[0] = cpu_low_status;
	ipc_mailbox_write_msg(&ipc_msg_to_high);

	while(1){
		//Poll PHY RX start
		status = poll_mac_rx();

		//Poll mailbox read msg
		if(ipc_mailbox_read_msg(&ipc_msg_from_high) == IPC_MBOX_SUCCESS){
			process_ipc_msg_from_high(&ipc_msg_from_high);
		}
	}
	return 0;
}

void process_ipc_msg_from_high(wlan_ipc_msg* msg){
	u16 tx_pkt_buf;
	u8 rate;
	tx_frame_info* tx_mpdu;
	wlan_ipc_msg ipc_msg_to_high;
	u32 status;
	mac_header_80211* tx_80211_header;
	beacon_probe_frame* beacon;
	u16 n_dbps;
	u32 isLocked, owner;

		switch(IPC_MBOX_MSG_ID_TO_MSG(msg->msg_id)){
			case IPC_MBOX_CONFIG_RF_IFC:
				process_config_rf_ifc((ipc_config_rf_ifc*)ipc_msg_from_high_payload);
			break;

			case IPC_MBOX_CONFIG_MAC:
				process_config_mac((ipc_config_mac*)ipc_msg_from_high_payload);
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

					//xil_printf("CPU_LOW: processing buffer %d, length = %d, rate = %d\n", tx_pkt_buf, tx_mpdu->length, tx_mpdu->rate);

					//Convert human-readable rates into PHY rates
					//n_dbps is used to calculate duration of received ACKs.
					//This rate selection is specified in 9.7.6.5.2 of 802.11-2012
					switch(tx_mpdu->rate){
						case WLAN_MAC_RATE_1M:
							warp_printf(PL_ERROR, "Error: DSSS rate was selected for transmission. Only OFDM transmissions are supported.\n");
						break;
						case WLAN_MAC_RATE_6M:
							rate = WLAN_PHY_RATE_BPSK12;
							n_dbps = N_DBPS_R6;
						break;
						case WLAN_MAC_RATE_9M:
							rate = WLAN_PHY_RATE_BPSK34;
							n_dbps = N_DBPS_R6;
						break;
						case WLAN_MAC_RATE_12M:
							rate = WLAN_PHY_RATE_QPSK12;
							n_dbps = N_DBPS_R12;
						break;
						case WLAN_MAC_RATE_18M:
							rate = WLAN_PHY_RATE_QPSK34;
							n_dbps = N_DBPS_R12;
						break;
						case WLAN_MAC_RATE_24M:
							rate = WLAN_PHY_RATE_16QAM12;
							n_dbps = N_DBPS_R24;
						break;
						case WLAN_MAC_RATE_36M:
							rate = WLAN_PHY_RATE_16QAM34;
							n_dbps = N_DBPS_R24;
						break;
						case WLAN_MAC_RATE_48M:
							rate = WLAN_PHY_RATE_64QAM23;
							n_dbps = N_DBPS_R24;
						break;
						case WLAN_MAC_RATE_54M:
							rate = WLAN_PHY_RATE_64QAM34;
							n_dbps = N_DBPS_R24;
						break;
					}

					if((tx_mpdu->flags) & TX_MPDU_FLAGS_FILL_DURATION){
						tx_80211_header = (mac_header_80211*)(TX_PKT_BUF_TO_ADDR(tx_pkt_buf)+PHY_TX_PKT_BUF_MPDU_OFFSET);
						tx_80211_header->duration_id = wlan_ofdm_txtime(sizeof(mac_header_80211_ACK)+WLAN_PHY_FCS_NBYTES, n_dbps) + T_SIFS;
					}

					if((tx_mpdu->flags) & TX_MPDU_FLAGS_FILL_TIMESTAMP){
						beacon = (beacon_probe_frame*)(TX_PKT_BUF_TO_ADDR(tx_pkt_buf)+PHY_TX_PKT_BUF_MPDU_OFFSET+sizeof(mac_header_80211));
						beacon->timestamp = get_usec_timestamp();
					}

					//
					status = frame_transmit(tx_pkt_buf, rate, tx_mpdu->length);

					if(status == 0){
						tx_mpdu->state_verbose = TX_MPDU_STATE_VERBOSE_SUCCESS;
					} else {
						tx_mpdu->state_verbose = TX_MPDU_STATE_VERBOSE_FAILURE;
					}

					//Debug
					if(tx_mpdu->retry_count>0){
						//TODO: Raise GPIO
						//xil_printf("retry: %d\n",tx_mpdu->retry_count);
					//	REG_SET_BITS(WLAN_RX_DEBUG_GPIO,0x88);
					}

					//Debug

					tx_mpdu->state = TX_MPDU_STATE_EMPTY;

					if(unlock_pkt_buf_tx(tx_pkt_buf) != PKT_BUF_MUTEX_SUCCESS){
						warp_printf(PL_ERROR, "Error: unable to unlock TX pkt_buf %d\n", tx_pkt_buf);
						send_exception(EXC_MUTEX_TX_FAILURE);
					} else {
						ipc_msg_to_high.msg_id =  IPC_MBOX_MSG_ID(IPC_MBOX_TX_MPDU_DONE);
						ipc_msg_to_high.num_payload_words = 0;
						ipc_msg_to_high.arg0 = tx_pkt_buf;
						ipc_mailbox_write_msg(&ipc_msg_to_high);
					}

					//REG_CLEAR_BITS(WLAN_RX_DEBUG_GPIO,0x88);

				}
			break;
		}
}


u32 frame_receive(void* pkt_buf_addr, u8 rate, u16 length){
	//This function is called after a good SIGNAL field is detected by either PHY (OFDM or DSSS)
	//It is the responsibility of this function to wait until a sufficient number of bytes have been received
	// before it can start to process those bytes. When this function is called the eventual checksum status is
	// unknown. The packet contents can be provisionally processed (e.g. prepare an ACK for fast transmission),
	// but post-reception actions must be conditioned on the eventual FCS status (good or bad).
	//
	//Two primary job responsibilities of this function:
	// (1): Prepare outgoing ACK packets and instruct the MAC_DCF_HW core whether or not to send ACKs
	// (2): Pass up FCS-valid MPDUs to CPU_HIGH

	u32 return_value;
	u32 tx_length;
	u8 tx_rate;
	u8 unicast_to_me, to_broadcast;
	u16 rssi;
	u8 lna_gain;

	rx_frame_info* mpdu_info;
	mac_header_80211* rx_header;
	wlan_ipc_msg ipc_msg_to_high;



	return_value = 0;

	//Update the MPDU info struct (stored at 0 offset in the pkt buffer)
	mpdu_info = (rx_frame_info*)pkt_buf_addr;



	//Apply the mac_header_80211 template to the first bytes of the received MPDU
	rx_header = (mac_header_80211*)((void*)(pkt_buf_addr + PHY_RX_PKT_BUF_MPDU_OFFSET));

	if(length<sizeof(mac_header_80211_ACK)){
		//warp_printf(PL_ERROR, "Error: received packet of length %d, which is not valid\n", length);
		wlan_mac_dcf_hw_rx_finish();
		wlan_mac_dcf_hw_unblock_rx_phy();
		return return_value;
	}

	//tx_rate will be used in the construction of ACK packets. tx_rate is set to the incoming rx_rate
	//This rate selection is specified in 9.7.6.5.2 of 802.11-2012
	switch(rate){
		case WLAN_MAC_RATE_1M:
			tx_rate = WLAN_PHY_RATE_BPSK12; //DSSS transmissions are not supported.
		break;
		case WLAN_MAC_RATE_6M:
			tx_rate = WLAN_PHY_RATE_BPSK12;
		break;
		case WLAN_MAC_RATE_9M:
			tx_rate = WLAN_PHY_RATE_BPSK12;
		break;
		case WLAN_MAC_RATE_12M:
			tx_rate = WLAN_PHY_RATE_QPSK12;
		break;
		case WLAN_MAC_RATE_18M:
			tx_rate = WLAN_PHY_RATE_QPSK12;
		break;
		case WLAN_MAC_RATE_24M:
			tx_rate = WLAN_PHY_RATE_16QAM12;
		break;
		case WLAN_MAC_RATE_36M:
			tx_rate = WLAN_PHY_RATE_16QAM12;
		break;
		case WLAN_MAC_RATE_48M:
			tx_rate = WLAN_PHY_RATE_16QAM12;
		break;
		case WLAN_MAC_RATE_54M:
			tx_rate = WLAN_PHY_RATE_16QAM12;
		break;
	}

	//Wait until the PHY has written enough bytes so that the first address field can be processed
	while(wlan_mac_get_last_byte_index() < MAC_HW_LASTBYTE_ADDR1){};

	unicast_to_me = wlan_addr_eq(rx_header->address_1, hw_info.hw_addr_wlan);
	to_broadcast = wlan_addr_eq(rx_header->address_1, bcast_addr);

	//Prep outgoing ACK just in case it needs to be sent
	// ACKs are only sent for non-control frames addressed to this node
	if(unicast_to_me && !WLAN_IS_CTRL_FRAME(rx_header)) {

		//Delay param here is SIFS - rx latency - tx latency (determined experimentally)
		// TODO: Confirm this TxSIFS time for various Rx lengths and rates
		wlan_mac_auto_tx_params(TX_PKT_BUF_ACK, 48);

		tx_length = wlan_create_ack_frame((void*)(TX_PKT_BUF_TO_ADDR(TX_PKT_BUF_ACK) + PHY_TX_PKT_BUF_MPDU_OFFSET), rx_header->address_2);

		//Auto-Tx enable requires rising edge; one rising edge results in 0 or 1 transmissions, depending on Rx FCS
		wlan_mac_auto_tx_en(0);
		wlan_mac_auto_tx_en(1);

		wlan_phy_set_tx_signal(TX_PKT_BUF_ACK, tx_rate, tx_length + WLAN_PHY_FCS_NBYTES);

	}

	mpdu_info->length = (u16)length;
	mpdu_info->rate = (u8)rate;

	rssi = wlan_phy_rx_get_pkt_rssi();
	lna_gain = wlan_phy_rx_get_agc_RFG();

	//if(rate == WLAN_MAC_RATE_1M){
		//TODO: In this version of the hardware, RSSI is not latched on DSSS events.
	//	mpdu_info->rx_power = -100;
	//} else {
		mpdu_info->rx_power = calculate_rx_power(mac_param_band, rssi, lna_gain);
	//}
	mpdu_info->channel = mac_param_chan;


	//IPC_MBOX_GRP_PKT_BUF -> IPC_MBOX_GRP_RX_MPDU_DONE
	ipc_msg_to_high.msg_id = IPC_MBOX_MSG_ID(IPC_MBOX_RX_MPDU_READY);
	ipc_msg_to_high.arg0 = rx_pkt_buf;
	ipc_msg_to_high.num_payload_words = 0;

	if((rx_header->frame_control_1) == MAC_FRAME_CTRL1_SUBTYPE_ACK){
		return_value |= POLL_MAC_TYPE_ACK;
	}

	if(wlan_mac_dcf_hw_rx_finish() == RX_DONE_FCS_GOOD) {
		return_value |= POLL_MAC_STATUS_GOOD;

		if(unicast_to_me || to_broadcast){
			return_value |= POLL_MAC_ADDR_MATCH;

			if(!WLAN_IS_CTRL_FRAME(rx_header)) {
				//This packet should be passed up to CPU_high for further processing

				//Unlock the pkt buf mutex before passing the packet up
				// If this fails, something has gone horribly wrong
				if(unlock_pkt_buf_rx(rx_pkt_buf) != PKT_BUF_MUTEX_SUCCESS){
					xil_printf("Error: unable to unlock RX pkt_buf %d\n", rx_pkt_buf);
					send_exception(EXC_MUTEX_RX_FAILURE);
				} else {

					if(length >= sizeof(mac_header_80211)){
						mpdu_info->state = RX_MPDU_STATE_FCS_GOOD;

						ipc_mailbox_write_msg(&ipc_msg_to_high);

						//Find a free packet buffer and beging receiving packets there (blocks until free buf is found)
						lock_empty_rx_pkt_buf();

					} else {
						//TODO: This is a software fix to a known issue whose root cause is still a mystery. Occasionally, the PHY will report receptions of non-control packets
						//whose length is less than a full 801.11 header. This should not be possible, so this code is only to catch this case and not send these packets up
						//to CPU_HIGH.
						warp_printf(PL_ERROR, "Error: received non-control packet of length %d, which is not valid\n", length);
					}
				}
			} //END if(not control packet)
		} //END if (to_me or to_broadcast)
	} //END if (FCS good)

	//Unblock the PHY post-Rx (no harm calling this if the PHY isn't actually blocked)
	wlan_mac_dcf_hw_unblock_rx_phy();

	return return_value;
}

int frame_transmit(u8 pkt_buf, u8 rate, u16 length) {
	//This function manages the MAC_DCF_HW core. It is recursive -- it will call itself if retransmissions are needed.

	u8 req_timeout;
	u16 n_slots;
	u32 tx_status, rx_status;
	u8 expect_ack;
	tx_frame_info* mpdu_info = (tx_frame_info*) (TX_PKT_BUF_TO_ADDR(pkt_buf));



	//Check if the higher-layer MAC requires this transmission have a post-Tx timeout
	req_timeout = ((mpdu_info->flags) & TX_MPDU_FLAGS_REQ_TO) != 0;


	if(req_timeout == 0) update_cw(DCF_CW_UPDATE_BCAST_TX, pkt_buf);

	n_slots = rand_num_slots();

	//Write the SIGNAL field (interpreted by the PHY during Tx waveform generation)
	wlan_phy_set_tx_signal(pkt_buf, rate, length + WLAN_PHY_FCS_NBYTES);

	//Write the Tx params to the mac_dcf_hw core
	wlan_mac_MPDU_tx_params(pkt_buf, n_slots, req_timeout);

	//Submit the MPDU for transmission
	wlan_mac_MPDU_tx_start(1);
	wlan_mac_MPDU_tx_start(0);
	//FIXME: Check if this is a race condition

	//Wait for the MPDU Tx to finish
	do{

		tx_status = wlan_mac_get_status();

		//TODO: This is a software fix for a MAC_DCF_HW race condition
		if((tx_status & WLAN_MAC_STATUS_MASK_MPDU_TX_DONE) || ((tx_status & WLAN_MAC_STATUS_MASK_MPDU_TX_STATE)==WLAN_MAC_STATUS_MPDU_TX_STATE_DONE)) {
		//if(tx_status & WLAN_MAC_STATUS_MASK_MPDU_TX_DONE) {
			switch(tx_status & WLAN_MAC_STATUS_MASK_MPDU_TX_RESULT){
				case WLAN_MAC_STATUS_MPDU_TX_RESULT_SUCCESS:
					//Tx didn't require timeout, completed successfully
					REG_CLEAR_BITS(WLAN_RX_DEBUG_GPIO, 0xFF);
					return 0;
				break;

				case WLAN_MAC_STATUS_MPDU_TX_RESULT_TIMED_OUT:
					//Tx required tmieout, timeout expired with no receptions

					if(tx_status & WLAN_MAC_STATUS_MASK_PHY_CCA_BUSY) {
						REG_SET_BITS(WLAN_RX_DEBUG_GPIO, 0xFF);
					}
					//Update the contention window
					if(update_cw(DCF_CW_UPDATE_MPDU_TX_ERR, pkt_buf)) {
						REG_CLEAR_BITS(WLAN_RX_DEBUG_GPIO, 0xFF);
						return -1;
					}

					//Start a random backoff interval using the updated CW
					n_slots = rand_num_slots();
					wlan_mac_set_backoff_num_slots(n_slots);
					wlan_mac_backoff_start(1);
					wlan_mac_backoff_start(0);

					//Re-submit the same MPDU for re-transmission (it will defer to the backoff started above)
					REG_CLEAR_BITS(WLAN_RX_DEBUG_GPIO, 0xFF);
					return frame_transmit(pkt_buf, rate, length);

				break;
				case WLAN_MAC_STATUS_MPDU_TX_RESULT_RX_STARTED:
					expect_ack = 1;
					rx_status = poll_mac_rx();
					if((rx_status & POLL_MAC_TYPE_ACK) && (rx_status & POLL_MAC_STATUS_GOOD) && (rx_status & POLL_MAC_ADDR_MATCH) && (rx_status & POLL_MAC_STATUS_RECEIVED_PKT) && expect_ack){
						update_cw(DCF_CW_UPDATE_MPDU_RX_ACK, pkt_buf);
						n_slots = rand_num_slots();
						wlan_mac_dcf_hw_start_backoff(n_slots);
						REG_CLEAR_BITS(WLAN_RX_DEBUG_GPIO, 0xFF);
						return 0;
					} else {
						if(update_cw(DCF_CW_UPDATE_MPDU_TX_ERR, pkt_buf)){
							n_slots = rand_num_slots();
							wlan_mac_dcf_hw_start_backoff(n_slots);

							REG_CLEAR_BITS(WLAN_RX_DEBUG_GPIO, 0xFF);

							return -1;
						} else{
							n_slots = rand_num_slots();
							wlan_mac_dcf_hw_start_backoff(n_slots);
							REG_CLEAR_BITS(WLAN_RX_DEBUG_GPIO, 0xFF);
							return frame_transmit(pkt_buf, rate, length);
						}
					}
				break;
			}
		} else {
			if( (tx_status&WLAN_MAC_STATUS_MASK_PHY_RX_ACTIVE)){
				rx_status = poll_mac_rx();
			}
		}
	} while(tx_status & WLAN_MAC_STATUS_MASK_MPDU_TX_PENDING);

	REG_CLEAR_BITS(WLAN_RX_DEBUG_GPIO, 0xFF);
	return 0;

}

inline int update_cw(u8 reason, u8 pkt_buf){
	u32* station_rc_ptr;
	u8* rc_ptr;
	u8 retry_limit;
	tx_frame_info* tx_mpdu = (tx_frame_info*)TX_PKT_BUF_TO_ADDR(pkt_buf);

	mac_header_80211* tx_80211_header;

	tx_80211_header = (mac_header_80211*)((void*)(TX_PKT_BUF_TO_ADDR(pkt_buf)+PHY_TX_PKT_BUF_MPDU_OFFSET));

	rc_ptr = &(tx_mpdu->retry_count);

	if(tx_mpdu->length > RTS_THRESHOLD){
		station_rc_ptr = &stationLongRetryCount;
	} else {
		station_rc_ptr = &stationShortRetryCount;
	}

	retry_limit = tx_mpdu->retry_max;

	switch(reason){
		case DCF_CW_UPDATE_MPDU_TX_ERR:
			//Update counts and contention windows
			(*rc_ptr)++;
			(*station_rc_ptr)++;
			if(*rc_ptr == retry_limit) return -1;
			if(*station_rc_ptr == retry_limit){
				cw_exp = DCF_CW_EXP_MIN;
			} else {
				cw_exp = min(cw_exp+1, DCF_CW_EXP_MAX);
			}

			//Raise retry flag in mpdu
			tx_80211_header->frame_control_2 = (tx_80211_header->frame_control_2) | MAC_FRAME_CTRL2_FLAG_RETRY;
		break;
		case DCF_CW_UPDATE_BCAST_TX:
		case DCF_CW_UPDATE_MPDU_RX_ACK:
			//Update counts and contention windows
			(*station_rc_ptr) = 0;
			cw_exp = DCF_CW_EXP_MIN;
		break;
	}

	return 0;

}

inline unsigned int rand_num_slots(){
//Generates a uniform random value between [0, (2^(CW_EXP) - 1)], where CW_EXP is a positive integer
//This function assumed RAND_MAX = 2^31.
// |	CW_EXP	|	CW			|
// |	4		|	[0, 15]		|
// |	5       |	[0, 31]		|
// |	6		|	[0, 63]		|
// |	7		|	[0, 123]		|
// |	8		|	[0, 511]		|
// |	9		|	[0, 1023]	|
	volatile u32 n_slots = ((unsigned int)rand() >> (32-(cw_exp+1)));
	return n_slots;
}

void wlan_mac_dcf_hw_start_backoff(u16 num_slots) {
	//WLAN_MAC_REG_SW_BACKOFF_CTRL:
	// b[15:0]: Num slots
	// b[31]: Start backoff

	//Write num_slots and toggle start
	Xil_Out32(WLAN_MAC_REG_SW_BACKOFF_CTRL, (num_slots & 0xFFFF) | 0x80000000);
	Xil_Out32(WLAN_MAC_REG_SW_BACKOFF_CTRL, (num_slots & 0xFFFF));

	return;
}

void mac_dcf_init(){
	wlan_ipc_msg ipc_msg_to_high;
	u16 i;
	rx_frame_info* rx_mpdu;

	//Enable blocking of the Rx PHY following good-FCS reception
	REG_SET_BITS(WLAN_MAC_REG_CONTROL, (WLAN_MAC_CTRL_MASK_RX_PHY_BLOCK_EN | WLAN_MAC_CTRL_MASK_BLOCK_RX_ON_TX));
	REG_CLEAR_BITS(WLAN_MAC_REG_CONTROL, WLAN_MAC_CTRL_MASK_DISABLE_NAV);

	//TODO: These values need tweaking with scope to match the 802.11 standard
	wlan_mac_set_slot(9*10);
	wlan_mac_set_SIFS(10*10);
	wlan_mac_set_DIFS(28*10);
	wlan_mac_set_EIFS(128*10);
	wlan_mac_set_timeout(80*10);
	wlan_mac_set_TxDIFS(26*10);
	wlan_mac_set_MAC_slot(8*10);
	wlan_mac_set_NAV_adj(0*10);

	stationShortRetryCount = 0;
	stationLongRetryCount = 0;
	cw_exp = DCF_CW_EXP_MIN;

	wlan_mac_auto_tx_params(0, (20*10));

	//Clear any stale Rx events
	wlan_mac_dcf_hw_unblock_rx_phy();

	bcast_addr[0] = 0xFF;
	bcast_addr[1] = 0xFF;
	bcast_addr[2] = 0xFF;
	bcast_addr[3] = 0xFF;
	bcast_addr[4] = 0xFF;
	bcast_addr[5] = 0xFF;

	// Initialize the HW info structure
	wlan_mac_init_hw_info();

#ifdef _DEBUG_
	print_wlan_mac_hw_info( &hw_info );
#endif

	//Send a message to other processor to identify mac_addr
	ipc_msg_to_high.msg_id = IPC_MBOX_MSG_ID(IPC_MBOX_HW_INFO);
	ipc_msg_to_high.num_payload_words = 8;
	ipc_msg_to_high.payload_ptr = (u32 *) &(hw_info);

//	ipc_msg_to_high.payload_ptr = &(ipc_msg_to_high_payload[0]);
//	memcpy((void*) &(ipc_msg_to_high_payload[0]), (void*) &(hw_info), sizeof( wlan_mac_hw_info ) );
	ipc_mailbox_write_msg(&ipc_msg_to_high);

	for(i=0;i < NUM_RX_PKT_BUFS; i++){
		rx_mpdu = (rx_frame_info*)RX_PKT_BUF_TO_ADDR(i);
		rx_mpdu->state = RX_MPDU_STATE_EMPTY;
	}

}


void wlan_mac_init_hw_info( void ) {

	// Initialize the wlan_mac_hw_info structure to all zeros
	//
	memset( (void*)( &hw_info ), 0x0, sizeof( wlan_mac_hw_info ) );


	// Set General Node information
    hw_info.serial_number = w3_eeprom_readSerialNum(EEPROM_BASEADDR);
    hw_info.fpga_dna[1]   = w3_eeprom_read_fpga_dna(EEPROM_BASEADDR, 1);
    hw_info.fpga_dna[0]   = w3_eeprom_read_fpga_dna(EEPROM_BASEADDR, 0);

    // Set HW Addresses
    //   - NOTE:  The w3_eeprom_readEthAddr() function handles the case when the WARP v3
    //     hardware does not have a valid Ethernet address
    //
	w3_eeprom_readEthAddr(EEPROM_BASEADDR, 0, hw_info.hw_addr_wlan);
	w3_eeprom_readEthAddr(EEPROM_BASEADDR, 1, hw_info.hw_addr_wn);

    // WARPNet will use ethernet device 1 unless you change this function
    hw_info.wn_exp_eth_device = 1;
}

#ifdef _DEBUG_

void print_wlan_mac_hw_info( wlan_mac_hw_info * info ) {
	int i;

	xil_printf("WLAN MAC HW INFO:  \n");
	xil_printf("  Type             :  0x%8x\n", info->type);
	xil_printf("  Serial Number    :  %d\n",    info->serial_number);
	xil_printf("  FPGA DNA         :  0x%8x  0x%8x\n", info->fpga_dna[1], info->fpga_dna[0]);
	xil_printf("  WLAN EXP ETH Dev :  %d\n",    info->wn_exp_eth_device);

	xil_printf("  WLAN EXP HW Addr :  %02x",    info->hw_addr_wn[0]);
	for( i = 1; i < WLAN_MAC_ETH_ADDR_LEN; i++ ) {
		xil_printf(":%02x", info->hw_addr_wn[i]);
	}
	xil_printf("\n");

	xil_printf("  WLAN HW Addr     :  %02x",    info->hw_addr_wlan[0]);
	for( i = 1; i < WLAN_MAC_ETH_ADDR_LEN; i++ ) {
		xil_printf(":%02x", info->hw_addr_wlan[i]);
	}
	xil_printf("\n");

	xil_printf("END \n");

}

#endif


void wlan_mac_dcf_hw_unblock_rx_phy() {
	//Posedge on WLAN_MAC_CTRL_MASK_RX_PHY_BLOCK_RESET unblocks PHY (clear then set here to ensure posedge)
	REG_CLEAR_BITS(WLAN_MAC_REG_CONTROL, WLAN_MAC_CTRL_MASK_RX_PHY_BLOCK_RESET);
	REG_SET_BITS(WLAN_MAC_REG_CONTROL, WLAN_MAC_CTRL_MASK_RX_PHY_BLOCK_RESET);
	REG_CLEAR_BITS(WLAN_MAC_REG_CONTROL, WLAN_MAC_CTRL_MASK_RX_PHY_BLOCK_RESET);

	return;
}

inline u32 poll_mac_rx(){
	u32 return_status = 0;
	u32 rate, length;
	u32 mac_hw_status = wlan_mac_get_status();

	//is the MAC currently blocking the rx PHY
	//WLAN_MAC_STATUS_MASK_RX_PHY_BLOCKED
	if(mac_hw_status & (WLAN_MAC_STATUS_MASK_PHY_RX_ACTIVE | WLAN_MAC_STATUS_MASK_RX_PHY_BLOCKED)) {

		return_status |= POLL_MAC_STATUS_RECEIVED_PKT; //We received something in this poll

		length = wlan_mac_get_rx_phy_length() - WLAN_PHY_FCS_NBYTES; //Strip off FCS
		rate =  wlan_mac_get_rx_phy_rate();

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
		}

		if(wlan_mac_get_rx_phy_sel() == WLAN_RX_PHY_OFDM) {
			//OFDM packet is being received
			return_status |= frame_receive((void *)RX_PKT_BUF_TO_ADDR(rx_pkt_buf), rate, length);
		} else {
			//DSSS packet is being received
			length = length-5;
			return_status |= frame_receive((void *)RX_PKT_BUF_TO_ADDR(rx_pkt_buf), rate, length);
		}
		//wlan_mac_dcf_hw_unblock_rx_phy();
	}

	return return_status;
}

int wlan_create_ack_frame(void* pkt_buf, u8* address_ra) {

	mac_header_80211_ACK* ack_header;
	ack_header = (mac_header_80211_ACK*)(pkt_buf);

	ack_header->frame_control_1 = MAC_FRAME_CTRL1_SUBTYPE_ACK;
	ack_header->frame_control_2 = 0;
	ack_header->duration_id = 0;
	memcpy(ack_header->address_ra, address_ra, 6);

	return sizeof(mac_header_80211_ACK);
}

inline int wlan_mac_dcf_hw_rx_finish(){
	u32 mac_status;
	//Wait for the packet to finish
	do{
		mac_status = wlan_mac_get_status();
	} while(mac_status & WLAN_MAC_STATUS_MASK_PHY_RX_ACTIVE);

	//Check FCS

	if(mac_status & WLAN_MAC_STATUS_MASK_RX_FCS_GOOD) {
		green_led_index = (green_led_index + 1) % NUM_LEDS;
		userio_write_leds_green(USERIO_BASEADDR, (1<<green_led_index));
		return 0;
	} else {
		wlan_mac_auto_tx_en(0);
		red_led_index = (red_led_index + 1) % NUM_LEDS;
		userio_write_leds_red(USERIO_BASEADDR, (1<<red_led_index));
		return -1;
	}

}

inline void lock_empty_rx_pkt_buf(){
	//This function blocks until it safely finds a packet buffer for the PHY RX to stash receptions
	rx_frame_info* rx_mpdu;
	u32 i = 1;

	while(1){
		rx_pkt_buf = (rx_pkt_buf+1) % NUM_RX_PKT_BUFS;
		rx_mpdu = (rx_frame_info*) RX_PKT_BUF_TO_ADDR(rx_pkt_buf);
		if((rx_mpdu->state) == RX_MPDU_STATE_EMPTY){
			if(lock_pkt_buf_rx(rx_pkt_buf) == PKT_BUF_MUTEX_SUCCESS){

				//bzero((void *)(RX_PKT_BUF_TO_ADDR(rx_pkt_buf)), 2048);

				rx_mpdu->state = RX_MPDU_STATE_RX_PENDING;
				wlan_phy_rx_pkt_buf_ofdm(rx_pkt_buf);
				wlan_phy_rx_pkt_buf_dsss(rx_pkt_buf);


				return;
			}
		}
		i++;
	}
}

inline u64 get_usec_timestamp(){
	u32 timestamp_high_u32;
	u32 timestamp_low_u32;
	u64 timestamp_u64;
	timestamp_high_u32 = Xil_In32(WLAN_MAC_REG_TIMESTAMP_MSB);
	timestamp_low_u32 = Xil_In32(WLAN_MAC_REG_TIMESTAMP_LSB);
	timestamp_u64 = (((u64)timestamp_high_u32)<<32) + ((u64)timestamp_low_u32);
	return timestamp_u64;
}

inline u64 get_rx_start_timestamp() {
	u32 timestamp_high_u32;
	u32 timestamp_low_u32;
	u64 timestamp_u64;
	timestamp_high_u32 = Xil_In32(WLAN_MAC_REG_RX_TIMESTAMP_MSB);
	timestamp_low_u32 = Xil_In32(WLAN_MAC_REG_RX_TIMESTAMP_LSB);
	timestamp_u64 = (((u64)timestamp_high_u32)<<32) + ((u64)timestamp_low_u32);
	return timestamp_u64;
}

void process_config_rf_ifc(ipc_config_rf_ifc* config_rf_ifc){

	if((config_rf_ifc->channel)!=0xFF){
		mac_param_chan = config_rf_ifc->channel;
		//TODO: allow mac_param_chan to select 5GHz channels
		radio_controller_setCenterFrequency(RC_BASEADDR, RC_RFA, mac_param_band, mac_param_chan);
		warp_printf(PL_ERROR, "CPU_LOW: Tuned to channel %d\n", mac_param_chan);
	}
}

void process_config_mac(ipc_config_mac* config_mac){

}


inline void send_exception(u32 reason){
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

#define RSSI_SLOPE_BITSHIFT		4
#define RSSI_OFFSET_LNA_LOW		(-61)
#define RSSI_OFFSET_LNA_MED		(-76)
#define RSSI_OFFSET_LNA_HIGH	(-92)
inline int calculate_rx_power(u8 band, u16 rssi, u8 lna_gain){
	int power = -100;

	//TODO: In this version of hardware, RSSI is latched pre-AGC so we should assume a high LNA gain
	// lna_gain = 3;

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
