/** @file wlan_mac_dcf.c
 *  @brief Distributed Coordination Function
 *
 *  This contains code to implement the 802.11 DCF.
 *
 *  @copyright Copyright 2014, Mango Communications. All rights reserved.
 *          Distributed under the Mango Communications Reference Design License
 *				See LICENSE.txt included in the design archive or
 *				at http://mangocomm.com/802.11/license
 *
 *  @author Chris Hunter (chunter [at] mangocomm.com)
 *  @author Patrick Murphy (murphpo [at] mangocomm.com)
 *  @author Erik Welsh (welsh [at] mangocomm.com)
 *  @bug
 *  - NAV timing needs to be verified
 *  - 5 GHz support needs to be added.
 */

/***************************** Include Files *********************************/

// Xilinx SDK includes
#include "xparameters.h"
#include <stdio.h>
#include <stdlib.h>
#include "xtmrctr.h"
#include "xio.h"
#include <string.h>

// WARP includes
#include "wlan_mac_low.h"
#include "w3_userio.h"
#include "radio_controller.h"

#include "wlan_mac_ipc_util.h"
#include "wlan_mac_802_11_defs.h"
#include "wlan_mac_misc_util.h"
#include "wlan_phy_util.h"
#include "wlan_mac_dcf.h"

#include "wlan_exp.h"

/*************************** Constant Definitions ****************************/

#define WARPNET_TYPE_80211_LOW         WARPNET_TYPE_80211_LOW_DCF
#define NUM_LEDS                       4

/*********************** Global Variable Definitions *************************/


/*************************** Variable Definitions ****************************/
static u32              stationShortRetryCount;
static u32              stationLongRetryCount;
static u32              cw_exp;

static u8               eeprom_addr[6];

u8                      red_led_index;
u8                      green_led_index;

/******************************** Functions **********************************/

int main(){
	wlan_mac_hw_info* hw_info;
	xil_printf("\f");
	xil_printf("----- Mango 802.11 Reference Design -----\n");
	xil_printf("----- v0.9 Beta -------------------------\n");
	xil_printf("----- wlan_mac_dcf ----------------------\n");
	xil_printf("Compiled %s %s\n\n", __DATE__, __TIME__);

	xil_printf("Note: this UART is currently printing from CPU_LOW. To view prints from\n");
	xil_printf("and interact with CPU_HIGH, raise the right-most User I/O DIP switch bit.\n");
	xil_printf("This switch can be toggled live while the design is running.\n\n");

	stationShortRetryCount = 0;
	stationLongRetryCount = 0;
	cw_exp = DCF_CW_EXP_MIN;

	wlan_tx_config_ant_mode(TX_ANTMODE_SISO_ANTA);

	red_led_index = 0;
	green_led_index = 0;
	userio_write_leds_green(USERIO_BASEADDR, (1<<green_led_index));
	userio_write_leds_red(USERIO_BASEADDR, (1<<red_led_index));

	wlan_mac_low_init(WARPNET_TYPE_80211_LOW);

	hw_info = wlan_mac_low_get_hw_info();
	memcpy(eeprom_addr,hw_info->hw_addr_wlan,6);

	wlan_mac_low_set_frame_rx_callback((void*)frame_receive);
	wlan_mac_low_set_frame_tx_callback((void*)frame_transmit);

	if(lock_pkt_buf_tx(TX_PKT_BUF_ACK) != PKT_BUF_MUTEX_SUCCESS){
		warp_printf(PL_ERROR, "Error: unable to lock ack packet buf %d\n", TX_PKT_BUF_ACK);
		wlan_mac_low_send_exception(EXC_MUTEX_TX_FAILURE);
		return -1;
	}

	wlan_mac_low_finish_init();

    xil_printf("Initialization Finished\n");

	while(1){
		//Poll PHY RX start
		wlan_mac_low_poll_frame_rx();

		//Poll IPC rx
		wlan_mac_low_poll_ipc_rx();
	}
	return 0;
}

u32 frame_receive(u8 rx_pkt_buf, u8 rate, u16 length){
	//This function is called after a good SIGNAL field is detected by either PHY (OFDM or DSSS)
	//It is the responsibility of this function to wait until a sufficient number of bytes have been received
	// before it can start to process those bytes. When this function is called the eventual checksum status is
	// unknown. The packet contents can be provisionally processed (e.g. prepare an ACK for fast transmission),
	// but post-reception actions must be conditioned on the eventual FCS status (good or bad).
	//
	// Note: The timing of this function is critical for correct operation of the 802.11 DCF. It is not
	// safe to add large delays to this function (e.g. xil_printf or usleep)
	//
	//Two primary job responsibilities of this function:
	// (1): Prepare outgoing ACK packets and instruct the MAC_DCF_HW core whether or not to send ACKs
	// (2): Pass up MPDUs (FCS valid or invalid) to CPU_HIGH

	u32 return_value;
	u32 tx_length;
	u8 tx_rate;
	u8 unicast_to_me, to_multicast;
	u16 rssi;
	u8 lna_gain;
	u8 active_rx_ant;
	u32 rx_filter;
	u8 pass_up;

	rx_frame_info* mpdu_info;
	mac_header_80211* rx_header;

	REG_SET_BITS(WLAN_RX_DEBUG_GPIO,0x40);

	void* pkt_buf_addr = (void *)RX_PKT_BUF_TO_ADDR(rx_pkt_buf);

	return_value = 0;

	//Update the MPDU info struct (stored at 0 offset in the pkt buffer)
	mpdu_info = (rx_frame_info*)pkt_buf_addr;

	//Apply the mac_header_80211 template to the first bytes of the received MPDU
	rx_header = (mac_header_80211*)((void*)(pkt_buf_addr + PHY_RX_PKT_BUF_MPDU_OFFSET));

	if(length<sizeof(mac_header_80211_ACK)){
		//warp_printf(PL_ERROR, "Error: received packet of length %d, which is not valid\n", length);
		wlan_mac_dcf_hw_rx_finish();
		wlan_mac_dcf_hw_unblock_rx_phy();
		REG_CLEAR_BITS(WLAN_RX_DEBUG_GPIO,0x40);
		return return_value;
	}

	//tx_rate will be used in the construction of ACK packets. tx_rate is set to the incoming rx_rate
	//This rate selection is specified in 9.7.6.5.2 of 802.11-2012
	switch(rate){
		default:
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

	unsigned char ack_tx_ant_mask = 0;
	active_rx_ant = wlan_phy_rx_get_active_rx_ant();
	switch(active_rx_ant){
		case RX_ANTMODE_SISO_ANTA:
			ack_tx_ant_mask |= 0x1;
		break;
		case RX_ANTMODE_SISO_ANTB:
			ack_tx_ant_mask |= 0x2;
		break;
		case RX_ANTMODE_SISO_ANTC:
			ack_tx_ant_mask |= 0x4;
		break;
		case RX_ANTMODE_SISO_ANTD:
			ack_tx_ant_mask |= 0x8;
		break;
		default:
			ack_tx_ant_mask = 0x1;
		break;
	}

	//Wait until the PHY has written enough bytes so that the first address field can be processed
	while(wlan_mac_get_last_byte_index() < MAC_HW_LASTBYTE_ADDR1){};

	unicast_to_me = wlan_addr_eq(rx_header->address_1, eeprom_addr);
	to_multicast = wlan_addr_mcast(rx_header->address_1);

	//Prep outgoing ACK just in case it needs to be sent
	// ACKs are only sent for non-control frames addressed to this node
	if(unicast_to_me && !WLAN_IS_CTRL_FRAME(rx_header)) {
		//Note: the auto tx subsystem will only fire if enabled by software AND the preceeding reception
		//has a good FCS. So, as software, we do not need to worry about FCS status when enabling the
		//the subsystem.

		//Auto TX Delay is in units of 100ns. This delay runs from RXEND of the preceeding reception.
		wlan_mac_auto_tx_params(TX_PKT_BUF_ACK, ((T_SIFS*10)-((TX_PHY_DLY_100NSEC))),wlan_mac_low_dbm_to_gain_target(wlan_mac_low_get_current_ctrl_tx_pow()), ack_tx_ant_mask);

		tx_length = wlan_create_ack_frame((void*)(TX_PKT_BUF_TO_ADDR(TX_PKT_BUF_ACK) + PHY_TX_PKT_BUF_MPDU_OFFSET), rx_header->address_2);

		//Auto-Tx enable requires rising edge; one rising edge results in 0 or 1 transmissions, depending on Rx FCS
		wlan_mac_auto_tx_en(0);
		wlan_mac_auto_tx_en(1);

		wlan_phy_set_tx_signal(TX_PKT_BUF_ACK, tx_rate, tx_length + WLAN_PHY_FCS_NBYTES);

	}

	mpdu_info->flags = 0;
	mpdu_info->length = (u16)length;
	mpdu_info->rate = (u8)rate;

	mpdu_info->channel = wlan_mac_low_get_active_channel();

	if((rx_header->frame_control_1) == MAC_FRAME_CTRL1_SUBTYPE_ACK){
		return_value |= POLL_MAC_TYPE_ACK;
	}

	mpdu_info->timestamp = get_rx_start_timestamp();

	mpdu_info->state = wlan_mac_dcf_hw_rx_finish(); //Blocks until reception is complete

	active_rx_ant = wlan_phy_rx_get_active_rx_ant();
	mpdu_info->ant_mode = active_rx_ant;

	mpdu_info->rf_gain = wlan_phy_rx_get_agc_RFG(active_rx_ant);
	mpdu_info->bb_gain = wlan_phy_rx_get_agc_BBG(active_rx_ant);

	rssi = wlan_phy_rx_get_pkt_rssi(active_rx_ant);

	lna_gain = wlan_phy_rx_get_agc_RFG(active_rx_ant);

	mpdu_info->rx_power = wlan_mac_low_calculate_rx_power(rssi, lna_gain);

	if(mpdu_info->state == RX_MPDU_STATE_FCS_GOOD){
		green_led_index = (green_led_index + 1) % NUM_LEDS;
		userio_write_leds_green(USERIO_BASEADDR, (1<<green_led_index));

		return_value |= POLL_MAC_STATUS_GOOD;

		rx_filter = wlan_mac_low_get_current_rx_filter();

		switch(rx_filter & RX_FILTER_HDR_MASK){
			default:
			case RX_FILTER_HDR_ADDR_MATCH_MPDU:
				pass_up = (unicast_to_me || to_multicast) && !WLAN_IS_CTRL_FRAME(rx_header);
			break;
			case RX_FILTER_HDR_ALL_MPDU:
				pass_up = !WLAN_IS_CTRL_FRAME(rx_header);
			break;
			case RX_FILTER_HDR_ALL:
				pass_up = 1;
			break;
		}

		if(!WLAN_IS_CTRL_FRAME(rx_header) && (mpdu_info->length < sizeof(mac_header_80211))){
			pass_up = 0;
		}

		if(unicast_to_me){
			return_value |= POLL_MAC_ADDR_MATCH;
		}

		if(pass_up){
			//This packet should be passed up to CPU_high for further processing

			if(!WLAN_IS_CTRL_FRAME(rx_header)){
				if(unicast_to_me){
					//This good FCS, unicast, noncontrol packet was ACKed.
					mpdu_info->flags |= RX_MPDU_FLAGS_ACKED;
				}

				if((rx_header->frame_control_2) & MAC_FRAME_CTRL2_FLAG_RETRY){
					mpdu_info->flags |= RX_MPDU_FLAGS_RETRY;
				}
			}

			//Unlock the pkt buf mutex before passing the packet up
			// If this fails, something has gone horribly wrong
			if(unlock_pkt_buf_rx(rx_pkt_buf) != PKT_BUF_MUTEX_SUCCESS){
				xil_printf("Error: unable to unlock RX pkt_buf %d\n", rx_pkt_buf);
				wlan_mac_low_send_exception(EXC_MUTEX_RX_FAILURE);
			} else {
				wlan_mac_low_frame_ipc_send();
				//Find a free packet buffer and begin receiving packets there (blocks until free buf is found)
				wlan_mac_low_lock_empty_rx_pkt_buf();
			}
		} //END if (to_me or to_multicast)
	} else {
		red_led_index = (red_led_index + 1) % NUM_LEDS;
		userio_write_leds_red(USERIO_BASEADDR, (1<<red_led_index));


		rx_filter = wlan_mac_low_get_current_rx_filter();

		switch(rx_filter & RX_FILTER_FCS_MASK){
			default:
			case RX_FILTER_FCS_GOOD:
				pass_up = 0;
			break;
			case RX_FILTER_FCS_ALL:
				pass_up = 1;
			break;
		}

		if(pass_up){
			//Unlock the pkt buf mutex before passing the packet up
			// If this fails, something has gone horribly wrong
			if(unlock_pkt_buf_rx(rx_pkt_buf) != PKT_BUF_MUTEX_SUCCESS){
				xil_printf("Error: unable to unlock RX pkt_buf %d\n", rx_pkt_buf);
				wlan_mac_low_send_exception(EXC_MUTEX_RX_FAILURE);
			} else {
				wlan_mac_low_frame_ipc_send();
				//Find a free packet buffer and begin receiving packets there (blocks until free buf is found)
				wlan_mac_low_lock_empty_rx_pkt_buf();
			}
		}

	} //END else (FCS bad)

	//Unblock the PHY post-Rx (no harm calling this if the PHY isn't actually blocked)
	wlan_mac_dcf_hw_unblock_rx_phy();

	//If auto-tx ACK is currently transmitting, wait for it to finish
	while(wlan_mac_get_status() & WLAN_MAC_STATUS_MASK_AUTO_TX_PENDING){}

	REG_CLEAR_BITS(WLAN_RX_DEBUG_GPIO,0x40);
	return return_value;
}


int frame_transmit(u8 pkt_buf, u8 rate, u16 length, wlan_mac_low_tx_details* low_tx_details) {
	//This function manages the MAC_DCF_HW core.

	u32 i;
	u8 req_timeout;
	u16 n_slots;
	u32 tx_status, rx_status;
	u8 expect_ack;
	tx_frame_info* mpdu_info = (tx_frame_info*) (TX_PKT_BUF_TO_ADDR(pkt_buf));
	u64 last_tx_timestamp;

	last_tx_timestamp = (u64)(mpdu_info->delay_accept) + (u64)(mpdu_info->timestamp_create);

	for(i=0; i<mpdu_info->params.mac.num_tx_max ; i++){
		//Loop over retransmissions
		//Note: this loop will terminate early if retransmissions aren't needed
		//(i.e. ACK is received)

		// TODO
		//  * Make backoff slot selection on retransmissions less confusing
		//  * Insert backoff slot selections into low_tx_details
		//  * Set tx antenna mode based on phy param. This should be done
		//    after fixing antenna mode for ACK Tx to be a function of received antenna

		//Check if the higher-layer MAC requires this transmission have a post-Tx timeout
		req_timeout = ((mpdu_info->flags) & TX_MPDU_FLAGS_REQ_TO) != 0;

		//Write the SIGNAL field (interpreted by the PHY during Tx waveform generation)
		wlan_phy_set_tx_signal(pkt_buf, rate, length + WLAN_PHY_FCS_NBYTES);

		unsigned char mpdu_tx_ant_mask = 0;
		switch(mpdu_info->params.phy.antenna_mode) {
			case TX_ANTMODE_SISO_ANTA:
				mpdu_tx_ant_mask |= 0x1;
			break;
			case TX_ANTMODE_SISO_ANTB:
				mpdu_tx_ant_mask |= 0x2;
			break;
			case TX_ANTMODE_SISO_ANTC:
				mpdu_tx_ant_mask |= 0x4;
			break;
			case TX_ANTMODE_SISO_ANTD:
				mpdu_tx_ant_mask |= 0x8;
			break;
			default:
				mpdu_tx_ant_mask = 0x1;
			break;
		}

		if(i == 0){
			//This is the first transmission, so we speculatively draw a backoff in case
			//the backoff counter is currently 0 but the medium is busy. Prior to all other
			//(re)transmissions, an explicit backoff will have been started at the end of
			//the previous iteration of this loop.
			n_slots = rand_num_slots();
			wlan_mac_MPDU_tx_params(pkt_buf, n_slots, req_timeout, wlan_mac_low_dbm_to_gain_target(mpdu_info->params.phy.power), mpdu_tx_ant_mask);
		} else {
			REG_SET_BITS(WLAN_RX_DEBUG_GPIO,0x20);
			wlan_mac_MPDU_tx_params(pkt_buf, 0, req_timeout, wlan_mac_low_dbm_to_gain_target(mpdu_info->params.phy.power), mpdu_tx_ant_mask);
			REG_CLEAR_BITS(WLAN_RX_DEBUG_GPIO,0x20);
		}

		//Before we mess with any PHY state, we need to make sure it isn't actively
		//transmitting. For example, it may be sending an ACK when we get to this part of the code
		while(wlan_mac_get_status() & WLAN_MAC_STATUS_MASK_PHY_TX_ACTIVE){}

		//Submit the MPDU for transmission - this starts the MAC hardware's MPDU Tx state machine
		wlan_mac_MPDU_tx_start(1);
		wlan_mac_MPDU_tx_start(0);

#if 0
		!!! Something like this code will be used to determine the number of slots actually used
		!!! since the hardware will ignore the num_slots argument if medium is idle for DIFS
		tx_status = wlan_mac_get_status();
		if(tx_status & WLAN_MAC_STATUS_MASK_MPDU_TX_STATE == WLAN_MAC_STATUS_MPDU_TX_STATE_DEFER){
			low_tx_details[i].
		} else {

		}
#endif

		//Wait for the MPDU Tx to finish
		do{
			if(low_tx_details != NULL){
				low_tx_details[i].phy_params.rate = mpdu_info->params.phy.rate;
				low_tx_details[i].phy_params.power = mpdu_info->params.phy.power;
				low_tx_details[i].phy_params.antenna_mode = mpdu_info->params.phy.antenna_mode;
				low_tx_details[i].chan_num = wlan_mac_low_get_active_channel();
				low_tx_details[i].num_slots = n_slots; //TODO
				low_tx_details[i].cw = (1 << cw_exp)-1; //(2^(CW_EXP) - 1)
			}
			tx_status = wlan_mac_get_status();

			if(tx_status & WLAN_MAC_STATUS_MASK_MPDU_TX_DONE) {
				if(low_tx_details != NULL){
					low_tx_details[i].tx_start_delta = (u32)(get_tx_start_timestamp() - last_tx_timestamp);
					last_tx_timestamp = get_tx_start_timestamp();
				}
				switch(tx_status & WLAN_MAC_STATUS_MASK_MPDU_TX_RESULT){
					case WLAN_MAC_STATUS_MPDU_TX_RESULT_SUCCESS:
						//Tx didn't require timeout, completed successfully
						update_cw(DCF_CW_UPDATE_BCAST_TX, pkt_buf);
						n_slots = rand_num_slots();
						wlan_mac_dcf_hw_start_backoff(n_slots);
						return 0;
					break;
					case WLAN_MAC_STATUS_MPDU_TX_RESULT_RX_STARTED:
						expect_ack = 1;
						rx_status = wlan_mac_low_poll_frame_rx();
						if((rx_status & POLL_MAC_TYPE_ACK) && (rx_status & POLL_MAC_STATUS_GOOD) && (rx_status & POLL_MAC_ADDR_MATCH) && (rx_status & POLL_MAC_STATUS_RECEIVED_PKT) && expect_ack){
							update_cw(DCF_CW_UPDATE_MPDU_RX_ACK, pkt_buf);
							n_slots = rand_num_slots();
							wlan_mac_dcf_hw_start_backoff(n_slots);
							return 0;
						} else {
							if(update_cw(DCF_CW_UPDATE_MPDU_TX_ERR, pkt_buf)){
								n_slots = rand_num_slots();
								wlan_mac_dcf_hw_start_backoff(n_slots);
								break; //Transmission has failed. Halt loop
							} else{
								n_slots = rand_num_slots();
								wlan_mac_dcf_hw_start_backoff(n_slots);
								continue; //Begin retransmission.
							}
						}
					break;
					case WLAN_MAC_STATUS_MPDU_TX_RESULT_TIMED_OUT:
						//Tx required timeout, timeout expired with no receptions

						//Update the contention window
						if(update_cw(DCF_CW_UPDATE_MPDU_TX_ERR, pkt_buf)) {
							n_slots = rand_num_slots();
							wlan_mac_dcf_hw_start_backoff(n_slots);
							break; //Transmission has failed. Halt loop
						}

						//Start a random backoff interval using the updated CW
						n_slots = rand_num_slots();
						wlan_mac_dcf_hw_start_backoff(n_slots);

						//Re-submit the same MPDU for re-transmission (it will defer to the backoff started above)
						continue;

					break;

				}
			} else {
				if( (tx_status&WLAN_MAC_STATUS_MASK_PHY_RX_ACTIVE)){
					wlan_mac_low_poll_frame_rx();
				}
			}
		} while(tx_status & WLAN_MAC_STATUS_MASK_MPDU_TX_PENDING);



	} //end retransmission loop


	return -1;

}


inline int update_cw(u8 reason, u8 pkt_buf){
	u32* station_rc_ptr;
	u8* rc_ptr;
	u8 retry_limit;
	tx_frame_info* tx_mpdu = (tx_frame_info*)TX_PKT_BUF_TO_ADDR(pkt_buf);

	mac_header_80211* tx_80211_header;

	tx_80211_header = (mac_header_80211*)((void*)(TX_PKT_BUF_TO_ADDR(pkt_buf)+PHY_TX_PKT_BUF_MPDU_OFFSET));

	rc_ptr = &(tx_mpdu->num_tx);

	if(tx_mpdu->length > RTS_THRESHOLD){
		station_rc_ptr = &stationLongRetryCount;
	} else {
		station_rc_ptr = &stationShortRetryCount;
	}

	retry_limit = tx_mpdu->params.mac.num_tx_max;

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
			(*rc_ptr)++;
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
// |	7		|	[0, 123]	|
// |	8		|	[0, 255]	|
// |	9		|	[0, 511]	|
// |	10		|	[0, 1023]	|
	volatile u32 n_slots;

	n_slots = ((unsigned int)rand() >> (32-(cw_exp+1)));

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




int wlan_create_ack_frame(void* pkt_buf, u8* address_ra) {

	mac_header_80211_ACK* ack_header;
	ack_header = (mac_header_80211_ACK*)(pkt_buf);

	ack_header->frame_control_1 = MAC_FRAME_CTRL1_SUBTYPE_ACK;
	ack_header->frame_control_2 = 0;
	ack_header->duration_id = 0;
	memcpy(ack_header->address_ra, address_ra, 6);

	return sizeof(mac_header_80211_ACK);
}
