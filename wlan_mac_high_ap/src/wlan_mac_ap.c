/** @file wlan_mac_ap.c
 *  @brief Access Point
 *
 *  This contains code for the 802.11 Access Point.
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

//Xilinx SDK includes
#include "stdio.h"
#include "stdlib.h"
#include "xtmrctr.h"
#include "xio.h"
#include "string.h"
#include "xintc.h"
#include "xparameters.h"

//802.11 ref design includes
#include "wlan_mac_addr_filter.h"
#include "wlan_mac_ipc_util.h"
#include "wlan_mac_misc_util.h"
#include "wlan_mac_802_11_defs.h"
#include "wlan_mac_queue.h"
#include "wlan_mac_ltg.h"
#include "wlan_mac_high.h"
#include "wlan_mac_packet_types.h"
#include "wlan_mac_eth_util.h"
#include "wlan_mac_event_log.h"
#include "wlan_mac_entries.h"
#include "wlan_mac_ap.h"
#include "wlan_mac_schedule.h"
#include "wlan_mac_dl_list.h"
#include "ascii_characters.h"

// Experiments framework includes
#include "wlan_exp.h"
#include "wlan_exp_common.h"
#include "wlan_exp_node.h"
#include "wlan_exp_node_ap.h"
#include "wlan_exp_transport.h"


/*************************** Constant Definitions ****************************/
#define  WLAN_EXP_ETH                  WN_ETH_B
#define  WLAN_EXP_NODE_TYPE            (WARPNET_TYPE_80211_BASE + WARPNET_TYPE_80211_HIGH_AP)

#define  WLAN_DEFAULT_CHANNEL          1
#define  WLAN_DEFAULT_TX_PWR		   10


/*************************** Variable Definitions ****************************/

// SSID variables
static char default_AP_SSID[] = "WARP-AP";
char*       access_point_ssid;

// Common TX header for 802.11 packets
mac_header_80211_common tx_header_common;

// Control variables
u8 allow_assoc;
u8 perma_assoc_mode;

// Default Transmission Parameters
tx_params default_unicast_mgmt_tx_params;
tx_params default_unicast_data_tx_params;
tx_params default_multicast_mgmt_tx_params;
tx_params default_multicast_data_tx_params;

// Lists to hold association table and Tx/Rx statistics
dl_list		 association_table;
dl_list		 statistics_table;

// Tx queue variables;
u32			 max_queue_size;

// AP channel
u32 		 mac_param_chan;

// MAC address
static u8 wlan_mac_addr[6];

u8 tim_bitmap[1] = {0x0};
u8 tim_control = 1;

/******************************** Functions **********************************/

int main(){

	xil_printf("\f");
	xil_printf("----- Mango 802.11 Reference Design -----\n");
	xil_printf("----- v0.9 Beta -------------------------\n");
	xil_printf("----- wlan_mac_ap -----------------------\n");

	xil_printf("Compiled %s %s\n\n", __DATE__, __TIME__);

	//heap_init() must be executed before any use of malloc. This explicit init
	// handles the case of soft-reset of the MicroBlaze leaving stale values in the heap RAM
	wlan_mac_high_heap_init();

	//Initialize the MAC framework
	wlan_mac_high_init();

    // Set Global variables
	perma_assoc_mode     = 0;


	//Define the default PHY and MAC params for all transmissions

	//New associations adopt these unicast params; the per-node params can be
	// overridden via wlan_exp calls or by custom C code
	default_unicast_data_tx_params.mac.num_tx_max = MAX_NUM_TX;
	default_unicast_data_tx_params.phy.power = WLAN_DEFAULT_TX_PWR;
	default_unicast_data_tx_params.phy.rate = WLAN_MAC_RATE_18M;
	default_unicast_data_tx_params.phy.antenna_mode = TX_ANTMODE_SISO_ANTA;

	default_unicast_mgmt_tx_params.mac.num_tx_max = MAX_NUM_TX;
	default_unicast_mgmt_tx_params.phy.power = WLAN_DEFAULT_TX_PWR;
	default_unicast_mgmt_tx_params.phy.rate = WLAN_MAC_RATE_6M;
	default_unicast_mgmt_tx_params.phy.antenna_mode = TX_ANTMODE_SISO_ANTA;

	//All multicast traffic (incl. broadcast) uses these default Tx params
	default_multicast_data_tx_params.mac.num_tx_max = 1;
	default_multicast_data_tx_params.phy.power = WLAN_DEFAULT_TX_PWR;
	default_multicast_data_tx_params.phy.rate = WLAN_MAC_RATE_18M;
	default_multicast_data_tx_params.phy.antenna_mode = TX_ANTMODE_SISO_ANTA;

	default_multicast_mgmt_tx_params.mac.num_tx_max = 1;
	default_multicast_mgmt_tx_params.phy.power = WLAN_DEFAULT_TX_PWR;
	default_multicast_mgmt_tx_params.phy.rate = WLAN_MAC_RATE_6M;
	default_multicast_mgmt_tx_params.phy.antenna_mode = TX_ANTMODE_SISO_ANTA;


#ifdef USE_WARPNET_WLAN_EXP
	//Configure and initialize the wlan_exp framework
	wlan_exp_configure(WLAN_EXP_NODE_TYPE, WLAN_EXP_ETH);
#endif

	//Setup the association table and stats lists
	dl_list_init(&association_table);
	dl_list_init(&statistics_table);

	//Calculate the maximum length of any Tx queue
	// (queue_total_size()- eth_get_num_rx_bd()) is the number of queue entries available after dedicating some to the ETH DMA
	// MAX_PER_FLOW_QUEUE is the absolute max length of any queue; long queues (a.k.a. buffer bloat) are bad
	max_queue_size = min((queue_total_size()- eth_get_num_rx_bd()) / (association_table.length+1), MAX_TX_QUEUE_LEN);

	// Initialize callbacks
	wlan_mac_util_set_eth_rx_callback(       (void*)ethernet_receive);
	wlan_mac_high_set_mpdu_tx_done_callback( (void*)mpdu_transmit_done);
	wlan_mac_high_set_mpdu_rx_callback(      (void*)mpdu_rx_process);
	wlan_mac_high_set_pb_u_callback(         (void*)up_button);

	wlan_mac_high_set_uart_rx_callback(      (void*)uart_rx);
	wlan_mac_high_set_mpdu_accept_callback(  (void*)poll_tx_queues);
    wlan_mac_ltg_sched_set_callback(         (void*)ltg_event);

    // Configure the wireless-wired encapsulation mode (AP and STA behaviors are different)
    wlan_mac_util_set_eth_encap_mode(ENCAP_MODE_AP);

    // Wait for CPU Low to initialize
	while( wlan_mac_high_is_cpu_low_initialized() == 0 ){
		xil_printf("waiting on CPU_LOW to boot\n");
	}

	// The node's MAC address is stored in the EEPROM, accessible only to CPU Low
	// CPU Low provides this to CPU High after it boots
	memcpy((void*) &(wlan_mac_addr[0]), (void*) wlan_mac_high_get_eeprom_mac_addr(), 6);

    // Set Header information
	tx_header_common.address_2 = &(wlan_mac_addr[0]);
	tx_header_common.seq_num   = 0;

    // Initialize hex display
	wlan_mac_high_write_hex_display(0);

	// Configure default radio and PHY params via messages to CPU Low
	mac_param_chan = WLAN_DEFAULT_CHANNEL;
	wlan_mac_high_set_channel( mac_param_chan );
	wlan_mac_high_set_rx_ant_mode(RX_ANTMODE_SISO_ANTA);
	wlan_mac_high_set_tx_ctrl_pow(WLAN_DEFAULT_TX_PWR);

	// Configure CPU Low's filter for passing Rx packets up to CPU High
	//  Default is "promiscuous" mode - pass all data and management packets with good or bad checksums
	//   This allows logging of all data/management receptions, even if they're not intended for this node
	wlan_mac_high_set_rx_filter_mode( (RX_FILTER_FCS_ALL | RX_FILTER_HDR_ALL_MPDU) );

	// Set SSID
	access_point_ssid = wlan_mac_high_malloc(strlen(default_AP_SSID)+1);
	if(access_point_ssid != NULL) {
		strcpy(access_point_ssid, default_AP_SSID);
	} else {
		xil_printf("ERROR: Unable to set SSID!\n");
	}

	// Initialize interrupts
	wlan_mac_high_interrupt_init();

    // Setup default scheduled events:
	//  Periodic beacon transmissions
	wlan_mac_schedule_event_repeated(SCHEDULE_COARSE, BEACON_INTERVAL_US, SCHEDULE_REPEAT_FOREVER, (void*)beacon_transmit);

	//  Periodic check for timed-out associations
	wlan_mac_schedule_event_repeated(SCHEDULE_COARSE, ASSOCIATION_CHECK_INTERVAL_US, SCHEDULE_REPEAT_FOREVER, (void*)association_timestamp_check);

	//  Periodic blinking of hex display leds (to indicate new associations are allowed)
	wlan_mac_high_enable_hex_dot_blink();

	// By default accept new associations forever
	enable_associations( ASSOCIATION_ALLOW_PERMANENT );

	// Reset the event log
	event_log_reset();


#if 0
	/////// TODO DEBUG  WRITE EXAMPLE ///////
	u32 	idx_write;
	u32*	payload_write;
	#define NUM_WORDS_TO_WRITE 5

	ipc_reg_read_write* write_example = wlan_mac_high_malloc(sizeof(ipc_reg_read_write)+(NUM_WORDS_TO_WRITE*sizeof(u32)));

	//Base address in CPU_LOW's memory to write to
	write_example->baseaddr = 0x12345678;

	//Number of words to write to CPU_LOW's memory
	write_example->num_words = NUM_WORDS_TO_WRITE;

	//Payload to be written immediately follows the ipc_reg_read_write
	payload_write = (u32*)((u8*)write_example + sizeof(ipc_reg_read_write));

	//Construct payload to be written
	for(idx_write = 0; idx_write < NUM_WORDS_TO_WRITE; idx_write++){
		payload_write[idx_write] = idx_write;
	}

	//Tell the framework to send write_example. Note: the first argument of this function is the number of words
	//to write over IPC, which is NUM_WORDS_TO_WRITE plus the number of words needed for the ipc_reg_read_write header
	wlan_mac_high_write_low_mem(NUM_WORDS_TO_WRITE + (sizeof(ipc_reg_read_write)/sizeof(u32)) , (u32*)write_example);

	wlan_mac_high_free(write_example);
	/////// TODO DEBUG  WRITE EXAMPLE ///////
#endif

	// Print AP information to the terminal
    xil_printf("WLAN MAC AP boot complete: \n");
    xil_printf("  SSID    : %s \n", access_point_ssid);
    xil_printf("  Channel : %d \n", mac_param_chan);
	xil_printf("  MAC Addr: %02x-%02x-%02x-%02x-%02x-%02x\n\n",wlan_mac_addr[0],wlan_mac_addr[1],wlan_mac_addr[2],wlan_mac_addr[3],wlan_mac_addr[4],wlan_mac_addr[5]);

#ifdef WLAN_USE_UART_MENU
	xil_printf("\nAt any time, press the Esc key in your terminal to access the AP menu\n");
#endif

#ifdef USE_WARPNET_WLAN_EXP
	// Set AP processing callbacks
	node_set_process_callback( (void *)wlan_exp_node_ap_processCmd );
#endif

	// Finally enable all interrupts to start handling wireless and wired traffic
	wlan_mac_high_interrupt_start();

#if 0
	/////// TODO DEBUG  READ EXAMPLE ///////
	//wlan_mac_high_interrupt_stop();
	u32 	idx_read;
	u32*	payload_read;
	#define NUM_WORDS_TO_READ 5

	payload_read = wlan_mac_high_malloc(NUM_WORDS_TO_READ * sizeof(u32));

	//Read NUM_WORDS_TO_READ words from 0x12345678 in CPU_LOW's memory space
	wlan_mac_high_read_low_mem(NUM_WORDS_TO_READ, 0x12345678, payload_read);

	for(idx_read = 0; idx_read < NUM_WORDS_TO_READ; idx_read++){
		xil_printf("[%d] = 0x%08x\n",idx_read, payload_read[idx_read]);
	}

	wlan_mac_high_free(payload_read);

	//wlan_mac_high_interrupt_start();
	/////// TODO DEBUG  READ EXAMPLE ///////
#endif

	while(1) {
#ifdef USE_WARPNET_WLAN_EXP
		//The wlan_exp Ethernet handling is not interrupt based. Periodic polls of the wlan_exp
		// transport are required to service new commands. All other node activity (wired/wireless Tx/Rx,
		//  scheduled events, user interaction, etc) are handled via interrupt service routines
		transport_poll( WLAN_EXP_ETH );
#endif

	}

	//Unreachable, but non-void return keeps the compiler happy
	return 0;
}


#ifdef _DEBUG_

void print_buf(unsigned char *buf, int size)
{
	int i;
	for (i=0; i<size; i++) {
        xil_printf("%2x ", buf[i]);
        if ( (((i + 1) % 16) == 0) && ((i + 1) != size) ) {
            xil_printf("\n");
        }
	}
	xil_printf("\n\n");
}

#endif


/**
 * @brief Poll Tx queues to select next available packet to transmit
 *
 * This function is called whenever the upper MAC is ready to send a new packet
 * to the lower MAC for transmission. The next packet to transmit is selected
 * from one of the currently-enabled Tx queues.
 *
 * The reference implementation uses a simple queue prioritization scheme:
 *  - Two queue groups are defined: Management (MGMT_QGRP) and Data (DATA_QGRP)
 *   - The Management group contains one queue for all management traffic
 *   - The Data group contains one queue for multicast data plus one queue per associated STA
 *  - The code alternates its polling between queue groups
 *  - In each group queues are polled via round robin
 *
 *  This scheme gives priority to management transmissions to help avoid timeouts during
 *  association handshakes and treats each associated STA with equal priority.
 *
 * This function uses the framework function dequeue_transmit_checkin() to check individual queues
 * If dequeue_transmit_checkin() is passed a not-empty queue, it will dequeue and transmit a packet, then
 *  return a non-zero status. Thus the calls below terminate polling as soon as any call to dequeue_transmit_checkin()
 *  returns with a non-zero value, allowing the next call to poll_tx_queues() to continue the queue polling process.
 *
 * @param None
 * @return None
 */
void poll_tx_queues(){
	u32 i,k;

	#define NUM_QUEUE_GROUPS 2
	typedef enum {MGMT_QGRP, DATA_QGRP} queue_group_t;

	//Remember the next group to poll between calls to this function
	// This implements the ping-pong poll between the MGMT_QGRP and DATA_QGRP groups
	static queue_group_t next_queue_group = MGMT_QGRP;
	queue_group_t curr_queue_group;

	//Remember the last queue polled between calls to this function
	// This implements the round-robin poll of queues in the DATA_QGRP group
	static dl_entry* next_station_info_entry = NULL;
	dl_entry* curr_station_info_entry;

	station_info* curr_station_info;

	if( wlan_mac_high_is_ready_for_tx() ){

		for(k = 0; k < NUM_QUEUE_GROUPS; k++){

			curr_queue_group = next_queue_group;

			switch(curr_queue_group){
				case MGMT_QGRP:
					next_queue_group = DATA_QGRP;
					if(dequeue_transmit_checkin(MANAGEMENT_QID)){
						return;
					}
				break;
				case DATA_QGRP:
					next_queue_group = MGMT_QGRP;
					curr_station_info_entry = next_station_info_entry;

						for(i = 0; i < (association_table.length + 1); i++) {
							//Loop through all associated stations' queues and the broadcast queue
							if(curr_station_info_entry == NULL){
								//Check the broadcast queue
								next_station_info_entry = association_table.first;
								if(dequeue_transmit_checkin(MCAST_QID)){
									//Found a not-empty queue, transmitted a packet
									return;
								} else {
									curr_station_info_entry = next_station_info_entry;
								}
							} else {
								curr_station_info = (station_info*)(curr_station_info_entry->data);
								if( wlan_mac_high_is_valid_association(&association_table, curr_station_info) ){
									if(curr_station_info_entry == association_table.last){
										//We've reached the end of the table, so we wrap around to the beginning
										next_station_info_entry = NULL;
									} else {
										next_station_info_entry = dl_entry_next(curr_station_info_entry);
									}

									if(dequeue_transmit_checkin(AID_TO_QID(curr_station_info->AID))){
										//Found a not-empty queue, transmitted a packet
										return;
									} else {
										curr_station_info_entry = next_station_info_entry;
									}
								} else {
									//This curr_station_info is invalid. Perhaps it was removed from
									//the association table before poll_tx_queues was called. We will
									//start the round robin checking back at broadcast.
									next_station_info_entry = NULL;
									return;
								} //END if(is_valid_association)
							}
						} //END for loop over association table
				break;
			} //END switch(queue group)
		} //END loop over queue groups
	} //END CPU low is ready

	return;
}

/**
 * @brief Purges all packets from all Tx queues
 *
 * This function discards all currently en-queued packets awaiting transmission and returns all
 * queue entries to the free pool.
 *
 * This function does not discard packets already submitted to the lower-level MAC for transmission
 *
 * @param None
 * @return None
 */
void purge_all_data_tx_queue(){
	u32 i;
	dl_entry*	  curr_station_info_entry;
	station_info* curr_station_info;

	// Purge all data transmit queues
	purge_queue(MCAST_QID);                                    		// Broadcast Queue
	curr_station_info_entry = association_table.first;

	for(i=0; i < association_table.length; i++){
		curr_station_info = (station_info*)(curr_station_info_entry->data);
		purge_queue(AID_TO_QID(curr_station_info->AID));       		// Each unicast queue
		curr_station_info_entry = dl_entry_next(curr_station_info_entry);
	}
}

/**
 * @brief Callback to handle a packet after it was transmitted by the lower-level MAC
 *
 * This function is called when CPU Low indicates it has completed the Tx process for a packet previously
 * submitted by CPU High.
 *
 * CPU High has two responsibilities post-Tx:
 *  - Cleanup any resources dedicated to the packet
 *  - Update any statistics and log info to reflect the Tx result
 *
 * @param tx_frame_info* tx_mpdu
 *  - Pointer to the MPDU which was just transmitted
 * @param wlan_mac_low_tx_details* tx_low_details
 *  - Pointer to the array of data recorded by the lower-level MAC about each re-transmission of the MPDU
 * @param u16 num_tx_low_details
 *  - number of elements in array pointed to by previous argument
 * @return None
*/
void mpdu_transmit_done(tx_frame_info* tx_mpdu, wlan_mac_low_tx_details* tx_low_details, u16 num_tx_low_details) {
	u32 i;
	tx_high_entry* tx_high_event_log_entry;
	tx_low_entry*  tx_low_event_log_entry;
	station_info* station;
	dl_entry*	  entry;
	u8 			  pkt_type;

	frame_statistics_txrx* frame_stats = NULL;

	//Get a pointer to the MPDU payload in the packet buffer
	void* mpdu = (u8*)tx_mpdu + PHY_TX_PKT_BUF_MPDU_OFFSET;
	u8* mpdu_ptr_u8 = (u8*)mpdu;

	//Get a pointer to the MAC header in the MPDU
	mac_header_80211* tx_80211_header;
	tx_80211_header = (mac_header_80211*)((void *)mpdu_ptr_u8);

	u64 ts_old = 0;
	u32 payload_log_len;
	u32 extra_payload;
	u32 transfer_len;
	u32 total_payload_len = tx_mpdu->length;

	pkt_type = wlan_mac_high_pkt_type(mpdu, tx_mpdu->length);

	for(i = 0; i < num_tx_low_details; i++) {

		//Request space for a TX_LOW log entry
		tx_low_event_log_entry = (tx_low_entry *)wlan_exp_log_create_entry( ENTRY_TYPE_TX_LOW, sizeof(tx_low_entry) );

		if(tx_low_event_log_entry != NULL){
			//TX_LOW entries only store the MAC header - the full payload may be included in the associated TX entry
			transfer_len = sizeof(mac_header_80211);

			tx_low_event_log_entry->mac_payload_log_len = transfer_len;
			wlan_mac_high_cdma_start_transfer((&((tx_low_entry*)tx_low_event_log_entry)->mac_payload), tx_80211_header, transfer_len);

			// Zero pad log entry if transfer_len was less than the allocated space in the log (ie MIN_MAC_PAYLOAD_LOG_LEN)
			if(transfer_len < MIN_MAC_PAYLOAD_LOG_LEN){
				bzero((u8*)(((u32)((tx_low_entry*)tx_low_event_log_entry)->mac_payload) + transfer_len), (MIN_MAC_PAYLOAD_LOG_LEN - transfer_len));
			}


			if((i+1 == (tx_mpdu->num_tx)) && (tx_mpdu->tx_result == TX_MPDU_RESULT_SUCCESS)){
				tx_low_event_log_entry->flags = TX_LOW_FLAGS_WAS_ACKED;
			} else {
				tx_low_event_log_entry->flags = 0;
			}





			//Compute the timestamp of the actual Tx event
			// CPU low accumulates time deltas relative to original enqueue time (easier to store u32 deltas vs u64 times)
			tx_low_event_log_entry->timestamp_send            = (u64)(  tx_mpdu->timestamp_create + (u64)(tx_mpdu->delay_accept) + (u64)(tx_low_details[i].tx_start_delta) + (u64)ts_old);

			tx_low_event_log_entry->unique_seq				  = tx_mpdu->unique_seq;
			tx_low_event_log_entry->transmission_count        = i+1;
			tx_low_event_log_entry->chan_num                  = tx_low_details[i].chan_num;
			tx_low_event_log_entry->num_slots				  = tx_low_details[i].num_slots;
			tx_low_event_log_entry->cw						  = tx_low_details[i].cw;
			memcpy((&((tx_low_entry*)tx_low_event_log_entry)->phy_params), &(tx_low_details[i].phy_params), sizeof(phy_tx_params));
			tx_low_event_log_entry->length                    = tx_mpdu->length;
			tx_low_event_log_entry->pkt_type				  = wlan_mac_high_pkt_type(mpdu, tx_mpdu->length);
			wlan_mac_high_cdma_finish_transfer();

			//CPU Low updates the retry flag in the header for any re-transmissions
			// Re-create the original header for the first TX_LOW by de-asserting the flag
			if(i==0) {
				//This is the first transmission
				((mac_header_80211*)(tx_low_event_log_entry->mac_payload))->frame_control_2 &= ~MAC_FRAME_CTRL2_FLAG_RETRY;
			} else {
				//This is all subsequent transmissions
				((mac_header_80211*)(tx_low_event_log_entry->mac_payload))->frame_control_2 |= MAC_FRAME_CTRL2_FLAG_RETRY;
			}

#ifdef _DEBUG_
			xil_printf("TX LOW  : %8d    %8d    \n", transfer_len, MIN_MAC_PAYLOAD_LOG_LEN);
	        print_buf((u8 *)((u32)tx_low_event_log_entry - 8), sizeof(tx_low_entry) + 12);
#endif
		}

		//Accumulate the time-between-transmissions, used to calculate absolute time of each TX_LOW event above
		ts_old += tx_low_details[i].tx_start_delta;
	}//END loop over TX_LOW entries

	// Payloads in log must be at least MIN_MAC_PAYLOAD_LOG_LEN bytes and always multiple of 4 bytes (integral number of u32 words)
	payload_log_len = min( max((1 + ( ( ( total_payload_len ) - 1) / 4) )*4 , MIN_MAC_PAYLOAD_LOG_LEN) , mac_payload_log_len );
	extra_payload   = (payload_log_len > MIN_MAC_PAYLOAD_LOG_LEN) ? (payload_log_len - MIN_MAC_PAYLOAD_LOG_LEN) : 0;

	//Request space for a TX entry
	tx_high_event_log_entry = (tx_high_entry *)wlan_exp_log_create_entry( ENTRY_TYPE_TX_HIGH, sizeof(tx_high_entry) + extra_payload );

	if(tx_high_event_log_entry != NULL){
		//Fill in the TX log entry
		// This is done one field at a time, as the TX log entry format is not a byte-for-byte copy of the tx_frame_info
		tx_high_event_log_entry->mac_payload_log_len = total_payload_len;

		// Have to compute the correct number of bytes to transfer so that we don't walk off the end of either
		// the log entry allocation or the packet buffer.
        if (total_payload_len < MIN_MAC_PAYLOAD_LOG_LEN) {
    		transfer_len = total_payload_len;
        } else {
    		transfer_len = MIN_MAC_PAYLOAD_LOG_LEN + min(extra_payload, (total_payload_len - MIN_MAC_PAYLOAD_LOG_LEN));
        }

		wlan_mac_high_cdma_start_transfer((&((tx_high_entry*)tx_high_event_log_entry)->mac_payload), tx_80211_header, transfer_len);

		// Zero pad log entry if transfer_len was less than the allocated space in the log (ie MIN_MAC_PAYLOAD_LOG_LEN + extra_payload)
		if(transfer_len < (MIN_MAC_PAYLOAD_LOG_LEN + extra_payload)){
			bzero((u8*)(((u32)((tx_high_entry*)tx_high_event_log_entry)->mac_payload) + transfer_len), ((MIN_MAC_PAYLOAD_LOG_LEN + extra_payload) - transfer_len) );
		}

		bzero(tx_high_event_log_entry->padding , sizeof(tx_high_event_log_entry->padding));
		tx_high_event_log_entry->unique_seq				  = tx_mpdu->unique_seq;
		tx_high_event_log_entry->queue_id				  = tx_mpdu->QID;
		tx_high_event_log_entry->result                   = tx_mpdu->tx_result;
		tx_high_event_log_entry->power              	  = tx_mpdu->params.phy.power;
		tx_high_event_log_entry->length                   = tx_mpdu->length;
		tx_high_event_log_entry->rate                     = tx_mpdu->params.phy.rate;
		tx_high_event_log_entry->chan_num				  = mac_param_chan;
		tx_high_event_log_entry->pkt_type				  = pkt_type;
		tx_high_event_log_entry->num_tx                   = tx_mpdu->num_tx;
		tx_high_event_log_entry->timestamp_create         = tx_mpdu->timestamp_create;
		tx_high_event_log_entry->delay_accept             = tx_mpdu->delay_accept;
		tx_high_event_log_entry->delay_done               = tx_mpdu->delay_done;
		tx_high_event_log_entry->ant_mode				  = tx_mpdu->params.phy.antenna_mode;

#ifdef _DEBUG_
		xil_printf("TX HIGH : %8d    %8d    %8d    %8d    %8d\n", transfer_len, MIN_MAC_PAYLOAD_LOG_LEN, total_payload_len, extra_payload, payload_log_len);
        print_buf((u8 *)((u32)tx_high_event_log_entry - 8), sizeof(tx_high_entry) + extra_payload + 12);
#endif
	}

	//Update the statistics for the node to which the packet was just transmitted
	if(tx_mpdu->AID != 0) {
		entry = wlan_mac_high_find_station_info_AID(&association_table, tx_mpdu->AID);

		if(entry != NULL){
			station = (station_info*)(entry->data);

			switch(pkt_type){
				case PKT_TYPE_DATA_ENCAP_ETH:
				case PKT_TYPE_DATA_ENCAP_LTG:
					frame_stats = &(station->stats->data);
				break;

				case PKT_TYPE_MGMT:
					frame_stats = &(station->stats->mgmt);
				break;
			}

			//Update Transmission Stats
			if(frame_stats != NULL){

				(frame_stats->tx_num_packets_total)++;
				(frame_stats->tx_num_bytes_total) += tx_mpdu->length;

				(frame_stats->tx_num_packets_low) += (tx_mpdu->num_tx);

				if((tx_mpdu->tx_result) == TX_MPDU_RESULT_SUCCESS){
					(frame_stats->tx_num_packets_success)++;
					(frame_stats->tx_num_bytes_success) += tx_mpdu->length;
				}

			}

		}
	} //END if valid AID

	//Send log entry to wlan_exp controller immediately (not currently supported)
	/*
	if (tx_high_event_log_entry != NULL) {
        wn_transmit_log_entry((void *)tx_high_event_log_entry);
	}
	 */

	return;
}

/**
 * @brief Callback to handle push of up button
 *
 * Reference implementation uses up button to set current association mode, cycling through:
 *  - ASSOCIATION_ALLOW_NONE: allow no associations
 *  - ASSOCIATION_ALLOW_TEMPORARY: accept new associations for fixed interval
 *  - ASSOCIATION_ALLOW_PERMANENT: accept new associations forever
*/
void up_button(){

	u32 curr_assoc_mode = get_associations_status();

	switch ( curr_assoc_mode ) {

        case ASSOCIATION_ALLOW_NONE:
        	// Update state to allow associations temporarily
        	wlan_mac_high_enable_hex_dot_blink();
    		enable_associations( ASSOCIATION_ALLOW_TEMPORARY );
    		wlan_mac_schedule_event(SCHEDULE_COARSE,ASSOCIATION_ALLOW_INTERVAL_US, (void*)disable_associations);
        break;

        case ASSOCIATION_ALLOW_TEMPORARY:
        	// Update state to allow associations forever
    		enable_associations( ASSOCIATION_ALLOW_PERMANENT );
    		xil_printf("Allowing associations indefinitely\n");
        break;

        case ASSOCIATION_ALLOW_PERMANENT:
        	// Update state to disallow associations
    		enable_associations( ASSOCIATION_ALLOW_TEMPORARY );
    		disable_associations();
        break;
	}


	return;
}

/**
 * @brief Callback to handle new Local Traffic Generator event
 *
 * This function is called when the LTG scheduler determines a traffic generator should create a new packet. The
 * behavior of this function depends entirely on the LTG payload parameters.
 *
 * The reference implementation defines 3 LTG payload types:
 *  - LTG_PYLD_TYPE_FIXED: generate 1 fixed-length packet to single destination; callback_arg is pointer to ltg_pyld_fixed struct
 *  - LTG_PYLD_TYPE_UNIFORM_RAND: generate 1 random-length packet to signle destimation; callback_arg is pointer to ltg_pyld_uniform_rand struct
 *  - LTG_PYLD_TYPE_ALL_ASSOC_FIXED: generate 1 fixed-length packet to each associated station; callback_arg is poitner to ltg_pyld_all_assoc_fixed struct
 *
 * @param u32 id
 *  - Unique ID of the previously created LTG
 * @param void* callback_arg
 *  - Callback argument provided at LTG creation time; interpretation depends on LTG type
 * @return None
*/
void ltg_event(u32 id, void* callback_arg){
	tx_queue_element* curr_tx_queue_element;
	tx_queue_buffer* curr_tx_queue_buffer;
	u32 tx_length;
	u32 payload_length = 0;
	u8* mpdu_ptr_u8;
	llc_header* llc_hdr;
	dl_entry*	  station_info_entry;
	station_info* station;
	u8* addr_da;
	u8 is_multicast;

	switch(((ltg_pyld_hdr*)callback_arg)->type){
		case LTG_PYLD_TYPE_FIXED:
			addr_da = ((ltg_pyld_fixed*)callback_arg)->addr_da;
			is_multicast = wlan_addr_mcast(addr_da);
			payload_length = ((ltg_pyld_fixed*)callback_arg)->length;
			station_info_entry = wlan_mac_high_find_station_info_ADDR(&association_table, addr_da);
		break;
		case LTG_PYLD_TYPE_UNIFORM_RAND:
			addr_da = ((ltg_pyld_uniform_rand*)callback_arg)->addr_da;
			is_multicast = wlan_addr_mcast(addr_da);
			payload_length = (rand()%(((ltg_pyld_uniform_rand*)(callback_arg))->max_length - ((ltg_pyld_uniform_rand*)(callback_arg))->min_length))+((ltg_pyld_uniform_rand*)(callback_arg))->min_length;
			station_info_entry = wlan_mac_high_find_station_info_ADDR(&association_table, addr_da);
		break;
		case LTG_PYLD_TYPE_ALL_ASSOC_FIXED:
			payload_length = ((ltg_pyld_all_assoc_fixed*)callback_arg)->length;
			is_multicast = 0;
			station_info_entry = association_table.first;
		break;
		default:
			xil_printf("ERROR ltg_event: Unknown LTG Payload Type! (%d)\n", ((ltg_pyld_hdr*)callback_arg)->type);
			addr_da = 0;
			return;
		break;
	}

	if(is_multicast){
		if(queue_num_queued(MCAST_QID) < max_queue_size) {
			//Send a Data packet to this station
			//Checkout 1 element from the queue;
			curr_tx_queue_element = queue_checkout();

			if(curr_tx_queue_element != NULL){ //There was at least 1 free queue element

				curr_tx_queue_buffer = ((tx_queue_buffer*)(curr_tx_queue_element->data));

				wlan_mac_high_setup_tx_header( &tx_header_common, addr_da, wlan_mac_addr );

				mpdu_ptr_u8 = (u8*)(curr_tx_queue_buffer->frame);
				tx_length = wlan_create_data_frame((void*)mpdu_ptr_u8, &tx_header_common, MAC_FRAME_CTRL2_FLAG_FROM_DS);

				mpdu_ptr_u8 += sizeof(mac_header_80211);
				llc_hdr = (llc_header*)(mpdu_ptr_u8);

				//Prepare the MPDU LLC header
				llc_hdr->dsap = LLC_SNAP;
				llc_hdr->ssap = LLC_SNAP;
				llc_hdr->control_field = LLC_CNTRL_UNNUMBERED;
				bzero((void *)(llc_hdr->org_code), 3); //Org Code 0x000000: Encapsulated Ethernet
				llc_hdr->type = LLC_TYPE_WLAN_LTG;

				//LTG packets always have LLC header, plus any extra payload requested by user
				tx_length += max(payload_length, sizeof(llc_header));

				//Finally prepare the 802.11 header
				wlan_mac_high_setup_tx_frame_info ( &tx_header_common, curr_tx_queue_element, tx_length, (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO), MCAST_QID);

				//Update the queue entry metadata to reflect the new new queue entry contents
				curr_tx_queue_buffer->metadata.metadata_type = QUEUE_METADATA_TYPE_TX_PARAMS;
				curr_tx_queue_buffer->metadata.metadata_ptr = (u32)(&default_multicast_data_tx_params);

				//Submit the new packet to the appropriate queue
				enqueue_after_tail(MCAST_QID, curr_tx_queue_element);

				//Poll all Tx queues, in case the just-submitted packet can be de-queueued and transmitted immediately
				poll_tx_queues();
			}
		} //END successful queue checkout
	} else {
		//Iterate over destination STAs
		// Single-packet LTGs will execute this loop once


		while(station_info_entry != NULL) {

			station = (station_info*)(station_info_entry->data);

			if(queue_num_queued(AID_TO_QID(station->AID)) < max_queue_size) {
				//Send a Data packet to this station
				//Checkout 1 element from the queue;
				curr_tx_queue_element = queue_checkout();

				if(curr_tx_queue_element != NULL){ //There was at least 1 free queue element

					curr_tx_queue_buffer = ((tx_queue_buffer*)(curr_tx_queue_element->data));

					wlan_mac_high_setup_tx_header( &tx_header_common, station->addr, wlan_mac_addr );

					mpdu_ptr_u8 = (u8*)(curr_tx_queue_buffer->frame);
					tx_length = wlan_create_data_frame((void*)mpdu_ptr_u8, &tx_header_common, MAC_FRAME_CTRL2_FLAG_FROM_DS);

					mpdu_ptr_u8 += sizeof(mac_header_80211);
					llc_hdr = (llc_header*)(mpdu_ptr_u8);

					//Prepare the MPDU LLC header
					llc_hdr->dsap = LLC_SNAP;
					llc_hdr->ssap = LLC_SNAP;
					llc_hdr->control_field = LLC_CNTRL_UNNUMBERED;
					bzero((void *)(llc_hdr->org_code), 3); //Org Code 0x000000: Encapsulated Ethernet
					llc_hdr->type = LLC_TYPE_WLAN_LTG;

					//LTG packets always have LLC header, plus any extra payload requested by user
					tx_length += max(payload_length, sizeof(llc_header));

					//Finally prepare the 802.11 header
					wlan_mac_high_setup_tx_frame_info ( &tx_header_common, curr_tx_queue_element, tx_length, (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO), AID_TO_QID(station->AID));

					//Update the queue entry metadata to reflect the new new queue entry contents
					curr_tx_queue_buffer->metadata.metadata_type = QUEUE_METADATA_TYPE_STATION_INFO;
					curr_tx_queue_buffer->metadata.metadata_ptr = (u32)station;
					curr_tx_queue_buffer->frame_info.AID = station->AID;

					//Submit the new packet to the appropriate queue
					enqueue_after_tail(AID_TO_QID(station->AID), curr_tx_queue_element);

					//Poll all Tx queues, in case the just-submitted packet can be de-queueued and transmitted immediately
					poll_tx_queues();
				}
			} //END successful queue checkout

			//Select next STA if transmitted to all; otherwise terminate loop
			if(((ltg_pyld_hdr*)callback_arg)->type == LTG_PYLD_TYPE_ALL_ASSOC_FIXED){
				station_info_entry = dl_entry_next(station_info_entry);
			} else {
				station_info_entry = NULL;
			}

		} //END iterate over all selected STAs

	}

	return;
}


/**
 * @brief Callback to handle insertion of an Ethernet reception into the corresponding wireless Tx queue
 *
 * This function is called when a new Ethernet packet is received that must be transmitted via the wireless interface.
 * The packet must be encapsulated before it is passed to this function. Ethernet encapsulation is impleted in the mac_high framework.
 *
 * The tx_queue_list argument is a DL list, but must contain exactly one queue entry which contains the encapsulated packet
 * A list container is used here to ease merging of the list with the target queue.
 *
 * @param tx_queue_element* curr_tx_queue_element
 *  - A single queue element containing the packet to transmit
 * @param u8* eth_dest
 *  - 6-byte destination address from original Ethernet packet
 * @param u8* eth_src
 *  - 6-byte source address from original Ethernet packet
 * @param u16 tx_length
 *  - Length (in bytes) of the packet payload
 * @return 1 for successful enqueuing of the packet, 0 otherwise
*/
int ethernet_receive(tx_queue_element* curr_tx_queue_element, u8* eth_dest, u8* eth_src, u16 tx_length){
	//Receives the pre-encapsulated Ethernet frames
	station_info* station;
	dl_entry* entry;

	tx_queue_buffer* curr_tx_queue_buffer = (tx_queue_buffer*)(curr_tx_queue_element->data);

	wlan_mac_high_setup_tx_header( &tx_header_common, (u8*)(&(eth_dest[0])), (u8*)(&(eth_src[0])) );

	wlan_create_data_frame((void*)(curr_tx_queue_buffer->frame), &tx_header_common, MAC_FRAME_CTRL2_FLAG_FROM_DS);

	if(wlan_addr_mcast(eth_dest)){
		if(queue_num_queued(MCAST_QID) < max_queue_size){
			wlan_mac_high_setup_tx_frame_info ( &tx_header_common, curr_tx_queue_element, tx_length, 0 , MCAST_QID);

			curr_tx_queue_buffer->metadata.metadata_type = QUEUE_METADATA_TYPE_TX_PARAMS;
			curr_tx_queue_buffer->metadata.metadata_ptr = (u32)&(default_multicast_data_tx_params);

			enqueue_after_tail(MCAST_QID, curr_tx_queue_element);
			poll_tx_queues();
		} else {
			return 0;
		}

	} else {
		//Check associations
		//Is this packet meant for a station we are associated with?
		entry = wlan_mac_high_find_station_info_ADDR(&association_table, eth_dest);
		if( entry != NULL ) {
			station = (station_info*)(entry->data);
			if(queue_num_queued(AID_TO_QID(station->AID)) < max_queue_size){
				wlan_mac_high_setup_tx_frame_info ( &tx_header_common, curr_tx_queue_element, tx_length, (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO), AID_TO_QID(station->AID));

				curr_tx_queue_buffer->metadata.metadata_type = QUEUE_METADATA_TYPE_STATION_INFO;
				curr_tx_queue_buffer->metadata.metadata_ptr = (u32)station;
				curr_tx_queue_buffer->frame_info.AID = station->AID;


				enqueue_after_tail(AID_TO_QID(station->AID), curr_tx_queue_element);
				poll_tx_queues();
			} else {
				return 0;
			}
		} else {
			//Checkin this packet_bd so that it can be checked out again
			return 0;
		}


	}

	return 1;

}



void beacon_transmit() {
 	u16 tx_length;
 	tx_queue_element*	curr_tx_queue_element;
 	tx_queue_buffer* 	curr_tx_queue_buffer;

 	curr_tx_queue_element = queue_checkout();

 	if(curr_tx_queue_element != NULL){ //There was at least 1 free queue element
 		curr_tx_queue_buffer = (tx_queue_buffer*)(curr_tx_queue_element->data);

 		wlan_mac_high_setup_tx_header( &tx_header_common, (u8 *)bcast_addr, wlan_mac_addr );

        tx_length = wlan_create_beacon_frame(
			(void*)(curr_tx_queue_buffer->frame),
			&tx_header_common,
			BEACON_INTERVAL_MS,
			strlen(access_point_ssid),
			(u8*)access_point_ssid,
			mac_param_chan,
			1,
			tim_control,tim_bitmap);

 		wlan_mac_high_setup_tx_frame_info ( &tx_header_common, curr_tx_queue_element, tx_length, TX_MPDU_FLAGS_FILL_TIMESTAMP, MANAGEMENT_QID );

 		curr_tx_queue_buffer->metadata.metadata_type = QUEUE_METADATA_TYPE_TX_PARAMS;
 		curr_tx_queue_buffer->metadata.metadata_ptr = (u32)(&default_multicast_mgmt_tx_params);

 		enqueue_after_tail(MANAGEMENT_QID, curr_tx_queue_element);

 		poll_tx_queues();
 	}

 	return;
}


void association_timestamp_check() {

	u32 				i;
	u64 				time_since_last_rx;
	tx_queue_element* 	curr_tx_queue_element;
	tx_queue_buffer* 	curr_tx_queue_buffer;


	u32 tx_length;
	station_info* curr_station_info;
	dl_entry* next_station_info_entry;
	dl_entry* curr_station_info_entry;
	station_info_entry* associated_station_log_entry;

	next_station_info_entry = association_table.first;

	for(i=0; i < association_table.length; i++) {
		curr_station_info_entry = next_station_info_entry;
		next_station_info_entry = dl_entry_next(curr_station_info_entry);

		curr_station_info = (station_info*)(curr_station_info_entry->data);

		time_since_last_rx = (get_usec_timestamp() - curr_station_info->rx.last_timestamp);
		if((time_since_last_rx > ASSOCIATION_TIMEOUT_US) && ((curr_station_info->flags & STATION_INFO_FLAG_DISABLE_ASSOC_CHECK) == 0)){
			//Send De-authentication

		 	//Checkout 1 element from the queue;
			curr_tx_queue_element = queue_checkout();

		 	if(curr_tx_queue_element != NULL){ //There was at least 1 free queue element

		 		curr_tx_queue_buffer = (tx_queue_buffer*)(curr_tx_queue_element->data);

		 		wlan_mac_high_setup_tx_header( &tx_header_common, curr_station_info->addr, wlan_mac_addr );

		 		tx_length = wlan_create_deauth_frame((void*)(curr_tx_queue_buffer->frame), &tx_header_common, DEAUTH_REASON_INACTIVITY);

		 		wlan_mac_high_setup_tx_frame_info ( &tx_header_common, curr_tx_queue_element, tx_length, (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO), MANAGEMENT_QID);

		 		curr_tx_queue_buffer->metadata.metadata_type = QUEUE_METADATA_TYPE_TX_PARAMS;
		 		curr_tx_queue_buffer->metadata.metadata_ptr = (u32)(&default_unicast_mgmt_tx_params);

		 		enqueue_after_tail(MANAGEMENT_QID, curr_tx_queue_element);
		 		poll_tx_queues();

		 		//Purge any packets in the queue meant for this node
		 		purge_queue(AID_TO_QID(curr_station_info->AID));

				//Remove this STA from association list
				associated_station_log_entry = (station_info_entry*)wlan_exp_log_create_entry( ENTRY_TYPE_STATION_INFO, sizeof(station_info_entry));
				if(associated_station_log_entry != NULL){
					associated_station_log_entry->timestamp = get_usec_timestamp();
					memcpy((u8*)(&(associated_station_log_entry->info)),(u8*)(curr_station_info), sizeof(station_info_base) );
					associated_station_log_entry->info.AID = 0;
				}
				xil_printf("\n\nDisassociation due to inactivity:\n");
				wlan_mac_high_remove_association( &association_table, &statistics_table, curr_station_info->addr );
			}
		}
	}
	return;
}




void mpdu_rx_process(void* pkt_buf_addr, u8 rate, u16 length) {
	void * mpdu = pkt_buf_addr + PHY_RX_PKT_BUF_MPDU_OFFSET;
	u8* mpdu_ptr_u8 = (u8*)mpdu;
	u16 tx_length;
	u8 send_response = 0;
	mac_header_80211* rx_80211_header;
	rx_80211_header = (mac_header_80211*)((void *)mpdu_ptr_u8);
	u16 rx_seq;
	tx_queue_element*	curr_tx_queue_element;
	tx_queue_buffer* 	curr_tx_queue_buffer;
	u32 payload_log_len;
	u32 extra_payload;
	u32 transfer_len;
	u8 unicast_to_me, to_multicast;
	station_info_entry* associated_station_log_entry;


	dl_entry*	associated_station_entry;
	station_info* associated_station = NULL;
	statistics_txrx* station_stats = NULL;
	u8 eth_send;
	u8 allow_auth = 0;


	rx_common_entry* rx_event_log_entry;

	rx_frame_info* mpdu_info = (rx_frame_info*)pkt_buf_addr;

	typedef enum {PAYLOAD_FIRST, CHAN_EST_FIRST} copy_order_t;
	copy_order_t copy_order;

	mpdu_info->additional_info = (u32)NULL;

	//*************
	// Event logging
	//*************

	// Determine length required for log entry
	payload_log_len = min( max((1 + ( ( ( length ) - 1) / 4) )*4 , MIN_MAC_PAYLOAD_LOG_LEN) , mac_payload_log_len );
	extra_payload   = (payload_log_len > MIN_MAC_PAYLOAD_LOG_LEN) ? (payload_log_len - MIN_MAC_PAYLOAD_LOG_LEN) : 0;

	if(rate != WLAN_MAC_RATE_1M){
		rx_event_log_entry = (rx_common_entry*)wlan_exp_log_create_entry( ENTRY_TYPE_RX_OFDM, sizeof(rx_ofdm_entry) + extra_payload );
	} else {
		rx_event_log_entry = (rx_common_entry*)wlan_exp_log_create_entry( ENTRY_TYPE_RX_DSSS, sizeof(rx_dsss_entry) + extra_payload );
	}

	if(rx_event_log_entry != NULL){

		//For maximum pipelining, we'll break up the two major log copy operations (packet payload + [optional] channel estimates)
		//We will start the CDMA operation for whichever of those copies is shorter, then fill in the rest of the log entry
		//while that copy is under way, and then start the CDMA operation for the larger (which will first block on the shorter if
		//it is still going).

		if(rate == WLAN_MAC_RATE_1M){
			//This is a DSSS packet that has no channel estimates
			copy_order = PAYLOAD_FIRST;
		} else {
			//This is an OFDM packet that contains channel estimates
#ifdef WLAN_MAC_ENTRIES_LOG_CHAN_EST
			if( sizeof(mpdu_info->channel_est) < ( length ) ){
				copy_order = CHAN_EST_FIRST;
			} else {
				copy_order = PAYLOAD_FIRST;
			}
#else
			copy_order = PAYLOAD_FIRST;
#endif
		}

		// Have to compute the correct number of bytes to transfer to the log so that we don't walk off the end of either
		// the log entry allocation or the packet buffer
        if (length < MIN_MAC_PAYLOAD_LOG_LEN) {
    		transfer_len = length;
        } else {
    		transfer_len = MIN_MAC_PAYLOAD_LOG_LEN + min(extra_payload, (length - MIN_MAC_PAYLOAD_LOG_LEN));
        }

		switch(copy_order){
			case PAYLOAD_FIRST:
				if( rate != WLAN_MAC_RATE_1M ){
					((rx_ofdm_entry*)rx_event_log_entry)->mac_payload_log_len = length;
					wlan_mac_high_cdma_start_transfer((((rx_ofdm_entry*)rx_event_log_entry)->mac_payload), rx_80211_header, transfer_len);

					// Zero pad log entry if transfer_len was less than the allocated space in the log (ie MIN_MAC_PAYLOAD_LOG_LEN + extra_payload)
					if(transfer_len < (MIN_MAC_PAYLOAD_LOG_LEN + extra_payload)){
						bzero((u8*)(((u32)((rx_ofdm_entry*)rx_event_log_entry)->mac_payload) + transfer_len), ((MIN_MAC_PAYLOAD_LOG_LEN + extra_payload) - transfer_len));
					}
				} else {
					((rx_dsss_entry*)rx_event_log_entry)->mac_payload_log_len = length;
					wlan_mac_high_cdma_start_transfer((((rx_dsss_entry*)rx_event_log_entry)->mac_payload), rx_80211_header, transfer_len);

					// Zero pad log entry if transfer_len was less than the allocated space in the log (ie MIN_MAC_PAYLOAD_LOG_LEN + extra_payload)
					if(transfer_len < (MIN_MAC_PAYLOAD_LOG_LEN + extra_payload)){
						bzero((u8*)(((u32)((rx_dsss_entry*)rx_event_log_entry)->mac_payload) + transfer_len), ((MIN_MAC_PAYLOAD_LOG_LEN + extra_payload) - transfer_len));
					}
				}
			break;

			case CHAN_EST_FIRST:
#ifdef WLAN_MAC_ENTRIES_LOG_CHAN_EST
				if(rate != WLAN_MAC_RATE_1M) wlan_mac_high_cdma_start_transfer(((rx_ofdm_entry*)rx_event_log_entry)->channel_est, mpdu_info->channel_est, sizeof(mpdu_info->channel_est));
#endif
			break;
		}

		rx_event_log_entry->fcs_status = (mpdu_info->state == RX_MPDU_STATE_FCS_GOOD) ? RX_ENTRY_FCS_GOOD : RX_ENTRY_FCS_BAD;
		rx_event_log_entry->timestamp  =  mpdu_info->timestamp;
		rx_event_log_entry->power      = mpdu_info->rx_power;
		rx_event_log_entry->rf_gain    = mpdu_info->rf_gain;
		rx_event_log_entry->bb_gain    = mpdu_info->bb_gain;
		rx_event_log_entry->length     = mpdu_info->length;
		rx_event_log_entry->rate       = mpdu_info->rate;
		rx_event_log_entry->pkt_type   = wlan_mac_high_pkt_type(mpdu,length);
		rx_event_log_entry->chan_num   = mac_param_chan;
		rx_event_log_entry->ant_mode   = mpdu_info->ant_mode;
		rx_event_log_entry->flags      = 0;

		switch(copy_order){
			case CHAN_EST_FIRST:
				if( rate != WLAN_MAC_RATE_1M ){
					((rx_ofdm_entry*)rx_event_log_entry)->mac_payload_log_len = length;
					wlan_mac_high_cdma_start_transfer((((rx_ofdm_entry*)rx_event_log_entry)->mac_payload), rx_80211_header, transfer_len);

					// Zero pad log entry if transfer_len was less than the allocated space in the log (ie MIN_MAC_PAYLOAD_LOG_LEN + extra_payload)
					if(transfer_len < (MIN_MAC_PAYLOAD_LOG_LEN + extra_payload)){
						bzero((u8*)(((u32)((rx_ofdm_entry*)rx_event_log_entry)->mac_payload) + transfer_len), ((MIN_MAC_PAYLOAD_LOG_LEN + extra_payload) - transfer_len));
					}
				} else {
					((rx_dsss_entry*)rx_event_log_entry)->mac_payload_log_len = length;
					wlan_mac_high_cdma_start_transfer((((rx_dsss_entry*)rx_event_log_entry)->mac_payload), rx_80211_header, transfer_len);

					// Zero pad log entry if transfer_len was less than the allocated space in the log (ie MIN_MAC_PAYLOAD_LOG_LEN + extra_payload)
					if(transfer_len < (MIN_MAC_PAYLOAD_LOG_LEN + extra_payload)){
						bzero((u8*)(((u32)((rx_dsss_entry*)rx_event_log_entry)->mac_payload) + transfer_len), ((MIN_MAC_PAYLOAD_LOG_LEN + extra_payload) - transfer_len));
					}
				}
			break;

			case PAYLOAD_FIRST:
#ifdef WLAN_MAC_ENTRIES_LOG_CHAN_EST
				if(rate != WLAN_MAC_RATE_1M) wlan_mac_high_cdma_start_transfer(((rx_ofdm_entry*)rx_event_log_entry)->channel_est, mpdu_info->channel_est, sizeof(mpdu_info->channel_est));
#endif
			break;
		}

#ifdef _DEBUG_
		xil_printf("RX      : %8d    %8d    %8d    %8d    %8d\n", transfer_len, MIN_MAC_PAYLOAD_LOG_LEN, length, extra_payload, payload_log_len);
        print_buf((u8 *)((u32)rx_event_log_entry - 8), sizeof(rx_ofdm_entry) + extra_payload + 12);
#endif
	}

	unicast_to_me = wlan_addr_eq(rx_80211_header->address_1, wlan_mac_addr);
	to_multicast = wlan_addr_mcast(rx_80211_header->address_1);

	if( mpdu_info->state == RX_MPDU_STATE_FCS_GOOD && (unicast_to_me || to_multicast)){
		//GOOD FCS
		associated_station_entry = wlan_mac_high_find_station_info_ADDR(&association_table, (rx_80211_header->address_2));

		if( associated_station_entry != NULL ){
			associated_station = (station_info*)(associated_station_entry->data);
			mpdu_info->additional_info = (u32)associated_station;
			station_stats = associated_station->stats;
			rx_seq = ((rx_80211_header->sequence_control)>>4)&0xFFF;

			associated_station->rx.last_timestamp = get_usec_timestamp();
			associated_station->rx.last_power = mpdu_info->rx_power;
			associated_station->rx.last_rate = mpdu_info->rate;

			//Check if duplicate
			if( (associated_station->rx.last_seq != 0)  && (associated_station->rx.last_seq == rx_seq) ) {
				//Received seq num matched previously received seq num for this STA; ignore the MPDU and finish function
				if(rx_event_log_entry != NULL){
					rx_event_log_entry->flags |= RX_ENTRY_FLAGS_IS_DUPLICATE;
				}
				goto mpdu_rx_process_end;

			} else {
				associated_station->rx.last_seq = rx_seq;
			}
		} else {
			station_stats = wlan_mac_high_add_statistics(&statistics_table, NULL, rx_80211_header->address_2);
		}

		if(station_stats != NULL){
			station_stats->last_rx_timestamp = get_usec_timestamp();
			if((rx_80211_header->frame_control_1 & 0xF) == MAC_FRAME_CTRL1_TYPE_DATA){
				((station_stats)->data.rx_num_packets)++;
				((station_stats)->data.rx_num_bytes) += mpdu_info->length;
			} else if((rx_80211_header->frame_control_1 & 0xF) == MAC_FRAME_CTRL1_TYPE_MGMT) {
				((station_stats)->mgmt.rx_num_packets)++;
				((station_stats)->mgmt.rx_num_bytes) += mpdu_info->length;
			}
		}

		switch(rx_80211_header->frame_control_1) {
			case (MAC_FRAME_CTRL1_SUBTYPE_DATA): //Data Packet
				if(associated_station != NULL){
					if((rx_80211_header->frame_control_2) & MAC_FRAME_CTRL2_FLAG_TO_DS) {
						//MPDU is flagged as destined to the DS
						eth_send = 1;
						if(wlan_addr_mcast(rx_80211_header->address_3)){
							curr_tx_queue_element = queue_checkout();

							if(curr_tx_queue_element != NULL){ //There was at least 1 free queue element
								curr_tx_queue_buffer = (tx_queue_buffer*)(curr_tx_queue_element->data);

								wlan_mac_high_setup_tx_header( &tx_header_common, rx_80211_header->address_3, rx_80211_header->address_2);
								mpdu_ptr_u8 = curr_tx_queue_buffer->frame;
								tx_length = wlan_create_data_frame((void*)mpdu_ptr_u8, &tx_header_common, MAC_FRAME_CTRL2_FLAG_FROM_DS);
								mpdu_ptr_u8 += sizeof(mac_header_80211);
								memcpy(mpdu_ptr_u8, (void*)rx_80211_header + sizeof(mac_header_80211), mpdu_info->length - sizeof(mac_header_80211));
								wlan_mac_high_setup_tx_frame_info ( &tx_header_common, curr_tx_queue_element, mpdu_info->length, 0, MCAST_QID );

								curr_tx_queue_buffer->metadata.metadata_type = QUEUE_METADATA_TYPE_TX_PARAMS;
								curr_tx_queue_buffer->metadata.metadata_ptr = (u32)(&default_multicast_data_tx_params);

								enqueue_after_tail(MCAST_QID, curr_tx_queue_element);
								poll_tx_queues();
							}

						} else {
							associated_station_entry = wlan_mac_high_find_station_info_ADDR(&association_table, rx_80211_header->address_3);

							if(associated_station_entry != NULL){
								associated_station = (station_info*)(associated_station_entry->data);
								curr_tx_queue_element = queue_checkout();

								if(curr_tx_queue_element != NULL){ //There was at least 1 free queue element
									curr_tx_queue_buffer = (tx_queue_buffer*)(curr_tx_queue_element->data);

									wlan_mac_high_setup_tx_header( &tx_header_common, rx_80211_header->address_3, rx_80211_header->address_2);
									mpdu_ptr_u8 = curr_tx_queue_buffer->frame;
									tx_length = wlan_create_data_frame((void*)mpdu_ptr_u8, &tx_header_common, MAC_FRAME_CTRL2_FLAG_FROM_DS);
									mpdu_ptr_u8 += sizeof(mac_header_80211);
									memcpy(mpdu_ptr_u8, (void*)rx_80211_header + sizeof(mac_header_80211), mpdu_info->length - sizeof(mac_header_80211));
									wlan_mac_high_setup_tx_frame_info ( &tx_header_common, curr_tx_queue_element, mpdu_info->length , (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO), AID_TO_QID(associated_station->AID) );

									curr_tx_queue_buffer->metadata.metadata_type = QUEUE_METADATA_TYPE_STATION_INFO;
									curr_tx_queue_buffer->metadata.metadata_ptr = (u32)(associated_station);
									curr_tx_queue_buffer->frame_info.AID = associated_station->AID;

									enqueue_after_tail(AID_TO_QID(associated_station->AID),  curr_tx_queue_element);

									poll_tx_queues();
									#ifndef ALLOW_ETH_TX_OF_WIRELESS_TX
									eth_send = 0;
									#endif
								}
							}
						}

						if(eth_send){
							wlan_mpdu_eth_send(mpdu,length);
						}

					}
				} else {
					//TODO: Formally adopt conventions from 10.3 in 802.11-2012 for STA state transitions
					if(wlan_addr_eq(rx_80211_header->address_1, wlan_mac_addr)){

						//Received a data frame from a STA that claims to be associated with this AP but is not in the AP association table
						// Discard the MPDU and reply with a de-authentication frame to trigger re-association at the STA

						warp_printf(PL_WARNING, "Data from non-associated station: [%x %x %x %x %x %x], issuing de-authentication\n", rx_80211_header->address_2[0],rx_80211_header->address_2[1],rx_80211_header->address_2[2],rx_80211_header->address_2[3],rx_80211_header->address_2[4],rx_80211_header->address_2[5]);
						warp_printf(PL_WARNING, "Address 3: [%x %x %x %x %x %x]\n", rx_80211_header->address_3[0],rx_80211_header->address_3[1],rx_80211_header->address_3[2],rx_80211_header->address_3[3],rx_80211_header->address_3[4],rx_80211_header->address_3[5]);

						//Send De-authentication
						//Checkout 1 element from the queue;
						curr_tx_queue_element = queue_checkout();
						if(curr_tx_queue_element != NULL){ //There was at least 1 free queue element
							curr_tx_queue_buffer = (tx_queue_buffer*)(curr_tx_queue_element->data);

							wlan_mac_high_setup_tx_header( &tx_header_common, rx_80211_header->address_2, wlan_mac_addr );

							tx_length = wlan_create_deauth_frame((void*)(curr_tx_queue_buffer->frame), &tx_header_common, DEAUTH_REASON_NONASSOCIATED_STA);

							wlan_mac_high_setup_tx_frame_info ( &tx_header_common, curr_tx_queue_element, tx_length, (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO), MANAGEMENT_QID );

							curr_tx_queue_buffer->metadata.metadata_type = QUEUE_METADATA_TYPE_TX_PARAMS;
							curr_tx_queue_buffer->metadata.metadata_ptr = (u32)(&default_unicast_mgmt_tx_params);

							enqueue_after_tail(MANAGEMENT_QID, curr_tx_queue_element);
							poll_tx_queues();
						}

					}
				}//END if(associated_station != NULL)

			break;

			case (MAC_FRAME_CTRL1_SUBTYPE_PROBE_REQ): //Probe Request Packet
				if(wlan_addr_eq(rx_80211_header->address_3, bcast_addr)) {
					//BSS Id: Broadcast
					mpdu_ptr_u8 += sizeof(mac_header_80211);
					while(((u32)mpdu_ptr_u8 -  (u32)mpdu)<= length){ //Loop through tagged parameters
						switch(mpdu_ptr_u8[0]){ //What kind of tag is this?
							case TAG_SSID_PARAMS: //SSID parameter set
								if((mpdu_ptr_u8[1]==0) || (memcmp(mpdu_ptr_u8+2, (u8*)access_point_ssid,mpdu_ptr_u8[1])==0)) {
									//Broadcast SSID or my SSID - send unicast probe response
									send_response = 1;
								}
							break;
							case TAG_SUPPORTED_RATES: //Supported rates
							break;
							case TAG_EXT_SUPPORTED_RATES: //Extended supported rates
							break;
							case TAG_DS_PARAMS: //DS Parameter set (e.g. channel)
							break;
						}
						mpdu_ptr_u8 += mpdu_ptr_u8[1]+2; //Move up to the next tag
					}

					//xil_printf("Probe Req Rx %d %d\n", send_response, allow_assoc);

					if(send_response && allow_assoc) {

						//Checkout 1 element from the queue;
						curr_tx_queue_element = queue_checkout();

						if(curr_tx_queue_element != NULL){ //There was at least 1 free queue element
							curr_tx_queue_buffer = (tx_queue_buffer*)(curr_tx_queue_element->data);

							wlan_mac_high_setup_tx_header( &tx_header_common, rx_80211_header->address_2, wlan_mac_addr );

							tx_length = wlan_create_probe_resp_frame((void*)(curr_tx_queue_buffer->frame), &tx_header_common, BEACON_INTERVAL_MS, strlen(access_point_ssid), (u8*)access_point_ssid, mac_param_chan);

							wlan_mac_high_setup_tx_frame_info ( &tx_header_common, curr_tx_queue_element, tx_length, (TX_MPDU_FLAGS_FILL_TIMESTAMP | TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO), MANAGEMENT_QID );

							curr_tx_queue_buffer->metadata.metadata_type = QUEUE_METADATA_TYPE_TX_PARAMS;
							curr_tx_queue_buffer->metadata.metadata_ptr = (u32)(&default_unicast_mgmt_tx_params);

							enqueue_after_tail(MANAGEMENT_QID, curr_tx_queue_element);
							poll_tx_queues();
							//xil_printf("Probe Response Tx\n");
						}
						// Finish function
						goto mpdu_rx_process_end;
					}
				}
			break;

			case (MAC_FRAME_CTRL1_SUBTYPE_AUTH): //Authentication Packet
				if(wlan_addr_eq(rx_80211_header->address_3, wlan_mac_addr) && wlan_mac_addr_filter_is_allowed(rx_80211_header->address_2)) {
					mpdu_ptr_u8 += sizeof(mac_header_80211);
					switch(((authentication_frame*)mpdu_ptr_u8)->auth_algorithm ){
						case AUTH_ALGO_OPEN_SYSTEM:
							allow_auth = 1;
						break;
						default:
							allow_auth = 0;
						break;
					}
				}

				if(allow_auth){
					if(((authentication_frame*)mpdu_ptr_u8)->auth_sequence == AUTH_SEQ_REQ){//This is an auth packet from a requester
						//Checkout 1 element from the queue;
						curr_tx_queue_element = queue_checkout();

						if(curr_tx_queue_element != NULL){ //There was at least 1 free queue element
							curr_tx_queue_buffer = (tx_queue_buffer*)(curr_tx_queue_element->data);

							wlan_mac_high_setup_tx_header( &tx_header_common, rx_80211_header->address_2, wlan_mac_addr );

							tx_length = wlan_create_auth_frame((void*)(curr_tx_queue_buffer->frame), &tx_header_common, AUTH_ALGO_OPEN_SYSTEM, AUTH_SEQ_RESP, STATUS_SUCCESS);

							wlan_mac_high_setup_tx_frame_info ( &tx_header_common, curr_tx_queue_element, tx_length, (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO), MANAGEMENT_QID );

							curr_tx_queue_buffer->metadata.metadata_type = QUEUE_METADATA_TYPE_TX_PARAMS;
							curr_tx_queue_buffer->metadata.metadata_ptr = (u32)(&default_unicast_mgmt_tx_params);

							enqueue_after_tail(MANAGEMENT_QID, curr_tx_queue_element);
							poll_tx_queues();
						}
						goto mpdu_rx_process_end;
					}
				} else {
					if(((authentication_frame*)mpdu_ptr_u8)->auth_sequence == AUTH_SEQ_REQ){
						//Checkout 1 element from the queue;
						curr_tx_queue_element = queue_checkout();

						if(curr_tx_queue_element == NULL){ //There was at least 1 free queue element
							curr_tx_queue_buffer = (tx_queue_buffer*)(curr_tx_queue_element->data);

							wlan_mac_high_setup_tx_header( &tx_header_common, rx_80211_header->address_2, wlan_mac_addr );

							tx_length = wlan_create_auth_frame((void*)(curr_tx_queue_buffer->frame), &tx_header_common, AUTH_ALGO_OPEN_SYSTEM, AUTH_SEQ_RESP, STATUS_AUTH_REJECT_UNSPECIFIED);

							wlan_mac_high_setup_tx_frame_info ( &tx_header_common, curr_tx_queue_element, tx_length, (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO), MANAGEMENT_QID );

							curr_tx_queue_buffer->metadata.metadata_type = QUEUE_METADATA_TYPE_TX_PARAMS;
							curr_tx_queue_buffer->metadata.metadata_ptr = (u32)(&default_unicast_mgmt_tx_params);

							enqueue_after_tail(MANAGEMENT_QID, curr_tx_queue_element);
							poll_tx_queues();
						}
					}
					// Finish function
					goto mpdu_rx_process_end;
				}
			break;

			case (MAC_FRAME_CTRL1_SUBTYPE_REASSOC_REQ): //Re-association Request
			case (MAC_FRAME_CTRL1_SUBTYPE_ASSOC_REQ): //Association Request

				if(wlan_addr_eq(rx_80211_header->address_3, wlan_mac_addr)) {


					if(association_table.length < MAX_NUM_ASSOC) associated_station = wlan_mac_high_add_association(&association_table, &statistics_table, rx_80211_header->address_2, ADD_ASSOCIATION_ANY_AID);

					if(associated_station != NULL) {

						associated_station_log_entry = (station_info_entry*)wlan_exp_log_create_entry( ENTRY_TYPE_STATION_INFO, sizeof(station_info_entry));
						if(associated_station_log_entry != NULL){
							associated_station_log_entry->timestamp = get_usec_timestamp();
							memcpy((u8*)(&(associated_station_log_entry->info)),(u8*)(associated_station), sizeof(station_info_base) );
						}

						memcpy(&(associated_station->tx),&default_unicast_data_tx_params, sizeof(tx_params));

						//Checkout 1 element from the queue;
						curr_tx_queue_element = queue_checkout();

						if(curr_tx_queue_element != NULL){ //There was at least 1 free queue element
							curr_tx_queue_buffer = (tx_queue_buffer*)(curr_tx_queue_element->data);

							wlan_mac_high_setup_tx_header( &tx_header_common, rx_80211_header->address_2, wlan_mac_addr );

							tx_length = wlan_create_association_response_frame((void*)(curr_tx_queue_buffer->frame), &tx_header_common, STATUS_SUCCESS, associated_station->AID);

							wlan_mac_high_setup_tx_frame_info ( &tx_header_common, curr_tx_queue_element, tx_length, (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO), AID_TO_QID(associated_station->AID) );

							curr_tx_queue_buffer->metadata.metadata_type = QUEUE_METADATA_TYPE_STATION_INFO;
							curr_tx_queue_buffer->metadata.metadata_ptr = (u32)associated_station;
							curr_tx_queue_buffer->frame_info.AID = associated_station->AID;

							enqueue_after_tail(AID_TO_QID(associated_station->AID), curr_tx_queue_element);
							poll_tx_queues();
						}
						// Finish function
						goto mpdu_rx_process_end;
					} else {
						//Checkout 1 element from the queue;
						curr_tx_queue_element = queue_checkout();

						if(curr_tx_queue_element != NULL){ //There was at least 1 free queue element
							curr_tx_queue_buffer = (tx_queue_buffer*)(curr_tx_queue_element->data);

							wlan_mac_high_setup_tx_header( &tx_header_common, rx_80211_header->address_2, wlan_mac_addr );

							tx_length = wlan_create_association_response_frame((void*)(curr_tx_queue_buffer->frame), &tx_header_common, STATUS_REJECT_TOO_MANY_ASSOCIATIONS, 0);

							wlan_mac_high_setup_tx_frame_info ( &tx_header_common, curr_tx_queue_element, tx_length, (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO), MANAGEMENT_QID );

							curr_tx_queue_buffer->metadata.metadata_type = QUEUE_METADATA_TYPE_TX_PARAMS;
							curr_tx_queue_buffer->metadata.metadata_ptr = (u32)(&default_unicast_mgmt_tx_params);

							enqueue_after_tail(MANAGEMENT_QID, curr_tx_queue_element);
							poll_tx_queues();
						}
					}

				}
			break;

			case (MAC_FRAME_CTRL1_SUBTYPE_DISASSOC): //Disassociation

					if(associated_station != NULL){
						associated_station_log_entry = (station_info_entry*)wlan_exp_log_create_entry( ENTRY_TYPE_STATION_INFO, sizeof(station_info_entry));
						if(associated_station_log_entry != NULL){
							associated_station_log_entry->timestamp = get_usec_timestamp();
							memcpy((u8*)(&(associated_station_log_entry->info)),(u8*)(associated_station), sizeof(station_info_base) );
							associated_station_log_entry->info.AID = 0;
						}
					}
					wlan_mac_high_remove_association(&association_table, &statistics_table, rx_80211_header->address_2);

			break;

			default:
				//This should be left as a verbose print. It occurs often when communicating with mobile devices since they tend to send
				//null data frames (type: DATA, subtype: 0x4) for power management reasons.
				warp_printf(PL_VERBOSE, "Received unknown frame control type/subtype %x\n",rx_80211_header->frame_control_1);
			break;
		}
		goto mpdu_rx_process_end;

	} else {
		//Bad FCS
		goto mpdu_rx_process_end;
	}
	mpdu_rx_process_end:
	if (rx_event_log_entry != NULL) {
		//wn_transmit_log_entry((void *)rx_event_log_entry);
	}
}

u32  get_associations_status() {
	// Get the status of associations for the AP
	//   - 00 -> Associations not allowed
	//   - 01 -> Associations allowed for a window
	//   - 11 -> Associations allowed permanently

	return ( perma_assoc_mode * 2 ) + allow_assoc;
}

void enable_associations( u32 permanent_association ){
	// Send a message to other processor to tell it to enable associations
#ifdef _DEBUG_
	xil_printf("Allowing new associations\n");
#endif

	// Set the DSSS value in CPU Low
	wlan_mac_high_set_dsss( 1 );

    // Set the global variable
	allow_assoc = 1;

	// Set the global variable for permanently allowing associations
	switch ( permanent_association ) {

        case ASSOCIATION_ALLOW_PERMANENT:
        	perma_assoc_mode = 1;
        break;

        case ASSOCIATION_ALLOW_TEMPORARY:
        	perma_assoc_mode = 0;
        break;
	}
}



void disable_associations(){
	// Send a message to other processor to tell it to disable associations
	if(perma_assoc_mode == 0){

#ifdef _DEBUG_
		xil_printf("Not allowing new associations\n");
#endif

		// Set the DSSS value in CPU Low
		wlan_mac_high_set_dsss( 0 );

        // Set the global variables
		allow_assoc      = 0;

		// Stop the animation on the hex displays from continuing
		wlan_mac_high_disable_hex_dot_blink();

		// Set the hex display
		wlan_mac_high_write_hex_display(association_table.length);
	}
}


/*****************************************************************************/
/**
* Reset Station Statistics
*
* Reset all statistics being kept for all stations
*
* @param    None.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void reset_station_statistics(){
	wlan_mac_high_reset_statistics(&statistics_table);
}


u32  deauthenticate_station( station_info* station ) {

	tx_queue_element*		curr_tx_queue_element;
	tx_queue_buffer* 		curr_tx_queue_buffer;
	u32            tx_length;
	u32            aid;
	station_info_entry* associated_station_log_entry;

	if(station == NULL){
		return 0;
	}

	// Get the AID
	aid = station->AID;

	// Checkout 1 element from the queue
	curr_tx_queue_element = queue_checkout();

	if(curr_tx_queue_element != NULL){ //There was at least 1 free queue element

		curr_tx_queue_buffer = (tx_queue_buffer*)(curr_tx_queue_element->data);

		purge_queue(AID_TO_QID(aid));

		// Create deauthentication packet
		wlan_mac_high_setup_tx_header( &tx_header_common, station->addr, wlan_mac_addr );

		tx_length = wlan_create_deauth_frame((void*)(curr_tx_queue_buffer->frame), &tx_header_common, DEAUTH_REASON_INACTIVITY);

		wlan_mac_high_setup_tx_frame_info ( &tx_header_common, curr_tx_queue_element, tx_length, (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO), MANAGEMENT_QID );

		curr_tx_queue_buffer->metadata.metadata_type = QUEUE_METADATA_TYPE_TX_PARAMS;
		curr_tx_queue_buffer->metadata.metadata_ptr = (u32)(&default_unicast_mgmt_tx_params);

		enqueue_after_tail(MANAGEMENT_QID, curr_tx_queue_element);
		poll_tx_queues();



		// Remove this STA from association list
		associated_station_log_entry = (station_info_entry*)wlan_exp_log_create_entry( ENTRY_TYPE_STATION_INFO, sizeof(station_info_entry));
		if(associated_station_log_entry != NULL){
			associated_station_log_entry->timestamp = get_usec_timestamp();
			memcpy((u8*)(&(associated_station_log_entry->info)),(u8*)(station), sizeof(station_info_base) );
			associated_station_log_entry->info.AID = 0;
		}
		wlan_mac_high_remove_association( &association_table, &statistics_table, station->addr );
	}

	wlan_mac_high_write_hex_display(association_table.length);

	return aid;
}



/*****************************************************************************/
/**
* Deauthenticate all stations in the Association Table
*
* Loop through all associations in the table and deauthenticate the stations
*
* @param    None.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void deauthenticate_stations(){
	u32 i;
	station_info* curr_station_info;
	dl_entry* next_station_info_entry;
	dl_entry* curr_station_info_entry;

	next_station_info_entry = association_table.first;
	for (i = 0; i < association_table.length ; i++){
		curr_station_info_entry = next_station_info_entry;
		next_station_info_entry = dl_entry_next(curr_station_info_entry);
		curr_station_info = (station_info*)(curr_station_info_entry->data);
		deauthenticate_station(curr_station_info);
	}
}

dl_list * get_statistics(){
	return &statistics_table;
}

dl_list * get_station_info_list(){
	return &association_table;
}
