/** @file wlan_mac_sta.c
 *  @brief Station
 *
 *  This contains code for the 802.11 Station.
 *
 *  @copyright Copyright 2014, Mango Communications. All rights reserved.
 *          Distributed under the Mango Communications Reference Design License
 *				See LICENSE.txt included in the design archive or
 *				at http://mangocomm.com/802.11/license
 *
 *  @author Chris Hunter (chunter [at] mangocomm.com)
 *  @author Patrick Murphy (murphpo [at] mangocomm.com)
 *  @author Erik Welsh (welsh [at] mangocomm.com)
 *  @bug No known bugs
 */

/***************************** Include Files *********************************/

//Xilinx SDK includes
#include "xparameters.h"
#include "stdio.h"
#include "stdlib.h"
#include "xtmrctr.h"
#include "xio.h"
#include "string.h"
#include "xintc.h"

//WARP includes
#include "wlan_mac_ipc_util.h"
#include "wlan_mac_misc_util.h"
#include "wlan_mac_802_11_defs.h"
#include "wlan_mac_queue.h"
#include "wlan_mac_event_log.h"
#include "wlan_mac_entries.h"
#include "wlan_mac_ltg.h"
#include "wlan_mac_high.h"
#include "wlan_mac_packet_types.h"
#include "wlan_mac_eth_util.h"
#include "wlan_mac_sta.h"
#include "ascii_characters.h"
#include "wlan_mac_schedule.h"
#include "wlan_mac_dl_list.h"


// WLAN Exp includes
#include "wlan_exp.h"
#include "wlan_exp_common.h"
#include "wlan_exp_node.h"
#include "wlan_exp_node_sta.h"
#include "wlan_exp_transport.h"


/*************************** Constant Definitions ****************************/

#define  WLAN_EXP_ETH                  		WN_ETH_B
#define  WLAN_EXP_NODE_TYPE                 WARPNET_TYPE_80211_BASE + WARPNET_TYPE_80211_HIGH_STA

#define  WLAN_DEFAULT_CHANNEL               1
#define  WLAN_DEFAULT_TX_PWR				10


/*********************** Global Variable Definitions *************************/


/*************************** Variable Definitions ****************************/

// If you want this station to try to associate to a known AP at boot, type
//   the string here. Otherwise, let it be an empty string.
static char default_AP_SSID[] = "WARP-AP";
char*  access_point_ssid;

// Common TX header for 802.11 packets
mac_header_80211_common tx_header_common;

// Default Transmission Parameters;
tx_params default_unicast_mgmt_tx_params;
tx_params default_unicast_data_tx_params;
tx_params default_multicast_mgmt_tx_params;
tx_params default_multicast_data_tx_params;

int association_state;                      // Section 10.3 of 802.11-2012
u8  uart_mode;
u8  active_scan;

u8  repeated_active_scan_scheduled;
u32 active_scan_schedule_id;
u8  pause_queue;


// Access point information
ap_info* ap_list;
u8       num_ap_list;

u8       access_point_num_basic_rates;
u8       access_point_basic_rates[NUM_BASIC_RATES_MAX];


// Association Table variables
dl_list		 association_table;
dl_list		 statistics_table;
u8			 ap_addr[6];

u32			 max_queue_size;


// AP channel
u32 mac_param_chan;
u32 mac_param_chan_save;


// MAC address
static u8 wlan_mac_addr[6];


/*************************** Functions Prototypes ****************************/


#ifdef WLAN_USE_UART_MENU

void uart_rx(u8 rxByte);

#else

void uart_rx(u8 rxByte){ };

#endif

void add_temp();



/******************************** Functions **********************************/

int main() {

	//This function should be executed first. It will zero out memory, and if that
	//memory is used before calling this function, unexpected results may happen.
	wlan_mac_high_heap_init();

    // Initialize AP list
	num_ap_list = 0;
	//free(ap_list);
	ap_list = NULL;
	repeated_active_scan_scheduled = 0;

	max_queue_size = MAX_TX_QUEUE_LEN;

	//Unpause the queue
	pause_queue = 0;

	xil_printf("----- Mango 802.11 Reference Design -----\n");
	xil_printf("----- v0.9 Beta -------------------------\n");
	xil_printf("----- wlan_mac_sta ----------------------\n");


	xil_printf("Compiled %s %s\n\n", __DATE__, __TIME__);

	//xil_printf("_heap_start = 0x%x, %x\n", *(char*)(_heap_start),_heap_start);
	//xil_printf("_heap_end = 0x%x, %x\n", *(char*)(_heap_end),_heap_end);

    // Set Global variables
	default_unicast_data_tx_params.mac.num_tx_max = MAX_NUM_TX;
	default_unicast_data_tx_params.phy.power = WLAN_DEFAULT_TX_PWR;
	default_unicast_data_tx_params.phy.rate = WLAN_MAC_RATE_18M;
	default_unicast_data_tx_params.phy.antenna_mode = TX_ANTMODE_SISO_ANTA;

	default_unicast_mgmt_tx_params.mac.num_tx_max = MAX_NUM_TX;
	default_unicast_mgmt_tx_params.phy.power = WLAN_DEFAULT_TX_PWR;
	default_unicast_mgmt_tx_params.phy.rate = WLAN_MAC_RATE_6M;
	default_unicast_mgmt_tx_params.phy.antenna_mode = TX_ANTMODE_SISO_ANTA;

	default_multicast_mgmt_tx_params.mac.num_tx_max = 1;
	default_multicast_mgmt_tx_params.phy.power = WLAN_DEFAULT_TX_PWR;
	default_multicast_mgmt_tx_params.phy.rate = WLAN_MAC_RATE_6M;
	default_multicast_mgmt_tx_params.phy.antenna_mode = TX_ANTMODE_SISO_ANTA;


	// Initialize the utility library
    wlan_mac_high_init();
#ifdef USE_WARPNET_WLAN_EXP
	wlan_exp_configure(WLAN_EXP_NODE_TYPE, WLAN_EXP_ETH);
#endif

	// Initialize callbacks
	wlan_mac_util_set_eth_rx_callback(       (void*)ethernet_receive);
	wlan_mac_high_set_mpdu_tx_done_callback( (void*)mpdu_transmit_done);
	wlan_mac_high_set_mpdu_rx_callback(      (void*)mpdu_rx_process);
	wlan_mac_high_set_uart_rx_callback(      (void*)uart_rx);
	wlan_mac_high_set_mpdu_accept_callback(  (void*)poll_tx_queues);
	wlan_mac_ltg_sched_set_callback(         (void*)ltg_event);

	wlan_mac_util_set_eth_encap_mode(ENCAP_MODE_STA);

	dl_list_init(&association_table);
	dl_list_init(&statistics_table);

	// Set default SSID for AP
	access_point_ssid = wlan_mac_high_malloc(strlen(default_AP_SSID)+1);
	strcpy(access_point_ssid,default_AP_SSID);


	// Set Association state for station to AP
	association_state = 1;

    // Wait for CPU Low to initialize
	while( wlan_mac_high_is_cpu_low_initialized() == 0){
		xil_printf("waiting on CPU_LOW to boot\n");
	};


	// CPU Low will pass HW information to CPU High as part of the boot process
	//   - Get necessary HW information
	memcpy((void*) &(wlan_mac_addr[0]), (void*) wlan_mac_high_get_eeprom_mac_addr(), 6);


    // Set Header information
	tx_header_common.address_2 = &(wlan_mac_addr[0]);
	tx_header_common.seq_num = 0;


    // Initialize hex display
	wlan_mac_high_write_hex_display(0);


	// Set up channel
	mac_param_chan = WLAN_DEFAULT_CHANNEL;
	mac_param_chan_save = mac_param_chan;
	wlan_mac_high_set_channel( mac_param_chan );
	wlan_mac_high_set_rx_ant_mode(RX_ANTMODE_SISO_ANTA);
	wlan_mac_high_set_tx_ctrl_pow(WLAN_DEFAULT_TX_PWR);

	// Configure CPU Low's filter for passing Rx packets up to CPU High
	//  Default is "promiscuous" mode - pass all data and management packets with good or bad checksums
	//   This allows logging of all data/management receptions, even if they're not intended for this node
	wlan_mac_high_set_rx_filter_mode(RX_FILTER_FCS_ALL | RX_FILTER_HDR_ALL_MPDU);

    // Initialize interrupts
	wlan_mac_high_interrupt_init();

    // Schedule all events
//	wlan_mac_schedule_event_repeated(SCHEDULE_COARSE, 10000000, SCHEDULE_REPEAT_FOREVER, (void*)add_temp);  // Collect temperature once every 10 seconds //TODO add back in


	// Reset the event log
	event_log_reset();

	// Print Station information to the terminal
    xil_printf("WLAN MAC Station boot complete: \n");
    xil_printf("  Default SSID : %s \n", access_point_ssid);
    xil_printf("  Channel      : %d \n", mac_param_chan);
	xil_printf("  MAC Addr     : %02x-%02x-%02x-%02x-%02x-%02x\n\n",wlan_mac_addr[0],wlan_mac_addr[1],wlan_mac_addr[2],wlan_mac_addr[3],wlan_mac_addr[4],wlan_mac_addr[5]);


#ifdef WLAN_USE_UART_MENU
	uart_mode = UART_MODE_MAIN;

	xil_printf("\nAt any time, press the Esc key in your terminal to access the AP menu\n");
#endif


	// If there is a default SSID, initiate a probe request
	if( strlen(access_point_ssid) > 0 ) start_active_scan();

#ifdef USE_WARPNET_WLAN_EXP
	// Set AP processing callbacks
	node_set_process_callback( (void *)wlan_exp_node_sta_processCmd );
#endif


	wlan_mac_high_interrupt_start();


	while(1){
		//The design is entirely interrupt based. When no events need to be processed, the processor
		//will spin in this loop until an interrupt happens

#ifdef USE_WARPNET_WLAN_EXP
//		wlan_mac_high_interrupt_stop();
		transport_poll( WLAN_EXP_ETH );
//		wlan_mac_high_interrupt_start();
#endif
	}
	return -1;
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


void add_temp() {
	add_temperature_to_log(WN_TRANSMIT);
}


void poll_tx_queues(){
	u8 i;
	#define MAX_NUM_QUEUE 2
	if(pause_queue == 0){
		static u32 queue_index = 0;
		if( wlan_mac_high_is_ready_for_tx() ){
			for(i=0;i<MAX_NUM_QUEUE;i++){
				//Alternate between checking the unassociated queue and the associated queue
				queue_index = (queue_index+1)%MAX_NUM_QUEUE;

				switch(queue_index){
					case 0:
						if(dequeue_transmit_checkin(MANAGEMENT_QID)){
							return;
						}
					break;

					case 1:
						if(dequeue_transmit_checkin(UNICAST_QID)){
							return;
						}
					break;
				}


			}
		}
	}

}

void purge_all_data_tx_queue(){

	// Purge all data transmit queues
	purge_queue(MCAST_QID);           // Broadcast Queue
	purge_queue(UNICAST_QID);         // Unicast Queue
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
void mpdu_transmit_done(tx_frame_info* tx_mpdu, wlan_mac_low_tx_details* tx_low_details, u16 num_tx_low_details){
	u32 i;
	tx_high_entry* tx_high_event_log_entry;
	tx_low_entry*  tx_low_event_log_entry;
	station_info* station;
	dl_entry*	  station_info_entry;

	u8 			  pkt_type;

	frame_statistics_txrx* frame_stats = NULL;

	//Get a pointer to the MPDU payload in the packet buffer
	void * mpdu = (void*)tx_mpdu + PHY_TX_PKT_BUF_MPDU_OFFSET;
	u8* mpdu_ptr_u8 = (u8*)mpdu;

	//Get a pointer to the MAC header in the MPDU
	mac_header_80211* tx_80211_header;
	tx_80211_header = (mac_header_80211*)((void *)mpdu_ptr_u8);

	u64 ts_old = 0;
	u32 payload_log_len;
	u32 extra_payload;
	u32 transfer_len;
	u32 total_payload_len = tx_mpdu->length;

	pkt_type = wlan_mac_high_pkt_type(mpdu,tx_mpdu->length);

	for(i = 0; i < num_tx_low_details; i++){

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

			tx_low_event_log_entry->unique_seq				  = tx_mpdu->unique_seq;
			tx_low_event_log_entry->transmission_count        = i+1;
			tx_low_event_log_entry->timestamp_send            = (u64)(  tx_mpdu->timestamp_create + (u64)(tx_mpdu->delay_accept) + (u64)(tx_low_details[i].tx_start_delta) + (u64)ts_old);
			tx_low_event_log_entry->chan_num                  = tx_low_details[i].chan_num;
			tx_low_event_log_entry->num_slots				  = tx_low_details[i].num_slots;
			tx_low_event_log_entry->cw						  = tx_low_details[i].cw;
			memcpy((&((tx_low_entry*)tx_low_event_log_entry)->phy_params), &(tx_low_details[i].phy_params), sizeof(phy_tx_params));
			tx_low_event_log_entry->length                    = tx_mpdu->length;
			tx_low_event_log_entry->pkt_type				  = pkt_type;
			wlan_mac_high_cdma_finish_transfer();

			if(i==0){
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

		ts_old += tx_low_details[i].tx_start_delta;
	}

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
		tx_high_event_log_entry->power                    = tx_mpdu->params.phy.power;
		tx_high_event_log_entry->length                   = tx_mpdu->length;
		tx_high_event_log_entry->rate                     = tx_mpdu->params.phy.rate;
		tx_high_event_log_entry->chan_num				 = mac_param_chan;
		tx_high_event_log_entry->pkt_type				 = wlan_mac_high_pkt_type(mpdu,tx_mpdu->length);
		tx_high_event_log_entry->num_tx                   = tx_mpdu->num_tx;
		tx_high_event_log_entry->timestamp_create         = tx_mpdu->timestamp_create;
		tx_high_event_log_entry->delay_accept             = tx_mpdu->delay_accept;
		tx_high_event_log_entry->delay_done               = tx_mpdu->delay_done;
		tx_high_event_log_entry->ant_mode				 = tx_mpdu->params.phy.antenna_mode;

#ifdef _DEBUG_
		xil_printf("TX HIGH : %8d    %8d    %8d    %8d    %8d\n", transfer_len, MIN_MAC_PAYLOAD_LOG_LEN, total_payload_len, extra_payload, payload_log_len);
        print_buf((u8 *)((u32)tx_high_event_log_entry - 8), sizeof(tx_high_entry) + extra_payload + 12);
#endif
	}

	if(tx_mpdu->AID != 0){
		station_info_entry = wlan_mac_high_find_station_info_AID(&association_table, tx_mpdu->AID);
		if(station_info_entry != NULL){
			station = (station_info*)(station_info_entry->data);
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
	}
}




void attempt_association(){
	//It is assumed that the global "access_point" has a valid BSSID (MAC Address).
	//This function should only be called after selecting an access point through active scan

	static u8      curr_try = 0;
	u16            tx_length;
	tx_queue_element*	curr_tx_queue_element;
	tx_queue_buffer* 	curr_tx_queue_buffer;

	switch(association_state){

		case 1:
			//Initial start state, unauthenticated, unassociated
			//Checkout 1 element from the queue;
			curr_try = 0;
		break;

		case 2:
			//Authenticated, not associated
			curr_try = 0;
			//Checkout 1 element from the queue;
			curr_tx_queue_element = queue_checkout();
			if(curr_tx_queue_element != NULL){ //There was at least 1 free queue element
				curr_tx_queue_buffer = (tx_queue_buffer*)(curr_tx_queue_element->data);

				wlan_mac_high_setup_tx_header( &tx_header_common, ap_addr, ap_addr);

				tx_length = wlan_create_association_req_frame((void*)(curr_tx_queue_buffer->frame), &tx_header_common, (u8)strlen(access_point_ssid), (u8*)access_point_ssid, access_point_num_basic_rates, access_point_basic_rates);

		 		wlan_mac_high_setup_tx_frame_info ( &tx_header_common, curr_tx_queue_element, tx_length, (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO), MANAGEMENT_QID );

		 		curr_tx_queue_buffer->metadata.metadata_type = QUEUE_METADATA_TYPE_TX_PARAMS;
		 		curr_tx_queue_buffer->metadata.metadata_ptr = (u32)(&default_unicast_mgmt_tx_params);

				enqueue_after_tail(MANAGEMENT_QID, curr_tx_queue_element);
				poll_tx_queues();
			}
			if( curr_try < (ASSOCIATION_NUM_TRYS - 1) ){
				wlan_mac_schedule_event(SCHEDULE_COARSE, ASSOCIATION_TIMEOUT_US, (void*)attempt_association);
				curr_try++;
			} else {
				curr_try = 0;
				if( strlen(access_point_ssid) > 0 ) start_active_scan();
			}

		break;

		case 3:
			//Authenticated and associated (Pending RSN Authentication)
			//Not-applicable for current 802.11 Reference Design
			curr_try = 0;
		break;

		case 4:
			//Authenticated and associated
			curr_try = 0;

		break;
	}

	return;
}




void attempt_authentication(){
	//It is assumed that the global "access_point" has a valid BSSID (MAC Address).
	//This function should only be called after selecting an access point through active scan

	static u8      			curr_try = 0;
	u16            			tx_length;
	tx_queue_element*	   	curr_tx_queue_element;
	tx_queue_buffer* 		curr_tx_queue_buffer;

	switch(association_state){

		case 1:
			//Initial start state, unauthenticated, unassociated
			//Checkout 1 element from the queue;
			curr_tx_queue_element = queue_checkout();
			if(curr_tx_queue_element != NULL){ //There was at least 1 free queue element
				curr_tx_queue_buffer = (tx_queue_buffer*)(curr_tx_queue_element->data);

				wlan_mac_high_setup_tx_header( &tx_header_common, ap_addr, ap_addr );

				tx_length = wlan_create_auth_frame((void*)(curr_tx_queue_buffer->frame), &tx_header_common, AUTH_ALGO_OPEN_SYSTEM, AUTH_SEQ_REQ, STATUS_SUCCESS);

		 		wlan_mac_high_setup_tx_frame_info (&tx_header_common, curr_tx_queue_element, tx_length, (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO), MANAGEMENT_QID );

		 		curr_tx_queue_buffer->metadata.metadata_type = QUEUE_METADATA_TYPE_TX_PARAMS;
		 		curr_tx_queue_buffer->metadata.metadata_ptr = (u32)(&default_unicast_mgmt_tx_params);

				enqueue_after_tail(MANAGEMENT_QID, curr_tx_queue_element);
				poll_tx_queues();
			}
			if( curr_try < (AUTHENTICATION_NUM_TRYS - 1) ){
				wlan_mac_schedule_event(SCHEDULE_COARSE, AUTHENTICATION_TIMEOUT_US, (void*)attempt_authentication);
				curr_try++;
			} else {
				curr_try = 0;
				if( strlen(access_point_ssid) > 0 ) start_active_scan();
			}


		break;

		case 2:
			//Authenticated, not associated
			curr_try = 0;
		break;

		case 3:
			//Authenticated and associated (Pending RSN Authentication)
			//Not-applicable for current 802.11 Reference Design
			curr_try = 0;
		break;

		case 4:
			//Authenticated and associated
			curr_try = 0;

		break;

	}

	return;
}


void start_active_scan(){
	//Purge any knowledge of existing APs
	stop_active_scan();
	xil_printf("Starting active scan\n");
	num_ap_list = 0;
	wlan_mac_high_free(ap_list);
	ap_list = NULL;
	association_state = 1;
	active_scan = 1;
	repeated_active_scan_scheduled = 1;
	active_scan_schedule_id = wlan_mac_schedule_event_repeated(SCHEDULE_COARSE, ACTIVE_SCAN_UPDATE_RATE, SCHEDULE_REPEAT_FOREVER, (void*)probe_req_transmit);
	probe_req_transmit();
}

void stop_active_scan(){
	xil_printf("Stopping active scan\n");
	if(repeated_active_scan_scheduled) wlan_mac_remove_schedule(SCHEDULE_COARSE, active_scan_schedule_id);
	active_scan = 0;
	repeated_active_scan_scheduled = 0;
}

void testtest(){
	xil_printf("testtest\n");

}

void probe_req_transmit(){
	u32 i;

	static u8 curr_channel_index = 0;
	u16 tx_length;
	tx_queue_element*	curr_tx_queue_element;
	tx_queue_buffer* 	curr_tx_queue_buffer;

	mac_param_chan = curr_channel_index + 1; //+1 is to shift [0,10] index to [1,11] channel number

	//Send a message to other processor to tell it to switch channels
	wlan_mac_high_set_channel( mac_param_chan );

	//Send probe request

	//xil_printf("Probe Req SSID: %s, Len: %d\n",access_point_ssid, strlen(access_point_ssid));

	for(i = 0; i<0; i++){ //NUM_PROBE_REQ
		//Checkout 1 element from the queue;
		curr_tx_queue_element = queue_checkout();
		if(curr_tx_queue_element != NULL){ //There was at least 1 free queue element
			curr_tx_queue_buffer = (tx_queue_buffer*)(curr_tx_queue_element->data);

			wlan_mac_high_setup_tx_header( &tx_header_common, (u8 *)bcast_addr, (u8 *)bcast_addr );

			tx_length = wlan_create_probe_req_frame((void*)(curr_tx_queue_buffer->frame),&tx_header_common, strlen(access_point_ssid), (u8*)access_point_ssid, mac_param_chan);

			wlan_mac_high_setup_tx_frame_info ( &tx_header_common, curr_tx_queue_element, tx_length, 0, MANAGEMENT_QID );

			curr_tx_queue_buffer->metadata.metadata_type = QUEUE_METADATA_TYPE_TX_PARAMS;
			curr_tx_queue_buffer->metadata.metadata_ptr = (u32)(&default_multicast_mgmt_tx_params);

			enqueue_after_tail(MANAGEMENT_QID, curr_tx_queue_element);
			poll_tx_queues();
		}
	}

	curr_channel_index = (curr_channel_index+1)%11;

	if(curr_channel_index > 0){
	    wlan_mac_schedule_event(SCHEDULE_COARSE, ACTIVE_SCAN_DWELL, (void*)probe_req_transmit);
	} else {
		wlan_mac_schedule_event(SCHEDULE_COARSE, ACTIVE_SCAN_DWELL, (void*)print_ap_list);
	}

}



int ethernet_receive(tx_queue_element* curr_tx_queue_element, u8* eth_dest, u8* eth_src, u16 tx_length){
	tx_queue_buffer* 	curr_tx_queue_buffer;
	station_info* 		ap_station_info;

	if(association_table.length == 1){
		ap_station_info = (station_info*)((association_table.first)->data);
		//Receives the pre-encapsulated Ethernet frames
		curr_tx_queue_buffer = (tx_queue_buffer*)(curr_tx_queue_element->data);

		wlan_mac_high_setup_tx_header( &tx_header_common, ap_station_info->addr,(u8*)(&(eth_dest[0])));

		wlan_create_data_frame((void*)(curr_tx_queue_buffer->frame), &tx_header_common, MAC_FRAME_CTRL2_FLAG_TO_DS);

		if(queue_num_queued(UNICAST_QID) < max_queue_size){
			wlan_mac_high_setup_tx_frame_info ( &tx_header_common, curr_tx_queue_element, tx_length, (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO), UNICAST_QID );

			curr_tx_queue_buffer->metadata.metadata_type = QUEUE_METADATA_TYPE_STATION_INFO;
			curr_tx_queue_buffer->metadata.metadata_ptr = (u32)ap_station_info;
			curr_tx_queue_buffer->frame_info.AID = ap_station_info->AID;

			enqueue_after_tail(UNICAST_QID, curr_tx_queue_element);
			poll_tx_queues();
		} else {
			return 0;
		}
		return 1;
	} else {
		//STA is not currently associated, so we won't send any eth frames
		return 0;
	}
}




void mpdu_rx_process(void* pkt_buf_addr, u8 rate, u16 length) {
	u32 i;
	void * mpdu = pkt_buf_addr + PHY_RX_PKT_BUF_MPDU_OFFSET;
	u8* mpdu_ptr_u8 = (u8*)mpdu;
	station_info* associated_station = NULL;
	dl_entry*	  associated_station_entry;

	statistics_txrx* station_stats = NULL;
	ap_info* curr_ap_info = NULL;
	char* ssid;

	mac_header_80211* rx_80211_header;
	rx_80211_header = (mac_header_80211*)((void *)mpdu_ptr_u8);
	u16 rx_seq;

	rx_common_entry* rx_event_log_entry;

	rx_frame_info* mpdu_info = (rx_frame_info*)pkt_buf_addr;

	u8 is_associated = 0;

	typedef enum {PAYLOAD_FIRST, CHAN_EST_FIRST} copy_order_t;
	copy_order_t copy_order;
	u32 payload_log_len;
	u32 extra_payload;
	u32 transfer_len;
	u8 unicast_to_me, to_multicast;
	station_info_entry* associated_station_log_entry;

	//*************
	// Event logging
	//*************

	// Determine length required for log entry
	payload_log_len = min( max((1 + ( ( ( length ) - 1) / 4) )*4 , sizeof(mac_header_80211)) , mac_payload_log_len );
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

		associated_station_entry = wlan_mac_high_find_station_info_ADDR(&association_table, (rx_80211_header->address_2));

		if(associated_station_entry != NULL) {
			associated_station = (station_info*)(associated_station_entry->data);
			is_associated = 1;
			station_stats = associated_station->stats;
			rx_seq = ((rx_80211_header->sequence_control)>>4)&0xFFF;
			//Check if duplicate
			associated_station->rx.last_timestamp = get_usec_timestamp();
			associated_station->rx.last_power = mpdu_info->rx_power;
			associated_station->rx.last_rate = mpdu_info->rate;

			//xil_printf("%d ? %d\n", access_point.rx.last_seq, rx_seq);

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
			if(is_associated){
				if((rx_80211_header->frame_control_2) & MAC_FRAME_CTRL2_FLAG_FROM_DS) {
					//MPDU is flagged as destined to the DS - send it for de-encapsulation and Ethernet Tx (if appropriate)
					wlan_mpdu_eth_send(mpdu,length);
				}
			}
			break;

			case (MAC_FRAME_CTRL1_SUBTYPE_ASSOC_RESP): //Association response
				if(association_state == 2){
					mpdu_ptr_u8 += sizeof(mac_header_80211);

					if(((association_response_frame*)mpdu_ptr_u8)->status_code == STATUS_SUCCESS){
						association_state = 4;

						if(association_table.length > 0){

							associated_station_log_entry = (station_info_entry*)wlan_exp_log_create_entry( ENTRY_TYPE_STATION_INFO, sizeof(station_info_entry));
							if(associated_station_log_entry != NULL){
								associated_station_log_entry->timestamp = get_usec_timestamp();
								memcpy((u8*)(&(associated_station_log_entry->info)),(u8*)((association_table.first)->data), sizeof(station_info_base) );
								associated_station_log_entry->info.AID = 0;
							}

							wlan_mac_high_remove_association(&association_table, &statistics_table, associated_station->addr);
						}

						associated_station = wlan_mac_high_add_association(&association_table, &statistics_table, rx_80211_header->address_2, (((association_response_frame*)mpdu_ptr_u8)->association_id)&~0xC000);

						associated_station_log_entry = (station_info_entry*)wlan_exp_log_create_entry( ENTRY_TYPE_STATION_INFO, sizeof(station_info_entry));
						if(associated_station_log_entry != NULL){
							associated_station_log_entry->timestamp = get_usec_timestamp();
							memcpy((u8*)(&(associated_station_log_entry->info)),(u8*)(associated_station), sizeof(station_info_base) );
						}


						wlan_mac_high_write_hex_display(associated_station->AID);

						memcpy(&(associated_station->tx),&default_unicast_data_tx_params, sizeof(tx_params));

						xil_printf("Association succeeded\n");
					} else {
						association_state = -1;
						xil_printf("Association failed, reason code %d\n", ((association_response_frame*)mpdu_ptr_u8)->status_code);
					}
				}

			break;

			case (MAC_FRAME_CTRL1_SUBTYPE_AUTH): //Authentication
					if(association_state == 1 && wlan_addr_eq(rx_80211_header->address_3, ap_addr) && wlan_addr_eq(rx_80211_header->address_1, wlan_mac_addr)) {
						mpdu_ptr_u8 += sizeof(mac_header_80211);
						switch(((authentication_frame*)mpdu_ptr_u8)->auth_algorithm){
							case AUTH_ALGO_OPEN_SYSTEM:
								if(((authentication_frame*)mpdu_ptr_u8)->auth_sequence == AUTH_SEQ_RESP){//This is an auth response
									if(((authentication_frame*)mpdu_ptr_u8)->status_code == STATUS_SUCCESS){
										//AP is letting us authenticate
										association_state = 2;
										attempt_association();
									}
									// Finish function
									goto mpdu_rx_process_end;
								}
							break;
						}
					}

			break;

			case (MAC_FRAME_CTRL1_SUBTYPE_DEAUTH): //Deauthentication
					if(wlan_addr_eq(rx_80211_header->address_1, wlan_mac_addr) && (wlan_mac_high_find_station_info_ADDR(&association_table, rx_80211_header->address_2) != NULL)){

						associated_station_log_entry = (station_info_entry*)wlan_exp_log_create_entry( ENTRY_TYPE_STATION_INFO, sizeof(station_info_entry));
						if(associated_station_log_entry != NULL){
							associated_station_log_entry->timestamp = get_usec_timestamp();
							memcpy((u8*)(&(associated_station_log_entry->info)),(u8*)((association_table.first)->data), sizeof(station_info_base) );
							associated_station_log_entry->info.AID = 0;
						}

						wlan_mac_high_remove_association(&association_table, &statistics_table, rx_80211_header->address_2);
						purge_queue(UNICAST_QID);
						wlan_mac_high_write_hex_display(0);
						if( strlen(access_point_ssid) > 0 && association_state == 4){
							start_active_scan();
						}
					}
			break;

			case (MAC_FRAME_CTRL1_SUBTYPE_BEACON): //Beacon Packet
			case (MAC_FRAME_CTRL1_SUBTYPE_PROBE_RESP): //Probe Response Packet

					if(active_scan){

					for (i=0;i<num_ap_list;i++){

						if(wlan_addr_eq(ap_list[i].bssid, rx_80211_header->address_3)){
							curr_ap_info = &(ap_list[i]);
							//xil_printf("     Matched at 0x%08x\n", curr_ap_info);
							break;
						}
					}

					if(curr_ap_info == NULL){

						if(ap_list == NULL){
							ap_list = wlan_mac_high_malloc(sizeof(ap_info)*(num_ap_list+1));
						} else {
							ap_list = wlan_mac_high_realloc(ap_list, sizeof(ap_info)*(num_ap_list+1));
						}

						if(ap_list != NULL){
							num_ap_list++;
							curr_ap_info = &(ap_list[num_ap_list-1]);
						} else {
							xil_printf("Reallocation of ap_list failed\n");
							// Finish function
							goto mpdu_rx_process_end;
						}

					}

					curr_ap_info->rx_power = mpdu_info->rx_power;
					curr_ap_info->num_basic_rates = 0;

					//Copy BSSID into ap_info struct
					memcpy(curr_ap_info->bssid, rx_80211_header->address_3,6);

					mpdu_ptr_u8 += sizeof(mac_header_80211);
					if((((beacon_probe_frame*)mpdu_ptr_u8)->capabilities)&CAPABILITIES_PRIVACY){
						curr_ap_info->private = 1;
					} else {
						curr_ap_info->private = 0;
					}

					mpdu_ptr_u8 += sizeof(beacon_probe_frame);
					//xil_printf("\n");
					while(((u32)mpdu_ptr_u8 -  (u32)mpdu)<= length){ //Loop through tagged parameters
						switch(mpdu_ptr_u8[0]){ //What kind of tag is this?
							case TAG_SSID_PARAMS: //SSID parameter set
								ssid = (char*)(&(mpdu_ptr_u8[2]));


								memcpy(curr_ap_info->ssid, ssid ,min(mpdu_ptr_u8[1],SSID_LEN_MAX-1));
								//Terminate the string
								(curr_ap_info->ssid)[min(mpdu_ptr_u8[1],SSID_LEN_MAX-1)] = 0;

							break;
							case TAG_SUPPORTED_RATES: //Supported rates
								for(i=0;i < mpdu_ptr_u8[1]; i++){
									if(mpdu_ptr_u8[2+i]&RATE_BASIC){
										//This is a basic rate. It is required by the AP in order to associate.
										if((curr_ap_info->num_basic_rates) < NUM_BASIC_RATES_MAX){

											if(wlan_mac_high_valid_tagged_rate(mpdu_ptr_u8[2+i])){

												(curr_ap_info->basic_rates)[(curr_ap_info->num_basic_rates)] = mpdu_ptr_u8[2+i];
												(curr_ap_info->num_basic_rates)++;
											} else {

											}
										} else {
										}
									}
								}
							break;
							case TAG_EXT_SUPPORTED_RATES: //Extended supported rates
								for(i=0;i < mpdu_ptr_u8[1]; i++){
										if(mpdu_ptr_u8[2+i]&RATE_BASIC){
											//This is a basic rate. It is required by the AP in order to associate.
											if((curr_ap_info->num_basic_rates) < NUM_BASIC_RATES_MAX){

												if(wlan_mac_high_valid_tagged_rate(mpdu_ptr_u8[2+i])){
												//	xil_printf("Basic rate #%d: 0x%x\n", (curr_ap_info->num_basic_rates), mpdu_ptr_u8[2+i]);

													(curr_ap_info->basic_rates)[(curr_ap_info->num_basic_rates)] = mpdu_ptr_u8[2+i];
													(curr_ap_info->num_basic_rates)++;
												} else {
													//xil_printf("Invalid tagged rate. ignoring.");
												}
											} else {
												//xil_printf("Error: too many rates were flagged as basic. ignoring.");
											}
										}
									}

							break;
							case TAG_DS_PARAMS: //DS Parameter set (e.g. channel)
								curr_ap_info->chan = mpdu_ptr_u8[2];
							break;
						}
						mpdu_ptr_u8 += mpdu_ptr_u8[1]+2; //Move up to the next tag
					}

				}

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

	if ((rx_event_log_entry != NULL) && ((rx_event_log_entry->rate) != WLAN_MAC_RATE_1M)) {
		wn_transmit_log_entry((void *)rx_event_log_entry);
	}
}




void ltg_event(u32 id, void* callback_arg){

	tx_queue_element* 	curr_tx_queue_element;
	tx_queue_buffer* 	curr_tx_queue_buffer;

	u32 tx_length;
	u8* mpdu_ptr_u8;
	llc_header* llc_hdr;
	u32 payload_length = 0;
	u8* addr_da;
	station_info* ap_station_info;

	switch(((ltg_pyld_hdr*)callback_arg)->type){
		case LTG_PYLD_TYPE_FIXED:
			addr_da = ((ltg_pyld_fixed*)callback_arg)->addr_da;
			payload_length = ((ltg_pyld_fixed*)callback_arg)->length;
		break;
		case LTG_PYLD_TYPE_UNIFORM_RAND:
			addr_da = ((ltg_pyld_uniform_rand*)callback_arg)->addr_da;
			payload_length = (rand()%(((ltg_pyld_uniform_rand*)(callback_arg))->max_length - ((ltg_pyld_uniform_rand*)(callback_arg))->min_length))+((ltg_pyld_uniform_rand*)(callback_arg))->min_length;
		break;
		default:
			addr_da = 0;
		break;
	}
	if((association_table.length > 0)){

		ap_station_info = (station_info*)((association_table.first)->data);

		//Send a Data packet to AP
		//Checkout 1 element from the queue;

		if(queue_num_queued(UNICAST_QID) < max_queue_size){

			curr_tx_queue_element = queue_checkout();

			if(curr_tx_queue_element != NULL){ //There was at least 1 free queue element
				curr_tx_queue_buffer = (tx_queue_buffer*)(curr_tx_queue_element->data);

				wlan_mac_high_setup_tx_header( &tx_header_common, ap_station_info->addr, addr_da );

				mpdu_ptr_u8 = (u8*)(curr_tx_queue_buffer->frame);
				tx_length = wlan_create_data_frame((void*)mpdu_ptr_u8, &tx_header_common, MAC_FRAME_CTRL2_FLAG_TO_DS);

				mpdu_ptr_u8 += sizeof(mac_header_80211);
				llc_hdr = (llc_header*)(mpdu_ptr_u8);

				//Prepare the MPDU LLC header
				llc_hdr->dsap = LLC_SNAP;
				llc_hdr->ssap = LLC_SNAP;
				llc_hdr->control_field = LLC_CNTRL_UNNUMBERED;
				bzero((void *)(llc_hdr->org_code), 3); //Org Code 0x000000: Encapsulated Ethernet
				llc_hdr->type = LLC_TYPE_WLAN_LTG;

				tx_length += max(payload_length, sizeof(llc_header));

				wlan_mac_high_setup_tx_frame_info (&tx_header_common, curr_tx_queue_element, tx_length,(TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO), UNICAST_QID );

				curr_tx_queue_buffer->metadata.metadata_type = QUEUE_METADATA_TYPE_STATION_INFO;
				curr_tx_queue_buffer->metadata.metadata_ptr = (u32)ap_station_info;
				curr_tx_queue_buffer->frame_info.AID = ap_station_info->AID;

				enqueue_after_tail(UNICAST_QID, curr_tx_queue_element);
				poll_tx_queues();
			}
		}
	}

}

void print_ap_list(){
	u32 i,j;
	char str[4];
	u16 ap_sel;

	uart_mode = UART_MODE_AP_LIST;
	//active_scan = 0;
	pause_queue = 0;

	//Revert to the previous channel that we were on prior to the active scan
	mac_param_chan = mac_param_chan_save;
	wlan_mac_high_set_channel( mac_param_chan );

//	xil_printf("\f");

	xil_printf("************************ AP List *************************\n");

	for(i=0; i<num_ap_list; i++){
		xil_printf("[%d] SSID:     %s ", i, ap_list[i].ssid);
		if(ap_list[i].private == 1){
			xil_printf("(*)\n");
		} else {
			xil_printf("\n");
		}

		xil_printf("    BSSID:         %02x-%02x-%02x-%02x-%02x-%02x\n", ap_list[i].bssid[0],ap_list[i].bssid[1],ap_list[i].bssid[2],ap_list[i].bssid[3],ap_list[i].bssid[4],ap_list[i].bssid[5]);
		xil_printf("    Channel:       %d\n",ap_list[i].chan);
		xil_printf("    Rx Power:      %d dBm\n",ap_list[i].rx_power);
		xil_printf("    Basic Rates:   ");
		for(j = 0; j < (ap_list[i].num_basic_rates); j++ ){
			wlan_mac_high_tagged_rate_to_readable_rate(ap_list[i].basic_rates[j], str);
			xil_printf("%s, ",str);
		}
		xil_printf("\b\b \n");

	}


	if(strlen(access_point_ssid) == 0){
		xil_printf("\n(*) Private Network (not supported)\n");
		xil_printf("\n To join a network, type the number next to the SSID that\n");
		xil_printf("you want to join and press enter. Otherwise, press Esc to return\n");
		xil_printf("AP Selection: ");
	} else {
		for(i=0; i<num_ap_list; i++){
			if(strcmp(access_point_ssid,ap_list[i].ssid) == 0){
				ap_sel = i;
				if( ap_list[ap_sel].private == 0) {
					mac_param_chan = ap_list[ap_sel].chan;

					//Send a message to other processor to tell it to switch channels
					wlan_mac_high_set_channel( mac_param_chan );

					xil_printf("\nAttempting to join %s\n", ap_list[ap_sel].ssid);
					memcpy(ap_addr, ap_list[ap_sel].bssid, 6);

					access_point_ssid = wlan_mac_high_realloc(access_point_ssid, strlen(ap_list[ap_sel].ssid)+1);
					strcpy(access_point_ssid,ap_list[ap_sel].ssid);

					access_point_num_basic_rates = ap_list[ap_sel].num_basic_rates;
					memcpy(access_point_basic_rates, ap_list[ap_sel].basic_rates,access_point_num_basic_rates);

					stop_active_scan();
					association_state = 1;
					attempt_authentication();
					return;
				} else {
					xil_printf("AP with SSID %s is private\n", access_point_ssid);
					return;
				}
			}
		}
		xil_printf("Failed to find AP with SSID of %s\n", access_point_ssid);
	}

}

void reset_station_statistics(){
	wlan_mac_high_reset_statistics(&statistics_table);
}


dl_list * get_statistics(){
	return &statistics_table;
}

dl_list * get_station_info_list(){
	return &association_table;
}


/*****************************************************************************/
/**
* Get AP List
*
* This function will populate the buffer with:
*   buffer[0]      = Number of stations
*   buffer[1 .. N] = memcpy of the station information structure
* where N is less than max_words
*
* @param    stations      - Station info pointer
*           num_stations  - Number of stations to copy
*           buffer        - u32 pointer to the buffer to transfer the data
*           max_words     - The maximum number of words in the buffer
*
* @return	Number of words copied in to the buffer
*
* @note     None.
*
******************************************************************************/
int get_ap_list( ap_info * ap_list, u32 num_ap, u32 * buffer, u32 max_words ) {

	unsigned int size;
	unsigned int index;

	index     = 0;

	// Set number of Association entries
	buffer[ index++ ] = num_ap;

	// Get total size (in bytes) of data to be transmitted
	size   = num_ap * sizeof( ap_info );
	// Get total size of data (in words) to be transmitted
	index += size / sizeof( u32 );
    if ( (size > 0 ) && (index < max_words) ) {
        memcpy( &buffer[1], ap_list, size );
    }

#ifdef _DEBUG_
	#ifdef USE_WARPNET_WLAN_EXP
    wlan_exp_print_ap_list( ap_list, num_ap );
	#endif
#endif

	return index;
}


