////////////////////////////////////////////////////////////////////////////////
// File   : wlan_mac_ap.c
// Authors: Patrick Murphy (murphpo [at] mangocomm.com)
//			Chris Hunter (chunter [at] mangocomm.com)
//          Erik Welsh (welsh [at] mangocomm.com)
// License: Copyright 2013, Mango Communications. All rights reserved.
//          Distributed under the Mango Communications Reference Design License
//				See LICENSE.txt included in the design archive or
//				at http://mangocomm.com/802.11/license
////////////////////////////////////////////////////////////////////////////////


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
#include "wlan_mac_ipc.h"
#include "wlan_mac_misc_util.h"
#include "wlan_mac_802_11_defs.h"
#include "wlan_mac_queue.h"
#include "wlan_mac_ltg.h"
#include "wlan_mac_util.h"
#include "wlan_mac_packet_types.h"
#include "wlan_mac_eth_util.h"
#include "wlan_mac_ap.h"
#include "ascii_characters.h"

// WLAN Exp includes
#include "wlan_exp_common.h"
#include "wlan_exp_node.h"
#include "wlan_exp_node_ap.h"
#include "wlan_exp_transport.h"


/*************************** Constant Definitions ****************************/

#define  WLAN_EXP_ETH                  WN_ETH_B
#define  WLAN_EXP_TYPE                 WARPNET_TYPE_80211_BASE + WARPNET_TYPE_80211_AP


#define  WLAN_CHANNEL                  4



/*********************** Global Variable Definitions *************************/



/*************************** Variable Definitions ****************************/

// SSID variables
static char default_AP_SSID[] = "WARP-AP";
char*       access_point_ssid;

// Common TX header for 802.11 packets
mac_header_80211_common tx_header_common;

// Control variables
u8 allow_assoc;
u8 perma_assoc_mode;
u8 default_unicast_rate;
u8 enable_animation;

// Association Table variables
//   The last entry in associations[MAX_ASSOCIATIONS][] is swap space
station_info associations[MAX_ASSOCIATIONS+1];
u32          next_free_assoc_index;
u32			 max_queue_size;

// AP channel
u32 mac_param_chan;

// AP MAC address / Broadcast address
static u8 eeprom_mac_addr[6];
static u8 bcast_addr[6]      = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

u16 ltg_packet_size[MAX_ASSOCIATIONS];

/*************************** Functions Prototypes ****************************/


#ifdef WLAN_USE_UART_MENU

void uart_rx(u8 rxByte);

#else

void uart_rx(u8 rxByte){ };

#endif


void remove_station( unsigned int station_index );




/******************************** Functions **********************************/


int main(){
	u32 i;

	//This function should be executed first. It will zero out memory, and if that
	//memory is used before calling this function, unexpected results may happen.
	wlan_mac_util_init_data();

	xil_printf("\f----- wlan_mac_ap -----\n");
	xil_printf("Compiled %s %s\n", __DATE__, __TIME__);


    // Set Global variables
	perma_assoc_mode     = 0;
	default_unicast_rate = WLAN_MAC_RATE_18M;

	// Initialize the utility library
	wlan_lib_init();
	wlan_mac_util_set_eth_encap_mode(ENCAP_MODE_AP);
	wlan_mac_util_init( WLAN_EXP_TYPE );

	max_queue_size = (queue_total_size()- eth_bd_total_size()) / (next_free_assoc_index+1);


	// Initialize callbacks
	wlan_mac_util_set_eth_rx_callback(       (void*)ethernet_receive);
	wlan_mac_util_set_mpdu_tx_done_callback( (void*)mpdu_transmit_done);
	wlan_mac_util_set_mpdu_rx_callback(      (void*)mpdu_rx_process);
	wlan_mac_util_set_pb_u_callback(         (void*)up_button);
	wlan_mac_util_set_uart_rx_callback(      (void*)uart_rx);
	wlan_mac_util_set_ipc_rx_callback(       (void*)ipc_rx);
	wlan_mac_util_set_check_queue_callback(  (void*)check_tx_queue);

    wlan_mac_ltg_set_callback(               (void*)ltg_event);


    // Initialize interrupts
	interrupt_init();


	// Initialize Association Table
	next_free_assoc_index = 0;

	bzero(&(associations[0]),sizeof(station_info)*(MAX_ASSOCIATIONS+1));

	for(i=0;i<MAX_ASSOCIATIONS;i++){
		associations[i].AID = (1+i); //7.3.1.8 of 802.11-2007
		memset((void*)(&(associations[i].addr[0])), 0xFF,6);
		associations[i].seq = 0; //seq
		// Set default LTG parameters.
		// These are changed by either UART interaction or WARPnet
		ltg_packet_size[i] = 1470;
	}


    // Wait for CPU Low to initialize
	while( is_cpu_low_initialized() == 0 ){
		xil_printf("waiting on CPU_LOW to boot\n");
	};


	// CPU Low will pass HW information to CPU High as part of the boot process
	//   - Get necessary HW information
	memcpy((void*) &(eeprom_mac_addr[0]), (void*) get_eeprom_mac_addr(), 6);


    // Set Header information
	tx_header_common.address_2 = &(eeprom_mac_addr[0]);
	tx_header_common.seq_num   = 0;


    // Initialize hex display
	write_hex_display(0);


	// Set up channel
	mac_param_chan = WLAN_CHANNEL;
	set_mac_channel( mac_param_chan );


	// Set SSID
	access_point_ssid = malloc(strlen(default_AP_SSID)+1);
	strcpy(access_point_ssid,default_AP_SSID);


    // Schedule all events
	wlan_mac_schedule_event(SCHEDULE_COARSE, BEACON_INTERVAL_US, (void*)beacon_transmit);

	wlan_mac_schedule_event(SCHEDULE_COARSE, ASSOCIATION_CHECK_INTERVAL_US, (void*)association_timestamp_check);

	enable_animation = 1;
	wlan_mac_schedule_event(SCHEDULE_COARSE, ANIMATION_RATE_US, (void*)animate_hex);

	enable_associations();
	perma_assoc_mode = 1;   // By default, associations are allowed any time.
	wlan_mac_schedule_event(SCHEDULE_COARSE, ASSOCIATION_ALLOW_INTERVAL_US, (void*)disable_associations);


	// Print AP information to the terminal
    xil_printf("WLAN MAC AP boot complete: \n");
    xil_printf("  SSID    : %s \n", access_point_ssid);
    xil_printf("  Channel : %d \n", mac_param_chan);
	xil_printf("  MAC Addr: %x-%x-%x-%x-%x-%x\n\n",eeprom_mac_addr[0],eeprom_mac_addr[1],eeprom_mac_addr[2],eeprom_mac_addr[3],eeprom_mac_addr[4],eeprom_mac_addr[5]);


#ifdef WLAN_USE_UART_MENU
	xil_printf("\nAt any time, press the Esc key in your terminal to access the AP menu\n");
#endif

#ifdef USE_WARPNET_WLAN_EXP
	// Set AP processing callbacks
	node_set_process_callback( (void *)wlan_exp_node_ap_processCmd );
#endif

	while(1){
		//The design is entirely interrupt based. When no events need to be processed, the processor
		//will spin in this loop until an interrupt happens

#ifdef USE_WARPNET_WLAN_EXP
		transport_poll( WLAN_EXP_ETH );
#endif
	}
	return -1;
}



void check_tx_queue(){

	static u32 station_index = 0;
	u32 i;
	if( is_cpu_low_ready() ){
		for(i = 0; i < (next_free_assoc_index+1); i++){
			station_index = (station_index+1)%(next_free_assoc_index+1);

			if(station_index == next_free_assoc_index){
				//Check Broadcast Queue
				if(wlan_mac_poll_tx_queue(0)){
					return;
				}
			} else {
				//Check Station Queue
				if(wlan_mac_poll_tx_queue(associations[station_index].AID)){
					return;
				}
			}
		}

	}
}



void mpdu_transmit_done(tx_frame_info* tx_mpdu){
	u32 i;
	tx_event* tx_event_log_entry;

	void * mpdu = (void*)tx_mpdu + PHY_RX_PKT_BUF_MPDU_OFFSET;
	u8* mpdu_ptr_u8 = (u8*)mpdu;
	mac_header_80211* tx_80211_header;
	tx_80211_header = (mac_header_80211*)((void *)mpdu_ptr_u8);



	tx_event_log_entry = get_curr_tx_log();

	if(tx_event_log_entry != NULL){
		tx_event_log_entry->state = tx_mpdu->state;
		tx_event_log_entry->AID = 0;
		tx_event_log_entry->power = 0; //TODO
		tx_event_log_entry->length = tx_mpdu->length;
		tx_event_log_entry->rate = tx_mpdu->rate;
		tx_event_log_entry->mac_type = tx_80211_header->frame_control_1;
		tx_event_log_entry->seq = ((tx_80211_header->sequence_control)>>4)&0xFFF;
		tx_event_log_entry->retry_count = tx_mpdu->retry_count;

		increment_log();
	}


	if(tx_mpdu->AID != 0){
		for(i=0; i<next_free_assoc_index; i++){
			if( (associations[i].AID) == (tx_mpdu->AID) ) {

				if(tx_event_log_entry != NULL) tx_event_log_entry->AID = associations[i].AID;

				//Process this TX MPDU DONE event to update any statistics used in rate adaptation
				wlan_mac_util_process_tx_done(tx_mpdu, &(associations[i]));
				break;
			}
		}
	}
}




void up_button(){
	if(allow_assoc == 0){
		//AP is currently not allowing any associations to take place
		enable_animation = 1;
		wlan_mac_schedule_event(SCHEDULE_COARSE,ANIMATION_RATE_US, (void*)animate_hex);
		enable_associations();
		wlan_mac_schedule_event(SCHEDULE_COARSE,ASSOCIATION_ALLOW_INTERVAL_US, (void*)disable_associations);
	} else if(perma_assoc_mode == 0){
		//AP is currently allowing associations, but only for the small allow window.
		//Go into permanent allow association mode.
		perma_assoc_mode = 1;
		xil_printf("Allowing associations indefinitely\n");
	} else {
		//AP is permanently allowing associations. Toggle everything off.
		perma_assoc_mode = 0;
		disable_associations();
	}
}



void ltg_event(u32 id){
	u32 i;
	packet_bd_list checkout;
	packet_bd* tx_queue;
	u32 tx_length;
	u8* mpdu_ptr_u8;
	llc_header* llc_hdr;

	for(i=0; i < next_free_assoc_index; i++){
		if((u32)(associations[i].AID) == id){

			//We implement a soft limit on the size of the queue allowed for any
			//given station. This avoids the scenario where multiple backlogged
			//LTG flows favor a single user and starve everyone else.
			if(queue_num_queued(associations[i].AID) < max_queue_size){
				//Send a Data packet to this station
				//Checkout 1 element from the queue;
				queue_checkout(&checkout,1);

				if(checkout.length == 1){ //There was at least 1 free queue element
					tx_queue = checkout.first;

					setup_tx_header( &tx_header_common, associations[i].addr, eeprom_mac_addr );

					mpdu_ptr_u8 = (u8*)((tx_packet_buffer*)(tx_queue->buf_ptr))->frame;
					tx_length = wlan_create_data_frame((void*)((tx_packet_buffer*)(tx_queue->buf_ptr))->frame, &tx_header_common, MAC_FRAME_CTRL2_FLAG_FROM_DS);

					mpdu_ptr_u8 += sizeof(mac_header_80211);
					llc_hdr = (llc_header*)(mpdu_ptr_u8);

					//Prepare the MPDU LLC header
					llc_hdr->dsap = LLC_SNAP;
					llc_hdr->ssap = LLC_SNAP;
					llc_hdr->control_field = LLC_CNTRL_UNNUMBERED;
					bzero((void *)(llc_hdr->org_code), 3); //Org Code 0x000000: Encapsulated Ethernet
					llc_hdr->type = LLC_TYPE_CUSTOM;

					tx_length = ltg_packet_size[i];

					setup_tx_queue ( tx_queue, (void*)&(associations[i]), tx_length, MAX_RETRY,
									 (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO) );

					enqueue_after_end(associations[i].AID, &checkout);
					check_tx_queue();
				}

			}

		}
	}
}



int ethernet_receive(packet_bd_list* tx_queue_list, u8* eth_dest, u8* eth_src, u16 tx_length){
	//Receives the pre-encapsulated Ethernet frames

	packet_bd* tx_queue = tx_queue_list->first;

	u32 i;
	u8 is_associated = 0;

	setup_tx_header( &tx_header_common, (u8*)(&(eth_dest[0])), (u8*)(&(eth_src[0])) );

	wlan_create_data_frame((void*)((tx_packet_buffer*)(tx_queue->buf_ptr))->frame, &tx_header_common, MAC_FRAME_CTRL2_FLAG_FROM_DS);

	if(wlan_addr_eq(bcast_addr, eth_dest)){
		if(queue_num_queued(0) < max_queue_size){
			setup_tx_queue ( tx_queue, NULL, tx_length, 0, 0 );

			enqueue_after_end(0, tx_queue_list);
			check_tx_queue();
		} else {
			return 0;
		}

	} else {
		//Check associations
		//Is this packet meant for a station we are associated with?
		for(i=0; i < next_free_assoc_index; i++) {
			if(wlan_addr_eq(associations[i].addr, eth_dest)) {
				is_associated = 1;
				break;
			}
		}
		if(is_associated) {
			if(queue_num_queued(associations[i].AID) < max_queue_size){
				setup_tx_queue ( tx_queue, (void*)&(associations[i]), tx_length, MAX_RETRY,
								 (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO) );

				enqueue_after_end(associations[i].AID, tx_queue_list);
				check_tx_queue();
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
 	packet_bd_list checkout;
 	packet_bd*	tx_queue;

 	//Checkout 1 element from the queue;
 	queue_checkout(&checkout,1);

 	if(checkout.length == 1){ //There was at least 1 free queue element
 		tx_queue = checkout.first;

 		setup_tx_header( &tx_header_common, bcast_addr, eeprom_mac_addr );

        tx_length = wlan_create_beacon_frame((void*)((tx_packet_buffer*)(tx_queue->buf_ptr))->frame,&tx_header_common, BEACON_INTERVAL_MS, strlen(access_point_ssid), (u8*)access_point_ssid, mac_param_chan);

 		setup_tx_queue ( tx_queue, NULL, tx_length, 0, TX_MPDU_FLAGS_FILL_TIMESTAMP );

 		enqueue_after_end(0, &checkout);
 		check_tx_queue();
 	}

 	//Schedule the next beacon transmission
 	wlan_mac_schedule_event(SCHEDULE_COARSE,BEACON_INTERVAL_US, (void*)beacon_transmit);

 	return;
}



void association_timestamp_check() {

	u32 i, num_queued;
	u64 time_since_last_rx;
	packet_bd_list checkout,dequeue;
	packet_bd* tx_queue;
	u32 tx_length;

	for(i=0; i < next_free_assoc_index; i++) {

		time_since_last_rx = (get_usec_timestamp() - associations[i].rx_timestamp);
		if(time_since_last_rx > ASSOCIATION_TIMEOUT_US){
			//Send De-authentication

		 	//Checkout 1 element from the queue;
		 	queue_checkout(&checkout,1);

		 	if(checkout.length == 1){ //There was at least 1 free queue element
		 		tx_queue = checkout.first;

		 		setup_tx_header( &tx_header_common, associations[i].addr, eeprom_mac_addr );

		 		tx_length = wlan_create_deauth_frame((void*)((tx_packet_buffer*)(tx_queue->buf_ptr))->frame, &tx_header_common, DEAUTH_REASON_INACTIVITY);

		 		setup_tx_queue ( tx_queue, (void*)&(associations[i]), tx_length, MAX_RETRY,
		 				         (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO) );

		 		enqueue_after_end(associations[i].AID, &checkout);
		 		check_tx_queue();

		 		//Purge any packets in the queue meant for this node
				num_queued = queue_num_queued(associations[i].AID);
				if(num_queued>0){
					xil_printf("purging %d packets from queue for AID %d\n",num_queued,associations[i].AID);
					dequeue_from_beginning(&dequeue, associations[i].AID,1);
					queue_checkin(&dequeue);
				}

				//Remove this STA from association list
				remove_station( i );
				xil_printf("\n\nDisassociation due to inactivity:\n");
				print_associations();
			}
		}
	}

	wlan_mac_schedule_event(SCHEDULE_COARSE,ASSOCIATION_CHECK_INTERVAL_US, (void*)association_timestamp_check);
	return;
}




void mpdu_rx_process(void* pkt_buf_addr, u8 rate, u16 length) {
	void * mpdu = pkt_buf_addr + PHY_RX_PKT_BUF_MPDU_OFFSET;
	u8* mpdu_ptr_u8 = (u8*)mpdu;
	u16 tx_length;
	u8 send_response, allow_association, allow_disassociation, new_association;
	mac_header_80211* rx_80211_header;
	rx_80211_header = (mac_header_80211*)((void *)mpdu_ptr_u8);
	u16 rx_seq;
	packet_bd_list checkout;
	packet_bd*	tx_queue;
	station_info* associated_station;
	u8 eth_send;

	rx_event* rx_event_log_entry;

	rx_frame_info* mpdu_info = (rx_frame_info*)pkt_buf_addr;

	u32 i;
	u8 is_associated = 0;
	new_association = 0;

	rx_event_log_entry = get_curr_rx_log();

	if(rx_event_log_entry != NULL){
			rx_event_log_entry->state = mpdu_info->state;
			rx_event_log_entry->AID = 0;
			rx_event_log_entry->power = mpdu_info->rx_power;
			rx_event_log_entry->length = mpdu_info->length;
			rx_event_log_entry->rate = mpdu_info->rate;
			rx_event_log_entry->mac_type = rx_80211_header->frame_control_1;
			rx_event_log_entry->seq = ((rx_80211_header->sequence_control)>>4)&0xFFF;
			rx_event_log_entry->flags = 0; //TODO: fill in with retry flag, etc

			increment_log();
	}

	for(i=0; i < next_free_assoc_index; i++) {
		if(wlan_addr_eq(associations[i].addr, (rx_80211_header->address_2))) {
			is_associated = 1;
			associated_station = &(associations[i]);
			rx_seq = ((rx_80211_header->sequence_control)>>4)&0xFFF;
			//Check if duplicate
			associations[i].rx_timestamp = get_usec_timestamp();
			associations[i].last_rx_power = mpdu_info->rx_power;

			if( (associations[i].seq != 0)  && (associations[i].seq == rx_seq) ) {
				//Received seq num matched previously received seq num for this STA; ignore the MPDU and return
				return;

			} else {
				associations[i].seq = rx_seq;
			}

			if(rx_event_log_entry != NULL) rx_event_log_entry->AID = associations[i].AID;

			break;
		}
	}


	switch(rx_80211_header->frame_control_1) {
		case (MAC_FRAME_CTRL1_SUBTYPE_DATA): //Data Packet
			if(is_associated){
				if((rx_80211_header->frame_control_2) & MAC_FRAME_CTRL2_FLAG_TO_DS) {
					//MPDU is flagged as destined to the DS

					(associated_station->num_rx_success)++;
					(associated_station->num_rx_bytes) += mpdu_info->length;

					eth_send = 1;

					if(wlan_addr_eq(rx_80211_header->address_3,bcast_addr)){

						 	queue_checkout(&checkout,1);

						 	if(checkout.length == 1){ //There was at least 1 free queue element
						 		tx_queue = checkout.first;
						 		setup_tx_header( &tx_header_common, bcast_addr, rx_80211_header->address_2);
						 		mpdu_ptr_u8 = (u8*)((tx_packet_buffer*)(tx_queue->buf_ptr))->frame;
								tx_length = wlan_create_data_frame((void*)((tx_packet_buffer*)(tx_queue->buf_ptr))->frame, &tx_header_common, MAC_FRAME_CTRL2_FLAG_FROM_DS);
								mpdu_ptr_u8 += sizeof(mac_header_80211);
								memcpy(mpdu_ptr_u8, (void*)rx_80211_header + sizeof(mac_header_80211), mpdu_info->length - sizeof(mac_header_80211));
						 		setup_tx_queue ( tx_queue, NULL, mpdu_info->length, 0, 0 );
						 		enqueue_after_end(0, &checkout);
						 		check_tx_queue();
						 	}

					} else {
						for(i=0; i < next_free_assoc_index; i++) {
							if(wlan_addr_eq(associations[i].addr, (rx_80211_header->address_3))) {
								queue_checkout(&checkout,1);

								if(checkout.length == 1){ //There was at least 1 free queue element
									tx_queue = checkout.first;
									setup_tx_header( &tx_header_common, rx_80211_header->address_3, rx_80211_header->address_2);
									mpdu_ptr_u8 = (u8*)((tx_packet_buffer*)(tx_queue->buf_ptr))->frame;
									tx_length = wlan_create_data_frame((void*)((tx_packet_buffer*)(tx_queue->buf_ptr))->frame, &tx_header_common, MAC_FRAME_CTRL2_FLAG_FROM_DS);
									mpdu_ptr_u8 += sizeof(mac_header_80211);
									memcpy(mpdu_ptr_u8, (void*)rx_80211_header + sizeof(mac_header_80211), mpdu_info->length - sizeof(mac_header_80211));
									setup_tx_queue ( tx_queue, (void*)&(associations[i]), mpdu_info->length, MAX_RETRY,
										 				         (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO) );

									enqueue_after_end(associations[i].AID,  &checkout);

									check_tx_queue();
									#ifndef ALLOW_ETH_TX_OF_WIRELESS_TX
									eth_send = 0;
									#endif
								}

								break;
							}
						}
					}

					if(eth_send){
						wlan_mpdu_eth_send(mpdu,length);
					}

				}
			} else {
				//TODO: Formally adopt conventions from 10.3 in 802.11-2012 for STA state transitions
				if(wlan_addr_eq(rx_80211_header->address_1, eeprom_mac_addr)){
					if((rx_80211_header->address_3[0] == 0x33) && (rx_80211_header->address_3[1] == 0x33)){
						//TODO: This is an IPv6 Multicast packet. It should get de-encapsulated and sent over the wire
					} else {
						//Received a data frame from a STA that claims to be associated with this AP but is not in the AP association table
						// Discard the MPDU and reply with a de-authentication frame to trigger re-association at the STA

						warp_printf(PL_WARNING, "Data from non-associated station: [%x %x %x %x %x %x], issuing de-authentication\n", rx_80211_header->address_2[0],rx_80211_header->address_2[1],rx_80211_header->address_2[2],rx_80211_header->address_2[3],rx_80211_header->address_2[4],rx_80211_header->address_2[5]);
						warp_printf(PL_WARNING, "Address 3: [%x %x %x %x %x %x]\n", rx_80211_header->address_3[0],rx_80211_header->address_3[1],rx_80211_header->address_3[2],rx_80211_header->address_3[3],rx_80211_header->address_3[4],rx_80211_header->address_3[5]);

						//Send De-authentication
						//Checkout 1 element from the queue;
							queue_checkout(&checkout,1);

							if(checkout.length == 1){ //There was at least 1 free queue element
								tx_queue = checkout.first;

						 		setup_tx_header( &tx_header_common, rx_80211_header->address_2, eeprom_mac_addr );

								tx_length = wlan_create_deauth_frame((void*)((tx_packet_buffer*)(tx_queue->buf_ptr))->frame, &tx_header_common, DEAUTH_REASON_NONASSOCIATED_STA);

						 		setup_tx_queue ( tx_queue, NULL, tx_length, MAX_RETRY,
						 				         (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO) );

								enqueue_after_end(0, &checkout);
								check_tx_queue();
							}
					}
				}
			}//END if(is_associated)

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
				if(send_response && allow_assoc) {

					//Checkout 1 element from the queue;
					queue_checkout(&checkout,1);

					if(checkout.length == 1){ //There was at least 1 free queue element
						tx_queue = checkout.first;

						setup_tx_header( &tx_header_common, rx_80211_header->address_2, eeprom_mac_addr );

						tx_length = wlan_create_probe_resp_frame((void*)((tx_packet_buffer*)(tx_queue->buf_ptr))->frame, &tx_header_common, BEACON_INTERVAL_MS, strlen(access_point_ssid), (u8*)access_point_ssid, mac_param_chan);

				 		setup_tx_queue ( tx_queue, NULL, tx_length, MAX_RETRY,
				 				         (TX_MPDU_FLAGS_FILL_TIMESTAMP | TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO) );

						enqueue_after_end(0, &checkout);
						check_tx_queue();
					}

					return;
				}
			}
		break;

		case (MAC_FRAME_CTRL1_SUBTYPE_AUTH): //Authentication Packet

			if(wlan_addr_eq(rx_80211_header->address_3, eeprom_mac_addr)) {
					mpdu_ptr_u8 += sizeof(mac_header_80211);
					switch(((authentication_frame*)mpdu_ptr_u8)->auth_algorithm){
						case AUTH_ALGO_OPEN_SYSTEM:
							if(((authentication_frame*)mpdu_ptr_u8)->auth_sequence == AUTH_SEQ_REQ){//This is an auth packet from a requester
								//Checkout 1 element from the queue;
								queue_checkout(&checkout,1);

								if(checkout.length == 1){ //There was at least 1 free queue element
									tx_queue = checkout.first;

							 		setup_tx_header( &tx_header_common, rx_80211_header->address_2, eeprom_mac_addr );

									tx_length = wlan_create_auth_frame((void*)((tx_packet_buffer*)(tx_queue->buf_ptr))->frame, &tx_header_common, AUTH_ALGO_OPEN_SYSTEM, AUTH_SEQ_RESP, STATUS_SUCCESS);

							 		setup_tx_queue ( tx_queue, NULL, tx_length, MAX_RETRY,
							 				         (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO) );

									enqueue_after_end(0, &checkout);
									check_tx_queue();
								}

								return;
							}
						break;
						default:

							//Checkout 1 element from the queue;
							queue_checkout(&checkout,1);

							if(checkout.length == 1){ //There was at least 1 free queue element
								tx_queue = checkout.first;

						 		setup_tx_header( &tx_header_common, rx_80211_header->address_2, eeprom_mac_addr );

								tx_length = wlan_create_auth_frame((void*)((tx_packet_buffer*)(tx_queue->buf_ptr))->frame, &tx_header_common, AUTH_ALGO_OPEN_SYSTEM, AUTH_SEQ_RESP, STATUS_AUTH_REJECT_CHALLENGE_FAILURE);

						 		setup_tx_queue ( tx_queue, NULL, tx_length, MAX_RETRY,
						 				         (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO) );

								enqueue_after_end(0, &checkout);
								check_tx_queue();
							}

							warp_printf(PL_WARNING,"Unsupported authentication algorithm (0x%x)\n", ((authentication_frame*)mpdu_ptr_u8)->auth_algorithm);
							return;
						break;
					}
				}
		break;

		case (MAC_FRAME_CTRL1_SUBTYPE_REASSOC_REQ): //Re-association Request
		case (MAC_FRAME_CTRL1_SUBTYPE_ASSOC_REQ): //Association Request
			if(wlan_addr_eq(rx_80211_header->address_3, eeprom_mac_addr)) {
				for(i=0; i <= next_free_assoc_index; i++) {
					if(wlan_addr_eq((associations[i].addr), bcast_addr)) {
						allow_association = 1;
						new_association = 1;

						if(next_free_assoc_index < (MAX_ASSOCIATIONS-2)) {
							next_free_assoc_index++;
							max_queue_size = (queue_total_size()- eth_bd_total_size()) / (next_free_assoc_index+1);
						}
						break;

					} else if(wlan_addr_eq((associations[i].addr), rx_80211_header->address_2)) {
						allow_association = 1;
						new_association = 0;
						break;
					}
				}

				if(allow_association) {
					//Keep track of this association of this association
					memcpy(&(associations[i].addr[0]), rx_80211_header->address_2, 6);
					associations[i].tx_rate = default_unicast_rate; //Default tx_rate for this station. Rate adaptation may change this value.
					associations[i].num_tx_total = 0;
					associations[i].num_tx_success = 0;

					//associations[i].tx_rate = WLAN_MAC_RATE_16QAM34; //Default tx_rate for this station. Rate adaptation may change this value.

					//Checkout 1 element from the queue;
					queue_checkout(&checkout,1);

					if(checkout.length == 1){ //There was at least 1 free queue element
						tx_queue = checkout.first;

				 		setup_tx_header( &tx_header_common, rx_80211_header->address_2, eeprom_mac_addr );

						tx_length = wlan_create_association_response_frame((void*)((tx_packet_buffer*)(tx_queue->buf_ptr))->frame, &tx_header_common, STATUS_SUCCESS, associations[i].AID);

				 		setup_tx_queue ( tx_queue, (void*)&(associations[i]), tx_length, MAX_RETRY,
				 				         (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO) );

						enqueue_after_end(associations[i].AID, &checkout);
						check_tx_queue();
					}

					if(new_association == 1) {
						xil_printf("\n\nNew Association - ID %d\n", associations[i].AID);

						//Print the updated association table to the UART (slow, but useful for observing association success)
						print_associations();
					}

					return;
				}

			}
		break;

		case (MAC_FRAME_CTRL1_SUBTYPE_DISASSOC): //Disassociation
				if(wlan_addr_eq(rx_80211_header->address_3, eeprom_mac_addr)) {
					for(i=0;i<next_free_assoc_index;i++){
						if(wlan_addr_eq(associations[i].addr, rx_80211_header->address_2)) {
								allow_disassociation = 1;
							break;
						}
					}

					if(allow_disassociation) {
						remove_station( i );
						xil_printf("\n\nDisassociation:\n");
						print_associations();
					}
				}
		break;

		default:
			//This should be left as a verbose print. It occurs often when communicating with mobile devices since they tend to send
			//null data frames (type: DATA, subtype: 0x4) for power management reasons.
			warp_printf(PL_VERBOSE, "Received unknown frame control type/subtype %x\n",rx_80211_header->frame_control_1);
		break;
	}

	return;
}



void print_associations(){
	u64 timestamp = get_usec_timestamp();
	u32 i;

	write_hex_display(next_free_assoc_index);
	xil_printf("\n   Current Associations\n (MAC time = %d usec)\n",timestamp);
			xil_printf("|-ID-|----- MAC ADDR ----|\n");
	for(i=0; i < next_free_assoc_index; i++){
		if(wlan_addr_eq(associations[i].addr, bcast_addr)) {
			xil_printf("| %02x |                   |\n", associations[i].AID);
		} else {
			xil_printf("| %02x | %02x:%02x:%02x:%02x:%02x:%02x |\n", associations[i].AID,
					associations[i].addr[0],associations[i].addr[1],associations[i].addr[2],associations[i].addr[3],associations[i].addr[4],associations[i].addr[5]);
		}
	}
			xil_printf("|------------------------|\n");

	return;
}



void enable_associations(){
	// Send a message to other processor to tell it to enable associations
#ifdef _DEBUG_
	xil_printf("Allowing new associations\n");
#endif

	// Set the DSSS value in CPU Low
	set_dsss_value( 1 );

    // Set the global variables
	allow_assoc = 1;
}



void disable_associations(){

	// Send a message to other processor to tell it to disable associations
	if(perma_assoc_mode == 0){

#ifdef _DEBUG_
		xil_printf("Not allowing new associations\n");
#endif

		// Set the DSSS value in CPU Low
		set_dsss_value( 0 );

        // Set the global variables
		allow_assoc      = 0;
		enable_animation = 0;

		// Set the hex display
		write_hex_display(next_free_assoc_index);
		write_hex_display_dots(0);
	}
}



void animate_hex(){
	static u8 i = 0;
	if(enable_animation){
		//write_hex_display(next_free_assoc_index,i%2);
		write_hex_display_dots(i%2);
		i++;
		wlan_mac_schedule_event(SCHEDULE_COARSE, ANIMATION_RATE_US, (void*)animate_hex);
	}
}







void reset_station_statistics(){
	u32 i;
	for(i=0; i < next_free_assoc_index; i++){
		associations[i].num_tx_total = 0;
		associations[i].num_tx_success = 0;
		associations[i].num_rx_success = 0;
		associations[i].num_rx_bytes = 0;
	}
}




void deauthenticate_stations(){
	u32 i;
	packet_bd_list checkout, dequeue;
	u32            num_queued;
	packet_bd*     tx_queue;
	u32            tx_length;

	for(i=0; i < next_free_assoc_index; i++){
		//Send De-authentication

	 	//Checkout 1 element from the queue;
	 	queue_checkout(&checkout,1);

	 	if(checkout.length == 1){ //There was at least 1 free queue element
	 		tx_queue = checkout.first;

	 		setup_tx_header( &tx_header_common, associations[i].addr, eeprom_mac_addr );

	 		tx_length = wlan_create_deauth_frame((void*)((tx_packet_buffer*)(tx_queue->buf_ptr))->frame, &tx_header_common, DEAUTH_REASON_INACTIVITY);

	 		setup_tx_queue ( tx_queue, (void*)&(associations[i]), tx_length, MAX_RETRY,
	 				         (TX_MPDU_FLAGS_FILL_DURATION | TX_MPDU_FLAGS_REQ_TO) );

	 		enqueue_after_end(associations[i].AID, &checkout);
	 		check_tx_queue();

	 		//Purge any packets in the queue meant for this node
	 		num_queued = queue_num_queued(associations[i].AID);
	 		if(num_queued>0){
	 			xil_printf("purging %d packets from queue for AID %d\n",num_queued,associations[i].AID);
	 			dequeue_from_beginning(&dequeue, associations[i].AID,1);
	 			queue_checkin(&dequeue);
	 		}

			//Remove this STA from association list
	 		remove_station(i);
		}
	}
	write_hex_display(next_free_assoc_index);
}





/*****************************************************************************/
/**
* Remove Station from Association Table
*
* The association table is a packed list of stations.  To remove a station, we
* need to re-pack the table.
*
* @param    station_index  - Index of station to remove
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void remove_station( unsigned int station_index ) {

	// If there are stations in the association table and the station to remove
	//   is in the association table, then remove it.

#ifdef _DEBUG
	xil_printf("next_free_assoc_index = %d \n", next_free_assoc_index);
#endif

	if( ( next_free_assoc_index > 0 ) && ( station_index < MAX_ASSOCIATIONS ) ) {

		// Decrement global variable
		next_free_assoc_index--;

		max_queue_size = (queue_total_size()- eth_bd_total_size()) / (next_free_assoc_index+1);

		// Clear Association Address
		memcpy(&(associations[station_index].addr[0]), bcast_addr, 6);

		// If this station is not the last in the table, then re-pack
		if( station_index < next_free_assoc_index ) {
			//Copy from current index to the swap space
			memcpy(&(associations[MAX_ASSOCIATIONS]), &(associations[station_index]), sizeof(station_info));

			//Shift later entries back into the freed association entry
			memcpy(&(associations[station_index]), &(associations[station_index+1]), (next_free_assoc_index - station_index)*sizeof(station_info));

			//Copy from swap space to current free index
			memcpy(&(associations[next_free_assoc_index]), &(associations[MAX_ASSOCIATIONS]), sizeof(station_info));
		}
	}
}


