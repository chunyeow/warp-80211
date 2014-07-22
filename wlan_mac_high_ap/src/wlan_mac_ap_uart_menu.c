/** @file wlan_mac_ap_uart_menu.c
 *  @brief Access Point UART Menu
 *
 *  This contains code for the 802.11 Access Point's UART menu.
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
#include "wlan_mac_ap.h"
#include "ascii_characters.h"
#include "wlan_mac_schedule.h"


#ifndef WLAN_USE_UART_MENU

void uart_rx(u8 rxByte){ };

#else

static u8 uart_mode = UART_MODE_MAIN;
extern tx_params default_unicast_data_tx_params;


extern u32 mac_param_chan;

extern dl_list association_table;
extern dl_list statistics_table;

extern char* access_point_ssid;
static u32 cpu_high_status;
static u32 schedule_ID;
static u8 print_scheduled = 0;

u32 num_slots = SLOT_CONFIG_RAND;

ltg_pyld_all_assoc_fixed 	traffic_blast_pyld;
ltg_sched_periodic_params 	traffic_blast_sched;
u32							traffic_blast_ltg_id;

void uart_rx(u8 rxByte){

	u32 i;
	dl_entry* 	  curr_station_info_entry;
	station_info* curr_station_info;

	void* ltg_state;

	#define MAX_NUM_CHARS 31
	static char text_entry[MAX_NUM_CHARS+1];
	static u8 curr_char = 0;
    
	if(rxByte == ASCII_ESC){
		uart_mode = UART_MODE_MAIN;
		stop_periodic_print();
		print_menu();
		ltg_sched_remove(LTG_REMOVE_ALL);
		traffic_blast_ltg_id = LTG_ID_INVALID;
		return;
	}

	switch(uart_mode){
		case UART_MODE_MAIN:
			switch(rxByte){
				case ASCII_1:
					uart_mode = UART_MODE_INTERACTIVE;
					start_periodic_print();
				break;

				case ASCII_2:
					print_queue_status();
				break;

				case ASCII_3:
					print_all_observed_statistics();
				break;

				case ASCII_e:
			        event_log_config_logging(EVENT_LOG_LOGGING_DISABLE);
					print_event_log_size();
#ifdef _DEBUG_
			        print_event_log( 0xFFFF );
					print_event_log_size();
#endif
			        event_log_config_logging(EVENT_LOG_LOGGING_ENABLE);
				break;

				case ASCII_c:
					if(mac_param_chan > 1){
						deauthenticate_stations();
						(mac_param_chan--);

						//Send a message to other processor to tell it to switch channels
						wlan_mac_high_set_channel( mac_param_chan );
					} else {

					}
					xil_printf("(-) Channel: %d\n", mac_param_chan);


				break;
				case ASCII_C:
					if(mac_param_chan < 11){
						deauthenticate_stations();
						(mac_param_chan++);

						//Send a message to other processor to tell it to switch channels
						wlan_mac_high_set_channel( mac_param_chan );
					} else {

					}
					xil_printf("(+) Channel: %d\n", mac_param_chan);

				break;
				case ASCII_g:
					if((default_unicast_data_tx_params.phy.power) > TX_POWER_MIN_DBM){
						(default_unicast_data_tx_params.phy.power)--;
					} else {
						(default_unicast_data_tx_params.phy.power) = TX_POWER_MIN_DBM;
					}

					xil_printf("(-) Default Tx Power: %d dBm\n", (default_unicast_data_tx_params.phy.power));

				break;
				case ASCII_G:
					if((default_unicast_data_tx_params.phy.power) < TX_POWER_MAX_DBM){
						(default_unicast_data_tx_params.phy.power)++;
					} else {
						(default_unicast_data_tx_params.phy.power) = TX_POWER_MAX_DBM;
					}

					xil_printf("(+) Default Tx Power: %d dBm\n", (default_unicast_data_tx_params.phy.power));

				break;
				case ASCII_r:
					if((default_unicast_data_tx_params.phy.rate) > WLAN_MAC_RATE_6M){
						(default_unicast_data_tx_params.phy.rate)--;
					} else {
						(default_unicast_data_tx_params.phy.rate) = WLAN_MAC_RATE_6M;
					}

					curr_station_info_entry = association_table.first;
					for(i=0; i < association_table.length; i++){
						curr_station_info = (station_info*)(curr_station_info_entry->data);
						curr_station_info->tx.phy.rate = (default_unicast_data_tx_params.phy.rate);
						curr_station_info_entry = dl_entry_next(curr_station_info_entry);
					}

					xil_printf("(-) Default Unicast Rate: %d Mbps\n", wlan_lib_mac_rate_to_mbps((default_unicast_data_tx_params.phy.rate)));
				break;
				case ASCII_R:
					if((default_unicast_data_tx_params.phy.rate) < WLAN_MAC_RATE_54M){
						(default_unicast_data_tx_params.phy.rate)++;
					} else {
						(default_unicast_data_tx_params.phy.rate) = WLAN_MAC_RATE_54M;
					}

					curr_station_info_entry = association_table.first;
					for(i=0; i < association_table.length; i++){
						curr_station_info = (station_info*)(curr_station_info_entry->data);
						curr_station_info->tx.phy.rate = (default_unicast_data_tx_params.phy.rate);
						curr_station_info_entry = dl_entry_next(curr_station_info_entry);
					}
					xil_printf("(+) Default Unicast Rate: %d Mbps\n", wlan_lib_mac_rate_to_mbps((default_unicast_data_tx_params.phy.rate)));
				break;
				case ASCII_s:
					uart_mode = UART_MODE_SSID_CHANGE;
					deauthenticate_stations();
					curr_char = 0;
					print_ssid_menu();
				break;
				case ASCII_h:
					xil_printf("cpu_high_status = 0x%08x\n", cpu_high_status);
				break;
				case ASCII_m:
					wlan_mac_high_display_mallinfo();
				break;
			}
		break;

		case UART_MODE_INTERACTIVE:
			switch(rxByte){
				case ASCII_r:
					//Reset statistics
					reset_station_statistics();
				break;
				case ASCII_d:
					//Deauthenticate all stations
					deauthenticate_stations();
				break;

				case ASCII_b:
					//Toggle Traffic Blaster

					//First, we'll see if an LTG has been created and create a new one if not.

					if(traffic_blast_ltg_id == LTG_ID_INVALID){
						//Set up and start
						traffic_blast_pyld.hdr.type = LTG_PYLD_TYPE_ALL_ASSOC_FIXED;
						traffic_blast_pyld.length = 1400;
						traffic_blast_sched.duration_count = LTG_DURATION_FOREVER;
						traffic_blast_sched.interval_count = 0;

						traffic_blast_ltg_id = ltg_sched_create(LTG_SCHED_TYPE_PERIODIC, &traffic_blast_sched, &traffic_blast_pyld, NULL);

						if(traffic_blast_ltg_id == LTG_ID_INVALID){
							xil_printf("Error in creating LTG\n");
							break;
						}

					}

					//Second, we'll check to see if this LTG ID is currently running. If we just created it in the above,
					//this check isn't necessary (we know it's not running). We'll ask anyway so that we have fewer cases
					//to deal with.

					ltg_sched_get_state(traffic_blast_ltg_id, NULL, &ltg_state);

					//Note: Here I have the luxury of knowing what type ltg_state is. In general, the second argument
					//to ltg_sched_get_state can be used to figure out what type to cast ltg_state to

					switch(((ltg_sched_periodic_state*)ltg_state)->hdr.enabled){
						case 0:
							//LTG is not running, so let's start it
							ltg_sched_start(traffic_blast_ltg_id);
						break;

						case 1:
							//LTG is running, so let's stop it
							ltg_sched_stop(traffic_blast_ltg_id);
						break;
					}
				break;

			}
		break;

		case UART_MODE_SSID_CHANGE:
			switch(rxByte){
				case ASCII_CR:


					text_entry[curr_char] = 0;
					curr_char = 0;
					uart_mode = UART_MODE_MAIN;

					access_point_ssid = wlan_mac_high_realloc(access_point_ssid, strlen(text_entry)+1);
					strcpy(access_point_ssid,text_entry);
					xil_printf("\nSetting new SSID: %s\n", access_point_ssid);

				break;
				case ASCII_DEL:
					if(curr_char > 0){
						curr_char--;
						xil_printf("\b \b");
					}

				break;
				default:
					if( (rxByte <= ASCII_z) && (rxByte >= ASCII_A) ){
						//the user entered a character

						if(curr_char < MAX_NUM_CHARS){
							xil_printf("%c", rxByte);
							text_entry[curr_char] = rxByte;
							curr_char++;
						}
					}
				break;
			}
		break;
	}
}

void print_ssid_menu(){
	xil_printf("\f");
	xil_printf("Current SSID: %s\n", access_point_ssid);
	xil_printf("To change the current SSID, please type a new string and press enter\n");
	xil_printf(": ");
}


void print_queue_status(){
	u32 i;
	dl_entry* curr_entry;
	station_info* curr_station_info;
	xil_printf("\nQueue Status:\n");
	xil_printf(" FREE || MCAST|");

	curr_entry = association_table.first;
	for(i=0; i < association_table.length; i++){
		curr_station_info = (station_info*)(curr_entry->data);
		xil_printf("%6d|", curr_station_info->AID);
		curr_entry = dl_entry_next(curr_entry);
	}
	xil_printf("\n");

	xil_printf("%6d||%6d|",queue_num_free(),queue_num_queued(MCAST_QID));

	curr_entry = association_table.first;
	for(i=0; i < association_table.length; i++){
		curr_station_info = (station_info*)(curr_entry->data);
		xil_printf("%6d|", queue_num_queued(AID_TO_QID(curr_station_info->AID)));
		curr_entry = dl_entry_next(curr_entry);
	}
	xil_printf("\n");

}

void print_menu(){
	xil_printf("\f");
	xil_printf("********************** AP Menu **********************\n");
	xil_printf("[1] - Interactive AP Status\n");
	xil_printf("[2] - Print Queue Status\n");
	xil_printf("[3] - Print all Observed Statistics\n");
	xil_printf("\n");
	xil_printf("[c/C] - change channel (note: changing channel will\n");
	xil_printf("        purge any associations, forcing stations to\n");
	xil_printf("        join the network again)\n");
	xil_printf("[r/R] - change default unicast rate\n");
	xil_printf("[s]   - change SSID (note: changing SSID will purge)\n");
	xil_printf("        any associations)\n");
	xil_printf("*****************************************************\n");
}





void print_station_status(){

//	xil_printf("print_station_status\n");

//#if 0
	u32 i;
	station_info* curr_station_info;
	dl_entry*	  curr_entry;

	u64 timestamp;

	if(uart_mode == UART_MODE_INTERACTIVE){
		timestamp = get_usec_timestamp();
		xil_printf("\f");
		//xil_printf("next_free_assoc_index = %d\n", next_free_assoc_index);

		curr_entry = association_table.first;


		for(i=0; i < association_table.length; i++){
			curr_station_info = (station_info*)(curr_entry->data);
			xil_printf("---------------------------------------------------\n");
			if(curr_station_info->hostname[0] != 0){
				xil_printf(" Hostname: %s\n", curr_station_info->hostname);
			}
			xil_printf(" AID: %02x -- MAC Addr: %02x:%02x:%02x:%02x:%02x:%02x\n", curr_station_info->AID,
					curr_station_info->addr[0],curr_station_info->addr[1],curr_station_info->addr[2],curr_station_info->addr[3],curr_station_info->addr[4],curr_station_info->addr[5]);

			xil_printf("     - Last heard from         %d ms ago\n",((u32)(timestamp - (curr_station_info->rx.last_timestamp)))/1000);
			xil_printf("     - Last Rx Power:          %d dBm\n",curr_station_info->rx.last_power);
			xil_printf("     - # of queued MPDUs:      %d\n", queue_num_queued(AID_TO_QID(curr_station_info->AID)));
			xil_printf("     - # Tx High Data MPDUs:   %d (%d successful)\n", curr_station_info->stats->data.tx_num_packets_total, curr_station_info->stats->data.tx_num_packets_success);
			xil_printf("     - # Tx High Data bytes:   %d (%d successful)\n", (u32)(curr_station_info->stats->data.tx_num_bytes_total), (u32)(curr_station_info->stats->data.tx_num_bytes_success));
			xil_printf("     - # Tx Low Data MPDUs:    %d\n", curr_station_info->stats->data.tx_num_packets_low);
			xil_printf("     - # Tx High Mgmt MPDUs:   %d (%d successful)\n", curr_station_info->stats->mgmt.tx_num_packets_total, curr_station_info->stats->mgmt.tx_num_packets_success);
			xil_printf("     - # Tx High Mgmt bytes:   %d (%d successful)\n", (u32)(curr_station_info->stats->mgmt.tx_num_bytes_total), (u32)(curr_station_info->stats->mgmt.tx_num_bytes_success));
			xil_printf("     - # Tx Low Mgmt MPDUs:    %d\n", curr_station_info->stats->mgmt.tx_num_packets_low);
			xil_printf("     - # Rx Data MPDUs:        %d\n", curr_station_info->stats->data.rx_num_packets);
			xil_printf("     - # Rx Data Bytes:        %d\n", curr_station_info->stats->data.rx_num_bytes);
			xil_printf("     - # Rx Mgmt MPDUs:        %d\n", curr_station_info->stats->mgmt.rx_num_packets);
			xil_printf("     - # Rx Mgmt Bytes:        %d\n", curr_station_info->stats->mgmt.rx_num_bytes);

			curr_entry = dl_entry_next(curr_entry);

		}

			xil_printf("---------------------------------------------------\n");
			xil_printf("\n");
			xil_printf("[r] - reset statistics\n");
			xil_printf("[d] - deauthenticate all stations\n\n");

	}

//
}

void print_all_observed_statistics(){
	u32 i;
	statistics_txrx* curr_statistics;
	dl_entry* curr_statistics_entry;

	curr_statistics_entry = statistics_table.first;

	xil_printf("\nAll Statistics:\n");
	for(i=0; i<statistics_table.length; i++){
		curr_statistics = (statistics_txrx*)(curr_statistics_entry->data);


		xil_printf("---------------------------------------------------\n");
		xil_printf("%02x:%02x:%02x:%02x:%02x:%02x\n", curr_statistics->addr[0],curr_statistics->addr[1],curr_statistics->addr[2],curr_statistics->addr[3],curr_statistics->addr[4],curr_statistics->addr[5]);
		xil_printf("     - Last timestamp: %d usec\n", (u32)curr_statistics->last_rx_timestamp);
		xil_printf("     - Associated?       %d\n", curr_statistics->is_associated);
		xil_printf("     - # Tx High Data MPDUs:   %d (%d successful)\n", curr_statistics->data.tx_num_packets_total, curr_statistics->data.tx_num_packets_success);
		xil_printf("     - # Tx High Data bytes:   %d (%d successful)\n", (u32)(curr_statistics->data.tx_num_bytes_total), (u32)(curr_statistics->data.tx_num_bytes_success));
		xil_printf("     - # Tx Low Data MPDUs:    %d\n", curr_statistics->data.tx_num_packets_low);
		xil_printf("     - # Tx High Mgmt MPDUs:   %d (%d successful)\n", curr_statistics->mgmt.tx_num_packets_total, curr_statistics->mgmt.tx_num_packets_success);
		xil_printf("     - # Tx High Mgmt bytes:   %d (%d successful)\n", (u32)(curr_statistics->mgmt.tx_num_bytes_total), (u32)(curr_statistics->mgmt.tx_num_bytes_success));
		xil_printf("     - # Tx Low Mgmt MPDUs:    %d\n", curr_statistics->mgmt.tx_num_packets_low);
		xil_printf("     - # Rx Data MPDUs:        %d\n", curr_statistics->data.rx_num_packets);
		xil_printf("     - # Rx Data Bytes:        %d\n", curr_statistics->data.rx_num_bytes);
		xil_printf("     - # Rx Mgmt MPDUs:        %d\n", curr_statistics->mgmt.rx_num_packets);
		xil_printf("     - # Rx Mgmt Bytes:        %d\n", curr_statistics->mgmt.rx_num_bytes);

		curr_statistics_entry = dl_entry_next(curr_statistics_entry);
	}
}

void start_periodic_print(){
	stop_periodic_print();
	print_station_status();
	print_scheduled = 1;
	schedule_ID = wlan_mac_schedule_event_repeated(SCHEDULE_COARSE, 1000000, SCHEDULE_REPEAT_FOREVER, (void*)print_station_status);
}

void stop_periodic_print(){
	if(print_scheduled){
		print_scheduled = 0;
		wlan_mac_remove_schedule(SCHEDULE_COARSE, schedule_ID);
	}
}

int is_qwerty_row(u8 rxByte){
	int return_value = 0;

	switch(rxByte){
		case ASCII_Q:
		case ASCII_W:
		case ASCII_E:
		case ASCII_R:
		case ASCII_T:
		case ASCII_Y:
		case ASCII_U:
		case ASCII_I:
		case ASCII_O:
		case ASCII_P:
			return_value = 1;
		break;
	}

	return return_value;
}

int qwerty_row_to_number(u8 rxByte){
	int return_value = -1;

	switch(rxByte){
		case ASCII_Q:
			return_value = 1;
		break;
		case ASCII_W:
			return_value = 2;
		break;
		case ASCII_E:
			return_value = 3;
		break;
		case ASCII_R:
			return_value = 4;
		break;
		case ASCII_T:
			return_value = 5;
		break;
		case ASCII_Y:
			return_value = 6;
		break;
		case ASCII_U:
			return_value = 7;
		break;
		case ASCII_I:
			return_value = 8;
		break;
		case ASCII_O:
			return_value = 9;
		break;
		case ASCII_P:
			return_value = 0;
		break;
	}

	return return_value;
}

#endif


