/** @file wlan_mac_sta_uart_menu.c
 *  @brief Station UART Menu
 *
 *  This contains code for the 802.11 Station's UART menu.
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
#include "wlan_mac_ltg.h"
#include "wlan_mac_high.h"
#include "wlan_mac_packet_types.h"
#include "wlan_mac_eth_util.h"
#include "wlan_mac_sta.h"
#include "ascii_characters.h"
#include "wlan_mac_schedule.h"
#include "wlan_mac_event_log.h"


#ifdef WLAN_USE_UART_MENU


// SSID variables
extern char*  access_point_ssid;

// Control variables
extern tx_params default_unicast_data_tx_params;
extern int association_state;                      // Section 10.3 of 802.11-2012
extern u8  uart_mode;
extern u8  active_scan;
static u32 schedule_ID;
static u8 print_scheduled = 0;

// Access point information
extern ap_info* ap_list;
extern u8       num_ap_list;

extern u8       access_point_num_basic_rates;
extern u8       access_point_basic_rates[NUM_BASIC_RATES_MAX];

extern u8 pause_queue;


// Association Table variables
extern dl_list		  association_table;
extern dl_list 		  statistics_table;
extern u8			  ap_addr[6];

// AP channel
extern u32 mac_param_chan;
extern u32 mac_param_chan_save;

u32 num_slots = SLOT_CONFIG_RAND;

void uart_rx(u8 rxByte){

	station_info* access_point = ((station_info*)(association_table.first));

	#define MAX_NUM_AP_CHARS 4
	static char numerical_entry[MAX_NUM_AP_CHARS+1];
	static u8 curr_decade = 0;

	#define MAX_NUM_CHARS 31

	u16 ap_sel;

	if(rxByte == ASCII_ESC){
		uart_mode = UART_MODE_MAIN;

		stop_active_scan();

		if(print_scheduled){
			wlan_mac_remove_schedule(SCHEDULE_COARSE, schedule_ID);
		}
		print_menu();

		ltg_sched_remove(LTG_REMOVE_ALL);

		return;
	}

	switch(uart_mode){
		case UART_MODE_MAIN:
			switch(rxByte){
				case ASCII_1:
					uart_mode = UART_MODE_INTERACTIVE;
					print_station_status(1);
				break;

				case ASCII_2:
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

				case ASCII_a:
					//Send bcast probe requests across all channels
					if(active_scan == 0){
						num_ap_list = 0;
						//xil_printf("- Free 0x%08x\n",ap_list);
						wlan_mac_high_free(ap_list);
						ap_list = NULL;
						active_scan = 1;
						access_point_ssid = wlan_mac_high_realloc(access_point_ssid, 1);
						*access_point_ssid = 0;
						//xil_printf("+++ starting active scan\n");
						pause_queue = 1;
						mac_param_chan_save = mac_param_chan;
						probe_req_transmit();
					}
				break;

				case ASCII_r:
					if((default_unicast_data_tx_params.phy.rate) > WLAN_MAC_RATE_6M){
						(default_unicast_data_tx_params.phy.rate)--;
					} else {
						(default_unicast_data_tx_params.phy.rate) = WLAN_MAC_RATE_6M;
					}

					if(association_table.length > 0) access_point->tx.phy.rate = (default_unicast_data_tx_params.phy.rate);


					xil_printf("(-) Default Unicast Rate: %d Mbps\n", wlan_lib_mac_rate_to_mbps((default_unicast_data_tx_params.phy.rate)));
				break;
				case ASCII_R:
					if((default_unicast_data_tx_params.phy.rate) < WLAN_MAC_RATE_54M){
						(default_unicast_data_tx_params.phy.rate)++;
					} else {
						(default_unicast_data_tx_params.phy.rate) = WLAN_MAC_RATE_54M;
					}

					if(association_table.length > 0) access_point->tx.phy.rate = (default_unicast_data_tx_params.phy.rate);

					xil_printf("(+) Default Unicast Rate: %d Mbps\n", wlan_lib_mac_rate_to_mbps((default_unicast_data_tx_params.phy.rate)));
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
			}
		break;
		case UART_MODE_INTERACTIVE:
			switch(rxByte){
				case ASCII_r:
					//Reset statistics
					reset_station_statistics();
				break;
			}
		break;
		case UART_MODE_AP_LIST:
			switch(rxByte){
				case ASCII_CR:

					numerical_entry[curr_decade] = 0;
					curr_decade = 0;

					ap_sel = str2num(numerical_entry);

					if( (ap_sel >= 0) && (ap_sel <= (num_ap_list-1))){

						if( ap_list[ap_sel].private == 0) {
							uart_mode = UART_MODE_MAIN;
							mac_param_chan = ap_list[ap_sel].chan;

							wlan_mac_high_set_channel(mac_param_chan);


							xil_printf("\nAttempting to join %s\n", ap_list[ap_sel].ssid);
							memcpy(ap_addr, ap_list[ap_sel].bssid, 6);

							access_point_ssid = wlan_mac_high_realloc(access_point_ssid, strlen(ap_list[ap_sel].ssid)+1);
							//xil_printf("allocated %d bytes in 0x%08x\n", strlen(ap_list[ap_sel].ssid), access_point_ssid);
							strcpy(access_point_ssid,ap_list[ap_sel].ssid);

							access_point_num_basic_rates = ap_list[ap_sel].num_basic_rates;
							memcpy(access_point_basic_rates, ap_list[ap_sel].basic_rates,access_point_num_basic_rates);

							association_state = 1;
							attempt_authentication();

						} else {
							xil_printf("\nInvalid selection, please choose an AP that is not private: ");
						}


					} else {

						xil_printf("\nInvalid selection, please choose a number between [0,%d]: ", num_ap_list-1);

					}



				break;
				case ASCII_DEL:
					if(curr_decade > 0){
						curr_decade--;
						xil_printf("\b \b");
					}

				break;
				default:
					if( (rxByte <= ASCII_9) && (rxByte >= ASCII_0) ){
						//the user entered a character

						if(curr_decade < MAX_NUM_AP_CHARS){
							xil_printf("%c", rxByte);
							numerical_entry[curr_decade] = rxByte;
							curr_decade++;
						}



					}

				break;

			}
		break;

	}


}


void print_menu(){
	xil_printf("\f");
	xil_printf("********************** Station Menu **********************\n");
	xil_printf("[1] - Interactive Station Status\n");
	xil_printf("[2] - Print all Observed Statistics\n");
	xil_printf("\n");
	xil_printf("[a] - 	active scan and display nearby APs\n");
	xil_printf("[r/R] - change default unicast rate\n");
}

void print_station_status(u8 manual_call){

	u64 timestamp;
	dl_entry* access_point_entry = association_table.first;
	station_info* access_point = ((station_info*)(access_point_entry->data));
	statistics_txrx* curr_statistics;


	if(uart_mode == UART_MODE_INTERACTIVE){
		timestamp = get_usec_timestamp();
		xil_printf("\f");
		xil_printf("---------------------------------------------------\n");

			if(association_table.length > 0){
				xil_printf(" AID: %02x -- MAC Addr: %02x:%02x:%02x:%02x:%02x:%02x\n", access_point->AID,
							access_point->addr[0],access_point->addr[1],access_point->addr[2],access_point->addr[3],access_point->addr[4],access_point->addr[5]);

				curr_statistics = access_point->stats;

				xil_printf("     - Last heard from    %d ms ago\n",((u32)(timestamp - (access_point->rx.last_timestamp)))/1000);
				xil_printf("     - Last Rx Power:     %d dBm\n",access_point->rx.last_power);
				xil_printf("     - # of queued MPDUs: %d\n", queue_num_queued(UNICAST_QID));
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
			}
		xil_printf("---------------------------------------------------\n");
		xil_printf("\n");
		xil_printf("[r] - reset statistics\n\n");
		xil_printf(" The interactive STA menu supports sending arbitrary traffic\n");
		xil_printf(" to an associated AP. To use this feature, press the number 1\n");
		xil_printf(" Pressing Esc at any time will halt all local traffic\n");
		xil_printf(" generation and return you to the main menu.");

		//Update display
		schedule_ID = wlan_mac_schedule_event(SCHEDULE_COARSE, 1000000, (void*)print_station_status);

	}
}

void print_all_observed_statistics(){
	u32 i;
	dl_entry*	curr_statistics_entry;
	statistics_txrx* curr_statistics;

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




#endif


