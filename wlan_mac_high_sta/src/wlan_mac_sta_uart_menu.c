////////////////////////////////////////////////////////////////////////////////
// File   : wlan_mac_sta_uart_menu.c
// Authors: Patrick Murphy (murphpo [at] mangocomm.com)
//			Chris Hunter (chunter [at] mangocomm.com)
// License: Copyright 2013, Mango Communications. All rights reserved.
//          Distributed under the Mango Communications Reference Design License
//				See LICENSE.txt included in the design archive or
//				at http://mangocomm.com/802.11/license
////////////////////////////////////////////////////////////////////////////////

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
#include "wlan_mac_util.h"
#include "wlan_mac_packet_types.h"
#include "wlan_mac_eth_util.h"
#include "wlan_mac_sta.h"
#include "ascii_characters.h"


#ifdef WLAN_USE_UART_MENU


// SSID variables
extern char*  access_point_ssid;

// Control variables
extern u8  default_unicast_rate;
extern int association_state;                      // Section 10.3 of 802.11-2012
extern u8  uart_mode;
extern u8  active_scan;

// Access point information
extern ap_info* ap_list;
extern u8       num_ap_list;

extern u8       access_point_num_basic_rates;
extern u8       access_point_basic_rates[NUM_BASIC_RATES_MAX];

extern u8 pause_queue;


// Association Table variables
//   The last entry in associations[MAX_ASSOCIATIONS][] is swap space
extern station_info access_point;

// AP channel
extern u32 mac_param_chan;
extern u32 mac_param_chan_save;

// LTG variables
u8 ltg_enable;
extern u16 ltg_packet_size;
cbr_params cbr_parameters;


void uart_rx(u8 rxByte){
	#define MAX_NUM_AP_CHARS 4
	static char numerical_entry[MAX_NUM_AP_CHARS+1];
	static u8 curr_decade = 0;

	#define MAX_NUM_CHARS 31
	static char text_entry[MAX_NUM_CHARS+1];
	static u8 curr_char = 0;

	u16 ap_sel;
	wlan_ipc_msg ipc_msg_to_low;
	u32 ipc_msg_to_low_payload[1];
	ipc_config_rf_ifc* config_rf_ifc;

	if(rxByte == ASCII_ESC){
		uart_mode = UART_MODE_MAIN;
		ltg_enable = 0;
		stop_ltg(0);

		print_menu();

		return;
	}

	switch(uart_mode){
		case UART_MODE_MAIN:
			switch(rxByte){
				case ASCII_1:
					uart_mode = UART_MODE_INTERACTIVE;
					print_station_status(1);
				break;

				case ASCII_a:
					//Send bcast probe requests across all channels
					if(active_scan ==0){
						num_ap_list = 0;
						//xil_printf("- Free 0x%08x\n",ap_list);
						free(ap_list);
						ap_list = NULL;
						active_scan = 1;
						access_point_ssid = realloc(access_point_ssid, 1);
						*access_point_ssid = 0;
						//xil_printf("+++ starting active scan\n");
						pause_queue = 1;
						mac_param_chan_save = mac_param_chan;
						probe_req_transmit();
					}
				break;

				case ASCII_r:
					if(default_unicast_rate > WLAN_MAC_RATE_6M){
						default_unicast_rate--;
					} else {
						default_unicast_rate = WLAN_MAC_RATE_6M;
					}


					access_point.tx_rate = default_unicast_rate;


					xil_printf("(-) Default Unicast Rate: %d Mbps\n", wlan_lib_mac_rate_to_mbps(default_unicast_rate));
				break;
				case ASCII_R:
					if(default_unicast_rate < WLAN_MAC_RATE_54M){
						default_unicast_rate++;
					} else {
						default_unicast_rate = WLAN_MAC_RATE_54M;
					}

					access_point.tx_rate = default_unicast_rate;

					xil_printf("(+) Default Unicast Rate: %d Mbps\n", wlan_lib_mac_rate_to_mbps(default_unicast_rate));
				break;
			}
		break;
		case UART_MODE_INTERACTIVE:
			switch(rxByte){
				case ASCII_r:
					//Reset statistics
					reset_station_statistics();
				break;
				case ASCII_1:
					if(access_point.AID != 0){
						uart_mode = UART_MODE_LTG_SIZE_CHANGE;
						curr_char = 0;

						if(ltg_enable == 0){
							print_ltg_size_menu();
						} else {
							ltg_enable = 0;
							stop_ltg(0);
							uart_mode = UART_MODE_INTERACTIVE;
							print_station_status(1);
						}

					}
				break;
			}
		break;
		case UART_MODE_LTG_SIZE_CHANGE:
			switch(rxByte){
				case ASCII_CR:
					text_entry[curr_char] = 0;
					curr_char = 0;

					ltg_packet_size =  str2num(text_entry);

					uart_mode = UART_MODE_LTG_INTERVAL_CHANGE;
					print_ltg_interval_menu();

				break;
				case ASCII_DEL:
					if(curr_char > 0){
						curr_char--;
						xil_printf("\b \b");
					}
				break;
				default:
					if( (rxByte <= ASCII_9) && (rxByte >= ASCII_0) ){
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
		case UART_MODE_LTG_INTERVAL_CHANGE:
			switch(rxByte){
				case ASCII_CR:
					text_entry[curr_char] = 0;
					curr_char = 0;

					cbr_parameters.interval_usec = str2num(text_entry);

					start_ltg(0, LTG_TYPE_CBR, &cbr_parameters);
					ltg_enable = 1;

					uart_mode = UART_MODE_INTERACTIVE;
					print_station_status(1);

				break;
				case ASCII_DEL:
					if(curr_char > 0){
						curr_char--;
						xil_printf("\b \b");
					}
				break;
				default:
					if( (rxByte <= ASCII_9) && (rxByte >= ASCII_0) ){
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

							//Send a message to other processor to tell it to switch channels
							ipc_msg_to_low.msg_id = IPC_MBOX_MSG_ID(IPC_MBOX_CONFIG_RF_IFC);
							ipc_msg_to_low.num_payload_words = sizeof(ipc_config_rf_ifc)/sizeof(u32);
							ipc_msg_to_low.payload_ptr = &(ipc_msg_to_low_payload[0]);
							init_ipc_config(config_rf_ifc,ipc_msg_to_low_payload,ipc_config_rf_ifc);
							config_rf_ifc->channel = mac_param_chan;
							ipc_mailbox_write_msg(&ipc_msg_to_low);


							xil_printf("\nAttempting to join %s\n", ap_list[ap_sel].ssid);
							memcpy(access_point.addr, ap_list[ap_sel].bssid, 6);

							access_point_ssid = realloc(access_point_ssid, strlen(ap_list[ap_sel].ssid)+1);
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

void print_ltg_size_menu(){

	xil_printf("\n\n Configuring Local Traffic Generator (LTG) for AP\n");

	xil_printf("\nEnter packet payload size (in bytes): ");


}

void print_ltg_interval_menu(){

	xil_printf("\nEnter packet Tx interval (in microseconds): ");

}


void print_menu(){
	xil_printf("\f");
	xil_printf("********************** Station Menu **********************\n");
	xil_printf("[1] - Interactive Station Status\n");
	xil_printf("\n");
	xil_printf("[a] - 	active scan and display nearby APs\n");
	xil_printf("[r/R] - change default unicast rate\n");
}

void print_station_status(u8 manual_call){

	u64 timestamp;
	static u8 print_scheduled = 0;


	if((manual_call == 1 && print_scheduled == 0) || (manual_call == 0 && print_scheduled == 1)){
		//This awkward logic is to handle the fact that our event scheduler doesn't currently have a
		//way to remove a scheduled event and stop it from occurring. Without this protection, quick
		//UART inputs could easy begin a chain of print_station events > 1 per second. Eventually
		//you'd run out of scheduled event slots and cause problems.
		if(uart_mode == UART_MODE_INTERACTIVE){
			timestamp = get_usec_timestamp();
			xil_printf("\f");

			xil_printf("---------------------------------------------------\n");
			xil_printf(" AID: %02x -- MAC Addr: %02x:%02x:%02x:%02x:%02x:%02x\n", access_point.AID,
				access_point.addr[0],access_point.addr[1],access_point.addr[2],access_point.addr[3],access_point.addr[4],access_point.addr[5]);
				if(access_point.AID > 0){
					if(ltg_enable){
						xil_printf("  LTG Enabled\n");
						xil_printf("  Packet Size: %d bytes\n", ltg_packet_size);
						xil_printf("  Packet Tx Interval: %d microseconds\n", cbr_parameters.interval_usec);
					}
					xil_printf("     - Last heard from %d ms ago\n",((u32)(timestamp - (access_point.rx_timestamp)))/1000);
					xil_printf("     - Last Rx Power: %d dBm\n",access_point.last_rx_power);
					xil_printf("     - # of queued MPDUs: %d\n", queue_num_queued(access_point.AID));
					xil_printf("     - # Tx MPDUs: %d (%d successful)\n", access_point.num_tx_total, access_point.num_tx_success);
					xil_printf("     - # Rx MPDUs: %d (%d bytes)\n", access_point.num_rx_success, access_point.num_rx_bytes);
				}
			xil_printf("---------------------------------------------------\n");
			xil_printf("\n");
			xil_printf("[r] - reset statistics\n\n");
			xil_printf(" The interactive STA menu supports sending arbitrary traffic\n");
			xil_printf(" to an associated AP. To use this feature, press the number 1\n");
			xil_printf(" Pressing Esc at any time will halt all local traffic\n");
			xil_printf(" generation and return you to the main menu.");

			//Update display
			wlan_mac_schedule_event(SCHEDULE_COARSE, 1000000, (void*)print_station_status);
			print_scheduled = 1;
		} else {
			print_scheduled = 0;
		}
	}
}




#endif


