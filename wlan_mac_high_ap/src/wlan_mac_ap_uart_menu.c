////////////////////////////////////////////////////////////////////////////////
// File   : wlan_mac_ap_uart_menu.c
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
#include "wlan_mac_ap.h"
#include "ascii_characters.h"


#ifdef WLAN_USE_UART_MENU

static u8 uart_mode = UART_MODE_MAIN;
extern u8 default_unicast_rate;
extern u32 mac_param_chan;

extern station_info associations[MAX_ASSOCIATIONS+1];
extern u32 next_free_assoc_index;
u8 ltg_enable[MAX_ASSOCIATIONS];
extern u16 ltg_packet_size[MAX_ASSOCIATIONS];
cbr_params cbr_parameters[MAX_ASSOCIATIONS];

extern char* access_point_ssid;
static u8 curr_aid;
static u8 curr_association_index;

void uart_rx(u8 rxByte){
	wlan_ipc_msg ipc_msg_to_low;
	u32 ipc_msg_to_low_payload[1];
	ipc_config_rf_ifc* config_rf_ifc;
	u32 i;

	#define MAX_NUM_CHARS 31
	static char text_entry[MAX_NUM_CHARS+1];
	static u8 curr_char = 0;
    
	if(rxByte == ASCII_ESC){
		uart_mode = UART_MODE_MAIN;
		print_menu();

		for(i = 0; i < MAX_ASSOCIATIONS; i++){
			if(ltg_enable[i]){
				ltg_enable[i] = 0;
				stop_ltg(associations[i].AID);
			}

		}

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
					print_queue_status();
				break;

				case ASCII_e:
					print_event_log();
				break;

				case ASCII_c:

					if(mac_param_chan > 1){
						deauthenticate_stations();
						(mac_param_chan--);

						//Send a message to other processor to tell it to switch channels
						ipc_msg_to_low.msg_id = IPC_MBOX_MSG_ID(IPC_MBOX_CONFIG_RF_IFC);
						ipc_msg_to_low.num_payload_words = sizeof(ipc_config_rf_ifc)/sizeof(u32);
						ipc_msg_to_low.payload_ptr = &(ipc_msg_to_low_payload[0]);
						init_ipc_config(config_rf_ifc,ipc_msg_to_low_payload,ipc_config_rf_ifc);
						config_rf_ifc->channel = mac_param_chan;
						ipc_mailbox_write_msg(&ipc_msg_to_low);
					} else {

					}
					xil_printf("(-) Channel: %d\n", mac_param_chan);


				break;
				case ASCII_C:
					if(mac_param_chan < 11){
						deauthenticate_stations();
						(mac_param_chan++);

						//Send a message to other processor to tell it to switch channels
						ipc_msg_to_low.msg_id = IPC_MBOX_MSG_ID(IPC_MBOX_CONFIG_RF_IFC);
						ipc_msg_to_low.num_payload_words = sizeof(ipc_config_rf_ifc)/sizeof(u32);
						ipc_msg_to_low.payload_ptr = &(ipc_msg_to_low_payload[0]);
						init_ipc_config(config_rf_ifc,ipc_msg_to_low_payload,ipc_config_rf_ifc);
						config_rf_ifc->channel = mac_param_chan;
						ipc_mailbox_write_msg(&ipc_msg_to_low);
					} else {

					}
					xil_printf("(+) Channel: %d\n", mac_param_chan);

				break;
				case ASCII_r:
					if(default_unicast_rate > WLAN_MAC_RATE_6M){
						default_unicast_rate--;
					} else {
						default_unicast_rate = WLAN_MAC_RATE_6M;
					}

					for(i=0; i < next_free_assoc_index; i++){
						associations[i].tx_rate = default_unicast_rate;
					}

					xil_printf("(-) Default Unicast Rate: %d Mbps\n", wlan_lib_mac_rate_to_mbps(default_unicast_rate));
				break;
				case ASCII_R:
					if(default_unicast_rate < WLAN_MAC_RATE_54M){
						default_unicast_rate++;
					} else {
						default_unicast_rate = WLAN_MAC_RATE_54M;
					}

					for(i=0; i < next_free_assoc_index; i++){
						associations[i].tx_rate = default_unicast_rate;
					}
					xil_printf("(+) Default Unicast Rate: %d Mbps\n", wlan_lib_mac_rate_to_mbps(default_unicast_rate));
				break;
				case ASCII_s:
					uart_mode = UART_MODE_SSID_CHANGE;
					deauthenticate_stations();
					curr_char = 0;
					print_ssid_menu();
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
				default:
					if( (rxByte <= ASCII_9) && (rxByte >= ASCII_0) ){
						curr_aid = rxByte - 48;

						for(i=0; i < next_free_assoc_index; i++){
							if(associations[i].AID == curr_aid){
								uart_mode = UART_MODE_LTG_SIZE_CHANGE;
								curr_association_index = i;
								curr_char = 0;

								if(ltg_enable[i] == 0){
									print_ltg_size_menu();
								} else {
									ltg_enable[i] = 0;
									stop_ltg(associations[i].AID);
									uart_mode = UART_MODE_INTERACTIVE;
									print_station_status(1);
								}

							}

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

					//cbr_parameters[curr_association_index].interval_usec;
					ltg_packet_size[curr_association_index] =  str2num(text_entry);

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

						cbr_parameters[curr_association_index].interval_usec = str2num(text_entry);

						start_ltg(curr_aid, LTG_TYPE_CBR, &cbr_parameters[curr_association_index]);
						ltg_enable[curr_association_index] = 1;

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

		case UART_MODE_SSID_CHANGE:
			switch(rxByte){
				case ASCII_CR:


					text_entry[curr_char] = 0;
					curr_char = 0;
					uart_mode = UART_MODE_MAIN;

					access_point_ssid = realloc(access_point_ssid, strlen(text_entry)+1);
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

void print_ltg_size_menu(){

	xil_printf("\n\n Configuring Local Traffic Generator (LTG) for AID %d\n", curr_aid);

	xil_printf("\nEnter packet payload size (in bytes): ");


}

void print_ltg_interval_menu(){

	xil_printf("\nEnter packet Tx interval (in microseconds): ");

}

void print_ssid_menu(){
	xil_printf("\f");
	xil_printf("Current SSID: %s\n", access_point_ssid);
	xil_printf("To change the current SSID, please type a new string and press enter\n");
	xil_printf(": ");
}


void print_queue_status(){
	u32 i;
	xil_printf("\nQueue Status:\n");
	xil_printf(" FREE || BCAST|");

	for(i=0; i<next_free_assoc_index; i++){
		xil_printf("%6d|", associations[i].AID);
	}
	xil_printf("\n");

	xil_printf("%6d||%6d|",queue_num_free(),queue_num_queued(0));

	for(i=0; i<next_free_assoc_index; i++){
		xil_printf("%6d|", queue_num_queued(associations[i].AID));
	}
	xil_printf("\n");

}

void print_menu(){
	xil_printf("\f");
	xil_printf("********************** AP Menu **********************\n");
	xil_printf("[1] - Interactive AP Status\n");
	xil_printf("[2] - Print Queue Status\n");
	xil_printf("\n");
	xil_printf("[c/C] - change channel (note: changing channel will\n");
	xil_printf("        purge any associations, forcing stations to\n");
	xil_printf("        join the network again)\n");
	xil_printf("[r/R] - change default unicast rate\n");
	xil_printf("[s]   - change SSID (note: changing SSID will purge)\n");
	xil_printf("        any associations)\n");
	xil_printf("*****************************************************\n");
}





void print_station_status(u8 manual_call){
	u32 i;
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

			for(i=0; i < next_free_assoc_index; i++){
				xil_printf("---------------------------------------------------\n");
				xil_printf(" AID: %02x -- MAC Addr: %02x:%02x:%02x:%02x:%02x:%02x\n", associations[i].AID,
						associations[i].addr[0],associations[i].addr[1],associations[i].addr[2],associations[i].addr[3],associations[i].addr[4],associations[i].addr[5]);

				if(ltg_enable[i]){
					xil_printf("  LTG Enabled\n");
					xil_printf("  Packet Size: %d bytes\n", ltg_packet_size[i]);
					xil_printf("  Packet Tx Interval: %d microseconds\n", cbr_parameters[i].interval_usec);
				}

				xil_printf("     - Last heard from %d ms ago\n",((u32)(timestamp - (associations[i].rx_timestamp)))/1000);
				xil_printf("     - Last Rx Power: %d dBm\n",associations[i].last_rx_power);
				xil_printf("     - # of queued MPDUs: %d\n", queue_num_queued(associations[i].AID));
				xil_printf("     - # Tx MPDUs: %d (%d successful)\n", associations[i].num_tx_total, associations[i].num_tx_success);
				xil_printf("     - # Rx MPDUs: %d (%d bytes)\n", associations[i].num_rx_success, associations[i].num_rx_bytes);
			}
				xil_printf("---------------------------------------------------\n");
				xil_printf("\n");
				xil_printf("[r] - reset statistics\n");
				xil_printf("[d] - deauthenticate all stations\n\n");
				xil_printf(" The interactive AP menu supports sending arbitrary traffic\n");
				xil_printf(" to any associated station. To use this feature, press any number\n");
				xil_printf(" on the keyboard that corresponds to an associated station's AID\n");
				xil_printf(" and follow the prompts. Pressing Esc at any time will halt all\n");
				xil_printf(" local traffic generation and return you to the main menu.");




			//Update display
			wlan_mac_schedule_event(SCHEDULE_COARSE, 1000000, (void*)print_station_status);
			print_scheduled = 1;
		} else {
			print_scheduled = 0;
		}
	}


}

#endif


