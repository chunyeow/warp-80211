/** @file wlan_mac_sta.h
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
 */

/***************************** Include Files *********************************/


/*************************** Constant Definitions ****************************/
#ifndef WLAN_MAC_STA_H_
#define WLAN_MAC_STA_H_

#include "wlan_mac_dl_list.h"

// Enable the WLAN UART Menu
#define WLAN_USE_UART_MENU

// Tx queue IDs
#define MCAST_QID 		0
#define MANAGEMENT_QID 	1
#define UNICAST_QID 	2

// Common Defines
#define SSID_LEN_MAX                   32
#define NUM_BASIC_RATES_MAX            10

#define MAX_NUM_TX            7   ///max number of wireless Tx for any MPDU (= max_num_retransmissions + 1)
#define MAX_TX_QUEUE_LEN	  150 ///max number of entries in any Tx queue
#define MAX_NUM_ASSOC		  1   ///max number of associations the STA will attempt

// UART Menu Modes
#define UART_MODE_MAIN                 0
#define UART_MODE_INTERACTIVE          1
#define UART_MODE_AP_LIST              2

// Timing Parameters

//Timeout for association request-response handshake
#define ASSOCIATION_TIMEOUT_US         100000
#define ASSOCIATION_NUM_TRYS           5

//Timeout for authentication handshake
#define AUTHENTICATION_TIMEOUT_US      100000
#define AUTHENTICATION_NUM_TRYS        5

//Number of probe requests to send per channel when active scanning
#define NUM_PROBE_REQ                  5

//Time the active scan procedure will dwell on each channel before
//moving to the next channel (microseconds)
#define ACTIVE_SCAN_DWELL			   100000

//The amount of time between full active scans when looking for a particular SSID
//Note: This value must be larger than the maximum amount of time it takes for
//a single active scan. For an active scan over 11 channels, this value must be larger
//than 11*ACTIVE_SCAN_DWELL.
#define ACTIVE_SCAN_UPDATE_RATE		  5000000

// Information about APs
typedef struct{
	u8   bssid[6];
	u8   chan;
	u8   private;
	char ssid[SSID_LEN_MAX];
	u8   num_basic_rates;
	u8   basic_rates[NUM_BASIC_RATES_MAX];
	char rx_power;
} ap_info;


/*************************** Function Prototypes *****************************/
int main();

void ltg_event(u32 id, void* callback_arg);

int ethernet_receive(tx_queue_element* curr_tx_queue_element, u8* eth_dest, u8* eth_src, u16 tx_length);

void mpdu_rx_process(void* pkt_buf_addr, u8 rate, u16 length);
void mpdu_transmit_done(tx_frame_info* tx_mpdu, wlan_mac_low_tx_details* tx_low_details, u16 num_tx_low_details);
void poll_tx_queues();
void purge_all_data_tx_queue();

void start_active_scan();
void stop_active_scan();
void probe_req_transmit();

void attempt_authentication();

void reset_station_statistics();
dl_list * get_statistics();

int  get_ap_list( ap_info * ap_list, u32 num_ap, u32 * buffer, u32 max_words );

void print_menu();
void print_ap_list();
void print_station_status(u8 manual_call);
void uart_rx(u8 rxByte);
void print_all_observed_statistics();



#endif /* WLAN_MAC_STA_H_ */
