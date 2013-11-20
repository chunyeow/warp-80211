////////////////////////////////////////////////////////////////////////////////
// File   : wlan_mac_sta.h
// Authors: Patrick Murphy (murphpo [at] mangocomm.com)
//			Chris Hunter (chunter [at] mangocomm.com)
//          Erik Welsh (welsh [at] mangocomm.com)
// License: Copyright 2013, Mango Communications. All rights reserved.
//          Distributed under the Mango Communications Reference Design License
//				See LICENSE.txt included in the design archive or
//				at http://mangocomm.com/802.11/license
////////////////////////////////////////////////////////////////////////////////

/***************************** Include Files *********************************/


/*************************** Constant Definitions ****************************/
#ifndef WLAN_MAC_STA_H_
#define WLAN_MAC_STA_H_


// **********************************************************************
// Enable the WLAN UART Menu
//    NOTE:  To enable the WLAN Exp framework, please modify wlan_exp_common.h
#define WLAN_USE_UART_MENU



// **********************************************************************
// Common Defines
//
#define SSID_LEN_MAX                   32
#define NUM_BASIC_RATES_MAX            10

#define MAX_RETRY                       7



// **********************************************************************
// UART Menu Modes
//
#define UART_MODE_MAIN                 0
#define UART_MODE_INTERACTIVE          1
#define UART_MODE_AP_LIST              2
#define UART_MODE_LTG_SIZE_CHANGE	   3
#define UART_MODE_LTG_INTERVAL_CHANGE  4



// **********************************************************************
// Timing Parameters
//
#define ASSOCIATION_TIMEOUT_US         100000
#define ASSOCIATION_NUM_TRYS           5

#define AUTHENTICATION_TIMEOUT_US      100000
#define AUTHENTICATION_NUM_TRYS        5

#define NUM_PROBE_REQ                  5




/*********************** Global Structure Definitions ************************/
//
// Information about APs
//
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

void ltg_event(u32 id);

int ethernet_receive(packet_bd_list* tx_queue_list, u8* eth_dest, u8* eth_src, u16 tx_length);

void mpdu_rx_process(void* pkt_buf_addr, u8 rate, u16 length);
void mpdu_transmit_done(tx_frame_info* tx_mpdu);
void check_tx_queue();

void probe_req_transmit();

void attempt_authentication();

void reset_station_statistics();
void print_ltg_size_menu();
void print_ltg_interval_menu();
void print_menu();
void print_ap_list();
void print_station_status(u8 manual_call);

void uart_rx(u8 rxByte);



#endif /* WLAN_MAC_STA_H_ */
