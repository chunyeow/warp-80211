////////////////////////////////////////////////////////////////////////////////
// File   : wlan_mac_ap.h
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
#ifndef WLAN_MAC_AP_H_
#define WLAN_MAC_AP_H_


// **********************************************************************
// Enable the WLAN UART Menu
//    NOTE:  To enable the WLAN Exp framework, please modify wlan_exp_common.h
#define WLAN_USE_UART_MENU
//#define ALLOW_ETH_TX_OF_WIRELESS_TX

// **********************************************************************
// UART Menu Modes
//
#define UART_MODE_MAIN                 0
#define UART_MODE_INTERACTIVE          1
#define UART_MODE_SSID_CHANGE          2
#define UART_MODE_LTG_SIZE_CHANGE	   3
#define UART_MODE_LTG_INTERVAL_CHANGE  4

// **********************************************************************
// Common Defines
//
#define MAX_ASSOCIATIONS               8
#define MAX_RETRY                      7


// **********************************************************************
// Timing Parameters
//

// Time between beacon transmissions
//
#define BEACON_INTERVAL_MS             (100)
#define BEACON_INTERVAL_US             (BEACON_INTERVAL_MS*1000)

// Time between association table check
// Periodically, the association table is culled through and inactive stations are explicitly purged.
//
#define ASSOCIATION_CHECK_INTERVAL_MS  (10000)
#define ASSOCIATION_CHECK_INTERVAL_US  (ASSOCIATION_CHECK_INTERVAL_MS*1000)

// The amount of time since the last time a station was heard from.
// After this interval, a station can be purged from the association table
//
//#define ASSOCIATION_TIMEOUT_S          (600)
#define ASSOCIATION_TIMEOUT_S          (600)
#define ASSOCIATION_TIMEOUT_US         (ASSOCIATION_TIMEOUT_S*1000000)

// When the node is in the state where it temporarily allows associations, this interval
// defines how long the window for new associations is open
//
#define ASSOCIATION_ALLOW_INTERVAL_MS  (30000)
#define ASSOCIATION_ALLOW_INTERVAL_US  (ASSOCIATION_ALLOW_INTERVAL_MS*1000)

// Time between blinking behavior in hex displays
//
#define ANIMATION_RATE_US              (100000)



/*********************** Global Structure Definitions ************************/




/*************************** Function Prototypes *****************************/


int  main();

void ltg_event(u32 id);

int  ethernet_receive(packet_bd_list* tx_queue_list, u8* eth_dest, u8* eth_src, u16 tx_length);

void mpdu_rx_process(void* pkt_buf_addr, u8 rate, u16 length);
void mpdu_transmit_done(tx_frame_info* tx_mpdu);
void check_tx_queue();

void beacon_transmit();

void enable_associations();
void disable_associations();
void association_timestamp_check();

void reset_station_statistics();
void deauthenticate_stations();

void animate_hex();
void up_button();

void uart_rx(u8 rxByte);

void print_menu();
void print_ltg_size_menu();
void print_ltg_interval_menu();
void print_ssid_menu();
void print_associations();
void print_queue_status();
void print_station_status(u8 manual_call);


#endif /* WLAN_MAC_AP_H_ */
