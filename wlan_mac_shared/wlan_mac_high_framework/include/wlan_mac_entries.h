/** @file wlan_mac_entries.h
 *  @brief Event log
 *
 *  This contains the code for accessing event log.
 *
 *  @copyright Copyright 2014, Mango Communications. All rights reserved.
 *          Distributed under the Mango Communications Reference Design License
 *				See LICENSE.txt included in the design archive or
 *				at http://mangocomm.com/802.11/license
 *
 *	@note
 *  This is the only code that the user should modify in order to add entries
 *  to the event log.  To add a new entry, please follow the template provided
 *  and create:
 *    1) A new entry type in wlan_mac_entries.h
 *    2) Wrapper function:  get_next_empty_*_entry()
 *    3) Update the print function so that it is easy to print the log to the
 *    terminal
 *
 *  @author Chris Hunter (chunter [at] mangocomm.com)
 *  @author Patrick Murphy (murphpo [at] mangocomm.com)
 *  @author Erik Welsh (welsh [at] mangocomm.com)
 */

/***************************** Include Files *********************************/



/*************************** Constant Definitions ****************************/
#ifndef WLAN_MAC_ENTRIES_H_
#define WLAN_MAC_ENTRIES_H_

#include "wlan_mac_802_11_defs.h"
#include "wlan_mac_high.h"
#include "wlan_mac_misc_util.h"

#define WLAN_MAC_ENTRIES_LOG_CHAN_EST

// ****************************************************************************
// Define Entry Constants
//

//------------------------------------------------------------------------
// Entry Types

//-----------------------------------------------
// Management Entries

#define ENTRY_TYPE_NODE_INFO           1
#define ENTRY_TYPE_EXP_INFO            2
#define ENTRY_TYPE_STATION_INFO        3
#define ENTRY_TYPE_TEMPERATURE         4
#define ENTRY_TYPE_WN_CMD              5
#define ENTRY_TYPE_TIME_INFO           6

//-----------------------------------------------
// Receive Entries

#define ENTRY_TYPE_RX_OFDM             10
#define ENTRY_TYPE_RX_DSSS             11

//-----------------------------------------------
// Transmit Entries

#define ENTRY_TYPE_TX_HIGH             20
#define ENTRY_TYPE_TX_LOW              21

//-----------------------------------------------
// Statistics Entries

#define ENTRY_TYPE_TXRX_STATS          30




//------------------------------------------------------------------------
// MAC payload length

#define MIN_MAC_PAYLOAD_LOG_LEN                  24
#define MAX_MAC_PAYLOAD_LOG_LEN                  1500

// To never record payloads, you can set the min / max defines as follows:
//
//#define MAX_MAC_PAYLOAD_LOG_LEN                 MIN_MAC_PAYLOAD_LOG_LEN





/*********************** Global Structure Definitions ************************/

//-----------------------------------------------
// Node Info Entry
//   NOTE:  This structure was designed to work easily with the WARPNet Tag
//       Parameters.  The order and size of the fields match the corresponding
//       Tag Parameter so that population of this structure is easy.
//
//   NOTE:  This structure is always at the start of the event log.  There is
//       the methods to add this entry type to the log are in wlan_mac_event_log.*
//
typedef struct{
	u64                 timestamp;                         // Timestamp of the node info
	                                                       //   - This will reflect the oldest time of an
	                                                       //     entry for a given log wrap
	u32                 type;                              // WARPNet Node type
	u32                 id;                                // Node ID
	u32                 hw_gen;                            // WARP Hardware Generation
	u32                 wn_ver;                            // WARPNet version
	u64                 fpga_dna;                          // Node FPGA DNA
	u32                 serial_number;                     // Node serial number
	u32                 wlan_exp_ver;                      // WLAN Exp version
	u32                 wlan_mac_addr[2];                  // WLAN MAC Address
	u32                 wlan_scheduler_resolution;         // Minimum Scheduler resolution (microseconds)
} node_info_entry;


//-----------------------------------------------
// Experiment Info Entry
//
// NOTE:  When creating this entry, you need to allocate the size of the entry
//   plus the (info_length - 4).  For example:
//
//    (exp_info_entry *)wlan_exp_log_create_entry( ENTRY_TYPE_EXP_INFO, sizeof(exp_info_entry) + size - 4 )
//
// NOTE:  The longest Experiment Info is:  ((2^16 - 1) - (sizeof(exp_info_entry) - 4)) bytes
//
typedef struct{
	u64                 timestamp;               // Timestamp of the log entry
	u16                 info_type;               // Type of Experiment Info
	u16                 info_length;             // Length of the experiment info data (in bytes)
	u8                  info_payload[4];         // Reference to payload contents for easy access in C
	                                             //   You can treat this array as the first 4 bytes of
	                                             //   the info payload but the array is actually valid
	                                             //   for info_length bytes.
} exp_info_entry;


//-----------------------------------------------
// Station Info Entry
//
// Example request for a new station info entry:
//
//     (station_info_entry *)wlan_exp_log_create_entry( ENTRY_TYPE_STATION_INFO, sizeof(station_info_entry) )
//
typedef struct{
	u64                 timestamp;               // Timestamp of the log entry
	station_info_base   info;                    // Framework's station_info struct
} station_info_entry;

CASSERT(sizeof(station_info_entry) == 60, station_info_entry_alignment_check);


//-----------------------------------------------
// Temperature Entry
//   NOTE: The temperature values are copied directly from the system monitor and need
//         to be converted to Celsius:
//           celsius = ((double(temp)/65536.0)/0.00198421639) - 273.15;
//
// Example request for a new temperature entry:
//
//     (temperature_entry *)wlan_exp_log_create_entry( ENTRY_TYPE_TEMPERATURE, sizeof(temperature_entry) )
//
typedef struct{
	u64                 timestamp;               // Timestamp of the log entry
	u32                 id;                      // Node ID
	u32                 serial_number;           // Node serial number
	u32                 curr_temp;               // Current Temperature of the node
	u32                 min_temp;                // Minimum recorded temperature of the node
	u32                 max_temp;                // Maximum recorded temperature of the node
} temperature_entry;


//-----------------------------------------------
// WARPNet Command Entry
//
// Example request for a new WARPNet command info entry:
//
//     (wn_cmd_entry *)wlan_exp_log_create_entry( ENTRY_TYPE_WN_CMD, sizeof(wn_cmd_entry) )
//
typedef struct{
	u64                 timestamp;               // Timestamp of the log entry
	u32                 command;                 // WARPNet command
	u16                 src_id;                  // Source ID of the command
	u16                 num_args;                // Number of arguments
	u32                 args[10];	             // Data from the arguments
} wn_cmd_entry;


//-----------------------------------------------
// Time Info Entry
//
// Example request for a new Time info entry:
//
//     (time_info_entry *)wlan_exp_log_create_entry( ENTRY_TYPE_TIME_INFO, sizeof(time_info_entry) )
//
typedef struct{
	u64                 timestamp;               // Timestamp of the log entry (old timebase)
	u32                 time_id;                 // ID of the time info entry so that these entries
	                                             //   can be synced across multiple nodes
	u32                 reason;                  // Reason code for log entry:
	                                             //     0 - WN_SET_TIME
	                                             //     1 - BEACON
	                                             //     2 - WN_ADD_LOG
	u64                 new_time;                // New timebase  (0xFFFFFFFF_FFFFFFFF if unchanged)
	u64                 abs_time;                // Absolute time (0xFFFFFFFF_FFFFFFFF if not known)
} time_info_entry;

#define TIME_INFO_ENTRY_WN_SET_TIME              0
#define TIME_INFO_ENTRY_BEACON                   1
#define TIME_INFO_ENTRY_WN_ADD_LOG               2


//-----------------------------------------------
// TxRx Statistics Entry
//
//   NOTE:  To add TxRx Statistics to the log, please use one of the methods provided
//     in wlan_mac_event_log.*
//
typedef struct{
    u64                 timestamp;               // Timestamp of the log entry
	statistics_txrx     stats;                   // Framework's statistics struct
} txrx_stats_entry;


//-----------------------------------------------
// Common Receive Entry
//   NOTE:  rsvd field is to have a 32-bit aligned struct.  That way sizeof()
//          accurately reflects the number of bytes in the struct.
//
typedef struct{
	u64                 timestamp;               // Timestamp of the log entry
	u16	                length;                  // Length of the received packet
	u8                  rate;                    // Rate at which the packet was received
	s8                  power;                   // Power of the received packet
	u8	                fcs_status;              // FCS Status of the packet
	u8 	                pkt_type;                // Type of packet
	u8 	                chan_num;                // Channel on which the packet was received
	u8 	                ant_mode;                // Antenna mode of the received packet
	u8                  rf_gain;                 // RF gain of the received packet
	u8                  bb_gain;                 // Baseband gain of the received packet
	u16                 flags;                   // Flags
} rx_common_entry;

#define RX_ENTRY_FCS_GOOD                        0
#define RX_ENTRY_FCS_BAD                         1

#define RX_ENTRY_FLAGS_IS_DUPLICATE	             0x0001


//-----------------------------------------------
// Receive OFDM Entry
//
//   NOTE:  Entry mac_payload stored as u32 array to preserve alignment.
//
// Example request for a new receive OFDM entry:
//
//     (rx_ofdm_entry *)wlan_exp_log_create_entry( ENTRY_TYPE_RX_OFDM, sizeof(rx_ofdm_entry) + extra_payload  )
//
typedef struct{
	rx_common_entry     rx_entry;

#ifdef WLAN_MAC_ENTRIES_LOG_CHAN_EST
	u32	                channel_est[64];         // Channel estimates for the packet
#endif

	u32                 mac_payload_log_len;     // Number of payload bytes actually recorded in log entry
    u32                 mac_payload[MIN_MAC_PAYLOAD_LOG_LEN/4];
} rx_ofdm_entry;


//-----------------------------------------------
// Receive DSSS Entry
//
//   NOTE:  Entry mac_payload stored as u32 array to preserve alignment.
//
// Example request for a new receive DSSS entry:
//
//     (rx_dsss_entry *)wlan_exp_log_create_entry( ENTRY_TYPE_RX_DSSS, sizeof(rx_dsss_entry) + extra_payload )
//
typedef struct{
	rx_common_entry     rx_entry;
	u32                 mac_payload_log_len;     // Number of payload bytes actually recorded in log entry
	u32                 mac_payload[MIN_MAC_PAYLOAD_LOG_LEN/4];
} rx_dsss_entry;


//-----------------------------------------------
// High-level Transmit Entry
//   NOTE:  padding field is to have a 32-bit aligned struct.  That way sizeof()
//          accurately reflects the number of bytes in the struct.
//
//   NOTE:  Entry mac_payload stored as u32 array to preserve alignment.
//
// Example request for a new high-level transmit entry:
//
//     (tx_high_entry *)wlan_exp_log_create_entry( ENTRY_TYPE_TX_HIGH, sizeof(tx_high_entry) + extra_payload )
//
typedef struct{
	u64                 timestamp_create;        // Timestamp of the log entry creation
	u32                 delay_accept;            // Delay from timestamp_create to when accepted by CPU Low
	u32                 delay_done;              // Delay from delay_accept to when CPU Low was done
	u64	                unique_seq;              // Unique packet sequence number
	u8                  num_tx;                  // Number of Transmissions that it took to send the packet
	s8 	                power;                   // Power the packet was sent at
	u8                  chan_num;                // Channel on which the packet was sent
	u8                  rate;                    // Rate at which the packet was sent
	u16                 length;                  // Length of the packet
	u8                  result;                  // Result of the transmission
	u8                  pkt_type;                // Type of packet
	u8	                ant_mode;                // Antenna mode used for transmission
	u8					queue_id;				 // Queue ID this packet was sent from
	u8	                padding[2];              // Padding for alignment
	u32                 mac_payload_log_len;     // Number of payload bytes actually recorded in log entry
	u32                 mac_payload[MIN_MAC_PAYLOAD_LOG_LEN/4];
} tx_high_entry;


//-----------------------------------------------
// Low-level Transmit Entry
//   NOTE:  rsvd field is to have a 32-bit aligned struct.  That way sizeof()
//          accurately reflects the number of bytes in the struct.
//
//   NOTE:  Entry mac_payload stored as u32 array to preserve alignment.
//
// Example request for a new low-level transmit entry:
//
//     (tx_low_entry *)wlan_exp_log_create_entry( ENTRY_TYPE_TX_LOW, sizeof(tx_low_entry) )
//
typedef struct{
	u64                 timestamp_send;          // Timestamp of when packet was sent
	u64	                unique_seq;              // Unique packet sequence number
	phy_tx_params       phy_params;              // Transmission parameters
	u8	                transmission_count;      // What transmission was this packet
	u8 	                chan_num;                // Channel on which this packet was sent
	u16                 length;                  // Length of the packet
	u16                 num_slots;               // Number of backoff slots
	u16                 cw;                      // Contention Window
	u8 	                pkt_type;                // Type of packet
	u8					flags;					 // Misc. flags from CPU_HIGH
	u8	                reserved[2];             //
	u32                 mac_payload_log_len;     // Number of payload bytes actually recorded in log entry
	u32                 mac_payload[MIN_MAC_PAYLOAD_LOG_LEN/4];
} tx_low_entry;

#define TX_LOW_FLAGS_WAS_ACKED 0x01


/*************************** Function Prototypes *****************************/

extern u32 mac_payload_log_len;


/*************************** Function Prototypes *****************************/

//-----------------------------------------------
// Method to set the global variable mac_payload_log_len
//
void     wlan_exp_log_set_mac_payload_len(u32 payload_len);


//-----------------------------------------------
// Wrapper method to get an entry
//
void *   wlan_exp_log_create_entry(u16 entry_type_id, u16 entry_size);


//-----------------------------------------------
// Print function for all entries
//
#ifdef _DEBUG_
void print_entry( u32 entry_number, u32 entry_type, void * entry );
#endif

#endif /* WLAN_MAC_ENTRIES_H_ */
