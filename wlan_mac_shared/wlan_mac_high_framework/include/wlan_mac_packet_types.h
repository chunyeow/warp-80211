/** @file wlan_mac_packet_types.h
 *  @brief Packet Constructors
 *
 *  This contains code for constructing a variety of different types of MPDUs.
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


#ifndef WLAN_MAC_PACKET_TYPES_H_
#define WLAN_MAC_PACKET_TYPES_H_

typedef struct{
	u8* address_1;
	u8* address_2;
	u8* address_3;
	u8 frag_num;
	u8 reserved;
	u64 seq_num;
} mac_header_80211_common;

typedef struct{
	u16 auth_algorithm;
	u16 auth_sequence;
	u16 status_code;
} authentication_frame;

typedef struct{
	u16 reason_code;
} deauthentication_frame;

typedef struct{
	u16 capabilities;
	u16 status_code;
	u16 association_id;
} association_response_frame;

typedef struct{
	u16 capabilities;
	u16 listen_interval;
} association_request_frame;

typedef struct{
	u8 category;
	u8 action;
	u8 dialog_token;
	u8 element_id;
	u8 length;
	u8 measurement_token;
	u8 request_mode;
	u8 measurement_type;
	///Note, technically measurement action frames can follow this comment with additional fields of unknown length
	///But currently, the three types of measurements are all the same so for ease we'll hardcode that structure here
	u8 channel;
	u8 start_time[8];
	u8 duration[2];
} measurement_common_frame;

//#define MEASUREMENT_REQ_MODE_ENABLE  0x40
//#define MEASUREMENT_REQ_MODE_REQUEST 0x20
//#define MEASUREMENT_REQ_MODE_REPORT  0x10

#define MEASUREMENT_REQ_MODE_PARALLEL    0x01
#define MEASUREMENT_REQ_MODE_ENABLE	     0x02
#define MEASUREMENT_REQ_MODE_REPORTS     0x04
#define MEASUREMENT_REQ_MODE_AUTONOMOUS  0x08


#define MEASUREMENT_TYPE_BASIC 0
#define MEASUREMENT_TYPE_CCA 1
#define MEASUREMENT_TYPE_RPA 2


#define AUTH_ALGO_OPEN_SYSTEM 0x00

#define AUTH_SEQ_REQ 0x01
#define AUTH_SEQ_RESP 0x02

//Class 3 frame received from nonassociated STA
#define DEAUTH_REASON_INACTIVITY		4
#define DEAUTH_REASON_NONASSOCIATED_STA	7

// Status Codes: Table 7-23 in 802.11-2007
#define STATUS_SUCCESS 0
#define STATUS_AUTH_REJECT_UNSPECIFIED 1
#define STATUS_AUTH_REJECT_OUTSIDE_SCOPE 12
#define STATUS_AUTH_REJECT_CHALLENGE_FAILURE 15
#define STATUS_REJECT_TOO_MANY_ASSOCIATIONS 17

//#define wlan_create_beacon_frame(pkt_buf,common, beacon_interval, ssid_len, ssid, chan, tim_len, control, bitmap) wlan_create_beacon_probe_frame(pkt_buf, MAC_FRAME_CTRL1_SUBTYPE_BEACON, common, beacon_interval, ssid_len, ssid, chan, tim_len, control, bitmap)
//#define wlan_create_probe_resp_frame(pkt_buf,common, beacon_interval, ssid_len, ssid, chan, tim_len, control, bitmap) wlan_create_beacon_probe_frame(pkt_buf, MAC_FRAME_CTRL1_SUBTYPE_PROBE_RESP, common, beacon_interval, ssid_len, ssid, chan, tim_len, control, bitmap)
//int wlan_create_beacon_probe_frame(void* pkt_buf, u8 subtype, mac_header_80211_common* common, u16 beacon_interval, u8 ssid_len, u8* ssid, u8 chan, u8 tim_len, u8 tim_control, u8* tim_bitmap);

int wlan_create_beacon_frame(void* pkt_buf, mac_header_80211_common* common, u16 beacon_interval, u8 ssid_len, u8* ssid, u8 chan, u8 tim_len, u8 tim_control, u8* tim_bitmap);
int wlan_create_probe_resp_frame(void* pkt_buf, mac_header_80211_common* common, u16 beacon_interval, u8 ssid_len, u8* ssid, u8 chan);

int wlan_create_measurement_req_frame(void* pkt_buf, mac_header_80211_common* common, u8 measurement_type, u8 chan);
int wlan_create_probe_req_frame(void* pkt_buf, mac_header_80211_common* common, u8 ssid_len, u8* ssid, u8 chan);
int wlan_create_auth_frame(void* pkt_buf, mac_header_80211_common* common, u16 auth_algorithm,  u16 auth_seq, u16 status_code);
int wlan_create_deauth_frame(void* pkt_buf, mac_header_80211_common* common, u16 reason_code);
int wlan_create_association_response_frame(void* pkt_buf, mac_header_80211_common* common, u16 status, u16 AID);
#define wlan_create_association_req_frame(pkt_buf, common, ssid_len, ssid, num_basic_rates, basic_rates) wlan_create_reassoc_assoc_req_frame(pkt_buf, MAC_FRAME_CTRL1_SUBTYPE_ASSOC_REQ, common, ssid_len, ssid, num_basic_rates, basic_rates)
#define wlan_create_reassociation_req_frame(pkt_buf, common, ssid_len, ssid, num_basic_rates, basic_rates) wlan_create_reassoc_assoc_req_frame(pkt_buf, MAC_FRAME_CTRL1_SUBTYPE_REASSOC_REQ, common, ssid_len, ssid, num_basic_rates, basic_rates)
int wlan_create_reassoc_assoc_req_frame(void* pkt_buf, u8 frame_control_1, mac_header_80211_common* common, u8 ssid_len, u8* ssid, u8 num_basic_rates, u8* basic_rates);
int wlan_create_data_frame(void* pkt_buf, mac_header_80211_common* common, u8 flags);
u8 rate_union(u8* rate_vec_out, u8 num_rate_basic, u8* rate_basic, u8 num_rate_other, u8* rate_other);

#endif /* WLAN_MAC_PACKET_TYPES_H_ */
