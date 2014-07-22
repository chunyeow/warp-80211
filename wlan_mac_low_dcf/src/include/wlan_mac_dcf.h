/** @file wlan_mac_dcf.h
 *  @brief Distributed Coordination Function
 *
 *  This contains code to implement the 802.11 DCF.
 *
 *  @copyright Copyright 2014, Mango Communications. All rights reserved.
 *          Distributed under the Mango Communications Reference Design License
 *				See LICENSE.txt included in the design archive or
 *				at http://mangocomm.com/802.11/license
 *
 *  @author Chris Hunter (chunter [at] mangocomm.com)
 *  @author Patrick Murphy (murphpo [at] mangocomm.com)
 *  @author Erik Welsh (welsh [at] mangocomm.com)
 *  @bug
 *  - EIFS value is currently aggressive (DIFS). Needs calibration.
 */

#ifndef WLAN_MAC_DCF_H_
#define WLAN_MAC_DCF_H_

#define TX_PKT_BUF_ACK 15

//RTS/CTS is not currently supported. This threshold must remain larger than any outgoing MPDU
#define RTS_THRESHOLD 2000

//TODO: In a future release, control of these parameters will be made available to CPU_HIGH through IPC commands
#define DCF_CW_EXP_MIN 4
#define DCF_CW_EXP_MAX 10

//CW Update Reasons
#define DCF_CW_UPDATE_MPDU_TX_ERR 0
#define DCF_CW_UPDATE_MPDU_RX_ACK 1
#define DCF_CW_UPDATE_BCAST_TX 2

#define MAC_HW_LASTBYTE_ADDR1 (13)

int main();
int frame_transmit(u8 pkt_buf, u8 rate, u16 length, wlan_mac_low_tx_details* low_tx_details);
u32 frame_receive(u8 rx_pkt_buf, u8 rate, u16 length);
inline int update_cw(u8 reason, u8 pkt_buf);
inline unsigned int rand_num_slots();
void wlan_mac_dcf_hw_start_backoff(u16 num_slots);
int wlan_create_ack_frame(void* pkt_buf, u8* address_ra);



#endif /* WLAN_MAC_DCF_H_ */
