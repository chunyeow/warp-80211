/** @file wlan_mac_high.h
 *  @brief Low-level WLAN MAC High Framework
 *
 *  This contains the low-level code for accessing the WLAN MAC Low Framework.
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

#ifndef WLAN_MAC_LOW_H_
#define WLAN_MAC_LOW_H_

#include "wlan_mac_ipc_util.h"

#define USERIO_BASEADDR     XPAR_W3_USERIO_BASEADDR


/*************** WLAN MAC HW ***************/
//Register renames
//RO:
#define WLAN_MAC_REG_STATUS					XPAR_WLAN_MAC_DCF_HW_MEMMAP_STATUS
#define WLAN_MAC_REG_TIMESTAMP_MSB			XPAR_WLAN_MAC_DCF_HW_MEMMAP_TIMESTAMP_MSB
#define WLAN_MAC_REG_TIMESTAMP_LSB			XPAR_WLAN_MAC_DCF_HW_MEMMAP_TIMESTAMP_LSB
#define WLAN_MAC_REG_LATEST_RX_BYTE			XPAR_WLAN_MAC_DCF_HW_MEMMAP_LATEST_RX_BYTE
#define WLAN_MAC_REG_RX_RATE_LENGTH			XPAR_WLAN_MAC_DCF_HW_MEMMAP_RX_RATE_LENGTH
#define WLAN_MAC_REG_RX_TIMESTAMP_LSB		XPAR_WLAN_MAC_DCF_HW_MEMMAP_RX_START_TIMESTAMP_LSB
#define WLAN_MAC_REG_RX_TIMESTAMP_MSB		XPAR_WLAN_MAC_DCF_HW_MEMMAP_RX_START_TIMESTAMP_MSB
#define WLAN_MAC_REG_TX_TIMESTAMP_LSB		XPAR_WLAN_MAC_DCF_HW_MEMMAP_TX_START_TIMESTAMP_LSB
#define WLAN_MAC_REG_TX_TIMESTAMP_MSB		XPAR_WLAN_MAC_DCF_HW_MEMMAP_TX_START_TIMESTAMP_MSB

//RW:
#define WLAN_MAC_REG_AUTO_TX_PARAMS			XPAR_WLAN_MAC_DCF_HW_MEMMAP_AUTO_TX_PARAMS
#define WLAN_MAC_REG_CALIB_TIMES			XPAR_WLAN_MAC_DCF_HW_MEMMAP_CALIB_TIMES
#define WLAN_MAC_REG_IFS_1					XPAR_WLAN_MAC_DCF_HW_MEMMAP_IFS_INTERVALS1
#define WLAN_MAC_REG_IFS_2					XPAR_WLAN_MAC_DCF_HW_MEMMAP_IFS_INTERVALS2
#define WLAN_MAC_REG_MPDU_TX_START			XPAR_WLAN_MAC_DCF_HW_MEMMAP_TX_START
#define WLAN_MAC_REG_MPDU_TX_PARAMS			XPAR_WLAN_MAC_DCF_HW_MEMMAP_MPDU_TX_PARAMS
#define WLAN_MAC_REG_CONTROL				XPAR_WLAN_MAC_DCF_HW_MEMMAP_CONTROL
#define WLAN_MAC_REG_SW_BACKOFF_CTRL		XPAR_WLAN_MAC_DCF_HW_MEMMAP_BACKOFF_CTRL
#define WLAN_MAC_REG_SET_TIMESTAMP_LSB		XPAR_WLAN_MAC_DCF_HW_MEMMAP_TIMESTAMP_SET_LSB
#define WLAN_MAC_REG_SET_TIMESTAMP_MSB		XPAR_WLAN_MAC_DCF_HW_MEMMAP_TIMESTAMP_SET_MSB

//Bit masks for STATUS register
#define WLAN_MAC_STATUS_MASK_MPDU_TX_PENDING	0x0000800 //b[11]
#define WLAN_MAC_STATUS_MASK_MPDU_TX_DONE		0x0001000 //b[12]
#define WLAN_MAC_STATUS_MASK_PHY_TX_ACTIVE		0x0002000 //b[13]
#define WLAN_MAC_STATUS_MASK_PHY_RX_ACTIVE		0x0004000 //b[14]
#define WLAN_MAC_STATUS_MASK_MPDU_TX_RESULT		0x0018000 //b[16:15]
#define WLAN_MAC_STATUS_MASK_MPDU_TX_STATE		0x00E0000 //b[19:17]
#define WLAN_MAC_STATUS_MASK_NAV_BUSY			0x0100000 //b[20]
#define WLAN_MAC_STATUS_MASK_PHY_CCA_BUSY		0x0200000 //b[21]
#define WLAN_MAC_STATUS_MASK_AUTO_TX_PENDING	0x0400000 //b[22]
#define WLAN_MAC_STATUS_MASK_RX_FCS_GOOD		0x0800000 //b[23]
#define WLAN_MAC_STATUS_MASK_RX_PHY_BLOCKED		0x1000000 //b[24]

#define WLAN_MAC_STATUS_MPDU_TX_RESULT_SUCCESS		(0<<15)
#define WLAN_MAC_STATUS_MPDU_TX_RESULT_TIMED_OUT	(1<<15)
#define WLAN_MAC_STATUS_MPDU_TX_RESULT_RX_STARTED	(2<<15)

#define WLAN_MAC_STATUS_MPDU_TX_STATE_IDLE 			(0<<17)
#define WLAN_MAC_STATUS_MPDU_TX_STATE_DO_TX 		(1<<17)
#define WLAN_MAC_STATUS_MPDU_TX_STATe_START_BO		(2<<17)
#define WLAN_MAC_STATUS_MPDU_TX_STATE_DEFER 		(3<<17)
#define WLAN_MAC_STATUS_MPDU_TX_STATE_POST_TX 		(4<<17)
#define WLAN_MAC_STATUS_MPDU_TX_STATE_POST_TX_WAIT 	(5<<17)
#define WLAN_MAC_STATUS_MPDU_TX_STATE_DONE		 	(6<<17)

//Bit masks for CONTROL register
#define WLAN_MAC_CTRL_MASK_RESET					0x001
#define WLAN_MAC_CTRL_MASK_RX_PHY_BLOCK_EN			0x002
#define WLAN_MAC_CTRL_MASK_RX_PHY_BLOCK_RESET		0x004
#define WLAN_MAC_CTRL_MASK_DISABLE_NAV				0x008
#define WLAN_MAC_CTRL_MASK_BLOCK_RX_ON_TX			0x010
#define WLAN_MAC_CTRL_MASK_UPDATE_TIMESTAMP			0x020
#define WLAN_MAC_CTRL_MASK_BLOCK_RX_ON_VALID_RXEND	0x040 //blocks Rx on bad FCS pkts, only for logging/analysis
#define WLAN_MAC_CTRL_MASK_CCA_IGNORE_PHY_CS		0x080
#define WLAN_MAC_CTRL_MASK_CCA_IGNORE_TX_BUSY		0x100
#define WLAN_MAC_CTRL_MASK_CCA_IGNORE_NAV			0x200
#define WLAN_MAC_CTRL_MASK_RESET_NAV				0x400
#define WLAN_MAC_CTRL_MASK_RESET_BACKOFF			0x800

//Macros for reading/writing registers
#define wlan_mac_timestamp_lsb() Xil_In32(WLAN_MAC_REG_TIMESTAMP_LSB)
#define wlan_mac_timestamp_msb() Xil_In32(WLAN_MAC_REG_TIMESTAMP_MSB)

//WLAN_MAC_REG_SW_BACKOFF_CTRL:
// b[15:0]: Num Slots
// b[31]: Start
#define wlan_mac_set_backoff_num_slots(d) Xil_Out32(WLAN_MAC_REG_SW_BACKOFF_CTRL, ((Xil_In32(WLAN_MAC_REG_SW_BACKOFF_CTRL) & (~0x0000FFFF)) | ((d) & 0x0000FFFF)))
#define wlan_mac_backoff_start(x) Xil_Out32(WLAN_MAC_REG_SW_BACKOFF_CTRL, ((Xil_In32(WLAN_MAC_REG_SW_BACKOFF_CTRL) & (~0x80000000)) | (((x)<<31) & 0x80000000)))

//WLAN_MAC_IFS_1:
// b[9:0]: Slot
// b[29:20]: DIFS
#define wlan_mac_set_slot(d) Xil_Out32(WLAN_MAC_REG_IFS_1, ((Xil_In32(WLAN_MAC_REG_IFS_1) & (~0x000003FF)) | ((d) & 0x000003FF)))
#define wlan_mac_set_DIFS(d) Xil_Out32(WLAN_MAC_REG_IFS_1, ((Xil_In32(WLAN_MAC_REG_IFS_1) & (~0x3FF00000)) | (((d)<<20) & 0x3FF00000)))

//WLAN_MAC_IFS_2:
// b[15:0]: EIFS
// b[31:16]: ACK Timeout
#define wlan_mac_set_EIFS(d) Xil_Out32(WLAN_MAC_REG_IFS_2, ((Xil_In32(WLAN_MAC_REG_IFS_2) & (~0x0000FFFF)) | ((d) & 0x0000FFFF)))
#define wlan_mac_set_timeout(d) Xil_Out32(WLAN_MAC_REG_IFS_2, ((Xil_In32(WLAN_MAC_REG_IFS_2) & (~0xFFFF0000)) | (((d)<<16) & 0xFFFF0000)))

//WLAN_MAC_CALIB_TIMES:
// b[9:0]: TxDIFS
// b[31:24]: NAV Adj (Fix8_0 - signed char!)
#define wlan_mac_set_TxDIFS(d) Xil_Out32(WLAN_MAC_REG_CALIB_TIMES, ((Xil_In32(WLAN_MAC_REG_CALIB_TIMES) & (~0x000003FF)) | ((d) & 0x000003FF)))
#define wlan_mac_set_NAV_adj(d) Xil_Out32(WLAN_MAC_REG_CALIB_TIMES, ((Xil_In32(WLAN_MAC_REG_CALIB_TIMES) & (~0xFF000000)) | (((d)<<20) & 0xFF000000)))

//WLAN_MAC_REG_AUTO_TX_PARAMS:
// b[3:0]: Pkt buf
// b[13:4]: Pre-auto-Tx delay (MAC_SIFS)
// b[23:20]: Tx ant mask
// b[31]: Auto-Tx en
#define wlan_mac_auto_tx_params(pktBuf, preTx_delay, txGain, antMask) Xil_Out32(WLAN_MAC_REG_AUTO_TX_PARAMS, ( \
																				 (pktBuf) & 0xF) | \
																				 (((antMask) & 0xF)<<20) | \
																				 (((preTx_delay) & 0x3FF) << 4) | \
																				 (((txGain&0x3F)<<25)))

#define wlan_mac_auto_tx_en(x) Xil_Out32(WLAN_MAC_REG_AUTO_TX_PARAMS,((Xil_In32(WLAN_MAC_REG_AUTO_TX_PARAMS) & 0x7FFFFFFF)) | (((x) & 0x1) << 31))

//WLAN_MAC_MPDU_TX_PARAMS:
// b[3:0]: Pkt buf
// b[7:4]: Ant mask (0x1=RFA, 0x2=RFB)
// b[23:8]: Num pre-Tx backoff slots
// b[24]: Start post-Tx timeout
// b[30:25]: Tx gain
#define wlan_mac_MPDU_tx_params(pktBuf, preTx_backoff_slots, postTx_timeout_req, txGain, antMask) \
	Xil_Out32(WLAN_MAC_REG_MPDU_TX_PARAMS, \
			(pktBuf & 0xF) | \
			((antMask & 0xF) << 4) | \
			((preTx_backoff_slots & 0xFFFF) << 8) | \
			((postTx_timeout_req & 0x1) << 24) | \
			((txGain & 0x3F) << 25))

#define wlan_mac_MPDU_tx_start(x) Xil_Out32(WLAN_MAC_REG_MPDU_TX_START, (x&0x1))

#define wlan_mac_reset(x) Xil_Out32(WLAN_MAC_REG_CONTROL, (Xil_In32(WLAN_MAC_REG_CONTROL) & ~WLAN_MAC_CTRL_MASK_RESET) | (WLAN_MAC_CTRL_MASK_RESET & x))

#define wlan_mac_get_last_byte_index() (Xil_In32(WLAN_MAC_REG_LATEST_RX_BYTE) & 0x3FFF)
#define wlan_mac_get_last_byte() ((Xil_In32(WLAN_MAC_REG_LATEST_RX_BYTE) & 0x3FC000) >> 14)

#define wlan_mac_get_status() (Xil_In32(WLAN_MAC_REG_STATUS))
#define wlan_mac_get_backoff_count (Xil_In32(WLAN_MAC_REG_STATUS) & 0x7FF)

//WLAN_MAC_REG_RX_RATE_LENGTH:
// b[11:0]: Rx Length (in bytes)
// b[19:17]: Rx rate (4-bit code)
// b[24]: Rx PHY sel (1=OFDM, 0=DSSS)
#define wlan_mac_get_rx_phy_length() (Xil_In32(WLAN_MAC_REG_RX_RATE_LENGTH) & 0xFFF)
#define wlan_mac_get_rx_phy_rate() ((Xil_In32(WLAN_MAC_REG_RX_RATE_LENGTH) & 0xF0000) >> 16)
#define wlan_mac_get_rx_phy_sel() ((Xil_In32(WLAN_MAC_REG_RX_RATE_LENGTH) & 0x1000000) >> 24)

//MAC Timing Parameters
#define T_SLOT 9
#define T_SIFS 10
#define T_DIFS (T_SIFS + 2*T_SLOT)
#define T_EIFS 88
#define T_PHY_RX_START_DLY 25
#define T_TIMEOUT (T_SIFS+T_SLOT+T_PHY_RX_START_DLY)

#define WLAN_RX_PHY_DSSS	0
#define WLAN_RX_PHY_OFDM	1

#define POLL_MAC_STATUS_RECEIVED_PKT	0x00000001 //b[0]
#define POLL_MAC_STATUS_GOOD  		 	0x00000002 //b[1]
#define POLL_MAC_ADDR_MATCH				0x00000004 //b[2]
#define POLL_MAC_STATUS_TYPE			0x0000FF00 //b[15:8]

#define POLL_MAC_TYPE_DATA				(0<<8)
#define POLL_MAC_TYPE_ACK				(1<<8)
#define POLL_MAC_TYPE_OTHER				(255<<8)


//WN Low Params
#define LOW_PARAM_PHYSICAL_CS_THRESH	1


int wlan_mac_low_init(u32 type);
void wlan_mac_low_finish_init();
void wlan_mac_low_dcf_init();
inline void wlan_mac_low_send_exception(u32 reason);
inline void wlan_mac_low_poll_ipc_rx();
void process_ipc_msg_from_high(wlan_ipc_msg* msg);
void wlan_mac_low_set_time(u64 new_time);
void process_config_mac(ipc_config_mac* config_mac);
void wlan_mac_low_init_hw_info( u32 type );
wlan_mac_hw_info* wlan_mac_low_get_hw_info();
inline u32 wlan_mac_low_get_active_channel();
inline s8 wlan_mac_low_get_current_ctrl_tx_pow();
inline u32 wlan_mac_low_get_current_rx_filter();
inline int wlan_mac_low_calculate_rx_power(u16 rssi, u8 lna_gain);
inline u32 wlan_mac_low_poll_frame_rx();
void wlan_mac_low_set_frame_rx_callback(function_ptr_t callback);
void wlan_mac_low_set_frame_tx_callback(function_ptr_t callback);
void wlan_mac_low_frame_ipc_send();
inline void wlan_mac_low_lock_empty_rx_pkt_buf();
inline u64 get_usec_timestamp();
inline u64 get_rx_start_timestamp();
inline u64 get_tx_start_timestamp();
void wlan_mac_dcf_hw_unblock_rx_phy();
inline u32 wlan_mac_dcf_hw_rx_finish();
inline u8 wlan_mac_low_dbm_to_gain_target(s8 power);
inline void wlan_mac_reset_backoff_counter();
inline void wlan_mac_reset_NAV_counter();

#endif /* WLAN_MAC_LOW_H_ */
