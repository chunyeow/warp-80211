////////////////////////////////////////////////////////////////////////////////
// File   : wlan_phy_util.c
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
#include "stdarg.h"
#include "xtmrctr.h"
#include "xio.h"
#include "string.h"

//WARP includes
#include "w3_userio.h"
#include "w3_ad_controller.h"
#include "w3_clock_controller.h"
#include "w3_iic_eeprom.h"
#include "radio_controller.h"

//WLAN design includes
#include "wlan_mac_ipc_util.h"
#include "wlan_mac_misc_util.h"
#include "wlan_phy_util.h"

//LUT of number of ones in each byte (used to calculate PARITY in SIGNAL)
const u8 ones_in_chars[256] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8};

//Global variable definitions
XTmrCtr TimerCounter; /* The instance of the Tmrctr Device */

int w3_node_init() {

	int status;
	XTmrCtr *TmrCtrInstancePtr = &TimerCounter;
	int ret = XST_SUCCESS;

	microblaze_enable_exceptions();

	//Initialize the AD9512 clock buffers (RF reference and sampling clocks)
	status = clk_init(CLK_BASEADDR, 2);
	if(status != XST_SUCCESS) {
		xil_printf("w3_node_init: Error in clk_init (%d)\n", status);
		ret = XST_FAILURE;
	}

	//Initialize the AD9963 ADCs/DACs for on-board RF interfaces
	ad_init(AD_BASEADDR, (RFA_AD_CS | RFB_AD_CS), 3);
	xil_printf("AD Readback: 0x%08x\n", ad_spi_read(AD_BASEADDR, RFA_AD_CS, 0x32));

	if(status != XST_SUCCESS) {
		xil_printf("w3_node_init: Error in ad_init (%d)\n", status);
		ret = XST_FAILURE;
	}

	//Initialize the radio_controller core and MAX2829 transceivers for on-board RF interfaces
	status = radio_controller_init(RC_BASEADDR, (RC_RFA | RC_RFB), 1, 1);
	if(status != XST_SUCCESS) {
		xil_printf("w3_node_init: Error in radioController_initialize (%d)\n", status);
		ret = XST_FAILURE;
	}

	//Initialize the EEPROM read/write core
	iic_eeprom_init(EEPROM_BASEADDR, 0x64);
	if(status != XST_SUCCESS) {
		xil_printf("w3_node_init: Error in IIC_EEPROM_init (%d)\n", status);
		ret = XST_FAILURE;
	}

	//Initialize the timer counter
	status = XTmrCtr_Initialize(TmrCtrInstancePtr, TMRCTR_DEVICE_ID);
	if (status != XST_SUCCESS) {
		xil_printf("w3_node_init: Error in XtmrCtr_Initialize (%d)\n", status);
		ret = XST_FAILURE;
	}

	// Set timer 0 to into a "count down" mode
	XTmrCtr_SetOptions(TmrCtrInstancePtr, 0, (XTC_DOWN_COUNT_OPTION));

	//Give the PHY control of the red user LEDs (PHY counts 1-hot on SIGNAL errors)
	userio_set_ctrlSrc_hw(USERIO_BASEADDR, W3_USERIO_CTRLSRC_LEDS_RED);

	return ret;
}

void wlan_phy_set_tx_signal(u8 pkt_buf, u8 rate, u16 length){
	Xil_Out32((u32*)(TX_PKT_BUF_TO_ADDR(pkt_buf) + PHY_TX_PKT_BUF_PHY_HDR_OFFSET), WLAN_TX_SIGNAL_CALC(rate, length));
}

void wlan_phy_init() {

	//Assert Tx and Rx resets
	REG_SET_BITS(WLAN_RX_REG_CTRL, WLAN_RX_REG_CTRL_RESET);
	REG_SET_BITS(WLAN_TX_REG_CFG, WLAN_TX_REG_CFG_RESET);
	

/************ PHY Rx ************/

	//Enable DSSS Rx by default
	wlan_phy_DSSS_rx_enable();
	REG_SET_BITS(WLAN_RX_REG_CFG, WLAN_RX_REG_CFG_DSSS_RX_AGC_HOLD);

	REG_CLEAR_BITS(WLAN_RX_REG_CFG, WLAN_RX_REG_CFG_CFO_EST_BYPASS);

	//Enable write-enable byte order swap
	REG_SET_BITS(WLAN_RX_REG_CFG, WLAN_RX_REG_CFG_PKT_BUF_WEN_SWAP);

	//Block Rx inputs during Tx
	REG_SET_BITS(WLAN_RX_REG_CFG, WLAN_RX_REG_CFG_USE_TX_SIG_BLOCK);

	//Set LTS correlation threshold and timeout
	wlan_phy_rx_lts_corr_config(600 * PHY_RX_RSSI_SUM_LEN, 320/2);//SNR thresh, timeout/2
	wlan_phy_rx_lts_corr_thresholds(12500, 17000);//low SNR, high SNR corr thresh

	//Configure RSSI pkt det
	//wlan_phy_rx_pktDet_RSSI_cfg(8, 0xFFFF, 4); //Disable RSSI pkt det with high thresh
	wlan_phy_rx_pktDet_RSSI_cfg(PHY_RX_RSSI_SUM_LEN, (PHY_RX_RSSI_SUM_LEN * 300), 4); //Disable RSSI pkt det with high thresh
	
	//Configure auto-corr pkt det
	wlan_phy_rx_pktDet_autoCorr_cfg(200, 250, 4, 0x3F);
//	wlan_phy_rx_pktDet_autoCorr_cfg(255, 4095, 4, 0x3F); //Disable auto-corr with high thresh

	wlan_phy_rx_set_cca_thresh(PHY_RX_RSSI_SUM_LEN * 750);
	//wlan_phy_rx_set_cca_thresh(8*1023);

	wlan_phy_rx_set_extension(120); //num samp periods post done to extend CCA BUSY

	//Sane defaults for DSSS Rx (code_corr, timeout, despread_dly, length_pad)
	wlan_phy_DSSS_rx_config(0x600, 200, 5, 5);
	//wlan_phy_DSSS_rx_config(0xFFFF, 150, 5, 5);

/************ PHY Tx ************/
	//Setup PHY Tx
	REG_CLEAR_BITS(WLAN_TX_REG_START, 0xFFFFFFFF);//De-assert all starts

	//Set Tx duration extension, in units of sample periods (120=6usec)
	wlan_phy_tx_set_extension(120);

	//Set extension from last samp output to RF Tx -> Rx transition
	//Note: Old value of 20 was too short for 16-QAM packets
	wlan_phy_tx_set_txen_extension(50);

	//Set extension from RF Rx -> Tx to un-blocking Rx samples
	wlan_phy_tx_set_rx_invalid_extension(100);

	//Set digital scaling of preamble/payload signals before DACs (UFix12_0)
	wlan_phy_tx_set_scaling(0x2000, 0x2000);

/*********** AGC ***************/

	//Post Rx_done reset delays for [rxhp, g_rf, g_bb]
	wlan_agc_set_reset_timing(4, 250, 250);

	//RFG Thresh 3->2, 2->1, Avg_len_sel, V_DB_Adj, Init G_BB
	//wlan_agc_set_config( (256-56), (256-35), 0, 6, 24);
	wlan_agc_set_config( (256-56), (256-15), 0, 6, 24);


	//capt_rssi_1, capt_rssi_2, capt_v_db, agc_done
	wlan_agc_set_AGC_timing(1, 30, 90, 96);

	//start_dco, en_iir_filt
	wlan_agc_set_DCO_timing(100, (100+34));

	wlan_agc_set_target( (64-16) );

/************ Wrap Up ************/
	//Let PHY Tx take control of radio TXEN/RXEN
	REG_SET_BITS(WLAN_TX_REG_CFG, WLAN_TX_REG_CFG_SET_RC_RXEN);
	
	Xil_Out32(XPAR_WLAN_PHY_RX_MEMMAP_RSSI_THRESH, 300);

	//De-assert resets
	REG_CLEAR_BITS(WLAN_RX_REG_CTRL, WLAN_RX_REG_CTRL_RESET);
	REG_CLEAR_BITS(WLAN_TX_REG_CFG, WLAN_TX_REG_CFG_RESET);

	return;
}

void wlan_radio_init() {

	//Setup clocking and filtering (20MSps, 2x interp/decimate in AD9963)
	clk_config_dividers(CLK_BASEADDR, 2, (CLK_SAMP_OUTSEL_AD_RFA | CLK_SAMP_OUTSEL_AD_RFB));
	ad_config_filters(AD_BASEADDR, (RFA_AD_CS | RFB_AD_CS), 2, 2);

	//Setup RFA
	radio_controller_TxRxDisable(RC_BASEADDR, RC_RFA);
	radio_controller_apply_TxDCO_calibration(AD_BASEADDR, EEPROM_BASEADDR, RC_RFA);

	radio_controller_setCenterFrequency(RC_BASEADDR, RC_RFA, RC_24GHZ, 9);


	radio_controller_setRadioParam(RC_BASEADDR, RC_RFA, RC_PARAMID_RSSI_HIGH_BW_EN, 0);

	//Filter bandwidths
	radio_controller_setRadioParam(RC_BASEADDR, RC_RFA, RC_PARAMID_RXHPF_HIGH_CUTOFF_EN, 1);
	radio_controller_setRadioParam(RC_BASEADDR, RC_RFA, RC_PARAMID_RXLPF_BW, 1);
	radio_controller_setRadioParam(RC_BASEADDR, RC_RFA, RC_PARAMID_TXLPF_BW, 1);

#if 0
	//MGC
	radio_controller_setCtrlSource(RC_BASEADDR, RC_RFA, RC_REG0_RXHP_CTRLSRC, RC_CTRLSRC_REG);
	radio_controller_setRxHP(RC_BASEADDR, RC_RFA, RC_RXHP_OFF);
	radio_controller_setRxGainSource(RC_BASEADDR, RC_RFA, RC_GAINSRC_SPI);

	//Set Rx gains
	radio_controller_setRadioParam(RC_BASEADDR, RC_RFA, RC_PARAMID_RXGAIN_RF, 3);
	radio_controller_setRadioParam(RC_BASEADDR, RC_RFA, RC_PARAMID_RXGAIN_BB, 9);
#else
	//AGC
	radio_controller_setCtrlSource(RC_BASEADDR, RC_RFA, RC_REG0_RXHP_CTRLSRC, RC_CTRLSRC_HW);
	radio_controller_setRxGainSource(RC_BASEADDR, RC_RFA, RC_GAINSRC_HW);
#endif

	//Set Tx gains
	radio_controller_setTxGainSource(RC_BASEADDR, RC_RFA, RC_GAINSRC_REG);
	radio_controller_setRadioParam(RC_BASEADDR, RC_RFA, RC_PARAMID_TXGAIN_BB, 2);
	radio_controller_setTxGainTarget(RC_BASEADDR, RC_RFA, 45);

	//TODO: With this much TX gain, we are very likely seeing some clipping at the TX. That said, I haven't seen any performance
	//degradation from it. I have seen a big boost in range with the higher power.
//	radio_controller_setRadioParam(RC_BASEADDR, RC_RFA, RC_PARAMID_TXGAIN_BB, 3);
//	radio_controller_setTxGainTarget(RC_BASEADDR, RC_RFA, 60);
	
	//Set misc radio params
	radio_controller_setRadioParam(RC_BASEADDR, RC_RFA, RC_PARAMID_TXLINEARITY_PADRIVER, 0);
	radio_controller_setRadioParam(RC_BASEADDR, RC_RFA, RC_PARAMID_TXLINEARITY_VGA, 0);
	radio_controller_setRadioParam(RC_BASEADDR, RC_RFA, RC_PARAMID_TXLINEARITY_UPCONV, 0);

	//Set Tx state machine timing (dly_GainRamp, dly_PA, dly_TX, dly_PHY)
	radio_controller_setTxDelays(RC_BASEADDR, 100, 50, 2, 150);

	//Give the TX PHY control of RXEN and TXEN
	radio_controller_setCtrlSource(RC_BASEADDR, RC_RFA, (RC_REG0_TXEN_CTRLSRC | RC_REG0_RXEN_CTRLSRC), RC_CTRLSRC_HW);

	return;
}


inline void wlan_tx_start() {

	//Start the PHY Tx immediately; this bypasses the mac_hw MPDU Tx state machine
	// This should only be used for debug - normal transmissions should use mac_hw
	REG_SET_BITS(WLAN_TX_REG_START, WLAN_TX_REG_START_VIA_RC);
	REG_CLEAR_BITS(WLAN_TX_REG_START, WLAN_TX_REG_START_VIA_RC);

	//TODO: I removed this usleep, and nothing seemed to break. Is it critical?
	//usleep(3);

	return;
}

inline u16 wlan_ofdm_txtime(u16 length, u16 n_DBPS){
	//u16: length of PSDU (does not include FCS length)
	//u16: n_DBPS: data bits per OFDM symbol

	//18.4.3 of IEEE 802.11-2012
	//Returns duration of transmission in microseconds

	u16 txTime;
	u16 n_sym, n_b;

	//Calculate num bits:
	// 16: SERVICE field
	// 8*length: actual MAC payload
	// 6: TAIL bits (zeros, required in all pkts to terminate FEC)
	n_b = (16+(8*length)+6);

	//Calculate num OFDM syms
	// This integer divide is effectively floor(n_b / n_DBPS)
	n_sym = n_b / n_DBPS;

	//If actual n_sym was non-integer, round up
	// This is effectively ceil(n_b / n_DBPS)
	if(n_sym*n_DBPS < n_b) n_sym++;

	txTime = TXTIME_T_PREAMBLE + TXTIME_T_SIGNAL + TXTIME_T_SYM * n_sym;

	return txTime;
}

inline void wlan_tx_buffer_sel(u8 n) {
	//The register-selected Tx pkt buffer is only used for transmissions that
	// are initiated via wlan_tx_start(); normal MAC transmissions will use
	// the mac_hw Tx functions, which override this pkt buf selection

	Xil_Out32(WLAN_TX_REG_PKT_BUF_SEL, n);
	return;
}

inline int wlan_tx_isrunning() {
	return (Xil_In32(WLAN_TX_REG_STATUS) & WLAN_TX_REG_STATUS_TX_RUNNING);
}

//Local implementation of usleep (MB has no built-in timer, so stdlib doesn't have usleep)
void usleep(u32 duration){
	XTmrCtr *TmrCtrInstancePtr = &TimerCounter;
	XTmrCtr_SetResetValue(TmrCtrInstancePtr,0,duration*(TIMER_FREQ/1000000));

	XTmrCtr_Start(TmrCtrInstancePtr,0);

	volatile u8 isExpired = 0;
	while(isExpired!=1){
		isExpired = XTmrCtr_IsExpired(TmrCtrInstancePtr,0);
	}
	XTmrCtr_Reset(TmrCtrInstancePtr,0);
	return;
}

void process_config_phy_rx(ipc_config_phy_rx* config_phy_rx){
	if(config_phy_rx->enable_dsss != 0xFF){
		if(config_phy_rx->enable_dsss == 1){
			//xil_printf("Enabling DSSS\n");
			wlan_phy_DSSS_rx_enable();
		} else {
			//xil_printf("Disabling DSSS\n");
			wlan_phy_DSSS_rx_disable();
		}
	}

}


void process_config_phy_tx(ipc_config_phy_tx* config_phy_tx){

}
