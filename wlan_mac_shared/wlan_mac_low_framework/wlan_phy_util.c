/** @file wlan_phy_util.c
 *  @brief Physical Layer Utility
 *
 *  This contains code for configuring low-level parameters in the PHY and hardware.
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
 *  - PHY boots into a state where it won't receive until after the first transmission
 */

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

#ifdef WLAN_4RF_EN
	//Turn on clocks to FMC
	clk_config_outputs(CLK_BASEADDR, CLK_OUTPUT_ON, (CLK_SAMP_OUTSEL_FMC | CLK_RFREF_OUTSEL_FMC));

	//FMC samp clock divider = 2 (40MHz sampling reference, same as on-board AD9963 ref clk)
	clk_config_dividers(CLK_BASEADDR, 2, CLK_SAMP_OUTSEL_FMC);

	//FMC RF ref clock divider = 2 (40MHz RF reference, same as on-board MAX2829 ref clk)
	clk_config_dividers(CLK_BASEADDR, 2, CLK_RFREF_OUTSEL_FMC);
#endif

	//Initialize the AD9963 ADCs/DACs for on-board RF interfaces
	ad_init(AD_BASEADDR, AD_ALL_RF, 3);
	xil_printf("AD Readback: 0x%08x\n", ad_spi_read(AD_BASEADDR, RFA_AD_CS, 0x32));

	if(status != XST_SUCCESS) {
		xil_printf("w3_node_init: Error in ad_init (%d)\n", status);
		ret = XST_FAILURE;
	}

	//Initialize the radio_controller core and MAX2829 transceivers for on-board RF interfaces
	status = radio_controller_init(RC_BASEADDR, RC_ALL_RF, 1, 1);

	if(status != XST_SUCCESS) {
		xil_printf("w3_node_init: Error in radioController_initialize (%d)\n", status);
		//Comment out allow boot even if an RF interfce doesn't lock (hack for debugging - not for reference release)
		ret = XST_FAILURE;
	}

	//Initialize the EEPROM read/write core
	iic_eeprom_init(EEPROM_BASEADDR, 0x64);

#ifdef WLAN_4RF_EN
	iic_eeprom_init(FMC_EEPROM_BASEADDR, 0x64);
#endif

	//Initialize the timer counter
	status = XTmrCtr_Initialize(TmrCtrInstancePtr, TMRCTR_DEVICE_ID);
	if (status != XST_SUCCESS) {
		xil_printf("w3_node_init: Error in XtmrCtr_Initialize (%d)\n", status);
		ret = XST_FAILURE;
	}

	// Set timer 0 to into a "count down" mode
	XTmrCtr_SetOptions(TmrCtrInstancePtr, 0, (XTC_DOWN_COUNT_OPTION));

	//Give the PHY control of the red user LEDs (PHY counts 1-hot on SIGNAL errors)
	//Note: Uncommenting this line will make the RED LEDs controlled by hardware.
	//This will move the LEDs on PHY bad signal events
	//userio_set_ctrlSrc_hw(USERIO_BASEADDR, W3_USERIO_CTRLSRC_LEDS_RED);

	return ret;
}

inline void wlan_phy_set_tx_signal(u8 pkt_buf, u8 rate, u16 length) {
	Xil_Out32((u32*)(TX_PKT_BUF_TO_ADDR(pkt_buf) + PHY_TX_PKT_BUF_PHY_HDR_OFFSET), WLAN_TX_SIGNAL_CALC(rate, length));
	return;
}


void wlan_rx_config_ant_mode(u32 ant_mode) {

	//Disable all Rx modes first; selectively re-enabled in switch below
	REG_CLEAR_BITS(WLAN_RX_REG_CFG, (
			WLAN_RX_REG_CFG_PKT_DET_EN_ANT_A |
			WLAN_RX_REG_CFG_PKT_DET_EN_ANT_B |
			WLAN_RX_REG_CFG_PKT_DET_EN_ANT_C |
			WLAN_RX_REG_CFG_PKT_DET_EN_ANT_D |
			WLAN_RX_REG_CFG_SWITCHING_DIV_EN |
			WLAN_RX_REG_CFG_ANT_SEL_MASK));

	radio_controller_setCtrlSource(RC_BASEADDR, RC_ALL_RF, RC_REG0_RXEN_CTRLSRC, RC_CTRLSRC_REG);

	switch(ant_mode) {
		case RX_ANTMODE_SISO_ANTA:
			REG_SET_BITS(WLAN_RX_REG_CFG, WLAN_RX_REG_CFG_PKT_DET_EN_ANT_A);
			wlan_phy_select_rx_antenna(0);
			radio_controller_setCtrlSource(RC_BASEADDR, RC_RFA, RC_REG0_RXEN_CTRLSRC, RC_CTRLSRC_HW);
			wlan_agc_config(0);
			break;

		case RX_ANTMODE_SISO_ANTB:
			REG_SET_BITS(WLAN_RX_REG_CFG, WLAN_RX_REG_CFG_PKT_DET_EN_ANT_B);
			wlan_phy_select_rx_antenna(1);
			radio_controller_setCtrlSource(RC_BASEADDR, RC_RFB, RC_REG0_RXEN_CTRLSRC, RC_CTRLSRC_HW);
			wlan_agc_config(1);
			break;

		case RX_ANTMODE_SISO_ANTC:
			REG_SET_BITS(WLAN_RX_REG_CFG, WLAN_RX_REG_CFG_PKT_DET_EN_ANT_C);
			wlan_phy_select_rx_antenna(2);
			radio_controller_setCtrlSource(RC_BASEADDR, RC_RFC, RC_REG0_RXEN_CTRLSRC, RC_CTRLSRC_HW);

			wlan_agc_config(2);

			break;

		case RX_ANTMODE_SISO_ANTD:
			REG_SET_BITS(WLAN_RX_REG_CFG, WLAN_RX_REG_CFG_PKT_DET_EN_ANT_D);
			wlan_phy_select_rx_antenna(3);
			radio_controller_setCtrlSource(RC_BASEADDR, RC_RFD, RC_REG0_RXEN_CTRLSRC, RC_CTRLSRC_HW);
			wlan_agc_config(3);
			break;

		case RX_ANTMODE_SISO_SELDIV_2ANT:
			REG_SET_BITS(WLAN_RX_REG_CFG, (WLAN_RX_REG_CFG_PKT_DET_EN_ANT_A | WLAN_RX_REG_CFG_PKT_DET_EN_ANT_B | WLAN_RX_REG_CFG_SWITCHING_DIV_EN));
			radio_controller_setCtrlSource(RC_BASEADDR, (RC_RFA | RC_RFB), RC_REG0_RXEN_CTRLSRC, RC_CTRLSRC_HW);
			break;

		case RX_ANTMODE_SISO_SELDIV_4ANT:
			REG_SET_BITS(WLAN_RX_REG_CFG, (WLAN_RX_REG_CFG_PKT_DET_EN_ANT_A | WLAN_RX_REG_CFG_PKT_DET_EN_ANT_B | WLAN_RX_REG_CFG_PKT_DET_EN_ANT_C | WLAN_RX_REG_CFG_PKT_DET_EN_ANT_D | WLAN_RX_REG_CFG_SWITCHING_DIV_EN));
			radio_controller_setCtrlSource(RC_BASEADDR, RC_ALL_RF, RC_REG0_RXEN_CTRLSRC, RC_CTRLSRC_HW);
			break;

		default:
			//Default to SISO on A if user provides invalid mode
			xil_printf("wlan_rx_config_ant_mode ERROR: Invalid Mode - Defaulting to SISO on A\n");
			REG_SET_BITS(WLAN_RX_REG_CFG, WLAN_RX_REG_CFG_PKT_DET_EN_ANT_A);
			wlan_phy_select_rx_antenna(0);
			radio_controller_setCtrlSource(RC_BASEADDR, RC_RFA, RC_REG0_RXEN_CTRLSRC, RC_CTRLSRC_HW);
			break;
	}

	return;
}

void wlan_tx_config_ant_mode(u32 ant_mode) {
	return;
	/*OLD - DELETE WHEN v40 HW WORKS!*/
	REG_CLEAR_BITS(WLAN_TX_REG_CFG, (WLAN_TX_REG_CFG_ANT_A_TXEN | WLAN_TX_REG_CFG_ANT_B_TXEN | WLAN_TX_REG_CFG_ANT_C_TXEN | WLAN_TX_REG_CFG_ANT_D_TXEN));
	radio_controller_setCtrlSource(RC_BASEADDR, RC_ALL_RF, RC_REG0_TXEN_CTRLSRC, RC_CTRLSRC_REG);

	switch(ant_mode) {
		case TX_ANTMODE_SISO_ANTA:
			REG_SET_BITS(WLAN_TX_REG_CFG, WLAN_TX_REG_CFG_ANT_A_TXEN);
			radio_controller_setCtrlSource(RC_BASEADDR, RC_RFA, RC_REG0_TXEN_CTRLSRC, RC_CTRLSRC_HW);
			break;

		case TX_ANTMODE_SISO_ANTB:
			REG_SET_BITS(WLAN_TX_REG_CFG, WLAN_TX_REG_CFG_ANT_B_TXEN);
			radio_controller_setCtrlSource(RC_BASEADDR, RC_RFB, RC_REG0_TXEN_CTRLSRC, RC_CTRLSRC_HW);
			break;

		case TX_ANTMODE_SISO_ANTC:
			REG_SET_BITS(WLAN_TX_REG_CFG, WLAN_TX_REG_CFG_ANT_C_TXEN);
			radio_controller_setCtrlSource(RC_BASEADDR, RC_RFC, RC_REG0_TXEN_CTRLSRC, RC_CTRLSRC_HW);
			break;

		case TX_ANTMODE_SISO_ANTD:
			REG_SET_BITS(WLAN_TX_REG_CFG, WLAN_TX_REG_CFG_ANT_D_TXEN);
			radio_controller_setCtrlSource(RC_BASEADDR, RC_RFD, RC_REG0_TXEN_CTRLSRC, RC_CTRLSRC_HW);
			break;

		default:
			//Default to SISO on A if user provides invalid mode
			xil_printf("wlan_tx_config_ant_mode ERROR: Invalid Mode - Defaulting to SISO on A\n");
			REG_SET_BITS(WLAN_TX_REG_CFG, WLAN_TX_REG_CFG_ANT_A_TXEN);
			radio_controller_setCtrlSource(RC_BASEADDR, RC_RFA, RC_REG0_TXEN_CTRLSRC, RC_CTRLSRC_HW);
			break;
	}
	return;
}



void wlan_phy_init() {

	//Assert Tx and Rx resets
	REG_SET_BITS(WLAN_RX_REG_CTRL, WLAN_RX_REG_CTRL_RESET);
	REG_SET_BITS(WLAN_TX_REG_CFG, WLAN_TX_REG_CFG_RESET);

/************ PHY Rx ************/

	//Enable DSSS Rx by default
	wlan_phy_DSSS_rx_enable();
	//wlan_phy_DSSS_rx_disable();

	//Sane defaults for DSSS Rx (code_corr, timeout, despread_dly, length_pad)
	wlan_phy_DSSS_rx_config(0x600, 200, 5, 5);

	//Allow the DSSS receiver to keep the AGC locked (otherwise AGC resets when OFDM LTS corr times out)
	REG_SET_BITS(WLAN_RX_REG_CFG, WLAN_RX_REG_CFG_DSSS_RX_AGC_HOLD);

	//Enable LTS-based CFO correction
	REG_CLEAR_BITS(WLAN_RX_REG_CFG, WLAN_RX_REG_CFG_CFO_EST_BYPASS);

	//Enable byte order swap for payloads and chan ests
	REG_SET_BITS(WLAN_RX_REG_CFG, WLAN_RX_REG_CFG_PKT_BUF_WEN_SWAP);
	REG_CLEAR_BITS(WLAN_RX_REG_CFG, WLAN_RX_REG_CFG_CHAN_EST_WEN_SWAP);

	//Enable writing OFDM chan ests to Rx pkt buffer
	REG_SET_BITS(WLAN_RX_REG_CFG, WLAN_RX_REG_CFG_RECORD_CHAN_EST);

	//Block Rx inputs during Tx
	REG_SET_BITS(WLAN_RX_REG_CFG, WLAN_RX_REG_CFG_USE_TX_SIG_BLOCK);

	//FFT config
	wlan_phy_rx_set_fft_window_offset(3);
	wlan_phy_rx_set_fft_scaling(5);

	//Set LTS correlation threshold and timeout
	// 1023 disables LTS threshold switch (one threshold worked across SNRs in our testing)
	// Timeout value is doubled in hardware (350/2 becomes a timeout of 350 sample periods)
	wlan_phy_rx_lts_corr_config(1023 * PHY_RX_RSSI_SUM_LEN, 350/2);

	//LTS correlation thresholds (low NSR, high SNR)
	wlan_phy_rx_lts_corr_thresholds(12500, 12500); //FIXME

	//Configure RSSI pkt det
 	// RSSI pkt det disabled by default (auto-corr detection worked across SNRs in our testing)
 	wlan_phy_rx_pktDet_RSSI_cfg(PHY_RX_RSSI_SUM_LEN, (PHY_RX_RSSI_SUM_LEN * 1023), 4);

	//Configure auto-corr pkt det (corr, energy, min-duration, post-reset-wait)
	wlan_phy_rx_pktDet_autoCorr_cfg(200, 50, 4, 0x3F);

	//Configure the default antenna selections as SISO Tx/Rx on RF A
	wlan_rx_config_ant_mode(RX_ANTMODE_SISO_ANTA);

	//Set physical carrier sensing threshold
	wlan_phy_rx_set_cca_thresh(PHY_RX_RSSI_SUM_LEN * 480); //-62dBm from 802.11-2012
	//wlan_phy_rx_set_cca_thresh(PHY_RX_RSSI_SUM_LEN * 750);
	//wlan_phy_rx_set_cca_thresh(PHY_RX_RSSI_SUM_LEN * 1023);

	//Set post Rx extension (number of sample periods post-Rx the PHY waits before asserting Rx END - must be long enough for decoding latency at 64QAM 3/4)
	wlan_phy_rx_set_extension(PHY_RX_SIG_EXT_USEC*20); //num samp periods post done to extend CCA BUSY

	//Configure channel estimate capture (64 subcarriers, 4 bytes each)
	// Chan ests start at sizeof(rx_frame_info) - sizeof(chan_est)
	wlan_phy_rx_pkt_buf_h_est_offset((PHY_RX_PKT_BUF_PHY_HDR_OFFSET - (64*4)));
	

/************ PHY Tx ************/
	
	//De-assert all starts
	REG_CLEAR_BITS(WLAN_TX_REG_START, 0xFFFFFFFF);

	//Set Tx duration extension, in units of sample periods
	wlan_phy_tx_set_extension(PHY_TX_SIG_EXT_USEC*20);

	//Set extension from last samp output to RF Tx -> Rx transition
	// This delay allows the Tx pipeline to finish driving samples into DACs
	//  and for DAC->RF frontend to finish output Tx waveform
	wlan_phy_tx_set_txen_extension(50);

	//Set extension from RF Rx -> Tx to un-blocking Rx samples
	wlan_phy_tx_set_rx_invalid_extension(150); //100

	//Set digital scaling of preamble/payload signals before DACs (UFix12_0)
	wlan_phy_tx_set_scaling(0x2000, 0x2000);

	//Enable the Tx PHY 4-bit TxEn port that captures the MAC's selection of active antennas per Tx
	REG_SET_BITS(WLAN_TX_REG_CFG, WLAN_TX_REG_CFG_USE_MAC_ANT_MASKS);

/*********** AGC ***************/
	wlan_agc_config(0);

/************ Wrap Up ************/

	//Set MSB of RSSI_THRESH register to use summed RSSI for debug output
	Xil_Out32(XPAR_WLAN_PHY_RX_MEMMAP_RSSI_THRESH, ((1<<31) | (PHY_RX_RSSI_SUM_LEN * 150)));

	//De-assert resets
	REG_CLEAR_BITS(WLAN_RX_REG_CTRL, WLAN_RX_REG_CTRL_RESET);
	REG_CLEAR_BITS(WLAN_TX_REG_CFG, WLAN_TX_REG_CFG_RESET);

	//Let PHY Tx take control of radio TXEN/RXEN
	REG_CLEAR_BITS(WLAN_TX_REG_CFG, WLAN_TX_REG_CFG_SET_RC_RXEN);
	REG_SET_BITS(WLAN_TX_REG_CFG, WLAN_TX_REG_CFG_SET_RC_RXEN);

	return;
}

void wlan_agc_config(u8 ant_id) {
	//ant_id argument allows per-antenna AGC settings, in case FMC module has different
	// response than on-board RF interfaces. Testing so far indicates the settings below
	// work fine for all RF interfaces

	//Post Rx_done reset delays for [rxhp, g_rf, g_bb]
	wlan_agc_set_reset_timing(4, 250, 250);

	//AGC config:
	//RFG Thresh 3->2, 2->1, Avg_len_sel, V_DB_Adj, Init G_BB
	wlan_agc_set_config( (256-56), (256-37), 0, 6, 24);

	//AGC RSSI->Rx power offsets
	wlan_agc_set_RSSI_pwr_calib(100, 85, 70);

	//AGC timing: capt_rssi_1, capt_rssi_2, capt_v_db, agc_done
	wlan_agc_set_AGC_timing(1, 30, 90, 96);

	//AGC timing: start_dco, en_iir_filt
	wlan_agc_set_DCO_timing(100, (100+34));

	//AGC target output power (log scale)
	wlan_agc_set_target( (64-16) );

#if 0
		xil_printf("Switching to MGC for ant %d\n", ant_id);
		radio_controller_setCtrlSource(RC_BASEADDR, RC_ALL_RF, RC_REG0_RXHP_CTRLSRC, RC_CTRLSRC_REG);
		radio_controller_setRxHP(RC_BASEADDR, RC_ALL_RF, RC_RXHP_OFF);
		radio_controller_setRxGainSource(RC_BASEADDR, RC_ALL_RF, RC_GAINSRC_SPI);

		//Set Rx gains
		radio_controller_setRadioParam(RC_BASEADDR, RC_ALL_RF, RC_PARAMID_RXGAIN_RF, 3);
		radio_controller_setRadioParam(RC_BASEADDR, RC_ALL_RF, RC_PARAMID_RXGAIN_BB, 8);
#endif
	return;
}

void wlan_radio_init() {

	//Setup clocking and filtering (20MSps, 2x interp/decimate in AD9963)
	clk_config_dividers(CLK_BASEADDR, 2, (CLK_SAMP_OUTSEL_AD_RFA | CLK_SAMP_OUTSEL_AD_RFB));

	ad_config_filters(AD_BASEADDR, AD_ALL_RF, 2, 2);


	//Setup RFA
	radio_controller_TxRxDisable(RC_BASEADDR, RC_ALL_RF);

	radio_controller_apply_TxDCO_calibration(AD_BASEADDR, EEPROM_BASEADDR, (RC_RFA | RC_RFB));
#ifdef WLAN_4RF_EN
	radio_controller_apply_TxDCO_calibration(AD_BASEADDR, FMC_EEPROM_BASEADDR, (RC_RFC | RC_RFD));
#endif

	radio_controller_setCenterFrequency(RC_BASEADDR, RC_ALL_RF, RC_24GHZ, 4);


	radio_controller_setRadioParam(RC_BASEADDR, RC_ALL_RF, RC_PARAMID_RSSI_HIGH_BW_EN, 0);

	//Filter bandwidths
	radio_controller_setRadioParam(RC_BASEADDR, RC_ALL_RF, RC_PARAMID_RXHPF_HIGH_CUTOFF_EN, 1);
	radio_controller_setRadioParam(RC_BASEADDR, RC_ALL_RF, RC_PARAMID_RXLPF_BW, 1);
	radio_controller_setRadioParam(RC_BASEADDR, RC_ALL_RF, RC_PARAMID_TXLPF_BW, 1);

#if 0
	//MGC
	radio_controller_setCtrlSource(RC_BASEADDR, RC_ALL_RF, RC_REG0_RXHP_CTRLSRC, RC_CTRLSRC_REG);
	radio_controller_setRxHP(RC_BASEADDR, RC_ALL_RF, RC_RXHP_OFF);
	radio_controller_setRxGainSource(RC_BASEADDR, RC_ALL_RF, RC_GAINSRC_SPI);

	//Set Rx gains
	radio_controller_setRadioParam(RC_BASEADDR, RC_ALL_RF, RC_PARAMID_RXGAIN_RF, 1);
	radio_controller_setRadioParam(RC_BASEADDR, RC_ALL_RF, RC_PARAMID_RXGAIN_BB, 8);

#else
	//AGC
	radio_controller_setCtrlSource(RC_BASEADDR, RC_ALL_RF, RC_REG0_RXHP_CTRLSRC, RC_CTRLSRC_HW);
	radio_controller_setRxGainSource(RC_BASEADDR, RC_ALL_RF, RC_GAINSRC_HW);
#endif

	//Set Tx gains
	//radio_controller_setTxGainSource(RC_BASEADDR, RC_ALL_RF, RC_GAINSRC_REG); //Used for software control of gains
	//radio_controller_setTxGainTarget(RC_BASEADDR, RC_ALL_RF, 45);
	radio_controller_setTxGainSource(RC_BASEADDR, RC_ALL_RF, RC_GAINSRC_HW); //Used for hardware control of gains

	radio_controller_setRadioParam(RC_BASEADDR, RC_ALL_RF, RC_PARAMID_TXGAIN_BB, 2);
	

	//Set misc radio params
	radio_controller_setRadioParam(RC_BASEADDR, RC_ALL_RF, RC_PARAMID_TXLINEARITY_PADRIVER, 0);
	radio_controller_setRadioParam(RC_BASEADDR, RC_ALL_RF, RC_PARAMID_TXLINEARITY_VGA, 0);
	radio_controller_setRadioParam(RC_BASEADDR, RC_ALL_RF, RC_PARAMID_TXLINEARITY_UPCONV, 0);

	//Set Tx state machine timing             (dly_GainRamp, dly_PA, dly_TX, dly_PHY)
	radio_controller_setTxDelays(RC_BASEADDR, 40, 20, 0, TX_RC_PHYSTART_DLY); //240 PA time after 180 PHY time is critical point

	//Configure the radio_controller Tx/Rx enable control sources
	// The Tx PHY drives a 4-bit TxEn, one bit per RF interface
	// The Tx PHY drives a 1-bit RxEn, common to all RF interfaces
	//  MAC software should select active Rx interface by changing RFA/RFB RxEn ctrl src between _HW and _REG
	radio_controller_setCtrlSource(RC_BASEADDR, RC_RFA, (RC_REG0_RXEN_CTRLSRC), RC_CTRLSRC_HW);
	radio_controller_setCtrlSource(RC_BASEADDR, RC_RFB, (RC_REG0_RXEN_CTRLSRC), RC_CTRLSRC_REG);

	radio_controller_setCtrlSource(RC_BASEADDR, (RC_RFA | RC_RFB), (RC_REG0_TXEN_CTRLSRC), RC_CTRLSRC_HW);

	//Disable any hardware control of RFC/RFD
	radio_controller_setCtrlSource(RC_BASEADDR, (RC_RFC | RC_RFD), (RC_REG0_RXEN_CTRLSRC | RC_REG0_TXEN_CTRLSRC), RC_CTRLSRC_REG);

/*
   OLD - DELTE WHEN v40 HW WORKS
 	radio_controller_setCtrlSource(RC_BASEADDR, RC_RFA, (RC_REG0_TXEN_CTRLSRC | RC_REG0_RXEN_CTRLSRC), RC_CTRLSRC_HW);
	radio_controller_setCtrlSource(RC_BASEADDR, RC_RFB, (RC_REG0_TXEN_CTRLSRC | RC_REG0_RXEN_CTRLSRC), RC_CTRLSRC_REG);
	radio_controller_setCtrlSource(RC_BASEADDR, RC_RFC, (RC_REG0_TXEN_CTRLSRC | RC_REG0_RXEN_CTRLSRC), RC_CTRLSRC_REG);
	radio_controller_setCtrlSource(RC_BASEADDR, RC_RFD, (RC_REG0_TXEN_CTRLSRC | RC_REG0_RXEN_CTRLSRC), RC_CTRLSRC_REG);
*/
	return;
}


inline void wlan_tx_start() {

	//Start the PHY Tx immediately; this bypasses the mac_hw MPDU Tx state machine
	// This should only be used for debug - normal transmissions should use mac_hw
	REG_SET_BITS(WLAN_TX_REG_START, WLAN_TX_REG_START_VIA_RC);
	REG_CLEAR_BITS(WLAN_TX_REG_START, WLAN_TX_REG_START_VIA_RC);

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

	Xil_Out32(WLAN_TX_REG_PKT_BUF_SEL, ((Xil_In32(WLAN_TX_REG_PKT_BUF_SEL) & ~0xF) | (n&0xF)) );
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
