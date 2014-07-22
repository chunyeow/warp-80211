/** @file wlan_exp_common.c
 *  @brief Experiment Framework (Common)
 *
 *  This contains the code for WARPnet Experimental Framework.
 *
 *  @copyright Copyright 2014, Mango Communications. All rights reserved.
 *          Distributed under the Mango Communications Reference Design License
 *				See LICENSE.txt included in the design archive or
 *				at http://mangocomm.com/802.11/license
 *
 *  @author Chris Hunter (chunter [at] mangocomm.com)
 *  @author Patrick Murphy (murphpo [at] mangocomm.com)
 *  @author Erik Welsh (welsh [at] mangocomm.com)
 *  @bug No known bugs.
 */

/***************************** Include Files *********************************/

#include "wlan_exp_common.h"




// Xilinx / Standard library includes

#include <xparameters.h>
#include <xil_io.h>
#include <xio.h>

#include "string.h"

// WLAN includes

#include "wlan_mac_high.h"

wlan_mac_hw_info* hw_info_ptr;

void wlan_exp_configure(u32 type, u32 eth_dev_num){
	hw_info_ptr = wlan_mac_high_get_hw_info();

	hw_info_ptr->type              = type;
	hw_info_ptr->wn_exp_eth_device = eth_dev_num;

}


#ifdef USE_WARPNET_WLAN_EXP

/*************************** Constant Definitions ****************************/

/*********************** Global Variable Definitions *************************/

/*************************** Variable Definitions ****************************/

/*************************** Functions Prototypes ****************************/

#ifdef _DEBUG_
void wlan_exp_print_station_status( station_info * stations, unsigned int num_stations );
#endif


/**************************** Common Functions *******************************/


/*****************************************************************************/
/**
* Get Station Status
*
* This function will populate the buffer with:
*   buffer[0]      = Number of stations
*   buffer[1 .. N] = memcpy of the station information structure
* where N is less than max_words
*
* @param    stations      - Station info pointer
*           num_stations  - Number of stations to copy
*           buffer        - u32 pointer to the buffer to transfer the data
*           max_words     - The maximum number of words in the buffer
*
* @return	Number of words copied in to the buffer
*
* @note     None.
*
******************************************************************************/
int get_station_status( station_info * stations, u32 num_stations, u32 * buffer, u32 max_words ) {

	unsigned int size;
	unsigned int index;

	index     = 0;

	// Set number of Association entries
	buffer[ index++ ] = num_stations;

	// Get total size (in bytes) of data to be transmitted
	size   = num_stations * sizeof( station_info );

	// Get total size of data (in words) to be transmitted
	index += size / sizeof( u32 );

    if ( (size > 0 ) && (index < max_words) ) {

        memcpy( &buffer[1], stations, size );
    }

#ifdef _DEBUG_
    wlan_exp_print_station_status( stations, num_stations );
#endif

	return index;
}



/*****************************************************************************/
/**
* Get MAC Address
*
* This function will populate the MAC address buffer, dest, with the MAC
* address coming over the network (byte swapped).  This uses the same formating
* as the HW address parameter from transport.c
*
* @param    src  - Source buffer of MAC address (u32, byte swapped)
*           dest - Destination buffer of MAC address
*
* @return	None.
*
* @note     None.
*
******************************************************************************/
void wlan_exp_get_mac_addr( u32 * src, u8 * dest) {

    dest[0] = (src[0] >> 16) & 0xFF;
    dest[1] = (src[0] >> 24) & 0xFF;
    dest[2] = (src[1] >>  0) & 0xFF;
    dest[3] = (src[1] >>  8) & 0xFF;
    dest[4] = (src[1] >> 16) & 0xFF;
    dest[5] = (src[1] >> 24) & 0xFF;

}






#ifdef _DEBUG_


/*****************************************************************************/
/**
* Print Station Status
*
* This function will print a list of station_info structures
*
* @param    stations     - pointer to the station_info list
*           num_stations - number of station_info structures in the list
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void wlan_exp_print_station_status( station_info * stations, unsigned int num_stations ){
	u32 i;


	// !!! FIX !!!
	for(i=0; i < num_stations; i++){
		xil_printf("---------------------------------------------------\n");
		xil_printf(" AID: %02x -- MAC Addr: %02x:%02x:%02x:%02x:%02x:%02x\n", stations[i].AID,
				stations[i].addr[0],stations[i].addr[1],stations[i].addr[2],stations[i].addr[3],stations[i].addr[4],stations[i].addr[5]);
		xil_printf("     - Last heard on %d ms with sequence number %d\n",((u32)(stations[i].rx_timestamp))/1000, stations[i].seq);
		xil_printf("     - Last Rx Power    : %d dBm\n",stations[i].last_rx_power);
		xil_printf("     - Tx Rate          : %d\n", stations[i].tx_rate);
		xil_printf("     - # Tx MPDUs       : %d (%d successful)\n", stations[i].num_tx_total, stations[i].num_tx_success);
		xil_printf("     - # Rx MPDUs       : %d (%d bytes)\n", stations[i].num_rx_success, stations[i].num_rx_bytes);
	}

}


#endif


// End USE_WARPNET_WLAN_EXP
#endif
