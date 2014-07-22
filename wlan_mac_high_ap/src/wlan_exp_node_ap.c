/** @file wlan_exp_node_ap.c
 *  @brief Access Point WARPNet Experiment
 *
 *  This contains code for the 802.11 Access Point's WARPNet experiment interface.
 *
 *  @copyright Copyright 2014, Mango Communications. All rights reserved.
 *          Distributed under the Mango Communications Reference Design License
 *				See LICENSE.txt included in the design archive or
 *				at http://mangocomm.com/802.11/license
 *
 *  @author Chris Hunter (chunter [at] mangocomm.com)
 *  @author Patrick Murphy (murphpo [at] mangocomm.com)
 *  @author Erik Welsh (welsh [at] mangocomm.com)
 *  @bug No known bugs
 */

/***************************** Include Files *********************************/

#include "wlan_exp_common.h"
#include "wlan_exp.h"
#include "wlan_exp_node.h"
#include "wlan_exp_node_ap.h"


#ifdef USE_WARPNET_WLAN_EXP

// Xilinx includes
#include <xparameters.h>
#include <xil_io.h>
#include <Xio.h>
#include "xintc.h"


// Library includes
#include "string.h"
#include "stdlib.h"

//WARP includes
#include "wlan_mac_ipc_util.h"
#include "wlan_mac_misc_util.h"
#include "wlan_mac_802_11_defs.h"
#include "wlan_mac_queue.h"
#include "wlan_mac_ltg.h"
#include "wlan_mac_packet_types.h"
#include "wlan_mac_eth_util.h"
#include "wlan_mac_dl_list.h"
#include "wlan_mac_ap.h"



/*************************** Constant Definitions ****************************/


/*********************** Global Variable Definitions *************************/

extern dl_list		association_table;

extern char       * access_point_ssid;

extern u32          mac_param_chan;

/*************************** Variable Definitions ****************************/


/*************************** Functions Prototypes ****************************/




/******************************** Functions **********************************/


/*****************************************************************************/
/**
* Node Commands
*
* This function is part of the callback system for the Ethernet transport
* and will be executed when a valid node commands is recevied.
*
* @param    Command Header         - WARPNet Command Header
*           Command Arguments      - WARPNet Command Arguments
*           Response Header        - WARPNet Response Header
*           Response Arguments     - WARPNet Response Arguments
*           Packet Source          - Ethernet Packet Source
*           Ethernet Device Number - Indicates which Ethernet device packet came from
*
* @return	None.
*
* @note		See on-line documentation for more information about the ethernet
*           packet structure for WARPNet:  www.warpproject.org
*
******************************************************************************/
int wlan_exp_node_ap_processCmd( unsigned int cmdID, const wn_cmdHdr* cmdHdr, const void* cmdArgs, wn_respHdr* respHdr, void* respArgs, void* pktSrc, unsigned int eth_dev_num){
	//IMPORTANT ENDIAN NOTES:
	// -cmdHdr is safe to access directly (pre-swapped if needed)
	// -cmdArgs is *not* pre-swapped, since the framework doesn't know what it is
	// -respHdr will be swapped by the framework; user code should fill it normally
	// -respArgs will *not* be swapped by the framework, since only user code knows what it is
	//    Any data added to respArgs by the code below must be endian-safe (swapped on AXI hardware)

	const u32   * cmdArgs32  = cmdArgs;
	u32         * respArgs32 = respArgs;

	unsigned int  respIndex  = 0;                  // This function is called w/ same state as node_processCmd
	unsigned int  respSent   = NO_RESP_SENT;       // Initialize return value to NO_RESP_SENT
    // unsigned int  max_words  = 300;                // Max number of u32 words that can be sent in the packet (~1200 bytes)
                                                   //   If we need more, then we will need to rework this to send multiple response packets
    int           status;

    u32           temp;
    u32           msg_cmd;

    // Note:    
    //   Response header cmd, length, and numArgs fields have already been initialized.
    
    
#ifdef _DEBUG_
	xil_printf("In wlan_exp_node_ap_processCmd():  ID = %d \n", cmdID);
#endif

	switch(cmdID){

		//---------------------------------------------------------------------
		// TODO:  THIS FUNCTION IS NOT COMPLETE
		case CMDID_NODE_GET_STATION_INFO:
			xil_printf("AP - Set association table not supported\n");
		break;


		//---------------------------------------------------------------------
		case CMDID_NODE_DISASSOCIATE:
            // NODE_DISASSOCIATE Packet Format:
            //   - Note:  All u32 parameters in cmdArgs32 are byte swapped so use Xil_Ntohl()
            //
            //   - cmdArgs32[0] - AID
			//                  - 0xFFFF - Disassociate all
			//
			//   - Returns      0      - AID not found
            //                  AID    - AID that was disassociated
			//                  0xFFFF - All AIDs)
            //
			temp = Xil_Ntohl(cmdArgs32[0]);

			dl_entry * entry;

			// Check argument to see if we are diassociating one or all AIDs
			if ( temp == 0xFFFF ) {
				deauthenticate_stations();                     // Deauthenticate all stations
			} else {
				entry = wlan_mac_high_find_station_info_AID( &association_table, temp );
                temp = deauthenticate_station( (station_info*)(entry->data) );
			}

			// Print message to the UART to show which node was disassociated
			xil_printf("Node Disassociate - AP  - 0x%x\n", temp);

			// Send response of current channel
            respArgs32[respIndex++] = Xil_Htonl( temp );

			respHdr->length += (respIndex * sizeof(respArgs32));
			respHdr->numArgs = respIndex;
		break;

        
		//---------------------------------------------------------------------
		case CMDID_NODE_CHANNEL:
			//   - cmdArgs32[0]      - Command
			//   - cmdArgs32[1]      - Channel

			msg_cmd = Xil_Ntohl(cmdArgs32[0]);
			temp    = Xil_Ntohl(cmdArgs32[1]);
			status  = CMD_PARAM_SUCCESS;

			if ( msg_cmd == CMD_PARAM_WRITE_VAL ) {
				// Set the Channel
				//   NOTE:  We modulate temp so that we always have a valid channel
				temp = temp % 12;          // Get a channel number between 0 - 11
				if ( temp == 0 ) temp++;   // Change all values of 0 to 1

				// NOTE:  This function must be implemented in all child classes
				// deauthenticate_stations(); // First deauthenticate all stations //TODO: not sure this should be here for WARPnet

				mac_param_chan = temp;
				wlan_mac_high_set_channel( mac_param_chan );

			    xil_printf("Setting Channel = %d\n", mac_param_chan);
			}

			// Send response
            respArgs32[respIndex++] = Xil_Htonl( status );
            respArgs32[respIndex++] = Xil_Htonl( mac_param_chan );

			respHdr->length += (respIndex * sizeof(respArgs32));
			respHdr->numArgs = respIndex;
		break;


		//---------------------------------------------------------------------
        case NODE_AP_ALLOW_ASSOCIATIONS:
            // NODE_AP_ALLOW_ASSOCIATIONS Packet Format:
            //   - Note:  All u32 parameters in cmdArgs32 are byte swapped so use Xil_Ntohl()
            //
            //   - cmdArgs32[0] - 0xFFFF - Permanently turn on associations
			//                  - Others - Temporarily turn on associations
			//
			//   - Returns the status of the associations
            //
			xil_printf("Associations     Allowed - AP - ");

			temp = Xil_Ntohl(cmdArgs32[0]);

			if ( temp == 0xFFFF ) {
	    		enable_associations( ASSOCIATION_ALLOW_PERMANENT );
				xil_printf("Permanent \n");
			} else {
	    		enable_associations( ASSOCIATION_ALLOW_TEMPORARY );
				xil_printf("Temporary \n");
			}

            respArgs32[respIndex++] = Xil_Htonl( get_associations_status() );

			respHdr->length += (respIndex * sizeof(respArgs32));
			respHdr->numArgs = respIndex;
        break;


		//---------------------------------------------------------------------
		case NODE_AP_DISALLOW_ASSOCIATIONS:
            // NODE_AP_DISALLOW_ASSOCIATIONS Packet Format:
			//
			//   - Returns the status of the associations
            //
			disable_associations();

			temp = get_associations_status();

			if ( temp == ASSOCIATION_ALLOW_NONE ) {
				xil_printf("Associations NOT Allowed - AP\n");
			} else {
				xil_printf("Associations     Allowed - AP - Failed to Disable Associations\n");
			}

            respArgs32[respIndex++] = Xil_Htonl( temp );

			respHdr->length += (respIndex * sizeof(respArgs32));
			respHdr->numArgs = respIndex;
		break;


		//---------------------------------------------------------------------
		case NODE_AP_GET_SSID:
            // NODE_AP_GET_SSID Packet Format:
			//
            //   - respArgs32[0] - Number of characters the new SSID
            //   - respArgs32[1] - Packed array of ascii character values
			//                       NOTE: The characters are copied with a straight strcpy and must
			//                         be correctly processed on the host side
            //
			xil_printf("Get SSID - AP - %s\n", access_point_ssid);

			// Get the number of characters in the SSID
			temp = strlen(access_point_ssid);

			// Send response of current channel
            respArgs32[respIndex++] = Xil_Htonl( temp );

			strcpy( (char *)&respArgs32[respIndex], access_point_ssid );

			respIndex       += ( temp / sizeof(respArgs32) ) + 1;

			respHdr->length += (respIndex * sizeof(respArgs32));
			respHdr->numArgs = respIndex;
		break;


		//---------------------------------------------------------------------
		case NODE_AP_SET_SSID:
            // NODE_AP_SET_SSID Packet Format:
			//
            //   - cmdArgs32[0] - Number of characters in the new SSID
            //   - cmdArgs32[1] - Packed array of ascii character values
			//                      NOTE: The characters are assumed to be in the correct order
			//
			//   - Returns nothing
            //
			temp        = Xil_Ntohl(cmdArgs32[0]);
			char * ssid = (char *)&cmdArgs32[1];

			// Deauthenticate all stations since we are changing the SSID
			deauthenticate_stations();

			// Re-allocate memory for the new SSID and copy the characters of the new SSID
			access_point_ssid = wlan_mac_high_realloc(access_point_ssid, (temp + 1));
			strcpy(access_point_ssid, ssid);

			xil_printf("Set SSID - AP:  %s\n", access_point_ssid);
		break;


        //---------------------------------------------------------------------
		default:
			xil_printf("Unknown node command: 0x%x\n", cmdID);
		break;
	}

	return respSent;
}


#endif
