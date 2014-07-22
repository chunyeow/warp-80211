/** @file wlan_mac_addr_mac_filter.c
 *  @brief Address Filter
 *
 *  This contains code for the filtering MAC addresses
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

#include "stdio.h"
#include "xil_types.h"
#include "wlan_mac_addr_filter.h"

///For the whitelist_range_mask, bits that are 1 are treated as "any" and bits that are 0 are treated as "must equal"
///For the whitelist_range_compare, locations of zeroed bits in the mask must match whitelist_range_compare for incoming addresses

//Any Addresses
static u8 whitelist_range_mask[6]    = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static u8 whitelist_range_compare[6]   = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

//No Range of Addresses
//static u8 whitelist_range_mask[6]      = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//static u8 whitelist_range_compare[6]   = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

//WARP-only Addresses
//static u8 whitelist_range_mask[6]      = { 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF };
//static u8 whitelist_range_compare[6]	  = { 0x40, 0xD8, 0x55, 0x04, 0x20, 0x00 };


#define NUM_WHITELIST_NODES 2
/// whitelist_compare contains all addresses that are explicitly allowed on the network
static u8 whitelist_compare[NUM_WHITELIST_NODES][6] = { { 0x40, 0xD8, 0x55, 0x04, 0x21, 0x30  }, \
										                { 0x40, 0xD8, 0x55, 0x04, 0x21, 0x3A } };

static u8 warp_range_mask[6]      = { 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF };
static u8 warp_range_compare[6]   = { 0x40, 0xD8, 0x55, 0x04, 0x20, 0x00 };

u8 wlan_mac_addr_filter_is_allowed(u8* addr){
	u32 i,j;
	u32 sum;

	//First, check if the incoming address is within the allowable range of addresses
	sum = 0;
	for(i=0; i<6; i++){
		sum += (whitelist_range_mask[i] | whitelist_range_compare[i]) == (whitelist_range_mask[i] | addr[i]);
	}
	if(sum == 6) return 1;

	//Next, check if the incoming address is one of the individual whitelisted addresses
	for(j=0; j<NUM_WHITELIST_NODES; j++){
		sum = 0;
		for(i=0; i<6; i++){
			sum += (whitelist_compare[j][i]) == (addr[i]);
		}
		if(sum == 6) return 1;
	}

	//If the code made it this far, we aren't allowing this address to join the network.
	return 0;
}

u8 wlan_mac_addr_is_warp(u8* addr){
	u32 i;
	u32 sum;

	//First, check if the incoming address is within the allowable range of addresses
	sum = 0;
	for(i=0; i<6; i++){
		sum += (warp_range_mask[i] | warp_range_compare[i]) == (warp_range_mask[i] | addr[i]);
	}
	if(sum == 6){
		return 1;
	}
	return 0;
}

