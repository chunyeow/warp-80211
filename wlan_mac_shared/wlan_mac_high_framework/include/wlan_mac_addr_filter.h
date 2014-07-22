/** @file wlan_mac_addr_mac_filter.h
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

u8 wlan_mac_addr_filter_is_allowed(u8* addr);
u8 wlan_mac_addr_is_warp(u8* addr);
