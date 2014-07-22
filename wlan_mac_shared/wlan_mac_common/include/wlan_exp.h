/** @file wlan_exp_common.h
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
 */

/***************************** Include Files *********************************/
// Include xil_types so function prototypes can use u8/u16/u32 data types
#include "xil_types.h"


/*************************** Constant Definitions ****************************/
#ifndef WLAN_EXP_H_
#define WLAN_EXP_H_


// **********************************************************************
// Use WARPNet Interface
//
// Note: Please leave this undefined. WARPnet functionality will be added
// to a future release.
//

#define USE_WARPNET_WLAN_EXP


// **********************************************************************
// WARPNet Controls
//

// Wait for WARPNet Ethernet interface to be ready before continuing boot
//   - Disabled by default
#define WLAN_EXP_WAIT_FOR_ETH          0



// **********************************************************************
// WARPNet Version Information
//

// Version info (MAJOR.MINOR.REV, all must be ints)
//     MAJOR and MINOR are both u8, while REV is u16
#define WARPNET_VER_MAJOR         2
#define WARPNET_VER_MINOR         0
#define WARPNET_VER_REV           0

#define REQ_WARPNET_HW_VER        (WARPNET_VER_MAJOR<<24)|(WARPNET_VER_MINOR<<16)|(WARPNET_VER_REV)


// Version info (MAJOR.MINOR.REV, all must be ints)
//     MAJOR and MINOR are both u8, while REV is u16
#define WLAN_EXP_VER_MAJOR        0
#define WLAN_EXP_VER_MINOR        9
#define WLAN_EXP_VER_REV          3

#define REQ_WLAN_EXP_HW_VER       (WLAN_EXP_VER_MAJOR<<24)|(WLAN_EXP_VER_MINOR<<16)|(WLAN_EXP_VER_REV)


// Define the WARPNet Type to communicate the type of wn_node.  Current values are:
//   Type                              Values
//   802.11                            0x00010000 - 0x0001FFFF
//       - Of the lower four digits, the lowest two describe CPU Low, while the upper two
//         describe CPU High.  For example:  in CPU High AP = 0x01; STA = 0x02, while in
//         CPU Low DCF = 0x01.  This creates the following supported combinations.
//
//     802.11 AP DCF                   0x00010101
//     802.11 Station DCF              0x00010201
//
#define WARPNET_TYPE_80211_BASE        0x00010000
#define WARPNET_TYPE_80211_HIGH_AP     0x00000100
#define WARPNET_TYPE_80211_HIGH_STA    0x00000200

#define WARPNET_TYPE_80211_LOW_DCF     0x00000001


// **********************************************************************
// WARP Hardware Version Information
//
#ifdef WARP_HW_VER_v2
#define WARP_HW_VERSION                2
#endif

#ifdef WARP_HW_VER_v3
#define WARP_HW_VERSION                3
#endif



// **********************************************************************
// WARPNet Tag Parameter group defines
//
#define WARPNET_GRP                    0xFF
#define NODE_GRP                       0x00
#define TRANS_GRP                      0x10


// Global WARPNet commands
//
#define CMDID_WARPNET_TYPE             0xFFFFFF


/*********************** Global Structure Definitions ************************/


/*************************** Function Prototypes *****************************/


#endif /* WLAN_EXP_H_ */
