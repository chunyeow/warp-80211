////////////////////////////////////////////////////////////////////////////////
// File   :	wlan_exp_node.h
// Authors:	Chris Hunter (chunter [at] mangocomm.com)
//			Patrick Murphy (murphpo [at] mangocomm.com)
//          Erik Welsh (welsh [at] mangocomm.com)
// License:	Copyright 2013, Mango Communications. All rights reserved.
//			Distributed under the WARP license  (http://warpproject.org/license)
////////////////////////////////////////////////////////////////////////////////


/***************************** Include Files *********************************/
#include "wlan_exp_common.h"


// WLAN MAC includes for common functions
#include "wlan_mac_util.h"


/*************************** Constant Definitions ****************************/
#ifndef WLAN_EXP_NODE_H_
#define WLAN_EXP_NODE_H_



// ****************************************************************************
// Define WLAN Exp Node Commands
//
#define NODE_INFO               1
#define NODE_IDENTIFY           2
#define NODE_CONFIG_SETUP       3
#define NODE_CONFIG_RESET       4


// ****************************************************************************
// Define Node Parameters
//   - NOTE:  To add another parameter, add the define before "NODE_MAX_PARAMETER"
//     and then change the value of "NODE_MAX_PARAMETER" to be the largest value
//     in the list so it is easy to iterate over all parameters
//
#define NODE_TYPE               0
#define NODE_ID                 1
#define NODE_HW_GEN             2
#define NODE_DESIGN_VER         3
#define NODE_SERIAL_NUM         4
#define NODE_FPGA_DNA           5
#define NODE_MAX_PARAMETER      6


/*********************** Global Structure Definitions ************************/

// **********************************************************************
// WARPNet Node Info Structure
//
typedef struct {

    u32   type;            // Type of WARPNet node
    u32   node;            // Only first 16 bits are valid
    u32   hw_generation;   // Only first  8 bits are valid
	u32   design_ver;      // Only first 24 bits are valid

	u32   serial_number;
	u32   fpga_dna[FPGA_DNA_LEN];

    u32   eth_device;
    u8    hw_addr[ETH_ADDR_LEN];
    u8    ip_addr[IP_VERSION];
    u32   unicast_port;
    u32   broadcast_port;

} wn_node_info;



/*************************** Function Prototypes *****************************/

// WLAN Exp node commands
//
int  wlan_exp_node_init( unsigned int type, unsigned int serial_number, unsigned int *fpga_dna, unsigned int eth_dev_num, unsigned char *hw_addr );

void node_set_process_callback(void(*callback)());
int  node_get_parameters(u32 * buffer, unsigned int max_words, unsigned char network);



#endif /* WLAN_EXP_NODE_H_ */
