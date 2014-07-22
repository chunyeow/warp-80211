/** @file wlan_mac_fmc_pkt.h
 *  @brief FMC Packet Exchange
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

#ifndef WLAN_MAC_FMC_PKT_H_
#define WLAN_MAC_FMC_PKT_H_

#include "xmbox.h"

//FMC PKT MBOX Enable
//#define FMC_PKT_EN

#define FMC_MBOX_DEVICE_ID			XPAR_MBOX_0_DEVICE_ID
#define MBOX_ALIGN_OFFSET 2
#define FMC_IPC_DELIMITER	0x2452A9C0
#define FMC_IPC_MSG_ID_PKT_TO_W3   0x6BF8
#define FMC_IPC_MSG_ID_PKT_FROM_W3 0xEA6D

typedef struct {
	u32 delimiter;
	u16 msg_id;
	u16	size_bytes;
} wlan_fmc_ipc_msg;

int wlan_fmc_pkt_init ();
int fmc_ipc_rx();
int wlan_fmc_pkt_eth_send(u8* eth_hdr, u16 length);
void wlan_XMbox_WriteBlocking(XMbox *InstancePtr, u32 *BufferPtr, u32 RequestedBytes);
int wlan_XMbox_Read(XMbox *InstancePtr, u32 *BufferPtr, u32 RequestedBytes, u32 *BytesRecvdPtr);
int wlan_fmc_pkt_mailbox_setup_interrupt(XIntc* intc);
void FMCMailboxIntrHandler(void *CallbackRef);

#endif /* WLAN_MAC_FMC_PKT_H_ */
