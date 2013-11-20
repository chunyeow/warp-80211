////////////////////////////////////////////////////////////////////////////////
// File   : wlan_lib.c
// Authors: Patrick Murphy (murphpo [at] mangocomm.com)
//			Chris Hunter (chunter [at] mangocomm.com)
// License: Copyright 2013, Mango Communications. All rights reserved.
//          Distributed under the Mango Communications Reference Design License
//				See LICENSE.txt included in the design archive or
//				at http://mangocomm.com/802.11/license
////////////////////////////////////////////////////////////////////////////////

#include "stdlib.h"
#include "stdio.h"

#include "xstatus.h"
#include "xmutex.h"
#include "xmbox.h"
#include "xparameters.h"

#ifdef XPAR_INTC_0_DEVICE_ID
#include "xintc.h"
#include "xil_exception.h"
#endif

#include "wlan_mac_ipc_util.h"
#include "wlan_mac_misc_util.h"
#include "wlan_mac_802_11_defs.h"


#ifdef XPAR_INTC_0_DEVICE_ID

#define MAILBOX_RIT	0	/* mailbox receive interrupt threshold */
#define MAILBOX_SIT	0	/* mailbox send interrupt threshold */
#define MBOX_DEVICE_ID		XPAR_MBOX_0_DEVICE_ID
#define INTC_DEVICE_ID		XPAR_INTC_0_DEVICE_ID
#define MBOX_INTR_ID		XPAR_INTC_0_MBOX_0_VEC_ID

static XIntc* Intc_ptr;
function_ptr_t mailbox_rx_callback;

#endif


void nullCallback(void* param){};

XMbox ipc_mailbox;
XMutex pkt_buf_mutex;



int wlan_lib_init () {
	u32 i;

	#ifdef XPAR_INTC_0_DEVICE_ID
	mailbox_rx_callback = (function_ptr_t)nullCallback;
	#endif

	//Initialize the pkt buffer mutex core
	XMutex_Config *mutex_ConfigPtr;
	mutex_ConfigPtr = XMutex_LookupConfig(PKT_BUF_MUTEX_DEVICE_ID);
	XMutex_CfgInitialize(&pkt_buf_mutex, mutex_ConfigPtr, mutex_ConfigPtr->BaseAddress);

	//Initialize the inter-processor mailbox core
	XMbox_Config *mbox_ConfigPtr;
	mbox_ConfigPtr = XMbox_LookupConfig(MAILBOX_DEVICE_ID);
	XMbox_CfgInitialize(&ipc_mailbox, mbox_ConfigPtr, mbox_ConfigPtr->BaseAddress);

	//Unlock all mutexes this CPU might own at boot
	// Most unlocks will fail harmlessly, but this helps cleanup state on soft reset
	for(i=0; i < NUM_TX_PKT_BUFS; i++) {
		unlock_pkt_buf_tx(i);
	}
	for(i=0; i < NUM_RX_PKT_BUFS; i++) {
		unlock_pkt_buf_rx(i);
	}

	return 0;
}

#ifdef XPAR_INTC_0_DEVICE_ID

int wlan_lib_setup_mailbox_interrupt(XIntc* intc){
	int Status;

	Intc_ptr = intc;

	XMbox_SetSendThreshold(&ipc_mailbox, MAILBOX_SIT);
	XMbox_SetReceiveThreshold(&ipc_mailbox, MAILBOX_RIT);

	Status = XIntc_Connect(Intc_ptr, MBOX_INTR_ID, (XInterruptHandler)MailboxIntrHandler, (void *)&ipc_mailbox);

	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	//XMbox_SetInterruptEnable(&ipc_mailbox, XMB_IX_STA | XMB_IX_RTA | XMB_IX_ERR);
	XMbox_SetInterruptEnable(&ipc_mailbox, XMB_IX_RTA);

	XIntc_Enable(Intc_ptr, MBOX_INTR_ID);

	return 0;
}

void wlan_lib_setup_mailbox_rx_callback( void(*callback)()){
	mailbox_rx_callback = (function_ptr_t)callback;
}

void MailboxIntrHandler(void *CallbackRef){
	u32 Mask;
	XMbox *MboxInstPtr = (XMbox *)CallbackRef;

	XIntc_Stop(Intc_ptr);

	Mask = XMbox_GetInterruptStatus(MboxInstPtr);

	XMbox_ClearInterrupt(MboxInstPtr, XMB_IX_RTA);

//	xil_printf("INTERRUPT: 0x%x\n",Mask);

//	if (Mask & XMB_IX_STA) {
		//Send interrupt. Do nothing.
//	}

	if (Mask & XMB_IX_RTA) {
		//xil_printf("CAALLLL BAAACCKKK addr = 0x%08x\n", mailbox_rx_callback);
		mailbox_rx_callback();
	}

//	if (Mask & XMB_IX_ERR) {
//		xil_printf("Error = 0x%x\n", XMbox_ReadReg(MboxInstPtr->Config.BaseAddress,XMB_ERROR_REG_OFFSET));
//		warp_printf(PL_ERROR, "Error reported by Mailbox via interrupt\n");
//	}

	XIntc_Start(Intc_ptr, XIN_REAL_MODE);


}



#endif
inline int wlan_lib_mac_rate_to_mbps (u8 rate) {
	switch(rate){
		case WLAN_MAC_RATE_1M:
			return 1;
		break;
		case WLAN_MAC_RATE_6M:
			return 6;
		break;
		case WLAN_MAC_RATE_9M:
			return 9;
		break;
		case WLAN_MAC_RATE_12M:
			return 12;
		break;
		case WLAN_MAC_RATE_18M:
			return 18;
		break;
		case WLAN_MAC_RATE_24M:
			return 24;
		break;
		case WLAN_MAC_RATE_36M:
			return 36;
		break;
		case WLAN_MAC_RATE_48M:
			return 48;
		break;
		case WLAN_MAC_RATE_54M:
			return 54;
		break;
	}
	return -1;
}

/************** Pkt Buffer Mutex Management ************/
int lock_pkt_buf_tx(u8 pkt_buf_ind) {
	int status;

	//Check inputs
	if(pkt_buf_ind >= NUM_TX_PKT_BUFS)
		return PKT_BUF_MUTEX_FAIL_INVALID_BUF;


	status = XMutex_Trylock(&pkt_buf_mutex, (pkt_buf_ind + PKT_BUF_MUTEX_TX_BASE));

	if(status == XST_SUCCESS)
		return PKT_BUF_MUTEX_SUCCESS;
	else
		return PKT_BUF_MUTEX_FAIL_ALREADY_LOCKED;
}

int lock_pkt_buf_rx(u8 pkt_buf_ind) {
	int status;

	//Check inputs
	if(pkt_buf_ind >= NUM_RX_PKT_BUFS)
		return PKT_BUF_MUTEX_FAIL_INVALID_BUF;


	status = XMutex_Trylock(&pkt_buf_mutex, (pkt_buf_ind + PKT_BUF_MUTEX_RX_BASE));

	if(status == XST_SUCCESS)
		return PKT_BUF_MUTEX_SUCCESS;
	else
		return PKT_BUF_MUTEX_FAIL_ALREADY_LOCKED;
}

int unlock_pkt_buf_tx(u8 pkt_buf_ind) {
	int status;

	//Check inputs
	if(pkt_buf_ind >= NUM_TX_PKT_BUFS)
		return PKT_BUF_MUTEX_FAIL_INVALID_BUF;

	status = XMutex_Unlock(&pkt_buf_mutex, (pkt_buf_ind + PKT_BUF_MUTEX_TX_BASE));

	if(status == XST_SUCCESS)
		return PKT_BUF_MUTEX_SUCCESS;
	else
		return PKT_BUF_MUTEX_FAIL_NOT_LOCK_OWNER;
}

int unlock_pkt_buf_rx(u8 pkt_buf_ind) {
	int status;

	//Check inputs
	if(pkt_buf_ind >= NUM_RX_PKT_BUFS)
		return PKT_BUF_MUTEX_FAIL_INVALID_BUF;

	status = XMutex_Unlock(&pkt_buf_mutex, (pkt_buf_ind + PKT_BUF_MUTEX_RX_BASE));

	if(status == XST_SUCCESS)
		return PKT_BUF_MUTEX_SUCCESS;
	else
		return PKT_BUF_MUTEX_FAIL_NOT_LOCK_OWNER;
}

int status_pkt_buf_tx(u8 pkt_buf_ind, u32* Locked, u32 *Owner){

	//Check inputs
	if(pkt_buf_ind >= NUM_TX_PKT_BUFS)
		return PKT_BUF_MUTEX_FAIL_INVALID_BUF;

	XMutex_GetStatus(&pkt_buf_mutex, (pkt_buf_ind + PKT_BUF_MUTEX_TX_BASE), Locked, Owner);

	return PKT_BUF_MUTEX_SUCCESS;
}

int status_pkt_buf_rx(u8 pkt_buf_ind, u32* Locked, u32 *Owner){

	//Check inputs
	if(pkt_buf_ind >= NUM_RX_PKT_BUFS)
		return PKT_BUF_MUTEX_FAIL_INVALID_BUF;

	XMutex_GetStatus(&pkt_buf_mutex, (pkt_buf_ind + PKT_BUF_MUTEX_RX_BASE), Locked, Owner);

	return PKT_BUF_MUTEX_SUCCESS;
}

/************** Inter-processor Messaging ************/


int ipc_mailbox_write_msg(wlan_ipc_msg* msg) {

	//Check that msg points to a valid IPC message
	if( ((msg->msg_id) & IPC_MBOX_MSG_ID_DELIM) != IPC_MBOX_MSG_ID_DELIM) {
		return IPC_MBOX_INVALID_MSG;
	}

	//Check that msg isn't too long
	if( (msg->num_payload_words) > IPC_MBOX_MAX_MSG_WORDS) {
		return IPC_MBOX_INVALID_MSG;
	}

	//Write msg header (first 32b word)
	XMbox_WriteBlocking(&ipc_mailbox, (u32*)msg, 4);

	if((msg->num_payload_words) > 0) {
		//Write msg payload
		XMbox_WriteBlocking(&ipc_mailbox, (u32*)(msg->payload_ptr), (u32)(4 * (msg->num_payload_words)));
	}

	return IPC_MBOX_SUCCESS;
}


inline int ipc_mailbox_read_isempty(){
	return XMbox_IsEmpty(&ipc_mailbox);
}

int ipc_mailbox_read_msg(wlan_ipc_msg* msg) {
	u32 bytes_read;
	int status;

	//Mailbox read functions:
	//int XMbox_Read(XMbox *InstancePtr, u32 *BufferPtr, u32 RequestedBytes, u32 *BytesRecvdPtr);
	//void XMbox_ReadBlocking(XMbox *InstancePtr, u32 *BufferPtr, u32 RequestedBytes)
	//status = XMbox_IsEmpty(&ipc_mailbox);

	//DEBUG
	//return IPC_MBOX_INVALID_MSG;
	//DEBUG

	if(XMbox_IsEmpty(&ipc_mailbox)){
		return IPC_MBOX_INVALID_MSG;
	}


	//Attempt to read one 32b word from the mailbox into the user-supplied msg struct
	status = XMbox_Read(&ipc_mailbox, (u32*)msg, 4, &bytes_read);
	if((status != XST_SUCCESS) || (bytes_read != 4) ) {
		return IPC_MBOX_NO_MSG_AVAIL;
	}

	//Check if the received word is a valid msg
	if( ((msg->msg_id) & IPC_MBOX_MSG_ID_DELIM) != IPC_MBOX_MSG_ID_DELIM) {
		XMbox_Flush(&ipc_mailbox);
		return IPC_MBOX_INVALID_MSG;
	}

	//Check that msg isn't too long
	if( (msg->num_payload_words) > IPC_MBOX_MAX_MSG_WORDS) {
		XMbox_Flush(&ipc_mailbox);
		return IPC_MBOX_INVALID_MSG;
	}

	//Message header must have been valid; wait for all remaining words
	if((msg->num_payload_words) > 0) {
		XMbox_ReadBlocking(&ipc_mailbox, (u32*)(msg->payload_ptr), 4 * (msg->num_payload_words));
	}

	return IPC_MBOX_SUCCESS;
}
