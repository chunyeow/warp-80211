/** @file wlan_mac_eth_util.c
 *  @brief Ethernet Framework
 *
 *  Contains code for using Ethernet, including encapsulation and de-encapsulation.
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

#include "xaxiethernet.h"
#include "xaxidma.h"
#include "xparameters.h"
#include "xintc.h"
#include "wlan_mac_ipc_util.h"
#include "wlan_mac_misc_util.h"
#include "wlan_mac_802_11_defs.h"
#include "wlan_mac_high.h"
#include "wlan_mac_packet_types.h"
#include "wlan_mac_dl_list.h"
#include "wlan_mac_queue.h"
#include "wlan_mac_fmc_pkt.h"
#include "wlan_mac_eth_util.h"

#include "wlan_mac_queue.h"

//Global variable for instance of the axi_dma driver, scoped to this file only
static XAxiDma ETH_A_DMA_Instance;

//Top-level code defines the callback, wlan_mac_util does the actual calling
// It's sufficient to refer to the wlan_mac_util callback by name here
function_ptr_t eth_rx_callback;

u8 eth_encap_mode;

static XIntc* Intc_ptr;

u32 ETH_A_NUM_RX_BD;

#define ETH_A_RX_INTR_ID		XPAR_INTC_0_AXIDMA_0_S2MM_INTROUT_VEC_ID
#define ETH_A_TX_INTR_ID		XPAR_INTC_0_AXIDMA_0_MM2S_INTROUT_VEC_ID

//The station code's implementation of encapsulation and de-encapsulation has an important
//limitation: only one device may be plugged into the station's Ethernet port. The station
//does not provide NAT. It assumes that the last received Ethernet src MAC address is used
//as the destination MAC address on any Ethernet transmissions. This is fine when there is
//only one device on the station's Ethernet port, but will definitely not work if the station
//is plugged into a switch with more than one device.
u8 eth_sta_mac_addr[6];

extern wlan_mac_hw_info   	hw_info;

/**
 * @brief Initializes the axi_dma hardware and framework for handling Ethernet Tx/Rx via the DMA
 *
 * This function must be called once at boot, before any Ethernet configuration or Tx/Rx operations.
 *
 * @return 0 on success
*/
int wlan_eth_init() {
		int status;

		eth_rx_callback = (function_ptr_t)nullCallback;

		ETH_A_NUM_RX_BD = min(queue_total_size()/2,200);
		xil_printf("Setting up %d DMA BDs\n", ETH_A_NUM_RX_BD);

		//The TEMAC driver is only used during init - all packet interactions are handed via the DMA driver
		XAxiEthernet_Config *ETH_A_MAC_CFG_ptr;
		XAxiEthernet ETH_A_MAC_Instance;

		ETH_A_MAC_CFG_ptr = XAxiEthernet_LookupConfig(ETH_A_MAC_DEVICE_ID);
		status = XAxiEthernet_CfgInitialize(&ETH_A_MAC_Instance, ETH_A_MAC_CFG_ptr, ETH_A_MAC_CFG_ptr->BaseAddress);
		if (status != XST_SUCCESS) {xil_printf("Error in XAxiEthernet_CfgInitialize! Err = %d\n", status); return -1;};

		//Setup the TEMAC options
		status  = XAxiEthernet_ClearOptions(&ETH_A_MAC_Instance, XAE_LENTYPE_ERR_OPTION | XAE_FLOW_CONTROL_OPTION | XAE_JUMBO_OPTION);
		status |= XAxiEthernet_SetOptions(&ETH_A_MAC_Instance, XAE_FCS_STRIP_OPTION | XAE_PROMISC_OPTION | XAE_MULTICAST_OPTION | XAE_BROADCAST_OPTION | XAE_FCS_INSERT_OPTION);
		status |= XAxiEthernet_SetOptions(&ETH_A_MAC_Instance, XAE_RECEIVER_ENABLE_OPTION | XAE_TRANSMITTER_ENABLE_OPTION);
		if (status != XST_SUCCESS) {xil_printf("Error in XAxiEthernet_Set/ClearOptions! Err = %d\n", status); return -1;};

		XAxiEthernet_SetOperatingSpeed(&ETH_A_MAC_Instance, ETH_A_LINK_SPEED);

		//Initialize the axi_dma attached to the TEMAC
		status = wlan_eth_dma_init();

		XAxiEthernet_Start(&ETH_A_MAC_Instance);

		return 0;
}

/**
 * @brief Sets the MAC callback for Ethernet receptions
 *
 * The framework will call the MAC's callback for each Ethernet reception that is a candidate
 * for wireless transmission. The framework may reject some packets for malformed or unrecognized
 * Ethernet headers. The MAC may reject more, based on Ethernet address or packet contents.
 *
 * @param void(*callback)
 *  -Function pointer to the MAC's Ethernet Rx callback
 */
void wlan_mac_util_set_eth_rx_callback(void(*callback)()){
	eth_rx_callback = (function_ptr_t)callback;
	return;
}

/**
 * @brief Sets the wired-wireless encapsulation mode
 *
 * @param u8 mode
 *  -Must be ENCAP_MODE_AP or ENCAP_MODE_STA, indicating AP-style or STA-style encapsulation/de-encapsulation
 */
void wlan_mac_util_set_eth_encap_mode(u8 mode) {
	eth_encap_mode = mode;
	return;
}

/**
 * @brief Returns the total number of axi_dma Rx buffer descriptors
 */
inline int eth_get_num_rx_bd() {
	return ETH_A_NUM_RX_BD;
}

/**
 * @brief Configures and connects the axi_dma interrupt to the system interrupt controller
 *
 * @param XIntc* intc
 *  - axi_intc driver instance - this function must be called after the axi_intc is setup
 * @return 0 on success, non-zero otherwise
*/
int wlan_eth_setup_interrupt(XIntc* intc){
	Intc_ptr = intc;
	int Status;

	//The interrupt controller will remember an arbitrary value and pass it to the callback
	// when this interrupt fires. We use this to pass the axi_dma Rx BD ring pointer into
	// the eth_rx_interrupt_handler() callback
	XAxiDma_BdRing *RxRingPtr = XAxiDma_GetRxRing(&ETH_A_DMA_Instance);

	//Connect the axi_dma interrupt
	Status = XIntc_Connect(Intc_ptr, ETH_A_RX_INTR_ID, (XInterruptHandler)eth_rx_interrupt_handler, RxRingPtr);

	if (Status != XST_SUCCESS) {
		xil_printf("ERROR: Failed to connect axi_dma interrupt to intc! (%d)\n", Status);
		return XST_FAILURE;
	}

	XIntc_Enable(Intc_ptr, ETH_A_RX_INTR_ID);

	return 0;
}

/**
 * @brief Interrupt handler for ETH DMA receptions
 *
* @param void* callbarck_arg
 *  - Argument passed in by interrupt controller (pointer to axi_dma Rx BD ring for Eth Rx)
*/
void eth_rx_interrupt_handler(void *callbarck_arg) {
	XAxiDma_BdRing *RxRingPtr = (XAxiDma_BdRing *) callbarck_arg;
	u32 IrqStatus;

#ifdef _ISR_PERF_MON_EN_
	wlan_mac_high_set_debug_gpio(ISR_PERF_MON_GPIO_MASK);
#endif

	//FIXME: Why do we explicitly disable interrupts here? Doesn't everything below happen in the context
	// of the ISR anyway?
	XIntc_Stop(Intc_ptr);

	IrqStatus = XAxiDma_BdRingGetIrq(RxRingPtr);
	XAxiDma_BdRingAckIrq(RxRingPtr, IrqStatus);

	if ((IrqStatus & (XAXIDMA_IRQ_DELAY_MASK | XAXIDMA_IRQ_IOC_MASK))) {
		//Interrupt status indicates at least one reception is completed
		// Call helper function to handle the received packet
		wlan_poll_eth_rx();
	}

	XIntc_Start(Intc_ptr, XIN_REAL_MODE);

#ifdef _ISR_PERF_MON_EN_
	wlan_mac_high_clear_debug_gpio(ISR_PERF_MON_GPIO_MASK);
#endif
	return;
}

/**
 * @brief Initializes the axi_dma core that handles Tx/Rx of Ethernet packets on ETH A
 *
 * Refer to the axi_dma docs and axi_ethernet driver examples for more details on using
 * the axi_dma's scatter-gather mode to handle Ethernet Tx/Rx.
 *
 * @return 0 on success, -1 otherwise
*/
int wlan_eth_dma_init() {
	int status;
	int bd_count;
	int i;
	u32 buf_addr;

	XAxiDma_Config *ETH_A_DMA_CFG_ptr;

	XAxiDma_Bd ETH_DMA_BD_Template;
	XAxiDma_BdRing *ETH_A_TxRing_ptr;
	XAxiDma_BdRing *ETH_A_RxRing_ptr;

	XAxiDma_Bd *first_bd_ptr;
	XAxiDma_Bd *cur_bd_ptr;

	tx_queue_element*	curr_tx_queue_element;

	ETH_A_DMA_CFG_ptr = XAxiDma_LookupConfig(ETH_A_DMA_DEV_ID);
	status = XAxiDma_CfgInitialize(&ETH_A_DMA_Instance, ETH_A_DMA_CFG_ptr);
	if(status != XST_SUCCESS) {xil_printf("Error in XAxiDma_CfgInitialize! Err = %d\n", status); return -1;}

	//Zero-out the template buffer descriptor
	XAxiDma_BdClear(&ETH_DMA_BD_Template);

	//Fetch handles to the Tx and Rx BD rings
	ETH_A_TxRing_ptr = XAxiDma_GetTxRing(&ETH_A_DMA_Instance);
	ETH_A_RxRing_ptr = XAxiDma_GetRxRing(&ETH_A_DMA_Instance);

	//Disable all Tx/Rx DMA interrupts
	XAxiDma_BdRingIntDisable(ETH_A_TxRing_ptr, XAXIDMA_IRQ_ALL_MASK);
	XAxiDma_BdRingIntDisable(ETH_A_RxRing_ptr, XAXIDMA_IRQ_ALL_MASK);

	//Disable delays and coalescing by default
	// We observed no performance increase with interrupt coalescing
	XAxiDma_BdRingSetCoalesce(ETH_A_TxRing_ptr, 1, 0);
	XAxiDma_BdRingSetCoalesce(ETH_A_RxRing_ptr, 1, 0);

	//Setup Tx/Rx buffer descriptor rings in memory
	status =  XAxiDma_BdRingCreate(ETH_A_TxRing_ptr, ETH_A_TX_BD_SPACE_BASE, ETH_A_TX_BD_SPACE_BASE, XAXIDMA_BD_MINIMUM_ALIGNMENT, ETH_A_NUM_TX_BD);
	status |= XAxiDma_BdRingCreate(ETH_A_RxRing_ptr, ETH_A_RX_BD_SPACE_BASE, ETH_A_RX_BD_SPACE_BASE, XAXIDMA_BD_MINIMUM_ALIGNMENT, ETH_A_NUM_RX_BD);
	if(status != XST_SUCCESS) {xil_printf("Error creating DMA BD Rings! Err = %d\n", status); return -1;}

	//Populate each ring with empty buffer descriptors
	status =  XAxiDma_BdRingClone(ETH_A_TxRing_ptr, &ETH_DMA_BD_Template);
	status |= XAxiDma_BdRingClone(ETH_A_RxRing_ptr, &ETH_DMA_BD_Template);
	if(status != XST_SUCCESS) {xil_printf("Error in XAxiDma_BdRingClone()! Err = %d\n", status); return -1;}

	//Start the DMA Tx channel
	// No Eth packets are transmitted until actual Tx BD's are pushed to the DMA hardware
	status = XAxiDma_BdRingStart(ETH_A_TxRing_ptr);

	//Initialize the Rx buffer descriptors
	bd_count = XAxiDma_BdRingGetFreeCnt(ETH_A_RxRing_ptr);
	if(bd_count != ETH_A_NUM_RX_BD) {xil_printf("Error in Eth Rx DMA init - not all Rx BDs were free at boot\n");}

	status = XAxiDma_BdRingAlloc(ETH_A_RxRing_ptr, bd_count, &first_bd_ptr);
	if(status != XST_SUCCESS) {xil_printf("Error in XAxiDma_BdRingAlloc()! Err = %d\n", status); return -1;}

	//Iterate over each Rx buffer descriptor
	cur_bd_ptr = first_bd_ptr;
	for(i = 0; i < bd_count; i++) {

		curr_tx_queue_element = queue_checkout();

		if(curr_tx_queue_element == NULL){
			xil_printf("Error during wlan_eth_dma_init: unable to check out sufficient tx_queue_element\n");
			return -1;
		}

		//Set the memory address for this BD's buffer to the corresponding Tx queue entry buffer
		// The Ethernet payload will be copied to an offset in the queue entry, leaving room for meta data at the front
		buf_addr = (u32)((void*)((tx_queue_buffer*)(curr_tx_queue_element->data))->frame + sizeof(mac_header_80211) + sizeof(llc_header) - sizeof(ethernet_header));
		status = XAxiDma_BdSetBufAddr(cur_bd_ptr, buf_addr);
		if(status != XST_SUCCESS) {xil_printf("XAxiDma_BdSetBufAddr failed (bd %d, addr 0x08x)! Err = %d\n", i, buf_addr, status); return -1;}

		//Set every Rx BD to max length (this assures 1 BD per Rx pkt)
		status = XAxiDma_BdSetLength(cur_bd_ptr, ETH_A_PKT_BUF_SIZE, ETH_A_RxRing_ptr->MaxTransferLen);
		if(status != XST_SUCCESS) {xil_printf("XAxiDma_BdSetLength failed (bd %d, addr 0x08x)! Err = %d\n", i, buf_addr, status); return -1;}

		//Rx BD's don't need control flags before use; DMA populates these post-Rx
		XAxiDma_BdSetCtrl(cur_bd_ptr, 0);

		//BD ID is arbitrary; use pointer to the dl_entry associated with this BD
		XAxiDma_BdSetId(cur_bd_ptr, (u32)curr_tx_queue_element);

		//Update cur_bd_ptr to the next BD in the chain for the next iteration
		cur_bd_ptr = XAxiDma_BdRingNext(ETH_A_RxRing_ptr, cur_bd_ptr);
	}

	//Push the Rx BD ring to hardware and start receiving
	status = XAxiDma_BdRingToHw(ETH_A_RxRing_ptr, bd_count, first_bd_ptr);

	//Enable Interrupts
	XAxiDma_BdRingIntEnable(ETH_A_RxRing_ptr, XAXIDMA_IRQ_ALL_MASK);

	status |= XAxiDma_BdRingStart(ETH_A_RxRing_ptr);
	if(status != XST_SUCCESS) {xil_printf("Error in XAxiDma_BdRingToHw/XAxiDma_BdRingStart(ETH_A_RxRing_ptr)! Err = %d\n", status); return -1;}

	return 0;
}


/**
 * @brief De-encapsulates a wireless reception and prepares it for transmission via Ethernet
 *
 * This function implements the de-encapsulation process for the wireless-wired portal. See
 * the 802.11 Reference Design user guide for more details:
 * http://warpproject.org/trac/wiki/802.11/MAC/Upper/MACHighFramework/EthEncap
 *
 * In addition to de-encapsulation this function implements one extra behavior. When the AP
 * observes a DHCP request about to be transmitted via Ethernet, it inspects the DHCP payload
 * to extract the hostname field. When the source packet was a wireless transmission from an
 * associated STA, this hostname field reflects the user-specified name of the device which
 * generated the DHCP request. For most devices this name is human readable (like "jacks-phone").
 * The AP copies this hostname to the station_info.hostname field in the STA's entry in
 * the association table.
 *
 * This functionality is purely for user convenience- the AP only displays the hostname,
 * it never makes state decisions based on its value. The hostname can bogus or missing
 * without affecting the behavior of the AP.
 * @param u8* mpdu
 *  - Pointer to the first byte of the packet received from the wireless interface
 * @param u32 length
 *  - Length (in bytes) of the packet to send
 * @return 0 for successful de-encapsulation, -1 otherwise; failure usually indicates malformed or unrecognized LLC header
*/
int wlan_mpdu_eth_send(void* mpdu, u16 length) {
	int status;
	u8* eth_mid_ptr;

	mac_header_80211* rx80211_hdr;
	llc_header* llc_hdr;
	ethernet_header* eth_hdr;

	rx_frame_info* frame_info;

	ipv4_header* ip_hdr;
	arp_packet* arp;
	udp_header* udp;
	dhcp_packet* dhcp;

	u8 continue_loop;
	u8 is_dhcp_req = 0;

	u8 addr_cache[6];

	u32 len_to_send;

	if(length < (sizeof(mac_header_80211) + sizeof(llc_header))){
		xil_printf("Error in wlan_mpdu_eth_send: length of %d is too small... must be at least %d\n", length, (sizeof(mac_header_80211) + sizeof(llc_header)));
		return -1;
	}

	//Get helper pointers to various byte offsets in the packet payload
	rx80211_hdr = (mac_header_80211*)((void *)mpdu);
	llc_hdr = (llc_header*)((void *)mpdu + sizeof(mac_header_80211));
	eth_hdr = (ethernet_header*)((void *)mpdu + sizeof(mac_header_80211) + sizeof(llc_header) - sizeof(ethernet_header));

	//Calculate length of de-encapsulated Ethernet packet
	len_to_send = length - sizeof(mac_header_80211) - sizeof(llc_header) + sizeof(ethernet_header);

	//Do de-encapsulation of wireless packet
	switch(eth_encap_mode) {
		case ENCAP_MODE_AP:

			//Map the 802.11 header address fields to Ethernet header address fields
			// For AP de-encapsulation, (eth.dest == wlan.addr3) and (eth.src == wlan.addr2)
			memmove(eth_hdr->address_destination, rx80211_hdr->address_3, 6);
			memmove(eth_hdr->address_source, 	  rx80211_hdr->address_2, 6);

			//Set the ETHER_TYPE field in the Ethernet header
			switch(llc_hdr->type){
				case LLC_TYPE_ARP:
					eth_hdr->type = ETH_TYPE_ARP;
				break;

				case LLC_TYPE_IP:
					eth_hdr->type = ETH_TYPE_IP;

					//Check if this is a DHCP discover packet in a UDP packet
					// If so, extract the hostname field from the DHCP discover payload and
					//  update the corresponding STA association table entry
					// This hostname is purely for convenience- the hostname is easier to recognize than
					//  the STA MAC address. The hostname can be blank without affecting any AP functionality.
					ip_hdr = (ipv4_header*)((void*)eth_hdr + sizeof(ethernet_header));
					if(ip_hdr->prot == IPV4_PROT_UDP){
						udp = (udp_header*)((void*)ip_hdr + 4*((u8)(ip_hdr->ver_ihl) & 0xF));

						if(Xil_Ntohs(udp->src_port) == UDP_SRC_PORT_BOOTPC || Xil_Ntohs(udp->src_port) == UDP_SRC_PORT_BOOTPS){
							dhcp = (dhcp_packet*)((void*)udp + sizeof(udp_header));

							if(Xil_Ntohl(dhcp->magic_cookie) == DHCP_MAGIC_COOKIE){
									eth_mid_ptr = (u8*)((void*)dhcp + sizeof(dhcp_packet));

									//Iterate over all tagged parameters in the DHCP request, looking for the hostname parameter
									// Stop after 20 tagged parameters (handles case of mal-formed DHCP packets missing END tag)
									continue_loop = 20;
									while(continue_loop) {
										continue_loop--;
										switch(eth_mid_ptr[0]) {

											case DHCP_OPTION_TAG_TYPE:
												if((eth_mid_ptr[2] == DHCP_OPTION_TYPE_DISCOVER) ||
												   (eth_mid_ptr[2] == DHCP_OPTION_TYPE_REQUEST)) {
														is_dhcp_req = 1;
												}
											break;

											case DHCP_HOST_NAME:
												if(is_dhcp_req) {
													//Look backwards from the MPDU payload to find the wireless Rx pkt metadata (the rx_frame_info struct)
													frame_info = (rx_frame_info*)((u8*)mpdu  - PHY_RX_PKT_BUF_MPDU_OFFSET);

													if(frame_info->additional_info != NULL) {
														//rx_frame_info has pointer to STA entry in association table - fill in that entry's hostname field

														//Copy the string from the DHCP payload into the hostname field
														memcpy(((station_info*)(frame_info->additional_info))->hostname,
																&(eth_mid_ptr[2]),
																min(STATION_INFO_HOSTNAME_MAXLEN, eth_mid_ptr[1]));

														//Null-terminate the string in the association table entry
														((station_info*)(frame_info->additional_info))->hostname[ min(STATION_INFO_HOSTNAME_MAXLEN, eth_mid_ptr[1]) ] = NULL;
													}
										   	}
											break;

											case DHCP_OPTION_END:
												continue_loop = 0;
											break;
										}//END switch(DHCP tag type)

										//Increment by size of current tagged parameter
										eth_mid_ptr += (2+eth_mid_ptr[1]);

									}//END iterate over DHCP tags
							}
						}//END is DHCP?
					}//END is UDP?
				break;//END case(IP packet)

				default:
					//Invalid or unsupported Eth type; punt
					return -1;
				break;
			}//END switch(LLC type)
		break;

		case ENCAP_MODE_STA:
			if(wlan_addr_eq(rx80211_hdr->address_3, hw_info.hw_addr_wlan)){
				//This case handles the behavior of an AP reflecting a station-sent broadcast packet back out over the air.
				//Without this filtering, a station would forward the packet it just transmitted back to its wired interface.
				//This screws up DHCP and ARP behavior on the connected PC.
				return -1;
			}

			//Make temp copy of the 802.11 header address 3 field
			memcpy(addr_cache, rx80211_hdr->address_3, 6);

			//If this packet is addressed to this STA, use the wired device's MAC address as the Eth dest address
			if(wlan_addr_eq(rx80211_hdr->address_1, hw_info.hw_addr_wlan)) {
				memcpy(eth_hdr->address_destination, eth_sta_mac_addr, 6);
			} else {
				memmove(eth_hdr->address_destination, rx80211_hdr->address_1, 6);
			}

			//Insert the Eth source, from the 802.11 header address 1 field
			memcpy(eth_hdr->address_source, addr_cache, 6);

			switch(llc_hdr->type){
				case LLC_TYPE_ARP:
					eth_hdr->type = ETH_TYPE_ARP;

					//If the ARP packet is addressed to this STA wireless address, replace the ARP dest address
					// with the connected wired device's MAC address
					arp = (arp_packet*)((void*)eth_hdr + sizeof(ethernet_header));
					if(wlan_addr_eq(arp->eth_dst, hw_info.hw_addr_wlan)) {
						memcpy(arp->eth_dst, eth_sta_mac_addr, 6);
					}
				break;

				case ETH_TYPE_IP:
					eth_hdr->type = ETH_TYPE_IP;
					ip_hdr = (ipv4_header*)((void*)eth_hdr + sizeof(ethernet_header));
				break;

				default:
					//Invalid or unsupported Eth type; punt
					return -1;
				break;
			}
		break;
	}

#ifdef FMC_PKT_EN
	status = wlan_fmc_pkt_eth_send((u8*)eth_hdr, len_to_send);
	if(status != 0) {xil_printf("Error in wlan_fmc_pkt_eth_send! Err = %d\n", status); return -1;}
#else
	status = wlan_eth_dma_send((u8*)eth_hdr, len_to_send);
	if(status != 0) {xil_printf("Error in wlan_mac_send_eth! Err = %d\n", status); return -1;}
#endif


	return 0;
}


/**
 * @brief Transmits a packet over Ethernet using the ETH DMA
 *
 * This function transmits a single packet via Ethernet using the axi_dma. The packet passed via pkt_ptr
 * must be a valid Ethernet packet, complete with 14-byte Ethernet header. This function does not check
 * for a valid header - the calling function must ensure this.
 *
 * The packet must be stored in a memory location accessible by the ETH A axi_dma core. The MicroBlaze DLMB
 * is *not* accessible to the DMA. Thus packets cannot be stored in malloc'd areas (the heap is in the DLMB).
 * In the reference implementation all Ethernet transmissions start as wireless receptions. Thus, the Ethernet
 * payloads are stored in the wireless Rx packet buffer, which is accessible by the DMA.
 *
 * Custom code which needs to send Ethernet packets can use a spare wireless Tx/Rx packet buffer, a spare
 * Tx queue entry in DRAM or the user scratch space in DRAM to create custom Ethernet payloads.
 *
 * This function blocks until the Ethernet transmission completes.
 *
 * @param u8* pkt_ptr
 *  - Pointer to the first byte of the Ethernet header of the packet to send; valid header must be created before calling this function
 * @param u32 length
 *  - Length (in bytes) of the packet to send
 * @return 0 for successful Ethernet transmission, -1 otherwise
*/
int wlan_eth_dma_send(u8* pkt_ptr, u32 length) {
	int status;
	XAxiDma_BdRing *txRing_ptr;
	XAxiDma_Bd *cur_bd_ptr;


	if( (length == 0) || (length > 1514) ){
		xil_printf("Error in wlan_eth_dma_send: length = %d\n", length);
		return -1;
	}

	//Important: if the data cache is enabled the cache must be flushed before attempting to send a packet
	// via the ETH DMA. The DMA will read packet contents directly from RAM, bypassing any cache checking
	// normally done by the MicroBlaze
	//The data cache is disabled by default in the reference implementation
	//Xil_DCacheFlushRange((u32)TxPacket, MAX_PKT_LEN);

	//Check if the user-supplied pointer is in the DLMB, unreachable by the DMA
	if( ((u32)pkt_ptr > XPAR_MB_HIGH_DLMB_BRAM_CNTLR_0_BASEADDR && (u32)pkt_ptr < XPAR_MB_HIGH_DLMB_BRAM_CNTLR_0_HIGHADDR) ||
		((u32)pkt_ptr > XPAR_MB_HIGH_DLMB_BRAM_CNTLR_1_BASEADDR && (u32)pkt_ptr < XPAR_MB_HIGH_DLMB_BRAM_CNTLR_1_HIGHADDR))	{

		xil_printf("Error in Eth DMA send -- source address (0x%08x) not reachable by DMA\n", pkt_ptr);

		//FIXME: The out_of_range check was commented out below- why? Comment-in this return -1 instead? It should do the same thing.
		//	return -1;
	}

	//if(out_of_range == 0){

	//Get pointer to the axi_dma Tx buffer descriptor ring



	txRing_ptr = XAxiDma_GetTxRing(&ETH_A_DMA_Instance);

	//FIXME: Now that we have checked out a ring, we should be sure to check it back in even if configuring it fails
	//for some reason.

	//Allocate and setup one Tx BD
	status = XAxiDma_BdRingAlloc(txRing_ptr, 1, &cur_bd_ptr);
	//if((u32)cur_bd_ptr != 0xBF54C000) {xil_printf("TX BD at wrong address! 0x08%x != 0x%08x\n", cur_bd_ptr, 0xBF54C000);}

	status |= XAxiDma_BdSetBufAddr(cur_bd_ptr, (u32)pkt_ptr);
	status |= XAxiDma_BdSetLength(cur_bd_ptr, length, txRing_ptr->MaxTransferLen);
	if(status != XST_SUCCESS) {
		xil_printf("length = %d, txRing_ptr->MaxTransferLen = %d\n", length, txRing_ptr->MaxTransferLen );
		xil_printf("Error in setting ETH Tx BD! Err = %d\n", status);
		//while(1){}
		return -1;
	}

	//When using 1 BD for 1 pkt set both start-of-frame (SOF) and end-of-frame (EOF)
	XAxiDma_BdSetCtrl(cur_bd_ptr, (XAXIDMA_BD_CTRL_TXSOF_MASK | XAXIDMA_BD_CTRL_TXEOF_MASK) );

	//Set arbitrary ID (DMA ignores this)
	XAxiDma_BdSetId(cur_bd_ptr, (u32)pkt_ptr);

	//Push the BD ring to hardware; this initiates the actual DMA transfer and Ethernet Tx
	status = XAxiDma_BdRingToHw(txRing_ptr, 1, cur_bd_ptr);
	if(status != XST_SUCCESS) {xil_printf("Error in XAxiDma_BdRingToHw(txRing_ptr)! Err = %d\n", status); return -1;}

	//Wait for this DMA transfer to finish (will be replaced by post-Tx ISR)
	while (XAxiDma_BdRingFromHw(txRing_ptr, 1, &cur_bd_ptr) == 0) {/*Do Nothing*/}

	//Free the BD for future use
	status = XAxiDma_BdRingFree(txRing_ptr, 1, cur_bd_ptr);
	if(status != XST_SUCCESS) {xil_printf("Error in XAxiDma_BdRingFree(txRing_ptr, 1)! Err = %d\n", status); return -1;}

	return 0;

	//} else {
	//	xil_printf("Error in Eth DMA send -- source address not reachable by DMA\n");
	//	return -1;
	//}
}

/**
 * @brief Process any Ethernet packets that have been received by the ETH DMA
 *
 * This function checks for any ETH DMA buffer descriptors that have been used by the DMA, indicating
 * new Ethernet receptions. For each occupied buffer descriptor this function encapsulates the Ethernet packet
 * and calls the MAC's callback to either enqueue (for eventual wireless Tx) or reject the packet.
 *
 * Occupied ETH DMA buffer descriptors are freed and resubmitted to hardware for use by future Ethernet receptions
 *
 * This function requires the MAC implement a function (assigned to the eth_rx_callback function pointer) that
 * returns 0 or 1, indicating the MAC's handling of the packet:
 *  0: Packet was not enqueued and will not be processed by wireless MAC; framework should immediately discard
 *  1: Packet was enqueued for eventual wireless transmission; MAC will check in occupied queue entry when finished
*/
inline void wlan_poll_eth_rx() {
	XAxiDma_BdRing *rxRing_ptr;
	XAxiDma_Bd *cur_bd_ptr;
	XAxiDma_Bd *first_bd_ptr;
	u8* mpdu_start_ptr;
	u8* eth_start_ptr;
	tx_queue_element* curr_tx_queue_element;
	u32 eth_rx_len, eth_rx_buf;
	u32 mpdu_tx_len;
	u32 i;

	int bd_count;
	int status;
	int packet_is_queued;

	u8 eth_dest[6];
	u8 eth_src[6];

	static u32 max_bd_count = 0;

	//Retrieve the ETH DMA Rx ring
	rxRing_ptr = XAxiDma_GetRxRing(&ETH_A_DMA_Instance);

	//Lookup how many ETH Rx packets have been processed by the DMA
	bd_count = XAxiDma_BdRingFromHw(rxRing_ptr, XAXIDMA_ALL_BDS, &first_bd_ptr);
	cur_bd_ptr = first_bd_ptr;

	if(bd_count == 0) {
		//No Rx BDs have been processed - no new Eth receptions waiting
		return;
	}

	//Update local stats variable (used to gauge high-water mark for ETH DMA activity)
	if(bd_count > max_bd_count){
		max_bd_count = bd_count;
		//xil_printf("max_bd_count = %d\n",max_bd_count);
	}

	//At least one new ETH Rx packet is ready for processing
	for(i=0; i<bd_count; i++) {
		packet_is_queued = 0;

		//A packet has been received and transferred by DMA
		// We use the DMA "ID" field to hold a pointer to the Tx queue entry containing the packet contents
		curr_tx_queue_element = (tx_queue_element*)XAxiDma_BdGetId(cur_bd_ptr);

		//Lookup length and data pointers from the DMA metadata
		eth_rx_len = XAxiDma_BdGetActualLength(cur_bd_ptr, rxRing_ptr->MaxTransferLen);
		eth_rx_buf = XAxiDma_BdGetBufAddr(cur_bd_ptr);

		//After encapsulation, byte[0] of the MPDU will be at byte[0] of the queue entry frame buffer
		mpdu_start_ptr = (void*)((tx_queue_buffer*)(curr_tx_queue_element->data))->frame;
		eth_start_ptr = (u8*)eth_rx_buf;

		//Encapsulate the Ethernet packet
		// See the 802.11 Ref Design user guide for details on the encapsulation process
		mpdu_tx_len = wlan_eth_encap(mpdu_start_ptr, eth_dest, eth_src, eth_start_ptr, eth_rx_len);

		if(mpdu_tx_len == 0) {
			//Encapsulation failed for some reason (probably unknown ETHERTYPE value)
			// Don't pass the invalid frame to the MAC - just cleanup and punt
			packet_is_queued = 0;
		} else {
			//Call the MAC's callback to process the packet
			// MAC will either enqueue the packet for eventual transmission or reject the packet
			packet_is_queued = eth_rx_callback(curr_tx_queue_element, eth_dest, eth_src, mpdu_tx_len);
		}

		//If the packet was not successfully enqueued, discard it and return its queue entry to the free pool
		// For packets that are successfully enqueued, this cleanup is part of the post-wireless-Tx handler
		if(packet_is_queued == 0) {
			//Either the packet was invalid, or the MAC code failed to enqueue this packet
			// The MAC will fail if the appropriate queue was full or the Ethernet addresses were not recognized.

			//Return the occupied queue entry to the free pool
			queue_checkin(curr_tx_queue_element);
		}

		//Free the ETH DMA buffer descriptor
		status = XAxiDma_BdRingFree(rxRing_ptr, 1, cur_bd_ptr);
		if(status != XST_SUCCESS) {xil_printf("Error in XAxiDma_BdRingFree of Rx BD! Err = %d\n", status); return;}

		//Call helper function to reassign just-freed DMA buffer descriptor to a new queue entry
		wlan_eth_dma_update();

		//Update cur_bd_ptr to the next BD in the chain for the next iteration
		cur_bd_ptr = XAxiDma_BdRingNext(rxRing_ptr, cur_bd_ptr);
	}

	return;
}

/**
 * @brief Encapsulates Ethernet packets for wireless transmission
 *
 * This function implements the encapsulation process for 802.11 transmission of Ethernet packets
 *
 * The encapsulation process depends on the node's role:
 * AP:
 *  -Copy original packet's source and destination addresses to temporary space
 *  -Add an LLC header (8 bytes) in front of the Ethernet payload
 *   -LLC header includes original packet's ETHER_TYPE field; only IPV4 and ARP are currently supported
 *
 * STA:
 *  -Copy original packet's source and destination addresses to temporary space
 *  -Add an LLC header (8 bytes) in front of the Ethernet payload
 *   -LLC header includes original packet's ETHER_TYPE field; only IPV4 and ARP are currently supported
 *  -If packet is ARP Request, overwrite ARP header's source address with STA wireless MAC address
 *  -If packet is UDP packet containing a DHCP request
 *    -Assert DHCP header's BROADCAST flag
 *    -Disable the UDP packet checksum (otherwise it would be invliad after modifying the BROADCAST flag)
 *
 * Refer to the 802.11 Reference Design user guide for more details:
 *  http://warpproject.org/trac/wiki/802.11/MAC/Upper/MACHighFramework/EthEncap
 *
 * @param u8* mpdu_start_ptr
 *  - Pointer to the first byte of the MPDU payload (first byte of the eventual MAC header)
 * @param u8* eth_dest
 *  - Pointer to 6 bytes of free memory; will be overwritten with Ethernet packet's destination address
 * @param u8* eth_src
 *  - Pointer to 6 bytes of free memory; will be overwritten with Ethernet packet's source address
 * @param u8* eth_start_ptr
 *  - Pointer to first byte of received Ethernet packet's header
 * @param u32 eth_rx_len
 *  - Length (in bytes) of the packet payload
 * @return 0 for if packet type is unrecognized (failed encapsulation), otherwise returns length of encapsulated packet (in bytes)
*/
int wlan_eth_encap(u8* mpdu_start_ptr, u8* eth_dest, u8* eth_src, u8* eth_start_ptr, u32 eth_rx_len){
	u8* eth_mid_ptr;
	u8 continue_loop;
	ethernet_header* eth_hdr;
	ipv4_header* ip_hdr;
	arp_packet* arp;
	udp_header* udp;
	dhcp_packet* dhcp;

	llc_header* llc_hdr;
	u32 mpdu_tx_len;

	//Calculate actual wireless Tx len (eth payload - eth header + wireless header)
	mpdu_tx_len = eth_rx_len - sizeof(ethernet_header) + sizeof(llc_header) + sizeof(mac_header_80211);

	//Helper pointers to interpret/fill fields in the new MPDU
	eth_hdr = (ethernet_header*)eth_start_ptr;
	llc_hdr = (llc_header*)(mpdu_start_ptr + sizeof(mac_header_80211));

	//Copy the src/dest addresses from the received Eth packet to temp space
	memcpy(eth_src, eth_hdr->address_source, 6);
	memcpy(eth_dest, eth_hdr->address_destination, 6);

	//Prepare the MPDU LLC header
	llc_hdr->dsap = LLC_SNAP;
	llc_hdr->ssap = LLC_SNAP;
	llc_hdr->control_field = LLC_CNTRL_UNNUMBERED;
	bzero((void *)(llc_hdr->org_code), 3); //Org Code 0x000000: Encapsulated Ethernet

	switch(eth_encap_mode) {
		case ENCAP_MODE_AP:
			switch(eth_hdr->type) {
				case ETH_TYPE_ARP:
					llc_hdr->type = LLC_TYPE_ARP;
					arp = (arp_packet*)((void*)eth_hdr + sizeof(ethernet_header));
				break;
				case ETH_TYPE_IP:
					llc_hdr->type = LLC_TYPE_IP;
				break;
				default:
					//Unknown/unsupported EtherType; don't process the Eth frame
					return 0;
				break;
			}

		break;

		case ENCAP_MODE_STA:

			//Save this ethernet src address
			memcpy(eth_sta_mac_addr, eth_src, 6);
			memcpy(eth_src, hw_info.hw_addr_wlan, 6);

			switch(eth_hdr->type) {
				case ETH_TYPE_ARP:
					llc_hdr->type = LLC_TYPE_ARP;

					//Overwrite ARP request source MAC address field with the station's wireless MAC address.
					arp = (arp_packet*)((void*)eth_hdr + sizeof(ethernet_header));
					memcpy(arp->eth_src, hw_info.hw_addr_wlan, 6);

				break;
				case ETH_TYPE_IP:

					llc_hdr->type = LLC_TYPE_IP;

					//Check if IPv4 packet is a DHCP Discover in a UDP frame
					ip_hdr = (ipv4_header*)((void*)eth_hdr + sizeof(ethernet_header));
					if(ip_hdr->prot == IPV4_PROT_UDP){
						udp = (udp_header*)((void*)ip_hdr + 4*((u8)(ip_hdr->ver_ihl) & 0xF));

						if(Xil_Ntohs(udp->src_port) == UDP_SRC_PORT_BOOTPC || Xil_Ntohs(udp->src_port) == UDP_SRC_PORT_BOOTPS){
							//This is a DHCP Discover packet, which contains the source hardware address
							//deep inside the packet (in addition to its usual location in the Eth header).
							//For STA encapsulation, we need to overwrite this address with the MAC addr
							//of the wireless station.

							//Disable the checksum since we are about to mess with the bytes in the packet
							udp->checksum = 0;

							dhcp = (dhcp_packet*)((void*)udp + sizeof(udp_header));

							if(Xil_Ntohl(dhcp->magic_cookie) == DHCP_MAGIC_COOKIE){
								eth_mid_ptr = (u8*)((void*)dhcp + sizeof(dhcp_packet));

								//Assert the DHCP Discover's BROADCAST flag; this signals to any DHCP severs that their responses
								// should be sent to the broadcast address. This is necessary for the DHCP response to propagate back
								// through the wired-wireless portal at the AP, through the STA Rx MAC filters, back out the
								// wireless-wired portal at the STA, and finally into the DHCP listener at the wired device
								dhcp->flags = Xil_Htons(DHCP_BOOTP_FLAGS_BROADCAST);

								//Tagged DHCP Options
								continue_loop = 1;

								//FIXME: Why does this loop exist? It doesn't affect any state. It looks like it mucked
								// with addresses in old code, but maybe this isn't required anymore?
								while(continue_loop){
									switch(eth_mid_ptr[0]){

										case DHCP_OPTION_TAG_TYPE:
											switch(eth_mid_ptr[2]){
												case DHCP_OPTION_TYPE_DISCOVER:
												case DHCP_OPTION_TYPE_REQUEST:
													//memcpy(dhcp->chaddr,hw_info.hw_addr_wlan,6);
												break;

											}

										break;

										case DHCP_OPTION_TAG_IDENTIFIER:
											//memcpy(&(eth_mid_ptr[3]),hw_info.hw_addr_wlan,6);

										break;

										case DHCP_OPTION_END:
											continue_loop = 0;
										break;
									}
									eth_mid_ptr += (2+eth_mid_ptr[1]);
								}//END loop over DHCP tags
							}//END is DHCP valid
						}//END is DHCP
					}//END is UDP
				break;
				default:
					//Unknown/unsupported EtherType; don't process the Eth frame
					return 0;
				break;
		}//END switch(pkt type)

		break;

	}//END switch(encap mode)

	//If we got this far, the packet was successfully encapsulated; return the post-encapsulation length
	return mpdu_tx_len;
}

/**
 * @brief Recycles any free ETH Rx DMA buffer descriptors
 *
 * This function checks if any ETH DMA Rx buffer descriptors have been freed by the packet handling
 * code above. For each free BD, this function attempts to checkout a Tx queue entry and assign its
 * payload to the BD. If successful, the BD is then submitted to the DMA hardware for use by future
 * Ethernet receptions. If unsuccessful the BD is left free, to be recycled on the next iteration of
 * this function.
 *
 * The total number of ETH Rx buffer descriptors is set at boot during the DMA init. The same number
 * of Tx queue entries are effectively reserved by the MAC in its queue size calculations. This function
 * can handle the case of more ETH Rx BDs than free Tx queue entries, though this should never happen.
 *
 * This function should be called frequently to assure enough Rx BDs are available to the DMA hardware.
 * The reference implementation calls this function immediately upon freeing an Rx BD. This is perhaps
 * a bit aggressive, but we have not observed any resulting performance penalty.
 *
*/
void wlan_eth_dma_update() {
	int bd_count;
	int status;
	XAxiDma_BdRing *ETH_A_RxRing_ptr;
	XAxiDma_Bd *first_bd_ptr;
	XAxiDma_Bd *cur_bd_ptr;
	dl_list checkout;
	dl_entry*	tx_queue_entry;
	u32 i;
	u32 buf_addr;
	u32 num_available_packet_bd;

	ETH_A_RxRing_ptr = XAxiDma_GetRxRing(&ETH_A_DMA_Instance);
	bd_count = XAxiDma_BdRingGetFreeCnt(ETH_A_RxRing_ptr);

	num_available_packet_bd = queue_num_free();

	//Calculate number of BD-queue pairs to attempt
	// Minimum of:
	//  -Number of free BDs in the ETH Rx ring
	//  -Number of free Tx queue entries
	u32 bd_queue_pairs_to_process = min(num_available_packet_bd, bd_count);

	if(bd_queue_pairs_to_process > 0) {
		//Checkout free queue entries for each BD we can process
		queue_checkout_list(&checkout, bd_queue_pairs_to_process);

		//Update number of BDs to process if queue couldn't provide enough free entries
		// Handles rare race of queue status changing between queue_num_free() and queue_checkout()
		bd_queue_pairs_to_process = min(bd_queue_pairs_to_process, checkout.length);

		status = XAxiDma_BdRingAlloc(ETH_A_RxRing_ptr, bd_queue_pairs_to_process, &first_bd_ptr);
		if(status != XST_SUCCESS) {xil_printf("Error in XAxiDma_BdRingAlloc()! Err = %d\n", status); return;}

		tx_queue_entry = checkout.first;

		//Iterate over each Rx buffer descriptor
		cur_bd_ptr = first_bd_ptr;
		for(i = 0; i < bd_queue_pairs_to_process; i++) {
			//Set the memory address for this BD's buffer to the Tx queue entry's data area
			// This pointer is offset by the size of a MAC header and LLC header, which results in the Ethernet
			//  payload being copied to its post-encapsulated location. This speeds up the encapsulation process by
			//  skipping any re-copying of Ethernet payloads
			buf_addr = (u32)((void*)((tx_queue_buffer*)(tx_queue_entry->data))->frame + sizeof(mac_header_80211) + sizeof(llc_header) - sizeof(ethernet_header));
			status = XAxiDma_BdSetBufAddr(cur_bd_ptr, buf_addr);
			if(status != XST_SUCCESS) {xil_printf("XAxiDma_BdSetBufAddr failed (bd %d, addr 0x08x)! Err = %d\n", i, buf_addr, status); return;}

			//Set every Rx BD to max length (this assures 1 BD per Rx pkt)
			// The ETH DMA hardware will record the actual Rx length in its per-BD meta data
			status = XAxiDma_BdSetLength(cur_bd_ptr, ETH_A_PKT_BUF_SIZE, ETH_A_RxRing_ptr->MaxTransferLen);
			if(status != XST_SUCCESS) {xil_printf("XAxiDma_BdSetLength failed (bd %d, addr 0x08x)! Err = %d\n", i, buf_addr, status); return;}

			//Rx BD's don't need control flags before use; DMA populates these post-Rx
			XAxiDma_BdSetCtrl(cur_bd_ptr, 0);

			//BD ID is arbitrary; use pointer to the queue entry associated with this BD
			XAxiDma_BdSetId(cur_bd_ptr, (u32)tx_queue_entry);

			//Update the BD and queue entry pointers to the next list elements (this loop traverses both lists simultaneously)
			cur_bd_ptr = XAxiDma_BdRingNext(ETH_A_RxRing_ptr, cur_bd_ptr);
			tx_queue_entry = dl_entry_next(tx_queue_entry);
		}

		//Push the Rx BD ring to hardware and start receiving
		status = XAxiDma_BdRingToHw(ETH_A_RxRing_ptr, bd_queue_pairs_to_process, first_bd_ptr);
		if(status != XST_SUCCESS) {xil_printf("XAxiDma_BdRingToHw failed! Err = %d\n", status);}
	}
	return;
}

