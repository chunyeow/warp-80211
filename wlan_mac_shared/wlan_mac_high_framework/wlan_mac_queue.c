/** @file wlan_mac_queue.c
 *  @brief Transmit Queue Framework
 *
 *  This contains code for accessing the transmit queue.
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

#include "xil_types.h"
#include "stdlib.h"
#include "stdio.h"
#include "xparameters.h"
#include "xintc.h"
#include "string.h"

#include "wlan_mac_ipc_util.h"
#include "wlan_mac_high.h"
#include "wlan_mac_queue.h"
#include "wlan_mac_dl_list.h"
#include "wlan_mac_eth_util.h"

//This list holds all of the empty, free elements
static dl_list queue_free;

//This queue_tx vector will get filled in with elements from the queue_free list
//Note: this implementation sparsely packs the queue_tx array to allow fast
//indexing at the cost of some wasted memory. The queue_tx array will be
//reallocated whenever the upper-level MAC asks to enqueue at an index
//that is larger than the current size of the array. It is assumed that this
//index will not continue to grow over the course of execution, otherwise
//this array will continue to grow and eventually be unable to be reallocated.
//Practically speaking, this means an AP needs to re-use the AIDs it issues
//stations if it wants to use the AIDs as an index into the tx queue.
static dl_list* queue_tx;
static u16 num_queue_tx;


static u32 QUEUE_NUM_DL_ENTRY;
static u32 MAX_SIZE_FOR_PACKET_BD_DL_LIST;

static void* QUEUE_BUFFER_SPACE;

static u8 dram_present;

void queue_dram_present(u8 present){
	dram_present = present;
}

int queue_init(){
	u32 i;

	if(dram_present == 1){
		MAX_SIZE_FOR_PACKET_BD_DL_LIST = 48000;
		QUEUE_NUM_DL_ENTRY = MAX_SIZE_FOR_PACKET_BD_DL_LIST / (sizeof(dl_entry));
		xil_printf("Queue of %d placed in DRAM: using %d kB\n", QUEUE_NUM_DL_ENTRY, (QUEUE_NUM_DL_ENTRY*QUEUE_BUFFER_SIZE)/1024);
		QUEUE_BUFFER_SPACE = (void*)(DDR3_BASEADDR);

	} else {
		xil_printf("A working DRAM SODIMM has not been detected on this board. DRAM is required for\n");
		xil_printf("the wireless transmission queue. Halting\n");
		while(1){}
	}



	dl_list_init(&queue_free);

	bzero((void*)QUEUE_BUFFER_SPACE, QUEUE_NUM_DL_ENTRY*QUEUE_BUFFER_SIZE);

	//At boot, every dl_entry buffer descriptor is free
	//To set up the doubly linked list, we exploit the fact that we know the starting state is sequential.
	//This matrix addressing is not safe once the queue is used. The insert/remove helper functions should be used
	dl_entry* dl_entry_base;
	dl_entry_base = (dl_entry*)(QUEUE_DL_ENTRY_SPACE_BASE);

	for(i=0;i<QUEUE_NUM_DL_ENTRY;i++){
		dl_entry_base[i].data = (void*)(QUEUE_BUFFER_SPACE + (i*QUEUE_BUFFER_SIZE));
		dl_entry_insertEnd(&queue_free,&(dl_entry_base[i]));
	}

	num_queue_tx = 0;
	queue_tx = NULL;

	return QUEUE_NUM_DL_ENTRY*QUEUE_BUFFER_SIZE;

}

int queue_total_size(){
	return QUEUE_NUM_DL_ENTRY;
}

void purge_queue(u16 queue_sel){
	u32             num_queued;
	u32			    i;
	tx_queue_element* curr_tx_queue_element;

	// The queue purge is not interrupt safe
	wlan_mac_high_interrupt_stop();

	num_queued = queue_num_queued(queue_sel);

	if( num_queued > 0 ){
		xil_printf("purging %d packets from queue for queue ID %d\n", num_queued, queue_sel);

		for(i = 0; i < num_queued; i++){
			curr_tx_queue_element = dequeue_from_head(queue_sel);
			queue_checkin(curr_tx_queue_element);
		}
	}

	// Re-enable interrupts now that we are done
	wlan_mac_high_interrupt_start();
}

void enqueue_after_tail(u16 queue_sel, tx_queue_element* tqe){
	u32 i;

	dl_entry* curr_dl_entry = (dl_entry*)tqe;

    if((queue_sel+1) > num_queue_tx){
    	queue_tx = wlan_mac_high_realloc(queue_tx, (queue_sel+1)*sizeof(dl_list));

    	if(queue_tx == NULL){
    		xil_printf("Error in reallocating %d bytes for tx queue\n", (queue_sel+1)*sizeof(dl_list));
    	}

    	for(i = num_queue_tx; i <= queue_sel; i++){
    		dl_list_init(&(queue_tx[i]));
    	}

    	num_queue_tx = queue_sel+1;

    }

	dl_entry_insertEnd(&(queue_tx[queue_sel]),curr_dl_entry);

	return;
}

tx_queue_element* dequeue_from_head(u16 queue_sel){
	tx_queue_element* tqe;
	dl_entry* curr_dl_entry;

	if((queue_sel+1) > num_queue_tx){
		//The calling function is asking to empty from a queue_tx element that is
		//outside of the bounds that are currently allocated. This is not an error
		//condition, as it can happen when an AP checks to see if any packets are
		//ready to send to a station before any have ever been queued up for that
		//station. In this case, we simply return the newly initialized new_list
		//that says that no packets are currently in the queue_sel queue_tx.
		return NULL;
	} else {
		if(queue_tx[queue_sel].length == 0){
			return NULL;
		} else {
			curr_dl_entry = (queue_tx[queue_sel].first);
			dl_entry_remove(&queue_tx[queue_sel],curr_dl_entry);
			tqe = (tx_queue_element*)curr_dl_entry;
			return tqe;
		}
	}
}

u32 queue_num_free(){
	return queue_free.length;
}

u32 queue_num_queued(u16 queue_sel){
	if((queue_sel+1) > num_queue_tx){
		return 0;
	} else {
		return queue_tx[queue_sel].length;
	}
}

tx_queue_element* queue_checkout(){
	tx_queue_element* tqe;

	if(queue_free.length > 0){
		tqe = ((tx_queue_element*)(queue_free.first));
		dl_entry_remove(&queue_free,queue_free.first);
		((tx_queue_buffer*)(tqe->data))->metadata.metadata_type = QUEUE_METADATA_TYPE_IGNORE;
		return tqe;
	} else {
		return NULL;
	}
}

void queue_checkin(tx_queue_element* tqe){
	dl_entry_insertEnd(&queue_free,(dl_entry*)tqe);
	return;
}

void queue_checkout_list(dl_list* new_list, u16 num_tqe){
	//Checks out up to num_packet_bd number of packet_bds from the free list. If num_packet_bd are not free,
	//then this function will return the number that are free and only check out that many.

	u32 i,num_checkout;
	dl_entry* curr_dl_entry;

	dl_list_init(new_list);

	if(num_tqe <= queue_free.length){
		num_checkout = num_tqe;
	} else {
		num_checkout = queue_free.length;
	}

	//Traverse the queue_free and update the pointers
	for (i=0;i<num_checkout;i++){
		curr_dl_entry = (queue_free.first);

		//Remove from free list
		dl_entry_remove(&queue_free,curr_dl_entry);
		//Add to new checkout list
		dl_entry_insertEnd(new_list,curr_dl_entry);
	}
	return;
}



inline int dequeue_transmit_checkin(u16 queue_sel){
	int return_value = 0;

	tx_queue_element* curr_tx_queue_element;

	if(wlan_mac_high_is_ready_for_tx()){

		curr_tx_queue_element = dequeue_from_head(queue_sel);

		if(curr_tx_queue_element != NULL){
			return_value = 1;
			wlan_mac_high_mpdu_transmit(curr_tx_queue_element);
			queue_checkin(curr_tx_queue_element);
			wlan_eth_dma_update();
		}
	}
	return return_value;
}
