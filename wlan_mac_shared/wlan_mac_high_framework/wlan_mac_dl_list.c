/** @file wlan_mac_dl_list.c
 *  @brief Doubly-linked List Framework
 *
 *  This contains the code for managing doubly-linked lists.
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
#include "wlan_mac_dl_list.h"
#include "stdio.h"


/******************************** Functions **********************************/
void dl_entry_insertAfter(dl_list* list, dl_entry* entry, dl_entry* entry_new){
	dl_entry_prev(entry_new) = entry;
	dl_entry_next(entry_new) = dl_entry_next(entry);
	if(dl_entry_next(entry) == NULL){
		list->last = entry_new;
	} else {
		dl_entry_prev(dl_entry_next(entry)) = entry_new;
	}
	dl_entry_next(entry) = entry_new;
	(list->length)++;
	return;
}

void dl_entry_insertBefore(dl_list* list, dl_entry* entry, dl_entry* entry_new){
	dl_entry_prev(entry_new) = dl_entry_prev(entry);
	dl_entry_next(entry_new) = entry;
	if(dl_entry_prev(entry) == NULL){
		list->first = entry_new;
	} else {
		dl_entry_next(dl_entry_prev(entry)) = entry_new;
	}
	dl_entry_prev(entry) = entry_new;
	(list->length)++;
	return;
}

void dl_entry_insertBeginning(dl_list* list, dl_entry* entry_new){
	if(list->first == NULL){
		list->first = entry_new;
		list->last = entry_new;
		dl_entry_prev(entry_new) = NULL;
		dl_entry_next(entry_new) = NULL;
		(list->length)++;
	} else {
		dl_entry_insertBefore(list, list->first, entry_new);
	}
	return;
}

void dl_entry_insertEnd(dl_list* list, dl_entry* entry_new){
	if(list->last == NULL){
		dl_entry_insertBeginning(list,entry_new);
	} else {
		dl_entry_insertAfter(list,list->last, entry_new);
	}
	return;
}

void dl_entry_remove(dl_list* list, dl_entry* entry){
	if(list->length > 0){
		if(dl_entry_prev(entry) == NULL){
			list->first = dl_entry_next(entry);
		} else {
			dl_entry_next(dl_entry_prev(entry)) = dl_entry_next(entry);
		}

		if(dl_entry_next(entry) == NULL){
			list->last = dl_entry_prev(entry);
		} else {
			dl_entry_prev(dl_entry_next(entry)) = dl_entry_prev(entry);
		}
		(list->length)--;
	} else {
		xil_printf("Error: attempted to remove dl_entry from dl_list that has nothing in it\n");
	}
}

void dl_list_init(dl_list* list){
	list->first = NULL;
	list->last = NULL;
	list->length = 0;
	return;
}
