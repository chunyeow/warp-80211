/** @file wlan_mac_dl_list.h
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
 */


#ifndef WLAN_MAC_DL_LIST_H_

#include "xil_types.h"

#define WLAN_MAC_DL_LIST_H_

typedef struct dl_entry dl_entry;
struct dl_entry{
	dl_entry* next;
	dl_entry* prev;
	void*     data;
};

//Helper macros for traversing the doubly-linked list
#define dl_entry_next(x) (((dl_entry*)x)->next)
#define dl_entry_prev(x) (((dl_entry*)x)->prev)

typedef struct {
	dl_entry* first;
	dl_entry* last;
	u16 length;
} dl_list;



void dl_entry_insertAfter(dl_list* list, dl_entry* entry, dl_entry* entry_new);
void dl_entry_insertBefore(dl_list* list, dl_entry* entry, dl_entry* entry_new);
void dl_entry_insertBeginning(dl_list* list, dl_entry* entry_new);
void dl_entry_insertEnd(dl_list* list, dl_entry* entry_new);
void dl_entry_remove(dl_list* list, dl_entry* entry);
void dl_list_init(dl_list* list);


#endif /* WLAN_MAC_DL_LIST_H_ */
