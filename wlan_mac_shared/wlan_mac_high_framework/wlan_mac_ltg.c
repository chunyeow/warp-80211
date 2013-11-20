////////////////////////////////////////////////////////////////////////////////
// File   : wlan_mac_ltg.c
// Authors: Patrick Murphy (murphpo [at] mangocomm.com)
//			Chris Hunter (chunter [at] mangocomm.com)
// License: Copyright 2013, Mango Communications. All rights reserved.
//          Distributed under the Mango Communications Reference Design License
//				See LICENSE.txt included in the design archive or
//				at http://mangocomm.com/802.11/license
////////////////////////////////////////////////////////////////////////////////

#include "xil_types.h"
#include "stdlib.h"
#include "stdio.h"
#include "xparameters.h"
#include "xintc.h"
#include "string.h"

#include "wlan_mac_ipc_util.h"
#include "wlan_mac_util.h"
#include "wlan_mac_ltg.h"

static traffic_generator_list tg_list;

static function_ptr_t ltg_callback;

int wlan_mac_ltg_init(){

	int return_value = 0;

	traffic_generator_list_init(&tg_list);
	ltg_callback = (function_ptr_t)nullCallback;

	wlan_mac_schedule_event(SCHEDULE_FINE, 0, (void*)check_ltg);

	return return_value;
}

void wlan_mac_ltg_set_callback(void(*callback)()){
	ltg_callback = (function_ptr_t)callback;
}


int start_ltg(u32 id, u32 type, void* params){
	u32 i;
	u64 timestamp = get_usec_timestamp();
	traffic_generator* curr_tg;
	traffic_generator* new_tg;

	int return_value = 0;

	//First, loop through the tg_list and see if this id event is already present.
	curr_tg = tg_list.first;
	for(i = 0; i < tg_list.length; i++ ){
		if( (curr_tg->id)==id){
			xil_printf("Error in LTG: Type %d is already started. Use stop_ltg\n", id);
			return -1;
		}
		curr_tg = curr_tg->next;
	}

	//Create a new tg for this id
	new_tg = create_ltg();
	if(new_tg != NULL){
		new_tg->id = id;
		new_tg->type = type;
		switch(type){
			case LTG_TYPE_CBR:
				new_tg->params = malloc(sizeof(cbr_params));
				if(new_tg->params != NULL){
					memcpy(new_tg->params, params, sizeof(cbr_params));
					new_tg->timestamp = timestamp;

					if(tg_list.length == 0){
						//start the scheduler if it isn't already running
						wlan_mac_schedule_event(SCHEDULE_FINE, 0, (void*)check_ltg);
					}

					traffic_generator_insertEnd(&tg_list,new_tg);
				} else {
					xil_printf("Failed to initialize parameter struct\n");
					destroy_ltg(new_tg);
					return -1;
				}

			break;
			default:
				xil_printf("Unknown type %d, destroying traffic_generator struct\n");
				destroy_ltg(new_tg);
				return -1;
			break;
		}


	} else {
		xil_printf("Failed to initialize traffic_generator struct\n");
		return -1;
	}

	return return_value;
}

void check_ltg(){
	u64 timestamp;
	traffic_generator* curr_tg;
	u32 i;

	if(tg_list.length > 0){
		timestamp = get_usec_timestamp();
		curr_tg = tg_list.first;
		for(i = 0; i < tg_list.length; i++ ){
			switch(curr_tg->type){
				case LTG_TYPE_CBR:
					if(timestamp >= (((cbr_params*)(curr_tg->params))->interval_usec  + curr_tg->timestamp) ){
						curr_tg->timestamp = timestamp;
						ltg_callback(curr_tg->id);
					}
				break;
			}

			curr_tg = curr_tg->next;
		}
		wlan_mac_schedule_event(SCHEDULE_FINE, 0, (void*)check_ltg);
	}


	return;
}

int stop_ltg(u32 id){

	u32 i;
	traffic_generator* curr_tg;

	curr_tg = tg_list.first;
	for(i = 0; i < tg_list.length; i++ ){
		if( (curr_tg->id)==id){
			traffic_generator_remove(&tg_list, curr_tg);
			destroy_ltg(curr_tg);
			return 0;
		}
		curr_tg = curr_tg->next;
	}


	xil_printf("Error: LTG ID %d not present\n", id);
	return -1;
}


traffic_generator* create_ltg(){
	return (traffic_generator*)malloc(sizeof(traffic_generator));
}

void destroy_ltg(traffic_generator* tg){

	switch(tg->type){
		case LTG_TYPE_CBR:
			free(tg->params);
		break;
	}

	free(tg);
	return;
}


void traffic_generator_insertAfter(traffic_generator_list* list, traffic_generator* tg, traffic_generator* tg_new){
	tg_new->prev = tg;
	tg_new->next = tg->next;
	if(tg->next == NULL){
		list->last = tg_new;
	} else {
		tg->next->prev = tg_new;
	}
	tg->next = tg_new;
	(list->length)++;
	return;
}

void traffic_generator_insertBefore(traffic_generator_list* list, traffic_generator* tg, traffic_generator* tg_new){
	tg_new->prev = tg->prev;
	tg_new->next = tg;
	if(tg->prev == NULL){
		list->first = tg_new;
	} else {
		tg->prev->next = tg_new;
	}
	tg->prev = tg_new;
	(list->length)++;
	return;
}

void traffic_generator_insertBeginning(traffic_generator_list* list, traffic_generator* tg_new){
	if(list->first == NULL){
		list->first = tg_new;
		list->last = tg_new;
		tg_new->prev = NULL;
		tg_new->next = NULL;
		(list->length)++;
	} else {
		traffic_generator_insertBefore(list, list->first, tg_new);
	}
	return;
}

void traffic_generator_insertEnd(traffic_generator_list* list, traffic_generator* tg_new){
	if(list->last == NULL){
		traffic_generator_insertBeginning(list,tg_new);
	} else {
		traffic_generator_insertAfter(list,list->last, tg_new);
	}
	return;
}

void traffic_generator_remove(traffic_generator_list* list, traffic_generator* tg){
	if(tg->prev == NULL){
		list->first = tg->next;
	} else {
		tg->prev->next = tg->next;
	}

	if(tg->next == NULL){
		list->last = tg->prev;
	} else {
		tg->next->prev = tg->prev;
	}
	(list->length)--;
}

void traffic_generator_list_init(traffic_generator_list* list){
	list->first = NULL;
	list->last = NULL;
	list->length = 0;
	return;
}
