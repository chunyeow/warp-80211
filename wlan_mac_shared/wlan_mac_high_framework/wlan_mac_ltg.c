/** @file wlan_mac_ltg.c
 *  @brief Local Traffic Generator
 *
 *  This contains code for scheduling local traffic directly from the
 *  board.
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
#include "wlan_mac_dl_list.h"

#include "wlan_exp_common.h"

#include "wlan_mac_ipc_util.h"
#include "wlan_mac_high.h"
#include "wlan_mac_ltg.h"
#include "wlan_mac_schedule.h"

static dl_list tg_list;

static function_ptr_t ltg_callback;

static u64 num_ltg_checks;
static u32 schedule_id;
static u8  schedule_running;

int wlan_mac_ltg_sched_init(){

	int return_value = 0;
	schedule_running = 0;
	num_ltg_checks = 0;
	ltg_sched_remove(LTG_REMOVE_ALL);
	dl_list_init(&tg_list);
	ltg_callback = (function_ptr_t)nullCallback;

	return return_value;
}

void wlan_mac_ltg_sched_set_callback(void(*callback)()){
	ltg_callback = (function_ptr_t)callback;
}

u32 ltg_sched_create(u32 type, void* params, void* callback_arg, void(*cleanup_callback)()){

	static u32 id = 0;
	u32 return_value;

	tg_schedule* curr_tg;
	dl_entry*	 curr_tg_dl_entry;

	//Create a new tg for this id
	curr_tg_dl_entry = ltg_sched_create_l();

	if(curr_tg_dl_entry == NULL){
		return_value = LTG_ID_INVALID;
		return return_value;
	}

	dl_entry_insertEnd(&tg_list,curr_tg_dl_entry);

	curr_tg = (tg_schedule*)(curr_tg_dl_entry->data);

	curr_tg->id = id;

	return_value = id;

	id++;
	if(id == LTG_ID_INVALID){
		id++;
	}

	curr_tg->type = type;
	curr_tg->cleanup_callback = (function_ptr_t)cleanup_callback;
	switch(type){
		case LTG_SCHED_TYPE_PERIODIC:
			curr_tg->params = wlan_mac_high_malloc(sizeof(ltg_sched_periodic_params));
			curr_tg->state = wlan_mac_high_malloc(sizeof(ltg_sched_periodic_state));

			if(curr_tg->params != NULL && curr_tg->state != NULL){
				memcpy(curr_tg->params, params, sizeof(ltg_sched_periodic_params));

				curr_tg->callback_arg = callback_arg;
			} else {
				xil_printf("Failed to initialize LTG structs\n");
				dl_entry_remove(&tg_list,curr_tg_dl_entry);
				ltg_sched_destroy_l(curr_tg_dl_entry);
				return -1;
			}
		break;
		case LTG_SCHED_TYPE_UNIFORM_RAND:
			curr_tg->params = wlan_mac_high_malloc(sizeof(ltg_sched_uniform_rand_params));
			curr_tg->state = wlan_mac_high_malloc(sizeof(ltg_sched_uniform_rand_params));

			if(curr_tg->params != NULL && curr_tg->state != NULL){
				memcpy(curr_tg->params, params, sizeof(ltg_sched_uniform_rand_params));
				curr_tg->callback_arg = callback_arg;
			} else {
				xil_printf("Failed to initialize LTG structs\n");
				dl_entry_remove(&tg_list,curr_tg_dl_entry);
				ltg_sched_destroy_l(curr_tg_dl_entry);
				return_value = LTG_ID_INVALID;
				return return_value;
			}
		break;
		default:
			xil_printf("Unknown type %d, destroying tg_schedule struct\n");
			dl_entry_remove(&tg_list,curr_tg_dl_entry);
			ltg_sched_destroy_l(curr_tg_dl_entry);
			return_value = LTG_ID_INVALID;
			return return_value;
		break;
	}
	((ltg_sched_state_hdr*)(curr_tg->state))->enabled = 0;
	return return_value;
}

int ltg_sched_start(u32 id){
	dl_entry*	curr_tg_dl_entry;

	if (id == LTG_START_ALL) {
		return ltg_sched_start_all();
	} else {
        // Single ID case
		curr_tg_dl_entry = ltg_sched_find_tg_schedule(id);

		if(curr_tg_dl_entry != NULL){
			return ltg_sched_start_l(curr_tg_dl_entry);
		} else {
			xil_printf("Failed to start LTG ID: %d. Please ensure LTG is configured before starting\n", id);
			return -1;
		}
	}
}


int ltg_sched_start_all(){
	u32 i;
	u32 list_len;
	int ret_val = 0;
	tg_schedule* curr_tg;
	dl_entry* next_tg_dl_entry;
	dl_entry* curr_tg_dl_entry;

	list_len         = tg_list.length;
	next_tg_dl_entry = tg_list.first;

	wlan_mac_high_interrupt_stop();

	for(i = 0; i < list_len; i++ ){
		curr_tg_dl_entry = next_tg_dl_entry;
		next_tg_dl_entry = dl_entry_next(next_tg_dl_entry);

		curr_tg = (tg_schedule*)(curr_tg_dl_entry->data);

		if(ltg_sched_start_l(curr_tg_dl_entry) == -1) {
			xil_printf("Failed to start LTG ID: %d. Please ensure LTG is configured before starting\n", (curr_tg->id));
			ret_val = -1;
		}
	}

	wlan_mac_high_interrupt_start();

	return ret_val;
}


int ltg_sched_start_l(dl_entry* curr_tg_dl_entry){
	tg_schedule* curr_tg = (tg_schedule*)(curr_tg_dl_entry->data);
	u64 random_timestamp;

	switch(curr_tg->type){
		case LTG_SCHED_TYPE_PERIODIC:

			curr_tg->target = num_ltg_checks + (((ltg_sched_periodic_params*)(curr_tg->params))->interval_count);

			if(((ltg_sched_periodic_params*)(curr_tg->params))->duration_count != LTG_DURATION_FOREVER){
				curr_tg->stop_target = num_ltg_checks + ((ltg_sched_periodic_params*)(curr_tg->params))->duration_count;
			} else {
				curr_tg->stop_target = LTG_DURATION_FOREVER;
			}

			((ltg_sched_state_hdr*)(curr_tg->state))->enabled = 1;
		break;
		case LTG_SCHED_TYPE_UNIFORM_RAND:
			random_timestamp = (rand()%(((ltg_sched_uniform_rand_params*)(curr_tg->params))->max_interval_count - ((ltg_sched_uniform_rand_params*)(curr_tg->params))->min_interval_count))+((ltg_sched_uniform_rand_params*)(curr_tg->params))->min_interval_count;
			curr_tg->target = num_ltg_checks + (random_timestamp/FAST_TIMER_DUR_US);

			if(((ltg_sched_uniform_rand_params*)(curr_tg->params))->duration_count != LTG_DURATION_FOREVER){
				curr_tg->stop_target = num_ltg_checks + ((ltg_sched_periodic_params*)(curr_tg->params))->duration_count;
			} else {
				curr_tg->stop_target = LTG_DURATION_FOREVER;
			}

			((ltg_sched_state_hdr*)(curr_tg->state))->enabled = 1;
		break;

		default:
			xil_printf("Unknown type %d, destroying tg_schedule struct\n");
			dl_entry_remove(&tg_list,curr_tg_dl_entry);
			ltg_sched_destroy_l(curr_tg_dl_entry);
			return -1;
		break;
	}

	if(schedule_running == 0){
		schedule_running = 1;

		schedule_id = wlan_mac_schedule_event_repeated(SCHEDULE_FINE, 0, SCHEDULE_REPEAT_FOREVER, (void*)ltg_sched_check);
	}

	return 0;
}


void ltg_sched_check(){
	tg_schedule* curr_tg;
	dl_entry*	 curr_tg_dl_entry;
	u64 		 random_timestamp;

	u32 i;

	num_ltg_checks++;
	if(tg_list.length > 0){

		curr_tg_dl_entry = tg_list.first;
		for( i = 0; i < tg_list.length; i++ ){
			curr_tg = (tg_schedule*)(curr_tg_dl_entry->data);

			if(((ltg_sched_state_hdr*)(curr_tg->state))->enabled){

				if( num_ltg_checks >= ( curr_tg->target ) ){
					switch(curr_tg->type){
						case LTG_SCHED_TYPE_PERIODIC:
							curr_tg->target = num_ltg_checks + (((ltg_sched_periodic_params*)(curr_tg->params))->interval_count);
						break;
						case LTG_SCHED_TYPE_UNIFORM_RAND:
							random_timestamp = (rand()%(((ltg_sched_uniform_rand_params*)(curr_tg->params))->max_interval_count - ((ltg_sched_uniform_rand_params*)(curr_tg->params))->min_interval_count))+((ltg_sched_uniform_rand_params*)(curr_tg->params))->min_interval_count;
							curr_tg->target = num_ltg_checks + (random_timestamp/FAST_TIMER_DUR_US);
						break;
						default:
							ltg_sched_stop_l(curr_tg_dl_entry);
							return;
						break;
					}
					ltg_callback(curr_tg->id, curr_tg->callback_arg);
				}

				if( curr_tg->stop_target != LTG_DURATION_FOREVER && num_ltg_checks >= ( curr_tg->stop_target )){
					ltg_sched_stop_l(curr_tg_dl_entry);
				}

			}

			curr_tg_dl_entry = dl_entry_next(curr_tg_dl_entry);
		}
	}

	return;
}

int ltg_sched_stop(u32 id){
	dl_entry*	 curr_tg_dl_entry;

	if (id == LTG_STOP_ALL) {
		return ltg_sched_stop_all();
	} else {
        // Single ID case
		curr_tg_dl_entry = ltg_sched_find_tg_schedule(id);

		if(curr_tg_dl_entry != NULL){
			return ltg_sched_stop_l(curr_tg_dl_entry);
		} else {
			xil_printf("Failed to stop LTG ID: %d. Please ensure LTG is configured before stopping\n", id);
			return -1;
		}
	}
}


int ltg_sched_stop_all(){
	u32 i;
	u32 list_len;
	dl_entry*    next_tg_dl_entry;
	dl_entry*    curr_tg_dl_entry;

	list_len         = tg_list.length;
	next_tg_dl_entry = tg_list.first;

	wlan_mac_high_interrupt_stop();

	for(i = 0; i < list_len; i++ ){
		curr_tg_dl_entry = next_tg_dl_entry;
		next_tg_dl_entry = dl_entry_next(curr_tg_dl_entry);
		ltg_sched_stop_l(curr_tg_dl_entry);
	}

	wlan_mac_high_interrupt_start();

	return 0;
}


int ltg_sched_stop_l(dl_entry* curr_tg_dl_entry){
	tg_schedule* curr_tg = (tg_schedule*)(curr_tg_dl_entry->data);

	((ltg_sched_state_hdr*)(curr_tg->state))->enabled = 0;
	if(tg_list.length == 0 && schedule_running == 1){
		wlan_mac_remove_schedule(SCHEDULE_FINE, schedule_id);
		schedule_running = 0;
	}
	return 0;
}

int ltg_sched_get_state(u32 id, u32* type, void** state){
	//This function returns the type of schedule corresponding to the id argument
	//It fills in the state argument with the state of the schedule

	tg_schedule* curr_tg;
	dl_entry*	 curr_tg_dl_entry;

	curr_tg_dl_entry = ltg_sched_find_tg_schedule(id);
	if(curr_tg_dl_entry == NULL){
		return -1;
	}

	curr_tg = (tg_schedule*)(curr_tg_dl_entry->data);

	if(type != NULL) *type = curr_tg->type;
	if(state != NULL) *state = curr_tg->state;

	switch(curr_tg->type){
		case LTG_SCHED_TYPE_PERIODIC:
			if(num_ltg_checks < (curr_tg->target) ){
				((ltg_sched_periodic_state*)(curr_tg->state))->time_to_next_count = (u32)(curr_tg->target - num_ltg_checks);
			} else {
				((ltg_sched_periodic_state*)(curr_tg->state))->time_to_next_count = 0;
			}
		break;

		case LTG_SCHED_TYPE_UNIFORM_RAND:
			if(num_ltg_checks < (curr_tg->target) ){
				((ltg_sched_uniform_rand_state*)(curr_tg->state))->time_to_next_count = (u32)(curr_tg->target - num_ltg_checks);
			} else {
				((ltg_sched_uniform_rand_state*)(curr_tg->state))->time_to_next_count = 0;
			}
		break;
		default:
			xil_printf("Unknown type %d\n", curr_tg->type);
			return -1;
		break;
	}


	return 0;

}
int ltg_sched_get_params(u32 id, void** params){
	//This function returns the type of the schedule corresponding to the id argument
	//It fills in the current parameters of the schedule into the params argument
	tg_schedule* curr_tg;
	dl_entry*	 curr_tg_dl_entry;

	curr_tg_dl_entry = ltg_sched_find_tg_schedule(id);
	if(curr_tg_dl_entry == NULL){
		return -1;
	}

	curr_tg = (tg_schedule*)(curr_tg_dl_entry->data);

	*params = curr_tg->params;

	return 0;
}

int ltg_sched_get_callback_arg(u32 id, void** callback_arg){
	tg_schedule* curr_tg;
	dl_entry*	 curr_tg_dl_entry;

	curr_tg_dl_entry = ltg_sched_find_tg_schedule(id);
	if(curr_tg_dl_entry == NULL){
		return -1;
	}

	curr_tg = (tg_schedule*)(curr_tg_dl_entry->data);

	*callback_arg = curr_tg->callback_arg;

	return 0;
}


int ltg_sched_remove(u32 id){
	tg_schedule* curr_tg;
	dl_entry*	 curr_tg_dl_entry;

	if (id == LTG_REMOVE_ALL) {
		return ltg_sched_remove_all();
	} else {
        // Single ID case
		curr_tg_dl_entry = ltg_sched_find_tg_schedule(id);

		if(curr_tg_dl_entry != NULL){
			curr_tg = (tg_schedule*)(curr_tg_dl_entry->data);

			ltg_sched_stop_l(curr_tg_dl_entry);
			dl_entry_remove(&tg_list, curr_tg_dl_entry);
			if(curr_tg->cleanup_callback != NULL){
				curr_tg->cleanup_callback(curr_tg->id, curr_tg->callback_arg);
			}
			ltg_sched_destroy_l(curr_tg_dl_entry);

			return 0;
		} else {
			xil_printf("Failed to remove LTG ID: %d. Please ensure LTG is configured before removing\n", id);
			return -1;
		}
	}
}


int ltg_sched_remove_all(){
	tg_schedule* curr_tg;
	dl_entry* 	 next_tg_dl_entry;
	dl_entry* 	 curr_tg_dl_entry;

	next_tg_dl_entry = tg_list.first;

	wlan_mac_high_interrupt_stop();

	// NOTE:  Cannot use a for loop for this iteration b/c we are removing
	//   elements from the list.
	while(next_tg_dl_entry != NULL){
		curr_tg_dl_entry = next_tg_dl_entry;
		next_tg_dl_entry = dl_entry_next(curr_tg_dl_entry);

		curr_tg = (tg_schedule*)(curr_tg_dl_entry->data);

		ltg_sched_stop_l(curr_tg_dl_entry);
		dl_entry_remove(&tg_list, curr_tg_dl_entry);
		if(curr_tg->cleanup_callback != NULL){
			curr_tg->cleanup_callback(curr_tg->id, curr_tg->callback_arg);
		}
		ltg_sched_destroy_l(curr_tg_dl_entry);
	}

	wlan_mac_high_interrupt_start();

	return 0;
}


dl_entry* ltg_sched_create_l(){
	dl_entry* curr_tg_dl_entry;
	tg_schedule* curr_tg;

	curr_tg_dl_entry = (dl_entry*)wlan_mac_high_malloc(sizeof(dl_entry));

	if(curr_tg_dl_entry == NULL){
		return NULL;
	}

	curr_tg = (tg_schedule*)wlan_mac_high_malloc(sizeof(tg_schedule));

	if(curr_tg == NULL){
		wlan_mac_high_free(curr_tg_dl_entry);
		return NULL;
	}

	curr_tg_dl_entry->data = (void*)curr_tg;

	return curr_tg_dl_entry;
}

void ltg_sched_destroy_params(tg_schedule *tg){
	switch(tg->type){
		case LTG_SCHED_TYPE_PERIODIC:
		case LTG_SCHED_TYPE_UNIFORM_RAND:
			wlan_mac_high_free(tg->params);
			wlan_mac_high_free(tg->state);
		break;
	}
}

void ltg_sched_destroy_l(dl_entry* tg_dl_entry){
	tg_schedule* curr_tg;

	curr_tg = (tg_schedule*)(tg_dl_entry->data);

	ltg_sched_destroy_params(curr_tg);
	wlan_mac_high_free(tg_dl_entry);
	wlan_mac_high_free(curr_tg);
	return;
}

dl_entry* ltg_sched_find_tg_schedule(u32 id){
	u32 i;
	dl_entry*	 curr_tg_dl_entry;
	tg_schedule* curr_tg;

	curr_tg_dl_entry = tg_list.first;
	for(i = 0; i < tg_list.length; i++ ){
		curr_tg = (tg_schedule*)(curr_tg_dl_entry->data);
		if( (curr_tg->id)==id){
			return curr_tg_dl_entry;
		}
		curr_tg_dl_entry = dl_entry_next(curr_tg_dl_entry);
	}
	return NULL;
}


// NOTE:  The src information is from the network and must be byte swapped
void * ltg_sched_deserialize(u32 * src, u32 * ret_type, u32 * ret_size) {
	u32    temp, temp2;
    u16    type;
    u16    size;

    void * ret_val = NULL;

    temp  = Xil_Ntohl(src[0]);
    type  = (temp >> 16) & 0xFFFF;
    size  = (temp & 0xFFFF);

    xil_printf("LTG Sched:  type = %d, size = %d\n", type, size);

    switch(type){
        case LTG_SCHED_TYPE_PERIODIC:
        	if (size == 3){
        		ret_val = (void *) wlan_mac_high_malloc(sizeof(ltg_sched_periodic_params));
        	    if (ret_val != NULL){
        	    	((ltg_sched_periodic_params *)ret_val)->interval_count = (Xil_Ntohl(src[1]))/LTG_POLL_INTERVAL;

        	    	temp     = Xil_Ntohl(src[2]);
        	    	temp2    = Xil_Ntohl(src[3]);
        	    	((ltg_sched_periodic_params *)ret_val)->duration_count = ((((u64)temp)<<32) + ((u64)temp2))/LTG_POLL_INTERVAL;

        	    	xil_printf("LTG Sched Periodic: %d usec for %d usec\n",
        	    			       LTG_POLL_INTERVAL * ((ltg_sched_periodic_params *)ret_val)->interval_count,
        	    			       (u32)(LTG_POLL_INTERVAL * (((ltg_sched_periodic_params *)ret_val)->duration_count)));
        	    }
        	}
    	break;

        case LTG_SCHED_TYPE_UNIFORM_RAND:
        	if (size == 4){
        		ret_val = (void *) wlan_mac_high_malloc(sizeof(ltg_sched_uniform_rand_params));
        	    if (ret_val != NULL){
        	    	((ltg_sched_uniform_rand_params *)ret_val)->min_interval_count = Xil_Ntohl(src[1])/LTG_POLL_INTERVAL;
        	    	((ltg_sched_uniform_rand_params *)ret_val)->max_interval_count = Xil_Ntohl(src[2])/LTG_POLL_INTERVAL;

        	    	temp     = Xil_Ntohl(src[3]);
        	    	temp2    = Xil_Ntohl(src[4]);
        	    	((ltg_sched_uniform_rand_params *)ret_val)->duration_count = ((((u64)temp)<<32) + ((u64)temp2))/LTG_POLL_INTERVAL;

        	    	xil_printf("LTG Sched Uniform Rand: [%d %d] usec for %d usec\n",
        	    			       LTG_POLL_INTERVAL * ((ltg_sched_uniform_rand_params *)ret_val)->min_interval_count,
        	    			       LTG_POLL_INTERVAL * ((ltg_sched_uniform_rand_params *)ret_val)->max_interval_count,
 			                       (u32)(LTG_POLL_INTERVAL * (((ltg_sched_uniform_rand_params *)ret_val)->duration_count)));
        	    }
        	}
        break;
    }

    // Set return values
    *ret_type = type;
    *ret_size = size;
	return ret_val;
}


// NOTE:  The src information is from the network and must be byte swapped
void * ltg_payload_deserialize(u32 * src, u32 * ret_type, u32 * ret_size) {
	u32    temp;
    u16    type;
    u16    size;
    void * ret_val = NULL;

    temp  = Xil_Ntohl(src[0]);
    type  = (temp >> 16) & 0xFFFF;
    size  = (temp & 0xFFFF);


    xil_printf("LTG Payload:  type = %d, size = %d\n", type, size);

    switch(type){
        case LTG_PYLD_TYPE_FIXED:
        	if (size == 3){
        		ret_val = (void *) wlan_mac_high_malloc(sizeof(ltg_pyld_fixed));
        	    if (ret_val != NULL){
					((ltg_pyld_fixed *)ret_val)->hdr.type = LTG_PYLD_TYPE_FIXED;
					wlan_exp_get_mac_addr(&src[1], &((ltg_pyld_fixed *)ret_val)->addr_da[0]);
        	    	((ltg_pyld_fixed *)ret_val)->length   = Xil_Ntohl(src[3]) & 0xFFFF;

        	    	xil_printf("LTG Payload Fixed: %d bytes\n", ((ltg_pyld_fixed *)ret_val)->length);
        	    }
        	}
    	break;

        case LTG_PYLD_TYPE_UNIFORM_RAND:
        	if (size == 4){
        		ret_val = (void *) wlan_mac_high_malloc(sizeof(ltg_pyld_uniform_rand));
        	    if (ret_val != NULL){
					((ltg_pyld_uniform_rand *)ret_val)->hdr.type   = LTG_PYLD_TYPE_UNIFORM_RAND;
					wlan_exp_get_mac_addr(&src[1], &((ltg_pyld_fixed *)ret_val)->addr_da[0]);
        	    	((ltg_pyld_uniform_rand *)ret_val)->min_length = Xil_Ntohl(src[3]) & 0xFFFF;
        	    	((ltg_pyld_uniform_rand *)ret_val)->max_length = Xil_Ntohl(src[4]) & 0xFFFF;

        	    	xil_printf("LTG Payload Uniform Rand: [%d %d] bytes\n", ((ltg_pyld_uniform_rand *)ret_val)->min_length, ((ltg_pyld_uniform_rand *)ret_val)->max_length);
        	    }
        	}
        break;

        case LTG_PYLD_TYPE_ALL_ASSOC_FIXED:
        	if (size == 1){
        		ret_val = (void *) wlan_mac_high_malloc(sizeof(ltg_pyld_all_assoc_fixed));
        	    if (ret_val != NULL){
					((ltg_pyld_all_assoc_fixed *)ret_val)->hdr.type = LTG_PYLD_TYPE_ALL_ASSOC_FIXED;
        	    	((ltg_pyld_all_assoc_fixed *)ret_val)->length   = Xil_Ntohl(src[1]) & 0xFFFF;

        	    	xil_printf("LTG Payload All Assoc Fixed: %d bytes\n", ((ltg_pyld_all_assoc_fixed *)ret_val)->length);
        	    }
        	}
        break;
    }

    // Set return values
    *ret_type = type;
    *ret_size = size;
	return ret_val;
}






