/** @file wlan_mac_ltg.h
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
 */

#ifndef WLAN_MAC_LTG_H_
#define WLAN_MAC_LTG_H_

#include "wlan_mac_dl_list.h"

//LTG Schedules define the times when LTG event callbacks are called.
#define LTG_SCHED_TYPE_PERIODIC			1
#define LTG_SCHED_TYPE_UNIFORM_RAND	 	2

//LTG Payloads define how payloads are constructed once the LTG event callbacks
//are called. For example, the LTG_SCHED_TYPE_PERIODIC schedule that employs the
//LTG_PYLD_TYPE_FIXED would result in a constant bit rate (CBR) traffic
//profile
#define LTG_PYLD_TYPE_FIXED				1
#define LTG_PYLD_TYPE_UNIFORM_RAND		2
#define LTG_PYLD_TYPE_ALL_ASSOC_FIXED	3


#define LTG_REMOVE_ALL                  0xFFFFFFFF
#define LTG_START_ALL                   0xFFFFFFFF
#define LTG_STOP_ALL                    0xFFFFFFFF


//In spirit, tg_schedule is derived from dl_entry. Since C
//lacks a formal notion of inheritance, we adopt a popular
//alternative idiom for inheritance where the dl_entry
//is the first entry in the new structure. Since structures
//will never be padded before their first entry, it is safe
//to cast back and forth between the tg_schedule and dl_entry.
typedef struct tg_schedule tg_schedule;
struct tg_schedule{
	u32 id;
	u32 type;
	u64 target;
	u64	stop_target;
	void* params;
	void* callback_arg;
	function_ptr_t cleanup_callback;
	void* state;
};

//LTG Schedules

#define LTG_DURATION_FOREVER 0

typedef struct {
	u8 enabled;
	u8 reserved[3];
} ltg_sched_state_hdr;

typedef struct {
	u32 interval_count;
	u64 duration_count;
} ltg_sched_periodic_params;

typedef struct {
	ltg_sched_state_hdr hdr;
	u32 time_to_next_count;
} ltg_sched_periodic_state;

typedef struct {
	u32 min_interval_count;
	u32 max_interval_count;
	u64 duration_count;
} ltg_sched_uniform_rand_params;

typedef struct {
	ltg_sched_state_hdr hdr;
	u32 time_to_next_count;
} ltg_sched_uniform_rand_state;

//LTG Payload Profiles

typedef struct {
	u32 type;
} ltg_pyld_hdr;

typedef struct {
	ltg_pyld_hdr hdr;
	u8 addr_da[6];
	u16 length;
} ltg_pyld_fixed;

typedef struct {
	ltg_pyld_hdr hdr;
	u16 length;
	u16 padding;
} ltg_pyld_all_assoc_fixed;

typedef struct {
	ltg_pyld_hdr hdr;
	u8 addr_da[6];
	u16 min_length;
	u16 max_length;
	u16 padding;
} ltg_pyld_uniform_rand;


//Note: This definition simply reflects the use of the fast timer for LTG polling. To increase LTG
//polling rate at the cost of more overhead in checking LTGs, increase the speed of the fast timer.
#define LTG_POLL_INTERVAL              FAST_TIMER_DUR_US

#define LTG_ID_INVALID	               0xFFFFFFFF

//External function to LTG -- user code interacts with the LTG via these functions
int wlan_mac_ltg_sched_init();
void wlan_mac_ltg_sched_set_callback(void(*callback)());
u32 ltg_sched_create(u32 type, void* params, void* callback_arg, void(*callback)());
int ltg_sched_remove(u32 id);
int ltg_sched_remove_all();
int ltg_sched_start(u32 id);
int ltg_sched_start_all();
int ltg_sched_stop(u32 id);
int ltg_sched_stop_all();
int ltg_sched_start_l(dl_entry* curr_tg_dl_entry);
int ltg_sched_stop_l(dl_entry* curr_tg_dl_entry);
int ltg_sched_get_state(u32 id, u32* type, void** state);
int ltg_sched_get_params(u32 id, void** params);
int ltg_sched_get_callback_arg(u32 id, void** callback_arg);

// WLAN Exp function to LTG -- users may call these directly or modify if needed
void * ltg_sched_deserialize(u32 * src, u32 * ret_type, u32 * ret_size);
void * ltg_payload_deserialize(u32 * src, u32 * ret_type, u32 * ret_size);

//Internal functions to LTG -- users should not need to call these directly
void ltg_sched_check();
dl_entry* ltg_sched_create_l();
void ltg_sched_destroy_l(dl_entry* tg_dl_entry);
void ltg_sched_destroy_params(tg_schedule *tg);
dl_entry* ltg_sched_find_tg_schedule(u32 id);

#endif /* WLAN_MAC_LTG_H_ */
