/** @file wlan_mac_schedule.h
 *  @brief Scheduler
 *
 *  This set of functions allows upper-level MAC implementations
 *	to schedule the execution of a provided callback for some point
 *	in the future.
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


#ifndef WLAN_MAC_SCHEDULE_H_
#define WLAN_MAC_SCHEDULE_H_

#include "xintc.h"
#include "wlan_mac_dl_list.h"

//In spirit, wlan_sched is derived from dl_entry. Since C
//lacks a formal notion of inheritance, we adopt a popular
//alternative idiom for inheritance where the dl_entry
//is the first entry in the new structure. Since structures
//will never be padded before their first entry, it is safe
//to cast back and forth between the wlan_sched and dl_entry.
typedef struct {
	u32 id;
	u32 delay;
	u32 num_calls;
	u64 target;
	function_ptr_t callback;
} wlan_sched;

//Special value for num_calls parameter of wlan_sched
#define SCHEDULE_REPEAT_FOREVER 0xFFFFFFFF

#define SCHEDULE_FINE	0
#define SCHEDULE_COARSE 1

#define TMRCTR_DEVICE_ID	XPAR_TMRCTR_0_DEVICE_ID

#define TIMER_FREQ          XPAR_TMRCTR_0_CLOCK_FREQ_HZ
#define TIMER_CNTR_FAST	 0
#define TIMER_CNTR_SLOW	 1

#define	FAST_TIMER_DUR_US 100
#define	SLOW_TIMER_DUR_US 100000

//Return value from wlan_mac_schedule_event
#define SCHEDULE_FAILURE 0xFFFFFFFF


int wlan_mac_schedule_init();
int wlan_mac_schedule_setup_interrupt(XIntc* intc);

#define CALL_FOREVER 0xFFFFFF
#define wlan_mac_schedule_event(scheduler_sel,delay,callback) wlan_mac_schedule_event_repeated(scheduler_sel,delay,1,callback)
u32 wlan_mac_schedule_event_repeated(u8 scheduler_sel, u32 delay, u32 num_calls, void(*callback)());
void wlan_mac_remove_schedule(u8 scheduler_sel, u32 id);

dl_entry* find_schedule(u8 scheduler_sel, u32 id);
void timer_handler(void *CallBackRef, u8 TmrCtrNumber);
void XTmrCtr_CustomInterruptHandler(void *InstancePtr);

#endif /* WLAN_MAC_SCHEDULE_H_ */
