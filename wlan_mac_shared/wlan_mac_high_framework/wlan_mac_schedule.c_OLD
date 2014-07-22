/** @file wlan_mac_schedule.c
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
 *  @bug No known bugs.
 */

/***************************** Include Files *********************************/
#include "xparameters.h"
//#include "stdlib.h"
#include "xil_types.h"
#include "wlan_mac_high.h"
#include "wlan_mac_schedule.h"
#include "xtmrctr.h"
#include "xil_exception.h"
#include "xintc.h"

/*************************** Variable Definitions ****************************/
static XTmrCtr     TimerCounterInst;
static XIntc*      InterruptController_ptr;

static u32 schedule_count;

static u64 num_coarse_checks;
static u64 num_fine_checks;
static dl_list wlan_sched_coarse;
static dl_list wlan_sched_fine;

/******************************** Functions **********************************/

/*****************************************************************************/
/**
* Initializes the schedulers.
*
* @param    None
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
int wlan_mac_schedule_init(){
	int Status;

	//Initialize internal variables
	schedule_count = 0;
	num_coarse_checks = 0;
	num_fine_checks = 0;
	dl_list_init(&wlan_sched_coarse);
	dl_list_init(&wlan_sched_fine);

	//Set up the timer
	Status = XTmrCtr_Initialize(&TimerCounterInst, TMRCTR_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		xil_printf("XTmrCtr failed to initialize\n");
		return -1;
	}

	//Set the handler for Timer
	XTmrCtr_SetHandler(&TimerCounterInst, timer_handler, &TimerCounterInst);

	//Enable interrupt of timer and auto-reload so it continues repeatedly
	XTmrCtr_SetOptions(&TimerCounterInst, TIMER_CNTR_FAST, XTC_DOWN_COUNT_OPTION | XTC_INT_MODE_OPTION);
	XTmrCtr_SetOptions(&TimerCounterInst, TIMER_CNTR_SLOW, XTC_DOWN_COUNT_OPTION | XTC_INT_MODE_OPTION);


	return 0;
}

/*****************************************************************************/
/**
* Prints an event
*
* @param    intc             - pointer to instance of interrupt controller
*
* @return	status			 - success or failure of attempt to connect
*
* @note		None.
*
******************************************************************************/
int wlan_mac_schedule_setup_interrupt(XIntc* intc){
	//Connect Timer to Interrupt Controller
	int status;

	InterruptController_ptr = intc;
	status =  XIntc_Connect(InterruptController_ptr, TMRCTR_INTERRUPT_ID, (XInterruptHandler)XTmrCtr_CustomInterruptHandler, &TimerCounterInst);
	XIntc_Enable(InterruptController_ptr, TMRCTR_INTERRUPT_ID);
	return status;
}

/*****************************************************************************/
/**
* Schedules the execution of a callback for some time in the future
*
* @param    scheduler_sel    - SCHEDULE_COARSE or SCHEDULE_FINE
* 			delay			 - interval (in microseconds) until callback should be called
* 			num_calls		 - number of repetitions or CALL_FOREVER for permanent periodic
* 			callback		 - function pointer to callback
*
* @return	id  			 - ID of scheduled event or SCHEDULE_FAILURE if error
*
* @note		None.
*
******************************************************************************/
u32 wlan_mac_schedule_event_repeated(u8 scheduler_sel, u32 delay, u32 num_calls, void(*callback)()){
	u32 id;

	dl_entry*	entry_ptr = wlan_mac_high_malloc(sizeof(dl_entry));

	if(entry_ptr == NULL){
		//malloc has failed. Return failure condition
		return SCHEDULE_FAILURE;
	}

	wlan_sched* sched_ptr = wlan_mac_high_malloc(sizeof(wlan_sched));

	if(sched_ptr == NULL){
		//malloc has failed. Return failure condition
		wlan_mac_high_free(entry_ptr);
		return SCHEDULE_FAILURE;
	}

	//Attach the schedule struct to this dl_entry
	entry_ptr->data = sched_ptr;

	id = (schedule_count++);
	if(id == SCHEDULE_FAILURE) id = 0; //SCHEDULE_FAILURE is not a valid ID, so we wrap back to 0 and start again
	sched_ptr->id = id;
	sched_ptr->num_calls = num_calls;
	sched_ptr->callback = (function_ptr_t)callback;

	switch(scheduler_sel){
		case SCHEDULE_COARSE:
			sched_ptr->delay = delay/SLOW_TIMER_DUR_US;
			sched_ptr->target = num_coarse_checks + (u64)(sched_ptr->delay);

			if(wlan_sched_coarse.length == 0){
				XTmrCtr_SetResetValue(&TimerCounterInst, TIMER_CNTR_SLOW, SLOW_TIMER_DUR_US*(TIMER_FREQ/1000000));
				XTmrCtr_Start(&TimerCounterInst, TIMER_CNTR_SLOW);
			}
			dl_entry_insertEnd(&wlan_sched_coarse, entry_ptr);
		break;
		case SCHEDULE_FINE:
			sched_ptr->delay = delay/FAST_TIMER_DUR_US;
			sched_ptr->target = num_fine_checks + (u64)(sched_ptr->delay);

			if(wlan_sched_fine.length == 0){
				XTmrCtr_SetResetValue(&TimerCounterInst, TIMER_CNTR_FAST, FAST_TIMER_DUR_US*(TIMER_FREQ/1000000));
				XTmrCtr_Start(&TimerCounterInst, TIMER_CNTR_FAST);
			}
			dl_entry_insertEnd(&wlan_sched_fine, entry_ptr);
		break;
	}

	return id;
}

/*****************************************************************************/
/**
* Cancels the execution of a scheduled callback
*
* @param    scheduler_sel    - SCHEDULE_COARSE or SCHEDULE_FINE
* 			id			 	 - ID of schedule that should be removed
*
* @return	None
*
* @note		This function will fail silently if the ID parameter does not match
* 			any currently running schedule event IDs.
*
******************************************************************************/
void wlan_mac_remove_schedule(u8 scheduler_sel, u32 id){
	dl_entry*	curr_entry_ptr;
	wlan_sched* curr_sched_ptr;

	curr_entry_ptr = find_schedule(scheduler_sel, id);
	curr_sched_ptr = (wlan_sched*)(curr_entry_ptr->data);

	switch(scheduler_sel){
		case SCHEDULE_COARSE:
			if(curr_sched_ptr != NULL){
				dl_entry_remove(&wlan_sched_coarse,curr_entry_ptr);
				wlan_mac_high_free(curr_entry_ptr);
				wlan_mac_high_free(curr_sched_ptr);
			}

			if(wlan_sched_coarse.length == 0){
				//We just removed the last schedule, but the timer is still running. We should stop it.
				//When a future schedule is added, it will restart the timer at that time.
				XTmrCtr_Stop(&TimerCounterInst, TIMER_CNTR_SLOW);
			}

		break;
		case SCHEDULE_FINE:
			if(curr_sched_ptr != NULL){
				dl_entry_remove(&wlan_sched_fine,curr_entry_ptr);
				wlan_mac_high_free(curr_entry_ptr);
				wlan_mac_high_free(curr_sched_ptr);
			}

			if(wlan_sched_fine.length == 0){
				//We just removed the last schedule, but the timer is still running. We should stop it.
				//When a future schedule is added, it will restart the timer at that time.
				XTmrCtr_Stop(&TimerCounterInst, TIMER_CNTR_FAST);
			}

		break;
	}
}

/*****************************************************************************/
/**
* Internal callback used by the timer interrupt handler. This function should
* not be called by the upper-level MAC.
*
* @param    CallBackRef    - reserved
* 			TmrCtrNumber   - ID of which timer has expired
*
* @return	None
*
* @note     None
*
******************************************************************************/
void timer_handler(void *CallBackRef, u8 TmrCtrNumber){


	dl_entry*	next_entry_ptr;
	dl_entry*	curr_entry_ptr;
	wlan_sched* curr_sched_ptr;

	switch(TmrCtrNumber){
		case TIMER_CNTR_FAST:
			num_fine_checks++;
			next_entry_ptr = wlan_sched_fine.first;

			//for(i=0; i<(wlan_sched_fine.length); i++){
			while(next_entry_ptr != NULL){
				curr_entry_ptr = next_entry_ptr;
				next_entry_ptr = dl_entry_next(next_entry_ptr);

				curr_sched_ptr = (wlan_sched*)(curr_entry_ptr->data);

				if(num_fine_checks >= (curr_sched_ptr->target)){
				//	xil_printf("%d: FINE:   %d > %d\n", curr_sched_ptr->id, (u32)num_fine_checks, (u32)(curr_sched_ptr->target));
					curr_sched_ptr->callback(curr_sched_ptr->id);
					if(curr_sched_ptr->num_calls != SCHEDULE_REPEAT_FOREVER && curr_sched_ptr->num_calls != 0){
						(curr_sched_ptr->num_calls)--;
					}
					if(curr_sched_ptr->num_calls == 0){
						dl_entry_remove(&wlan_sched_fine,curr_entry_ptr);
						wlan_mac_high_free(curr_entry_ptr);
						wlan_mac_high_free(curr_sched_ptr);
					} else {
						curr_sched_ptr->target = num_fine_checks + (u64)(curr_sched_ptr->delay);
					}
				}
			}

			if(wlan_sched_fine.length > 0){
				//There are still schedules pending. Restart the timer.
				XTmrCtr_SetResetValue(&TimerCounterInst, TIMER_CNTR_FAST, FAST_TIMER_DUR_US*(TIMER_FREQ/1000000));
				XTmrCtr_Start(&TimerCounterInst, TIMER_CNTR_FAST);
			}

		break;

		case TIMER_CNTR_SLOW:
			num_coarse_checks++;
			next_entry_ptr = wlan_sched_coarse.first;

			while(next_entry_ptr != NULL){
				curr_entry_ptr = next_entry_ptr;
				next_entry_ptr = dl_entry_next(next_entry_ptr);

				curr_sched_ptr = (wlan_sched*)(curr_entry_ptr->data);

				if(num_coarse_checks >= (curr_sched_ptr->target)){

					curr_sched_ptr->callback(curr_sched_ptr->id);

					if(curr_sched_ptr->num_calls != SCHEDULE_REPEAT_FOREVER && curr_sched_ptr->num_calls != 0){
						(curr_sched_ptr->num_calls)--;
					}
					if(curr_sched_ptr->num_calls == 0){
						dl_entry_remove(&wlan_sched_coarse,curr_entry_ptr);
						wlan_mac_high_free(curr_entry_ptr);
						wlan_mac_high_free(curr_sched_ptr);
					} else {
						curr_sched_ptr->target = num_coarse_checks + (u64)(curr_sched_ptr->delay);
					}
				}
			}

			if(wlan_sched_coarse.length > 0){
				//There are still schedules pending. Restart the timer.
				XTmrCtr_SetResetValue(&TimerCounterInst, TIMER_CNTR_SLOW, SLOW_TIMER_DUR_US*(TIMER_FREQ/1000000));
				XTmrCtr_Start(&TimerCounterInst, TIMER_CNTR_SLOW);
			}

		break;
	}
}

/*****************************************************************************/
/**
* Find schedule that corresponds to a given ID
*
* @param    scheduler_sel   - SCHEDULE_COARSE or SCHEDULE_FINE
* 			id    			- id of the scheduler that should be returned
*
* @return   dl_entry*       - pointer to the doubly-linked list entry that, in turn,
* 						      points to the schedule
*
* @note     This function is a modified implementation of XTmrCtr_InterruptHandler
* 			in the xtmrctr_intr.c driver. It has been modified to remove an explicit
* 			reset of the timer.
*
******************************************************************************/
dl_entry* find_schedule(u8 scheduler_sel, u32 id){
	dl_entry*	curr_dl_entry;
	wlan_sched* curr_wlan_sched;
	u32 i;

	switch(scheduler_sel){
		case SCHEDULE_COARSE:
			curr_dl_entry = wlan_sched_coarse.first;
			for(i = 0 ; i < wlan_sched_coarse.length ; i++){
				curr_wlan_sched = (wlan_sched*)(curr_dl_entry->data);
				if(curr_wlan_sched->id == id){
					return curr_dl_entry;
				}
				curr_dl_entry = dl_entry_next(curr_dl_entry);
			}
		break;
		case SCHEDULE_FINE:
			curr_dl_entry = wlan_sched_fine.first;
			for(i = 0 ; i < wlan_sched_fine.length ; i++){
				curr_wlan_sched = (wlan_sched*)(curr_dl_entry->data);
				if(curr_wlan_sched->id == id){
					return curr_dl_entry;
				}
				curr_dl_entry = dl_entry_next(curr_dl_entry);
			}
		break;
	}
	return NULL;
}

/*****************************************************************************/
/**
* Timer interrupt handler
*
* @param    InstancePtr    - pointer to the timer instance
*
* @return	None
*
* @note     This function is a modified implementation of XTmrCtr_InterruptHandler
* 			in the xtmrctr_intr.c driver. It has been modified to remove an explicit
* 			reset of the timer.
*
******************************************************************************/
void XTmrCtr_CustomInterruptHandler(void *InstancePtr){

	XTmrCtr *TmrCtrPtr = NULL;
	u8 TmrCtrNumber;
	u32 ControlStatusReg;

#ifdef _ISR_PERF_MON_EN_
	wlan_mac_high_set_debug_gpio(ISR_PERF_MON_GPIO_MASK);
#endif

	/*
	 * Verify that each of the inputs are valid.
	 */
	Xil_AssertVoid(InstancePtr != NULL);

	/*
	 * Convert the non-typed pointer to an timer/counter instance pointer
	 * such that there is access to the timer/counter
	 */
	TmrCtrPtr = (XTmrCtr *) InstancePtr;

	/*
	 * Loop thru each timer counter in the device and call the callback
	 * function for each timer which has caused an interrupt
	 */
	for (TmrCtrNumber = 0;
		TmrCtrNumber < XTC_DEVICE_TIMER_COUNT; TmrCtrNumber++) {

		ControlStatusReg = XTmrCtr_ReadReg(TmrCtrPtr->BaseAddress,
						   TmrCtrNumber,
						   XTC_TCSR_OFFSET);
		/*
		 * Check if interrupt is enabled
		 */
		if (ControlStatusReg & XTC_CSR_ENABLE_INT_MASK) {

			/*
			 * Check if timer expired and interrupt occured
			 */
			if (ControlStatusReg & XTC_CSR_INT_OCCURED_MASK) {
				/*
				 * Increment statistics for the number of
				 * interrupts and call the callback to handle
				 * any application specific processing
				 */
				TmrCtrPtr->Stats.Interrupts++;
				TmrCtrPtr->Handler(TmrCtrPtr->CallBackRef,
						   TmrCtrNumber);
				/*
				 * Read the new Control/Status Register content.
				 */
				ControlStatusReg =
					XTmrCtr_ReadReg(TmrCtrPtr->BaseAddress,
								TmrCtrNumber,
								XTC_TCSR_OFFSET);
				/*
				 * Acknowledge the interrupt by clearing the
				 * interrupt bit in the timer control status
				 * register, this is done after calling the
				 * handler so the application could call
				 * IsExpired, the interrupt is cleared by
				 * writing a 1 to the interrupt bit of the
				 * register without changing any of the other
				 * bits
				 */
				XTmrCtr_WriteReg(TmrCtrPtr->BaseAddress,
						 TmrCtrNumber,
						 XTC_TCSR_OFFSET,
						 ControlStatusReg |
						 XTC_CSR_INT_OCCURED_MASK);
			}
		}
	}
#ifdef _ISR_PERF_MON_EN_
	wlan_mac_high_clear_debug_gpio(ISR_PERF_MON_GPIO_MASK);
#endif
}
