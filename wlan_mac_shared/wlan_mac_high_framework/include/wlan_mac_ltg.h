////////////////////////////////////////////////////////////////////////////////
// File   : wlan_mac_ltg.h
// Authors: Patrick Murphy (murphpo [at] mangocomm.com)
//			Chris Hunter (chunter [at] mangocomm.com)
// License: Copyright 2013, Mango Communications. All rights reserved.
//          Distributed under the Mango Communications Reference Design License
//				See LICENSE.txt included in the design archive or
//				at http://mangocomm.com/802.11/license
////////////////////////////////////////////////////////////////////////////////

#ifndef WLAN_MAC_LTG_H_
#define WLAN_MAC_LTG_H_

#define LTG_TYPE_CBR 1


typedef struct traffic_generator traffic_generator;
struct traffic_generator{
	u32 id;
	u32 type;
	u64 timestamp;
	void* params;
	traffic_generator* next;
	traffic_generator* prev;
};

typedef struct {
	traffic_generator* first;
	traffic_generator* last;
	u16 length;
} traffic_generator_list;

typedef struct {
	u32 interval_usec;
} cbr_params;


int wlan_mac_ltg_init();
void wlan_mac_ltg_set_callback(void(*callback)());
int start_ltg(u32 id, u32 type, void* params);
int stop_ltg(u32 id);
void check_ltg();



traffic_generator* create_ltg();
void destroy_ltg(traffic_generator* tg);

void traffic_generator_insertAfter(traffic_generator_list* ring, traffic_generator* tg, traffic_generator* tg_new);
void traffic_generator_insertBefore(traffic_generator_list* ring, traffic_generator* tg, traffic_generator* tg_new);
void traffic_generator_insertBeginning(traffic_generator_list* ring, traffic_generator* tg_new);
void traffic_generator_insertEnd(traffic_generator_list* ring, traffic_generator* tg_new);
void traffic_generator_remove(traffic_generator_list* ring, traffic_generator* tg);
void traffic_generator_list_init(traffic_generator_list* list);


#endif /* WLAN_MAC_LTG_H_ */
