#ifndef __EXAMPLE_WIFI_ROAMING_H__
#define __EXAMPLE_WIFI_ROAMING_H__

#include "wifi_structures.h"

//fast reconnect callback fun
extern wifi_do_fast_connect_ptr p_wifi_do_fast_connect;
extern write_fast_connect_info_ptr p_store_fast_connect_info;

void example_wifi_roaming_client_plus(void);
#endif