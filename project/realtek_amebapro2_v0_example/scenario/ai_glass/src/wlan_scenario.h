#ifndef _WLAN_SCENARIO_H
#define _WLAN_SCENARIO_H

#include <platform_stdlib.h>
#include "platform_opts.h"
#include "wifi_structures.h"
#include "wifi_fast_connect.h"
#include "system_data_api.h"

// Static IP ADDRESS for AI glass
#define AI_GLASS_AP_IP_ADDR0    192
#define AI_GLASS_AP_IP_ADDR1    168
#define AI_GLASS_AP_IP_ADDR2    43
#define AI_GLASS_AP_IP_ADDR3    1

// NETMASK
#define AI_GLASS_AP_NETMASK_ADDR0   255
#define AI_GLASS_AP_NETMASK_ADDR1   255
#define AI_GLASS_AP_NETMASK_ADDR2   255
#define AI_GLASS_AP_NETMASK_ADDR3   0

// Gateway Address
#define AI_GLASS_AP_GW_ADDR0    192
#define AI_GLASS_AP_GW_ADDR1    168
#define AI_GLASS_AP_GW_ADDR2    43
#define AI_GLASS_AP_GW_ADDR3    1

// AP ssid
#define AI_GLASS_AP_SSID        "AI_GLASS_AP"

// AP password
#define AI_GLASS_AP_PASSWORD    "rtkaiglass"

// AP channel
#define AI_GLASS_AP_CHANNEL     40

// HTTP connection timeout in second
#define HTTPD_CONNECT_TIMEOUT   15

// Connection status
#define MAX_AP_SSID_VALUE_LEN       33
#define MAX_AP_PASSWORD_LEN         65
#define WLAN_STAT_IDLE              0
#define WLAN_STAT_HTTP_IDLE         1
#define WLAN_STAT_HTTP_CONNECTED    2

enum {
	WLAN_SET_FAIL       = -1,
	WLAN_SET_OK         = 0,
};

int wifi_enable_sta_mode(rtw_network_info_t *connect_param, int timeout, int retry);
int wifi_disable_sta_mode(void);
int wifi_enable_ap_mode(const char *ssid, const char *password, int channel, int timeout);
int wifi_disable_ap_mode(void);
int wifi_get_ap_setting(rtw_softap_info_t *wifi_cfg);
int wifi_get_connect_status(void);
void wifi_set_up_file_delete_flag(uint8_t flag);

#endif
