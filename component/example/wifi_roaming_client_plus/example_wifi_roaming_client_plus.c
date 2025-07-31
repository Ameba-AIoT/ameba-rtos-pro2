#include <autoconf.h>
#include <FreeRTOS.h>
#include "task.h"
#include <platform_stdlib.h>
#include <wifi_conf.h>
#include <lwip_netconf.h>
#include "flash_api.h"
#include "device_lock.h"
#include "wifi_fast_connect.h"
#include "example_wifi_roaming_client_plus.h"
#include "system_data_api.h"

#include "diag.h"
#include "main.h"
#include "log_service.h"
#include "sys_api.h"

#include <platform_opts.h>
#include "osdep_service.h"
#include "rtc_api.h"
#include "wait_api.h"

//Client setting

static u8 g_roaming_enable = 0;				//Check roaming enable or disable flah
static int rssi_scan_threshold = -65;		//when current ap's rssi < rssi_scan_threshold, start to scan a better ap.
static u8 find_better_rssi_delta = 5;		//target ap's rssi - current ap's rssi > find_better_rssi_delta
static u8 g_start_roaming_time = 60;		//60		//roaming start time (unit second)
static int g_duration_roaming_time = 90;	//90		//roaming duration time (unit second)
static int g_idle_roaming_time = 86400;		//86400		//roaming idle timer (unit second)
static int roam_max_count = 5;				//Max task for stopping roaming
static int roam_scan_max_count = 3;			//Max scan time for stopping wifi scan and roaming
//Other setting
__attribute__((section(".retention.data"))) uint16_t retention_rtc_enable __attribute__((aligned(32))) = 0;
__attribute__((section(".retention.data"))) time_t retention_rtc_initial_time __attribute__((aligned(32))) = 0;
__attribute__((section(".retention.data"))) uint16_t retention_roaming_max_count __attribute__((aligned(32))) = 0;
static TimerHandle_t roaming_idle_timer = NULL;
time_t rtc_initial_time = 0;
time_t rtc_check_time = 0;
int roaming_plus_dbg = 0;					//for debug log
static int g_check_raoming_time = 1;
static int g_active_scan_time = 40;		//active scan time
static int g_passive_scan_time = 110;		//passive scan time
#define SCAN_BUFLEN 500 					//each scan list length= 14 + ssid_length(32MAX). so SCAN_BUFLEN should be AP_NUM*(14+32) at least
#define MAX_POLLING_TIME 3					//Wifi scan without reaching the rssi standard for three consecutive times
static TaskHandle_t roaming_plus_thread_handle = NULL;
u32 roam_count = 0;
extern int wifi_get_sta_max_data_rate(OUT unsigned char *inidata_rate);

#define ROAMING_DBG(...) do { \
							if(roaming_plus_dbg){\
								printf(__VA_ARGS__); \
 							}\
						}while(0)

//user should config channel plan
typedef struct channel_plan {
	u8 channel[14];
	u8	len;
} channel_plan_t;

channel_plan_t roaming_channel_plan = {{1, 6, 11, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14}, 14};


typedef struct wifi_roaming_ap {
	u8 	ssid[33];
	u8 	bssid[ETH_ALEN];
	u8	channel;
	rtw_security_t		security_type;
	s32	rssi;
#if CONFIG_LWIP_LAYER
	u8	ip[4];
#endif
} wifi_roaming_ap_t;

enum {
	FAST_CONNECT_SPECIFIC_CH = 0,
	FAST_CONNECT_ALL_CH  = 1
};

#if CONFIG_LWIP_LAYER
extern struct netif xnetif[NET_IF_NUM];
#endif
static wifi_roaming_ap_t *ap_list;
#define SCAN_AP_LIST_MAX 128
int partial_scan_ap_index = 0;
static wifi_roaming_ap_t partial_scan_ap_list[SCAN_AP_LIST_MAX];
static u8 pscan_enable = _TRUE; // if set _TRUE, please set pscan_channel_list
static u8 pscan_channel_list[] = {1}; // set by customer
static unsigned short ping_seq = 0;
#if defined(CONFIG_FAST_DHCP) && CONFIG_FAST_DHCP
extern uint32_t offer_ip;
extern uint32_t server_ip;
#endif

int wifi_do_connect(rtw_network_info_t wifi)
{
	int ret;
	uint32_t wifi_retry_connect = 5;

	char empty_bssid[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	if (memcmp(wifi.bssid.octet, empty_bssid, 6)) {
		printf("\n\r[wifi_do_connect] "MAC_FMT"\n\r", MAC_ARG(wifi.bssid.octet));
	} else {
		printf("\n\r[wifi_do_connect] NULL\n\r");
	}

WIFI_RETRY_LOOP:

	ret = wifi_connect(&wifi, 1);
	if (ret != RTW_SUCCESS) {
		wifi_retry_connect--;
		if (wifi_retry_connect > 0) {
			vTaskDelay(300);
			printf("wifi retry\r\n");
			goto WIFI_RETRY_LOOP;
		}
	}

	if (ret == RTW_SUCCESS) {
		LwIP_DHCP(0, DHCP_START);
		//clean arp; old arp table may not update.
		etharp_cleanup_netif(&xnetif[0]);
	}

	return ret;
}

static int wlan_roaming_connect(struct wlan_fast_reconnect *data)
{
	ROAMING_DBG("%s()", __func__);
	unsigned long tick1 = xTaskGetTickCount();
	unsigned long tick2;

	uint32_t	channel;
	uint32_t    security_type;
	u8 key_id ;
	int ret;
	rtw_network_info_t wifi = {0};
	struct psk_info PSK_INFO;

#if CONFIG_LWIP_LAYER
	netif_set_up(&xnetif[0]);
#endif

	memset(&PSK_INFO, 0, sizeof(struct psk_info));
	memcpy(PSK_INFO.psk_essid, data->psk_essid, sizeof(data->psk_essid));
	memcpy(PSK_INFO.psk_passphrase, data->psk_passphrase, sizeof(data->psk_passphrase));
	memcpy(PSK_INFO.wpa_global_PSK, data->wpa_global_PSK, sizeof(data->wpa_global_PSK));
	wifi_psk_info_set(&PSK_INFO);

	channel = data->channel;
	key_id = channel >> 28;
	channel &= 0xff;
	security_type = data->security_type;

	//set partial scan for entering to listen beacon quickly
	wifi.channel = (u8)channel;
	wifi.pscan_option = PSCAN_FAST_SURVEY;
	wifi.security_type = security_type;
	//SSID
	strcpy((char *)wifi.ssid.val, (char *)(data->psk_essid));
	wifi.ssid.len = strlen((char *)(data->psk_essid));

	switch (security_type) {
	case RTW_SECURITY_WEP_PSK:
		wifi.password = (unsigned char *) data->psk_passphrase;
		wifi.password_len = strlen((char *)data->psk_passphrase);
		wifi.key_id = key_id;
		break;
	case RTW_SECURITY_WPA_TKIP_PSK:
	case RTW_SECURITY_WPA_AES_PSK:
	case RTW_SECURITY_WPA_MIXED_PSK:
	case RTW_SECURITY_WPA2_TKIP_PSK:
	case RTW_SECURITY_WPA2_AES_PSK:
	case RTW_SECURITY_WPA2_MIXED_PSK:
	case RTW_SECURITY_WPA_WPA2_TKIP_PSK:
	case RTW_SECURITY_WPA_WPA2_AES_PSK:
	case RTW_SECURITY_WPA_WPA2_MIXED_PSK:
#ifdef CONFIG_SAE_SUPPORT
	case RTW_SECURITY_WPA3_AES_PSK:
	case RTW_SECURITY_WPA3_GCMP_PSK:
	case RTW_SECURITY_WPA2_WPA3_MIXED:
#endif
		wifi.password = (unsigned char *) data->psk_passphrase;
		wifi.password_len = strlen((char *)data->psk_passphrase);
		break;
	default:
		break;
	}


	// 1.start to connect roaming target ap(store the roaming ap in ap_list)
	wifi.channel = ap_list->channel;
	rtw_memcpy(wifi.bssid.octet, ap_list->bssid, 6);
	wifi.roam_en = 1;
	ret = wifi_do_connect(wifi);

	//2. it connect back to original AP if the failure of connecting to the target raoming ap
	if (ret != RTW_SUCCESS) {
		ROAMING_DBG("[%s] Back to default ap\n\r", __FUNCTION__);
		wifi.channel = data->channel;
		wifi.roam_en = 0;
		rtw_memcpy(wifi.bssid.octet, data->bssid, 6);
		ret = wifi_do_connect(wifi);
	}

	tick2 = xTaskGetTickCount();
	ROAMING_DBG("\n\r == Roaming connect done  ==\n\n\r");
	return ret;
}

static u32 wifi_roaming_plus_find_ap_from_scan_buf(char *target_ssid, void *user_data, int ap_num)
{
	u32 target_security = *(u32 *)user_data;
	rtw_scan_result_t *scanned_ap_info;
	u32 i = 0;
	char *scan_buf = NULL;
	int is_new_ap = 1;
	scan_buf = (char *)rtw_zmalloc(ap_num * sizeof(rtw_scan_result_t));
	if (scan_buf == NULL) {
		printf("malloc scan buf for example wifi roaming plus\n");
		return -1;
	}

	if (wifi_get_scan_records((unsigned int *)(&ap_num), scan_buf) < 0) {
		rtw_mfree((u8 *)scan_buf, 0);
		return -1;
	}

	for (i = 0; i < ap_num; i++) {
		scanned_ap_info = (rtw_scan_result_t *)(scan_buf + i * sizeof(rtw_scan_result_t));
		ROAMING_DBG("Partial Scan (i: %d) Scan ap:"MAC_FMT", Channel %d, Rssi: %d\n", i, MAC_ARG(scanned_ap_info->BSSID.octet), scanned_ap_info->channel,
					scanned_ap_info->signal_strength);
		if (target_security == scanned_ap_info->security ||
			((target_security & (WPA3_SECURITY | WPA2_SECURITY | WPA_SECURITY)) && ((scanned_ap_info->security) & (WPA3_SECURITY | WPA2_SECURITY | WPA_SECURITY)))) {
#if 0
			if (ap_list->rssi < scanned_ap_info->signal_strength) {
				ROAMING_DBG("rssi(%d) is better than last(%d)\n", scanned_ap_info->signal_strength, ap_list->rssi);
				memset(ap_list, 0, sizeof(wifi_roaming_ap_t));
				memcpy(ap_list->bssid, scanned_ap_info->BSSID.octet, ETH_ALEN);
				ap_list->channel = scanned_ap_info->channel;
				ap_list->rssi = scanned_ap_info->signal_strength;
			}
#else
			// check if the ap is dupulicate and insert new best rssi
			for (int i = 0; i < partial_scan_ap_index; i++) {
				if (memcmp(partial_scan_ap_list[i].bssid, scanned_ap_info->BSSID.octet, ETH_ALEN) == 0) {
					if (scanned_ap_info->signal_strength >= partial_scan_ap_list[i].rssi) {
						printf("[Replace] MAC: "MAC_FMT", rssi(%d -> %d)", MAC_ARG(scanned_ap_info->BSSID.octet), partial_scan_ap_list[i].rssi, scanned_ap_info->signal_strength);
						partial_scan_ap_list[i].rssi = scanned_ap_info->signal_strength;
					}
					is_new_ap = 0;
				}
			}

			if (is_new_ap == 1) {
				memcpy(partial_scan_ap_list[partial_scan_ap_index].bssid, scanned_ap_info->BSSID.octet, ETH_ALEN);
				partial_scan_ap_list[partial_scan_ap_index].channel = scanned_ap_info->channel;
				partial_scan_ap_list[partial_scan_ap_index].rssi = scanned_ap_info->signal_strength;
				partial_scan_ap_index = partial_scan_ap_index + 1;
			}
#endif
		}
	}
	rtw_mfree((u8 *)scan_buf, 0);

	return 0;
}

int wifi_roaming_scan(u32 retry)
{
	wifi_roaming_ap_t	roaming_ap;
	rtw_wifi_setting_t	setting;
	channel_plan_t channel_plan_temp = roaming_channel_plan;
	u8 ch = 0;
	u8 first_5g = 0;
	int cur_rssi = 0;
	int rssi_delta = find_better_rssi_delta;
	rtw_phy_statistics_t phy_statistics;
	rtw_scan_param_t scan_param;
	int scanned_ap_num = 0;

	memset(&setting, 0, sizeof(rtw_wifi_setting_t));
	memset(&roaming_ap, 0, sizeof(wifi_roaming_ap_t));
	roaming_ap.rssi = -100;

	wifi_get_setting(WLAN0_IDX, &setting);
	strcpy((char *)roaming_ap.ssid, (char const *)setting.ssid);
	roaming_ap.security_type =  setting.security_type;
	rtw_memcpy(roaming_ap.bssid, setting.bssid, 6);

	for (ch = 0 ; ch < channel_plan_temp.len; ch++) {
		if (channel_plan_temp.channel[ch]) {
			pscan_channel_list[0] = channel_plan_temp.channel[ch];
			//ROAMING_DBG("scan(%d)\n", pscan_channel_list[0]);
			//set scan_param for scan
			rtw_memset(&scan_param, 0, sizeof(rtw_scan_param_t));
			scan_param.ssid = (char *)roaming_ap.ssid;
			scan_param.channel_list = pscan_channel_list;
			scan_param.channel_list_num = 1;
			scan_param.chan_scan_time.active_scan_time = g_active_scan_time;
			scan_param.chan_scan_time.passive_scan_time = g_passive_scan_time;
			scanned_ap_num = wifi_scan_networks(&scan_param, 1);
			if (scanned_ap_num > 0) {
				ROAMING_DBG("scan(%d)\n", pscan_channel_list[0]);
				wifi_roaming_plus_find_ap_from_scan_buf((char *)roaming_ap.ssid, (void *)&roaming_ap.security_type, scanned_ap_num);
				ROAMING_DBG("scan(%d) done!\n", pscan_channel_list[0]);
			}
			//ROAMING_DBG("scan(%d) done!\n", pscan_channel_list[0]);
			channel_plan_temp.channel[ch] = 0;
			wifi_fetch_phy_statistic(&phy_statistics);
			cur_rssi = phy_statistics.rssi;
			rssi_delta = find_better_rssi_delta;
#if 0
			if (ap_list->rssi - cur_rssi > rssi_delta && (memcmp(roaming_ap.bssid, ap_list->bssid, ETH_ALEN))) {
				printf("\r\n[Wifi roaming plus]: Find a better ap on channel %d, rssi = %d, cur_rssi=%d, mac: "MAC_FMT"\n"
					   , ap_list->channel, ap_list->rssi, cur_rssi, MAC_ARG(ap_list->bssid));
				return 1;
			}
#endif
			vTaskDelay(200);
		}
	}

	//select best rssi and printf all candidate ap
	printf("-------------------------------------Scan AP List(num %d)-------------------------------------\n\r", partial_scan_ap_index);
	for (int i = 0; i < partial_scan_ap_index; i++) {
		printf("Scan AP (number: %d) MAC:"MAC_FMT", Channel: %d, Rssi: %d\n"
			   , i, MAC_ARG(partial_scan_ap_list[i].bssid), partial_scan_ap_list[i].channel, partial_scan_ap_list[i].rssi);
		if (ap_list->rssi < partial_scan_ap_list[i].rssi) {
			//ROAMING_DBG("rssi(%d) is better than last(%d)\n", partial_scan_ap_list[i].rssi, ap_list->rssi);
			memset(ap_list, 0, sizeof(wifi_roaming_ap_t));
			memcpy(ap_list->bssid, partial_scan_ap_list[i].bssid, ETH_ALEN);
			ap_list->channel = partial_scan_ap_list[i].channel;
			ap_list->rssi = partial_scan_ap_list[i].rssi;
		}
	}
	printf("--------------------------------------Scan AP List(End)--------------------------------------\n\r");


	//select best rssi and printf all candidate ap
	printf("\n\r-------------------------------------Candidate AP List-------------------------------------\n\r");
	for (int i = 0; i < partial_scan_ap_index; i++) {
		if (partial_scan_ap_list[i].rssi - cur_rssi > rssi_delta) {
			printf("Candidate AP (number: %d) MAC:"MAC_FMT", Channel: %d, Rssi: %d\n"
				   , i, MAC_ARG(partial_scan_ap_list[i].bssid), partial_scan_ap_list[i].channel, partial_scan_ap_list[i].rssi);
		}
	}
	printf("------------------------------------Candidate AP List(End)------------------------------------\n\r");


	if (ap_list->rssi - cur_rssi > rssi_delta && (memcmp(roaming_ap.bssid, ap_list->bssid, ETH_ALEN))) {
		printf("\n\r-------------------------------------Find the target ap to Roaming-------------------------------------\n\r");
		printf("Find a better ap on channel %d, target_rssi = %d, cur_rssi = %d, rssi_delta: %d, target_ap_mac: "MAC_FMT"\n"
			   , ap_list->channel, ap_list->rssi, cur_rssi, rssi_delta, MAC_ARG(ap_list->bssid));
		printf("-----------------------------------Find the target ap to Roaming(End)-----------------------------------\n\r\n\r");
		return 1;
	}

	printf("\n\r[%s]: Find a better ap fail, roaming scan retry: %d, roam_scan_max_count: %d!\n\r\n\r", __FUNCTION__, retry, roam_scan_max_count);
	return 0;
}

void wifi_roaming_plus_thread(void *param)
{
	ROAMING_DBG("[%s] start!\n", __func__);
	unsigned long tick1;
	unsigned long tick2;
	unsigned long tick_diff;
	(void)param;
	signed char ap_rssi;
	rtw_phy_statistics_t phy_statistics;
	u32	polling_count = 0;
	struct wlan_fast_reconnect read_data = {0};
	u8 rate = 0;
	int roam_scan_count = 1;
	vTaskDelay(g_start_roaming_time * configTICK_RATE_HZ);
	tick1 = xTaskGetTickCount();
	while (1) {
		if (wifi_is_running(WLAN0_IDX) && ((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) && (*(u32 *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
			wifi_fetch_phy_statistic(&phy_statistics);
			ap_rssi = phy_statistics.rssi;
			wifi_get_sta_max_data_rate(&rate);
			printf("[%s] Current rssi(%d),scan threshold rssi(%d); data_rate: (%d)\n\r", __func__, ap_rssi, rssi_scan_threshold, rate);
			if (ap_rssi < rssi_scan_threshold) {
				if ((polling_count >= MAX_POLLING_TIME) && (roam_scan_count <= roam_scan_max_count)) {
					printf("[%s] Start roaming scan, current rssi(%d) < scan threshold rssi(%d)\n\r"
						   , __func__, ap_rssi, rssi_scan_threshold);
					ap_list = (wifi_roaming_ap_t *)malloc(sizeof(wifi_roaming_ap_t));
					memset(ap_list, 0, sizeof(wifi_roaming_ap_t));
					ap_list->rssi = -100;

					partial_scan_ap_index = 0;
					memset(partial_scan_ap_list, 0, SCAN_AP_LIST_MAX * sizeof(wifi_roaming_ap_t));
					memset(&read_data, 0xff, sizeof(struct wlan_fast_reconnect));
					sys_read_wlan_data_from_flash((uint8_t *) &read_data,  sizeof(struct wlan_fast_reconnect));
					/*1.find a better ap*/
					if (wifi_roaming_scan(roam_scan_count)) {
						if (wifi_is_running(WLAN0_IDX) && ((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) && (*(u32 *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
							/*2.connect a better ap*/
							printf("[%s] Start roaming connection\n\r", __func__);
							//read_data.channel = ap_list->channel;
							//rtw_memcpy(read_data.bssid, ap_list->bssid, 6);
							wlan_roaming_connect(&read_data);
							//printf("Roaming task stop, exceed timeout!\n\r");
							roam_count = 0;
							retention_roaming_max_count = roam_count;
							g_roaming_enable = 0;
							roaming_plus_thread_handle = NULL;
							vTaskDelete(NULL);
						}
					} else { //scan fail
						roam_scan_count++;
						//retention_roaming_max_count = roam_count;
						//ROAMING_DBG("Roaming retention_roaming_max_count: %d\n\r", retention_roaming_max_count);
					}
					free(ap_list);
					polling_count = 0;
				} else {
					polling_count++;
				}
			} else {
				polling_count = 0;
			}
		}

		//Over the max scan time, so we need to stop the roaming task
		if (roam_scan_count > roam_scan_max_count) {
			//Record Roaming task count
			roam_count = roam_count + 1;
			retention_roaming_max_count = roam_count;
			//printf("\n\rRoaming (%d > %d) task stop, exceed roaming retry\n\r", roam_scan_count, roam_scan_max_count);
			printf("\n\rRoaming task stop, exceed roam_scan_max_count\n\r", roam_scan_count, roam_scan_max_count);
			g_roaming_enable = 0;
			roaming_plus_thread_handle = NULL;
			vTaskDelete(NULL);
		}

		//Check wifi roaming duration time
		tick2 = xTaskGetTickCount();
		tick_diff = tick2 - tick1;
		if (tick_diff > (g_duration_roaming_time * configTICK_RATE_HZ)) {
			//Record Roaming task count
			roam_count = roam_count + 1;
			retention_roaming_max_count = roam_count;
			printf("Roaming task stop, exceed timeout!\n\r");
			g_roaming_enable = 0;
			roaming_plus_thread_handle = NULL;
			vTaskDelete(NULL);
		}
		//Idle
		vTaskDelay(g_check_raoming_time * configTICK_RATE_HZ);
	}

}

#if 1
int wifi_start_roaming_task(void)
{

	int ret = 0;
	if (roaming_plus_thread_handle == NULL) {
		if (xTaskCreate(wifi_roaming_plus_thread, ((const char *)"wifi_roaming_thread"), 1280, NULL, tskIDLE_PRIORITY + 1, &roaming_plus_thread_handle) != pdPASS) {
			printf("\n\r%s xTaskCreate(wifi_roaming_thread) failed", __FUNCTION__);
			ret = -1;
		} else {
			g_roaming_enable = 1;
		}
	} else {
		printf("%s xTaskCreate(wifi_roaming_thread) exit\n\r", __FUNCTION__);
		ret = -1;
	}
	printf("Enable wifi roaming task ret: %d\n\r", ret);
	return ret;
}

void wifi_stop_roaming_task(void)
{

	if (roaming_plus_thread_handle) {
		g_roaming_enable = 0;
		vTaskDelete(roaming_plus_thread_handle);
		roaming_plus_thread_handle = NULL;
	}

}

void wifi_set_roaming_startup_time(u8 startup_timeout)
{
	g_start_roaming_time = startup_timeout;
}

void wifi_set_roaming_duration_time(int duration_time)
{
	g_duration_roaming_time = duration_time;
}

void wifi_set_roaming_max_count(int roam_max_counts)
{
	roam_max_count = roam_max_counts;
}

void wifi_set_roaming_scan_max_count(int roam_scan_max_counts)
{
	roam_scan_max_count = roam_scan_max_counts;
}

void wifi_set_roaming_rssi_threshold(int rssi_scan_thresholds)
{
	rssi_scan_threshold = rssi_scan_thresholds;
}

void wifi_set_roaming_partial_scan_time(int active_scan_time, int passive_scan_time)
{
	g_active_scan_time = active_scan_time;
	g_passive_scan_time = passive_scan_time;
}

void wifi_set_find_better_rssi_delta(u8 rssi_delta)
{
	find_better_rssi_delta = rssi_delta;
}

void wifi_set_idle_roaming_time(int idle_roaming_time)
{
	g_idle_roaming_time = idle_roaming_time;
}

void print_ROAMING_help(void)
{
	printf("ROAM=[enable|disable],[options]\r\n");

	printf("\r\n");
	printf("ROAM=startup_time,%d\r\n", g_start_roaming_time);
	printf("\tSetting roaming start time after %ds\r\n", g_start_roaming_time);

	printf("\r\n");
	printf("ROAM=duration_time,%d\r\n", g_duration_roaming_time);
	printf("\tSetting roaming duarion time after %ds and then stop roaming\r\n", g_duration_roaming_time);

	printf("\r\n");
	printf("ROAM=roam_max_count,%d\r\n", roam_max_count);
	printf("\tSetting roaming max count after %d and then kill the roaming timer\r\n", roam_max_count);

	printf("\r\n");
	printf("ROAM=rssi,%d\r\n", rssi_scan_threshold);
	printf("\tSetting roaming scan rssi threshold %d\r\n", rssi_scan_threshold);

	printf("\r\n");
	printf("ROAM=partial_scan,%d,%d\r\n", g_active_scan_time, g_passive_scan_time);
	printf("\tSetting roaming partial_scan about active scan: %dms and  passive scan: %dms\r\n", g_active_scan_time, g_passive_scan_time);

	printf("\r\n");
	printf("ROAM=rssi_delta,%d\r\n", find_better_rssi_delta);
	printf("\tSetting roaming find better rssi delta: %d\r\n", find_better_rssi_delta);

	printf("\r\n");
	printf("ROAM=idle_timer,%d\r\n", g_idle_roaming_time);
	printf("\tSetting roaming idle timer: %ds\r\n", g_idle_roaming_time);

	printf("\r\n");
	printf("ROAM=roam_scan_max_count,%d\r\n", roam_scan_max_count);
	printf("\tSetting roaming max scan count after %d and then stop roaming\r\n", roam_scan_max_count);
}

void fROAM(void *arg)
{
	int argc;
	char *argv[MAX_ARGC] = {0};

	argc = parse_param(arg, argv);

	do {
		if (argc == 1) {
			print_ROAMING_help();
			break;
		}

		if (strcmp(argv[1], "enable") == 0) {
			int ret = 0;
			ret = wifi_start_roaming_task();
		} else if (strcmp(argv[1], "disable") == 0) {
			printf("\n\rDisable wifi roaming task\n\r");
			wifi_stop_roaming_task();
		} else if (strcmp(argv[1], "startup_time") == 0) {
			if (argc == 3) {
				printf("\n\rstartup_time=%02d", atoi(argv[2]));
				u8 roaming_timeout = atoi(argv[2]);
				wifi_set_roaming_startup_time(roaming_timeout);
			}
		} else if (strcmp(argv[1], "duration_time") == 0) {
			if (argc == 3) {
				printf("\n\rduration_time=%02d", atoi(argv[2]));
				int duration_time = atoi(argv[2]);
				wifi_set_roaming_duration_time(duration_time);
			}
		} else if (strcmp(argv[1], "roam_max_count") == 0) {
			if (argc == 3) {
				printf("\n\rroam_max_count=%02d", atoi(argv[2]));
				int max_count = atoi(argv[2]);
				wifi_set_roaming_max_count(max_count);
			}
		} else if (strcmp(argv[1], "rssi") == 0) {
			if (argc == 3) {
				printf("\n\rrssi_scan_threshold=%02d", atoi(argv[2]));
				int rssi_threshold = atoi(argv[2]);
				wifi_set_roaming_rssi_threshold(rssi_threshold);
			}
		} else if (strcmp(argv[1], "rssi_delta") == 0) {
			if (argc == 3) {
				printf("\n\rfind_better_rssi_delta=%02d", atoi(argv[2]));
				int rssi_delta = atoi(argv[2]);
				wifi_set_find_better_rssi_delta(rssi_delta);
			}
		} else if (strcmp(argv[1], "partial_scan") == 0) {
			if (argc == 4) {
				printf("\n\rg_active_scan_time=%02d, g_passive_scan_time=%02d", atoi(argv[2]), atoi(argv[3]));
				int active_scan_time = atoi(argv[2]);
				int passive_scan_time = atoi(argv[3]);
				wifi_set_roaming_partial_scan_time(active_scan_time, passive_scan_time);
			}
		} else if (strcmp(argv[1], "idle_timer") == 0) {
			if (argc == 3) {
				printf("\n\rg_idle_roaming_time=%02d", atoi(argv[2]));
				int idle_roaming_time = atoi(argv[2]);
				wifi_set_idle_roaming_time(idle_roaming_time);
			}
		} else if (strcmp(argv[1], "debug") == 0) {
			if (argc == 3) {
				printf("\n\rroaming_plus_dbg=%02d", atoi(argv[2]));
				roaming_plus_dbg = atoi(argv[2]);
			}
		} else if (strcmp(argv[1], "read_flash") == 0) {
			u8 *read_data = NULL;
			int k = 0;
			sys_read_wlan_data_from_flash((uint8_t *) read_data,  sizeof(struct wlan_fast_reconnect));
			printf("\n\rRead size: %d\n\r", sizeof(struct wlan_fast_reconnect));
			for (int i = 0; i < sizeof(struct wlan_fast_reconnect); i++) {
				k++;
				if (i == sizeof(struct wlan_fast_reconnect)) {
					printf("\n\r--------------------end--------------------\n\r");
				}

				printf("%02x ", read_data[i]);

				if (k % 16 == 0) {
					printf("\n\r");
				}
			}
		} else if (strcmp(argv[1], "roam_scan_max_count") == 0) {
			if (argc == 3) {
				printf("\n\rroam_scan_max_counts=%02d", atoi(argv[2]));
				int roam_scan_max_counts = atoi(argv[2]);
				wifi_set_roaming_scan_max_count(roam_scan_max_counts);
			}
		}
	} while (0);
}
log_item_t at_roaming_items[ ] = {
	{"ROAM", fROAM,},
};
#endif

void roaming_idle_timeout_handler(void *param)
{
	time_t diff_time;
	int ret = 0;
	//Over the max scan time, so we need to stop the roaming task
	if (roam_count >= roam_max_count) {
		printf("Roaming_idle (%d > %d) roaming stop, because it exceed the roam_max_count\n\r", roam_count, roam_max_count);
		g_roaming_enable = 0;
		roaming_plus_thread_handle = NULL;
		retention_rtc_enable = 0;
		retention_rtc_initial_time = 0;
		retention_roaming_max_count = 0;

		if (pdFAIL == xTimerDelete(roaming_idle_timer, 0)) {
			printf("ERROR: roaming_idle_timeout_handler xTimerDelete\n\r");
		}
		roaming_idle_timer = NULL;
		return;
	}

	rtc_check_time = rtc_read();
	diff_time = rtc_check_time - rtc_initial_time;
	if (diff_time < g_idle_roaming_time) {
		return;
	}

	rtc_initial_time = rtc_check_time;
	retention_rtc_initial_time = rtc_initial_time;
	//printf("[roaming_idle_timeout_handler] retention_rtc_initial_time: %lld\n\r",retention_rtc_initial_time);
	wifi_set_roaming_startup_time(0);
	ret = wifi_start_roaming_task();
}

void roaming_idle_change_period(int timeout)
{
	printf("roaming_idle_change_period: %d\n\r", timeout);

	if (pdFAIL == xTimerChangePeriod(roaming_idle_timer, (TickType_t)timeout, 0)) {
		printf("\n\rERROR: roaming_idle_change_period\n\r");
	}
}

void roaming_idle_start(void)
{

	roaming_idle_timer = xTimerCreate("roaming_idle_timer",    /* Text name to facilitate debugging. */
									  1000 / portTICK_RATE_MS, /* Tick every 1 sec */
									  pdTRUE,     /* The timer will auto-reload themselves when they expire. */
									  NULL,     /* In this case this is not used as the timer has its own callback. */
									  (TimerCallbackFunction_t)roaming_idle_timeout_handler);  /* The callback to be called when the timer expires. */

	if (roaming_idle_timer != NULL) {
		if (pdFAIL == xTimerStart(roaming_idle_timer, (TickType_t) 0)) {
			printf("ERROR: roaming_idle_start xTimerStart\n\r");

			/* active timer failed, delete timer. */
			if (pdFAIL == xTimerDelete(roaming_idle_timer, 0)) {
				printf("ERROR: roaming_idle_start xTimerDelete\n\r");
			} else {
				roaming_idle_timer = NULL;
			}
		}
	}
}

void example_wifi_roaming_client_plus(void)
{
	int ret = 0;

	//rtc setup and restore to retention for idle_roaming
	printf("[example_wifi_roaming_client_plus] retention_rtc_enable: %d, retention_rtc_initial_time: %lld; retention_roaming_max_count: %d\n\r"
		   , retention_rtc_enable, retention_rtc_initial_time, retention_roaming_max_count);
	rtc_init();
	if (retention_rtc_enable != 1) {
		//Initialize the rtc time and retention data
		rtc_write(0);
		retention_rtc_enable = 1;
		retention_rtc_initial_time = 0;
		retention_roaming_max_count = 0;
		rtc_initial_time = rtc_read();
		//Start roaming task
		ret = wifi_start_roaming_task();
	}

	//Backup tetention data about rtc_time and roam retry count
	rtc_initial_time = retention_rtc_initial_time;
	roam_count = retention_roaming_max_count;

	//roaming idle timer
	roaming_idle_start();

	log_service_add_table(at_roaming_items, sizeof(at_roaming_items) / sizeof(at_roaming_items[0]));

	return;
}