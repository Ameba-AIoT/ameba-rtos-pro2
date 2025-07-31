#include <autoconf.h>
#include <FreeRTOS.h>
#include "task.h"
#include <platform_stdlib.h>
#include <wifi_conf.h>
#include <lwip_netconf.h>
#include "flash_api.h"
#include "device_lock.h"
#include "wifi_fast_connect.h"
#include "example_wifi_roaming_plus.h"
#include "system_data_api.h"

#include "diag.h"
#include "main.h"
#include "log_service.h"
#include "sys_api.h"

#include <platform_opts.h>
#include "osdep_service.h"


#define BAND_STEERING_FUNC 0

#if (BAND_STEERING_FUNC == 1)
void bandsteering_task(void *param);
static void wifi_disconnect_hdl(char *buf, int buf_len, int flags, void *userdata);
static TaskHandle_t bandsteering_thread_handle = NULL;
#define BLASK_AP_NUM 8
#define RX_DISCONNECT_MAX_CNT 2
int bandsteering_retry_connect = 3;
int bandsteering_retry_scan = 3;
int bandsteer_dbg_flag = 1;
int g_deauth_cnt = 0;
int g_blackap_num = 0;
u8 g_ap_mac[RX_DISCONNECT_MAX_CNT][ETH_ALEN];
u8 g_blackap_mac[BLASK_AP_NUM][ETH_ALEN];
#define BANDSTEER_DBG(...) do { \
							if(bandsteer_dbg_flag){\
								printf(__VA_ARGS__); \
 							}\
						}while(0)
#endif

static int RSSI_SCAN_THRESHOLD = -70;	//when current ap's rssi < RSSI_SCAN_THRESHOLD, start to scan a better ap.
#define FIND_BETTER_RSSI_DELTA 5	//target ap's rssi - current ap's rssi > FIND_BETTER_RSSI_DELTA
#define SUPPORT_SCAN_5G_CHANNEL 0
#define MAX_CH_NUM 8		// config the max channel number store in flash
#define MAX_AP_NUM 8		// config the max ap number store in flash
int ROAMING_PLUS_DBG = 1;		//for debug log
#define SCAN_BUFLEN 500 	//each scan list length= 14 + ssid_length(32MAX). so SCAN_BUFLEN should be AP_NUM*(14+32) at least
static int MAX_SCAN_TIME = 5;		//Max scan time for stopping wifi scan and roaming
#define MAX_POLLING_TIME 2	//Wifi scan without reaching the rssi standard for three consecutive times
//fast reconnect callback fun
extern wifi_do_fast_connect_ptr p_wifi_do_fast_connect;
extern write_fast_connect_info_ptr p_store_fast_connect_info;
extern int wifi_get_sta_max_data_rate(OUT unsigned char *inidata_rate);

#define ROAMING_DBG(...) do { \
							if(ROAMING_PLUS_DBG){\
								printf(__VA_ARGS__); \
 							}\
						}while(0)

//user should config channel plan
typedef struct channel_plan {
#if SUPPORT_SCAN_5G_CHANNEL
	u8 channel[39];
#else
	u8 channel[14];
#endif
	u8	len;
} channel_plan_t;
#if SUPPORT_SCAN_5G_CHANNEL
channel_plan_t roaming_channel_plan = {{
		1, 6, 11, 149, 153, 157, 161, 165, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108,
		112, 116, 120, 124, 128, 132, 136, 140, 144
	}, 38
};
#else
channel_plan_t roaming_channel_plan = {{1, 6, 11, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13}, 13};
#endif

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

//ext info in flash
struct ap_additional_info {
	u8 ap_bssid[ETH_ALEN];
	uint32_t sta_ip;
	uint32_t server_ip;
} ap_additional_info_t;

//info in flash
struct wifi_roaming_data {
	struct wlan_fast_reconnect ap_info;
	u8 num;
	u8 ap_n;
#if (BAND_STEERING_FUNC == 1)
	u8 blackap_num;
#endif
	u32 channel[MAX_CH_NUM];
	struct ap_additional_info add_ap_info[MAX_AP_NUM];
#if (BAND_STEERING_FUNC == 1)
	u8 blackap_bssid[BLASK_AP_NUM][ETH_ALEN];
#endif
} wifi_roaming_data_t;

enum {
	FAST_CONNECT_SPECIFIC_CH = 0,
	FAST_CONNECT_ALL_CH  = 1
};

#if CONFIG_LWIP_LAYER
extern struct netif xnetif[NET_IF_NUM];
#endif
static wifi_roaming_ap_t *ap_list = NULL;
static u8 pscan_enable = _TRUE; // if set _TRUE, please set pscan_channel_list
static u8 pscan_channel_list[] = {1}; // set by customer
static unsigned short ping_seq = 0;
#if defined(CONFIG_FAST_DHCP) && CONFIG_FAST_DHCP
extern uint32_t offer_ip;
extern uint32_t server_ip;
#endif

static int  wifi_write_ap_info_to_flash_ext(u8 *data, u32 len);
int  wifi_write_ap_info_to_flash(unsigned int offer_ip, unsigned int server_ip);
//ATCMD
static TaskHandle_t roaming_plus_thread_handle = NULL;
static u8 g_roaming_enable = 0;
static u8 g_start_roaming_time = 60;
static int g_duration_roaming_time = 90;
static int g_check_raoming_time = 1;
static int g_active_scan_time = 100;
static int g_passive_scan_time = 110;
static int wlan_fast_connect(struct wifi_roaming_data *data, u8 scan_type)
{
	ROAMING_DBG("%s()", __func__);
	unsigned long tick1 = xTaskGetTickCount();
	unsigned long tick2, tick3, tick4, tick5;

	uint32_t	channel;
	uint32_t    security_type;
	u8 key_id ;
	int ret;
	uint32_t wifi_retry_connect = 3; //For fast wifi connect retry
	rtw_network_info_t wifi = {0};
	struct ap_additional_info store_dhcp_info = {0};
	rtw_wifi_setting_t ap_info = {0};
	struct psk_info PSK_INFO;
	struct ip_addr server_ip_backup;
	u32 serverip_backup;
	u8 *gw;
#if CONFIG_LWIP_LAYER
	netif_set_up(&xnetif[0]);
#endif
	//disable autoreconnect to manually reconnect the specific ap or channel.
#if CONFIG_AUTO_RECONNECT
	wifi_config_autoreconnect(0, 0, 0);
#endif

	memset(&PSK_INFO, 0, sizeof(struct psk_info));
	memcpy(PSK_INFO.psk_essid, data->ap_info.psk_essid, sizeof(data->ap_info.psk_essid));
	memcpy(PSK_INFO.psk_passphrase, data->ap_info.psk_passphrase, sizeof(data->ap_info.psk_passphrase));
	memcpy(PSK_INFO.wpa_global_PSK, data->ap_info.wpa_global_PSK, sizeof(data->ap_info.wpa_global_PSK));
	wifi_psk_info_set(&PSK_INFO);

	channel = data->ap_info.channel;
	key_id = channel >> 28;
	channel &= 0xff;
	security_type = data->ap_info.security_type;

	//set partial scan for entering to listen beacon quickly
WIFI_RETRY_LOOP:
	if (scan_type == FAST_CONNECT_SPECIFIC_CH) {
		wifi.channel = (u8)channel;
		wifi.pscan_option = PSCAN_FAST_SURVEY;
	}
	wifi.security_type = security_type;
	//SSID
	strcpy((char *)wifi.ssid.val, (char *)(data->ap_info.psk_essid));
	wifi.ssid.len = strlen((char *)(data->ap_info.psk_essid));

	switch (security_type) {
	case RTW_SECURITY_WEP_PSK:
		wifi.password = (unsigned char *) data->ap_info.psk_passphrase;
		wifi.password_len = strlen((char *)data->ap_info.psk_passphrase);
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
		wifi.password = (unsigned char *) data->ap_info.psk_passphrase;
		wifi.password_len = strlen((char *)data->ap_info.psk_passphrase);
		break;
	default:
		break;
	}

#if defined(CONFIG_FAST_DHCP) && CONFIG_FAST_DHCP
	offer_ip = data->ap_info.offer_ip;
	server_ip = data->ap_info.server_ip;
#endif
	// 1.connect
	ret = wifi_connect(&wifi, 1);
	tick2 = xTaskGetTickCount();

	if (ret != RTW_SUCCESS) {
		wifi_retry_connect--;
		if (wifi_retry_connect > 0) {
			printf("[Wifi roaming plus]: wifi retry connect\r\n");
			goto WIFI_RETRY_LOOP;
		}
	}
	// 2.dhcp
	if (ret == RTW_SUCCESS) {
		tick4 = xTaskGetTickCount();
#if defined(CONFIG_FAST_DHCP) && CONFIG_FAST_DHCP
		//get offer ip in flash
		if ((data->ap_n < MAX_AP_NUM) && (scan_type == FAST_CONNECT_SPECIFIC_CH)) {
			int i = 0;
			for (i = 0; i < data->ap_n; i++) {
				if (ap_list) {
					if (memcmp(ap_list->bssid, data->add_ap_info[i].ap_bssid, ETH_ALEN) == 0) {
						offer_ip = data->add_ap_info[i].sta_ip;
						server_ip = data->add_ap_info[i].server_ip;
						ROAMING_DBG("\n\r Find the ehter_addr in flash() \n");
						break;
					}
				}
			}
		}
#endif
		LwIP_DHCP(0, DHCP_START);
		tick5 = xTaskGetTickCount();
		ROAMING_DBG("dhcp time %d\n", (tick5 - tick4));
		//clean arp? old arp table may not update.
		etharp_cleanup_netif(&xnetif[0]);

		//store dhcp info for each ap.
		wifi_get_setting(WLAN0_IDX, &ap_info);
		rtw_memcpy(store_dhcp_info.ap_bssid, ap_info.bssid, 6);
		store_dhcp_info.sta_ip = xnetif[0].ip_addr.addr;
		gw = LwIP_GetGW(0);
		IP4_ADDR(ip_2_ip4(&server_ip_backup), gw[0], gw[1], gw[2], gw[3]);
		serverip_backup = ip4_addr_get_u32(ip_2_ip4(&server_ip_backup));
		store_dhcp_info.server_ip = serverip_backup;
		wifi_write_ap_info_to_flash_ext((u8 *)&store_dhcp_info, sizeof(struct ap_additional_info));
	} else {
		ROAMING_DBG("\r\n[Wifi roaming plus]: No need to do dhcp\n");
	}


#if CONFIG_AUTO_RECONNECT
	wifi_config_autoreconnect(2, AUTO_RECONNECT_COUNT, AUTO_RECONNECT_INTERVAL);
#endif
	tick3 = xTaskGetTickCount();
	ROAMING_DBG("\n\r == Roaming connect done  after %d ms = %d ms (connection) + %d ms (DHCP) ==\n", (tick3 - tick1), (tick2 - tick1), (tick3 - tick2));
	return ret;
}

static int  wifi_write_ap_info_to_flash_ext(u8 *data, u32 len)
{
	(void)len;
	flash_t flash;
	u8 n = 0;
	struct wifi_roaming_data read_data = {0};
	u32 tick1 = xTaskGetTickCount();
#if (BAND_STEERING_FUNC == 1)
	int is_black_ap_list = 0;
#endif
	ROAMING_DBG("%s()\n", __FUNCTION__);
	if (!data) {
		return -1;
	}

	memset(&read_data, 0xff, sizeof(struct wifi_roaming_data));
	sys_read_wlan_data_from_flash((uint8_t *) &read_data,  sizeof(struct wifi_roaming_data));

	if (read_data.ap_n == 0xff) {
		read_data.ap_n = 0;
	}

#if (BAND_STEERING_FUNC == 1)
	//blackap_num
	if (read_data.blackap_num == 0xff || read_data.ap_n == 0xff) {
		read_data.blackap_num = 0;
	}
#endif

	if (read_data.ap_n < MAX_AP_NUM) {
		for (n = (read_data.ap_n + 1); n > 0; n--) {
			if (memcmp(read_data.add_ap_info[n - 1].ap_bssid, ((struct ap_additional_info *)data)->ap_bssid, 6) == 0) {
				read_data.add_ap_info[n].sta_ip = ((struct ap_additional_info *)data)->sta_ip;
				read_data.add_ap_info[n].server_ip = ((struct ap_additional_info *)data)->server_ip;
				ROAMING_DBG("[wifi_write_ap_info_to_flash_ext] Have stored this bssid\n");
				break;
			}
		}
#if (BAND_STEERING_FUNC == 1)
		//Check black_ap in flash
		int black_ap_num = read_data.blackap_num;
		if (black_ap_num == 0xff) {
			black_ap_num = 0;
		}
		for (int i = 0; i < black_ap_num; i++) {
			if (memcmp(read_data.blackap_bssid[i], ((struct ap_additional_info *)data)->ap_bssid, 6) == 0) {
				is_black_ap_list = 1;
				BANDSTEER_DBG("[wifi_write_ap_info_to_flash_ext] Find Black_AP in flash, so we don't add Black_AP into flash\n\r");
				break;
			}
		}
		if ((n == 0) && (is_black_ap_list == 0)) {
#else
		if (n == 0) {
#endif
			read_data.ap_n ++;
			memcpy((u8 *)(read_data.add_ap_info[read_data.ap_n - 1].ap_bssid), ((struct ap_additional_info *)data)->ap_bssid, 6);
			read_data.add_ap_info[read_data.ap_n - 1].sta_ip = ((struct ap_additional_info *)data)->sta_ip;
			read_data.add_ap_info[read_data.ap_n - 1].server_ip = ((struct ap_additional_info *)data)->server_ip;

			sys_erase_system_data();

			ROAMING_DBG("[wifi_write_ap_info_to_flash_ext] Add additional AP info into flash\n");
			sys_write_wlan_data_to_flash((uint8_t *)&read_data, sizeof(struct wifi_roaming_data));
		}
	} else {
		ROAMING_DBG("%s(): For more AP infos, Please change MAX_AP_NUM first!\n", __func__);
	}
	u32 tick2 = xTaskGetTickCount();
	ROAMING_DBG("write ap_info_ext to flash [%d]ms\n\r", (tick2 - tick1));
	return 0;
}

int  wifi_write_ap_info_to_flash(unsigned int offer_ip, unsigned int server_ip)
{

	/* To avoid gcc warnings */
#if(!defined(CONFIG_FAST_DHCP) || (!CONFIG_FAST_DHCP))
	(void) offer_ip, server_ip;
#endif
	flash_t flash;
	u8 i = 0;
	struct wifi_roaming_data read_data = {0};
	u8 ap_change = 0;
	u32 tick1 = xTaskGetTickCount();
	struct wlan_fast_reconnect fast_connect_info;
	rtw_wifi_setting_t setting;
	struct psk_info PSK_info;
	u32 channel = 0;

	ROAMING_DBG("%s()\n", __FUNCTION__);

	/* STEP1: get current connect info from wifi driver*/
	if (wifi_get_setting(WLAN0_IDX, &setting) || setting.mode == RTW_MODE_AP) {
		RTW_API_INFO("\r\n %s():wifi_get_setting fail or ap mode", __func__);
		return RTW_ERROR;
	}
	channel = (u32)setting.channel;
	rtw_memset(&fast_connect_info, 0, sizeof(struct wlan_fast_reconnect));

	switch (setting.security_type) {
	case RTW_SECURITY_OPEN:
		rtw_memcpy(fast_connect_info.psk_essid, setting.ssid, strlen((const char *)setting.ssid));
		fast_connect_info.security_type = RTW_SECURITY_OPEN;
		break;
	case RTW_SECURITY_WEP_PSK:
		rtw_memcpy(fast_connect_info.psk_essid, setting.ssid, strlen((const char *)setting.ssid));
		rtw_memcpy(fast_connect_info.psk_passphrase, setting.password, strlen((const char *)setting.password));
		channel |= (setting.key_idx) << 28;
		fast_connect_info.security_type = RTW_SECURITY_WEP_PSK;
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
		rtw_memset(&PSK_info, 0, sizeof(struct psk_info));
		wifi_psk_info_get(&PSK_info);
		rtw_memcpy(fast_connect_info.psk_essid, PSK_info.psk_essid, sizeof(fast_connect_info.psk_essid));
		rtw_memcpy(fast_connect_info.psk_passphrase, PSK_info.psk_passphrase, sizeof(fast_connect_info.psk_passphrase));
		rtw_memcpy(fast_connect_info.wpa_global_PSK, PSK_info.wpa_global_PSK, sizeof(fast_connect_info.wpa_global_PSK));
		fast_connect_info.security_type = setting.security_type;
		break;

	default:
		break;
	}

	rtw_memcpy(&(fast_connect_info.channel), &channel, 4);
#if defined(CONFIG_FAST_DHCP) && CONFIG_FAST_DHCP
	fast_connect_info.offer_ip = offer_ip;
	fast_connect_info.server_ip = server_ip;
#endif


	//Store bssid to flash
	u8 bssid[6];
	if (rtw_wx_get_wap(0, bssid) == 0) {
		//printf("[wifi_write_ap_info_to_flash] "MAC_FMT"\n\r", MAC_ARG(bssid));
		rtw_memcpy(fast_connect_info.bssid, bssid, 6);
	}

	memset(&read_data, 0xff, sizeof(struct wifi_roaming_data));
	sys_read_wlan_data_from_flash((uint8_t *) &read_data,  sizeof(struct wifi_roaming_data));

	if (read_data.num == 0xff) {
		read_data.num = 0;
	}

	//check is common info changed? Is this a new channel?
	//if (read_data.num) {
	/*check if ap info {ssid/password/security_type} has changed*/
	if (memcmp((u8 *)fast_connect_info.psk_essid, (u8 *)read_data.ap_info.psk_essid, 32)) {
		printf("\r\n[Wifi roaming plus]: ap ssid change\n");
		ap_change = 1;
		goto exit;
	} else if (memcmp((u8 *)fast_connect_info.psk_passphrase, (u8 *)(read_data.ap_info.psk_passphrase), 32)) {
		printf("\r\n[Wifi roaming plus]: ap password change\n");
		ap_change = 1;
		goto exit;
	} else if (fast_connect_info.security_type != read_data.ap_info.security_type) {
		printf("\r\n[Wifi roaming plus]: ap security type change\n");
		ap_change = 1;
		goto exit;
	} else { /*ap info doesn't change*/
		for (i = 0; i < read_data.num; i++) {
			if ((read_data.channel[i] == fast_connect_info.channel) && (fast_connect_info.channel == read_data.ap_info.channel)) {
				ROAMING_DBG("Already stored this channel(%d)\n", fast_connect_info.channel);
				return -1;
			}
		}
	}
	//}

#if (BAND_STEERING_FUNC == 1)
	//Check black_ap in flash
	int is_black_ap_list = 0;
	int black_ap_num = read_data.blackap_num;
	if (black_ap_num == 0xff) {
		black_ap_num = 0;
	}
	for (int i = 0; i < black_ap_num; i++) {
		if (memcmp(read_data.blackap_bssid[i], fast_connect_info.bssid, 6) == 0) {
			is_black_ap_list = 1;
			BANDSTEER_DBG("[wifi_write_ap_info_to_flash] Find Black_AP in flash, so we don't add Black_AP info into flash\n\r");
			return 0;
		}
	}
#endif

exit:
	if (ap_change) {
		printf("\r\n[Wifi roaming plus]: erase flash and restore new ap info\n");
		memset((u8 *)&read_data, 0xff, sizeof(struct  wifi_roaming_data));
		read_data.num = 1;
	} else {
		printf("\r\n[Wifi roaming plus]: Add a new channel into flash\n");
		read_data.num++;
	}

	sys_erase_system_data();
	read_data.channel[read_data.num - 1] = fast_connect_info.channel; //store channel
	//only first ap's detail info has to be stored.
	//if (read_data.num >= 1) {
	memcpy((u8 *)&read_data.ap_info, &fast_connect_info, sizeof(struct wlan_fast_reconnect));    //store fast connect info
	//}
	sys_write_wlan_data_to_flash((uint8_t *)&read_data, sizeof(struct wifi_roaming_data));
	u32 tick2 = xTaskGetTickCount();
	ROAMING_DBG("write ap_info into flash [%d]ms\n", (tick2 - tick1));
	return 0;
}

int wifi_init_done_callback_roaming(void)
{
	flash_t flash;
	struct wifi_roaming_data read_data = {0};
	int fast_connection_data = 0;
	memset(&read_data, 0xff, sizeof(struct wifi_roaming_data));
	sys_read_wlan_data_from_flash((uint8_t *) &read_data,  sizeof(struct wifi_roaming_data));
	/* Check whether stored flash profile is empty */
	struct wlan_fast_reconnect *empty_data;
	empty_data = (struct wlan_fast_reconnect *)malloc(sizeof(struct wlan_fast_reconnect));
	if (empty_data) {
		memset(empty_data, 0xff, sizeof(struct wlan_fast_reconnect));
		if (memcmp(empty_data, &read_data, sizeof(struct wlan_fast_reconnect)) == 0) {
			//printf("[FAST_CONNECT] Fast connect profile is empty, abort fast connection\n");
			fast_connection_data = 1;
			//return 0;
		}
		free(empty_data);
	}
	/* Check whether stored flash profile is empty */
	if (((read_data.num == 0) || (read_data.num > MAX_CH_NUM)) && (fast_connection_data)) {
		printf("\r\n[Wifi roaming plus]: Fast connect profile is empty, abort fast connection\n");
	}
	/* Find the best ap in flash profile */
	else {
		ROAMING_DBG("\r\n[Wifi roaming plus]: Connect to the best ap\n");
		wlan_fast_connect(&read_data, FAST_CONNECT_SPECIFIC_CH);
	}
	return 0;
}

void example_wifi_roaming_plus_init(void)
{
	// Call back from wlan driver after wlan init done
	p_wifi_do_fast_connect = wifi_init_done_callback_roaming;

	// Call back from application layer after wifi_connection success
	p_store_fast_connect_info = wifi_write_ap_info_to_flash;
}


static u32 wifi_roaming_plus_find_ap_from_scan_buf(char *target_ssid, void *user_data, int ap_num)
{
	u32 target_security = *(u32 *)user_data;
	rtw_scan_result_t *scanned_ap_info;
	u32 i = 0;
	char *scan_buf = NULL;

	scan_buf = (char *)rtw_zmalloc(ap_num * sizeof(rtw_scan_result_t));
	if (scan_buf == NULL) {
		printf("malloc scan buf for example wifi roaming plus\n");
		return -1;
	}

	if (wifi_get_scan_records((unsigned int *)(&ap_num), scan_buf) < 0) {
		rtw_mfree((u8 *)scan_buf, 0);
		return -1;
	}

#if (BAND_STEERING_FUNC == 1)
	struct wifi_roaming_data roaming_data = {0};
	int is_blackap_list = 0;
	memset(&roaming_data, 0xff, sizeof(struct wifi_roaming_data));

	if (g_blackap_num <= 0 || g_blackap_num >= BLASK_AP_NUM) {
		BANDSTEER_DBG("[wifi_roaming_plus_find_ap_from_scan_buf] read blackap list from ap\n\r");
		sys_read_wlan_data_from_flash((uint8_t *)&roaming_data, sizeof(struct wifi_roaming_data));

		g_blackap_num = roaming_data.blackap_num;

		BANDSTEER_DBG("read from flash g_blackap_num: %d\n\r", g_blackap_num);
		if (g_blackap_num > 0 && g_blackap_num < BLASK_AP_NUM) {
			for (int i = 0; i < g_blackap_num; i++) {
				BANDSTEER_DBG("[wifi_roaming_plus_find_ap_from_scan_buf] read from flash (%d) bssid: "MAC_FMT"\n\r"
							  , i, MAC_ARG(roaming_data.blackap_bssid[i]));
				memcpy(g_blackap_mac[i], roaming_data.blackap_bssid[i], 6);
			}
		}
	}
#endif

	for (i = 0; i < ap_num; i++) {
		scanned_ap_info = (rtw_scan_result_t *)(scan_buf + i * sizeof(rtw_scan_result_t));
		ROAMING_DBG("(i: %d)Scan ap:"MAC_FMT"(%d), rssi: %d\n", i, MAC_ARG(scanned_ap_info->BSSID.octet), scanned_ap_info->channel, scanned_ap_info->signal_strength);
#if (BAND_STEERING_FUNC == 1)
		//Check Black list
		for (int j = 0; j < g_blackap_num ; j++) {
			if (memcmp(scanned_ap_info->BSSID.octet, g_blackap_mac[j], 6) == 0) {
				is_blackap_list = 1;
				BANDSTEER_DBG("[wifi_roaming_plus_find_ap_from_scan_buf] find black ap("MAC_FMT") in flash\n\r", MAC_ARG(g_blackap_mac[j]));
				break;
			}
		}
		if (is_blackap_list == 1) {
			continue;
		}

		BANDSTEER_DBG("[wifi_roaming_plus_find_ap_from_scan_buf] check security and rssi for roaming\n\r");
#endif
		if (target_security == scanned_ap_info->security ||
			((target_security & (WPA2_SECURITY | WPA_SECURITY)) && ((scanned_ap_info->security) & (WPA2_SECURITY | WPA_SECURITY)))) {
			if (ap_list->rssi < scanned_ap_info->signal_strength) {
				ROAMING_DBG("rssi(%d) is better than last(%d)\n", scanned_ap_info->signal_strength, ap_list->rssi);
				memset(ap_list, 0, sizeof(wifi_roaming_ap_t));
				memcpy(ap_list->bssid, scanned_ap_info->BSSID.octet, ETH_ALEN);
				ap_list->channel = scanned_ap_info->channel;
				ap_list->rssi = scanned_ap_info->signal_strength;
			}
		}
	}
	rtw_mfree((u8 *)scan_buf, 0);

	return 0;
}

int wifi_roaming_scan(struct wifi_roaming_data  read_data, u32 retry)
{
	wifi_roaming_ap_t	roaming_ap;
	rtw_wifi_setting_t	setting;
	channel_plan_t channel_plan_temp = roaming_channel_plan;
	u8 ch = 0;
	u8 first_5g = 0;
	int cur_rssi, rssi_delta;
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

	/*scan specific channels*/
	if (0 < read_data.num && read_data.num < MAX_CH_NUM) {
		ROAMING_DBG("\r\n %s():try to find a better ap in flash\n", __func__);
		while (read_data.num) {
			pscan_channel_list[0] = read_data.channel[read_data.num - 1];
			read_data.num--;
			//set scan_param for scan
			rtw_memset(&scan_param, 0, sizeof(rtw_scan_param_t));
			scan_param.ssid = (char *)roaming_ap.ssid;
			scan_param.channel_list = pscan_channel_list;
			scan_param.channel_list_num = 1;
			scan_param.chan_scan_time.active_scan_time = g_active_scan_time;
			scan_param.chan_scan_time.passive_scan_time = g_passive_scan_time;
			ROAMING_DBG("scan(%d)\n", pscan_channel_list[0]);
			scanned_ap_num = wifi_scan_networks(&scan_param, 1);
			if (scanned_ap_num > 0) {
				wifi_roaming_plus_find_ap_from_scan_buf((char *)roaming_ap.ssid, (void *)&roaming_ap.security_type, scanned_ap_num);
			}
			//	(void *)&target_security, SCAN_BUFLEN, (char*)read_data.ap_info.psk_essid, strlen((char const*)read_data.ap_info.psk_essid));
			ROAMING_DBG("scan done(%d)\n", pscan_channel_list[0]);
			for (ch = 0 ; ch < channel_plan_temp.len; ch++) {
				if (channel_plan_temp.channel[ch] == pscan_channel_list[0]) {
					channel_plan_temp.channel[ch] = 0;//skip scan later
					break;
				}
			}
			wifi_fetch_phy_statistic(&phy_statistics);
			cur_rssi = phy_statistics.rssi;
			rssi_delta = FIND_BETTER_RSSI_DELTA;
			if (ap_list->rssi - cur_rssi > rssi_delta && (memcmp(roaming_ap.bssid, ap_list->bssid, ETH_ALEN))) {
				printf("\r\n[Wifi roaming plus]: Find a better ap in flash successful, rssi = %d, cur_rssi=%d\n", ap_list->rssi, cur_rssi);
				return 1;
			}
			vTaskDelay(500);
		}
	}

	/*scan other channels*/
	ROAMING_DBG("\r\n %s():Find the best ap in flash fail, rssi = %d, try to find in other channels\n", __func__, ap_list->rssi);
#if SUPPORT_SCAN_5G_CHANNEL
	if (xTaskGetTickCount() % 2) {
		first_5g = 1;  //force 5g first
		ROAMING_DBG("scan 5g first\n");
	} else {
		first_5g = 0;
		ROAMING_DBG("scan 2.4g first\n");
	}

	for (ch = 0 ; ch < channel_plan_temp.len; ch++) {
		if ((first_5g && (channel_plan_temp.channel[ch] > 15))
			|| (!first_5g && (channel_plan_temp.channel[ch] > 0 &&  channel_plan_temp.channel[ch] < 15))) {
			pscan_channel_list[0] = channel_plan_temp.channel[ch];
			ROAMING_DBG("scan(%d)\n", pscan_channel_list[0]);
			//set scan_param for scan
			rtw_memset(&scan_param, 0, sizeof(rtw_scan_param_t));
			scan_param.ssid = (char *)roaming_ap.ssid;
			scan_param.channel_list = pscan_channel_list;
			scan_param.channel_list_num = 1;
			scan_param.chan_scan_time.active_scan_time = g_active_scan_time;
			scan_param.chan_scan_time.passive_scan_time = g_passive_scan_time;
			scanned_ap_num = wifi_scan_networks(&scan_param, 1);
			if (scanned_ap_num > 0) {
				wifi_roaming_plus_find_ap_from_scan_buf((char *)roaming_ap.ssid, (void *)&roaming_ap.security_type, scanned_ap_num);
			}
			ROAMING_DBG("scan(%d) done!\n", pscan_channel_list[0]);
			channel_plan_temp.channel[ch] = 0;
			wifi_fetch_phy_statistic(&phy_statistics);
			cur_rssi = phy_statistics.rssi;
			rssi_delta = FIND_BETTER_RSSI_DELTA;
			if (ap_list->rssi - cur_rssi > rssi_delta && (memcmp(roaming_ap.bssid, ap_list->bssid, ETH_ALEN))) {
				printf("\r\n[Wifi roaming plus]: Find a better ap on channel %d, rssi = %d, cur_rssi=%d\n", ap_list->channel, ap_list->rssi, cur_rssi);
				return 1;
			}
			vTaskDelay(500);
		}
	}
#endif
	for (ch = 0 ; ch < channel_plan_temp.len; ch++) {
		if (channel_plan_temp.channel[ch]) {
			pscan_channel_list[0] = channel_plan_temp.channel[ch];
			ROAMING_DBG("scan(%d)\n", pscan_channel_list[0]);
			//set scan_param for scan
			rtw_memset(&scan_param, 0, sizeof(rtw_scan_param_t));
			scan_param.ssid = (char *)roaming_ap.ssid;
			scan_param.channel_list = pscan_channel_list;
			scan_param.channel_list_num = 1;
			scan_param.chan_scan_time.active_scan_time = g_active_scan_time;
			scan_param.chan_scan_time.passive_scan_time = g_passive_scan_time;
			scanned_ap_num = wifi_scan_networks(&scan_param, 1);
			if (scanned_ap_num > 0) {
				wifi_roaming_plus_find_ap_from_scan_buf((char *)roaming_ap.ssid, (void *)&roaming_ap.security_type, scanned_ap_num);
			}
			ROAMING_DBG("scan(%d) done!\n", pscan_channel_list[0]);
			channel_plan_temp.channel[ch] = 0;
			wifi_fetch_phy_statistic(&phy_statistics);
			cur_rssi = phy_statistics.rssi;
			rssi_delta = FIND_BETTER_RSSI_DELTA;
			if (ap_list->rssi - cur_rssi > rssi_delta && (memcmp(roaming_ap.bssid, ap_list->bssid, ETH_ALEN))) {
				printf("\r\n[Wifi roaming plus]: Find a better ap on channel %d, rssi = %d, cur_rssi=%d\n", ap_list->channel, ap_list->rssi, cur_rssi);
				return 1;
			}
			vTaskDelay(500);
		}
	}
	printf("\r\n[Wifi roaming plus]: Find a better ap fail,retry:%d!\n", retry);
	return 0;
}


void wifi_roaming_plus_thread(void *param)
{
	ROAMING_DBG("\n %s()\n", __func__);
	unsigned long tick1;
	unsigned long tick2;
	unsigned long tick_diff;
	(void)param;
	signed char ap_rssi;
	rtw_phy_statistics_t phy_statistics;
	u32 scan_retry = 0;
	u32	polling_count = 0;
	struct wifi_roaming_data read_data = {0};
	u8 rate = 0;

#if (BAND_STEERING_FUNC == 1)
	//Init disconnection callback for bandsteering
	init_event_callback_list();
	wifi_reg_event_handler(WIFI_EVENT_DISCONNECT, wifi_disconnect_hdl, NULL);
#endif

	vTaskDelay(g_start_roaming_time * configTICK_RATE_HZ);
	printf("\n Example: example_wifi_roaming_plus task start\n", __func__);
	tick1 = xTaskGetTickCount();
	while (1) {
		if (wifi_is_running(WLAN0_IDX) && ((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) && (*(u32 *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
			wifi_fetch_phy_statistic(&phy_statistics);
			ap_rssi = phy_statistics.rssi;
			wifi_get_sta_max_data_rate(&rate);
			printf("\r\n %s():Current rssi(%d),scan threshold rssi(%d); data_rate: (%d)\n", __func__, ap_rssi, RSSI_SCAN_THRESHOLD, rate);
			if (ap_rssi < RSSI_SCAN_THRESHOLD) {
				if ((polling_count >= MAX_POLLING_TIME) && (scan_retry < MAX_SCAN_TIME)) {
					printf("\r\n[Wifi roaming plus]: Start scan, current rssi(%d) < scan threshold rssi(%d) \n", ap_rssi, RSSI_SCAN_THRESHOLD);
					ap_list = (wifi_roaming_ap_t *)malloc(sizeof(wifi_roaming_ap_t));
					memset(ap_list, 0, sizeof(wifi_roaming_ap_t));
					ap_list->rssi = -100;
					memset(&read_data, 0xff, sizeof(struct wifi_roaming_data));
					sys_read_wlan_data_from_flash((uint8_t *) &read_data,  sizeof(struct wifi_roaming_data));
					/*1.find a better ap*/
					if (wifi_roaming_scan(read_data, scan_retry)) {
						scan_retry = 0;
						if (wifi_is_running(WLAN0_IDX) && ((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) && (*(u32 *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
							wifi_fetch_phy_statistic(&phy_statistics);
							ap_rssi = phy_statistics.rssi;
							if (ap_rssi > RSSI_SCAN_THRESHOLD + 5) {
								/*no need to roaming*/
								printf("\r\n[Wifi roaming plus]: Cancel roaming, current rssi=%d\n", ap_rssi);
							} else {
								/*2.connect a better ap*/
								printf("\r\n[Wifi roaming plus]: Start roaming, current rssi(%d) < roaming threshold rssi(%d),target ap rssi(%d)\n", \
									   ap_rssi, RSSI_SCAN_THRESHOLD, ap_list->rssi);
								read_data.ap_info.channel = ap_list->channel;
								wlan_fast_connect(&read_data, FAST_CONNECT_SPECIFIC_CH);
							}
						}
					} else { //scan fail
						scan_retry++;
					}
					free(ap_list);
					polling_count = 0;
				} else {
					polling_count++;
				}
			} else {
				polling_count = 0;
				scan_retry = 0;
			}
		}

		//Over the max scan time, so we need to stop the roaming task
		if (scan_retry >= MAX_SCAN_TIME) {
			printf("\n\rRoaming (%d > %d) task stop\n\r", scan_retry, MAX_SCAN_TIME);
			g_roaming_enable = 0;
			roaming_plus_thread_handle = NULL;
			vTaskDelete(NULL);
		}

		//Check wifi roaming duration time
		tick2 = xTaskGetTickCount();
		tick_diff = tick2 - tick1;
		if (tick_diff > ((g_duration_roaming_time - g_check_raoming_time) * configTICK_RATE_HZ)) {
			printf("\n\rRoaming (%d > %d) task stop\n\r"
				   , tick_diff, ((g_duration_roaming_time - g_check_raoming_time) * configTICK_RATE_HZ));
			g_roaming_enable = 0;
			roaming_plus_thread_handle = NULL;
			vTaskDelete(NULL);
		}
		//Idle
		vTaskDelay(g_check_raoming_time * configTICK_RATE_HZ);
	}

}

#if 1
void wifi_stop_roaming_task(void)
{

	if (roaming_plus_thread_handle) {
		vTaskDelete(roaming_plus_thread_handle);
		roaming_plus_thread_handle = NULL;
		g_roaming_enable = 0;
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

void wifi_set_roaming_max_count(int max_scan_time)
{
	MAX_SCAN_TIME = max_scan_time;
}

void wifi_set_roaming_rssi_threshold(int rssi_scan_threshold)
{
	RSSI_SCAN_THRESHOLD = rssi_scan_threshold;
}

void wifi_set_roaming_partial_scan_time(int active_scan_time, int passive_scan_time)
{
	g_active_scan_time = active_scan_time;
	g_passive_scan_time = passive_scan_time;
}

void print_ROAMING_help(void)
{
	printf("ROAM=[enable|disable],[options]\r\n");

	printf("\r\n");
	printf("ROAM=startup_time,30\r\n");
	printf("\tSetting roaming start time after 30s\r\n");

	printf("\r\n");
	printf("ROAM=duration_time,30\r\n");
	printf("\tSetting roaming duarion time after 30s and then stop roaming\r\n");

	printf("\r\n");
	printf("ROAM=roam_max_count,6\r\n");
	printf("\tSetting roaming max scan count after 6 and then stop roaming\r\n");

	printf("\r\n");
	printf("ROAM=rssi,-80\r\n");
	printf("\tSetting roaming scan rssi threshold -80\r\n");

	printf("\r\n");
	printf("ROAM=partial_scan,80,90\r\n");
	printf("\tSetting roaming partial_scan about active scan: 80 and  passive scan: 90\r\n");
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
			printf("\n\rEnable wifi roaming task\n\r");
			if (roaming_plus_thread_handle == NULL) {
				if (xTaskCreate(wifi_roaming_plus_thread, ((const char *)"wifi_roaming_thread"), 1280, NULL, tskIDLE_PRIORITY + 1, &roaming_plus_thread_handle) != pdPASS) {
					printf("\n\r%s xTaskCreate(wifi_roaming_thread) failed", __FUNCTION__);
				} else {
					g_roaming_enable = 1;
				}
			} else {
				printf("\n\r%s xTaskCreate(wifi_roaming_thread) exit", __FUNCTION__);
			}
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
				printf("\n\rMAX_SCAN_TIME=%02d", atoi(argv[2]));
				int max_count = atoi(argv[2]);
				wifi_set_roaming_max_count(max_count);
			}
		} else if (strcmp(argv[1], "rssi") == 0) {
			if (argc == 3) {
				printf("\n\rRSSI_SCAN_THRESHOLD=%02d", atoi(argv[2]));
				int rssi_threshold = atoi(argv[2]);
				wifi_set_roaming_rssi_threshold(rssi_threshold);
			}
		} else if (strcmp(argv[1], "partial_scan") == 0) {
			if (argc == 4) {
				printf("\n\rg_active_scan_time=%02d, g_passive_scan_time=%02d", atoi(argv[2]), atoi(argv[3]));
				int active_scan_time = atoi(argv[2]);
				int passive_scan_time = atoi(argv[3]);
				wifi_set_roaming_partial_scan_time(active_scan_time, passive_scan_time);
			}
		} else if (strcmp(argv[1], "debug") == 0) {
			if (argc == 3) {
				printf("\n\rROAMING_PLUS_DBG=%02d", atoi(argv[2]));
				ROAMING_PLUS_DBG = atoi(argv[2]);
			}
		} else if (strcmp(argv[1], "read_flash") == 0) {
			u8 *read_data = NULL;
			int k = 0;
			sys_read_wlan_data_from_flash((uint8_t *) read_data,  sizeof(struct wifi_roaming_data));
			printf("\n\rRead size: %d\n\r", sizeof(struct wifi_roaming_data));
			for (int i = 0; i < sizeof(struct wifi_roaming_data); i++) {
				k++;
				if (i == sizeof(struct wlan_fast_reconnect)) {
					printf("\n\r--------------------end--------------------\n\r");
				}

				printf("%02x ", read_data[i]);

				if (k % 16 == 0) {
					printf("\n\r");
				}
			}
		}
	} while (0);
}
log_item_t at_roaming_items[ ] = {
	{"ROAM", fROAM,},
};
#endif
void example_wifi_roaming_plus(void)
{
	log_service_add_table(at_roaming_items, sizeof(at_roaming_items) / sizeof(at_roaming_items[0]));
	if (roaming_plus_thread_handle == NULL) {
		if (xTaskCreate(wifi_roaming_plus_thread, ((const char *)"wifi_roaming_thread"), 1280, NULL, tskIDLE_PRIORITY + 1, &roaming_plus_thread_handle) != pdPASS) {
			printf("\n\r%s xTaskCreate(wifi_roaming_thread) failed", __FUNCTION__);
		} else {
			g_roaming_enable = 1;
		}
	} else {
		printf("\n\r%s xTaskCreate(wifi_roaming_thread) exit", __FUNCTION__);
	}

	return;
}

#if (BAND_STEERING_FUNC == 1)
#define IPADDR_NULL          ((u32_t)0xFFFFFFFFUL)
#define CHANNEL_NULL          ((u32_t)0xFFFFFFFFUL)
static int  wifi_write_black_ap_info_to_flash(u8 *data, int is_black_mac, u32 channel, u8 *blackap_bssid)
{
	flash_t flash;
	u8 n = 0;
	struct wifi_roaming_data read_data = {0};
	char empty_bssid[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

	if (!data) {
		return -1;
	}

	memset(&read_data, 0xff, sizeof(struct wifi_roaming_data));
	sys_read_wlan_data_from_flash((uint8_t *) &read_data,  sizeof(struct wifi_roaming_data));

	if (read_data.ap_n == 0xff) {
		BANDSTEER_DBG("%s() return\n", __FUNCTION__);
		return 0;
	}

	if (read_data.blackap_num < MAX_AP_NUM) {
		BANDSTEER_DBG("[wifi_write_black_ap_info_to_flash]-------------check channel-------------\n\r");
		for (n = (read_data.num + 1); n > 0; n--) {
			//BANDSTEER_DBG("[wifi_write_black_ap_info_to_flash] (n: %d) (read_chl: %d, chl: %d)\n\r",n,read_data.channel[n - 1],channel);
			if (read_data.channel[n - 1] == channel) {
				BANDSTEER_DBG("[wifi_write_black_ap_info_to_flash] n: %d: same channel: %d\n\r", n, channel);
				read_data.channel[n - 1] = CHANNEL_NULL;
				break;
			}
		}

		BANDSTEER_DBG("[wifi_write_black_ap_info_to_flash]--------------check bssid--------------\n\r");
		for (n = (read_data.ap_n + 1); n > 0; n--) {
			//BANDSTEER_DBG("[wifi_write_black_ap_info_to_flash] n: %d\n\r",n);
			if (memcmp(read_data.add_ap_info[n - 1].ap_bssid, blackap_bssid, 6) == 0) {
				BANDSTEER_DBG("[wifi_write_black_ap_info_to_flash] n: %d: bssid: "MAC_FMT", blackap: "MAC_FMT"\n\r"
							  , n, MAC_ARG(read_data.add_ap_info[n - 1].ap_bssid), MAC_ARG(blackap_bssid));
				//number
				read_data.ap_n --;
				read_data.num--;
				//BANDSTEER_DBG("[wifi_write_black_ap_info_to_flash] (after) ap_n: %d, blackap_num: %d\n\r",read_data.ap_n, read_data.blackap_num);
				memcpy((u8 *)(read_data.add_ap_info[n - 1].ap_bssid), empty_bssid, 6);
				read_data.add_ap_info[n - 1].sta_ip = IPADDR_NULL;
				read_data.add_ap_info[n - 1].server_ip = IPADDR_NULL;

				//add black ap
				if (is_black_mac == 0) {
					BANDSTEER_DBG("[wifi_write_black_ap_info_to_flash] add black list to flash\n\r");
					memcpy(read_data.blackap_bssid[read_data.blackap_num], blackap_bssid, 6);
					read_data.blackap_num++;
				}
				BANDSTEER_DBG("[wifi_write_black_ap_info_to_flash] Add additional BLACK AP info into flash\n\r");
				sys_erase_system_data();
				sys_write_wlan_data_to_flash((uint8_t *)&read_data, sizeof(struct wifi_roaming_data));
				break;
			}
		}


	} else {
		BANDSTEER_DBG("%s(): For more AP infos, Please change MAX_AP_NUM first!\n", __func__);
	}

	return 0;
}

int bandsteering_get_flash_bssid(u8 *mac)
{
	int ret = 0;
	struct wifi_roaming_data read_roaming_data = {0};
	struct wlan_fast_reconnect read_fast_connect_data = {0};
	struct wlan_fast_reconnect *empty_data;
	//Empty bssid
	char empty_bssid[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	char empty_bssid_null[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	memset(&read_fast_connect_data, 0xff, sizeof(struct wlan_fast_reconnect));
	sys_read_wlan_data_from_flash((uint8_t *) &read_fast_connect_data,  sizeof(struct wlan_fast_reconnect));
	empty_data = (struct wlan_fast_reconnect *)malloc(sizeof(struct wlan_fast_reconnect));
	if (empty_data) {
		memset(empty_data, 0xff, sizeof(struct wlan_fast_reconnect));
		/* Check whether stored flash profile is empty */
		if (memcmp(empty_data, &read_fast_connect_data, sizeof(struct wlan_fast_reconnect)) == 0) {
			free(empty_data);
			return -1;
		}
		free(empty_data);
	}

	memset(&read_roaming_data, 0xff, sizeof(struct wifi_roaming_data));
	sys_read_wlan_data_from_flash((uint8_t *) &read_roaming_data,  sizeof(struct wlan_fast_reconnect));
	if (memcmp(read_fast_connect_data.bssid, empty_bssid, 6) && memcmp(read_fast_connect_data.bssid, empty_bssid_null, 6)) {
		rtw_memcpy(mac, read_fast_connect_data.bssid, 6);
		//BANDSTEER_DBG("[bandsteering_get_flash_bssid] "MAC_FMT"\n\r", MAC_ARG(mac));
		ret = 0;
	} else {
		//BANDSTEER_DBG("[bandsteering_task] NULL\n\r");
		ret = -1;
	}
	return ret;

}

static void wifi_disconnect_hdl(char *buf, int buf_len, int flags, void *userdata)
{
	u16 disconn_reason;
	disconn_reason = *(u16 *)(buf + 6);
	if (disconn_reason == 65535) {
		BANDSTEER_DBG("[wifi_disconnect_hdl] no beacon\n\r");
	} else {
		// ap disconnect reason code
		if (disconn_reason > 0) {
			g_deauth_cnt++;
			//Record rx disconnect mac
			u8 flash_bssid[ETH_ALEN];
			int ret = 0;
			int disc_index = g_deauth_cnt - 1;
			BANDSTEER_DBG("-------------------------(Receive deauth)---------------------------------------\n\r");
			BANDSTEER_DBG("[wifi_disconnect_hdl] g_deauth_cnt: %d, disc_index: %d\n\r", g_deauth_cnt, disc_index);
			if ((disc_index >= 0) && (disc_index < RX_DISCONNECT_MAX_CNT)) {
				ret = bandsteering_get_flash_bssid(flash_bssid);
				if (ret == 0) {
					memcpy(g_ap_mac[disc_index], flash_bssid, 6);
					//BANDSTEER_DBG("[bandsteering_task] Flash_bssid: "MAC_FMT"\n\r",MAC_ARG(flash_bssid));
				}
			}
			if (g_deauth_cnt >= RX_DISCONNECT_MAX_CNT) {
				BANDSTEER_DBG("[wifi_disconnect_hdl] bandsteering_task start\n\r");
				if (bandsteering_thread_handle == NULL) {
					if (xTaskCreate(bandsteering_task, ((const char *)"bandsteering_task"), 4096, NULL, tskIDLE_PRIORITY + 2, &bandsteering_thread_handle) != pdPASS) {
						BANDSTEER_DBG("\n\r%s xTaskCreate(bandsteering_task) failed\n", __FUNCTION__);
					}
				}
			}
		}
	}
}

void example_wifi_roaming_enable(u8 enable)
{
	if (enable == 1) {
		// Call back from wlan driver after wlan init done
		p_wifi_do_fast_connect = wifi_init_done_callback_roaming;

		// Call back from application layer after wifi_connection success
		p_store_fast_connect_info = wifi_write_ap_info_to_flash;
	} else if (enable == 2) {
		// Call back from wlan driver after wlan init done
		p_wifi_do_fast_connect = wifi_init_done_callback_roaming;

		// p_store_fast_connect_info null
		p_store_fast_connect_info = NULL;
	}
}

void bandsteering_task(void *param)
{
	int timeout = 60;
	int timeout_cnt = 0;
	int disc_index = g_deauth_cnt - 1;
	u8 flash_bssid[ETH_ALEN];
	int ret = -1;
	//struct wlan_fast_reconnect *data= NULL;
	rtw_network_info_t wifi_flash_info = {0};
	uint32_t    security_type;
	int scan_retry = bandsteering_retry_scan;
	int connection_retry = bandsteering_retry_connect;
	rtw_wifi_setting_t setting;
	//int cur_channel = setting.channel;
	u8 *channel_list = NULL;
	unsigned int scanned_AP_num = 0;
	int num_channel = 0;
	rtw_scan_param_t scan_param;
	rtw_scan_result_t *scanned_ap_info;
	rtw_scan_result_t *target_ap = NULL;
	int i = 0;
	char *scan_buf = NULL;
	int switch_index = -1;
	int is_black_list = 0;
	rtw_network_info_t wifi = {0};
	unsigned char cur_apmac[ETH_ALEN];
	u32	channel;
	struct ap_additional_info store_blackap_info = {0};
	//store dhcp info for each ap after bandsteering connection.
	rtw_wifi_setting_t ap_info = {0};
	struct ap_additional_info store_dhcp_info = {0};
	u8 *gw;
	struct ip_addr server_ip_backup;
	u32 serverip_backup;

	ret = bandsteering_get_flash_bssid(flash_bssid);
	wifi_config_autoreconnect(RTW_AUTORECONNECT_DISABLE, 0, 0);
	BANDSTEER_DBG("-----------------------(Rx_disconnect list)-------------------------------------\n\r");
	//Check continuous rx_disconnect from same ap
	int is_same_rx_deauth = 1;
	BANDSTEER_DBG("g_ap_mac num: flash_bssid: "MAC_FMT"\n\r", MAC_ARG(flash_bssid));
	if (ret == 0) {
		for (int j = 0; j < RX_DISCONNECT_MAX_CNT ; j++) {
			BANDSTEER_DBG("(j: %d) (compare) bssid: "MAC_FMT" memp: %d\n\r", j, MAC_ARG(g_ap_mac[j]), memcmp(flash_bssid, g_ap_mac[j], ETH_ALEN));
			if (memcmp(flash_bssid, g_ap_mac[j], ETH_ALEN)) {
				BANDSTEER_DBG("Differ rx disassoc mac\n\r");
				is_same_rx_deauth = 0;
			}
		}
	} else {
		is_same_rx_deauth = 0;
	}

	if (is_same_rx_deauth == 1) {
		BANDSTEER_DBG("[bandsteering_task] Going (%d)!!!!!\n\r", g_deauth_cnt);
		g_deauth_cnt = 0;
	} else {
		BANDSTEER_DBG("[bandsteering_task] rx deauth isn't continuous\n\r");
		g_deauth_cnt = 0;
		ret = -1;
		goto exit;
	}

	//AP info from flash
	struct wlan_fast_reconnect data = {0};
	memset(&data, 0xff, sizeof(struct wlan_fast_reconnect));
	sys_read_wlan_data_from_flash((uint8_t *)&data, sizeof(struct wlan_fast_reconnect));

	/* Check whether stored flash profile is empty */
	struct wlan_fast_reconnect *empty_data;
	empty_data = (struct wlan_fast_reconnect *)malloc(sizeof(struct wlan_fast_reconnect));
	if (empty_data) {
		memset(empty_data, 0xff, sizeof(struct wlan_fast_reconnect));
		if (memcmp(empty_data, &data, sizeof(struct wlan_fast_reconnect)) == 0) {
			BANDSTEER_DBG("[bandsteering_task] Fast connect profile is empty, abort bandsteering\n");
			free(empty_data);
			bandsteering_thread_handle = NULL;
			vTaskDelete(NULL);
		}
		free(empty_data);
	}

	//SSID
	strcpy((char *)wifi_flash_info.ssid.val, (char *)(data.psk_essid));
	wifi_flash_info.ssid.len = strlen((char *)(data.psk_essid));

	//BSSID
	char empty_bssid[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	if (memcmp(data.bssid, empty_bssid, 6)) {
		rtw_memcpy(wifi_flash_info.bssid.octet, data.bssid, 6);
		rtw_memcpy(flash_bssid, data.bssid, 6);
		//BANDSTEER_DBG("[bandsteering_task] (read flash) "MAC_FMT"\n\r", MAC_ARG(wifi_flash_info.bssid.octet));
	} else {
		//BANDSTEER_DBG("[bandsteering_task] NULL\n\r");
		ret = -1;
		goto exit;
	}

	//Channel
	channel = data.channel;
	channel &= 0xff;

	//Security
	security_type = data.security_type;
	wifi_flash_info.security_type = security_type;

	//PASSOWRD
	switch (security_type) {
	case RTW_SECURITY_WPA_TKIP_PSK:
	case RTW_SECURITY_WPA_AES_PSK:
	case RTW_SECURITY_WPA_MIXED_PSK:
	case RTW_SECURITY_WPA2_AES_PSK:
	case RTW_SECURITY_WPA2_TKIP_PSK:
	case RTW_SECURITY_WPA2_MIXED_PSK:
	case RTW_SECURITY_WPA_WPA2_TKIP_PSK:
	case RTW_SECURITY_WPA_WPA2_AES_PSK:
	case RTW_SECURITY_WPA_WPA2_MIXED_PSK:
#ifdef CONFIG_SAE_SUPPORT
	case RTW_SECURITY_WPA3_AES_PSK:
	case RTW_SECURITY_WPA3_GCMP_PSK:
	case RTW_SECURITY_WPA2_WPA3_MIXED:
#endif
		wifi_flash_info.password = (unsigned char *)(data.psk_passphrase);
		wifi_flash_info.password_len = strlen((char *)(data.psk_passphrase));
		break;
	default:
		break;
	}

	BANDSTEER_DBG("---------------------------(Black list)-----------------------------------------\n\r");
	//Read from flash about black ap
	struct wifi_roaming_data roaming_data = {0};

	memset(&roaming_data, 0xff, sizeof(struct wifi_roaming_data));
	sys_read_wlan_data_from_flash((uint8_t *)&roaming_data, sizeof(struct wifi_roaming_data));

	g_blackap_num = roaming_data.blackap_num;
	if (g_blackap_num == 0xff) {
		g_blackap_num = 0;
	}

	BANDSTEER_DBG("read from flash g_blackap_num: %d\n\r", g_blackap_num);
	if (g_blackap_num > 0 && g_blackap_num < BLASK_AP_NUM) {
		for (int i = 0; i < g_blackap_num; i++) {
			BANDSTEER_DBG("read from flash (%d) bssid: "MAC_FMT"\n\r"
						  , i, MAC_ARG(roaming_data.blackap_bssid[i]));
			memcpy(g_blackap_mac[i], roaming_data.blackap_bssid[i], 6);
		}
	}


	//add black list
	int index = g_blackap_num;
	int is_black_mac = 0;
	if (g_blackap_num < BLASK_AP_NUM) {
		if (index == 0) {
			memcpy(g_blackap_mac[index], flash_bssid, ETH_ALEN);
			g_blackap_num = g_blackap_num + 1;
		} else {
			for (int j = 0; j < g_blackap_num; j++) {
				if (memcmp(flash_bssid, g_blackap_mac[j], ETH_ALEN) == 0) {
					BANDSTEER_DBG("find it is_black_mac dont add\n\r");
					is_black_mac = 1;
				}
			}
			if (is_black_mac == 0) {
				memcpy(g_blackap_mac[index], flash_bssid, ETH_ALEN);
				g_blackap_num = g_blackap_num + 1;
			}
		}
	}

	//write black list to flash
	wifi_write_black_ap_info_to_flash((u8 *)&store_blackap_info, is_black_mac, channel, flash_bssid);

	//printf black list for debug
	BANDSTEER_DBG("\n\r[bandsteering_task] Black list: \n\r");
	for (int k = 0 ; k < g_blackap_num ; k++) {
		printf("(%d) bssid: "MAC_FMT"\n\r", k, MAC_ARG(g_blackap_mac[k]));
	}

	//wifi scan
	BANDSTEER_DBG("---------------------------(Scanning)-------------------------------------------\n\r");
	rtw_memset(&scan_param, 0, sizeof(rtw_scan_param_t));
	scan_param.ssid = (char *)wifi_flash_info.ssid.val;
	//scan_param.scan_user_callback = print_ssid_scan_result_test;
	scan_param.channel_list = channel_list;
	scan_param.channel_list_num = num_channel;

SCAN_RETRY_LOOP:
	scanned_AP_num = wifi_scan_networks(&scan_param, 1);

	if (scanned_AP_num <= 0) {
		scan_retry--;
		if (scan_retry > 0) {
			BANDSTEER_DBG("wifi_scan_networks retry\n\r\n");
			vTaskDelay(300);
			goto SCAN_RETRY_LOOP;
		}

		BANDSTEER_DBG("wifi_scan_networks null\n\r");
		ret = -1;
		goto exit;
	}

	BANDSTEER_DBG("[bandsteering_task] wifi_scan_networks scanned_AP_num: %d\n\r", scanned_AP_num);
	scan_buf = (char *)rtw_zmalloc(scanned_AP_num * sizeof(rtw_scan_result_t));
	if (scan_buf == NULL) {
		BANDSTEER_DBG("malloc scan buf for ATPN fail\n\r");
		goto exit;
	}

	if (wifi_get_scan_records(&scanned_AP_num, scan_buf) < 0) {
		rtw_mfree((u8 *)scan_buf, 0);
		ret = -1;
		goto exit;
	}

	//Debug messenge
	BANDSTEER_DBG("[bandsteering_task] scanned_AP_num: %d\n\r", scanned_AP_num);
	for (i = 0; i < scanned_AP_num; i++) {
		//printf("\n\r__print_ssid_scan_result_test__ i: %d\n\r",i);
		scanned_ap_info = (rtw_scan_result_t *)(scan_buf + i * sizeof(rtw_scan_result_t));
		scanned_ap_info->SSID.val[scanned_ap_info->SSID.len] = 0; /* Ensure the SSID is null terminated */
		BANDSTEER_DBG(MAC_FMT, MAC_ARG(scanned_ap_info->BSSID.octet));
		BANDSTEER_DBG(" %d\t ", scanned_ap_info->signal_strength);
		BANDSTEER_DBG(" %d\t  ", scanned_ap_info->channel);
		BANDSTEER_DBG("\n\r");
	}

	BANDSTEER_DBG("---------------------------(Connection)-----------------------------------------\n\r");

	//Select switch band for target ap
	for (i = 0; i < scanned_AP_num; i++) {
		scanned_ap_info = (rtw_scan_result_t *)(scan_buf + i * sizeof(rtw_scan_result_t));
		scanned_ap_info->SSID.val[scanned_ap_info->SSID.len] = 0; /* Ensure the SSID is null terminated */
		is_black_list = 0;
		BANDSTEER_DBG("__g_blackap_num__ %d\n\r", g_blackap_num);
		for (int j = 0; j < g_blackap_num; j++) {
			BANDSTEER_DBG("(%d) mac: %02x:%02x:%02x:%02x:%02x:%02x\n\r", j,
						  g_blackap_mac[j][0], g_blackap_mac[j][1], g_blackap_mac[j][2],
						  g_blackap_mac[j][3], g_blackap_mac[j][4], g_blackap_mac[j][5]);
			if (memcmp(scanned_ap_info->BSSID.octet, g_blackap_mac[j], ETH_ALEN) == 0) {
				BANDSTEER_DBG("find it is_black_list\n\r");
				is_black_list = 1;
			}
		}
		if (is_black_list == 0) {
			switch_index = i;
			BANDSTEER_DBG("find it find it find it, i: %d\n\r", i);
			break;
		}

	}

	if (switch_index == -1) {
		BANDSTEER_DBG("[bandsteering_task] Don't find target ap to switch\n\r");
		ret = -1;
		goto exit;
	}

	BANDSTEER_DBG("__bandsteering_task__ switch_index: %d\n\r", switch_index);
	if (switch_index < scanned_AP_num) {
		target_ap = (rtw_scan_result_t *)(scan_buf + switch_index * sizeof(rtw_scan_result_t));
		target_ap->SSID.val[target_ap->SSID.len] = 0; /* Ensure the SSID is null terminated */
		BANDSTEER_DBG(MAC_FMT, MAC_ARG(target_ap->BSSID.octet));
		BANDSTEER_DBG(" %d\t ", target_ap->signal_strength);
		BANDSTEER_DBG(" %d\t  ", target_ap->channel);
	}
	BANDSTEER_DBG("\n\r");

	wifi.channel = target_ap->channel;
	memcpy(wifi.bssid.octet, target_ap->BSSID.octet, 6);
	wifi.security_type = wifi_flash_info.security_type;

	strcpy((char *)wifi.ssid.val, (char *)(wifi_flash_info.ssid.val));
	wifi.ssid.len = wifi_flash_info.ssid.len;
	wifi.password = wifi_flash_info.password;
	wifi.password_len = wifi_flash_info.password_len;

#if CONFIG_AUTO_RECONNECT
	//setup reconnection flag
	wifi_config_autoreconnect(1, AUTO_RECONNECT_COUNT, AUTO_RECONNECT_INTERVAL);
#endif

CONNECTION_RETRY_LOOP:
	BANDSTEER_DBG("[bandsteering_task] (connection) ssid: %s(len: %d), bssid: "MAC_FMT", pwd: %s(len: %d), chl: %d, security: %x\n\r"
				  , wifi.ssid.val, wifi.ssid.len, MAC_ARG(target_ap->BSSID.octet)
				  , wifi.password, wifi.password_len, wifi.channel, wifi.security_type);

	BANDSTEER_DBG("------------------------------(End!)--------------------------------------------\n\r");

	ret = wifi_connect(&wifi, 1);
	if (ret != RTW_SUCCESS) {
		connection_retry--;
		if (connection_retry > 0) {
			/* Add the delay to wait for the _rtw_join_timeout_handler
			 * If there is no this delay, there are some error when rhe AP
			 * send the disassociation frame. It will cause the connection
			 * to be failed at first time after resetting. So keep 300ms delay
			 * here. For the detail about this error, please refer to
			 * [RSWLANDIOT-1954].
			 */
			vTaskDelay(300);
			BANDSTEER_DBG("bandsteering retry\r\n");
			goto CONNECTION_RETRY_LOOP;
		}
		BANDSTEER_DBG("ERROR: Can't connect to AP\n\r");
		ret = -1;
		goto exit;
	}
	//example_wifi_roaming_enable(1);
#if CONFIG_LWIP_LAYER
	/* Start DHCPClient */
	LwIP_DHCP(0, DHCP_START);
#endif

	//store dhcp info for each ap after bandsteering connection.
	wifi_get_setting(WLAN0_IDX, &ap_info);
	rtw_memcpy(store_dhcp_info.ap_bssid, ap_info.bssid, 6);
	store_dhcp_info.sta_ip = xnetif[0].ip_addr.addr;
	gw = LwIP_GetGW(0);
	IP4_ADDR(ip_2_ip4(&server_ip_backup), gw[0], gw[1], gw[2], gw[3]);
	serverip_backup = ip4_addr_get_u32(ip_2_ip4(&server_ip_backup));
	store_dhcp_info.server_ip = serverip_backup;
	wifi_write_ap_info_to_flash_ext((u8 *)&store_dhcp_info, sizeof(struct ap_additional_info));
	BANDSTEER_DBG("[bandsteering_task] wifi_write_ap_info_to_flash_ext done\n\r");

exit:
	if (scan_buf) {
		rtw_mfree((u8 *)scan_buf, 0);
	}

	if (ret != RTW_SUCCESS) {
		//Used by FAST RECONNECTION
		example_wifi_roaming_enable(2);
		if (p_wifi_do_fast_connect) {
			p_wifi_do_fast_connect();
			BANDSTEER_DBG("[bandsteering_task] p_wifi_do_fast_connect done\n\r");
		}
	}

	example_wifi_roaming_enable(1);
	bandsteering_thread_handle = NULL;
	vTaskDelete(NULL);
}
#endif