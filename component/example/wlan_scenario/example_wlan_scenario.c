#include <platform_opts.h>
#include "FreeRTOS.h"
#include "task.h"
#include <platform_stdlib.h>
#include <lwip_netconf.h>
#include "wifi_constants.h"
#include "wifi_structures.h"
#include "lwip_netconf.h"
#include "wifi_conf.h"
#include "dhcp/dhcps.h"
#include "wifi_wps_config.h"
#include "osdep_service.h"
#include "example_wlan_scenario.h"

#if CONFIG_LWIP_LAYER
extern struct netif xnetif[NET_IF_NUM];
#endif

static rtw_result_t scan_result_handler(unsigned int scanned_AP_num, void *user_data);
static rtw_result_t scan_result_RSSI_handler(unsigned int scanned_AP_num, void *user_data);

const char *ssid = "";
const char *password = "";

rtw_join_status_t last_join_status = RTW_JOINSTATUS_UNKNOWN;

static void print_wifi_setting(const char *ifname, rtw_wifi_setting_t *pSetting)
{
	printf("\n\r\nWIFI  %s Setting:", ifname);
	printf("\n\r==============================");

	switch (pSetting->mode) {
	case RTW_MODE_AP:
		printf("\n\r		MODE => AP");
		break;
	case RTW_MODE_STA:
		printf("\n\r		MODE => STATION");
		break;
	default:
		printf("\n\r		MODE => UNKNOWN");
	}
	printf("\n\r		SSID => %s", pSetting->ssid);
	printf("\n\r	 CHANNEL => %d", pSetting->channel);

	switch (pSetting->security_type) {
	case RTW_SECURITY_OPEN:
		printf("\n\r  SECURITY => OPEN");
		break;
	case RTW_SECURITY_WEP_PSK:
		printf("\n\r  SECURITY => WEP");
		printf("\n\r KEY INDEX => %d", pSetting->key_idx);
		break;
	case RTW_SECURITY_WPA_TKIP_PSK:
		printf("\n\r  SECURITY => WPA TKIP");
		break;
	case RTW_SECURITY_WPA2_TKIP_PSK:
		printf("\n\r  SECURITY => WPA2 TKIP");
		break;
	case RTW_SECURITY_WPA_AES_PSK:
		printf("\n\r  SECURITY => WPA AES");
		break;
	case RTW_SECURITY_WPA_MIXED_PSK:
		printf("\n\r  SECURITY => WPA Mixed");
		break;
	case RTW_SECURITY_WPA2_AES_PSK:
		printf("\n\r  SECURITY => WPA2 AES");
		break;
	case RTW_SECURITY_WPA2_MIXED_PSK:
		printf("\n\r  SECURITY => WPA2 Mixed");
		break;
	case RTW_SECURITY_WPA_WPA2_TKIP_PSK:
		printf("\n\r  SECURITY => WPA/WPA2 TKIP");
		break;
	case RTW_SECURITY_WPA_WPA2_AES_PSK:
		printf("\n\r  SECURITY => WPA/WPA2 AES");
		break;
	case RTW_SECURITY_WPA_WPA2_MIXED_PSK:
		printf("\n\r  SECURITY => WPA/WPA2 Mixed");
		break;
	case RTW_SECURITY_WPA3_AES_PSK:
		printf("\n\r  SECURITY => WPA3 AES");
		break;
	case RTW_SECURITY_WPA2_WPA3_MIXED:
		printf("\n\r  SECURITY => WPA2/WPA3 AES");
		break;
	case RTW_SECURITY_WPA3_GCMP_PSK:
		printf("\n\r  SECURITY => GCMP");
		break;
	default:
		printf("\n\r  SECURITY => UNKNOWN");
	}

	printf("\n\r	PASSWORD => %s", pSetting->password);
	printf("\n\r");
}

/**
 * @brief  Scan network
 * @note  Process Flow:
 *		- Enable Wi-Fi with STA mode
 *		- Scan network
 */
static void scan_network(void)
{
	printf("\n\n[WLAN_SCENARIO_EXAMPLE] Wi-Fi example for network scan...\n");

	/*********************************************************************************
	*	1. Enable Wi-Fi with STA mode
	**********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Enable Wi-Fi\n");
	if (wifi_on(RTW_MODE_STA) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_on failed\n");
		return;
	}


	/**********************************************************************************
	*	2. Scan network
	**********************************************************************************/
	// Scan network and print them out.
	// Can refer to fATWS() in atcmd_wifi.c and scan_result_handler() below.
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Scan network\n");
	rtw_scan_param_t scan_param;
	rtw_memset(&scan_param, 0, sizeof(scan_param));
	scan_param.scan_user_callback = scan_result_handler;
	if (wifi_scan_networks(&scan_param, 0) != RTW_SUCCESS) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_scan_networks() failed\n");
		return;
	}
	// Wait for scan finished.
	vTaskDelay(5000);
}

/**
 * @brief  Authentication
 * @note  Process Flow:
 *		- Enable Wi-Fi with STA mode
 *		- Connect to AP by different authentications
 *			- WPS-PBC
 *			- WPS-PIN static PIN
 *			- WPS-PIN dynamic PIN
 *			- open
 *			- WEP open (64 bit)
 *			- WEP open (128 bit)
 *			- WEP shared (64 bit)
 *			- WEP shared (128 bit)
 *			- WPA-PSK (TKIP)
 *			- WPA-PSK (AES)
 *			- WPA2-PSK (TKIP)
 *			- WPA2-PSK (AES)
 *		- Show Wi-Fi information
 */
static void authentication(void)
{
	printf("\n\n[WLAN_SCENARIO_EXAMPLE] Wi-Fi example for authentication...\n");
	rtw_network_info_t connect_param = {0};

	/*********************************************************************************
	*	1. Enable Wi-Fi with STA mode
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Enable Wi-Fi\n");
	if (wifi_on(RTW_MODE_STA) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_on failed\n");
		return;
	}


	/*********************************************************************************
	*	2. Connect to AP by different authentications
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Connect to AP\n");
#if defined(CONFIG_ENABLE_WPS) && (CONFIG_ENABLE_WPS)
#if 1
	// By WPS-PBC.
	char *argv[2];
	argv[1] = (char *)"pbc";
	cmd_wps(2, argv);
#elif 0
	// By WPS-PIN static PIN. With specified PIN code 92402508 as example.
	char *argv[3];
	argv[1] = (char *)"pin";
	argv[2] = (char *)"92402508";
	cmd_wps(3, argv);
#elif 0
	// By WPS-PIN dynamic PIN.
	char *argv[2];
	argv[1] = (char *)"pin";
	cmd_wps(2, argv);
#endif
#else
	printf("Please set CONFIG_ENABLE_WPS 1 in platform_opts.h to enable WPS\n");
#endif

#if 0
	// By open.
	ssid = "Test_ap";
	memcpy(connect_param.ssid.val, ssid, strlen(ssid));
	connect_param.ssid.len = strlen(ssid);
	connect_param.password = (unsigned char *)password;
	connect_param.password_len = strlen(password);
	connect_param.security_type = RTW_SECURITY_OPEN;
	if (wifi_connect(&connect_param, 1) == RTW_SUCCESS) {
		LwIP_DHCP(0, DHCP_START);
	}
#elif 0
	// By WEP open (64 bit).
	ssid = "Test_ap";
	password = "pass1";	// 5 ASCII
	//password = "AAAAAAAAAA";	// 10 HEX
	int key_id = 2; 	// value of key_id is 0, 1, 2, or 3.

	memcpy(connect_param.ssid.val, ssid, strlen(ssid));
	connect_param.ssid.len = strlen(ssid);
	connect_param.password = (unsigned char *)password;
	connect_param.password_len = strlen(password);
	connect_param.key_id = key_id;
	connect_param.security_type = RTW_SECURITY_WEP_PSK;
	if (wifi_connect(&connect_param, 1) == RTW_SUCCESS) {
		LwIP_DHCP(0, DHCP_START);
	} else {
		printf("Connect Fail...\n");
	}
#elif 0
	// By WEP open (128 bit).
	ssid = "Test_ap";
	password = "password12345";	// 13 ASCII
	//password = "AAAAABBBBBCCCCCDDDDD123456";	// 26 HEX
	int key_id = 2; 	// value of key_id is 0, 1, 2, or 3.

	memcpy(connect_param.ssid.val, ssid, strlen(ssid));
	connect_param.ssid.len = strlen(ssid);
	connect_param.password = (unsigned char *)password;
	connect_param.password_len = strlen(password);
	connect_param.key_id = key_id;
	connect_param.security_type = RTW_SECURITY_WEP_PSK;
	if (wifi_connect(&connect_param, 1) == RTW_SUCCESS) {
		LwIP_DHCP(0, DHCP_START);
	}
#elif 0
	// By WEP shared (64 bit).
	ssid = "Test_ap";
	//password = "pass1";	// 5 ASCII
	password = "AAAAAAAAAA";	// 10 HEX
	int key_id = 2; 	// value of key_id is 0, 1, 2, or 3.

	memcpy(connect_param.ssid.val, ssid, strlen(ssid));
	connect_param.ssid.len = strlen(ssid);
	connect_param.password = (unsigned char *)password;
	connect_param.password_len = strlen(password);
	connect_param.key_id = key_id;
	connect_param.security_type = RTW_SECURITY_WEP_SHARED;
	if (wifi_connect(&connect_param, 1) == RTW_SUCCESS) {
		LwIP_DHCP(0, DHCP_START);
	}
#elif 0
	// By WEP shared (128 bit).
	ssid = "Test_ap";
	//password = "password12345";	// 13 ASCII
	password = "AAAAABBBBBCCCCCDDDDD123456";	// 26 HEX
	int key_id = 2; 	// value of key_id is 0, 1, 2, or 3.

	memcpy(connect_param.ssid.val, ssid, strlen(ssid));
	connect_param.ssid.len = strlen(ssid);
	connect_param.password = (unsigned char *)password;
	connect_param.password_len = strlen(password);
	connect_param.key_id = key_id;
	connect_param.security_type = RTW_SECURITY_WEP_SHARED;
	if (wifi_connect(&connect_param, 1) == RTW_SUCCESS) {
		LwIP_DHCP(0, DHCP_START);
	}

#elif 0
	// By WPA-PSK (TKIP)
	ssid = "Test_ap";
	password = "12345678";
	memcpy(connect_param.ssid.val, ssid, strlen(ssid));
	connect_param.ssid.len = strlen(ssid);
	connect_param.password = (unsigned char *)password;
	connect_param.password_len = strlen(password);
	connect_param.security_type = RTW_SECURITY_WPA_TKIP_PSK;
	if (wifi_connect(&connect_param, 1) == RTW_SUCCESS) {
		LwIP_DHCP(0, DHCP_START);
	}
#elif 0
	// By WPA-PSK (AES)
	ssid = "Test_ap";
	password = "12345678";
	memcpy(connect_param.ssid.val, ssid, strlen(ssid));
	connect_param.ssid.len = strlen(ssid);
	connect_param.password = (unsigned char *)password;
	connect_param.password_len = strlen(password);
	connect_param.security_type = RTW_SECURITY_WPA_AES_PSK;
	if (wifi_connect(&connect_param, 1) == RTW_SUCCESS) {
		LwIP_DHCP(0, DHCP_START);
	}
#elif 0
	// By WPA2-PSK (TKIP)
	ssid = "Test_ap";
	password = "12345678";
	memcpy(connect_param.ssid.val, ssid, strlen(ssid));
	connect_param.ssid.len = strlen(ssid);
	connect_param.password = (unsigned char *)password;
	connect_param.password_len = strlen(password);
	connect_param.security_type = RTW_SECURITY_WPA2_TKIP_PSK;
	if (wifi_connect(&connect_param, 1) == RTW_SUCCESS) {
		LwIP_DHCP(0, DHCP_START);
	}
#elif 0
	// By WPA2-PSK (AES)
	ssid = "Test_ap";
	password = "12345678";
	memcpy(connect_param.ssid.val, ssid, strlen(ssid));
	connect_param.ssid.len = strlen(ssid);
	connect_param.password = (unsigned char *)password;
	connect_param.password_len = strlen(password);
	connect_param.security_type = RTW_SECURITY_WPA2_AES_PSK;
	if (wifi_connect(&connect_param, 1) == RTW_SUCCESS) {
		LwIP_DHCP(0, DHCP_START);
	}
#endif
	/*********************************************************************************
	*	3. Show Wi-Fi information
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Show Wi-Fi information\n");
	rtw_wifi_setting_t setting;
	wifi_get_setting(WLAN0_IDX, &setting);
	print_wifi_setting(WLAN0_NAME, &setting);
}

/**
 * @brief  Wi-Fi example for mode switch case 1: Switch to infrastructure AP mode.
 * @note  Process Flow:
 *		- Disable Wi-Fi
 *		- Enable Wi-Fi with AP mode
 *		- Start AP
 *		- Check AP running
 *		- Start DHCP server
 */
static void mode_switch_1(void)
{
	printf("\n\n[WLAN_SCENARIO_EXAMPLE] Wi-Fi example mode switch case 1...\n");

	/*********************************************************************************
	*	1. Disable Wi-Fi
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Disable Wi-Fi\n");
	wifi_off();
	vTaskDelay(20);


	/*********************************************************************************
	*	2. Enable Wi-Fi with AP mode
	*********************************************************************************/
#if CONFIG_LWIP_LAYER
	dhcps_deinit();
	u32 addr = WIFI_MAKEU32(AP_IP_ADDR0, AP_IP_ADDR1, AP_IP_ADDR2, AP_IP_ADDR3);
	u32 netmask = WIFI_MAKEU32(AP_NETMASK_ADDR0, AP_NETMASK_ADDR1, AP_NETMASK_ADDR2, AP_NETMASK_ADDR3);
	u32 gw = WIFI_MAKEU32(AP_GW_ADDR0, AP_GW_ADDR1, AP_GW_ADDR2, AP_GW_ADDR3);
	LwIP_SetIP(0, addr, netmask, gw);
#endif
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Enable Wi-Fi with AP mode\n");
	if (wifi_on(RTW_MODE_AP) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_on failed\n");
		return;
	}


	/*********************************************************************************
	*	3. Start AP
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Start AP\n");
	ssid = (char *)"RTL_AP";
	password = (char *)"12345678";
	rtw_softap_info_t softAP_config = {0};
	softAP_config.ssid.len = strlen(ssid);
	rtw_memcpy(softAP_config.ssid.val, (char *)ssid, softAP_config.ssid.len);
	softAP_config.password = (unsigned char *)password;
	softAP_config.password_len = strlen(password);
	softAP_config.channel = 6;
	softAP_config.security_type = RTW_SECURITY_WPA2_AES_PSK;
	if (wifi_start_ap(&softAP_config) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_start_ap failed\n");
		return;
	}


	/*********************************************************************************
	*	4. Check AP running
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Check AP running\n");
	int timeout = 20;
	while (1) {
		rtw_wifi_setting_t setting;
		wifi_get_setting(WLAN0_IDX, &setting);
		if (strlen((char *)setting.ssid) > 0) {
			if (strcmp((const char *)setting.ssid, (const char *)ssid) == 0) {
				printf("\n\r[WLAN_SCENARIO_EXAMPLE] %s started\n", ssid);
				break;
			}
		}
		if (timeout == 0) {
			printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: Start AP timeout\n");
			return;
		}
		vTaskDelay(1 * configTICK_RATE_HZ);
		timeout --;
	}


	/*********************************************************************************
	*	5. Start DHCP server
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Start DHCP server\n");
	// For more setting about DHCP, please reference fATWA in atcmd_wifi.c.
#if CONFIG_LWIP_LAYER
	dhcps_init(&xnetif[0]);
#endif
}

/**
 * @brief  Wi-Fi example for mode switch case 2: Switch to infrastructure STA mode.
 * @note  Process Flow:
 *		- Disable Wi-Fi
 *		- Enable Wi-Fi with STA mode
 *		- Connect to AP using STA mode
 *		- Show Wi-Fi information
 */
static void mode_switch_2(void)
{
	rtw_network_info_t connect_param = {0};
	printf("\n\n[WLAN_SCENARIO_EXAMPLE] Wi-Fi example mode switch case 2...\n");

	/*********************************************************************************
	*	1. Disable Wi-Fi
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Disable Wi-Fi\n");
	wifi_off();
	vTaskDelay(20);


	/*********************************************************************************
	*	2. Enable Wi-Fi with STA mode
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Enable Wi-Fi with STA mode\n");
	if (wifi_on(RTW_MODE_STA) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_on failed\n");
		return;
	}

	/*********************************************************************************
	*	3. Connect to AP using STA mode
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Connect to AP using STA mode\n");
	ssid = "Test_ap";
	password = "12345678";
	memcpy(connect_param.ssid.val, ssid, strlen(ssid));
	connect_param.ssid.len = strlen(ssid);
	connect_param.password = (unsigned char *)password;
	connect_param.password_len = strlen(password);
	connect_param.security_type = RTW_SECURITY_WPA2_AES_PSK;
	if (wifi_connect(&connect_param, 1) == RTW_SUCCESS) {
		LwIP_DHCP(0, DHCP_START);
	}


	/*********************************************************************************
	*	4. Show Wi-Fi information
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Show Wi-Fi information\n");
	rtw_wifi_setting_t setting;
	wifi_get_setting(WLAN0_IDX, &setting);
	print_wifi_setting(WLAN0_NAME, &setting);
}

/**
 * @brief  Wi-Fi example for mode switch case 3: Switch to infrastructure P2P Autonomous GO mode.
 * @note  Process Flow:
 *		- Enable Wi-Fi Direct mode
 *		- Enable Wi-Fi Direct Automonous GO
 *		- Show Wi-Fi Direct Information
 *		- Disable Wi-Fi Direct mode
 */
static void mode_switch_3(void)
{
	printf("\n\n[WLAN_SCENARIO_EXAMPLE] Wi-Fi example mode switch case 3...\n");
#if defined(CONFIG_ENABLE_P2P) && (CONFIG_ENABLE_P2P)
	/*********************************************************************************
	*	1. Enable Wi-Fi Direct mode
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Enable Wi-Fi Direct mode\n");
	// Start Wi-Fi Direct mode.
	// cmd_wifi_p2p_start() will re-enable the Wi-Fi with P2P mode and initialize P2P data.
	cmd_wifi_p2p_start(NULL, NULL);


	/*********************************************************************************
	*	2. Enable Wi-Fi Direct Automonous GO
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Enable Wi-Fi Direct Automonous GO\n");
	// Start Wi-Fi Direct Automonous GO.
	// The GO related parameters can be set in cmd_wifi_p2p_auto_go_start() function declaration.
	if (cmd_wifi_p2p_auto_go_start(NULL, NULL) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: cmd_wifi_p2p_auto_go_start() failed\n");
		return;
	}


	/*********************************************************************************
	*	3. Show Wi-Fi Direct Information
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Show Wi-Fi Direct Information\n");
	// Show the Wi-Fi Direct Info.
	cmd_p2p_info(NULL, NULL);


	vTaskDelay(60000);


	/*********************************************************************************
	*	4. Disable Wi-Fi Direct mode
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Disable Wi-Fi Direct mode\n");
	// Disable Wi-Fi Direct GO. Will disable Wi-Fi either.
	// This command has to be invoked to release the P2P resource.
	cmd_wifi_p2p_stop(NULL, NULL);
#else
	printf("Please set CONFIG_ENABLE_P2P 1 in platform_opts.h to enable P2P\n");
#endif
}

/**
 * @brief  Wi-Fi example for mode switch case 4: Mode switching time between AP and STA.
 * @note  Process Flow:
 *		- Enable Wi-Fi with STA mode
 *		- Disable STA mode and start AP (First measurement)
 *		- Stop AP and enable STA mode (Second measurement)
 */
static void mode_switch_4(void)
{
	printf("\n\n[WLAN_SCENARIO_EXAMPLE] Wi-Fi example mode switch case 4...\n");
	rtw_network_info_t connect_param = {0};
	// First measurement.
	unsigned long tick_STA_to_AP_begin;
	unsigned long tick_STA_to_AP_end;
	// Second measurement.
	unsigned long tick_AP_to_STA_begin;
	unsigned long tick_AP_to_STA_end;

	/*********************************************************************************
	*	1. Enable Wi-Fi with STA mode
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Enable Wi-Fi with STA mode\n");
	if (wifi_on(RTW_MODE_STA) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_on failed\n");
		return;
	}

	/*********************************************************************************
	*	2. Disable STA mode and start AP (First measurement)
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Disable STA mode and start AP (First measurement)\n");
	// Begin time.
	tick_STA_to_AP_begin = xTaskGetTickCount();

	wifi_off();

#if CONFIG_LWIP_LAYER
	dhcps_deinit();
	u32 addr = WIFI_MAKEU32(AP_IP_ADDR0, AP_IP_ADDR1, AP_IP_ADDR2, AP_IP_ADDR3);
	u32 netmask = WIFI_MAKEU32(AP_NETMASK_ADDR0, AP_NETMASK_ADDR1, AP_NETMASK_ADDR2, AP_NETMASK_ADDR3);
	u32 gw = WIFI_MAKEU32(AP_GW_ADDR0, AP_GW_ADDR1, AP_GW_ADDR2, AP_GW_ADDR3);
	LwIP_SetIP(0, addr, netmask, gw);
#endif
	if (wifi_on(RTW_MODE_AP) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_on failed\n");
		return;
	}

	ssid = "RTL_AP";
	password = "12345678";
	rtw_softap_info_t softAP_config = {0};
	softAP_config.ssid.len = strlen(ssid);
	rtw_memcpy(softAP_config.ssid.val, (unsigned char *)ssid, softAP_config.ssid.len);
	softAP_config.password = (unsigned char *)password;
	softAP_config.password_len = strlen(password);
	softAP_config.channel = 6;
	softAP_config.security_type = RTW_SECURITY_WPA2_AES_PSK;
	if (wifi_start_ap(&softAP_config) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_start_ap failed\n");
		return;
	}

	int timeout = 20;
	while (1) {
		rtw_wifi_setting_t setting;
		wifi_get_setting(WLAN0_IDX, &setting);
		if (strlen((char *)setting.ssid) > 0) {
			if (strcmp((const char *)setting.ssid, (const char *)ssid) == 0) {
				printf("\n\r[WLAN_SCENARIO_EXAMPLE] %s started\n", ssid);
				break;
			}
		}
		if (timeout == 0) {
			printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: Start AP timeout\n");
			return;
		}
		vTaskDelay(1 * configTICK_RATE_HZ);
		timeout --;
	}

#if CONFIG_LWIP_LAYER
	dhcps_init(&xnetif[0]);
#endif
	// End time.
	tick_STA_to_AP_end = xTaskGetTickCount();
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Time diff switch from STA mode to start AP: %d ms\n",
		   (tick_STA_to_AP_end - tick_STA_to_AP_begin));


	vTaskDelay(60000);


	/*********************************************************************************
	*	3. Stop AP and enable STA mode (Second measurement)
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Stop AP and enable STA mode (Second measurement)\n");
	// Begin time.
	tick_AP_to_STA_begin = xTaskGetTickCount();

	wifi_off();

	if (wifi_on(RTW_MODE_STA) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_on failed\n");
		return;
	}

	// Connect to AP by WPA2-AES security
	ssid = "Test_ap";
	password = "12345678";
	memcpy(connect_param.ssid.val, ssid, strlen(ssid));
	connect_param.ssid.len = strlen(ssid);
	connect_param.password = (unsigned char *)password;
	connect_param.password_len = strlen(password);
	connect_param.security_type = RTW_SECURITY_WPA2_AES_PSK;
	if (wifi_connect(&connect_param, 1) == RTW_SUCCESS) {
		LwIP_DHCP(0, DHCP_START);
	}

	// End time.
	tick_AP_to_STA_end = xTaskGetTickCount();
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Time diff switch from stop AP to enable STA mode: %d ms\n",
		   (tick_AP_to_STA_end - tick_AP_to_STA_begin));
}

/**
 * @brief  Wi-Fi example for mode switch case 5: Mode switching time between P2P(autonomousGO) and STA.
 * @note  Process Flow:
 *		- Enable Wi-Fi with STA mode
 *		- Disable STA mode and start P2P GO (First measurement)
 *		- Stop P2P GO and enable STA mode (Second measurement)
 */
static void mode_switch_5(void)
{
	printf("\n\n[WLAN_SCENARIO_EXAMPLE] Wi-Fi example mode switch case 5...\n");
	rtw_network_info_t connect_param = {0};
#if defined(CONFIG_ENABLE_P2P) && (CONFIG_ENABLE_P2P)
	// First measurement.
	unsigned long tick_STA_to_P2PGO_begin;
	unsigned long tick_STA_to_P2PGO_end;
	// Second measurement.
	unsigned long tick_P2PGO_to_STA_begin;
	unsigned long tick_P2PGO_to_STA_end;
#endif
	/*********************************************************************************
	*	1. Enable Wi-Fi with STA mode
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Enable Wi-Fi with STA mode\n");
	if (wifi_on(RTW_MODE_STA) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_on failed\n");
		return;
	}
#if defined(CONFIG_ENABLE_P2P) && (CONFIG_ENABLE_P2P)
	/*********************************************************************************
	*	2. Disable STA mode and start P2P GO (First measurement)
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Disable STA mode and start AP (First measurement)\n");
	// Begin time.
	tick_STA_to_P2PGO_begin = xTaskGetTickCount();

	wifi_off();

	cmd_wifi_p2p_start(NULL, NULL);

	if (cmd_wifi_p2p_auto_go_start(NULL, NULL) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: cmd_wifi_p2p_auto_go_start() failed\n");
		return;
	}

	// End time.
	tick_STA_to_P2PGO_end = xTaskGetTickCount();
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Time diff switch from STA mode to start P2P GO: %d ms\n",
		   (tick_STA_to_P2PGO_end - tick_STA_to_P2PGO_begin));


	vTaskDelay(60000);


	/*********************************************************************************
	*	3. Stop P2P GO and enable STA mode (Second measurement)
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Stop P2P GO and enable STA mode (Second measurement)\n");
	// Begin time.
	tick_P2PGO_to_STA_begin = xTaskGetTickCount();

	cmd_wifi_p2p_stop(NULL, NULL);
#else
	printf("Please set CONFIG_ENABLE_P2P 1 in platform_opts.h to enable P2P\n");
#endif
	if (wifi_on(RTW_MODE_STA) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_on failed\n");
		return;
	}

	// Connect to AP by WPA2-AES security
	ssid = "Test_ap";
	password = "12345678";
	memcpy(connect_param.ssid.val, ssid, strlen(ssid));
	connect_param.ssid.len = strlen(ssid);
	connect_param.password = (unsigned char *)password;
	connect_param.password_len = strlen(password);
	connect_param.security_type = RTW_SECURITY_WPA2_AES_PSK;
	if (wifi_connect(&connect_param, 1) == RTW_SUCCESS) {
		LwIP_DHCP(0, DHCP_START);
	}

#if defined(CONFIG_ENABLE_P2P) && (CONFIG_ENABLE_P2P)
	// End time.
	tick_P2PGO_to_STA_end = xTaskGetTickCount();
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Time diff switch from stop P2P GO to enable STA mode: %d ms\n",
		   (tick_P2PGO_to_STA_end - tick_P2PGO_to_STA_begin));
#endif
}


/**
 * @brief  Wi-Fi example for mode switch case 6: Switch to infrastructure AP mode with hidden SSID.
 * @note  Process Flow:
 *		- Disable Wi-Fi
 *		- Enable Wi-Fi with AP mode
 *		- Start AP with hidden SSID
 *		- Check AP running
 *		- Start DHCP server
 */
static void mode_switch_6(void)
{
	printf("\n\n[WLAN_SCENARIO_EXAMPLE] Wi-Fi example mode switch case 6...\n");

	/*********************************************************************************
	*	1. Disable Wi-Fi
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Disable Wi-Fi\n");
	wifi_off();
	vTaskDelay(20);


#if CONFIG_LWIP_LAYER
	dhcps_deinit();
	u32 addr = WIFI_MAKEU32(AP_IP_ADDR0, AP_IP_ADDR1, AP_IP_ADDR2, AP_IP_ADDR3);
	u32 netmask = WIFI_MAKEU32(AP_NETMASK_ADDR0, AP_NETMASK_ADDR1, AP_NETMASK_ADDR2, AP_NETMASK_ADDR3);
	u32 gw = WIFI_MAKEU32(AP_GW_ADDR0, AP_GW_ADDR1, AP_GW_ADDR2, AP_GW_ADDR3);
	LwIP_SetIP(0, addr, netmask, gw);
#endif

	/*********************************************************************************
	*	2. Enable Wi-Fi with AP mode
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Enable Wi-Fi with AP mode\n");
	if (wifi_on(RTW_MODE_AP) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_on failed\n");
		return;
	}


	/*********************************************************************************
	*	3. Start AP with hidden SSID
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Start AP with hidden SSID\n");
	ssid = "RTL_AP";
	password = "12345678";
	rtw_softap_info_t softAP_config = {0};
	softAP_config.ssid.len = strlen(ssid);
	rtw_memcpy(softAP_config.ssid.val, (unsigned char *)ssid, softAP_config.ssid.len);
	softAP_config.password = (unsigned char *)password;
	softAP_config.password_len = strlen(password);
	softAP_config.channel = 6;
	softAP_config.security_type = RTW_SECURITY_WPA2_AES_PSK;
	softAP_config.hidden_ssid = 1;
	if (wifi_start_ap(&softAP_config) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_start_ap(hidden ssid) failed\n");
		return;
	}


	/*********************************************************************************
	*	4. Check AP running
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Check AP running\n");
	int timeout = 20;
	while (1) {
		rtw_wifi_setting_t setting;
		wifi_get_setting(WLAN0_IDX, &setting);
		if (strlen((char *)setting.ssid) > 0) {
			if (strcmp((const char *)setting.ssid, (const char *)ssid) == 0) {
				printf("\n\r[WLAN_SCENARIO_EXAMPLE] %s started\n", ssid);
				break;
			}
		}
		if (timeout == 0) {
			printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: Start AP timeout\n");
			return;
		}
		vTaskDelay(1 * configTICK_RATE_HZ);
		timeout --;
	}


	/*********************************************************************************
	*	5. Start DHCP server
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Start DHCP server\n");
	// For more setting about DHCP, please reference fATWA in atcmd_wifi.c.
#if CONFIG_LWIP_LAYER
	dhcps_init(&xnetif[0]);
#endif
}

/**
 * @brief  Wi-Fi example for mode switch case 7: Mode switching between concurrent mode and STA.
 * @note  Process Flow:
 *		- Enable Wi-Fi with concurrent (STA + AP) mode
 *		  - Disable Wi-Fi
 *		  - Start AP
 *		  - Check AP running
 *		  - Start DHCP server
 *		  - Connect to AP using STA mode
 *		- Disable concurrent (STA + AP) mode and start STA mode
 */
static void mode_switch_7(void)
{
	printf("\n\n[WLAN_SCENARIO_EXAMPLE] Wi-Fi example mode switch case 7...\n");

	/*********************************************************************************
	*	1. Enable Wi-Fi with concurrent (STA + AP) mode
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Enable Wi-Fi with concurrent (STA + AP) mode\n");

	/*********************************************************************************
	*	1-1. Disable Wi-Fi
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Disable Wi-Fi\n");
	wifi_off();
	vTaskDelay(20);

	/*********************************************************************************
	*	1-2. Enable Wi-Fi with STA + AP mode
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Enable Wi-Fi with STA + AP mode\n");
	if (wifi_on(RTW_MODE_STA_AP) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_on failed\n");
		return;
	}

	/*********************************************************************************
	*	1-3. Start AP
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Start AP\n");
	ssid = "RTL_AP";
	password = "12345678";
	rtw_softap_info_t softAP_config = {0};
	softAP_config.ssid.len = strlen(ssid);
	rtw_memcpy(softAP_config.ssid.val, (unsigned char *)ssid, softAP_config.ssid.len);
	softAP_config.password = (unsigned char *)password;
	softAP_config.password_len = strlen(password);
	softAP_config.channel = 6;
	softAP_config.security_type = RTW_SECURITY_WPA2_AES_PSK;
	if (wifi_start_ap(&softAP_config) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_start_ap failed\n");
		return;
	}

	/*********************************************************************************
	*	1-4. Check AP running
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Check AP running\n");
	int timeout = 20;
	while (1) {
		rtw_wifi_setting_t setting;
		wifi_get_setting(WLAN1_IDX, &setting);
		if (strlen((char *)setting.ssid) > 0) {
			if (strcmp((const char *) setting.ssid, (const char *)ssid) == 0) {
				printf("\n\r[WLAN_SCENARIO_EXAMPLE] %s started\n", ssid);
				break;
			}
		}
		if (timeout == 0) {
			printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: Start AP timeout\n");
			return;
		}
		vTaskDelay(1 * configTICK_RATE_HZ);
		timeout --;
	}

	/*********************************************************************************
	*	1-5. Start DHCP server
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Start DHCP server\n");
#if CONFIG_LWIP_LAYER
	dhcps_init(&xnetif[1]);
	vTaskDelay(1000);
#endif

	/*********************************************************************************
	*	1-6. Connect to AP using STA mode
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Connect to AP\n");
	rtw_network_info_t connect_param = {0};

	ssid = "Test_ap";
	password = "12345678";

	memcpy(connect_param.ssid.val, ssid, strlen(ssid));
	connect_param.ssid.len = strlen(ssid);
	connect_param.password = (unsigned char *)password;
	connect_param.password_len = strlen(password);
	connect_param.security_type = RTW_SECURITY_WPA2_AES_PSK;
	if (wifi_connect(&connect_param, 1) == RTW_SUCCESS) {
		LwIP_DHCP(0, DHCP_START);
	}

	/*********************************************************************************
	*	2. Disable concurrent (STA + AP) mode and start STA mode
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Disable concurrent (STA + AP) mode and start STA mode\n");
	wifi_off();
	vTaskDelay(20);

	if (wifi_on(RTW_MODE_STA) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_on failed\n");
		return;
	}

	ssid = "Test_ap";
	password = "12345678";
	memcpy(connect_param.ssid.val, ssid, strlen(ssid));
	connect_param.ssid.len = strlen(ssid);
	connect_param.password = (unsigned char *)password;
	connect_param.password_len = strlen(password);
	connect_param.security_type = RTW_SECURITY_WPA2_AES_PSK;
	if (wifi_connect(&connect_param, 1) == RTW_SUCCESS) {
		LwIP_DHCP(0, DHCP_START);
	}
}



/**
 * @brief  Wi-Fi example for scenario case 1.
 * @note  Process Flow:
 *		- Enable Wi-Fi with STA mode
 *		- Connect to AP by WPS enrollee static PIN mode (If failed, re-connect one time.)
 *		- Enable Wi-Fi Direct GO
 *			(It will re-enable WiFi, the original connection to AP would be broken.)
 */
static void scenario_1(void)
{
	printf("\n\n[WLAN_SCENARIO_EXAMPLE] Wi-Fi example scenario case 1...\n");

	/*********************************************************************************
	*	1. Enable Wi-Fi with STA mode
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Enable Wi-Fi\n");
	if (wifi_on(RTW_MODE_STA) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_on failed\n");
		return;
	}

#if defined(CONFIG_ENABLE_WPS) && (CONFIG_ENABLE_WPS)
	/*********************************************************************************
	*	2. Connect to AP by WPS enrollee PIN mode
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Connect to AP by WPS enrollee PIN mode with PIN code: 92402508\n");
	// Start static WPS-PIN enrollee with PIN code: 92402508.
	// As the process beginning, please enter the PIN code in AP side.
	// It will take at most 2 min to do the procedure.
	char *argv[3];
	argv[1] = (char *)"pin";
	argv[2] = (char *)"92402508";
	cmd_wps(3, argv);

	// If not connected, retry one time.
	if (wifi_is_connected_to_ap() != RTW_SUCCESS) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] WPS enrollee failed, reconnect one time\n");
		cmd_wps(3, argv);
	}
#else
	printf("Please set CONFIG_ENABLE_WPS 1 in platform_opts.h to enable WPS\n");
#endif
#if defined(CONFIG_ENABLE_P2P) && (CONFIG_ENABLE_P2P)
	/*********************************************************************************
	*	3. Enable Wi-Fi Direct GO
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Enable Wi-Fi Direct GO\n");
	// Start Wi-Fi Direct mode.
	// cmd_wifi_p2p_start() will re-enable the Wi-Fi with P2P mode and initialize P2P data.
	cmd_wifi_p2p_start(NULL, NULL);
	// Start Wi-Fi Direct Group Owner mode.
	// The GO related parameters can be set in cmd_wifi_p2p_auto_go_start() function declaration.
	if (cmd_wifi_p2p_auto_go_start(NULL, NULL) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: cmd_wifi_p2p_auto_go_start() failed\n");
		return;
	}
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Wi-Fi Direct Group Owner mode enabled\n");

	// Show the Wi-Fi Direct Info.
	cmd_p2p_info(NULL, NULL);
#else
	printf("Please set CONFIG_ENABLE_P2P 1 in platform_opts.h to enable P2P\n");
#endif
}

/**
 * @brief  Wi-Fi example for scenario case 2.
 * @note  Process Flow:
 *		- Enable Wi-Fi Direct GO
 *		- Disable Wi-Fi Direct GO, and enable Wi-Fi with STA mode
 *			(Disable Wi-Fi Direct GO must be done to release P2P resource.)
 *		- Connect to AP by WPS enrollee PBC mode (If failed, re-connect one time.)
 */
static void scenario_2(void)
{
	printf("\n\n[WLAN_SCENARIO_EXAMPLE] Wi-Fi example scenario case 2...\n");
#if defined(CONFIG_ENABLE_P2P) && (CONFIG_ENABLE_P2P)
	/*********************************************************************************
	*	1. Enable Wi-Fi Direct GO
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Enable Wi-Fi Direct GO\n");
	// Start Wi-Fi Direct mode.
	// cmd_wifi_p2p_start() will re-enable the Wi-Fi with P2P mode and initialize P2P data.
	cmd_wifi_p2p_start(NULL, NULL);
	// Start Wi-Fi Direct Group Owner mode.
	// The GO related parameters can be set in cmd_wifi_p2p_auto_go_start() function declaration.
	if (cmd_wifi_p2p_auto_go_start(NULL, NULL) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: cmd_wifi_p2p_auto_go_start() failed\n");
		return;
	}
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Wi-Fi Direct Group Owner mode enabled\n");
	// Show the Wi-Fi Direct Info.
	cmd_p2p_info(NULL, NULL);


	/*********************************************************************************
	*	2. Disable Wi-Fi Direct GO, and enable Wi-Fi with STA mode
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Disable Wi-Fi Direct GO, and enable Wi-Fi\n");
	// Disable Wi-Fi Direct GO.
	// This command has to be invoked to release the P2P resource.
	cmd_wifi_p2p_stop(NULL, NULL);
	// Enable Wi-Fi with STA mode.
	if (wifi_on(RTW_MODE_STA) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_on() failed\n");
		return;
	}
#else
	printf("Please set CONFIG_ENABLE_P2P 1 in platform_opts.h to enable P2P\n");
#endif
#if defined(CONFIG_ENABLE_WPS) && (CONFIG_ENABLE_WPS)
	/*********************************************************************************
	*	3. Connect to AP by WPS enrollee PBC mode
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Connect to AP by WPS enrollee PBC mode\n");
	// Start WPS-PBC enrollee.
	// As the process beginning, please push the WPS button on AP.
	// It will take at most 2 min to do the procedure.
	char *argv[2];
	argv[1] = (char *)"pbc";
	cmd_wps(2, argv);

	// If not connected, retry one time.
	if (wifi_is_connected_to_ap() != RTW_SUCCESS) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] WPS enrollee failed, reconnect one time\n");
		cmd_wps(2, argv);
	}
#else
	printf("Please set CONFIG_ENABLE_WPS 1 in platform_opts.h to enable WPS\n");
#endif
}

/**
 * @brief  Wi-Fi example for scenario case 3.
 * @note  Process Flow:
 *		- Enable Wi-Fi with STA mode
 *		- Scan network
 *		- Connect to AP use STA mode (If failed, re-connect one time.)
 *		- Enable Wi-Fi Direct GO
 *			(It will re-enable WiFi, the original connection to AP would be broken.)
 */
static void scenario_3(void)
{
	printf("\n\n[WLAN_SCENARIO_EXAMPLE] Wi-Fi example scenario case 3...\n");

	/*********************************************************************************
	*	1. Enable Wi-Fi with STA mode
	**********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Enable Wi-Fi\n");
	if (wifi_on(RTW_MODE_STA) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_on failed\n");
		return;
	}


	/**********************************************************************************
	*	2. Scan network
	**********************************************************************************/
	// Scan network and print them out.
	// Can refer to fATWS() in atcmd_wifi.c.
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Scan network\n");
	rtw_scan_param_t scan_param;
	rtw_memset(&scan_param, 0, sizeof(rtw_scan_param_t));
	scan_param.scan_user_callback = scan_result_handler;
	if (wifi_scan_networks(&scan_param, 0) != RTW_SUCCESS) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_scan_networks() failed\n");
		return;
	}
	// Wait for scan finished.
	vTaskDelay(5000);


	/**********************************************************************************
	*	3. Connect to AP use STA mode (If failed, re-connect one time.)
	**********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Connect to AP use STA mode\n");

	// Set the auto-reconnect mode with retry 1 time(limit is 2) and timeout 5 seconds.
	// This command need to be set before invoke wifi_connect() to make reconnection work.
	wifi_config_autoreconnect(1, 2, 5);

	// Connect to AP with PSK-WPA2-AES.
	ssid = "Test_ap";
	password = "12345678";
	rtw_network_info_t connect_param = {0};
	memcpy(connect_param.ssid.val, ssid, strlen(ssid));
	connect_param.ssid.len = strlen(ssid);
	connect_param.password = (unsigned char *)password;
	connect_param.password_len = strlen(password);
	connect_param.security_type = RTW_SECURITY_WPA2_AES_PSK;
	if (wifi_connect(&connect_param, 1) == RTW_SUCCESS) {
		LwIP_DHCP(0, DHCP_START);
	}

	// Show Wi-Fi info.
	rtw_wifi_setting_t setting;
	wifi_get_setting(WLAN0_IDX, &setting);
	print_wifi_setting(WLAN0_NAME, &setting);

#if defined(CONFIG_ENABLE_P2P) && (CONFIG_ENABLE_P2P)
	/**********************************************************************************
	*	4. Enable Wi-Fi Direct GO
	**********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Enable Wi-Fi Direct GO\n");
	// Start Wi-Fi Direct mode.
	// cmd_wifi_p2p_start() will re-enable the Wi-Fi with P2P mode and initialize P2P data.
	cmd_wifi_p2p_start(NULL, NULL);
	// Start Wi-Fi Direct Group Owner mode.
	// The GO related parameters can be set in cmd_wifi_p2p_auto_go_start() function declaration.
	if (cmd_wifi_p2p_auto_go_start(NULL, NULL) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: cmd_wifi_p2p_auto_go_start() failed\n");
		return;
	}
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Wi-Fi Direct Group Owner mode enabled\n");

	// Show the Wi-Fi Direct Info.
	cmd_p2p_info(NULL, NULL);
#else
	printf("Please set CONFIG_ENABLE_P2P 1 in platform_opts.h to enable P2P\n");
#endif
}

/**
 * @brief  Wi-Fi example for scenario case 4.
 * @note  Process Flow:
 *		- Enable Wi-Fi with STA mode
 *		- Connect to AP by WPS enrollee PBC mode (If failed, re-connect one time.)
 *		- Disconnect from AP
 *		- Enable Wi-Fi Direct GO
 *		- Disable Wi-Fi Direct GO, and enable Wi-Fi with STA mode
 *			(Disable Wi-Fi Direct GO must be done to release P2P resource.)
 *		- Connect to AP use STA mode (If failed, re-connect one time.)
 *		- Disconnect from AP
 *		- Disable Wi-Fi
 */
static void scenario_4(void)
{
	printf("\n\n[WLAN_SCENARIO_EXAMPLE] Wi-Fi example scenario case 4...\n");

	/**********************************************************************************
	*	1. Enable Wi-Fi with STA mode
	**********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Enable Wi-Fi\n");
	if (wifi_on(RTW_MODE_STA) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_on failed\n");
		return;
	}

#if defined(CONFIG_ENABLE_WPS) && (CONFIG_ENABLE_WPS)
	/**********************************************************************************
	*	2. Connect to AP by WPS enrollee PBC mode
	**********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Connect to AP by WPS enrollee PBC mode\n");
	// Start WPS-PBC enrollee.
	// As the process beginning, please push the WPS button on AP.
	// It will take at most 2 min to do the procedure.
	char *argv[2];
	argv[1] = (char *)"pbc";
	cmd_wps(2, argv);

	// If not connected, retry one time.
	if (wifi_is_connected_to_ap() != RTW_SUCCESS) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] WPS enrollee failed, reconnect one time\n");
		cmd_wps(2, argv);
	}
#else
	printf("Please set CONFIG_ENABLE_WPS 1 in platform_opts.h to enable WPS\n");
#endif

	/**********************************************************************************
	*	3. Disconnect from AP
	**********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Disconnect from AP\n");
	if (wifi_disconnect() < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_disconnect() failed\n");
		return;
	}

	// Show Wi-Fi info.
	rtw_wifi_setting_t setting;
	wifi_get_setting(WLAN0_IDX, &setting);
	print_wifi_setting(WLAN0_NAME, &setting);

#if defined(CONFIG_ENABLE_P2P) && (CONFIG_ENABLE_P2P)
	/**********************************************************************************
	*	4. Enable Wi-Fi Direct GO
	**********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Enable Wi-Fi Direct GO\n");
	// Start Wi-Fi Direct mode.
	// cmd_wifi_p2p_start() will re-enable the Wi-Fi with P2P mode and initialize P2P data.
	cmd_wifi_p2p_start(NULL, NULL);
	// Start Wi-Fi Direct Group Owner mode.
	// The GO related parameters can be set in cmd_wifi_p2p_auto_go_start() function declaration.
	if (cmd_wifi_p2p_auto_go_start(NULL, NULL) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: cmd_wifi_p2p_auto_go_start() failed\n");
		return;
	}
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Wi-Fi Direct Group Owner mode enabled\n");

	// Show the Wi-Fi Direct Info.
	cmd_p2p_info(NULL, NULL);


	/**********************************************************************************
	*	5. Disable Wi-Fi Direct GO, and enable Wi-Fi with STA mode
	**********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Disable Wi-Fi Direct GO, and enable Wi-Fi\n");
	// Disable Wi-Fi Direct GO.
	// This command has to be invoked to release the P2P resource.
	cmd_wifi_p2p_stop(NULL, NULL);
#else
	printf("Please set CONFIG_ENABLE_P2P 1 in platform_opts.h to enable P2P\n");
#endif
	// Enable Wi-Fi on STA mode.
	if (wifi_on(RTW_MODE_STA) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_on() failed\n");
		return;
	}


	/**********************************************************************************
	*	6. Connect to AP use STA mode
	**********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Connect to AP use STA mode\n");

	// Set the auto-reconnect mode with retry 1 time(limit is 2) and timeout 5 seconds.
	// This command need to be set before invoke wifi_connect() to make reconnection work.
	wifi_config_autoreconnect(1, 2, 5);

	// Connect to AP with Open mode.
	ssid = "Test_ap";
	rtw_network_info_t connect_param = {0};
	memcpy(connect_param.ssid.val, ssid, strlen(ssid));
	connect_param.ssid.len = strlen(ssid);
	connect_param.security_type = RTW_SECURITY_OPEN;
	if (wifi_connect(&connect_param, 1) == RTW_SUCCESS) {
		LwIP_DHCP(0, DHCP_START);
	}

	// Show Wi-Fi info.
	wifi_get_setting(WLAN0_IDX, &setting);
	print_wifi_setting(WLAN0_NAME, &setting);


	/**********************************************************************************
	*	7. Disconnect from AP
	**********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Disconnect from AP\n");
	if (wifi_disconnect() < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_disconnect() failed\n");
		return;
	}

	// Show Wi-Fi info.
	wifi_get_setting(WLAN0_IDX, &setting);
	print_wifi_setting(WLAN0_NAME, &setting);


	/**********************************************************************************
	*	8. Disable Wi-Fi
	**********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Disable Wi-Fi\n");
	if (wifi_off() != RTW_SUCCESS) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_off() failed\n");
		return;
	}
}

/**
 * @brief  Wi-Fi example for scenario case 5.
 * @note  Process Flow:
 *		- Enable Wi-Fi with STA mode
 *		- Connect to AP using STA mode, check the connection result based on join status
 *		- Show Wi-Fi information
 *		- Get AP's RSSI
 */
void wifi_join_status_callback(rtw_join_status_t join_status)
{
	u8 error_flag = 0;
	u16 reason_code = 0;
	if (join_status == RTW_JOINSTATUS_FAIL) {
		/* process error flag*/
		if (last_join_status == RTW_JOINSTATUS_SCANNING) {
			error_flag = RTW_NONE_NETWORK;
			printf("\n\r[WLAN_SCENARIO_EXAMPLE] Cannot scan AP\n");
		} else if (last_join_status == RTW_JOINSTATUS_AUTHENTICATING) {
			error_flag = RTW_AUTH_FAIL;
			printf("\n\r[WLAN_SCENARIO_EXAMPLE] Connection fail caused by auth failure. Please check the AP and connect again\n");
		} else if (last_join_status == RTW_JOINSTATUS_AUTHENTICATED || last_join_status == RTW_JOINSTATUS_ASSOCIATING) {
			error_flag = RTW_ASSOC_FAIL;
			printf("\n\r[WLAN_SCENARIO_EXAMPLE] Connection fail caused by assiciation failure. Please check the AP and connect again\n");
		} else if (last_join_status == RTW_JOINSTATUS_ASSOCIATED || last_join_status == RTW_JOINSTATUS_4WAY_HANDSHAKING) {
			error_flag = RTW_4WAY_HANDSHAKE_TIMEOUT;
			wifi_get_disconn_reason_code(&reason_code);
			printf("\n\r[WLAN_SCENARIO_EXAMPLE] Connection fail caused by 4-way handshake failure. reason code=%d\n", reason_code);
		}
	}
	last_join_status = join_status;
}

static void scenario_5(void)
{
	printf("\n\n[WLAN_SCENARIO_EXAMPLE] Wi-Fi example scenario case 5...\n");

	/*********************************************************************************
	*	1. Enable Wi-Fi with STA mode
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Enable Wi-Fi\n");
	if (wifi_on(RTW_MODE_STA) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_on failed\n");
		return;
	}


	/*********************************************************************************
	*	2. Connect to AP using STA mode, check the connection result
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Connect to AP using STA mode\n");
	ssid = "Test_ap";
	password = "12345678";
	rtw_network_info_t connect_param = {0};
	u8 ret = 0;
	memcpy(connect_param.ssid.val, ssid, strlen(ssid));
	connect_param.ssid.len = strlen(ssid);
	connect_param.password = (unsigned char *)password;
	connect_param.password_len = strlen(password);
	connect_param.security_type = RTW_SECURITY_WPA2_AES_PSK;
	connect_param.joinstatus_user_callback = wifi_join_status_callback;
	ret = wifi_connect(&connect_param, 1);
	if (ret == RTW_SUCCESS) {
		ret = LwIP_DHCP(0, DHCP_START);
		// DHCP success
		if (ret == DHCP_ADDRESS_ASSIGNED) {
			printf("\n\r[WLAN_SCENARIO_EXAMPLE] DHCP success\n");
		}
		// DHCP fail, might caused by timeout or the AP did not enable DHCP server
		else if (ret == DHCP_TIMEOUT) {
			printf("\n\r[WLAN_SCENARIO_EXAMPLE] DHCP fail, might caused by timeout or the AP did not enable DHCP server\n");
		}
	} else if (ret == RTW_INVALID_KEY) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] Password length incorrect or not the same password as AP used\n");
	}


	/*********************************************************************************
	*	3. Show Wi-Fi information
	*********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Show Wi-Fi information\n");
	rtw_wifi_setting_t setting;
	wifi_get_setting(WLAN0_IDX, &setting);
	print_wifi_setting(WLAN0_NAME, &setting);


	/*********************************************************************************
	*	4. Get AP's RSSI
	*********************************************************************************/
	rtw_phy_statistics_t phy_statistics;
	wifi_fetch_phy_statistic(&phy_statistics);
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Get AP RSSI: %d\n", phy_statistics.rssi);
}

/**
 * @brief  Wi-Fi example for scenario case 6.
 * @note  Process Flow:
 *		- Enable Wi-Fi with STA mode
 *		- Scan network and handle the RSSI value (in dBm)
 */
static void scenario_6(void)
{
	printf("\n\n[WLAN_SCENARIO_EXAMPLE] Wi-Fi example scenario case 6...\n");

	/*********************************************************************************
	*	1. Enable Wi-Fi with STA mode
	**********************************************************************************/
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Enable Wi-Fi\n");
	if (wifi_on(RTW_MODE_STA) < 0) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_on failed\n");
		return;
	}


	/**********************************************************************************
	*	2. Scan network and handle the RSSI value
	**********************************************************************************/
	// Scan network and print the RSSI & SSID out.
	// Can refer to fATWS() in atcmd_wifi.c and scan_result_RSSI_handler() below.
	printf("\n\r[WLAN_SCENARIO_EXAMPLE] Scan network\n");
	rtw_scan_param_t scan_param;
	rtw_memset(&scan_param, 0, sizeof(rtw_scan_param_t));
	scan_param.scan_user_callback = scan_result_RSSI_handler;
	if (wifi_scan_networks(&scan_param, 0) != RTW_SUCCESS) {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: wifi_scan_networks() failed\n");
		return;
	}
}

// For processing the scanned result -> just output them.
// Can refer to fATWS() in atcmd_wifi.c.
static rtw_result_t scan_result_handler(unsigned int scanned_AP_num, void *user_data)
{
	/* To avoid gcc warnings */
	(void) user_data;

	rtw_scan_result_t *scanned_ap_info;
	char *scan_buf = NULL;
	int i = 0;

	if (scanned_AP_num == 0) {
		return RTW_ERROR;
	}

	scan_buf = (char *)rtw_zmalloc(scanned_AP_num * sizeof(rtw_scan_result_t));
	if (scan_buf == NULL) {
		printf("malloc scan buf fail for example_wlan_scenario\n");
		return RTW_ERROR;
	}

	if (wifi_get_scan_records(&scanned_AP_num, scan_buf) < 0) {
		rtw_mfree((uint8_t *)scan_buf, 0);
		return RTW_ERROR;
	}

	for (i = 0; i < scanned_AP_num; i++) {
		scanned_ap_info = (rtw_scan_result_t *)(scan_buf + i * sizeof(rtw_scan_result_t));
		scanned_ap_info->SSID.val[scanned_ap_info->SSID.len] = 0; /* Ensure the SSID is null terminated */

		printf("%d\t", (i + 1));
		printf("%s\t ", (scanned_ap_info->bss_type == RTW_BSS_TYPE_ADHOC) ? "Adhoc" : "Infra");
		printf("%02x:%02x:%02x:%02x:%02x:%02x", MAC_ARG(scanned_ap_info->BSSID.octet));
		printf(" %d\t ", scanned_ap_info->signal_strength);
		printf(" %d\t  ", scanned_ap_info->channel);
		printf(" %d\t  ", scanned_ap_info->wps_type);
		printf("%s\t\t ", (scanned_ap_info->security == RTW_SECURITY_OPEN) ? "Open" :
			   (scanned_ap_info->security == RTW_SECURITY_WEP_PSK) ? "WEP" :
			   (scanned_ap_info->security == RTW_SECURITY_WPA_TKIP_PSK) ? "WPA TKIP" :
			   (scanned_ap_info->security == RTW_SECURITY_WPA_AES_PSK) ? "WPA AES" :
			   (scanned_ap_info->security == RTW_SECURITY_WPA2_AES_PSK) ? "WPA2 AES" :
			   (scanned_ap_info->security == RTW_SECURITY_WPA2_TKIP_PSK) ? "WPA2 TKIP" :
			   (scanned_ap_info->security == RTW_SECURITY_WPA2_MIXED_PSK) ? "WPA2 Mixed" :
			   (scanned_ap_info->security == RTW_SECURITY_WPA_WPA2_MIXED) ? "WPA/WPA2 AES" :
			   (scanned_ap_info->security == RTW_SECURITY_WPA3_AES_PSK) ? "WPA3 AES" :
			   (scanned_ap_info->security == RTW_SECURITY_WPA2_WPA3_MIXED) ? "WP2/WPA3 AES" :
			   "Unknown");
		printf(" %s ", scanned_ap_info->SSID.val);
		printf("\r\n");
	}
	rtw_mfree((uint8_t *)scan_buf, 0);
	return RTW_SUCCESS;
}

// For processing the scanned result -> output RSSI & SSID.
// Can refer to fATWS() in atcmd_wifi.c.
static rtw_result_t scan_result_RSSI_handler(unsigned int scanned_AP_num, void *user_data)
{
	/* To avoid gcc warnings */
	(void) user_data;

	rtw_scan_result_t *scanned_ap_info;
	char *scan_buf = NULL;
	int i = 0;

	if (scanned_AP_num == 0) {
		return RTW_ERROR;
	}
	scan_buf = (char *)rtw_zmalloc(scanned_AP_num * sizeof(rtw_scan_result_t));
	if (scan_buf == NULL) {
		printf("malloc scan buf fail for example_wlan_scenario\n");
		return RTW_ERROR;
	}

	if (wifi_get_scan_records(&scanned_AP_num, scan_buf) < 0) {
		rtw_mfree((uint8_t *)scan_buf, 0);
		return RTW_ERROR;
	}

	for (i = 0; i < scanned_AP_num; i++) {
		scanned_ap_info = (rtw_scan_result_t *)(scan_buf + i * sizeof(rtw_scan_result_t));
		scanned_ap_info->SSID.val[scanned_ap_info->SSID.len] = 0; /* Ensure the SSID is null terminated */

		printf("%d\t", (i + 1));
		printf(" RSSI: %d\t", scanned_ap_info->signal_strength);
		printf(" SSID: %s", scanned_ap_info->SSID.val);
		printf("\r\n");
	}
	rtw_mfree((uint8_t *)scan_buf, 0);
	return RTW_SUCCESS;
}

static void example_wlan_scenario_thread(void *in_id)
{
	char *id = in_id;
	printf("\nExample: wlan_scenario \n");
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif
	// Wait for other task stable.
	vTaskDelay(4000);

	if (strcmp(id, "S") == 0)
		// Scan network.
	{
		scan_network();
	} else if (strcmp(id, "A") == 0)
		// Authentication example.
	{
		authentication();
	} else if (strcmp(id, "M1") == 0)
		// Mode switch case 1.
	{
		mode_switch_1();
	} else if (strcmp(id, "M2") == 0)
		// Mode switch case 2.
	{
		mode_switch_2();
	} else if (strcmp(id, "M3") == 0)
		// Mode switch case 3.
	{
		mode_switch_3();
	} else if (strcmp(id, "M4") == 0)
		// Mode switch case 4.
	{
		mode_switch_4();
	} else if (strcmp(id, "M5") == 0)
		// Mode switch case 5.
	{
		mode_switch_5();
	} else if (strcmp(id, "M6") == 0)
		// Mode switch case 6.
	{
		mode_switch_6();
	} else if (strcmp(id, "M7") == 0)
		// Mode switch case 7.
	{
		mode_switch_7();
	} else if (strcmp(id, "S1") == 0)
		// Scenario case 1.
	{
		scenario_1();
	} else if (strcmp(id, "S2") == 0)
		// Scenario case 2.
	{
		scenario_2();
	} else if (strcmp(id, "S3") == 0)
		// Scenario case 3.
	{
		scenario_3();
	} else if (strcmp(id, "S4") == 0)
		// Scenario case 4.
	{
		scenario_4();
	} else if (strcmp(id, "S5") == 0)
		// Scenario case 5.
	{
		scenario_5();
	} else if (strcmp(id, "S6") == 0)
		// Scenario case 6.
	{
		scenario_6();
	} else {
		printf("\n\r[WLAN_SCENARIO_EXAMPLE] ERROR: Invalid case identity\n");
	}

	vTaskDelete(NULL);
}

void example_wlan_scenario(char *id)
{
	if (xTaskCreate(example_wlan_scenario_thread, ((const char *)"example_wlan_scenario_thread"), 1024, (void *const) id, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate failed\n", __FUNCTION__);
	}
}

