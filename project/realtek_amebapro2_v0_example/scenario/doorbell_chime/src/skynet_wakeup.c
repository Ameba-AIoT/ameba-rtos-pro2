#include "FreeRTOS.h"
#include "main.h"
#include <example_entry.h>
#include "platform_autoconf.h"
#include "icc_api.h"
#include "sys_api_ext.h"
#include "wifi_conf.h"
#include <lwip_netconf.h>

#include "hal_power_mode.h"
#include "power_mode_api.h"
#include "isp_api.h"


#include "SKYNET_IOTAPI.h"
#include "app_setting.h"

#ifndef DOORBELL_CHIME_MODE_ENABLE

#define ICC_CMD_REQ_STANDBY                     (0x20)

extern st_GetWakeupInfo gstGetWakeupInfo;

extern char *skynet_UID;

static uint32_t interval_ms = 55000;//SKYNET_IOT_LOGIN_KEEPALIVE_INTERVAL_SECOND * 1000;
static uint32_t resend_ms = 10000;//3000;

static int gKeepAliveSock = 0;

extern struct netif xnetif[NET_IF_NUM];

static uint16_t server_port = SKYNET_IOT_WAKEUP_SERVER_PORT;

extern char gUID[25];
extern char gKey[6];
extern int loginFail;

void skynet_set_tcp_connected_pattern(wowlan_pattern_t *pattern)
{
	// This pattern make STA can be wake from a connected TCP socket
	memset(pattern, 0, sizeof(wowlan_pattern_t));

	char buf[32];
	char mac[6];
	char ip_protocol[2] = {0x08, 0x00}; // IP {08,00} ARP {08,06}
	char ip_ver[1] = {0x45};
	char tcp_protocol[1] = {0x06}; // 0x06 for tcp
	char tcp_port[2] = {(server_port >> 8) & 0xFF, server_port & 0xFF};
	char flag2[1] = {0x18}; // PSH + ACK
	uint8_t *ip = LwIP_GetIP(&xnetif[0]);
	u8 uc_mask[6] = {0x3f, 0x70, 0x80, 0xc0, 0x0F, 0x80};

	wifi_get_mac_address(buf);
	sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
	rtw_memcpy(pattern->eth_da, mac, 6);
	rtw_memcpy(pattern->eth_proto_type, ip_protocol, 2);
	rtw_memcpy(pattern->header_len, ip_ver, 1);
	rtw_memcpy(pattern->ip_proto, tcp_protocol, 1);
	rtw_memcpy(pattern->ip_da, ip, 4);
	rtw_memcpy(pattern->src_port, tcp_port, 2);
	rtw_memcpy(pattern->flag2, flag2, 1);
	rtw_memcpy(pattern->mask, uc_mask, 6);
}

static void skynet_task_sleep(void *param)
{
	char strKeepAliveCmd[8];
	//memset(strKeepAliveCmd, 0, sizeof(strKeepAliveCmd));
	//strcpy(strKeepAliveCmd, "Z");
	strKeepAliveCmd[0] = 'Z';
	int ret = wifi_set_tcp_keep_alive_offload(gKeepAliveSock, strKeepAliveCmd,
			  1, interval_ms, resend_ms, 1);

	printf("skynet_task_sleep start......ret[%d] gKeepAliveSock[%d]\r\n", ret, gKeepAliveSock);

	vTaskDelay(1000);

#if 1
	wowlan_pattern_t test_pattern;
	skynet_set_tcp_connected_pattern(&test_pattern);
	wifi_wowlan_set_pattern(test_pattern);
	vTaskDelay(2000);
#endif
	wifi_wowlan_set_dtimto(1, 1, 50, 10);

	if (rtl8195b_suspend(0) == 0) {
		printf("ENTER sleep mode......\r\n");
		//uint16_t wakeup_event = SLP_WLAN | SLP_GPIO;
		//uint16_t wakeup_event = BIT6;
		//icc_user_cmd_t cmd = { .cmd_b.cmd = ICC_CMD_REQ_STANDBY, .cmd_b.para0 = wakeup_event };
		//icc_cmd_send (cmd.cmd_w, 0, 1000 * 1000, NULL);
		send_icc_cmd_poweroff();

		while (1) {
			vTaskDelay(1000);
		}
	}

	printf("skynet_task_sleep -> exit......\r\n");

	vTaskDelete(NULL);
}

void skynet_create_sleep_task(void)
{
	if (xTaskCreate(skynet_task_sleep, "skynet_task_sleep", 1024, NULL, (tskIDLE_PRIORITY + 2), NULL) != pdPASS) {
		printf("cCreateTask(skynet_task_sleep) failed!\n");
	}
}

char *gWakeupUserData = "SKYNET to test wakeup function";

void skynet_WakeupAuthCB(int nSocketID, int nResult, void *pUserData)
{
	printf("[skynet_WakeupAuthCB] nSocketID = %d\r\n", nSocketID);
	printf("\tnResult = %d\r\n", nResult);
	printf("\tpUserData = %s\r\n", (char *)pUserData);

	if (nResult == 0) {
		gKeepAliveSock = nSocketID;
		skynet_create_sleep_task();
	} else {
		printf("skynet_WakeupAuthCB Fail\r\n");
		loginFail = 1;
	}
}

void skynet_wakeup_exec(void)
{
	int nRet;

	SKYNET_set_wakeup_auth_callback(skynet_WakeupAuthCB, (void *)gWakeupUserData);
	nRet = SKYNET_wakeup_auth(gUID, &gstGetWakeupInfo);
	printf("SKYNET_wakeup_auth() ret = %d, skynet_UID = %s\r\n", nRet, gUID);
}

#endif
