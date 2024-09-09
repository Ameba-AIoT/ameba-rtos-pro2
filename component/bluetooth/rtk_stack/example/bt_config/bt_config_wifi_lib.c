#if defined(CONFIG_PLATFORM_8721D)
#include "ameba_soc.h"
#endif
#include "bt_config_wifi.h"
#include "bt_config_wifi_lib.h"
#include <drv_types.h>

#define BC_LIB_DEBUG 0

#if BC_LIB_DEBUG
	#define BC_LIB_DBG_PREFIX			"\n\r[BT Config Wifi][LIB_DBG] "
	#define	BC_LIB_DBG(...)				printf(BC_LIB_DBG_PREFIX __VA_ARGS__);
	#define BC_LIB_DBG_NOPREFIX(...)	printf(__VA_ARGS__);
#else
	#define BC_LIB_DBG(...)
	#define BC_LIB_DBG_NOPREFIX(...)
#endif

#if BC_LIB_DEBUG
	#define BC_LIB_PREFIX		"\n\r[BT Config Wifi][LIB] "
	#define	BC_LIB_printf(...)	printf(BC_LIB_PREFIX __VA_ARGS__);
#else
	#define	BC_LIB_printf(...)	printf(__VA_ARGS__);
#endif

#define MAC_FMT "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC_ARG(x) ((uint8_t*)(x))[0],((uint8_t*)(x))[1],((uint8_t*)(x))[2],((uint8_t*)(x))[3],((uint8_t*)(x))[4],((uint8_t*)(x))[5]
#define htons(x) ((((x) & 0x00ffUL) << 8) | (((x) & 0xff00UL) >> 8))
#define ntohs(x) ((((x) & 0x00ffUL) << 8) | (((x) & 0xff00UL) >> 8))

uint8_t airsync_specific = 0;
airsync_send_data_handler p_airsync_send_data_handler = NULL;
uint16_t BC_cmd_task_stack_size = 1024;
static uint8_t BC_cmd_task_running = 0;
static _sema BC_buf_mutex = NULL;
static _xqueue BC_cmdQueue = NULL;
static struct task_struct BC_cmd_task_hdl;
static uint16_t BC_cmd_processing_type = 0;
static uint16_t BC_last_cmd = 0;
// Write Request: Scan
static struct _BC_scan_result *BC_APP_scan_result = NULL; // APP Scan result
static struct BC_wifi_scan_result *BC_user_scan_result = NULL; // User Scan result
// Write Request: Status
static uint8_t BC_configured = 0;
// Read Request
static uint16_t BC_scan_section_idx = 0;
static uint32_t BC_read_len = 0;
static uint8_t *BC_read_buf = NULL;

typedef enum {
	BC_APP_BAND_2G            = 0x0,
	BC_APP_BAND_5G            = 0x1,
	BC_APP_BAND_UNKNOWN       = 0x2,
} BC_APP_band_t;

typedef enum {
	BC_APP_SECURITY_OPEN           = 0,
	BC_APP_SECURITY_WPA2_AES_PSK   = 1,
	BC_APP_SECURITY_WEP_PSK        = 2,
	BC_APP_SECURITY_UNKNOWN        = 3,
} BC_APP_security_t;

typedef enum {
	BC_APP_STATE_DISABLED            = 0x0,
	BC_APP_STATE_IDLE                = 0x1,
	BC_APP_STATE_SCANNING            = 0x2, /* Defined in Spec, but not used*/
	BC_APP_STATE_STARTED             = 0x3, /* Defined in Spec, but not used*/
	BC_APP_STATE_CONNECTED           = 0x4,
	BC_APP_STATE_WAITFORKEY          = 0x5, /* Defined in Spec, but not used*/
	BC_APP_STATE_WRONG_PASSWORD      = 0xf,
} BC_APP_status_t;

/* TLV related */
/* ==========================*/
static uint16_t BC_get_TLV_tag(uint8_t *buf)
{
	uint16_t tag = ReadEF2Byte(buf);
	return ntohs(tag);
}

static uint8_t* BC_get_TLV_pValue(uint8_t *buf)
{
	uint8_t *pvalue = buf + BC_MSG_HDR_LEN;
	return pvalue;
}

static void BC_add_TLV_header(uint8_t *buf, uint32_t *buf_len, uint16_t tag, uint32_t data_len)
{
	WriteEF2Byte(buf, htons(tag));
	WriteEF2Byte(buf + 2, htons(data_len));
	*buf_len = BC_MSG_HDR_LEN + data_len;
}

static uint8_t* BC_search_TLV(uint8_t *buf, uint16_t target_tag, uint32_t buf_len, uint16_t *value_len)
{
	uint16_t tag, length;
	uint32_t offset = 0;

	while (offset < buf_len) {
		tag = ntohs(ReadEF2Byte(buf + offset));
		length = ntohs(ReadEF2Byte(buf + offset + 2));
		if (tag == target_tag) {
			if ((buf_len - offset) >= (BC_MSG_HDR_LEN + length)) {	// buf space > length
				*value_len = length;
				return buf + offset + BC_MSG_HDR_LEN;
			}
			else {
				break;
			}
		}
		offset += (BC_MSG_HDR_LEN + length);
	}
	return NULL;
}

static const char* BC_tag_string(uint16_t tag)
{
	switch (tag) {
		case TAG_BT_CMD_SCAN:
			return "Scan AP";
		case TAG_BT_CMD_REPLY_SCAN:
			return "Scan AP Result";
		case TAG_BT_CMD_CONNECT:
			return "Connect to remote AP";
		case TAG_BT_CMD_REPLY_CONNECT:
			return "Reply ACK for connect AP";
		case TAG_BT_CMD_GET_STATUS:
			return "Get Connection Status";
		case TAG_BT_CMD_REPLY_GET_STATUS:
			return "Reply Connection Status";
		case TAG_BT_CMD_GET_WLAN_BAND:
			return "Get Band Capability";
		case TAG_BT_CMD_REPLY_WLAN_BAND:
			return "Reply Band Capability";
		default:
			return "Unknown Command";
	}
}
/* End TLV related*/
/* ==========================*/


/* format conversion related */
/* ==========================*/
// the mapping function that mapping rtw_security_t to iPhone/Android app security type
BC_APP_security_t BC_translate_security_to_app(rtw_security_t user_security)
{
	BC_APP_security_t app_security = BC_APP_SECURITY_UNKNOWN;

	switch (user_security) {
		case RTW_SECURITY_OPEN:
			app_security = BC_APP_SECURITY_OPEN;
			break;
		case RTW_SECURITY_WPA_TKIP_PSK:
		case RTW_SECURITY_WPA_AES_PSK:
		case RTW_SECURITY_WPA_MIXED_PSK:
		case RTW_SECURITY_WPA2_AES_PSK:
		case RTW_SECURITY_WPA2_TKIP_PSK:
		case RTW_SECURITY_WPA2_MIXED_PSK:
		case RTW_SECURITY_WPA_WPA2_TKIP_PSK:
		case RTW_SECURITY_WPA_WPA2_AES_PSK:
		case RTW_SECURITY_WPA_WPA2_MIXED_PSK:
		case RTW_SECURITY_WPA3_AES_PSK:
		case RTW_SECURITY_WPA3_GCMP_PSK:
		case RTW_SECURITY_WPA2_WPA3_MIXED:		  
			app_security = BC_APP_SECURITY_WPA2_AES_PSK;
			break;
		case RTW_SECURITY_WEP_PSK:
		case RTW_SECURITY_WEP_SHARED:
			app_security = BC_APP_SECURITY_WEP_PSK;
			break;
		default:
			BC_LIB_DBG("[Error] unknown user_security (%x)\n\r",user_security);
	}
	return app_security;
}

rtw_security_t BC_translate_security_from_app(uint8_t app_security)
{
	rtw_security_t user_security = RTW_SECURITY_UNKNOWN;

	switch (app_security) {
		case BC_APP_SECURITY_OPEN:
			user_security = RTW_SECURITY_OPEN;
			break;
		case BC_APP_SECURITY_WPA2_AES_PSK:
			user_security = RTW_SECURITY_WPA2_AES_PSK;
			break;
		case BC_APP_SECURITY_WEP_PSK:
			user_security = RTW_SECURITY_WEP_PSK;
			break;
		default:
			break;
	}
	return user_security;
}

BC_band_t BC_translate_band_from_app(uint8_t app_band)
{
	BC_band_t user_band = BC_BAND_UNKNOWN;

	switch (app_band) {
		case BC_APP_BAND_2G:
			user_band = BC_BAND_2G;
			break;
		case BC_APP_BAND_5G:
			user_band = BC_BAND_5G;
			break;
		default:
			break;
	}
	return user_band;
}

uint8_t BC_translate_state_to_app(BC_status_t user_state)
{
	BC_APP_status_t app_state = BC_APP_STATE_IDLE;

	switch (user_state) {
		case BC_STATE_DISABLED:
			app_state = BC_APP_STATE_DISABLED;
			break;
	  	case BC_STATE_IDLE:
			app_state = BC_APP_STATE_IDLE;
			break;
		case BC_STATE_CONNECTED:
			app_state = BC_APP_STATE_CONNECTED;
			break;
		case BC_STATE_WRONG_PASSWORD:
			app_state = BC_APP_STATE_WRONG_PASSWORD;
			break;
		default:
			break;
	}
	return app_state;
}

/* End format conversion related*/
/* ==========================*/


/* request related */
/* ==========================*/
static int BC_req_unknown(void)
{
	int ret = -1;
	uint8_t* rpl_pValue = NULL;
	char *result = "unknown request";

	if (airsync_specific)
		memset(BC_read_buf, 0, BC_MAX_RPL_LEN_AIRSYNC);
	else
		memset(BC_read_buf, 0, BC_MAX_RPL_LEN);
	rpl_pValue = BC_get_TLV_pValue(BC_read_buf);

	memcpy(rpl_pValue, result, strlen(result));

	BC_add_TLV_header(BC_read_buf, &BC_read_len, TAG_BT_CMD_REPLY_FAIL, strlen(result));

	return ret;
}

static int BC_req_band(uint8_t *req, uint16_t req_len)
{
	int ret = -1;
	uint8_t* rpl_pValue = NULL;
	struct _BC_band_info Band_info;
	BC_band_t user_band = BC_BAND_UNKNOWN;

	BC_configured = 0;

	if (req[0] != 0) {
		BC_LIB_printf("Invalid command\n\r");
		return ret;
	}

	user_band = BC_req_band_hdl(); // user defined function

	if (airsync_specific)
		memset(BC_read_buf, 0, BC_MAX_RPL_LEN_AIRSYNC);
	else
		memset(BC_read_buf, 0, BC_MAX_RPL_LEN);
	rpl_pValue = BC_get_TLV_pValue(BC_read_buf);

	Band_info.support_2_4g = (user_band & BC_BAND_2G)? 1: 0;
	Band_info.support_5g = (user_band & BC_BAND_5G)? 1: 0;
	Band_info.product_type = 1; // Ameba: 1

	memcpy(rpl_pValue, &Band_info, sizeof(struct _BC_band_info));

	BC_add_TLV_header(BC_read_buf, &BC_read_len, TAG_BT_CMD_REPLY_WLAN_BAND, sizeof(struct _BC_band_info));
	return 0;
}

static uint8_t BC_dbm_to_percentage(signed short dbm)
{
	uint8_t percentage = 0;
	percentage = (uint8_t) (dbm + 95);
	percentage = (uint8_t) ((percentage *20)/18);
	return percentage;
}

static void BC_scan_result_convert(struct BC_wifi_scan_result* user_scan_result, struct _BC_scan_result *app_scan_result)
{
	BC_LIB_DBG("BC_APP_scan_result->number =  %d\n\r", app_scan_result->number);
	for(int i=0; i<app_scan_result->number; i++) {
		app_scan_result->bss_info[i].security_type = BC_translate_security_to_app(user_scan_result->ap_info[i].security);
		memcpy(app_scan_result->bss_info[i].BSSID, user_scan_result->ap_info[i].BSSID.octet,6);
		memcpy(app_scan_result->bss_info[i].SSID, user_scan_result->ap_info[i].SSID.val, user_scan_result->ap_info[i].SSID.len);
		app_scan_result->bss_info[i].channelNumber = (uint8_t) user_scan_result->ap_info[i].channel;
		app_scan_result->bss_info[i].rssi = BC_dbm_to_percentage(user_scan_result->ap_info[i].signal_strength);

		BC_LIB_DBG_NOPREFIX("[%d] SSID: [%s], BSSID: ["MAC_FMT"], channel: [%d], RSSI: [%d (%d dBm)]\r\n",
			i,app_scan_result->bss_info[i].SSID, MAC_ARG(app_scan_result->bss_info[i].BSSID),
			app_scan_result->bss_info[i].channelNumber, app_scan_result->bss_info[i].rssi,
			user_scan_result->ap_info[i].signal_strength);
	}
}

static int BC_req_scan(uint8_t *req, uint16_t req_len)
{
	int ret = -1;
	uint8_t* rpl_pValue = NULL;
	uint8_t app_BC_band = BC_APP_BAND_UNKNOWN;
	BC_band_t user_BC_band = BC_BAND_UNKNOWN;

	BC_configured = 0;

	app_BC_band = req[0];
	user_BC_band = BC_translate_band_from_app(app_BC_band);
	if (user_BC_band == BC_BAND_UNKNOWN) {
		BC_LIB_printf("Invalid command\n\r");
		return ret;
	}

	memset(BC_user_scan_result, 0, sizeof(struct BC_wifi_scan_result));
	ret = BC_req_scan_hdl(user_BC_band, BC_user_scan_result);
	if (ret == -1) {
		BC_LIB_DBG("Scan Fail\n\r");
		return ret;
	}

	memset(BC_APP_scan_result, 0, sizeof(struct _BC_scan_result));
	BC_APP_scan_result->band = app_BC_band;
	if (airsync_specific) {
		BC_APP_scan_result->number = (BC_user_scan_result->ap_num < BC_MAX_WIFI_SCAN_AP_NUM_AIRSYNC)? BC_user_scan_result->ap_num : BC_MAX_WIFI_SCAN_AP_NUM_AIRSYNC;
	} else {
		BC_APP_scan_result->number = (BC_user_scan_result->ap_num < BC_MAX_WIFI_SCAN_AP_NUM)? BC_user_scan_result->ap_num : BC_MAX_WIFI_SCAN_AP_NUM;
	}
	BC_scan_result_convert(BC_user_scan_result, BC_APP_scan_result);

	if (airsync_specific)
		memset(BC_read_buf, 0, BC_MAX_RPL_LEN_AIRSYNC);
	else
		memset(BC_read_buf, 0, BC_MAX_RPL_LEN);
	rpl_pValue = BC_get_TLV_pValue(BC_read_buf);

	WriteEF1Byte(rpl_pValue, app_BC_band);
	// other information of scan result is filled into BC_read_buf later (when processing Read Requets)

	BC_add_TLV_header(BC_read_buf, &BC_read_len, TAG_BT_CMD_REPLY_SCAN, sizeof(struct _BC_scan_result));
	return 0;
}

static int BC_req_connect(uint8_t *req, uint16_t req_len)
{
	int ret = -1;
	uint8_t* rpl_pValue = NULL;
	struct _BC_connect_AP_info apInfo, *p_apInfo = &apInfo;
	BC_band_t user_BC_band = BC_BAND_UNKNOWN;
	rtw_security_t user_BC_security = RTW_SECURITY_OPEN;

	memcpy(p_apInfo, req, sizeof(struct _BC_connect_AP_info));
	user_BC_security = BC_translate_security_from_app(p_apInfo->security_type);
	user_BC_band = BC_translate_band_from_app(p_apInfo->band);
	if ((user_BC_security == RTW_SECURITY_UNKNOWN)|| (user_BC_band == BC_BAND_UNKNOWN)) {
		BC_LIB_printf("Invalid command\n\r");
		return ret;
	}

	BC_req_connect_hdl(p_apInfo->SSID, p_apInfo->password, p_apInfo->BSSID, user_BC_security, user_BC_band);
	BC_configured = 1;
	// send ACK of CONNECT REQ, no matter connect success or not
	if (airsync_specific)
		memset(BC_read_buf, 0, BC_MAX_RPL_LEN_AIRSYNC);
	else
		memset(BC_read_buf, 0, BC_MAX_RPL_LEN);
	rpl_pValue = BC_get_TLV_pValue(BC_read_buf);

	WriteEF1Byte(rpl_pValue, 1);

	BC_add_TLV_header(BC_read_buf, &BC_read_len, TAG_BT_CMD_REPLY_CONNECT, 1);

	return 0;
}

int BC_req_status(uint8_t *req, uint16_t req_len)
{
	int ret = -1;
	uint8_t* rpl_pValue = NULL;
	struct _BC_status BC_Status;
	BC_status_t user_BC_status = BC_STATE_IDLE;
	rtw_security_t user_security = RTW_SECURITY_OPEN;
	uint8_t user_channel = 0;
	int user_rssi_dbm = 0;

	if (airsync_specific)
		memset(BC_read_buf, 0, BC_MAX_RPL_LEN_AIRSYNC);
	else
		memset(BC_read_buf, 0, BC_MAX_RPL_LEN);
	rpl_pValue = BC_get_TLV_pValue(BC_read_buf);

	BC_req_status_hdl(&user_BC_status, BC_Status.SSID, BC_Status.BSSID, &user_security, &user_channel, &user_rssi_dbm);

	BC_Status.status = BC_translate_state_to_app(user_BC_status);
	// user_BC_status got from wifi_disconn_hdl may be wrong, so just reply IDLE (not connected) instead of WRONG PASSWORD
	if (BC_Status.status == BC_APP_STATE_WRONG_PASSWORD)
		BC_Status.status = BC_APP_STATE_IDLE;
	BC_Status.security_type = BC_translate_security_to_app(user_security);
	BC_Status.band = (user_channel <= 14)? BC_APP_BAND_2G : BC_APP_BAND_5G;
	BC_Status.rssi = BC_dbm_to_percentage(user_rssi_dbm);
	BC_Status.config_status = BC_configured;

	if (BC_Status.status == BC_APP_STATE_CONNECTED) {
		if (BC_Status.security_type == BC_APP_SECURITY_UNKNOWN) {
			BC_LIB_printf("Unknown security (%x)\n\r", user_security);
			return ret;
		}
		BC_LIB_DBG("[%s] Connected\n\r", __FUNCTION__);
		BC_LIB_DBG_NOPREFIX("  SSID: [%s], BSSID: ["MAC_FMT"], security_type: [0x%02x], rssi: [%d%% (%d dBm)]\n\r",
							BC_Status.SSID, MAC_ARG(BC_Status.BSSID), BC_Status.security_type, BC_Status.rssi, user_rssi_dbm);
	} else {
		BC_LIB_DBG("[%s] Not connected, State 0x%x\n\r", __FUNCTION__, BC_Status.status);
	}

	memcpy(rpl_pValue, &BC_Status, sizeof(struct _BC_status));

	BC_add_TLV_header(BC_read_buf, &BC_read_len, TAG_BT_CMD_REPLY_GET_STATUS, sizeof(struct _BC_status));
	return 0;
}

int BC_handle_write_request(uint8_t *req, uint32_t req_len)
{
	rtw_down_sema(&BC_buf_mutex);

	int ret = -1;
	uint16_t tag = 0;
	uint8_t* tlv_pValue = NULL;
	uint16_t tlv_len = 0;

	tag = BC_get_TLV_tag(req);

	BC_LIB_DBG("Request %d bytes -> [%s]", req_len, BC_tag_string(tag));
	for(uint32_t i=0; i<req_len; i++) {
		if (i%16 == 0) {
			BC_LIB_DBG_NOPREFIX("\n\r");
		}
		BC_LIB_DBG_NOPREFIX("%02X ", req[i]);
	}
	BC_LIB_DBG_NOPREFIX("\n\r");

	tlv_pValue = BC_search_TLV(req, tag, req_len, &tlv_len);
	if (tlv_pValue == NULL) {
		BC_LIB_printf("Invalid command\n\r");
		tag = TAG_BT_CMD_REPLY_FAIL;
	}

	BC_cmd_processing_type = tag;

	switch (tag) {
		case TAG_BT_CMD_GET_WLAN_BAND:
			ret = BC_req_band(tlv_pValue, tlv_len);
			break;
		case TAG_BT_CMD_SCAN:
			BC_scan_section_idx = 0;
			ret = BC_req_scan(tlv_pValue, tlv_len);
			break;
		case TAG_BT_CMD_CONNECT:
			if (BC_last_cmd == TAG_BT_CMD_CONNECT) {
				BC_LIB_DBG("Last command was Connect, ignore duplicated Connect Reqeuest\n\r");
				ret = 0;
			}
			else {
				ret = BC_req_connect(tlv_pValue, tlv_len);
			}
			break;
		case TAG_BT_CMD_GET_STATUS:
			ret = BC_req_status(tlv_pValue, tlv_len);
			break;
		default:
			BC_LIB_printf("Invalid command\n\r");
			break;
	}
	if (ret < 0)
	{
		BC_LIB_DBG("[%s] %s ret = %d\n\r", __FUNCTION__, BC_tag_string(tag), ret);
		ret = BC_req_unknown();
	}

	BC_last_cmd = tag;
	rtw_up_sema(&BC_buf_mutex);

	return ret;
}

/* End request related*/
/* ==========================*/


/* Task related */
/* ==========================*/
void BC_cmd_task(void *arg)
{
	( void ) arg;
	int ret = -1;

	bt_config_create_secure_context();
	BC_command cmd;
	BC_cmd_task_running = 1;

	while (BC_cmd_task_running) {
		if (rtw_pop_from_xqueue(&BC_cmdQueue, &cmd, 0xFFFFFFFF) == 0) {
			ret = BC_handle_write_request(cmd.data, cmd.len);
			if (airsync_specific && p_airsync_send_data_handler) {
				if (ret >= 0) {
					uint8_t *read_buf = NULL;
					uint32_t read_buf_len = 0;

					BC_handle_read_request(&read_buf, &read_buf_len, 0);
					if (read_buf != NULL) {
						p_airsync_send_data_handler(read_buf, read_buf_len);
					}
				}
			}
		}
		rtw_yield_os();
	}

	BC_LIB_DBG("\n\rDelete BC_cmd_task\n\r");
	rtw_thread_exit();
}
/* End task related*/
/* ==========================*/


/* API exposed to User */
/* ==========================*/
void BC_cmd_task_init(void)
{
	if (BC_cmd_task_running) {
		BC_LIB_printf("BC_cmd_task already on\n\r");
		return;
	}

	if (airsync_specific)
		BC_read_buf = (uint8_t *)rtw_malloc(BC_MAX_RPL_LEN_AIRSYNC);
	else
		BC_read_buf = (uint8_t *)rtw_malloc(BC_MAX_RPL_LEN);
	if (BC_read_buf == NULL) {
		BC_LIB_printf("Failed to create BC_read_buf\n\r");
		return;
	}

	BC_APP_scan_result = (struct _BC_scan_result*)rtw_malloc(sizeof(struct _BC_scan_result)); // internal scan result for APP
	if (BC_APP_scan_result == NULL) {
		BC_LIB_printf("Failed to create BC_APP_scan_result\n\r");
		return;
	}

	BC_user_scan_result = (struct BC_wifi_scan_result*)rtw_malloc(sizeof(struct BC_wifi_scan_result)); // scan result from User
	if (BC_user_scan_result == NULL) {
		BC_LIB_printf("Failed to create BC_user_scan_result\n\r");
		return;
	}

	rtw_init_xqueue(&BC_cmdQueue, "BC_cmdQueue", sizeof(BC_command), 1);	// pass the cmd string
	if (BC_cmdQueue == NULL) {
		BC_LIB_printf("Failed to create cmdQueue\n\r");
		return;
	}

	rtw_init_sema(&BC_buf_mutex, 1);
	if (BC_buf_mutex == NULL) {
		BC_LIB_printf("Failed to create BC_buf_mutex\n\r");
		return;
	}

	if(rtw_create_task(&BC_cmd_task_hdl, ((const char*)"BC_cmd_task"), BC_cmd_task_stack_size, 1, BC_cmd_task, NULL) != pdPASS) {
		BC_LIB_printf("Failed to create BC_cmd_task\n\r");
		return;
	}
}

void BC_cmd_task_deinit(void)
{
	if (!BC_cmd_task_running) {
		BC_LIB_printf("BC_cmd_task is not running\n\r");
		return;
	}

	BC_cmd_task_running = 0;
	rtw_delete_task(&BC_cmd_task_hdl);
	rtw_deinit_xqueue(&BC_cmdQueue);
	rtw_free_sema(&BC_buf_mutex);
	rtw_free(BC_read_buf);
	rtw_free(BC_APP_scan_result);
	rtw_free(BC_user_scan_result);
	BC_LIB_DBG("BC_cmd_task deinitalized\n\r");
}

void BC_send_cmd(uint8_t *cmd, uint32_t len)
{
	if (!BC_cmd_task_running)
		return;

	BC_command BC_cmd;
	uint16_t tag = 0;

	memset(&BC_cmd, 0, sizeof(BC_cmd));
	BC_cmd.len = (len < BC_MAX_REQ_LEN)? len : BC_MAX_REQ_LEN;
	memcpy(BC_cmd.data, cmd, BC_cmd.len);
	rtw_push_to_xqueue(&BC_cmdQueue, &BC_cmd, 0xFFFFFFFF);

	tag = BC_get_TLV_tag(BC_cmd.data);
	// wait until the incomming cmd being processed
	while (BC_cmd_processing_type != tag)
		rtw_msleep_os(20);
}

void BC_handle_read_request(uint8_t **pRead_buf, uint32_t *pRead_buf_len, uint16_t read_offset)
{
	rtw_down_sema(&BC_buf_mutex);

	uint16_t tag = BC_get_TLV_tag(BC_read_buf);

	if (read_offset != 0) {
		// Just return current BC_read_buf
		BC_LIB_DBG("Read Long continue (offset %d)\n\r", read_offset);
		*pRead_buf = BC_read_buf;
		*pRead_buf_len = (tag == TAG_BT_CMD_REPLY_SCAN)? BC_SCAN_SECTION_LEN : BC_read_len;

		rtw_up_sema(&BC_buf_mutex);
		return;
	}

	if (tag == TAG_BT_CMD_REPLY_SCAN) {
		// scan result data length = 2633 bytes
		// send 3 ap info at a time => 64/3 = 22 times, 1~21: 3 ap info, 22: 1 info
		// header = type (2) + length (2) + band (1) + apNum (4) = 9
		// each ap info = 41
		// section length = 9+41*3 = 132 bytes

		int max_section = 0;
		int unit, max_ap;

		if (airsync_specific) {
			unit = BC_APP_scan_result->number;
			max_ap = BC_APP_scan_result->number;
		} else {
			unit = BC_SCAN_RESLUT_UNIT;
			max_ap = BC_MAX_WIFI_SCAN_AP_NUM;
		}

		BC_scan_section_idx++; // index strats from 1
		max_section = max_ap / unit;
		if ( max_ap % unit != 0)
			max_section ++;

		BC_LIB_DBG("Sending Site Survey Result...(%d/%d)\n\r", BC_scan_section_idx, max_section);

		// modify apNum (int) [5][6][7][8] to section index (short) [5][6] + total (short) [7][8]
		uint16_t section_idx = htons(BC_scan_section_idx*unit);
		uint16_t ap_num = htons((uint16_t)BC_APP_scan_result->number);
		WriteEF2Byte(BC_read_buf + 5, section_idx);
		WriteEF2Byte(BC_read_buf + 7, ap_num);

		if (BC_scan_section_idx == max_section && max_ap % unit != 0) {	// last section, reset;
			memcpy(BC_read_buf+9,&BC_APP_scan_result->bss_info[(BC_scan_section_idx-1)*unit], (max_ap%unit) * sizeof(struct _BC_bss_info));
			BC_scan_section_idx = 0;
		}
		else {
			memcpy(BC_read_buf+9,&BC_APP_scan_result->bss_info[(BC_scan_section_idx-1)*unit], unit * sizeof(struct _BC_bss_info));
		}

		*pRead_buf = BC_read_buf;
		if (airsync_specific) {
			*pRead_buf_len = 9 + 41 * BC_APP_scan_result->number;
		} else {
			*pRead_buf_len = BC_SCAN_SECTION_LEN;
		}
	}
	else {
		*pRead_buf = BC_read_buf;
		*pRead_buf_len = BC_read_len;
	}

	BC_LIB_DBG("Reply %d bytes -> [%s]", *pRead_buf_len, BC_tag_string(tag));
	for(uint32_t i=0; i < *pRead_buf_len; i++) {
		if (i%16 == 0) {
			BC_LIB_DBG_NOPREFIX("\n\r");
		}
		BC_LIB_DBG_NOPREFIX("%02X ", (*pRead_buf)[i]);
	}
	BC_LIB_DBG_NOPREFIX("\n\r");

	rtw_up_sema(&BC_buf_mutex);
}
/* End API exposed to User*/
/* ==========================*/