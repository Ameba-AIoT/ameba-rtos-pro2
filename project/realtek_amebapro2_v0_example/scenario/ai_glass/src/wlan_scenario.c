#include <platform_opts.h>
#include "FreeRTOS.h"
#include "task.h"
#include <platform_stdlib.h>
#include <lwip_netconf.h>
#include "wifi_constants.h"
#include "lwip_netconf.h"
#include "wifi_conf.h"
#include "dhcp/dhcps.h"
#include "wifi_wps_config.h"
#include "osdep_service.h"
#include "wlan_scenario.h"
#include <httpd/httpd.h>
#include "media_filesystem.h"

#if CONFIG_LWIP_LAYER
extern struct netif xnetif[NET_IF_NUM];
#endif
static rtw_softap_info_t softAP_config = {0};
static uint8_t wifi_pass_word[MAX_AP_PASSWORD_LEN] = {0};

static void pingpong_cb(struct httpd_conn *conn)
{
	char *user_agent = NULL;

	// test log to show brief header parsing
	httpd_conn_dump_header(conn);

	// test log to show extra User-Agent header field
	if (httpd_request_get_header_field(conn, (char *)"User-Agent", &user_agent) != -1) {
		printf("\nUser-Agent=[%s]\n", user_agent);
		httpd_free(user_agent);
	}

	// GET homepage
	if (httpd_request_is_method(conn, (char *)"GET")) {
		httpd_response_write_header_start(conn, (char *)"200 OK", (char *)"text/plain", 0);
		httpd_response_write_header(conn, (char *)"Connection", (char *)"close");
		httpd_response_write_header_finish(conn);
	} else {
		// HTTP/1.1 405 Method Not Allowed
		httpd_response_method_not_allowed(conn, NULL);
	}

	httpd_conn_close(conn);
}

static void media_list_cb(struct httpd_conn *conn)
{
	char *user_agent = NULL;
	uint16_t file_num = 0;
	char *file_list = NULL;

	// test log to show brief header parsing
	httpd_conn_dump_header(conn);

	// test log to show extra User-Agent header field
	if (httpd_request_get_header_field(conn, (char *)"User-Agent", &user_agent) != -1) {
		printf("\nUser-Agent=[%s]\n", user_agent);
		httpd_free(user_agent);
	}

	// GET homepage
	if (httpd_request_is_method(conn, (char *)"GET")) {
		const char *extensions[] = { ".mp4", ".csv", ".jpeg", ".jpg" };
		uint16_t num_extensions = sizeof(extensions) / sizeof(extensions[0]);
		cJSON *list_json = extdisk_get_filelist("", &file_num, extensions, num_extensions, "ai_snapshot.jpg");
		if (list_json != NULL) {
			file_list = cJSON_Print(list_json);
			cJSON_Delete(list_json);
			//printf("%s\r\n", file_list);
		} else {
			printf("file list is NULL\n");
		}
		uint32_t file_list_len = strlen(file_list);
		httpd_response_write_header_start(conn, (char *)"200 OK", (char *)"text/plain", file_list_len);
		httpd_response_write_header(conn, (char *)"Connection", (char *)"close");
		httpd_response_write_header_finish(conn);
		httpd_response_write_data(conn, (uint8_t *)file_list, file_list_len);
	} else {
		// HTTP/1.1 405 Method Not Allowed
		httpd_response_method_not_allowed(conn, NULL);
	}

	httpd_conn_close(conn);
}

#define HTTP_DATA_BUF_SIZE 1024
static uint8_t data_buf[HTTP_DATA_BUF_SIZE] = {0};
static int httpd_request_get_path_key(struct httpd_conn *conn, const char *key, char **value)
{
	int ret = 0;
	size_t value_len;

	*value = NULL;

	if (conn->request.path) {
		uint8_t *ptr = conn->request.path + 1;
		uint8_t *ptr_tmp = NULL;

		while (ptr < (conn->request.path + conn->request.path_len)) {
			if (memcmp(ptr, key, strlen(key)) == 0) {
				ptr = ptr + strlen(key);
				ptr_tmp = ptr;
				while (ptr < (conn->request.path + conn->request.path_len)) {
					ptr ++;
				}

				if (ptr - ptr_tmp) {
					value_len = ptr - ptr_tmp;
					*value = (char *) malloc(value_len + 1);
					if (*value) {
						memset(*value, 0, value_len + 1);
						memcpy(*value, ptr_tmp, value_len);
					} else {
						printf("ERROR: malloc fail");
						goto exit;
					}
				}
				break;
			}
			ptr ++;
		}
	}

exit:
	if (*value == NULL) {
		ret = -1;
		printf("no value found");
	}

	return ret;
}

static void media_getfile_cb(struct httpd_conn *conn)
{
	char *user_agent = NULL;
	char *filename = NULL;
	FILE  *http_file = NULL;

	// test log to show brief header parsing
	httpd_conn_dump_header(conn);

	// test log to show extra User-Agent header field
	if (httpd_request_get_header_field(conn, (char *)"User-Agent", &user_agent) != -1) {
		printf("\nUser-Agent=[%s]\n", user_agent);
		httpd_free(user_agent);
	}

	// GET homepage
	if (httpd_request_is_method(conn, (char *)"GET")) {
		if ((httpd_request_get_path_key(conn, (char *)"media/", &filename) != -1)) {
			// write HTTP response
			httpd_response_write_header_start(conn, (char *)"200 OK", (char *)"text/plain", 0);
			httpd_response_write_header(conn, (char *)"Connection", (char *)"close");
			httpd_response_write_header_finish(conn);

			http_file = extdisk_fopen(filename, "r");
			if (!http_file) {
				printf("Open file %s failed\r\n", filename);
				goto httpd_conn_end;
			} else {
				printf("Open file %s success\r\n", filename);
			}
			int br = 0;
			extdisk_fseek(http_file, 0, SEEK_SET);
			while (1) {
				br = extdisk_fread(data_buf, 1, HTTP_DATA_BUF_SIZE, http_file);
				if (br < 0) {
					printf("Read ERROR\r\n");
					goto httpd_conn_end;
				} else {
					int ret = 0;
					int send_timeout = 3000;
					if (conn->sock != -1) {
						setsockopt(conn->sock, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));
					}
					ret = httpd_response_write_data(conn, data_buf, br);
					if (ret <= 0) {
						printf("http error ret = %d\r\n", ret);
						goto httpd_conn_end;
					}
					if (br != HTTP_DATA_BUF_SIZE) {
						break;
					}
				}
			}
			extdisk_fclose(http_file);
			http_file = NULL;
			extdisk_remove(filename);
		} else {
			// HTTP/1.1 400 Bad Request
			httpd_response_bad_request(conn, (char *)"Bad Request");
		}
	} else {
		// HTTP/1.1 405 Method Not Allowed
		httpd_response_method_not_allowed(conn, NULL);
	}
httpd_conn_end:
	if (filename) {
		httpd_free(filename);
		filename = NULL;
	}
	if (http_file) {
		extdisk_fclose(http_file);
		http_file = NULL;
	}
	httpd_conn_close(conn);
}

int wifi_get_ap_setting(rtw_softap_info_t *wifi_cfg)
{
	if (wifi_cfg) {
		wifi_cfg->ssid.len = softAP_config.ssid.len;
		memcpy(wifi_cfg->ssid.val, softAP_config.ssid.val, wifi_cfg->ssid.len);
		wifi_cfg->hidden_ssid = softAP_config.hidden_ssid;
		wifi_cfg->security_type = softAP_config.security_type;
		memcpy(wifi_cfg->password, softAP_config.password, softAP_config.password_len);
		wifi_cfg->password_len = softAP_config.password_len;
		wifi_cfg->channel = softAP_config.channel;
		return WLAN_SET_OK;
	}
	return WLAN_SET_FAIL;
}

int wifi_disable_ap_mode(void);

static void deinit_dhcp(void)
{
	// Enable Wi-Fi with AP mode
#if CONFIG_LWIP_LAYER
	dhcps_deinit();
	uint32_t addr = WIFI_MAKEU32(AI_GLASS_AP_IP_ADDR0, AI_GLASS_AP_IP_ADDR1, AI_GLASS_AP_IP_ADDR2, AI_GLASS_AP_IP_ADDR3);
	uint32_t netmask = WIFI_MAKEU32(AI_GLASS_AP_NETMASK_ADDR0, AI_GLASS_AP_NETMASK_ADDR1, AI_GLASS_AP_NETMASK_ADDR2, AI_GLASS_AP_NETMASK_ADDR3);
	uint32_t gw = WIFI_MAKEU32(AI_GLASS_AP_GW_ADDR0, AI_GLASS_AP_GW_ADDR1, AI_GLASS_AP_GW_ADDR2, AI_GLASS_AP_GW_ADDR3);
	LwIP_SetIP(0, addr, netmask, gw);
#endif
}

int wifi_enable_ap_mode(const char *ssid, const char *password, int channel, int timeout)
{
	printf("AI glass wifi_enable_ap_mode\r\n");
#if CONFIG_INIT_NET
#if CONFIG_LWIP_LAYER
	// Initilaize the LwIP stack, if the LwIP is not initalized yet
	extern int lwip_init_done;
	if (!lwip_init_done) {
		LwIP_Init();
	}
#endif
#endif

	printf("AI glass Enable Wi-Fi with AP mode\r\n");
	extern rtw_mode_t wifi_mode;
	if (wifi_mode == RTW_MODE_AP && wifi_is_running(WLAN0_IDX)) {
		if (strncmp((const char *)softAP_config.ssid.val, (const char *)ssid, softAP_config.ssid.len) == 0 &&
			strncmp((const char *)softAP_config.password, (const char *)password, softAP_config.password_len) == 0) {
			goto set_http;
		} else {
			wifi_disable_ap_mode();
			deinit_dhcp();
		}
	} else {
		deinit_dhcp();
		if (wifi_is_running(WLAN0_IDX)) {
			if (wifi_set_mode(RTW_MODE_AP) < 0) {
				printf("AI glass ERROR: wifi change mode failed\r\n");
				return WLAN_SET_FAIL;
			}
		} else {
			if (wifi_on(RTW_MODE_AP) < 0) {
				printf("AI glass ERROR: wifi_on failed\r\n");
				return WLAN_SET_FAIL;
			}
		}
	}

	// Start AP
	printf("AI glass Start AP\r\n");
	softAP_config.ssid.len = strlen(ssid);
	memcpy(softAP_config.ssid.val, (char *)ssid, softAP_config.ssid.len);

	memset(wifi_pass_word, 0x00, MAX_AP_PASSWORD_LEN);
	softAP_config.password_len = (strlen(password) > (MAX_AP_PASSWORD_LEN - 1) ? (MAX_AP_PASSWORD_LEN - 1) : strlen(password));
	memcpy(wifi_pass_word, password, softAP_config.password_len);
	softAP_config.password = (unsigned char *)wifi_pass_word;

	softAP_config.channel = channel;
	softAP_config.security_type = RTW_SECURITY_WPA2_AES_PSK;
	if (wifi_start_ap(&softAP_config) < 0) {
		printf("AI glass ERROR: wifi_start_ap failed\r\n");
		return WLAN_SET_FAIL;
	}

	// Check AP running
	printf("AI glass Check AP running\r\n");
	while (1) {
		rtw_wifi_setting_t setting;
		wifi_get_setting(WLAN0_IDX, &setting);
		if (strlen((char *)setting.ssid) > 0) {
			if (strcmp((const char *)setting.ssid, (const char *)ssid) == 0) {
				printf("AI glass %s started\r\n", ssid);
				break;
			}
		}
		if (timeout == 0) {
			printf("AI glass ERROR: Start AP timeout\r\n");
			return WLAN_SET_FAIL;
		}
		vTaskDelay(1 * configTICK_RATE_HZ);
		timeout --;
	}

	// Start DHCP server
	printf("AI glass Start DHCP server\r\n");

#if CONFIG_LWIP_LAYER
	dhcps_init(&xnetif[0]);
#endif
	httpd_reg_page_callback((char *)"/pingpong", pingpong_cb);
	httpd_reg_page_callback((char *)"/media-list", media_list_cb);
	httpd_reg_page_callback((char *)"/media/*", media_getfile_cb);

set_http:
	if (!httpd_is_running()) {
		httpd_setup_idle_timeout(HTTPD_CONNECT_TIMEOUT);
		if (httpd_start(8080, 5, 4096, HTTPD_THREAD_SINGLE, HTTPD_SECURE_NONE) != 0) {
			printf("ERROR: httpd_start");
			httpd_clear_page_callbacks();
			return WLAN_SET_OK;
		}
	}
	return WLAN_SET_OK;
}

int wifi_disable_ap_mode(void)
{
	printf("AI glass wifi_disable_ap_mode\r\n");
	httpd_stop();
	while (httpd_is_running()) {
		vTaskDelay(1);
	}
	printf("http service disable\r\n");
	if (!wifi_off()) {
		return WLAN_SET_OK;
	}
	return WLAN_SET_FAIL;
}

int wifi_get_connect_status(void)
{
	if (httpd_is_running()) {
		if (httpd_get_active_connection_num() == 0) {
			return WLAN_STAT_HTTP_IDLE;
		} else {
			return WLAN_STAT_HTTP_CONNECTED;
		}
	} else {
		return WLAN_STAT_IDLE;
	}
}