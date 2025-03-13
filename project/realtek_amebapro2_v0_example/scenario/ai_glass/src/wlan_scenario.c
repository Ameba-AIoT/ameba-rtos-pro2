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
#include "ai_glass_dbg.h"

#define USE_HTTPS                   0
#define DELETE_FILE_AFTER_UPLOAD    1
#define HTTP_PORT                   8080 //80
#define HTTPS_PORT                  8080 //443

// CONFIG_USE_POLARSSL in platform_opts.h, default CONFIG_USE_POLARSSL = 0
// These are the certificate, key provide by TLS
// User could use their cert since the test cert treaed unsafety for some user
#if defined(USE_HTTPS) && USE_HTTPS
#if (HTTPD_USE_TLS == HTTPD_TLS_POLARSSL)
#include <polarssl/certs.h>
#define HTTPS_SRC_CRT   test_srv_crt
#define HTTPS_SRC_KEY   test_srv_key
#define HTTPS_CA_CRT    test_ca_crt
#elif (HTTPD_USE_TLS == HTTPD_TLS_MBEDTLS)
#include <mbedtls/certs.h>
#define HTTPS_SRC_CRT   mbedtls_test_srv_crt
#define HTTPS_SRC_KEY   mbedtls_test_srv_key
#define HTTPS_CA_CRT    mbedtls_test_ca_crt
#endif
#endif

#define HTTP_DATA_BUF_SIZE      4096

static uint8_t data_buf[HTTP_DATA_BUF_SIZE] = {0};
static uint8_t delete_file_after_upload = DELETE_FILE_AFTER_UPLOAD;

#define READ_STATUS_IDLE        0
#define READ_STATUS_ERROR       1
#define READ_STATUS_TIMEOUT     2
#define READ_STATUS_EOF         3
#define READ_STATUS_WERROR      4

#define TASK_NOTIFY_END         0
#define TASK_NOTIFY_VALID       1
#define TASK_NOTIFY_ERROR       -1
#define TASK_NOTIFY_WERROR      -2

#define WRITE_STATUS_ERROR      1
#define WRITE_STATUS_EOF        3
#define WRITE_STATUS_RERROR     4

// For Queue method
#define QUEUE_LENGTH            3
#define QUEUE_ITEM_SIZE         HTTP_DATA_BUF_SIZE

typedef struct {
	int id;
	uint8_t message[QUEUE_ITEM_SIZE];
} Message_t;

static QueueHandle_t file_queue = NULL;
static TaskHandle_t core_taskhandle = NULL;
static TaskHandle_t read_taskhandle = NULL;
static TaskHandle_t send_taskhandle = NULL;

volatile int flag = 0;

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
		WLAN_SCEN_MSG("\nUser-Agent=[%s]\n", user_agent);
		httpd_free(user_agent);
	}

	// GET homepage
	if (httpd_request_is_method(conn, (char *)"GET")) {
		httpd_response_write_header_start(conn, (char *)"200 OK", (char *)"text/plain", 0);
		httpd_response_write_header(conn, (char *)"Access-Control-Allow-Origin", (char *)"*");
		//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Methods", (char *)"GET, POST, OPTIONS");
		//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Headers", (char *)"Content-Type");
		//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Credentials", (char *)"true");
		httpd_response_write_header(conn, (char *)"Connection", (char *)"close");
		httpd_response_write_header_finish(conn);
	} else if (httpd_request_is_method(conn, (char *)"OPTIONS")) {
		// Handle pre-flight OPTIONS request for CORS
		httpd_response_write_header_start(conn, (char *)"204 No Content", NULL, 0);

		// Add CORS headers for preflight request
		httpd_response_write_header(conn, (char *)"Access-Control-Allow-Origin", (char *)"*");
		//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Methods", (char *)"GET, POST, OPTIONS");
		//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Headers", (char *)"Content-Type");
		//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Credentials", (char *)"true");
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
		WLAN_SCEN_MSG("\nUser-Agent=[%s]\n", user_agent);
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
			WLAN_SCEN_MSG("%s\r\n", file_list);
		} else {
			WLAN_SCEN_MSG("file list is NULL\n");
		}
		// Save filelist to EMMC
		extdisk_save_file_cntlist();
		uint32_t file_list_len = strlen(file_list);
		httpd_response_write_header_start(conn, (char *)"200 OK", (char *)"text/plain", file_list_len);
		httpd_response_write_header(conn, (char *)"Access-Control-Allow-Origin", (char *)"*");
		//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Methods", (char *)"GET, POST, OPTIONS");
		//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Headers", (char *)"Content-Type");
		//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Credentials", (char *)"true");
		httpd_response_write_header(conn, (char *)"Connection", (char *)"close");
		httpd_response_write_header_finish(conn);
		httpd_response_write_data(conn, (uint8_t *)file_list, file_list_len);
	} else if (httpd_request_is_method(conn, (char *)"OPTIONS")) {
		// Handle pre-flight OPTIONS request for CORS
		httpd_response_write_header_start(conn, (char *)"204 No Content", NULL, 0);

		// Add CORS headers for preflight request
		httpd_response_write_header(conn, (char *)"Access-Control-Allow-Origin", (char *)"*");
		//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Methods", (char *)"GET, POST, OPTIONS");
		//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Headers", (char *)"Content-Type");
		//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Credentials", (char *)"true");
		httpd_response_write_header_finish(conn);
	} else {
		// HTTP/1.1 405 Method Not Allowed
		httpd_response_method_not_allowed(conn, NULL);
	}

	httpd_conn_close(conn);
}

//Initial value
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
						WLAN_SCEN_ERR("ERROR: malloc fail");
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
		WLAN_SCEN_ERR("no value found");
	}

	return ret;
}

static void transfer_file_normal_internal(struct httpd_conn *conn, char *filename)
{
	FILE *http_file = extdisk_fopen(filename, "r");
	if (http_file == NULL) {
		return;
	}
	int br = 0;
	extdisk_fseek(http_file, 0, SEEK_SET);
	while (1) {
		br = extdisk_fread(data_buf, 1, HTTP_DATA_BUF_SIZE, http_file);

		if (br < 0) {
			WLAN_SCEN_ERR("Read ERROR, error num %d\r\n", br);
			break;
		} else {
			int ret = 0;
			int send_timeout = 3000;
			if (conn->sock != -1) {
				setsockopt(conn->sock, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));
			}

			ret = httpd_response_write_data(conn, data_buf, br);

			if (ret <= 0) {
				WLAN_SCEN_ERR("http error ret = %d\r\n", ret);
				break;
			}
			if (br != HTTP_DATA_BUF_SIZE) {
				break;
			}
		}
	}

	if (http_file) {
		extdisk_fclose(http_file);
		http_file = NULL;
	}

	return;
}

static void http_file_send_thread(void *pvParameters)
{
	char *filename = (char *)pvParameters;
	FILE *http_file = extdisk_fopen(filename, "r");
	if (http_file == NULL) {
		WLAN_SCEN_ERR("[Reader Task] Read ERROR\n");
		WLAN_SCEN_ERR("[Reader Task] Send notify READ_STATUS_ERROR to Writer Task Handle for error\r\n");
		xTaskNotify(send_taskhandle, TASK_NOTIFY_ERROR, eSetValueWithOverwrite); // Notify error
	}
	int reader_status = READ_STATUS_ERROR;
	Message_t message;

	extdisk_fseek(http_file, 0, SEEK_SET);
	while (1) {
		// Read data from the file
		int br = extdisk_fread(message.message, 1, QUEUE_ITEM_SIZE, http_file);
		if (br < 0) {
			reader_status = READ_STATUS_ERROR;
			break;
		}

		if (br == 0) { // EOF detected
			reader_status = READ_STATUS_EOF;
			break;
		}

		// Fill in the message metadata
		message.id = br; // Use the number of bytes read as the ID

		// Send the message to the queue
		if (xQueueSend(file_queue, &message, portMAX_DELAY) != pdPASS) {
			WLAN_SCEN_ERR("Failed to enqueue message.\r\n");
			break;
		}
	}

	if (reader_status == READ_STATUS_ERROR) {
		WLAN_SCEN_ERR("[Reader Task] Read ERROR\n");
		WLAN_SCEN_ERR("[Reader Task] Send notify READ_STATUS_ERROR to Writer Task Handle for error\r\n");
		xTaskNotify(send_taskhandle, TASK_NOTIFY_ERROR, eSetValueWithOverwrite); // Notify error
	} else if (reader_status == READ_STATUS_EOF) {
		WLAN_SCEN_MSG("[Reader Task] EOF Detected\r\n");
		WLAN_SCEN_MSG("[Reader Task] Send notify 0 to Writer Task Handle for EOF\r\n");
		// Signal end of sending by sending a zero-length message
		message.id = 0;
		xQueueSend(file_queue, &message, portMAX_DELAY);
	}
	if (http_file) {
		extdisk_fclose(http_file);
		http_file = NULL;
	}

	vTaskDelete(NULL);
}

// Receiver Task
static void http_file_read_thread(void *pvParameters)
{
	struct httpd_conn *conn = (struct httpd_conn *)pvParameters;
	Message_t receivedMessage;
	int writer_status = 0;
	int total_bw = 0;

	while (1) {
		// Receive data from the queue
		if (xQueueReceive(file_queue, &receivedMessage, portMAX_DELAY) == pdPASS) {
			if (receivedMessage.id == 0) {
				writer_status = WRITE_STATUS_EOF;
				WLAN_SCEN_MSG("Get total time = %d\r\n", total_bw);
				break;
			}
			// Count the total read file size
			total_bw += receivedMessage.id;
			int send_timeout = 3000;
			if (conn->sock != -1) {
				setsockopt(conn->sock, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));
			}
			// Send data over the connection
			int ret = httpd_response_write_data(conn, receivedMessage.message, receivedMessage.id);
			if (ret <= 0) {
				writer_status = WRITE_STATUS_ERROR;
				break;
			}
		}
	}

	if (writer_status == WRITE_STATUS_EOF) {
		WLAN_SCEN_MSG("[WRITER TASK] EOF received from reader.\r\n");
		xTaskNotify(core_taskhandle, TASK_NOTIFY_END, eSetValueWithOverwrite);  // Notify main callback of completion
	} else if (writer_status == WRITE_STATUS_RERROR) {
		WLAN_SCEN_ERR("[WRITER TASK] Reader reported an error.\r\n");
		xTaskNotify(core_taskhandle, TASK_NOTIFY_ERROR, eSetValueWithOverwrite);
	} else if (writer_status == WRITE_STATUS_ERROR) {
		WLAN_SCEN_ERR("[WRITER TASK] httpd response write data error.\r\n");
		xTaskNotify(core_taskhandle, TASK_NOTIFY_WERROR, eSetValueWithOverwrite);
	}

	vTaskDelete(NULL);
}


static void media_getfile_cb(struct httpd_conn *conn)
{
	char *filename = NULL;
	char *user_agent = NULL;

	// test log to show brief header parsing
	httpd_conn_dump_header(conn);

	// test log to show extra User-Agent header field
	uint32_t start_time_httpd_request_get_header_field = rtw_get_current_time();
	if (httpd_request_get_header_field(conn, (char *)"User-Agent", &user_agent) != -1) {
		WLAN_SCEN_MSG("\nUser-Agent=[%s]\n", user_agent);
		httpd_free(user_agent);
	}

	// GET homepage
	if (httpd_request_is_method(conn, (char *)"GET")) {
		if (httpd_request_get_path_key(conn, (char *)"media/", &filename) != -1) {
			//http_file = extdisk_fopen(filename, "r");

			// Write HTTP headers
			httpd_response_write_header_start(conn, (char *)"200 OK", (char *)"text/plain", 0);
			httpd_response_write_header(conn, (char *)"Access-Control-Allow-Origin", (char *)"*");
			//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Methods", (char *)"GET, POST, OPTIONS");
			//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Headers", (char *)"Content-Type");
			//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Credentials", (char *)"true");
			httpd_response_write_header(conn, (char *)"Connection", (char *)"close");
			httpd_response_write_header_finish(conn);

			file_queue = xQueueCreate(QUEUE_LENGTH, sizeof(Message_t));
			if (file_queue == NULL) {
				WLAN_SCEN_WARN("Failed to create queue.\r\n");
				transfer_file_normal_internal(conn, filename);
				goto http_end;
			}

			core_taskhandle = xTaskGetCurrentTaskHandle();

			if (xTaskCreate(http_file_send_thread, "Sender", 8192, (void *)filename, 5, &read_taskhandle) != pdPASS) {
				WLAN_SCEN_WARN("Failed to create ReaderTask\n");
				transfer_file_normal_internal(conn, filename);
				vQueueDelete(file_queue);
				file_queue = NULL;
				goto http_end;
			}

			if (xTaskCreate(http_file_read_thread, "Receiver", 8192, (void *)conn, 5, &send_taskhandle) != pdPASS) {
				WLAN_SCEN_WARN("Failed to create WriterTask\n");
				vTaskDelete(read_taskhandle);
				vQueueDelete(file_queue);
				file_queue = NULL;
				transfer_file_normal_internal(conn, filename);
				goto http_end;
			}

			int status = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			if (status == TASK_NOTIFY_END) {
				WLAN_SCEN_MSG("Write %s completed successfully\r\n", filename);
			} else if (status == TASK_NOTIFY_ERROR) {
				WLAN_SCEN_ERR("Write %s failed.\r\n", filename);
			} else if (status == TASK_NOTIFY_WERROR) {
				WLAN_SCEN_ERR("Write %s failed.\r\n [WRITE_TASK]", filename);
				vTaskDelete(read_taskhandle);
			}
			vQueueDelete(file_queue);
			file_queue = NULL;
			send_taskhandle = NULL;
			read_taskhandle = NULL;
			//extdisk_fclose(http_file);
			//http_file = NULL;
			if (delete_file_after_upload) {
				extdisk_remove(filename);
				// Save filelist to EMMC
				extdisk_save_file_cntlist();
			}
		} else {
			// HTTP/1.1 400 Bad Request
			httpd_response_bad_request(conn, (char *)"Bad Request");
		}
	} else if (httpd_request_is_method(conn, (char *)"OPTIONS")) {
		// Handle pre-flight OPTIONS request for CORS
		httpd_response_write_header_start(conn, (char *)"204 No Content", NULL, 0);

		// Add CORS headers for preflight request
		httpd_response_write_header(conn, (char *)"Access-Control-Allow-Origin", (char *)"*");
		//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Methods", (char *)"GET, POST, OPTIONS");
		//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Headers", (char *)"Content-Type");
		//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Credentials", (char *)"true");
		httpd_response_write_header_finish(conn);
	} else {
		// HTTP/1.1 405 Method Not Allowed
		httpd_response_method_not_allowed(conn, NULL);
	}

http_end:
	if (filename) {
		httpd_free(filename);
		filename = NULL;
	}
	//if (http_file) {
	//extdisk_fclose(http_file);
	//http_file = NULL;
	//}
	httpd_conn_close(conn);
	WLAN_SCEN_MSG("[%s] httpd_conn_end (close files): %lu ms\r\n", __func__, rtw_get_current_time() - start_time_httpd_request_get_header_field);
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
	WLAN_SCEN_MSG("AI glass wifi_enable_ap_mode\r\n");
#if CONFIG_INIT_NET
#if CONFIG_LWIP_LAYER
	// Initilaize the LwIP stack, if the LwIP is not initalized yet
	extern int lwip_init_done;
	if (!lwip_init_done) {
		LwIP_Init();
	}
#endif
#endif

	WLAN_SCEN_MSG("AI glass Enable Wi-Fi with AP mode\r\n");
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
				WLAN_SCEN_ERR("AI glass ERROR: wifi change mode failed\r\n");
				return WLAN_SET_FAIL;
			}
		} else {
			if (wifi_on(RTW_MODE_AP) < 0) {
				WLAN_SCEN_ERR("AI glass ERROR: wifi_on failed\r\n");
				return WLAN_SET_FAIL;
			}
		}
	}

	// Start AP
	WLAN_SCEN_MSG("AI glass Start AP\r\n");
	softAP_config.ssid.len = strlen(ssid);
	memcpy(softAP_config.ssid.val, (char *)ssid, softAP_config.ssid.len);

	memset(wifi_pass_word, 0x00, MAX_AP_PASSWORD_LEN);
	softAP_config.password_len = (strlen(password) > (MAX_AP_PASSWORD_LEN - 1) ? (MAX_AP_PASSWORD_LEN - 1) : strlen(password));
	memcpy(wifi_pass_word, password, softAP_config.password_len);
	softAP_config.password = (unsigned char *)wifi_pass_word;

	softAP_config.channel = channel;
	softAP_config.security_type = RTW_SECURITY_WPA2_AES_PSK;
	if (wifi_start_ap(&softAP_config) < 0) {
		WLAN_SCEN_ERR("AI glass ERROR: wifi_start_ap failed\r\n");
		return WLAN_SET_FAIL;
	}

	// Check AP running
	WLAN_SCEN_MSG("AI glass Check AP running\r\n");
	while (1) {
		rtw_wifi_setting_t setting;
		wifi_get_setting(WLAN0_IDX, &setting);
		if (strlen((char *)setting.ssid) > 0) {
			if (strcmp((const char *)setting.ssid, (const char *)ssid) == 0) {
				WLAN_SCEN_MSG("AI glass %s started\r\n", ssid);
				break;
			}
		}
		if (timeout == 0) {
			WLAN_SCEN_ERR("AI glass ERROR: Start AP timeout\r\n");
			return WLAN_SET_FAIL;
		}
		vTaskDelay(1 * configTICK_RATE_HZ);
		timeout --;
	}

	// Start DHCP server
	WLAN_SCEN_MSG("AI glass Start DHCP server\r\n");

#if CONFIG_LWIP_LAYER
	dhcps_init(&xnetif[0]);
#endif

set_http:
	if (!httpd_is_running()) {
		httpd_reg_page_callback((char *)"/media/*", media_getfile_cb);
		httpd_reg_page_callback((char *)"/pingpong", pingpong_cb);
		httpd_reg_page_callback((char *)"/media-list", media_list_cb);
		httpd_setup_priority(5);
		httpd_setup_idle_timeout(HTTPD_CONNECT_TIMEOUT);
#if defined(USE_HTTPS) && USE_HTTPS
		// Set up http certificate
		if (httpd_setup_cert(HTTPS_SRC_CRT, HTTPS_SRC_KEY, HTTPS_CA_CRT) != 0) {
			WLAN_SCEN_ERR("\nERROR: httpd_setup_cert\n");
			return WLAN_SET_FAIL;
		}
#endif
#if defined(USE_HTTPS) && USE_HTTPS
		if (httpd_start(HTTPS_PORT, 5, 4096, HTTPD_THREAD_SINGLE, HTTPD_SECURE_TLS) != 0) {
#else
		if (httpd_start(HTTP_PORT, 5, 4096, HTTPD_THREAD_SINGLE, HTTPD_SECURE_NONE) != 0) {
#endif
			WLAN_SCEN_ERR("ERROR: httpd_start");
			httpd_clear_page_callbacks();
			return WLAN_SET_FAIL;
		}
	}
	return WLAN_SET_OK;
}

int wifi_disable_ap_mode(void)
{
	WLAN_SCEN_MSG("AI glass wifi_disable_ap_mode\r\n");
	httpd_stop();
	while (httpd_is_running()) {
		vTaskDelay(1);
	}
	WLAN_SCEN_MSG("http service disable\r\n");
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

void wifi_set_up_file_delete_flag(uint8_t flag)
{
	if (flag) {
		WLAN_SCEN_MSG("File will be deleted after upload successfully\r\n");
		delete_file_after_upload = 1;
	} else {
		WLAN_SCEN_MSG("File will be remained in EMMC after upload successfully\r\n");
		delete_file_after_upload = 0;
	}
}