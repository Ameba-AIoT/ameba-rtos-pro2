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

#define HTTP_OTA_TEST               1

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

#define WRITE_TASK_COMPLETED_BIT    (0x01)
#define WRITE_TASK_SUCCESS_BIT      (0x02)
#define READ_TASK_COMPLETED_BIT     (0x04)
#define READ_TASK_SUCCESS_BIT       (0x08)
#define TASK_NOTIFY_COMPLETED_MASK  (WRITE_TASK_COMPLETED_BIT | READ_TASK_COMPLETED_BIT)

// For Queue method
#define QUEUE_LENGTH            3
#define QUEUE_ITEM_SIZE         HTTP_DATA_BUF_SIZE

typedef struct {
	int id;
	uint8_t fileread;
	uint8_t message[QUEUE_ITEM_SIZE];
} file_msg_t;

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

#if defined(HTTP_OTA_TEST) && HTTP_OTA_TEST
#include <ota_8735b.h>
#include "httpc/httpc.h"
#define OTA_STATE_IDLE                      0
#define OTA_STATE_ERROR                     1
#define OTA_STATE_RECV_START_SIGNAL         2
#define OTA_STATE_DOWNLOAD_FW_IN_PROGRESS   3
#define OTA_STATE_DOWNLOAD_FW_COMPLETED     4
#define OTA_STATE_REBOOT                    5

#define OTA_STATUS_BUFFER_SIZE 64

static int http_ota_status = OTA_STATE_IDLE;
static int convert_ota_status_to_string(int ota_status, char *ota_status_str)
{
	int ret = 0;
	if (ota_status_str) {
		switch (ota_status) {
		case OTA_STATE_IDLE:
			strcpy(ota_status_str, "OTA_STATE_IDLE");
			break;
		case OTA_STATE_ERROR:
			strcpy(ota_status_str, "OTA_STATE_ERROR");
			break;
		case OTA_STATE_RECV_START_SIGNAL:
			strcpy(ota_status_str, "OTA_STATE_RECV_START_SIGNAL");
			break;
		case OTA_STATE_DOWNLOAD_FW_IN_PROGRESS:
			strcpy(ota_status_str, "OTA_STATE_DOWNLOAD_FW_IN_PROGRESS");
			break;
		case OTA_STATE_DOWNLOAD_FW_COMPLETED:
			strcpy(ota_status_str, "OTA_STATE_DOWNLOAD_FW_COMPLETED");
			break;
		case OTA_STATE_REBOOT:
			strcpy(ota_status_str, "OTA_STATE_REBOOT");
			break;
		default:
			ret = -1;
			break;
		}
	} else {
		ret = -1;
	}
	return ret;
}

static int server_port = 3000;
static const char *server_host = "192.168.43.2";
static void ota_httpc_send_thread(void *param)
{
	/* To avoid gcc warnings */
	(void) param;

#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1) && defined(CONFIG_SSL_CLIENT_PRIVATE_IN_TZ) && (CONFIG_SSL_CLIENT_PRIVATE_IN_TZ == 1)
	extern void rtw_create_secure_context(u32 secure_stack_size);
	rtw_create_secure_context(STACKSIZE * 2);
	extern int NS_ENTRY secure_mbedtls_platform_set_calloc_free(void);
	secure_mbedtls_platform_set_calloc_free();
	extern void NS_ENTRY secure_set_ns_device_lock(void (*device_mutex_lock_func)(uint32_t), void (*device_mutex_unlock_func)(uint32_t));
	secure_set_ns_device_lock(device_mutex_lock, device_mutex_unlock);
#endif

	struct httpc_conn *conn = NULL;

	// Delay to wait for IP by DHCP
	WLAN_SCEN_MSG("HTTP Client OTA TESTING THREADS\r\n");

#if USE_HTTPS
	conn = httpc_conn_new(HTTPC_SECURE_TLS, NULL, NULL, NULL);
#else
	conn = httpc_conn_new(HTTPC_SECURE_NONE, NULL, NULL, NULL);
#endif
	char *iot_json = NULL;
	if (conn) {
		while (httpd_is_running()) {
			vTaskDelay(1000);
#if USE_HTTPS
			if (httpc_conn_connect(conn, (char *)server_host, server_port, 2000) != 0)
#else
			if (httpc_conn_connect(conn, (char *)server_host, server_port, 2000) != 0)
#endif
			{
				WLAN_SCEN_ERR("ERROR: httpc_conn_connect\r\n");
				continue;
			} else {
				WLAN_SCEN_MSG("Connect Successfully\r\n");
			}

			if (http_ota_status != OTA_STATE_IDLE) {
				WLAN_SCEN_ERR("HTTP OTA status is not in the idle status\r\n");
				httpc_conn_close(conn);
				continue;
			} else {
				cJSON *IOTJSObject = cJSON_CreateObject();
				cJSON_AddItemToObject(IOTJSObject, "OTA_state", cJSON_CreateString("OTA_STATE_IDLE"));
				iot_json = cJSON_Print(IOTJSObject);
				cJSON_Delete(IOTJSObject);
				// HTTP GET request
				// start a header and add Host (added automatically), Content-Type and Content-Length (added by input param)
				httpc_request_write_header_start(conn, (char *)"POST", (char *)"/api/connectedclients", (char *)"application/json", strlen(iot_json));
				// add other required header fields if necessary
				httpc_request_write_header(conn, (char *)"Connection", (char *)"keep-alive");
				// finish and send header
				httpc_request_write_header_finish(conn);
				httpc_request_write_data(conn, (uint8_t *)iot_json, strlen(iot_json));

				// receive response header
				if (httpc_response_read_header(conn) == 0) {
					httpc_conn_dump_header(conn);
					// receive response body
					if (httpc_response_is_status(conn, (char *)"200 OK")) {
						uint8_t buf[1024];
						int read_size = 0;
						uint32_t total_size = 0;

						while (1) {
							memset(buf, 0, sizeof(buf));
							read_size = httpc_response_read_data(conn, buf, sizeof(buf) - 1);

							if (read_size > 0) {
								total_size += read_size;
								WLAN_SCEN_WARN("%s", buf);
							} else {
								break;
							}

							if (conn->response.content_len && (total_size >= conn->response.content_len)) {
								break;
							}
						}
					}
				}
				httpc_conn_close(conn);
			}
		}
	}

	httpc_conn_free(conn);
	free(iot_json);
	vTaskDelete(NULL);
}

static void ota_httpc_process_thread(void *param)
{
	int ret = -1;

	ret = http_update_ota((char *)server_host, server_port, (char *)"api/uploadfile");

	http_ota_status = OTA_STATE_DOWNLOAD_FW_COMPLETED;

	WLAN_SCEN_MSG("[%s] Update task exit\r\n", __FUNCTION__);
	if (!ret) {
		WLAN_SCEN_MSG("[%s] Ready to reboot\r\n", __FUNCTION__);
		http_ota_status = OTA_STATE_REBOOT;
	} else {
		WLAN_SCEN_MSG("[%s] OTA Failed ret = %d, but reboot\r\n", __FUNCTION__, ret);
		http_ota_status = OTA_STATE_ERROR;
	}
	ota_platform_reset();
	vTaskDelete(NULL);
}

#define SERVER_READ_BUF_SIZE    100
#define SERVER_READ_SLICE_SIZE  2
static void ota_start_cb(struct httpd_conn *conn)
{
	char *user_agent = NULL;

	// test log to show brief header parsing
	httpd_conn_dump_header(conn);
	WLAN_SCEN_MSG("ota_start_cb \r\n");
	// test log to show extra User-Agent header field
	if (httpd_request_get_header_field(conn, (char *)"User-Agent", &user_agent) != -1) {
		WLAN_SCEN_MSG("\nUser-Agent=[%s]\n", user_agent);
		httpd_free(user_agent);
	}

	uint8_t ota_read_buf[SERVER_READ_BUF_SIZE] = {0};
	if (httpd_request_is_method(conn, (char *)"POST")) {
		int slice_len = 0;
		int read_len = 0;
		WLAN_SCEN_MSG("Content Length.%d\r\n", conn->request.content_len);
		if (http_ota_status == OTA_STATE_IDLE) {
			http_ota_status = OTA_STATE_RECV_START_SIGNAL;
		} else {
			httpd_response_method_not_allowed(conn, NULL);
			goto endofota;
		}

		while (read_len < conn->request.content_len) {
			slice_len = (conn->request.content_len - read_len) > SERVER_READ_SLICE_SIZE ? conn->request.content_len - read_len : SERVER_READ_SLICE_SIZE;
			int true_len = httpd_request_read_data(conn, ota_read_buf + read_len, slice_len);
			WLAN_SCEN_MSG("Content Length.%d/%d\r\n", true_len, conn->request.content_len);
			read_len += slice_len;
			if (read_len + SERVER_READ_SLICE_SIZE >= SERVER_READ_BUF_SIZE) {
				break;
			}
		}

		httpd_response_write_header_start(conn, (char *)"200 OK", (char *)"text/plain", 0);
		httpd_response_write_header(conn, (char *)"Access-Control-Allow-Origin", (char *)"*");
		//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Methods", (char *)"GET, POST, OPTIONS");
		//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Headers", (char *)"Content-Type");
		//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Credentials", (char *)"true");
		httpd_response_write_header(conn, (char *)"Connection", (char *)"close");
		httpd_response_write_header_finish(conn);

		WLAN_SCEN_MSG("[OTA] Received start OTA signal from UI.%s\r\n", ota_read_buf);

		if (strstr((const char *)ota_read_buf, "start_ota")) {
			WLAN_SCEN_MSG("[OTA] Received start OTA signal from UI.\r\n");
			if (http_ota_status == OTA_STATE_RECV_START_SIGNAL) {
				WLAN_SCEN_MSG("[OTA] Change Status.\r\n");
				if (xTaskCreate(ota_httpc_process_thread, (const char *)"ota_httpc_process_thread", 1024, NULL, tskIDLE_PRIORITY + 7, NULL) != pdPASS) {
					http_ota_status = OTA_STATE_IDLE;
					WLAN_SCEN_ERR("\n\r[%s] Create update task failed", __FUNCTION__);
				}
			}
		} else {
			http_ota_status = OTA_STATE_IDLE;
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
endofota:
	httpd_conn_close(conn);
}
#endif

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

static void transfer_file_normal_internal(struct httpd_conn *conn, FILE *http_file)
{
	while (1) {
		int br = extdisk_fread(data_buf, 1, HTTP_DATA_BUF_SIZE, http_file);

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

	return;
}

static void http_file_send_thread(void *pvParameters)
{
	FILE *http_file = (FILE *)pvParameters;
	file_msg_t msg = {0};
	uint32_t notifyValue;
	int send_success = 1;

	extdisk_fseek(http_file, 0, SEEK_SET);
	while (1) {
		// Check for stop signal from writer task
		if (xTaskNotifyWait(0, 0, &notifyValue, 0) == pdPASS) {
			if (notifyValue == TASK_NOTIFY_ERROR || notifyValue == TASK_NOTIFY_WERROR) {
				WLAN_SCEN_ERR("[Reader Task] Stopping read due to writer error.\r\n");
				send_success = 0;
				break;
			}
		}

		// Read data from the file
		if (msg.fileread == 0) {
			int br = extdisk_fread(msg.message, 1, QUEUE_ITEM_SIZE, http_file);
			if (br < 0) {
				WLAN_SCEN_ERR("[Reader Task] Read ERROR\r\n");
				WLAN_SCEN_ERR("[Reader Task] Send notify READ_STATUS_ERROR to Writer Task Handle for error\r\n");
				if (send_taskhandle) {
					xTaskNotify(send_taskhandle, TASK_NOTIFY_ERROR, eSetValueWithOverwrite); // Notify error
				}
				send_success = 0;
				break;
			}

			if (br == 0) { // EOF detected
				WLAN_SCEN_MSG("[Reader Task] EOF Detected\r\n");
				WLAN_SCEN_MSG("[Reader Task] Send notify 0 to Writer Task Handle for EOF\r\n");
				msg.id = 0;
				if (file_queue) {
					xQueueSend(file_queue, &msg, portMAX_DELAY);
				}
				break;
			}

			// Fill in the message metadata
			msg.fileread = 1;
			msg.id = br; // Use the number of bytes read as the ID
		}

		if (file_queue) {
			if (xQueueSend(file_queue, &msg, 20) != pdPASS) {
				WLAN_SCEN_INFO("[Reader Task] enqueue message full\r\n");
			} else {
				msg.fileread = 0;
			}
		} else {
			WLAN_SCEN_ERR("[Reader Task] Queue is not valid.\r\n");
			send_success = 0;
			break;
		}
	}

	// Notify completion to core taskhandle
	xTaskNotify(core_taskhandle, READ_TASK_COMPLETED_BIT | (send_success ? READ_TASK_SUCCESS_BIT : 0), eSetBits);
	vTaskDelete(NULL);
}

static void http_file_read_thread(void *pvParameters)
{
	struct httpd_conn *conn = (struct httpd_conn *)pvParameters;
	file_msg_t rcv_msg;
	int writer_status = 0;
	int total_bw = 0;
	int rcv_success = 1;

	while (1) {
		if (xQueueReceive(file_queue, &rcv_msg, portMAX_DELAY) == pdPASS) {
			if (rcv_msg.id == 0) {
				writer_status = WRITE_STATUS_EOF;
				WLAN_SCEN_MSG("Get total time = %d\r\n", total_bw);
				break;
			}
			total_bw += rcv_msg.id;
			int send_timeout = 3000;
			if (conn->sock != -1) {
				setsockopt(conn->sock, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));
			}
			int ret = httpd_response_write_data(conn, rcv_msg.message, rcv_msg.id);
			if (ret <= 0) {
				writer_status = WRITE_STATUS_ERROR;
				WLAN_SCEN_ERR("[WRITER TASK] httpd_response_write_data ret = %d\r\n", ret);
				xTaskNotify(read_taskhandle, TASK_NOTIFY_WERROR, eSetValueWithOverwrite);
				rcv_success = 0;
				break;
			}
		} else {
			WLAN_SCEN_ERR("[WRITER TASK] xQueueReceive fail\r\n");
			xTaskNotify(read_taskhandle, TASK_NOTIFY_ERROR, eSetValueWithOverwrite);
			rcv_success = 0;
			break;
		}
	}

	if (writer_status == WRITE_STATUS_EOF) {
		WLAN_SCEN_MSG("[WRITER TASK] EOF received from reader.\r\n");
	} else if (writer_status == WRITE_STATUS_ERROR) {
		WLAN_SCEN_ERR("[WRITER TASK] httpd response write data error.\r\n");
	}

	xTaskNotify(core_taskhandle, WRITE_TASK_COMPLETED_BIT | (rcv_success ? WRITE_TASK_SUCCESS_BIT : 0), eSetBits);
	vTaskDelete(NULL);
}

static void media_getfile_cb(struct httpd_conn *conn)
{
	char *filename = NULL;
	char *user_agent = NULL;
	FILE *http_file = NULL;

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
			http_file = extdisk_fopen(filename, "r");
			if (http_file == NULL) {
				httpd_response_bad_request(conn, (char *)"Bad Request: No such file");
				goto http_end;
			}
			//extdisk_fseek(http_file, 0, SEEK_SET);

			// Write HTTP headers
			httpd_response_write_header_start(conn, (char *)"200 OK", (char *)"text/plain", 0);
			httpd_response_write_header(conn, (char *)"Access-Control-Allow-Origin", (char *)"*");
			//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Methods", (char *)"GET, POST, OPTIONS");
			//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Headers", (char *)"Content-Type");
			//httpd_response_write_header(conn, (char *)"Access-Control-Allow-Credentials", (char *)"true");
			httpd_response_write_header(conn, (char *)"Connection", (char *)"close");
			httpd_response_write_header_finish(conn);

			file_queue = xQueueCreate(QUEUE_LENGTH, sizeof(file_msg_t));
			if (file_queue == NULL) {
				WLAN_SCEN_WARN("Failed to create queue.\r\n");
				transfer_file_normal_internal(conn, http_file);
				goto http_end;
			}

			core_taskhandle = xTaskGetCurrentTaskHandle();

			if (xTaskCreate(http_file_send_thread, "Sender", 8192, (void *)http_file, 5, &read_taskhandle) != pdPASS) {
				WLAN_SCEN_WARN("Failed to create ReaderTask\n");
				transfer_file_normal_internal(conn, http_file);
				vQueueDelete(file_queue);
				file_queue = NULL;
				goto http_end;
			}

			if (xTaskCreate(http_file_read_thread, "Receiver", 8192, (void *)conn, 5, &send_taskhandle) != pdPASS) {
				WLAN_SCEN_WARN("Failed to create WriterTask\n");
				vTaskDelete(read_taskhandle);
				vQueueDelete(file_queue);
				file_queue = NULL;
				transfer_file_normal_internal(conn, http_file);
				goto http_end;
			}

			// Wait for both tasks to signal completion
			uint32_t notifyValue = 0;
			do {
				notifyValue |= ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
				WLAN_SCEN_MSG("wait notify notifyValue = %lx\r\n", notifyValue);
			} while ((notifyValue & TASK_NOTIFY_COMPLETED_MASK) != TASK_NOTIFY_COMPLETED_MASK);

			vQueueDelete(file_queue);
			file_queue = NULL;
			send_taskhandle = NULL;
			read_taskhandle = NULL;
			extdisk_fclose(http_file);
			http_file = NULL;
			if ((notifyValue & (WRITE_TASK_SUCCESS_BIT | READ_TASK_SUCCESS_BIT)) == (WRITE_TASK_SUCCESS_BIT | READ_TASK_SUCCESS_BIT)) {
				WLAN_SCEN_MSG("Http send %s completed successfully\r\n", filename);
				if (delete_file_after_upload) {
					extdisk_remove(filename);
					extdisk_save_file_cntlist();
					WLAN_SCEN_MSG("Delete file %s\r\n", filename);
				} else {
					WLAN_SCEN_MSG("Keep file %s\r\n", filename);
				}
			} else {
				WLAN_SCEN_WARN("Http send %s fail, ret = %lx\r\n", filename, notifyValue);
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
	if (http_file) {
		extdisk_fclose(http_file);
		http_file = NULL;
	}
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
#if defined(HTTP_OTA_TEST) && HTTP_OTA_TEST
		httpd_reg_page_callback((char *)"/ota-start", ota_start_cb);
#endif
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
#if defined(HTTP_OTA_TEST) && HTTP_OTA_TEST
		if (xTaskCreate(ota_httpc_send_thread, (const char *)"ota_httpc_send_thread", 1024, NULL, tskIDLE_PRIORITY + 5, NULL) != pdPASS) {
			WLAN_SCEN_ERR("\n\r[%s] Create update task failed", __FUNCTION__);
			return WLAN_SET_FAIL;
		}
#endif
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