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
#include <string.h>
#include "mmf2_mediatime_8735b.h"
#include "ai_glass_media.h"
#include "ai_glass_initialize.h"
#include "ftl_common_api.h"

#define USE_HTTPS                   1
#define DELETE_FILE_AFTER_UPLOAD    0
#define HTTP_PORT                   8080 //80
#define HTTPS_PORT                  8080 //443

#define HTTP_OTA_TEST               1
#define MULTI_THREAD_WIFI_SEND_PIC_DATA 1

// CONFIG_USE_POLARSSL in platform_opts.h, default CONFIG_USE_POLARSSL = 0
// These are the certificate, key provide by TLS
// User could use their cert since the test cert treaed unsafety for some user
#if defined(USE_HTTPS) && USE_HTTPS
// #if (HTTPD_USE_TLS == HTTPD_TLS_POLARSSL)
// #include <polarssl/certs.h>
// #define HTTPS_SRC_CRT   test_srv_crt
// #define HTTPS_SRC_KEY   test_srv_key
// #define HTTPS_CA_CRT    test_ca_crt
// #elif (HTTPD_USE_TLS == HTTPD_TLS_MBEDTLS)
// #include <mbedtls/certs.h>
// #define HTTPS_SRC_CRT   mbedtls_test_srv_crt
// #define HTTPS_SRC_KEY   mbedtls_test_srv_key
// #define HTTPS_CA_CRT    mbedtls_test_ca_crt
// #endif

#include "ai_glass_certs.h"

#define HTTPS_SRC_CRT    ai_glass_src_cert_pem
#define HTTPS_SRC_KEY    ai_glass_src_key_pem
#define HTTPS_CA_CRT     ai_glass_ca_cert_pem

#endif

//FOR OTA
#define SERVER_READ_BUF_SIZE    2000
#define TOTAL_LEN               (SERVER_READ_BUF_SIZE * 2 + 2)
#define TOTOAL_FILE_NUM         2
static uint8_t read_buf[TOTAL_LEN] = {0};
static int terminate_signal = 0;

//FOR HTTP DOWNLOAD
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
#define QUEUE_LENGTH            8
#define QUEUE_ITEM_SIZE         HTTP_DATA_BUF_SIZE

typedef struct {
	int id;
	uint8_t fileread;
	uint8_t message[QUEUE_ITEM_SIZE];
} file_msg_t;

typedef struct {
	struct httpd_conn *conn;
    uint8_t *data;
    uint32_t length;
	TaskHandle_t caller_task_handle;
} heap_send_param_t;

heap_ota_data_t *g_heap_ota_data = NULL;

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

static int multicast_port = 5353;
static const char *multicast_group_ip = "224.0.0.251";
static void ota_multicast_send_thread(void *param)
{
	int err = 0;
	int socket = -1;
	// Set NETIF_FLAG_IGMP flag for netif which should process IGMP messages
	xnetif[0].flags |= NETIF_FLAG_IGMP;
	if ((socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		WLAN_SCEN_ERR("ERROR: socket - AF_INET, SOCK_DGRAM\r\n");
		err = -1;
	}
	// Add multicast group membership on this interface
	if (err == 0) {
		struct ip_mreq imr;
		imr.imr_multiaddr.s_addr = inet_addr(multicast_group_ip);
		imr.imr_interface.s_addr = INADDR_ANY;
		err = setsockopt(socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &imr, sizeof(imr));
		if (err < 0) {
			WLAN_SCEN_ERR("ERROR: setsockopt - IP_ADD_MEMBERSHIP\r\n");
		}
	}
	// Specify outgoing interface too
	if (err == 0) {
		struct in_addr intfAddr;
		intfAddr.s_addr = INADDR_ANY;
		err = setsockopt(socket, IPPROTO_IP, IP_MULTICAST_IF, &intfAddr, sizeof(struct in_addr));
		if (err < 0) {
			WLAN_SCEN_ERR("ERROR: setsockopt - IP_MULTICAST_IF\r\n");
		}
	}
	// And start listening for packets
	if (err == 0) {
		struct sockaddr_in bindAddr;
		bindAddr.sin_family = AF_INET;
		bindAddr.sin_port = htons(multicast_port);
		bindAddr.sin_addr.s_addr = INADDR_ANY;
		err = bind(socket, (struct sockaddr *)&bindAddr, sizeof(bindAddr));
		if (err < 0) {
			WLAN_SCEN_ERR("ERROR: bind\r\n");
		}
	}
	cJSON *IOTJSObject = cJSON_CreateObject();
	cJSON_AddItemToObject(IOTJSObject, "OTA_state", cJSON_CreateString("OTA_STATE_IDLE"));
	char *iot_json = cJSON_Print(IOTJSObject);
	cJSON_Delete(IOTJSObject);
	if (err == 0) {
		while (1) {
			vTaskDelay(5000);
			if (http_ota_status != OTA_STATE_IDLE) {
				WLAN_SCEN_MSG("HTTP OTA status is not in the idle status, stop sending multicast packet\r\n");
			} else {
				if (terminate_signal == 1) {
					WLAN_SCEN_MSG("terminate signal received\r\n");
					break;
				}
				int sendLen;
				struct sockaddr to;
				struct sockaddr_in *to_sin = (struct sockaddr_in *)&to;
				to_sin->sin_family = AF_INET;
				to_sin->sin_port = htons(multicast_port);
				to_sin->sin_addr.s_addr = inet_addr(multicast_group_ip);
				if ((sendLen = sendto(socket, iot_json, strlen(iot_json), 0, &to, sizeof(struct sockaddr))) < 0) {
					WLAN_SCEN_ERR("ERROR: sendto %s\r\n", multicast_group_ip);
				} else {
					WLAN_SCEN_MSG("sendto - %d bytes to %s:%d\r\n", sendLen, multicast_group_ip, multicast_port);
				}
			}
		}
	} else if (socket != -1) {
		WLAN_SCEN_ERR("ERROR: socket = -1\r\n");
		close(socket);
	}
	free(iot_json);
	close(socket);
	vTaskDelete(NULL);
}

static int ota_server_port = 3000;
static char ota_server_host[INET_ADDRSTRLEN] = "192.168.43.2";
static void ota_httpc_process_thread(void *param)
{
	int ret = -1;

	ret = http_update_ota((char *)ota_server_host, ota_server_port, (char *)"api/uploadfile");

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
				struct sockaddr_in client_addr;
				socklen_t addr_len = sizeof(client_addr);
				if (getpeername(conn->sock, (struct sockaddr *)&client_addr, &addr_len) == -1) {
					printf("getpeername failed\r\n");
					goto endofota;
				} else {
					if (inet_ntop(AF_INET, &client_addr.sin_addr, ota_server_host, sizeof(ota_server_host)) != NULL) {
						printf("Client IP address: %s\n", ota_server_host);
					} else {
						printf("Client IP address failed\n");
						goto endofota;
					}
				}
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
		const char *extensions[] = { ".mp4", ".csv", ".jpeg", ".jpg", ".txt"};
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

static void http_file_read_thread(void *pvParameters)
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
				critical_process_started = 0;
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
				critical_process_started = 0;
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
			critical_process_started = 0;
			break;
		}
	}

	// Notify completion to core taskhandle
	xTaskNotify(core_taskhandle, READ_TASK_COMPLETED_BIT | (send_success ? READ_TASK_SUCCESS_BIT : 0), eSetBits);
	vTaskDelete(NULL);
}

static void http_file_send_thread(void *pvParameters)
{
	struct httpd_conn *conn = (struct httpd_conn *)pvParameters;
	file_msg_t rcv_msg;
	int writer_status = 0;
	int total_bw = 0;
	int rcv_success = 1;

	uint32_t start_time = 0;
	uint32_t end_time = 0;
	uint32_t total_bytes_sent = 0;
	int started = 0;

	// int send_timeout = 3000;
	// if (conn->sock != -1) {
	// 	setsockopt(conn->sock, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));
	// }

	while (1) {
		if (xQueueReceive(file_queue, &rcv_msg, portMAX_DELAY) == pdPASS) {
			if (rcv_msg.id == 0) {
				writer_status = WRITE_STATUS_EOF;
				WLAN_SCEN_MSG("Get total queued bytes = %d\r\n", total_bw);
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
			} else if (ret > 0) {
				if (!started) {
					start_time = mm_read_mediatime_ms();
					started = 1;
				}
				total_bytes_sent += ret;
			}
		} else {
			WLAN_SCEN_ERR("[WRITER TASK] xQueueReceive fail\r\n");
			xTaskNotify(read_taskhandle, TASK_NOTIFY_ERROR, eSetValueWithOverwrite);
			rcv_success = 0;
			break;
		}
	}

	// End timing
	if (started) {
		end_time = mm_read_mediatime_ms();
		uint32_t duration_ms = end_time - start_time;

		if (duration_ms > 0) {
			float throughput_kbps = (total_bytes_sent * 8.0f) / duration_ms;
			float throughput_MBps = (total_bytes_sent / 1024.0f / 1024.0f) / (duration_ms / 1000.0f);

			WLAN_SCEN_MSG("\n=== WIFI THROUGHPUT ===\n");
			WLAN_SCEN_MSG("Bytes Sent (actual): %lu\n", total_bytes_sent);
			WLAN_SCEN_MSG("Time: %lu ms\n", duration_ms);
			WLAN_SCEN_MSG("Throughput: %.2f kbps\n", throughput_kbps);
			WLAN_SCEN_MSG("Throughput: %.2f MB/s\n", throughput_MBps);
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

static void http_heap_send_thread(void *pvParameters) {
    heap_send_param_t *param = (heap_send_param_t *)pvParameters;
    
    uint8_t *data = param->data;
    uint32_t length = param->length;
    uint32_t  sent = 0;
    int ret;
	
	
    while (sent < length) {
		uint32_t  chunk_size = length - sent;
		if (chunk_size > HTTP_DATA_BUF_SIZE) {
			chunk_size = HTTP_DATA_BUF_SIZE;  // cap to 4KB per send
			WLAN_SCEN_MSG("Sending chunk size: %lu bytes\n", chunk_size);
		}
		int send_timeout = 3000;

		if (param->conn->sock != -1) {
			setsockopt(param->conn->sock, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));
		}
        ret = httpd_response_write_data(param->conn, data + sent, chunk_size);
        if (ret <= 0) {
            // Error sending data
            WLAN_SCEN_WARN("Failed to send data: %d\n", ret);
            break;
        }
        sent += ret;
    }

    if (sent == length) {
        WLAN_SCEN_MSG("Send complete: %lu bytes sent\n", sent);
    } else {
        WLAN_SCEN_MSG("Send incomplete: %lu/%lu bytes sent\n", sent, length);
    }

    // Cleanup
    free(data);
    free(param);

	// Notify caller task that sending is done
    if (param->caller_task_handle) {
        xTaskNotifyGive(param->caller_task_handle);
    }

    vTaskDelete(NULL); // Delete this task when done
}

static void print_wifi_setting(const char *ifname, rtw_wifi_setting_t *pSetting)
{
#ifndef CONFIG_INIC_NO_FLASH
 
    RTW_API_INFO("\n\r\nWIFI  %s Setting:", ifname);
    RTW_API_INFO("\n\r==============================");
 
    switch (pSetting->mode) {
    case RTW_MODE_AP:
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
        at_printf("\r\nAP,");
#endif
        RTW_API_INFO("\n\r      MODE => AP");
        break;
    case RTW_MODE_STA:
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
        at_printf("\r\nSTA,");
#endif
        RTW_API_INFO("\n\r      MODE => STATION");
        break;
    default:
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
        at_printf("\r\nUNKNOWN,");
#endif
        RTW_API_INFO("\n\r      MODE => UNKNOWN");
    }
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
    at_printf("%s,%d,", pSetting->ssid, pSetting->channel);
#endif
    RTW_API_INFO("\n\r      SSID => %s", pSetting->ssid);
    RTW_API_INFO("\n\r   CHANNEL => %d", pSetting->channel);

	ai_glass_wifi_param_t wifi_param = {0};
	media_get_wifi_params_from_flash(&wifi_param);
    wifi_param.channel = pSetting->channel;

    media_update_wifi_params(&wifi_param);
 
    switch (pSetting->security_type) {
    case RTW_SECURITY_OPEN:
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
        at_printf("OPEN,");
#endif
        RTW_API_INFO("\n\r  SECURITY => OPEN");
        break;
    case RTW_SECURITY_WEP_PSK:
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
        at_printf("WEP,%d,", pSetting->key_idx);
#endif
        RTW_API_INFO("\n\r  SECURITY => WEP");
        RTW_API_INFO("\n\r KEY INDEX => %d", pSetting->key_idx);
        break;
    case RTW_SECURITY_WPA_TKIP_PSK:
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
        at_printf("WPA TKIP,");
#endif
        RTW_API_INFO("\n\r  SECURITY => WPA TKIP");
        break;
    case RTW_SECURITY_WPA2_TKIP_PSK:
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
        at_printf("WPA2 TKIP,");
#endif
        RTW_API_INFO("\n\r  SECURITY => WPA2 TKIP");
        break;
    case RTW_SECURITY_WPA_AES_PSK:
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
        at_printf("WPA AES,");
#endif
        RTW_API_INFO("\n\r  SECURITY => WPA AES");
        break;
    case RTW_SECURITY_WPA_MIXED_PSK:
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
        at_printf("WPA MIX,");
#endif
        RTW_API_INFO("\n\r  SECURITY => WPA Mixed");
        break;
    case RTW_SECURITY_WPA2_AES_PSK:
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
        at_printf("WPA2 AES,");
#endif
        RTW_API_INFO("\n\r  SECURITY => WPA2 AES");
        break;
    case RTW_SECURITY_WPA2_MIXED_PSK:
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
        at_printf("WPA2 Mixd,");
#endif
        RTW_API_INFO("\n\r  SECURITY => WPA2 Mixed");
        break;
    case RTW_SECURITY_WPA_WPA2_TKIP_PSK:
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
        at_printf("WPA/WPA2 TKIP,");
#endif
        RTW_API_INFO("\n\r  SECURITY => WPA/WPA2 TKIP");
        break;
    case RTW_SECURITY_WPA_WPA2_AES_PSK:
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
        at_printf("WPA/WPA2 AES,");
#endif
        RTW_API_INFO("\n\r  SECURITY => WPA/WPA2 AES");
        break;
    case RTW_SECURITY_WPA_WPA2_MIXED_PSK:
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
        at_printf("WPA/WPA2 Mixd,");
#endif
        RTW_API_INFO("\n\r  SECURITY => WPA/WPA2 Mixed");
        break;
    case RTW_SECURITY_WPA3_AES_PSK:
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
        at_printf("WPA3 SAE AES,");
#endif
        RTW_API_INFO("\n\r  SECURITY => WPA3 SAE AES");
        break;
    case RTW_SECURITY_WPA3_GCMP_PSK:
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
        at_printf("WPA3 GCMP,");
#endif
        RTW_API_INFO("\n\r  SECURITY => WPA3 GCMP");
        break;
    case RTW_SECURITY_WPA2_WPA3_MIXED:
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
        at_printf("WPA2/WPA3 AES,");
#endif
        RTW_API_INFO("\n\r  SECURITY => WPA2/WPA3 AES");
        break;
    default:
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
        at_printf("UNKNOWN,");
#endif
        RTW_API_INFO("\n\r  SECURITY => UNKNOWN");
    }
 
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
    at_printf("%s,", pSetting->password);
#endif
    RTW_API_INFO("\n\r  PASSWORD => %s", pSetting->password);
    RTW_API_INFO("\n\r");
#endif
}

void taskprint(void) {
	rtw_phy_statistics_t phy_statistics;
	printf("[ATWR]: _AT_WLAN_GET_RSSI_\n\r");
	wifi_fetch_phy_statistic(&phy_statistics);
	printf("\n\rrssi = %d", phy_statistics.rssi);
	printf("\n\r");
    int i = 0;
#if CONFIG_LWIP_LAYER
    u8 *mac = LwIP_GetMAC(0);
    u8 *ip = LwIP_GetIP(0);
#if LWIP_VERSION_MAJOR >= 2 && LWIP_VERSION_MINOR >= 1
#if LWIP_IPV6
    u8 *ipv6_0 = LwIP_GetIPv6_linklocal(&xnetif[0]);
#if LWIP_IPV6_DHCP6
    u8 *ipv6_1 = LwIP_GetIPv6_global(&xnetif[0]);
#endif
#endif
#endif
    u8 *gw = LwIP_GetGW(0);
    u8 *msk = LwIP_GetMASK(0);
#endif
    u8 *ifname[2] = {(u8 *)WLAN0_NAME, (u8 *)WLAN1_NAME};
    rtw_wifi_setting_t setting;
    rtw_sw_statistics_t stats;
#ifdef CONFIG_RTK_MESH
    int path_tbl_no;
    struct path_sel_entry Entry;
#endif
 
    printf("[ATW?]: _AT_WLAN_INFO_\n\r");
    for (i = 0; i < NET_IF_NUM; i++) {
        if (wifi_is_running(i)) {
#if CONFIG_LWIP_LAYER
            mac = LwIP_GetMAC(i);
            ip = LwIP_GetIP(i);
#if LWIP_VERSION_MAJOR >= 2 && LWIP_VERSION_MINOR >= 1
#if LWIP_IPV6
            ipv6_0 = LwIP_GetIPv6_linklocal(&xnetif[i]);
#if LWIP_IPV6_DHCP6
            ipv6_1 = LwIP_GetIPv6_global(&xnetif[i]);
#endif
#endif
#endif
            gw = LwIP_GetGW(i);
            msk = LwIP_GetMASK(i);
#endif
            printf("\n\r\nWIFI %s Status: Running",  ifname[i]);
            printf("\n\r==============================");
 
            wifi_get_sw_statistic(i, &stats);
            printf("\ntx stat: tx_packets=%d, tx_dropped=%d, tx_bytes=%d\n", (unsigned int)stats.tx_packets, (unsigned int)stats.tx_dropped, (unsigned int)stats.tx_bytes);
            printf("rx stat: rx_packets=%d, rx_dropped=%d, rx_bytes=%d, rx_overflow=%d\n", (unsigned int)stats.rx_packets, (unsigned int)stats.rx_dropped,
                   (unsigned int)stats.rx_bytes, (unsigned int)stats.rx_overflow);
            if (i == 0) {
                printf("max_skbbuf_used_num=%d, skbbuf_used_num=%d\n", stats.max_skbbuf_used_number, stats.skbbuf_used_number);
                printf("max_skbdata_used_num=%d, skbdata_used_num=%d\n", stats.max_skbdata_used_number, stats.skbdata_used_number);
            }
            wifi_get_setting(i, &setting);
            print_wifi_setting((const char *)ifname[i], &setting);
 
#if CONFIG_LWIP_LAYER
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
            at_printf("%02x:%02x:%02x:%02x:%02x:%02x,", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]) ;
            at_printf("%d.%d.%d.%d,", ip[0], ip[1], ip[2], ip[3]);
            at_printf("%d.%d.%d.%d", gw[0], gw[1], gw[2], gw[3]);
#endif
            printf("\n\rInterface (%s)", ifname[i]);
            printf("\n\r==============================");
            printf("\n\r\tMAC => %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]) ;
            printf("\n\r\tIP  => %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
#if LWIP_VERSION_MAJOR >= 2 && LWIP_VERSION_MINOR >= 1
#if LWIP_IPV6
            printf("\n\r\tLink-local IPV6 => %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x",
                   ipv6_0[0], ipv6_0[1],  ipv6_0[2],  ipv6_0[3],  ipv6_0[4],  ipv6_0[5],  ipv6_0[6], ipv6_0[7],
                   ipv6_0[8], ipv6_0[9], ipv6_0[10], ipv6_0[11], ipv6_0[12], ipv6_0[13], ipv6_0[14], ipv6_0[15]);
#if LWIP_IPV6_DHCP6
            printf("\n\r\tIPV6            => %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x",
                   ipv6_1[0], ipv6_1[1],  ipv6_1[2],  ipv6_1[3],  ipv6_1[4],  ipv6_1[5],  ipv6_1[6], ipv6_1[7],
                   ipv6_1[8], ipv6_1[9], ipv6_1[10], ipv6_1[11], ipv6_1[12], ipv6_1[13], ipv6_1[14], ipv6_1[15]);
#endif
#endif
#endif
            printf("\n\r\tGW  => %d.%d.%d.%d", gw[0], gw[1], gw[2], gw[3]);
            printf("\n\r\tmsk  => %d.%d.%d.%d\n\r", msk[0], msk[1], msk[2], msk[3]);
#endif
            if (setting.mode == RTW_MODE_AP || i == 1) {
                int client_number;
                struct {
                    int    count;
                    rtw_mac_t mac_list[AP_STA_NUM];
                } client_info;
 
                client_info.count = AP_STA_NUM;
                wifi_get_associated_client_list(&client_info, sizeof(client_info));
 
                printf("\n\rAssociated Client List:");
                printf("\n\r==============================");
 
                if (client_info.count == 0) {
                    printf("\n\rClient Num: %d\n\r", client_info.count);
                } else {
                    printf("\n\rClient Num: %d", client_info.count);
                    for (client_number = 0; client_number < client_info.count; client_number++) {
                        printf("\n\rClient %d:", client_number + 1);
                        printf("\n\r\tMAC => "MAC_FMT"",
                               MAC_ARG(client_info.mac_list[client_number].octet));
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
                        at_printf("\r\nCLIENT : %d,"MAC_FMT"", client_number + 1, MAC_ARG(client_info.mac_list[client_number].octet));
#endif
#ifdef CONFIG_RTK_MESH
                        if (query_table(client_info.mac_list[client_number].octet, &Entry) == 1) {
                            printf("\n\r\tPATH table => NEXT HOP:"MAC_FMT"\tSN:%d\tMETRIC:%d\t", MAC_ARG(Entry.nexthopMAC), Entry.sn, Entry.metric);
                        }
 
#endif
 
                    }
                    printf("\n\r");
                }
#ifdef CONFIG_RTK_MESH
                printf("\n\rPATH table");
                if (query_whole_table(&path_tbl_no) == 1) {
                    printf("\n\rThere are total %d PATH table", path_tbl_no);
                }
#endif
            }
        }
// show the ethernet interface info
        else {
#if CONFIG_ETHERNET
            if (i == NET_IF_NUM - 1) {
#if CONFIG_LWIP_LAYER
                mac = LwIP_GetMAC(i);
                ip = LwIP_GetIP(i);
                gw = LwIP_GetGW(i);
                printf("\n\rInterface ethernet\n");
                printf("\n\r==============================");
                printf("\n\r\tMAC => %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]) ;
                printf("\n\r\tIP  => %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
                printf("\n\r\tGW  => %d.%d.%d.%d\n\r", gw[0], gw[1], gw[2], gw[3]);
#endif // end CONFIG_LWIP_LAYER
            }
#endif // end CONFIG_ETHERNET
        }
    }
 
#if defined(configUSE_TRACE_FACILITY) && (configUSE_TRACE_FACILITY == 1) && (configUSE_STATS_FORMATTING_FUNCTIONS == 1)
    {
        int buf_len = uxTaskGetNumberOfTasks() * 32;
        char *pcWriteBuffer = NULL;
 
        if (buf_len < 1024) {
            buf_len = 1024;
        }
 
        pcWriteBuffer = (char *)rtw_zmalloc(buf_len);
        if (pcWriteBuffer == NULL) {
            printf("malloc pcWriteBuffer for ATW? fail\n");
            return;
        }
        vTaskList((char *)pcWriteBuffer);
        printf("\n\rTask List: \n\r%s", pcWriteBuffer);
 
        if (pcWriteBuffer) {
            rtw_mfree((u8 *)pcWriteBuffer, 0);
        }
    }
#endif
 
#if (defined(SUPPORT_UART_LOG_SERVICE) && SUPPORT_UART_LOG_SERVICE) || (defined(CONFIG_EXAMPLE_SPI_ATCMD) && CONFIG_EXAMPLE_SPI_ATCMD)
    at_printf("\r\n[ATW?] OK");
#endif
}

static void media_getfile_cb(struct httpd_conn *conn)
{ 
	critical_process_started = 1;

	char *filename = NULL;
	char *user_agent = NULL;
	FILE *http_file = NULL;
	char *buffer = NULL; 
	
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
#if EXTDISK_LOG
			if (strcmp(filename, "uart_log.txt") == 0) {
				ai_glass_extdisk_log_stop();
			}
#endif
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
			
#if defined(MULTI_THREAD_WIFI_SEND_PIC_DATA) && MULTI_THREAD_WIFI_SEND_PIC_DATA
			file_queue = xQueueCreate(QUEUE_LENGTH, sizeof(file_msg_t));
			if (file_queue == NULL) {
				WLAN_SCEN_WARN("Failed to create queue.\r\n");
				transfer_file_normal_internal(conn, http_file);
				goto http_end;
			}

			core_taskhandle = xTaskGetCurrentTaskHandle();

			if (xTaskCreate(http_file_read_thread, "Receiver", 8192, (void *)http_file, 5, &read_taskhandle) != pdPASS) {
				WLAN_SCEN_WARN("Failed to create ReaderTask\n");
				transfer_file_normal_internal(conn, http_file);
				vQueueDelete(file_queue);
				file_queue = NULL;
				goto http_end;
			}

			if (xTaskCreate(http_file_send_thread, "Sender", 8192, (void *)conn, 5, &send_taskhandle) != pdPASS) {
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
			// extdisk_fseek(http_file, 0, SEEK_END); 
			// int file_size = extdisk_ftell(http_file);
			// printf("File_size: %d\r\n", file_size);
			extdisk_fclose(http_file);
			http_file = NULL;
			if ((notifyValue & (WRITE_TASK_SUCCESS_BIT | READ_TASK_SUCCESS_BIT)) == (WRITE_TASK_SUCCESS_BIT | READ_TASK_SUCCESS_BIT)) {
				WLAN_SCEN_MSG("Http send %s completed successfully\r\n", filename);

				// DELETE FILE MECHANISM HAS CHANGED
				if (delete_file_after_upload) {
					// vTaskDelay(2000);
					extdisk_remove(filename);
					extdisk_save_file_cntlist();
					WLAN_SCEN_MSG("Delete file %s\r\n", filename);
				} else {
					WLAN_SCEN_MSG("Keep file %s\r\n", filename);
				}
			} else {
				WLAN_SCEN_WARN("Http send %s fail, ret = %lx\r\n", filename, notifyValue);
			}
#else
			buffer = (char *)malloc(10 * 1024 * 1024);
			if (!buffer) {
				WLAN_SCEN_ERR("Failed to allocate buffer!\n");
				httpd_response_bad_request(conn, (char *)"Memory allocation failed");
				goto http_end;  // buffer is NULL here, safe to free in http_end
			}

			uint32_t  total_read = 0;
			while (1) {
				uint32_t  bytes_read = extdisk_fread(buffer + total_read, 1, (4 * 1024 * 1024) - total_read, http_file);
				if (bytes_read == 0) {
					break;
				}
				total_read += bytes_read;

				if (total_read >= (10 * 1024 * 1024)) {
					WLAN_SCEN_ERR("Warning: file too large for 4MB buffer!\n");
					break;
				}
			}
			WLAN_SCEN_MSG("Image size read into buffer: %lu bytes\n", total_read);

			// Prepare the parameter for the send thread
			heap_send_param_t *param = (heap_send_param_t *)malloc(sizeof(heap_send_param_t));
			if (!param) {
				WLAN_SCEN_ERR("Failed to allocate send param!\n");
				goto http_end;
			}

			param->data = buffer;
			param->length = total_read;
			param->conn = conn;  
			param->caller_task_handle = xTaskGetCurrentTaskHandle();
			extdisk_fclose(http_file);
			http_file = NULL;
			// Create the sending task
			if (xTaskCreate(http_heap_send_thread, "Sender", 8192, (void *)param, 5, NULL) != pdPASS) {
				WLAN_SCEN_ERR("Failed to create send thread\n");
				free(param->data);  // you malloc-ed this earlier
				free(param);
				buffer = NULL;      // prevent double free in http_end
				goto http_end;
			}
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			// Thread owns buffer now, don’t free it again
			buffer = NULL;
#endif
		} else {
			// HTTP/1.1 400 Bad Request
			httpd_response_bad_request(conn, (char *)"Bad Request: Not able to get the resource from the endpoint\r\n");

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
	if (buffer) {
		free(buffer);
		buffer = NULL;
	}
	httpd_conn_close(conn);
	WLAN_SCEN_MSG("[%s] httpd_conn_end (close files): %lu ms\r\n", __func__, rtw_get_current_time() - start_time_httpd_request_get_header_field);
	critical_process_started = 0;
}

static char *extract_value(const char *body, const char *key, int *ptr)
{
	int start = (int)strstr(body, key);
	if (!start) {
		return NULL; // Key not found
	}

	// Move pointer after 'name="key"'
	start = (int)strstr((const char *)start, "\r\n\r\n");
	if (!start) {
		return NULL;
	}
	start += 4; // Skip over the "\r\n\r\n"

	// Find the end of the value (boundary marker)
	int end = (int)strstr((const char *)start, "\r\n");
	if (!end) {
		*ptr = 0;
		return NULL;
	}
	*ptr = end;
	// Allocate memory for extracted value
	size_t len = end - start;
	char *value = (char *)malloc(len + 1);
	if (!value) {
		return NULL;
	}

	strncpy(value, (const char *)start, len);
	value[len] = '\0'; // Null-terminate the string
	return value;
}

static void *memmem(const char *haystack, size_t hlen, const void *needle, size_t nlen)
{
	int needle_first;
	const char *p = haystack;
	size_t plen = hlen;

	if (!nlen) {
		return NULL;
	}

	needle_first = *(unsigned char *)needle;

	while (plen >= nlen && (p = memchr(p, needle_first, plen - nlen + 1))) {
		if (!memcmp(p, needle, nlen)) {
			return (void *)p;
		}

		p++;
		plen = hlen - (p - haystack);
	}

	return NULL;
}

static void *binary_search(const void *haystack, size_t haystack_len, const void *needle, size_t needle_len)
{
	void *ptr = (void *)memmem(haystack, haystack_len, needle, needle_len);
	return ptr;
}

/*
Example Parser format for OTA:

POST /upload HTTP/1.1
Host: (8735 http server address)
Content-Type: multipart/form-data; boundary=----WebKitFormBoundary7MA4YWxkTrZu0gW
Content-Length: (total body size)

------WebKitFormBoundary7MA4YWxkTrZu0gW
Content-Disposition: form-data; name="wifi_version"

wifi_ota_v($wifi_version).bin
------WebKitFormBoundary7MA4YWxkTrZu0gW
Content-Disposition: form-data; name="bt_version"

x.x.x.x
------WebKitFormBoundary7MA4YWxkTrZu0gW
Content-Disposition: form-data; name="wififile"; filename="wifi_ota_v($wifi_version).bin"
Content-Type: application/octet-stream

(contents of the file)
------WebKitFormBoundary7MA4YWxkTrZu0gW
Content-Disposition: form-data; name="btfile"; filename="bt_ota_v($bt_version).bin"
Content-Type: application/octet-stream

(contents of the file)
------WebKitFormBoundary7MA4YWxkTrZu0gW--

*/

void hex_dump(const char *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        printf("%02X ", (unsigned char)data[i]);
        if ((i + 1) % 16 == 0) printf("\n");
    }
    printf("\n");
}


static void save_ota_files_to_emmc_from_http_cb(struct httpd_conn *conn)
{
	critical_process_started = 1;
	// POST /save-ota-files
	if (httpd_request_is_method(conn, (char *)"POST")) {
		char *content_type = NULL;
		char *boundary = NULL;
		FILE *wifi_ota_file = NULL;
		FILE *bt_ota_file = NULL;
		char fwfilename[64];
		char *wifi_version = NULL;
		char *bt_version = NULL;

		// Extract boundary
		if (httpd_request_get_header_field(conn, (char *)"Content-Type", &content_type) != -1) {
			char *boundary_start = strstr(content_type, "boundary=");
			if (boundary_start) {
				boundary = boundary_start + 9; // Skip "boundary="
				WLAN_SCEN_MSG("Boundary: %s\n", boundary);
			} else {
				WLAN_SCEN_ERR("Failed to find boundary\r\n");
				httpd_response_bad_request(conn, (char *)"Bad Request: Failed to find boundary\r\n");
				goto endofparser;
			}
		}

		char boundary_marker[128];
		snprintf(boundary_marker, sizeof(boundary_marker), "--%s", boundary);

		int read_len = 0;

		size_t binary_size = 0;
		int inside_binary_section = 0;
		int file_count = 0;

		size_t content_lengt = conn->request.content_len;
		WLAN_SCEN_MSG("Content-Length: %d\r\n", content_lengt);
		// Parser version
		memset(read_buf, 0, SERVER_READ_BUF_SIZE * 2 + 2);

		int chunk_size = (conn->request.content_len - read_len) > SERVER_READ_BUF_SIZE ? SERVER_READ_BUF_SIZE : (conn->request.content_len - read_len);

		read_len = chunk_size;

		// bytes_read remain how many data in readbuffer
		int bytes_read = httpd_request_read_data(conn, read_buf, chunk_size);

		if (bytes_read <= 0) {
			WLAN_SCEN_ERR("Read version failed\r\n");
			httpd_response_bad_request(conn, (char *)"Bad Request: Read version failed\r\n");
			goto endofparser;
		}

		read_len = bytes_read;

		// boundary
		char *version_ptr = binary_search(read_buf, bytes_read, boundary_marker, strlen(boundary_marker));

		// hex_dump(version_ptr, 8192);

		// Find wifi version
		int next_ptr = 0;
		wifi_version = extract_value((char *)version_ptr, "name=\"wifi_version\"", &next_ptr);
		version_ptr = (char *)(uintptr_t)next_ptr;
		WLAN_SCEN_MSG("wifi pointer: %s\r\n", version_ptr);
		bt_version = extract_value((char *)version_ptr, "name=\"bt_version\"", &next_ptr);
		version_ptr = (char *)(uintptr_t)next_ptr;
		WLAN_SCEN_MSG("bt pointer: %s\r\n", version_ptr);

		if (wifi_version == NULL || bt_version == NULL) {
			// HTTP/1.1 400 Bad Request
			WLAN_SCEN_ERR("[EXTRACT WIFI OR BT VERSION] fail, returning httpd response bad request 400\r\n");
			httpd_response_bad_request(conn, (char *)"Bad Request: Not able to get wifi / bt version\r\n");
			goto endofparser;
		}

		snprintf(fwfilename, sizeof(fwfilename), "%s", wifi_version);
		// Open the two files first
		wifi_ota_file = extdisk_fopen(fwfilename, "wb");

		if (!wifi_ota_file) {
			WLAN_SCEN_ERR("Failed to open wifi ota file for writing.\n");
			httpd_response_bad_request(conn, (char *)"Bad Request: Failed to open wifi ota file for writing.\r\n");
			goto endofparser;
		}

		snprintf(fwfilename, sizeof(fwfilename), "bt_ota_v%s.bin", bt_version);

		bt_ota_file = extdisk_fopen(fwfilename, "wb");

		if (!bt_ota_file) {
			WLAN_SCEN_ERR("Failed to open bt ota file for writing.\n");
			httpd_response_bad_request(conn, (char *)"Bad Request: Failed to open bt ota file for writing.\r\n");
			goto endofparser;
		}

		// Print extracted values
		WLAN_SCEN_MSG("WiFi Version: %s\n", wifi_version ? wifi_version : "Not Found");
		WLAN_SCEN_MSG("BT Version: %s\n", bt_version ? bt_version : "Not Found");

		bytes_read -= (version_ptr - (char *)read_buf);
		memmove(read_buf, version_ptr, bytes_read);

		// Read request body in chunks
		while (1) {
			if (bytes_read < strlen(boundary_marker) && read_len < content_lengt) {
				chunk_size = (conn->request.content_len - read_len) > SERVER_READ_BUF_SIZE ? SERVER_READ_BUF_SIZE : (conn->request.content_len - read_len);
				int tmp_read = httpd_request_read_data(conn, read_buf + bytes_read, chunk_size);
				WLAN_SCEN_INFO("Tmp_read in loop: %d\r\n", tmp_read);
				if (tmp_read < 0) {
					break;
				}
				if (tmp_read != SERVER_READ_BUF_SIZE) {
					WLAN_SCEN_INFO("Remain Data is %s\r\n", read_buf);
				}
				read_len += tmp_read;
				bytes_read += tmp_read;
			} else {
				WLAN_SCEN_INFO("Remain Data is %s\r\n", read_buf);
			}

			if (inside_binary_section) {
				char *boundary_ending_marker = binary_search(read_buf, bytes_read, boundary_marker, strlen(boundary_marker));

				if (boundary_ending_marker) {
					//Binary size need to minus 2 bytes at the end before writing into file. This corresponds to \r\n .
					binary_size = boundary_ending_marker - (char *)read_buf - strlen("\r\n");
					if (binary_size > 0) {
						extdisk_fwrite(read_buf, binary_size, 1, (file_count == 0) ? wifi_ota_file : bt_ota_file);
					} else if (binary_size < 0) {
						WLAN_SCEN_ERR("binary size is negative\r\n");
						httpd_response_bad_request(conn, (char *)"Bad Request: binary size is negative\r\n");
						goto endofparser;
					}
					inside_binary_section = 0;
					file_count++;
					bytes_read -= (boundary_ending_marker - (char *)read_buf);
					memmove(read_buf, boundary_ending_marker, bytes_read);
					if (file_count >= TOTOAL_FILE_NUM) {
						WLAN_SCEN_MSG("Break if file count greater than %d.\r\n", TOTOAL_FILE_NUM);
						break;
					}
					WLAN_SCEN_MSG("End of the file, file count %d\r\n", file_count);
					continue;
				} else {
					if (bytes_read - (strlen(boundary_marker) - 1) > 0) {
						extdisk_fwrite(read_buf, bytes_read - (strlen(boundary_marker) - 1), 1, (file_count == 0) ? wifi_ota_file : bt_ota_file);
					} else if (bytes_read - (strlen(boundary_marker) - 1) < 0) {
						WLAN_SCEN_ERR("ERROR: bytes_read - (strlen(boundary_marker) - 1) is negative\r\n");
						httpd_response_bad_request(conn, (char *)"Bad Request: remain length less than boundary\r\n");
						goto endofparser;
					}
					memmove(read_buf, read_buf + (bytes_read - (strlen(boundary_marker) - 1)), strlen(boundary_marker) - 1);
					bytes_read = strlen(boundary_marker) - 1;
					continue;
				}
			} else {
				char *boundary_ending_marker = binary_search(read_buf, bytes_read, boundary_marker, strlen(boundary_marker));

				if (boundary_ending_marker) {
					bytes_read -= (boundary_ending_marker - (char *)read_buf) + strlen(boundary_marker);
					memmove(read_buf, boundary_ending_marker + strlen(boundary_marker), bytes_read);
					char *binary_start = binary_search((char *)read_buf, bytes_read, "Content-Type: application/octet-stream", strlen("Content-Type: application/octet-stream"));

					if (binary_start) {
						bytes_read -= (binary_start - (char *)read_buf) + strlen("Content-Type: application/octet-stream");
						memmove(read_buf, binary_start + strlen("Content-Type: application/octet-stream"), bytes_read);
						binary_start = binary_search((char *)read_buf, bytes_read, "\r\n\r\n", strlen("\r\n\r\n"));

						if (binary_start) {
							bytes_read -= (binary_start - (char *)read_buf) + strlen("\r\n\r\n");
							memmove(read_buf, binary_start + strlen("\r\n\r\n"), bytes_read);
							inside_binary_section = 1;
							continue;
						} else {
							WLAN_SCEN_ERR("Should not enter here. Entering here means you can find the content type but not the next line.\r\n"); // By right not supposed to enter :recommended to do a while loop
							httpd_response_bad_request(conn, (char *)"Bad Request: format is invalid\r\n");
							goto endofparser;
						}
					}
				} else {
					WLAN_SCEN_ERR("Bad connection and stuck in here\r\n"); // TODO: May need to reset board if entered here.
					httpd_response_bad_request(conn, (char *)"Bad Request: could not find the boundary\r\n");
					goto endofparser;
				}
			}
			vTaskDelay(pdMS_TO_TICKS(100));
			if (read_len >= content_lengt) {
				break;
			}
		}
		// write HTTP response
		httpd_response_write_header_start(conn, (char *)"200 OK", (char *)"text/plain", 0);
		httpd_response_write_header(conn, (char *)"Connection", (char *)"close");
		httpd_response_write_header_finish(conn);

endofparser:
		if (content_type) {
			httpd_free(content_type);
		}
		if (wifi_ota_file) {
			extdisk_fclose(wifi_ota_file);
		}
		if (bt_ota_file) {
			extdisk_fclose(bt_ota_file);
		}
		// Free allocated memory
		if (wifi_version) {
			free(wifi_version);
		}
		if (bt_version) {
			free(bt_version);
		}
	}
	httpd_conn_close(conn);
	critical_process_started = 0;
}

static void save_ota_wifi_file_to_heap_from_http_cb(struct httpd_conn *conn)
{
    critical_process_started = 1;
	if (httpd_request_is_method(conn, (char *)"POST")) {
		char *content_type = NULL;
		char *boundary = NULL;
		char *wifi_version = NULL;
		uint8_t *wifi_ota_buffer = NULL;
		uint32_t wifi_ota_write_offset = 0;
		size_t content_length = 0;

		// Extract boundary
		if (httpd_request_get_header_field(conn, (char *)"Content-Type", &content_type) != -1) {
			char *boundary_start = strstr(content_type, "boundary=");
			if (boundary_start) {
				boundary = boundary_start + 9; // Skip "boundary="
				WLAN_SCEN_MSG("Boundary: %s\n", boundary);
			} else {
				WLAN_SCEN_ERR("Failed to find boundary\r\n");
				httpd_response_bad_request(conn, (char *)"Bad Request: Failed to find boundary\r\n");
				goto endofparser;
			}
		}

		char boundary_marker[128];
		snprintf(boundary_marker, sizeof(boundary_marker), "--%s", boundary);

		int read_len = 0;
		size_t binary_size = 0;
		int inside_binary_section = 0;
		int file_count = 0;
		size_t content_lengt = conn->request.content_len;

		WLAN_SCEN_MSG("Content-Length: %d\r\n", content_lengt);

		// Parser version
		memset(read_buf, 0, SERVER_READ_BUF_SIZE * 2 + 2);

		int chunk_size = (conn->request.content_len - read_len) > SERVER_READ_BUF_SIZE ? SERVER_READ_BUF_SIZE : (conn->request.content_len - read_len);
		read_len = chunk_size;

		// bytes_read remain how many data in readbuffer
		int bytes_read = httpd_request_read_data(conn, read_buf, chunk_size);
		if (bytes_read <= 0) {
			WLAN_SCEN_ERR("Read version failed\r\n");
			httpd_response_bad_request(conn, (char *)"Bad Request: Read version failed\r\n");
			goto endofparser;
		}

		read_len = bytes_read;

		// boundary
		char *version_ptr = binary_search(read_buf, bytes_read, boundary_marker, strlen(boundary_marker));

		// Find wifi version
		int next_ptr = 0;
		wifi_version = extract_value((char *)version_ptr, "name=\"wifi_version\"", &next_ptr);
		version_ptr = (char *)(uintptr_t)next_ptr;
		WLAN_SCEN_MSG("wifi pointer: %s\r\n", version_ptr);

		if (!wifi_version) {
			WLAN_SCEN_ERR("No wifi version found\n");
			httpd_response_bad_request(conn, (char *)"Bad Request: Need wifi_version\r\n");
			goto endofparser;
		}
		WLAN_SCEN_MSG("WiFi OTA version: %s\n", wifi_version);

		// Allocate heap buffer
		if (strncmp(wifi_version, "wifi_ota_v", strlen("wifi_ota_v")) == 0) {
			wifi_ota_buffer = (uint8_t *)malloc(5 * 1024 * 1024);  // 5MB
		} else if (strncmp(wifi_version, "boot_ota_v", strlen("boot_ota_v")) == 0) {
			wifi_ota_buffer = (uint8_t *)malloc(512 * 1024);  // 512KB
		} else if (strncmp(wifi_version, "nn_ota_v", strlen("nn_ota_v")) == 0) {
			wifi_ota_buffer = (uint8_t *)malloc(6 * 1024 * 1024);  // 6MB
		} else if (strncmp(wifi_version, "isp_iq_ota_v", strlen("isp_iq_ota_v")) == 0) {
			wifi_ota_buffer = (uint8_t *)malloc(1024 * 1024);  // 1MB
		} else {
			WLAN_SCEN_ERR("Unknown OTA file: %s\n", wifi_version);
			goto endofparser;
		}

		if (!wifi_ota_buffer) {
			WLAN_SCEN_ERR("Failed to allocate OTA buffer!\n");
			httpd_response_bad_request(conn, (char *)"Memory allocation failed for OTA buffer\r\n");
			goto endofparser;
		}

		bytes_read -= (version_ptr - (char *)read_buf);
		memmove(read_buf, version_ptr, bytes_read);

		// Read request body in chunks
		while (1) {
			if (bytes_read < strlen(boundary_marker) && read_len < content_lengt) {
				chunk_size = (conn->request.content_len - read_len) > SERVER_READ_BUF_SIZE ? SERVER_READ_BUF_SIZE : (conn->request.content_len - read_len);
				int tmp_read = httpd_request_read_data(conn, read_buf + bytes_read, chunk_size);
				if (tmp_read < 0) break;
				read_len += tmp_read;
				bytes_read += tmp_read;
			}

			if (inside_binary_section) {
				char *boundary_ending_marker = binary_search(read_buf, bytes_read, boundary_marker, strlen(boundary_marker));
				if (boundary_ending_marker) {
					binary_size = boundary_ending_marker - (char *)read_buf - strlen("\r\n");
					if (binary_size > 0) {
						memcpy(wifi_ota_buffer + wifi_ota_write_offset, read_buf, binary_size);
						wifi_ota_write_offset += binary_size;
					} else if (binary_size < 0) {
						WLAN_SCEN_ERR("binary size is negative\r\n");
						httpd_response_bad_request(conn, (char *)"Bad Request: binary size is negative\r\n");
						goto endofparser;
					}

					inside_binary_section = 0;
					file_count++;
					bytes_read -= (boundary_ending_marker - (char *)read_buf);
					memmove(read_buf, boundary_ending_marker, bytes_read);

					if (file_count >= TOTOAL_FILE_NUM) {
						WLAN_SCEN_MSG("Break if file count greater than %d.\r\n", TOTOAL_FILE_NUM);
						break;
					}
					WLAN_SCEN_MSG("End of the file, file count %d\r\n", file_count);
					continue;
				} else {
					if (bytes_read - (strlen(boundary_marker) - 1) > 0) {
						memcpy(wifi_ota_buffer + wifi_ota_write_offset, read_buf, bytes_read - (strlen(boundary_marker) - 1));
						wifi_ota_write_offset += (bytes_read - (strlen(boundary_marker) - 1));
					} else if (bytes_read - (strlen(boundary_marker) - 1) < 0) {
						WLAN_SCEN_ERR("ERROR: bytes_read - (strlen(boundary_marker) - 1) is negative\r\n");
						httpd_response_bad_request(conn, (char *)"Bad Request: remain length less than boundary\r\n");
						goto endofparser;
					}
					memmove(read_buf, read_buf + (bytes_read - (strlen(boundary_marker) - 1)), strlen(boundary_marker) - 1);
					bytes_read = strlen(boundary_marker) - 1;
					continue;
				}
			} else {
				char *boundary_ending_marker = binary_search(read_buf, bytes_read, boundary_marker, strlen(boundary_marker));
				if (boundary_ending_marker) {
					bytes_read -= (boundary_ending_marker - (char *)read_buf) + strlen(boundary_marker);
					memmove(read_buf, boundary_ending_marker + strlen(boundary_marker), bytes_read);

					char *binary_start = binary_search((char *)read_buf, bytes_read, "Content-Type: application/octet-stream", strlen("Content-Type: application/octet-stream"));
					if (binary_start) {
						bytes_read -= (binary_start - (char *)read_buf) + strlen("Content-Type: application/octet-stream");
						memmove(read_buf, binary_start + strlen("Content-Type: application/octet-stream"), bytes_read);

						binary_start = binary_search((char *)read_buf, bytes_read, "\r\n\r\n", strlen("\r\n\r\n"));
						if (binary_start) {
							bytes_read -= (binary_start - (char *)read_buf) + strlen("\r\n\r\n");
							memmove(read_buf, binary_start + strlen("\r\n\r\n"), bytes_read);
							inside_binary_section = 1;
							continue;
						} else {
							WLAN_SCEN_ERR("Invalid format: found content type but no \\r\\n\\r\\n\r\n");
							httpd_response_bad_request(conn, (char *)"Bad Request: format is invalid\r\n");
							goto endofparser;
						}
					}
				} else {
					WLAN_SCEN_ERR("Bad connection: could not find the boundary\r\n");
					httpd_response_bad_request(conn, (char *)"Bad Request: could not find the boundary\r\n");
					goto endofparser;
				}
			}

			vTaskDelay(pdMS_TO_TICKS(100));
			if (read_len >= content_lengt) break;
		}

		// --- Print first 32 bytes and last 21 bytes ---
		WLAN_SCEN_INFO("First 32 bytes of OTA buffer (hex): ");
		for (int i = 0; i < 32 && i < wifi_ota_write_offset; i++) {
			WLAN_SCEN_INFO("%02X ", wifi_ota_buffer[i]);
		}
		WLAN_SCEN_INFO("\n");

		if (wifi_ota_write_offset >= 21) {
			WLAN_SCEN_INFO("Last 21 bytes of OTA buffer (hex): ");
			for (int i = wifi_ota_write_offset - 21; i < wifi_ota_write_offset; i++) {
				WLAN_SCEN_INFO("%02X ", wifi_ota_buffer[i]);
			}
			WLAN_SCEN_INFO("\n");
		}

		// --- HTTP response ---
		httpd_response_write_header_start(conn, (char *)"200 OK", (char *)"text/plain", 0);
		httpd_response_write_header(conn, (char *)"Connection", (char *)"close");
		httpd_response_write_header_finish(conn);

		if (!g_heap_ota_data) {
			g_heap_ota_data = malloc(sizeof(heap_ota_data_t));
			if (!g_heap_ota_data) goto endofparser;
			memset(g_heap_ota_data, 0, sizeof(heap_ota_data_t)); // initialize
		}
	
		if (strncmp(wifi_version, "wifi_ota_v", strlen("wifi_ota_v")) == 0) {
			g_heap_ota_data->wifi_data = wifi_ota_buffer;
			g_heap_ota_data->wifi_length = wifi_ota_write_offset;
			strncpy(g_heap_ota_data->wifi_filename, wifi_version, sizeof(g_heap_ota_data->wifi_filename)-1);
			g_heap_ota_data->wifi_filename[sizeof(g_heap_ota_data->wifi_filename)-1] = '\0';
		} else if (strncmp(wifi_version, "boot_ota_v", strlen("boot_ota_v")) == 0) {
			g_heap_ota_data->boot_data = wifi_ota_buffer;
			g_heap_ota_data->boot_length = wifi_ota_write_offset;
			strncpy(g_heap_ota_data->boot_filename, wifi_version, sizeof(g_heap_ota_data->boot_filename)-1);
			g_heap_ota_data->boot_filename[sizeof(g_heap_ota_data->boot_filename)-1] = '\0';
		} else if (strncmp(wifi_version, "nn_ota_v", strlen("nn_ota_v")) == 0) {
			g_heap_ota_data->nn_data = wifi_ota_buffer;
			g_heap_ota_data->nn_length = wifi_ota_write_offset;
			strncpy(g_heap_ota_data->nn_filename, wifi_version, sizeof(g_heap_ota_data->nn_filename)-1);
			g_heap_ota_data->nn_filename[sizeof(g_heap_ota_data->nn_filename)-1] = '\0';
		} else if (strncmp(wifi_version, "isp_iq_ota_v", strlen("isp_iq_ota_v")) == 0) {
			g_heap_ota_data->isp_iq_data = wifi_ota_buffer;
			g_heap_ota_data->isp_iq_length = wifi_ota_write_offset;
			strncpy(g_heap_ota_data->isp_iq_filename, wifi_version, sizeof(g_heap_ota_data->isp_iq_filename)-1);
			g_heap_ota_data->isp_iq_filename[sizeof(g_heap_ota_data->isp_iq_filename)-1] = '\0';
		} else {
			WLAN_SCEN_ERR("Unknown OTA file: %s\n", wifi_version);
			goto endofparser;
		}

		wifi_ota_buffer = NULL;

	endofparser:
		if (content_type) httpd_free(content_type);
		if (wifi_ota_buffer) free(wifi_ota_buffer);
		if (wifi_version) free(wifi_version);
		wifi_ota_write_offset = 0;
	}
    critical_process_started = 0;
    httpd_conn_close(conn);
	WLAN_SCEN_MSG("end of save_ota_wifi_file_to_heap_from_http_cb\r\n");
}

static void save_ota_bt_file_to_heap_from_http_cb(struct httpd_conn *conn)
{
	critical_process_started = 1;
	if (httpd_request_is_method(conn, (char *)"POST")) {
		char *content_type = NULL;
		char *boundary = NULL;
		char *bt_version = NULL;
		uint8_t *bt_ota_buffer = NULL;
		uint32_t bt_ota_write_offset = 0;
		size_t content_length = 0;

		// Extract boundary
		if (httpd_request_get_header_field(conn, (char *)"Content-Type", &content_type) != -1) {
			char *boundary_start = strstr(content_type, "boundary=");
			if (boundary_start) {
				boundary = boundary_start + 9; // Skip "boundary="
				WLAN_SCEN_MSG("Boundary: %s\n", boundary);
			} else {
				WLAN_SCEN_ERR("Failed to find boundary\r\n");
				httpd_response_bad_request(conn, (char *)"Bad Request: Failed to find boundary\r\n");
				goto endofparser;
			}
		}

		char boundary_marker[128];
		snprintf(boundary_marker, sizeof(boundary_marker), "--%s", boundary);

		int read_len = 0;
		size_t binary_size = 0;
		int inside_binary_section = 0;
		int file_count = 0;
		size_t content_lengt = conn->request.content_len;

		WLAN_SCEN_MSG("Content-Length: %d\r\n", content_lengt);

		// Parser version
		memset(read_buf, 0, SERVER_READ_BUF_SIZE * 2 + 2);

		int chunk_size = (conn->request.content_len - read_len) > SERVER_READ_BUF_SIZE ? SERVER_READ_BUF_SIZE : (conn->request.content_len - read_len);
		read_len = chunk_size;

		// bytes_read remain how many data in readbuffer
		int bytes_read = httpd_request_read_data(conn, read_buf, chunk_size);
		if (bytes_read <= 0) {
			WLAN_SCEN_ERR("Read version failed\r\n");
			httpd_response_bad_request(conn, (char *)"Bad Request: Read version failed\r\n");
			goto endofparser;
		}

		read_len = bytes_read;

		// boundary
		char *version_ptr = binary_search(read_buf, bytes_read, boundary_marker, strlen(boundary_marker));

		// Find wifi version
		int next_ptr = 0;

		bt_version = extract_value((char *)version_ptr, "name=\"bt_version\"", &next_ptr);
		version_ptr = (char *)(uintptr_t)next_ptr;

		if (!bt_version) {
			WLAN_SCEN_ERR("No bt version found\n");
			httpd_response_bad_request(conn, (char *)"Bad Request: Need wifi_version\r\n");
			goto endofparser;
		} else {
			bt_ota_buffer = (uint8_t *)malloc(10 * 1024 * 1024);  // 10MB
			WLAN_SCEN_MSG("BT OTA version: %s\n", bt_version);
		}
		
		bytes_read -= (version_ptr - (char *)read_buf);
		memmove(read_buf, version_ptr, bytes_read);

		// Read request body in chunks
		while (1) {
			if (bytes_read < strlen(boundary_marker) && read_len < content_lengt) {
				chunk_size = (conn->request.content_len - read_len) > SERVER_READ_BUF_SIZE ? SERVER_READ_BUF_SIZE : (conn->request.content_len - read_len);
				int tmp_read = httpd_request_read_data(conn, read_buf + bytes_read, chunk_size);
				if (tmp_read < 0) break;
				read_len += tmp_read;
				bytes_read += tmp_read;
			}

			if (inside_binary_section) {
				char *boundary_ending_marker = binary_search(read_buf, bytes_read, boundary_marker, strlen(boundary_marker));
				if (boundary_ending_marker) {
					binary_size = boundary_ending_marker - (char *)read_buf - strlen("\r\n");
					if (binary_size > 0) {
						memcpy(bt_ota_buffer + bt_ota_write_offset, read_buf, binary_size);
						bt_ota_write_offset += binary_size;
					} else if (binary_size < 0) {
						WLAN_SCEN_ERR("binary size is negative\r\n");
						httpd_response_bad_request(conn, (char *)"Bad Request: binary size is negative\r\n");
						goto endofparser;
					}

					inside_binary_section = 0;
					file_count++;
					bytes_read -= (boundary_ending_marker - (char *)read_buf);
					memmove(read_buf, boundary_ending_marker, bytes_read);

					if (file_count >= TOTOAL_FILE_NUM) {
						WLAN_SCEN_MSG("Break if file count greater than %d.\r\n", TOTOAL_FILE_NUM);
						break;
					}
					WLAN_SCEN_MSG("End of the file, file count %d\r\n", file_count);
					continue;
				} else {
					if (bytes_read - (strlen(boundary_marker) - 1) > 0) {
						memcpy(bt_ota_buffer + bt_ota_write_offset, read_buf, bytes_read - (strlen(boundary_marker) - 1));
						bt_ota_write_offset += (bytes_read - (strlen(boundary_marker) - 1));
					} else if (bytes_read - (strlen(boundary_marker) - 1) < 0) {
						WLAN_SCEN_ERR("ERROR: bytes_read - (strlen(boundary_marker) - 1) is negative\r\n");
						httpd_response_bad_request(conn, (char *)"Bad Request: remain length less than boundary\r\n");
						goto endofparser;
					}
					memmove(read_buf, read_buf + (bytes_read - (strlen(boundary_marker) - 1)), strlen(boundary_marker) - 1);
					bytes_read = strlen(boundary_marker) - 1;
					continue;
				}
			} else {
				char *boundary_ending_marker = binary_search(read_buf, bytes_read, boundary_marker, strlen(boundary_marker));
				if (boundary_ending_marker) {
					bytes_read -= (boundary_ending_marker - (char *)read_buf) + strlen(boundary_marker);
					memmove(read_buf, boundary_ending_marker + strlen(boundary_marker), bytes_read);

					char *binary_start = binary_search((char *)read_buf, bytes_read, "Content-Type: application/octet-stream", strlen("Content-Type: application/octet-stream"));
					if (binary_start) {
						bytes_read -= (binary_start - (char *)read_buf) + strlen("Content-Type: application/octet-stream");
						memmove(read_buf, binary_start + strlen("Content-Type: application/octet-stream"), bytes_read);

						binary_start = binary_search((char *)read_buf, bytes_read, "\r\n\r\n", strlen("\r\n\r\n"));
						if (binary_start) {
							bytes_read -= (binary_start - (char *)read_buf) + strlen("\r\n\r\n");
							memmove(read_buf, binary_start + strlen("\r\n\r\n"), bytes_read);
							inside_binary_section = 1;
							continue;
						} else {
							WLAN_SCEN_ERR("Invalid format: found content type but no \\r\\n\\r\\n\r\n");
							httpd_response_bad_request(conn, (char *)"Bad Request: format is invalid\r\n");
							goto endofparser;
						}
					}
				} else {
					WLAN_SCEN_ERR("Bad connection: could not find the boundary\r\n");
					httpd_response_bad_request(conn, (char *)"Bad Request: could not find the boundary\r\n");
					goto endofparser;
				}
			}

			vTaskDelay(pdMS_TO_TICKS(100));
			if (read_len >= content_lengt) break;
		}

		// --- Print first 32 bytes and last 21 bytes ---
		WLAN_SCEN_INFO("First 32 bytes of OTA buffer (hex): ");
		for (int i = 0; i < 32 && i < bt_ota_write_offset; i++) {
			WLAN_SCEN_INFO("%02X ", bt_ota_buffer[i]);
		}
		WLAN_SCEN_INFO("\n");

		if (bt_ota_write_offset >= 21) {
			WLAN_SCEN_INFO("Last 21 bytes of OTA buffer (hex): ");
			for (int i = bt_ota_write_offset - 21; i < bt_ota_write_offset; i++) {
				WLAN_SCEN_INFO("%02X ", bt_ota_buffer[i]);
			}
			WLAN_SCEN_INFO("\n");
		}

		// --- HTTP response ---
		httpd_response_write_header_start(conn, (char *)"200 OK", (char *)"text/plain", 0);
		httpd_response_write_header(conn, (char *)"Connection", (char *)"close");
		httpd_response_write_header_finish(conn);

		if (!g_heap_ota_data) {
			g_heap_ota_data = malloc(sizeof(heap_ota_data_t));
			if (!g_heap_ota_data) goto endofparser;
			memset(g_heap_ota_data, 0, sizeof(heap_ota_data_t)); // initialize
		}
		
		if (bt_version) {
			g_heap_ota_data->bt_data = bt_ota_buffer;
			g_heap_ota_data->bt_length = bt_ota_write_offset;
			strncpy(g_heap_ota_data->bt_version, bt_version, sizeof(g_heap_ota_data->bt_version)-1);
			g_heap_ota_data->bt_version[sizeof(g_heap_ota_data->bt_version)-1] = '\0';
		} else {
			WLAN_SCEN_ERR("Unknown OTA file: %s\n", bt_version);
			goto endofparser;
		}

		bt_ota_buffer = NULL;

	endofparser:
		if (content_type) httpd_free(content_type);
		if (bt_ota_buffer) free(bt_ota_buffer);
		if (bt_version) free(bt_version);
		bt_ota_write_offset = 0;
	}
    critical_process_started = 0;
    httpd_conn_close(conn);
	WLAN_SCEN_MSG("end of save_ota_bt_file_to_heap_from_http_cb\r\n");
	
}

static void delete_file_cb(struct httpd_conn *conn)
{
	critical_process_started = 1;
	// POST /delete-file
	if (httpd_request_is_method(conn, (char *)"POST")) {
		char *content_type = NULL;
		char *boundary = NULL;

		char *delete_file_name = NULL;

		// Extract boundary
		if (httpd_request_get_header_field(conn, (char *)"Content-Type", &content_type) != -1) {
			char *boundary_start = strstr(content_type, "boundary=");
			if (boundary_start) {
				boundary = boundary_start + 9; // Skip "boundary="
				WLAN_SCEN_MSG("Boundary: %s\n", boundary);
			} else {
				WLAN_SCEN_ERR("Failed to find boundary\r\n");
				httpd_response_bad_request(conn, (char *)"Bad Request: Failed to find boundary\r\n");
				goto endofparser;
			}
		}

		char boundary_marker[128];
		snprintf(boundary_marker, sizeof(boundary_marker), "--%s", boundary);

		int read_len = 0;

		size_t binary_size = 0;
		int inside_binary_section = 0;
		int file_count = 0;

		size_t content_lengt = conn->request.content_len;
		WLAN_SCEN_MSG("Content-Length: %d\r\n", content_lengt);
		// Parser version
		memset(read_buf, 0, SERVER_READ_BUF_SIZE * 2 + 2);

		int chunk_size = (conn->request.content_len - read_len) > SERVER_READ_BUF_SIZE ? SERVER_READ_BUF_SIZE : (conn->request.content_len - read_len);

		read_len = chunk_size;

		// bytes_read remain how many data in readbuffer
		int bytes_read = httpd_request_read_data(conn, read_buf, chunk_size);

		if (bytes_read <= 0) {
			WLAN_SCEN_ERR("Read version failed\r\n");
			httpd_response_bad_request(conn, (char *)"Bad Request: Read version failed\r\n");
			goto endofparser;
		}

		read_len = bytes_read;

		// boundary
		char *version_ptr = binary_search(read_buf, bytes_read, boundary_marker, strlen(boundary_marker));

		int next_ptr = 0;
		delete_file_name = extract_value((char *)version_ptr, "name=\"delete_file_name\"", &next_ptr);
		version_ptr = (char *)(uintptr_t)next_ptr;
		// WLAN_SCEN_MSG("delete_file_name pointer: %s\r\n", version_ptr);


		// Print extracted values
		WLAN_SCEN_MSG("Extract file name to be deleted: %s\r\n", delete_file_name ? delete_file_name : "Not Found");
#if EXTDISK_LOG
		if (strcmp(delete_file_name, "uart_log.txt") == 0) {
			ai_glass_extdisk_log_stop();
		}
#endif
		if (delete_file_name != NULL) {
			extdisk_remove(delete_file_name);
			extdisk_save_file_cntlist();
			WLAN_SCEN_MSG("Deleted file %s\r\n", delete_file_name);
			uartcmdpacket_t dummy_param;
			memset(&dummy_param, 0, sizeof(dummy_param));
			AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_FILE_CNT\r\n");
			ai_glass_init_external_disk();
			uint8_t result = AI_GLASS_CMD_COMPLETE;
			uint16_t film_num = extdisk_get_filecount(SYS_COUNT_FILM_LABEL);
			uint16_t snapshot_num = extdisk_get_filecount(SYS_COUNT_PIC_LABEL);

			AI_GLASS_MSG("mp4 file num = %u\r\n", film_num);
			AI_GLASS_MSG("jpg file num = %u\r\n", snapshot_num);
			uart_resp_get_file_cnt(&dummy_param, film_num, snapshot_num, result);
			AI_GLASS_INFO("end of UART_RX_OPC_CMD_GET_FILE_CNT\r\n");
			AI_GLASS_INFO("get UART_RX_OPC_CMD_GET_SD_INFO %lu\r\n", mm_read_mediatime_ms());

			uint64_t device_used_bytes = fatfs_get_used_space_byte();
			uint64_t device_total_bytes = device_used_bytes + fatfs_get_free_space_byte();
			uint32_t device_used_Kbytes = (uint32_t)(device_used_bytes / 1024);
			uint32_t device_total_Kbytes = (uint32_t)(device_total_bytes / 1024);
			uart_resp_get_sd_info(&dummy_param, device_total_Kbytes, device_used_Kbytes);
			AI_GLASS_MSG("Get device memory: %lu/%luKB\r\n", device_used_Kbytes, device_total_Kbytes);
			AI_GLASS_INFO("end of UART_RX_OPC_CMD_GET_SD_INFO %lu\r\n", mm_read_mediatime_ms());
		}
		else {
			WLAN_SCEN_ERR("File name is not extracted successfully\r\n");
			httpd_response_bad_request(conn, (char *)"Bad Request: File name (to be deleted) is not extracted successfully\r\n");
			goto endofparser;
		}

		// write HTTP response
		httpd_response_write_header_start(conn, (char *)"200 OK", (char *)"text/plain", 0);
		httpd_response_write_header(conn, (char *)"Connection", (char *)"close");
		httpd_response_write_header_finish(conn);

		endofparser:
			if (content_type) {
				httpd_free(content_type);
			}
			// Free allocated memory
			if (delete_file_name) {
				free(delete_file_name);
			}
	}
	httpd_conn_close(conn);	
	critical_process_started = 0;
}

static void delete_all_files_cb(struct httpd_conn *conn)
{
	critical_process_started = 1;

    if (httpd_request_is_method(conn, (char *)"POST")) {

        WLAN_SCEN_MSG("Reformatting disk\r\n");
        ai_glass_disk_reformat();

        httpd_response_write_header_start(conn, (char *)"200 OK", (char *)"text/plain", 0);
        httpd_response_write_header(conn, (char *)"Connection", (char *)"close");
        httpd_response_write_header_finish(conn);

    } else {
        httpd_response_write_header_start(conn, (char *)"405 Method Not Allowed", (char *)"text/plain", 0);
        httpd_response_write_header(conn, (char *)"Connection", (char *)"close");
        httpd_response_write_header_finish(conn);
    }

    httpd_conn_close(conn);

	critical_process_started = 0;
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

// TODO function to enable station mode and at the same time start the http server
int wifi_enable_sta_mode(rtw_network_info_t *connect_param, int timeout, int retry)
{
	struct wlan_fast_reconnect read_data = {0};
	/* get last time fast connect info from flash */
	memset(&read_data, 0xff, sizeof(struct wlan_fast_reconnect));
	sys_read_wlan_data_from_flash((uint8_t *) &read_data,  sizeof(struct wlan_fast_reconnect));
	WLAN_SCEN_WARN("AI glass deinit dhcps %lu\r\n", mm_read_mediatime_ms());
	dhcps_deinit();
	WLAN_SCEN_WARN("AI glass wifi_enable_station_mode %lu\r\n", mm_read_mediatime_ms());

#if CONFIG_INIT_NET
#if CONFIG_LWIP_LAYER
	// Initilaize the LwIP stack, if the LwIP is not initalized yet
	extern int lwip_init_done;
	if (!lwip_init_done) {
		LwIP_Init();
	}
#endif
#endif

	if (wifi_on(RTW_MODE_STA) < 0) {
		AI_GLASS_ERR("\n\r[SET STATION MODE] ERROR: wifi_on failed\n");
		return WLAN_SET_FAIL;
	}

	WLAN_SCEN_WARN("wifi_connect cmd done %lu\r\n", mm_read_mediatime_ms());

	wifi_config_autoreconnect_ms(0, retry, timeout);
	int ret = 0;

	if (strcmp((const char *) read_data.psk_essid, (const char *) connect_param->ssid.val) != 0) {
		
		unsigned char *wifi_channel_buf = malloc(FLASH_WIFI_CHANNEL_BLOCK_SIZE);
		if (!wifi_channel_buf) {
			AI_GLASS_ERR("Failed to allocate memory for wifi_channel_buf\n");
			return WLAN_SET_FAIL;
		}
		unsigned int flash_addr = FLASH_WIFI_CHANNEL_BLOCK_BASE;
		ftl_common_read(flash_addr, wifi_channel_buf, FLASH_WIFI_CHANNEL_BLOCK_SIZE);
		
		ai_glass_wifi_param_t param;

		memcpy(&param, wifi_channel_buf + 6, sizeof(param));
		
        if (param.channel == 0xFF || param.channel == 0) {
            AI_GLASS_MSG("Flash channel invalid, doing full scan\r\n");
            connect_param->channel = 0;
            connect_param->pscan_option = 0;
        } else {
            AI_GLASS_MSG("Using saved channel %u\r\n", param.channel);
            wifi_set_channel(param.channel);
            connect_param->channel = param.channel;
            connect_param->pscan_option = PSCAN_FAST_SURVEY;
        }
	
		ret = wifi_connect(connect_param, 1);
		if (ret != RTW_SUCCESS) {
			WLAN_SCEN_WARN("First wifi_connect failed, try again... %lu\r\n",
						mm_read_mediatime_ms());

			ret = wifi_connect(connect_param, 1);
			if (ret != RTW_SUCCESS) {
				WLAN_SCEN_WARN("Second wifi_connect failed, try again... %lu\r\n",
							mm_read_mediatime_ms());

				wifi_config_autoreconnect_ms(1, retry, timeout);
				if (connect_param->channel != 0) {
					connect_param->channel = 0;
					wifi_set_channel(connect_param->channel);
					connect_param->pscan_option = 0;
				}

				ret = wifi_connect(connect_param, 1);
			}
		}
		
		free(wifi_channel_buf);
		wifi_channel_buf = NULL;

		WLAN_SCEN_WARN("wifi_connect cmd done %lu\r\n", mm_read_mediatime_ms());

		WLAN_SCEN_WARN("LwIP_DHCP start %lu\r\n", mm_read_mediatime_ms());
		if(ret == RTW_SUCCESS) {
			LwIP_DHCP(0, DHCP_START);
			WLAN_SCEN_WARN("LwIP_DHCP start done %lu\r\n", mm_read_mediatime_ms());

			WLAN_SCEN_WARN("Connecting to station... %lu\r\n", mm_read_mediatime_ms());
		}
		else {
			WLAN_SCEN_WARN("wifi connect failure... %lu\r\n", mm_read_mediatime_ms());
		}
	}
	WLAN_SCEN_WARN("STA mode start done\r\n");

	//ensure terminate signal is 0
	terminate_signal = 0;

#if defined(HTTP_OTA_TEST) && HTTP_OTA_TEST
	if (xTaskCreate(ota_multicast_send_thread, (const char *)"ota_multicast_send_thread", 1024, NULL, tskIDLE_PRIORITY + 5, NULL) != pdPASS) {
		WLAN_SCEN_ERR("\n\r[%s] Create update task failed", __FUNCTION__);
		return WLAN_SET_FAIL;
	}
#endif

	WLAN_SCEN_WARN("STA mode start dhcp server %lu\r\n", mm_read_mediatime_ms());
	// Setup httpd server
	if (!httpd_is_running()) {
		httpd_reg_page_callback((char *)"/media/*", media_getfile_cb);
		httpd_reg_page_callback((char *)"/pingpong", pingpong_cb);
		httpd_reg_page_callback((char *)"/media-list", media_list_cb);
		httpd_reg_page_callback((char *)"/save-ota-files", save_ota_files_to_emmc_from_http_cb);
		httpd_reg_page_callback((char *)"/save-wifi-ota", save_ota_wifi_file_to_heap_from_http_cb);
		httpd_reg_page_callback((char *)"/save-bt-ota", save_ota_bt_file_to_heap_from_http_cb);
		httpd_reg_page_callback((char *)"/delete-file", delete_file_cb);
		httpd_reg_page_callback((char *)"/delete-all-files", delete_all_files_cb);
#if defined(HTTP_OTA_TEST) && HTTP_OTA_TEST
		httpd_reg_page_callback((char *)"/ota-start", ota_start_cb);
#endif
		httpd_setup_debug(HTTPD_DEBUG_VERBOSE);
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
	WLAN_SCEN_WARN("STA mode start dhcp server done %lu\r\n", mm_read_mediatime_ms());
	return WLAN_SET_OK;
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
	WLAN_SCEN_WARN("AI glass wifi_enable_ap_mode\r\n");
	wifi_fast_connect_enable(0);
#if CONFIG_INIT_NET
#if CONFIG_LWIP_LAYER
	// Initilaize the LwIP stack, if the LwIP is not initalized yet
	extern int lwip_init_done;
	if (!lwip_init_done) {
		LwIP_Init();
	}
#endif
#endif
	terminate_signal = 0;
	WLAN_SCEN_WARN("AI glass Enable Wi-Fi with AP mode\r\n");
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
	WLAN_SCEN_WARN("AI glass Start AP\r\n");
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
	WLAN_SCEN_WARN("AI glass Check AP running\r\n");
	while (1) {
		rtw_wifi_setting_t setting;
		wifi_get_setting(WLAN0_IDX, &setting);
		if (strlen((char *)setting.ssid) > 0) {
			if (strcmp((const char *)setting.ssid, (const char *)ssid) == 0) {
				WLAN_SCEN_WARN("AI glass %s started\r\n", ssid);
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
	WLAN_SCEN_WARN("AI glass Start DHCP server\r\n");
	dhcps_set_compatibilty_enable(1);

#if CONFIG_LWIP_LAYER
	dhcps_init(&xnetif[0]);
#endif

set_http:
#if defined(HTTP_OTA_TEST) && HTTP_OTA_TEST
	if (xTaskCreate(ota_multicast_send_thread, (const char *)"ota_multicast_send_thread", 1024, NULL, tskIDLE_PRIORITY + 5,  NULL) != pdPASS) {
		WLAN_SCEN_ERR("\n\r[%s] Create update task failed", __FUNCTION__);
		return WLAN_SET_FAIL;
	}
#endif
	if (!httpd_is_running()) {
		httpd_reg_page_callback((char *)"/media/*", media_getfile_cb);
		httpd_reg_page_callback((char *)"/pingpong", pingpong_cb);
		httpd_reg_page_callback((char *)"/media-list", media_list_cb);
		httpd_reg_page_callback((char *)"/save-ota-files", save_ota_files_to_emmc_from_http_cb);
		httpd_reg_page_callback((char *)"/save-wifi-ota", save_ota_wifi_file_to_heap_from_http_cb);
		httpd_reg_page_callback((char *)"/save-bt-ota", save_ota_bt_file_to_heap_from_http_cb);
		httpd_reg_page_callback((char *)"/delete-file", delete_file_cb);
		httpd_reg_page_callback((char *)"/delete-all-files", delete_all_files_cb);
#if defined(HTTP_OTA_TEST) && HTTP_OTA_TEST
		httpd_reg_page_callback((char *)"/ota-start", ota_start_cb);
#endif
		httpd_setup_debug(HTTPD_DEBUG_VERBOSE);
		httpd_setup_priority(5);
		httpd_setup_idle_timeout(HTTPD_CONNECT_TIMEOUT);
#if defined(USE_HTTPS) && USE_HTTPS
		// Set up http certificate
		if (httpd_setup_cert(HTTPS_SRC_CRT, HTTPS_SRC_KEY, HTTPS_CA_CRT) != 0) {
			WLAN_SCEN_ERR("ERROR: httpd_setup_cert\r\n");
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
	WLAN_SCEN_WARN("AI glass wifi_disable_ap_mode= %lu\r\n", mm_read_mediatime_ms());
	//WLAN_SCEN_MSG("dhcp stop start= %lu\r\n", mm_read_mediatime_ms());
	//httpd_stop();
	//while (httpd_is_running()) {
	//vTaskDelay(1);
	//}
	terminate_signal = 1;

	WLAN_SCEN_WARN("http service disable= %lu\r\n", mm_read_mediatime_ms());
	if (!wifi_off()) {
		return WLAN_SET_OK;
	}
	WLAN_SCEN_WARN("wlan off done= %lu\r\n", mm_read_mediatime_ms());
	return WLAN_SET_FAIL;
}

int wifi_disable_sta_mode(void)
{
	WLAN_SCEN_WARN("AI glass wifi_disable_sta_mode= %lu\r\n", mm_read_mediatime_ms());
	// WLAN_SCEN_MSG("dhcp stop start= %lu\r\n", mm_read_mediatime_ms());
	// httpd_stop();
	// while (httpd_is_running()) {
	// vTaskDelay(1);
	// }

	terminate_signal = 1;

	WLAN_SCEN_WARN("http service disable= %lu\r\n", mm_read_mediatime_ms());
	if (!wifi_off()) {
		return WLAN_SET_OK;
	}
	WLAN_SCEN_WARN("wlan off done= %lu\r\n", mm_read_mediatime_ms());
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
