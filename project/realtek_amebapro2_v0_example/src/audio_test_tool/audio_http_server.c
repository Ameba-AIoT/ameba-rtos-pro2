#include <FreeRTOS.h>
#include <task.h>
#include <platform_stdlib.h>
#include "wifi_conf.h"
#include "lwip_netconf.h"
#include <httpd/httpd.h>
#include "audio_http_server.h"
#include "vfs.h"

#define USE_HTTPS_AUDIO    0

#if USE_HTTPS_AUDIO
// use test_srv_crt, test_srv_key, test_ca_list in PolarSSL certs.c
#if (HTTPD_USE_TLS == HTTPD_TLS_POLARSSL)
#include <polarssl/certs.h>
#elif (HTTPD_USE_TLS == HTTPD_TLS_MBEDTLS)
#include <mbedtls/certs.h>
#endif
#endif

#define MAX_TAG_LEN 32
#define MAXIMUM_FILE_SIZE 100
static char audio_disk_tag[MAX_TAG_LEN];
static uint8_t audio_disk_initialed = 0;

static int audio_file_filter(const char *file_name, int file_name_len)
{
	if (file_name_len) {
		// dump the audio file with ".wav" in the end
		if (file_name[file_name_len - 3] == '.' && file_name[file_name_len - 2] == 'w' && file_name[file_name_len - 1] == 'a' && file_name[file_name_len] == 'v') {
			return 1;
		}
	}
	return 0;
}

static int audio_wavfile_filter_fn(const struct dirent *ent)
{
	//printf("file name: %s\r\n", ent->d_name);
	// only collect the audio file

	if (ent->d_type == DT_DIR) {
		return 0;
	} else {
		if (ent->d_name) {
			int i;

			for (i = 0; i <= PATH_MAX; i++) {
				// find the '\0' (string end)
				if (!(ent->d_name[i])) {
					break;
				}
			}
			return audio_file_filter(ent->d_name, i - 1);
		} else {
			return 0;
		}
	}
}

#define MAX_LINK_LEN 128
char body_filelink[MAX_LINK_LEN];
static void audio_homepage_cb(struct httpd_conn *conn)
{
	char *user_agent = NULL;

	// test log to show brief header parsing
	httpd_conn_dump_header(conn);

	// test log to show extra User-Agent header field
	if (httpd_request_get_header_field(conn, (char *)"User-Agent", &user_agent) != -1) {
		//printf("\nUser-Agent=[%s]\n", user_agent);
		httpd_free(user_agent);
	}



	// GET homepage
	if (httpd_request_is_method(conn, (char *)"GET")) {
		const char *body_head =
			"<HTML><BODY>" \
			"find the audio files below<BR>" \
			"<BR>";
		const char *body_end = "<HTML><BODY>";

		if (audio_disk_initialed) {
			// scandir: audio_disk_tag:/
			struct dirent **namelist = (struct dirent **)malloc(sizeof(struct dirent *)*MAXIMUM_FILE_SIZE);
			int count = scandir((const char *)audio_disk_tag, &namelist, audio_wavfile_filter_fn, alphasort);
			//printf("==> %d files in %s \n\r", count, audio_disk_tag);

			//int body_filelist_len = 0;
			// write HTTP response
			httpd_response_write_header_start(conn, (char *)"200 OK", (char *)"text/html", 0);
			httpd_response_write_header(conn, (char *)"Connection", (char *)"close");
			httpd_response_write_header_finish(conn);

			// write html body
			httpd_response_write_data(conn, (uint8_t *)body_head, strlen((const char *)body_head));
			for (int i = 0; i < count; i++) {
				//printf("[%d] %s len = %d, actual = %d\n\r", i, namelist[i]->d_name, namelist[i]->d_namlen, strlen((const char *)namelist[i]->d_name));
				memset(body_filelink, 0, MAX_LINK_LEN);
				snprintf(body_filelink, MAX_LINK_LEN - 1, "<A href=\"/audio/dump?filename=%s\" target=\"_blank\" download=\"%s\">%s</A><BR>", (const char *)namelist[i]->d_name,
						 (const char *)namelist[i]->d_name, (const char *)namelist[i]->d_name);
				httpd_response_write_data(conn, (uint8_t *)body_filelink, strlen((const char *)body_filelink));
				free(namelist[i]);
			}
			free(namelist);
			httpd_response_write_data(conn, (uint8_t *)body_end, strlen((const char *)body_end));

		} else {
			int body_len = 0;
			char *body;
			//printf("The audio disk name is not register\r\n");
			const char *body_err = "The audio disk is not register yet<BR>";
			body_len = strlen((const char *)body_head) + strlen((const char *)body_err) + strlen((const char *)body_end);
			body = malloc(body_len + 1);
			if (body) {
				memset(body, 0, body_len + 1);
				memcpy(body, body_head, strlen((const char *)body_head));
				memcpy(body + strlen((const char *)body_head), body_err, strlen((const char *)body_err));
				memcpy(body + strlen((const char *)body_head) + strlen((const char *)body_err), body_end, strlen((const char *)body_end));
			} else {
				goto httpd_conn_end;
			}
			// write HTTP response
			httpd_response_write_header_start(conn, (char *)"200 OK", (char *)"text/html", body_len);
			httpd_response_write_header(conn, (char *)"Connection", (char *)"close");
			httpd_response_write_header_finish(conn);
			httpd_response_write_data(conn, (uint8_t *)body, strlen((const char *)body));
			free(body);
		}
	} else {
		// HTTP/1.1 405 Method Not Allowed
		httpd_response_method_not_allowed(conn, NULL);
	}
httpd_conn_end:
	httpd_conn_close(conn);
}

#define AUDIO_DATA_BUF_SIZE 1024
uint8_t audio_data_buf[AUDIO_DATA_BUF_SIZE];
static char http_ram_path[64];
static void pb_audiodump_get_cb(struct httpd_conn *conn)
{
	// GET /test_post
	char *filename = NULL;
	FILE  *http_ram_file = NULL;
	if (httpd_request_is_method(conn, (char *)"GET")) {

		// get 'filename' in query string
		if ((httpd_request_get_query_key(conn, (char *)"filename", &filename) != -1)) {

			//printf("filename = %s\r\n", filename);

			// write HTTP response
			httpd_response_write_header_start(conn, (char *)"200 OK", (char *)"text/plain", 0);
			httpd_response_write_header(conn, (char *)"Connection", (char *)"close");
			httpd_response_write_header_finish(conn);

			if (audio_disk_initialed) {
				snprintf(http_ram_path, sizeof(http_ram_path), "%s%s", audio_disk_tag, filename);
				http_ram_file = fopen(http_ram_path, "r");
				if (!http_ram_file) {
					goto httpd_conn_end;
				}
			}
			int br = 0;
			int res = 0;
			res = fseek(http_ram_file, 0, SEEK_SET);
			while (1) {
				br = fread(audio_data_buf, 1, AUDIO_DATA_BUF_SIZE, http_ram_file);
				if (br < 0) {
					printf("Read ERROR\r\n");
					goto httpd_conn_end;
				} else {
					int ret = 0;
					int send_timeout = 3000;
					if (conn->sock != -1) {
						setsockopt(conn->sock, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));
					}
					ret = httpd_response_write_data(conn, audio_data_buf, br);
					if (ret <= 0) {
						printf("http error ret = %d\r\n", ret);
						goto httpd_conn_end;
					}
					if (br != AUDIO_DATA_BUF_SIZE) {
						break;
					}
				}
			}
		} else {
			// HTTP/1.1 400 Bad Request
			httpd_response_bad_request(conn, (char *)"Bad Request - test1 or test2 not in query string");
		}


	} else {
		// HTTP/1.1 405 Method Not Allowed
		httpd_response_method_not_allowed(conn, NULL);
	}

httpd_conn_end:
	if (filename) {
		httpd_free(filename);
	}
	if (http_ram_file) {
		fclose(http_ram_file);
	}
	httpd_conn_close(conn);
}

// register the audio page in http server
static void register_audio_dump_page(void)
{
	httpd_reg_page_callback((char *)"/audio", audio_homepage_cb);
	httpd_reg_page_callback((char *)"/audio/dump", pb_audiodump_get_cb);
}

#define wifi_wait_time 500 //Here we wait 5 second to wiat the fast connect 
static void audio_httpd_thread(void *param)
{
	/* To avoid gcc warnings */
	(void) param;
	uint32_t wifi_wait_count = 0;
	while (!((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) && (*(u32 *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
		vTaskDelay(10);
		wifi_wait_count++;
		if (wifi_wait_count == wifi_wait_time) {
			printf("\r\nuse ATW0, ATW1, ATWC to make wifi connection\r\n");
			printf("wait for wifi connection...\r\n");
		}
	}
	//printf("audio_httpd_thread\r\n");
#if USE_HTTPS_AUDIO
#if (HTTPD_USE_TLS == HTTPD_TLS_POLARSSL)
	if (httpd_setup_cert(test_srv_crt, test_srv_key, test_ca_crt) != 0) {
#elif (HTTPD_USE_TLS == HTTPD_TLS_MBEDTLS)
	if (httpd_setup_cert(mbedtls_test_srv_crt, mbedtls_test_srv_key, mbedtls_test_ca_crt) != 0) {
#endif
		printf("ERROR: httpd_setup_cert\r\n");
		goto exit;
	}
#endif

	register_audio_dump_page();

#if USE_HTTPS_AUDIO
	if (httpd_start(443, 5, 4096, HTTPD_THREAD_SINGLE, HTTPD_SECURE_TLS) != 0) {
#else
	if (httpd_start(80, 5, 4096, HTTPD_THREAD_SINGLE, HTTPD_SECURE_NONE) != 0) {
#endif
		printf("ERROR: httpd_start\r\n");
		httpd_clear_page_callbacks();
	}
#if USE_HTTPS_AUDIO
exit:
#endif
	vTaskDelete(NULL);
}

// register what the disk the audio files are located
void httpd_register_audio_disk_tag(const char *tag_name, int tag_len) {
	if (tag_len < MAX_TAG_LEN) {
		memset(audio_disk_tag, 0, MAX_TAG_LEN);
		snprintf(audio_disk_tag, MAX_TAG_LEN, "%s:/", tag_name);
		printf("httpd audio disk name %s\r\n", audio_disk_tag);
		audio_disk_initialed = 1;
	} else if (audio_disk_initialed) {
		printf("Invlaid httpd audio tag_len %d >= %d. Use the previous disk name %s\r\n", tag_len, MAX_TAG_LEN, audio_disk_tag);
		audio_disk_initialed = 1;
	} else {
		printf("Invlaid httpd audio tag_len %d >= %d.\r\n", tag_len, MAX_TAG_LEN);
		audio_disk_initialed = 0;
	}
}

// for user already construct a http server in their program
void httpd_register_audio_dump_page(void) {
	register_audio_dump_page();
}

// create a http server for the audio dump
void audio_httpd_create(void) {
	if (xTaskCreate(audio_httpd_thread, ((const char *)"audio_httpd_thread"), 2048, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(audio_httpd_thread) failed", __FUNCTION__);
	}
}
