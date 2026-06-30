#include <FreeRTOS.h>
#include <task.h>
#include <platform_stdlib.h>
#include <httpd/ameba_httpd.h>
#include <platform_opts.h>
#include "htdocs_bin.c"

#define RTK_DEMO_VERSION ("1.0.0")

static unsigned char *htdocs = NULL;
static unsigned int htdocs_len = 0;

void ws_viewer_set_buf(unsigned int buf)
{
	htdocs = (unsigned char *)buf;
	printf("htdocs %p\r\n",buf);
}
void ws_viewer_set_len(unsigned int len)
{
	htdocs_len = len;
	printf("htdocs len %p\r\n",len);
}


typedef struct web_path_s {
    const uint8_t *path;
    const uint8_t *mime;
    const uint8_t *encoding;
    const uint8_t *data;
    uint32_t data_len;
} web_path_t;

inline uint8_t *read_int32(uint8_t *p, int32_t *value)
{
    *value = *((int32_t *)p);
    return p + sizeof(int32_t);
}
#define align4(n)   (((n+3)>>2)<<2)

typedef int (*enumerate_t)(web_path_t *web_path, void *user_data);
static void enumerate_items(uint8_t *p, enumerate_t pfunc, void *user_data)
{
	web_path_t item;
    for (;;) {
		int32_t len;
        p = read_int32(p, &len);
        if (len == 0) 
            break;

        item.path = p;
        p += align4(len);

        p = read_int32(p, &len);
        item.mime = p;
        p += align4(len);

		p = read_int32(p, &len);
		item.encoding = p;
		p += align4(len);

        p = read_int32(p, &len);
        item.data = p;
		item.data_len = len;
        p += align4(len);

		if (pfunc(&item, user_data) == 0) { // stop
			return;
		}
    }
	memset(&item, 0, sizeof(web_path_t));
}

typedef struct match_s {
	const char *target;
	int target_len;
	web_path_t *web_path;
} match_t;

static int match_name(web_path_t *web_path, void *user_data)
{
	match_t *match = (match_t *)user_data;
	//printf("%s\r\n", web_path->path);
	//printf("%s %d %d\r\n", match->target, match->target_len, strlen((char *)web_path->path));
	if (strncmp((char *)web_path->path, match->target, match->target_len) == 0) {
		if (strlen((char *)web_path->path) == match->target_len) {
			memcpy(match->web_path, web_path, sizeof(web_path_t));
			return 0; // found ! and stop searching
		}
	}

	return 1; // continue
}

static void search_path(uint8_t *p, web_path_t *web_path, const char *target, int target_len) 
{
	match_t match;

	if ((target_len==1) && (target[0] == '/')) {
		target = (const char *)"/index";
		target_len = strlen(target);
	}
	match.target = target;
	match.target_len = target_len;
	match.web_path = web_path;

	enumerate_items(p, match_name, &match);
}

static void file_serve(struct httpd_conn *conn)
{
	if (minihttpd_request_is_method(conn, "GET")) {
		web_path_t item;
		memset(&item, 0, sizeof(web_path_t));

		search_path(htdocs, &item, (char *)conn->request.path, conn->request.path_len);
		if (item.path == 0) {
			// not found
			const char *not_found = "<html>Not found</html>";
			minihttpd_response_write_header_start(conn, "404 NOT FOUND", (char *)"text/html", strlen(not_found));
			minihttpd_response_write_header(conn, "Connection", "close");
			minihttpd_response_write_header_finish(conn);
			minihttpd_response_write_data(conn, (void *)not_found, strlen(not_found));
		} else {
			// found and serve
			minihttpd_response_write_header_start(conn, "200 OK", (char *)item.mime, item.data_len);
			if (strlen((char *)item.encoding) > 0) {
				minihttpd_response_write_header(conn, "Content-Encoding", (char *)item.encoding);
			}
			minihttpd_response_write_header(conn, "Connection", "close");
			minihttpd_response_write_header_finish(conn);
			minihttpd_response_write_data(conn, (void *)item.data, item.data_len);
		}
	} else {
		// HTTP/1.1 405 Method Not Allowed
		minihttpd_response_method_not_allowed(conn, NULL);
	}

	minihttpd_conn_close(conn);
}

int register_callback(web_path_t *web_path, void *user_data)
{
	minihttpd_reg_page_callback((const char *)web_path->path, file_serve);
	return 1; // continue
}

static void firmware_version(struct httpd_conn *conn)
{
	if (minihttpd_request_is_method(conn, "GET")) {
		minihttpd_response_write_header_start(conn, "200 OK", "text/plain", strlen(RTK_DEMO_VERSION));
		minihttpd_response_write_header(conn, "Connection", "close");
		minihttpd_response_write_header_finish(conn);
		minihttpd_response_write_data(conn, (void*)RTK_DEMO_VERSION, strlen(RTK_DEMO_VERSION));
	} else {
		// HTTP/1.1 405 Method Not Allowed
		minihttpd_response_method_not_allowed(conn, NULL);
	}

	minihttpd_conn_close(conn);
}

static int get_http_query_key(char *path, char *key, char **value)
{
  // Find the query string in the path (if there is one)
  char *query_start = strchr(path, '?');
  if (!query_start) {
    return 0;  // No query string found
  }

  // Search for the key in the query string
  char *key_start = strstr(query_start + 1, key);
  if (!key_start) {
    return 0;  // Key not found in query string
  }

  // Find the value associated with the key
  char *value_start = key_start + strlen(key);
  if (*value_start != '=') {
    return 0;  // Invalid query string format
  }
  value_start++;  // Skip the equals sign

  // Find the end of the value string
  char *value_end = strchr(value_start, '&');
  if (!value_end) {
	value_end = strstr(value_start, " HTTP");
  }

  // Allocate memory for the value string and copy it
  size_t value_len = value_end - value_start;
  *value = malloc(value_len + 1);
  if (!*value) {
    return -1;  // Memory allocation error
  }
  strncpy(*value, value_start, value_len);
  (*value)[value_len] = '\0';

  return 1;  // Success
}

static void register_callbacks(void)
{
	enumerate_items(htdocs, register_callback, 0);
}

static void httpd_thread(void *param)
{

	if (htdocs == NULL) {
		printf("Failed to read %s from sdcard... \r\n", "htdocs.bin");
		printf("read htdoc from flash... \r\n");
		htdocs = htdocs_bin;
		htdocs_len = htdocs_bin_len;
	}

	/* To avoid gcc warnings */
	(void) param;
	register_callbacks();
	minihttpd_reg_page_callback("/", file_serve); // register root
	minihttpd_reg_page_callback("/fw_ver", firmware_version);

	if (minihttpd_start(80, 5, 4096, HTTPD_THREAD_SINGLE, HTTPD_SECURE_NONE) != 0) {
		printf("ERROR: minihttpd_start");
		minihttpd_clear_page_callbacks();
	}

	vTaskDelete(NULL);
}

void start_httpd(void) {
	if (xTaskCreate(httpd_thread, ((const char *)"httpd_thread"), 2048, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(httpd_thread) failed", __FUNCTION__);
	}
}
