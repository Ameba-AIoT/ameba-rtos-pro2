#include <string.h>
#include <stdlib.h>
#include "httpd/ameba_httpd.h"
#include "minihttpd.h"

static server_t *server = 0;

void minihttpd_conn_dump_header(struct httpd_conn *conn)
{
    client_dump_header((conn_t *)conn->obj);
}

int minihttpd_request_get_header_field(struct httpd_conn *conn, const char *name, char **value)
{
    return client_get_header_field((conn_t *)conn->obj, name, value);
}

void minihttpd_free(void *p)
{
    free(p);
}

int minihttpd_request_is_method(struct httpd_conn *conn, const char *method)
{
    string_s m;
    client_get_method((conn_t *)conn->obj, &m);
    if ((strncmp(m.start, method, strlen(method)) == 0)
            && (strlen(method) == (size_t)m.len)) {
        return 1;
    }
    return 0;
}

void minihttpd_conn_close(struct httpd_conn *conn)
{
    // Do nothing
}

void minihttpd_response_method_not_allowed(struct httpd_conn *conn, void *data)
{
    // output error 405, but do nothing
    static const char *method_now_allowed = "<html><h1>Method Now Allowed</h1></html>";
    minihttpd_response_write_header_start(conn, "405 Method Now Allowed", "text/html", strlen(method_now_allowed));
    minihttpd_response_write_header_finish(conn);
    minihttpd_response_write_data(conn, (void *)method_now_allowed, strlen(method_now_allowed));
}

void minihttpd_response_write_header_start(struct httpd_conn *conn, const char *res, const char *mime_type, size_t len)
{
    client_response_write_header_start((conn_t *)conn->obj, res, mime_type, len);
}

void minihttpd_response_write_header(struct httpd_conn *conn, const char *name, const char *value)
{
    client_response_write_header((conn_t *)conn->obj, name, value);
}

void minihttpd_response_write_header_finish(struct httpd_conn *conn)
{
    client_response_write_header_finish((conn_t *)conn->obj);
}

void minihttpd_response_write_data(struct httpd_conn *conn, void *body, size_t size)
{
    client_response_write_data((conn_t *)conn->obj, body, size);
}

int minihttpd_start(int port, int a, int b, int thread_mode, int security)
{
    server_start(server, port, 5); // num_worker = 5
    return 0;
}

static void bridge_func(conn_t *conn, void *param)
{
    struct httpd_conn c;
    string_s path;

    client_get_path(conn, &path);
    c.obj = conn;
    c.request.path = path.start;
    c.request.path_len = path.len;
    http_cb_t f = (http_cb_t)param;
    f(&c);
}

void minihttpd_reg_page_callback(const char *path, http_cb_t cb)
{
    if (server == 0) {
        server = server_new();
    }
    server_reg_callback(server, path, bridge_func, (void *)cb);
}

void minihttpd_clear_page_callbacks()
{
    server_clear_callbacks(server);
}

void minihttpd_response_internal_server_error(struct httpd_conn *conn, void *data)
{
    // do nothing, but send internal error here
}

size_t minihttpd_request_read_data(struct httpd_conn *conn, void *body, int bodylen)
{
    // do nothing
    return 0;
}

void minihttpd_response_bad_request(struct httpd_conn *conn, const char *output)
{
    // do nothing
}

int minihttpd_request_get_query_key(struct httpd_conn *conn, const char *key, char **value)
{
    return -1;
}
