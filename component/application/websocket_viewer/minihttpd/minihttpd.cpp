#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
//#include <poll.h>
#include "channel.hpp"
#include "minihttpd.h"
#include "ministd/mymap.hpp"
#include "ministd/mystring.hpp"

#define SZ_BUF 1024
#define MAX_LINES 30
#define MAX_HEADERS 30
#define MAX_PARAMS 30

using namespace ministd;

typedef struct callback_t_
{
	const char *path;
	callback_func_t func;
	void *param;
} callback_t;

// for encapsulate
struct server_t_
{
	int sock;
	map<string, callback_t> *serv_map;
};

typedef struct header_t_
{
	string_t key;
	string_t value;
} header_t;

struct conn_t_
{
	int sock;
	server_t *server;

	string_t method;
	string_t path;
	string_t proto;

	// read data buffer
	char r_buf[SZ_BUF];
	int r_cursor;
	int r_sz_buf;

	// write data buffer (for header)
	char w_buf[SZ_BUF];
	int w_cursor;
	int sz_wbuf;

	// line & header structure
	string_t lines[MAX_LINES];
	header_t headers[MAX_HEADERS];
	header_t params[MAX_PARAMS];
	int num_lines;
	int num_params;

	// runtime parser
	char *newline_start;
	int newline_count;
	int is_r;
};

static void fatal(const char *msg)
{
	perror(msg);
	exit(1);
}

enum
{
	HEADER_CONT = 0,
	HEADER_END
};

static int check_http_header(conn_t *conn, char *p, int size)
{
	for (int i = 0; i < size; i++) {
		if (conn->newline_start == 0)
			conn->newline_start = (char *)p;

		if (*p == '\n') {
			conn->newline_count++;
			conn->lines[conn->num_lines].start = conn->newline_start;
			conn->lines[conn->num_lines].len = p - conn->newline_start;
			if (conn->is_r == 1)
			{
				conn->lines[conn->num_lines].len--;
			}
			conn->num_lines++;
			conn->newline_start = 0;
			conn->is_r = 0; // clear
		} else {
			if (*p == '\r') {
				if (conn->is_r) {
					conn->newline_count = 0;
				} else {
					conn->is_r = 1;
				}
			} else {
				conn->is_r = 0;
				conn->newline_count = 0;
			}
		}
		if (conn->newline_count >= 2) {
			conn->num_lines--;
			return HEADER_END;
		}

		p++;
	}

	return HEADER_CONT;
}

static void print_s(string_s *s)
{
	for (int i = 0; i < s->len; i++) {
		putc(s->start[i], stdout);
	}
}

static void parse_http_proto(conn_t *conn, string_t *req)
{
	char *sp = req->start;
	int len = req->len;
	char *p = sp;

	conn->method.start = p;
	while (p - sp < len) {
		if (*p == 32)
		{
			conn->method.len = p - sp;
			break;
		}
		p++;
	}
	while (p - sp < len) {
		if (*p != 32)
		{
			conn->path.start = p;
			break;
		}
		p++;
	}
	while (p - sp < len) {
		if (*p == 32)
		{
			conn->path.len = p - conn->path.start;
			break;
		}
		p++;
	}
	while (p - sp < len) {
		if (*p != 32)
		{
			conn->proto.start = p;
			break;
		}
		p++;
	}
	conn->proto.len = len - (p - sp);

	// get param
	string_t param;
	memset(&param, 0, sizeof(string_t));
	p = sp = conn->path.start;
	len = conn->path.len;
	while (p - sp < len) {
		if (*p == '?')
		{
			param.start = p + 1;
			param.len = len - (p - sp + 1);
			conn->path.len = p - sp;
			break;
		}
		p++;
	}

	// split param
	if (param.len > 0) {
		int num_params = 0, len = param.len;
		p = sp = param.start;
		while (len > 0) {
			string_t *key = &conn->params[num_params].key;
			string_t *value = &conn->params[num_params].value;
			key->start = sp;
			while (p - sp < len) {
				if (*p == '=') {
					key->len = p - sp;
					p++; // skip '='
					break;
				}
				p++;
			}
			if (p - sp >= len) {
				// end
				len = 0;
				break;
			}
			value->start = p;
			while (p - sp < len) {
				if (*p == '&')
				{
					value->len = p - value->start;
					num_params++;
					p++; // skip '&'
					break;
				}
				p++;
			}
			if (p - sp == len) {
				// end of parameters
				value->len = p - value->start;
				num_params++;
			}

			int tl = key->len + value->len + 2;
			len -= tl;
			sp += tl;
		}
		conn->num_params = num_params;
	}

#if 0
	if (conn->num_params > 0) {
		for (int i=0; i<conn->num_params; i++) {
			printf("param --> (%d)\n", i);
			printf("	k: "); print_s(&conn->params[i].key); printf("\n");
			printf("	v: "); print_s(&conn->params[i].value); printf("\n");
			printf("\n");
		}
	}
#endif
}

static void parse_header(string_t *s, header_t *h)
{
	char *sp = s->start;
	int len = s->len;
	h->key.start = s->start;
	char *p = sp;
	while (p - sp < len) {
		if (*p == ':' || *p == 32)
		{
			h->key.len = p - sp;
			break;
		}
		p++;
	}
	while (p - sp < len) {
		if (*p != ':' && *p != 32)
		{
			h->value.start = p;
			break;
		}
		p++;
	}
	h->value.len = len - (p - sp);
}

static void process_http(conn_t *conn)
{
	if (conn->num_lines == 0)
		return;

	parse_http_proto(conn, &conn->lines[0]);
	for (int i = 1; i < conn->num_lines; i++) {
		parse_header(&conn->lines[i], &conn->headers[i]);
	}
}

static void dispatch_http(server_t *server, conn_t *conn)
{
	string path(conn->path.start, conn->path.len);
	auto i = server->serv_map->find(path);
	if (i != server->serv_map->end()) {
		i->second.func(conn, i->second.param);
	}
}

static void accept_request(server_t *server, int client_sock)
{
	conn_t conn;
	memset(&conn, 0, sizeof(conn_t));

	conn.server = server;
	conn.sock = client_sock;

#if 0
	struct sockaddr_in client_name;
	int client_name_len = sizeof(client_name);
	getsockname(client_sock, (struct sockaddr *)&client_name, (socklen_t *)&client_name_len);
	printf("Connect [%s:%u]\n", inet_ntoa(client_name.sin_addr), ntohs(client_name.sin_port));
#endif

	for (;;) {
		struct pollfd pfd;
		pfd.fd = conn.sock;
		pfd.events = POLLIN | POLLPRI | POLLERR;
		if (poll(&pfd, 1, 500) == 0) {
			// timeout
			printf("poll read timeout, drop this connection (%d bytes)\n", conn.r_sz_buf);
			break;
		}
		if (pfd.revents & POLLERR) {
			break;
		}
		else if ((pfd.revents & POLLIN) || (pfd.events & POLLPRI)) {
			if (SZ_BUF - conn.r_sz_buf > 0) {
				int n = recv(conn.sock, conn.r_buf + conn.r_sz_buf, SZ_BUF - conn.r_sz_buf, 0);
				if (n > 0) {
					char *p = conn.r_buf + conn.r_sz_buf;
					conn.r_sz_buf += n;
					if (check_http_header(&conn, p, n) == HEADER_END) {
						process_http(&conn);
						dispatch_http(server, &conn);
						break;
					}
				}
			} else {
				printf("error: not enough space for http header\n");
			}
		}
	}

	// TODO: check keep alive things
	close(conn.sock);
}

typedef struct chan_data_s
{
	server_t *server;
	int sock;
} chan_data_t;

static void *http_worker(void *param)
{
	Channel<chan_data_t> *chan = (Channel<chan_data_t> *)param;
	for (;;) {
		chan_data_t data;
		data = chan->pop_front();
		accept_request(data.server, data.sock);
	}

	pthread_exit(NULL);
}

int server_start(server_t *server, int port, int num_worker)
{
	struct sockaddr_in name;

	printf("http server at port: %d\n", port);
	server->sock = socket(PF_INET, SOCK_STREAM, 0);
	if (server->sock == -1)
		fatal("socket");

	// set reuse
	int on = 1;
	setsockopt(server->sock, SOL_SOCKET, SO_REUSEADDR, (char *)&on, sizeof(on));
	memset(&name, 0, sizeof(name));
	name.sin_family = AF_INET;
	name.sin_port = htons(port);
	name.sin_addr.s_addr = htonl(INADDR_ANY);
	if (bind(server->sock, (struct sockaddr *)&name, sizeof(name)) < 0)
		fatal("bind");

	if (port == 0) { // if dynamically allocating a port
		int namelen = sizeof(name);
		if (getsockname(server->sock, (struct sockaddr *)&name, (socklen_t *)&namelen) == -1)
			fatal("getsockname");
		port = ntohs(name.sin_port);
	}

	if (listen(server->sock, 5) < 0) {
		fatal("listen error");
	}

	Channel<chan_data_t> chan(num_worker);
	for (int i = 0; i < num_worker; i++) {
		pthread_t t;
		pthread_attr_t attr;
		size_t stacksize;
		pthread_attr_init(&attr);
		pthread_attr_getstacksize(&attr, &stacksize);
		printf("Default stack size = %u\n", stacksize); //-> 2048
		stacksize *= 16;
		printf("New stack size = %u\n", stacksize);
		pthread_attr_setstacksize(&attr, stacksize);
		pthread_create(&t, &attr, http_worker, (void *)&chan);
	}

	for (;;) {
		struct sockaddr_in client_name;
		int client_name_len = sizeof(client_name);
		int client_sock = accept(server->sock, (struct sockaddr *)&client_name, (socklen_t *)&client_name_len);
		if (client_sock == -1)
		{
			fatal("fail to accept socket");
		}
		chan_data_t data;
		data.server = server;
		data.sock = client_sock;
		chan.push_back(data);
	}

	return 0;
}

void client_get_path(conn_t *conn, string_t *s)
{
	memcpy(s, &conn->path, sizeof(string_t));
}

void client_get_method(conn_t *conn, string_t *s)
{
	memcpy(s, &conn->method, sizeof(string_t));
}

void client_get_proto(conn_t *conn, string_t *s)
{
	memcpy(s, &conn->proto, sizeof(string_t));
}

void client_response_write_header_start(conn_t *conn, const char *resp, const char *type, int content_len)
{
	sprintf(conn->w_buf, "HTTP/1.1 %s\nContent-Type:%s\nContent-Length:%d\n", resp, type, content_len);
	conn->w_cursor += strlen(conn->w_buf);
}

void client_response_write_header(conn_t *conn, const char *key, const char *value)
{
	char *p = &conn->w_buf[conn->w_cursor];
	sprintf(p, "%s: %s\n", key, value);
	conn->w_cursor += strlen(p);
}

static void sock_send(int sock, uint8_t *data, int len)
{
	while (len > 0) {
		struct pollfd pfd;
		pfd.fd = sock;
		pfd.events = POLLOUT | POLLERR;
		if (poll(&pfd, 1, 2000) == 0) {
			// timeout
			printf("poll write timeout, drop this connection\n");
			break;
		}
		if (pfd.revents & POLLERR) {
			printf("poll error flag, break\n");
			break;
		}
		else if (pfd.revents & POLLOUT) {
			int n = send(sock, data, len, MSG_NOSIGNAL);
			if (n > 0) {
				len -= n;
				data += n;
			} else if (n < 0) {
				printf("send error, break\n");
				break;
			}
		}
	}
}

void client_response_write_header_finish(conn_t *conn)
{
	char *p = &conn->w_buf[conn->w_cursor];
	*p++ = '\n';
	conn->w_cursor++;
	sock_send(conn->sock, (uint8_t *)conn->w_buf, conn->w_cursor);
}

void client_response_write_data(conn_t *conn, void *data, int len)
{
	sock_send(conn->sock, (uint8_t *)data, len);
}

server_t *server_new()
{
	server_t *server = (server_t *)malloc(sizeof(server_t));
	memset(server, 0, sizeof(server_t));
	server->serv_map = new map<string, callback_t>;

	return server;
}

int server_close(server_t *server)
{
	close(server->sock);
	delete server->serv_map;

	free(server);

	return 0;
}

int client_get_header_field(conn_t *conn, const char *name, char **value)
{
	for (int i = 1; i < conn->num_lines; i++) {
		if (strncmp(conn->headers[i].key.start, name, strlen(name)) == 0 && strlen(name) == (size_t)conn->headers[i].value.len) {
			*value = conn->headers[i].value.start;
			return 0;
		}
	}
	return -1;
}

void client_dump_header(conn_t *conn)
{
	for (int i = 1; i < conn->num_lines; i++) {
		print_s(&conn->headers[i].key);
		printf(": ");
		print_s(&conn->headers[i].value);
		printf("\n");
	}
}

void server_reg_callback(server_t *server, const char *path, callback_func_t func, void *param)
{
	callback_t cb;

	cb.path = path;
	cb.func = func;
	cb.param = param;

	server->serv_map->insert({string(path), cb});
}

void server_clear_callbacks(server_t *server)
{
	server->serv_map->clear();
}
