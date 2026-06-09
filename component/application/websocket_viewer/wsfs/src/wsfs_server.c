/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 * All rights reserved.
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "wsfs_internal.h"
#include "wsfs_socket.h"

#include <base64.h>
#include <sha1.h>
#include <utf8.h>

#define WSFS_HANDSHAKE_MAGIC "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
#define WSFS_SELECT_TIMEOUT_MS 10

static void wsfs_apply_defaults(wsfs_server_config_t *config)
{
	if (config == NULL)
		return;

	config->host = WSFS_DEFAULT_HOST;
	config->port = WSFS_DEFAULT_PORT;
	config->max_clients = WSFS_DEFAULT_MAX_CLIENTS;
	config->recv_buffer_size = WSFS_DEFAULT_RECV_BUFFER_SIZE;
	config->max_frame_size = WSFS_DEFAULT_MAX_FRAME_SIZE;
	config->handshake_buffer_cap = WSFS_DEFAULT_HANDSHAKE_BUFFER_CAP;
	memset(&config->callbacks, 0, sizeof(config->callbacks));
}

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

static wsfs_status_t wsfs_buffer_grow(wsfs_buffer_t *buffer,
				       size_t min_capacity)
{
	if (buffer->capacity >= min_capacity)
		return WSFS_STATUS_OK;

	size_t capacity = buffer->capacity == 0 ? 512 : buffer->capacity;
	if (buffer->capacity == 0 && min_capacity > capacity)
		capacity = min_capacity;
	while (capacity < min_capacity)
		capacity *= 2;

	uint8_t *next = (uint8_t *)wsfs_realloc(
		buffer->data, capacity * sizeof(uint8_t), __func__);
	if (next == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_ALLOCATION_FAILED;
	}

	buffer->data = next;
	buffer->capacity = capacity;
	return WSFS_STATUS_OK;
}

static void wsfs_slot_reset(wsfs_slot_t *slot)
{
	if (slot == NULL)
		return;

	slot->fd = WSFS_INVALID_SOCKET;
	slot->conn = NULL;
	slot->wants_write = 0;
}

static void wsfs_free_connection(wsfs_connection_impl_t *conn)
{
	if (conn == NULL)
		return;

	wsfs_connection_deinit(conn);
	wsfs_free(conn, __func__);
}

void wsfs_refresh_fd_sets(wsfs_server_impl_t *impl)
{
	if (impl == NULL)
		return;

	FD_ZERO(&impl->read_fds_master);
	FD_ZERO(&impl->write_fds_master);
	impl->max_fd = -1;

	if (impl->listen_fd != WSFS_INVALID_SOCKET) {
		FD_SET(impl->listen_fd, &impl->read_fds_master);
		impl->max_fd = impl->listen_fd;
	}

	for (size_t i = 1; i < impl->slot_count; ++i) {
		wsfs_slot_t *slot = &impl->slots[i];
		if (slot->fd == WSFS_INVALID_SOCKET)
			continue;
		FD_SET(slot->fd, &impl->read_fds_master);
		if (slot->wants_write)
			FD_SET(slot->fd, &impl->write_fds_master);
		if (slot->fd > impl->max_fd)
			impl->max_fd = slot->fd;
	}
}

static wsfs_status_t wsfs_allocate_runtime(wsfs_server_impl_t *impl)
{
	impl->slot_count = impl->config.max_clients + 1;

	impl->slots = (wsfs_slot_t *)calloc(impl->slot_count,
					 sizeof(wsfs_slot_t));
	if (impl->slots == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_ALLOCATION_FAILED;
	}

	impl->recv_scratch = (uint8_t *)malloc(impl->config.recv_buffer_size);
	if (impl->recv_scratch == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_ALLOCATION_FAILED;
	}

	for (size_t i = 0; i < impl->slot_count; ++i) {
		wsfs_slot_reset(&impl->slots[i]);
	}

	impl->max_fd = -1;
	FD_ZERO(&impl->read_fds_master);
	FD_ZERO(&impl->write_fds_master);

	return WSFS_STATUS_OK;
}

void wsfs_server_config_defaults(wsfs_server_config_t *config)
{
	if (config == NULL)
		return;

	wsfs_apply_defaults(config);
}

wsfs_status_t wsfs_server_init(wsfs_server_t *server,
			       const wsfs_server_config_t *config)
{
	if (server == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_ARGUMENT;
	}

	wsfs_server_impl_t *impl =
		(wsfs_server_impl_t *)calloc(1, sizeof(*impl));
	if (impl == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_ALLOCATION_FAILED;
	}

	wsfs_server_config_t effective;
	wsfs_apply_defaults(&effective);

	if (config != NULL) {
		if (config->host != NULL)
			effective.host = config->host;
		if (config->port != 0)
			effective.port = config->port;
		if (config->max_clients != 0)
			effective.max_clients = config->max_clients;
		if (config->recv_buffer_size != 0)
			effective.recv_buffer_size = config->recv_buffer_size;
		if (config->max_frame_size != 0)
			effective.max_frame_size = config->max_frame_size;
		if (config->handshake_buffer_cap != 0)
			effective.handshake_buffer_cap =
			config->handshake_buffer_cap;
		effective.callbacks = config->callbacks;
	}

	impl->config = effective;
	impl->listen_fd = WSFS_INVALID_SOCKET;
	impl->stop_flag = 0;
	wsfs_status_t status = wsfs_allocate_runtime(impl);
	if (status != WSFS_STATUS_OK) {
		wsfs_free(impl->slots, __func__);
		wsfs_free(impl->recv_scratch, __func__);
		wsfs_free(impl, __func__);
		return status;
	}

	wsfs_server_set_impl(server, impl);
	return WSFS_STATUS_OK;
}

static void wsfs_server_close_listener(wsfs_server_impl_t *impl)
{
	if (impl->listen_fd != WSFS_INVALID_SOCKET) {
		wsfs_socket_close(impl->listen_fd);
		impl->listen_fd = WSFS_INVALID_SOCKET;
	}
	if (impl->slot_count > 0) {
		wsfs_slot_reset(&impl->slots[0]);
	}
	wsfs_refresh_fd_sets(impl);
}

void wsfs_server_deinit(wsfs_server_t *server)
{
	if (server == NULL)
		return;

	wsfs_server_impl_t *impl = wsfs_server_get_impl(server);
	if (impl == NULL)
		return;

	wsfs_server_close_listener(impl);

	for (size_t i = 1; i < impl->slot_count; ++i) {
		if (impl->slots[i].conn != NULL)
			wsfs_free_connection(impl->slots[i].conn);
	}

	wsfs_free(impl->slots, __func__);
	wsfs_free(impl->recv_scratch, __func__);
	wsfs_free(impl, __func__);
	wsfs_server_set_impl(server, NULL);
}

static wsfs_status_t wsfs_server_open_listener(wsfs_server_impl_t *impl)
{
	if (impl->listen_fd != WSFS_INVALID_SOCKET)
		return WSFS_STATUS_OK;

	wsfs_socket_t fd = socket(AF_INET, SOCK_STREAM, 0);
	if (fd == WSFS_INVALID_SOCKET) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_IO_ERROR;
	}

	int yes = 1;
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons((uint16_t)impl->config.port);

	const char *host = impl->config.host;
	if (host == NULL || host[0] == '\0' || strcmp(host, "0.0.0.0") == 0) {
		addr.sin_addr.s_addr = htonl(INADDR_ANY);
	} else {
		if (inet_pton(AF_INET, host, &addr.sin_addr) != 1) {
			wsfs_socket_close(fd);
			WSFS_ERROR_PRINTF(" ");
			return WSFS_STATUS_IO_ERROR;
		}
	}

	if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
		wsfs_socket_close(fd);
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_IO_ERROR;
	}

	if (listen(fd, (int)impl->config.max_clients) != 0) {
		wsfs_socket_close(fd);
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_IO_ERROR;
	}

	impl->listen_fd = fd;
	impl->slots[0].fd = impl->listen_fd;
	impl->slots[0].conn = NULL;
	wsfs_refresh_fd_sets(impl);
	return WSFS_STATUS_OK;
}

static wsfs_status_t wsfs_handshake_append(wsfs_connection_impl_t *conn,
					 const uint8_t *data, size_t length)
{
	if (length == 0)
		return WSFS_STATUS_OK;

	size_t new_len = conn->handshake_buffer.length + length;
	if (new_len > conn->server->config.handshake_buffer_cap) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_PROTOCOL_ERROR;
	}

	wsfs_status_t reserve = wsfs_buffer_grow(&conn->handshake_buffer,
						      new_len);
	if (reserve != WSFS_STATUS_OK)
		return reserve;

	memcpy(conn->handshake_buffer.data + conn->handshake_buffer.length,
	       data, length);
	conn->handshake_buffer.length = new_len;
	return WSFS_STATUS_OK;
}

static ssize_t wsfs_find_handshake_end(const wsfs_connection_impl_t *conn)
{
	if (conn->handshake_buffer.length < 4)
		return -1;

	const uint8_t *buffer = conn->handshake_buffer.data;
	size_t len = conn->handshake_buffer.length;
	for (size_t i = 0; i + 3 < len; ++i) {
		if (buffer[i] == '\r' && buffer[i + 1] == '\n' &&
		    buffer[i + 2] == '\r' && buffer[i + 3] == '\n')
			return (ssize_t)(i + 4);
	}
	return -1;
}

static void wsfs_send_http_error(wsfs_connection_impl_t *conn)
{
	static const char response[] =
		"HTTP/1.1 400 Bad Request\r\n"
		"Connection: close\r\n"
		"Content-Length: 0\r\n"
		"\r\n";

	if (conn == NULL || conn->fd == WSFS_INVALID_SOCKET)
		return;

	(void)send(conn->fd, response, sizeof(response) - 1, 0);
}

static bool wsfs_header_token_contains(const char *value,
					    const char *token)
{
	if (value == NULL || token == NULL)
		return false;

	size_t value_len = strlen(value);
	char *lower = (char *)malloc(value_len + 1);
	if (lower == NULL)
		return false;

	for (size_t i = 0; i < value_len; ++i) {
		char c = value[i];
		if (c >= 'A' && c <= 'Z')
			lower[i] = (char)(c + 32);
		else
			lower[i] = c;
	}
	lower[value_len] = '\0';

	bool contains = strstr(lower, token) != NULL;
	wsfs_free(lower, __func__);
	return contains;
}

static bool wsfs_is_valid_close_code(uint16_t code)
{
	if (code < 1000)
		return false;
	if (code >= 1000 && code <= 1015) {
		return !(code == 1004 || code == 1005 || code == 1006);
	}
	if (code >= 3000 && code <= 4999)
		return true;
	return false;
}

static wsfs_status_t wsfs_consume_bytes(wsfs_server_impl_t *impl,
				     wsfs_connection_impl_t *conn,
				     const uint8_t *data, size_t length,
				     uint16_t *close_code);

static void wsfs_collapse_lws(char *value)
{
	if (value == NULL)
		return;

	size_t write_index = 0;
	for (size_t read_index = 0; value[read_index] != '\0'; ++read_index) {
		char ch = value[read_index];
		if (ch == '\r' || ch == '\n')
			continue;
		value[write_index++] = ch;
	}
	value[write_index] = '\0';
}

static wsfs_status_t wsfs_complete_handshake(wsfs_server_impl_t *impl,
					 wsfs_connection_impl_t *conn)
{
	size_t len = conn->handshake_buffer.length;
	char *request = (char *)malloc(len + 1);
	if (request == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_ALLOCATION_FAILED;
	}

	memcpy(request, conn->handshake_buffer.data, len);
	request[len] = '\0';

	char *saveptr = NULL;
	char *line = strtok_r(request, "\r\n", &saveptr);
	if (line == NULL) {
		wsfs_free(request, __func__);
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_PROTOCOL_ERROR;
	}

	char *method = strtok(line, " ");
	char *path = strtok(NULL, " ");
	char *version = strtok(NULL, " ");
	if (method == NULL || path == NULL || version == NULL) {
		wsfs_free(request, __func__);
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_PROTOCOL_ERROR;
	}

	if (strcmp(method, "GET") != 0 || strncmp(version, "HTTP/1.1", 8) != 0) {
		wsfs_free(request, __func__);
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_PROTOCOL_ERROR;
	}

	bool has_upgrade = false;
	bool has_connection = false;
	bool has_version = false;
	bool has_host = false;
	char sec_key[128] = {0};

	for (line = strtok_r(NULL, "\r\n", &saveptr); line != NULL;
	     line = strtok_r(NULL, "\r\n", &saveptr)) {
		char *colon = strchr(line, ':');
		if (colon == NULL)
			continue;

		*colon = '\0';
		char *key = line;
		char *value = colon + 1;
		while (*value == ' ' || *value == '\t')
			++value;

		if (strcasecmp(key, "Upgrade") == 0) {
			if (wsfs_header_token_contains(value, "websocket"))
				has_upgrade = true;
		} else if (strcasecmp(key, "Connection") == 0) {
			if (wsfs_header_token_contains(value, "upgrade"))
				has_connection = true;
		} else if (strcasecmp(key, "Sec-WebSocket-Version") == 0) {
			has_version = (strcmp(value, "13") == 0);
		} else if (strcasecmp(key, "Sec-WebSocket-Key") == 0) {
			strncpy(sec_key, value, sizeof(sec_key) - 1);
		} else if (strcasecmp(key, "Host") == 0) {
			has_host = true;
		}
	}

	char *end = sec_key + strlen(sec_key);
	while (end > sec_key && (end[-1] == ' ' || end[-1] == '\t'))
		*--end = '\0';

	if (!has_upgrade || !has_connection || !has_version ||
	    !has_host || sec_key[0] == '\0') {
		wsfs_free(request, __func__);
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_PROTOCOL_ERROR;
	}

	char accept_src[256];
	snprintf(accept_src, sizeof(accept_src), "%s%s", sec_key,
	         WSFS_HANDSHAKE_MAGIC);

	SHA1Context ctx;
	uint8_t hash[SHA1HashSize];
	SHA1Reset(&ctx);
	SHA1Input(&ctx, (const uint8_t *)accept_src,
		  (unsigned int)strlen(accept_src));
	SHA1Result(&ctx, hash);

	unsigned char *accept_key = base64_encode(hash, SHA1HashSize, NULL);
	if (accept_key == NULL) {
		wsfs_free(request, __func__);
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_ALLOCATION_FAILED;
	}
	wsfs_collapse_lws((char *)accept_key);

	char response[256];
	snprintf(response, sizeof(response),
	         "HTTP/1.1 101 Switching Protocols\r\n"
	         "Upgrade: websocket\r\n"
	         "Connection: Upgrade\r\n"
	         "Sec-WebSocket-Accept: %s\r\n\r\n",
	         accept_key);

	wsfs_free(accept_key, __func__);
	wsfs_free(request, __func__);

	wsfs_status_t status = wsfs_enqueue_raw(conn,
					(const uint8_t *)response,
					strlen(response));
	if (status != WSFS_STATUS_OK)
		return status;
	conn->handshake_complete = 1;
	conn->state = WSFS_CONN_OPEN;
	wsfs_frame_reader_reset(&conn->frame_reader);
	wsfs_update_poll_events(impl, conn);

	if (impl->config.callbacks.on_open)
		impl->config.callbacks.on_open(&conn->api);

	return WSFS_STATUS_OK;
}

static wsfs_status_t wsfs_handle_handshake(wsfs_server_impl_t *impl,
					wsfs_connection_impl_t *conn,
					const uint8_t *data, size_t length,
					uint16_t *close_code,
					bool *completed)
{
	*completed = false;

	wsfs_status_t status = wsfs_handshake_append(conn, data, length);
	if (status != WSFS_STATUS_OK)
		return status;

	ssize_t handshake_len = wsfs_find_handshake_end(conn);
	if (handshake_len < 0)
		return WSFS_STATUS_OK;

	size_t total_len = conn->handshake_buffer.length;
	size_t consumed = (size_t)handshake_len;
	size_t leftover = total_len > consumed ? total_len - consumed : 0;
	uint8_t *spill = NULL;
	if (leftover > 0) {
		spill = (uint8_t *)malloc(leftover);
		if (spill == NULL) {
			WSFS_ERROR_PRINTF(" ");
			return WSFS_STATUS_ALLOCATION_FAILED;
		}
		memcpy(spill, conn->handshake_buffer.data + consumed, leftover);
	}

	conn->handshake_buffer.length = (size_t)handshake_len;
	status = wsfs_complete_handshake(impl, conn);
	if (status != WSFS_STATUS_OK) {
		wsfs_free(spill, __func__);
		return status;
	}

	conn->handshake_buffer.length = 0;
	*completed = true;

	if (leftover > 0) {
		status = wsfs_consume_bytes(impl, conn, spill, leftover,
					close_code);
	}

	wsfs_free(spill, __func__);
	return status;
}

static wsfs_status_t wsfs_handle_data_frame(wsfs_server_impl_t *impl,
                                           wsfs_connection_impl_t *conn,
                                           wsfs_frame_reader_t *reader,
                                           uint16_t *close_code)
{
	wsfs_buffer_t *payload = wsfs_frame_payload_buffer(reader);
	size_t length = (size_t)wsfs_frame_payload_length(reader);
	wsfs_opcode_t opcode = reader->opcode;
	int fin = wsfs_frame_is_fin(reader);

	if (opcode == WSFS_OPCODE_CONTINUATION) {
		if (!conn->fragmented_in_progress) {
			WSFS_ERROR_PRINTF(" ");
			return WSFS_STATUS_PROTOCOL_ERROR;
		}
	} else if (opcode == WSFS_OPCODE_TEXT || opcode == WSFS_OPCODE_BINARY) {
		if (conn->fragmented_in_progress) {
			WSFS_ERROR_PRINTF(" ");
			return WSFS_STATUS_PROTOCOL_ERROR;
		}
		conn->fragmented_in_progress = !fin;
		conn->message_opcode = opcode;
		if (!fin)
			conn->message_buffer.length = 0;
	} else {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_PROTOCOL_ERROR;
	}

	if (conn->fragmented_in_progress || opcode == WSFS_OPCODE_CONTINUATION) {
		size_t new_len = conn->message_buffer.length + length;
		wsfs_status_t reserve = wsfs_buffer_grow(&conn->message_buffer,
						      new_len);
		if (reserve != WSFS_STATUS_OK)
			return reserve;
		memcpy(conn->message_buffer.data + conn->message_buffer.length,
		       payload->data, length);
		conn->message_buffer.length = new_len;
	}

	if (fin) {
		const uint8_t *data_ptr;
		size_t data_len;
		wsfs_opcode_t deliver_opcode;

		if (conn->fragmented_in_progress || opcode == WSFS_OPCODE_CONTINUATION) {
			data_ptr = conn->message_buffer.data;
			data_len = conn->message_buffer.length;
			deliver_opcode = conn->message_opcode;
			conn->fragmented_in_progress = false;
			conn->message_buffer.length = 0;
		} else {
			data_ptr = payload->data;
			data_len = length;
			deliver_opcode = opcode;
			conn->message_buffer.length = 0;
		}

		if (deliver_opcode == WSFS_OPCODE_TEXT) {
			if (!is_utf8_len((uint8_t *)data_ptr, data_len)) {
				if (close_code)
					*close_code = 1007;
				WSFS_ERROR_PRINTF(" ");
				return WSFS_STATUS_PROTOCOL_ERROR;
			}
		}

		if (impl->config.callbacks.on_message) {
			wsfs_message_view_t view;
			view.opcode = deliver_opcode;
			view.data = data_ptr;
			view.length = data_len;
			impl->config.callbacks.on_message(&conn->api, &view);
		}
	}

	return WSFS_STATUS_OK;
}

static wsfs_status_t wsfs_handle_control_frame(wsfs_server_impl_t *impl,
				        wsfs_connection_impl_t *conn,
				        wsfs_frame_reader_t *reader,
				        uint16_t *close_code)
{
	size_t length = (size_t)wsfs_frame_payload_length(reader);
	wsfs_buffer_t *payload = wsfs_frame_payload_buffer(reader);

	switch (reader->opcode) {
	case WSFS_OPCODE_CLOSE: {
		uint16_t code = 1000;
		if (length == 1)
			return WSFS_STATUS_PROTOCOL_ERROR;
		if (length >= 2) {
			code = (uint16_t)((payload->data[0] << 8) |
					 payload->data[1]);
			if (!wsfs_is_valid_close_code(code))
				code = 1002;
		}

		conn->close_received = 1;
		if (!conn->close_sent) {
			wsfs_status_t st = wsfs_connection_queue_close(conn, code);
			if (st != WSFS_STATUS_OK)
				return st;
		}
		conn->state = WSFS_CONN_CLOSING;
		if (close_code)
			*close_code = code;
		return WSFS_STATUS_OK;
	}
	case WSFS_OPCODE_PING:
		wsfs_connection_queue_frame(conn, WSFS_OPCODE_PONG, 1,
					 payload->data, length, 1);
		return WSFS_STATUS_OK;
	case WSFS_OPCODE_PONG:
		return WSFS_STATUS_OK;
	default:
		if (close_code)
			*close_code = 1002;
		return WSFS_STATUS_PROTOCOL_ERROR;
	}
}

static wsfs_status_t wsfs_process_frame(wsfs_server_impl_t *impl,
				    wsfs_connection_impl_t *conn,
				    uint16_t *close_code)
{
	wsfs_frame_reader_t *reader = &conn->frame_reader;
	if (wsfs_frame_is_control(reader))
		return wsfs_handle_control_frame(impl, conn, reader, close_code);
	return wsfs_handle_data_frame(impl, conn, reader, close_code);
}

static wsfs_status_t wsfs_consume_bytes(wsfs_server_impl_t *impl,
				     wsfs_connection_impl_t *conn,
				     const uint8_t *data, size_t length,
				     uint16_t *close_code)
{
	size_t offset = 0;
	while (offset < length) {
		if (wsfs_frame_reader_needs_header(&conn->frame_reader)) {
			size_t need = wsfs_frame_reader_header_bytes(
				&conn->frame_reader);
			size_t to_copy = length - offset < need ?
				length - offset
				: need;
			memcpy(wsfs_frame_reader_header_buffer(
				       &conn->frame_reader),
			       data + offset, to_copy);
			conn->frame_reader.header_count += to_copy;
			offset += to_copy;

			if (!wsfs_frame_reader_needs_header(&conn->frame_reader)) {
				wsfs_status_t st = wsfs_frame_reader_parse_header(
					&conn->frame_reader, &impl->config,
					close_code);
				if (st != WSFS_STATUS_OK)
					return st;
			}
			continue;
		}

		size_t consumed = 0;
		wsfs_status_t st = wsfs_frame_reader_consume_payload(
			&conn->frame_reader, data + offset,
			length - offset, &impl->config, &consumed,
			close_code);
		if (st != WSFS_STATUS_OK)
			return st;
		offset += consumed;

		if (conn->frame_reader.received_length ==
		    conn->frame_reader.declared_length) {
			st = wsfs_process_frame(impl, conn, close_code);
			wsfs_frame_reader_reset(&conn->frame_reader);
			if (st != WSFS_STATUS_OK)
				return st;
		}
	}

	return WSFS_STATUS_OK;
}

static void wsfs_invoke_on_close(wsfs_server_impl_t *impl,
			       wsfs_connection_impl_t *conn,
			       uint16_t close_code)
{
	if (!conn->handshake_complete)
		return;
	if (impl->config.callbacks.on_close)
		impl->config.callbacks.on_close(&conn->api, close_code);
}

static void wsfs_drop_slot(wsfs_server_impl_t *impl, size_t index,
			  uint16_t close_code, bool notify)
{
	if (index >= impl->slot_count)
		return;

	wsfs_slot_t *slot = &impl->slots[index];
	wsfs_connection_impl_t *conn = slot->conn;

	if (conn != NULL) {
		if (notify)
			wsfs_invoke_on_close(impl, conn, close_code);
		wsfs_free_connection(conn);
	}

	wsfs_slot_reset(slot);
	wsfs_refresh_fd_sets(impl);
}

static wsfs_status_t wsfs_accept_new(wsfs_server_impl_t *impl)
{
	while (1) {
		struct sockaddr_in addr;
		socklen_t addrlen = sizeof(addr);
		wsfs_socket_t client = accept(impl->listen_fd,
				      (struct sockaddr *)&addr, &addrlen);
		if (client == WSFS_INVALID_SOCKET) {
			int err = wsfs_socket_errno();
			if (err == EINTR)
				continue;
			return WSFS_STATUS_IO_ERROR;
		}

		/* Remove non-blocking mode setting - use blocking send instead */
		/* int flags = fcntl(client, F_GETFL, 0);
		if (flags == -1 || fcntl(client, F_SETFL, flags | O_NONBLOCK) == -1) {
			wsfs_socket_close(client);
			return WSFS_STATUS_OK;
		} */

		int one = 1;
		(void)setsockopt(client, IPPROTO_TCP, TCP_NODELAY, &one,
				sizeof(one));

		size_t slot_index = 0;
		for (size_t i = 1; i < impl->slot_count; ++i) {
			if (impl->slots[i].fd == WSFS_INVALID_SOCKET) {
				slot_index = i;
				break;
			}
		}

		if (slot_index == 0) {
			wsfs_socket_close(client);
			return WSFS_STATUS_OK;
		}

		wsfs_connection_impl_t *conn =
			(wsfs_connection_impl_t *)calloc(1, sizeof(*conn));
		if (conn == NULL) {
			wsfs_socket_close(client);
			return WSFS_STATUS_OK;
		}

		wsfs_status_t status = wsfs_connection_init(conn, impl, client);
		if (status != WSFS_STATUS_OK) {
			wsfs_free_connection(conn);
			wsfs_socket_close(client);
			return WSFS_STATUS_OK;
		}

		conn->slot_index = slot_index;
		if (inet_ntop(AF_INET, &addr.sin_addr,
			      conn->peer_ip, sizeof(conn->peer_ip)) == NULL)
			conn->peer_ip[0] = '\0';
		conn->peer_port = ntohs(addr.sin_port);

		impl->slots[slot_index].fd = client;
		impl->slots[slot_index].conn = conn;
		impl->slots[slot_index].wants_write = 0;
		wsfs_refresh_fd_sets(impl);

		return WSFS_STATUS_OK;
	}
}

static wsfs_status_t wsfs_handle_client_read(wsfs_server_impl_t *impl,
			     size_t index, uint16_t *close_code)
{
	wsfs_connection_impl_t *conn = impl->slots[index].conn;
	if (conn == NULL)
		return WSFS_STATUS_OK;

	ssize_t n;
	while (true) {
		n = recv(conn->fd, impl->recv_scratch,
			 impl->config.recv_buffer_size, 0);
		if (n < 0) {
			int err = wsfs_socket_errno();
			if (err == EINTR)
				continue;
			WSFS_ERROR_PRINTF(" ");
			return WSFS_STATUS_IO_ERROR;
		}
		break;
	}

	if (n == 0) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_PROTOCOL_ERROR;
	}

	if (conn->state == WSFS_CONN_HANDSHAKE) {
		bool completed = false;
		wsfs_status_t status = wsfs_handle_handshake(
			impl, conn, impl->recv_scratch, (size_t)n,
			close_code, &completed);
		if (status != WSFS_STATUS_OK) {
			if (status == WSFS_STATUS_PROTOCOL_ERROR)
				wsfs_send_http_error(conn);
			return status;
		}
		return WSFS_STATUS_OK;
	}

	return wsfs_consume_bytes(
		impl, conn, impl->recv_scratch, (size_t)n, close_code);
}

static wsfs_status_t wsfs_handle_client_write(wsfs_server_impl_t *impl,
	                                       size_t index,
	                                       uint16_t *close_code)
{
	/* Using blocking send mode, no longer need to send data via select events */
	/* All data has been sent synchronously in wsfs_connection_send_text/binary */
	wsfs_connection_impl_t *conn = impl->slots[index].conn;
	if (conn == NULL)
		return WSFS_STATUS_OK;

	/* For backward compatibility, retain queue flushing functionality (if residual data exists) */
	return wsfs_connection_flush_pending(conn, close_code);
}

wsfs_status_t wsfs_server_run(wsfs_server_t *server)
{
	if (server == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_ARGUMENT;
	}

	wsfs_server_impl_t *impl = wsfs_server_get_impl(server);
	if (impl == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_STATE;
	}

	wsfs_status_t status = wsfs_server_open_listener(impl);
	if (status != WSFS_STATUS_OK)
		return status;

	while (!impl->stop_flag) {
		fd_set read_set = impl->read_fds_master;
		fd_set write_set = impl->write_fds_master;
		struct timeval timeout;
		timeout.tv_sec = WSFS_SELECT_TIMEOUT_MS / 1000;
		timeout.tv_usec = (WSFS_SELECT_TIMEOUT_MS % 1000) * 1000;

		int max_fd = impl->max_fd;
		if (max_fd < 0) {
			WSFS_DEBUG_PRINTF("listen_fd=%d\r\n", impl->listen_fd);
			status = WSFS_STATUS_INVALID_STATE;
			break;
		}

		int ready = wsfs_socket_select(max_fd + 1, &read_set, &write_set,
					       NULL, &timeout);
		if (ready < 0) {
			int err = wsfs_socket_errno();
			if (err == EINTR)
				continue;
			WSFS_ERROR_PRINTF(" ");
			status = WSFS_STATUS_IO_ERROR;
			break;
		}

		if (ready == 0)
			continue;

		if (impl->listen_fd != WSFS_INVALID_SOCKET &&
		    FD_ISSET(impl->listen_fd, &read_set)) {
			status = wsfs_accept_new(impl);
			if (status != WSFS_STATUS_OK)
				break;
		}

		for (size_t i = 1; i < impl->slot_count; ++i) {
			wsfs_slot_t *slot = &impl->slots[i];
			int fd = slot->fd;
			if (fd == WSFS_INVALID_SOCKET)
				continue;

			int want_read = FD_ISSET(fd, &read_set);
			int want_write = FD_ISSET(fd, &write_set);
			if (!want_read && !want_write)
				continue;

			uint16_t close_code = 1006;

			if (want_read) {
				status = wsfs_handle_client_read(impl, i, &close_code);
				if (status != WSFS_STATUS_OK) {
					wsfs_drop_slot(impl, i, close_code, true);
					if (status != WSFS_STATUS_PROTOCOL_ERROR)
						continue;
					else
						continue;
				}
			}

			if (want_write) {
				status = wsfs_handle_client_write(impl, i, &close_code);
				if (status != WSFS_STATUS_OK) {
					wsfs_drop_slot(impl, i, close_code, true);
					if (status != WSFS_STATUS_PROTOCOL_ERROR)
						continue;
				}
			}
		}
	}

	wsfs_server_close_listener(impl);
	for (size_t i = 1; i < impl->slot_count; ++i)
		wsfs_drop_slot(impl, i, 1006, true);

	return status;
}

void wsfs_server_request_stop(wsfs_server_t *server)
{
	wsfs_server_impl_t *impl = wsfs_server_get_impl(server);
	if (impl != NULL)
		impl->stop_flag = 1;
}

static wsfs_status_t wsfs_validate_send_state(wsfs_connection_impl_t *conn)
{
	if (conn == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_ARGUMENT;
	}
	if (conn->state != WSFS_CONN_OPEN) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_STATE;
	}
	return WSFS_STATUS_OK;
}

wsfs_status_t wsfs_connection_send_text(wsfs_connection_t *conn_public,
				      const char *text,
				      size_t length)
{
	if (conn_public == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_ARGUMENT;
	}

	wsfs_connection_impl_t *conn = wsfs_connection_get_impl(conn_public);
	wsfs_status_t status = wsfs_validate_send_state(conn);
	if (status != WSFS_STATUS_OK)
		return status;

	if (text == NULL && length > 0) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_ARGUMENT;
	}
	if (text == NULL) {
		text = "";
		length = 0;
	} else if (length == 0) {
		length = strlen(text);
	}

	if (!is_utf8_len((uint8_t *)text, length)) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_ARGUMENT;
	}

	/* Use the new blocking send function, wait until all data is sent before returning */
	return wsfs_connection_send_frame_blocking(conn, WSFS_OPCODE_TEXT, 1,
		(const uint8_t *)text, length, 0);
}

wsfs_status_t wsfs_connection_send_binary(wsfs_connection_t *conn_public,
					 const uint8_t *payload,
					 size_t length)
{
	if (conn_public == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_ARGUMENT;
	}

	wsfs_connection_impl_t *conn = wsfs_connection_get_impl(conn_public);
	wsfs_status_t status = wsfs_validate_send_state(conn);
	if (status != WSFS_STATUS_OK)
		return status;

	if (payload == NULL && length > 0) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_ARGUMENT;
	}
	if (payload == NULL)
		length = 0;

	/* Use the new blocking send function, wait until all data is sent before returning */
	return wsfs_connection_send_frame_blocking(conn, WSFS_OPCODE_BINARY, 1,
		payload, length, 0);
}

wsfs_status_t wsfs_connection_close(wsfs_connection_t *conn_public,
				 uint16_t close_code)
{
	if (conn_public == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_ARGUMENT;
	}

	wsfs_connection_impl_t *conn = wsfs_connection_get_impl(conn_public);
	if (conn == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_ARGUMENT;
	}

	if (conn->state == WSFS_CONN_CLOSED) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_STATE;
	}

	if (conn->close_sent)
		return WSFS_STATUS_OK;

	if (!wsfs_is_valid_close_code(close_code))
		close_code = 1000;

	wsfs_status_t status = wsfs_connection_queue_close(conn, close_code);
	if (status == WSFS_STATUS_OK)
		conn->state = WSFS_CONN_CLOSING;
	return status;
}

void wsfs_connection_set_user_data(wsfs_connection_t *conn_public,
				     void *data)
{
	wsfs_connection_impl_t *conn = wsfs_connection_get_impl(conn_public);
	if (conn != NULL)
		conn->user_data = data;
}

void *wsfs_connection_get_user_data(const wsfs_connection_t *conn_public)
{
	const wsfs_connection_impl_t *conn =
		conn_public != NULL ?
		(const wsfs_connection_impl_t *)conn_public->impl
		: NULL;
	return conn != NULL ? conn->user_data : NULL;
}

const char *wsfs_connection_peer_ip(const wsfs_connection_t *conn_public)
{
	const wsfs_connection_impl_t *conn =
		conn_public != NULL ?
		(const wsfs_connection_impl_t *)conn_public->impl
		: NULL;
	return conn != NULL ? conn->peer_ip : NULL;
}

uint16_t wsfs_connection_peer_port(const wsfs_connection_t *conn_public)
{
	const wsfs_connection_impl_t *conn =
		conn_public != NULL ?
		(const wsfs_connection_impl_t *)conn_public->impl
		: NULL;
	return conn != NULL ? conn->peer_port : 0;
}
