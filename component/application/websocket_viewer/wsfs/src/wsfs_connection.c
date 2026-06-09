/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 * All rights reserved.
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "wsfs_internal.h"
#include <utf8.h>

#define WSFS_INITIAL_QUEUE_CAP 4

static void wsfs_send_queue_clear(wsfs_connection_impl_t *conn)
{
	if (conn->send_queue.frames == NULL)
		return;

	for (size_t i = 0; i < conn->send_queue.count; ++i)
		wsfs_free(conn->send_queue.frames[i].data, __func__);

	wsfs_free(conn->send_queue.frames, __func__);
	conn->send_queue.frames = NULL;
	conn->send_queue.count = 0;
	conn->send_queue.capacity = 0;
	conn->send_queue.queued_bytes = 0;
}

wsfs_status_t wsfs_connection_init(wsfs_connection_impl_t *conn,
			   wsfs_server_impl_t *server, wsfs_socket_t fd)
{
	if (conn == NULL || server == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_ARGUMENT;
	}

	memset(conn, 0, sizeof(*conn));
	conn->api.impl = conn;
	conn->fd = fd;
	conn->state = WSFS_CONN_HANDSHAKE;
	conn->server = server;
	conn->slot_index = 0;
	conn->peer_port = 0;
	conn->user_data = NULL;

	wsfs_status_t status =
		wsfs_frame_reader_init(&conn->frame_reader, 0);
	if (status != WSFS_STATUS_OK)
		return status;

	return WSFS_STATUS_OK;
}

void wsfs_connection_deinit(wsfs_connection_impl_t *conn)
{
	if (conn == NULL)
		return;

	wsfs_free(conn->handshake_buffer.data, __func__);
	wsfs_free(conn->frame_reader.payload.data, __func__);
	wsfs_free(conn->message_buffer.data, __func__);

	wsfs_send_queue_clear(conn);

	if (conn->fd != WSFS_INVALID_SOCKET)
		wsfs_socket_close(conn->fd);

	memset(conn, 0, sizeof(*conn));
}

static wsfs_status_t wsfs_send_queue_reserve(wsfs_connection_impl_t *conn,
					     size_t new_capacity)
{
	if (conn->send_queue.capacity >= new_capacity)
		return WSFS_STATUS_OK;

	wsfs_send_frame_t *frames =
		(wsfs_send_frame_t *)wsfs_realloc(
			conn->send_queue.frames,
			new_capacity * sizeof(wsfs_send_frame_t), __func__);
	if (frames == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_ALLOCATION_FAILED;
	}

	conn->send_queue.frames = frames;
	conn->send_queue.capacity = new_capacity;
	return WSFS_STATUS_OK;
}

wsfs_status_t wsfs_enqueue_raw(wsfs_connection_impl_t *conn,
		       const uint8_t *data, size_t length)
{
	if (conn == NULL || conn->server == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_ARGUMENT;
	}
	if (length == 0)
		return WSFS_STATUS_OK;

	size_t new_total = conn->send_queue.queued_bytes + length;
	size_t cap = WSFS_DEFAULT_SEND_QUEUE_CAP;
	printf("new_total=%u, cap=%u, queued_bytes=%u, new_length=%u\r\n", 
		   new_total, cap, conn->send_queue.queued_bytes, length);
	if (new_total > cap) {
		printf("Queue full: %s:%d, queued_bytes=%u, trying to add=%u, cap=%u\r\n", 
			   __FUNCTION__, __LINE__, conn->send_queue.queued_bytes, length, cap);
		
		// Try to send some existing data first
		if (conn->send_queue.count > 0) {
			printf("Attempting to flush pending data before rejecting new data\r\n");
			wsfs_connection_flush_pending(conn, NULL);
			
			// Recalculate to check if there is space
			new_total = conn->send_queue.queued_bytes + length;
			if (new_total <= cap) {
				printf("After flush: queued_bytes=%u, can now accept new data\r\n", 
					   conn->send_queue.queued_bytes);
				// Continue processing
			} else {
				printf("Still no space after flush, rejecting new data\r\n");
				return WSFS_STATUS_PROTOCOL_ERROR;
			}
		} else {
			// Queue is empty but still exceeds limit, this should not happen
			printf("Empty queue but new data exceeds cap, rejecting\r\n");
			return WSFS_STATUS_PROTOCOL_ERROR;
		}
	}

	if (conn->send_queue.count == conn->send_queue.capacity) {
		size_t next_capacity = conn->send_queue.capacity == 0
			         ? WSFS_INITIAL_QUEUE_CAP
			         : conn->send_queue.capacity * 2;
		wsfs_status_t reserve =
			wsfs_send_queue_reserve(conn, next_capacity);
		if (reserve != WSFS_STATUS_OK)
			return reserve;
	}

	int flush_immediately = conn->send_queue.count == 0;

	uint8_t *frame = (uint8_t *)malloc(length);
	if (frame == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_ALLOCATION_FAILED;
	}

	memcpy(frame, data, length);

	size_t index = conn->send_queue.count++;
	conn->send_queue.frames[index].data = frame;
	conn->send_queue.frames[index].length = length;
	conn->send_queue.frames[index].offset = 0;
	conn->send_queue.queued_bytes += length;

	printf("wsfs_enqueue_raw: added frame[%u], length=%u, offset=0, queued_bytes=%u (after add)\r\n",
		   index, length, conn->send_queue.queued_bytes);

	wsfs_update_poll_events(conn->server, conn);

	if (flush_immediately && conn->state == WSFS_CONN_OPEN &&
	    conn->fd != WSFS_INVALID_SOCKET) {
		wsfs_status_t flush_status =
			wsfs_connection_flush_pending(conn, NULL);
		if (flush_status != WSFS_STATUS_OK &&
		    flush_status != WSFS_STATUS_PROTOCOL_ERROR)
			return flush_status;
		/* Flush may report WSFS_STATUS_PROTOCOL_ERROR if a close handshake
		 * completes; treat that as success for the caller. */
	}
	return WSFS_STATUS_OK;
}

void wsfs_update_poll_events(wsfs_server_impl_t *server,
	     wsfs_connection_impl_t *conn)
{
	if (server == NULL || conn == NULL)
		return;
	if (conn->slot_index >= server->slot_count)
		return;

	wsfs_slot_t *slot = &server->slots[conn->slot_index];
	slot->wants_write = conn->send_queue.count > 0 ? 1 : 0;
	wsfs_refresh_fd_sets(server);
}

wsfs_status_t wsfs_connection_flush_pending(wsfs_connection_impl_t *conn,
				  uint16_t *close_code)
{
	if (conn == NULL || conn->server == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_ARGUMENT;
	}
	if (conn->fd == WSFS_INVALID_SOCKET)
		return WSFS_STATUS_OK;

	while (conn->send_queue.count > 0) {
		wsfs_send_frame_t *frame = &conn->send_queue.frames[0];
		
		// Guard check: ensure offset does not exceed length
		if (frame->offset >= frame->length) {
			printf("ERROR: frame->offset(%u) >= frame->length(%u), removing corrupted frame\r\n",
				   frame->offset, frame->length);
			
			// Adjust queued_bytes: subtract the remaining bytes that should have been sent for this frame
			if (frame->length > frame->offset) {
				size_t remaining_bytes = frame->length - frame->offset;
				if (remaining_bytes <= conn->send_queue.queued_bytes) {
					conn->send_queue.queued_bytes -= remaining_bytes;
				} else {
					conn->send_queue.queued_bytes = 0;
				}
			}
			
			wsfs_free(frame->data, __func__);
			if (conn->send_queue.count > 1) {
				memmove(conn->send_queue.frames,
					conn->send_queue.frames + 1,
					(--conn->send_queue.count) *
					 sizeof(wsfs_send_frame_t));
			} else {
				conn->send_queue.count = 0;
			}
			continue;
		}
		
		size_t remaining = frame->length - frame->offset;
		ssize_t sent = send(conn->fd, frame->data + frame->offset,
				 remaining, MSG_NOSIGNAL);
		if (sent < 0) {
			int err = wsfs_socket_errno();
			if (err == EINTR)
				continue;
			if (err == EAGAIN || err == EWOULDBLOCK)
				break;
			printf("%s:%d: send() err: %d\r\n", __FUNCTION__, __LINE__, err);
			return WSFS_STATUS_IO_ERROR;
		}
		if (sent == 0) {
			printf("send() returned 0, socket may be congested\r\n");
			break;
		}

		printf("sent=%u, remaining=%u, queued_bytes=%u\r\n", 
			   sent, remaining, conn->send_queue.queued_bytes);

		// Guard check: sent should not exceed remaining
		if ((size_t)sent > remaining) {
			printf("ERROR: sent(%u) > remaining(%u), data corruption!\r\n",
				   sent, remaining);
			// Force set to remaining to prevent offset from exceeding length
			sent = (ssize_t)remaining;
		}

		frame->offset += (size_t)sent;
		
		// Guard check: ensure we don't subtract more than queued_bytes
		if ((size_t)sent <= conn->send_queue.queued_bytes) {
			conn->send_queue.queued_bytes -= (size_t)sent;
		} else {
			printf("Warning: sent(%u) > queued_bytes(%u), setting queued_bytes to 0\r\n", 
				   sent, conn->send_queue.queued_bytes);
			conn->send_queue.queued_bytes = 0;
		}

		if (frame->offset == frame->length) {
			wsfs_free(frame->data, __func__);
			if (conn->send_queue.count > 1) {
				memmove(conn->send_queue.frames,
					conn->send_queue.frames + 1,
					(--conn->send_queue.count) *
					 sizeof(wsfs_send_frame_t));
			} else {
				conn->send_queue.count = 0;
			}
		} else {
			break;
		}
	}

	wsfs_update_poll_events(conn->server, conn);

	if (conn->state == WSFS_CONN_CLOSING && conn->send_queue.count == 0) {
		conn->state = WSFS_CONN_CLOSED;
		if (close_code != NULL)
			*close_code = conn->close_received ? 1000 : 1006;
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_PROTOCOL_ERROR;
	}

	return WSFS_STATUS_OK;
}

static wsfs_status_t wsfs_send_frame_blocking(wsfs_connection_impl_t *conn,
					       wsfs_opcode_t opcode, uint8_t fin,
					       const uint8_t *payload, size_t length,
					       int is_control)
{
	if (conn == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_ARGUMENT;
	}

	if (conn->fd == WSFS_INVALID_SOCKET) {
		WSFS_ERROR_PRINTF("Invalid socket\r\n");
		return WSFS_STATUS_IO_ERROR;
	}

	if (is_control && length > 125) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_PROTOCOL_ERROR;
	}

	/* Calculate WebSocket frame header size */
	size_t header_size = 2;
	uint8_t len_code;
	if (length <= 125) {
		len_code = (uint8_t)length;
	} else if (length <= 0xFFFF) {
		header_size += 2;
		len_code = 126;
	} else {
		header_size += 8;
		len_code = 127;
	}

	/* Build WebSocket frame */
	size_t total = header_size + length;
	uint8_t *frame = (uint8_t *)malloc(total);
	if (frame == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_ALLOCATION_FAILED;
	}

	//printf("wsfs_send_frame_blocking: opcode=%d, fin=%d, payload_len=%u, header=%u, total=%u\r\n",
	//	   opcode, fin, length, header_size, total);

	/* Build frame header */
	frame[0] = (uint8_t)((fin ? 0x80 : 0x00) | (opcode & 0x0F));
	frame[1] = len_code;
	size_t offset = 2;
	if (len_code == 126) {
		frame[offset++] = (uint8_t)((length >> 8) & 0xFF);
		frame[offset++] = (uint8_t)(length & 0xFF);
	} else if (len_code == 127) {
		uint64_t value = (uint64_t)length;
		for (int i = 7; i >= 0; --i)
			frame[offset++] = (uint8_t)((value >> (i * 8)) & 0xFF);
	}

	/* duplicate payload */
	if (length > 0 && payload != NULL)
		memcpy(frame + offset, payload, length);

	/* Blocking send: loop until all data is sent */
	size_t total_sent = 0;
	while (total_sent < total) {
		ssize_t sent = send(conn->fd, frame + total_sent, total - total_sent, 0);
		if (sent < 0) {
			int err = wsfs_socket_errno();
			if (err == EINTR) {
				/* Interrupted by signal, retry */
				continue;
			}
			printf("send() error: %d, total_sent=%u/%u\r\n", err, total_sent, total);
			free(frame);
			return WSFS_STATUS_IO_ERROR;
		}
		if (sent == 0) {
			/* Connection closed */
			printf("send() returned 0, connection closed, total_sent=%u/%u\r\n", 
				   total_sent, total);
			free(frame);
			return WSFS_STATUS_IO_ERROR;
		}
		
		total_sent += (size_t)sent;
		//printf("send() sent=%d, total_sent=%u/%u\r\n", sent, total_sent, total);
	}

	free(frame);
	//printf("wsfs_send_frame_blocking: completed, sent %u bytes\r\n", total_sent);
	return WSFS_STATUS_OK;
}


static wsfs_status_t wsfs_build_frame(wsfs_connection_impl_t *conn,
				       wsfs_opcode_t opcode, uint8_t fin,
				       const uint8_t *payload, size_t length,
				       int is_control)
{
	if (conn == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_ARGUMENT;
	}

	if (is_control && length > 125) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_PROTOCOL_ERROR;
	}

	size_t header = 2;
	uint8_t len_code;
	if (length <= 125) {
		len_code = (uint8_t)length;
	} else if (length <= 0xFFFF) {
		header += 2;
		len_code = 126;
	} else {
		header += 8;
		len_code = 127;
	}

	size_t total = header + length;
	uint8_t *frame = (uint8_t *)malloc(total);
	if (frame == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_ALLOCATION_FAILED;
	}

	printf("wsfs_build_frame: opcode=%d, fin=%d, payload_len=%u, header=%u, total=%u\r\n",
		   opcode, fin, length, header, total);

	frame[0] = (uint8_t)((fin ? 0x80 : 0x00) | (opcode & 0x0F));
	frame[1] = len_code;
	size_t offset = 2;
	if (len_code == 126) {
		frame[offset++] = (uint8_t)((length >> 8) & 0xFF);
		frame[offset++] = (uint8_t)(length & 0xFF);
	} else if (len_code == 127) {
		uint64_t value = (uint64_t)length;
		for (int i = 7; i >= 0; --i)
			frame[offset++] = (uint8_t)((value >> (i * 8)) & 0xFF);
	}

	if (length > 0 && payload != NULL)
		memcpy(frame + offset, payload, length);

	do {
		wsfs_status_t status = wsfs_enqueue_raw(conn, frame, total);
		if (status != WSFS_STATUS_OK) {
			wsfs_free(frame, __func__);
			return status;
		}
	} while (0);

	wsfs_free(frame, __func__);
	return WSFS_STATUS_OK;
}

wsfs_status_t wsfs_connection_queue_frame(wsfs_connection_impl_t *conn,
					  wsfs_opcode_t opcode, uint8_t fin,
					  const uint8_t *payload, size_t length,
					  int is_control)
{
	return wsfs_build_frame(conn, opcode, fin, payload, length, is_control);
}

wsfs_status_t wsfs_connection_send_frame_blocking(wsfs_connection_impl_t *conn,
						   wsfs_opcode_t opcode, uint8_t fin,
						   const uint8_t *payload, size_t length,
						   int is_control)
{
	return wsfs_send_frame_blocking(conn, opcode, fin, payload, length, is_control);
}

wsfs_status_t wsfs_connection_queue_close(wsfs_connection_impl_t *conn,
					  uint16_t close_code)
{
	uint8_t payload[2];
	size_t length = 0;
	if (close_code != 0) {
		payload[0] = (uint8_t)((close_code >> 8) & 0xFF);
		payload[1] = (uint8_t)(close_code & 0xFF);
		length = 2;
	}

	wsfs_status_t status =
		wsfs_connection_queue_frame(conn, WSFS_OPCODE_CLOSE, 1,
					payload, length, 1);
	if (status == WSFS_STATUS_OK) {
		conn->close_sent = 1;
		if (conn->state != WSFS_CONN_CLOSING &&
		    conn->state != WSFS_CONN_CLOSED)
			conn->state = WSFS_CONN_CLOSING;
	}

	return status;
}
