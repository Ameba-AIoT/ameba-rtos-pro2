/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 * All rights reserved.
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "wsfs_internal.h"

static size_t wsfs_frame_header_required(wsfs_frame_reader_t *reader)
{
	if (reader->header_count < 2)
		return 2;

	if (reader->header_required > reader->header_count)
		return reader->header_required;

	uint8_t b1 = reader->header[1];
	size_t required = 2;
	uint8_t len_code = (uint8_t)(b1 & 0x7F);
	if (len_code == 126) {
		required += 2;
	} else if (len_code == 127) {
		required += 8;
	}
	if (b1 & 0x80)
		required += 4;

	reader->header_required = required;
	return required;
}

static wsfs_status_t wsfs_buffer_reserve(wsfs_buffer_t *buffer,
					 size_t capacity)
{
	if (buffer->capacity >= capacity)
		return WSFS_STATUS_OK;

	uint8_t *next = wsfs_realloc(buffer->data,
					     capacity * sizeof(uint8_t), __func__);
	if (next == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_ALLOCATION_FAILED;
	}

	buffer->data = next;
	buffer->capacity = capacity;
	return WSFS_STATUS_OK;
}

wsfs_status_t wsfs_frame_reader_init(wsfs_frame_reader_t *reader,
				     size_t initial_capacity)
{
	if (reader == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_ARGUMENT;
	}

	memset(reader, 0, sizeof(*reader));
	reader->header_required = 2;

	if (initial_capacity > 0) {
		reader->payload.data = wsfs_realloc(
			NULL, initial_capacity * sizeof(uint8_t), __func__);
		if (reader->payload.data == NULL) {
			WSFS_ERROR_PRINTF(" ");
			return WSFS_STATUS_ALLOCATION_FAILED;
		}
		reader->payload.capacity = initial_capacity;
	}

	return WSFS_STATUS_OK;
}

void wsfs_frame_reader_reset(wsfs_frame_reader_t *reader)
{
	if (reader == NULL)
		return;

	reader->header_count = 0;
	reader->header_required = 2;
	reader->fin = 0;
	reader->opcode = WSFS_OPCODE_CONTINUATION;
	reader->masked = 0;
	reader->declared_length = 0;
	reader->received_length = 0;
	reader->payload.length = 0;
}

int wsfs_frame_reader_needs_header(wsfs_frame_reader_t *reader)
{
	if (reader == NULL)
		return 1;

	return reader->header_count < wsfs_frame_header_required(reader);
}

size_t wsfs_frame_reader_header_bytes(wsfs_frame_reader_t *reader)
{
	size_t required = wsfs_frame_header_required(reader);
	if (reader->header_count >= required)
		return 0;
	return required - reader->header_count;
}

uint8_t *wsfs_frame_reader_header_buffer(wsfs_frame_reader_t *reader)
{
	return reader->header + reader->header_count;
}

static uint64_t wsfs_read_u64(const uint8_t *p)
{
	uint64_t value = 0;
	for (int i = 0; i < 8; ++i)
		value = (value << 8) | (uint64_t)p[i];
	return value;
}

static uint16_t wsfs_read_u16(const uint8_t *p)
{
	return (uint16_t)((p[0] << 8) | p[1]);
}

wsfs_status_t wsfs_frame_reader_parse_header(wsfs_frame_reader_t *reader,
					     const wsfs_server_config_t *cfg,
					     uint16_t *close_code)
{
	if (reader == NULL || cfg == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_ARGUMENT;
	}

	size_t required = wsfs_frame_header_required(reader);
	if (reader->header_count < required) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_STATE;
	}

	uint8_t b0 = reader->header[0];
	uint8_t b1 = reader->header[1];

	reader->fin = (uint8_t)((b0 >> 7) & 0x01);

	if ((b0 & 0x70) != 0) {
		if (close_code)
			*close_code = 1002;
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_PROTOCOL_ERROR;
	}

	reader->opcode = (wsfs_opcode_t)(b0 & 0x0F);
	reader->masked = (uint8_t)((b1 >> 7) & 0x01);

	if (!reader->masked) {
		if (close_code)
			*close_code = 1002;
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_PROTOCOL_ERROR;
	}

	uint64_t declared = (uint64_t)(b1 & 0x7F);
	size_t offset = 2;
	if (declared == 126) {
		declared = wsfs_read_u16(reader->header + offset);
		offset += 2;
	} else if (declared == 127) {
		declared = wsfs_read_u64(reader->header + offset);
		offset += 8;
		if (declared > ((uint64_t)1 << 63)) {
			if (close_code)
				*close_code = 1009;
			WSFS_ERROR_PRINTF(" ");
			return WSFS_STATUS_PROTOCOL_ERROR;
		}
	}

	for (int i = 0; i < 4; ++i)
		reader->mask[i] = reader->header[offset + i];

	reader->declared_length = declared;
	reader->received_length = 0;
	reader->payload.length = 0;

	if (declared > cfg->max_frame_size) {
		if (close_code)
			*close_code = 1009;
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_PROTOCOL_ERROR;
	}

	wsfs_status_t reserve =
		wsfs_buffer_reserve(&reader->payload, (size_t)declared);
	if (reserve != WSFS_STATUS_OK) {
		if (close_code)
			*close_code = 1002;
		WSFS_ERROR_PRINTF(" ");
		return reserve;
	}

	return WSFS_STATUS_OK;
}

wsfs_status_t wsfs_frame_reader_consume_payload(wsfs_frame_reader_t *reader,
						const uint8_t *data,
						size_t length,
						const wsfs_server_config_t *cfg,
						size_t *consumed,
						uint16_t *close_code)
{
	(void)cfg;

	if (reader == NULL || data == NULL) {
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_INVALID_ARGUMENT;
	}

	size_t remaining = 0;
	if (reader->declared_length >= reader->received_length)
		remaining = (size_t)(reader->declared_length -
				     reader->received_length);

	size_t to_copy = length < remaining ? length : remaining;
	if (consumed != NULL)
		*consumed = to_copy;

	uint8_t *dest =
		reader->payload.data + reader->received_length;

	for (size_t i = 0; i < to_copy; ++i) {
		uint8_t byte = data[i];
		byte ^= reader->mask[(reader->received_length + i) % 4];
		dest[i] = byte;
	}

	reader->received_length += to_copy;
	reader->payload.length = (size_t)reader->received_length;

	if (reader->received_length > reader->declared_length) {
		if (close_code)
			*close_code = 1002;
		WSFS_ERROR_PRINTF(" ");
		return WSFS_STATUS_PROTOCOL_ERROR;
	}

	return WSFS_STATUS_OK;
}

int wsfs_frame_is_control(const wsfs_frame_reader_t *reader)
{
	if (reader == NULL)
		return 0;
	return ((uint8_t)reader->opcode & 0x08) != 0;
}

int wsfs_frame_is_fin(const wsfs_frame_reader_t *reader)
{
	if (reader == NULL)
		return 0;
	return reader->fin != 0;
}

uint64_t wsfs_frame_payload_length(const wsfs_frame_reader_t *reader)
{
	return reader != NULL ? reader->received_length : 0;
}

wsfs_buffer_t *wsfs_frame_payload_buffer(wsfs_frame_reader_t *reader)
{
	return reader != NULL ? &reader->payload : NULL;
}
