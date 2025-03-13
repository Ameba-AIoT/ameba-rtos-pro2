#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "mmf2_mediatime_8735b.h"
#include "sliding_windows.h"
#include "ai_glass_dbg.h"

typedef void (*FreeItemFn)(void *);
typedef void *(*InitItemFn)(void);

// Function to determine if a windows is full
#define IS_VALID_WINDOW_SIZE(value) \
    ((value) >= MIN_WINDOW_SIZE && \
     (value) <= MAX_WINDOW_SIZE)

#define SEQ_NUM_WRAPAROUND 0xFFFFFFFF

// Function to determine if a sequence number is in range considering overflow
static bool is_seq_in_range(uint32_t seq, uint32_t base, uint32_t next)
{
	if (base <= next) {
		return (seq > base && seq <= next);
	} else {  // handle wrap-around scenario
		return (seq > base || seq <= next);
	}
}

// Function to determine if a windows is full
static bool is_window_full(uint32_t send_base, uint32_t next_seq, uint16_t window_size)
{
	uint32_t effective_size = (next_seq >= send_base) ? (next_seq - send_base) : (SEQ_NUM_WRAPAROUND - send_base + next_seq + 1);
	return effective_size >= window_size;
}

static QueueHandle_t create_queue_and_fill(int length, size_t item_size, void **item_array, InitItemFn init_item_fn, FreeItemFn free_item_fn)
{
	QueueHandle_t queue = xQueueCreate(length, item_size);
	if (queue == NULL) {
		return NULL;
	}

	for (int i = 0; i < length; i++) {
		item_array[i] = init_item_fn();
		if (!item_array[i]) {
			for (int j = 0; j < i; j++) {
				free_item_fn(item_array[j]);
			}
			vQueueDelete(queue);
			return NULL;
		}
		if (xQueueSend(queue, &item_array[i], 0) != pdPASS) {
			// Free already allocated items before returning NULL
			for (int j = 0; j <= i; j++) {
				free_item_fn(item_array[j]);
			}
			vQueueDelete(queue);
			return NULL;
		}
	}
	return queue;
}

static void free_win_item(void *item)
{
	payload_info_t *win_pakcet = (payload_info_t *)item;
	if (win_pakcet) {
		if (win_pakcet->payload) {
			free(win_pakcet->payload);
			win_pakcet->payload = NULL;
		}
		free(win_pakcet);
	}
}

static void delete_queue_if_exists(QueueHandle_t queue, FreeItemFn free_item_fn)
{
	void *item;
	if (queue) {
		while (xQueueReceive(queue, &item, 0) == pdPASS) {
			if (free_item_fn) {
				free_item_fn(item);
			}
		}
		vQueueDelete(queue);
	}
}

static uint32_t get_current_time_ms(void)
{
	return mm_read_mediatime_ms();
}

static void *init_win_item(void)
{
	payload_info_t *win_pakcet = malloc(sizeof(payload_info_t));
	if (win_pakcet) {
		memset(win_pakcet, 0, sizeof(payload_info_t));
	}
	return win_pakcet;
}

static int resend_packet_with_logging(SlidingWindow *window, const window_item_t *item)
{
	// Log the resend operation
	SLIDING_WIN_MSG("Resending packet Seq=%u, Retry count=%u\n", item->packet.seq, item->retry_count);

	// Implement the actual transmission logic
	return window->send_packet(&item->packet);
}

static void send_packet_thread(void *param)
{
	SlidingWindow *window = (SlidingWindow *)param;
	const int retransmit_count = 3; // User-defined number of packets to retransmit

	while (1) {
		if (xSemaphoreTake(window->lock, portMAX_DELAY) == pdTRUE) {
			uint32_t current_time = get_current_time_ms();

			// Handle duplicate ACK-based retransmission
			if (window->duplicate_ack_count >= MAX_DUPLICATE_ACKS) {
				SLIDING_WIN_MSG("Triggered retransmission for duplicate ACK: %u\r\n", window->last_ack);

				for (uint32_t i = 0; i < retransmit_count; ++i) {
					uint32_t index = (window->last_ack + i) % MAX_WINDOW_SIZE;
					window_item_t *item = &window->items[index];

					if (item->packet.spayload && !item->acked) {
						SLIDING_WIN_MSG("Retransmitting packet Seq=%u\r\n", item->packet.seq);
						resend_packet_with_logging(window, item);
						item->timestamp = get_current_time_ms();
					}
				}

				window->duplicate_ack_count = 0; // Reset counter after retransmission
			}

			// Handle timeout-based retransmission
			for (uint32_t i = 0; i < MAX_WINDOW_SIZE; ++i) {
				uint32_t resend_index = (i + window->send_base) % MAX_WINDOW_SIZE;
				window_item_t *item = &window->items[resend_index];

				if (!item->acked && item->packet.spayload) {
					if (current_time - item->timestamp >= TIMEOUT_PERIOD) {
						if (item->retry_count < MAX_RETRIES) {
							item->retry_count++;
							resend_packet_with_logging(window, item);
							item->timestamp = get_current_time_ms();
						} else {
							SLIDING_WIN_MSG("Packet Seq=%u failed after %d retries\r\n", item->packet.seq, MAX_RETRIES);
							item->acked = true;
							payload_info_t *sended_payload = item->packet.spayload;
							xQueueSend(window->payload_recycle_queue, &sended_payload, 0);
							item->packet.spayload = NULL;
						}
					}
				}
			}

			// Send new packets if window is not full
			if (!is_window_full(window->send_base, window->next_seq, window->window_size)) {
				payload_info_t *new_payload = NULL;
				if (xQueueReceive(window->payload_ready_queue, &new_payload, 0) == pdTRUE) {
					uint32_t index = window->next_seq % MAX_WINDOW_SIZE;
					uint32_t send_time = get_current_time_ms();
					window_item_t *item = &window->items[index];
					item->packet.seq = window->next_seq;
					item->packet.spayload = new_payload;
					item->retry_count = 0;
					item->acked = false;

					window->send_packet(&item->packet);
					item->timestamp = get_current_time_ms();
					window->next_seq = (window->next_seq + 1) % SEQ_NUM_WRAPAROUND; // to prevent
				}
			}

			xSemaphoreGive(window->lock);
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

static int setup_ack_extend(ack_info_t *ack_info, const uint8_t *extend_data)
{
	if (extend_data == NULL || ack_info->extend_length == 0) {
		ack_info->extend_length = 0;
		return 0;
	}

	ack_info->extend_data = (uint8_t *)malloc(ack_info->extend_length * sizeof(uint8_t));

	if (ack_info->extend_data == NULL) {
		return -1;
	}
	memcpy(ack_info->extend_data, extend_data, ack_info->extend_length);
	return 0;
}

static void free_ack_extend(ack_info_t *ack_info)
{
	uint8_t *extend = (uint8_t *)(ack_info->extend_data);
	if (extend) {
		free(extend);
		extend = NULL;
	}
}

static void recycle_window_item(SlidingWindow *window, uint32_t start_seq, uint32_t exp_seq)
{
	for (uint32_t j = start_seq; j != exp_seq; j = (j + 1) % SEQ_NUM_WRAPAROUND) {
		uint32_t idx = j % MAX_WINDOW_SIZE;
		if (!window->items[idx].acked && window->items[idx].packet.spayload && window->items[idx].packet.seq == j) {
			window->items[idx].acked = true;
			window->items[idx].retry_count = 0;
			payload_info_t *sended_payload = window->items[idx].packet.spayload;
			xQueueSend(window->payload_recycle_queue, &sended_payload, 0);
			window->items[idx].packet.spayload = NULL;
			SLIDING_WIN_MSG("ACK received for Seq=%u; Packet recycled.\r\n", window->items[idx].packet.seq);
		}
	}
}

static void process_received_ack(SlidingWindow *window, ack_info_t *ack_info)
{
	if (xSemaphoreTake(window->lock, portMAX_DELAY) == pdTRUE) {
		// check the ack range
		if (is_seq_in_range(ack_info->ack_seq, window->send_base, window->next_seq)) {
			SLIDING_WIN_MSG("Valid ACK received. Updating send_base from %u to %u.\r\n", window->send_base, ack_info->ack_seq);
			sliding_update_window_size(window, ack_info->window_size);
			recycle_window_item(window, window->send_base, ack_info->ack_seq);

			uint32_t extend_box_count = 0;
			uint32_t ext_ptr = 0;
			// If the length is smaller or equal to ID box size, the extend data is the end or broken
			while (ext_ptr + 4 < ack_info->extend_length && extend_box_count < MAX_EXTEND_BOX_NUM) {
				uint32_t extend_id = ack_info->extend_data[ext_ptr] | (ack_info->extend_data[ext_ptr + 1] << 8) | (ack_info->extend_data[ext_ptr + 2] << 16) |
									 (ack_info->extend_data[ext_ptr + 3] << 24);
				ext_ptr += 4;

				switch (extend_id) {
				case SLIDING_EXT_ID_SACK: {
					if (ext_ptr + sizeof(sack_data_t) > ack_info->extend_length) {
						goto endofext;
					}
					sack_data_t *sack_info = (sack_data_t *)(ack_info->extend_data + ext_ptr);
					recycle_window_item(window, sack_info->start_seq, sack_info->exp_seq);
					ext_ptr += sizeof(sack_data_t);
				}
				break;
				}
				extend_box_count += 1;
			}

endofext:
			window->send_base = ack_info->ack_seq;
			if (window->last_ack != ack_info->ack_seq) {
				window->last_ack = ack_info->ack_seq;
				window->duplicate_ack_count = 0;
			} else {
				window->duplicate_ack_count ++;
			}
		} else {
			SLIDING_WIN_MSG("Received ACK %u is out of range (send_base: %u, next_seq: %u)\r\n", ack_info->ack_seq, window->send_base, window->next_seq);
		}

		xSemaphoreGive(window->lock);
	}
}

static void ack_queue_thread(void *param)
{
	SlidingWindow *window = (SlidingWindow *)param;
	ack_info_t received_info = {0};

	while (1) {
		if (xQueueReceive(window->ack_queue, &received_info, portMAX_DELAY) == pdTRUE) {
			process_received_ack(window, &received_info);
			free_ack_extend(&received_info);
		}
	}
}

static int enqueue_payload(SlidingWindow *window, const payload_info_t *payload, int timeout)
{
	if (window) {
		// Avoid memory allocation failure
		payload_info_t *new_payload = NULL;

		if (xQueueReceive(window->payload_recycle_queue, &new_payload, timeout) == pdTRUE) {
			new_payload->length = payload->length;
			new_payload->label = payload->label;
			memcpy(new_payload->payload, payload->payload, payload->length);

			if (xQueueSend(window->payload_ready_queue, &new_payload, 0) != pdTRUE) {
				SLIDING_WIN_ERR("Failed to queue payload for sending\r\n");
				return -1;
			}
		} else {
			SLIDING_WIN_ERR("Failed to obtain a recycled payload slot\r\n");
			return -1;
		}

		return 0;
	}
	return -1;
}

void sliding_on_ack_received(SlidingWindow *window, ack_info_t *ack_info, const uint8_t *extend_data)
{
	ack_info_t send_info = {0};
	if (window) {
		memcpy(&send_info, ack_info, sizeof(ack_info_t));
		setup_ack_extend(&send_info, extend_data);
		BaseType_t queueStatus = xQueueSendToFront(window->ack_queue, &send_info, 0);
		if (queueStatus != pdPASS) {
			// Handle queue overflow (e.g., log the error, increase queue size, or discard newest data)
			free_ack_extend(&send_info);
		}
	}
}

int sliding_send_data(SlidingWindow *window, uint8_t *data, uint16_t data_length, bool data_continue, int timeout)
{
	if (!window) {
		return -1;
	}

	if (data == NULL || data_length == 0) {
		return 0;
	}

	uint16_t data_offset = 0;
	uint32_t data_count = 0;
	payload_info_t spayload = {0};

	while (data_offset < data_length) {
		spayload.payload = data + data_offset;
		uint32_t seg_payload_size = window->seg_payload_size;
		spayload.label = 0x00;
		SLIDING_WIN_INFO("data_count = %u\r\n", data_count);
		data_count ++;
		if (data_length - data_offset > seg_payload_size) {
			spayload.length = seg_payload_size;
		} else {
			spayload.length = data_length - data_offset;
			if (!data_continue) {
				spayload.label |= PACKET_LABEL_EOF;
				SLIDING_WIN_INFO("set up label\r\n");
			}
		}
		data_offset += spayload.length;

		if (enqueue_payload(window, &spayload, timeout) != 0) {
			SLIDING_WIN_ERR("Failed to enqueue payload for window with sequence starting at %u\n", window->send_base);
			return -1;
		}
	}

	return 0;
}

void delete_sliding_window(SlidingWindow *window)
{
	if (!window) {
		return;
	}

	if (xSemaphoreTake(window->lock, portMAX_DELAY) == pdTRUE) {
		if (window->sendtask) {
			vTaskDelete(window->sendtask);
			window->sendtask = NULL;
		}
		if (window->recvtask) {
			vTaskDelete(window->recvtask);
			window->recvtask = NULL;
		}
		// Free payload memory for all item
		for (uint32_t i = 0; i < MAX_WINDOW_SIZE; ++i) {
			window_item_t *item = &window->items[i];
			if (item->packet.spayload->payload) {
				payload_info_t *sended_payload = item->packet.spayload;
				xQueueSend(window->payload_recycle_queue, &sended_payload, 0);
				item->packet.spayload = NULL;
			}
		}
		if (window->payload_ready_queue) {
			delete_queue_if_exists(window->payload_ready_queue, free_win_item);
			window->payload_ready_queue = NULL;
		}
		if (window->payload_recycle_queue) {
			delete_queue_if_exists(window->payload_recycle_queue, free_win_item);
			window->payload_recycle_queue = NULL;
		}
		if (window->ack_queue) {
			vQueueDelete(window->ack_queue);
			window->ack_queue = NULL;
		}
		vSemaphoreDelete(window->lock);
		window->lock = NULL;
	}

	free(window);
}

void sliding_update_window_size(SlidingWindow *window, uint16_t window_size)
{
	if (window->window_size != window_size) {
		if (!IS_VALID_WINDOW_SIZE(window_size)) {
			SLIDING_WIN_INFO("set to maximum window size %u\r\n", MAX_WINDOW_SIZE);
			window->window_size = MAX_WINDOW_SIZE;
		} else {
			SLIDING_WIN_INFO("update window size %u\r\n", window_size);
			window->window_size = window_size;
		}
	}
}

SlidingWindow *create_sliding_window(int (*send_packet)(const sliding_pkt_t *), uint16_t window_size, uint32_t start_seq, uint32_t max_payload_size)
{
	SlidingWindow *window = malloc(sizeof(SlidingWindow));
	if (!window) {
		return NULL;
	}

	memset(window, 0, sizeof(SlidingWindow));
	if (send_packet == NULL) {
		free(window);
		return NULL;
	}
	window->next_seq = start_seq;
	window->send_base = window->next_seq;
	window->send_packet = send_packet;
	window->max_payload_size = max_payload_size;
	window->seg_payload_size = window->max_payload_size;

	sliding_update_window_size(window, window_size);

	window->ack_queue = xQueueCreate(ACK_QUEUE_LENGTH, sizeof(ack_info_t));
	if (!window->ack_queue) {
		delete_sliding_window(window);
		return NULL;
	}
	payload_info_t *win_item[PAYLOAD_QUEUE_LENGTH];
	window->payload_ready_queue = create_queue_and_fill(PAYLOAD_QUEUE_LENGTH, sizeof(payload_info_t *), (void **)win_item, init_win_item, free_win_item);
	if (!window->payload_ready_queue) {
		delete_sliding_window(window);
		return NULL;
	}

	window->payload_recycle_queue = xQueueCreate(PAYLOAD_QUEUE_LENGTH, sizeof(payload_info_t *));
	if (!window->payload_recycle_queue) {
		delete_sliding_window(window);
		return NULL;
	}

	payload_info_t *new_payload = NULL;
	while (xQueueReceive(window->payload_ready_queue, &new_payload, 0) == pdTRUE) {
		if (new_payload) {
			memset(new_payload, 0, sizeof(payload_info_t));
			new_payload->payload = malloc(window->max_payload_size);
			if (new_payload->payload) {
				memset(new_payload->payload, 0, window->max_payload_size);
				xQueueSend(window->payload_recycle_queue, &new_payload, 0);
			} else {
				SLIDING_WIN_ERR("new_payload->payload fail\r\n");
				delete_sliding_window(window);
				return NULL;
			}
		}
	}

	window->lock = xSemaphoreCreateMutex();
	if (!window->lock) {
		SLIDING_WIN_ERR("window->lock fail\r\n");
		delete_sliding_window(window);
		return NULL;
	}

	if (xTaskCreate(send_packet_thread, ((const char *)"send_packet_thread"), SLIDING_SEND_STACK, window, tskIDLE_PRIORITY + SLIDING_SEND_PRIORITY,
					&(window->sendtask)) != pdPASS) {
		SLIDING_WIN_ERR("Task send_packet_thread create fail\r\n");
		delete_sliding_window(window);
		return NULL;
	}

	if (xTaskCreate(ack_queue_thread, ((const char *)"ack_queue_thread"), SLIDING_ACK_STACK, window, tskIDLE_PRIORITY + SLIDING_ACK_PRIORITY,
					&(window->recvtask)) != pdPASS) {
		SLIDING_WIN_ERR("Task ack_queue_thread create fail\r\n");
		delete_sliding_window(window);
		return NULL;
	}

	return window;
}

void sliding_update_seg_data_size(SlidingWindow *window, uint32_t seg_payload_size)
{
	if (seg_payload_size >= 1 && seg_payload_size <= window->max_payload_size) {
		window->seg_payload_size = seg_payload_size;
		SLIDING_WIN_INFO("Update seg payload size success %u\r\n", window->seg_payload_size);
		return;
	}
	SLIDING_WIN_ERR("Update seg payload size fail\r\n");
	return;
}
