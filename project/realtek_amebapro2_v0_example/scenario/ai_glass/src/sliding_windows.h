#ifndef __SLIDING_WINDOWS_H__
#define __SLIDING_WINDOWS_H__

/******************************************************************************
 *
 * Copyright(c) 2007 - 2015 Realtek Corporation. All rights reserved.
 *
 *
 ******************************************************************************/
#define MAX_PAYLOAD_SIZE        1024
#define ACK_QUEUE_LENGTH        10
#define PAYLOAD_QUEUE_LENGTH    10
#define MAX_WINDOW_SIZE         8
#define MIN_WINDOW_SIZE         1
#define TIMEOUT_PERIOD          1000 // milliseconds
#define MAX_RETRIES             30
#define MAX_DUPLICATE_ACKS      3
#define SLIDING_SEND_PRIORITY   7
#define SLIDING_ACK_PRIORITY    6
#define SLIDING_SEND_STACK      4096
#define SLIDING_ACK_STACK       4096
#define MAX_EXTEND_BOX_NUM      4

#define PACKET_LABEL_EOF        0x01
#define PACKET_LABEL_FIN        0x02

// For getting ack
typedef struct {
	uint32_t start_seq;
	uint32_t exp_seq;
} sack_data_t;

typedef struct {
	uint32_t ack_seq;
	uint16_t window_size;
	uint8_t label;
	uint8_t reserve;
	uint32_t extend_length;
	uint8_t *extend_data;
} ack_info_t;

// For sending the packet
typedef struct {
	uint8_t label;
	uint8_t reserve0;
	uint8_t reserve1;
	uint8_t reserve2;
	uint16_t length;
	uint8_t *payload;
} payload_info_t;

typedef struct {
	uint32_t seq;
	payload_info_t *spayload;
} sliding_pkt_t;

typedef struct {
	uint32_t timestamp;
	sliding_pkt_t packet;
	bool acked;
	uint8_t retry_count;
} window_item_t;

typedef struct {
	uint32_t max_payload_size;
	uint32_t seg_payload_size;
	uint32_t send_base;
	uint32_t next_seq;
	uint16_t window_size;
	window_item_t items[MAX_WINDOW_SIZE];
	QueueHandle_t payload_ready_queue;
	QueueHandle_t payload_recycle_queue;
	QueueHandle_t ack_queue;
	SemaphoreHandle_t lock;
	uint32_t last_ack;
	uint8_t duplicate_ack_count;
	TaskHandle_t sendtask;
	TaskHandle_t recvtask;
	int (*send_packet)(const sliding_pkt_t *packet);
} SlidingWindow;

// Sliding window ID for the extend box
enum {
	SLIDING_EXT_ID_SACK = 5,
};

void sliding_on_ack_received(SlidingWindow *window, ack_info_t *ack_info, const uint8_t *extend_data);
int sliding_send_data(SlidingWindow *window, uint8_t *data, uint16_t data_length, bool data_continue, int timeout);
void delete_sliding_window(SlidingWindow *window);
SlidingWindow *create_sliding_window(int (*send_packet)(const sliding_pkt_t *), uint16_t window_size, uint32_t start_seq, uint32_t max_payload_size);
void sliding_update_seg_data_size(SlidingWindow *window, uint32_t seg_payload_size);
void sliding_update_window_size(SlidingWindow *window, uint16_t window_size);

#endif //#ifndef __SLIDING_WINDOWS_H__
