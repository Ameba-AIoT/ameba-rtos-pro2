/******************************************************************************
 *
 * Copyright(c) 2007 - 2015 Realtek Corporation. All rights reserved.
 *
 *
 ******************************************************************************/
#include <platform_opts.h>
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include <platform_stdlib.h>
#include "semphr.h"
//#include "device.h"
#include "serial_api.h"
#include "serial_ex_api.h"
#include "uart_service.h"
#include "ai_glass_media.h"
#include "ai_glass_dbg.h"

#define UART_SYNC_WORD  0xAA

#define UART_CMD_PRIORITY       5
#define UART_CRITICAL_PRIORITY  7
#define UART_ACK_PRIORITY       8
#define UART_RECV_PRIORITY      8

#define UART_THREAD_NUM         3       // The thread num for uart common command

// UART infomation for both ic to sync
uint8_t uart_protocal_version = UART_PROTOCAL_VER;
uint16_t uart_buff_size = UART_DEFAULT_BUF_SIZE;
uint16_t uart_pic_size = UART_DEFAULT_PIC_SIZE;
uint8_t uart_wifi_ic_type = UART_WIFI_IC_TYPE;

//
static TaskHandle_t uartcmdtask[UART_THREAD_NUM] = {0};
static TaskHandle_t uartcrticaltask = NULL;
static TaskHandle_t uartacktask = NULL;
static TaskHandle_t uartrecvtask = NULL;

static QueueHandle_t rx_uart_recycle = NULL;            // This queue is for rx uart recycle
static QueueHandle_t rx_uart_ready = NULL;              // This queue is for rx uart ready
static QueueHandle_t tx_uart_recycle = NULL;            // This queue is for tx uart recycle
static QueueHandle_t tx_uart_ready = NULL;              // This queue is for tx uart ready

static QueueHandle_t rx_critical_recycle = NULL;        // This queue is for rx critical recycle
static QueueHandle_t rx_critical_ready = NULL;          // This queue is for rx critical ready
static QueueHandle_t tx_critical_recycle = NULL;        // This queue is for tx critical recycle
static QueueHandle_t tx_critical_ready = NULL;          // This queue is for tx critical ready

static QueueHandle_t rx_uart_ack_recycle = NULL;        // This queue is for rx uart ack recycle
static QueueHandle_t rx_uart_ack_ready = NULL;          // This queue is for rx uart ack ready

static QueueHandle_t rx_uart_ack_tmp_recycle = NULL;    // This queue is for rx uart ack ready

static QueueHandle_t tx_uart_ack_recycle = NULL;        // This queue is for tx uart ack recycle
static QueueHandle_t tx_uart_ack_ready = NULL;          // This queue is for tx uart ack ready

static SemaphoreHandle_t uart_tx_free_sema = NULL;

static SemaphoreHandle_t tx_resp_mutex = NULL;
static SemaphoreHandle_t tx_pkt_sema = NULL;
static SemaphoreHandle_t tx_ack_mutex = NULL;

static uint8_t rx_exp_seq = 0;
static uint8_t tx_exp_seq = 0;

typedef struct serial_service_s {
	serial_t    sobj;
	uint8_t     uart_init;
} serial_service_t;
static serial_service_t srobj = {0};

// uart cmd callback function
static callback_entry_t uartcmdcb_table[UART_RX_OPC_COUNT];

// Buffer for DMA
static uint8_t uart_txbuf[UART_MAX_PIC_SIZE] __attribute__((aligned(32))) = {0};
//static uint8_t uart_rxbuf[UART_MAX_PIC_SIZE] __attribute__((aligned(32))) = {0};

static StreamBufferHandle_t uart_rx_stream = NULL;

static uint8_t calculate_checksum(uint8_t sync_word, uint8_t seq_number, uint16_t resp_opcode, uart_params_t *params_array, uint16_t *length)
{
	uint8_t checksum = 0;

	// Calculate checksum with sync word and seq number
	//checksum += sync_word; // Remove the sync word from checksum counting
	checksum += seq_number;

	// Calculate checksum with resp_opcode
	checksum += (resp_opcode & 0xFF);        // Low byte of resp_opcode
	checksum += ((resp_opcode >> 8) & 0xFF); // High byte of resp_opcode

	// Update the length with resp_opcode length (2)
	*length = 2;

	// Calculate with all params and sum up the length
	for (uart_params_t *param = params_array; param != NULL; param = param->next) {
		// Add params length to total length
		*length += param->length;
		// Calculate with param
		for (uint16_t i = 0; i < param->length; i++) {
			checksum += param->data[i];
		}
	}

	// Calculate with total length
	checksum += (*length & 0xFF);             // Low byte of length
	checksum += ((*length >> 8) & 0xFF);      // High byte of length
	UART_SRV_MSG("resp_opcode = 0x%04x, length %d\r\n", resp_opcode, *length);

	// Return the 1's complement of the checksum
	return (0xff - checksum + 1); //((~checksum)+1);
}

static uint8_t get_expected_tx_seq_number(void)
{
	tx_exp_seq = (tx_exp_seq % 255) + 1;
	return tx_exp_seq;
}

static uint8_t get_expected_rx_seq_number(void)
{
	rx_exp_seq = (rx_exp_seq % 255) + 1;
	return rx_exp_seq;
}

typedef void (*FreeItemFn)(void *);
typedef void *(*InitItemFn)(void);

static void free_cmd_item(void *item)
{
	uartcmdpacket_t *cmd_pakcet = (uartcmdpacket_t *)item;
	if (cmd_pakcet->uart_pkt.data_buf) {
		free(cmd_pakcet->uart_pkt.data_buf);
	}
	free(cmd_pakcet);
}

static void free_pkt_item(void *item)
{
	uartpacket_t *pakcet = (uartpacket_t *)item;
	if (pakcet->data_buf) {
		free(pakcet->data_buf);
	}
	free(pakcet);
}

static void free_ack_item(void *item)
{
	free(item);
}

static void *init_cmd_item(void)
{
	uartcmdpacket_t *cmd_pakcet = malloc(sizeof(uartcmdpacket_t));
	if (cmd_pakcet) {
		memset(cmd_pakcet, 0, sizeof(uartcmdpacket_t));
	}
	return cmd_pakcet;
}
#if 0 // the tx queue is not need yet
static void *init_pkt_item(void)
{
	uartpacket_t *pakcet = malloc(sizeof(uartpacket_t));
	if (pakcet) {
		memset(pakcet, 0, sizeof(uartpacket_t));
	}
	return pakcet;
}
#endif
static void *init_ack_item(void)
{
	uartackpacket_t *ack_pakcet = malloc(sizeof(uartackpacket_t));
	if (ack_pakcet) {
		memset(ack_pakcet, 0, sizeof(uartackpacket_t));
	}
	return ack_pakcet;
}

static void delete_queue_if_exists(QueueHandle_t *queue, FreeItemFn free_item_fn)
{
	void *item;
	if (*queue) {
		while (xQueueReceive(*queue, &item, 0) == pdPASS) {
			if (free_item_fn) {
				free_item_fn(item);
			}
		}
		vQueueDelete(*queue);
		*queue = NULL;
	}
}

static void free_rx_uart_queue(void)
{
	delete_queue_if_exists(&rx_uart_recycle, free_cmd_item);
	delete_queue_if_exists(&rx_uart_ready, free_cmd_item);
	delete_queue_if_exists(&rx_critical_recycle, free_cmd_item);
	delete_queue_if_exists(&rx_critical_ready, free_cmd_item);
	delete_queue_if_exists(&rx_uart_ack_recycle, free_ack_item);
	delete_queue_if_exists(&rx_uart_ack_tmp_recycle, free_ack_item);
	delete_queue_if_exists(&rx_uart_ack_ready, free_ack_item);
}

static void free_tx_uart_queue(void)
{
	delete_queue_if_exists(&tx_uart_recycle, free_pkt_item);
	delete_queue_if_exists(&tx_uart_ready, free_pkt_item);
	delete_queue_if_exists(&tx_critical_recycle, free_pkt_item);
	delete_queue_if_exists(&tx_critical_ready, free_pkt_item);
	delete_queue_if_exists(&tx_uart_ack_recycle, free_ack_item);
	delete_queue_if_exists(&tx_uart_ack_ready, free_ack_item);
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
				if (free_item_fn) {
					free_item_fn(item_array[j]);
				}
			}
			vQueueDelete(queue);
			return NULL;
		}
		if (xQueueSend(queue, &item_array[i], 0) != pdPASS) {
			// Free already allocated items before returning NULL
			for (int j = 0; j <= i; j++) {
				if (free_item_fn) {
					free_item_fn(item_array[j]);
				}
			}
			vQueueDelete(queue);
			return NULL;
		}
	}
	return queue;
}

static int create_rx_uart_queue(int queue_length, int critical_queue_length, int ack_queue_length)
{
	uartcmdpacket_t *cmd_item[queue_length];
	uartackpacket_t *ack_item[ack_queue_length];

	rx_uart_recycle = create_queue_and_fill(queue_length, sizeof(uartcmdpacket_t *), (void **)cmd_item, init_cmd_item, free_cmd_item);
	if (!rx_uart_recycle) {
		UART_SRV_ERR("Failed to create rx_uart_recycle queue\n");
		free_rx_uart_queue();
		return UART_QUEUE_CREATE_FAIL;
	}
	rx_uart_ready = xQueueCreate(queue_length, sizeof(uartcmdpacket_t *));
	if (!rx_uart_ready) {
		UART_SRV_ERR("Failed to create rx_uart_ready queue\n");
		free_rx_uart_queue();
		return UART_QUEUE_CREATE_FAIL;
	}

	rx_critical_recycle = create_queue_and_fill(critical_queue_length, sizeof(uartcmdpacket_t *), (void **)cmd_item, init_cmd_item, free_cmd_item);
	if (!rx_critical_recycle) {
		UART_SRV_ERR("Failed to create rx_critical_recycle queue\n");
		free_rx_uart_queue();
		return UART_QUEUE_CREATE_FAIL;
	}
	rx_critical_ready = xQueueCreate(critical_queue_length, sizeof(uartcmdpacket_t *));
	if (!rx_critical_ready) {
		UART_SRV_ERR("Failed to create rx_critical_ready queue\n");
		free_rx_uart_queue();
		return UART_QUEUE_CREATE_FAIL;
	}

	rx_uart_ack_recycle = create_queue_and_fill(ack_queue_length, sizeof(uartackpacket_t *), (void **)ack_item, init_ack_item, free_ack_item);
	if (!rx_uart_ack_recycle) {
		UART_SRV_ERR("Failed to create rx_uart_ack_recycle queue\n");
		free_rx_uart_queue();
		return UART_QUEUE_CREATE_FAIL;
	}
	rx_uart_ack_tmp_recycle = xQueueCreate(ack_queue_length, sizeof(uartackpacket_t *));
	if (!rx_uart_ack_tmp_recycle) {
		UART_SRV_ERR("Failed to create rx_uart_ack_tmp_recycle queue\n");
		free_rx_uart_queue();
		return UART_QUEUE_CREATE_FAIL;
	}
	rx_uart_ack_ready = xQueueCreate(ack_queue_length, sizeof(uartackpacket_t *));
	if (!rx_uart_ack_ready) {
		UART_SRV_ERR("Failed to create rx_uart_ack_ready queue\n");
		free_rx_uart_queue();
		return UART_QUEUE_CREATE_FAIL;
	}
	return UART_OK;
}

static int create_tx_uart_queue(int queue_length, int critical_queue_length, int ack_queue_length)
{
	uartackpacket_t *ack_item[ack_queue_length];
#if 0 // the tx queue is not need yet
	uartpacket_t *pkt_item[queue_length];
	tx_uart_recycle = create_queue_and_fill(queue_length, sizeof(uartpacket_t *), (void **)pkt_item, init_pkt_item, free_pkt_item);
	if (!tx_uart_recycle) {
		UART_SRV_ERR("Failed to create tx_uart_recycle queue\n");
		free_tx_uart_queue();
		return UART_QUEUE_CREATE_FAIL;
	}
	tx_uart_ready = xQueueCreate(queue_length, sizeof(uartpacket_t *));
	if (!tx_uart_ready) {
		UART_SRV_ERR("Failed to create tx_uart_ready queue\n");
		free_tx_uart_queue();
		return UART_QUEUE_CREATE_FAIL;
	}

	tx_critical_recycle = create_queue_and_fill(critical_queue_length, sizeof(uartpacket_t *), (void **)pkt_item, init_pkt_item, free_pkt_item);
	if (!tx_critical_recycle) {
		UART_SRV_ERR("Failed to create tx_critical_recycle queue\n");
		free_tx_uart_queue();
		return UART_QUEUE_CREATE_FAIL;
	}
	tx_critical_ready = xQueueCreate(critical_queue_length, sizeof(uartpacket_t *));
	if (!tx_critical_ready) {
		UART_SRV_ERR("Failed to create tx_critical_ready queue\n");
		free_tx_uart_queue();
		return UART_QUEUE_CREATE_FAIL;
	}
#endif
	tx_uart_ack_recycle = create_queue_and_fill(ack_queue_length, sizeof(uartackpacket_t *), (void **)ack_item, init_ack_item, free_ack_item);
	if (!tx_uart_ack_recycle) {
		UART_SRV_ERR("Failed to create tx_uart_ack_recycle queue\n");
		free_tx_uart_queue();
		return UART_QUEUE_CREATE_FAIL;
	}
	tx_uart_ack_ready = xQueueCreate(ack_queue_length, sizeof(uartackpacket_t *));
	if (!tx_uart_ack_ready) {
		UART_SRV_ERR("Failed to create tx_uart_ack_ready queue\n");
		free_tx_uart_queue();
		return UART_QUEUE_CREATE_FAIL;
	}
	return UART_OK;
}

static int uart_send_ack(uint8_t recv_seq_number, uint16_t recv_opcode, uint8_t status)
{
	uartackpacket_t *ack_packet;
	if (!IS_VALID_UART_RX_OPC(recv_opcode)) {
		UART_SRV_ERR("[UART WARNING] Received cmd packet with invalid opcode 0x%04x\r\n", recv_opcode);
		return UART_OK;
	}
	if (IS_NO_ACK_UART_RX_OPC(recv_opcode)) {
		UART_SRV_MSG("Received cmd packet with no ack opcode 0x%04x\r\n", recv_opcode);
		return UART_OK;
	}
	int ret = xQueueReceive(tx_uart_ack_recycle, (void *)&ack_packet, 0);
	if (ret == pdPASS) {
		// Initialize and populate ack_packet only if it's successfully received
		ack_packet->length = ACK_DATA_LEN; // length not including sync_word(1), seq_number(1), length(2), and checksum(1)
		ack_packet->opcode = UART_TX_OPC_ACK;
		ack_packet->recv_opcode = recv_opcode;
		ack_packet->status = status;

		if (xQueueSend(tx_uart_ack_ready, (void *)&ack_packet, 0) != pdPASS) {
			UART_SRV_ERR("Failed to ready ack_packet\r\n");
			return UART_SEND_PACKET_FAIL;
		}
	} else {
		// Handle error if no ack_packet is available in tx_uart_ack_recycle
		UART_SRV_WARN("the ack queue is now busy\r\n");
		return UART_ACK_PACKET_UNAVAILABLE;
	}

	return UART_OK; // Return success code if everything went well
}


static void uart_service_receive_handler(uint8_t received_byte)
{
	static uartcmdpacket_t *rx_pkt = NULL;
	static uartackpacket_t *rx_ack_pkt = NULL;
	static uint32_t index = 0;
	static uint8_t rx_current_seq = 0;
	static uint16_t rx_current_len = 0;
	static uint16_t rx_current_opcode = 0;
	static uint8_t rx_current_checksum = 0;
	static uint8_t rx_current_status = AI_GLASS_CMD_UNKNOWN;

	switch (index) {
	case 0:
		// check command header
		if (received_byte == UART_SYNC_WORD) {
			// header scorrect keep dump packet
			rx_current_checksum = 0;
			index++;
		} else {
			// header incorrect return
			index = 0;
		}
		break;
	case 1:
		rx_current_seq = received_byte;
		rx_current_checksum += received_byte;
		index++;
		break;
	case 2:
		rx_current_len = received_byte;
		rx_current_checksum += received_byte;
		index++;
		break;
	case 3:
		rx_current_len |= (uint16_t)received_byte << 8;
		rx_current_checksum += received_byte;
		index++;
		if (rx_current_len < 2) {
			// length incorrect return
			index = 0;
		}
		break;
	case 4:
		rx_current_opcode = received_byte;
		rx_current_checksum += received_byte;
		index++;
		break;
	case 5:
		rx_current_opcode |= (uint16_t)received_byte << 8;
		rx_current_checksum += received_byte;
		index++;
		if (rx_current_opcode == UART_RX_OPC_ACK) {
			UART_SRV_INFO("get ack packet rx_current_opcode\r\n");
			if (xQueueReceive(rx_uart_ack_recycle, (void *)&rx_ack_pkt, 0) == pdPASS) {
				if (rx_current_len != ACK_DATA_LEN) {
					UART_SRV_WARN("rxack length expect %d but get %d\r\n", ACK_DATA_LEN, rx_current_len);
				}
				rx_ack_pkt->length = rx_current_len;
				rx_ack_pkt->opcode = rx_current_opcode;
				rx_ack_pkt->sync_word = UART_SYNC_WORD;
				rx_ack_pkt->seq_number = rx_current_seq;
			}
		} else if (IS_VALID_UART_RX_OPC(rx_current_opcode)) {
			QueueHandle_t *queuerecycle = (IS_CRITICAL_UART_RX_OPC(rx_current_opcode) ? &rx_critical_recycle : &rx_uart_recycle);
			if (xQueueReceive((*queuerecycle), (void *)&rx_pkt, 0) == pdPASS) {
				if (rx_pkt->uart_pkt.data_buf) {
					free(rx_pkt->uart_pkt.data_buf);
					rx_pkt->uart_pkt.data_buf = NULL;
				}
				rx_pkt->uart_pkt.data_buf = malloc(rx_current_len);
				if (!rx_pkt->uart_pkt.data_buf) {
					UART_SRV_WARN("rxpkt buf %d alloc fail\r\n", rx_current_len);
					xQueueSend((*queuerecycle), (void *)&rx_pkt, 0);
					rx_pkt = NULL;
				}
				rx_pkt->uart_pkt.length = rx_current_len;
				rx_pkt->uart_pkt.opcode = rx_current_opcode;
				rx_pkt->uart_pkt.sync_word = UART_SYNC_WORD;
				rx_pkt->uart_pkt.seq_number = rx_current_seq;
				rx_pkt->exp_seq_number = get_expected_rx_seq_number();
				rx_current_status = AI_GLASS_CMD_COMPLETE;
			} else {
				rx_current_status = AI_GLASS_BUSY;
			}
		} else {
			rx_current_status = AI_GLASS_CMD_UNKNOWN;
		}
		break;
	default:
		if (index == (4 + rx_current_len)) {
			uint8_t exp_checksum = 0XFF - rx_current_checksum + 1;
			if (rx_ack_pkt) {
				if (exp_checksum == received_byte && IS_VALID_UART_TX_OPC(rx_ack_pkt->recv_opcode) && rx_ack_pkt->recv_opcode != UART_TX_OPC_ACK) {
					rx_ack_pkt->checksum = received_byte;
					// Check sum and ack will be verify in thread, to reduce the irq loading
					rx_ack_pkt->get_ts = xTaskGetTickCountFromISR();
					xQueueSend(rx_uart_ack_ready, (void *)&rx_ack_pkt, 0);
					rx_ack_pkt = NULL;
				} else {
					// Ack check sum fail, send back to recycle queue
					xQueueSend(rx_uart_ack_recycle, (void *)&rx_ack_pkt, 0);
					rx_ack_pkt = NULL;
					UART_SRV_WARN("rx ack check 0x%02x, 0x%02x, 0x%02x, 0x%02x\r\n", rx_ack_pkt->recv_opcode, rx_current_checksum, 0XFF - rx_current_checksum + 1, received_byte);
				}
			} else if (rx_current_opcode != UART_RX_OPC_ACK) {
				// TODO: Check if the sequence is valid or not
				uart_send_ack(rx_current_seq, rx_current_opcode, rx_current_status);
				if (rx_pkt) {
					QueueHandle_t *queuerecycle = (IS_CRITICAL_UART_RX_OPC(rx_current_opcode) ? &rx_critical_recycle : &rx_uart_recycle);
					QueueHandle_t *queueready = (IS_CRITICAL_UART_RX_OPC(rx_current_opcode) ? &rx_critical_ready : &rx_uart_ready);
					if (exp_checksum == received_byte) {
						rx_pkt->uart_pkt.checksum = received_byte;
						// Check sum and ack will be verify in thread, to reduce the irq loading
						xQueueSend((*queueready), (void *)&rx_pkt, 0);
						rx_pkt = NULL;
					} else {
						if (rx_pkt->uart_pkt.data_buf) {
							free(rx_pkt->uart_pkt.data_buf);
							rx_pkt->uart_pkt.data_buf = NULL;
						}
						// Check sum and ack will be verify in thread, to reduce the irq loading
						xQueueSend((*queuerecycle), (void *)&rx_pkt, 0);
						rx_pkt = NULL;
						UART_SRV_WARN("rx cmd check\r\n");
					}
				}
			}
			rx_current_checksum = 0;
			index = 0;
		} else if (index >= 6 && index < (4 + rx_current_len)) { // length include opcode length
			rx_current_checksum += received_byte;
			if (rx_ack_pkt) {
				if (index == 6) {
					rx_ack_pkt->recv_opcode = received_byte;
				} else if (index == 7) {
					rx_ack_pkt->recv_opcode |= received_byte << 8;
				} else if (index == 8) {
					rx_ack_pkt->status = received_byte;
				}
			} else if (rx_pkt && rx_pkt->uart_pkt.data_buf) {
				rx_pkt->uart_pkt.data_buf[index - 6] = received_byte;
			}
			index++;
		}
		break;
	}
}

static void process_uart_recv_thread(void *params)
{
	uint8_t data[UART_MAX_BUF_SIZE] = {0};
	uint32_t bytes_read = 0;
	while (1) {
		bytes_read = xStreamBufferReceive(uart_rx_stream, data, UART_MAX_BUF_SIZE, portMAX_DELAY);
		for (int i = 0; i < bytes_read; i++) {
			uart_service_receive_handler(data[i]);
		}
	}
}

uint8_t rx_uart_tmp_buf[UART_MAX_BUF_SIZE];
static void uart_service_irq(uint32_t id, SerialIrq event)
{
	serial_t *sobj = (void *)id;
	uint32_t xBytesGet = 0;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (event == RxIrq) {
		uint8_t rc = 0;
		while (serial_readable(sobj)) {
			rc = (uint8_t)serial_getc(sobj);
			rx_uart_tmp_buf[xBytesGet] = rc;
			xBytesGet ++;
			if (xBytesGet >= UART_MAX_BUF_SIZE) {
				break;
			}
		}
		if (xBytesGet > 0 && xBytesGet <= UART_MAX_BUF_SIZE) {
			xStreamBufferSendFromISR(uart_rx_stream, rx_uart_tmp_buf, xBytesGet, NULL);
			if (xHigherPriorityTaskWoken) {
				taskYIELD();
			}
		}
	}

	if (event == TxIrq) {
	}
}

// This thread is for uart command process
static void process_uart_cmd_thread(void *params)
{
	uartcmdpacket_t *recv_pkt;

	while (1) {
		if (xQueueReceive(rx_uart_ready, (void *)&recv_pkt, portMAX_DELAY) == pdPASS) {
			UART_SRV_INFO("Get cmd packet\r\n");
			if (uartcmdcb_table[recv_pkt->uart_pkt.opcode - UART_RX_OPC_MIN].callback != NULL) {
				uartcmdcb_table[recv_pkt->uart_pkt.opcode - UART_RX_OPC_MIN].callback(recv_pkt);
			} else {
				UART_SRV_MSG("the callbacck function for 0x%04x critical command is not registered yet\r\n", recv_pkt->uart_pkt.opcode);
			}

			// Free data buffer if allocated
			if (recv_pkt->uart_pkt.data_buf) {
				free(recv_pkt->uart_pkt.data_buf);
				recv_pkt->uart_pkt.data_buf = NULL;
			}

			// End of cmd function process
			if (xQueueSend(rx_uart_recycle, (void *)&recv_pkt, 0) != pdPASS) {
				// Handle error
				UART_SRV_ERR("Failed to recycle recv_pkt\r\n");
			}
			UART_SRV_INFO("End of process_uart_cmd_thread\r\n");
		}
	}
}

// This thread is for critical uart command process
static void process_uart_critical_thread(void *params)
{
	uartcmdpacket_t *recv_pkt;
	// This thread is for critical command (like power off, stop stream, stop transporting)
	while (1) {
		if (xQueueReceive(rx_critical_ready, (void *)&recv_pkt, portMAX_DELAY) == pdPASS) {
			UART_SRV_INFO("Get critical packet\r\n");
			if (uartcmdcb_table[recv_pkt->uart_pkt.opcode - UART_RX_OPC_MIN].callback != NULL) {
				uartcmdcb_table[recv_pkt->uart_pkt.opcode - UART_RX_OPC_MIN].callback(recv_pkt);
			} else {
				UART_SRV_MSG("the callbacck function for 0x%04x critical command is not registered yet\r\n", recv_pkt->uart_pkt.opcode);
			}

			// Free data buffer if allocated
			if (recv_pkt->uart_pkt.data_buf) {
				free(recv_pkt->uart_pkt.data_buf);
				recv_pkt->uart_pkt.data_buf = NULL;
			}

			// End of cmd function process
			if (xQueueSend(rx_critical_recycle, (void *)&recv_pkt, 0) != pdPASS) {
				// Handle error
				UART_SRV_ERR("Failed to recycle recv_pkt\r\n");
				// Depending on the system, consider freeing recv_pkt or handling it differently
			}
			UART_SRV_INFO("End of process_uart_critical_thread\r\n");
		}
	}
}

// This thread is for handling the ack when the rx receive cmd packet
static void process_uart_ack_thread(void *params)
{
	uartackpacket_t *ack_packet;
	// This thread is for critical command (like power off, stop stream, stop transporting)
	while (1) {
		if (xQueueReceive(tx_uart_ack_ready, (void *)&ack_packet, portMAX_DELAY) == pdPASS) {
			uart_params_t status_params = {
				.data = (uint8_t *) & (ack_packet->status),
				.length = 1,
				.next = NULL
			};
			uart_params_t opcode_params = {
				.data = (uint8_t *) & (ack_packet->recv_opcode),
				.length = 2,
				.next = &status_params
			};
			int ret = uart_send_packet(UART_TX_OPC_ACK, &opcode_params, true, 10000);
			if (ret != UART_OK) {
				// Handle error, e.g., log or attempt recovery
				UART_SRV_ERR("Error sending ACK packet: 0x%02x\r\n", ret);
			}
			// End of the cmd function process
			if (xQueueSend(tx_uart_ack_recycle, (void *)&ack_packet, 0) != pdPASS) {
				// Handle error
				UART_SRV_ERR("Failed to recycle ack_packet\r\n");
				// Depending on the system, consider freeing recv_pkt or handling it differently
			}
			UART_SRV_INFO("End of process_uart_ack_thread\r\n");
		}
	}
}

static void uart_send_str_done(uint32_t id)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (id != (int) & (srobj.sobj)) {
		void (*dFreeItemFn)(void *) = NULL;
		dFreeItemFn(NULL);
	}
	xSemaphoreGiveFromISR(uart_tx_free_sema, &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken) {
		taskYIELD();
	}
}

#if 0 // Todo
static int uart_send_buffer(serial_service_t *srobj, uint8_t *pstr, uint16_t length, uint8_t endbuf)
{
	if (!(srobj->uart_init)) {
		return UART_SEND_PACKET_FAIL;
	}
	serial_t *sobj = (serial_t *) & (srobj->sobj);

	static uint16_t send_idx = 0;
	uint16_t input_idx = 0;
	if (send_print) {
		for (uint16_t i = 0; i < length; i++) {
			UART_SRV_MSG("tx 0x%02x\r\n", pstr[i]);
		}
	}

	while (length > 0) {
		uint16_t tmp_uart_pic_size = UART_MAX_PIC_SIZE;
		uint16_t remaining_space = tmp_uart_pic_size - send_idx;
		int ret = 0;
		if (length >= remaining_space) {
			if (xSemaphoreTake(uart_tx_free_sema, portMAX_DELAY) == pdTRUE) {
				memcpy(uart_txbuf + send_idx, pstr + input_idx, remaining_space);
				length -= remaining_space;
				input_idx += remaining_space;
				ret = serial_send_stream_dma(sobj, (char *)uart_txbuf, tmp_uart_pic_size);
				send_idx = 0;
				if (ret != HAL_OK) {
					UART_SRV_ERR("%s Error(%d) \r\n", __FUNCTION__, ret);
					if (ret != HAL_BUSY) {
						xSemaphoreGive(uart_tx_free_sema);
					}
					return UART_SEND_PACKET_FAIL;
				}
			} else {
				UART_SRV_ERR("uart_tx_free_sema get failed\r\n");
				return UART_SEND_PACKET_FAIL;
			}
		} else {
			if (xSemaphoreTake(uart_tx_free_sema, portMAX_DELAY) == pdTRUE) {
				memcpy(uart_txbuf + send_idx, pstr + input_idx, length);
				ret = serial_send_stream_dma(sobj, (char *)uart_txbuf, length);
				send_idx = 0;
				length = 0;
				if (ret != HAL_OK) {
					UART_SRV_ERR("%s Error(%d) \r\n", __FUNCTION__, ret);
					if (ret != HAL_BUSY) {
						xSemaphoreGive(uart_tx_free_sema);
					}
					return UART_SEND_PACKET_FAIL;
				}
			} else {
				UART_SRV_ERR("uart_tx_free_sema get failed\r\n");
				return UART_SEND_PACKET_FAIL;
			}
		}
	}

	return UART_OK;
}
#else
static int uart_send_buffer(serial_service_t *srobj, uint8_t *pstr, uint16_t length, uint8_t endbuf)
{
	if (!(srobj->uart_init)) {
		return UART_SEND_PACKET_FAIL;
	}
	serial_t *sobj = (serial_t *) & (srobj->sobj);

	static uint16_t send_idx = 0;
	uint16_t input_idx = 0;

	while (length > 0) {
		uint16_t tmp_uart_pic_size = uart_pic_size;
		uint16_t remaining_space = tmp_uart_pic_size - send_idx;
		int ret = 0;
		if (length >= remaining_space) {
			if (xSemaphoreTake(uart_tx_free_sema, portMAX_DELAY) == pdTRUE) {
				memcpy(uart_txbuf + send_idx, pstr + input_idx, remaining_space);
				length -= remaining_space;
				input_idx += remaining_space;
				ret = serial_send_stream_dma(sobj, (char *)uart_txbuf, tmp_uart_pic_size);
				send_idx = 0;
				if (ret != HAL_OK) {
					UART_SRV_ERR("%s Error(%d) \r\n", __FUNCTION__, ret);
					if (ret != HAL_BUSY) {
						xSemaphoreGive(uart_tx_free_sema);
					}
					return UART_SEND_PACKET_FAIL;
				}
			} else {
				UART_SRV_ERR("uart_tx_free_sema get failed\r\n");
				return UART_SEND_PACKET_FAIL;
			}
		} else {
			if (xSemaphoreTake(uart_tx_free_sema, portMAX_DELAY) == pdTRUE) {
				memcpy(uart_txbuf + send_idx, pstr + input_idx, length);
				send_idx += length;
				length = 0;
				if (endbuf) {
					for (uint16_t i = 0; i < send_idx; i++) {
						UART_SRV_MSG("uart_txbuf 0x%02x\r\n", uart_txbuf[i]);
					}
					ret = serial_send_stream_dma(sobj, (char *)uart_txbuf, send_idx);
					send_idx = 0;
					if (ret != HAL_OK) {
						UART_SRV_ERR("%s Error(%d) \r\n", __FUNCTION__, ret);
						if (ret != HAL_BUSY) {
							xSemaphoreGive(uart_tx_free_sema);
						}
						return UART_SEND_PACKET_FAIL;
					}
				} else {
					xSemaphoreGive(uart_tx_free_sema);
				}
			} else {
				UART_SRV_ERR("uart_tx_free_sema get failed\r\n");
				return UART_SEND_PACKET_FAIL;
			}
		}
	}

	return UART_OK;
}
#endif

#define PACKET_RESEND_TIME 3
int uart_send_packet(uint16_t resp_opcode, uart_params_t *params_head, bool ignore_ack, int timeout)
{
	int tmp_ret = 0;
	int ret = UART_OK;
	uartackpacket_t *ack_packet = NULL;
	uint8_t sync_word = UART_SYNC_WORD;
	int retry_time = PACKET_RESEND_TIME;
	uint8_t tx_seq = 0;

ResendPacket:
	// Try to take the resp mutex if not sending an ACK response
	if (!ignore_ack && xSemaphoreTake(tx_resp_mutex, timeout) != pdTRUE) {
		UART_SRV_WARN("uart_send_packet 0x%02x get tx_resp_mutex timeout\r\n", resp_opcode);
		return UART_SEND_PACKET_TIMEOUT;
	}

	// Try to take the packet mutex
	if (xSemaphoreTake(tx_pkt_sema, timeout) != pdTRUE) {
		if (!ignore_ack) {
			xSemaphoreGive(tx_resp_mutex);
		}
		UART_SRV_WARN("uart_send_packet 0x%02x get tx_pkt_sema timeout\r\n", resp_opcode);
		return UART_SEND_PACKET_TIMEOUT;
	}
	uint32_t packet_send_time = xTaskGetTickCount();
	uint16_t length = 0;
	if (tx_seq == 0) {
		tx_seq = get_expected_tx_seq_number();
	}
	uint8_t checksum = calculate_checksum(sync_word, tx_seq, resp_opcode, params_head, &length);

	// Array of data to be sent
	struct {
		const uint8_t *data;
		int len;
	} tx_data[] = {
		{ &sync_word, 1 },
		{ &tx_seq, 1 },
		{ (uint8_t *) &length, 2 },
		{ (uint8_t *) &resp_opcode, 2 },
	};

#if SEND_ACK_SHOW
	if (resp_opcode == UART_TX_OPC_ACK) {
		send_print = 1;
	}
#endif
	// Send all data before params
	for (int i = 0; i < sizeof(tx_data) / sizeof(tx_data[0]); i++) {
		ret = uart_send_buffer(&(srobj), (uint8_t *)tx_data[i].data, tx_data[i].len, 0);
		if (ret != UART_OK) {
			UART_SRV_ERR("send pakcet 0x%04x fail\r\n", resp_opcode);
			xSemaphoreGive(tx_pkt_sema);
			goto end;
		}
	}

	// Send all params data
	for (uart_params_t *param = params_head; param != NULL; param = param->next) {
		ret = uart_send_buffer(&(srobj), param->data, param->length, 0);
		if (ret != UART_OK) {
			UART_SRV_ERR("send pakcet 0x%04x fail\r\n", resp_opcode);
			xSemaphoreGive(tx_pkt_sema);
			goto end;
		}
	}

	// Send check sum
	tmp_ret = uart_send_buffer(&(srobj), &checksum, 1, 1);
	if (tmp_ret != UART_OK) {
		UART_SRV_ERR("send pakcet 0x%04x fail\r\n", resp_opcode);
		xSemaphoreGive(tx_pkt_sema);
		goto end;
	}

	if (resp_opcode == UART_TX_OPC_ACK) {
		UART_SRV_MSG("send ack\r\n");
	}

	// Update sequence number
	xSemaphoreGive(tx_pkt_sema);

	// Wait for ACK if not sending an ACK response
	if (!ignore_ack) {
		if (xSemaphoreTake(tx_ack_mutex, timeout) != pdTRUE) {
			UART_SRV_ERR("uart_send_packet 0x%02x get tx_ack_mutex timeout\r\n", resp_opcode);
			ret = UART_ACK_TIMEOUT;
			goto end;
		}

		while (xQueueReceive(rx_uart_ack_tmp_recycle, (void *)&ack_packet, 0)) {
			if (ack_packet->get_ts < packet_send_time) {
				xQueueSend(rx_uart_ack_recycle, (void *)&ack_packet, 0);
				UART_SRV_MSG("Recycle old ack opcode 0x%02x receive time %lu, packet send time at %lu\r\n", ack_packet->recv_opcode, ack_packet->get_ts, packet_send_time);
			} else if (ack_packet->recv_opcode == resp_opcode) {
				xQueueSend(rx_uart_ack_recycle, (void *)&ack_packet, 0);
				UART_SRV_MSG("Get Ack from recycle tmp queue opcode 0x%02x receive time %lu, packet send time %lu\r\n", ack_packet->recv_opcode, ack_packet->get_ts,
							 packet_send_time);
				if (ack_packet->status != AI_GLASS_CMD_COMPLETE) {
					ret = UART_ACK_TIMEOUT;
				} else {
					ret = UART_OK;
				}
				xSemaphoreGive(tx_ack_mutex);
				goto end;
			} else {
				xQueueSend(rx_uart_ack_tmp_recycle, (void *)&ack_packet, 0);
			}
		}
RegetAck:
		tmp_ret = xQueueReceive(rx_uart_ack_ready, (void *)&ack_packet, timeout / 3);
		if (tmp_ret == pdTRUE) {
			if (ack_packet->get_ts >= packet_send_time) {
				if (ack_packet->recv_opcode != resp_opcode) {
					xQueueSend(rx_uart_ack_tmp_recycle, (void *)&ack_packet, 0);
					UART_SRV_MSG("Get ack opcode 0x%02x but expexted opcode 0x%02x, send the ack to temp buffer\r\n", ack_packet->recv_opcode, resp_opcode);
					goto RegetAck;
				} else {
					UART_SRV_MSG("rx ack sync_word = 0x%02x\r\n", ack_packet->sync_word);
					UART_SRV_MSG("rx ack seq_number = 0x%02x\r\n", ack_packet->seq_number);
					UART_SRV_MSG("rx ack length = 0x%04x\r\n", ack_packet->length);
					UART_SRV_MSG("rx ack opcode = 0x%04x\r\n", ack_packet->opcode);
					UART_SRV_MSG("rx ack recv_opcode = 0x%02x\r\n", ack_packet->recv_opcode);
					UART_SRV_MSG("rx ack status = 0x%02x\r\n", ack_packet->status);
					UART_SRV_MSG("rx ack checksum = 0x%02x\r\n", ack_packet->checksum);
					UART_SRV_MSG("rx ack packet_send_time = 0x%08lx\r\n", packet_send_time);
					UART_SRV_MSG("rx ack get timestamp = 0x%08lx\r\n", ack_packet->get_ts);
					xQueueSend(rx_uart_ack_recycle, (void *)&ack_packet, 0);
					if (ack_packet->status != AI_GLASS_CMD_COMPLETE) {
						ret = UART_ACK_TIMEOUT;
					} else {
						ret = UART_OK;
					}
					xSemaphoreGive(tx_ack_mutex);
				}
			} else {
				xQueueSend(rx_uart_ack_recycle, (void *)&ack_packet, 0);
				UART_SRV_MSG("Recycle old ack opcode 0x%02x receive time %lu, packet send time at %lu\r\n", ack_packet->recv_opcode, ack_packet->get_ts, packet_send_time);
				goto RegetAck;
			}
		} else {
			UART_SRV_MSG("Wait ack for opcode 0x%02x timeout\r\n", resp_opcode);
			ret = UART_ACK_TIMEOUT;
			xSemaphoreGive(tx_ack_mutex);
			if (retry_time != 0) {
				UART_SRV_MSG("Retry send 0x%02x, remain retry time %d\r\n", resp_opcode, retry_time);
				xSemaphoreGive(tx_resp_mutex);
				vTaskDelay(200);
				retry_time --;
				goto ResendPacket;
			} else {
				UART_SRV_MSG("Retry send 0x%02x, remain retry time %d, Send fail\r\n", resp_opcode, retry_time);
				ret = UART_ACK_TIMEOUT;
				goto end;
			}
		}
	}

end:
	if (!ignore_ack) {
		xSemaphoreGive(tx_resp_mutex);
	}
	return ret;
}

void uart_service_send_pwron_cmd(void)
{
	if (srobj.uart_init) {
		uint8_t power_status = AI_GLASS_CMD_COMPLETE;
		uart_params_t pwr_start_pkt = {
			.data = &power_status,
			.length = 1,
			.next = NULL
		};
		uart_send_packet(UART_TX_OPC_RESP_POWER_ON, &pwr_start_pkt, false, 2000);
	} else {
		UART_SRV_WARN("the uart service is not initialed yet, send fail\r\n");
	}
}

int uart_service_init(PinName tx_pin, PinName rx_pin, int baudrate)
{
	int ret = 0;
	serial_init(&(srobj.sobj), tx_pin, rx_pin);

	uart_rx_stream = xStreamBufferCreate(UART_MAX_BUF_SIZE, 1);
	if (uart_rx_stream == NULL) {
		UART_SRV_ERR("uart_rx_stream fail\r\n");
		ret = UART_SERVICE_INIT_FAIL;
		return ret;
	}

	serial_baud(&(srobj.sobj), baudrate);
	serial_format(&(srobj.sobj), 8, ParityNone, 1);
	serial_irq_handler(&(srobj.sobj), uart_service_irq, (uint32_t) & (srobj.sobj));
	serial_send_comp_handler(&(srobj.sobj), (void *)uart_send_str_done, (uint32_t) & (srobj.sobj));

	ret = create_rx_uart_queue(MAX_UART_QUEUE_SIZE, MAX_CRITICAL_QUEUE_SIZE, MAX_UARTACK_QUEUE_SIZE);
	if (ret != UART_OK) {
		UART_SRV_ERR("create_rx_uart_queue fail\r\n");
		return ret;
	}
	ret = create_tx_uart_queue(MAX_UART_QUEUE_SIZE, MAX_CRITICAL_QUEUE_SIZE, MAX_UARTACK_QUEUE_SIZE);
	if (ret != UART_OK) {
		UART_SRV_ERR("create_tx_uart_queue fail\r\n");
		return ret;
	}
	uart_tx_free_sema = xSemaphoreCreateBinary();
	if (uart_tx_free_sema == NULL) {
		UART_SRV_ERR("uart_tx_free_sema create fail\r\n");
		return UART_SERVICE_INIT_FAIL;
	}
	xSemaphoreGive(uart_tx_free_sema);
	tx_resp_mutex = xSemaphoreCreateMutex();
	if (tx_resp_mutex == NULL) {
		UART_SRV_ERR("tx_resp_mutex create fail\r\n");
		return UART_SERVICE_INIT_FAIL;
	}
	tx_pkt_sema = xSemaphoreCreateBinary();
	if (tx_pkt_sema == NULL) {
		UART_SRV_ERR("tx_pkt_sema create fail\r\n");
		return UART_SERVICE_INIT_FAIL;
	}
	xSemaphoreGive(tx_pkt_sema);
	tx_ack_mutex = xSemaphoreCreateMutex();
	if (tx_ack_mutex == NULL) {
		UART_SRV_ERR("tx_ack_mutex create fail\r\n");
		return UART_SERVICE_INIT_FAIL;
	}
	char threadname[32] = {0};
	for (int i = 0; i < UART_THREAD_NUM; i++) {
		snprintf(threadname, 32, "UARTCMD[%d]", i);
		if (xTaskCreate(process_uart_cmd_thread, ((const char *)threadname), 4096, NULL, tskIDLE_PRIORITY + UART_CMD_PRIORITY, &(uartcmdtask[i])) != pdPASS) {
			UART_SRV_ERR("Task process_uart_cmd_thread %d create fail\r\n", i);
			return UART_SERVICE_INIT_FAIL;
		}
	}
	if (xTaskCreate(process_uart_critical_thread, ((const char *)"UARTCRITICAL"), 4096, NULL, tskIDLE_PRIORITY + UART_CRITICAL_PRIORITY,
					&uartcrticaltask) != pdPASS) {
		UART_SRV_ERR("Task process_uart_critical_thread create fail\r\n");
		return UART_SERVICE_INIT_FAIL;
	}
	if (xTaskCreate(process_uart_ack_thread, ((const char *)"UARTAck"), 4096, NULL, tskIDLE_PRIORITY + UART_ACK_PRIORITY, &uartacktask) != pdPASS) {
		UART_SRV_ERR("Task process_uart_ack_thread create fail\r\n");
		return UART_SERVICE_INIT_FAIL;
	}
	if (xTaskCreate(process_uart_recv_thread, ((const char *)"UARTRecv"), 4096, NULL, tskIDLE_PRIORITY + UART_RECV_PRIORITY, &uartrecvtask) != pdPASS) {
		UART_SRV_ERR("Task process_uart_recv_thread create fail\r\n");
		return UART_SERVICE_INIT_FAIL;
	}
	srobj.uart_init = 1;
	return UART_OK;
}

void uart_service_poweroff(void)
{
	if (srobj.uart_init) {
		srobj.uart_init = 0;
		serial_free(&(srobj.sobj));
	}
	return;
}

void uart_service_deinit(void)
{
	uart_service_poweroff();
	for (int i = 0; i < UART_THREAD_NUM; i++) {
		if (uartcmdtask[i] != NULL) {
			vTaskDelete(uartcmdtask[i]);
			uartcmdtask[i] = NULL;
		}
	}
	if (uartcrticaltask) {
		vTaskDelete(uartcrticaltask);
		uartcrticaltask = NULL;
	}
	if (uartacktask) {
		vTaskDelete(uartacktask);
		uartacktask = NULL;
	}
	if (uartrecvtask) {
		vTaskDelete(uartrecvtask);
		uartrecvtask = NULL;
	}

	if (uart_rx_stream) {
		vStreamBufferDelete(uart_rx_stream);
		uart_rx_stream = NULL;
	}

	free_rx_uart_queue();
	free_tx_uart_queue();
	if (uart_tx_free_sema != NULL) {
		vSemaphoreDelete(uart_tx_free_sema);
		uart_tx_free_sema = NULL;
	}

	if (tx_resp_mutex != NULL) {
		vSemaphoreDelete(tx_resp_mutex);
		tx_resp_mutex = NULL;
	}

	if (tx_pkt_sema != NULL) {
		vSemaphoreDelete(tx_pkt_sema);
		tx_pkt_sema = NULL;
	}

	if (tx_ack_mutex != NULL) {
		vSemaphoreDelete(tx_ack_mutex);
		tx_ack_mutex = NULL;
	}

	return;
}

int uart_service_rx_cmd_reg(uint16_t uart_cmd_type, callback_t uart_cmd_fun)
{
	if (IS_VALID_UART_RX_OPC(uart_cmd_type)) {
		UART_SRV_MSG("uart_cmd_type found\r\n");
		uartcmdcb_table[uart_cmd_type - UART_RX_OPC_MIN].opcode = uart_cmd_type;
		uartcmdcb_table[uart_cmd_type - UART_RX_OPC_MIN].callback = uart_cmd_fun;
		return UART_OK;
	}

	UART_SRV_WARN("uart_cmd_type not found\r\n");
	return UART_REG_CALLBACK_FAIL;
}

int uart_service_start(int send_power_start)
{
	if (srobj.uart_init) {
		serial_irq_set(&(srobj.sobj), RxIrq, 1);
		serial_irq_set(&(srobj.sobj), TxIrq, 1);
		// Send power on in the start
		// disable this value
		if (send_power_start) {
			uart_service_send_pwron_cmd();
		}
	} else {
		return UART_START_FAIL;
	}

	return UART_OK;
}