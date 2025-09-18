#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "atcmd_mqtt.h"
#include "lwipconf.h"
#include "diag.h"
#include "log_service.h"
#include "stream_buffer.h"
#include "stdio_port_func.h"

static MQTTClient *mqtt_client = NULL;
static const char *mqtt_response_topic = NULL;
static StreamBufferHandle_t response_stream = NULL;
static int atcmd_mqtt_enabled = 0;

extern char log_buf[LOG_SERVICE_BUFLEN];
extern xSemaphoreHandle log_rx_interrupt_sema;

// MQTT Output redirection
extern int check_in_critical(void);
static unsigned atcmd_mqtt_write_buffer(unsigned fd, const void *buf, unsigned len)
{
	if (buf && len > 0 && response_stream) {
		// Using Stream Buffer to cache response data
		if (check_in_critical()) {
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			size_t bytes_sent = xStreamBufferSendFromISR(response_stream, buf, len, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			return bytes_sent;
		} else {
			size_t bytes_sent = xStreamBufferSend(response_stream, buf, len, pdMS_TO_TICKS(100));
			return bytes_sent;
		}
	}
	return len;
}

static unsigned atcmd_mqtt_read_buffer(unsigned fd, void *buf, unsigned len)
{
	return 0;
}

int atcmd_mqtt_process(const char *cmd, int cmd_len, char *response, int max_resp_len)
{
	if (!atcmd_mqtt_enabled || !cmd || cmd_len <= 0) {
		return -1;
	}

	mqtt_printf(MQTT_INFO, "Processing AT command: %.*s\n", cmd_len, cmd);

	// Clear the response stream
	if (response_stream) {
		xStreamBufferReset(response_stream);
	}

	if (cmd_len >= LOG_SERVICE_BUFLEN) {
		cmd_len = LOG_SERVICE_BUFLEN - 1;
	}
	memset(log_buf, '\0', LOG_SERVICE_BUFLEN);
	memcpy(log_buf, cmd, cmd_len);

	// Trigger AT command processing
	if (log_rx_interrupt_sema != NULL) {
		rtw_up_sema((_sema *)&log_rx_interrupt_sema);
	}

	vTaskDelay(pdMS_TO_TICKS(300));

	int total_received = 0;
	if (response_stream && response && max_resp_len > 1) {
		char *current_pos = response;
		int remaining_space = max_resp_len - 1;
		
		// Continue receiving data until there is no more data or pixels
		while (remaining_space > 0) {
			size_t bytes_received = xStreamBufferReceive(
				response_stream, 
				current_pos, 
				remaining_space,
				pdMS_TO_TICKS(50)
			);
			
			if (bytes_received == 0) {
				break; 
			}
			
			total_received += bytes_received;
			current_pos += bytes_received;
			remaining_space -= bytes_received;
		}
		
		response[total_received] = '\0';  
	} else if (!response_stream && response && max_resp_len > 0) {
		// Handling Stream Buffer Unavailability
		mqtt_printf(MQTT_WARNING, "Stream buffer not available, cannot capture response\n");
		response[0] = '\0';  
	}

	mqtt_printf(MQTT_INFO, "AT command response length: %d\n", total_received);

	return total_received;
}

// Setting up the MQTT client
void atcmd_mqtt_set_client(MQTTClient *client, const char *response_topic)
{
	mqtt_client = client;
	mqtt_response_topic = response_topic;
}

void atcmd_mqtt_init(void)
{
	if (!atcmd_mqtt_enabled) {
		response_stream = xStreamBufferCreate(4096, 1);
		if (response_stream == NULL) {
			mqtt_printf(MQTT_ERROR, "Failed to create stream buffer\n");
			return;  // Initialization failed, the enabled flag is not set
		}
		atcmd_mqtt_enabled = 1;
		extern void remote_stdio_init(void *read_cb, void *write_cb);

		// Redirecting remote output to MQTT
		remote_stdio_init(atcmd_mqtt_read_buffer, atcmd_mqtt_write_buffer);
		mqtt_printf(MQTT_INFO, "AT command over MQTT service initialized\n");
	}
}

void atcmd_mqtt_deinit(void)
{
	atcmd_mqtt_enabled = 0;
	if (response_stream) {
		vStreamBufferDelete(response_stream);
		response_stream = NULL;
	}
}