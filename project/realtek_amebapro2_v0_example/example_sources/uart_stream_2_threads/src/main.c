#include "device.h"
#include "serial_api.h"
#include "serial_ex_api.h"
#include "task.h"
//#include "main.h"

#define TX_DMA_MODE   1    // 0: interrupt mode, 1: DMA mode
#define RX_DMA_MODE   1    // 0: interrupt mode, 1: DMA mode

/*UART pin location:
   UART0:
   PA_23  (TX)
   PA_18  (RX)
   */
#define UART_TX    PE_1
#define UART_RX    PE_2

#define UART_TIMEOUT_MS     3000    // 3 sec

#define SRX_BUF_SZ      1536
#define STX_BUF_SZ      1536

#define UART_STACK_SZ       2048


char tx_buf[STX_BUF_SZ] __attribute__((aligned(32))) = {0};
char rx_buf[SRX_BUF_SZ] __attribute__((aligned(32))) = {0};
volatile uint32_t tx_bytes = 0;
volatile uint32_t rx_bytes = 0;
unsigned int tx_size;

SemaphoreHandle_t  UartHWSema;  // Uart HW resource
SemaphoreHandle_t  UartRxSema;  // RX done
SemaphoreHandle_t  UartTxSema;  // TX ready, also use as TX done

void uart_send_done(uint32_t id)
{
	serial_t *sobj = (void *)id;
	signed portBASE_TYPE xHigherPriorityTaskWoken;

	//tx_bytes += sobj->tx_len;
	tx_bytes += tx_size;
	xSemaphoreGiveFromISR(UartTxSema, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void uart_recv_done(uint32_t id)
{
	serial_t *sobj = (void *)id;
	signed portBASE_TYPE xHigherPriorityTaskWoken;

	//rx_bytes += sobj->rx_len;
	rx_bytes += sobj->uart_adp.rx_count;
	xSemaphoreGiveFromISR(UartRxSema, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void uart_tx_thread(void *param)
{
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif
	serial_t *psobj = param; // UART object
	int i;
	int tx_byte_timeout;
	int32_t ret;
	unsigned int loop_cnt = 0;
	//unsigned int tx_size;

	// Initial TX buffer
	for (i = 0; i < STX_BUF_SZ; i++) {
		tx_buf[i] = 0x30 + (i % 10);
	}
	tx_buf[STX_BUF_SZ - 1] = 0; // end of string

	while (1) {
		loop_cnt++;
		tx_size = (loop_cnt % STX_BUF_SZ) + 1;
		// Wait TX Rady (TX Done)
		if (xSemaphoreTake(UartTxSema, (TickType_t) UART_TIMEOUT_MS / portTICK_RATE_MS) != pdTRUE) {
			dbg_printf("%s: send timeout!!\r\n");
			xSemaphoreTake(UartHWSema, portMAX_DELAY);  // get the semaphore before access the HW
			tx_byte_timeout = serial_send_stream_abort(psobj);
			xSemaphoreGive(UartHWSema);     // return the semaphore after access the HW
			if (tx_byte_timeout > 0) {
				tx_bytes += tx_byte_timeout;
			}

			xSemaphoreGive(UartTxSema);     // Ready to TX
		} else {
			xSemaphoreTake(UartHWSema, portMAX_DELAY);  // get the semaphore before access the HW
#if TX_DMA_MODE
			ret = serial_send_stream_dma(psobj, tx_buf, tx_size);
#else
			ret = serial_send_stream(psobj, tx_buf, tx_size);
#endif
			xSemaphoreGive(UartHWSema);     // return the semaphore after access the HW
			if (ret != 0) {
				dbg_printf("uart_tx_thread: send error %d\r\n", ret);
				xSemaphoreGive(UartTxSema);     // Ready to TX
			}
		}

	}

	vTaskDelete(NULL);

}

void uart_rx_thread(void *param)
{
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif
	serial_t *psobj = param; // UART object
	int rx_byte_timeout;
	int ret;
	unsigned int loop_cnt = 0;
	unsigned int rx_size;


	while (1) {
		loop_cnt++;
		rx_size = (loop_cnt % SRX_BUF_SZ) + 1;
		xSemaphoreTake(UartHWSema, portMAX_DELAY);  // get the semaphore before access the HW
#if RX_DMA_MODE
		ret = serial_recv_stream_dma(psobj, rx_buf, rx_size);
#else
		ret = serial_recv_stream(psobj, rx_buf, rx_size);
#endif
		xSemaphoreGive(UartHWSema);     // return the semaphore after access the HW
		if (ret == 0) {
			// Wait RX done
			if (xSemaphoreTake(UartRxSema, (TickType_t) UART_TIMEOUT_MS / portTICK_RATE_MS) != pdTRUE) {
				xSemaphoreTake(UartHWSema, portMAX_DELAY);  // get the semaphore before access the HW
				rx_byte_timeout = serial_recv_stream_abort(psobj);
				xSemaphoreGive(UartHWSema);     // return the semaphore after access the HW
				if (rx_byte_timeout > 0) {
					rx_bytes += rx_byte_timeout;
				}
			}
		} else {
			dbg_printf("uart_rx_thread: receive error %d\r\n", ret);
		}
	}

	vTaskDelete(NULL);

}

void uart_test_demo(void *param)
{
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif
	serial_t sobj;  // UART object
	int ret;

	serial_init(&sobj, UART_TX, UART_RX);
	serial_baud(&sobj, 38400);
	serial_format(&sobj, 8, ParityNone, 1);

	serial_send_comp_handler(&sobj, (void *)uart_send_done, (uint32_t) &sobj);
	serial_recv_comp_handler(&sobj, (void *)uart_recv_done, (uint32_t) &sobj);

	// Create semaphore for UART HW resource
	UartHWSema = xSemaphoreCreateBinary();
	xSemaphoreGive(UartHWSema);

	// Create semaphore for UART RX done(received espected bytes or timeout)
	UartRxSema = xSemaphoreCreateBinary();

	// Create semaphore for UART TX done
	UartTxSema = xSemaphoreCreateBinary();
	xSemaphoreGive(UartTxSema);      // Ready to TX

	if (xTaskCreate(uart_rx_thread, ((const char *)"uart_recv_thread"), (UART_STACK_SZ >> 2), &sobj, tskIDLE_PRIORITY + 2, NULL) != pdPASS) {
		dbg_printf("xTaskCreate(uart_rx_thread) failed\r\n");
	}

	if (xTaskCreate(uart_tx_thread, ((const char *)"uart_tx_thread"), (UART_STACK_SZ >> 2), &sobj, tskIDLE_PRIORITY + 2, NULL) != pdPASS) {
		dbg_printf("xTaskCreate(uart_tx_thread) failed\r\n");
	}

	while (1) {
		dbg_printf("tx_bytes:%d rx_bytes:%d\r\n", tx_bytes, rx_bytes);
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

void main(void)
{
	// create demo Task
	if (xTaskCreate((TaskFunction_t)uart_test_demo, "uart test demo", (2048 / 2), (void *)NULL, (tskIDLE_PRIORITY + 1), NULL) != pdPASS) {
		dbg_printf("Cannot create uart test demo task\n\r");
		goto end_demo;
	}


	vTaskStartScheduler();


end_demo:
	while (1);
}



