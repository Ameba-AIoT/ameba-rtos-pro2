/******************************************************************************
 *
 * Copyright(c) 2007 - 2015 Realtek Corporation. All rights reserved.
 *
 * SPI Slave module for AI Glass
 *
 * Implements a SPI0 slave loopback test:
 *   - Initializes SPI0 in slave mode (8-bit, mode 0, CS active low)
 *   - Uses DMA for TX/RX (or PIO if SPI_DMA_DEMO == 0)
 *   - Transfers 4 MB total (2048-byte blocks x 2048 loops)
 *   - Verifies RX data matches TX data
 *   - Reports timing and mismatch info via dbg_printf
 *
 ******************************************************************************/
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "gpio_api.h"
#include "spi_api.h"
#include "spi_ex_api.h"
#include "wait_api.h"
#include "sys_api.h"
#include "diag.h"

#include "spi_init.h"

/* ---------------------------------------------------------------------------
 * Configuration
 * ------------------------------------------------------------------------- */
#define TEST_BUF_SIZE       2048
#define TX_TOTAL_SIZE       (4 * 1024 * 1024)   // 4MB
#define TX_LOOPS            (TX_TOTAL_SIZE / TEST_BUF_SIZE)
#define SCLK_FREQ           31250000
#define SPI_DMA_DEMO        1
#define GPIO_SYNC_PIN       PF_1

// SPI0 (S0) pin mapping
#define SPI0_MOSI  PE_3
#define SPI0_MISO  PE_2
#define SPI0_SCLK  PE_1
#define SPI0_CS    PE_4

/* ---------------------------------------------------------------------------
 * External / forward declarations
 * ------------------------------------------------------------------------- */
extern void hal_ssi_toggle_between_frame(phal_ssi_adaptor_t phal_ssi_adaptor, u8 ctl);

/* ---------------------------------------------------------------------------
 * Module-level globals
 * ------------------------------------------------------------------------- */
static char TxBuf[TEST_BUF_SIZE] __attribute__((aligned(32)));
static char RxBuf[TEST_BUF_SIZE] __attribute__((aligned(32)));
static spi_t spi_slave;
static gpio_t GPIO_Syc;

static SemaphoreHandle_t tx_done_sem;
static SemaphoreHandle_t rx_done_sem;

/* ---------------------------------------------------------------------------
 * Callback
 * ------------------------------------------------------------------------- */
static void slave_trx_done_callback(void *pdata, SpiIrq event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    switch (event) {
    case SpiRxIrq:
        xSemaphoreGiveFromISR(rx_done_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        break;
    case SpiTxIrq:
        xSemaphoreGiveFromISR(tx_done_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        break;
    default:
        dbg_printf("SPI slave: unknown interrupt event! \r\n");
    }
}

/* ---------------------------------------------------------------------------
 * SPI slave task
 * ------------------------------------------------------------------------- */
static void spi_tx_task(void *param)
{
    int i;

    dbg_printf("\r\n   SPI Stream Twoboard Concurrent DEMO   \r\n");

    /* --- GPIO sync pin --- */
    gpio_init(&GPIO_Syc, GPIO_SYNC_PIN);
    gpio_dir(&GPIO_Syc, PIN_OUTPUT);
    gpio_mode(&GPIO_Syc, PullNone);
    gpio_write(&GPIO_Syc, 0);

    /* --- Semaphores --- */
    tx_done_sem = xSemaphoreCreateBinary();
    rx_done_sem = xSemaphoreCreateBinary();

    /* Unregister pins from any previous GPIO assignment (system init may claim them) */
    if (SPI0_MOSI != NC) hal_pinmux_unregister((uint32_t)SPI0_MOSI, PID_GPIO);
    if (SPI0_MISO != NC) hal_pinmux_unregister((uint32_t)SPI0_MISO, PID_GPIO);
    if (SPI0_SCLK != NC) hal_pinmux_unregister((uint32_t)SPI0_SCLK, PID_GPIO);
    if (SPI0_CS != NC) hal_pinmux_unregister((uint32_t)SPI0_CS, PID_GPIO);

    /* --- SPI init (slave) --- */
    spi_init(&spi_slave, SPI0_MOSI, SPI0_MISO, SPI0_SCLK, SPI0_CS);
    spi_format(&spi_slave, 8,
               ((int)SPI_SCLK_IDLE_LOW | (int)SPI_SCLK_TOGGLE_START), 1);
    hal_ssi_toggle_between_frame(&spi_slave.hal_ssi_adaptor, ENABLE);

    /* Wait for SPI bus to be ready */
    while (spi_busy(&spi_slave)) {
        dbg_printf("Wait SPI Bus Ready... \r\n");
        wait_ms(1000);
    }

    /* --- Fill TX buffer --- */
    memset(TxBuf, 0, TEST_BUF_SIZE);
    for (i = 0; i < TEST_BUF_SIZE; i++) {
        TxBuf[i] = i;
    }

    /* --- Hook IRQ & flush RX FIFO --- */
    spi_irq_hook(&spi_slave, (spi_irq_handler)slave_trx_done_callback,
                 (uint32_t)&spi_slave);
    spi_flush_rx_fifo(&spi_slave);

    /* --- Main transfer loop --- */
    dbg_printf("Start TX 4MB (%d loops x %d bytes) ...\r\n",
               TX_LOOPS, TEST_BUF_SIZE);
    u32 t_start = hal_read_cur_time();

    for (i = 0; i < TX_LOOPS; i++) {

#if SPI_DMA_DEMO
        spi_slave_read_stream_dma(&spi_slave, RxBuf, TEST_BUF_SIZE);
        spi_slave_write_stream_dma(&spi_slave, TxBuf, TEST_BUF_SIZE);
#else
        spi_slave_read_stream(&spi_slave, RxBuf, TEST_BUF_SIZE);
        spi_slave_write_stream(&spi_slave, TxBuf, TEST_BUF_SIZE);
#endif
        gpio_write(&GPIO_Syc, 1);

        xSemaphoreTake(tx_done_sem, portMAX_DELAY);
        xSemaphoreTake(rx_done_sem, portMAX_DELAY);
        while (spi_busy(&spi_slave)) {
            wait_us(1);
        }

        /* Verify */
        for (int j = 0; j < TEST_BUF_SIZE; j++) {
            if (TxBuf[j] != RxBuf[j]) {
                dbg_printf("Loop %d: mismatch at byte %d "
                           "(tx=0x%02x rx=0x%02x)\r\n",
                           i, j,
                           (unsigned char)TxBuf[j],
                           (unsigned char)RxBuf[j]);
            }
        }

        gpio_write(&GPIO_Syc, 0);
    }

    u32 t_end = hal_read_cur_time();
    u32 elapsed_us = t_end - t_start;
    dbg_printf("TX 4MB done: %lu us (%lu ms, %lu.%03lu s)\r\n",
               elapsed_us, elapsed_us / 1000,
               elapsed_us / 1000000, (elapsed_us % 1000000) / 1000);

    /* --- Cleanup --- */
    spi_free(&spi_slave);
    dbg_printf("SPI Demo finished. \r\n");

    vTaskDelete(NULL);
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */
void spi_slave_start(void)
{
    if (xTaskCreate(spi_tx_task, "spi_tx", 4096, NULL,
                    tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        dbg_printf("SPI slave: xTaskCreate(spi_tx) failed\r\n");
    }
}