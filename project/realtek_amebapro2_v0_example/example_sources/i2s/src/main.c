/* This is software data check example */

#include "cmsis.h"
#include "i2s_api.h"
#include "wait_api.h"

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
//#include "alc5651.c"

i2s_t i2s_obj;

volatile u8 page_cnt = 0;

#define I2S_DMA_PAGE_SIZE	64   // 2 ~ 4096
#define I2S_DMA_PAGE_NUM    4   // Vaild number is 2~4

u8 i2s_tx_buf[I2S_DMA_PAGE_SIZE * I2S_DMA_PAGE_NUM]__attribute__((aligned(0x20)));;
u8 i2s_rx_buf[I2S_DMA_PAGE_SIZE * I2S_DMA_PAGE_NUM]__attribute__((aligned(0x20)));;

#define I2S_SCLK_PIN            PF_13//PE_0
#define I2S_WS_PIN              PF_15//PE_3
#define I2S_TX_PIN              PF_14//PE_1
#define I2S_RX_PIN              PF_12//PE_4
#define I2S_MCK_PIN             PF_11//PE_2 //PE_3 or NC
#define I2S_TX1_PIN             NC
#define I2S_TX2_PIN             NC

void i2s_initial_ram(void)
{
	int j, tx_index, tx_pattern;

	for (j = 0; j < ((I2S_DMA_PAGE_SIZE * I2S_DMA_PAGE_NUM) >> 2) ; j++) {

		tx_pattern = j + 1;
		tx_index = j * 4;
		i2s_tx_buf[tx_index ]   = (tx_pattern & 0x00FF);
		i2s_tx_buf[tx_index + 1] = (tx_pattern & 0xFF00) >> 8;
		i2s_tx_buf[tx_index + 2] = (tx_pattern & 0xFF0000) >> 16;
		i2s_tx_buf[tx_index + 3] = (tx_pattern & 0xFF000000) >> 24;

		if ((j % 8) == 7) {
			dbg_printf(" %4x\r\n", tx_pattern);
		} else {
			dbg_printf(" %4x ", tx_pattern);
		}
	}

}

void i2s_data_check(void)
{
	int j, rx_index, rx_pattern;

	for (j = 0; j < ((I2S_DMA_PAGE_SIZE * I2S_DMA_PAGE_NUM) >> 2) ; j++) {

		rx_index = j * 4;
		rx_pattern = (i2s_rx_buf[rx_index]  | (i2s_rx_buf[rx_index + 1] << 8) \
					  | (i2s_rx_buf[rx_index + 2] << 16) | (i2s_rx_buf[rx_index + 3] << 24));

		if ((j % 8) == 7) {
			dbg_printf(" %4x\r\n", rx_pattern);
		} else {
			dbg_printf(" %4x ", rx_pattern);
		}
	}
}

void test_tx_complete(void *data, char *pbuf)
{
	i2s_t *obj = (i2s_t *)data;

	i2s_send_page(obj, (uint32_t *)pbuf);
}

void test_rx_complete(void *data, char *pbuf)
{
	i2s_t *obj = (i2s_t *)data;

	if (page_cnt < 8) {
		page_cnt++;
	} else {
		i2s_disable(obj);
	}

	i2s_recv_page(obj);
}

void main(void)
{
	int *ptx_buf;
	int i, j;

	dbg_printf(" \r\n");
	dbg_printf(" \n Set TX Pattern \r\n");
	i2s_initial_ram();

	// I2S init
	i2s_init(&i2s_obj, I2S_SCLK_PIN, I2S_WS_PIN, I2S_TX_PIN, I2S_RX_PIN, I2S_MCK_PIN);
	i2s_set_param(&i2s_obj, CH_STEREO, SR_32KHZ, WL_24b);
	i2s_set_direction(&i2s_obj, I2S_DIR_TX);
	i2s_set_loopback(&i2s_obj, ENABLE);
	i2s_set_dma_buffer(&i2s_obj, (char *)i2s_tx_buf, (char *)i2s_rx_buf, \
					   I2S_DMA_PAGE_NUM, I2S_DMA_PAGE_SIZE);
	i2s_tx_irq_handler(&i2s_obj, (i2s_irq_handler)test_tx_complete, (uint32_t)&i2s_obj);
	i2s_rx_irq_handler(&i2s_obj, (i2s_irq_handler)test_rx_complete, (uint32_t)&i2s_obj);

	/* rx need clock, let tx out first */
	for (j = 0; j < I2S_DMA_PAGE_NUM; j++) {
		ptx_buf = i2s_get_tx_page(&i2s_obj);
		if (ptx_buf) {
			i2s_send_page(&i2s_obj, (uint32_t *)ptx_buf);
		}
		i2s_recv_page(&i2s_obj);
	}

	// Start DMA
	i2s_enable(&i2s_obj);

	while (1) {

		if (page_cnt == 8) {
			dbg_printf(" \n Check RX Data \r\n");
			i2s_data_check();
			break;
		}
	}

	i2s_deinit(&i2s_obj);
	while (1);
}
