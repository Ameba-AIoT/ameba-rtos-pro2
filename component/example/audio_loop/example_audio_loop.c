#include "platform_opts.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include "audio_api.h"
#include "avcodec.h"
#include "example_audio_loop.h"

#define AD_PAGE_SIZE 512 //64*N bytes 
#define TX_AD_PAGE_SIZE AD_PAGE_SIZE
#define RX_AD_PAGE_SIZE AD_PAGE_SIZE
#define DMA_AD_PAGE_NUM 4

#if defined(CONFIG_PLATFORM_8735B)
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
#include "osdep_service.h"
#endif
#if IS_CUT_TEST(CONFIG_CHIP_VER)
#define DMIC_CLK_PIN    PE_2 //PE_0
#define DMIC_DATA_PIN   PE_4
#else
#define DMIC_CLK_PIN    PD_16 //PD_14
#define DMIC_DATA_PIN   PD_18
#endif
#endif

static audio_t audio_obj;
static uint8_t ad_dma_txdata[TX_AD_PAGE_SIZE * DMA_AD_PAGE_NUM]__attribute__((aligned(0x20)));
static uint8_t ad_dma_rxdata[RX_AD_PAGE_SIZE * DMA_AD_PAGE_NUM]__attribute__((aligned(0x20)));
static uint8_t ad_dma_rxbuf_stereo_l[RX_AD_PAGE_SIZE]__attribute__((aligned(0x20)));
static uint8_t ad_dma_rxbuf_stereo_r[RX_AD_PAGE_SIZE]__attribute__((aligned(0x20)));
volatile uint32_t chg_test_flag = 0;

static void audio_tx_irq(uint32_t arg, uint8_t *pbuf)
{
	audio_t *obj = (audio_t *)arg;

	if (audio_get_tx_error_cnt(obj) != 0x00) {
		dbg_printf("tx page error !!! \r\n");
	}

}

static void audio_rx_irq(uint32_t arg, uint8_t *pbuf)
{
	audio_t *obj = (audio_t *)arg;
	uint8_t *ptx_addre;
	uint32_t j;

	if (audio_get_rx_error_cnt(obj) != 0x00) {
		dbg_printf("rx page error !!! \r\n");
	}

	ptx_addre = audio_get_tx_page_adr(obj);

	if (CONFIG_MIC_TYPE == USE_DMIC_STEREO) {
		if (chg_test_flag == 0) {
			for (j = 0; j < (AD_PAGE_SIZE / sizeof(int16_t) / 2); j++) {
				ad_dma_rxbuf_stereo_l[0 + j * 4] = pbuf[0 + j * 8];
				ad_dma_rxbuf_stereo_l[1 + j * 4] = pbuf[1 + j * 8];
				ad_dma_rxbuf_stereo_l[2 + j * 4] = pbuf[4 + j * 8];
				ad_dma_rxbuf_stereo_l[3 + j * 4] = pbuf[5 + j * 8];
			}

			for (j = 0; j < (AD_PAGE_SIZE / sizeof(int16_t) / 2); j++) {
				ad_dma_rxbuf_stereo_r[0 + j * 4] = pbuf[2 + j * 8];
				ad_dma_rxbuf_stereo_r[1 + j * 4] = pbuf[3 + j * 8];
				ad_dma_rxbuf_stereo_r[2 + j * 4] = pbuf[6 + j * 8];
				ad_dma_rxbuf_stereo_r[3 + j * 4] = pbuf[7 + j * 8];
			}

			chg_test_flag = 1;
		} else {
			for (j = 0; j < (AD_PAGE_SIZE / sizeof(int16_t) / 2); j++) {
				ad_dma_rxbuf_stereo_l[0 + j * 4] = pbuf[0 + j * 8];
				ad_dma_rxbuf_stereo_l[1 + j * 4] = pbuf[1 + j * 8];
				ad_dma_rxbuf_stereo_l[2 + j * 4] = pbuf[4 + j * 8];
				ad_dma_rxbuf_stereo_l[3 + j * 4] = pbuf[5 + j * 8];
			}

			for (j = 0; j < (AD_PAGE_SIZE / sizeof(int16_t) / 2); j++) {
				ad_dma_rxbuf_stereo_r[0 + j * 4] = pbuf[2 + j * 8];
				ad_dma_rxbuf_stereo_r[1 + j * 4] = pbuf[3 + j * 8];
				ad_dma_rxbuf_stereo_r[2 + j * 4] = pbuf[6 + j * 8];
				ad_dma_rxbuf_stereo_r[3 + j * 4] = pbuf[7 + j * 8];
			}
			chg_test_flag = 0;

			memcpy((void *)ptx_addre, (void *)ad_dma_rxbuf_stereo_l, TX_AD_PAGE_SIZE);
			//memcpy((void*)ptx_addre, (void*)ad_dma_rxbuf_stereo_r, TX_AD_PAGE_SIZE);

			audio_set_tx_page(obj, ptx_addre);
		}
	} else {
		memcpy((void *)ptx_addre, (void *)pbuf, TX_AD_PAGE_SIZE);
		audio_set_tx_page(obj, ptx_addre);
	}
	audio_set_rx_page(obj); // submit a new page for receive

}

static void setting_audio_dmic(void)
{
	uint32_t i;
	uint8_t *ptx_buf;
	printf("Start audio loop example: Use DMic\r\n");

	//DMIC pinmux
	audio_dmic_pinmux(&audio_obj, DMIC_CLK_PIN, DMIC_DATA_PIN);

	//Audio Init
	if (CONFIG_MIC_TYPE == USE_DMIC_L) {
		audio_init(&audio_obj, OUTPUT_SINGLE_EDNED, AUDIO_LEFT_DMIC, AUDIO_CODEC_2p8V); //AUDIO_LEFT_DMIC, MIC_SINGLE_EDNED
	} else if (CONFIG_MIC_TYPE == USE_DMIC_R) {
		audio_init(&audio_obj, OUTPUT_SINGLE_EDNED, AUDIO_RIGHT_DMIC, AUDIO_CODEC_2p8V); //AUDIO_RIGHT_DMIC, MIC_SINGLE_EDNED
	} else if (CONFIG_MIC_TYPE == USE_DMIC_STEREO) {
		audio_init(&audio_obj, OUTPUT_SINGLE_EDNED, AUDIO_STEREO_DMIC, AUDIO_CODEC_2p8V); //AUDIO_STEREO_DMIC, MIC_SINGLE_EDNED
	}


#if defined(CONFIG_PLATFORM_8735B)
	if (CONFIG_MIC_TYPE == USE_DMIC_STEREO) {
		audio_set_param_adv(&audio_obj, ASR_16KHZ, WL_16BIT, A_MONO, A_STEREO);
	} else {
		audio_set_param_adv(&audio_obj, ASR_16KHZ, WL_16BIT, A_MONO, A_MONO);
	}
#else
	audio_set_param(&audio_obj, ASR_16KHZ, WL_16BIT);
#endif

	audio_set_dma_buffer(&audio_obj, ad_dma_txdata, ad_dma_rxdata, AD_PAGE_SIZE, DMA_AD_PAGE_NUM);

#if defined(CONFIG_PLATFORM_8735B)
	//Init RX dma
	audio_rx_irq_handler(&audio_obj, (audio_irq_handler)audio_rx_irq, (uint32_t *)&audio_obj);

	//Init TX dma
	audio_tx_irq_handler(&audio_obj, (audio_irq_handler)audio_tx_irq, (uint32_t *)&audio_obj);
#else
	//Init RX dma
	audio_rx_irq_handler(&audio_obj, (audio_irq_handler)audio_rx_irq, (uint32_t)&audio_obj);

	//Init TX dma
	audio_tx_irq_handler(&audio_obj, (audio_irq_handler)audio_tx_irq, (uint32_t)&audio_obj);
#endif

	/* Use (DMA page count -1) because occur RX interrupt in first */
	for (i = 0; i < (DMA_AD_PAGE_NUM - 1); i++) {
		ptx_buf = audio_get_tx_page_adr(&audio_obj);
		if (ptx_buf) {
			audio_set_tx_page(&audio_obj, ptx_buf);
		}
		audio_set_rx_page(&audio_obj);
	}

	audio_mic_analog_gain(&audio_obj, ENABLE, MIC_20DB);
	audio_l_dmic_gain(&audio_obj, DMIC_BOOST_24DB);
	//audio_r_dmic_gain(&audio_obj, DMIC_BOOST_24DB);
}

static void setting_audio_amic(void)
{
	uint32_t i;
	uint8_t *ptx_buf;
	printf("Start audio loop example: Use AMic\r\n");

	//Audio Init
	audio_init(&audio_obj, OUTPUT_SINGLE_EDNED, MIC_SINGLE_EDNED, AUDIO_CODEC_2p8V);

#if defined(CONFIG_PLATFORM_8735B)
	audio_set_param_adv(&audio_obj, ASR_16KHZ, WL_16BIT, A_MONO, A_MONO);
#else
	audio_set_param(&audio_obj, ASR_16KHZ, WL_16BIT);
#endif

	audio_set_dma_buffer(&audio_obj, ad_dma_txdata, ad_dma_rxdata, AD_PAGE_SIZE, DMA_AD_PAGE_NUM);

#if defined(CONFIG_PLATFORM_8735B)
	//Init RX dma
	audio_rx_irq_handler(&audio_obj, (audio_irq_handler)audio_rx_irq, (uint32_t *)&audio_obj);

	//Init TX dma
	audio_tx_irq_handler(&audio_obj, (audio_irq_handler)audio_tx_irq, (uint32_t *)&audio_obj);
#else
	//Init RX dma
	audio_rx_irq_handler(&audio_obj, (audio_irq_handler)audio_rx_irq, (uint32_t)&audio_obj);

	//Init TX dma
	audio_tx_irq_handler(&audio_obj, (audio_irq_handler)audio_tx_irq, (uint32_t)&audio_obj);
#endif

	/* Use (DMA page count -1) because occur RX interrupt in first */
	for (i = 0; i < (DMA_AD_PAGE_NUM - 1); i++) {
		ptx_buf = audio_get_tx_page_adr(&audio_obj);
		if (ptx_buf) {
			audio_set_tx_page(&audio_obj, ptx_buf);
		}
		audio_set_rx_page(&audio_obj);
	}

	audio_mic_analog_gain(&audio_obj, ENABLE, MIC_20DB);
}

void example_audio_thread(void *param)
{
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1) && defined(CONFIG_PLATFORM_8735B)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif
	//AUDIO debug message
	/*
	DBG_INFO_MSG_ON(_DBG_SPORT_);
	DBG_WARN_MSG_ON(_DBG_SPORT_);
	DBG_ERR_MSG_ON(_DBG_SPORT_);

	DBG_INFO_MSG_ON(_DBG_AUDIO_);
	DBG_WARN_MSG_ON(_DBG_AUDIO_);
	DBG_ERR_MSG_ON(_DBG_AUDIO_);
	*/

	//Audio and Mic type setting
	if (CONFIG_MIC_TYPE >= USE_DMIC_L && CONFIG_MIC_TYPE <= USE_DMIC_STEREO) {
		setting_audio_dmic();
	} else if (CONFIG_MIC_TYPE == USE_AMIC) {
		setting_audio_amic();
	} else {
		printf("unknown mic type!!\r\n");
	}

	//Audio TRX Start
	audio_trx_start(&audio_obj);

	vTaskDelete(NULL);
}


void example_audio_loop(void)
{
	if (xTaskCreate(example_audio_thread, "example_audio_thread", 1024, (void *) NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(example_audio_thread) failed", __FUNCTION__);
	}
}