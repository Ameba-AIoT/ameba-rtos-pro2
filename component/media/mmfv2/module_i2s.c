/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include "mmf2_module.h"
#include "i2s_api.h"
#include "module_i2s.h"

#define USE_I2S_MODULE 1

//------------------------------------------------------------------------------

#define AUDIO_DMA_PAGE_NUM 4
#define AUDIO_DMA_PAGE_SIZE (640)	//64*N bytes, max: 4095. 128, 4032  

#define TX_PAGE_SIZE 	AUDIO_DMA_PAGE_SIZE //64*N bytes, max: 4095. 128, 4032 
#define TX_PAGE_NUM 	AUDIO_DMA_PAGE_NUM
#define RX_PAGE_SIZE 	AUDIO_DMA_PAGE_SIZE //64*N bytes, max: 4095. 128, 4032
#define RX_PAGE_NUM 	AUDIO_DMA_PAGE_NUM

#define TX_CACHE_DEPTH	16

#if (defined(CONFIG_PLATFORM_8735B) && (CONFIG_PLATFORM_8735B == 1))
#if IS_CUT_TEST(CONFIG_CHIP_VER)
//amebapro2 i2s pin
#define I2S_SCLK_PIN            PE_0
#define I2S_WS_PIN              PE_3
#define I2S_TX_PIN              PE_1
#define I2S_RX_PIN              PE_4
#define I2S_MCK_PIN             NC //PE_2 or NC
#define I2S_TX1_PIN             NC
#define I2S_TX2_PIN             NC
#else
#define I2S_SCLK_PIN            PD_14
#define I2S_WS_PIN              PD_17
#define I2S_TX_PIN              PD_15
#define I2S_RX_PIN              PD_18
#define I2S_MCK_PIN             NC //PD_16 or NC
#define I2S_TX1_PIN             NC
#define I2S_TX2_PIN             NC
#endif
#else
//amebapro i2s pin
#define I2S_SCLK_PIN            PE_1
#define I2S_WS_PIN              PE_0
#define I2S_TX_PIN              PE_2
#define I2S_RX_PIN              PE_5
#define I2S_MCK_PIN             NC //PE_3 or NC
#define I2S_TX1_PIN             NC
#define I2S_TX2_PIN             NC
#endif

static uint8_t dma_txdata[TX_PAGE_SIZE * TX_PAGE_NUM]__attribute__((aligned(0x20)));
static uint8_t dma_rxdata[RX_PAGE_SIZE * RX_PAGE_NUM]__attribute__((aligned(0x20)));

#define AUDIO_TX_PCM_QUEUE_LENGTH (20)

typedef struct pcm_tx_cache_s {
	xQueueHandle queue;
	uint16_t idx;
	uint8_t  buffer[AUDIO_DMA_PAGE_SIZE];	// for sw output cache handler
	uint8_t  txbuf[AUDIO_DMA_PAGE_SIZE];	// for interrupt
} pcm_tx_cache_t;

static pcm_tx_cache_t	*pcm_tx_cache = NULL;

//extern int i2s_err;
#define LOGIC_INPUT_NUM	4
static int logic_input_num = 1;		// for non-mix mode

#if LOGIC_INPUT_NUM==2
#warning ****VERY IMPORTANT : AUDIO must connect to MIMO/MISO INPUT 0 and 1 ****
// TODO : remove this limitation
#endif

#if LOGIC_INPUT_NUM!=4 && LOGIC_INPUT_NUM!=2
#error ONLY SUPPORT 4 and 2
#endif

//-------------AEC interrupt handler ------------------------------------------
#if defined(ENABLE_SPEEX_AEC) && ENABLE_SPEEX_AEC==1
#include "speex/speex_echo.h"
#include "speex/speex_preprocess.h"

static uint8_t last_tx_buf[AUDIO_DMA_PAGE_SIZE / 2];
static uint8_t last_rx_buf[AUDIO_DMA_PAGE_SIZE / 2];

#define SPEEX_SAMPLE_RATE (8000)
//#define NN (AUDIO_DMA_PAGE_SIZE/2)
#define NN (AUDIO_DMA_PAGE_SIZE/(2*2))
#define TAIL_LENGTH_IN_MILISECONDS (20)
#define TAIL (TAIL_LENGTH_IN_MILISECONDS * (SPEEX_SAMPLE_RATE / 1000) )
#endif //ENABLE_SPEEX_AEC

static int shrink_data(char *source, char *dest, int input_size, i2s_ctx_t *ctx)
{
	uint32_t *p1, *p2;
	uint32_t tmp_32;
	uint16_t *p3;
	uint16_t tmp_16;

	p1 = (uint32_t *)source;
	if (ctx->wl_shrink == 1) {
		p2 = (uint32_t *)dest;
	} else if (ctx->wl_shrink == 2) {
		p3 = (uint16_t *)dest;
	} else {
		return -1;
	}

	for (int j = 0; j < input_size / (4 * ctx->ch_shrink * ctx->sr_shrink); j++) { // Devied by 4 : byte to uint32_t
		if (ctx->wl_shrink == 1) {
			tmp_32 = p1[j * ctx->ch_shrink * ctx->sr_shrink];
			p2[j] = tmp_32;
		} else if (ctx->wl_shrink == 2) {
			tmp_16 = (p1[j * ctx->ch_shrink * ctx->sr_shrink]) >> (4 * ctx->wl_shrink);
			p3[j] = tmp_16;
		}
	}
	return input_size / (ctx->ch_shrink * ctx->sr_shrink * ctx->wl_shrink);
}

static void i2s_tx_complete(uint32_t arg, uint8_t *pbuf)
{

}

static void i2s_rx_complete(uint32_t arg, uint8_t *pbuf)
{
	i2s_ctx_t *ctx = (i2s_ctx_t *)arg;
	i2s_t *obj = (i2s_t *)ctx->i2s_obj;
	uint32_t i2s_rx_ts = mm_read_mediatime_ms_fromisr();
	// set timestamp to 1st sample
	i2s_rx_ts -= 1000 * (AUDIO_DMA_PAGE_SIZE / ctx->word_length) / ctx->sample_rate;
#if defined(ENABLE_SPEEX_AEC) && ENABLE_SPEEX_AEC==1
	if (ctx->params.enable_aec) {
		//printf("\n\renable_aec\n\r");
		int shrink_size = shrink_data(pbuf, last_rx_buf, AUDIO_DMA_PAGE_SIZE, ctx);
		if (ctx->aec_rx_done_sema) {
			xSemaphoreGiveFromISR(ctx->aec_rx_done_sema, NULL);
		}
	} else
#endif
	{
		BaseType_t xTaskWokenByReceive = pdFALSE;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		mm_context_t *mctx = (mm_context_t *)ctx->parent;
		mm_queue_item_t *output_item;

		if (mctx->output_recycle && (xQueueReceiveFromISR(mctx->output_recycle, &output_item, &xTaskWokenByReceive) == pdTRUE)) {
			//shrink_data is used for downsampling and stereo to mono
			int shrink_size = shrink_data((void *)pbuf, (void *)output_item->data_addr, RX_PAGE_SIZE, ctx);
			u8 *ptr = (u8 *)output_item->data_addr;
			output_item->size = shrink_size;
			output_item->timestamp = i2s_rx_ts;
			//if (i2s_err == 1) {
			//memset((void *)output_item->data_addr, 0, shrink_size);
			//}
			xQueueSendFromISR(mctx->output_ready, (void *)&output_item, &xHigherPriorityTaskWoken);
		}
		i2s_recv_page(obj);
		if (xHigherPriorityTaskWoken || xTaskWokenByReceive) {
			taskYIELD();
		}
	}
}

#if defined(ENABLE_SPEEX_AEC) && ENABLE_SPEEX_AEC==1
static void i2s_rx_handle_thread(void *param)
{
	i2s_ctx_t *ctx = (i2s_ctx_t *)param;
	mm_context_t *mctx = (mm_context_t *)ctx->parent;
	mm_queue_item_t *output_item;

	i2s_t *obj = (i2s_t *)ctx->i2s_obj;

	SpeexEchoState *st;
	SpeexPreprocessState *den;
	int sampleRate = SPEEX_SAMPLE_RATE;

	st = speex_echo_state_init(NN, TAIL);
	den = speex_preprocess_state_init(NN, sampleRate);
	speex_echo_ctl(st, SPEEX_ECHO_SET_SAMPLING_RATE, &sampleRate);
	speex_preprocess_ctl(den, SPEEX_PREPROCESS_SET_ECHO_STATE, st);


	while (1) {
		xSemaphoreTake(ctx->aec_rx_done_sema, portMAX_DELAY);

		uint32_t i2s_rx_ts = mm_read_mediatime_ms_fromisr();

		if (!mctx->output_recycle || (xQueueReceive(mctx->output_recycle, &output_item, 0xFFFFFFFF) != pdTRUE)) {
			// drop current frame
			// NOT SEND
			i2s_recv_page(obj);
			continue;
		}

		speex_echo_cancellation(st, (void *)(last_rx_buf), (void *)(last_tx_buf), (void *)output_item->data_addr);
		speex_preprocess_run(den, (void *)output_item->data_addr);

		output_item->size = RX_PAGE_SIZE / 2;
		output_item->timestamp = i2s_rx_ts;

		xQueueSend(mctx->output_ready, (void *)&output_item, 0xFFFFFFFF);
		//printf("\n\r1\n\r");
		i2s_recv_page(obj);
	}
}
#endif

int i2s_control(void *p, int cmd, int arg)
{
	i2s_ctx_t  *ctx = (i2s_ctx_t *)p;
	uint32_t    out_sample_rate = 0;
	uint8_t     out_word_length = 0;

	switch (cmd) {
	case CMD_I2S_SET_PARAMS:
		memcpy(&ctx->params, (void *)arg, sizeof(i2s_params_t));
		break;
	case CMD_I2S_GET_PARAMS:
		memcpy((void *)arg, &ctx->params, sizeof(i2s_params_t));
		break;
	case CMD_I2S_APPLY:
#if defined(ENABLE_SPEEX_AEC) && ENABLE_SPEEX_AEC==1
		if (ctx->params.enable_aec) {
			ctx->aec_rx_done_sema = xSemaphoreCreateBinary();
			if (!ctx->aec_rx_done_sema) {
				goto i2s_control_fail;
			}
			if (xTaskCreate(i2s_rx_handle_thread, ((const char *)"i2s_rx"), 256, (void *)ctx, tskIDLE_PRIORITY + 1, &ctx->aec_rx_task) != pdPASS) {
				rt_printf("\r\n i2s_rx_handle_thread: Create Task Error\n");
				goto i2s_control_fail;
			}
		}
#endif
		// mix mode --> LOGIC_INPUT_NUM input, non-mix mode 1 input
		if (ctx->params.mix_mode) {
			logic_input_num = LOGIC_INPUT_NUM;
		}
		pcm_tx_cache = malloc(logic_input_num * sizeof(pcm_tx_cache_t));
		if (!pcm_tx_cache) {
			rt_printf("\r\n pcm tx cache: Create Error\n");
			goto i2s_control_fail;
		}
		memset(pcm_tx_cache, 0, logic_input_num * sizeof(pcm_tx_cache_t));
		for (int i = 0; i < logic_input_num; i++) {
			memset(pcm_tx_cache[i].txbuf, 0x80, AUDIO_DMA_PAGE_SIZE);	// set to audio level 0;
			pcm_tx_cache[i].queue = xQueueCreate(AUDIO_TX_PCM_QUEUE_LENGTH, AUDIO_DMA_PAGE_SIZE);
			if (!pcm_tx_cache[i].queue) {
				goto i2s_control_fail;
			}
		}

		if (ctx->params.sample_rate == SR_8KHZ) {
			ctx->sample_rate = 8000;
		} else if (ctx->params.sample_rate == SR_16KHZ) {
			ctx->sample_rate = 16000;
		} else if (ctx->params.sample_rate == SR_32KHZ) {
			ctx->sample_rate = 32000;
		} else if (ctx->params.sample_rate == SR_44p1KHZ) {
			ctx->sample_rate = 44100;
		} else if (ctx->params.sample_rate == SR_48KHZ) {
			ctx->sample_rate = 48000;
		} else if (ctx->params.sample_rate == SR_88p2KHZ) {
			ctx->sample_rate = 88200;
		} else if (ctx->params.sample_rate == SR_96KHZ) {
			ctx->sample_rate = 96000;
		} else {
			printf("[I2S Ctrl Err]There is no proper definition of i2s sample_rate.\n\r");
			goto i2s_control_fail;
		}

		if (ctx->params.out_sample_rate == SR_8KHZ) {
			out_sample_rate = 8000;
		} else if (ctx->params.out_sample_rate == SR_16KHZ) {
			out_sample_rate = 16000;
		} else if (ctx->params.out_sample_rate == SR_32KHZ) {
			out_sample_rate = 32000;
		} else if (ctx->params.out_sample_rate == SR_44p1KHZ) {
			out_sample_rate = 44100;
		} else if (ctx->params.out_sample_rate == SR_48KHZ) {
			out_sample_rate = 48000;
		} else if (ctx->params.out_sample_rate == SR_88p2KHZ) {
			out_sample_rate = 88200;
		} else if (ctx->params.out_sample_rate == SR_96KHZ) {
			out_sample_rate = 96000;
		} else {
			printf("[I2S Ctrl Err]There is no proper definition of i2s out_sample_rate.\n\r");
			goto i2s_control_fail;
		}

		if (((float)ctx->sample_rate / (float)out_sample_rate) < 1) {
			printf("[I2S Ctrl Err]out_sample_rate less than sample_rate\n\r");
			goto i2s_control_fail;
		}
		if (ctx->sample_rate % out_sample_rate != 0) {
			printf("[I2S Ctrl Err]sample_rate must can be divied by out_sample_rate\n\r");
			goto i2s_control_fail;
		} else {
			ctx->sr_shrink = ctx->sample_rate / out_sample_rate;
		}

		if (ctx->params.word_length == WL_16b) {
			ctx->word_length = 2;
		} else if (ctx->params.word_length == WL_24b) {
			ctx->word_length = 4;    //actually transmit 32bit wide in i2s PAGE.
		} else if (ctx->params.word_length == WL_32b) {
			ctx->word_length = 4;
		} else {
			printf("[I2S Ctrl Err]There is no proper definition of i2s word_length.\n\r");
			goto i2s_control_fail;
		}

		if (ctx->params.out_word_length == WL_16b) {
			out_word_length = 2;
		} else if (ctx->params.out_word_length == WL_24b) {
			out_word_length = 4;    //actually transmit 32bit wide in i2s PAGE.
		} else if (ctx->params.out_word_length == WL_32b) {
			out_word_length = 4;
		} else {
			printf("[I2S Ctrl Err]There is no proper definition of i2s out_word_length.\n\r");
			goto i2s_control_fail;
		}

		if (((float)ctx->word_length / (float)out_word_length) < 1) {
			printf("[I2S Ctrl Err]out_word_length less than word_length\n\r");
			goto i2s_control_fail;
		}
		if (ctx->word_length % out_word_length != 0) {
			printf("[I2S Ctrl Err]word_length must can be divied by out_word_length\n\r");
			goto i2s_control_fail;
		} else {
			ctx->wl_shrink = ctx->word_length / out_word_length;
		}

		//For stereo to mono
		if ((ctx->ch_shrink = ctx->params.channel / ctx->params.out_channel) <= 0) {
			printf("[I2S Ctrl Err]Channel setting error!\n\r");
			goto i2s_control_fail;
		}

		i2s_set_param(ctx->i2s_obj, CH_STEREO, ctx->params.sample_rate, ctx->params.word_length);  // ASR_8KHZ, ASR_16KHZ //ASR_48KHZ
		i2s_set_direction(ctx->i2s_obj, I2S_DIR_RX);
		//For RX_Only
		for (int j = 0; j < RX_PAGE_NUM; j++) {
			i2s_recv_page(ctx->i2s_obj);
		}
		/* For TXRX */
		//for (int j=0;j<AUDIO_DMA_PAGE_NUM;j++) {
		//    int *ptx_buf = i2s_get_tx_page(ctx->i2s_obj);
		//   if (ptx_buf) {
		//        i2s_send_page(ctx->i2s_obj, (uint32_t*)ptx_buf);
		//    }
		//    i2s_recv_page(ctx->i2s_obj);
		//}

		i2s_enable(ctx->i2s_obj);
		break;
	}
	return 0;

i2s_control_fail:
	mm_printf("i2s_control fail\n\r");
	if (ctx->aec_rx_done_sema) {
		vSemaphoreDelete(ctx->aec_rx_done_sema);
		ctx->aec_rx_done_sema = NULL;
	}

	if (ctx->aec_rx_task) {
		vTaskDelete(ctx->aec_rx_task);
		ctx->aec_rx_task = NULL;
	}

	if (pcm_tx_cache) {
		for (int i = 0; i < logic_input_num; i++)
			if (pcm_tx_cache[i].queue) {
				vQueueDelete(pcm_tx_cache[i].queue);
			}
		free(pcm_tx_cache);
	}
	return -1;
}

int i2s_handle(void *p, void *input, void *output)
{
	//audio_ctx_t* ctx = (audio_ctx_t*)p;
	mm_queue_item_t *input_item = (mm_queue_item_t *)input;

	uint8_t *input_data = (uint8_t *)input_item->data_addr;

	(void)output;

	int cache_idx = input_item->in_idx > (logic_input_num - 1) ? 0 : input_item->in_idx;
	pcm_tx_cache_t *cache = &pcm_tx_cache[cache_idx];

	for (int i = 0; i < input_item->size; i++) {
		cache->buffer[cache->idx++] = input_data[i];
		if (cache->idx == AUDIO_DMA_PAGE_SIZE) {
			if (xQueueSend(cache->queue, cache->buffer, 1000) != pdTRUE) {
				printf("fail to send tx queue\r\n");
			}
			cache->idx = 0;
		}
	}
	return 0;
}

void *i2s_destroy(void *p)
{
	i2s_ctx_t *ctx = (i2s_ctx_t *)p;

	if (pcm_tx_cache) {
		for (int i = 0; i < logic_input_num; i++)
			if (pcm_tx_cache[i].queue) {
				vQueueDelete(pcm_tx_cache[i].queue);
			}
		free(pcm_tx_cache);
	}
	//if(pcm_tx_cache) free(pcm_tx_cache);
	//if(audio_tx_pcm_queue) vQueueDelete(audio_tx_pcm_queue);

	if (ctx && ctx->aec_rx_done_sema) {
		vSemaphoreDelete(ctx->aec_rx_done_sema);
	}
	if (ctx && ctx->aec_rx_task) {
		vTaskDelete(ctx->aec_rx_task);
	}
	if (ctx && ctx->i2s_obj) {
		free(ctx->i2s_obj);
	}
	if (ctx) {
		free(ctx);
	}
	return NULL;
}

void *i2s_create(void *parent)
{
	i2s_ctx_t *ctx = malloc(sizeof(i2s_ctx_t));
	if (!ctx) {
		printf("i2s contex malloc error\n\r");
		return NULL;
	}
	ctx->parent = parent;

	ctx->i2s_obj = malloc(sizeof(i2s_t));
	if (!ctx->i2s_obj)	{
		goto i2s_create_fail;
	}
	memset(ctx->i2s_obj, 0, sizeof(i2s_t));

#if (defined(CONFIG_PLATFORM_8735B) && (CONFIG_PLATFORM_8735B == 1))
	// I2S init
	i2s_init(ctx->i2s_obj, I2S_SCLK_PIN, I2S_WS_PIN, I2S_TX_PIN, I2S_RX_PIN, I2S_MCK_PIN);
#else
	// I2S init
	i2s_init(ctx->i2s_obj, I2S_SCLK_PIN, I2S_WS_PIN, I2S_TX_PIN, I2S_RX_PIN, I2S_MCK_PIN, I2S_TX1_PIN, I2S_TX2_PIN);
#endif

	// Init TX,RX dma
	i2s_set_dma_buffer(ctx->i2s_obj, (char *)dma_txdata, (char *)dma_rxdata, AUDIO_DMA_PAGE_NUM, AUDIO_DMA_PAGE_SIZE);
	i2s_rx_irq_handler(ctx->i2s_obj, (i2s_irq_handler)i2s_rx_complete, (uint32_t)ctx);
	i2s_tx_irq_handler(ctx->i2s_obj, (i2s_irq_handler)i2s_tx_complete, (uint32_t)ctx);

	return ctx;

i2s_create_fail:

	i2s_destroy((void *)ctx);
	return NULL;
}

void *i2s_new_item(void *p)
{
	return malloc(AUDIO_DMA_PAGE_SIZE);
}

void *i2s_del_item(void *p, void *d)
{
	if (d) {
		free(d);
	}
	return NULL;
}

mm_module_t i2s_module = {
	.create = i2s_create,
	.destroy = i2s_destroy,
	.control = i2s_control,
	.handle = i2s_handle,

	.new_item = i2s_new_item,
	.del_item = i2s_del_item,
	.rsz_item = NULL,

	.output_type = MM_TYPE_ADSP,     // no output
	.module_type = MM_TYPE_ASRC,
	.name = "I2S"
};
