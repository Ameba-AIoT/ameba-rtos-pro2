#include "platform_opts.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "wifi_conf.h"
#include "log_service.h"

#include "module_video.h"
#include "mmf2_pro2_video_config.h"
#include "sensor.h"
#include "example_media_video_to_storage.h"

#include <stdio.h>
#include "vfs.h"

#include "avcodec.h"
#include "mp4_muxer.h"

#include "audio_api.h"
#include "avcodec.h"

#define V1_CHANNEL 0
#if USE_SENSOR == SENSOR_GC4653
#define V1_RESOLUTION VIDEO_2K
#define V1_FPS 15
#define V1_GOP 15
#else
#define V1_RESOLUTION VIDEO_FHD
#define V1_FPS 30
#define V1_GOP 30
#endif
#define V1_BPS 2*1024*1024
#define V1_RCMODE 2 // 1: CBR, 2: VBR

#define USE_H265 0

#if USE_H265
#include "sample_h265.h"
#define VIDEO_TYPE VIDEO_HEVC
#define VIDEO_CODEC AV_CODEC_ID_H265
#else
#include "sample_h264.h"
#define VIDEO_TYPE VIDEO_H264
#define VIDEO_CODEC AV_CODEC_ID_H264
#endif

#if V1_RESOLUTION == VIDEO_VGA
#define V1_WIDTH	640
#define V1_HEIGHT	480
#elif V1_RESOLUTION == VIDEO_HD
#define V1_WIDTH	1280
#define V1_HEIGHT	720
#elif V1_RESOLUTION == VIDEO_FHD
#define V1_WIDTH	1920
#define V1_HEIGHT	1080
#elif V1_RESOLUTION == VIDEO_2K
#define V1_WIDTH	2560
#define V1_HEIGHT	1440
#endif

static video_params_t video_v1_params = {
	.stream_id = V1_CHANNEL,
	.type = VIDEO_TYPE,
	.resolution = V1_RESOLUTION,
	.width = V1_WIDTH,
	.height = V1_HEIGHT,
	.bps = V1_BPS,
	.fps = V1_FPS,
	.gop = V1_GOP,
	.rc_mode = V1_RCMODE,
	.direct_output = 1
};

static xQueueHandle muxer_queue;
static xQueueHandle audio_buf_recycle;

#define VIDEO_IDLE  0x00
#define VIDEO_START 0X01
#define VIDEO_WRITE 0X02
#define VIDEO_STOP  0x03
#define VIDEO_QUIT  0x04

static int video_status = 0;
static int video_record = VIDEO_IDLE;

#define AD_PAGE_SIZE 320 //20ms
#define TX_AD_PAGE_SIZE AD_PAGE_SIZE
#define RX_AD_PAGE_SIZE AD_PAGE_SIZE
#define DMA_AD_PAGE_NUM 4

#define MUXER_QUEUE_DEPTH     (20)
#define AUDIO_BUF_QUEUE_DEPTH (20)

typedef struct {
	uint32_t data_addr;
	uint32_t type;
	uint32_t timestamp;
	uint32_t size;
	uint32_t channel;
} mp4_muxer_ctx;

static uint32_t audio_buf_temp[AUDIO_BUF_QUEUE_DEPTH];

static audio_t audio_obj;
static uint8_t ad_dma_txdata[TX_AD_PAGE_SIZE * DMA_AD_PAGE_NUM]__attribute__((aligned(0x20)));
static uint8_t ad_dma_rxdata[RX_AD_PAGE_SIZE * DMA_AD_PAGE_NUM]__attribute__((aligned(0x20)));

static void audio_rx_irq(uint32_t arg, uint8_t *pbuf)
{
	audio_t *obj = (audio_t *)arg;
	BaseType_t xTaskWokenByReceive = pdFALSE;
	BaseType_t xHigherPriorityTaskWoken;

	mp4_muxer_ctx ctx;
	uint32_t timestamp = xTaskGetTickCountFromISR();

	if (audio_get_rx_error_cnt(obj) != 0x00) {
		dbg_printf("rx page error !!! \r\n");
	}

	int is_output_ready = 0;
	unsigned char *buf;
	is_output_ready = xQueueReceiveFromISR(audio_buf_recycle, &buf, &xTaskWokenByReceive) == pdTRUE;
	if (is_output_ready) {
		memcpy((void *)buf, (void *)pbuf, TX_AD_PAGE_SIZE);
		ctx.data_addr = (uint32_t)buf;
		ctx.channel = 1;
		ctx.size = TX_AD_PAGE_SIZE;
		ctx.timestamp = timestamp;
		ctx.type = AV_CODEC_ID_PCMA;
		xQueueSendFromISR(muxer_queue, &ctx, &xHigherPriorityTaskWoken);
	}

	audio_set_rx_page(obj); // submit a new page for receive

	if (xHigherPriorityTaskWoken || xTaskWokenByReceive) {
		taskYIELD();
	}
}

static void setting_audio_amic(void)
{
	uint32_t i;

	printf("Start audio loop example: Use AMic\r\n");

	//Audio Init
	audio_init(&audio_obj, OUTPUT_SINGLE_EDNED, MIC_SINGLE_EDNED, AUDIO_CODEC_2p8V);

	audio_set_param(&audio_obj, ASR_8KHZ, WL_16BIT);

	audio_set_dma_buffer(&audio_obj, ad_dma_txdata, ad_dma_rxdata, AD_PAGE_SIZE, DMA_AD_PAGE_NUM);

	//Init RX dma
	audio_rx_irq_handler(&audio_obj, (audio_irq_handler)audio_rx_irq, (uint32_t *)&audio_obj);

	/* Use (DMA page count -1) because occur RX interrupt in first */
	for (i = 0; i < (DMA_AD_PAGE_NUM - 1); i++) {
		audio_set_rx_page(&audio_obj);
	}

	audio_mic_analog_gain(&audio_obj, ENABLE, MIC_20DB);

	audio_rx_start(&audio_obj);
}

static void video_output_cb(void *param1, void  *param2, uint32_t arg)
{
	enc2out_t *enc2out = (enc2out_t *)param1;
	mp4_muxer_ctx ctx;
	uint32_t timestamp = xTaskGetTickCount();
	video_status = 1;
	if ((enc2out->codec & (CODEC_H264 | CODEC_HEVC | CODEC_JPEG)) != 0) {
		dcache_invalidate_by_addr((uint32_t *)enc2out->enc_addr, enc2out->enc_len);
		ctx.data_addr = (uint32_t)enc2out->enc_addr;
		ctx.channel = enc2out->ch;
		ctx.size = enc2out->enc_len;
		ctx.timestamp = timestamp;
		ctx.type = AV_CODEC_ID_H264;
		if (xQueueSend(muxer_queue, (void *)&ctx, 10) != pdTRUE) {
			video_encbuf_release(enc2out->ch, enc2out->codec, enc2out->enc_len);
		}
	} else if ((enc2out->codec & (CODEC_NV12 | CODEC_RGB | CODEC_NV16)) != 0) {
		dcache_invalidate_by_addr((uint32_t *)enc2out->isp_addr, enc2out->enc_len);
	}
	video_status = 0;
}

static void MP4_START(void *arg)
{
	if (video_record == VIDEO_IDLE) {
		printf("MP4 START\r\n");
		video_record = VIDEO_START;
	} else {
		printf("MP4 IS NOT IDLE\r\n");
	}
}

static void MP4_STOP(void *arg)
{
	if (video_record == VIDEO_WRITE) {
		printf("MP4 STOP\r\n");
		video_record = VIDEO_STOP;
	} else {
		printf("MP4 IS NOT WRITING\r\n");
	}
}

static void MP4_QUIT(void *arg)
{
	if (video_record == VIDEO_WRITE) {
		printf("Please stop the record first\r\n");
	} else if (video_record == VIDEO_IDLE) {
		video_record = VIDEO_QUIT;
		printf("RECORD QUIT\r\n");
	}
}

log_item_t at_mp4_recorder_items[ ] = {
	{"MSTA", MP4_START,},
	{"MSTO", MP4_STOP,},
	{"MQUI", MP4_QUIT,},
};

static void example_media_video_to_storage_thread(void *param)
{
	int iq_addr, sensor_addr, ret;

	unsigned char *buf = NULL;
	unsigned char *audio_buf = malloc(AD_PAGE_SIZE / 2);

	int count = 0;

#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(2048);
#endif

	log_service_add_table(at_mp4_recorder_items, sizeof(at_mp4_recorder_items) / sizeof(at_mp4_recorder_items[0]));

	//////////////////////////////////MP4 INIT//////////////////////////////////////////
	mp4_muxer_ctx ctx;

	pmp4_context mp4_ctx = (pmp4_context) malloc(sizeof(mp4_context));
	mp4_params_t mp4_params;
	memset(&mp4_params, 0x00, sizeof(mp4_params_t));
	memset(mp4_ctx, 0x00, sizeof(mp4_context));
	mp4_params.vfs_format_enable = 1;//Enable the vfs format
	mp4_params.use_self_file_name = 1;
	mp4_params.width = video_v1_params.width;
	mp4_params.height = video_v1_params.height;
	mp4_params.fps = video_v1_params.fps;
	mp4_params.gop = video_v1_params.gop;
	mp4_params.record_length = 10;
	mp4_params.record_type = STORAGE_ALL;
	mp4_params.mp4_audio_format = AUDIO_ALAW;
	mp4_params.sample_rate = 8000;
	mp4_params.channel = 1;
	mp4_params.record_file_num = 1;
	mp4_params.mp4_audio_duration = 20;
	mp4_params.fatfs_buf_size = 256 * 1024;
	sprintf(mp4_params.record_file_name, "sd:/AmebaPro2.mp4");
	printf("mp4_params.record_file_name %s\r\n", mp4_params.record_file_name);
	muxer_queue = xQueueCreate(MUXER_QUEUE_DEPTH, sizeof(mp4_muxer_ctx));

	xQueueReset(muxer_queue);
	//////////////////////////////////STORAGE INIT//////////////////////////////////////////
	vfs_init(NULL);
	if (vfs_user_register("sd", VFS_FATFS, VFS_INF_SD) < 0) {
		printf("The vfs is wrong\r\n");
		goto EXIT;
	}
	//////////////////////////////////VIDEO INIT//////////////////////////////////////////
	int voe_heap_size = video_voe_presetting(1, V1_WIDTH, V1_HEIGHT, V1_BPS, 0,
						0, 0, 0, 0, 0,
						0, 0, 0, 0, 0,
						0, 0, 0);

	int sensor_id_value = 0;
	for (int i = 0; i < SENSOR_MAX; i++) {
		if (sen_id[i] == USE_SENSOR) {
			sensor_id_value = i;
			break;
		}
	}
	voe_get_sensor_info(sensor_id_value, &iq_addr, &sensor_addr);
	//voe_get_sensor_info(USE_SENSOR, &iq_addr, &sensor_addr);

	video_init(iq_addr, sensor_addr);

	ret = video_open(&video_v1_params, video_output_cb, NULL);

	if (ret < 0) {
		printf("Please check sensor fisrt, the ID is %d\r\n", USE_SENSOR);
		while (1) {
			vTaskDelay(100);
		}
	}

	//////////////////////////////////AUDIO INIT//////////////////////////////////////////

	audio_buf_recycle = xQueueCreate(AUDIO_BUF_QUEUE_DEPTH, sizeof(unsigned char *));

	xQueueReset(audio_buf_recycle);

	for (int i = 0; i < AUDIO_BUF_QUEUE_DEPTH; i++) {
		unsigned char *ptr = malloc(AD_PAGE_SIZE);
		memset(ptr, 0x00, AD_PAGE_SIZE);
		audio_buf_temp[i] = (uint32_t)ptr;
		if (xQueueSend(audio_buf_recycle, (void *)&ptr, 10) != pdPASS) {
			printf("It can't send the message\r\n");
		}
	}

	setting_audio_amic();

	printf("MSTA ->START RECORD, MSTO ->STOP RECORD, MQUI ->QUIT RECORD\r\n");
	//////////////////////////////////STORAGE PROCEDURE//////////////////////////////////////////
	while (1) {
		if (xQueueReceive(muxer_queue, (void *)&ctx, 10) != pdTRUE) {
			continue;
		}
		buf = (unsigned char *)ctx.data_addr;
		if ((video_record == VIDEO_IDLE) || (video_record == VIDEO_QUIT)) {
			if (ctx.type == AV_CODEC_ID_PCMA) {
				xQueueSend(audio_buf_recycle, &buf, 10);
			} else {
				video_encbuf_release(ctx.channel, ctx.type, ctx.size);
			}
			if (video_record == VIDEO_QUIT) {
				printf("EXIT RECORD\r\n");
				break;
			}
		} else {
			if (video_record == VIDEO_START) {
				count++;
				memset(mp4_params.record_file_name, 0x00, sizeof(mp4_params.record_file_name));
				sprintf(mp4_params.record_file_name, "%s_%d.mp4", "sd:/AmebaPro2", count);
				mp4_muxer_open(mp4_ctx, mp4_params);
				video_record = VIDEO_WRITE;
			}
			if (ctx.type == AV_CODEC_ID_PCMA) {
				extern uint8_t encodeA(short pcm_val);
				short *input_buf = (short *)ctx.data_addr;
				for (int i = 0; i <  AD_PAGE_SIZE / sizeof(short); i++) { //For G711 encode
					audio_buf[i] = encodeA(input_buf[i]);
				}
				ret = mp4_muxer_write_audio(mp4_ctx, audio_buf, ctx.size / 2, AV_CODEC_ID_PCMA, ctx.timestamp);
				xQueueSend(audio_buf_recycle, &buf, 10);
			} else {
				ret = mp4_muxer_write_video(mp4_ctx, buf, ctx.size, AV_CODEC_ID_H264, ctx.timestamp);
				if ((ret == WRITE_MP4_DONE) || (video_record == VIDEO_STOP) || (ret == WRITE_MP4_ERROR)) {
					mp4_muxer_close(mp4_ctx);
					video_encbuf_release(ctx.channel, ctx.type, ctx.size);
					video_record = VIDEO_IDLE;
				} else {
					video_encbuf_release(ctx.channel, ctx.type, ctx.size);
				}
			}
		}
	}

	while (video_status) {
		vTaskDelay(1);
	}
	video_close(video_v1_params.stream_id);
	audio_rx_stop(&audio_obj);
	vTaskDelay(100);
	audio_deinit(&audio_obj);
	for (int i = 0; i < AUDIO_BUF_QUEUE_DEPTH; i++) {
		unsigned char *ptr = (unsigned char *)audio_buf_temp[i];
		if (ptr) {
			free(ptr);
		}
	}
	xQueueReset(audio_buf_recycle);
	xQueueReset(muxer_queue);
	vQueueDelete(audio_buf_recycle);
	vQueueDelete(muxer_queue);
EXIT:
	vfs_user_unregister("sd", VFS_FATFS, VFS_INF_SD);
	vTaskDelete(NULL);
}

void example_media_video_to_storage(void)
{
	if (xTaskCreate(example_media_video_to_storage_thread, ((const char *)"example_media_video_to_storage_thread"), 1024, NULL, tskIDLE_PRIORITY + 2,
					NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(example_media_video_to_storage_thread) failed", __FUNCTION__);
	}
}

