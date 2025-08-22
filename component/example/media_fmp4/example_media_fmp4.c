/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "platform_opts.h"
#include "log_service.h"

#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_miso.h"

#include "module_video.h"
#include "module_audio.h"
#include "module_aac.h"
#include "module_fmp4.h"

#include "mmf2_pro2_video_config.h"
#include "vfs.h"

/*****************************************************************************
* ISP channel : 0
* Video type  : H264
*****************************************************************************/

#define V1_CHANNEL 0
#define V1_RESOLUTION VIDEO_HD
#define V1_FPS 30
#define V1_GOP 30
#define V1_BPS 512*1024
#define V1_RCMODE 1 // 1: CBR, 2: VBR

#define VIDEO_TYPE VIDEO_H264
#define VIDEO_CODEC AV_CODEC_ID_H264

#if V1_RESOLUTION == VIDEO_VGA
#define V1_WIDTH	640
#define V1_HEIGHT	480
#elif V1_RESOLUTION == VIDEO_HD
#define V1_WIDTH	1280
#define V1_HEIGHT	720
#elif V1_RESOLUTION == VIDEO_FHD
#define V1_WIDTH	1920
#define V1_HEIGHT	1080
#endif

static mm_context_t *video_v1_ctx       = NULL;
static mm_context_t *audio_ctx          = NULL;
static mm_context_t *aac_ctx            = NULL;
static mm_context_t *fmp4_ctx           = NULL;

static mm_siso_t *siso_audio_aac        = NULL;
static mm_miso_t *miso_video_aac_fmp4   = NULL;

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
	.use_static_addr = 1
};

#if !USE_DEFAULT_AUDIO_SET
static audio_params_t audio_params;
static void audio_params_customized_setting(void)
{
	memcpy(&audio_params, &default_audio_params, sizeof(audio_params_t));
}
#endif

static aac_params_t aac_params = {
	.sample_rate = 8000,
	.channel = 1,
	.trans_type = AAC_TYPE_ADTS,
	.object_type = AAC_AOT_LC,
	.bitrate = 32000,

	.mem_total_size = 10 * 1024,
	.mem_block_size = 128,
	.mem_frame_size = 1024
};

static void example_media_fmp4_thread(void *param)
{
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(2048);
#endif

	printf("\r\n=== Example fmp4 main ===\r\n");

	//init virtual filesystem
	vfs_init(NULL);
	vfs_user_register("sd", VFS_FATFS, VFS_INF_SD);
	vfs_user_register("ram", VFS_FATFS, VFS_INF_RAM);

	int voe_heap_size = video_voe_presetting(1, V1_WIDTH, V1_HEIGHT, V1_BPS, 0,
						0, 0, 0, 0, 0,
						0, 0, 0, 0, 0,
						0, 0, 0);

	printf("\r\n voe heap size = %d\r\n", voe_heap_size);

	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, V1_FPS * 3);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto mmf2_exmaple_av_fmp4_fail;
	}

	audio_ctx = mm_module_open(&audio_module);
	if (audio_ctx) {
#if !USE_DEFAULT_AUDIO_SET
		audio_params_customized_setting();
		mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
#endif
		mm_module_ctrl(audio_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(audio_ctx, CMD_AUDIO_APPLY, 0);
	} else {
		printf("audio open fail\n\r");
		goto mmf2_exmaple_av_fmp4_fail;
	}

	aac_ctx = mm_module_open(&aac_module);
	if (aac_ctx) {
		mm_module_ctrl(aac_ctx, CMD_AAC_SET_PARAMS, (int)&aac_params);
		mm_module_ctrl(aac_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(aac_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(aac_ctx, CMD_AAC_INIT_MEM_POOL, 0);
		mm_module_ctrl(aac_ctx, CMD_AAC_APPLY, 0);
	} else {
		printf("aac open fail\n\r");
		goto mmf2_exmaple_av_fmp4_fail;
	}

	fmp4_ctx = mm_module_open(&fmp4_module);
	if (fmp4_ctx) {
		mm_module_ctrl(fmp4_ctx, CMD_FMP4_SET_WIDTH, (int)video_v1_params.width);
		mm_module_ctrl(fmp4_ctx, CMD_FMP4_SET_HEIGHT, (int)video_v1_params.height);
		mm_module_ctrl(fmp4_ctx, CMD_FMP4_SET_FILENAME, (int)"ram:/fmp4_tmp.mp4");
		if (mm_module_ctrl(fmp4_ctx, CMD_FMP4_FILE_OPEN, 0) < 0) { // need to check if fail to open a file in ram
			printf("Fail to open file\r\n");
			goto mmf2_exmaple_av_fmp4_fail;
		}
	} else {
		printf("fmp4 open fail\n\r");
		goto mmf2_exmaple_av_fmp4_fail;
	}

	siso_audio_aac = siso_create();
	if (siso_audio_aac) {
		siso_ctrl(siso_audio_aac, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
		siso_ctrl(siso_audio_aac, MMIC_CMD_ADD_OUTPUT, (uint32_t)aac_ctx, 0);
		siso_ctrl(siso_audio_aac, MMIC_CMD_SET_STACKSIZE, 44 * 1024, 0);
		siso_start(siso_audio_aac);
	} else {
		printf("siso_audio_aac open fail\n\r");
		goto mmf2_exmaple_av_fmp4_fail;
	}

	miso_video_aac_fmp4 = miso_create();
	if (miso_video_aac_fmp4) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		miso_ctrl(miso_video_aac_fmp4, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		miso_ctrl(miso_video_aac_fmp4, MMIC_CMD_ADD_INPUT0, (uint32_t)video_v1_ctx, 0);
		miso_ctrl(miso_video_aac_fmp4, MMIC_CMD_ADD_INPUT1, (uint32_t)aac_ctx, 0);
		miso_ctrl(miso_video_aac_fmp4, MMIC_CMD_ADD_OUTPUT, (uint32_t)fmp4_ctx, 0);
		miso_start(miso_video_aac_fmp4);
	} else {
		printf("miso_video_aac_fmp4 open fail\n\r");
		goto mmf2_exmaple_av_fmp4_fail;
	}
	printf("miso_video_aac_fmp4 started\n\r");

	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0

	// record 15s fmp4 data to RAM
	int record_duration = 15;
	TickType_t time_start = xTaskGetTickCount();
	while ((xTaskGetTickCount() - time_start) < record_duration * 1000) {
		printf("Record fmp4 to RAM file system: %.2f s\r\n", (float)(xTaskGetTickCount() - time_start) / 1000);
		vTaskDelay(100);
	}

	// Pause Linker
	miso_pause(miso_video_aac_fmp4, MM_OUTPUT);
	siso_pause(siso_audio_aac);

	// Stop module
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, V1_CHANNEL);
	mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_TRX, 0);
	mm_module_ctrl(aac_ctx, CMD_AAC_STOP, 0);

	// Delete linker
	miso_delete(miso_video_aac_fmp4);
	siso_delete(siso_audio_aac);

	// close the ram file and add "mfra box" at the end
	mm_module_ctrl(fmp4_ctx, CMD_FMP4_FILE_CLOSE, 0);

	// Close module
	mm_module_close(video_v1_ctx);
	mm_module_close(audio_ctx);
	mm_module_close(aac_ctx);
	mm_module_close(fmp4_ctx);

	// calculate the file size in ram
	FILE *ram_file = fopen("ram:/fmp4_tmp.mp4", "r");
	if (ram_file == NULL) {
		printf("Fail to open RAM file\r\n");
		goto mmf2_exmaple_av_fmp4_fail;
	}
	fseek(ram_file, 0, SEEK_END);
	int fmp4_size = ftell(ram_file);
	printf("fmp4 size in ram = %d\r\n", fmp4_size);
	fseek(ram_file, 0, SEEK_SET);

	// read the fmp4 from ram to a buffer
	uint8_t *pBuf_ramFile = (uint8_t *)malloc(fmp4_size);
	if (pBuf_ramFile == NULL) {
		printf("Fail to allocate memory...\r\n");
		goto mmf2_exmaple_av_fmp4_fail;
	}
	fread(pBuf_ramFile, fmp4_size, 1, ram_file);
	fclose(ram_file);

	// open a file in sd card for fmp4 verification
	FILE *sd_file = fopen("sd:/fmp4_in_sd_out.mp4", "wb+");
	if (sd_file == NULL) {
		printf("Fail to open SD file\r\n");
		goto mmf2_exmaple_av_fmp4_fail;
	}
	// write the file from buffer to sd card
	printf("\r\nWriting fmp4 to sd card......");
	fwrite(pBuf_ramFile, fmp4_size, 1, sd_file);
	printf("\r\nWrite fmp4 to sd card done.");

	// close the file and close the sd card
	fclose(sd_file);
	free(pBuf_ramFile);
	printf("\r\nRecord fmp4 %ds ok.", record_duration);

	// deinit virtual filesystem
	vfs_user_unregister("sd", VFS_FATFS, VFS_INF_SD);
	vfs_user_unregister("ram", VFS_FATFS, VFS_INF_RAM);
	vfs_deinit(NULL);


mmf2_exmaple_av_fmp4_fail:

	vTaskDelete(NULL);
}

void example_media_fmp4(void)
{
	if (xTaskCreate(example_media_fmp4_thread, ((const char *)"example_media_fmp4_thread"), 4096, NULL, 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(example_media_fmp4_thread) failed\n\r", __FUNCTION__);
	}
	return;
}