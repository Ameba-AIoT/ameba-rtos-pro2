/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "platform_opts.h"

#include "mmf2_link.h"
#include "mmf2_siso.h"

#include "avcodec.h"
#include "module_video.h"
#include "module_demuxer.h"
#include "module_kvs_webrtc.h"
#include "example_kvs_webrtc_playback_mmf.h"

static mm_context_t *demuxer_ctx	        = NULL;
static mm_context_t *kvs_webrtc_v1_a1_ctx   = NULL;

static mm_siso_t *siso_demuxer_webrtc_v1_a1	= NULL;

static demuxer_params_t demuxer_params = {
	.start_time     = 10000,
	.stream_type    = STREAM_ALL,
	.loop_mode      = 1, //0 no loop, 1 loop

	.record_file_name = "AmebaPro_recording.mp4",
	.mem_total_size = 1 * 1024 * 1024,
	.mem_block_size = 128
};

#include "wifi_conf.h"
#include "lwip_netconf.h"
#define wifi_wait_time 500 //Here we wait 5 second to wiat the fast connect 
static void wifi_common_init(void)
{
	uint32_t wifi_wait_count = 0;

	while (!((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) && (*(u32 *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
		vTaskDelay(10);
		wifi_wait_count++;
		if (wifi_wait_count == wifi_wait_time) {
			printf("\r\nuse ATW0, ATW1, ATWC to make wifi connection\r\n");
			printf("wait for wifi connection...\r\n");
		}
	}
}

void example_kvs_webrtc_playback_mmf_thread(void *param)
{
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(2048);
#endif

	if (!voe_boot_fsc_status()) {
		wifi_common_init();
	}

	demuxer_ctx = mm_module_open(&demuxer_module);
	if (demuxer_ctx) {
		mm_module_ctrl(demuxer_ctx, CMD_DEMUXER_SET_PARAMS, (int)&demuxer_params);
		mm_module_ctrl(demuxer_ctx, MM_CMD_SET_QUEUE_LEN, 12);
		mm_module_ctrl(demuxer_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(demuxer_ctx, CMD_DEMUXER_INIT_MEM_POOL, 0);
		mm_module_ctrl(demuxer_ctx, CMD_DEMUXER_OPEN, 0);
		printf("CMD_DEMUXER_STREAM_START\r\n");
		mm_module_ctrl(demuxer_ctx, CMD_DEMUXER_STREAM_START, 0);
	} else {
		rt_printf("DEMUXER open fail\n\r");
		goto example_kvs_webrtc_playback_mmf;
	}

	kvs_webrtc_v1_a1_ctx = mm_module_open(&kvs_webrtc_module);
	if (kvs_webrtc_v1_a1_ctx) {
		mm_module_ctrl(kvs_webrtc_v1_a1_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(kvs_webrtc_v1_a1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(kvs_webrtc_v1_a1_ctx, CMD_KVS_WEBRTC_SET_APPLY, 0);
	} else {
		rt_printf("KVS open fail\n\r");
		goto example_kvs_webrtc_playback_mmf;
	}

	siso_demuxer_webrtc_v1_a1 = siso_create();
	if (siso_demuxer_webrtc_v1_a1) {
		siso_ctrl(siso_demuxer_webrtc_v1_a1, MMIC_CMD_ADD_INPUT, (uint32_t)demuxer_ctx, 0);
		siso_ctrl(siso_demuxer_webrtc_v1_a1, MMIC_CMD_ADD_OUTPUT, (uint32_t)kvs_webrtc_v1_a1_ctx, 0);
		siso_start(siso_demuxer_webrtc_v1_a1);
	} else {
		rt_printf("siso_demuxer_webrtc_v1_a1 open fail\n\r");
		goto example_kvs_webrtc_playback_mmf;
	}
	rt_printf("siso_demuxer_webrtc_v1_a1 started\n\r");

example_kvs_webrtc_playback_mmf:

	// TODO: exit condition or signal
	while (1) {
		vTaskDelay(1000);
	}
}

void example_kvs_webrtc_playback_mmf(void)
{
	/*user can start their own task here*/
	if (xTaskCreate(example_kvs_webrtc_playback_mmf_thread, ((const char *)"example_kvs_webrtc_playback_mmf_thread"), 4096, NULL, tskIDLE_PRIORITY + 1,
					NULL) != pdPASS) {
		printf("\r\n example_kvs_webrtc_playback_mmf_thread: Create Task Error\n");
	}
}