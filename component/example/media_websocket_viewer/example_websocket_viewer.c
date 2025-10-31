/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "module_video.h"
#include "module_web_viewer.h"
#include "mmf2_pro2_video_config.h"
#include "log_service.h"
#include "sensor.h"

/*****************************************************************************
* ISP channel : 0
* Video type  : H264/HEVC
*****************************************************************************/

#define V1_CHANNEL 0
#define V1_BPS 2*1024*1024
#define V1_RCMODE 2 // 1: CBR, 2: VBR

#define VIDEO_TYPE VIDEO_H264
#define VIDEO_CODEC AV_CODEC_ID_H264

static mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *wview_v1_ctx			= NULL;
static mm_siso_t *siso_video_wview_v1		= NULL;

static video_params_t video_v1_params = {
	.stream_id = V1_CHANNEL,
	.type = VIDEO_TYPE,
	.bps = V1_BPS,
	.rc_mode = V1_RCMODE,
	.use_static_addr = 1,
	.level = VCENC_H264_LEVEL_4,
	.profile = VCENC_H264_BASE_PROFILE,
	.cavlc = 1 /* [0,1] H.264 entropy coding mode, 0 for CAVLC, 1 for CABAC */
};
#define wifi_wait_time 500 //Here we wait 5 second to wiat the fast connect
//------------------------------------------------------------------------------
// common code for network connection
//------------------------------------------------------------------------------
#include "wifi_conf.h"
#include "lwip_netconf.h"

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

static void mmf2_video_websocket_viewer(void *param)
{
	printf("hello world\r\n");
	wifi_common_init();
	/*sensor capacity check & video parameter setting*/
	video_v1_params.resolution = VIDEO_FHD;
	video_v1_params.width = sensor_params[USE_SENSOR].sensor_width;
	video_v1_params.height = sensor_params[USE_SENSOR].sensor_height;
	video_v1_params.fps = sensor_params[USE_SENSOR].sensor_fps;
	video_v1_params.gop = sensor_params[USE_SENSOR].sensor_fps;
#if (USE_UPDATED_VIDEO_HEAP == 0)
	int voe_heap_size = video_voe_presetting(1, video_v1_params.width, video_v1_params.height, V1_BPS, 0,
						0, 0, 0, 0, 0,
						0, 0, 0, 0, 0,
						0, 0, 0);
#else
	int voe_heap_size = video_voe_presetting_by_params(&video_v1_params, 0, NULL, 0, NULL, 0, NULL);
#endif
	printf("\r\n voe heap size = %d\r\n", voe_heap_size);
	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, video_v1_params.fps * 3);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		rt_printf("video open fail\n\r");
		goto mmf2_video_web_viewer_fail;
	}
	encode_rc_parm_t rc_parm;
	rc_parm.minQp = 15;
	rc_parm.maxQp = 35;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_RCPARAM, (int)&rc_parm);

	wview_v1_ctx = mm_module_open(&websocket_viewer_module);
	if (wview_v1_ctx) {
		rt_printf("web viewer open success\n\r");
	} else {
		rt_printf("web viewer open fail\n\r");
		goto mmf2_video_web_viewer_fail;
	}
	siso_video_wview_v1 = siso_create();

	if (siso_video_wview_v1) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_video_wview_v1, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_video_wview_v1, MMIC_CMD_ADD_INPUT, (uint32_t)video_v1_ctx, 0);
		siso_ctrl(siso_video_wview_v1, MMIC_CMD_ADD_OUTPUT, (uint32_t)wview_v1_ctx, 0);
		siso_start(siso_video_wview_v1);
	} else {
		rt_printf("siso_video_wview_v1 open fail\n\r");
		goto mmf2_video_web_viewer_fail;
	}
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0
	mm_module_ctrl(wview_v1_ctx, CMD_WEB_VIEWER_APPLY, 0);

mmf2_video_web_viewer_fail:
	vTaskDelete(NULL);
	return;
}

void example_websocket_viewer(void)
{
	if (xTaskCreate(mmf2_video_websocket_viewer, ((const char *)"mmf2_video_websocket_viewer"), 1024, NULL, tskIDLE_PRIORITY + 2,
					NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(mmf2_video_websocket_viewer) failed", __FUNCTION__);
	}
}