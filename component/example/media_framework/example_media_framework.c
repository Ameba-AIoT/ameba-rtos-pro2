/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "example_media_framework.h"
#include <FreeRTOS.h>
#include <task.h>
#include "platform_opts.h"

#ifndef CONFIG_PLATFORM_8735B
#include "platform_autoconf.h"
#else
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
#include "osdep_service.h"
#endif
#endif

#define wifi_wait_time 500 //Here we wait 5 second to wiat the fast connect 
//------------------------------------------------------------------------------
// common code for network connection
//------------------------------------------------------------------------------
#include "wifi_conf.h"
#include "lwip_netconf.h"

#if DEVICE_SURPORT_VIDEO
#include "sensor_service.h"
void sensor_board_init()
{
#if CONFIG_LIGHT_SENSOR
	init_sensor_service();
#else
	ir_cut_init(NULL);
	ir_cut_enable(1);
#endif
}
#endif

void common_init(void)
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

/*
static void stop_all_streaming()
{

}

static void close_all_context()
{

}*/

//------------------------------------------------------------------------------
// audio only examples
//------------------------------------------------------------------------------
void example_mmf2_audio_only(void)
{

	// 1 Audio (AAC) -> RTSP (A)
	// mmf2_example_a_init();

	// audio -> audio , audio loopback
	// mmf2_example_audioloop_init();

	// i2s -> audio , audio loopback
	// mmf2_example_i2s_audio_init();

	// audio -> G711E -> G711D -> audio
	// mmf2_example_g711loop_init();

	// audio -> AAC -> AAD -> audio
	// mmf2_example_aacloop_init();

	// RTP -> AAD -> audio
	// mmf2_example_rtp_aad_init();

	// 2WAY audio
	// mmf2_example_2way_audio_init();

	// ARRAY (PCMU) -> RTSP (A)
	// mmf2_example_pcmu_array_rtsp_init();

	// ARRAY (AAC) -> RTSP (A)
	// mmf2_example_aac_array_rtsp_init();

	// audio -> OPUSC -> OPUSD -> audio
	// mmf2_example_opusloop_init();

	// 1 Audio (OPUSC) -> RTSP (A)
	// mmf2_example_a_opus_init();

	// RTP -> OPUSD -> audio
	// mmf2_example_rtp_opusd_init();

	// 1 Audio (OPUSC) -> RTSP (A) and RTP -> OPUSD -> audio
	// mmf2_example_2way_audio_opus_init();

	// 1 Audio (PCMU) -> RTSP (A) and RTP / DOORBELL -> PCMU -> audio
	// mmf2_example_2way_audio_g711_doorbell_init();

	// 1 Array (PCM) -> audio
	// mmf2_example_pcm_array_audio_init();

	// ARRAY (OPUS) -> RTSP (A)
	// mmf2_example_opus_array_rtsp_init();
}

//------------------------------------------------------------------------------
// video support examples
//------------------------------------------------------------------------------
#if !defined(CONFIG_PLATFORM_8735B)
void example_mmf2_video_surport(void)
{

	// CH1 Video -> H264 -> RTSP
	//mmf2_example_v1_init();

	// CH2 Video -> H264 -> RTSP
	//mmf2_example_v2_init();

	// CH3 Video -> JPEG -> RTSP
#if ENABLE_V3_JPEG == V3_JPEG_STREAMING
	mmf2_example_v3_init();
#endif

	// 1 Video (H264) -> 2 RTSP (V1, V2)
	//mmf2_example_simo_init();

	// 1 Video (H264) 1 Audio -> RTSP
	//mmf2_example_av_init();

	// 2 Video (H264) 1 Audio -> 2 RTSP (V1+A, V2+A)
	//mmf2_example_av2_init();

	// 1 Video (H264) 1 Audio -> 2 RTSP (V+A)
	//mmf2_example_av21_init();

	// Joint test
	// ISP   -> H264 -> RTSP (with AUDIO)
	// ISP   -> H264 -> RTSP (with AUDIO)
	// AUDIO -> AAC  -> RTSP
	// RTP   -> AAD  -> AUDIO
	//mmf2_example_joint_test_init();

	// 1 Video (H264) 1 Audio -> MP4 (SD card)
#if ISP_BOOT_MODE_ENABLE //For first frame demo
#if ISP_BOOT_MP4
	mmf2_example_av_mp4_init();

	//HTTP File Server
	//mmf2_example_av_mp4_httpfs_init();
#endif
#if ISP_BOOT_RTSP
	mmf2_example_v1_init();
#endif
#else
	//mmf2_example_av_mp4_init();

	//HTTP File Server
	//mmf2_example_av_mp4_httpfs_init();
#endif
	// Joint test RTSP MP4
	// ISP   -> H264 -> RTSP (V1)
	// ISP   -> H264 -> MP4 (V2)
	// AUDIO -> AAC  -> RTSP and mp4
	// RTP   -> AAD  -> AUDIO
#if ISP_BOOT_MODE_ENABLE //For first frame demo
#if ISP_BOOT_MUX
	mmf2_example_joint_test_rtsp_mp4_init();
#endif
#else
	//mmf2_example_joint_test_rtsp_mp4_init();
#endif

	// 1V1A RTSP MP4
	// ISP   -> H264 -> RTSP and mp4
	// AUDIO -> AAC  -> RTSP and mp4
	//mmf2_example_av_rtsp_mp4_init();

	// H264 and 2way audio (G711, PCMU)
	// ISP   -> H264 -> RTSP (V1)
	// AUDIO -> G711E  -> RTSP
	// RTP   -> G711D  -> AUDIO
	// ARRAY (PCMU) -> G711D -> AUDIO (doorbell)
	//mmf2_example_h264_2way_audio_pcmu_doorbell_init();

	// H264 and 2way audio (G711, PCMU)
	// ISP   -> H264 -> RTSP (V1)
	// AUDIO -> G711E  -> RTSP
	// RTP   -> G711D  -> AUDIO
	//mmf2_example_h264_2way_audio_pcmu_init();

	// ARRAY (H264) -> RTSP (V)
	//mmf2_example_h264_array_rtsp_init();

	// V1 parameter change
	//mmf2_example_v1_param_change_init();
}
#endif

void example_mmf2_signal_stream_main(void *param)
{
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1) && defined(CONFIG_PLATFORM_8735B)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif
	//int ret;
#if !defined(CONFIG_PLATFORM_8735B)
#if ISP_BOOT_MODE_ENABLE == 0
	common_init();
#endif
#else
	common_init();
#endif

	example_mmf2_audio_only();

#if !defined(CONFIG_PLATFORM_8735B)
#if DEVICE_SURPORT_VIDEO
	sensor_board_init();
	example_mmf2_video_surport();
#endif
#endif

	// TODO: exit condition or signal
	while (1) {
		vTaskDelay(1000);
	}
}

//TODO: isp boot stream parameter, should be placed in other place
#if !defined(CONFIG_PLATFORM_8735B)
#if CONFIG_EXAMPLE_MEDIA_FRAMEWORK && ISP_BOOT_MODE_ENABLE
#include "isp_api.h"
#include "mmf2_video_config.h"
#include "sensor.h"
#include "isp_boot.h"

#define CINIT_DATA_SECTION SECTION(".cinit.data")
CINIT_DATA_SECTION isp_boot_stream_t isp_boot_stream = {
	.width = V1_WIDTH,
	.height = V1_HEIGHT,
	.isp_id = 0,
	.hw_slot_num = V1_HW_SLOT,
	.fps = V1_FPS,
	.format = ISP_FORMAT_YUV420_SEMIPLANAR,
	.pin_idx = ISP_PIN_IDX,
	.mode = ISP_FAST_BOOT,
	.interface = ISP_INTERFACE_MIPI,
	.clk = SENSOR_CLK_USE,
	.sensor_fps = SENSOR_FPS,
	.isp_fw_location = ISP_FW_LOCATION
};
#endif
#endif

void example_media_framework(void)
{
	/*user can start their own task here*/
	if (xTaskCreate(example_mmf2_signal_stream_main, ((const char *)"mmf2_1"), 2048, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\r\n example_media_two_source_main: Create Task Error\n");
	}
}
