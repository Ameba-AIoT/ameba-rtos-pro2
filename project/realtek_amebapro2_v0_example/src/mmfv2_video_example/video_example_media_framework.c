/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "video_example_media_framework.h"
#include <FreeRTOS.h>
#include <task.h>
#include "module_video.h"
#include "mmf2_pro2_video_config.h"

#define wifi_wait_time 500 //Here we wait 5 second to wiat the fast connect
//------------------------------------------------------------------------------
// common code for network connection
//------------------------------------------------------------------------------
#include "wifi_conf.h"
#include "lwip_netconf.h"

static int video_init_done = 0;

int video_example_get_init_status(void)
{
	return video_init_done;
}

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

#include "sensor_service.h"
static void sensor_board_init(void)
{
//init IR-Cut on RTK sensor board
#if(CONFIG_RTK_EVB_IR_CTRL >= 1)
	ir_cut_init(NULL);
	ir_cut_enable(1);
#endif
//init HW-Als & IR-LED on RTK light board
#if(CONFIG_RTK_EVB_IR_CTRL == 2)
	ir_ctrl_init(NULL);
	ambient_light_sensor_init(NULL);
	ambient_light_sensor_power(1);
#endif
}

//------------------------------------------------------------------------------
// video support examples
//------------------------------------------------------------------------------
static void example_mmf2_video_surport(void)
{

	// CH1 Video -> H264/HEVC -> RTSP
	mmf2_video_example_v1_init();

	// CH2 Video -> H264/HEVC -> RTSP
	//mmf2_video_example_v2_init();

	// CH3 Video -> JPEG -> RTSP
	//mmf2_video_example_v3_init();

	// CH1 Video -> H264/HEVC -> RTSP + SNAPSHOT
	//mmf2_video_example_v1_shapshot_init();
	
	// CH1 Video -> SNAPSHOT
	//mmf2_video_example_v1_snapshot_hr_init();

	// CH1 Video -> SNAPSHOT + HTTP File Server
	//mmf2_video_example_v1_shapshot_httpfs_init();

	// 1 Video (H264/HEVC) -> 2 RTSP (V1, V2)
	//mmf2_video_example_simo_init();

	// 1 Video (H264/HEVC) 1 Audio -> RTSP
	//mmf2_video_example_av_init();

	// 2 Video (H264/HEVC) 1 Audio -> 2 RTSP (V1+A, V2+A)
	//mmf2_video_example_av2_init();

	// 1 Video (H264/HEVC) 1 Audio -> 2 RTSP (V+A)
	//mmf2_video_example_av21_init();

	// 1 Video (H264/HEVC) 1 Audio -> MP4 (SD card)
	//mmf2_video_example_av_mp4_init();

	// 1V1A RTSP MP4
	// H264 -> RTSP and mp4
	// AUDIO -> AAC  -> RTSP and mp4
	//mmf2_video_example_av_rtsp_mp4_init();

	// Joint test
	// H264 -> RTSP (with AUDIO)
	// H264 -> RTSP (with AUDIO)
	// AUDIO -> AAC  -> RTSP
	// RTP   -> AAD  -> AUDIO
	//mmf2_video_example_joint_test_init();

	// Joint test RTSP MP4
	// H264 -> MP4 (V1)
	// H264 -> RTSP (V2)
	// AUDIO -> AAC  -> RTSP and mp4
	// RTP   -> AAD  -> AUDIO
	//mmf2_video_example_joint_test_rtsp_mp4_init();

	// H264 and 2way audio (G711, PCMU)
	// H264 -> RTSP (V1)
	// AUDIO -> G711E  -> RTSP
	// RTP   -> G711D  -> AUDIO
	// ARRAY (PCMU) -> G711D -> AUDIO (doorbell)
	//mmf2_video_example_2way_audio_pcmu_doorbell_init();

	// H264 and 2way audio (G711, PCMU)
	// H264 -> RTSP (V1)
	// AUDIO -> G711E  -> RTSP
	// RTP   -> G711D  -> AUDIO
	//mmf2_video_example_2way_audio_pcmu_init();

	// ARRAY (H264) -> RTSP (V)
	//mmf2_video_example_array_rtsp_init();

	// V1 parameter change
	//mmf2_video_example_v1_param_change_init();

	// V1 day night mode change
	//mmf2_video_example_v1_day_night_change_init();

	// V1 apply mask
	//mmf2_video_example_v1_mask_init();

	// V1 rate control example
	//mmf2_video_example_v1_rate_control_init();

	//HTTP File Server
	//mmf2_video_example_av_mp4_httpfs_init();

	// H264 -> RTSP (V1)
	// RGB  -> NN object detect (V4)
	//mmf2_video_example_vipnn_rtsp_init();

	// H264 -> RTSP (V1)
	// RGB  -> NN face detect (V4) -> NN face recognition
	//mmf2_video_example_face_rtsp_init();

	// NN 3 models cascade example
	//mmf2_video_example_fd_lm_mfn_sim_rtsp_init();

	// H264 -> RTSP (V1)
	// RGB  -> NN object detect (V4)
	// RGB  -> NN face detect (V4) -> NN face recognition
	// AUDIO -> NN audio classification
	//mmf2_video_example_joint_test_all_nn_rtsp_init();

	// MP4 -> RTSP (V1)
	//mmf2_video_example_demuxer_rtsp_init();

	// ARRAY (H264, G711) -> MP4
	//mmf2_video_example_h264_pcmu_array_mp4_init();

	// AUDIO -> NN audio classification
	//mmf2_video_example_audio_vipnn_init();

	// H264 -> RTSP (V1)
	// RGB  -> motion detection (V4)
	//mmf2_video_example_md_rtsp_init();

	// MD dynamic record event
	// H264 -> MP4 (V1)
	// H264 -> RTSP (V2)
	// RGB  -> motion detection (V4)
	// AUDIO -> AAC  -> RTSP and mp4
	//mmf2_video_example_md_mp4_init();

	//mmf2_video_example_bayercap_rtsp_init();

	// H264 -> RTSP (V1)
	// RGB  -> motion detection (V4) -> NN object detect
	//mmf2_video_example_md_nn_rtsp_init();

	// Joint test RTSP MP4 with fast camera start
	//mmf2_video_example_joint_test_rtsp_mp4_init_fcs();

	// H264 -> RTSP (V1)
	// RGB  -> NN face detect (V4)
	//mmf2_video_example_vipnn_facedet_init();

	// EXTERNAL DATA -> JPEG
	//mmf2_video_example_jpeg_external_init();

	// H264 -> RTSP (V1)
	// H264 -> RTSP (V2, Sync mode)
	// RGB  -> NN face detect (V4)
	//mmf2_video_example_vipnn_facedet_sync_init();

	// CH1 Video -> JPEG (SNAPSHOT, Sync mode)
	//mmf2_video_example_vipnn_facedet_sync_snapshot_init();

	// H264 -> RTSP (V1)
	// RGB  -> NN palm detect -> NN hand detect(V4)
	//mmf2_video_example_vipnn_handgesture_init();

	// Joint test RTSP MP4 with NN
	// H264 -> MP4  (V1)
	// H264 -> RTSP (V2)
	// RGB  -> NN object detect (V4)
	// RGB  -> NN face detect (V4) -> NN face recognition (optional)
	// AUDIO -> AAC  -> RTSP and mp4
	// AUDIO -> NN audio classification
	// RTP   -> AAD  -> AUDIO
	//mmf2_video_example_joint_test_vipnn_rtsp_mp4_init();

	// H264 -> RTSP (V1)
	// RGB  -> NN image classification (V4)
	//mmf2_video_example_vipnn_classify_rtsp_init();

	//V1 timslapse mp4 example
	//mmf2_video_example_timelapse_mp4_init();

	video_init_done = 1;
}

void video_example_main(void *param)
{
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(2048);
#endif
	if (!voe_boot_fsc_status()) {
		wifi_common_init();
		sensor_board_init();
	}

	example_mmf2_video_surport();

	// Disable video log
	vTaskDelay(1000);
	video_ctrl(0, VIDEO_DEBUG, 0);

	// TODO: exit condition or signal
	while (1) {
		vTaskDelay(10000);
		// extern mm_context_t *video_v1_ctx;
		// mm_module_ctrl(video_v1_ctx, CMD_VIDEO_PRINT_INFO, 0);
		// check video frame here
		if (hal_video_get_no_video_time() > 1000 * 30) {
			printf("no video frame time > %d ms", hal_video_get_no_video_time());
			//reopen video or system reset

			sys_reset();
		}
#if 0	//get encode buffer info	
		int ret = 0;
		int enc_size = 0;
		int out_buf_size;
		int out_rsvd_size;
		ret = video_get_buffer_info(0, &enc_size, &out_buf_size, &out_rsvd_size);
		printf("enc_size = %d\r\n", enc_size >> 10);
		printf("out_buf_size = %d\r\n", out_buf_size >> 10);
		printf("out_rsvd_size = %d\r\n", out_rsvd_size >> 10);
#endif

	}
}

void video_example_media_framework(void)
{
	/*user can start their own task here*/
	if (xTaskCreate(video_example_main, ((const char *)"mmf2_video"), 4096, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\r\n video_example_main: Create Task Error\n");
	}
}
