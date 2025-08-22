#ifndef MMF2_SD_EXAMPLE_H
#define MMF2_SD_EXAMPLE_H

#include "video_common_api.h"

#include "platform_opts.h"
#include "module_isp.h"
#include "module_h264.h"
#include "module_jpeg.h"
#include "module_rtsp2.h"
#include "module_mp4.h"
#include "module_audio.h"
#include "module_aac.h"
#include "module_aad.h"
#include "module_g711.h"
#include "module_rtp.h"
#include "module_array.h"
#include "module_i2s.h"
#include "module_httpfs.h"

#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_simo.h"
#include "mmf2_miso.h"
#include "mmf2_mimo.h"
#include "sensor_service.h"
#include "jpeg_snapshot.h"
#include "snapshot_tftp_handler.h"
#include "sample_doorbell_pcmu.h"
#include "sample_pcmu.h"
#include "sample_aac.h"
#include "sample_h264.h"
#include "isp_boot.h"
#include "sensor.h"

#if ISP_BOOT_MODE_ENABLE
#include "hal_power_mode.h"
#include "power_mode_api.h"
#endif

#include "platform_autoconf.h"

#define VIDEO_240P			1
#define VIDEO_480P			2
#define VIDEO_720P			3
#define VIDEO_1080P			4

#define V3_JPEG_OFF			0
#define V3_JPEG_STREAMING	1
#define V3_JPEG_SNAPSHOT	2

#define SNAPSHOT_TFTP_TYPE //The default setting is TFTP transfer. If undef the marco it will choose the sd transfer        

#define USING_I2S_MIC           0

// ---------- Settings ----------

//------------------------------------------------------------------------------
// V1
//------------------------------------------------------------------------------
#define V1_FPS						30 //Default is recording
#define V1_RESOLUTION				VIDEO_720P
#define V1_HW_SLOT					1
#define V1_SW_SLOT					3
// H264
#define V1_BITRATE					2*1024*1024
#define V1_MEM_BUF_SEC				10
#define V1_H264_RCMODE				RC_MODE_H264CBR
//#define V1_H264_QUEUE_LEN			(V1_FPS*(V2_MEM_BUF_SEC))
#define V1_H264_QUEUE_LEN			70
// JPEG
#define ENABLE_V1_SNAPSHOT			0 // snapshot during h264 streaming
#if ENABLE_V1_SNAPSHOT
#define V1_SNAPSHOT_LEVEL			1 // JPEG quality level
#endif

//------------------------------------------------------------------------------
// V2
//------------------------------------------------------------------------------
#define V2_FPS						15 //Default is streaming
#define V2_RESOLUTION				VIDEO_1080P
#define V2_HW_SLOT					1
#define V2_SW_SLOT					2

// H264
#define V2_BITRATE					1*1024*1024
#if ISP_BOOT_MODE_ENABLE
#define V2_MEM_BUF_SEC				20
#else
#define V2_MEM_BUF_SEC				10
#endif
#define V2_H264_RCMODE				RC_MODE_H264CBR
//#define V2_H264_QUEUE_LEN			(V2_FPS*(V2_MEM_BUF_SEC))
#define V2_H264_QUEUE_LEN			40

//------------------------------------------------------------------------------
// V3
//------------------------------------------------------------------------------
#define ENABLE_V3_JPEG						V3_JPEG_OFF
#if ENABLE_V3_JPEG == V3_JPEG_STREAMING
#define ENABLE_V3_SNAPSHOT_WHEN_STREAMING	0
#endif
#define V3_FPS								10
#define V3_RESOLUTION						VIDEO_720P
#define V3_HW_SLOT							2
#define V3_SW_SLOT							3
// JPEG
#define V3_JPEG_LEVEL						1 // quality level

// ---------- End of Settings ----------

#if V1_RESOLUTION == VIDEO_240P
#define V1_WIDTH	VIDEO_240P_WIDTH
#define V1_HEIGHT	VIDEO_240P_HEIGHT
#elif V1_RESOLUTION == VIDEO_480P
#define V1_WIDTH	VIDEO_480P_WIDTH
#define V1_HEIGHT	VIDEO_480P_HEIGHT
#elif V1_RESOLUTION == VIDEO_720P
#define V1_WIDTH	VIDEO_720P_WIDTH
#define V1_HEIGHT	VIDEO_720P_HEIGHT
#elif V1_RESOLUTION == VIDEO_1080P
#define V1_WIDTH	VIDEO_1080P_WIDTH
#define V1_HEIGHT	VIDEO_1080P_HEIGHT
#endif

#if V2_RESOLUTION == VIDEO_240P
#define V2_WIDTH	VIDEO_240P_WIDTH
#define V2_HEIGHT	VIDEO_240P_HEIGHT
#elif V2_RESOLUTION == VIDEO_480P
#define V2_WIDTH	VIDEO_480P_WIDTH
#define V2_HEIGHT	VIDEO_480P_HEIGHT
#elif V2_RESOLUTION == VIDEO_720P
#define V2_WIDTH	VIDEO_720P_WIDTH
#define V2_HEIGHT	VIDEO_720P_HEIGHT
#elif V2_RESOLUTION == VIDEO_1080P
#define V2_WIDTH	VIDEO_1080P_WIDTH
#define V2_HEIGHT	VIDEO_1080P_HEIGHT
#endif

#if V3_RESOLUTION == VIDEO_240P
#define V3_WIDTH	VIDEO_240P_WIDTH
#define V3_HEIGHT	VIDEO_240P_HEIGHT
#elif V3_RESOLUTION == VIDEO_480P
#define V3_WIDTH	VIDEO_480P_WIDTH
#define V3_HEIGHT	VIDEO_480P_HEIGHT
#elif V3_RESOLUTION == VIDEO_720P
#define V3_WIDTH	VIDEO_720P_WIDTH
#define V3_HEIGHT	VIDEO_720P_HEIGHT
#elif V3_RESOLUTION == VIDEO_1080P
#define V3_WIDTH	VIDEO_1080P_WIDTH
#define V3_HEIGHT	VIDEO_1080P_HEIGHT
#endif

// ---------- Buffer Size Settings ----------

#define V1_BLOCK_SIZE					1024*10
#define V1_FRAME_SIZE					V1_WIDTH*V1_HEIGHT/2
//#define V1_BUFFER_SIZE					V1_FRAME_SIZE*6
#define V1_BUFFER_SIZE					V1_FRAME_SIZE + (V1_BITRATE/8*V1_MEM_BUF_SEC)

#define V2_BLOCK_SIZE						1024*10
#define V2_FRAME_SIZE						V2_WIDTH*V2_HEIGHT/2
//#define V2_BUFFER_SIZE						V2_FRAME_SIZE*3
#define V2_BUFFER_SIZE						V2_FRAME_SIZE + (V2_BITRATE/8*V2_MEM_BUF_SEC)

#define V3_BLOCK_SIZE						1024*10
#define V3_FRAME_SIZE						V3_WIDTH*V3_HEIGHT/2
#define V3_BUFFER_SIZE						V3_FRAME_SIZE*3

// ---------- End of Buffer Size Settings ---------

#if (ENABLE_V1_SNAPSHOT) && (ENABLE_V3_JPEG)
#error "Don't set both ENABLE_V1_SNAPSHOT and ENABLE_V3_JPEG"
#elif (ENABLE_V1_SNAPSHOT) || (ENABLE_V3_JPEG == V3_JPEG_SNAPSHOT) || (ENABLE_V3_SNAPSHOT_WHEN_STREAMING)
#define ENABLE_SNAPSHOT 1
#endif

#if ENABLE_V1_SNAPSHOT
#define SNAPSHOT_WIDTH		V1_WIDTH
#define SNAPSHOT_HEIGHT		V1_HEIGHT
#define SNAPSHOT_FPS		V1_FPS
#define SNAPSHOT_LEVEL		V1_SNAPSHOT_LEVEL
#define SNAPSHOT_BUFFER_SIZE	V1_WIDTH*V1_HEIGHT/2
#elif ENABLE_V3_SNAPSHOT_WHEN_STREAMING || (ENABLE_V3_JPEG == V3_JPEG_SNAPSHOT)
#define SNAPSHOT_WIDTH		V3_WIDTH
#define SNAPSHOT_HEIGHT		V3_HEIGHT
#define SNAPSHOT_FPS		V3_FPS
#define SNAPSHOT_LEVEL		V3_JPEG_LEVEL
#define SNAPSHOT_BUFFER_SIZE	V3_WIDTH*V3_HEIGHT/2
#endif

void example_media_framework_sd_detect(void);

void mmf2_example_v1_init(void);
void mmf2_example_v2_init(void);
void mmf2_example_v3_init(void);
void mmf2_example_simo_init(void);
void mmf2_example_a_init(void);
void mmf2_example_av_init(void);
void mmf2_example_av2_init(void);
void mmf2_example_av21_init(void);
void mmf2_example_audioloop_init(void);
void mmf2_example_g711loop_init(void);
void mmf2_example_av_rtsp_mp4_init(void);
void mmf2_example_aacloop_init(void);
void mmf2_example_rtp_aad_init(void);
void mmf2_example_2way_audio_init(void);
void mmf2_example_joint_test_init(void);
void mmf2_example_av_mp4_init(void);
void mmf2_example_av_mp4_httpfs_init(void);
void mmf2_example_joint_test_rtsp_mp4_init(void);
void mmf2_example_h264_2way_audio_pcmu_doorbell_init(void);
void mmf2_example_h264_2way_audio_pcmu_init(void);
void mmf2_example_pcmu_array_rtsp_init(void);
void mmf2_example_aac_array_rtsp_init(void);
void mmf2_example_h264_array_rtsp_init(void);
void mmf2_example_v1_param_change_init(void);
void snapshot_setting(void);

extern isp_params_t isp_v1_params;
extern h264_params_t h264_v1_params;
extern rtsp2_params_t rtsp2_v1_params;
#if ISP_BOOT_MODE_ENABLE
extern isp_boot_stream_t isp_boot_stream;
#endif
extern isp_params_t isp_v2_params;
extern h264_params_t h264_v2_params;
extern rtsp2_params_t rtsp2_v2_params;
extern isp_params_t isp_v3_params;
extern jpeg_params_t jpeg_v3_params;
extern rtsp2_params_t rtsp2_v3_params;
extern rtsp2_params_t rtsp2_a_params;
extern rtsp2_params_t rtsp2_a_pcmu_params;
extern audio_params_t audio_params;
extern i2s_params_t i2s_params;
extern aac_params_t aac_params;
//extern mp4_params_t mp4_params;
extern mp4_params_t mp4_v1_params;
extern mp4_params_t mp4_v2_params;
extern mp4_params_t mp4_v3_params;
extern g711_params_t g711e_params;
extern g711_params_t g711d_params;
extern aad_params_t aad_params;
extern aad_params_t aad_rtp_params;
extern rtp_params_t rtp_aad_params;
extern rtp_params_t rtp_g711d_params;
extern array_params_t doorbell_pcmu_array_params;
extern array_params_t pcmu_array_params;
extern array_params_t aac_array_params;
extern rtsp2_params_t rtsp2_aac_array_params;
extern rtsp2_params_t rtsp2_array_params;
extern array_params_t h264_array_params;
extern httpfs_params_t httpfs_params;

extern mm_context_t *isp_v1_ctx;
extern mm_context_t *h264_v1_ctx;
extern mm_context_t *rtsp2_v1_ctx;
extern mm_siso_t *siso_isp_h264_v1;
extern mm_siso_t *siso_h264_rtsp_v1;
extern mm_context_t *isp_v2_ctx;
extern mm_context_t *h264_v2_ctx;
extern mm_context_t *rtsp2_v2_ctx;
extern mm_siso_t *siso_isp_h264_v2;
extern mm_siso_t *siso_h264_rtsp_v2;
extern mm_context_t *isp_v3_ctx;
extern mm_context_t *jpeg_v3_ctx;
extern mm_context_t *rtsp2_v3_ctx;
extern mm_siso_t *siso_isp_jpeg_v3;
extern mm_siso_t *siso_jpeg_rtsp_v3;
extern mm_simo_t *simo_h264_rtsp_v2_v3;
extern mm_context_t *audio_ctx;
extern mm_context_t *i2s_ctx;
extern mm_context_t *aac_ctx;
extern mm_context_t *rtsp2_ctx;
extern mm_siso_t *siso_audio_aac;
extern mm_siso_t *siso_aac_rtsp;
extern mm_miso_t *miso_h264_aac_rtsp;
extern mm_mimo_t *mimo_2v_1a_rtsp;
extern mm_context_t *rtsp2_v4_ctx;
extern mm_context_t *mp4_ctx;
extern mm_miso_t *miso_h264_aac_mp4;
extern mm_mimo_t *mimo_2v_1a_rtsp_mp4;
extern mm_mimo_t *mimo_1v_1a_rtsp_mp4;
extern mm_context_t *array_ctx;
extern mm_miso_t *miso_h264_g711_rtsp;
extern mm_siso_t *siso_rtp_g711d;
extern mm_miso_t *miso_rtp_array_g711d;
extern mm_siso_t *siso_array_rtsp;
extern mm_siso_t *siso_audio_loop;
extern mm_siso_t *siso_g711e_rtsp;
extern mm_context_t *g711e_ctx;
extern mm_context_t *g711d_ctx;
extern mm_siso_t *siso_audio_g711e;
extern mm_siso_t *siso_g711_e2d;
extern mm_siso_t *siso_g711d_audio;
extern mm_context_t *aad_ctx;
extern mm_siso_t *siso_aac_e2d;
extern mm_siso_t *siso_aad_audio;
extern mm_context_t *rtp_ctx;
extern mm_siso_t *siso_rtp_aad;
extern mm_context_t *httpfs_ctx;

#endif /* MMF2_EXAMPLE_H */