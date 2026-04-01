#ifndef MMF2_PRO2_VIDEO_CONFIG_H
#define MMF2_PRO2_VIDEO_CONFIG_H
#include "video_api.h"

/*ISP CHANNEL*/
//ISP channel 0/1/2:HEVC/JPEG/NV12 4:only RGB

/*ENCODE TYPE*/
//type define in the video_api.h
//type : 0:HEVC 1:H264 2:JPEG 3:NV12 4:RGB 5:HEVC+JPEG 6:H264+JPEG

#define USE_UPDATED_VIDEO_HEAP  0

#define	VIDEO_QCIF  0
#define	VIDEO_CIF   1
#define	VIDEO_WVGA  2
#define	VIDEO_VGA   3
#define	VIDEO_D1    4
#define	VIDEO_HD    5
#define	VIDEO_FHD   6
#define	VIDEO_3M    7
#define	VIDEO_5M    8
#define	VIDEO_2K    9

/*

static const int video_res_w[10] = {
	176, //QCIF
	352, // CIF
	640, // WVGA
	640, // VGA
	720, //D1
	1280, // HD
	1920, // FHD
	2048, // 3M
	2592, // 5M
	2560, // 2K
};

static const int video_res_h[10] = {
	144,  //QCIF
	288,  // CIF
	360,  // WVGA
	480,  // VGA
	480,  //D1
	720, // HD
	1080, // FHD
	1536, // 3M
	1944, // 5M
	1440, // 2K
};
*/

#define USR_CMD_RTSP_PAUSE			    (u32)0x1
#define USR_CMD_RTSP1_PAUSE 			(u32)(0x1 << 1)
#define USR_CMD_RTSP2_PAUSE 			(u32)(0x1 << 2)
#define USR_CMD_RTSP3_PAUSE 			(u32)(0x1 << 3)
#define USR_CMD_AUDIO_PAUSE 			(u32)(0x1 << 4)
#define USR_CMD_VIDEO_PAUSE 			(u32)(0x1 << 8)
#define USR_CMD_VIDEO1_PAUSE 			(u32)(0x1 << 9)
#define USR_CMD_RECORD_PAUSE            (u32)(0x1 << 12)
#define USR_CMD_RTP_PAUSE               (u32)(0x1 << 16)
#define USR_CMD_MBNSSD_PAUSE            (u32)(0x1 << 20)
#define USR_CMD_FRC_PAUSE               (u32)(0x1 << 21)
#define USR_CMD_VIPNN_PAUSE             (u32)(0x1 << 22)
#define USR_CMD_EXAMPLE_DEINIT		    (u32)(0x1 << 31)

#endif /* MMF2_PRO2_VIDEO_CONFIG_H */