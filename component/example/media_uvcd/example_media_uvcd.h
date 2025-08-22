#ifndef EXAMPLE_MEDIA_UVCD_H
#define EXAMPLE_MEDIA_UVCD_H

#include "sensor.h"
//#include "video_common_api.h"
#include "uvc/inc/usbd_uvc_desc.h"

#define UVCD_TUNNING_MODE  1// 1 :Contain four formats to support tunning tool (YUV2 NV12 H264 JPEG)
// 0 :Contain two formats to support h264 and jpeg format
#define CUSTOMER_AMEBAPRO_ADVANCE 0 //1: Based on the defined "UVCD_TUNNING_MODE", this option "1" is for customer release, "0" is for internal use.
void example_media_uvcd(void);

#define MAX_W sensor_params[USE_SENSOR].sensor_width
#define MAX_H sensor_params[USE_SENSOR].sensor_height

// Configure for supported format of uvcd
// If user only want to support one configuration, user can only enable one of them
#define UVCD_YUY2 1
#define UVCD_NV12 1
#define UVCD_MJPG 1
#define UVCD_H264 1
#define UVCD_H265 1

void example_tuning_rtsp_video_v2_init(void);

#endif /* MMF2_EXAMPLE_H */