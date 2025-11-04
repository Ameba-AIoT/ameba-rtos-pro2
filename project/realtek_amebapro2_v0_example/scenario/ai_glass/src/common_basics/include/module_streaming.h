#ifndef _MODULE_STREAMING_H
#define _MODULE_STREAMING_H

#include "sliding_windows.h"
#include "mmf2_module.h"
#include "dlist.h"
#include "basic_types.h"
#include "osdep_service.h"

#define TIME_SYNC_EN            1
#define TIME_SYNC_DIS           2


// Command definitions for streaming module
#define CMD_STREAMING_SET_PARAMS         MM_MODULE_CMD(0x00)
#define CMD_STREAMING_GET_PARAMS         MM_MODULE_CMD(0x01)
#define CMD_STREAMING_SET_STREAMING      MM_MODULE_CMD(0x02)
#define CMD_STREAMING_SELECT_STREAM      MM_MODULE_CMD(0x03)
#define CMD_STREAMING_SET_WINDOW_SIZE    MM_MODULE_CMD(0x04)
#define CMD_STREAMING_SET_SEG_SIZE       MM_MODULE_CMD(0x05)
#define CMD_STREAMING_SET_STOP_CB        MM_MODULE_CMD(0x06)
#define CMD_STREAMING_SET_FRAME_RATE     MM_MODULE_CMD(0x07)
#define CMD_STREAMING_SET_BIT_RATE       MM_MODULE_CMD(0x08)
#define CMD_STREAMING_SET_PACKET_RETRY   MM_MODULE_CMD(0x09)
#define CMD_STREAMING_SET_APPLY			 MM_MODULE_CMD(0x0a)

// Define types of streaming context or parameters
#define STREAMING_TYPE_NON_BLOCK         0
#define STREAMING_TYPE_BLOCK             1

#define STREAM_FLOW_ID_BASE              0
#define AVMEDIA_TYPE_VIDEO 0
#define AVMEDIA_TYPE_AUDIO 1
#define AV_CODEC_ID_H264  1
#define AV_CODEC_ID_PCMU  2
#define AV_CODEC_ID_PCMA  3
#define AV_CODEC_ID_MP4A_LATM 4
#define AV_CODEC_ID_H265 6
#define AV_CODEC_ID_OPUS 7

// Define main streaming context structure
typedef struct streaming_ctx_s {
    void *parent;
    SlidingWindow *window;
    uint32_t window_size;
    bool streaming_active;
    sliding_pkt_t spacket;
    uint32_t total_bytes_sent;
    int (*end_streaming_cb)(void);
    uint32_t timestamp;
} streaming_ctx_t;

extern mm_module_t streaming_module;

#endif // _MODULE_STREAMING_H
