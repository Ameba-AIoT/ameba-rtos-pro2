/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/

/* Headers for example */
#include "module_kvs_webrtc.h"
#include "AppMain.h"
#include "AppMediaSrc_AmebaPro2.h"
#include "AppCommon.h"

/* webrtc git version */
#include "kvs_webrtc_version.h"

/* Config for Ameba-Pro */
#include "sample_config_webrtc.h"
#define KVS_QUEUE_DEPTH         6
#define WEBRTC_AUDIO_FRAME_SIZE 256

/* Network */
#include <lwip_netconf.h>
#include "wifi_conf.h"
#include <sntp/sntp.h>
#include "mbedtls/config.h"
#include "mbedtls/platform.h"

uint8_t *ameba_get_ip(void)
{
	return LwIP_GetIP(0);
}

/* Virtual file system */
#include "vfs.h"

/* Audio/Video */
#include "avcodec.h"

kvsWebrtcMediaQueue_t *pkvsWebrtcMediaQ;

static void ameba_platform_init(void)
{
#if defined(MBEDTLS_PLATFORM_C)
	mbedtls_platform_set_calloc_free(calloc, free);
#endif

	sntp_init();
	while (getEpochTimestampInHundredsOfNanos(NULL) < 10000000000000000ULL) {
		vTaskDelay(50 / portTICK_PERIOD_MS);
		printf("[KVS WebRTC module]: waiting get epoch timer\r\n");
	}

	// sd virtual file syetem register
	vfs_init(NULL);
	vfs_user_register("sd", VFS_FATFS, VFS_INF_SD);
}


static void kvs_webrtc_main_thread(void *param)
{
	kvs_webrtc_ctx_t *ctx = (kvs_webrtc_ctx_t *)param;

	printf("=== KVS Example Start ===\n\r");

	// display the git version
	printf("[KVS WebRTC module]: webrtc branch name = %s\r\n", webrtc_branch_name);
	printf("[KVS WebRTC module]: webrtc commit hash = %s\r\n", webrtc_commit_hash);

	// ameba platform init
	ameba_platform_init();

	// webrtc main
	WebRTCAppMain(&gAppMediaSrc);

	// sd virtual file syetem unregister
	vfs_user_unregister("sd", VFS_FATFS, VFS_INF_SD);
	vfs_deinit(NULL);

	vTaskDelete(NULL);
}


#ifdef ENABLE_AUDIO_SENDRECV
static void kvs_webrtc_audio_thread(void *param)
{
	kvs_webrtc_ctx_t *ctx = (kvs_webrtc_ctx_t *)param;
	webrtc_mm_t audio_rev_buf;

	while (!ctx->mediaStop) {
		if (xQueueReceive(pkvsWebrtcMediaQ->AudioRecvQueue, &audio_rev_buf, 50 / portTICK_PERIOD_MS) != pdTRUE) {
			continue;    // should not happen
		}

		mm_context_t *mctx = (mm_context_t *)ctx->parent;
		mm_queue_item_t *output_item;
		if (xQueueReceive(mctx->output_recycle, &output_item, 0xFFFFFFFF) == pdTRUE) {
			memcpy((void *)output_item->data_addr, (void *)audio_rev_buf.pData, audio_rev_buf.size);
			output_item->size = audio_rev_buf.size;
			output_item->type = audio_rev_buf.type;
			output_item->timestamp = audio_rev_buf.timestamp;
			xQueueSend(mctx->output_ready, (void *)&output_item, 0xFFFFFFFF);
			if (audio_rev_buf.pData && audio_rev_buf.bFreeData) {
				free(audio_rev_buf.pData);
			}
		}
	}

	vTaskDelete(NULL);
}
#endif /* ENABLE_AUDIO_SENDRECV */

void kvsWebrtcMediaSendToQueue(webrtc_mm_t *pMediaItem, QueueHandle_t xQueue)
{
	if (uxQueueSpacesAvailable(xQueue) == 0) { /* if queue is full, drop the oldest one */
		webrtc_mm_t tmp_mm;
		xQueueReceive(xQueue, &tmp_mm, 0);
		if (tmp_mm.pData && tmp_mm.bFreeData) {
			free(tmp_mm.pData);
		}
	}
	xQueueSend(xQueue, pMediaItem, 0);
}

int __kvs_webrtc_handle(void *p, void *input, void *output)
{
	kvs_webrtc_ctx_t *ctx = (kvs_webrtc_ctx_t *)p;
	mm_queue_item_t *input_item = (mm_queue_item_t *)input;

	getApp(&ctx->pkvsWebrtcAppConf);
	if (ctx->pkvsWebrtcAppConf == NULL || ctx->mediaStop) {
		return 0;
	}

	Frame frame;
	if (input_item->type == AV_CODEC_ID_H264) {
		frame.trackId = DEFAULT_VIDEO_TRACK_ID;
	} else if ((input_item->type == AV_CODEC_ID_PCMU) || (input_item->type == AV_CODEC_ID_PCMA) || (input_item->type == AV_CODEC_ID_OPUS)) {
		frame.trackId = DEFAULT_AUDIO_TRACK_ID;
	}
	frame.flags = app_media_h264_is_i_frame(input_item->data_addr) ? FRAME_FLAG_KEY_FRAME : FRAME_FLAG_NONE;
	frame.frameData = (uint8_t *)input_item->data_addr;
	frame.size = input_item->size;
	frame.presentationTs = getEpochTimestampInHundredsOfNanos(&input_item->timestamp);
	frame.version = FRAME_CURRENT_VERSION;
	frame.decodingTs = frame.presentationTs;

	ctx->pkvsWebrtcAppMediaSendFun((void *)ctx->pkvsWebrtcAppConf, (void *)&frame);

	return 0;
}

int kvs_webrtc_handle(void *p, void *input, void *output)
{
	kvs_webrtc_ctx_t *ctx = (kvs_webrtc_ctx_t *)p;
	mm_queue_item_t *input_item = (mm_queue_item_t *)input;

	if (ctx->pkvsWebrtcAppMediaSendFun) {
		return __kvs_webrtc_handle(p, input, output);
	}

	if (ctx->mediaStop) {
		return 0;
	}

	webrtc_mm_t mm;
	mm.size = input_item->size;
#if 1
	mm.pData = (uint8_t *)malloc(mm.size);
	if (!mm.pData) {
		printf("fail to allocate memory for webrtc media frame\r\n");
		return -1;
	}
	memcpy(mm.pData, (uint8_t *)input_item->data_addr, mm.size);
	mm.bFreeData = true;
#else
	//TODO: use the mmf buffer securely
	mm.pData = (uint8_t *)input_item->data_addr;  /* use the buffer from mmf */
	mm.bFreeData = false;  /* false: use mmf buffer; true: use allocated buffer */
#endif
	mm.timestamp = input_item->timestamp;
	mm.type = input_item->type;

	if (mm.type == AV_CODEC_ID_H264) {
		kvsWebrtcMediaSendToQueue(&mm, pkvsWebrtcMediaQ->VideoSendQueue);
	} else if ((mm.type == AV_CODEC_ID_PCMU) || (mm.type == AV_CODEC_ID_PCMA) || (mm.type == AV_CODEC_ID_OPUS)) {
		kvsWebrtcMediaSendToQueue(&mm, pkvsWebrtcMediaQ->AudioSendQueue);
	} else {
		printf("[KVS WebRTC module]: input type cannot be handled:%ld\r\n", input_item->type);
	}

	return 0;
}


static void kvsWebrtcMediaQueueCreate(kvsWebrtcMediaQueue_t *MediaQ)
{
	MediaQ->VideoSendQueue = xQueueCreate(KVS_QUEUE_DEPTH, sizeof(webrtc_mm_t));
	xQueueReset(MediaQ->VideoSendQueue);

	MediaQ->AudioSendQueue = xQueueCreate(KVS_QUEUE_DEPTH, sizeof(webrtc_mm_t));
	xQueueReset(MediaQ->AudioSendQueue);

#if defined(ENABLE_AUDIO_SENDRECV)
	//Create a queue to receive the G711 or Opus audio frame from viewer
	MediaQ->AudioRecvQueue = xQueueCreate(KVS_QUEUE_DEPTH, sizeof(webrtc_mm_t));
	xQueueReset(MediaQ->AudioRecvQueue);
#endif
	printf("[KVS WebRTC module]: media queue inited.\r\n");
}

static void kvsWebrtcMediaQueueFlushDelete(QueueHandle_t xQueue)
{
	webrtc_mm_t tmp_mm;
	while (xQueueReceive(xQueue, &tmp_mm, 0) == pdTRUE) {
		if (tmp_mm.pData && tmp_mm.bFreeData) {
			free(tmp_mm.pData); /* if the buffer is from mmf, don't free it here */
		}
	}
	vQueueDelete(xQueue);
}

static void kvsWebrtcMediaQueueDestroy(kvsWebrtcMediaQueue_t *MediaQ)
{
	kvsWebrtcMediaQueueFlushDelete(MediaQ->VideoSendQueue);
	kvsWebrtcMediaQueueFlushDelete(MediaQ->AudioSendQueue);
#if defined(ENABLE_AUDIO_SENDRECV)
	kvsWebrtcMediaQueueFlushDelete(MediaQ->AudioRecvQueue);
#endif
	printf("[KVS WebRTC module]: media queue deleted.\r\n");
}


int kvs_webrtc_control(void *p, int cmd, int arg)
{
	kvs_webrtc_ctx_t *ctx = (kvs_webrtc_ctx_t *)p;

	switch (cmd) {

	case CMD_KVS_WEBRTC_SET_APPLY:
		kvsWebrtcMediaQueueCreate(pkvsWebrtcMediaQ);
		if (xTaskCreate(kvs_webrtc_main_thread, ((const char *)"kvs_webrtc_main_thread"), 2048, (void *)ctx, tskIDLE_PRIORITY + 1,
						&ctx->kvs_webrtc_module_main_task) != pdPASS) {
			printf("[KVS WebRTC module]: %s xTaskCreate(kvs_webrtc_main_thread) failed\n\r", __FUNCTION__);
		}
#ifdef ENABLE_AUDIO_SENDRECV
		if (xTaskCreate(kvs_webrtc_audio_thread, ((const char *)"kvs_webrtc_audio_thread"), 512, (void *)ctx, tskIDLE_PRIORITY + 1,
						&ctx->kvs_webrtc_module_audio_recv_task) != pdPASS) {
			printf("[KVS WebRTC module]: %s xTaskCreate(kvs_webrtc_audio_thread) failed\n\r", __FUNCTION__);
		}
#endif
		ctx->mediaStop = 0;
		break;
	case CMD_KVS_WEBRTC_STOP:
		quitApp(); //kvs_webrtc_main_thread will be deleted
		ctx->mediaStop = 1; //stop media source and kvs_webrtc_audio_thread will be deleted
		kvsWebrtcMediaQueueDestroy(pkvsWebrtcMediaQ); //flush item in media queue then delete the queue
		break;
	}
	return 0;
}


void *kvs_webrtc_destroy(void *p)
{
	kvs_webrtc_ctx_t *ctx = (kvs_webrtc_ctx_t *)p;
	if (ctx) {
		free(ctx);
	}
	if (pkvsWebrtcMediaQ != NULL) {
		free(pkvsWebrtcMediaQ);
		pkvsWebrtcMediaQ = NULL;
	}
	return NULL;
}


void *kvs_webrtc_create(void *parent)
{
	kvs_webrtc_ctx_t *ctx = malloc(sizeof(kvs_webrtc_ctx_t));
	if (!ctx) {
		return NULL;
	}
	memset(ctx, 0, sizeof(kvs_webrtc_ctx_t));
	ctx->parent = parent;

	pkvsWebrtcMediaQ = (kvsWebrtcMediaQueue_t *)malloc(sizeof(kvsWebrtcMediaQueue_t));
	if (!pkvsWebrtcMediaQ) {
		goto cleanup;
	}

	// register a function(app_media_sendMediaFrame) to send data to peer in handle; or set NULL to send data to media queue buffer
	ctx->pkvsWebrtcAppMediaSendFun = NULL;

	printf("[KVS WebRTC module]: module created.\r\n");

	return ctx;

cleanup:
	kvs_webrtc_destroy(ctx);
	return NULL;
}


void *kvs_webrtc_new_item(void *p)
{
	kvs_webrtc_ctx_t *ctx = (kvs_webrtc_ctx_t *)p;
	(void)ctx;

	return (void *)malloc(WEBRTC_AUDIO_FRAME_SIZE * 2);
}


void *kvs_webrtc_del_item(void *p, void *d)
{
	(void)p;
	if (d) {
		free(d);
	}
	return NULL;
}


mm_module_t kvs_webrtc_module = {
	.create = kvs_webrtc_create,
	.destroy = kvs_webrtc_destroy,
	.control = kvs_webrtc_control,
	.handle = kvs_webrtc_handle,

	.new_item = kvs_webrtc_new_item,
	.del_item = kvs_webrtc_del_item,

	.output_type = MM_TYPE_ASINK,       // output for audio sink
	.module_type = MM_TYPE_AVSINK,      // module type is video algorithm
	.name = "KVS_WebRTC"
};
