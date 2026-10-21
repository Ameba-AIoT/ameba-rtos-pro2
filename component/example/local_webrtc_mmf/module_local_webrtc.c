/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright 2026 AmebaPro2 WebRTC Contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "logging.h"
#include "module_local_webrtc.h"
#include "platform_opts.h"
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_miso.h"
#include "module_video.h"
#include "module_audio.h"
#include "module_g711.h"
#include "module_opusc.h"
#include "module_opusd.h"
#include "opus_defines.h"
#include "avcodec.h"
#include "video_api.h"
#include "FreeRTOS.h"
#include "networking_utils.h"

#include <string.h>

/* used to monitor skb resource */
extern int skbbuf_used_num;
extern int skbdata_used_num;
extern int max_local_skb_num;
extern int max_skb_buf_num;

#define MEDIA_PORT_SKB_BUFFER_THRESHOLD            ( 64 )
#define MEDIA_PORT_WEBRTC_AUDIO_FRAME_SIZE         ( 256 )
/* Queue item buffer size: must be large enough for the largest frame in the
 * receive path (browser → AmebaPro2). Currently only Opus audio is received,
 * with frames typically ~40 bytes. 4KB provides generous headroom. */
#define MEDIA_PORT_WEBRTC_QUEUE_ITEM_SIZE          ( 4 * 1024 )

extern void local_webrtc_master_entry(void);
mm_context_t *pWebrtcMmContext = NULL;

static int HandleModuleFrameHook(void *p, void *input, void *output)
{
	int ret = 0;
	MediaModuleContext_t *pCtx = (MediaModuleContext_t *)p;
	MediaFrame_t frame;
	mm_queue_item_t *pInputItem = (mm_queue_item_t *)input;

	(void) output;

	if (pCtx->mediaStart != 0) {
		do {
			/* Set SKB buffer threshold to manage memory allocation. */
			if ((skbdata_used_num > (max_skb_buf_num - MEDIA_PORT_SKB_BUFFER_THRESHOLD)) ||
				(skbbuf_used_num > (max_local_skb_num - MEDIA_PORT_SKB_BUFFER_THRESHOLD))) {
				ret = -1;
				break;
			}

			frame.size = pInputItem->size;
			if (frame.size == 0) {
				LogWarn(("Zero-size frame dropped"));
				ret = -1;
				break;
			}
			frame.pData = (uint8_t *) pvPortMalloc(frame.size);
			if (!frame.pData) {
				LogWarn(("Fail to allocate memory for webrtc media frame, size: %lu", frame.size));
				ret = -1;
				break;
			}

			memcpy(frame.pData,
				   (uint8_t *)pInputItem->data_addr,
				   frame.size);
			/*
			 * Ownership contract: frame.pData is allocated here and freed
			 * downstream. The callback (onVideoFrameReadyToSendFunc /
			 * onAudioFrameReadyToSendFunc) sends the frame via
			 * AppMediaSource_RecvFrame into a message queue. The queue
			 * consumer (VideoTx_Task / AudioTx_Task in app_media_source.c)
			 * checks freeData after onMediaSinkHookFunc returns and always
			 * calls vPortFree(pData) when freeData is 1. No explicit free
			 * is needed on the success path here.
			 */
			frame.freeData = 1;
			frame.timestampUs = NetworkingUtils_GetCurrentTimeUs(&pInputItem->timestamp);

			if ((pInputItem->type == AV_CODEC_ID_H264) || (pInputItem->type == AV_CODEC_ID_H265)) {
				if (pCtx->onVideoFrameReadyToSendFunc) {
					frame.trackKind = TRANSCEIVER_TRACK_KIND_VIDEO;
					(void) pCtx->onVideoFrameReadyToSendFunc(pCtx->pOnVideoFrameReadyToSendCustomContext,
							&frame);
				} else {
					LogError(("No available ready to send callback function pointer for video."));
					vPortFree(frame.pData);
					ret = -1;
				}
			} else if ((pInputItem->type == AV_CODEC_ID_OPUS) ||
					   (pInputItem->type == AV_CODEC_ID_PCMU)) {
				if (pCtx->onAudioFrameReadyToSendFunc) {
					frame.trackKind = TRANSCEIVER_TRACK_KIND_AUDIO;
					(void) pCtx->onAudioFrameReadyToSendFunc(pCtx->pOnAudioFrameReadyToSendCustomContext,
							&frame);
				} else {
					LogError(("No available ready to send callback function pointer for audio."));
					vPortFree(frame.pData);
					ret = -1;
				}
			} else {
				LogWarn(("Input type cannot be handled: %ld", pInputItem->type));
				vPortFree(frame.pData);
				ret = -1;
			}
		} while (pdFALSE);
	}

	return ret;
}

static void local_webrtc_main_thread(void *param)
{
	(void) param;

	LogInfo(("[Local WebRTC module]: === Local WebRTC Start ==="));
	local_webrtc_master_entry();
	LogInfo(("[Local WebRTC module]: local_webrtc_master_entry called, master task created"));
	vTaskDelete(NULL);
}

static int ControlModuleHook(void *p, int cmd, int arg)
{
	MediaModuleContext_t *pCtx = (MediaModuleContext_t *)p;

	switch (cmd) {
	case CMD_LOCAL_WEBRTC_SET_APPLY:
		if (xTaskCreate(local_webrtc_main_thread, "local_webrtc_main", 2048, (void *)pCtx, tskIDLE_PRIORITY + 1,
						&pCtx->local_webrtc_module_main_task) != pdPASS) {
			LogError(("[Local WebRTC module]: xTaskCreate(local_webrtc_main) failed"));
		}
		break;
	case CMD_LOCAL_WEBRTC_START:
		pCtx->mediaStart = 1;
		break;
	case CMD_LOCAL_WEBRTC_STOP:
		pCtx->mediaStart = 0;
		break;
	case CMD_LOCAL_WEBRTC_REG_VIDEO_SEND_CALLBACK:
		pCtx->onVideoFrameReadyToSendFunc = (OnFrameReadyToSend_t) arg;
		break;
	case CMD_LOCAL_WEBRTC_REG_VIDEO_SEND_CALLBACK_CUSTOM_CONTEXT:
		pCtx->pOnVideoFrameReadyToSendCustomContext = (void *) arg;
		break;
	case CMD_LOCAL_WEBRTC_REG_AUDIO_SEND_CALLBACK:
		pCtx->onAudioFrameReadyToSendFunc = (OnFrameReadyToSend_t) arg;
		break;
	case CMD_LOCAL_WEBRTC_REG_AUDIO_SEND_CALLBACK_CUSTOM_CONTEXT:
		pCtx->pOnAudioFrameReadyToSendCustomContext = (void *) arg;
		break;
	default:
		LogWarn(("Unknown module command: %d", cmd));
		break;
	}
	return 0;
}

static void *DestroyModuleHook(void *p)
{
	MediaModuleContext_t *ctx = (MediaModuleContext_t *)p;
	if (ctx) {
		vPortFree(ctx);
	}
	return NULL;
}

static void *CreateModuleHook(void *parent)
{
	MediaModuleContext_t *ctx = pvPortMalloc(sizeof(MediaModuleContext_t));

	if (ctx) {
		memset(ctx, 0, sizeof(MediaModuleContext_t));
		ctx->pParent = parent;
	}

	return ctx;
}

static void *NewModuleItemHook(void *p)
{
	void *pBuffer = pvPortMalloc(MEDIA_PORT_WEBRTC_QUEUE_ITEM_SIZE);

	(void) p;

	if (pBuffer == NULL) {
		LogError(("Fail to allocate buffer for module item."));
	}

	return pBuffer;
}

static void *DeleteModuleItemHook(void *p, void *d)
{
	(void) p;

	if (d != NULL) {
		vPortFree(d);
	}

	return NULL;
}

mm_module_t local_webrtc_module  = {
	.create = CreateModuleHook,
	.destroy = DestroyModuleHook,
	.control = ControlModuleHook,
	.handle = HandleModuleFrameHook,

	.new_item = NewModuleItemHook,
	.del_item = DeleteModuleItemHook,

	.output_type = MM_TYPE_ASINK,
	.module_type = MM_TYPE_AVSINK,
	.name = "Local_WebRTC"
};
