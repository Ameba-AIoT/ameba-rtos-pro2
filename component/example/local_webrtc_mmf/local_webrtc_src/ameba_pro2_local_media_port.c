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

#include "module_local_webrtc.h"
#include "logging.h"
#include "platform_opts.h"

#include "avcodec.h"
#include "video_api.h"

#include "FreeRTOS.h"

int32_t AppMediaSourcePort_Init(void)
{
	LogInfo(("[Local WebRTC Port] AppMediaSourcePort_Init called."));
	return 0;
}

int32_t AppMediaSourcePort_Start(OnFrameReadyToSend_t onVideoFrameReadyToSendFunc,
								 void *pOnVideoFrameReadyToSendCustomContext,
								 OnFrameReadyToSend_t onAudioFrameReadyToSendFunc,
								 void *pOnAudioFrameReadyToSendCustomContext)
{
	int32_t ret = 0;

	mm_module_ctrl(pWebrtcMmContext,
				   CMD_LOCAL_WEBRTC_REG_VIDEO_SEND_CALLBACK,
				   (int)onVideoFrameReadyToSendFunc);
	mm_module_ctrl(pWebrtcMmContext,
				   CMD_LOCAL_WEBRTC_REG_VIDEO_SEND_CALLBACK_CUSTOM_CONTEXT,
				   (int)pOnVideoFrameReadyToSendCustomContext);
	mm_module_ctrl(pWebrtcMmContext,
				   CMD_LOCAL_WEBRTC_REG_AUDIO_SEND_CALLBACK,
				   (int)onAudioFrameReadyToSendFunc);
	mm_module_ctrl(pWebrtcMmContext,
				   CMD_LOCAL_WEBRTC_REG_AUDIO_SEND_CALLBACK_CUSTOM_CONTEXT,
				   (int)pOnAudioFrameReadyToSendCustomContext);
	mm_module_ctrl(pWebrtcMmContext,
				   CMD_LOCAL_WEBRTC_START,
				   0);

	return ret;
}

void AppMediaSourcePort_Stop(void)
{
	mm_module_ctrl(pWebrtcMmContext, CMD_LOCAL_WEBRTC_STOP, 0);
}

void AppMediaSourcePort_Destroy(void)
{
	LogInfo(("[Local WebRTC Port]: AppMediaSourcePort_Destroy called."));
}

void AppMediaSourcePort_PlayAudioFrame(MediaFrame_t *pFrame)
{
	uint8_t skipProcess = 0U;
	mm_queue_item_t *output_item;

	if (pFrame == NULL) {
		LogError(("Invalid input, pFrame: %p", pFrame));
		skipProcess = 1U;
	} else if (pFrame->trackKind != TRANSCEIVER_TRACK_KIND_AUDIO) {
		LogError(("Dropping non-audio frame, track kind: %d", pFrame->trackKind));
		skipProcess = 1U;
	}

	if (skipProcess == 0U) {
		LogDebug(("Playing audio frame with length: %lu", pFrame->size));

		if (xQueueReceive(pWebrtcMmContext->output_recycle, &output_item, pdMS_TO_TICKS(1000)) == pdTRUE) {
			memcpy((void *)output_item->data_addr, (void *)pFrame->pData, pFrame->size);

#if AUDIO_OPUS
			output_item->type = AV_CODEC_ID_OPUS;
#elif AUDIO_G711_MULAW
			output_item->type = AV_CODEC_ID_PCMU;
#elif AUDIO_G711_ALAW
			output_item->type = AV_CODEC_ID_PCMA;
#endif

			output_item->size = pFrame->size;
			output_item->timestamp = pFrame->timestampUs;
			xQueueSend(pWebrtcMmContext->output_ready, (void *)&output_item, 0xFFFFFFFF);
		} else {
			LogWarn(("No free output queue item for audio frame"));
		}
	}
}
