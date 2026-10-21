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

#ifndef MODULE_LOCAL_WEBRTC_H
#define MODULE_LOCAL_WEBRTC_H

#include "mmf2_module.h"
#include "app_media_source_port.h"

#define CMD_LOCAL_WEBRTC_SET_PARAMS                              MM_MODULE_CMD( 0x00 )
#define CMD_LOCAL_WEBRTC_GET_PARAMS                              MM_MODULE_CMD( 0x01 )
#define CMD_LOCAL_WEBRTC_SET_APPLY                               MM_MODULE_CMD( 0x02 )
#define CMD_LOCAL_WEBRTC_STOP                                    MM_MODULE_CMD( 0x03 )
#define CMD_LOCAL_WEBRTC_START                                   MM_MODULE_CMD( 0x04 )
#define CMD_LOCAL_WEBRTC_REG_VIDEO_SEND_CALLBACK                 MM_MODULE_CMD( 0x05 )
#define CMD_LOCAL_WEBRTC_REG_VIDEO_SEND_CALLBACK_CUSTOM_CONTEXT  MM_MODULE_CMD( 0x06 )
#define CMD_LOCAL_WEBRTC_REG_AUDIO_SEND_CALLBACK                 MM_MODULE_CMD( 0x07 )
#define CMD_LOCAL_WEBRTC_REG_AUDIO_SEND_CALLBACK_CUSTOM_CONTEXT  MM_MODULE_CMD( 0x08 )

typedef struct MediaModuleContext {
	void *pParent;
	TaskHandle_t local_webrtc_module_main_task;

	uint8_t mediaStart;

	OnFrameReadyToSend_t onVideoFrameReadyToSendFunc;
	void *pOnVideoFrameReadyToSendCustomContext;
	OnFrameReadyToSend_t onAudioFrameReadyToSendFunc;
	void *pOnAudioFrameReadyToSendCustomContext;
} MediaModuleContext_t;

extern mm_module_t local_webrtc_module;
extern mm_context_t *pWebrtcMmContext;

#endif /* MODULE_LOCAL_WEBRTC_H */
