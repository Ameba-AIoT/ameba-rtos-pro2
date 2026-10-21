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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "FreeRTOS.h"
#include "task.h"

#include "logging.h"
#include "local_master.h"
#include "local_app_common.h"
#include "app_media_source.h"
#include "module_local_webrtc.h"

static AppContext_t appContext;
static AppMediaSourcesContext_t appMediaSourceContext;

static int32_t InitTransceiver(void *pMediaCtx,
							   TransceiverTrackKind_t trackKind,
							   Transceiver_t *pTranceiver);

static int32_t OnMediaSinkHook(void *pCustom, MediaFrame_t *pFrame);

static int32_t InitTransceiver(void *pMediaCtx,
							   TransceiverTrackKind_t trackKind,
							   Transceiver_t *pTranceiver)
{
	int32_t ret = 0;
	AppMediaSourcesContext_t *pMediaSourceContext = (AppMediaSourcesContext_t *)pMediaCtx;

	if (pMediaCtx == NULL || pTranceiver == NULL) {
		LogError(("Invalid input"));
		return -1;
	}

	switch (trackKind) {
	case TRANSCEIVER_TRACK_KIND_VIDEO:
		ret = AppMediaSource_InitVideoTransceiver(pMediaSourceContext, pTranceiver);
		break;
	case TRANSCEIVER_TRACK_KIND_AUDIO:
		ret = AppMediaSource_InitAudioTransceiver(pMediaSourceContext, pTranceiver);
		break;
	default:
		LogError(("Unknown track kind: %d", trackKind));
		ret = -2;
		break;
	}

	return ret;
}

static int32_t OnMediaSinkHook(void *pCustom, MediaFrame_t *pFrame)
{
	int32_t ret = 0;
	AppContext_t *pAppContext = (AppContext_t *)pCustom;
	PeerConnectionResult_t peerConnectionResult;
	PeerConnectionFrame_t peerConnectionFrame;
	Transceiver_t *pTransceiver = NULL;
	int i;

	if (pAppContext == NULL || pFrame == NULL) {
		return -1;
	}

	peerConnectionFrame.version = PEER_CONNECTION_FRAME_CURRENT_VERSION;
	peerConnectionFrame.presentationUs = pFrame->timestampUs;
	peerConnectionFrame.pData = pFrame->pData;
	peerConnectionFrame.dataLength = pFrame->size;

	for (i = 0; i < LOCAL_WEBRTC_MAX_VIEWER_NUM; i++) {
		if (pFrame->trackKind == TRANSCEIVER_TRACK_KIND_VIDEO) {
			pTransceiver = &pAppContext->appSessions[i].transceivers[DEMO_TRANSCEIVER_MEDIA_INDEX_VIDEO];
		} else if (pFrame->trackKind == TRANSCEIVER_TRACK_KIND_AUDIO) {
			pTransceiver = &pAppContext->appSessions[i].transceivers[DEMO_TRANSCEIVER_MEDIA_INDEX_AUDIO];
		} else {
			break;
		}

		if (pAppContext->appSessions[i].peerConnectionSession.state ==
			PEER_CONNECTION_SESSION_STATE_CONNECTION_READY) {
			peerConnectionResult = PeerConnection_WriteFrame(
									   &pAppContext->appSessions[i].peerConnectionSession,
									   pTransceiver,
									   &peerConnectionFrame);

			if (peerConnectionResult != PEER_CONNECTION_RESULT_OK) {
				LogError(("Fail to write %s frame, result: %d",
						  (pFrame->trackKind == TRANSCEIVER_TRACK_KIND_VIDEO) ? "video" : "audio",
						  peerConnectionResult));
				ret = -3;
			}
		}
	}

	return ret;
}

static int32_t InitializeAppMediaSource(AppContext_t *pAppContext,
										AppMediaSourcesContext_t *pAppMediaSourceContext)
{
	if (pAppContext == NULL || pAppMediaSourceContext == NULL) {
		return -1;
	}

	return AppMediaSource_Init(pAppMediaSourceContext,
							   OnMediaSinkHook,
							   pAppContext);
}

static void Master_Task(void *pParameter)
{
	int32_t ret = 0;

	(void)pParameter;

	LogInfo(("Start Local WebRTC Master_Task."));

	/* Initialize the WebRTC application */
	ret = LocalAppCommon_Init(&appContext, InitTransceiver, &appMediaSourceContext);
	if (ret != 0) {
		LogError(("LocalAppCommon_Init failed"));
		goto exit;
	}

	/* Initialize media source (MMF integration) */
	ret = InitializeAppMediaSource(&appContext, &appMediaSourceContext);
	if (ret != 0) {
		LogError(("InitializeAppMediaSource failed"));
		goto exit;
	}

	/* Start HTTP signaling server (this blocks the task) */
	LogInfo(("Starting HTTP signaling server..."));
	LocalAppCommon_StartHttpServer(&appContext);

	/* Keep task alive */
	for (;;) {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}

exit:
	for (;;) {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void local_webrtc_master_entry(void)
{
	LogInfo(("local_webrtc_master_entry called"));

	if (xTaskCreate(Master_Task,
					"LocalWrtcMaster",
					8192,
					NULL,
					tskIDLE_PRIORITY + 4,
					NULL) != pdPASS) {
		LogError(("xTaskCreate(Master_Task) failed"));
	}
}
