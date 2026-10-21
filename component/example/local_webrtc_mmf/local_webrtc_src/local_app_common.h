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

#ifndef LOCAL_APP_COMMON_H
#define LOCAL_APP_COMMON_H

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "sdp_controller.h"
#include "peer_connection.h"
#include "app_media_source.h"

#define DEMO_SDP_BUFFER_MAX_LENGTH                        (10000)
/* PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH is defined in
   peer_connection_data_types.h — do not redefine it here. */
#define DEMO_TRANSCEIVER_MEDIA_INDEX_VIDEO                (0)
#define DEMO_TRANSCEIVER_MEDIA_INDEX_AUDIO                (1)
#define REMOTE_ID_MAX_LENGTH                              (256)
#define LOCAL_WEBRTC_MAX_VIEWER_NUM                       (1)

#define DEMO_ICE_CANDIDATE_JSON_IPV4_TEMPLATE \
    "candidate:%u 1 udp %lu %d.%d.%d.%d %d typ %s"
#define DEMO_ICE_CANDIDATE_JSON_IPV6_TEMPLATE \
    "candidate:%u 1 udp %lu %02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X " \
    "%d typ %s raddr ::/0 rport 0 generation 0"
#define DEMO_JSON_CANDIDATE_MAX_LENGTH                    (512)
#define LOCAL_WEBRTC_MAX_CANDIDATES                       (20)

struct AppMediaSourcesContext;
typedef struct AppMediaSourcesContext AppMediaSourcesContext_t;

typedef int32_t (* InitTransceiverFunc_t)(void *pCtx,
		TransceiverTrackKind_t trackKind,
		Transceiver_t *pTranceiver);

typedef struct AppSession {
	char remoteClientId[REMOTE_ID_MAX_LENGTH];
	size_t remoteClientIdLength;

	PeerConnectionSession_t peerConnectionSession;
	Transceiver_t transceivers[PEER_CONNECTION_TRANSCEIVER_MAX_COUNT];
} AppSession_t;

typedef struct AppContext {
	char sdpConstructedBuffer[PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH];
	size_t sdpConstructedBufferLength;

	char sdpBuffer[PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH];

	AppSession_t appSessions[LOCAL_WEBRTC_MAX_VIEWER_NUM];

	InitTransceiverFunc_t initTransceiverFunc;
	AppMediaSourcesContext_t *pAppMediaSourcesContext;

	/* SDP offer from browser (via HTTP POST) */
	char receivedOffer[PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH];
	size_t receivedOfferLength;

	/* SDP answer to send back to browser */
	char finalAnswer[PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH];
	size_t finalAnswerLength;

	/* ICE candidate collection (non-trickle ICE) */
	char localCandidates[LOCAL_WEBRTC_MAX_CANDIDATES][DEMO_JSON_CANDIDATE_MAX_LENGTH];
	volatile size_t localCandidateCount;
	/* HTTP server */
	TaskHandle_t httpServerTaskHandle;
} AppContext_t;

int LocalAppCommon_Init(AppContext_t *pAppContext,
						InitTransceiverFunc_t initTransceiverFunc,
						void *pMediaContext);

int LocalAppCommon_StartPeerConnection(AppContext_t *pAppContext,
									   AppSession_t *pAppSession);

int LocalAppCommon_HandleSdpOffer(AppContext_t *pAppContext,
								  const char *pSdpOffer,
								  size_t sdpOfferLength,
								  char *pAnswerSdp,
								  size_t *pAnswerSdpLength);

void LocalAppCommon_StartHttpServer(AppContext_t *pAppContext);

#endif /* LOCAL_APP_COMMON_H */
