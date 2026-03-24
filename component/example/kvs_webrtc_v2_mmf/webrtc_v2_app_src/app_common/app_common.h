/*
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef APP_COMMON_H
#define APP_COMMON_H

#include <stdio.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#include "sdp_controller.h"
#include "signaling_controller.h"
#include "peer_connection.h"

#define DEMO_SDP_BUFFER_MAX_LENGTH ( 10000 )
#define DEMO_TRANSCEIVER_MEDIA_INDEX_VIDEO ( 0 )
#define DEMO_TRANSCEIVER_MEDIA_INDEX_AUDIO ( 1 )
#define REMOTE_ID_MAX_LENGTH    ( 256 )

struct AppMediaSourcesContext;
typedef struct AppMediaSourcesContext AppMediaSourcesContext_t;

typedef int32_t ( * InitTransceiverFunc_t )( void * pCtx,
                                             TransceiverTrackKind_t trackKind,
                                             Transceiver_t * pTranceiver );

typedef struct AppSession
{
    /* The remote client ID, representing the remote peer, from signaling message. */
    char remoteClientId[ REMOTE_ID_MAX_LENGTH ];
    size_t remoteClientIdLength;

    /* Configuration. */
    uint8_t canTrickleIce;

    /* Peer connection session. */
    PeerConnectionSession_t peerConnectionSession;
    Transceiver_t transceivers[ PEER_CONNECTION_TRANSCEIVER_MAX_COUNT ];

    /* Initialized signaling controller. */
    SignalingControllerContext_t * pSignalingControllerContext;
} AppSession_t;

typedef struct AppContext
{
    /* Signaling controller. */
    EventGroupHandle_t signalingConnectionBarrier;
    SignalingControllerContext_t signalingControllerContext;
    TaskHandle_t * pSignalingControllerTaskHandler;
    char signalingControllerClientId[ SIGNALING_CONTROLLER_REMOTE_ID_MAX_LENGTH ];
    size_t signalingControllerClientIdLength;
    SignalingRole_t signalingControllerRole;

    char sdpConstructedBuffer[ PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH ];
    size_t sdpConstructedBufferLength;

    char sdpBuffer[ PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH ];

    /* Peer Connection. */
    AppSession_t appSessions[ AWS_MAX_VIEWER_NUM ];

    /* Media context. */
    InitTransceiverFunc_t initTransceiverFunc;
    AppMediaSourcesContext_t * pAppMediaSourcesContext;

    IceControllerNatTraversalConfig_t natTraversalConfig;
} AppContext_t;

int AppCommon_Init( AppContext_t * pAppContext,
                    InitTransceiverFunc_t initTransceiverFunc,
                    void * pMediaContext );
int AppCommon_StartSignalingController( AppContext_t * pAppContext );
void AppCommon_WaitSignalingControllerStop( AppContext_t * pAppContext );
AppSession_t * AppCommon_GetPeerConnectionSession( AppContext_t * pAppContext,
                                                   const char * pRemoteClientId,
                                                   size_t remoteClientIdLength );

#endif /* APP_COMMON_H */
