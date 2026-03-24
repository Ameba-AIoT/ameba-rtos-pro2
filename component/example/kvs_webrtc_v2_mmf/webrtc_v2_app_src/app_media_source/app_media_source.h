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

#ifndef APP_MEDIA_SOURCE_H
#define APP_MEDIA_SOURCE_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "message_queue.h"
#include "peer_connection.h"
#include "app_media_source_port.h"

/* FreeRTOS includes. */
#include "semphr.h"

typedef struct AppMediaSourcesContext AppMediaSourcesContext_t;
typedef int32_t (* AppMediaSourceOnMediaSinkHook)( void * pCustom,
                                                   MediaFrame_t * pFrame );

typedef struct AppMediaSourceContext
{
    MessageQueueHandler_t dataTxQueue;
    MessageQueueHandler_t dataRxQueue;
    uint8_t numReadyPeer;
    TransceiverTrackKind_t trackKind;

    AppMediaSourcesContext_t * pSourcesContext;
} AppMediaSourceContext_t;

typedef struct AppMediaSourcesContext
{
    /* Mutex to protect totalNumReadyPeer because we might receive multiple ready/close message from different tasks. */
    SemaphoreHandle_t mediaMutex;

    AppMediaSourceContext_t videoContext;
    AppMediaSourceContext_t audioContext;

    AppMediaSourceOnMediaSinkHook onMediaSinkHookFunc;
    void * pOnMediaSinkHookCustom;
    uint8_t totalNumReadyPeer;
} AppMediaSourcesContext_t;

int32_t AppMediaSource_Init( AppMediaSourcesContext_t * pCtx,
                             AppMediaSourceOnMediaSinkHook onMediaSinkHookFunc,
                             void * pOnMediaSinkHookCustom );
int32_t AppMediaSource_InitVideoTransceiver( AppMediaSourcesContext_t * pCtx,
                                             Transceiver_t * pVideoTranceiver );
int32_t AppMediaSource_InitAudioTransceiver( AppMediaSourcesContext_t * pCtx,
                                             Transceiver_t * pAudioTranceiver );
int32_t AppMediaSource_RecvFrame( AppMediaSourcesContext_t * pCtx,
                                  MediaFrame_t * pFrame );

#ifdef __cplusplus
}
#endif

#endif /* APP_MEDIA_SOURCE_H */
