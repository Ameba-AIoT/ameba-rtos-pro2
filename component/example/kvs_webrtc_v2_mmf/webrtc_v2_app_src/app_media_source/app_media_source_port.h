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

#ifndef APP_MEDIA_SOURCE_PORT_H
#define APP_MEDIA_SOURCE_PORT_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "transceiver_data_types.h"

typedef struct MediaFrame {
    uint8_t * pData;
    uint32_t size;
    uint64_t timestampUs;
    TransceiverTrackKind_t trackKind;
    uint8_t freeData;  /* indicate user need to free pData after using it */
} MediaFrame_t;

typedef int32_t (* OnFrameReadyToSend_t)( void * pCtx,
                                          MediaFrame_t * pFrame );

int32_t AppMediaSourcePort_Init( void );
int32_t AppMediaSourcePort_Start( OnFrameReadyToSend_t onVideoFrameReadyToSendFunc,
                                  void * pOnVideoFrameReadyToSendCustomContext,
                                  OnFrameReadyToSend_t onAudioFrameReadyToSendFunc,
                                  void * pOnAudioFrameReadyToSendCustomContext );
void AppMediaSourcePort_Stop( void );
void AppMediaSourcePort_Destroy( void );
void AppMediaSourcePort_PlayAudioFrame( MediaFrame_t * pFrame );

#ifdef __cplusplus
}
#endif

#endif /* APP_MEDIA_SOURCE_PORT_H */
