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

#ifndef PEER_CONNECTION_JITTER_BUFFER_H
#define PEER_CONNECTION_JITTER_BUFFER_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Standard includes. */
#include <stdint.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"

#include "peer_connection_data_types.h"

PeerConnectionResult_t PeerConnectionJitterBuffer_Create( PeerConnectionJitterBuffer_t * pJitterBuffer,
                                                          OnJitterBufferFrameReadyCallback_t onFrameReadyCallbackFunc,
                                                          void * pOnFrameReadyCallbackContext,
                                                          OnJitterBufferFrameDropCallback_t onFrameDropCallbackFunc,
                                                          void * pOnFrameDropCallbackContext,
                                                          uint32_t tolerenceBufferSec,  // buffer time in seconds
                                                          uint32_t codec,
                                                          uint32_t clockRate );

void PeerConnectionJitterBuffer_Free( PeerConnectionJitterBuffer_t * pJitterBuffer );

PeerConnectionResult_t PeerConnectionJitterBuffer_AllocateBuffer( PeerConnectionJitterBuffer_t * pJitterBuffer,
                                                                  PeerConnectionJitterBufferPacket_t ** ppOutPacket,
                                                                  size_t packetBufferSize,
                                                                  uint16_t rtpSeq );

PeerConnectionResult_t PeerConnectionJitterBuffer_GetPacket( PeerConnectionJitterBuffer_t * pJitterBuffer,
                                                             uint16_t rtpSeq,
                                                             PeerConnectionJitterBufferPacket_t ** ppOutPacket );

PeerConnectionResult_t PeerConnectionJitterBuffer_Push( PeerConnectionJitterBuffer_t * pJitterBuffer,
                                                        PeerConnectionJitterBufferPacket_t * pPacket );

PeerConnectionResult_t PeerConnectionJitterBuffer_FillFrame( PeerConnectionJitterBuffer_t * pJitterBuffer,
                                                             uint16_t rtpSeqStart,
                                                             uint16_t rtpSeqEnd,
                                                             uint8_t * pOutBuffer,
                                                             size_t * pOutBufferLength,
                                                             uint32_t * pRtpTimestamp );

#ifdef __cplusplus
}
#endif

#endif /* PEER_CONNECTION_JITTER_BUFFER_H */
