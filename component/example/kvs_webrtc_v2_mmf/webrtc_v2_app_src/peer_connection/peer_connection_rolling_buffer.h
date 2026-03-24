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

#ifndef PEER_CONNECTION_ROLLING_BUFFER_H
#define PEER_CONNECTION_ROLLING_BUFFER_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Standard includes. */
#include <stdint.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"

#include "peer_connection_data_types.h"

#define PEER_CONNECTION_ROLLING_BUFFER_DURATION_IN_SECONDS ( 3 )

PeerConnectionResult_t PeerConnectionRollingBuffer_Create( PeerConnectionRollingBuffer_t * pRollingBuffer,
                                                           uint32_t rollingbufferBitRate,  // bps
                                                           uint32_t rollingbufferDurationSec,  // duration in seconds
                                                           size_t maxSizePerPacket );

void PeerConnectionRollingBuffer_Free( PeerConnectionRollingBuffer_t * pRollingBuffer );

PeerConnectionResult_t PeerConnectionRollingBuffer_GetRtpSequenceBuffer( PeerConnectionRollingBuffer_t * pRollingBuffer,
                                                                         uint16_t rtpSeq,
                                                                         PeerConnectionRollingBufferPacket_t ** ppPacket );

void PeerConnectionRollingBuffer_DiscardRtpSequenceBuffer( PeerConnectionRollingBuffer_t * pRollingBuffer,
                                                           PeerConnectionRollingBufferPacket_t * pPacket );

PeerConnectionResult_t PeerConnectionRollingBuffer_SearchRtpSequenceBuffer( PeerConnectionRollingBuffer_t * pRollingBuffer,
                                                                            uint16_t rtpSeq,
                                                                            PeerConnectionRollingBufferPacket_t ** ppPacket );

PeerConnectionResult_t PeerConnectionRollingBuffer_SetPacket( PeerConnectionRollingBuffer_t * pRollingBuffer,
                                                              uint16_t rtpSeq,
                                                              PeerConnectionRollingBufferPacket_t * pPacket );

#ifdef __cplusplus
}
#endif

#endif /* PEER_CONNECTION_ROLLING_BUFFER_H */
