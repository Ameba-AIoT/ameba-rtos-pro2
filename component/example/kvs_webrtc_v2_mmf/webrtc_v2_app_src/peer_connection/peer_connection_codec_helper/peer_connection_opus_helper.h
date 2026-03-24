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

#ifndef PEER_CONNECTION_OPUS_HELPER_H
#define PEER_CONNECTION_OPUS_HELPER_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Standard includes. */
#include <stdint.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"

#include "peer_connection_data_types.h"

PeerConnectionResult_t PeerConnectionOpusHelper_GetOpusPacketProperty( PeerConnectionJitterBufferPacket_t * pPacket,
                                                                       uint8_t * pIsStartPacket );

PeerConnectionResult_t PeerConnectionOpusHelper_FillFrameOpus( PeerConnectionJitterBuffer_t * pJitterBuffer,
                                                               uint16_t rtpSeqStart,
                                                               uint16_t rtpSeqEnd,
                                                               uint8_t * pOutBuffer,
                                                               size_t * pOutBufferLength,
                                                               uint32_t * pRtpTimestamp );

PeerConnectionResult_t PeerConnectionOpusHelper_WriteOpusFrame( PeerConnectionSession_t * pSession,
                                                                Transceiver_t * pTransceiver,
                                                                const PeerConnectionFrame_t * pFrame );

#ifdef __cplusplus
}
#endif

#endif /* PEER_CONNECTION_OPUS_HELPER_H */
