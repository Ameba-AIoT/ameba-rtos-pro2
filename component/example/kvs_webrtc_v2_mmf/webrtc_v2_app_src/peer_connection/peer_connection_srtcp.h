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

#ifndef PEER_CONNECTION_SRTCP_H
#define PEER_CONNECTION_SRTCP_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>

#include "peer_connection_data_types.h"

/* 28 Bytes of RTCP with 0 Reception Reports + 14 bytes of SRTCP */
#define PEER_CONNECTION_SRTCP_RTCP_PACKET_MIN_LENGTH      ( 42 )

PeerConnectionResult_t PeerConnectionSrtp_HandleSrtcpPacket( PeerConnectionSession_t * pSession,
                                                             uint8_t * pBuffer,
                                                             size_t bufferLength );
PeerConnectionResult_t PeerConnectionSrtcp_ConstructSenderReportPacket( PeerConnectionSession_t * pSession,
                                                                        RtcpSenderReport_t * pSenderReport,
                                                                        uint8_t * pOutputSrtcpPacket,
                                                                        size_t * pOutputSrtcpPacketLength );

#ifdef __cplusplus
}
#endif

#endif /* PEER_CONNECTION_SRTCP_H */