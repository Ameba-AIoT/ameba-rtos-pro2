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

#ifndef PEER_CONNECTION_CODEC_HELPER_H
#define PEER_CONNECTION_CODEC_HELPER_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include "logging.h"
#include "peer_connection.h"
#include "peer_connection_jitter_buffer.h"
#include "peer_connection_srtp.h"
#include "peer_connection_rolling_buffer.h"
#include "peer_connection_jitter_buffer.h"
#if METRIC_PRINT_ENABLED
#include "metric.h"
#endif

#include "FreeRTOS.h"
#include "ice_controller.h"
#include "networking_utils.h"

#define PEER_CONNECTION_JITTER_BUFFER_MAX_PACKETS_NUM_IN_A_FRAME ( 32 )
#define PEER_CONNECTION_JITTER_BUFFER_SEQ_WRAPPER_THRESHOLD ( 10 )
#define PEER_CONNECTION_JITTER_BUFFER_TIMESTAMP_WRAPPER_THRESHOLD_SEC ( 0.1 )
#define PEER_CONNECTION_JITTER_BUFFER_WRAP( x, max ) ( ( x ) % max )
#define PEER_CONNECTION_JITTER_BUFFER_INCREASE_WITH_WRAP( x, y, max ) ( PEER_CONNECTION_JITTER_BUFFER_WRAP( ( x ) + ( y ),\ max ) )
#define PEER_CONNECTION_JITTER_BUFFER_DECREASE_WITH_WRAP( x, y, max ) ( PEER_CONNECTION_JITTER_BUFFER_WRAP( ( x ) - ( y ),\ max ) )

/* At write frame, we reserve 2 bytes at the beginning of payload buffer for re-transmission if RTX is enabled. */
/* The format of a retransmission packet is shown below:
    0                   1                   2                   3
    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |                         RTP Header                            |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |            OSN                |                               |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+                               |
 |                  Original RTP Packet Payload                  |
 |                                                               |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */
#define PEER_CONNECTION_SRTP_RTX_WRITE_RESERVED_BYTES ( 2 )

#define PEER_CONNECTION_SRTP_H264_MAX_NALUS_IN_A_FRAME        ( 64 )
#define PEER_CONNECTION_SRTP_H265_MAX_NALUS_IN_A_FRAME        ( 64 )
#define PEER_CONNECTION_SRTP_RTP_PAYLOAD_MAX_LENGTH      ( 1200 )

#define PEER_CONNECTION_SRTP_VIDEO_CLOCKRATE ( uint32_t ) 90000
#define PEER_CONNECTION_SRTP_OPUS_CLOCKRATE  ( uint32_t ) 48000
#define PEER_CONNECTION_SRTP_PCM_CLOCKRATE   ( uint32_t ) 8000

#define PEER_CONNECTION_SRTP_US_IN_A_SECOND ( 1000000 )
#define PEER_CONNECTION_SRTP_CONVERT_TIME_US_TO_RTP_TIMESTAMP( clockRate, presentationUs ) ( uint32_t )( ( ( ( presentationUs ) * ( clockRate ) ) / PEER_CONNECTION_SRTP_US_IN_A_SECOND ) & 0xFFFFFFFF )
#define PEER_CONNECTION_SRTP_CONVERT_RTP_TIMESTAMP_TO_TIME_US( clockRate, rtpTimestamp ) ( ( uint64_t )( rtpTimestamp ) * PEER_CONNECTION_SRTP_US_IN_A_SECOND / ( clockRate ) )

#define PEER_CONNECTION_SRTP_JITTER_BUFFER_TOLERENCE_TIME_SECOND ( 2 )

/*
 *
     0                   1                   2                   3
      0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |       0xBE    |    0xDE       |           length=1            |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |  ID   | L=1   |transport-wide sequence number | zero padding  |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */
// https://tools.ietf.org/html/draft-holmer-rmcat-transport-wide-cc-extensions-01
#define PEER_CONNECTION_SRTP_TWCC_EXT_PROFILE ( 0xBEDE )
#define PEER_CONNECTION_SRTP_GET_TWCC_PAYLOAD( extId, sequenceNum ) ( ( ( ( extId ) & 0xfu ) << 28u ) | ( 1u << 24u ) | ( ( uint32_t ) ( sequenceNum ) << 8u ) )
#define PEER_CONNECTION_SRTCP_NACK_MAX_SEQ_NUM ( 128 )

#ifdef __cplusplus
}
#endif

#endif /* PEER_CONNECTION_CODEC_HELPER_H */
