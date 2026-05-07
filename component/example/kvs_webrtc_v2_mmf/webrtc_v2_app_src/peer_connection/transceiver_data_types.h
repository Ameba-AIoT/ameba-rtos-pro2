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

#ifndef TRANSCEIVER_DATA_TYPES_H
#define TRANSCEIVER_DATA_TYPES_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Standard includes. */
#include <stdio.h>

#define TRANSCEIVER_STREAM_ID_MAX_LENGTH ( 256 )
#define TRANSCEIVER_TRACK_ID_MAX_LENGTH ( 256 )
#define TRANSCEIVER_CODEC_STRING_MAX_LENGTH ( 3 ) /* The maximum value of codec is now 127, which has length 3 in string. */

#define TRANSCEIVER_IS_CODEC_ENABLED( bitmap, bit ) ( bitmap & ( 1 << bit ) )
#define TRANSCEIVER_ENABLE_CODEC( bitmap, bit ) ( bitmap |= ( 1 << bit ) )

typedef enum TransceiverCallbackEvent
{
    TRANSCEIVER_CB_EVENT_NONE = 0,
    TRANSCEIVER_CB_EVENT_REMOTE_PEER_READY,
    TRANSCEIVER_CB_EVENT_REMOTE_PEER_CLOSED,
    TRANSCEIVER_CB_EVENT_MAX,
} TransceiverCallbackEvent_t;

typedef struct TransceiverRemotePeerReadyMsg
{
    void * pContext;
} TransceiverRemotePeerReadyMsg_t;

typedef struct TransceiverCallbackContent
{
    union
    {
        void * pContext; /* TRANSCEIVER_CB_EVENT_REMOTE_PEER_READY */
    };
} TransceiverCallbackContent_t;

typedef int32_t (* OnPcEventCallback_t)( void * pCustomContext,
                                         TransceiverCallbackEvent_t event,
                                         TransceiverCallbackContent_t * pEventMsg );

typedef enum TransceiverDefaultRtcCodec
{
    TRANSCEIVER_RTC_CODEC_DEFAULT_PAYLOAD_MULAW = 0,
    TRANSCEIVER_RTC_CODEC_DEFAULT_PAYLOAD_ALAW = 8,
    TRANSCEIVER_RTC_CODEC_DEFAULT_PAYLOAD_VP8 = 96,
    TRANSCEIVER_RTC_CODEC_DEFAULT_PAYLOAD_OPUS = 111,
    TRANSCEIVER_RTC_CODEC_DEFAULT_PAYLOAD_H264 = 125,
    TRANSCEIVER_RTC_CODEC_DEFAULT_PAYLOAD_H265 = 127,
    TRANSCEIVER_RTC_CODEC_DEFAULT_PAYLOAD_UNKNOWN = 0xFF,
} TransceiverDefaultRtcCodec_t;

typedef enum TransceiverRtcCodecBit
{
    TRANSCEIVER_RTC_CODEC_UNKNOWN_BIT = 0,
    TRANSCEIVER_RTC_CODEC_MULAW_BIT = 1,
    TRANSCEIVER_RTC_CODEC_ALAW_BIT = 2,
    TRANSCEIVER_RTC_CODEC_VP8_BIT = 3,
    TRANSCEIVER_RTC_CODEC_OPUS_BIT = 4,
    TRANSCEIVER_RTC_CODEC_H264_PROFILE_42E01F_LEVEL_ASYMMETRY_ALLOWED_PACKETIZATION_BIT = 5,
    TRANSCEIVER_RTC_CODEC_H265_BIT = 6,
    TRANSCEIVER_RTC_CODEC_AAC_BIT = 7,
    TRANSCEIVER_RTC_CODEC_NUM = 8,
} TransceiverRtcCodecBit_t;

typedef enum TransceiverTrackKind
{
    TRANSCEIVER_TRACK_KIND_UNKNOWN = 0,
    TRANSCEIVER_TRACK_KIND_AUDIO, //!< Audio track. Track information is set before add transceiver
    TRANSCEIVER_TRACK_KIND_VIDEO, //!< Video track. Track information is set before add transceiver
    TRANSCEIVER_TRACK_KIND_DATA_CHANNEL,
} TransceiverTrackKind_t;

typedef enum TransceiverDirection
{
    TRANSCEIVER_TRACK_DIRECTION_UNKNOWN = 0,
    TRANSCEIVER_TRACK_DIRECTION_SENDRECV,
    TRANSCEIVER_TRACK_DIRECTION_SENDONLY,
    TRANSCEIVER_TRACK_DIRECTION_RECVONLY,
    TRANSCEIVER_TRACK_DIRECTION_INACTIVE,
} TransceiverDirection_t;

typedef struct TransceiverRtcpStats
{
    /* RTCP Sender Report Stats. */
    uint64_t rtpPacketsTransmitted;    // Total number of bytes sent for this SSRC. Calculated as defined in [RFC3550] section 6.4.1.
    uint64_t rtpBytesTransmitted;      // The total number of payload octets (i.e., not including header or padding )
} TransceiverRtcpStats_t;

typedef struct TransceiverRtpSender
{
    /* RTCP Sender Report Stats. */
    uint64_t rtpTimeOffset;
    uint64_t rtpFirstFrameWallClockTimeUs;
} TransceiverRtpSender_t;

typedef struct Transceiver
{
    TransceiverTrackKind_t trackKind;
    TransceiverDirection_t direction;
    uint32_t codecBitMap; // Use TransceiverRtcCodecBit_t to set corresponding bits
    uint32_t rollingbufferDurationSec;
    uint32_t rollingbufferBitRate; // bps
    char streamId[ TRANSCEIVER_STREAM_ID_MAX_LENGTH ];
    size_t streamIdLength;
    char trackId[ TRANSCEIVER_TRACK_ID_MAX_LENGTH ];
    size_t trackIdLength;
    uint32_t ssrc;
    uint32_t rtxSsrc;

    OnPcEventCallback_t onPcEventCallbackFunc;
    void * pOnPcEventCustomContext;

    TransceiverRtcpStats_t rtcpStats;
    TransceiverRtpSender_t rtpSender;
} Transceiver_t;

#ifdef __cplusplus
}
#endif

#endif /* TRANSCEIVER_DATA_TYPES_H */
