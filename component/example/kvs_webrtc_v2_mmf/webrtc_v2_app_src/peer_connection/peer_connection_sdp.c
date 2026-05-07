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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "logging.h"
#include "sdp_controller.h"
#include "string_utils.h"
#include "peer_connection_sdp.h"

#define PEER_CONNECTION_SDP_ORIGIN_DEFAULT_USER_NAME "-"
#define PEER_CONNECTION_SDP_ORIGIN_DEFAULT_SESSION_VERSION ( 2 )
#define PEER_CONNECTION_SDP_ORIGIN_DEFAULT_NET_TYPE "IN"
#define PEER_CONNECTION_SDP_ORIGIN_IPV4_TYPE "IP4"
#define PEER_CONNECTION_SDP_ORIGIN_DEFAULT_IP_ADDRESS "127.0.0.1"

#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_SETUP "setup"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_SETUP_LENGTH ( 5 )
#define PEER_CONNECTION_SDP_MEDIA_DTLS_ROLE_ACTIVE "active"
#define PEER_CONNECTION_SDP_MEDIA_DTLS_ROLE_ACTIVE_LENGTH ( 6 )
#define PEER_CONNECTION_SDP_MEDIA_DTLS_ROLE_ACTPASS "actpass"
#define PEER_CONNECTION_SDP_MEDIA_DTLS_ROLE_ACTPASS_LENGTH ( 7 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_MSID "msid"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_MSID_LENGTH ( 4 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_RTCP "rtcp"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_RTCP_LENGTH ( 4 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTCP "9 IN IP4 0.0.0.0"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTCP_LENGTH ( 16 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_ICE_UFRAG "ice-ufrag"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_ICE_UFRAG_LENGTH ( 9 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_ICE_PWD "ice-pwd"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_ICE_PWD_LENGTH ( 7 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_ICE_OPTION "ice-options"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_ICE_OPTION_LENGTH ( 11 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_ICE_OPTION "trickle"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_ICE_OPTION_LENGTH ( 7 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_FINGERPRINT "fingerprint"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_FINGERPRINT_LENGTH ( 11 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_SSRC "ssrc"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_SSRC_LENGTH ( 4 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_MID "mid"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_MID_LENGTH ( 3 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_SENDRECV "sendrecv"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_SENDRECV_LENGTH ( 8 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_SENDONLY "sendonly"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_SENDONLY_LENGTH ( 8 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_RECVONLY "recvonly"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_RECVONLY_LENGTH ( 8 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_INACTIVE "inactive"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_INACTIVE_LENGTH ( 8 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_RTCP_MUX "rtcp-mux"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_RTCP_MUX_LENGTH ( 8 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_RTCP_RSIZE "rtcp-rsize"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_RTCP_RSIZE_LENGTH ( 10 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_RTPMAP "rtpmap"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_RTPMAP_LENGTH ( 6 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTPMAP_H264 "H264/90000"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTPMAP_H264_LENGTH ( 10 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTPMAP_OPUS "opus/48000/2"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTPMAP_OPUS_LENGTH ( 12 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTPMAP_VP8 "VP8/90000"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTPMAP_VP8_LENGTH ( 9 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTPMAP_MULAW "PCMU/8000"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTPMAP_MULAW_LENGTH ( 9 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTPMAP_ALAW "PCMA/8000"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTPMAP_ALAW_LENGTH ( 9 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTPMAP_H265 "H265/90000"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTPMAP_H265_LENGTH ( 10 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_RTCP_FB "rtcp-fb"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_RTCP_FB_LENGTH ( 7 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_H264 "nack pli"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_H264_LENGTH ( 8 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_H265 PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_H264
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_H265_LENGTH PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_H264_LENGTH
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_GOOG_REMB "goog-remb"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_GOOG_REMB_LENGTH ( 9 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_FMTP "fmtp"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_FMTP_LENGTH ( 4 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_EXTMAP "extmap"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_EXTMAP_LENGTH ( 6 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_TWCC_EXT_URL "http://www.ietf.org/id/draft-holmer-rmcat-transport-wide-cc-extensions-01"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_TWCC_EXT_URL_LENGTH ( 73 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_TRANSPORT_CC "transport-cc"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_TRANSPORT_CC_LENGTH ( 12 )
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_CANDIDATE "candidate"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_CANDIDATE_LENGTH ( 9 )

#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_FMTP_H264_PROFILE_42E01F_LEVEL_ASYMMETRY_ALLOWED_PACKETIZATION "level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=42e01f"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_FMTP_OPUS "minptime=10;useinbandfec=1"
#define PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_VALUE_FMTP_H265 "profile-space=0;profile-id=0;tier-flag=0;level-id=0;interop-constraints=000000000000;sprop-vps=QAEMAf//" \
                                                            "AIAAAAMAAAMAAAMAAAMAALUCQA==;sprop-sps=QgEBAIAAAAMAAAMAAAMAAAMAAKACgIAtH+W1kkbQzkkktySqSfKSyA==;sprop-pps=RAHBpVgeSA=="

#define PEER_CONNECTION_SDP_H264_PACKETIZATION_MODE "packetization-mode=1"
#define PEER_CONNECTION_SDP_H264_PACKETIZATION_MODE_LENGTH ( 20 )
#define PEER_CONNECTION_SDP_H264_ASYMMETRY_ALLOWED "level-asymmetry-allowed=1"
#define PEER_CONNECTION_SDP_H264_ASYMMETRY_ALLOWED_LENGTH ( 25 )
#define PEER_CONNECTION_SDP_H264_PROFILE_LEVEL_ID "profile-level-id="
#define PEER_CONNECTION_SDP_H264_PROFILE_LEVEL_ID_LENGTH ( 17 )

#define PEER_CONNECTION_SDP_MAX_FMTP_APT_NUM ( 64 )

#define PEER_CONNECTION_SDP_GET_APT_CODEC_FROM_PAYLOAD( payload ) ( payload & 0xFF )
#define PEER_CONNECTION_SDP_GET_RTX_CODEC_FROM_PAYLOAD( payload ) ( payload >> 16 )
#define PEER_CONNECTION_SDP_SET_PAYLOAD( rtxPayload, aptPayload ) ( rtxPayload << 16 | aptPayload )


// profile-level-id:
//   A base16 [7] (hexadecimal) representation of the following
//   three bytes in the sequence parameter set NAL unit is specified
//   in [1]: 1) profile_idc, 2) a byte herein referred to as
//   profile-iop, composed of the values of constraint_set0_flag,
//   constraint_set1_flag, constraint_set2_flag,
//   constraint_set3_flag, constraint_set4_flag,
//   constraint_set5_flag, and reserved_zero_2bits in bit-
//   significance order, starting from the most-significant bit, and
//   3) level_id.
//
// Reference: https://tools.ietf.org/html/rfc6184#section-8.1
#define PEER_CONNECTION_SDP_H264_PROFILE_42E01F 0x42e01f
#define PEER_CONNECTION_SDP_H264_FMTP_SUBPROFILE_MASK 0xFFFF00
#define PEER_CONNECTION_SDP_H264_FMTP_PROFILE_LEVEL_MASK 0x0000FF
#define PEER_CONNECTION_SDP_H264_FMTP_MINIMUM_SCORE ( 10 )
#define PEER_CONNECTION_SDP_H264_FMTP_HIGHEST_SCORE ( 12 )

#define PEER_CONNECTION_SDP_CODEC_H264_VALUE "H264/90000"
#define PEER_CONNECTION_SDP_CODEC_H264_VALUE_LENGTH ( 10 )
#define PEER_CONNECTION_SDP_CODEC_H265_VALUE "H265/90000"
#define PEER_CONNECTION_SDP_CODEC_H265_VALUE_LENGTH ( 10 )
#define PEER_CONNECTION_SDP_CODEC_OPUS_VALUE "opus/48000/2"
#define PEER_CONNECTION_SDP_CODEC_OPUS_VALUE_LENGTH ( 12 )
#define PEER_CONNECTION_SDP_CODEC_VP8_VALUE "VP8/90000"
#define PEER_CONNECTION_SDP_CODEC_VP8_VALUE_LENGTH ( 9 )
#define PEER_CONNECTION_SDP_CODEC_MULAW_VALUE "PCMU/8000"
#define PEER_CONNECTION_SDP_CODEC_MULAW_VALUE_LENGTH ( 9 )
#define PEER_CONNECTION_SDP_CODEC_ALAW_VALUE "PCMA/8000"
#define PEER_CONNECTION_SDP_CODEC_ALAW_VALUE_LENGTH ( 9 )
#define PEER_CONNECTION_SDP_CODEC_RTX_VALUE "rtx/90000"
#define PEER_CONNECTION_SDP_CODEC_RTX_VALUE_LENGTH ( 9 )
#define PEER_CONNECTION_SDP_CODEC_APT_VALUE "apt="
#define PEER_CONNECTION_SDP_CODEC_APT_VALUE_LENGTH ( 4 )

#define PEER_CONNECTION_SDP_CODEC_MULAW_DEFAULT_INDEX "0"
#define PEER_CONNECTION_SDP_CODEC_MULAW_DEFAULT_INDEX_LENGTH ( 1 )
#define PEER_CONNECTION_SDP_CODEC_ALAW_DEFAULT_INDEX "8"
#define PEER_CONNECTION_SDP_CODEC_ALAW_DEFAULT_INDEX_LENGTH ( 1 )

static SdpControllerAttributes_t * FindH264FmtpAttribute( SdpControllerAttributes_t * pAttributes,
                                                          uint8_t attributeCount,
                                                          SdpControllerAttributes_t * pTargetRtpmapAttribute )
{
    const char * pCodecStart = NULL;
    size_t codecStringLength = 0;
    int i;
    SdpControllerAttributes_t * pTargetFmtpAttribute = NULL;

    if( pAttributes && pTargetRtpmapAttribute )
    {
        /* Find the corresponding codec payload from target RTPMAP attribute. */
        pCodecStart = pTargetRtpmapAttribute->pAttributeValue;
        while( pCodecStart && codecStringLength < pTargetRtpmapAttribute->attributeValueLength )
        {
            if( ( pCodecStart[ codecStringLength ] >= '0' ) && ( pCodecStart[ codecStringLength ] <= '9' ) )
            {
                codecStringLength++;
            }
            else
            {
                break;
            }
        }

        /* Find corresonding fmtp attribute. */
        if( codecStringLength )
        {
            for( i = 0; i < attributeCount; i++ )
            {
                if( ( pAttributes[i].attributeNameLength == PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_FMTP_LENGTH ) &&
                    ( strncmp( PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_FMTP, pAttributes[i].pAttributeName, PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_FMTP_LENGTH ) == 0 ) &&
                    ( pAttributes[i].attributeValueLength >= codecStringLength ) &&
                    ( strncmp( pCodecStart, pAttributes[i].pAttributeValue, codecStringLength ) == 0 ) )
                {
                    /* Found the fmtp. */
                    pTargetFmtpAttribute = &pAttributes[i];
                }
            }
        }
    }

    return pTargetFmtpAttribute;
}

static uint32_t CalculateH264ScoreByFmtp( SdpControllerAttributes_t * pTargetFmtpAttribute )
{
    uint32_t score = 0;
    const char * pProfileLevelIdStart = NULL;
    uint32_t profileLevelId = 0;
    StringUtilsResult_t stringResult;
    size_t remainLength = 0;

    do
    {
        if( !pTargetFmtpAttribute )
        {
            /* No target fmtp found, return 0. */
            break;
        }

        /* Calculate the score from fmtp. */
        if( StringUtils_StrStr( pTargetFmtpAttribute->pAttributeValue, pTargetFmtpAttribute->attributeValueLength,
                                PEER_CONNECTION_SDP_H264_PACKETIZATION_MODE, PEER_CONNECTION_SDP_H264_PACKETIZATION_MODE_LENGTH ) )
        {
            /* Packetization mode is mandatory. */
            score += PEER_CONNECTION_SDP_H264_FMTP_MINIMUM_SCORE;
        }

        if( StringUtils_StrStr( pTargetFmtpAttribute->pAttributeValue, pTargetFmtpAttribute->attributeValueLength,
                                PEER_CONNECTION_SDP_H264_ASYMMETRY_ALLOWED, PEER_CONNECTION_SDP_H264_ASYMMETRY_ALLOWED_LENGTH ) )
        {
            score++;
        }

        pProfileLevelIdStart = StringUtils_StrStr( pTargetFmtpAttribute->pAttributeValue, pTargetFmtpAttribute->attributeValueLength,
                                                   PEER_CONNECTION_SDP_H264_PROFILE_LEVEL_ID, PEER_CONNECTION_SDP_H264_PROFILE_LEVEL_ID_LENGTH );
        if( !pProfileLevelIdStart )
        {
            break;
        }

        /* Move pProfileLevelIdStart to the start of ID. */
        pProfileLevelIdStart = pProfileLevelIdStart + PEER_CONNECTION_SDP_H264_PROFILE_LEVEL_ID_LENGTH;
        remainLength = pTargetFmtpAttribute->pAttributeValue + pTargetFmtpAttribute->attributeValueLength - pProfileLevelIdStart;
        stringResult = StringUtils_ConvertStringToHex( pProfileLevelIdStart, remainLength, &profileLevelId );
        if( stringResult != STRING_UTILS_RESULT_OK )
        {
            LogWarn( ( "Fail to convert string(%d): %.*s to hex.",
                       remainLength,
                       ( int ) remainLength, pProfileLevelIdStart ) );
            break;
        }

        if( ( ( profileLevelId & PEER_CONNECTION_SDP_H264_FMTP_SUBPROFILE_MASK ) == ( PEER_CONNECTION_SDP_H264_PROFILE_42E01F & PEER_CONNECTION_SDP_H264_FMTP_SUBPROFILE_MASK ) ) &&
            ( ( profileLevelId & PEER_CONNECTION_SDP_H264_FMTP_PROFILE_LEVEL_MASK ) <= ( PEER_CONNECTION_SDP_H264_PROFILE_42E01F & PEER_CONNECTION_SDP_H264_FMTP_PROFILE_LEVEL_MASK ) ) )
        {
            score++;
        }
    } while( 0 );

    return score;
}

static uint32_t CollectAttributesCodec( SdpControllerAttributes_t * pAttributes,
                                        uint8_t attributeCount,
                                        uint32_t codecPayloads[TRANSCEIVER_RTC_CODEC_NUM] )
{
    uint32_t codecBitMap = 0, h264Score = 0, highestH264Score = 0;
    int i, j;
    StringUtilsResult_t stringResult;
    SdpControllerAttributes_t * pH264FmtpAttribute = NULL;
    const char * pAtp = NULL;
    size_t stringLength;
    uint32_t rtxPayload;
    uint32_t aptPayload;

    if( ( pAttributes == NULL ) ||
        ( codecPayloads == NULL ) )
    {
        LogError( ( "Invalid input, pAttributes: %p,codecPayloads: %p",
                    pAttributes,
                    codecPayloads ) );
    }
    else
    {
        for( i = 0; i < attributeCount; i++ )
        {
            if( ( pAttributes[i].attributeNameLength == PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_RTPMAP_LENGTH ) &&
                ( strncmp( PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_RTPMAP, pAttributes[i].pAttributeName, PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_RTPMAP_LENGTH ) == 0 ) )
            {
                if( ( highestH264Score < PEER_CONNECTION_SDP_H264_FMTP_HIGHEST_SCORE ) && ( pAttributes[i].attributeValueLength >= PEER_CONNECTION_SDP_CODEC_H264_VALUE_LENGTH ) &&
                    ( strncmp( PEER_CONNECTION_SDP_CODEC_H264_VALUE, pAttributes[i].pAttributeValue + pAttributes[i].attributeValueLength - PEER_CONNECTION_SDP_CODEC_H264_VALUE_LENGTH, PEER_CONNECTION_SDP_CODEC_H264_VALUE_LENGTH ) == 0 ) )
                {
                    pH264FmtpAttribute = FindH264FmtpAttribute( pAttributes, attributeCount, &pAttributes[i] );
                    h264Score = CalculateH264ScoreByFmtp( pH264FmtpAttribute );
                    if( ( h264Score >= PEER_CONNECTION_SDP_H264_FMTP_MINIMUM_SCORE ) && ( highestH264Score < h264Score ) )
                    {
                        stringResult = StringUtils_ConvertStringToUl( pAttributes[i].pAttributeValue, pAttributes[i].attributeValueLength - PEER_CONNECTION_SDP_CODEC_H264_VALUE_LENGTH, &aptPayload );
                        if( stringResult != STRING_UTILS_RESULT_OK )
                        {
                            LogWarn( ( "StringUtils_ConvertStringToUl fail, result %d, converting %.*s to %lu",
                                       stringResult,
                                       ( int ) pAttributes[i].attributeValueLength - PEER_CONNECTION_SDP_CODEC_H264_VALUE_LENGTH, pAttributes[i].pAttributeValue,
                                       aptPayload ) );
                        }
                        else
                        {
                            TRANSCEIVER_ENABLE_CODEC( codecBitMap, TRANSCEIVER_RTC_CODEC_H264_PROFILE_42E01F_LEVEL_ASYMMETRY_ALLOWED_PACKETIZATION_BIT );
                            codecPayloads[ TRANSCEIVER_RTC_CODEC_H264_PROFILE_42E01F_LEVEL_ASYMMETRY_ALLOWED_PACKETIZATION_BIT ] = PEER_CONNECTION_SDP_SET_PAYLOAD( 0, aptPayload );
                            LogDebug( ( "Found H264 codec: %lu", codecPayloads[ TRANSCEIVER_RTC_CODEC_H264_PROFILE_42E01F_LEVEL_ASYMMETRY_ALLOWED_PACKETIZATION_BIT ] ) );
                            highestH264Score = h264Score;
                        }
                    }
                }
                else if( !TRANSCEIVER_IS_CODEC_ENABLED( codecBitMap, TRANSCEIVER_RTC_CODEC_VP8_BIT ) && ( pAttributes[i].attributeValueLength >= PEER_CONNECTION_SDP_CODEC_VP8_VALUE_LENGTH ) &&
                         ( strncmp( PEER_CONNECTION_SDP_CODEC_VP8_VALUE, pAttributes[i].pAttributeValue + pAttributes[i].attributeValueLength - PEER_CONNECTION_SDP_CODEC_VP8_VALUE_LENGTH, PEER_CONNECTION_SDP_CODEC_VP8_VALUE_LENGTH ) == 0 ) )
                {
                    stringResult = StringUtils_ConvertStringToUl( pAttributes[i].pAttributeValue, pAttributes[i].attributeValueLength - PEER_CONNECTION_SDP_CODEC_VP8_VALUE_LENGTH, &aptPayload );
                    if( stringResult != STRING_UTILS_RESULT_OK )
                    {
                        LogWarn( ( "StringUtils_ConvertStringToUl fail, result %d, converting %.*s to %lu",
                                   stringResult,
                                   ( int ) pAttributes[i].attributeValueLength - PEER_CONNECTION_SDP_CODEC_VP8_VALUE_LENGTH, pAttributes[i].pAttributeValue,
                                   aptPayload ) );
                    }
                    else
                    {
                        TRANSCEIVER_ENABLE_CODEC( codecBitMap, TRANSCEIVER_RTC_CODEC_VP8_BIT );
                        codecPayloads[ TRANSCEIVER_RTC_CODEC_VP8_BIT ] = PEER_CONNECTION_SDP_SET_PAYLOAD( 0, aptPayload );
                        LogDebug( ( "Found VP8 codec: %lu", codecPayloads[ TRANSCEIVER_RTC_CODEC_VP8_BIT ] ) );
                    }
                }
                else if( !TRANSCEIVER_IS_CODEC_ENABLED( codecBitMap, TRANSCEIVER_RTC_CODEC_H265_BIT ) && ( pAttributes[i].attributeValueLength >= PEER_CONNECTION_SDP_CODEC_H265_VALUE_LENGTH ) &&
                         ( strncmp( PEER_CONNECTION_SDP_CODEC_H265_VALUE, pAttributes[i].pAttributeValue + pAttributes[i].attributeValueLength - PEER_CONNECTION_SDP_CODEC_H265_VALUE_LENGTH, PEER_CONNECTION_SDP_CODEC_H265_VALUE_LENGTH ) == 0 ) )
                {
                    stringResult = StringUtils_ConvertStringToUl( pAttributes[i].pAttributeValue, pAttributes[i].attributeValueLength - PEER_CONNECTION_SDP_CODEC_H265_VALUE_LENGTH, &aptPayload );
                    if( stringResult != STRING_UTILS_RESULT_OK )
                    {
                        LogWarn( ( "StringUtils_ConvertStringToUl fail, result %d, converting %.*s to %lu",
                                   stringResult,
                                   ( int ) pAttributes[i].attributeValueLength - PEER_CONNECTION_SDP_CODEC_H265_VALUE_LENGTH, pAttributes[i].pAttributeValue,
                                   aptPayload ) );
                    }
                    else
                    {
                        TRANSCEIVER_ENABLE_CODEC( codecBitMap, TRANSCEIVER_RTC_CODEC_H265_BIT );
                        codecPayloads[ TRANSCEIVER_RTC_CODEC_H265_BIT ] = PEER_CONNECTION_SDP_SET_PAYLOAD( 0, aptPayload );
                        LogDebug( ( "Found H265 codec: %lu", codecPayloads[ TRANSCEIVER_RTC_CODEC_H265_BIT ] ) );
                    }
                }
                else if( !TRANSCEIVER_IS_CODEC_ENABLED( codecBitMap, TRANSCEIVER_RTC_CODEC_OPUS_BIT ) && ( pAttributes[i].attributeValueLength >= PEER_CONNECTION_SDP_CODEC_OPUS_VALUE_LENGTH ) &&
                         ( strncmp( PEER_CONNECTION_SDP_CODEC_OPUS_VALUE, pAttributes[i].pAttributeValue + pAttributes[i].attributeValueLength - PEER_CONNECTION_SDP_CODEC_OPUS_VALUE_LENGTH, PEER_CONNECTION_SDP_CODEC_OPUS_VALUE_LENGTH ) == 0 ) )
                {
                    stringResult = StringUtils_ConvertStringToUl( pAttributes[i].pAttributeValue, pAttributes[i].attributeValueLength - PEER_CONNECTION_SDP_CODEC_OPUS_VALUE_LENGTH, &aptPayload );
                    if( stringResult != STRING_UTILS_RESULT_OK )
                    {
                        LogWarn( ( "StringUtils_ConvertStringToUl fail, result %d, converting %.*s to %lu",
                                   stringResult,
                                   ( int ) pAttributes[i].attributeValueLength - PEER_CONNECTION_SDP_CODEC_OPUS_VALUE_LENGTH, pAttributes[i].pAttributeValue,
                                   aptPayload ) );
                    }
                    else
                    {
                        TRANSCEIVER_ENABLE_CODEC( codecBitMap, TRANSCEIVER_RTC_CODEC_OPUS_BIT );
                        codecPayloads[ TRANSCEIVER_RTC_CODEC_OPUS_BIT ] = PEER_CONNECTION_SDP_SET_PAYLOAD( 0, aptPayload );
                        LogDebug( ( "Found OPUS codec: %lu", codecPayloads[ TRANSCEIVER_RTC_CODEC_OPUS_BIT ] ) );
                    }
                }
                else if( !TRANSCEIVER_IS_CODEC_ENABLED( codecBitMap, TRANSCEIVER_RTC_CODEC_MULAW_BIT ) && ( pAttributes[i].attributeValueLength >= PEER_CONNECTION_SDP_CODEC_MULAW_VALUE_LENGTH ) &&
                         ( strncmp( PEER_CONNECTION_SDP_CODEC_MULAW_VALUE, pAttributes[i].pAttributeValue + pAttributes[i].attributeValueLength - PEER_CONNECTION_SDP_CODEC_MULAW_VALUE_LENGTH, PEER_CONNECTION_SDP_CODEC_MULAW_VALUE_LENGTH ) == 0 ) )
                {
                    stringResult = StringUtils_ConvertStringToUl( pAttributes[i].pAttributeValue, pAttributes[i].attributeValueLength - PEER_CONNECTION_SDP_CODEC_MULAW_VALUE_LENGTH, &aptPayload );
                    if( stringResult != STRING_UTILS_RESULT_OK )
                    {
                        LogWarn( ( "StringUtils_ConvertStringToUl fail, result %d, converting %.*s to %lu",
                                   stringResult,
                                   ( int ) pAttributes[i].attributeValueLength - PEER_CONNECTION_SDP_CODEC_MULAW_VALUE_LENGTH, pAttributes[i].pAttributeValue,
                                   aptPayload ) );
                    }
                    else
                    {
                        TRANSCEIVER_ENABLE_CODEC( codecBitMap, TRANSCEIVER_RTC_CODEC_MULAW_BIT );
                        codecPayloads[ TRANSCEIVER_RTC_CODEC_MULAW_BIT ] = PEER_CONNECTION_SDP_SET_PAYLOAD( 0, aptPayload );
                        LogDebug( ( "Found MULAW codec: %lu", codecPayloads[ TRANSCEIVER_RTC_CODEC_MULAW_BIT ] ) );
                    }
                }
                else if( !TRANSCEIVER_IS_CODEC_ENABLED( codecBitMap, TRANSCEIVER_RTC_CODEC_ALAW_BIT ) && ( pAttributes[i].attributeValueLength >= PEER_CONNECTION_SDP_CODEC_ALAW_VALUE_LENGTH ) &&
                         ( strncmp( PEER_CONNECTION_SDP_CODEC_ALAW_VALUE, pAttributes[i].pAttributeValue + pAttributes[i].attributeValueLength - PEER_CONNECTION_SDP_CODEC_ALAW_VALUE_LENGTH, PEER_CONNECTION_SDP_CODEC_ALAW_VALUE_LENGTH ) == 0 ) )
                {
                    stringResult = StringUtils_ConvertStringToUl( pAttributes[i].pAttributeValue, pAttributes[i].attributeValueLength - PEER_CONNECTION_SDP_CODEC_ALAW_VALUE_LENGTH, &aptPayload );
                    if( stringResult != STRING_UTILS_RESULT_OK )
                    {
                        LogWarn( ( "StringUtils_ConvertStringToUl fail, result %d, converting %.*s to %lu",
                                   stringResult,
                                   ( int ) pAttributes[i].attributeValueLength - PEER_CONNECTION_SDP_CODEC_ALAW_VALUE_LENGTH, pAttributes[i].pAttributeValue,
                                   aptPayload ) );
                    }
                    else
                    {
                        TRANSCEIVER_ENABLE_CODEC( codecBitMap, TRANSCEIVER_RTC_CODEC_ALAW_BIT );
                        codecPayloads[ TRANSCEIVER_RTC_CODEC_ALAW_BIT ] = PEER_CONNECTION_SDP_SET_PAYLOAD( 0, aptPayload );
                        LogDebug( ( "Found ALAW codec: %lu", codecPayloads[ TRANSCEIVER_RTC_CODEC_ALAW_BIT ] ) );
                    }
                }
                else
                {
                    /* Do nothing if it's not known string. */
                }
            }
            else if( ( pAttributes[i].attributeNameLength == PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_FMTP_LENGTH ) &&
                     ( strncmp( PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_FMTP, pAttributes[i].pAttributeName, PEER_CONNECTION_SDP_MEDIA_ATTRIBUTE_NAME_FMTP_LENGTH ) == 0 ) )
            {
                pAtp = StringUtils_StrStr( pAttributes[i].pAttributeValue, pAttributes[i].attributeValueLength, PEER_CONNECTION_SDP_CODEC_APT_VALUE, PEER_CONNECTION_SDP_CODEC_APT_VALUE_LENGTH );

                if( pAtp == NULL )
                {
                    /* It's not RTX FMTP message, ignore it. */
                    continue;
                }

                /* Parse RTX payload */
                stringResult = StringUtils_ConvertStringToUl( pAttributes[i].pAttributeValue,
                                                              pAttributes[i].attributeValueLength, &rtxPayload );
                if( stringResult != STRING_UTILS_RESULT_OK )
                {
                    LogWarn( ( "StringUtils_ConvertStringToUl RTX payload fail, result %d, converting %.*s to %lu",
                               stringResult,
                               ( int ) pAttributes[i].attributeValueLength, pAttributes[i].pAttributeValue,
                               rtxPayload ) );
                    continue;
                }

                /* Parse APT payload */
                stringLength = pAttributes[i].pAttributeValue + pAttributes[i].attributeValueLength - pAtp - PEER_CONNECTION_SDP_CODEC_APT_VALUE_LENGTH;
                stringResult = StringUtils_ConvertStringToUl( pAtp + PEER_CONNECTION_SDP_CODEC_APT_VALUE_LENGTH,
                                                              stringLength, &aptPayload );
                if( stringResult != STRING_UTILS_RESULT_OK )
                {
                    LogWarn( ( "StringUtils_ConvertStringToUl APT payload fail, result %d, converting %.*s to %lu",
                               stringResult,
                               ( int ) pAttributes[i].attributeValueLength, pAttributes[i].pAttributeValue,
                               aptPayload ) );
                    continue;
                }

                /* Try match apt payload and update rtx payload. */
                for( j = TRANSCEIVER_RTC_CODEC_MULAW_BIT; j < TRANSCEIVER_RTC_CODEC_NUM; j++ )
                {
                    if( TRANSCEIVER_IS_CODEC_ENABLED( codecBitMap, j ) &&
                        ( PEER_CONNECTION_SDP_GET_APT_CODEC_FROM_PAYLOAD( codecPayloads[ j ] ) == aptPayload ) )
                    {
                        /* Found APT payload, update RTX payload. */
                        codecPayloads[ j ] = PEER_CONNECTION_SDP_SET_PAYLOAD( rtxPayload, aptPayload );
                    }
                }
            }
            else
            {
                /* Empty else. */
            }
        }
    }

    return codecBitMap;
}

static PeerConnectionResult_t GetPayloadTypesFromMedia( SdpControllerMediaDescription_t * pMediaDescription,
                                                        uint32_t * pCodecBitMap,
                                                        uint32_t codecPayloads[TRANSCEIVER_RTC_CODEC_NUM] )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    const char * pStart, * pEnd;
    int count = 0;
    size_t tokenLength, remainLength;

    if( ( pMediaDescription == NULL ) ||
        ( codecPayloads == NULL ) )
    {
        LogError( ( "Invalid input, pMediaDescription: %p, codecPayloads: %p",
                    pMediaDescription,
                    codecPayloads ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        pStart = pMediaDescription->pMediaName;
        remainLength = pMediaDescription->mediaNameLength;
        do {
            count++;
            pEnd = memchr( pStart, ' ', remainLength );

            if( pEnd )
            {
                tokenLength = pEnd - pStart;
            }
            else
            {
                tokenLength = remainLength;
            }

            /* Here is one media name example: "audio 9 UDP/TLS/RTP/SAVPF 111 63 103 104 9 0 8 106 105 13 110 112 113 126".
             * The beginning 3 tokens are not codec part, just ignore them here. */
            if( count > 3 )
            {
                if( ( tokenLength == PEER_CONNECTION_SDP_CODEC_MULAW_DEFAULT_INDEX_LENGTH ) &&
                    ( strncmp( PEER_CONNECTION_SDP_CODEC_MULAW_DEFAULT_INDEX, pStart, tokenLength ) == 0 ) )
                {
                    TRANSCEIVER_ENABLE_CODEC( *pCodecBitMap, TRANSCEIVER_RTC_CODEC_MULAW_BIT );
                    codecPayloads[ TRANSCEIVER_RTC_CODEC_MULAW_BIT ] = TRANSCEIVER_RTC_CODEC_DEFAULT_PAYLOAD_MULAW;
                    break;
                }
                else if( ( tokenLength == PEER_CONNECTION_SDP_CODEC_ALAW_DEFAULT_INDEX_LENGTH ) &&
                         ( strncmp( PEER_CONNECTION_SDP_CODEC_ALAW_DEFAULT_INDEX, pStart, tokenLength ) == 0 ) )
                {
                    TRANSCEIVER_ENABLE_CODEC( *pCodecBitMap, TRANSCEIVER_RTC_CODEC_ALAW_BIT );
                    codecPayloads[ TRANSCEIVER_RTC_CODEC_MULAW_BIT ] = TRANSCEIVER_RTC_CODEC_DEFAULT_PAYLOAD_ALAW;
                    break;
                }
                else
                {
                    /* Do nothing if it's neither MULAW nor ALAW. */
                }
            }

            if( pEnd )
            {
                pStart = pEnd + 1;
                remainLength = remainLength - tokenLength - 1; /* minus extra 1 for space. */
            }
        } while( pEnd != NULL );

        /* Find proper codec bit map by looking for rtpmap. */
        *pCodecBitMap |= CollectAttributesCodec( pMediaDescription->attributes,
                                                 pMediaDescription->mediaAttributesCount,
                                                 codecPayloads );
        LogDebug( ( "Scanned codec from remote media description, *pCodecBitMap: 0x%lx", *pCodecBitMap ) );
    }

    return ret;
}

static PeerConnectionResult_t SetPayloadType( PeerConnectionSession_t * pSession,
                                              SdpControllerMediaDescription_t * pMediaDescription,
                                              const uint32_t * pCodecBitMap,
                                              const uint32_t codecPayloads[TRANSCEIVER_RTC_CODEC_NUM],
                                              uint8_t isTransceiverCodecSet[ PEER_CONNECTION_TRANSCEIVER_MAX_COUNT ] )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    int currentTransceiverIdx;
    TransceiverTrackKind_t trackKind;
    uint32_t * pTargetCodecPayload = NULL;
    uint32_t * pTargetCodecRtxPayload = NULL;
    uint8_t * pIsTargetCodecPayloadSet = NULL;

    if( ( pSession == NULL ) ||
        ( pMediaDescription == NULL ) ||
        ( pCodecBitMap == NULL ) ||
        ( codecPayloads == NULL ) ||
        ( isTransceiverCodecSet == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pMediaDescription: %p,pCodecBitMap: %p, codecPayloads: %p, isTransceiverCodecSet: %p",
                    pSession,
                    pMediaDescription,
                    pCodecBitMap,
                    codecPayloads,
                    isTransceiverCodecSet ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( ( pMediaDescription->mediaNameLength >= 5 ) &&
            ( strncmp( pMediaDescription->pMediaName, "video", 5 ) == 0 ) )
        {
            trackKind = TRANSCEIVER_TRACK_KIND_VIDEO;
            pTargetCodecPayload = &pSession->rtpConfig.videoCodecPayload;
            pTargetCodecRtxPayload = &pSession->rtpConfig.videoCodecRtxPayload;
            pIsTargetCodecPayloadSet = &pSession->rtpConfig.isVideoCodecPayloadSet;
            LogDebug( ( "Appending video tranceiver" ) );
        }
        else if( ( pMediaDescription->mediaNameLength >= 5 ) &&
                 ( strncmp( pMediaDescription->pMediaName, "audio", 5 ) == 0 ) )
        {
            trackKind = TRANSCEIVER_TRACK_KIND_AUDIO;
            pTargetCodecPayload = &pSession->rtpConfig.audioCodecPayload;
            pTargetCodecRtxPayload = &pSession->rtpConfig.audioCodecRtxPayload;
            pIsTargetCodecPayloadSet = &pSession->rtpConfig.isAudioCodecPayloadSet;
            LogDebug( ( "Appending audio tranceiver" ) );
        }
        #if ENABLE_SCTP_DATA_CHANNEL
        else if( ( pMediaDescription->mediaNameLength >= 11 ) &&
                 ( strncmp( pMediaDescription->pMediaName, "application", 11 ) == 0 ) )
        {
            trackKind = TRANSCEIVER_TRACK_KIND_DATA_CHANNEL;
            pSession->ucEnableDataChannelRemote = 1;
            LogDebug( ( "Appending data channel" ) );
        }
        #endif
        else
        {
            /* Ignore unknown media type. */
            LogWarn( ( "Ignore unknown media type, media name: %.*s",
                       ( int ) pMediaDescription->mediaNameLength,
                       pMediaDescription->pMediaName ) );
            ret = PEER_CONNECTION_RESULT_UNKNOWN_SDP_TRACK_KIND;
        }
    }

    if( ( ret == PEER_CONNECTION_RESULT_OK ) && ( trackKind != TRANSCEIVER_TRACK_KIND_DATA_CHANNEL ) )
    {
        LogDebug( ( "Total transceiverCount: %lu", pSession->transceiverCount ) );
        for( currentTransceiverIdx = 0; currentTransceiverIdx < pSession->transceiverCount; currentTransceiverIdx++ )
        {
            LogDebug( ( "currentTransceiverIdx: %d", currentTransceiverIdx ) );
            if( ( isTransceiverCodecSet[currentTransceiverIdx] != 0 ) ||
                ( pSession->pTransceivers[currentTransceiverIdx]->trackKind != trackKind ) ||
                ( ( *pCodecBitMap & pSession->pTransceivers[currentTransceiverIdx]->codecBitMap ) == 0 ) )
            {
                LogDebug( ( "Skip transceiver index: %d, isTransceiverCodecSet[%d]: %d, pTransceivers[%d]->trackKind: %d, pTransceivers[%d]->codecBitMap: %lx",
                            currentTransceiverIdx,
                            currentTransceiverIdx, isTransceiverCodecSet[currentTransceiverIdx],
                            currentTransceiverIdx, pSession->pTransceivers[currentTransceiverIdx]->trackKind,
                            currentTransceiverIdx, pSession->pTransceivers[currentTransceiverIdx]->codecBitMap ) );
                continue;
            }

            if( TRANSCEIVER_IS_CODEC_ENABLED( *pCodecBitMap & pSession->pTransceivers[currentTransceiverIdx]->codecBitMap, TRANSCEIVER_RTC_CODEC_H264_PROFILE_42E01F_LEVEL_ASYMMETRY_ALLOWED_PACKETIZATION_BIT ) )
            {
                isTransceiverCodecSet[currentTransceiverIdx] = 1;
                *pIsTargetCodecPayloadSet = 1;
                *pTargetCodecPayload = PEER_CONNECTION_SDP_GET_APT_CODEC_FROM_PAYLOAD( codecPayloads[ TRANSCEIVER_RTC_CODEC_H264_PROFILE_42E01F_LEVEL_ASYMMETRY_ALLOWED_PACKETIZATION_BIT ] );
                *pTargetCodecRtxPayload = PEER_CONNECTION_SDP_GET_RTX_CODEC_FROM_PAYLOAD( codecPayloads[ TRANSCEIVER_RTC_CODEC_H264_PROFILE_42E01F_LEVEL_ASYMMETRY_ALLOWED_PACKETIZATION_BIT ] );
                if( pSession->mLinesTransceiverCount < PEER_CONNECTION_TRANSCEIVER_MAX_COUNT )
                {
                    pSession->pMLinesTransceivers[ pSession->mLinesTransceiverCount++ ] = pSession->pTransceivers[ currentTransceiverIdx ];
                }
                else
                {
                    LogWarn( ( "Cannot not store more transceiver pointers." ) );
                }
                break;
            }
            else if( TRANSCEIVER_IS_CODEC_ENABLED( *pCodecBitMap & pSession->pTransceivers[currentTransceiverIdx]->codecBitMap, TRANSCEIVER_RTC_CODEC_VP8_BIT ) )
            {
                isTransceiverCodecSet[currentTransceiverIdx] = 1;
                *pIsTargetCodecPayloadSet = 1;
                *pTargetCodecPayload = PEER_CONNECTION_SDP_GET_APT_CODEC_FROM_PAYLOAD( codecPayloads[ TRANSCEIVER_RTC_CODEC_VP8_BIT ] );
                *pTargetCodecRtxPayload = PEER_CONNECTION_SDP_GET_RTX_CODEC_FROM_PAYLOAD( codecPayloads[ TRANSCEIVER_RTC_CODEC_VP8_BIT ] );
                if( pSession->mLinesTransceiverCount < PEER_CONNECTION_TRANSCEIVER_MAX_COUNT )
                {
                    pSession->pMLinesTransceivers[ pSession->mLinesTransceiverCount++ ] = pSession->pTransceivers[ currentTransceiverIdx ];
                }
                else
                {
                    LogWarn( ( "Cannot not store more transceiver pointers." ) );
                }
                break;
            }
            else if( TRANSCEIVER_IS_CODEC_ENABLED( *pCodecBitMap & pSession->pTransceivers[currentTransceiverIdx]->codecBitMap, TRANSCEIVER_RTC_CODEC_H265_BIT ) )
            {
                isTransceiverCodecSet[currentTransceiverIdx] = 1;
                *pIsTargetCodecPayloadSet = 1;
                *pTargetCodecPayload = PEER_CONNECTION_SDP_GET_APT_CODEC_FROM_PAYLOAD( codecPayloads[ TRANSCEIVER_RTC_CODEC_H265_BIT ] );
                *pTargetCodecRtxPayload = PEER_CONNECTION_SDP_GET_RTX_CODEC_FROM_PAYLOAD( codecPayloads[ TRANSCEIVER_RTC_CODEC_H265_BIT ] );
                if( pSession->mLinesTransceiverCount < PEER_CONNECTION_TRANSCEIVER_MAX_COUNT )
                {
                    pSession->pMLinesTransceivers[ pSession->mLinesTransceiverCount++ ] = pSession->pTransceivers[ currentTransceiverIdx ];
                }
                else
                {
                    LogWarn( ( "Cannot not store more transceiver pointers." ) );
                }
                break;
            }
            else if( TRANSCEIVER_IS_CODEC_ENABLED( *pCodecBitMap & pSession->pTransceivers[currentTransceiverIdx]->codecBitMap, TRANSCEIVER_RTC_CODEC_OPUS_BIT ) )
            {
                isTransceiverCodecSet[currentTransceiverIdx] = 1;
                *pIsTargetCodecPayloadSet = 1;
                *pTargetCodecPayload = PEER_CONNECTION_SDP_GET_APT_CODEC_FROM_PAYLOAD( codecPayloads[ TRANSCEIVER_RTC_CODEC_OPUS_BIT ] );
                *pTargetCodecRtxPayload = PEER_CONNECTION_SDP_GET_RTX_CODEC_FROM_PAYLOAD( codecPayloads[ TRANSCEIVER_RTC_CODEC_OPUS_BIT ] );
                if( pSession->mLinesTransceiverCount < PEER_CONNECTION_TRANSCEIVER_MAX_COUNT )
                {
                    pSession->pMLinesTransceivers[ pSession->mLinesTransceiverCount++ ] = pSession->pTransceivers[ currentTransceiverIdx ];
                }
                else
                {
                    LogWarn( ( "Cannot not store more transceiver pointers." ) );
                }
                break;
            }
            else if( TRANSCEIVER_IS_CODEC_ENABLED( *pCodecBitMap & pSession->pTransceivers[currentTransceiverIdx]->codecBitMap, TRANSCEIVER_RTC_CODEC_MULAW_BIT ) )
            {
                isTransceiverCodecSet[currentTransceiverIdx] = 1;
                *pIsTargetCodecPayloadSet = 1;
                *pTargetCodecPayload = PEER_CONNECTION_SDP_GET_APT_CODEC_FROM_PAYLOAD( codecPayloads[ TRANSCEIVER_RTC_CODEC_MULAW_BIT ] );
                *pTargetCodecRtxPayload = PEER_CONNECTION_SDP_GET_RTX_CODEC_FROM_PAYLOAD( codecPayloads[ TRANSCEIVER_RTC_CODEC_MULAW_BIT ] );
                if( pSession->mLinesTransceiverCount < PEER_CONNECTION_TRANSCEIVER_MAX_COUNT )
                {
                    pSession->pMLinesTransceivers[ pSession->mLinesTransceiverCount++ ] = pSession->pTransceivers[ currentTransceiverIdx ];
                }
                else
                {
                    LogWarn( ( "Cannot not store more transceiver pointers." ) );
                }
                break;
            }
            else if( TRANSCEIVER_IS_CODEC_ENABLED( *pCodecBitMap & pSession->pTransceivers[currentTransceiverIdx]->codecBitMap, TRANSCEIVER_RTC_CODEC_ALAW_BIT ) )
            {
                isTransceiverCodecSet[currentTransceiverIdx] = 1;
                *pIsTargetCodecPayloadSet = 1;
                *pTargetCodecPayload = PEER_CONNECTION_SDP_GET_APT_CODEC_FROM_PAYLOAD( codecPayloads[ TRANSCEIVER_RTC_CODEC_ALAW_BIT ] );
                *pTargetCodecRtxPayload = PEER_CONNECTION_SDP_GET_RTX_CODEC_FROM_PAYLOAD( codecPayloads[ TRANSCEIVER_RTC_CODEC_ALAW_BIT ] );
                if( pSession->mLinesTransceiverCount < PEER_CONNECTION_TRANSCEIVER_MAX_COUNT )
                {
                    pSession->pMLinesTransceivers[ pSession->mLinesTransceiverCount++ ] = pSession->pTransceivers[ currentTransceiverIdx ];
                }
                else
                {
                    LogWarn( ( "Cannot not store more transceiver pointers." ) );
                }
                break;
            }
            else
            {
                /* Unexpected to enter this condition. */
            }
        }

        if( *pIsTargetCodecPayloadSet == 1 )
        {
            LogDebug( ( "Set payload type successfully, idx: %d, payload: 0x%lx, RTX payload: 0x%lx", currentTransceiverIdx, *pTargetCodecPayload, *pTargetCodecRtxPayload ) );
        }
        else
        {
            LogWarn( ( "Unable to set payload type, mediaCodecBitMap: 0x%lx", *pCodecBitMap ) );
        }
    }

    return ret;
}

static PeerConnectionResult_t PopulateMediaDescriptions( PeerConnectionSession_t * pSession,
                                                         PeerConnectionBufferSessionDescription_t * pRemoteBufferSessionDescription,
                                                         PeerConnectionBufferSessionDescription_t * pLocalBufferSessionDescription,
                                                         char ** ppBuffer,
                                                         size_t * pBufferLength )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    int i;
    SdpControllerResult_t retSdpController;
    SdpControllerPopulateMediaConfiguration_t populateConfiguration;

    memset( &populateConfiguration, 0, sizeof( SdpControllerPopulateMediaConfiguration_t ) );
    populateConfiguration.canTrickleIce = 1U;

    populateConfiguration.pCname = pSession->pCtx->localCname;
    populateConfiguration.cnameLength = strlen( pSession->pCtx->localCname );
    populateConfiguration.pUserName = pSession->pCtx->localUserName;
    populateConfiguration.userNameLength = strlen( pSession->pCtx->localUserName );
    populateConfiguration.pPassword = pSession->pCtx->localPassword;
    populateConfiguration.passwordLength = strlen( pSession->pCtx->localPassword );

    populateConfiguration.pLocalFingerprint = pSession->pCtx->dtlsContext.localCertFingerprint;
    populateConfiguration.localFingerprintLength = CERTIFICATE_FINGERPRINT_LENGTH;

    if( pRemoteBufferSessionDescription == NULL )
    {
        /* Populating SDP offer. */
        populateConfiguration.isOffer = 1U;
        populateConfiguration.twccExtId = 0U;

        for( i = 0; i < pSession->transceiverCount; i++ )
        {
            populateConfiguration.pTransceiver = pSession->pTransceivers[i];
            if( populateConfiguration.pTransceiver->trackKind == TRANSCEIVER_TRACK_KIND_VIDEO )
            {
                populateConfiguration.payloadType = pSession->rtpConfig.videoCodecPayload;
                populateConfiguration.rtxPayloadType = pSession->rtpConfig.videoCodecRtxPayload;
            }
            else
            {
                populateConfiguration.payloadType = pSession->rtpConfig.audioCodecPayload;
                populateConfiguration.rtxPayloadType = pSession->rtpConfig.audioCodecRtxPayload;
            }

            retSdpController = SdpController_PopulateSingleMedia( NULL,
                                                                  populateConfiguration,
                                                                  &pLocalBufferSessionDescription->sdpDescription.mediaDescriptions[ i ],
                                                                  i,
                                                                  ppBuffer,
                                                                  pBufferLength,
                                                                  populateConfiguration.pTransceiver->trackKind );
            if( retSdpController != SDP_CONTROLLER_RESULT_OK )
            {
                LogError( ( "Fail to populate single media description, result: %d", retSdpController ) );
                ret = PEER_CONNECTION_RESULT_FAIL_SDP_POPULATE_SINGLE_MEDIA_DESCRIPTION;
                break;
            }
            else
            {
                pLocalBufferSessionDescription->sdpDescription.mediaCount++;
            }
        }
        #if ENABLE_SCTP_DATA_CHANNEL
        {
            if( ( ret == PEER_CONNECTION_RESULT_OK ) && ( pSession->ucEnableDataChannelLocal != 0 ) )
            {
                if( i < SDP_CONTROLLER_MAX_SDP_MEDIA_DESCRIPTIONS_COUNT )
                {
                    populateConfiguration.pTransceiver = NULL;
                    retSdpController = SdpController_PopulateSingleMedia( NULL,
                                                                          populateConfiguration,
                                                                          &pLocalBufferSessionDescription->sdpDescription.mediaDescriptions[ i ],
                                                                          i,
                                                                          ppBuffer,
                                                                          pBufferLength,
                                                                          TRANSCEIVER_TRACK_KIND_DATA_CHANNEL );
                    if( retSdpController != SDP_CONTROLLER_RESULT_OK )
                    {
                        LogError( ( "Fail to populate single media data channel description, result: %d", retSdpController ) );
                        ret = PEER_CONNECTION_RESULT_FAIL_SDP_POPULATE_SINGLE_MEDIA_DESCRIPTION;
                    }
                    else
                    {
                        pLocalBufferSessionDescription->sdpDescription.mediaCount++;
                    }
                }
                else
                {
                    LogError( ( "Reached media description limit, failed to populate data channel description, index: %d", i ) );
                }
            }
        }
        #endif /* ENABLE_SCTP_DATA_CHANNEL */
    }
    else
    {
        /* Populating SDP answer. */
        populateConfiguration.isOffer = 0U;
        populateConfiguration.twccExtId = pSession->remoteSessionDescription.sdpDescription.quickAccess.twccExtId;

        for( i = 0; i < pSession->mLinesTransceiverCount; i++ )
        {
            populateConfiguration.pTransceiver = pSession->pMLinesTransceivers[i];
            if( populateConfiguration.pTransceiver->trackKind == TRANSCEIVER_TRACK_KIND_VIDEO )
            {
                populateConfiguration.payloadType = pSession->rtpConfig.videoCodecPayload;
                populateConfiguration.rtxPayloadType = pSession->rtpConfig.videoCodecRtxPayload;
            }
            else
            {
                populateConfiguration.payloadType = pSession->rtpConfig.audioCodecPayload;
                populateConfiguration.rtxPayloadType = pSession->rtpConfig.audioCodecRtxPayload;
            }

            retSdpController = SdpController_PopulateSingleMedia( &pRemoteBufferSessionDescription->sdpDescription.mediaDescriptions[ i ],
                                                                  populateConfiguration,
                                                                  &pLocalBufferSessionDescription->sdpDescription.mediaDescriptions[ i ],
                                                                  i,
                                                                  ppBuffer,
                                                                  pBufferLength,
                                                                  populateConfiguration.pTransceiver->trackKind );
            if( retSdpController != SDP_CONTROLLER_RESULT_OK )
            {
                LogError( ( "Fail to populate single media description, result: %d", retSdpController ) );
                ret = PEER_CONNECTION_RESULT_FAIL_SDP_POPULATE_SINGLE_MEDIA_DESCRIPTION;
                break;
            }
            else
            {
                pLocalBufferSessionDescription->sdpDescription.mediaCount++;
            }
        }

        #if ENABLE_SCTP_DATA_CHANNEL
        {

            if( ( ret == PEER_CONNECTION_RESULT_OK ) && ( pSession->ucEnableDataChannelRemote == 1 ) && ( i < SDP_CONTROLLER_MAX_SDP_MEDIA_DESCRIPTIONS_COUNT ) )
            {
                populateConfiguration.pTransceiver = NULL;
                retSdpController = SdpController_PopulateSingleMedia( &pRemoteBufferSessionDescription->sdpDescription.mediaDescriptions[ i ],
                                                                      populateConfiguration,
                                                                      &pLocalBufferSessionDescription->sdpDescription.mediaDescriptions[ i ],
                                                                      i,
                                                                      ppBuffer,
                                                                      pBufferLength,
                                                                      TRANSCEIVER_TRACK_KIND_DATA_CHANNEL );
                if( retSdpController != SDP_CONTROLLER_RESULT_OK )
                {
                    LogError( ( "Fail to populate single media data channel description, result: %d", retSdpController ) );
                    ret = PEER_CONNECTION_RESULT_FAIL_SDP_POPULATE_SINGLE_MEDIA_DESCRIPTION;
                }
                else
                {
                    pLocalBufferSessionDescription->sdpDescription.mediaCount++;
                }
            }
        }
        #endif /* ENABLE_SCTP_DATA_CHANNEL */
    }

    return ret;
}

static PeerConnectionResult_t PopulateSessionDescription( PeerConnectionSession_t * pSession,
                                                          PeerConnectionBufferSessionDescription_t * pRemoteBufferSessionDescription,
                                                          PeerConnectionBufferSessionDescription_t * pLocalBufferSessionDescription,
                                                          char ** ppBuffer,
                                                          size_t * pBufferLength )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    SdpControllerResult_t retSdpController;
    SdpControllerPopulateSessionConfiguration_t populateConfiguration;

    memset( &populateConfiguration, 0, sizeof( SdpControllerPopulateSessionConfiguration_t ) );
    populateConfiguration.canTrickleIce = 1U;
    if( pRemoteBufferSessionDescription == NULL )
    {
        populateConfiguration.isOffer = 1U;
        retSdpController = SdpController_PopulateSessionDescription( NULL,
                                                                     populateConfiguration,
                                                                     &pLocalBufferSessionDescription->sdpDescription,
                                                                     ppBuffer,
                                                                     pBufferLength );
    }
    else
    {
        retSdpController = SdpController_PopulateSessionDescription( &pRemoteBufferSessionDescription->sdpDescription,
                                                                     populateConfiguration,
                                                                     &pLocalBufferSessionDescription->sdpDescription,
                                                                     ppBuffer,
                                                                     pBufferLength );
    }
    if( retSdpController != SDP_CONTROLLER_RESULT_OK )
    {
        LogWarn( ( "Fail to populate session description, result: %d", retSdpController ) );
        ret = PEER_CONNECTION_RESULT_FAIL_SDP_POPULATE_SESSION_DESCRIPTION;
    }

    return ret;
}

static void SetReceiverSsrc( PeerConnectionBufferSessionDescription_t * pBufferSessionDescription )
{
    int i, j;
    uint8_t isVideoDescription = 0U;
    uint8_t isAudioDescription = 0U;
    SdpControllerMediaDescription_t * pMediaDescription;
    StringUtilsResult_t stringResult;
    uint32_t * pMediaSsrc;

    for( i = 0; i < pBufferSessionDescription->sdpDescription.mediaCount; i++ )
    {
        if( pBufferSessionDescription->sdpDescription.mediaDescriptions[i].mediaNameLength < 5 )
        {
            if( pBufferSessionDescription->sdpDescription.mediaDescriptions[i].mediaNameLength > 0 )
            {
                LogWarn( ( "The media name is not known source, media name: %.*s",
                           pBufferSessionDescription->sdpDescription.mediaDescriptions[i].mediaNameLength,
                           pBufferSessionDescription->sdpDescription.mediaDescriptions[i].pMediaName ) );
            }
            else
            {
                LogWarn( ( "No media name in this media description" ) );
            }
            continue;
        }
        isVideoDescription = strncmp( pBufferSessionDescription->sdpDescription.mediaDescriptions[i].pMediaName, "video", 5 ) == 0 ? 1U : 0U;
        isAudioDescription = strncmp( pBufferSessionDescription->sdpDescription.mediaDescriptions[i].pMediaName, "audio", 5 ) == 0 ? 1U : 0U;
        if( ( isVideoDescription == 0U ) && ( isAudioDescription == 0U ) )
        {
            LogWarn( ( "Non video/audio media description." ) );
            continue;
        }
        else if( isVideoDescription != 0U )
        {
            pMediaSsrc = &pBufferSessionDescription->sdpDescription.quickAccess.videoSsrc;
        }
        else
        {
            pMediaSsrc = &pBufferSessionDescription->sdpDescription.quickAccess.audioSsrc;
        }

        pMediaDescription = &pBufferSessionDescription->sdpDescription.mediaDescriptions[i];
        for( j = 0; j < pMediaDescription->mediaAttributesCount; j++ )
        {
            if( ( pMediaDescription->attributes[j].attributeNameLength == strlen( "ssrc" ) ) &&
                ( strncmp( pMediaDescription->attributes[j].pAttributeName, "ssrc", strlen( "ssrc" ) ) == 0 ) )
            {
                LogInfo( ( "Found SSRC attribute: %.*s",
                           ( int ) pMediaDescription->attributes[j].attributeValueLength,
                           pMediaDescription->attributes[j].pAttributeValue ) );
                stringResult = StringUtils_ConvertStringToUl( pMediaDescription->attributes[j].pAttributeValue,
                                                              pMediaDescription->attributes[j].attributeValueLength,
                                                              pMediaSsrc );
                if( stringResult != STRING_UTILS_RESULT_OK )
                {
                    LogError( ( "StringUtils_ConvertStringToUl fail, result %d, converting %.*s to %lu",
                                stringResult,
                                ( int ) pMediaDescription->attributes[j].attributeValueLength, pMediaDescription->attributes[j].pAttributeValue,
                                *pMediaSsrc ) );
                    continue;
                }

                /* Use first SSRC as media source SSRC. */
                break;
            }
        }
    }
}

PeerConnectionResult_t PeerConnectionSdp_DeserializeSdpMessage( PeerConnectionBufferSessionDescription_t * pBufferSessionDescription )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    SdpControllerResult_t retSdpController;

    if( pBufferSessionDescription == NULL )
    {
        LogError( ( "Invalid input, pBufferSessionDescription: %p", pBufferSessionDescription ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( ( pBufferSessionDescription->type == SDP_CONTROLLER_MESSAGE_TYPE_OFFER ) ||
            ( pBufferSessionDescription->type == SDP_CONTROLLER_MESSAGE_TYPE_ANSWER ) )
        {
            retSdpController = SdpController_DeserializeSdpMessage( pBufferSessionDescription->pSdpBuffer,
                                                                    pBufferSessionDescription->sdpBufferLength,
                                                                    &pBufferSessionDescription->sdpDescription );
            if( retSdpController != SDP_CONTROLLER_RESULT_OK )
            {
                LogError( ( "Unable to deserialize SDP offer, result: %d", retSdpController ) );
                ret = PEER_CONNECTION_RESULT_FAIL_SDP_DESERIALIZE_OFFER;
            }
        }
        else
        {
            LogError( ( "Unknown SDP type: %d", pBufferSessionDescription->type ) );
            ret = PEER_CONNECTION_RESULT_UNKNOWN_SDP_TYPE;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        SetReceiverSsrc( pBufferSessionDescription );
    }

    return ret;
}

PeerConnectionResult_t PeerConnectionSdp_SetPayloadTypes( PeerConnectionSession_t * pSession,
                                                          PeerConnectionBufferSessionDescription_t * pRemoteBufferSessionDescription )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    int i;
    uint8_t isTransceiverCodecSet[ PEER_CONNECTION_TRANSCEIVER_MAX_COUNT ] = { 0 };
    uint32_t remoteMediaCodecBitMap = 0;
    uint32_t remoteCodecPayloads[ TRANSCEIVER_RTC_CODEC_NUM ] = { 0 };

    if( ( pSession == NULL ) ||
        ( pRemoteBufferSessionDescription == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pRemoteBufferSessionDescription: %p",
                    pSession, pRemoteBufferSessionDescription ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else if( pRemoteBufferSessionDescription->pSdpBuffer == NULL )
    {
        LogError( ( "Invalid input, pRemoteBufferSessionDescription->pSdpBuffer: %p",
                    pRemoteBufferSessionDescription->pSdpBuffer ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else
    {
        /* Empty else marker. */
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* If it's SDP offer from remote peer, we should set the codec based on remote codec set. */
        for( i = 0; i < pRemoteBufferSessionDescription->sdpDescription.mediaCount; i++ )
        {
            ret = GetPayloadTypesFromMedia( &pRemoteBufferSessionDescription->sdpDescription.mediaDescriptions[i],
                                            &remoteMediaCodecBitMap,
                                            remoteCodecPayloads );
            if( ret != PEER_CONNECTION_RESULT_OK )
            {
                LogWarn( ( "Fail to get payload types from media idx: %d", i ) );
                ret = PEER_CONNECTION_RESULT_FAIL_SDP_GET_PAYLOAD_TYPES;
                continue;
            }

            ret = SetPayloadType( pSession,
                                  &pRemoteBufferSessionDescription->sdpDescription.mediaDescriptions[i],
                                  &remoteMediaCodecBitMap,
                                  remoteCodecPayloads,
                                  isTransceiverCodecSet );
            if( ret != PEER_CONNECTION_RESULT_OK )
            {
                LogWarn( ( "Fail to set payload type, ret: %d", ret ) );
                ret = PEER_CONNECTION_RESULT_FAIL_SDP_SET_PAYLOAD_TYPE;
                continue;
            }
        }
    }

    return ret;
}

PeerConnectionResult_t PeerConnectionSdp_PopulateSessionDescription( PeerConnectionSession_t * pSession,
                                                                     PeerConnectionBufferSessionDescription_t * pRemoteBufferSessionDescription,
                                                                     PeerConnectionBufferSessionDescription_t * pLocalBufferSessionDescription,
                                                                     char * pOutputSerializedSdpMessage,
                                                                     size_t * pOutputSerializedSdpMessageLength )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    char * pBuffer = NULL;
    size_t bufferLength = 0;
    SdpControllerResult_t retSdpController;

    if( ( pSession == NULL ) ||
        ( pLocalBufferSessionDescription == NULL ) ||
        ( pOutputSerializedSdpMessage == NULL ) ||
        ( pOutputSerializedSdpMessageLength == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pLocalBufferSessionDescription: %p, pOutputSerializedSdpMessage: %p, pOutputSerializedSdpMessageLength: %p",
                    pSession, pLocalBufferSessionDescription, pOutputSerializedSdpMessage, pOutputSerializedSdpMessageLength ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else if( pLocalBufferSessionDescription->pSdpBuffer == NULL )
    {
        LogError( ( "Invalid input, pLocalBufferSessionDescription->pSdpBuffer: %p",
                    pLocalBufferSessionDescription->pSdpBuffer ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else if( ( pRemoteBufferSessionDescription != NULL ) && ( pRemoteBufferSessionDescription->pSdpBuffer == NULL ) )
    {
        LogError( ( "Invalid input, pRemoteBufferSessionDescription->pSdpBuffer: %p",
                    pRemoteBufferSessionDescription->pSdpBuffer ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else
    {
        /* Empty else marker. */
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Add media descriptions, use the temp buffer to store SDP content for pointers to refer to. */
        pBuffer = pLocalBufferSessionDescription->pSdpBuffer;
        bufferLength = pLocalBufferSessionDescription->sdpBufferLength;
        ret = PopulateMediaDescriptions( pSession, pRemoteBufferSessionDescription, pLocalBufferSessionDescription, &pBuffer, &bufferLength );
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Add session descriptions.
         * Note that we need to session media count to populate session group attribute,
         * so this have to do after populate media sessions. */
        ret = PopulateSessionDescription( pSession, pRemoteBufferSessionDescription, pLocalBufferSessionDescription, &pBuffer, &bufferLength );
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Serialize the content into the buffer in pLocalBufferSessionDescription. */
        pBuffer = pOutputSerializedSdpMessage;
        bufferLength = *pOutputSerializedSdpMessageLength;
        retSdpController = SdpController_SerializeSdpMessageByDescription( pLocalBufferSessionDescription->type,
                                                                           &pLocalBufferSessionDescription->sdpDescription,
                                                                           pBuffer,
                                                                           &bufferLength );
        if( retSdpController != SDP_CONTROLLER_RESULT_OK )
        {
            LogWarn( ( "Fail to serialize session description, result: %d", retSdpController ) );
            ret = PEER_CONNECTION_RESULT_FAIL_SDP_POPULATE_SESSION_DESCRIPTION;
        }
        else
        {
            *pOutputSerializedSdpMessageLength = bufferLength;
        }
    }

    return ret;
}
