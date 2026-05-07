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

#include <stdlib.h>
#include "logging.h"
#include "sdp_controller.h"
#include "core_json.h"
#include "sdp_deserializer.h"
#include "sdp_serializer.h"
#include "string_utils.h"
#include "peer_connection.h"

#define SDP_CONTROLLER_ORIGIN_DEFAULT_USER_NAME "-"
#define SDP_CONTROLLER_ORIGIN_DEFAULT_SESSION_VERSION ( 2 )
#define SDP_CONTROLLER_ORIGIN_DEFAULT_NET_TYPE "IN"
#define SDP_CONTROLLER_ORIGIN_IPV4_TYPE "IP4"
#define SDP_CONTROLLER_ORIGIN_DEFAULT_IP_ADDRESS "127.0.0.1"

#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SETUP "setup"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SETUP_LENGTH ( 5 )
#define SDP_CONTROLLER_MEDIA_DTLS_ROLE_ACTIVE "active"
#define SDP_CONTROLLER_MEDIA_DTLS_ROLE_ACTIVE_LENGTH ( 6 )
#define SDP_CONTROLLER_MEDIA_DTLS_ROLE_ACTPASS "actpass"
#define SDP_CONTROLLER_MEDIA_DTLS_ROLE_ACTPASS_LENGTH ( 7 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_MSID "msid"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_MSID_LENGTH ( 4 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP "rtcp"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_LENGTH ( 4 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTCP "9 IN IP4 0.0.0.0"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTCP_LENGTH ( 16 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_UFRAG "ice-ufrag"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_UFRAG_LENGTH ( 9 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_PWD "ice-pwd"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_PWD_LENGTH ( 7 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_OPTION "ice-options"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_OPTION_LENGTH ( 11 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_ICE_OPTION "trickle"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_ICE_OPTION_LENGTH ( 7 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_FINGERPRINT "fingerprint"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_FINGERPRINT_LENGTH ( 11 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC "ssrc"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC_LENGTH ( 4 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC_GROUP "ssrc-group"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC_GROUP_LENGTH ( 10 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_MID "mid"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_MID_LENGTH ( 3 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SENDRECV "sendrecv"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SENDRECV_LENGTH ( 8 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SENDONLY "sendonly"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SENDONLY_LENGTH ( 8 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RECVONLY "recvonly"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RECVONLY_LENGTH ( 8 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_INACTIVE "inactive"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_INACTIVE_LENGTH ( 8 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_MUX "rtcp-mux"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_MUX_LENGTH ( 8 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_RSIZE "rtcp-rsize"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_RSIZE_LENGTH ( 10 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTPMAP "rtpmap"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTPMAP_LENGTH ( 6 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_H264 "H264/90000"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_H264_LENGTH ( 10 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_RTX_H264 "rtx/90000"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_RTX_H264_LENGTH ( 9 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_OPUS "opus/48000/2"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_OPUS_LENGTH ( 12 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_VP8 "VP8/90000"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_VP8_LENGTH ( 9 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_MULAW "PCMU/8000"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_MULAW_LENGTH ( 9 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_ALAW "PCMA/8000"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_ALAW_LENGTH ( 9 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_H265 "H265/90000"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_H265_LENGTH ( 10 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_FB "rtcp-fb"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_FB_LENGTH ( 7 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_VALUE "nack"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_VALUE_LENGTH ( 4 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_GOOG_REMB "goog-remb"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_GOOG_REMB_LENGTH ( 9 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_FMTP "fmtp"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_FMTP_LENGTH ( 4 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_EXTMAP "extmap"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_EXTMAP_LENGTH ( 6 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_TWCC_EXT_URL "http://www.ietf.org/id/draft-holmer-rmcat-transport-wide-cc-extensions-01"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_TWCC_EXT_URL_LENGTH ( 73 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_TRANSPORT_CC "transport-cc"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_TRANSPORT_CC_LENGTH ( 12 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_CANDIDATE "candidate"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_CANDIDATE_LENGTH ( 9 )
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_FINGERPRINT_PREFIX_LENGTH ( 8 ) // the length of "sha-256 "

#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_FMTP_H264_PROFILE_42E01F_LEVEL_ASYMMETRY_ALLOWED_PACKETIZATION "level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=42e01f"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_FMTP_OPUS "minptime=10;useinbandfec=1"
#define SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_FMTP_H265 "profile-space=0;profile-id=0;tier-flag=0;level-id=0;interop-constraints=000000000000;sprop-vps=QAEMAf//" \
    "AIAAAAMAAAMAAAMAAAMAALUCQA==;sprop-sps=QgEBAIAAAAMAAAMAAAMAAAMAAKACgIAtH+W1kkbQzkkktySqSfKSyA==;sprop-pps=RAHBpVgeSA=="

#define SDP_CONTROLLER_H264_PACKETIZATION_MODE "packetization-mode=1"
#define SDP_CONTROLLER_H264_PACKETIZATION_MODE_LENGTH ( 20 )
#define SDP_CONTROLLER_H264_ASYMMETRY_ALLOWED "level-asymmetry-allowed=1"
#define SDP_CONTROLLER_H264_ASYMMETRY_ALLOWED_LENGTH ( 25 )
#define SDP_CONTROLLER_H264_PROFILE_LEVEL_ID "profile-level-id="
#define SDP_CONTROLLER_H264_PROFILE_LEVEL_ID_LENGTH ( 17 )

#define SDP_CONTROLLER_MAX_FMTP_APT_NUM ( 64 )

#define SDP_CONTROLLER_GET_APT_CODEC_FROM_PAYLOAD( payload ) ( payload & 0xFF )
#define SDP_CONTROLLER_GET_RTX_CODEC_FROM_PAYLOAD( payload ) ( payload >> 16 )
#define SDP_CONTROLLER_SET_PAYLOAD( rtxPayload, aptPayload ) ( rtxPayload << 16 | aptPayload )

#define SDP_CONTROLLER_DATA_CHANNEL_ATTRIBUTE_NAME_MEDIA_NAME "application 9 UDP/DTLS/SCTP webrtc-datachannel"
#define SDP_CONTROLLER_DATA_CHANNEL_ATTRIBUTE_NAME_SCTP_PORT "sctp-port"
#define SDP_CONTROLLER_DATA_CHANNEL_ATTRIBUTE_NAME_SCTP_PORT_LENGTH ( 9 )
#define SDP_CONTROLLER_DATA_CHANNEL_ATTRIBUTE_VALUE_SCTP_PORT "5000"
#define SDP_CONTROLLER_DATA_CHANNEL_ATTRIBUTE_VALUE_SCTP_PORT_LENGTH ( 4 )

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
#define SDP_CONTROLLER_H264_PROFILE_42E01F 0x42e01f
#define SDP_CONTROLLER_H264_FMTP_SUBPROFILE_MASK 0xFFFF00
#define SDP_CONTROLLER_H264_FMTP_PROFILE_LEVEL_MASK 0x0000FF
#define SDP_CONTROLLER_H264_FMTP_MINIMUM_SCORE ( 10 )
#define SDP_CONTROLLER_H264_FMTP_HIGHEST_SCORE ( 12 )

#define SDP_CONTROLLER_CODEC_H264_VALUE "H264/90000"
#define SDP_CONTROLLER_CODEC_H264_VALUE_LENGTH ( 10 )
#define SDP_CONTROLLER_CODEC_H265_VALUE "H265/90000"
#define SDP_CONTROLLER_CODEC_H265_VALUE_LENGTH ( 10 )
#define SDP_CONTROLLER_CODEC_OPUS_VALUE "opus/48000/2"
#define SDP_CONTROLLER_CODEC_OPUS_VALUE_LENGTH ( 12 )
#define SDP_CONTROLLER_CODEC_VP8_VALUE "VP8/90000"
#define SDP_CONTROLLER_CODEC_VP8_VALUE_LENGTH ( 9 )
#define SDP_CONTROLLER_CODEC_MULAW_VALUE "PCMU/8000"
#define SDP_CONTROLLER_CODEC_MULAW_VALUE_LENGTH ( 9 )
#define SDP_CONTROLLER_CODEC_ALAW_VALUE "PCMA/8000"
#define SDP_CONTROLLER_CODEC_ALAW_VALUE_LENGTH ( 9 )
#define SDP_CONTROLLER_CODEC_RTX_VALUE "rtx/90000"
#define SDP_CONTROLLER_CODEC_RTX_VALUE_LENGTH ( 9 )
#define SDP_CONTROLLER_CODEC_APT_VALUE "apt="
#define SDP_CONTROLLER_CODEC_APT_VALUE_LENGTH ( 4 )

#define SDP_CONTROLLER_CODEC_MULAW_DEFAULT_INDEX "0"
#define SDP_CONTROLLER_CODEC_MULAW_DEFAULT_INDEX_LENGTH ( 1 )
#define SDP_CONTROLLER_CODEC_ALAW_DEFAULT_INDEX "8"
#define SDP_CONTROLLER_CODEC_ALAW_DEFAULT_INDEX_LENGTH ( 1 )

static SdpControllerResult_t ParseExtraAttributes( SdpControllerSdpDescription_t * pSdpDescription,
                                                   SdpAttribute_t * pAttribute );
static SdpControllerResult_t ParseMediaAttributes( SdpControllerSdpDescription_t * pSdpDescription,
                                                   const char * pAttributeBuffer,
                                                   size_t attributeBufferLength );
static SdpControllerResult_t ParseSessionAttributes( SdpControllerSdpDescription_t * pSdpDescription,
                                                     const char * pAttributeBuffer,
                                                     size_t attributeBufferLength );
static SdpControllerResult_t SerializeOrigin( SdpSerializerContext_t * pCtx,
                                              SdpControllerOrigin_t * pOrigin );
static SdpControllerResult_t SerializeTiming( SdpSerializerContext_t * pCtx,
                                              SdpControllerTiming_t * pTiming );
static SdpControllerResult_t SerializeAttributes( SdpSerializerContext_t * pCtx,
                                                  SdpControllerAttributes_t * pAttributes,
                                                  uint16_t attributeCount );
static SdpControllerResult_t SerializeConnectionInfo( SdpSerializerContext_t * pCtx,
                                                      SdpControllerConnectionInformation_t * pConnectionInfo );
static SdpControllerResult_t SerializeMedias( SdpSerializerContext_t * pCtx,
                                              SdpControllerMediaDescription_t * pMediaDescriptions,
                                              uint16_t mediaCount );
static SdpControllerResult_t SerializeSdpMessage( SdpControllerSdpDescription_t * pSdpDescription,
                                                  char * pOutputBuffer,
                                                  size_t * pOutputBufferSize );
static SdpControllerResult_t PopulateTransceiverSsrc( char ** ppBuffer,
                                                      size_t * pBufferLength,
                                                      SdpControllerMediaDescription_t * pLocalMediaDescription,
                                                      const Transceiver_t * pTransceiver,
                                                      const char * pCname,
                                                      size_t cnameLength,
                                                      uint32_t containRtx );
static SdpControllerResult_t PopulateRtcpFb( uint32_t payload,
                                             uint16_t twccExtId,
                                             char ** ppBuffer,
                                             size_t * pBufferLength,
                                             SdpControllerMediaDescription_t * pLocalMediaDescription );
static SdpControllerResult_t PopulateCodecAttributesH264Profile42E01FLevelAsymmetryAllowedPacketization( SdpControllerMediaDescription_t * pRemoteMediaDescription,
                                                                                                         const Transceiver_t * pTransceiver,
                                                                                                         uint32_t payload,
                                                                                                         uint32_t rtxPayload,
                                                                                                         char ** ppBuffer,
                                                                                                         size_t * pBufferLength,
                                                                                                         SdpControllerMediaDescription_t * pLocalMediaDescription );
static SdpControllerResult_t PopulateCodecAttributesOpus( SdpControllerMediaDescription_t * pRemoteMediaDescription,
                                                          const Transceiver_t * pTransceiver,
                                                          uint32_t payload,
                                                          uint32_t rtxPayload,
                                                          char ** ppBuffer,
                                                          size_t * pBufferLength,
                                                          SdpControllerMediaDescription_t * pLocalMediaDescription );
static SdpControllerResult_t PopulateCodecAttributesVp8( SdpControllerMediaDescription_t * pRemoteMediaDescription,
                                                         const Transceiver_t * pTransceiver,
                                                         uint32_t payload,
                                                         uint32_t rtxPayload,
                                                         char ** ppBuffer,
                                                         size_t * pBufferLength,
                                                         SdpControllerMediaDescription_t * pLocalMediaDescription );
static SdpControllerResult_t PopulateCodecAttributesMulaw( SdpControllerMediaDescription_t * pRemoteMediaDescription,
                                                           const Transceiver_t * pTransceiver,
                                                           uint32_t payload,
                                                           uint32_t rtxPayload,
                                                           char ** ppBuffer,
                                                           size_t * pBufferLength,
                                                           SdpControllerMediaDescription_t * pLocalMediaDescription );
static SdpControllerResult_t PopulateCodecAttributesAlaw( SdpControllerMediaDescription_t * pRemoteMediaDescription,
                                                          const Transceiver_t * pTransceiver,
                                                          uint32_t payload,
                                                          uint32_t rtxPayload,
                                                          char ** ppBuffer,
                                                          size_t * pBufferLength,
                                                          SdpControllerMediaDescription_t * pLocalMediaDescription );
static SdpControllerResult_t PopulateCodecAttributesH265( SdpControllerMediaDescription_t * pRemoteMediaDescription,
                                                          const Transceiver_t * pTransceiver,
                                                          uint32_t payload,
                                                          uint32_t rtxPayload,
                                                          char ** ppBuffer,
                                                          size_t * pBufferLength,
                                                          SdpControllerMediaDescription_t * pLocalMediaDescription );
static SdpControllerResult_t PopulateCodecAttributes( SdpControllerMediaDescription_t * pRemoteMediaDescription,
                                                      SdpControllerPopulateMediaConfiguration_t populateConfiguration,
                                                      char ** ppBuffer,
                                                      size_t * pBufferLength,
                                                      SdpControllerMediaDescription_t * pLocalMediaDescription );
static const SdpControllerAttributes_t * FindAttributeName( const SdpControllerAttributes_t * pAttributes,
                                                            size_t attributeCount,
                                                            char * pPattern,
                                                            size_t patternLength );
static const SdpControllerAttributes_t * FindFmtpBasedOnCodec( const SdpControllerAttributes_t * pAttributes,
                                                               size_t attributeCount,
                                                               uint32_t codec );

static SdpControllerResult_t ParseExtraAttributes( SdpControllerSdpDescription_t * pSdpDescription,
                                                   SdpAttribute_t * pAttribute )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    StringUtilsResult_t stringResult;

    if( ( pSdpDescription == NULL ) ||
        ( pAttribute == NULL ) )
    {
        LogError( ( "Fail to parse extra attributes, pSdpDescription: %p, pAttribute: %p", pSdpDescription, pAttribute ) );
        ret = SDP_CONTROLLER_RESULT_SDP_SESSION_ATTRIBUTE_MAX_EXCEDDED;
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        /* Checking extra attributes info. */
        if( ( pAttribute->attributeNameLength == SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_FINGERPRINT_LENGTH ) &&
            ( strncmp( SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_FINGERPRINT, pAttribute->pAttributeName, SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_FINGERPRINT_LENGTH ) == 0 ) &&
            ( pAttribute->attributeValueLength > SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_FINGERPRINT_PREFIX_LENGTH ) )
        {
            /* Found fingerprint, store it as extra info. */
            pSdpDescription->quickAccess.pFingerprint = pAttribute->pAttributeValue + SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_FINGERPRINT_PREFIX_LENGTH;
            pSdpDescription->quickAccess.fingerprintLength = pAttribute->attributeValueLength - SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_FINGERPRINT_PREFIX_LENGTH;
        }
        else if( ( pAttribute->attributeNameLength == SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SETUP_LENGTH ) &&
                 ( strncmp( SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SETUP, pAttribute->pAttributeName, SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SETUP_LENGTH ) == 0 ) )
        {
            /* Found setup, store it as extra info. */
            if( ( pAttribute->attributeValueLength == SDP_CONTROLLER_MEDIA_DTLS_ROLE_ACTIVE_LENGTH ) &&
                ( strncmp( SDP_CONTROLLER_MEDIA_DTLS_ROLE_ACTIVE, pAttribute->pAttributeValue, SDP_CONTROLLER_MEDIA_DTLS_ROLE_ACTIVE_LENGTH ) == 0 ) )
            {
                pSdpDescription->quickAccess.dtlsRole = SDP_CONTROLLER_DTLS_ROLE_ACTIVE;
            }
            else
            {
                pSdpDescription->quickAccess.dtlsRole = SDP_CONTROLLER_DTLS_ROLE_ACTPASS;
            }
        }
        else if( ( pAttribute->attributeNameLength == SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_OPTION_LENGTH ) &&
                 ( strncmp( SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_OPTION, pAttribute->pAttributeName, SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_OPTION_LENGTH ) == 0 ) &&
                 ( pAttribute->attributeValueLength == SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_ICE_OPTION_LENGTH ) &&
                 ( strncmp( SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_ICE_OPTION, pAttribute->pAttributeValue, SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_ICE_OPTION_LENGTH ) == 0 ) )
        {
            pSdpDescription->quickAccess.isIceTrickle = 1U;
        }
        else if( ( pAttribute->attributeNameLength == SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_UFRAG_LENGTH ) &&
                 ( strncmp( SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_UFRAG, pAttribute->pAttributeName, SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_UFRAG_LENGTH ) == 0 ) )
        {
            pSdpDescription->quickAccess.pIceUfrag = pAttribute->pAttributeValue;
            pSdpDescription->quickAccess.iceUfragLength = pAttribute->attributeValueLength;
        }
        else if( ( pAttribute->attributeNameLength == SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_PWD_LENGTH ) &&
                 ( strncmp( SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_PWD, pAttribute->pAttributeName, SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_PWD_LENGTH ) == 0 ) )
        {
            pSdpDescription->quickAccess.pIcePwd = pAttribute->pAttributeValue;
            pSdpDescription->quickAccess.icePwdLength = pAttribute->attributeValueLength;
        }
        else if( ( pAttribute->attributeNameLength == SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_EXTMAP_LENGTH ) &&
                 ( strncmp( SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_EXTMAP, pAttribute->pAttributeName, SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_EXTMAP_LENGTH ) == 0 ) &&
                 ( pAttribute->attributeValueLength > SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_TWCC_EXT_URL_LENGTH ) )
        {
            /* The attribute value length is confirmed larger than URL. */
            size_t length = pAttribute->attributeValueLength - SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_TWCC_EXT_URL_LENGTH;
            const char * pFindStart = pAttribute->pAttributeValue + length;
            if( strncmp( SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_TWCC_EXT_URL, pFindStart, SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_TWCC_EXT_URL_LENGTH ) == 0 )
            {
                /* Found TWCC ext URL. */
                stringResult = StringUtils_ConvertStringToUl( pAttribute->pAttributeValue, length, &pSdpDescription->quickAccess.twccExtId );
                if( stringResult != STRING_UTILS_RESULT_OK )
                {
                    LogError( ( "StringUtils_ConvertStringToUl fail, result %d, converting %.*s to %lu",
                                stringResult,
                                ( int ) length, pAttribute->pAttributeValue,
                                pSdpDescription->quickAccess.twccExtId ) );
                    ret = SDP_CONTROLLER_RESULT_SDP_INVALID_TWCC_ID;
                }
                else
                {
                    LogDebug( ( "Found TWCC, ID: %lu", pSdpDescription->quickAccess.twccExtId ) );
                }
            }
        }
        else if( ( pAttribute->attributeNameLength == SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_CANDIDATE_LENGTH ) &&
                 ( strncmp( SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_CANDIDATE, pAttribute->pAttributeName, SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_CANDIDATE_LENGTH ) == 0 ) )
        {
            /* Ensure we do not exceed the maximum candidate limit */
            if( pSdpDescription->quickAccess.remoteCandidateCount < SDP_CONTROLLER_MAX_SDP_ATTRIBUTES_COUNT )
            {
                pSdpDescription->quickAccess.pRemoteCandidates[ pSdpDescription->quickAccess.remoteCandidateCount ] = pAttribute->pAttributeValue;
                pSdpDescription->quickAccess.remoteCandidateLengths[ pSdpDescription->quickAccess.remoteCandidateCount ] = pAttribute->attributeValueLength;
                pSdpDescription->quickAccess.remoteCandidateCount++;
            }
            else
            {
                LogWarn( ( "Max candidate limit reached (%d), ignoring extra candidates", SDP_CONTROLLER_MAX_SDP_ATTRIBUTES_COUNT ) );
            }
        }
    }

    return ret;
}

static SdpControllerResult_t ParseMediaAttributes( SdpControllerSdpDescription_t * pSdpDescription,
                                                   const char * pAttributeBuffer,
                                                   size_t attributeBufferLength )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    SdpResult_t sdpResult = SDP_RESULT_OK;
    SdpAttribute_t attribute;
    uint8_t mediaIndex = pSdpDescription->mediaCount - 1;
    uint8_t * pAttributeCount = &pSdpDescription->mediaDescriptions[ mediaIndex ].mediaAttributesCount;

    if( pSdpDescription->mediaDescriptions[ mediaIndex ].mediaAttributesCount >= SDP_CONTROLLER_MAX_SDP_ATTRIBUTES_COUNT )
    {
        ret = SDP_CONTROLLER_RESULT_SDP_MEDIA_ATTRIBUTE_MAX_EXCEDDED;
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        sdpResult = SdpDeserializer_ParseAttribute( pAttributeBuffer, attributeBufferLength, &attribute );
        if( sdpResult != SDP_RESULT_OK )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_DESERIALIZER_PARSE_ATTRIBUTE;
        }
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        pSdpDescription->mediaDescriptions[ mediaIndex ].attributes[ *pAttributeCount ].pAttributeName = attribute.pAttributeName;
        pSdpDescription->mediaDescriptions[ mediaIndex ].attributes[ *pAttributeCount ].attributeNameLength = attribute.attributeNameLength;
        pSdpDescription->mediaDescriptions[ mediaIndex ].attributes[ *pAttributeCount ].pAttributeValue = attribute.pAttributeValue;
        pSdpDescription->mediaDescriptions[ mediaIndex ].attributes[ *pAttributeCount ].attributeValueLength = attribute.attributeValueLength;
        ( *pAttributeCount )++;
    }

    /* Parse extra attributes to accerlate SDP creation later. */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        ret = ParseExtraAttributes( pSdpDescription, &attribute );
    }

    return ret;
}

static SdpControllerResult_t ParseSessionAttributes( SdpControllerSdpDescription_t * pSdpDescription,
                                                     const char * pAttributeBuffer,
                                                     size_t attributeBufferLength )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    SdpResult_t sdpResult = SDP_RESULT_OK;
    SdpAttribute_t attribute;

    if( pSdpDescription->sessionAttributesCount >= SDP_CONTROLLER_MAX_SDP_ATTRIBUTES_COUNT )
    {
        ret = SDP_CONTROLLER_RESULT_SDP_SESSION_ATTRIBUTE_MAX_EXCEDDED;
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        sdpResult = SdpDeserializer_ParseAttribute( pAttributeBuffer, attributeBufferLength, &attribute );
        if( sdpResult != SDP_RESULT_OK )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_DESERIALIZER_PARSE_ATTRIBUTE;
        }
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        pSdpDescription->attributes[ pSdpDescription->sessionAttributesCount ].pAttributeName = attribute.pAttributeName;
        pSdpDescription->attributes[ pSdpDescription->sessionAttributesCount ].attributeNameLength = attribute.attributeNameLength;
        pSdpDescription->attributes[ pSdpDescription->sessionAttributesCount ].pAttributeValue = attribute.pAttributeValue;
        pSdpDescription->attributes[ pSdpDescription->sessionAttributesCount ].attributeValueLength = attribute.attributeValueLength;
        pSdpDescription->sessionAttributesCount++;
    }

    /* Parse extra attributes to accerlate SDP creation later. */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        ret = ParseExtraAttributes( pSdpDescription, &attribute );
    }

    return ret;
}

static SdpControllerResult_t SerializeOrigin( SdpSerializerContext_t * pCtx,
                                              SdpControllerOrigin_t * pOrigin )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    SdpResult_t sdpResult = SDP_RESULT_OK;
    SdpOriginator_t origin;

    memset( &origin, 0, sizeof( SdpOriginator_t ) );
    origin.sessionId = pOrigin->sessionId;
    origin.sessionVersion = pOrigin->sessionVersion;
    origin.pUserName = pOrigin->pUserName;
    origin.userNameLength = pOrigin->userNameLength;
    origin.connectionInfo.networkType = SDP_NETWORK_IN;
    origin.connectionInfo.addressType = SDP_ADDRESS_IPV4;
    origin.connectionInfo.pAddress = pOrigin->sdpConnectionInformation.pConnectionAddress;
    origin.connectionInfo.addressLength = pOrigin->sdpConnectionInformation.connectionAddressLength;

    sdpResult = SdpSerializer_AddOriginator( pCtx, SDP_TYPE_ORIGINATOR, &origin );
    if( sdpResult != SDP_RESULT_OK )
    {
        LogError( ( "Serialize SDP origin failure, result: %d", sdpResult ) );
        ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SERIALIZER_ADD;
    }

    return ret;
}

static SdpControllerResult_t SerializeTiming( SdpSerializerContext_t * pCtx,
                                              SdpControllerTiming_t * pTiming )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    SdpResult_t sdpResult = SDP_RESULT_OK;
    SdpTimeDescription_t time;

    memset( &time, 0, sizeof( SdpTimeDescription_t ) );
    time.startTime = pTiming->startTime;
    time.stopTime = pTiming->stopTime;

    sdpResult = SdpSerializer_AddTimeActive( pCtx, SDP_TYPE_TIME_ACTIVE, &time );
    if( sdpResult != SDP_RESULT_OK )
    {
        LogError( ( "Serialize SDP time active failure, result: %d", sdpResult ) );
        ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SERIALIZER_ADD;
    }

    return ret;
}

static SdpControllerResult_t SerializeAttributes( SdpSerializerContext_t * pCtx,
                                                  SdpControllerAttributes_t * pAttributes,
                                                  uint16_t attributeCount )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    SdpResult_t sdpResult = SDP_RESULT_OK;
    SdpAttribute_t attribute;
    uint32_t i;
    SdpControllerAttributes_t * pCurrentAttrubute = pAttributes;

    for( i = 0; i < attributeCount; i++ )
    {
        attribute.pAttributeName = ( pCurrentAttrubute + i )->pAttributeName;
        attribute.attributeNameLength = ( pCurrentAttrubute + i )->attributeNameLength;
        attribute.pAttributeValue = ( pCurrentAttrubute + i )->pAttributeValue;
        attribute.attributeValueLength = ( pCurrentAttrubute + i )->attributeValueLength;

        sdpResult = SdpSerializer_AddAttribute( pCtx, SDP_TYPE_ATTRIBUTE, &attribute );
        if( sdpResult != SDP_RESULT_OK )
        {
            LogError( ( "Serialize SDP attribute failure, result: %x, attribute name: %.*s, value: %.*s",
                        sdpResult,
                        ( attribute.attributeNameLength ), attribute.pAttributeName,
                        ( attribute.attributeValueLength ), attribute.pAttributeValue ) );
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SERIALIZER_ADD;
        }
    }

    return ret;
}

static SdpControllerResult_t SerializeConnectionInfo( SdpSerializerContext_t * pCtx,
                                                      SdpControllerConnectionInformation_t * pConnectionInfo )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    SdpResult_t sdpResult = SDP_RESULT_OK;
    SdpConnectionInfo_t connInfo;

    connInfo.networkType = SDP_NETWORK_IN;
    connInfo.addressType = SDP_ADDRESS_IPV4;
    connInfo.pAddress = pConnectionInfo->pConnectionAddress;
    connInfo.addressLength = pConnectionInfo->connectionAddressLength;

    sdpResult = SdpSerializer_AddConnectionInfo( pCtx, SDP_TYPE_CONNINFO, &connInfo );
    if( sdpResult != SDP_RESULT_OK )
    {
        LogError( ( "Serialize SDP connection information failure, result: %d", sdpResult ) );
        ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SERIALIZER_ADD;
    }

    return ret;
}

static SdpControllerResult_t SerializeMedias( SdpSerializerContext_t * pCtx,
                                              SdpControllerMediaDescription_t * pMediaDescriptions,
                                              uint16_t mediaCount )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    SdpResult_t sdpResult = SDP_RESULT_OK;
    uint32_t i;
    SdpControllerMediaDescription_t * pCurrentMedia = pMediaDescriptions;

    for( i = 0; i < mediaCount; i++ )
    {
        pCurrentMedia = pMediaDescriptions + i;

        /* Media name */
        sdpResult = SdpSerializer_AddBuffer( pCtx, SDP_TYPE_MEDIA, pCurrentMedia->pMediaName, pCurrentMedia->mediaNameLength );
        if( sdpResult != SDP_RESULT_OK )
        {
            LogError( ( "Serialize SDP media name failure, result: %d", sdpResult ) );
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SERIALIZER_ADD;
            break;
        }

        /* Media title */
        if( pCurrentMedia->pMediaTitle )
        {
            sdpResult = SdpSerializer_AddBuffer( pCtx, SDP_TYPE_MEDIA_TITLE, pCurrentMedia->pMediaTitle, pCurrentMedia->mediaTitleLength );
            if( sdpResult != SDP_RESULT_OK )
            {
                LogError( ( "Serialize SDP media title failure, result: %d", sdpResult ) );
                ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SERIALIZER_ADD;
                break;
            }
        }

        /* Media connection information */
        ret = SerializeConnectionInfo( pCtx, &pCurrentMedia->connectionInformation );
        if( ret != SDP_CONTROLLER_RESULT_OK )
        {
            break;
        }

        /* Append media attributes. */
        ret = SerializeAttributes( pCtx, &pCurrentMedia->attributes[ 0 ], pCurrentMedia->mediaAttributesCount );
        if( ret != SDP_CONTROLLER_RESULT_OK )
        {
            break;
        }
    }

    return ret;
}

static SdpControllerResult_t SerializeSdpMessage( SdpControllerSdpDescription_t * pSdpDescription,
                                                  char * pOutputBuffer,
                                                  size_t * pOutputBufferSize )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    SdpResult_t sdpResult = SDP_RESULT_OK;
    SdpSerializerContext_t ctx;
    const char * pBuffer;

    sdpResult = SdpSerializer_Init( &ctx, pOutputBuffer, *pOutputBufferSize );
    if( sdpResult != SDP_RESULT_OK )
    {
        LogError( ( "Init SDP serializer failure, result: %d", sdpResult ) );
        ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SERIALIZER_INIT;
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        /* Append version. */
        sdpResult = SdpSerializer_AddU64( &ctx, SDP_TYPE_VERSION, pSdpDescription->version );
        if( sdpResult != SDP_RESULT_OK )
        {
            LogError( ( "Serialize SDP version failure, result: %d", sdpResult ) );
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SERIALIZER_ADD;
        }
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        /* Append origin. */
        ret = SerializeOrigin( &ctx, &pSdpDescription->origin );
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        /* Append session name. */
        sdpResult = SdpSerializer_AddBuffer( &ctx, SDP_TYPE_SESSION_NAME, pSdpDescription->pSessionName, pSdpDescription->sessionNameLength );
        if( sdpResult != SDP_RESULT_OK )
        {
            LogError( ( "Serialize SDP session name failure, result: %d", sdpResult ) );
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SERIALIZER_ADD;
        }
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        /* Append timing description. */
        ret = SerializeTiming( &ctx, &pSdpDescription->timingDescription );
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        /* Append session attributes. */
        ret = SerializeAttributes( &ctx, &pSdpDescription->attributes[ 0 ], pSdpDescription->sessionAttributesCount );
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        /* Append media information. */
        ret = SerializeMedias( &ctx, &pSdpDescription->mediaDescriptions[ 0 ], pSdpDescription->mediaCount );
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        sdpResult = SdpSerializer_Finalize( &ctx, &pBuffer, pOutputBufferSize );
        if( sdpResult != SDP_RESULT_OK )
        {
            LogError( ( "Serialize SDP finalize failure, result: %d", sdpResult ) );
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SERIALIZER_ADD;
        }
    }

    return ret;
}

static SdpControllerResult_t PopulateTransceiverSsrc( char ** ppBuffer,
                                                      size_t * pBufferLength,
                                                      SdpControllerMediaDescription_t * pLocalMediaDescription,
                                                      const Transceiver_t * pTransceiver,
                                                      const char * pCname,
                                                      size_t cnameLength,
                                                      uint32_t containRtx )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    SdpControllerAttributes_t * pTargetAttribute = NULL;
    uint8_t * pTargetAttributeCount = NULL;
    int written = 0;
    char * pCurBuffer = NULL;
    size_t remainSize = 0;

    if( ( ppBuffer == NULL ) ||
        ( pBufferLength == NULL ) ||
        ( pLocalMediaDescription == NULL ) ||
        ( pTransceiver == NULL ) ||
        ( pCname == NULL ) )
    {
        LogError( ( "Invalid input, ppBuffer: %p, pBufferLength: %p, pLocalMediaDescription: %p, pTransceiver: %p, pCname: %p",
                    ppBuffer,
                    pBufferLength,
                    pLocalMediaDescription,
                    pTransceiver,
                    pCname ) );
        ret = SDP_CONTROLLER_RESULT_BAD_PARAMETER;
    }
    else
    {
        /* Initialize current buffer pointer/size and the pointer to the attribute count. */
        pCurBuffer = *ppBuffer;
        remainSize = *pBufferLength;
        pTargetAttributeCount = &pLocalMediaDescription->mediaAttributesCount;
    }

    /* CNAME */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC_LENGTH;

        written = snprintf( pCurBuffer, remainSize, "%lu cname:%.*s",
                            pTransceiver->ssrc,
                            ( int ) cnameLength, pCname );
        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
            LogError( ( "buffer has no space for SSRC CNAME" ) );
        }
        else
        {
            pTargetAttribute->pAttributeValue = pCurBuffer;
            pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
            *pTargetAttributeCount += 1;

            pCurBuffer += written;
            remainSize -= written;
        }
    }

    /* msid */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC_LENGTH;

        written = snprintf( pCurBuffer, remainSize, "%lu msid:%.*s %.*s",
                            pTransceiver->ssrc,
                            ( int ) pTransceiver->streamIdLength, pTransceiver->streamId,
                            ( int ) pTransceiver->trackIdLength, pTransceiver->trackId );
        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
            LogError( ( "buffer has no space for SSRC msid" ) );
        }
        else
        {
            pTargetAttribute->pAttributeValue = pCurBuffer;
            pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
            *pTargetAttributeCount += 1;

            pCurBuffer += written;
            remainSize -= written;
        }
    }

    /* mslabel */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC_LENGTH;

        written = snprintf( pCurBuffer, remainSize, "%lu mslabel:%.*s",
                            pTransceiver->ssrc,
                            ( int ) pTransceiver->streamIdLength, pTransceiver->streamId );
        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
            LogError( ( "buffer has no space for SSRC mslabel" ) );
        }
        else
        {
            pTargetAttribute->pAttributeValue = pCurBuffer;
            pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
            *pTargetAttributeCount += 1;

            pCurBuffer += written;
            remainSize -= written;
        }
    }

    /* label */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC_LENGTH;

        written = snprintf( pCurBuffer, remainSize, "%lu label:%.*s",
                            pTransceiver->ssrc,
                            ( int ) pTransceiver->trackIdLength, pTransceiver->trackId );
        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
            LogError( ( "buffer has no space for SSRC label" ) );
        }
        else
        {
            pTargetAttribute->pAttributeValue = pCurBuffer;
            pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
            *pTargetAttributeCount += 1;

            pCurBuffer += written;
            remainSize -= written;
        }
    }

    /* For RTX: cname */
    if( ( ret == SDP_CONTROLLER_RESULT_OK ) && ( containRtx != 0 ) )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC_LENGTH;

        written = snprintf( pCurBuffer, remainSize, "%lu cname:%.*s",
                            pTransceiver->rtxSsrc,
                            ( int ) cnameLength, pCname );
        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
            LogError( ( "buffer has no space for RTX SSRC CNAME" ) );
        }
        else
        {
            pTargetAttribute->pAttributeValue = pCurBuffer;
            pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
            *pTargetAttributeCount += 1;

            pCurBuffer += written;
            remainSize -= written;
        }
    }

    /* For RTX: msid */
    if( ( ret == SDP_CONTROLLER_RESULT_OK ) && ( containRtx != 0 ) )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC_LENGTH;

        written = snprintf( pCurBuffer, remainSize, "%lu msid:%.*s %.*sRTX",
                            pTransceiver->rtxSsrc,
                            ( int ) pTransceiver->streamIdLength, pTransceiver->streamId,
                            ( int ) pTransceiver->trackIdLength, pTransceiver->trackId );
        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
            LogError( ( "buffer has no space for RTX SSRC msid" ) );
        }
        else
        {
            pTargetAttribute->pAttributeValue = pCurBuffer;
            pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
            *pTargetAttributeCount += 1;

            pCurBuffer += written;
            remainSize -= written;
        }
    }

    /* For RTX: mslabel */
    if( ( ret == SDP_CONTROLLER_RESULT_OK ) && ( containRtx != 0 ) )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC_LENGTH;

        written = snprintf( pCurBuffer, remainSize, "%lu mslabel:%.*sRTX",
                            pTransceiver->rtxSsrc,
                            ( int ) pTransceiver->streamIdLength, pTransceiver->streamId );
        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
            LogError( ( "buffer has no space for RTX SSRC mslabel" ) );
        }
        else
        {
            pTargetAttribute->pAttributeValue = pCurBuffer;
            pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
            *pTargetAttributeCount += 1;

            pCurBuffer += written;
            remainSize -= written;
        }
    }

    /* For RTX: label */
    if( ( ret == SDP_CONTROLLER_RESULT_OK ) && ( containRtx != 0 ) )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC_LENGTH;

        written = snprintf( pCurBuffer, remainSize, "%lu label:%.*sRTX",
                            pTransceiver->rtxSsrc,
                            ( int ) pTransceiver->trackIdLength, pTransceiver->trackId );
        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
            LogError( ( "buffer has no space for RTX SSRC label" ) );
        }
        else
        {
            pTargetAttribute->pAttributeValue = pCurBuffer;
            pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
            *pTargetAttributeCount += 1;

            pCurBuffer += written;
            remainSize -= written;
        }
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        *ppBuffer = pCurBuffer;
        *pBufferLength = remainSize;
    }

    return ret;
}

static SdpControllerResult_t PopulateRtcpFb( uint32_t payload,
                                             uint16_t twccExtId,
                                             char ** ppBuffer,
                                             size_t * pBufferLength,
                                             SdpControllerMediaDescription_t * pLocalMediaDescription )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    SdpControllerAttributes_t * pTargetAttribute = NULL;
    uint8_t * pTargetAttributeCount = NULL;
    int written = 0;
    char * pCurBuffer = NULL;
    size_t remainSize = 0;

    if( ( ppBuffer == NULL ) ||
        ( pBufferLength == NULL ) ||
        ( pLocalMediaDescription == NULL ) )
    {
        LogError( ( "Invalid input, ppBuffer: %p, pBufferLength: %p, pLocalMediaDescription: %p",
                    ppBuffer,
                    pBufferLength,
                    pLocalMediaDescription ) );
        ret = SDP_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        pCurBuffer = *ppBuffer;
        remainSize = *pBufferLength;
        pTargetAttributeCount = &pLocalMediaDescription->mediaAttributesCount;

        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_FB;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_FB_LENGTH;

        written = snprintf( pCurBuffer, remainSize, "%lu %s",
                            payload,
                            SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_GOOG_REMB );
        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
            LogError( ( "buffer has no space for rtcp-fb H264 value" ) );
        }
        else
        {
            pTargetAttribute->pAttributeValue = pCurBuffer;
            pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
            *pTargetAttributeCount += 1;

            pCurBuffer += written;
            remainSize -= written;
        }
    }

    /* Append "rtcp-fb: ${codec} transport-cc" only if twccId is valid. */
    if( ( ret == SDP_CONTROLLER_RESULT_OK ) && ( twccExtId > 0 ) )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_FB;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_FB_LENGTH;

        written = snprintf( pCurBuffer, remainSize, "%lu %s",
                            payload,
                            SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_TRANSPORT_CC );
        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
            LogError( ( "buffer has no space for rtcp-fb transport-cc" ) );
        }
        else
        {
            pTargetAttribute->pAttributeValue = pCurBuffer;
            pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
            *pTargetAttributeCount += 1;

            pCurBuffer += written;
            remainSize -= written;
        }
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        *ppBuffer = pCurBuffer;
        *pBufferLength = remainSize;
    }

    return ret;
}

static SdpControllerResult_t PopulateCodecAttributesH264Profile42E01FLevelAsymmetryAllowedPacketization( SdpControllerMediaDescription_t * pRemoteMediaDescription,
                                                                                                         const Transceiver_t * pTransceiver,
                                                                                                         uint32_t payload,
                                                                                                         uint32_t rtxPayload,
                                                                                                         char ** ppBuffer,
                                                                                                         size_t * pBufferLength,
                                                                                                         SdpControllerMediaDescription_t * pLocalMediaDescription )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    uint8_t isOffer = pRemoteMediaDescription ? 0 : 1;
    SdpControllerAttributes_t * pTargetAttribute = NULL;
    const SdpControllerAttributes_t * pSourceAttribute = NULL;
    uint8_t * pTargetAttributeCount = NULL;
    int written = 0;
    char * pCurBuffer = NULL;
    size_t remainSize = 0;

    pTargetAttributeCount = &pLocalMediaDescription->mediaAttributesCount;
    pCurBuffer = *ppBuffer;
    remainSize = *pBufferLength;

    /* rtpmap for payload */
    pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
    pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTPMAP;
    pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTPMAP_LENGTH;

    written = snprintf( pCurBuffer, remainSize, "%lu %s",
                        payload,
                        SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_H264 );
    if( written < 0 )
    {
        ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
        LogError( ( "snprintf return unexpected value %d", written ) );
    }
    else if( written == remainSize )
    {
        ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
        LogError( ( "buffer has no space for rtpmap H264 value" ) );
    }
    else
    {
        pTargetAttribute->pAttributeValue = pCurBuffer;
        pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
        *pTargetAttributeCount += 1;

        pCurBuffer += written;
        remainSize -= written;
    }

    /* rtcp-fb: ${codec} nack */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_FB;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_FB_LENGTH;

        written = snprintf( pCurBuffer, remainSize, "%lu %s",
                            payload,
                            SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_VALUE );
        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
            LogError( ( "buffer has no space for rtcp-fb H264 value" ) );
        }
        else
        {
            pTargetAttribute->pAttributeValue = pCurBuffer;
            pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
            *pTargetAttributeCount += 1;

            pCurBuffer += written;
            remainSize -= written;
        }
    }

    /* fmtp */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        if( !isOffer )
        {
            /* If creating SDP answer, try find fmtp from the remote description. */
            pSourceAttribute = FindFmtpBasedOnCodec( pRemoteMediaDescription->attributes,
                                                     pRemoteMediaDescription->mediaAttributesCount,
                                                     payload );
        }

        /* Set fmtp only if:
         *   1. It's offer, or
         *   2. It's not offer but we found the corresponding fmtp from remote description. */
        if( isOffer || pSourceAttribute )
        {
            pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
            pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_FMTP;
            pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_FMTP_LENGTH;

            if( isOffer )
            {
                written = snprintf( pCurBuffer, remainSize, "%lu %s",
                                    payload,
                                    SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_FMTP_H264_PROFILE_42E01F_LEVEL_ASYMMETRY_ALLOWED_PACKETIZATION );
            }
            else
            {
                written = snprintf( pCurBuffer, remainSize, "%.*s",
                                    ( int ) pSourceAttribute->attributeValueLength, pSourceAttribute->pAttributeValue );
            }

            if( written < 0 )
            {
                ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
                LogError( ( "snprintf return unexpected value %d", written ) );
            }
            else if( written == remainSize )
            {
                ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
                LogError( ( "buffer has no space for fmtp" ) );
            }
            else
            {
                pTargetAttribute->pAttributeValue = pCurBuffer;
                pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
                *pTargetAttributeCount += 1;

                pCurBuffer += written;
                remainSize -= written;
            }
        }
    }

    /* RTX rtpmap */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        if( rtxPayload != 0 )
        {
            pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
            pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTPMAP;
            pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTPMAP_LENGTH;

            written = snprintf( pCurBuffer, remainSize, "%lu %s",
                                rtxPayload,
                                SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_RTX_H264 );
            if( written < 0 )
            {
                ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
                LogError( ( "snprintf return unexpected value %d", written ) );
            }
            else if( written == remainSize )
            {
                ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
                LogError( ( "buffer has no space for rtpmap H264 RTX value" ) );
            }
            else
            {
                pTargetAttribute->pAttributeValue = pCurBuffer;
                pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
                *pTargetAttributeCount += 1;

                pCurBuffer += written;
                remainSize -= written;
            }
        }
    }

    /* RTX fmtp */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        if( rtxPayload != 0 )
        {
            pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
            pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_FMTP;
            pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_FMTP_LENGTH;

            written = snprintf( pCurBuffer, remainSize, "%lu apt=%lu",
                                rtxPayload,
                                payload );

            if( written < 0 )
            {
                ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
                LogError( ( "snprintf return unexpected value %d", written ) );
            }
            else if( written == remainSize )
            {
                ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
                LogError( ( "buffer has no space for RTX fmtp" ) );
            }
            else
            {
                pTargetAttribute->pAttributeValue = pCurBuffer;
                pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
                *pTargetAttributeCount += 1;

                pCurBuffer += written;
                remainSize -= written;
            }
        }
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        *ppBuffer = pCurBuffer;
        *pBufferLength = remainSize;
    }

    return ret;
}

static SdpControllerResult_t PopulateCodecAttributesOpus( SdpControllerMediaDescription_t * pRemoteMediaDescription,
                                                          const Transceiver_t * pTransceiver,
                                                          uint32_t payload,
                                                          uint32_t rtxPayload,
                                                          char ** ppBuffer,
                                                          size_t * pBufferLength,
                                                          SdpControllerMediaDescription_t * pLocalMediaDescription )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    uint8_t isOffer = pRemoteMediaDescription ? 0 : 1;
    SdpControllerAttributes_t * pTargetAttribute = NULL;
    const SdpControllerAttributes_t * pSourceAttribute = NULL;
    uint8_t * pTargetAttributeCount = NULL;
    int written = 0;
    char * pCurBuffer = NULL;
    size_t remainSize = 0;

    pTargetAttributeCount = &pLocalMediaDescription->mediaAttributesCount;
    pCurBuffer = *ppBuffer;
    remainSize = *pBufferLength;

    /* rtpmap */
    pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
    pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTPMAP;
    pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTPMAP_LENGTH;

    written = snprintf( pCurBuffer, remainSize, "%lu %s",
                        payload,
                        SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_OPUS );
    if( written < 0 )
    {
        ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
        LogError( ( "snprintf return unexpected value %d", written ) );
    }
    else if( written == remainSize )
    {
        ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
        LogError( ( "buffer has no space for rtpmap OPUS" ) );
    }
    else
    {
        pTargetAttribute->pAttributeValue = pCurBuffer;
        pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
        *pTargetAttributeCount += 1;

        pCurBuffer += written;
        remainSize -= written;
    }

    /* rtpmap for RTX */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        if( rtxPayload != 0 )
        {
            LogWarn( ( "Ignore RTX for OPUS, RTX payload: %lu.", rtxPayload ) );
        }
    }

    /* fmtp */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        if( !isOffer )
        {
            /* If creating SDP answer, try find fmtp from the remote description. */
            pSourceAttribute = FindFmtpBasedOnCodec( pRemoteMediaDescription->attributes,
                                                     pRemoteMediaDescription->mediaAttributesCount,
                                                     payload );
        }

        /* Set fmtp only if:
         *   1. It's offer, or
         *   2. It's not offer but we found the corresponding fmtp from remote description. */
        if( isOffer || pSourceAttribute )
        {
            pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
            pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_FMTP;
            pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_FMTP_LENGTH;

            if( isOffer )
            {
                written = snprintf( pCurBuffer, remainSize, "%lu %s",
                                    payload,
                                    SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_FMTP_OPUS );
            }
            else
            {
                written = snprintf( pCurBuffer, remainSize, "%.*s",
                                    ( int ) pSourceAttribute->attributeValueLength, pSourceAttribute->pAttributeValue );
            }

            if( written < 0 )
            {
                ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
                LogError( ( "snprintf return unexpected value %d", written ) );
            }
            else if( written == remainSize )
            {
                ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
                LogError( ( "buffer has no space for fmtp" ) );
            }
            else
            {
                pTargetAttribute->pAttributeValue = pCurBuffer;
                pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
                *pTargetAttributeCount += 1;

                pCurBuffer += written;
                remainSize -= written;
            }
        }
    }

    /* rtcp-fb: ${codec} nack */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_FB;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_FB_LENGTH;

        written = snprintf( pCurBuffer, remainSize, "%lu %s",
                            payload,
                            SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_VALUE );
        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
            LogError( ( "buffer has no space for rtcp-fb H264 value" ) );
        }
        else
        {
            pTargetAttribute->pAttributeValue = pCurBuffer;
            pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
            *pTargetAttributeCount += 1;

            pCurBuffer += written;
            remainSize -= written;
        }
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        *ppBuffer = pCurBuffer;
        *pBufferLength = remainSize;
    }

    return ret;
}

static SdpControllerResult_t PopulateCodecAttributesVp8( SdpControllerMediaDescription_t * pRemoteMediaDescription,
                                                         const Transceiver_t * pTransceiver,
                                                         uint32_t payload,
                                                         uint32_t rtxPayload,
                                                         char ** ppBuffer,
                                                         size_t * pBufferLength,
                                                         SdpControllerMediaDescription_t * pLocalMediaDescription )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    SdpControllerAttributes_t * pTargetAttribute = NULL;
    uint8_t * pTargetAttributeCount = NULL;
    int written = 0;
    char * pCurBuffer = NULL;
    size_t remainSize = 0;

    pTargetAttributeCount = &pLocalMediaDescription->mediaAttributesCount;
    pCurBuffer = *ppBuffer;
    remainSize = *pBufferLength;

    /* rtpmap */
    pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
    pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTPMAP;
    pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTPMAP_LENGTH;

    written = snprintf( pCurBuffer, remainSize, "%lu %s",
                        payload,
                        SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_VP8 );
    if( written < 0 )
    {
        ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
        LogError( ( "snprintf return unexpected value %d", written ) );
    }
    else if( written == remainSize )
    {
        ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
        LogError( ( "buffer has no space for rtpmap VP8" ) );
    }
    else
    {
        pTargetAttribute->pAttributeValue = pCurBuffer;
        pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
        *pTargetAttributeCount += 1;

        pCurBuffer += written;
        remainSize -= written;
    }

    /* rtpmap for RTX */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        if( rtxPayload != 0 )
        {
            LogWarn( ( "Ignore RTX for VP8, RTX payload: %lu.", rtxPayload ) );
        }
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        *ppBuffer = pCurBuffer;
        *pBufferLength = remainSize;
    }

    return ret;
}

static SdpControllerResult_t PopulateCodecAttributesMulaw( SdpControllerMediaDescription_t * pRemoteMediaDescription,
                                                           const Transceiver_t * pTransceiver,
                                                           uint32_t payload,
                                                           uint32_t rtxPayload,
                                                           char ** ppBuffer,
                                                           size_t * pBufferLength,
                                                           SdpControllerMediaDescription_t * pLocalMediaDescription )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    SdpControllerAttributes_t * pTargetAttribute = NULL;
    uint8_t * pTargetAttributeCount = NULL;
    int written = 0;
    char * pCurBuffer = NULL;
    size_t remainSize = 0;

    pTargetAttributeCount = &pLocalMediaDescription->mediaAttributesCount;
    pCurBuffer = *ppBuffer;
    remainSize = *pBufferLength;

    /* rtpmap */
    pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
    pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTPMAP;
    pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTPMAP_LENGTH;

    written = snprintf( pCurBuffer, remainSize, "%lu %s",
                        payload,
                        SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_MULAW );
    if( written < 0 )
    {
        ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
        LogError( ( "snprintf return unexpected value %d", written ) );
    }
    else if( written == remainSize )
    {
        ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
        LogError( ( "buffer has no space for rtpmap MULAW" ) );
    }
    else
    {
        pTargetAttribute->pAttributeValue = pCurBuffer;
        pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
        *pTargetAttributeCount += 1;

        pCurBuffer += written;
        remainSize -= written;
    }

    /* rtpmap for RTX */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        if( rtxPayload != 0 )
        {
            LogWarn( ( "Ignore RTX for MULAW, RTX payload: %lu.", rtxPayload ) );
        }
    }

    /* rtcp-fb: ${codec} nack */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_FB;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_FB_LENGTH;

        written = snprintf( pCurBuffer, remainSize, "%lu %s",
                            payload,
                            SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_VALUE );
        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
            LogError( ( "buffer has no space for rtcp-fb H264 value" ) );
        }
        else
        {
            pTargetAttribute->pAttributeValue = pCurBuffer;
            pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
            *pTargetAttributeCount += 1;

            pCurBuffer += written;
            remainSize -= written;
        }
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        *ppBuffer = pCurBuffer;
        *pBufferLength = remainSize;
    }

    return ret;
}

static SdpControllerResult_t PopulateCodecAttributesAlaw( SdpControllerMediaDescription_t * pRemoteMediaDescription,
                                                          const Transceiver_t * pTransceiver,
                                                          uint32_t payload,
                                                          uint32_t rtxPayload,
                                                          char ** ppBuffer,
                                                          size_t * pBufferLength,
                                                          SdpControllerMediaDescription_t * pLocalMediaDescription )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    SdpControllerAttributes_t * pTargetAttribute = NULL;
    uint8_t * pTargetAttributeCount = NULL;
    int written = 0;
    char * pCurBuffer = NULL;
    size_t remainSize = 0;

    pTargetAttributeCount = &pLocalMediaDescription->mediaAttributesCount;
    pCurBuffer = *ppBuffer;
    remainSize = *pBufferLength;

    /* rtpmap */
    pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
    pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTPMAP;
    pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTPMAP_LENGTH;

    written = snprintf( pCurBuffer, remainSize, "%lu %s",
                        payload,
                        SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_ALAW );
    if( written < 0 )
    {
        ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
        LogError( ( "snprintf return unexpected value %d", written ) );
    }
    else if( written == remainSize )
    {
        ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
        LogError( ( "buffer has no space for rtpmap ALAW" ) );
    }
    else
    {
        pTargetAttribute->pAttributeValue = pCurBuffer;
        pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
        *pTargetAttributeCount += 1;

        pCurBuffer += written;
        remainSize -= written;
    }

    /* rtpmap for RTX */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        if( rtxPayload != 0 )
        {
            LogWarn( ( "Ignore RTX for ALAW, RTX payload: %lu.", rtxPayload ) );
        }
    }

    /* rtcp-fb: ${codec} nack */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_FB;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_FB_LENGTH;

        written = snprintf( pCurBuffer, remainSize, "%lu %s",
                            payload,
                            SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_VALUE );
        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
            LogError( ( "buffer has no space for rtcp-fb H264 value" ) );
        }
        else
        {
            pTargetAttribute->pAttributeValue = pCurBuffer;
            pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
            *pTargetAttributeCount += 1;

            pCurBuffer += written;
            remainSize -= written;
        }
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        *ppBuffer = pCurBuffer;
        *pBufferLength = remainSize;
    }

    return ret;
}

static SdpControllerResult_t PopulateCodecAttributesH265( SdpControllerMediaDescription_t * pRemoteMediaDescription,
                                                          const Transceiver_t * pTransceiver,
                                                          uint32_t payload,
                                                          uint32_t rtxPayload,
                                                          char ** ppBuffer,
                                                          size_t * pBufferLength,
                                                          SdpControllerMediaDescription_t * pLocalMediaDescription )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    uint8_t isOffer = pRemoteMediaDescription ? 0 : 1;
    SdpControllerAttributes_t * pTargetAttribute = NULL;
    const SdpControllerAttributes_t * pSourceAttribute = NULL;
    uint8_t * pTargetAttributeCount = NULL;
    int written = 0;
    char * pCurBuffer = NULL;
    size_t remainSize = 0;

    pTargetAttributeCount = &pLocalMediaDescription->mediaAttributesCount;
    pCurBuffer = *ppBuffer;
    remainSize = *pBufferLength;

    /* rtpmap */
    pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
    pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTPMAP;
    pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTPMAP_LENGTH;

    written = snprintf( pCurBuffer, remainSize, "%lu %s",
                        payload,
                        SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTPMAP_H265 );
    if( written < 0 )
    {
        ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
        LogError( ( "snprintf return unexpected value %d", written ) );
    }
    else if( written == remainSize )
    {
        ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
        LogError( ( "buffer has no space for rtpmap H265 value" ) );
    }
    else
    {
        pTargetAttribute->pAttributeValue = pCurBuffer;
        pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
        *pTargetAttributeCount += 1;

        pCurBuffer += written;
        remainSize -= written;
    }

    /* rtcp-fb */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_FB;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_FB_LENGTH;

        written = snprintf( pCurBuffer, remainSize, "%lu %s",
                            payload,
                            SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTCP_FB_VALUE );
        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
            LogError( ( "buffer has no space for rtcp-fb H265 value" ) );
        }
        else
        {
            pTargetAttribute->pAttributeValue = pCurBuffer;
            pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
            *pTargetAttributeCount += 1;

            pCurBuffer += written;
            remainSize -= written;
        }
    }

    /* fmtp */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        if( !isOffer )
        {
            /* If creating SDP answer, try find fmtp from the remote description. */
            pSourceAttribute = FindFmtpBasedOnCodec( pRemoteMediaDescription->attributes,
                                                     pRemoteMediaDescription->mediaAttributesCount,
                                                     payload );
        }

        /* Set fmtp only if:
         *   1. It's offer, or
         *   2. It's not offer but we found the corresponding fmtp from remote description. */
        if( isOffer || pSourceAttribute )
        {
            pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
            pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_FMTP;
            pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_FMTP_LENGTH;

            if( isOffer )
            {
                written = snprintf( pCurBuffer, remainSize, "%lu %s",
                                    payload,
                                    SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_FMTP_H265 );
            }
            else
            {
                written = snprintf( pCurBuffer, remainSize, "%.*s",
                                    ( int ) pSourceAttribute->attributeValueLength, pSourceAttribute->pAttributeValue );
            }

            if( written < 0 )
            {
                ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
                LogError( ( "snprintf return unexpected value %d", written ) );
            }
            else if( written == remainSize )
            {
                ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
                LogError( ( "buffer has no space for fmtp" ) );
            }
            else
            {
                pTargetAttribute->pAttributeValue = pCurBuffer;
                pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
                *pTargetAttributeCount += 1;

                pCurBuffer += written;
                remainSize -= written;
            }
        }
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        *ppBuffer = pCurBuffer;
        *pBufferLength = remainSize;
    }

    return ret;
}

static SdpControllerResult_t PopulateCodecAttributes( SdpControllerMediaDescription_t * pRemoteMediaDescription,
                                                      SdpControllerPopulateMediaConfiguration_t populateConfiguration,
                                                      char ** ppBuffer,
                                                      size_t * pBufferLength,
                                                      SdpControllerMediaDescription_t * pLocalMediaDescription )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    const Transceiver_t * pTransceiver = populateConfiguration.pTransceiver;
    uint32_t payload = populateConfiguration.payloadType;
    uint32_t rtxPayload = populateConfiguration.rtxPayloadType;

    if( ( ppBuffer == NULL ) ||
        ( pBufferLength == NULL ) ||
        ( pLocalMediaDescription == NULL ) ||
        ( pTransceiver == NULL ) )
    {
        ret = SDP_CONTROLLER_RESULT_BAD_PARAMETER;
        LogError( ( "Invalid input, ppBuffer: %p, pBufferLength: %p, pLocalMediaDescription: %p, pTransceiver: %p",
                    ppBuffer,
                    pBufferLength,
                    pLocalMediaDescription,
                    pTransceiver ) );
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        if( TRANSCEIVER_IS_CODEC_ENABLED( pTransceiver->codecBitMap, TRANSCEIVER_RTC_CODEC_H264_PROFILE_42E01F_LEVEL_ASYMMETRY_ALLOWED_PACKETIZATION_BIT ) )
        {
            ret = PopulateCodecAttributesH264Profile42E01FLevelAsymmetryAllowedPacketization( pRemoteMediaDescription, pTransceiver, payload, rtxPayload, ppBuffer, pBufferLength, pLocalMediaDescription );
        }
        else if( TRANSCEIVER_IS_CODEC_ENABLED( pTransceiver->codecBitMap, TRANSCEIVER_RTC_CODEC_OPUS_BIT ) )
        {
            ret = PopulateCodecAttributesOpus( pRemoteMediaDescription, pTransceiver, payload, rtxPayload, ppBuffer, pBufferLength, pLocalMediaDescription );
        }
        else if( TRANSCEIVER_IS_CODEC_ENABLED( pTransceiver->codecBitMap, TRANSCEIVER_RTC_CODEC_VP8_BIT ) )
        {
            ret = PopulateCodecAttributesVp8( pRemoteMediaDescription, pTransceiver, payload, rtxPayload, ppBuffer, pBufferLength, pLocalMediaDescription );
        }
        else if( TRANSCEIVER_IS_CODEC_ENABLED( pTransceiver->codecBitMap, TRANSCEIVER_RTC_CODEC_MULAW_BIT ) )
        {
            ret = PopulateCodecAttributesMulaw( pRemoteMediaDescription, pTransceiver, payload, rtxPayload, ppBuffer, pBufferLength, pLocalMediaDescription );
        }
        else if( TRANSCEIVER_IS_CODEC_ENABLED( pTransceiver->codecBitMap, TRANSCEIVER_RTC_CODEC_ALAW_BIT ) )
        {
            ret = PopulateCodecAttributesAlaw( pRemoteMediaDescription, pTransceiver, payload, rtxPayload, ppBuffer, pBufferLength, pLocalMediaDescription );
        }
        else if( TRANSCEIVER_IS_CODEC_ENABLED( pTransceiver->codecBitMap, TRANSCEIVER_RTC_CODEC_H265_BIT ) )
        {
            ret = PopulateCodecAttributesH265( pRemoteMediaDescription, pTransceiver, payload, rtxPayload, ppBuffer, pBufferLength, pLocalMediaDescription );
        }
        else
        {
            /* TODO: Unknown, no matching codec. */
            LogError( ( "Codec is not supported, codec bit map: %x", ( int ) pTransceiver->codecBitMap ) );
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_CODEC_NOT_SUPPORT;
        }
    }

    /* rtcp-fb: ${codec} goog-remb
     * rtcp-fb: ${codec} transport-cc */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        ret = PopulateRtcpFb( payload, populateConfiguration.twccExtId, ppBuffer, pBufferLength, pLocalMediaDescription );
    }

    return ret;
}

static const SdpControllerAttributes_t * FindAttributeName( const SdpControllerAttributes_t * pAttributes,
                                                            size_t attributeCount,
                                                            char * pPattern,
                                                            size_t patternLength )
{
    const SdpControllerAttributes_t * pRet = NULL;
    int i;

    if( ( pAttributes == NULL ) ||
        ( pPattern == NULL ) )
    {
        LogError( ( "Invalid input, pAttributes: %p, pPattern: %p", pAttributes, pPattern ) );
    }
    else
    {
        for( i = 0; i < attributeCount; i++ )
        {
            if( ( pAttributes[i].attributeNameLength == patternLength ) && ( strncmp( pAttributes[i].pAttributeName, pPattern, patternLength ) == 0 ) )
            {
                pRet = &pAttributes[i];
                break;
            }
        }
    }

    return pRet;
}

static const SdpControllerAttributes_t * FindFmtpBasedOnCodec( const SdpControllerAttributes_t * pAttributes,
                                                               size_t attributeCount,
                                                               uint32_t codec )
{
    const SdpControllerAttributes_t * pRet = NULL;
    char codecString[ TRANSCEIVER_CODEC_STRING_MAX_LENGTH + 1 ];
    int i, written;

    if( pAttributes == NULL )
    {
        LogError( ( "Invalid input, pAttributes: %p", pAttributes ) );
    }
    else
    {
        written = snprintf( codecString, TRANSCEIVER_CODEC_STRING_MAX_LENGTH + 1, "%lu", codec );
        if( written < 0 )
        {
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == TRANSCEIVER_CODEC_STRING_MAX_LENGTH + 1 )
        {
            LogError( ( "buffer has no space for codec string" ) );
        }
        else
        {
            for( i = 0; i < attributeCount; i++ )
            {
                if( ( pAttributes[i].attributeNameLength == SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_FMTP_LENGTH ) &&
                    ( strncmp( pAttributes[i].pAttributeName, SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_FMTP, SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_FMTP_LENGTH ) == 0 ) &&
                    ( pAttributes[i].attributeValueLength >= written ) &&
                    ( strncmp( pAttributes[i].pAttributeValue, codecString, written ) == 0 ) )
                {
                    pRet = &pAttributes[i];
                    break;
                }
            }
        }
    }

    return pRet;
}

static SdpControllerAttributes_t * MatchAttributesValuePrefix( SdpControllerAttributes_t * pAttributes,
                                                               size_t attributeNum,
                                                               const char * pPattern,
                                                               size_t patternLength )
{
    SdpControllerAttributes_t * pFound = NULL;
    int i;

    if( ( pAttributes == NULL ) || ( pPattern == NULL ) )
    {
        LogError( ( "Invalid input" ) );
    }
    else
    {
        for( i = 0; i < attributeNum; i++ )
        {
            if( ( ( pAttributes + i )->attributeValueLength >= patternLength ) &&
                ( strncmp( ( pAttributes + i )->pAttributeValue, pPattern, patternLength ) == 0 ) )
            {
                pFound = pAttributes + i;
            }
        }
    }

    return pFound;
}

static int AddSessionAttributeGroup( char * pBuffer,
                                     size_t remainSize,
                                     SdpControllerSdpDescription_t * pLocalSdpDescription,
                                     SdpControllerSdpDescription_t * pRemoteSdpDescription )
{
    int totalWritten = 0;
    int written = 0;
    char * pCurBuffer = pBuffer;
    int i;
    SdpControllerAttributes_t * pRemoteAttribute = NULL;

    if( ( pBuffer == NULL ) ||
        ( pLocalSdpDescription == NULL ) )
    {
        totalWritten = -1;
        LogError( ( "Invalid input" ) );
    }

    /* Append attribute name group. */
    if( totalWritten >= 0 )
    {
        written = snprintf( pCurBuffer, remainSize, "group" );
        if( written < 0 )
        {
            totalWritten = -1;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            totalWritten = -2;
            LogError( ( "buffer has no space for attribute name" ) );
        }
        else
        {
            pLocalSdpDescription->attributes[ pLocalSdpDescription->sessionAttributesCount ].pAttributeName = pCurBuffer;
            pLocalSdpDescription->attributes[ pLocalSdpDescription->sessionAttributesCount ].attributeNameLength = strlen( pCurBuffer );

            pCurBuffer += written;
            remainSize -= written;
            totalWritten += written;
        }
    }

    /* Append attribute value BUNDLE. */
    if( totalWritten >= 0 )
    {
        /* If we have SDP offer, reuse the BUNDLE string from it. */
        if( pRemoteSdpDescription != NULL )
        {
            pRemoteAttribute = MatchAttributesValuePrefix( pRemoteSdpDescription->attributes, SDP_CONTROLLER_MAX_SDP_ATTRIBUTES_COUNT, "BUNDLE", strlen( "BUNDLE" ) );
        }

        if( pRemoteAttribute != NULL )
        {
            written = snprintf( pCurBuffer, remainSize, "%.*s",
                                ( int ) pRemoteAttribute->attributeValueLength,
                                pRemoteAttribute->pAttributeValue );
            if( written < 0 )
            {
                totalWritten = -1;
                LogError( ( "snprintf return unexpected value %d", written ) );
            }
            else if( written == remainSize )
            {
                totalWritten = -2;
                LogError( ( "buffer has no space for attribute value" ) );
            }
            else
            {
                pLocalSdpDescription->attributes[ pLocalSdpDescription->sessionAttributesCount ].pAttributeValue = pCurBuffer;
                pLocalSdpDescription->attributes[ pLocalSdpDescription->sessionAttributesCount ].attributeValueLength = strlen( pCurBuffer );

                totalWritten += written;
            }
        }
    }

    /* Append attribute value BUNDLE if not ready from previous step. */
    if( ( totalWritten >= 0 ) && ( pRemoteAttribute == NULL ) )
    {
        written = snprintf( pCurBuffer, remainSize, "BUNDLE" );
        if( written < 0 )
        {
            totalWritten = -1;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            totalWritten = -2;
            LogError( ( "buffer has no space for attribute value" ) );
        }
        else
        {
            char * pAppendNumber = pCurBuffer + written;
            int offset = 0;

            totalWritten += written;

            for( i = 0; i < pLocalSdpDescription->mediaCount; i++ )
            {
                written = snprintf( pAppendNumber + offset, remainSize - offset, " %d", i );
                if( written < 0 )
                {
                    totalWritten = -1;
                    LogError( ( "snprintf return unexpected value %d", written ) );
                    break;
                }
                else if( written == remainSize - offset )
                {
                    totalWritten = -2;
                    LogError( ( "buffer has no space for attribute value" ) );
                    break;
                }
                else
                {
                    offset += written;
                }
            }

            pLocalSdpDescription->attributes[ pLocalSdpDescription->sessionAttributesCount ].pAttributeValue = pCurBuffer;
            pLocalSdpDescription->attributes[ pLocalSdpDescription->sessionAttributesCount ].attributeValueLength = strlen( pCurBuffer );
            totalWritten += offset;
        }
    }

    /* Update session attribute count. */
    if( totalWritten >= 0 )
    {
        pLocalSdpDescription->sessionAttributesCount++;
    }

    return totalWritten;
}

static int AddSessionAttributeIceOptions( char * pBuffer,
                                          size_t remainSize,
                                          SdpControllerSdpDescription_t * pLocalSdpDescription,
                                          SdpControllerSdpDescription_t * pRemoteSdpDescription )
{
    int totalWritten = 0;
    int written = 0;
    char * pCurBuffer = pBuffer;

    ( void ) pRemoteSdpDescription;

    if( ( pBuffer == NULL ) ||
        ( pLocalSdpDescription == NULL ) )
    {
        totalWritten = -1;
        LogError( ( "Invalid input" ) );
    }

    if( totalWritten >= 0 )
    {
        written = snprintf( pCurBuffer, remainSize, "ice-options" );
        if( written < 0 )
        {
            totalWritten = -1;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            totalWritten = -2;
            LogError( ( "buffer has no space for session attributes" ) );
        }
        else
        {
            pLocalSdpDescription->attributes[ pLocalSdpDescription->sessionAttributesCount ].pAttributeName = pCurBuffer;
            pLocalSdpDescription->attributes[ pLocalSdpDescription->sessionAttributesCount ].attributeNameLength = strlen( pCurBuffer );

            pCurBuffer += written;
            remainSize -= written;
            totalWritten += written;
        }
    }

    if( totalWritten >= 0 )
    {
        written = snprintf( pCurBuffer, remainSize, "trickle" );
        if( written < 0 )
        {
            totalWritten = -1;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            totalWritten = -2;
            LogError( ( "buffer has no space for session attributes" ) );
        }
        else
        {
            pLocalSdpDescription->attributes[ pLocalSdpDescription->sessionAttributesCount ].pAttributeValue = pCurBuffer;
            pLocalSdpDescription->attributes[ pLocalSdpDescription->sessionAttributesCount ].attributeValueLength = strlen( pCurBuffer );

            totalWritten += written;
            pLocalSdpDescription->sessionAttributesCount++;
        }
    }

    return totalWritten;
}

static int AddSessionAttributeMsidSemantic( char * pBuffer,
                                            size_t remainSize,
                                            SdpControllerSdpDescription_t * pLocalSdpDescription,
                                            SdpControllerSdpDescription_t * pRemoteSdpDescription )
{
    int totalWritten = 0;
    int written = 0;
    char * pCurBuffer = pBuffer;

    ( void ) pRemoteSdpDescription;

    if( ( pBuffer == NULL ) ||
        ( pLocalSdpDescription == NULL ) )
    {
        totalWritten = -1;
        LogError( ( "Invalid input" ) );
    }

    if( totalWritten >= 0 )
    {
        written = snprintf( pCurBuffer, remainSize, "msid-semantic" );
        if( written < 0 )
        {
            totalWritten = -1;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            totalWritten = -2;
            LogError( ( "buffer has no space for session attributes" ) );
        }
        else
        {
            pLocalSdpDescription->attributes[ pLocalSdpDescription->sessionAttributesCount ].pAttributeName = pCurBuffer;
            pLocalSdpDescription->attributes[ pLocalSdpDescription->sessionAttributesCount ].attributeNameLength = strlen( pCurBuffer );

            pCurBuffer += written;
            remainSize -= written;
            totalWritten += written;
        }
    }

    if( totalWritten >= 0 )
    {
        written = snprintf( pCurBuffer, remainSize, " WMS myKvsVideoStream" );
        if( written < 0 )
        {
            totalWritten = -1;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            totalWritten = -2;
            LogError( ( "buffer has no space for session attributes" ) );
        }
        else
        {
            pLocalSdpDescription->attributes[ pLocalSdpDescription->sessionAttributesCount ].pAttributeValue = pCurBuffer;
            pLocalSdpDescription->attributes[ pLocalSdpDescription->sessionAttributesCount ].attributeValueLength = strlen( pCurBuffer );

            totalWritten += written;
            pLocalSdpDescription->sessionAttributesCount++;
        }
    }

    return totalWritten;
}

static SdpControllerResult_t PopulateSessionAttributes( SdpControllerSdpDescription_t * pRemoteSdpDescription,
                                                        SdpControllerPopulateSessionConfiguration_t populateConfiguration,
                                                        SdpControllerSdpDescription_t * pLocalSdpDescription,
                                                        char ** ppBuffer,
                                                        size_t * pBufferLength )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    int written;
    size_t remainSize = 0U;
    char * pCurBuffer = NULL;

    if( ( ppBuffer == NULL ) ||
        ( *ppBuffer == NULL ) ||
        ( pBufferLength == NULL ) ||
        ( pLocalSdpDescription == NULL ) )
    {
        LogError( ( "Invalid input, ppBuffer: %p, pBufferLength: %p, pLocalSdpDescription: %p",
                    ppBuffer,
                    pBufferLength,
                    pLocalSdpDescription ) );
        ret = SDP_CONTROLLER_RESULT_BAD_PARAMETER;
    }
    else
    {
        remainSize = *pBufferLength;
        pCurBuffer = *ppBuffer;
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        /* a=group:BINDLE 0 1 ...
         * Note that we need to session media count to populate this value. */
        written = AddSessionAttributeGroup( pCurBuffer, remainSize, pLocalSdpDescription, pRemoteSdpDescription );
        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_ADD_SESSION_ATTRIBUTE_GROUP;
            LogError( ( "Fail to add group to session attribute with return %d", written ) );
        }
        else
        {
            pCurBuffer += written;
            remainSize -= written;
        }
    }

    /* ice-options. */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        /* a=ice-options:trickle */
        if( populateConfiguration.canTrickleIce != 0U )
        {
            written = AddSessionAttributeIceOptions( pCurBuffer, remainSize, pLocalSdpDescription, pRemoteSdpDescription );
            if( written < 0 )
            {
                ret = SDP_CONTROLLER_RESULT_SDP_FAIL_ADD_SESSION_ATTRIBUTE_ICE_OPTIONS;
                LogError( ( "Fail to add ice-options to session attribute with return %d", written ) );
            }
            else
            {
                pCurBuffer += written;
                remainSize -= written;
            }
        }
    }

    /* msid-semantic */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        /* a=msid-semantic: WMS myKvsVideoStream */
        written = AddSessionAttributeMsidSemantic( pCurBuffer, remainSize, pLocalSdpDescription, pRemoteSdpDescription );
        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_ADD_SESSION_ATTRIBUTE_MSID_SEMANTIC;
            LogError( ( "Fail to add msid-semantic to session attribute with return %d", written ) );
        }
        else
        {
            pCurBuffer += written;
            remainSize -= written;
        }
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        *ppBuffer = pCurBuffer;
        *pBufferLength = remainSize;
    }

    return ret;
}

SdpControllerResult_t SdpController_DeserializeSdpMessage( const char * pSdpContent,
                                                           size_t sdpContentLength,
                                                           SdpControllerSdpDescription_t * pSdpDescription )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    SdpResult_t sdpResult = SDP_RESULT_OK;
    StringUtilsResult_t stringResult;
    SdpDeserializerContext_t ctx;
    const char * pValue;
    size_t valueLength;
    uint8_t type;

    if( ( pSdpContent == NULL ) || ( pSdpDescription == NULL ) )
    {
        ret = SDP_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        memset( pSdpDescription, 0, sizeof( SdpControllerSdpDescription_t ) );

        sdpResult = SdpDeserializer_Init( &ctx, pSdpContent, sdpContentLength );
        if( sdpResult != SDP_RESULT_OK )
        {
            LogError( ( "Init SDP deserializer failure, result: %d", sdpResult ) );
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_DESERIALIZER_INIT;
        }
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        while( sdpResult == SDP_RESULT_OK )
        {
            sdpResult = SdpDeserializer_GetNext( &ctx, &type, &pValue, &valueLength );

            if( sdpResult != SDP_RESULT_OK )
            {
                break;
            }
            else if( type == SDP_TYPE_MEDIA )
            {
                pSdpDescription->mediaDescriptions[ pSdpDescription->mediaCount ].pMediaName = pValue;
                pSdpDescription->mediaDescriptions[ pSdpDescription->mediaCount ].mediaNameLength = valueLength;
                pSdpDescription->mediaCount++;
            }
            else if( pSdpDescription->mediaCount != 0 )
            {
                if( type == SDP_TYPE_ATTRIBUTE )
                {
                    ret = ParseMediaAttributes( pSdpDescription, pValue, valueLength );
                    if( ret != SDP_CONTROLLER_RESULT_OK )
                    {
                        LogError( ( "parseMediaAttributes fail, result %d", ret ) );
                        break;
                    }
                }
                else if( type == SDP_TYPE_SESSION_INFO )
                {
                    // Media Title
                    pSdpDescription->mediaDescriptions[ pSdpDescription->mediaCount - 1 ].pMediaTitle = pValue;
                    pSdpDescription->mediaDescriptions[ pSdpDescription->mediaCount - 1 ].mediaTitleLength = valueLength;
                }
                else
                {
                    /* Do nothing. */
                }
            }
            else
            {
                /* No media description before, these attributes belongs to session. */
                if( type == SDP_TYPE_SESSION_NAME )
                {
                    // SDP Session Name
                    pSdpDescription->pSessionName = pValue;
                    pSdpDescription->sessionNameLength = valueLength;
                }
                else if( type == SDP_TYPE_SESSION_INFO )
                {
                    // SDP Session Information
                    pSdpDescription->pSessionInformation = pValue;
                    pSdpDescription->sessionInformationLength = valueLength;
                }
                else if( type == SDP_TYPE_URI )
                {
                    // SDP URI
                    pSdpDescription->pUri = pValue;
                    pSdpDescription->uriLength = valueLength;
                }
                else if( type == SDP_TYPE_EMAIL )
                {
                    // SDP Email Address
                    pSdpDescription->pEmailAddress = pValue;
                    pSdpDescription->emailAddressLength = valueLength;
                }
                else if( type == SDP_TYPE_PHONE )
                {
                    // SDP Phone number
                    pSdpDescription->pPhoneNumber = pValue;
                    pSdpDescription->phoneNumberLength = valueLength;
                }
                else if( type == SDP_TYPE_VERSION )
                {
                    // Version
                    stringResult = StringUtils_ConvertStringToUl( pValue, valueLength, &pSdpDescription->version );
                    if( stringResult != STRING_UTILS_RESULT_OK )
                    {
                        LogError( ( "StringUtils_ConvertStringToUl fail, result %d, converting %.*s to %lu",
                                    stringResult,
                                    ( int ) valueLength, pValue,
                                    pSdpDescription->version ) );
                        ret = SDP_CONTROLLER_RESULT_SDP_INVALID_VERSION;
                        break;
                    }
                }
                else if( type == SDP_TYPE_ATTRIBUTE )
                {
                    ret = ParseSessionAttributes( pSdpDescription, pValue, valueLength );
                    if( ret != SDP_CONTROLLER_RESULT_OK )
                    {
                        LogError( ( "parseSessionAttributes fail, result %d", ret ) );
                        break;
                    }
                }
                else
                {
                    /* Do nothing. */
                }
            }
        }
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        if( pSdpDescription->quickAccess.pFingerprint == NULL )
        {
            /* fingerprint is mandatory in the SDP. */
            LogError( ( "No fingerprint found in the SDP content." ) );
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_NO_FINGERPRINT_FOUND;
        }
        else if( pSdpDescription->quickAccess.pIceUfrag == NULL )
        {
            /* ice-ufrag is mandatory in the SDP. */
            LogError( ( "No ice-ufrag found in the SDP content." ) );
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_NO_ICE_UFRAG_FOUND;
        }
        else if( pSdpDescription->quickAccess.pIcePwd == NULL )
        {
            /* ice-pwd is mandatory in the SDP. */
            LogError( ( "No ice-pwd found in the SDP content." ) );
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_NO_ICE_PWD_FOUND;
        }
        else
        {
            /* Everything is good. */
        }
    }

    return ret;
}

SdpControllerResult_t SdpController_SerializeSdpMessageByDescription( SdpControllerMessageType_t messageType,
                                                                      SdpControllerSdpDescription_t * pSdpDescription,
                                                                      char * pOutputSerializedSdpMessage,
                                                                      size_t * pOutputSerializedSdpMessageLength )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    int written;
    char * pCurrentOutput = pOutputSerializedSdpMessage;
    size_t outputBufferWrittenSize = 0U, remainSize;

    if( ( pSdpDescription == NULL ) || ( pOutputSerializedSdpMessage == NULL ) || ( pOutputSerializedSdpMessageLength == NULL ) )
    {
        LogError( ( "Invalid input, pSdpDescription: %p, pOutputSerializedSdpMessage: %p, pOutputSerializedSdpMessageLength: %p",
                    pSdpDescription,
                    pOutputSerializedSdpMessage,
                    pOutputSerializedSdpMessageLength ) );
        ret = SDP_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        remainSize = *pOutputSerializedSdpMessageLength - outputBufferWrittenSize;
        written = snprintf( pCurrentOutput, remainSize, SDP_CONTROLLER_MESSAGE_TEMPLATE_HEAD, messageType == SDP_CONTROLLER_MESSAGE_TYPE_OFFER ? "offer" : "answer" );

        if( written < 0 )
        {
            LogError( ( "Unexpected behavior, snprintf returns %d", written ) );
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
        }
        else if( written == remainSize )
        {
            LogError( ( "output buffer full" ) );
            ret = SDP_CONTROLLER_RESULT_SDP_CONVERTED_BUFFER_TOO_SMALL;
        }
        else
        {
            pCurrentOutput += written;
            outputBufferWrittenSize += written;
        }
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        remainSize = *pOutputSerializedSdpMessageLength - outputBufferWrittenSize;
        ret = SerializeSdpMessage( pSdpDescription, pCurrentOutput, &remainSize );
        if( ret == SDP_CONTROLLER_RESULT_OK )
        {
            /* remainSize is updated to written length in SerializeSdpMessage. */
            pCurrentOutput += remainSize;
            outputBufferWrittenSize += remainSize;
        }
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        remainSize = *pOutputSerializedSdpMessageLength - outputBufferWrittenSize;
        written = snprintf( pCurrentOutput, remainSize, SDP_CONTROLLER_MESSAGE_TEMPLATE_TAIL );

        if( written < 0 )
        {
            LogError( ( "Unexpected behavior, snprintf returns %d", written ) );
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
        }
        else if( written == remainSize )
        {
            LogError( ( "output buffer full" ) );
            ret = SDP_CONTROLLER_RESULT_SDP_CONVERTED_BUFFER_TOO_SMALL;
        }
        else
        {
            pCurrentOutput += written;
            outputBufferWrittenSize += written;
        }
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        /* Update written length for user. */
        *pOutputSerializedSdpMessageLength = outputBufferWrittenSize;
    }

    return ret;
}

static SdpControllerResult_t PopulateSessionOrigin( char ** ppBuffer,
                                                    size_t * pBufferLength,
                                                    SdpControllerOrigin_t * pOrigin )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;

    pOrigin->pUserName = SDP_CONTROLLER_ORIGIN_DEFAULT_USER_NAME;
    pOrigin->userNameLength = strlen( SDP_CONTROLLER_ORIGIN_DEFAULT_USER_NAME );
    pOrigin->sessionId = rand();
    pOrigin->sessionVersion = SDP_CONTROLLER_ORIGIN_DEFAULT_SESSION_VERSION;
    pOrigin->sdpConnectionInformation.pNetworkType = SDP_CONTROLLER_ORIGIN_DEFAULT_NET_TYPE;
    pOrigin->sdpConnectionInformation.networkTypeLength = strlen( SDP_CONTROLLER_ORIGIN_DEFAULT_NET_TYPE );
    pOrigin->sdpConnectionInformation.pAddressType = SDP_CONTROLLER_ORIGIN_IPV4_TYPE;
    pOrigin->sdpConnectionInformation.addressTypeLength = strlen( SDP_CONTROLLER_ORIGIN_IPV4_TYPE );
    pOrigin->sdpConnectionInformation.pConnectionAddress = SDP_CONTROLLER_ORIGIN_DEFAULT_IP_ADDRESS;
    pOrigin->sdpConnectionInformation.connectionAddressLength = strlen( SDP_CONTROLLER_ORIGIN_DEFAULT_IP_ADDRESS );

    return ret;
}

SdpControllerResult_t SdpController_PopulateSingleMedia( SdpControllerMediaDescription_t * pRemoteMediaDescription,
                                                         SdpControllerPopulateMediaConfiguration_t populateConfiguration,
                                                         SdpControllerMediaDescription_t * pLocalMediaDescription,
                                                         uint32_t currentMediaIdx,
                                                         char ** ppBuffer,
                                                         size_t * pBufferLength,
                                                         TransceiverTrackKind_t trackKind )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;
    char * pCurBuffer = NULL;
    size_t remainSize = 0;
    SdpControllerAttributes_t * pTargetAttribute = NULL;
    uint8_t * pTargetAttributeCount = NULL;
    const SdpControllerAttributes_t * pSourceAttribute = NULL;
    int written = 0;
    int i;

    if( ( pLocalMediaDescription == NULL ) ||
        ( ppBuffer == NULL ) ||
        ( pBufferLength == NULL ) )
    {
        LogError( ( "Invalid input, pLocalMediaDescription: %p, ppBuffer: %p, pBufferLength: %p",
                    pLocalMediaDescription,
                    ppBuffer,
                    pBufferLength ) );
        ret = SDP_CONTROLLER_RESULT_BAD_PARAMETER;
    }
    else if( ( populateConfiguration.pCname == NULL ) ||
             ( populateConfiguration.pLocalFingerprint == NULL ) ||
             ( populateConfiguration.pPassword == NULL ) ||
             ( ( trackKind != TRANSCEIVER_TRACK_KIND_DATA_CHANNEL ) && ( populateConfiguration.pTransceiver == NULL ) ) ||
             ( populateConfiguration.pUserName == NULL ) )
    {
        LogError( ( "Invalid input, pCname: %p, pLocalFingerprint: %p, pPassword: %p, pTransceiver: %p, pUserName: %p",
                    populateConfiguration.pCname,
                    populateConfiguration.pLocalFingerprint,
                    populateConfiguration.pPassword,
                    populateConfiguration.pTransceiver,
                    populateConfiguration.pUserName ) );
        ret = SDP_CONTROLLER_RESULT_BAD_PARAMETER;
    }
    else
    {
        /* Empty else marker. */
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        memset( pLocalMediaDescription, 0, sizeof( SdpControllerMediaDescription_t ) );
        pCurBuffer = *ppBuffer;
        remainSize = *pBufferLength;
        pTargetAttributeCount = &pLocalMediaDescription->mediaAttributesCount;
    }

    /* Set media name. */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        /* We support only one payload type, so only one payload type printed in media name. */
        switch( trackKind )
        {
            case TRANSCEIVER_TRACK_KIND_VIDEO:
            {
                if( populateConfiguration.rtxPayloadType == 0 )
                {
                    written = snprintf( pCurBuffer, remainSize, "video 9 UDP/TLS/RTP/SAVPF %lu", populateConfiguration.payloadType );
                }
                else
                {
                    written = snprintf( pCurBuffer, remainSize, "video 9 UDP/TLS/RTP/SAVPF %lu %lu", populateConfiguration.payloadType, populateConfiguration.rtxPayloadType );
                }
                break;
            }
            case TRANSCEIVER_TRACK_KIND_AUDIO:
            {
                if( populateConfiguration.rtxPayloadType == 0 )
                {
                    written = snprintf( pCurBuffer, remainSize, "audio 9 UDP/TLS/RTP/SAVPF %lu", populateConfiguration.payloadType );
                }
                else
                {
                    written = snprintf( pCurBuffer, remainSize, "audio 9 UDP/TLS/RTP/SAVPF %lu %lu", populateConfiguration.payloadType, populateConfiguration.rtxPayloadType );
                }
                break;
            }
            case TRANSCEIVER_TRACK_KIND_DATA_CHANNEL:
            {
                written = snprintf( pCurBuffer, remainSize, "%s", SDP_CONTROLLER_DATA_CHANNEL_ATTRIBUTE_NAME_MEDIA_NAME );
                break;
            }
            default:
            {
                LogError( ( "Unknown track kind: %d",
                            trackKind ) );
                ret = SDP_CONTROLLER_RESULT_BAD_PARAMETER;
                written = -1;
                break;
            }
        }

        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
            LogError( ( "buffer has no space for media name" ) );
        }
        else
        {
            pLocalMediaDescription->pMediaName = pCurBuffer;
            pLocalMediaDescription->mediaNameLength = strlen( pCurBuffer );

            pCurBuffer += written;
            remainSize -= written;
        }
    }

    /* Set media title and connection information. */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        pLocalMediaDescription->pMediaTitle = NULL;
        pLocalMediaDescription->mediaTitleLength = 0;
        pLocalMediaDescription->connectionInformation.pNetworkType = SDP_CONTROLLER_ORIGIN_DEFAULT_NET_TYPE;
        pLocalMediaDescription->connectionInformation.networkTypeLength = strlen( SDP_CONTROLLER_ORIGIN_DEFAULT_NET_TYPE );
        pLocalMediaDescription->connectionInformation.pAddressType = SDP_CONTROLLER_ORIGIN_IPV4_TYPE;
        pLocalMediaDescription->connectionInformation.addressTypeLength = strlen( SDP_CONTROLLER_ORIGIN_IPV4_TYPE );
        pLocalMediaDescription->connectionInformation.pConnectionAddress = SDP_CONTROLLER_ORIGIN_DEFAULT_IP_ADDRESS;
        pLocalMediaDescription->connectionInformation.connectionAddressLength = strlen( SDP_CONTROLLER_ORIGIN_DEFAULT_IP_ADDRESS );
    }

    /* msid */
    if( ( ret == SDP_CONTROLLER_RESULT_OK ) && ( trackKind != TRANSCEIVER_TRACK_KIND_DATA_CHANNEL ) )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_MSID;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_MSID_LENGTH;

        if( populateConfiguration.rtxPayloadType == 0 )
        {
            written = snprintf( pCurBuffer, remainSize, "%.*s %.*s",
                                ( int ) populateConfiguration.pTransceiver->streamIdLength, populateConfiguration.pTransceiver->streamId,
                                ( int ) populateConfiguration.pTransceiver->trackIdLength, populateConfiguration.pTransceiver->trackId );
        }
        else
        {
            written = snprintf( pCurBuffer, remainSize, "%.*s %.*sRTX",
                                ( int ) populateConfiguration.pTransceiver->streamIdLength, populateConfiguration.pTransceiver->streamId,
                                ( int ) populateConfiguration.pTransceiver->trackIdLength, populateConfiguration.pTransceiver->trackId );
        }

        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
            LogError( ( "buffer has no space for msid" ) );
        }
        else
        {
            pTargetAttribute->pAttributeValue = pCurBuffer;
            pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
            *pTargetAttributeCount += 1;

            pCurBuffer += written;
            remainSize -= written;
        }
    }

    /* For RTX: ssrc-group */
    if( ( ret == SDP_CONTROLLER_RESULT_OK ) && ( trackKind != TRANSCEIVER_TRACK_KIND_DATA_CHANNEL ) && ( populateConfiguration.rtxPayloadType != 0 ) )
    {
        /* msid */
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC_GROUP;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SSRC_GROUP_LENGTH;

        written = snprintf( pCurBuffer, remainSize, "FID %lu %lu",
                            populateConfiguration.pTransceiver->ssrc,
                            populateConfiguration.pTransceiver->rtxSsrc );

        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
            LogError( ( "buffer has no space for RTX ssrc-group" ) );
        }
        else
        {
            pTargetAttribute->pAttributeValue = pCurBuffer;
            pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
            *pTargetAttributeCount += 1;

            pCurBuffer += written;
            remainSize -= written;
        }
    }

    /* ssrc */
    if( ( ret == SDP_CONTROLLER_RESULT_OK ) && ( trackKind != TRANSCEIVER_TRACK_KIND_DATA_CHANNEL ) )
    {
        ret = PopulateTransceiverSsrc( &pCurBuffer, &remainSize, pLocalMediaDescription, populateConfiguration.pTransceiver, populateConfiguration.pCname, populateConfiguration.cnameLength, populateConfiguration.rtxPayloadType == 0 ? 0 : 1 );
    }

    /* rtcp, ice-ufrag, ice-pwd */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_LENGTH;
        pTargetAttribute->pAttributeValue = SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTCP;
        pTargetAttribute->attributeValueLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_RTCP_LENGTH;
        *pTargetAttributeCount += 1;

        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_UFRAG;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_UFRAG_LENGTH;
        pTargetAttribute->pAttributeValue = populateConfiguration.pUserName;
        pTargetAttribute->attributeValueLength = populateConfiguration.userNameLength;
        *pTargetAttributeCount += 1;

        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_PWD;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_PWD_LENGTH;
        pTargetAttribute->pAttributeValue = populateConfiguration.pPassword;
        pTargetAttribute->attributeValueLength = populateConfiguration.passwordLength;
        *pTargetAttributeCount += 1;
    }

    /* ice-options:trickle */
    if( ( ret == SDP_CONTROLLER_RESULT_OK ) &&
        ( populateConfiguration.canTrickleIce != 0 ) )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_OPTION;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_ICE_OPTION_LENGTH;
        pTargetAttribute->pAttributeValue = SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_ICE_OPTION;
        pTargetAttribute->attributeValueLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_VALUE_ICE_OPTION_LENGTH;
        *pTargetAttributeCount += 1;
    }

    /* Local fingerprint. */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_FINGERPRINT;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_FINGERPRINT_LENGTH;

        written = snprintf( pCurBuffer, remainSize, "sha-256 %.*s",
                            ( int ) populateConfiguration.localFingerprintLength, populateConfiguration.pLocalFingerprint );

        if( written < 0 )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
            LogError( ( "snprintf return unexpected value %d", written ) );
        }
        else if( written == remainSize )
        {
            ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
            LogError( ( "buffer has no space for msid" ) );
        }
        else
        {
            pTargetAttribute->pAttributeValue = pCurBuffer;
            pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
            *pTargetAttributeCount += 1;

            pCurBuffer += written;
            remainSize -= written;
        }
    }

    /* setup. */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SETUP;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SETUP_LENGTH;

        if( populateConfiguration.isOffer )
        {
            pTargetAttribute->pAttributeValue = SDP_CONTROLLER_MEDIA_DTLS_ROLE_ACTPASS;
            pTargetAttribute->attributeValueLength = SDP_CONTROLLER_MEDIA_DTLS_ROLE_ACTPASS_LENGTH;
        }
        else
        {
            pTargetAttribute->pAttributeValue = SDP_CONTROLLER_MEDIA_DTLS_ROLE_ACTIVE;
            pTargetAttribute->attributeValueLength = SDP_CONTROLLER_MEDIA_DTLS_ROLE_ACTIVE_LENGTH;
        }

        *pTargetAttributeCount += 1;
    }

    /* mid */
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        /* When populating offer, there is no remote description for reference. */
        if( populateConfiguration.isOffer == 0 )
        {
            /* Try to match the mid number in the remote media description. */
            pSourceAttribute = FindAttributeName( pRemoteMediaDescription->attributes,
                                                  pRemoteMediaDescription->mediaAttributesCount,
                                                  SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_MID,
                                                  SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_MID_LENGTH );
        }
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_MID;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_MID_LENGTH;

        if( pSourceAttribute != NULL )
        {
            pTargetAttribute->pAttributeValue = pSourceAttribute->pAttributeValue;
            pTargetAttribute->attributeValueLength = pSourceAttribute->attributeValueLength;
            *pTargetAttributeCount += 1;
        }
        else
        {
            written = snprintf( pCurBuffer, remainSize, "%lu",
                                currentMediaIdx );

            if( written < 0 )
            {
                ret = SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF;
                LogError( ( "snprintf return unexpected value %d", written ) );
            }
            else if( written == remainSize )
            {
                ret = SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL;
                LogError( ( "buffer has no space for mid" ) );
            }
            else
            {
                pTargetAttribute->pAttributeValue = pCurBuffer;
                pTargetAttribute->attributeValueLength = strlen( pCurBuffer );
                *pTargetAttributeCount += 1;

                pCurBuffer += written;
                remainSize -= written;
            }
        }
    }

    /* send/recv */
    if( ( ret == SDP_CONTROLLER_RESULT_OK ) && ( trackKind != TRANSCEIVER_TRACK_KIND_DATA_CHANNEL ) )
    {
        TransceiverDirection_t targetDirection = TRANSCEIVER_TRACK_DIRECTION_UNKNOWN;

        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeValue = NULL;
        pTargetAttribute->attributeValueLength = 0;

        /* Find target direction. */
        if( populateConfiguration.isOffer != 0 )
        {
            targetDirection = populateConfiguration.pTransceiver->direction;
        }
        else
        {
            // in case of a missing m-line, we respond with the same m-line but direction set to inactive
            if( populateConfiguration.pTransceiver->direction == TRANSCEIVER_TRACK_DIRECTION_INACTIVE )
            {
                targetDirection = TRANSCEIVER_TRACK_DIRECTION_INACTIVE;
            }
            else
            {
                for( i = 0; i < pRemoteMediaDescription->mediaAttributesCount; i++ )
                {
                    if( ( pRemoteMediaDescription->attributes[i].attributeNameLength == SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SENDRECV_LENGTH ) &&
                        ( strncmp( pRemoteMediaDescription->attributes[i].pAttributeName, SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SENDRECV, SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SENDRECV_LENGTH ) == 0 ) )
                    {
                        targetDirection = TRANSCEIVER_TRACK_DIRECTION_SENDRECV;
                        break;
                    }
                    else if( ( pRemoteMediaDescription->attributes[i].attributeNameLength == SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SENDONLY_LENGTH ) &&
                             ( strncmp( pRemoteMediaDescription->attributes[i].pAttributeName, SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SENDONLY, SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SENDONLY_LENGTH ) == 0 ) )
                    {
                        targetDirection = TRANSCEIVER_TRACK_DIRECTION_RECVONLY;
                        break;
                    }
                    else if( ( pRemoteMediaDescription->attributes[i].attributeNameLength == SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RECVONLY_LENGTH ) &&
                             ( strncmp( pRemoteMediaDescription->attributes[i].pAttributeName, SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RECVONLY, SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RECVONLY_LENGTH ) == 0 ) )
                    {
                        targetDirection = TRANSCEIVER_TRACK_DIRECTION_SENDONLY;
                        break;
                    }
                    else if( ( pRemoteMediaDescription->attributes[i].attributeNameLength == SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_INACTIVE_LENGTH ) &&
                             ( strncmp( pRemoteMediaDescription->attributes[i].pAttributeName, SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_INACTIVE, SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_INACTIVE_LENGTH ) == 0 ) )
                    {
                        targetDirection = TRANSCEIVER_TRACK_DIRECTION_INACTIVE;
                        break;
                    }
                }
            }
        }

        switch( targetDirection )
        {
            case TRANSCEIVER_TRACK_DIRECTION_SENDRECV:
                pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SENDRECV;
                pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SENDRECV_LENGTH;
                break;
            case TRANSCEIVER_TRACK_DIRECTION_SENDONLY:
                pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SENDONLY;
                pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_SENDONLY_LENGTH;
                break;
            case TRANSCEIVER_TRACK_DIRECTION_RECVONLY:
                pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RECVONLY;
                pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RECVONLY_LENGTH;
                break;
            case TRANSCEIVER_TRACK_DIRECTION_INACTIVE:
            default:
                // https://www.w3.org/TR/webrtc/#dom-rtcrtpopulateConfiguration.pTransceiverdirection
                LogWarn( ( "Incorrect/no transceiver direction set...this attribute will be set to inactive, target: %d", targetDirection ) );
                pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_INACTIVE;
                pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_INACTIVE_LENGTH;
        }

        *pTargetAttributeCount += 1;
    }

    /* rtcp-mux, rtcp-rsize */
    if( ( ret == SDP_CONTROLLER_RESULT_OK ) && ( trackKind != TRANSCEIVER_TRACK_KIND_DATA_CHANNEL ) )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_MUX;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_MUX_LENGTH;
        pTargetAttribute->pAttributeValue = NULL;
        pTargetAttribute->attributeValueLength = 0;

        *pTargetAttributeCount += 1;

        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_RSIZE;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_MEDIA_ATTRIBUTE_NAME_RTCP_RSIZE_LENGTH;
        pTargetAttribute->pAttributeValue = NULL;
        pTargetAttribute->attributeValueLength = 0;

        *pTargetAttributeCount += 1;
    }

    /* Popupate codec relevant attributes. */
    if( ( ret == SDP_CONTROLLER_RESULT_OK ) && ( trackKind != TRANSCEIVER_TRACK_KIND_DATA_CHANNEL ) )
    {
        ret = PopulateCodecAttributes( pRemoteMediaDescription,
                                       populateConfiguration,
                                       &pCurBuffer,
                                       &remainSize,
                                       pLocalMediaDescription );
    }

    /* sctp port */
    if( ( ret == SDP_CONTROLLER_RESULT_OK ) && ( trackKind == TRANSCEIVER_TRACK_KIND_DATA_CHANNEL ) )
    {
        pTargetAttribute = &pLocalMediaDescription->attributes[ *pTargetAttributeCount ];
        pTargetAttribute->pAttributeName = SDP_CONTROLLER_DATA_CHANNEL_ATTRIBUTE_NAME_SCTP_PORT;
        pTargetAttribute->attributeNameLength = SDP_CONTROLLER_DATA_CHANNEL_ATTRIBUTE_NAME_SCTP_PORT_LENGTH;
        pTargetAttribute->pAttributeValue = SDP_CONTROLLER_DATA_CHANNEL_ATTRIBUTE_VALUE_SCTP_PORT;
        pTargetAttribute->attributeValueLength = SDP_CONTROLLER_DATA_CHANNEL_ATTRIBUTE_VALUE_SCTP_PORT_LENGTH;
        *pTargetAttributeCount += 1;
    }
    
    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        *ppBuffer = pCurBuffer;
        *pBufferLength = remainSize;
    }

    return ret;
}

SdpControllerResult_t SdpController_PopulateSessionDescription( SdpControllerSdpDescription_t * pRemoteSessionDescription,
                                                                SdpControllerPopulateSessionConfiguration_t populateConfiguration,
                                                                SdpControllerSdpDescription_t * pLocalSessionDescription,
                                                                char ** ppBuffer,
                                                                size_t * pBufferLength )
{
    SdpControllerResult_t ret = SDP_CONTROLLER_RESULT_OK;

    if( ( pLocalSessionDescription == NULL ) ||
        ( ppBuffer == NULL ) ||
        ( pBufferLength == NULL ) )
    {
        LogError( ( "Invalid input, pLocalSessionDescription: %p, ppBuffer: %p, pBufferLength: %p",
                    pLocalSessionDescription,
                    ppBuffer,
                    pBufferLength ) );
        ret = SDP_CONTROLLER_RESULT_BAD_PARAMETER;
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        /* Session version. */
        pLocalSessionDescription->version = 0U;

        /* Session origin. */
        ret = PopulateSessionOrigin( ppBuffer, pBufferLength, &pLocalSessionDescription->origin );
    }

    if( ret == SDP_CONTROLLER_RESULT_OK )
    {
        /* Session name. */
        pLocalSessionDescription->pSessionName = SDP_CONTROLLER_ORIGIN_DEFAULT_SESSION_NAME;
        pLocalSessionDescription->sessionNameLength = strlen( SDP_CONTROLLER_ORIGIN_DEFAULT_SESSION_NAME );

        /* Session timing description. */
        pLocalSessionDescription->timingDescription.startTime = populateConfiguration.timingDescription.startTime;
        pLocalSessionDescription->timingDescription.stopTime = populateConfiguration.timingDescription.stopTime;

        ret = PopulateSessionAttributes( pRemoteSessionDescription, populateConfiguration, pLocalSessionDescription, ppBuffer, pBufferLength );
    }

    return ret;
}
