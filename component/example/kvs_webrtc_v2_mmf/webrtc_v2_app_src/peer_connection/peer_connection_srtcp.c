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
#include "peer_connection.h"
#include "peer_connection_srtcp.h"
#include "peer_connection_srtp.h"
#include "peer_connection_rolling_buffer.h"

/* API includes. */
#include "rtp_api.h"
#include "rtcp_api.h"
#include "rtcp_twcc_manager.h"
#include "ice_controller.h"
#include "networking_utils.h"

/*-----------------------------------------------------------*/

/* https://datatracker.ietf.org/doc/html/rfc3550#section-6.4 */
#define PEER_CONNECTION_RTCP_RECEIVER_REPORT_RECEPTION_REPORT_NUM    ( 31 )
#define PEER_CONNECTION_SRTCP_NACK_MAX_SEQ_NUM                       ( 128 )
#define PEER_CONNECTION_SRTCP_REMB_MAX_SSRC_NUM                      ( 255 )

/* https://datatracker.ietf.org/doc/html/rfc3550#section-6.4.1 */
#define PEER_CONNECTION_SRTCP_DLSR_TIMESCALE                         65536

/* https://tools.ietf.org/html/rfc3550#section-4 */
/* In some fields where a more compact representation is */
/*   appropriate, only the middle 32 bits are used; that is, the low 16 */
/*   bits of the integer part and the high 16 bits of the fractional part. */
#define PEER_CONNECTION_SRTCP_MID_NTP( currentTimeNTP )    ( uint32_t ) ( ( currentTimeNTP >> 16U ) & 0xffffffffULL )

/*-----------------------------------------------------------*/

static PeerConnectionResult_t PeerConnectionSrtcp_MatchRemoteBySsrc( PeerConnectionSession_t * pSession,
                                                                     uint32_t ssrc )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( pSession == NULL )
    {
        LogError( ( "Invalid input, pSession: %p", pSession ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( ( ssrc != pSession->rtpConfig.remoteAudioSsrc ) && ( ssrc != pSession->rtpConfig.remoteVideoSsrc ) )
        {
            LogWarn( ( "No transceiver for SSRC: %lu", ssrc ) );
            ret = PEER_CONNECTION_RESULT_UNKNOWN_SSRC;
        }
    }

    return ret;
}

static PeerConnectionResult_t ResendSrtpPacket( PeerConnectionSession_t * pSession,
                                                const Transceiver_t * pTransceiver,
                                                uint16_t rtpSeq,
                                                uint32_t ssrc )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    PeerConnectionSrtpSender_t * pSrtpSender = NULL;
    uint8_t isSenderLocked = 0;
    PeerConnectionRollingBufferPacket_t * pRollingBufferPacket = NULL;
    IceControllerResult_t resultIceController;
    uint8_t bufferAfterEncrypt = 1;
    uint8_t srtpBuffer[ PEER_CONNECTION_SRTP_RTP_PACKET_MAX_LENGTH ];
    uint8_t * pSrtpPacket = NULL;
    size_t srtpPacketLength = 0;
    uint32_t payloadType;
    uint16_t * pRtpSeq = NULL;
    uint16_t * pOsn = NULL;

    if( ( pSession == NULL ) || ( pTransceiver == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pTransceiver: %p", pSession, pTransceiver ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        pRtpSeq = &rtpSeq;

        if( pTransceiver->trackKind == TRANSCEIVER_TRACK_KIND_VIDEO )
        {
            pSrtpSender = &pSession->videoSrtpSender;
            payloadType = pSession->rtpConfig.videoCodecPayload;

            if( ( pSession->rtpConfig.videoCodecRtxPayload != 0 ) &&
                ( pSession->rtpConfig.videoCodecRtxPayload != pSession->rtpConfig.videoCodecPayload ) )
            {
                /* Use RTX payload type, sequence number and ssrc for re-transmission. */
                bufferAfterEncrypt = 0;
                payloadType = pSession->rtpConfig.videoCodecRtxPayload;
                pRtpSeq = &pSession->rtpConfig.videoRtxSequenceNumber;
                ssrc = pTransceiver->rtxSsrc;
            }
        }
        else
        {
            pSrtpSender = &pSession->audioSrtpSender;
            payloadType = pSession->rtpConfig.audioCodecPayload;

            if( ( pSession->rtpConfig.audioCodecRtxPayload != 0 ) &&
                ( pSession->rtpConfig.audioCodecRtxPayload != pSession->rtpConfig.audioCodecPayload ) )
            {
                /* Use RTX payload type, sequence number and ssrc for re-transmission. */
                bufferAfterEncrypt = 0;
                payloadType = pSession->rtpConfig.audioCodecRtxPayload;
                pRtpSeq = &pSession->rtpConfig.audioRtxSequenceNumber;
                ssrc = pTransceiver->rtxSsrc;
            }
        }

        /* Lock sender. */
        if( xSemaphoreTake( pSrtpSender->senderMutex, portMAX_DELAY ) == pdTRUE )
        {
            isSenderLocked = 1;
        }
        else
        {
            LogError( ( "Fail to take sender mutex" ) );
            ret = PEER_CONNECTION_RESULT_FAIL_TAKE_SENDER_MUTEX;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        ret = PeerConnectionRollingBuffer_SearchRtpSequenceBuffer( &pSrtpSender->txRollingBuffer,
                                                                   rtpSeq,
                                                                   &pRollingBufferPacket );

        if( ( ret != PEER_CONNECTION_RESULT_OK ) || ( pRollingBufferPacket == NULL ) )
        {
            LogWarn( ( "Fail to find target buffer, seq: %u", rtpSeq ) );
            ret = PEER_CONNECTION_RESULT_FAIL_RTP_PACKET_NOT_FOUND;
        }
        else
        {
            LogDebug( ( "Found target buffer, pRollingBufferPacket: %p, packetBufferLength: %u, pPacketBuffer: %p",
                        pRollingBufferPacket,
                        pRollingBufferPacket->packetBufferLength,
                        pRollingBufferPacket->pPacketBuffer ) );
            LogDebug( ( "Found target buffer, sequence in buffer: %u, target sequence: %u",
                        pRollingBufferPacket->rtpPacket.header.sequenceNumber, rtpSeq ) );
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( bufferAfterEncrypt == 0 )
        {
            /* Don't reset the header as re-using the setting from write frame.
             * Update sequence, SSRC, payload type and OSN for RTX packet. */
            pRollingBufferPacket->rtpPacket.header.sequenceNumber = ( *pRtpSeq )++;
            pRollingBufferPacket->rtpPacket.header.ssrc = ssrc;
            pRollingBufferPacket->rtpPacket.header.payloadType = payloadType;

            /* Follow RTX format to add OSN(original RTP sequence number) at the very beginning of payload.
             * Note that we reserve PEER_CONNECTION_SRTP_RTX_WRITE_RESERVED_BYTES at the beginning of buffer at write frame. */
            pOsn = ( uint16_t * ) pRollingBufferPacket->pPacketBuffer;
            *pOsn = htons( rtpSeq );
            pRollingBufferPacket->rtpPacket.payloadLength = pRollingBufferPacket->packetBufferLength + 2;
            pRollingBufferPacket->rtpPacket.pPayload = pRollingBufferPacket->pPacketBuffer;

            pSrtpPacket = srtpBuffer;
            srtpPacketLength = PEER_CONNECTION_SRTP_RTP_PACKET_MAX_LENGTH;

            /* PeerConnectionSrtp_ConstructSrtpPacket() serializes RTP packet and encrypt it. */
            ret = PeerConnectionSrtp_ConstructSrtpPacket( pSession,
                                                          &pRollingBufferPacket->rtpPacket,
                                                          pSrtpPacket,
                                                          &srtpPacketLength );
        }
        else
        {
            pSrtpPacket = pRollingBufferPacket->pPacketBuffer;
            srtpPacketLength = pRollingBufferPacket->packetBufferLength;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        resultIceController = IceController_SendToRemotePeer( &pSession->iceControllerContext,
                                                              pSrtpPacket,
                                                              srtpPacketLength );

        if( resultIceController != ICE_CONTROLLER_RESULT_OK )
        {
            LogWarn( ( "Fail to re-send RTP packet, ret: %d, seq: %u, SSRC: 0x%lx", resultIceController, rtpSeq, ssrc ) );
            ret = PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_RESEND_RTP_PACKET;
        }
        else
        {
            LogDebug( ( "Re-send RTP successfully, RTP seq: %u, SSRC: 0x%lx", rtpSeq, ssrc ) );
        }
    }

    if( isSenderLocked )
    {
        xSemaphoreGive( pSrtpSender->senderMutex );
    }

    return ret;
}

static PeerConnectionResult_t OnRtcpFirEvent( PeerConnectionSession_t * pSession,
                                              RtcpPacket_t * pRtcpPacket )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    RtcpResult_t resultRtcp;
    RtcpFirPacket_t firPacket;
    const Transceiver_t * pTransceiver = NULL;

    if( ( pSession == NULL ) || ( pRtcpPacket == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pRtcpPacket: %p", pSession, pRtcpPacket ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        memset( &firPacket,
                0,
                sizeof( RtcpFirPacket_t ) );
        resultRtcp = Rtcp_ParseFirPacket( &pSession->pCtx->rtcpContext,
                                          pRtcpPacket,
                                          &firPacket );

        if( resultRtcp != RTCP_RESULT_OK )
        {
            LogError( ( "Fail to parse RTCP FIR packet, result: %d", resultRtcp ) );
            ret = PEER_CONNECTION_RESULT_FAIL_RTCP_PARSE_FIR;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        ret = PeerConnection_MatchTransceiverBySsrc( pSession,
                                                     firPacket.senderSsrc,
                                                     &pTransceiver );
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Increase the FIR Counter as per our application design. */

        /* Initiate a callback to transmit an intra-picture to achieve resynchronization. */
    }
    else if( ret == PEER_CONNECTION_RESULT_UNKNOWN_SSRC )
    {
        LogWarn( ( "Received FIR for non existing ssrc: %lu", firPacket.senderSsrc ) );
    }
    else
    {
        /* Do Nothing */
    }

    return ret;
}

static PeerConnectionResult_t OnRtcpNackEvent( PeerConnectionSession_t * pSession,
                                               RtcpPacket_t * pRtcpPacket )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    RtcpResult_t resultRtcp;
    RtcpNackPacket_t nackPacket;
    const Transceiver_t * pTransceiver = NULL;
    uint16_t seqNumList[ PEER_CONNECTION_SRTCP_NACK_MAX_SEQ_NUM ];
    int i;

    if( ( pSession == NULL ) || ( pRtcpPacket == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pRtcpPacket: %p", pSession, pRtcpPacket ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        memset( &nackPacket, 0, sizeof( RtcpNackPacket_t ) );
        memset( &seqNumList[ 0 ], 0, sizeof( uint16_t ) * PEER_CONNECTION_SRTCP_NACK_MAX_SEQ_NUM );
        nackPacket.pSeqNumList = seqNumList;
        nackPacket.seqNumListLength = PEER_CONNECTION_SRTCP_NACK_MAX_SEQ_NUM;
        resultRtcp = Rtcp_ParseNackPacket( &pSession->pCtx->rtcpContext,
                                           pRtcpPacket,
                                           &nackPacket );

        if( resultRtcp != RTCP_RESULT_OK )
        {
            LogError( ( "Fail to parse RTCP NACK packet, result: %d", resultRtcp ) );
            ret = PEER_CONNECTION_RESULT_FAIL_RTCP_PARSE_NACK;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        ret = PeerConnection_MatchTransceiverBySsrc( pSession,
                                                     nackPacket.mediaSourceSsrc,
                                                     &pTransceiver );
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        for( i = 0; i < nackPacket.seqNumListLength; i++ )
        {
            /* Retransmit matching sequence number one by one. */
            ret = ResendSrtpPacket( pSession,
                                    pTransceiver,
                                    nackPacket.pSeqNumList[ i ],
                                    nackPacket.senderSsrc );

            if( ret != PEER_CONNECTION_RESULT_OK )
            {
                break;
            }
        }
    }

    return ret;
}
#if ENABLE_TWCC_SUPPORT
    static PeerConnectionResult_t OnRtcpTwccEvent( PeerConnectionSession_t * pSession,
                                                   RtcpPacket_t * pRtcpPacket )
    {
        PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
        RtcpResult_t resultRtcp;
        RtcpTwccManagerResult_t resultRtcpTwccManager = RTCP_TWCC_MANAGER_RESULT_OK;
        RtcpTwccPacket_t twccPacket;
        TwccPacketInfo_t * pTwccPacketInfo;
        TwccBandwidthInfo_t twccBandwidthInfo;
        PacketArrivalInfo_t packetArrivalInfo[ PEER_CONNECTION_RTCP_TWCC_MAX_ARRAY ];
        int i;


        if( ( pSession == NULL ) || ( pRtcpPacket == NULL ) )
        {
            LogError( ( "Invalid input, pSession: %p, pRtcpPacket: %p", pSession, pRtcpPacket ) );
            ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
        }

        if( ret == PEER_CONNECTION_RESULT_OK )
        {
            memset( &twccPacket,
                    0,
                    sizeof( RtcpTwccPacket_t ) );
            memset( &packetArrivalInfo[ 0 ],
                    0,
                    sizeof( PacketArrivalInfo_t ) * PEER_CONNECTION_RTCP_TWCC_MAX_ARRAY );
            twccPacket.pArrivalInfoList = packetArrivalInfo;
            twccPacket.arrivalInfoListLength = PEER_CONNECTION_RTCP_TWCC_MAX_ARRAY;
            resultRtcp = Rtcp_ParseTwccPacket( &pSession->pCtx->rtcpContext,
                                               pRtcpPacket,
                                               &twccPacket );

            if( resultRtcp != RTCP_RESULT_OK )
            {
                LogError( ( "Fail to parse RTCP TWCC packet, result: %d", resultRtcp ) );
                ret = PEER_CONNECTION_RESULT_FAIL_RTCP_PARSE_TWCC;
            }
        }

        if( ret == PEER_CONNECTION_RESULT_OK )
        {
            for( i = 0; i < twccPacket.arrivalInfoListLength; i++ )
            {
                /* Checks if the seq number already exists. */
                resultRtcpTwccManager = RtcpTwccManager_FindPacketInfo( &pSession->pCtx->rtcpTwccManager,
                                                                        twccPacket.pArrivalInfoList[ i ].seqNum,
                                                                        &pTwccPacketInfo );

                if( resultRtcpTwccManager == RTCP_TWCC_MANAGER_RESULT_OK )
                {
                    pTwccPacketInfo->localSentTime = twccPacket.pArrivalInfoList[ i ].remoteArrivalTime;
                }
            }
        }

        if( ret == PEER_CONNECTION_RESULT_OK )
        {
            resultRtcpTwccManager = RtcpTwccManager_HandleTwccPacket( &pSession->pCtx->rtcpTwccManager,
                                                                      &twccPacket,
                                                                      &twccBandwidthInfo );

            if( resultRtcpTwccManager != RTCP_TWCC_MANAGER_RESULT_OK )
            {
                LogError( ( "Fail to handle RTCP TWCC packet, result: %d", resultRtcpTwccManager ) );
                ret = PEER_CONNECTION_RESULT_FAIL_RTCP_HANDLE_TWCC;
            }
        }

        if( ret == PEER_CONNECTION_RESULT_OK )
        {
            if( ( twccBandwidthInfo.duration > 0 ) && ( pSession->pCtx->onBandwidthEstimationCallback != NULL ) )
            {
                /* Call the bandwidth estimation callback */
                pSession->pCtx->onBandwidthEstimationCallback( pSession->pCtx->pOnBandwidthEstimationCallbackContext,
                                                               &twccBandwidthInfo );
            }

            LogDebug( ( "TWCC Bandwidth Info : SentBytes - %llu, ReceivedBytes - %llu, SentPackets - %llu, ReceivedPackets - %llu, Duration - %lld", twccBandwidthInfo.sentBytes, twccBandwidthInfo.receivedBytes, twccBandwidthInfo.sentPackets, twccBandwidthInfo.receivedPackets, twccBandwidthInfo.duration ) );
        }

        return ret;
    }
#endif /* if ENABLE_TWCC_SUPPORT */

static PeerConnectionResult_t OnRtcpPliEvent( PeerConnectionSession_t * pSession,
                                              RtcpPacket_t * pRtcpPacket )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    RtcpResult_t resultRtcp;
    RtcpPliPacket_t pliPacket;
    const Transceiver_t * pTransceiver = NULL;

    if( ( pSession == NULL ) || ( pRtcpPacket == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pRtcpPacket: %p", pSession, pRtcpPacket ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        memset( &pliPacket,
                0,
                sizeof( RtcpPliPacket_t ) );
        resultRtcp = Rtcp_ParsePliPacket( &pSession->pCtx->rtcpContext,
                                          pRtcpPacket,
                                          &pliPacket );

        if( resultRtcp != RTCP_RESULT_OK )
        {
            LogError( ( "Fail to parse RTCP PLI packet, result: %d", resultRtcp ) );
            ret = PEER_CONNECTION_RESULT_FAIL_RTCP_PARSE_PLI;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        ret = PeerConnection_MatchTransceiverBySsrc( pSession,
                                                     pliPacket.mediaSourceSsrc,
                                                     &pTransceiver );
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Call the PLI callback if registered */
        if( pSession->onPictureLossIndicationCallback != NULL )
        {
            pSession->onPictureLossIndicationCallback( pSession->pPictureLossIndicationUserContext,
                                                       &pliPacket );
        }
    }
    else if( ret == PEER_CONNECTION_RESULT_UNKNOWN_SSRC )
    {
        LogError( ( "Received PLI for non existing ssrc: %lu", pliPacket.mediaSourceSsrc ) );
    }
    else
    {
        /* Do Nothing */
    }

    return ret;
}

/* TODO handle SLI packet https://tools.ietf.org/html/rfc4585#section-6.3.2 */
static PeerConnectionResult_t OnRtcpSliEvent( PeerConnectionSession_t * pSession,
                                              RtcpPacket_t * pRtcpPacket )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    RtcpResult_t resultRtcp;
    RtcpSliPacket_t sliPacket;
    const Transceiver_t * pTransceiver = NULL;

    if( ( pSession == NULL ) || ( pRtcpPacket == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pRtcpPacket: %p", pSession, pRtcpPacket ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        memset( &sliPacket,
                0,
                sizeof( RtcpSliPacket_t ) );
        resultRtcp = Rtcp_ParseSliPacket( &pSession->pCtx->rtcpContext,
                                          pRtcpPacket,
                                          &sliPacket );

        if( resultRtcp != RTCP_RESULT_OK )
        {
            LogError( ( "Fail to parse RTCP SLI packet, result: %d", resultRtcp ) );
            ret = PEER_CONNECTION_RESULT_FAIL_RTCP_PARSE_SLI;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        ret = PeerConnection_MatchTransceiverBySsrc( pSession,
                                                     sliPacket.mediaSourceSsrc,
                                                     &pTransceiver );
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Increase the Sli Counter as per our application design. */
    }
    else if( ret == PEER_CONNECTION_RESULT_UNKNOWN_SSRC )
    {
        LogWarn( ( "Received SLI for non existing ssrc: %lu", sliPacket.mediaSourceSsrc ) );
        ret = PEER_CONNECTION_RESULT_OK;
    }
    else
    {
        /* Do Nothing */
    }

    return ret;
}

static PeerConnectionResult_t OnRtcpRembEvent( PeerConnectionSession_t * pSession,
                                               RtcpPacket_t * pRtcpPacket )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    RtcpResult_t resultRtcp;
    RtcpRembPacket_t rembPacket;
    const Transceiver_t * pTransceiver = NULL;
    uint32_t ssrcList[ PEER_CONNECTION_SRTCP_REMB_MAX_SSRC_NUM ];
    int i;

    if( ( pSession == NULL ) || ( pRtcpPacket == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pRtcpPacket: %p", pSession, pRtcpPacket ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        memset( &rembPacket, 0, sizeof( RtcpRembPacket_t ) );
        memset( &ssrcList[ 0 ], 0, sizeof( uint32_t ) * PEER_CONNECTION_SRTCP_REMB_MAX_SSRC_NUM );
        rembPacket.pSsrcList = ssrcList;
        rembPacket.ssrcListLength = PEER_CONNECTION_SRTCP_REMB_MAX_SSRC_NUM;
        resultRtcp = Rtcp_ParseRembPacket( &pSession->pCtx->rtcpContext,
                                           pRtcpPacket,
                                           &rembPacket );

        if( resultRtcp != RTCP_RESULT_OK )
        {
            LogError( ( "Fail to parse RTCP REMB packet, result: %d", resultRtcp ) );
            ret = PEER_CONNECTION_RESULT_FAIL_RTCP_PARSE_REMB;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        for( i = 0; i < rembPacket.ssrcListLength; i++ )
        {
            ret = PeerConnection_MatchTransceiverBySsrc( pSession,
                                                         rembPacket.pSsrcList[ i ],
                                                         &pTransceiver );

            if( ret == PEER_CONNECTION_RESULT_OK )
            {
                /* Initiate a callback to update the estimated maximum bitrate to adjust sending rate based on receiver's feedback. */
            }
            else if( ret == PEER_CONNECTION_RESULT_UNKNOWN_SSRC )
            {
                LogWarn( ( "Received REMB for non existing ssrc: %lu", rembPacket.pSsrcList[ i ] ) );
            }
            else
            {
                /* Do Nothing */
            }
        }
    }

    return ret;
}

/* TODO better sender report handling https://tools.ietf.org/html/rfc3550#section-6.4.1 */
static PeerConnectionResult_t OnRtcpSenderReportEvent( PeerConnectionSession_t * pSession,
                                                       RtcpPacket_t * pRtcpPacket )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    RtcpResult_t resultRtcp;
    RtcpSenderReport_t senderReport;

    if( ( pSession == NULL ) || ( pRtcpPacket == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pRtcpPacket: %p", pSession, pRtcpPacket ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        memset( &senderReport,
                0,
                sizeof( RtcpSenderReport_t ) );
        resultRtcp = Rtcp_ParseSenderReport( &pSession->pCtx->rtcpContext,
                                             pRtcpPacket,
                                             &senderReport );

        if( resultRtcp != RTCP_RESULT_OK )
        {
            LogError( ( "Fail to parse RTCP SENDER REPORT packet, result: %d", resultRtcp ) );
            ret = PEER_CONNECTION_RESULT_FAIL_RTCP_PARSE_SENDER_REPORT;
        }
        else if( pRtcpPacket->payloadLength != RTCP_SENDER_REPORT_MIN_PAYLOAD_LENGTH )
        {
            /* TODO: handle sender report containing receiver report blocks */
            LogWarn( ( "Received Sender report with containing receiver report blocks, not supported yet." ) );

            /* Based on the RFC 3550 document, SR (Sender Report) receives report blocks when a sender is also receiving RTP data from other participants in the session.
             * Since Currently the Master isn't parsing RTP data packets from multiple sources (multiple SSRCs) since the last report.
             * In case this log gets printed we can encounter that even when the funcionality is not added, we are getting report blocks, so something is wrong. */
        }
        else
        {
            /* Do Nothing */
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        ret = PeerConnectionSrtcp_MatchRemoteBySsrc( pSession,
                                                     senderReport.senderSsrc );

        LogVerbose( ( "RTCP_PACKET_SENDER_REPORT, SSRC: %lu, NTP Time %llu RTP Time: %lu, PacketCount: %lu, OctetCount: %lu",
                      senderReport.senderSsrc,
                      senderReport.senderInfo.ntpTime,
                      senderReport.senderInfo.rtpTime,
                      senderReport.senderInfo.packetCount,
                      senderReport.senderInfo.octetCount ) );

        if( ret == PEER_CONNECTION_RESULT_UNKNOWN_SSRC )
        {
            LogWarn( ( "Received sender report for non existing ssrc: %lu", senderReport.senderSsrc ) );
            ret = PEER_CONNECTION_RESULT_OK;
        }
    }

    return ret;
}

static PeerConnectionResult_t OnRtcpReceiverReportEvent( PeerConnectionSession_t * pSession,
                                                         RtcpPacket_t * pRtcpPacket )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    RtcpResult_t resultRtcp;
    RtcpReceiverReport_t receiverReport;
    RtcpReceptionReport_t receptionReport[ PEER_CONNECTION_RTCP_RECEIVER_REPORT_RECEPTION_REPORT_NUM ];
    const Transceiver_t * pTransceiver = NULL;
    uint32_t roundTripPropagationDelay = 0;
    uint64_t currentTimeNTP = 0;
    int i;

    if( ( pSession == NULL ) || ( pRtcpPacket == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pRtcpPacket: %p", pSession, pRtcpPacket ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        memset( &receiverReport,
                0,
                sizeof( RtcpReceiverReport_t ) );
        memset( &receptionReport[ 0 ],
                0,
                sizeof( RtcpReceptionReport_t ) * PEER_CONNECTION_RTCP_RECEIVER_REPORT_RECEPTION_REPORT_NUM );

        receiverReport.pReceptionReports = receptionReport;
        receiverReport.numReceptionReports = PEER_CONNECTION_RTCP_RECEIVER_REPORT_RECEPTION_REPORT_NUM;

        resultRtcp = Rtcp_ParseReceiverReport( &pSession->pCtx->rtcpContext,
                                               pRtcpPacket,
                                               &receiverReport );

        if( resultRtcp != RTCP_RESULT_OK )
        {
            LogError( ( "Fail to parse RTCP RECEIVER REPORT packet, result: %d", resultRtcp ) );
            ret = PEER_CONNECTION_RESULT_FAIL_RTCP_PARSE_RECEIVER_REPORT;
        }

        /* https://tools.ietf.org/html/rfc3550#section-6.4.2 */
        /* One Report Block for Audio SSRC and One Report Block for Video SSRC */
        if( receiverReport.numReceptionReports > 2 )
        {
            /* TODO: handle multiple receiver report blocks */
            LogWarn( ( "Received receiver report with multiple reception reports, not supported yet." ) );

            /* Since Currently the Master isn't parsing RTP data packets from multiple sources (multiple SSRCs) since the last report.
             * Each reception report block provides statistics about the data received from a particular source. */
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        for( i = 0; i < receiverReport.numReceptionReports; i++ )
        {
            ret = PeerConnection_MatchTransceiverBySsrc( pSession,
                                                         receiverReport.pReceptionReports[ 0 ].sourceSsrc,
                                                         &pTransceiver );

            LogDebug( ( "RTCP_PACKET_TYPE_RECEIVER_REPORT, sender SSRC: %lu, source SSRC: %lu, fraction loss: %u, cumulative loss: %lu, highest seq: %lu, jit: %lu, lsr: %lu, dlsr: %lu",
                        receiverReport.senderSsrc,
                        receiverReport.pReceptionReports[ 0 ].sourceSsrc,
                        receiverReport.pReceptionReports[ 0 ].fractionLost,
                        receiverReport.pReceptionReports[ 0 ].cumulativePacketsLost,
                        receiverReport.pReceptionReports[ 0 ].extendedHighestSeqNumReceived,
                        receiverReport.pReceptionReports[ 0 ].interArrivalJitter,
                        receiverReport.pReceptionReports[ 0 ].lastSR,
                        receiverReport.pReceptionReports[ 0 ].delaySinceLastSR ) );

            if( ret == PEER_CONNECTION_RESULT_UNKNOWN_SSRC )
            {
                LogWarn( ( "Received receiver report for non existing ssrc: %lu", receiverReport.pReceptionReports[ 0 ].sourceSsrc ) );
                ret = PEER_CONNECTION_RESULT_OK;
                continue;
            }

            if( ( ret == PEER_CONNECTION_RESULT_OK ) && ( receiverReport.pReceptionReports[ i ].lastSR != 0 ) )
            {
                /* https://tools.ietf.org/html/rfc3550#section-6.4.1 */
                /*      Source SSRC_n can compute the round-trip propagation delay to */
                /*      SSRC_r by recording the time A when this reception report block is */
                /*      received.  It calculates the total round-trip time A-LSR using the */
                /*      last SR timestamp (LSR) field, and then subtracting this field to */
                /*      leave the round-trip propagation delay as (A - LSR - DLSR). */
                currentTimeNTP = NetworkingUtils_GetNTPTimeFromUnixTimeUs( NetworkingUtils_GetCurrentTimeUs( NULL ) );
                currentTimeNTP = PEER_CONNECTION_SRTCP_MID_NTP( currentTimeNTP );
                roundTripPropagationDelay = currentTimeNTP - receiverReport.pReceptionReports[ 0 ].lastSR - receiverReport.pReceptionReports[ 0 ].delaySinceLastSR;
                roundTripPropagationDelay = ( roundTripPropagationDelay * 1000 ) / PEER_CONNECTION_SRTCP_DLSR_TIMESCALE;                             /* The Round Trip Propogation Delay is in ms unit. */

                if( pTransceiver->trackKind == TRANSCEIVER_TRACK_KIND_AUDIO )
                {
                    LogVerbose( ( "RTCP_PACKET_TYPE_RECEIVER_REPORT Round Trip Propagation Delay for Audio : %lu ms", roundTripPropagationDelay ) );
                }
                else if( pTransceiver->trackKind == TRANSCEIVER_TRACK_KIND_VIDEO )
                {
                    LogVerbose( ( "RTCP_PACKET_TYPE_RECEIVER_REPORT Round Trip Propagation Delay for Video : %lu ms", roundTripPropagationDelay ) );
                }
            }
        }
    }

    /* Update stats if like reportsReceived counter, roundTripPropagationDelay, fraction Lost , roundTripTimeMeasurements counter  */

    return ret;
}

PeerConnectionResult_t PeerConnectionSrtcp_ConstructSenderReportPacket( PeerConnectionSession_t * pSession,
                                                                        RtcpSenderReport_t * pSenderReport,
                                                                        uint8_t * pOutputSrtcpPacket,
                                                                        size_t * pOutputSrtcpPacketLength )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    RtcpResult_t resultRtcp;
    size_t rtcpBufferLength;
    srtp_err_status_t errorStatus;
    uint8_t isLocked = 0U;

    if( ( pSession == NULL ) ||
        ( pSenderReport == NULL ) ||
        ( pOutputSrtcpPacket == NULL ) ||
        ( pOutputSrtcpPacketLength == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pSenderReport: %p, pOutputSrtcpPacket: %p, pOutputSrtcpPacketLength: %p",
                    pSession,
                    pSenderReport,
                    pOutputSrtcpPacket,
                    pOutputSrtcpPacketLength ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    /* Get buffer from sender for serializing RTCP packet */
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        rtcpBufferLength = *pOutputSrtcpPacketLength;
    }

    /* Contruct RTP packet for each payload buffer. */
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        resultRtcp = Rtcp_SerializeSenderReport( &pSession->pCtx->rtcpContext,
                                                 pSenderReport,
                                                 pOutputSrtcpPacket,
                                                 &rtcpBufferLength );

        if( resultRtcp != RTCP_RESULT_OK )
        {
            LogError( ( "Fail to serialize RTCP Sender Report, result: %d", resultRtcp ) );
            ret = PEER_CONNECTION_RESULT_FAIL_RTCP_SERIALIZE_SENDER_REPORT;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( xSemaphoreTake( pSession->srtpSessionMutex,
                            portMAX_DELAY ) == pdTRUE )
        {
            isLocked = 1U;
        }
        else
        {
            LogError( ( "Fail to take SRTP session mutex to construct SRTCP packet." ) );
            ret = PEER_CONNECTION_RESULT_FAIL_TAKE_SRTP_MUTEX;
        }
    }

    /* Encrypt it by SRTP. */
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( pSession->srtpTransmitSession != NULL )
        {
            errorStatus = srtp_protect_rtcp( pSession->srtpTransmitSession,
                                             pOutputSrtcpPacket,
                                             rtcpBufferLength,
                                             pOutputSrtcpPacket,
                                             pOutputSrtcpPacketLength,
                                             0 );
            if( errorStatus != srtp_err_status_ok )
            {
                LogError( ( "Fail to encrypt Tx SRTCP packet, errorStatus: %d", errorStatus ) );
                ret = PEER_CONNECTION_RESULT_FAIL_ENCRYPT_SRTP_RTCP_PACKET;
            }
        }
        else
        {
            LogWarn( ( "SRTP session has been freed before encrypting." ) );
            ret = PEER_CONNECTION_RESULT_FAIL_ENCRYPT_SRTP_RTCP_PACKET;
        }
    }

    if( isLocked != 0U )
    {
        xSemaphoreGive( pSession->srtpSessionMutex );
    }

    return ret;
}

PeerConnectionResult_t PeerConnectionSrtp_HandleSrtcpPacket( PeerConnectionSession_t * pSession,
                                                             uint8_t * pBuffer,
                                                             size_t bufferLength )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    srtp_err_status_t errorStatus;
    static uint8_t rtcpBuffer[ PEER_CONNECTION_SRTP_RTP_PACKET_MAX_LENGTH ];
    size_t rtcpBufferLength = PEER_CONNECTION_SRTP_RTP_PACKET_MAX_LENGTH;
    RtcpResult_t resultRtcp;
    RtcpPacket_t rtcpPacket;
    const uint8_t * currentPacket = rtcpBuffer;
    size_t remainingLength = 0;
    size_t currentPacketLength = 0;
    uint8_t isLocked = 0U;

    if( ( pSession == NULL ) || ( pBuffer == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pBuffer: %p", pSession, pBuffer ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( xSemaphoreTake( pSession->srtpSessionMutex,
                            portMAX_DELAY ) == pdTRUE )
        {
            isLocked = 1U;
        }
        else
        {
            LogError( ( "Fail to take SRTP session mutex to construct SRTCP packet." ) );
            ret = PEER_CONNECTION_RESULT_FAIL_TAKE_SRTP_MUTEX;
        }
    }

    /* Decrypt it by SRTP. */
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( pSession->srtpReceiveSession != NULL )
        {
            errorStatus = srtp_unprotect_rtcp( pSession->srtpReceiveSession,
                                               pBuffer,
                                               bufferLength,
                                               rtcpBuffer,
                                               &rtcpBufferLength );
            if( errorStatus != srtp_err_status_ok )
            {
                LogError( ( "Fail to decrypt Rx SRTCP packet, errorStatus: %d", errorStatus ) );
                ret = PEER_CONNECTION_RESULT_FAIL_DECRYPT_SRTP_RTP_PACKET;
            }
            else
            {
                remainingLength = rtcpBufferLength;
                LogVerbose( ( "Decrypt SRTCP packet successfully, decrypted length: %u", rtcpBufferLength ) );
            }
        }
        else
        {
            LogWarn( ( "SRTP session has been freed before decrypting." ) );
            ret = PEER_CONNECTION_RESULT_FAIL_DECRYPT_SRTP_RTP_PACKET;
        }
    }

    if( isLocked != 0U )
    {
        xSemaphoreGive( pSession->srtpSessionMutex );
    }

    while( ( remainingLength >= RTCP_HEADER_LENGTH ) &&
           ( ret == PEER_CONNECTION_RESULT_OK ) )
    {
        resultRtcp = Rtcp_DeserializePacket( &pSession->pCtx->rtcpContext,
                                             currentPacket,
                                             remainingLength,
                                             &rtcpPacket );

        if( resultRtcp != RTCP_RESULT_OK )
        {
            LogError( ( "Fail to deserialize RTCP packet, result: %d", resultRtcp ) );
            ret = PEER_CONNECTION_RESULT_FAIL_RTCP_DESERIALIZE;
        }

        /* Calculate the total length of current RTCP packet (including header) */
        currentPacketLength = rtcpPacket.payloadLength + RTCP_HEADER_LENGTH;

        if( ret == PEER_CONNECTION_RESULT_OK )
        {
            LogDebug( ( "Receiving RTCP type: %d", rtcpPacket.header.packetType ) );

            switch( rtcpPacket.header.packetType )
            {
                case RTCP_PACKET_FIR:
                    ret = OnRtcpFirEvent( pSession,
                                          &rtcpPacket );
                    break;

                case RTCP_PACKET_TRANSPORT_FEEDBACK_NACK:
                    ret = OnRtcpNackEvent( pSession,
                                           &rtcpPacket );
                    break;

                case RTCP_PACKET_TRANSPORT_FEEDBACK_TWCC:
                    #if ENABLE_TWCC_SUPPORT
                        ret = OnRtcpTwccEvent( pSession,
                                               &rtcpPacket );
                    #endif
                    break;

                case RTCP_PACKET_PAYLOAD_FEEDBACK_PLI:
                    ret = OnRtcpPliEvent( pSession,
                                          &rtcpPacket );
                    break;

                case RTCP_PACKET_PAYLOAD_FEEDBACK_SLI:
                    ret = OnRtcpSliEvent( pSession,
                                          &rtcpPacket );
                    break;

                case RTCP_PACKET_PAYLOAD_FEEDBACK_REMB:
                    ret = OnRtcpRembEvent( pSession,
                                           &rtcpPacket );
                    break;

                case RTCP_PACKET_SENDER_REPORT:
                    ret = OnRtcpSenderReportEvent( pSession,
                                                   &rtcpPacket );
                    break;

                case RTCP_PACKET_RECEIVER_REPORT:
                    ret = OnRtcpReceiverReportEvent( pSession,
                                                     &rtcpPacket );
                    break;

                case RTCP_PACKET_SOURCE_DESCRIPTION:
                    LogInfo( ( "unhandled RTCP packet type: RTCP_PACKET_SOURCE_DESCRIPTION" ) );
                    break;

                default:
                    LogWarn( ( "unhandled packet type %d", rtcpPacket.header.packetType ) );
                    break;
            }

            /*  Move to next RTCP packet in compound packet */
            currentPacket += currentPacketLength;
            remainingLength -= currentPacketLength;
        }
    }

    return ret;
}