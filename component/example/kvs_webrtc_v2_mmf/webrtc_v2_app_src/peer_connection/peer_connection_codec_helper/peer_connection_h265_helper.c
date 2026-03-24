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

#include "include/peer_connection_codec_helper.h"
#include "h265_packetizer.h"
#include "h265_depacketizer.h"

PeerConnectionResult_t PeerConnectionH265Helper_GetH265PacketProperty( PeerConnectionJitterBufferPacket_t * pPacket,
                                                                       uint8_t * pIsStartPacket )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    H265Result_t resultH265;
    uint32_t properties = 0;

    if( ( pPacket == NULL ) ||
        ( pIsStartPacket == NULL ) )
    {
        LogError( ( "Invalid input, pPacket: %p, pIsStartPacket: %p", pPacket, pIsStartPacket ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        resultH265 = H265Depacketizer_GetPacketProperties( pPacket->pPacketBuffer,
                                                           pPacket->packetBufferLength,
                                                           &properties );
        if( resultH265 != H265_RESULT_OK )
        {
            LogError( ( "Fail to get h265 packet properties, result: %d", resultH265 ) );
            ret = PEER_CONNECTION_RESULT_FAIL_DEPACKETIZER_GET_PROPERTIES;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        *pIsStartPacket = 0U;
        if( ( properties & H265_PACKET_PROPERTY_START_PACKET ) != 0 )
        {
            *pIsStartPacket = 1U;
        }
    }

    return ret;
}

PeerConnectionResult_t PeerConnectionH265Helper_FillFrameH265( PeerConnectionJitterBuffer_t * pJitterBuffer,
                                                               uint16_t rtpSeqStart,
                                                               uint16_t rtpSeqEnd,
                                                               uint8_t * pOutBuffer,
                                                               size_t * pOutBufferLength,
                                                               uint32_t * pRtpTimestamp )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    uint16_t i, index;
    PeerConnectionJitterBufferPacket_t * pPacket;
    H265Result_t resultH265;
    H265DepacketizerContext_t h265DepacketizerContext;
    H265Packet_t h265Packets[ PEER_CONNECTION_JITTER_BUFFER_MAX_PACKETS_NUM_IN_A_FRAME ];
    H265Packet_t h265Packet;
    H265Frame_t frame;
    uint32_t rtpTimestamp = 0;

    if( ( pJitterBuffer == NULL ) ||
        ( pOutBuffer == NULL ) ||
        ( pOutBufferLength == NULL ) ||
        ( pRtpTimestamp == NULL ) )
    {
        LogError( ( "Invalid input, pJitterBuffer: %p, pOutBuffer: %p, pOutBufferLength: %p, pRtpTimestamp: %p", pJitterBuffer, pOutBuffer, pOutBufferLength, pRtpTimestamp ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        resultH265 = H265Depacketizer_Init( &h265DepacketizerContext,
                                            h265Packets,
                                            PEER_CONNECTION_JITTER_BUFFER_MAX_PACKETS_NUM_IN_A_FRAME );
        if( resultH265 != H265_RESULT_OK )
        {
            LogError( ( "Fail to initialize H265 depacketizer, result: %d", resultH265 ) );
            ret = PEER_CONNECTION_RESULT_FAIL_DEPACKETIZER_INIT;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        for( i = rtpSeqStart; i != rtpSeqEnd + 1; i++ )
        {
            index = PEER_CONNECTION_JITTER_BUFFER_WRAP( i,
                                                        PEER_CONNECTION_JITTER_BUFFER_MAX_ENTRY_NUM );
            pPacket = &pJitterBuffer->rtpPackets[ index ];
            h265Packet.pPacketData = pPacket->pPacketBuffer;
            h265Packet.packetDataLength = pPacket->packetBufferLength;
            rtpTimestamp = pPacket->rtpTimestamp;
            LogDebug( ( "Adding packet seq: %u, length: %u, timestamp: %lu", i, h265Packet.packetDataLength, rtpTimestamp ) );

            resultH265 = H265Depacketizer_AddPacket( &h265DepacketizerContext,
                                                     &h265Packet );
            if( resultH265 != H265_RESULT_OK )
            {
                LogError( ( "Fail to add h265 depacketizer packet, result: %d", resultH265 ) );
                ret = PEER_CONNECTION_RESULT_FAIL_DEPACKETIZER_ADD_PACKET;
                break;
            }
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        frame.pFrameData = pOutBuffer;
        frame.frameDataLength = *pOutBufferLength;
        resultH265 = H265Depacketizer_GetFrame( &h265DepacketizerContext,
                                                &frame );
        if( resultH265 != H265_RESULT_OK )
        {
            LogError( ( "Fail to get h265 depacketizer frame, result: %d", resultH265 ) );
            ret = PEER_CONNECTION_RESULT_FAIL_DEPACKETIZER_GET_FRAME;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        *pOutBufferLength = frame.frameDataLength;
        *pRtpTimestamp = rtpTimestamp;
    }

    return ret;
}

PeerConnectionResult_t PeerConnectionH265Helper_WriteH265Frame( PeerConnectionSession_t * pSession,
                                                                Transceiver_t * pTransceiver,
                                                                const PeerConnectionFrame_t * pFrame )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    H265PacketizerContext_t h265PacketizerContext;
    H265Result_t resulth265;
    H265Packet_t packeth265;
    uint8_t rtpBuffer[ PEER_CONNECTION_SRTP_RTP_PACKET_MAX_LENGTH ];
    PeerConnectionRollingBufferPacket_t * pRollingBufferPacket = NULL;
    uint8_t * pSrtpPacket = NULL;
    size_t srtpPacketLength = 0;
    H265Nalu_t nalusArray[ PEER_CONNECTION_SRTP_H265_MAX_NALUS_IN_A_FRAME ];
    H265Frame_t h265Frame;
    PeerConnectionSrtpSender_t * pSrtpSender = NULL;
    uint8_t isLocked = 0;
    uint8_t bufferAfterEncrypt = 1;
    IceControllerResult_t resultIceController;
    uint16_t * pRtpSeq = NULL;
    uint32_t payloadType;
    uint32_t * pSsrc = NULL;
    uint32_t packetSent = 0;
    uint32_t bytesSent = 0;
    uint32_t randomRtpTimeoffset = 0;    // TODO : Spec required random rtp time offset ( current implementation of KVS SDK )
    #if ENABLE_TWCC_SUPPORT
    /* Add TWCC packet tracking */
    TwccPacketInfo_t packetInfo;
    #endif /* ENABLE_TWCC_SUPPORT */

    if( ( pSession == NULL ) ||
        ( pTransceiver == NULL ) ||
        ( pFrame == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pTransceiver: %p, pFrame: %p",
                    pSession, pTransceiver, pFrame ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else if( pTransceiver->trackKind != TRANSCEIVER_TRACK_KIND_VIDEO )
    {
        LogError( ( "Invalid track kind." ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else
    {
        /* Empty else marker. */
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        resulth265 = H265Packetizer_Init( &h265PacketizerContext,
                                          nalusArray,
                                          PEER_CONNECTION_SRTP_H265_MAX_NALUS_IN_A_FRAME );
        if( resulth265 != H265_RESULT_OK )
        {
            LogError( ( "Fail to init h265 packetizer, result: %d", resulth265 ) );
            ret = PEER_CONNECTION_RESULT_FAIL_PACKETIZER_INIT;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        h265Frame.pFrameData = pFrame->pData;
        h265Frame.frameDataLength = pFrame->dataLength;
        resulth265 = H265Packetizer_AddFrame( &h265PacketizerContext,
                                              &h265Frame );
        if( resulth265 != H265_RESULT_OK )
        {
            LogError( ( "Fail to add frame in  h265 packetizer, result: %d", resulth265 ) );
            ret = PEER_CONNECTION_RESULT_FAIL_PACKETIZER_ADD_FRAME;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        pSsrc = &pTransceiver->ssrc;
        pSrtpSender = &pSession->videoSrtpSender;
        pRtpSeq = &pSession->rtpConfig.videoSequenceNumber;
        payloadType = pSession->rtpConfig.videoCodecPayload;
        if( ( pSession->rtpConfig.videoCodecRtxPayload != 0 ) &&
            ( pSession->rtpConfig.videoCodecRtxPayload != pSession->rtpConfig.videoCodecPayload ) )
        {
            bufferAfterEncrypt = 0;
        }

        if( xSemaphoreTake( pSrtpSender->senderMutex,
                            portMAX_DELAY ) == pdTRUE )
        {
            isLocked = 1;
        }
        else
        {
            LogError( ( "Fail to take sender mutex" ) );
            ret = PEER_CONNECTION_RESULT_FAIL_TAKE_SENDER_MUTEX;
        }
    }
    while( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Get buffer from sender for later use.
         * PeerConnectionRollingBuffer_GetRtpSequenceBuffer() returns the buffer with its size.
         * If the bufferAfterEncrypt = 0, we store only RTP payload to the buffer.
         * If the bufferAfterEncrypt = 1, we store the encrypted SRTP packet to the buffer. */
        pRollingBufferPacket = NULL;
        ret = PeerConnectionRollingBuffer_GetRtpSequenceBuffer( &pSrtpSender->txRollingBuffer,
                                                                *pRtpSeq,
                                                                &pRollingBufferPacket );
        if( ( ret != PEER_CONNECTION_RESULT_OK ) ||
            ( pRollingBufferPacket == NULL ) )
        {
            LogWarn( ( "Fail to get RTP buffer for seq: %u", *pRtpSeq ) );
            break;
        }

        /* Get each NALU payload then serialize SRTP packet. */
        if( bufferAfterEncrypt == 0 )
        {
            packeth265.pPacketData = pRollingBufferPacket->pPacketBuffer + PEER_CONNECTION_SRTP_RTX_WRITE_RESERVED_BYTES;
            packeth265.packetDataLength = pRollingBufferPacket->packetBufferLength - PEER_CONNECTION_SRTP_RTX_WRITE_RESERVED_BYTES;

            /* Using local buffer for SRTP packet, use the entire packet length. */
            pSrtpPacket = rtpBuffer;
            srtpPacketLength = PEER_CONNECTION_SRTP_RTP_PACKET_MAX_LENGTH;
        }
        else
        {
            /* Using local buffer for RTP payload only, set RTP payload length. */
            packeth265.pPacketData = rtpBuffer;
            packeth265.packetDataLength = PEER_CONNECTION_SRTP_RTP_PAYLOAD_MAX_LENGTH;

            pSrtpPacket = pRollingBufferPacket->pPacketBuffer;
            srtpPacketLength = pRollingBufferPacket->packetBufferLength;
        }

        resulth265 = H265Packetizer_GetPacket( &h265PacketizerContext,
                                               &packeth265 );

        if( resulth265 == H265_RESULT_NO_MORE_PACKETS )
        {
            PeerConnectionRollingBuffer_DiscardRtpSequenceBuffer( &pSrtpSender->txRollingBuffer,
                                                                  pRollingBufferPacket );
            /* Early break because no packet available. */
            break;
        }
        else if( resulth265 == H265_RESULT_OK )
        {
            /* Prepare RTP packet for each payload buffer. */
            memset( &pRollingBufferPacket->rtpPacket, 0, sizeof( RtpPacket_t ) );
            pRollingBufferPacket->rtpPacket.header.payloadType = payloadType;
            pRollingBufferPacket->rtpPacket.header.sequenceNumber = *pRtpSeq;
            pRollingBufferPacket->rtpPacket.header.ssrc = *pSsrc;
            if( h265PacketizerContext.naluCount == 0 )
            {
                /* This is the last packet, set the marker. */
                pRollingBufferPacket->rtpPacket.header.flags |= RTP_HEADER_FLAG_MARKER;
            }

            pRollingBufferPacket->rtpPacket.header.csrcCount = 0;
            pRollingBufferPacket->rtpPacket.header.pCsrc = NULL;
            pRollingBufferPacket->rtpPacket.header.timestamp = PEER_CONNECTION_SRTP_CONVERT_TIME_US_TO_RTP_TIMESTAMP( PEER_CONNECTION_SRTP_VIDEO_CLOCKRATE,
                                                                                                                      pFrame->presentationUs );

            if( pSession->rtpConfig.twccId > 0 )
            {
                pRollingBufferPacket->rtpPacket.header.flags |= RTP_HEADER_FLAG_EXTENSION;
                pRollingBufferPacket->rtpPacket.header.extension.extensionProfile = PEER_CONNECTION_SRTP_TWCC_EXT_PROFILE;
                pRollingBufferPacket->rtpPacket.header.extension.extensionPayloadLength = 1;
                pRollingBufferPacket->twccExtensionPayload = PEER_CONNECTION_SRTP_GET_TWCC_PAYLOAD( pSession->rtpConfig.twccId,
                                                                                                    pSession->rtpConfig.twccSequence );
                pRollingBufferPacket->rtpPacket.header.extension.pExtensionPayload = &pRollingBufferPacket->twccExtensionPayload;

                #if ENABLE_TWCC_SUPPORT
                memset( &packetInfo,
                        0,
                        sizeof( TwccPacketInfo_t ) );
                packetInfo.packetSize = packeth265.packetDataLength;
                packetInfo.localSentTime = NetworkingUtils_GetCurrentTimeUs( NULL );
                packetInfo.packetSeqNum = pSession->rtpConfig.twccSequence;

                RtcpTwccManager_AddPacketInfo( &pSession->pCtx->rtcpTwccManager,
                                               &packetInfo );
                #endif /* ENABLE_TWCC_SUPPORT */

                pSession->rtpConfig.twccSequence++;
            }

            pRollingBufferPacket->rtpPacket.payloadLength = packeth265.packetDataLength;
            pRollingBufferPacket->rtpPacket.pPayload = packeth265.pPacketData;

            /* PeerConnectionSrtp_ConstructSrtpPacket() serializes RTP packet and encrypt it. */
            ret = PeerConnectionSrtp_ConstructSrtpPacket( pSession,
                                                          &pRollingBufferPacket->rtpPacket,
                                                          pSrtpPacket,
                                                          &srtpPacketLength );
        }
        else
        {
            LogError( ( "Fail to get h265 packet, result: %d", resulth265 ) );
            ret = PEER_CONNECTION_RESULT_FAIL_PACKETIZER_GET_PACKET;
        }

        if( ret == PEER_CONNECTION_RESULT_OK )
        {
            /* Update the rolling buffer length before storing. */
            if( bufferAfterEncrypt == 0 )
            {
                pRollingBufferPacket->packetBufferLength = packeth265.packetDataLength;
            }
            else
            {
                pRollingBufferPacket->packetBufferLength = srtpPacketLength;
            }

            /* Udpate the packet into rolling buffer. */
            ret = PeerConnectionRollingBuffer_SetPacket( &pSrtpSender->txRollingBuffer,
                                                         ( *pRtpSeq )++,
                                                         pRollingBufferPacket );
        }

        if( ( ret != PEER_CONNECTION_RESULT_OK ) && ( pRollingBufferPacket != NULL ) )
        {
            /* If any failure, release the allocated RTP buffer. */
            PeerConnectionRollingBuffer_DiscardRtpSequenceBuffer( &pSrtpSender->txRollingBuffer,
                                                                  pRollingBufferPacket );
        }

        /* Write the constructed RTP packets through network. */
        if( ret == PEER_CONNECTION_RESULT_OK )
        {
            resultIceController = IceController_SendToRemotePeer( &pSession->iceControllerContext,
                                                                  pSrtpPacket,
                                                                  srtpPacketLength );
            if( resultIceController != ICE_CONTROLLER_RESULT_OK )
            {
                LogWarn( ( "Fail to send RTP packet, ret: %d", resultIceController ) );
                ret = PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_SEND_RTP_PACKET;
            }
        }

        if( ret == PEER_CONNECTION_RESULT_OK )
        {
            packetSent++;
            bytesSent += pRollingBufferPacket->rtpPacket.payloadLength;
        }

        #if METRIC_PRINT_ENABLED
        if( ret == PEER_CONNECTION_RESULT_OK )
        {
            Metric_EndEvent( METRIC_EVENT_SENDING_FIRST_FRAME );
        }
        #endif
    }

    if( packetSent != 0 )
    {
        if( pTransceiver->rtpSender.rtpFirstFrameWallClockTimeUs == 0 )
        {
            pTransceiver->rtpSender.rtpFirstFrameWallClockTimeUs = NetworkingUtils_GetCurrentTimeUs( NULL );
            pTransceiver->rtpSender.rtpTimeOffset = randomRtpTimeoffset;
        }

        pTransceiver->rtcpStats.rtpPacketsTransmitted += packetSent;
        pTransceiver->rtcpStats.rtpBytesTransmitted += bytesSent;
    }

    if( isLocked )
    {
        xSemaphoreGive( pSrtpSender->senderMutex );
    }

    return ret;
}
