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
#include "peer_connection_srtp.h"
#include "peer_connection_rolling_buffer.h"
#include "peer_connection_jitter_buffer.h"
#if METRIC_PRINT_ENABLED
#include "metric.h"
#endif

/* API includes. */
#include "rtp_api.h"
#include "rtcp_api.h"
#include "ice_controller.h"
#include "networking_utils.h"
#include "peer_connection_codec_helper.h"
#include "peer_connection_g711_helper.h"
#include "peer_connection_h264_helper.h"
#include "peer_connection_h265_helper.h"
#include "peer_connection_opus_helper.h"

/*-----------------------------------------------------------*/

static PeerConnectionResult_t OnJitterBufferFrameReady( void * pCustomContext,
                                                        uint16_t startSequence,
                                                        uint16_t endSequence )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK, retFillFrame = PEER_CONNECTION_RESULT_OK;
    PeerConnectionSrtpReceiver_t * pSrtpReceiver = NULL;
    size_t frameBufferLength = PEER_CONNECTION_FRAME_BUFFER_SIZE;
    PeerConnectionFrame_t frame;
    uint32_t rtpTimestamp;

    if( pCustomContext == NULL )
    {
        LogError( ( "Invalid input, pCustomContext: %p", pCustomContext ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        pSrtpReceiver = ( PeerConnectionSrtpReceiver_t * ) pCustomContext;

        /* Return fail only when hitting critical issues. If fill fram API returns fail, we still return
         * OK to the jitter buffer to release these packet normally. */
        retFillFrame = PeerConnectionJitterBuffer_FillFrame( &pSrtpReceiver->rxJitterBuffer,
                                                             startSequence,
                                                             endSequence,
                                                             pSrtpReceiver->frameBuffer,
                                                             &frameBufferLength,
                                                             &rtpTimestamp );
        LogDebug( ( "Fill frame with result: %d, length: %u, start seq: %u, end seq: %u",
                    retFillFrame,
                    frameBufferLength,
                    startSequence,
                    endSequence ) );

        if( retFillFrame == PEER_CONNECTION_RESULT_OK )
        {
            if( pSrtpReceiver->onFrameReadyCallbackFunc )
            {
                memset( &frame, 0, sizeof( PeerConnectionFrame_t ) );
                frame.version = PEER_CONNECTION_FRAME_CURRENT_VERSION;
                frame.presentationUs = PEER_CONNECTION_SRTP_CONVERT_RTP_TIMESTAMP_TO_TIME_US( pSrtpReceiver->rxJitterBuffer.clockRate,
                                                                                            rtpTimestamp );
                frame.pData = pSrtpReceiver->frameBuffer;
                frame.dataLength = frameBufferLength;
                pSrtpReceiver->onFrameReadyCallbackFunc( pSrtpReceiver->pOnFrameReadyCallbackCustomContext,
                                                         &frame );
            }
        }
    }

    return ret;
}

static PeerConnectionResult_t OnJitterBufferFrameDrop( void * pCustomContext,
                                                       uint16_t startSequence,
                                                       uint16_t endSequence )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( pCustomContext == NULL )
    {
        LogError( ( "Invalid input, pCustomContext: %p", pCustomContext ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        LogDebug( ( "Dropping packets from start seq: %u to end seq: %u",
                    startSequence,
                    endSequence ) );
    }

    return ret;
}

PeerConnectionResult_t PeerConnectionSrtp_ConstructSrtpPacket( PeerConnectionSession_t * pSession,
                                                               RtpPacket_t * pPacketRtp,
                                                               uint8_t * pOutputSrtpPacket,
                                                               size_t * pOutputSrtpPacketLength )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    RtpResult_t resultRtp;
    size_t rtpBufferLength;
    srtp_err_status_t errorStatus;
    uint8_t isLocked = 0U;

    if( ( pSession == NULL ) ||
        ( pPacketRtp == NULL ) ||
        ( pOutputSrtpPacket == NULL ) ||
        ( pOutputSrtpPacketLength == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pPacketRtp: %p, pOutputSrtpPacket: %p, pOutputSrtpPacketLength: %p",
                    pSession,
                    pPacketRtp,
                    pOutputSrtpPacket,
                    pOutputSrtpPacketLength ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    /* Get buffer from sender for serializing RTP packet */
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        rtpBufferLength = *pOutputSrtpPacketLength;
    }

    /* Contruct RTP packet for each payload buffer. */
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        resultRtp = Rtp_Serialize( &pSession->pCtx->rtpContext,
                                   pPacketRtp,
                                   pOutputSrtpPacket,
                                   &rtpBufferLength );
        if( resultRtp != RTP_RESULT_OK )
        {
            LogError( ( "Fail to serialize RTP packet, result: %d", resultRtp ) );
            ret = PEER_CONNECTION_RESULT_FAIL_RTP_SERIALIZE;
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
            LogError( ( "Fail to take SRTP session mutex to construct SRTP packet." ) );
            ret = PEER_CONNECTION_RESULT_FAIL_TAKE_SRTP_MUTEX;
        }
    }

    /* Encrypt it by SRTP. */
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( pSession->srtpTransmitSession != NULL )
        {
            errorStatus = srtp_protect( pSession->srtpTransmitSession,
                                        pOutputSrtpPacket,
                                        rtpBufferLength,
                                        pOutputSrtpPacket,
                                        pOutputSrtpPacketLength,
                                        0 );
            if( errorStatus != srtp_err_status_ok )
            {
                LogError( ( "Fail to encrypt Tx SRTP packet, errorStatus: %d", errorStatus ) );
                ret = PEER_CONNECTION_RESULT_FAIL_ENCRYPT_SRTP_RTP_PACKET;
            }
        }
        else
        {
            LogWarn( ( "SRTP session has been freed before encrypting." ) );
        }
    }

    if( isLocked != 0U )
    {
        xSemaphoreGive( pSession->srtpSessionMutex );
    }

    return ret;
}

PeerConnectionResult_t PeerConnectionSrtp_Init( PeerConnectionSession_t * pSession )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    srtp_policy_t receivePolicy, transmitPolicy;
    void (* srtp_policy_setter)( srtp_crypto_policy_t * ) = NULL;
    void (* srtcp_policy_setter)( srtp_crypto_policy_t * ) = NULL;
    srtp_err_status_t errorStatus;
    PeerConnectionSrtpSender_t * pSrtpSender = NULL;
    PeerConnectionSrtpReceiver_t * pSrtpReceiver = NULL;
    int i;
    size_t maxSizePerPacket = PEER_CONNECTION_SRTP_RTP_PACKET_MAX_LENGTH;
    uint8_t isLocked = 0U;

    if( pSession == NULL )
    {
        LogError( ( "Invalid input, pSession: %p", pSession ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        switch( pSession->dtlsSession.xNetworkCredentials.dtlsKeyingMaterial.srtpProfile )
        {
            case KVS_SRTP_PROFILE_AES128_CM_HMAC_SHA1_80:
                srtp_policy_setter = srtp_crypto_policy_set_rtp_default;
                srtcp_policy_setter = srtp_crypto_policy_set_rtp_default;
                break;
            case KVS_SRTP_PROFILE_AES128_CM_HMAC_SHA1_32:
                srtp_policy_setter = srtp_crypto_policy_set_aes_cm_128_hmac_sha1_32;
                srtcp_policy_setter = srtp_crypto_policy_set_rtp_default;
                break;
            default:
                LogError( ( "Unknown SRTP profile: %d", pSession->dtlsSession.xNetworkCredentials.dtlsKeyingMaterial.srtpProfile ) );
                ret = PEER_CONNECTION_RESULT_UNKNOWN_SRTP_PROFILE;
                break;
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
            LogError( ( "Fail to take SRTP session mutex to create SRTP session instance." ) );
            ret = PEER_CONNECTION_RESULT_FAIL_TAKE_SRTP_MUTEX;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        memset( &receivePolicy, 0, sizeof( receivePolicy ) );
        srtp_policy_setter( &receivePolicy.rtp );
        srtcp_policy_setter( &receivePolicy.rtcp );

        if( pSession->dtlsSession.isServer == 0U )
        {
            receivePolicy.key = pSession->dtlsSession.xNetworkCredentials.dtlsKeyingMaterial.serverWriteKey;
        }
        else
        {
            receivePolicy.key = pSession->dtlsSession.xNetworkCredentials.dtlsKeyingMaterial.clientWriteKey;
        }
        receivePolicy.ssrc.type = ssrc_any_inbound;
        receivePolicy.next = NULL;

        errorStatus = srtp_create( &( pSession->srtpReceiveSession ),
                                   &receivePolicy );
        if( errorStatus != srtp_err_status_ok )
        {
            LogError( ( "Fail to create Rx SRTP session, errorStatus: %d", errorStatus ) );
            ret = PEER_CONNECTION_RESULT_FAIL_CREATE_SRTP_RX_SESSION;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        memset( &transmitPolicy, 0, sizeof( transmitPolicy ) );
        srtp_policy_setter( &transmitPolicy.rtp );
        srtcp_policy_setter( &transmitPolicy.rtcp );

        if( pSession->dtlsSession.isServer == 0U )
        {
            transmitPolicy.key = pSession->dtlsSession.xNetworkCredentials.dtlsKeyingMaterial.clientWriteKey;
        }
        else
        {
            transmitPolicy.key = pSession->dtlsSession.xNetworkCredentials.dtlsKeyingMaterial.serverWriteKey;
        }
        transmitPolicy.ssrc.type = ssrc_any_outbound;
        transmitPolicy.next = NULL;

        errorStatus = srtp_create( &( pSession->srtpTransmitSession ),
                                   &transmitPolicy );
        if( errorStatus != srtp_err_status_ok )
        {
            LogError( ( "Fail to create Tx SRTP session, errorStatus: %d", errorStatus ) );
            ret = PEER_CONNECTION_RESULT_FAIL_CREATE_SRTP_TX_SESSION;
        }
    }

    if( isLocked != 0U )
    {
        xSemaphoreGive( pSession->srtpSessionMutex );
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Initialize Rolling buffers. */
        for( i = 0; i < pSession->transceiverCount; i++ )
        {
            if( ( pSession->pTransceivers[i]->trackKind == TRANSCEIVER_TRACK_KIND_VIDEO ) &&
                ( ( pSession->pTransceivers[i]->direction == TRANSCEIVER_TRACK_DIRECTION_SENDRECV ) ||
                  ( pSession->pTransceivers[i]->direction == TRANSCEIVER_TRACK_DIRECTION_SENDONLY ) ) )
            {
                pSrtpSender = &pSession->videoSrtpSender;
                if( ( pSession->rtpConfig.videoCodecRtxPayload != 0 ) &&
                    ( pSession->rtpConfig.videoCodecRtxPayload != pSession->rtpConfig.videoCodecPayload ) )
                {
                    /* If we're using different payload type in re-transmission, we create the rolling buffer just for RTP payload. */
                    maxSizePerPacket = PEER_CONNECTION_SRTP_RTP_PAYLOAD_MAX_LENGTH;
                }
                ret = PeerConnectionRollingBuffer_Create( &pSession->videoSrtpSender.txRollingBuffer,
                                                          pSession->pTransceivers[i]->rollingbufferBitRate, // bps
                                                          pSession->pTransceivers[i]->rollingbufferDurationSec, // duration in seconds
                                                          maxSizePerPacket );
            }
            else if( ( pSession->pTransceivers[i]->trackKind == TRANSCEIVER_TRACK_KIND_AUDIO ) &&
                     ( ( pSession->pTransceivers[i]->direction == TRANSCEIVER_TRACK_DIRECTION_SENDRECV ) ||
                       ( pSession->pTransceivers[i]->direction == TRANSCEIVER_TRACK_DIRECTION_SENDONLY ) ) )
            {
                pSrtpSender = &pSession->audioSrtpSender;
                if( ( pSession->rtpConfig.audioCodecRtxPayload != 0 ) &&
                    ( pSession->rtpConfig.audioCodecRtxPayload != pSession->rtpConfig.audioCodecPayload ) )
                {
                    /* If we're using different payload type in re-transmission, we create the rolling buffer just for RTP payload. */
                    maxSizePerPacket = PEER_CONNECTION_SRTP_RTP_PAYLOAD_MAX_LENGTH;
                }
                ret = PeerConnectionRollingBuffer_Create( &pSession->audioSrtpSender.txRollingBuffer,
                                                          pSession->pTransceivers[i]->rollingbufferBitRate, // bps
                                                          pSession->pTransceivers[i]->rollingbufferDurationSec, // duration in seconds
                                                          maxSizePerPacket );
            }
            else
            {
                LogInfo( ( "No send needed for this transceiver, kind: %d, direction: %d",
                           pSession->pTransceivers[i]->trackKind,
                           pSession->pTransceivers[i]->direction ) );
            }

            if( ret != PEER_CONNECTION_RESULT_OK )
            {
                break;
            }

            /* Mutex can only be created in executing scheduler. */
            if( ( pSrtpSender != NULL ) &&
                ( pSrtpSender->isSenderMutexInit == 0U ) )
            {
                pSrtpSender->senderMutex = xSemaphoreCreateMutex();
                if( pSrtpSender->senderMutex == NULL )
                {
                    LogError( ( "Fail to create mutex for SRTP sender." ) );
                    ret = PEER_CONNECTION_RESULT_FAIL_CREATE_SENDER_MUTEX;
                    break;
                }
                pSrtpSender->isSenderMutexInit = 1U;
            }
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Initialize Jitter buffers. */
        for( i = 0; i < pSession->transceiverCount; i++ )
        {
            if( ( pSession->pTransceivers[i]->trackKind == TRANSCEIVER_TRACK_KIND_VIDEO ) &&
                ( ( pSession->pTransceivers[i]->direction == TRANSCEIVER_TRACK_DIRECTION_SENDRECV ) ||
                  ( pSession->pTransceivers[i]->direction == TRANSCEIVER_TRACK_DIRECTION_RECVONLY ) ) )
            {
                LogInfo( ( "Setting video receiver." ) );
                pSrtpReceiver = &pSession->videoSrtpReceiver;
                ret = PeerConnectionJitterBuffer_Create( &pSrtpReceiver->rxJitterBuffer,
                                                         OnJitterBufferFrameReady,
                                                         pSrtpReceiver,
                                                         OnJitterBufferFrameDrop,
                                                         pSrtpReceiver,
                                                         PEER_CONNECTION_SRTP_JITTER_BUFFER_TOLERENCE_TIME_SECOND,   // buffer time in seconds
                                                         pSession->pTransceivers[i]->codecBitMap,
                                                         PEER_CONNECTION_SRTP_VIDEO_CLOCKRATE );
            }
            else if( ( pSession->pTransceivers[i]->trackKind == TRANSCEIVER_TRACK_KIND_AUDIO ) &&
                     ( ( pSession->pTransceivers[i]->direction == TRANSCEIVER_TRACK_DIRECTION_SENDRECV ) ||
                       ( pSession->pTransceivers[i]->direction == TRANSCEIVER_TRACK_DIRECTION_RECVONLY ) ) )
            {
                LogInfo( ( "Setting audio receiver." ) );
                pSrtpReceiver = &pSession->audioSrtpReceiver;
                if( ( pSession->rtpConfig.audioCodecRtxPayload != 0 ) &&
                    ( pSession->rtpConfig.audioCodecRtxPayload != pSession->rtpConfig.audioCodecPayload ) )
                {
                    /* If we're using different payload type in re-transmission, we create the rolling buffer just for RTP payload. */
                    maxSizePerPacket = PEER_CONNECTION_SRTP_RTP_PAYLOAD_MAX_LENGTH;
                }
                ret = PeerConnectionJitterBuffer_Create( &pSrtpReceiver->rxJitterBuffer,
                                                         OnJitterBufferFrameReady,
                                                         pSrtpReceiver,
                                                         OnJitterBufferFrameDrop,
                                                         pSrtpReceiver,
                                                         PEER_CONNECTION_SRTP_JITTER_BUFFER_TOLERENCE_TIME_SECOND,   // buffer time in seconds
                                                         pSession->pTransceivers[i]->codecBitMap,
                                                         PEER_CONNECTION_SRTP_PCM_CLOCKRATE );
            }
            else
            {
                LogInfo( ( "No recv needed for this transceiver, kind: %d, direction: %d",
                           pSession->pTransceivers[i]->trackKind,
                           pSession->pTransceivers[i]->direction ) );
            }

            if( ret != PEER_CONNECTION_RESULT_OK )
            {
                break;
            }
        }
    }

    return ret;
}

PeerConnectionResult_t PeerConnectionSrtp_DeInit( PeerConnectionSession_t * pSession )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    srtp_err_status_t errorStatus;
    uint8_t isLocked = 0U;

    if( pSession == NULL )
    {
        LogError( ( "Invalid input, pSession: %p", pSession ) );
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
            LogError( ( "Fail to take SRTP session mutex to release SRTP session." ) );
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( pSession->srtpReceiveSession != NULL )
        {
            errorStatus = srtp_dealloc( pSession->srtpReceiveSession );
            if( errorStatus != srtp_err_status_ok )
            {
                LogError( ( "Fail to deallocate Rx SRTP session, errorStatus: %d", errorStatus ) );
            }
            pSession->srtpReceiveSession = NULL;
        }

        if( pSession->srtpTransmitSession != NULL )
        {
            errorStatus = srtp_dealloc( pSession->srtpTransmitSession );
            if( errorStatus != srtp_err_status_ok )
            {
                LogError( ( "Fail to deallocate Tx SRTP session, errorStatus: %d", errorStatus ) );
            }
            pSession->srtpTransmitSession = NULL;
        }
    }

    if( isLocked != 0U )
    {
        xSemaphoreGive( pSession->srtpSessionMutex );
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Clean up Video SRTP Sender */
        if( ( pSession->videoSrtpSender.isSenderMutexInit != 0U ) &&
            ( xSemaphoreTake( pSession->videoSrtpSender.senderMutex,
                              portMAX_DELAY ) == pdTRUE ) )
        {
            PeerConnectionRollingBuffer_Free( &pSession->videoSrtpSender.txRollingBuffer );
            xSemaphoreGive( pSession->videoSrtpSender.senderMutex );
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Clean up Audio SRTP Sender */
        if( ( pSession->audioSrtpSender.isSenderMutexInit != 0U ) &&
            ( xSemaphoreTake( pSession->audioSrtpSender.senderMutex,
                              portMAX_DELAY ) == pdTRUE ) )
        {
            PeerConnectionRollingBuffer_Free( &pSession->audioSrtpSender.txRollingBuffer );
            xSemaphoreGive( pSession->audioSrtpSender.senderMutex );
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Clean up Video SRTP Receiver */
        memset( pSession->videoSrtpReceiver.frameBuffer, 0, PEER_CONNECTION_FRAME_BUFFER_SIZE );
        PeerConnectionJitterBuffer_Free( &pSession->videoSrtpReceiver.rxJitterBuffer );
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Clean up Audio SRTP Receiver */
        memset( pSession->audioSrtpReceiver.frameBuffer, 0, PEER_CONNECTION_FRAME_BUFFER_SIZE );
        PeerConnectionJitterBuffer_Free( &pSession->audioSrtpReceiver.rxJitterBuffer );
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Reset callback functions */
        pSession->videoSrtpReceiver.onFrameReadyCallbackFunc = NULL;
        pSession->videoSrtpReceiver.pOnFrameReadyCallbackCustomContext = NULL;
        pSession->audioSrtpReceiver.onFrameReadyCallbackFunc = NULL;
        pSession->audioSrtpReceiver.pOnFrameReadyCallbackCustomContext = NULL;
    }

    return ret;
}

PeerConnectionResult_t PeerConnectionSrtp_HandleSrtpPacket( PeerConnectionSession_t * pSession,
                                                            uint8_t * pBuffer,
                                                            size_t bufferLength )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    srtp_err_status_t errorStatus;
    uint8_t rtpBuffer[ PEER_CONNECTION_SRTP_RTP_PACKET_MAX_LENGTH ];
    size_t rtpBufferLength = PEER_CONNECTION_SRTP_RTP_PACKET_MAX_LENGTH;
    RtpResult_t resultRtp;
    RtpPacket_t rtpPacket;
    PeerConnectionJitterBufferPacket_t * pJitterBufferPacket = NULL;
    PeerConnectionSrtpReceiver_t * pSrtpReceiver = NULL;
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
            LogError( ( "Fail to take SRTP session mutex to construct SRTP packet." ) );
            ret = PEER_CONNECTION_RESULT_FAIL_TAKE_SRTP_MUTEX;
        }
    }

    /* Decrypt it by SRTP. */
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( pSession->srtpReceiveSession != NULL )
        {
            errorStatus = srtp_unprotect( pSession->srtpReceiveSession,
                                          pBuffer,
                                          bufferLength,
                                          rtpBuffer,
                                          &rtpBufferLength );
            if( errorStatus != srtp_err_status_ok )
            {
                LogError( ( "Fail to decrypt Rx SRTP packet, errorStatus: %d", errorStatus ) );
                ret = PEER_CONNECTION_RESULT_FAIL_DECRYPT_SRTP_RTP_PACKET;
            }
            else
            {
                LogVerbose( ( "Decrypt SRTP packet successfully, decrypted length: %u", rtpBufferLength ) );
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

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Deserialize RTP packet. */
        resultRtp = Rtp_DeSerialize( &pSession->pCtx->rtpContext,
                                     rtpBuffer,
                                     rtpBufferLength,
                                     &rtpPacket );
        if( resultRtp != RTP_RESULT_OK )
        {
            LogError( ( "Fail to deserialize RTP packet, result: %d", resultRtp ) );
            ret = PEER_CONNECTION_RESULT_FAIL_RTP_DESERIALIZE;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( pSession->rtpConfig.remoteVideoSsrc == rtpPacket.header.ssrc )
        {
            pSrtpReceiver = &pSession->videoSrtpReceiver;
        }
        else if( pSession->rtpConfig.remoteAudioSsrc == rtpPacket.header.ssrc )
        {
            pSrtpReceiver = &pSession->audioSrtpReceiver;
        }
        else
        {
            LogWarn( ( "Received unknown SSRC: %lu RTP packet.", rtpPacket.header.ssrc ) );
            ret = PEER_CONNECTION_RESULT_FAIL_RTP_RX_NO_MATCHING_SSRC;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        ret = PeerConnectionJitterBuffer_AllocateBuffer( &pSrtpReceiver->rxJitterBuffer,
                                                         &pJitterBufferPacket,
                                                         rtpPacket.payloadLength,
                                                         rtpPacket.header.sequenceNumber );
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        memcpy( pJitterBufferPacket->pPacketBuffer, rtpPacket.pPayload, rtpPacket.payloadLength );
        pJitterBufferPacket->receiveTick = xTaskGetTickCount();
        pJitterBufferPacket->rtpTimestamp = rtpPacket.header.timestamp;
        pJitterBufferPacket->sequenceNumber = rtpPacket.header.sequenceNumber;
        // LogInfo( ( "Dumping RTP payload: %u, seq: %u, timestamp: %lu", rtpPacket.payloadLength, rtpPacket.header.sequenceNumber, rtpPacket.header.timestamp ) );
        // for( int i = 0; i < rtpPacket.payloadLength; i++ )
        // {
        //     printf( "%02x ", rtpPacket.pPayload[i] );
        // }
        // printf( "\n" );

        ret = PeerConnectionJitterBuffer_Push( &pSrtpReceiver->rxJitterBuffer,
                                               pJitterBufferPacket );
    }

    return ret;
}
