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
#include "FreeRTOS.h"
#include "task.h"
#include "logging.h"
#include "peer_connection.h"
#include "peer_connection_srtcp.h"
#include "peer_connection_srtp.h"
#include "peer_connection_sdp.h"
#include "rtp_api.h"
#include "rtcp_api.h"
#include "peer_connection_rolling_buffer.h"
#if METRIC_PRINT_ENABLED
#include "metric.h"
#endif
#include "peer_connection_codec_helper.h"
#include "peer_connection_g711_helper.h"
#include "peer_connection_h264_helper.h"
#include "peer_connection_h265_helper.h"
#include "peer_connection_opus_helper.h"
#include "networking_utils.h"

#include "lwip/sockets.h"

#if ENABLE_SCTP_DATA_CHANNEL
#include "peer_connection_sctp.h"
#endif


#define PEER_CONNECTION_SESSION_TASK_NAME "PcSnTsk"
#define PEER_CONNECTION_SESSION_RX_TASK_NAME "PcRxTsk" // For Ice controller to monitor socket Rx path
#define PEER_CONNECTION_MESSAGE_QUEUE_NAME "/PcSessionMq"
#define PEER_CONNECTION_AUDIO_TIMER_NAME "RtcpAudioSenderReportTimer"
#define PEER_CONNECTION_VIDEO_TIMER_NAME "RtcpVideoSenderReportTimer"
#define PEER_CONNECTION_CLOSE_SESSION_TIMER_NAME "CloseSnTimer"

#define PEER_CONNECTION_MAX_QUEUE_MSG_NUM ( 30 )
#define PEER_CONNECTION_RTCP_REPORT_TIMER_INTERVAL_MS ( 5000 )

#define PEER_CONNECTION_MAX_DTLS_DECRYPTED_DATA_LENGTH ( 2048 )

PeerConnectionContext_t peerConnectionContext = { 0 };

extern void IceControllerSocketListener_Task( void * pParameter );
static void PeerConnection_SessionTask( void * pParameter );
static void SessionProcessEndlessLoop( PeerConnectionSession_t * pSession );
static PeerConnectionResult_t SendPeerConnectionEvent( PeerConnectionSession_t * pSession,
                                                       PeerConnectionSessionRequestType_t requestType,
                                                       void * pRequestContent,
                                                       size_t contentLength );
static PeerConnectionResult_t HandleRequest( PeerConnectionSession_t * pSession,
                                             MessageQueueHandler_t * pRequestQueue );
static PeerConnectionResult_t HandleAddRemoteCandidateRequest( PeerConnectionSession_t * pSession,
                                                               PeerConnectionSessionRequestMessage_t * pRequestMessage );
static PeerConnectionResult_t HandleProcessIceCandidatesAndPairs( PeerConnectionSession_t * pSession,
                                                                  PeerConnectionSessionRequestMessage_t * pRequestMessage );
static PeerConnectionResult_t HandlePeriodConnectionCheck( PeerConnectionSession_t * pSession,
                                                           PeerConnectionSessionRequestMessage_t * pRequestMessage );
static PeerConnectionResult_t HandleIceClosing( PeerConnectionSession_t * pSession,
                                                PeerConnectionSessionRequestMessage_t * pRequestMessage );
static PeerConnectionResult_t PeerConnection_OnRtcpSenderReportCallback( PeerConnectionSession_t * pSession,
                                                                         PeerConnectionSessionRequestMessage_t * pRequestMessage );
static int32_t InitDtlsSession( PeerConnectionSession_t * pSession, uint8_t isServer );
static int32_t ExecuteDtlsHandshake( PeerConnectionSession_t * pSession );
static int32_t OnDtlsHandshakeComplete( PeerConnectionSession_t * pSession );
static void PeerConnection_SetTimer( PeerConnectionSession_t * pSession );

static void PeerConnection_SessionTask( void * pParameter )
{
    PeerConnectionSession_t * pSession = ( PeerConnectionSession_t * ) pParameter;
    EventBits_t uxBits = 0U;

    for( ;; )
    {
        LogDebug( ( "Waiting start up barrier." ) );
        uxBits = xEventGroupWaitBits( pSession->startupBarrier,
                                      PEER_CONNECTION_START_UP_BARRIER_BIT,
                                      pdTRUE,
                                      pdTRUE,
                                      portMAX_DELAY );
        if( ( uxBits & PEER_CONNECTION_START_UP_BARRIER_BIT ) != PEER_CONNECTION_START_UP_BARRIER_BIT )
        {
            LogError( ( "Unexpected return value from start up barrier, uxBits: 0x%lx", uxBits ) );
            continue;
        }

        LogDebug( ( "Entering peer connection session endless loop." ) );
        SessionProcessEndlessLoop( pSession );
    }
}

static void SessionProcessEndlessLoop( PeerConnectionSession_t * pSession )
{
    uint8_t skipProcess = 0;
    PeerConnectionResult_t result = PEER_CONNECTION_RESULT_OK;

    if( pSession == NULL )
    {
        LogError( ( "Invalid input, pSession: %p", pSession ) );
        skipProcess = 1;
    }

    for( ; !skipProcess; )
    {
        result = HandleRequest( pSession,
                                &pSession->requestQueue );
        if( ( result != PEER_CONNECTION_RESULT_OK ) &&
            ( result != PEER_CONNECTION_RESULT_CLOSING ) )
        {
            LogError( ( "Unexpected result while handling request, result: %d", result ) );
            break;
        }
        else if( result == PEER_CONNECTION_RESULT_CLOSING )
        {
            LogDebug( ( "Session is closing." ) );
            break;
        }
        else
        {
            /* Empty else marker. */
        }

        /* Check connection inactivity timeout. */
        if( ( pSession->state == PEER_CONNECTION_SESSION_STATE_CONNECTION_READY ) &&
            ( ( NetworkingUtils_GetCurrentTimeUs( NULL ) / 1000 ) > pSession->inactiveConnectionTimeoutMs ) )
        {
            LogInfo( ( "Detect inactive connection, closing peer connection session: %s.",
                       pSession->combinedName ) );
            PeerConnection_CloseSession( pSession );
        }
    }
}

static void EmptyMessageQueue( MessageQueueHandler_t * pMessageQueue )
{
    MessageQueueResult_t result;
    PeerConnectionSessionRequestMessage_t requestMsg;
    size_t requestMsgLength;

    if( pMessageQueue == NULL )
    {
        LogError( ( "Failed to clear message queue: queue pointer is NULL" ) );
    }
    else
    {
        do
        {
            result = MessageQueue_IsEmpty( pMessageQueue );
            if( result == MESSAGE_QUEUE_RESULT_MQ_HAVE_MESSAGE )
            {
                requestMsgLength = sizeof( PeerConnectionSessionRequestMessage_t );
                ( void ) MessageQueue_Recv( pMessageQueue,
                                            &requestMsg,
                                            &requestMsgLength );
            }
        } while( result == MESSAGE_QUEUE_RESULT_MQ_HAVE_MESSAGE );
    }
}

static PeerConnectionResult_t SendPeerConnectionEvent( PeerConnectionSession_t * pSession,
                                                       PeerConnectionSessionRequestType_t requestType,
                                                       void * pRequestContent,
                                                       size_t contentLength )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    MessageQueueResult_t retMessageQueue;
    PeerConnectionSessionRequestMessage_t requestMessage = {
        .requestType = requestType,
    };

    if( pSession == NULL )
    {
        LogError( ( "Invalid input, pSession: %p", pSession ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        retMessageQueue = MessageQueue_IsFull( &pSession->requestQueue );
        if( retMessageQueue == MESSAGE_QUEUE_RESULT_MQ_IS_FULL )
        {
            LogWarn( ( "The message queue in peer connection session: %p is full, dropping request type: %d",
                       pSession,
                       requestType ) );
            ret = PEER_CONNECTION_RESULT_FAIL_MQ_SEND;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( pRequestContent != NULL )
        {
            memcpy( &requestMessage.peerConnectionSessionRequestContent, pRequestContent, contentLength );
        }

        retMessageQueue = MessageQueue_Send( &pSession->requestQueue,
                                             &requestMessage,
                                             sizeof( PeerConnectionSessionRequestMessage_t ) );
        if( retMessageQueue != MESSAGE_QUEUE_RESULT_OK )
        {
            LogError( ( "Fail to send message queue, error: %d, requestType: %d", retMessageQueue, requestType ) );
            ret = PEER_CONNECTION_RESULT_FAIL_MQ_SEND;
        }
    }

    return ret;
}

static void OnRtcpSenderReportAudioTimerExpire( void * pParameter )
{
    PeerConnectionSession_t * pSession = ( PeerConnectionSession_t * ) pParameter;
    PeerConnectionSessionRequestMessage_t requestMessage;
    uint8_t i;
    uint64_t currentTimeUs = NetworkingUtils_GetCurrentTimeUs( NULL );

    for( i = 0; i < pSession->transceiverCount; i++ )
    {
        if( pSession->pTransceivers[ i ]->trackKind == TRANSCEIVER_TRACK_KIND_AUDIO )
        {
            requestMessage.peerConnectionSessionRequestContent.rtcpContent.currentTimeUs = currentTimeUs;
            requestMessage.peerConnectionSessionRequestContent.rtcpContent.pTransceiver = pSession->pTransceivers[ i ];

            ( void ) SendPeerConnectionEvent( pSession,
                                              PEER_CONNECTION_SESSION_REQUEST_TYPE_RTCP_SENDER_REPORT,
                                              &requestMessage.peerConnectionSessionRequestContent.rtcpContent,
                                              sizeof( requestMessage.peerConnectionSessionRequestContent.rtcpContent ) );
            break;
        }
    }
}

static void OnRtcpSenderReportVideoTimerExpire( void * pParameter )
{
    PeerConnectionSession_t * pSession = ( PeerConnectionSession_t * ) pParameter;
    PeerConnectionSessionRequestMessage_t requestMessage = {
        .requestType = PEER_CONNECTION_SESSION_REQUEST_TYPE_RTCP_SENDER_REPORT,
    };
    uint8_t i;
    uint64_t currentTimeUs = NetworkingUtils_GetCurrentTimeUs( NULL );

    for( i = 0; i < pSession->transceiverCount; i++ )
    {
        if( pSession->pTransceivers[ i ]->trackKind == TRANSCEIVER_TRACK_KIND_VIDEO )
        {
            requestMessage.peerConnectionSessionRequestContent.rtcpContent.currentTimeUs = currentTimeUs;
            requestMessage.peerConnectionSessionRequestContent.rtcpContent.pTransceiver = pSession->pTransceivers[ i ];

            ( void ) SendPeerConnectionEvent( pSession,
                                              PEER_CONNECTION_SESSION_REQUEST_TYPE_RTCP_SENDER_REPORT,
                                              &requestMessage.peerConnectionSessionRequestContent.rtcpContent,
                                              sizeof( requestMessage.peerConnectionSessionRequestContent.rtcpContent ) );
            break;
        }
    }
}

static void OnCloseSessionTimerExpire( void * pParameter )
{
    PeerConnectionSession_t * pSession = ( PeerConnectionSession_t * ) pParameter;
    EventBits_t uxBits = 0U;

    if( ( pSession != NULL ) &&
        ( pSession->state == PEER_CONNECTION_SESSION_STATE_START ) )
    {
        LogInfo( ( "Detect long time no SDP message scenario, closing peer connection session, pSession: %p", pSession ) );
        EmptyMessageQueue( &pSession->requestQueue );

        ( void ) SendPeerConnectionEvent( pSession,
                                          PEER_CONNECTION_SESSION_REQUEST_TYPE_PEER_CONNECTION_CLOSE_NO_ICE_FLOW,
                                          NULL,
                                          0 );

        /* Wake peer connection session to free resources. */
        uxBits = xEventGroupSetBits( pSession->startupBarrier,
                                     PEER_CONNECTION_START_UP_BARRIER_BIT );
        /* Since we configure xClearBitOnExit while waiting, the return value might be cleared automatically.
         * Thus we don't check return value here. */
        ( void ) uxBits;
    }
}

static void OnClosePeerConnection( PeerConnectionSession_t * pSession )
{
    if( pSession == NULL )
    {
        LogError( ( "Invalid close request with NULL session pointer." ) );
    }
    else
    {
        // Free each transceiver in the array
        pSession->transceiverCount = 0;
        pSession->mLinesTransceiverCount = 0;
        memset( pSession->pTransceivers, 0, sizeof( pSession->pTransceivers ) );

        /* Reset the state to inited for user to re-use. */
        pSession->state = PEER_CONNECTION_SESSION_STATE_INITED;
    }
}

static PeerConnectionResult_t HandleRequest( PeerConnectionSession_t * pSession,
                                             MessageQueueHandler_t * pRequestQueue )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    MessageQueueResult_t retMessageQueue;
    PeerConnectionSessionRequestMessage_t requestMsg;
    size_t requestMsgLength;

    /* Handle event. */
    requestMsgLength = sizeof( PeerConnectionSessionRequestMessage_t );
    retMessageQueue = MessageQueue_Recv( pRequestQueue,
                                         &requestMsg,
                                         &requestMsgLength );
    if( retMessageQueue == MESSAGE_QUEUE_RESULT_OK )
    {
        /* Received message, process it. */
        LogDebug( ( "Peer connection receives request with type: %d", requestMsg.requestType ) );
        switch( requestMsg.requestType )
        {
            case PEER_CONNECTION_SESSION_REQUEST_TYPE_ADD_REMOTE_CANDIDATE:
                ( void ) HandleAddRemoteCandidateRequest( pSession,
                                                          &requestMsg );
                break;
            case PEER_CONNECTION_SESSION_REQUEST_TYPE_PROCESS_ICE_CANDIDATES_AND_PAIRS:
                ( void ) HandleProcessIceCandidatesAndPairs( pSession,
                                                             &requestMsg );
                break;
            case PEER_CONNECTION_SESSION_REQUEST_TYPE_PERIOD_CONNECTION_CHECK:
                ( void ) HandlePeriodConnectionCheck( pSession,
                                                      &requestMsg );

                /* If a P2P connection is found and DTLS handshaking is in progress,
                 * invoke the handshake here to retry and prevent packet loss in transit. */
                if( pSession->state == PEER_CONNECTION_SESSION_STATE_P2P_CONNECTION_FOUND )
                {
                    ( void ) ExecuteDtlsHandshake( pSession );
                }
                break;
            case PEER_CONNECTION_SESSION_REQUEST_TYPE_ICE_CLOSING:
                ( void ) HandleIceClosing( pSession,
                                           &requestMsg );
                break;
            case PEER_CONNECTION_SESSION_REQUEST_TYPE_ICE_CLOSED:
                OnClosePeerConnection( pSession );
                ret = PEER_CONNECTION_RESULT_CLOSING;
                break;
            case PEER_CONNECTION_SESSION_REQUEST_TYPE_RTCP_SENDER_REPORT:
                ( void ) PeerConnection_OnRtcpSenderReportCallback( pSession,
                                                                    &requestMsg );
                break;
            case PEER_CONNECTION_SESSION_REQUEST_TYPE_PEER_CONNECTION_CLOSE:
                PeerConnection_CloseSession( pSession );
                break;
            case PEER_CONNECTION_SESSION_REQUEST_TYPE_PEER_CONNECTION_CLOSE_NO_ICE_FLOW:
                PeerConnection_CloseSession( pSession );

                /* Reset the state to init for next peer since ICE negotiation has not started yet */
                OnClosePeerConnection( pSession );
                break;
            default:
                /* Unknown request, drop it. */
                LogDebug( ( "Dropping unknown request %d", requestMsg.requestType ) );
                break;
        }
    }

    return ret;
}

static PeerConnectionResult_t HandleAddRemoteCandidateRequest( PeerConnectionSession_t * pSession,
                                                               PeerConnectionSessionRequestMessage_t * pRequestMessage )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    IceControllerResult_t iceControllerResult;
    IceControllerCandidate_t * pRemoteCandidate = ( IceControllerCandidate_t * )&pRequestMessage->peerConnectionSessionRequestContent;
    IceRemoteCandidateInfo_t remoteCandidateInfo;

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( pSession->state == PEER_CONNECTION_SESSION_STATE_FIND_CONNECTION )
        {
            memset( &remoteCandidateInfo, 0, sizeof( IceRemoteCandidateInfo_t ) );
            remoteCandidateInfo.candidateType = pRemoteCandidate->candidateType;
            remoteCandidateInfo.pEndpoint = &( pRemoteCandidate->iceEndpoint );
            remoteCandidateInfo.priority = pRemoteCandidate->priority;
            remoteCandidateInfo.remoteProtocol = pRemoteCandidate->protocol;

            iceControllerResult = IceController_AddRemoteCandidate( &pSession->iceControllerContext,
                                                                    &remoteCandidateInfo );
            if( iceControllerResult != ICE_CONTROLLER_RESULT_OK )
            {
                /* Skip unsupported candidate types (e.g. IPv6 or TCP candidates) - this is expected behavior. */
                LogDebug( ( "Fail to add remote candidate, result: %d.", iceControllerResult ) );
                ret = PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_ADD_REMOTE_CANDIDATE;
            }
        }
        else
        {
            LogDebug( ( "Cannot process candidate | Type=%d | Connection State=%d | Reason=Invalid connection state",
                        pRemoteCandidate->candidateType,
                        pSession->state ) );
        }
    }

    return ret;
}

static PeerConnectionResult_t HandleProcessIceCandidatesAndPairs( PeerConnectionSession_t * pSession,
                                                                  PeerConnectionSessionRequestMessage_t * pRequestMessage )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    IceControllerResult_t iceControllerResult;

    ( void ) pRequestMessage;

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        iceControllerResult = IceController_ProcessIceCandidatesAndPairs( &pSession->iceControllerContext );
        if( iceControllerResult != ICE_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Fail to process ICE candidates and pairs, result: %d", iceControllerResult ) );
            ret = PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_PROCESS_CANDIDATES_AND_PAIRS;
        }
    }

    return ret;
}

static PeerConnectionResult_t HandlePeriodConnectionCheck( PeerConnectionSession_t * pSession,
                                                           PeerConnectionSessionRequestMessage_t * pRequestMessage )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    IceControllerResult_t iceControllerResult;

    ( void ) pRequestMessage;

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        iceControllerResult = IceController_PeriodConnectionCheck( &pSession->iceControllerContext );
        if( iceControllerResult != ICE_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Fail to check ICE connection." ) );
            ret = PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_PERIOD_CONNECTION_CHECK;
        }
    }

    return ret;
}

static PeerConnectionResult_t HandleIceClosing( PeerConnectionSession_t * pSession,
                                                PeerConnectionSessionRequestMessage_t * pRequestMessage )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    IceControllerResult_t iceControllerResult;

    ( void ) pRequestMessage;

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        iceControllerResult = IceController_AddressClosing( &pSession->iceControllerContext );
        if( iceControllerResult != ICE_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Fail to close ICE context." ) );
            ret = PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_ADDRESS_CLOSING;
        }
    }

    return ret;
}

static PeerConnectionResult_t SendRemoteCandidateRequest( PeerConnectionSession_t * pSession,
                                                          IceControllerCandidate_t * pRemoteCandidate )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    TimerControllerResult_t retTimer;

    if( ( pSession == NULL ) || ( pRemoteCandidate == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pRemoteCandidate: %p", pSession, pRemoteCandidate ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    /* Record the timeout while receiving first candidate. */
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( pSession->state == PEER_CONNECTION_SESSION_STATE_START )
        {
            retTimer = TimerController_IsTimerSet( &pSession->closeSessionTimer );
            if( retTimer == TIMER_CONTROLLER_RESULT_NOT_SET )
            {
                TimerController_SetTimer( &pSession->closeSessionTimer,
                                          PEER_CONNECTION_WAIT_SDP_MESSAGE_TIMEOUT_MS,
                                          0U );
            }
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        ret = SendPeerConnectionEvent( pSession,
                                       PEER_CONNECTION_SESSION_REQUEST_TYPE_ADD_REMOTE_CANDIDATE,
                                       pRemoteCandidate,
                                       sizeof( IceControllerCandidate_t ) );
    }

    return ret;
}

static int32_t OnIceEventProcessIceCandidatesAndPairs( PeerConnectionSession_t * pSession )
{
    int32_t ret = 0;
    PeerConnectionResult_t result;

    if( pSession == NULL )
    {
        LogError( ( "Invalid input, pSession: %p", pSession ) );
        ret = -10;
    }

    if( ret == 0 )
    {
        result = SendPeerConnectionEvent( pSession,
                                          PEER_CONNECTION_SESSION_REQUEST_TYPE_PROCESS_ICE_CANDIDATES_AND_PAIRS,
                                          NULL,
                                          0 );
        if( result != PEER_CONNECTION_RESULT_OK )
        {
            ret = -11;
        }
    }

    return ret;
}

static int32_t OnIceEventPeriodicConnectionCheck( PeerConnectionSession_t * pSession )
{
    int32_t ret = 0;
    PeerConnectionResult_t result;

    if( pSession == NULL )
    {
        LogError( ( "Invalid input, pSession: %p", pSession ) );
        ret = -10;
    }

    if( ret == 0 )
    {
        result = SendPeerConnectionEvent( pSession,
                                          PEER_CONNECTION_SESSION_REQUEST_TYPE_PERIOD_CONNECTION_CHECK,
                                          NULL,
                                          0 );
        if( result != PEER_CONNECTION_RESULT_OK )
        {
            ret = -11;
        }
    }

    return ret;
}

static int32_t OnIceEventClosing( PeerConnectionSession_t * pSession )
{
    int32_t ret = 0;
    PeerConnectionResult_t result;

    if( pSession == NULL )
    {
        LogError( ( "Invalid input, pSession: %p", pSession ) );
        ret = -10;
    }

    if( ret == 0 )
    {
        result = SendPeerConnectionEvent( pSession,
                                          PEER_CONNECTION_SESSION_REQUEST_TYPE_ICE_CLOSING,
                                          NULL,
                                          0 );
        if( result != PEER_CONNECTION_RESULT_OK )
        {
            ret = -11;
        }
    }

    return ret;
}

static int32_t OnIceEventClosed( PeerConnectionSession_t * pSession )
{
    int32_t ret = 0;
    PeerConnectionResult_t result;

    if( pSession == NULL )
    {
        LogError( ( "Invalid input, pSession: %p", pSession ) );
        ret = -10;
    }

    if( ret == 0 )
    {
        result = SendPeerConnectionEvent( pSession,
                                          PEER_CONNECTION_SESSION_REQUEST_TYPE_ICE_CLOSED,
                                          NULL,
                                          0 );
        if( result != PEER_CONNECTION_RESULT_OK )
        {
            ret = -11;
        }
    }

    return ret;
}

static int32_t OnIceEventPeerConnectionClose( PeerConnectionSession_t * pSession )
{
    int32_t ret = 0;
    PeerConnectionResult_t result;

    if( pSession == NULL )
    {
        LogError( ( "Invalid input, pSession: %p", pSession ) );
        ret = -10;
    }

    if( ret == 0 )
    {
        result = SendPeerConnectionEvent( pSession,
                                          PEER_CONNECTION_SESSION_REQUEST_TYPE_PEER_CONNECTION_CLOSE,
                                          NULL,
                                          0 );
        if( result != PEER_CONNECTION_RESULT_OK )
        {
            ret = -11;
        }
    }

    return ret;
}

static int32_t HandleDtlsTermination( PeerConnectionSession_t * pSession )
{
    int32_t ret = 0;

    if( pSession == NULL )
    {
        LogError( ( "Invalid input, pSession: %p", pSession ) );
        ret = -10;
    }

    if( ret == 0 )
    {
        DTLS_Disconnect( &pSession->dtlsSession.xNetworkContext );

        /* Close the socket context to avoid any input packets triggering unexpected RTP/RTCP handling. */
        PeerConnection_CloseSession( pSession );
    }

    return ret;
}

static int32_t OnDtlsSendHook( void * pCustomContext,
                               const uint8_t * pBuffer,
                               size_t bufferLength )
{
    int32_t ret = 0;
    PeerConnectionSession_t * pSession = ( PeerConnectionSession_t * ) pCustomContext;
    IceControllerResult_t resultIceController = ICE_CONTROLLER_RESULT_OK;

    resultIceController = IceController_SendToRemotePeer( &pSession->iceControllerContext,
                                                          pBuffer,
                                                          bufferLength );
    if( resultIceController != ICE_CONTROLLER_RESULT_OK )
    {
        LogWarn( ( "Fail to send DTLS packet, ret: %d", resultIceController ) );
        ret = -1;
    }
    else
    {
        ret = bufferLength;
    }

    return ret;
}

static int32_t HandleIceEventCallback( void * pCustomContext,
                                       IceControllerCallbackEvent_t event,
                                       IceControllerCallbackContent_t * pEventMsg )
{
    int32_t ret = 0;
    PeerConnectionSession_t * pSession = ( PeerConnectionSession_t * ) pCustomContext;
    PeerConnectionIceLocalCandidate_t * pLocalCandidateReadyMsg = NULL;

    if( pCustomContext == NULL )
    {
        LogError( ( "Invalid input, pCustomContext: %p, pEventMsg: %p",
                    pCustomContext, pEventMsg ) );
        ret = -1;
    }
    else
    {
        LogDebug( ( "Receiving ICE event %d callback", event ) );
        switch( event )
        {
            case ICE_CONTROLLER_CB_EVENT_LOCAL_CANDIDATE_READY:
                if( pEventMsg != NULL )
                {
                    if( pSession->onIceCandidateReadyCallbackFunc != NULL )
                    {
                        pLocalCandidateReadyMsg = ( PeerConnectionIceLocalCandidate_t * ) &pEventMsg->iceControllerCallbackContent.localCandidateReadyMsg;
                        pSession->onIceCandidateReadyCallbackFunc( pSession->pOnLocalCandidateReadyCallbackCustomContext,
                                                                   pLocalCandidateReadyMsg );
                    }
                    else
                    {
                        LogError( ( "No proper callback function to handle local candidate ready message." ) );
                    }
                }
                else
                {
                    LogError( ( "Event message pointer must be valid in event: %d.", event ) );
                }
                break;
            case ICE_CONTROLLER_CB_EVENT_PROCESS_ICE_CANDIDATES_AND_PAIRS:
                ret = OnIceEventProcessIceCandidatesAndPairs( pSession );
                break;
            case ICE_CONTROLLER_CB_EVENT_PEER_TO_PEER_CONNECTION_FOUND:
                #if METRIC_PRINT_ENABLED
                Metric_StartEvent( METRIC_EVENT_PC_DTLS_HANDSHAKING );
                #endif
                /* Start DTLS handshaking. */
                ret = ExecuteDtlsHandshake( pSession );

                /* This must set after ExecuteDtlsHandshake, or the other thread might execute handshake earlier than expectation. */
                pSession->state = PEER_CONNECTION_SESSION_STATE_P2P_CONNECTION_FOUND;
                break;
            case ICE_CONTROLLER_CB_EVENT_PERIODIC_CONNECTION_CHECK:
                ret = OnIceEventPeriodicConnectionCheck( pSession );
                break;
            case ICE_CONTROLLER_CB_EVENT_ICE_CLOSING:
                ret = OnIceEventClosing( pSession );
                break;
            case ICE_CONTROLLER_CB_EVENT_ICE_CLOSED:
                ret = OnIceEventClosed( pSession );
                break;
            case ICE_CONTROLLER_CB_EVENT_ICE_CLOSE_NOTIFY:
                /* Trigger peer connection close flow because of ICE event. */
                ret = OnIceEventPeerConnectionClose( pSession );
                break;
            default:
                LogError( ( "Unknown event: %d",
                            event ) );
                ret = -2;
                break;
        }
    }

    return ret;
}

static int32_t InitDtlsSession( PeerConnectionSession_t * pSession, uint8_t isServer )
{
    int32_t ret = 0;
    DtlsTransportStatus_t xNetworkStatus = DTLS_SUCCESS;
    DtlsSession_t * pDtlsSession = NULL;

    if( pSession == NULL )
    {
        LogError( ( "Invalid input, pSession: %p", pSession ) );
        ret = -20;
    }

    if( ret == 0 )
    {
        /* Set the pParams member of the network context with desired transport. */
        pDtlsSession = &pSession->dtlsSession;
        pDtlsSession->xNetworkContext.pParams = &pDtlsSession->xDtlsTransportParams;

        /* Assign transport send callback function/context for different connection types. */
        memset( &pDtlsSession->xDtlsTransportParams, 0, sizeof( DtlsTransportParams_t ) );
        pDtlsSession->xDtlsTransportParams.onDtlsSendHook = OnDtlsSendHook;
        pDtlsSession->xDtlsTransportParams.pOnDtlsSendCustomContext = ( void * ) pSession;

        // /* Set the network credentials. */
        /* Disable SNI server name indication*/
        // https://mbed-tls.readthedocs.io/en/latest/kb/how-to/use-sni/
        pDtlsSession->xNetworkCredentials.disableSni = pdTRUE;

        pDtlsSession->isServer = isServer;
    }

    if( ret == 0 )
    {
        if( NULL == pSession->pCtx->dtlsContext.localCert.raw.p )
        {
            LogError( ( "Fail to get answer cert: NULL == pSession->pCtx->dtlsContext.localCert.raw.p" ) );
            ret = -23;
        }
        else
        {
            /* Assign local cert to the DTLS session. */
            pDtlsSession->xNetworkCredentials.pClientCert = &pSession->pCtx->dtlsContext.localCert;

            // /* Assign local key to the DTLS session. */
            pDtlsSession->xNetworkCredentials.pPrivateKey = &pSession->pCtx->dtlsContext.localKey;

            /* Attempt to create a DTLS connection. */
            xNetworkStatus = DTLS_Init( &pDtlsSession->xNetworkContext,
                                        &pDtlsSession->xNetworkCredentials,
                                        isServer );

            if( xNetworkStatus != DTLS_SUCCESS )
            {
                LogError( ( "Fail to initialize the DTLS session with return %d ", xNetworkStatus ) );
                ret = -24;
            }
        }
    }

    if( ret == 0 )
    {
        pSession->dtlsHandshakingTimeoutMs = ( NetworkingUtils_GetCurrentTimeUs( NULL ) / 1000 ) + PEER_CONNECTION_DTLS_HANDSHAKING_TIMEOUT_MS;
    }

    return ret;
}

static int32_t ExecuteDtlsHandshake( PeerConnectionSession_t * pSession )
{
    int32_t ret = 0;
    DtlsTransportStatus_t xNetworkStatus = DTLS_SUCCESS;
    DtlsSession_t * pDtlsSession = NULL;

    if( pSession == NULL )
    {
        LogError( ( "Invalid input, pSession: %p", pSession ) );
        ret = -30;
    }

    if( ret == 0 )
    {
        pDtlsSession = &pSession->dtlsSession;

        /* Trigger the DTLS handshaking to send client hello if necessary. */
        xNetworkStatus = DTLS_ExecuteHandshake( &pDtlsSession->xNetworkContext );

        if( xNetworkStatus == DTLS_HANDSHAKE_COMPLETE )
        {
            ret = OnDtlsHandshakeComplete( pSession );
        }
        else if( ( xNetworkStatus != DTLS_SUCCESS ) &&
                 ( xNetworkStatus != DTLS_HANDSHAKE_ALREADY_COMPLETE ) )
        {
            LogError( ( "Error happens when executing DTLS handshake, return %d", xNetworkStatus ) );
            ret = -31;
        }
        else
        {
            /* This condition means DTLS handshaking is not complete yet. Wait for Rx packet. */
        }
    }

    /* Check DTLS handshaking timeout. */
    if( ret == 0 )
    {
        if( ( NetworkingUtils_GetCurrentTimeUs( NULL ) / 1000 ) > pSession->dtlsHandshakingTimeoutMs )
        {
            LogError( ( "DTLS handshaking timeout, start closing flow." ) );
            PeerConnection_CloseSession( pSession );
        }
    }

    return ret;
}

static void PeerConnection_SetTimer( PeerConnectionSession_t * pSession )
{
    uint8_t i;
    TimerControllerResult_t retTimer = TIMER_CONTROLLER_RESULT_OK;

    for( i = 0; i < PEER_CONNECTION_TRANSCEIVER_MAX_COUNT; i++ )
    {
        if( pSession->pTransceivers[ i ]->trackKind == TRANSCEIVER_TRACK_KIND_AUDIO )
        {
            retTimer = TimerController_IsTimerSet( &pSession->rtcpAudioSenderReportTimer );
            if( retTimer == TIMER_CONTROLLER_RESULT_NOT_SET )
            {
                /* The timer is not set before, send the request immendiately and start rtcp audio Sender Report timer. */
                LogDebug( ( "Trigger rtcp audio Sender Report timer." ) );
                retTimer = TimerController_SetTimer( &pSession->rtcpAudioSenderReportTimer,
                                                     PEER_CONNECTION_RTCP_REPORT_TIMER_INTERVAL_MS + ( rand() % 200 ),
                                                     PEER_CONNECTION_RTCP_REPORT_TIMER_INTERVAL_MS + ( rand() % 200 ) );
                if( retTimer != TIMER_CONTROLLER_RESULT_OK )
                {
                    LogError( ( "Fail to start RTCP Audio Sender Report timer, result: %d", retTimer ) );
                }
            }
        }
        else if( pSession->pTransceivers[ i ]->trackKind == TRANSCEIVER_TRACK_KIND_VIDEO )
        {
            retTimer = TimerController_IsTimerSet( &pSession->rtcpVideoSenderReportTimer );
            if( retTimer == TIMER_CONTROLLER_RESULT_NOT_SET )
            {
                /* The timer is not set before, send the request immendiately and start rtcp video Sender Report timer. */
                LogDebug( ( "Trigger rtcp video Sender Report timer." ) );
                retTimer = TimerController_SetTimer( &pSession->rtcpVideoSenderReportTimer,
                                                     PEER_CONNECTION_RTCP_REPORT_TIMER_INTERVAL_MS + ( rand() % 200 ),
                                                     PEER_CONNECTION_RTCP_REPORT_TIMER_INTERVAL_MS + ( rand() % 200 ) );
                if( retTimer != TIMER_CONTROLLER_RESULT_OK )
                {
                    LogError( ( "Fail to start RTCP Video Sender Report timer, result: %d", retTimer ) );
                }
            }
        }
        else
        {
            /* Do Nothing, Coverity Happy. */
        }
    }
}

static int32_t OnDtlsHandshakeComplete( PeerConnectionSession_t * pSession )
{
    int32_t ret = 0;
    DtlsTransportStatus_t xNetworkStatus = DTLS_SUCCESS;
    PeerConnectionResult_t retPc;
    uint32_t i;

    LogDebug( ( "Complete DTLS handshaking." ) );
    #if METRIC_PRINT_ENABLED
    Metric_EndEvent( METRIC_EVENT_PC_DTLS_HANDSHAKING );
    #endif

    /* Verify remote fingerprint (if remote cert fingerprint is the expected one) */
    xNetworkStatus = DTLS_VerifyRemoteCertificateFingerprint( &pSession->dtlsSession.xNetworkContext.pParams->dtlsSslContext,
                                                              pSession->remoteCertFingerprint,
                                                              pSession->remoteCertFingerprintLength );

    if( xNetworkStatus != DTLS_SUCCESS )
    {
        LogError( ( "Fail to DTLS_VerifyRemoteCertificateFingerprint with return %d ", xNetworkStatus ) );
        ret = -0x1001;
    }

    if( ret == 0 )
    {
        /* Retrieve key material into DTLS session. */
        xNetworkStatus = DTLS_PopulateKeyingMaterial( &pSession->dtlsSession.xNetworkContext.pParams->dtlsSslContext,
                                                      &pSession->dtlsSession.xNetworkCredentials.dtlsKeyingMaterial );

        if( xNetworkStatus != DTLS_SUCCESS )
        {
            LogError( ( "Fail to DTLS_PopulateKeyingMaterial with return %d ", xNetworkStatus ) );
            ret = -0x1002;
        }
        else
        {
            LogDebug( ( "DTLS_PopulateKeyingMaterial with key_length: %i ", pSession->dtlsSession.xNetworkCredentials.dtlsKeyingMaterial.key_length ) );
        }
    }

    if( ret == 0 )
    {
        /* Initialize SRTP sessions. */
        retPc = PeerConnectionSrtp_Init( pSession );
        if( retPc != PEER_CONNECTION_RESULT_OK )
        {
            LogError( ( "Fail to create SRTP sessions, ret: %d", retPc ) );
            ret = -0x1003;
        }
    }

    #if ENABLE_SCTP_DATA_CHANNEL
    if( ret == 0 )
    {
        /* Initialize SCTP sessions. */
        if( pSession->ucEnableDataChannelRemote == 1 )
        {
            retPc = PeerConnectionSCTP_AllocateSCTP( pSession );
            if( retPc != PEER_CONNECTION_RESULT_OK )
            {
                LogError( ( "Fail to create SCTP sessions, ret: %d", retPc ) );
                ret = -0x1004;
            }
        }
    }
    #endif /* ENABLE_SCTP_DATA_CHANNEL */

    if( ret == 0 )
    {
        PeerConnection_SetTimer( pSession );
    }

    if( ret == 0 )
    {
        pSession->state = PEER_CONNECTION_SESSION_STATE_CONNECTION_READY;
        pSession->inactiveConnectionTimeoutMs = ( NetworkingUtils_GetCurrentTimeUs( NULL ) / 1000 ) + PEER_CONNECTION_INACTIVE_CONNECTION_TIMEOUT_MS;
        for( i = 0; i < pSession->transceiverCount; i++ )
        {
            if( pSession->pTransceivers[i]->onPcEventCallbackFunc )
            {
                pSession->pTransceivers[i]->onPcEventCallbackFunc( pSession->pTransceivers[i]->pOnPcEventCustomContext,
                                                                   TRANSCEIVER_CB_EVENT_REMOTE_PEER_READY,
                                                                   NULL );
            }
        }
    }

    if( ret == 0 )
    {
        IceController_HandleEvent( &pSession->iceControllerContext,
                                   ICE_CONTROLLER_EVENT_DTLS_HANDSHAKE_DONE );
    }

    if( ret != 0 )
    {
        PeerConnection_CloseSession( pSession );
    }

    return ret;
}

static int32_t ProcessDtlsPacket( PeerConnectionSession_t * pSession,
                                  uint8_t * pDtlsEncryptData,
                                  size_t dtlsEncryptDataLength )
{
    int32_t ret = 0;
    DtlsTransportStatus_t xNetworkStatus = DTLS_SUCCESS;
    uint8_t dtlsDecryptBuffer[ PEER_CONNECTION_MAX_DTLS_DECRYPTED_DATA_LENGTH ];
    size_t dtlsDecryptBufferLength = PEER_CONNECTION_MAX_DTLS_DECRYPTED_DATA_LENGTH;

    xNetworkStatus = DTLS_ProcessPacket( &pSession->dtlsSession.xNetworkContext,
                                         pDtlsEncryptData,
                                         dtlsEncryptDataLength,
                                         dtlsDecryptBuffer,
                                         &dtlsDecryptBufferLength );

    if( xNetworkStatus == DTLS_HANDSHAKE_COMPLETE )
    {
        ret = OnDtlsHandshakeComplete( pSession );
    }
    else if( xNetworkStatus == DTLS_SUCCESS )
    {
        #if ENABLE_SCTP_DATA_CHANNEL
        {
            if( pSession->state == PEER_CONNECTION_SESSION_STATE_CONNECTION_READY )
            {
                PeerConnectionSCTP_ProcessSCTPData( pSession,
                                                    dtlsDecryptBuffer,
                                                    dtlsDecryptBufferLength );
            }
        }
        #endif /* ENABLE_SCTP_DATA_CHANNEL */
    }
    else
    {
        LogInfo( ( "Error happens when process the DTLS packet, return %d", xNetworkStatus ) );
        ret = HandleDtlsTermination( pSession );
    }

    return ret;
}

static int32_t HandleNonStunPackets( void * pCustomContext,
                                     uint8_t * pBuffer,
                                     size_t bufferLength )
{
    int32_t ret = 0;
    PeerConnectionSession_t * pSession = ( PeerConnectionSession_t * ) pCustomContext;
    PeerConnectionResult_t resultPeerConnection;

    if( ( pCustomContext == NULL ) || ( pBuffer == NULL ) )
    {
        LogWarn( ( "Invalid input, pCustomContext: %p, pBuffer: %p",
                   pCustomContext, pBuffer ) );
        ret = -1;
    }

    if( ret == 0 )
    {
        if( bufferLength < 2 )
        {
            LogWarn( ( "Invalid buffer length: %u", bufferLength ) );
            ret = -2;
        }
        else if( ( pBuffer[0] > 127 ) && ( pBuffer[0] < 192 ) )
        {
            if( ( pBuffer[1] >= 192 ) && ( pBuffer[1] <= 223 ) )
            {
                /* RTCP packet */
                resultPeerConnection = PeerConnectionSrtp_HandleSrtcpPacket( pSession,
                                                                             pBuffer,
                                                                             bufferLength );
                if( resultPeerConnection != PEER_CONNECTION_RESULT_OK )
                {
                    LogWarn( ( "Failed to handle SRTCP packets, result: %d", resultPeerConnection ) );
                    ret = -2;
                }
                else
                {
                    pSession->inactiveConnectionTimeoutMs = ( NetworkingUtils_GetCurrentTimeUs( NULL ) / 1000 ) + PEER_CONNECTION_INACTIVE_CONNECTION_TIMEOUT_MS;
                }
            }
            else
            {
                /* RTP packet */
                resultPeerConnection = PeerConnectionSrtp_HandleSrtpPacket( pSession,
                                                                            pBuffer,
                                                                            bufferLength );
                if( resultPeerConnection != PEER_CONNECTION_RESULT_OK )
                {
                    LogWarn( ( "Failed to handle SRTP packets, result: %d", resultPeerConnection ) );
                    ret = -2;
                }
                else
                {
                    pSession->inactiveConnectionTimeoutMs = ( NetworkingUtils_GetCurrentTimeUs( NULL ) / 1000 ) + PEER_CONNECTION_INACTIVE_CONNECTION_TIMEOUT_MS;
                }
            }
        }
        else if( ( pBuffer[0] > 19 ) && ( pBuffer[0] < 64 ) )
        {
            /* Trigger the DTLS handshaking to send client hello if necessary
             * and process incoming DTLS data by forwarding to respective
             * libraries to process. */
            LogDebug( ( "Receiving %u bytes DTLS packet.", bufferLength ) );
            ret = ProcessDtlsPacket( pSession,
                                     pBuffer,
                                     bufferLength );
        }
        else
        {
            LogWarn( ( "drop unknown DTLS packet, length=%u, first byte=0x%02x",
                       bufferLength,
                       pBuffer[ 0 ] ) );
        }
    }

    return ret;
}

/* Generate a printable string that does not
 * need to be escaped when encoding in JSON
 */
static void generateJSONValidString( char * pDst,
                                     size_t length )
{
    size_t i = 0;
    uint8_t skipProcess = 0;
    const char jsonCharSet[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz+/";
    const size_t jsonCharSetLength = strlen( jsonCharSet );

    if( pDst == NULL )
    {
        skipProcess = 1;
    }

    if( skipProcess == 0 )
    {
        for( i = 0; i < length; i++ )
        {
            pDst[i] = jsonCharSet[ rand() % jsonCharSetLength ];
        }
    }
}

static PeerConnectionResult_t InitializeIceController( PeerConnectionSession_t * pSession,
                                                       PeerConnectionSessionConfiguration_t * pSessionConfig )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    IceControllerResult_t iceControllerResult;
    IceControllerIceServerConfig_t iceServerConfig;
    IceControllerInitConfig_t initConfig;

    if( pSession == NULL )
    {
        LogError( ( "Invalid input, pSession: %p", pSession ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        memset( &initConfig, 0, sizeof( IceControllerInitConfig_t ) );
        initConfig.natTraversalConfigBitmap = pSessionConfig->natTraversalConfigBitmap;
        initConfig.onIceEventCallbackFunc = HandleIceEventCallback;
        initConfig.pOnIceEventCallbackContext = pSession;
        initConfig.onRecvNonStunPacketFunc = HandleNonStunPackets;
        initConfig.pOnRecvNonStunPacketCallbackContext = pSession;
        iceControllerResult = IceController_Init( &pSession->iceControllerContext,
                                                  &initConfig );
        if( iceControllerResult != ICE_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Fail to initialize Ice Controller." ) );
            ret = PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_INIT;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        memset( &iceServerConfig, 0, sizeof( IceControllerIceServerConfig_t ) );
        iceServerConfig.pIceServers = pSessionConfig->iceServers;
        iceServerConfig.iceServersCount = pSessionConfig->iceServersCount;
        iceServerConfig.pRootCaPath = pSessionConfig->pRootCaPath;
        iceServerConfig.rootCaPathLength = pSessionConfig->rootCaPathLength;
        iceServerConfig.pRootCaPem = pSessionConfig->pRootCaPem;
        iceServerConfig.rootCaPemLength = pSessionConfig->rootCaPemLength;
        iceControllerResult = IceController_AddIceServerConfig( &pSession->iceControllerContext,
                                                                &iceServerConfig );
        if( iceControllerResult != ICE_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Fail to add Ice server config into Ice Controller." ) );
            ret = PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_ADD_ICE_SERVER_CONFIG;
        }
    }

    return ret;
}

static PeerConnectionResult_t DestroyIceController( PeerConnectionSession_t * pSession )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    IceControllerResult_t iceControllerResult;

    if( pSession == NULL )
    {
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        iceControllerResult = IceController_Destroy( &pSession->iceControllerContext );
        if( iceControllerResult != ICE_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Fail to destroy Ice Controller." ) );
            ret = PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_DESTROY;
        }
    }

    return ret;
}

static PeerConnectionResult_t PeerConnection_ResetTimer( PeerConnectionSession_t * pSession )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    TimerControllerResult_t timerControllerResult = TIMER_CONTROLLER_RESULT_OK;

    if( pSession == NULL )
    {
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        timerControllerResult = TimerController_IsTimerSet( &pSession->rtcpAudioSenderReportTimer );

        if( timerControllerResult == TIMER_CONTROLLER_RESULT_SET )
        {
            TimerController_Reset( &pSession->rtcpAudioSenderReportTimer );
            LogDebug( ( "Reset RTCP Audio sender report timer." ) );
        }
        else if( timerControllerResult == TIMER_CONTROLLER_RESULT_NOT_SET )
        {
            /* Do Nothing */
        }
        else
        {
            LogError( ( "Fail to reset RTCP Audio sender report timer." ) );
            ret = PEER_CONNECTION_RESULT_FAIL_TIMER_RESET;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        timerControllerResult = TimerController_IsTimerSet( &pSession->rtcpVideoSenderReportTimer );

        if( timerControllerResult == TIMER_CONTROLLER_RESULT_SET )
        {
            TimerController_Reset( &pSession->rtcpVideoSenderReportTimer );
            LogDebug( ( "Reset RTCP Video sender report timer." ) );
        }
        else if( timerControllerResult == TIMER_CONTROLLER_RESULT_NOT_SET )
        {
            /* Do Nothing */
        }
        else
        {
            LogError( ( "Fail to reset RTCP Video sender report timer." ) );
            ret = PEER_CONNECTION_RESULT_FAIL_TIMER_RESET;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        timerControllerResult = TimerController_IsTimerSet( &pSession->closeSessionTimer );

        if( timerControllerResult == TIMER_CONTROLLER_RESULT_SET )
        {
            TimerController_Reset( &pSession->closeSessionTimer );
            LogDebug( ( "Reset close session timer." ) );
        }
        else if( timerControllerResult == TIMER_CONTROLLER_RESULT_NOT_SET )
        {
            /* Do Nothing */
        }
        else
        {
            LogError( ( "Fail to reset close session timer." ) );
            ret = PEER_CONNECTION_RESULT_FAIL_TIMER_RESET;
        }
    }

    return ret;
}

static PeerConnectionResult_t AllocateTransceiver( PeerConnectionSession_t * pSession,
                                                   Transceiver_t * pTransceiver )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( pSession == NULL )
    {
        LogError( ( "Invalid input, pSession: %p", pSession ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( pSession->transceiverCount < PEER_CONNECTION_TRANSCEIVER_MAX_COUNT )
        {
            pTransceiver->ssrc = ( uint32_t ) rand();
            pTransceiver->rtxSsrc = ( uint32_t ) rand();
            pSession->pTransceivers[ pSession->transceiverCount ] = pTransceiver;
            pSession->transceiverCount++;
        }
        else
        {
            LogWarn( ( "No space to add transceiver" ) );
            ret = PEER_CONNECTION_RESULT_NO_FREE_TRANSCEIVER;
        }
    }

    return ret;
}

static PeerConnectionResult_t InitializeDtlsContext( PeerConnectionDtlsContext_t * pDtlsContext )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    DtlsTransportStatus_t xNetworkStatus = DTLS_SUCCESS;

    if( pDtlsContext == NULL )
    {
        LogError( ( "Invalid input, pDtlsContext: %p", pDtlsContext ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    /* Generate local cert in DER format. */
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        xNetworkStatus = DTLS_CreateCertificateAndKey( GENERATED_CERTIFICATE_BITS,
                                                       pdFALSE,
                                                       &pDtlsContext->localCert,
                                                       &pDtlsContext->localKey );
        if( xNetworkStatus != DTLS_SUCCESS )
        {
            LogError( ( "Fail to DTLS_CreateCertificateAndKey, return %d", xNetworkStatus ) );
            ret = PEER_CONNECTION_RESULT_FAIL_CREATE_CERT_AND_KEY;
        }
    }

    // Generate cert fingerprint
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        xNetworkStatus = DTLS_CreateCertificateFingerprint( &pDtlsContext->localCert,
                                                            pDtlsContext->localCertFingerprint,
                                                            CERTIFICATE_FINGERPRINT_LENGTH );
        if( xNetworkStatus != DTLS_SUCCESS )
        {
            LogError( ( "Fail to dtlsCertificateFingerprint answer cert, return %d", xNetworkStatus ) );
            ret = PEER_CONNECTION_RESULT_FAIL_CREATE_CERT_FINGERPRINT;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        pDtlsContext->isInitialized = 1;
    }

    return ret;
}

static PeerConnectionResult_t GetDefaultCodec( uint32_t codecBitMap,
                                               uint32_t * pOutputCodec )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( TRANSCEIVER_IS_CODEC_ENABLED( codecBitMap,
                                      TRANSCEIVER_RTC_CODEC_H264_PROFILE_42E01F_LEVEL_ASYMMETRY_ALLOWED_PACKETIZATION_BIT ) )
    {
        *pOutputCodec = TRANSCEIVER_RTC_CODEC_DEFAULT_PAYLOAD_H264;
    }
    else if( TRANSCEIVER_IS_CODEC_ENABLED( codecBitMap,
                                           TRANSCEIVER_RTC_CODEC_OPUS_BIT ) )
    {
        *pOutputCodec = TRANSCEIVER_RTC_CODEC_DEFAULT_PAYLOAD_OPUS;
    }
    else if( TRANSCEIVER_IS_CODEC_ENABLED( codecBitMap,
                                           TRANSCEIVER_RTC_CODEC_VP8_BIT ) )
    {
        *pOutputCodec = TRANSCEIVER_RTC_CODEC_DEFAULT_PAYLOAD_VP8;
    }
    else if( TRANSCEIVER_IS_CODEC_ENABLED( codecBitMap,
                                           TRANSCEIVER_RTC_CODEC_MULAW_BIT ) )
    {
        *pOutputCodec = TRANSCEIVER_RTC_CODEC_DEFAULT_PAYLOAD_MULAW;
    }
    else if( TRANSCEIVER_IS_CODEC_ENABLED( codecBitMap,
                                           TRANSCEIVER_RTC_CODEC_ALAW_BIT ) )
    {
        *pOutputCodec = TRANSCEIVER_RTC_CODEC_DEFAULT_PAYLOAD_ALAW;
    }
    else if( TRANSCEIVER_IS_CODEC_ENABLED( codecBitMap,
                                           TRANSCEIVER_RTC_CODEC_H265_BIT ) )
    {
        *pOutputCodec = TRANSCEIVER_RTC_CODEC_DEFAULT_PAYLOAD_H265;
    }
    else
    {
        ret = PEER_CONNECTION_RESULT_UNKNOWN_CODEC;
        LogError( ( "No default codec found." ) );
    }

    return ret;
}

static PeerConnectionResult_t SetDefaultPayloadTypes( PeerConnectionSession_t * pSession )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    int i;
    const Transceiver_t * pTransceiver = NULL;

    LogInfo( ( "Setting default payload types for session, transceiverCount: %lu",
               pSession->transceiverCount ) );
    for( i = 0; i < pSession->transceiverCount && ret == PEER_CONNECTION_RESULT_OK; i++ )
    {
        pTransceiver = pSession->pTransceivers[i];

        if( pTransceiver == NULL )
        {
            LogError( ( "This is not expected, keep this condition here to avoid NULL accessing, transceiverCount: %lu, current index: %d",
                        pSession->transceiverCount,
                        i ) );
            ret = PEER_CONNECTION_RESULT_UNKNOWN_TRANSCEIVER;
            break;
        }
        else if( pTransceiver->trackKind == TRANSCEIVER_TRACK_KIND_VIDEO )
        {
            pSession->rtpConfig.isVideoCodecPayloadSet = 1;
            pSession->rtpConfig.videoCodecRtxPayload = 0;
            pSession->rtpConfig.videoRtxSequenceNumber = 0;
            pSession->rtpConfig.videoSequenceNumber = 0;
            ret = GetDefaultCodec( pTransceiver->codecBitMap,
                                   &pSession->rtpConfig.videoCodecPayload );
        }
        else
        {
            pSession->rtpConfig.isAudioCodecPayloadSet = 1;
            pSession->rtpConfig.audioCodecRtxPayload = 0;
            pSession->rtpConfig.audioRtxSequenceNumber = 0;
            pSession->rtpConfig.audioSequenceNumber = 0;
            ret = GetDefaultCodec( pTransceiver->codecBitMap,
                                   &pSession->rtpConfig.audioCodecPayload );
        }
    }

    return ret;
}

PeerConnectionResult_t PeerConnection_Init( PeerConnectionSession_t * pSession,
                                            PeerConnectionSessionConfiguration_t * pSessionConfig )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    #if ENABLE_TWCC_SUPPORT
    RtcpTwccManagerResult_t resultRtcpTwccManager = RTCP_TWCC_MANAGER_RESULT_OK;
    #endif
    MessageQueueResult_t retMessageQueue;
    TimerControllerResult_t retTimer;
    char tempName[ 20 ];
    static uint8_t initSeq = 0;

    /* Avoid unused variable warning. */
    ( void ) pSessionConfig;

    if( ( pSession == NULL ) || ( pSessionConfig == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pSessionConfig: %p", pSession, pSessionConfig ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ( ret == PEER_CONNECTION_RESULT_OK ) &&
        ( peerConnectionContext.isInited == 0U ) )
    {
        peerConnectionContext.isInited = 1U;

        generateJSONValidString( peerConnectionContext.localUserName,
                                 PEER_CONNECTION_USER_NAME_LENGTH );
        peerConnectionContext.localUserName[ PEER_CONNECTION_USER_NAME_LENGTH ] = '\0';
        generateJSONValidString( peerConnectionContext.localPassword,
                                 PEER_CONNECTION_PASSWORD_LENGTH );
        peerConnectionContext.localPassword[ PEER_CONNECTION_PASSWORD_LENGTH ] = '\0';
        generateJSONValidString( peerConnectionContext.localCname,
                                 PEER_CONNECTION_CNAME_LENGTH );
        peerConnectionContext.localCname[ PEER_CONNECTION_CNAME_LENGTH ] = '\0';

        /* Generate answer cert in DER format */
        if( peerConnectionContext.dtlsContext.isInitialized == 0 )
        {
            /* pCtx->dtlsContext.isInitialized would be set to 1 in InitializeDtlsContext(). */
            ret = InitializeDtlsContext( &peerConnectionContext.dtlsContext );
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        memset( pSession,
                0,
                sizeof( PeerConnectionSession_t ) );

        /* Initialize request queue. */
        ( void ) snprintf( tempName,
                           sizeof( tempName ),
                           "%s%02d",
                           PEER_CONNECTION_MESSAGE_QUEUE_NAME,
                           initSeq );

        /* Delete message queue from previous round. */
        MessageQueue_Destroy( NULL,
                              tempName );

        retMessageQueue = MessageQueue_Create( &pSession->requestQueue,
                                               tempName,
                                               sizeof( PeerConnectionSessionRequestMessage_t ),
                                               PEER_CONNECTION_MAX_QUEUE_MSG_NUM );
        if( retMessageQueue != MESSAGE_QUEUE_RESULT_OK )
        {
            LogError( ( "Fail to open message queue" ) );
            ret = PEER_CONNECTION_RESULT_FAIL_MQ_INIT;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        pSession->srtpSessionMutex = xSemaphoreCreateMutex();
        if( pSession->srtpSessionMutex == NULL )
        {
            LogError( ( "Fail to create mutex of SRTP session." ) );
            ret = PEER_CONNECTION_RESULT_FAIL_CREATE_SRTP_MUTEX;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        pSession->startupBarrier = xEventGroupCreate();
        if( pSession->startupBarrier == NULL )
        {
            LogError( ( "Fail to create start up barrier." ) );
            ret = PEER_CONNECTION_RESULT_FAIL_CREATE_STARTUP_BARRIER;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Initialize session task. */
        ( void ) snprintf( tempName,
                           sizeof( tempName ),
                           "%s%02d",
                           PEER_CONNECTION_SESSION_TASK_NAME,
                           initSeq );

        if( xTaskCreate( PeerConnection_SessionTask,
                         tempName,
                         4096,
                         pSession,
                         tskIDLE_PRIORITY + 4,
                         pSession->pTaskHandler ) != pdPASS )
        {
            LogError( ( "xTaskCreate(%s) failed", tempName ) );
            ret = PEER_CONNECTION_RESULT_FAIL_CREATE_TASK_ICE_CONTROLLER;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Initialize other modules. */
        ret = InitializeIceController( pSession,
                                       pSessionConfig );
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        ( void ) snprintf( tempName,
                           sizeof( tempName ),
                           "%s%02d",
                           PEER_CONNECTION_SESSION_RX_TASK_NAME,
                           initSeq++ );
        if( xTaskCreate( IceControllerSocketListener_Task,
                         tempName,
                         4096,
                         &pSession->iceControllerContext,
                         tskIDLE_PRIORITY + 5,
                         NULL ) != pdPASS )
        {
            LogError( ( "xTaskCreate(%s) failed", tempName ) );
            ret = PEER_CONNECTION_RESULT_FAIL_CREATE_TASK_ICE_SOCK_LISTENER;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        pSession->state = PEER_CONNECTION_SESSION_STATE_INITED;
        pSession->pCtx = &peerConnectionContext;
    }

    #if ENABLE_TWCC_SUPPORT
        if( ret == PEER_CONNECTION_RESULT_OK )
        {
            resultRtcpTwccManager = RtcpTwccManager_Init( &pSession->pCtx->rtcpTwccManager,
                                                          pSession->pCtx->twccPacketInfo,
                                                          PEER_CONNECTION_RTCP_TWCC_MAX_ARRAY );
            if( resultRtcpTwccManager != RTCP_TWCC_MANAGER_RESULT_OK )
            {
                LogError( ( "Fail to Initialize RTCP TWCC Manager, result: %d", resultRtcpTwccManager ) );
                ret = PEER_CONNECTION_RESULT_FAIL_RTCP_TWCC_INIT;
            }
        }

        if( ret == PEER_CONNECTION_RESULT_OK )
        {
            pSession->twccMetaData.twccBitrateMutex = xSemaphoreCreateMutex();
            if( pSession->twccMetaData.twccBitrateMutex == NULL )
            {
                LogError( ( "Fail to create mutex for TWCC." ) );
                ret = PEER_CONNECTION_RESULT_FAIL_CREATE_TWCC_MUTEX;
            }
        }
    #endif /* ENABLE_TWCC_SUPPORT */

    /* Initialize timer for audio Sender Reports. */
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        retTimer = TimerController_Create( &pSession->rtcpAudioSenderReportTimer,
                                           PEER_CONNECTION_AUDIO_TIMER_NAME,
                                           PEER_CONNECTION_RTCP_REPORT_TIMER_INTERVAL_MS,
                                           PEER_CONNECTION_RTCP_REPORT_TIMER_INTERVAL_MS,
                                           OnRtcpSenderReportAudioTimerExpire,
                                           pSession );
        if( retTimer != TIMER_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Audio RTCP TimerController_Create return fail, result: %d", retTimer ) );
            ret = PEER_CONNECTION_RESULT_FAIL_TIMER_INIT;
        }
    }
    /* Initialize timer for video Sender Reports. */
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        retTimer = TimerController_Create( &pSession->rtcpVideoSenderReportTimer,
                                           PEER_CONNECTION_VIDEO_TIMER_NAME,
                                           PEER_CONNECTION_RTCP_REPORT_TIMER_INTERVAL_MS + 50,
                                           PEER_CONNECTION_RTCP_REPORT_TIMER_INTERVAL_MS,
                                           OnRtcpSenderReportVideoTimerExpire,
                                           pSession );
        if( retTimer != TIMER_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Video RTCP TimerController_Create return fail, result: %d", retTimer ) );
            ret = PEER_CONNECTION_RESULT_FAIL_TIMER_INIT;
        }
    }

    /* Initialize timer for close peer connection session. */
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        retTimer = TimerController_Create( &pSession->closeSessionTimer,
                                           PEER_CONNECTION_CLOSE_SESSION_TIMER_NAME,
                                           PEER_CONNECTION_WAIT_SDP_MESSAGE_TIMEOUT_MS,
                                           0U,
                                           OnCloseSessionTimerExpire,
                                           pSession );
        if( retTimer != TIMER_CONTROLLER_RESULT_OK )
        {
            LogError( ( "TimerController_Create return fail, result: %d", retTimer ) );
            ret = PEER_CONNECTION_RESULT_FAIL_TIMER_INIT;
        }
    }

    return ret;
}

PeerConnectionResult_t PeerConnection_Start( PeerConnectionSession_t * pSession )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( pSession == NULL )
    {
        LogError( ( "Invalid input, pSession: %p", pSession ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Clear all message queue because of new session is coming. */
        EmptyMessageQueue( &pSession->requestQueue );
        pSession->state = PEER_CONNECTION_SESSION_STATE_START;
    }

    return ret;
}

PeerConnectionResult_t PeerConnection_SetLocalDescription( PeerConnectionSession_t * pSession,
                                                           const PeerConnectionBufferSessionDescription_t * pBufferSessionDescription )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( ( pSession == NULL ) ||
        ( pBufferSessionDescription == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pBufferSessionDescription: %p",
                    pSession,
                    pBufferSessionDescription ) );
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* TODO: store configurations into session. */
    }

    return ret;
}

PeerConnectionResult_t PeerConnection_SetRemoteDescription( PeerConnectionSession_t * pSession,
                                                            const PeerConnectionBufferSessionDescription_t * pBufferSessionDescription )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    IceControllerResult_t iceControllerResult;
    RtpResult_t resultRtp;
    RtcpResult_t resultRtcp;
    PeerConnectionBufferSessionDescription_t * pTargetRemoteSdp = NULL;
    uint8_t i;
    EventBits_t uxBits = 0U;
    int32_t retDtls = 0;
    IceControllerStartConfig_t iceStartConfig;

    if( ( pSession == NULL ) ||
        ( pBufferSessionDescription == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pBufferSessionDescription: %p", pSession, pBufferSessionDescription ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else if( ( pBufferSessionDescription->pSdpBuffer == NULL ) ||
             ( pBufferSessionDescription->sdpBufferLength > PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH ) )
    {
        LogError( ( "Invalid input, pBufferSessionDescription->pSdpBuffer: %p, pBufferSessionDescription->sdpBufferLength: %u",
                    pBufferSessionDescription->pSdpBuffer, pBufferSessionDescription->sdpBufferLength ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else
    {
        /* Empty else marker. */
    }

    /* Use SDP controller to parse SDP message into data structure. */
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        pTargetRemoteSdp = &pSession->remoteSessionDescription;
        memset( pTargetRemoteSdp,
                0,
                sizeof( PeerConnectionBufferSessionDescription_t ) );
        pTargetRemoteSdp->pSdpBuffer = pSession->remoteSdpBuffer;
        pTargetRemoteSdp->sdpBufferLength = pBufferSessionDescription->sdpBufferLength;
        pTargetRemoteSdp->type = pBufferSessionDescription->type;
        memcpy( pTargetRemoteSdp->pSdpBuffer,
                pBufferSessionDescription->pSdpBuffer,
                pTargetRemoteSdp->sdpBufferLength );

        ret = PeerConnectionSdp_DeserializeSdpMessage( pTargetRemoteSdp );
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Update codec information based on transceivers. */
        ret = PeerConnectionSdp_SetPayloadTypes( pSession,
                                                 pTargetRemoteSdp );
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Validate deserialized SDP message. */
        if( ( pTargetRemoteSdp->sdpDescription.quickAccess.pIceUfrag == NULL ) ||
            ( pTargetRemoteSdp->sdpDescription.quickAccess.iceUfragLength + PEER_CONNECTION_USER_NAME_LENGTH > ( PEER_CONNECTION_USER_NAME_LENGTH << 1 ) ) )
        {
            LogWarn( ( "Remote user name is too long to store, length: %u", pTargetRemoteSdp->sdpDescription.quickAccess.iceUfragLength ) );
            ret = PEER_CONNECTION_RESULT_INVALID_REMOTE_USERNAME;
        }
        else if( ( pTargetRemoteSdp->sdpDescription.quickAccess.pIceUfrag == NULL ) ||
                 ( pTargetRemoteSdp->sdpDescription.quickAccess.icePwdLength > PEER_CONNECTION_PASSWORD_LENGTH ) )
        {
            LogWarn( ( "Remote user password is too long to store, length: %u", pTargetRemoteSdp->sdpDescription.quickAccess.icePwdLength ) );
            ret = PEER_CONNECTION_RESULT_INVALID_REMOTE_PASSWORD;
        }
        else
        {
            /* Empty else marker. */
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Follow the setup value of remote SDP message to configure local DTLS role. All possible values in setup
         * are 'active', 'actpass', and 'passive'. Follow rule in https://www.rfc-editor.org/rfc/rfc4572#section-6.2
         * to start DTLS server mode only if the remote side is using 'active' mode. */
        if( pTargetRemoteSdp->sdpDescription.quickAccess.dtlsRole == SDP_CONTROLLER_DTLS_ROLE_ACTIVE )
        {
            retDtls = InitDtlsSession( pSession, 1U );
        }
        else
        {
            retDtls = InitDtlsSession( pSession, 0U );
        }

        if( retDtls != 0 )
        {
            LogError( ( "Failed to initialize DTLS session, result: %ld", retDtls ) );
            ret = PEER_CONNECTION_RESULT_FAIL_INIT_DTLS_SESSION;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        memcpy( pSession->remoteUserName,
                pTargetRemoteSdp->sdpDescription.quickAccess.pIceUfrag,
                pTargetRemoteSdp->sdpDescription.quickAccess.iceUfragLength );
        pSession->remoteUserName[ pTargetRemoteSdp->sdpDescription.quickAccess.iceUfragLength ] = '\0';
        memcpy( pSession->remotePassword,
                pTargetRemoteSdp->sdpDescription.quickAccess.pIcePwd,
                pTargetRemoteSdp->sdpDescription.quickAccess.icePwdLength );
        pSession->remotePassword[ pTargetRemoteSdp->sdpDescription.quickAccess.icePwdLength ] = '\0';
        ( void ) snprintf( pSession->combinedName,
                           ( PEER_CONNECTION_USER_NAME_LENGTH << 1 ) + 2,
                           "%.*s:%.*s",
                           ( int ) pTargetRemoteSdp->sdpDescription.quickAccess.iceUfragLength,
                           pSession->remoteUserName,
                           PEER_CONNECTION_USER_NAME_LENGTH,
                           peerConnectionContext.localUserName );
        memcpy( pSession->remoteCertFingerprint,
                pTargetRemoteSdp->sdpDescription.quickAccess.pFingerprint,
                pTargetRemoteSdp->sdpDescription.quickAccess.fingerprintLength );
        pSession->remoteCertFingerprint[ pTargetRemoteSdp->sdpDescription.quickAccess.fingerprintLength ] = '\0';
        pSession->remoteCertFingerprintLength = pTargetRemoteSdp->sdpDescription.quickAccess.fingerprintLength;
        memset( &iceStartConfig, 0, sizeof( iceStartConfig ) );
        iceStartConfig.isControlling = pSession->dtlsSession.isServer != 0U? 1U:0U;
        iceStartConfig.pLocalUserName = &( peerConnectionContext.localUserName[ 0 ] );
        iceStartConfig.localUserNameLength = PEER_CONNECTION_USER_NAME_LENGTH;
        iceStartConfig.pLocalPassword = &( peerConnectionContext.localPassword[ 0 ] );
        iceStartConfig.localPasswordLength = PEER_CONNECTION_PASSWORD_LENGTH;
        iceStartConfig.pRemoteUserName = &( pSession->remoteUserName[ 0 ] );
        iceStartConfig.remoteUserNameLength = pTargetRemoteSdp->sdpDescription.quickAccess.iceUfragLength;
        iceStartConfig.pRemotePassword = &( pSession->remotePassword[ 0 ] );
        iceStartConfig.remotePasswordLength = pTargetRemoteSdp->sdpDescription.quickAccess.icePwdLength;
        iceStartConfig.pCombinedName = &( pSession->combinedName[ 0 ] );
        iceStartConfig.combinedNameLength = strlen( pSession->combinedName );

        iceControllerResult = IceController_Start( &pSession->iceControllerContext,
                                                   &iceStartConfig );
        if( iceControllerResult != ICE_CONTROLLER_RESULT_OK )
        {
            LogWarn( ( "IceController_Start fail, result: %d.", iceControllerResult ) );
            ret = PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_START;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        resultRtp = Rtp_Init( &peerConnectionContext.rtpContext );
        if( resultRtp != RTP_RESULT_OK )
        {
            LogError( ( "Fail to initialize RTP context, result: %d", resultRtp ) );
            ret = PEER_CONNECTION_RESULT_FAIL_RTP_INIT;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        resultRtcp = Rtcp_Init( &peerConnectionContext.rtcpContext );
        if( resultRtcp != RTCP_RESULT_OK )
        {
            LogError( ( "Fail to initialize RTCP context, result: %d", resultRtcp ) );
            ret = PEER_CONNECTION_RESULT_FAIL_RTCP_INIT;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        pSession->rtpConfig.videoRtxSequenceNumber = 0U;
        pSession->rtpConfig.audioRtxSequenceNumber = 0U;
        pSession->rtpConfig.twccId = ( uint16_t ) pTargetRemoteSdp->sdpDescription.quickAccess.twccExtId;
        pSession->rtpConfig.remoteVideoSsrc = pTargetRemoteSdp->sdpDescription.quickAccess.videoSsrc;
        pSession->rtpConfig.remoteAudioSsrc = pTargetRemoteSdp->sdpDescription.quickAccess.audioSsrc;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        pSession->state = PEER_CONNECTION_SESSION_STATE_FIND_CONNECTION;

        uxBits = xEventGroupSetBits( pSession->startupBarrier,
                                     PEER_CONNECTION_START_UP_BARRIER_BIT );
        /* Since we configure xClearBitOnExit while waiting, the return value might be cleared automatically.
         * Thus we don't check return value here. */
        ( void ) uxBits;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        LogVerbose( ( "Remote Candidates Count : %d", pTargetRemoteSdp->sdpDescription.quickAccess.remoteCandidateCount ) );
        if( pTargetRemoteSdp->sdpDescription.quickAccess.remoteCandidateCount != 0 )
        {
            for( i = 0; i < pTargetRemoteSdp->sdpDescription.quickAccess.remoteCandidateCount; i++ )
            {
                ret = PeerConnection_AddRemoteCandidate( pSession,
                                                         pTargetRemoteSdp->sdpDescription.quickAccess.pRemoteCandidates[ i ],
                                                         pTargetRemoteSdp->sdpDescription.quickAccess.remoteCandidateLengths[ i ] );

                if( ret != PEER_CONNECTION_RESULT_OK )
                {
                    LogError( ( "Fail to add remote candidate from SDP offer, ret: %d, candidate(%u): %.*s",
                                ret,
                                pTargetRemoteSdp->sdpDescription.quickAccess.remoteCandidateLengths[ i ],
                                ( int ) pTargetRemoteSdp->sdpDescription.quickAccess.remoteCandidateLengths[ i ],
                                pTargetRemoteSdp->sdpDescription.quickAccess.pRemoteCandidates[ i ] ) );
                    ret = PEER_CONNECTION_RESULT_OK;
                }
                else
                {
                    LogDebug( ( "Added remote candidate from SDP offer, candidate(%u): %.*s",
                                pTargetRemoteSdp->sdpDescription.quickAccess.remoteCandidateLengths[ i ],
                                ( int ) pTargetRemoteSdp->sdpDescription.quickAccess.remoteCandidateLengths[ i ],
                                pTargetRemoteSdp->sdpDescription.quickAccess.pRemoteCandidates[ i ] ) );
                }
            }

        }
    }

    return ret;
}

PeerConnectionResult_t PeerConnection_SetVideoOnFrame( PeerConnectionSession_t * pSession,
                                                       OnFrameReadyCallback_t onFrameReadyCallbackFunc,
                                                       void * pOnFrameReadyCallbackCustomContext )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( pSession == NULL )
    {
        LogError( ( "Invalid input, pSession: %p", pSession ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        pSession->videoSrtpReceiver.onFrameReadyCallbackFunc = onFrameReadyCallbackFunc;
        pSession->videoSrtpReceiver.pOnFrameReadyCallbackCustomContext = pOnFrameReadyCallbackCustomContext;
    }

    return ret;
}
PeerConnectionResult_t PeerConnection_SetAudioOnFrame( PeerConnectionSession_t * pSession,
                                                       OnFrameReadyCallback_t onFrameReadyCallbackFunc,
                                                       void * pOnFrameReadyCallbackCustomContext )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( pSession == NULL )
    {
        LogError( ( "Invalid input, pSession: %p", pSession ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        pSession->audioSrtpReceiver.onFrameReadyCallbackFunc = onFrameReadyCallbackFunc;
        pSession->audioSrtpReceiver.pOnFrameReadyCallbackCustomContext = pOnFrameReadyCallbackCustomContext;
    }

    return ret;
}

PeerConnectionResult_t PeerConnection_AddRemoteCandidate( PeerConnectionSession_t * pSession,
                                                          const char * pDecodeMessage,
                                                          size_t decodeMessageLength )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    IceControllerResult_t iceControllerResult;
    IceControllerCandidate_t candidate;

    if( ( pSession == NULL ) ||
        ( pDecodeMessage == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pDecodeMessage: %p",
                    pSession, pDecodeMessage ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        memset( &candidate, 0, sizeof( IceControllerCandidate_t ) );
        iceControllerResult = IceController_DeserializeIceCandidate( pDecodeMessage,
                                                                     decodeMessageLength,
                                                                     &candidate );
        if( iceControllerResult != ICE_CONTROLLER_RESULT_OK )
        {
            LogWarn( ( "IceController_DeserializeIceCandidate fail, result: %d, dropping ICE candidate.", iceControllerResult ) );
            ret = PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_DESERIALIZE_CANDIDATE;
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        ret = SendRemoteCandidateRequest( pSession,
                                          &candidate );
    }

    return ret;
}

PeerConnectionResult_t PeerConnection_AddTransceiver( PeerConnectionSession_t * pSession,
                                                      Transceiver_t * pTransceiver )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( pSession == NULL )
    {
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        ret = AllocateTransceiver( pSession,
                                   pTransceiver );
    }

    return ret;
}

#if ENABLE_SCTP_DATA_CHANNEL
PeerConnectionResult_t PeerConnection_AddDataChannel( PeerConnectionSession_t * pSession )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    pSession->ucEnableDataChannelLocal = 1;
    return ret;
}
#endif /* ENABLE_SCTP_DATA_CHANNEL */

PeerConnectionResult_t PeerConnection_MatchTransceiverBySsrc( PeerConnectionSession_t * pSession,
                                                              uint32_t ssrc,
                                                              const Transceiver_t ** ppTransceiver )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    int i;

    if( ( pSession == NULL ) || ( ppTransceiver == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, ppTransceiver: %p", pSession, ppTransceiver ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        *ppTransceiver = NULL;
        for( i = 0; i < pSession->transceiverCount; i++ )
        {
            if( ( ssrc == pSession->pTransceivers[i]->ssrc ) || ( ssrc == pSession->pTransceivers[i]->rtxSsrc ) )
            {
                *ppTransceiver = pSession->pTransceivers[i];
                break;
            }
        }

        if( i == pSession->transceiverCount )
        {
            LogWarn( ( "No transceiver for SSRC: %lu", ssrc ) );
            ret = PEER_CONNECTION_RESULT_UNKNOWN_SSRC;
        }
    }

    return ret;
}

PeerConnectionResult_t PeerConnection_CloseSession( PeerConnectionSession_t * pSession )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    uint8_t i;
    uint8_t notifyTransceiver = 0U;

    if( pSession == NULL )
    {
        LogError( ( "Invalid input, pSession: %p", pSession ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    /* Update session state and notify transceivers. */
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( pSession->state == PEER_CONNECTION_SESSION_STATE_CONNECTION_READY )
        {
            /* We only notify traceiver when start is triggered. */
            notifyTransceiver = 1U;
        }

        pSession->state = PEER_CONNECTION_SESSION_STATE_CLOSING;

        if( notifyTransceiver != 0U )
        {
            for( i = 0; i < pSession->transceiverCount; i++ )
            {
                if( pSession->pTransceivers[i]->onPcEventCallbackFunc )
                {
                    pSession->pTransceivers[i]->onPcEventCallbackFunc( pSession->pTransceivers[i]->pOnPcEventCustomContext,
                                                                       TRANSCEIVER_CB_EVENT_REMOTE_PEER_CLOSED,
                                                                       NULL );
                }
            }
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        ret = DestroyIceController( pSession );
    }

    #if ENABLE_SCTP_DATA_CHANNEL
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        PeerConnectionResult_t xSCTPRet;

        /* Clear enable remote data channel */
        pSession->ucEnableDataChannelRemote = 0;
        pSession->uKvsDataChannelCount = 0;

        /* Close and deallocate all data channels along with terminating
         * SCTP session. */
        xSCTPRet = PeerConnectionSCTP_DeallocateSCTP( pSession );
        if( xSCTPRet == PEER_CONNECTION_RESULT_OK )
        {
            LogDebug( ( "Closed SCTP session." ) );
        }
        else
        {
            LogError( ( "Fail to close SCTP session, result: %d", xSCTPRet ) );
            ret = PEER_CONNECTION_RESULT_FAIL_SCTP_CLOSE;
        }
    }
    #endif /* #if ENABLE_SCTP_DATA_CHANNEL */

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        ret = PeerConnectionSrtp_DeInit( pSession );
        if( ret != PEER_CONNECTION_RESULT_OK )
        {
            LogError( ( "PeerConnectionSrtp_DeInit fail, result: %d", ret ) );
        }
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Reset metrics. */
        pSession->iceControllerContext.metrics.isFirstConnectivityRequest = 1;

        ret = PeerConnection_ResetTimer( pSession );
        if( ret != PEER_CONNECTION_RESULT_OK )
        {
            LogError( ( "PeerConnection_ResetTimer fail, result: %d", ret ) );
        }
    }

    #if METRIC_PRINT_ENABLED
    Metric_PrintMetrics();
    Metric_ResetEvent();
    #endif

    return ret;
}

PeerConnectionResult_t PeerConnection_WriteFrame( PeerConnectionSession_t * pSession,
                                                  Transceiver_t * pTransceiver,
                                                  const PeerConnectionFrame_t * pFrame )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( ( pSession == NULL ) ||
        ( pTransceiver == NULL ) ||
        ( pFrame == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pTransceiver: %p, pFrame: %p",
                    pSession, pTransceiver, pFrame ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    /* Encode the frame into multiple payload buffers (>=1). */
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        if( pSession->state < PEER_CONNECTION_SESSION_STATE_CONNECTION_READY )
        {
            LogInfo( ( "This session is not ready for sending frames, state: %d.", pSession->state ) );
        }
        else if( TRANSCEIVER_IS_CODEC_ENABLED( pTransceiver->codecBitMap,
                                               TRANSCEIVER_RTC_CODEC_H264_PROFILE_42E01F_LEVEL_ASYMMETRY_ALLOWED_PACKETIZATION_BIT ) )
        {
            ret = PeerConnectionH264Helper_WriteH264Frame( pSession,
                                                           pTransceiver,
                                                           pFrame );
        }
        else if( TRANSCEIVER_IS_CODEC_ENABLED( pTransceiver->codecBitMap,
                                               TRANSCEIVER_RTC_CODEC_OPUS_BIT ) )
        {
            ret = PeerConnectionOpusHelper_WriteOpusFrame( pSession,
                                                           pTransceiver,
                                                           pFrame );
        }
        else if( TRANSCEIVER_IS_CODEC_ENABLED( pTransceiver->codecBitMap,
                                               TRANSCEIVER_RTC_CODEC_MULAW_BIT ) )
        {
            ret = PeerConnectionG711Helper_WriteG711Frame( pSession,
                                                           pTransceiver,
                                                           pFrame );
        }
        else if( TRANSCEIVER_IS_CODEC_ENABLED( pTransceiver->codecBitMap,
                                               TRANSCEIVER_RTC_CODEC_ALAW_BIT ) )
        {
            ret = PeerConnectionG711Helper_WriteG711Frame( pSession,
                                                           pTransceiver,
                                                           pFrame );
        }
        else if( TRANSCEIVER_IS_CODEC_ENABLED( pTransceiver->codecBitMap,
                                               TRANSCEIVER_RTC_CODEC_H265_BIT ) )
        {
            ret = PeerConnectionH265Helper_WriteH265Frame( pSession,
                                                           pTransceiver,
                                                           pFrame );
        }
        else
        {
            /* TODO: Unknown, no matching codec. */
            LogError( ( "Codec is not supported, codec bit map: 0x%x", ( int ) pTransceiver->codecBitMap ) );
            ret = PEER_CONNECTION_RESULT_UNKNOWN_TX_CODEC;
        }
    }

    return ret;
}

PeerConnectionResult_t PeerConnection_CreateOffer( PeerConnectionSession_t * pSession,
                                                   PeerConnectionBufferSessionDescription_t * pOutputBufferSessionDescription,
                                                   char * pOutputSerializedSdpMessage,
                                                   size_t * pOutputSerializedSdpMessageLength )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( ( pSession == NULL ) ||
        ( pOutputBufferSessionDescription == NULL ) ||
        ( pOutputSerializedSdpMessage == NULL ) ||
        ( pOutputSerializedSdpMessageLength == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pOutputBufferSessionDescription: %p, pOutputSerializedSdpMessage: %p, pOutputSerializedSdpMessageLength: %p",
                    pSession, pOutputBufferSessionDescription, pOutputSerializedSdpMessage, pOutputSerializedSdpMessageLength ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else if( pOutputBufferSessionDescription->pSdpBuffer == NULL )
    {
        LogError( ( "Invalid input, pOutputBufferSessionDescription->pSdpBuffer: %p", pOutputBufferSessionDescription->pSdpBuffer ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else
    {
        /* Empty else marker. */
    }

    /* Use the default codec while creating SDP offer. */
    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        /* Update codec information based on transceivers. */
        ret = SetDefaultPayloadTypes( pSession );
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        ret = PeerConnectionSdp_PopulateSessionDescription( pSession,
                                                            NULL,
                                                            pOutputBufferSessionDescription,
                                                            pOutputSerializedSdpMessage,
                                                            pOutputSerializedSdpMessageLength );
    }

    return ret;
}

PeerConnectionResult_t PeerConnection_CreateAnswer( PeerConnectionSession_t * pSession,
                                                    PeerConnectionBufferSessionDescription_t * pOutputBufferSessionDescription,
                                                    char * pOutputSerializedSdpMessage,
                                                    size_t * pOutputSerializedSdpMessageLength )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( ( pSession == NULL ) ||
        ( pOutputBufferSessionDescription == NULL ) ||
        ( pOutputSerializedSdpMessage == NULL ) ||
        ( pOutputSerializedSdpMessageLength == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pOutputBufferSessionDescription: %p, pOutputSerializedSdpMessage: %p, pOutputSerializedSdpMessageLength: %p",
                    pSession, pOutputBufferSessionDescription, pOutputSerializedSdpMessage, pOutputSerializedSdpMessageLength ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else if( pOutputBufferSessionDescription->pSdpBuffer == NULL )
    {
        LogError( ( "Invalid input, pOutputBufferSessionDescription->pSdpBuffer: %p", pOutputBufferSessionDescription->pSdpBuffer ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else
    {
        /* Empty else marker. */
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        ret = PeerConnectionSdp_PopulateSessionDescription( pSession,
                                                            &pSession->remoteSessionDescription,
                                                            pOutputBufferSessionDescription,
                                                            pOutputSerializedSdpMessage,
                                                            pOutputSerializedSdpMessageLength );
    }

    return ret;
}

PeerConnectionResult_t PeerConnection_SetOnLocalCandidateReady( PeerConnectionSession_t * pSession,
                                                                OnIceCandidateReadyCallback_t onLocalCandidateReadyCallbackFunc,
                                                                void * pOnLocalCandidateReadyCallbackCustomContext )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( ( pSession == NULL ) ||
        ( onLocalCandidateReadyCallbackFunc == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, onLocalCandidateReadyCallbackFunc: %p",
                    pSession, onLocalCandidateReadyCallbackFunc ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        pSession->onIceCandidateReadyCallbackFunc = onLocalCandidateReadyCallbackFunc;
        pSession->pOnLocalCandidateReadyCallbackCustomContext = pOnLocalCandidateReadyCallbackCustomContext;
    }

    return ret;
}

PeerConnectionResult_t PeerConnection_AddIceServerConfig( PeerConnectionSession_t * pSession,
                                                          PeerConnectionSessionConfiguration_t * pSessionConfig )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    IceControllerResult_t iceControllerResult;
    IceControllerIceServerConfig_t iceServerConfig;

    if( ( pSession == NULL ) ||
        ( pSessionConfig == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pSessionConfig: %p",
                    pSession, pSessionConfig ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ret == PEER_CONNECTION_RESULT_OK )
    {
        memset( &iceServerConfig, 0, sizeof( IceControllerIceServerConfig_t ) );
        iceServerConfig.pIceServers = pSessionConfig->iceServers;
        iceServerConfig.iceServersCount = pSessionConfig->iceServersCount;
        iceServerConfig.pRootCaPath = pSessionConfig->pRootCaPath;
        iceServerConfig.rootCaPathLength = pSessionConfig->rootCaPathLength;
        iceServerConfig.pRootCaPem = pSessionConfig->pRootCaPem;
        iceServerConfig.rootCaPemLength = pSessionConfig->rootCaPemLength;
        iceControllerResult = IceController_AddIceServerConfig( &pSession->iceControllerContext,
                                                                &iceServerConfig );
        if( iceControllerResult != ICE_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Fail to add Ice server config into Ice Controller." ) );
            ret = PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_ADD_ICE_SERVER_CONFIG;
        }
    }

    return ret;
}

static PeerConnectionResult_t PeerConnection_OnRtcpSenderReportCallback( PeerConnectionSession_t * pSession,
                                                                         PeerConnectionSessionRequestMessage_t * pRequestMessage )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;
    IceControllerResult_t iceControllerResult;
    RtcpSenderReport_t rtcpSenderReport = { 0 };
    uint8_t srtcpPacket[ PEER_CONNECTION_SRTCP_RTCP_PACKET_MIN_LENGTH ];
    uint8_t readyToSend = 0;
    size_t srtcpPacketLength = sizeof( srtcpPacket );
    uint64_t currentTimeUs = pRequestMessage->peerConnectionSessionRequestContent.rtcpContent.currentTimeUs;
    const Transceiver_t * pTransceiver = pRequestMessage->peerConnectionSessionRequestContent.rtcpContent.pTransceiver;

    if( ( pSession == NULL ) ||
        ( pTransceiver == NULL ) )
    {
        LogError( ( "Invalid input, pSession: %p, pTransceiver: %p",
                    pSession, pTransceiver ) );
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }

    if( ( ret == PEER_CONNECTION_RESULT_OK ) && ( pSession->srtpTransmitSession != NULL ) && ( currentTimeUs - pTransceiver->rtpSender.rtpFirstFrameWallClockTimeUs >= 2500 * 1000 ) )
    {
        readyToSend = 1;
    }

    if( !readyToSend )
    {
        LogVerbose( ( "Send Report No Frames are sent to SSRC :  %lu",
                      pTransceiver->ssrc ) );
    }
    else
    {
        if( pTransceiver->trackKind == TRANSCEIVER_TRACK_KIND_AUDIO )
        {
            rtcpSenderReport.senderInfo.rtpTime = pTransceiver->rtpSender.rtpTimeOffset + PEER_CONNECTION_SRTP_CONVERT_TIME_US_TO_RTP_TIMESTAMP( pSession->audioSrtpReceiver.rxJitterBuffer.clockRate,
                                                                                                                                                 currentTimeUs - pTransceiver->rtpSender.rtpFirstFrameWallClockTimeUs );
        }
        else
        {
            rtcpSenderReport.senderInfo.rtpTime = pTransceiver->rtpSender.rtpTimeOffset + PEER_CONNECTION_SRTP_CONVERT_TIME_US_TO_RTP_TIMESTAMP( pSession->videoSrtpReceiver.rxJitterBuffer.clockRate,
                                                                                                                                                 currentTimeUs - pTransceiver->rtpSender.rtpFirstFrameWallClockTimeUs );
        }
        rtcpSenderReport.senderSsrc = pTransceiver->ssrc;
        rtcpSenderReport.senderInfo.ntpTime = NetworkingUtils_GetNTPTimeFromUnixTimeUs( currentTimeUs );
        rtcpSenderReport.senderInfo.packetCount = pTransceiver->rtcpStats.rtpPacketsTransmitted;
        rtcpSenderReport.senderInfo.octetCount = pTransceiver->rtcpStats.rtpBytesTransmitted;

        /* Since the Master isn't processing the received RTP Packets. We don't have any reception reports.*/
        rtcpSenderReport.pReceptionReports = NULL;
        rtcpSenderReport.numReceptionReports = 0;

        ret = PeerConnectionSrtcp_ConstructSenderReportPacket( ( pSession ),
                                                               &( rtcpSenderReport ),
                                                               &( srtcpPacket[ 0 ] ),
                                                               &( srtcpPacketLength ) );
        if( ret != PEER_CONNECTION_RESULT_OK )
        {
            LogError( ( "Fail to serialize and encrypt RTCP Sender Report." ) );
            ret = PEER_CONNECTION_RESULT_FAIL_RTCP_SERIALIZE_SENDER_REPORT;
        }

        /* Send the constructed RTCP packets through network. */
        if( ret == PEER_CONNECTION_RESULT_OK )
        {
            iceControllerResult = IceController_SendToRemotePeer( &( pSession->iceControllerContext ),
                                                                  ( srtcpPacket ),
                                                                  srtcpPacketLength );

            if( iceControllerResult != ICE_CONTROLLER_RESULT_OK )
            {
                LogWarn( ( "Fail to send RTCP packet, ret: %d", iceControllerResult ) );
                ret = PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_SEND_RTCP_PACKET;
            }
            else
            {
                LogDebug( ( "Send RTCP Sender Report with Status : %u  to SSRC :  %lu, NTP Time :  %llu, RTP Time:  %lu,  PacketCount : %lu, OctetCount : %lu",ret,
                            rtcpSenderReport.senderSsrc, rtcpSenderReport.senderInfo.ntpTime, rtcpSenderReport.senderInfo.rtpTime, rtcpSenderReport.senderInfo.packetCount, rtcpSenderReport.senderInfo.octetCount ) );
            }
        }

    }

    return ret;
}

PeerConnectionResult_t PeerConnection_SetPictureLossIndicationCallback( PeerConnectionSession_t * pSession,
                                                                        OnPictureLossIndicationCallback_t onPictureLossIndicationCallback,
                                                                        void * pUserContext )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( ( pSession == NULL ) || ( onPictureLossIndicationCallback == NULL ) )
    {
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else
    {
        pSession->onPictureLossIndicationCallback = onPictureLossIndicationCallback;
        pSession->pPictureLossIndicationUserContext = pUserContext;
    }

    return ret;
}

#if ENABLE_TWCC_SUPPORT
PeerConnectionResult_t PeerConnection_SetSenderBandwidthEstimationCallback( PeerConnectionSession_t * pSession,
                                                                            OnBandwidthEstimationCallback_t onBandwidthEstimationCallback,
                                                                            void * pUserContext )
{
    PeerConnectionResult_t ret = PEER_CONNECTION_RESULT_OK;

    if( ( pSession == NULL ) || ( onBandwidthEstimationCallback == NULL ) )
    {
        ret = PEER_CONNECTION_RESULT_BAD_PARAMETER;
    }
    else
    {
        pSession->pCtx->onBandwidthEstimationCallback = onBandwidthEstimationCallback;
        pSession->pCtx->pOnBandwidthEstimationCallbackContext = pUserContext;
    }

    return ret;
}
#endif