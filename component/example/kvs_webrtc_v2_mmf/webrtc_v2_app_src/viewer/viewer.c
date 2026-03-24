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
#include <time.h>

#include "FreeRTOS.h"
#include "task.h"

#include "sys_api.h"      /* sys_backtrace_enable() */
#include "sntp/sntp.h"    /* SNTP series APIs */
#include "wifi_conf.h"    /* WiFi series APIs */
#include "lwip_netconf.h" /* LwIP_GetIP() */
#include "srtp.h"

#include "demo_config.h"
#include "app_common.h"
#include "app_media_source.h"
#include "logging.h"

#if METRIC_PRINT_ENABLED
#include "metric.h"
#endif

#if ENABLE_SCTP_DATA_CHANNEL
    #include "peer_connection_sctp.h"

    #define WEBRTC_APPLICATION_VIEWER_DATA_CHANNEL_NAME ( "TEST_DATA_CHANNEL" )
#endif /* ENABLE_SCTP_DATA_CHANNEL */

AppContext_t appContext;
AppMediaSourcesContext_t appMediaSourceContext;

static void Viewer_Task( void * pParameter );

static int32_t InitTransceiver( void * pMediaCtx,
                                TransceiverTrackKind_t trackKind,
                                Transceiver_t * pTranceiver );
static int32_t OnMediaSinkHook( void * pCustom,
                                MediaFrame_t * pFrame );
static int32_t InitializeAppMediaSource( AppContext_t * pAppContext,
                                         AppMediaSourcesContext_t * pAppMediaSourceContext );

static int32_t InitTransceiver( void * pMediaCtx,
                                TransceiverTrackKind_t trackKind,
                                Transceiver_t * pTranceiver )
{
    int32_t ret = 0;
    AppMediaSourcesContext_t * pMediaSourceContext = ( AppMediaSourcesContext_t * ) pMediaCtx;

    if( ( pMediaCtx == NULL ) || ( pTranceiver == NULL ) )
    {
        LogError( ( "Invalid input, pMediaCtx: %p, pTranceiver: %p", pMediaCtx, pTranceiver ) );
        ret = -1;
    }
    else if( ( trackKind != TRANSCEIVER_TRACK_KIND_VIDEO ) &&
             ( trackKind != TRANSCEIVER_TRACK_KIND_AUDIO ) )
    {
        LogError( ( "Invalid track kind: %d", trackKind ) );
        ret = -2;
    }
    else
    {
        /* Empty else marker. */
    }

    if( ret == 0 )
    {
        switch( trackKind )
        {
            case TRANSCEIVER_TRACK_KIND_VIDEO:
                ret = AppMediaSource_InitVideoTransceiver( pMediaSourceContext,
                                                           pTranceiver );
                break;
            case TRANSCEIVER_TRACK_KIND_AUDIO:
                ret = AppMediaSource_InitAudioTransceiver( pMediaSourceContext,
                                                           pTranceiver );
                break;
            default:
                LogError( ( "Invalid track kind: %d", trackKind ) );
                ret = -3;
                break;
        }
    }

    return ret;
}

static int32_t OnMediaSinkHook( void * pCustom,
                                MediaFrame_t * pFrame )
{
    int32_t ret = 0;
    AppContext_t * pAppContext = ( AppContext_t * ) pCustom;
    PeerConnectionResult_t peerConnectionResult;
    Transceiver_t * pTransceiver = NULL;
    PeerConnectionFrame_t peerConnectionFrame;
    int i;

    if( ( pAppContext == NULL ) || ( pFrame == NULL ) )
    {
        LogError( ( "Invalid input, pCustom: %p, pFrame: %p", pCustom, pFrame ) );
        ret = -1;
    }

    if( ret == 0 )
    {
        peerConnectionFrame.version = PEER_CONNECTION_FRAME_CURRENT_VERSION;
        peerConnectionFrame.presentationUs = pFrame->timestampUs;
        peerConnectionFrame.pData = pFrame->pData;
        peerConnectionFrame.dataLength = pFrame->size;

        for( i = 0; i < AWS_MAX_VIEWER_NUM; i++ )
        {
            if( pFrame->trackKind == TRANSCEIVER_TRACK_KIND_VIDEO )
            {
                pTransceiver = &pAppContext->appSessions[ i ].transceivers[ DEMO_TRANSCEIVER_MEDIA_INDEX_VIDEO ];
            }
            else if( pFrame->trackKind == TRANSCEIVER_TRACK_KIND_AUDIO )
            {
                pTransceiver = &pAppContext->appSessions[ i ].transceivers[ DEMO_TRANSCEIVER_MEDIA_INDEX_AUDIO ];
            }
            else
            {
                /* Unknown kind, skip that. */
                LogWarn( ( "Unknown track kind: %d", pFrame->trackKind ) );
                break;
            }

            if( pAppContext->appSessions[ i ].peerConnectionSession.state == PEER_CONNECTION_SESSION_STATE_CONNECTION_READY )
            {
                peerConnectionResult = PeerConnection_WriteFrame( &pAppContext->appSessions[ i ].peerConnectionSession,
                                                                  pTransceiver,
                                                                  &peerConnectionFrame );

                if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
                {
                    LogError( ( "Fail to write %s frame, result: %d", ( pFrame->trackKind == TRANSCEIVER_TRACK_KIND_VIDEO ) ? "video" : "audio",
                                peerConnectionResult ) );
                    ret = -3;
                }
            }
        }
    }

    return ret;
}

static int32_t InitializeAppMediaSource( AppContext_t * pAppContext,
                                         AppMediaSourcesContext_t * pAppMediaSourceContext )
{
    int32_t ret = 0;

    if( ( pAppContext == NULL ) ||
        ( pAppMediaSourceContext == NULL ) )
    {
        LogError( ( "Invalid input, pAppContext: %p, pAppMediaSourceContext: %p", pAppContext, pAppMediaSourceContext ) );
        ret = -1;
    }

    if( ret == 0 )
    {
        ret = AppMediaSource_Init( pAppMediaSourceContext,
                                   OnMediaSinkHook,
                                   pAppContext );
    }

    return ret;
}

static int SendSdpOffer( AppContext_t * pAppContext )
{
    int ret = 0;
    PeerConnectionResult_t peerConnectionResult;
    AppSession_t * pAppSession = NULL;
    PeerConnectionBufferSessionDescription_t bufferSessionDescription;
    SignalingControllerEventMessage_t signalingMessageSdpOffer = {
        .event = SIGNALING_CONTROLLER_EVENT_SEND_WSS_MESSAGE,
        .onCompleteCallback = NULL,
        .pOnCompleteCallbackContext = NULL,
    };
    size_t sdpOfferMessageLength = 0;
    SignalingControllerResult_t signalingControllerReturn;

    /* Use AppCommon_GetPeerConnectionSession to initialize peer connection, including transceivers. */
    pAppSession = AppCommon_GetPeerConnectionSession( pAppContext,
                                                      NULL,
                                                      0U );
    if( pAppSession == NULL )
    {
        LogError( ( "Fail to get available peer connection session" ) );
        ret = -1;
    }

#if ENABLE_SCTP_DATA_CHANNEL
    /* Add data channel support to SDP offer */
    if( ret == 0 )
    {
        peerConnectionResult = PeerConnection_AddDataChannel( &pAppSession->peerConnectionSession );
        if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
        {
            LogError( ( "Fail to add data channel, result = %d.", peerConnectionResult ) );
            ret = -2;
        }
    }

    if( ret == 0 )
    {
        PeerConnectionDataChannel_t * pChannel = NULL;
        peerConnectionResult = PeerConnectionSCTP_CreateDataChannel( &pAppSession->peerConnectionSession,
                                                                     WEBRTC_APPLICATION_VIEWER_DATA_CHANNEL_NAME,
                                                                     NULL,
                                                                     &pChannel );
        if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
        {
            LogError( ( "Fail to create data channel, result = %d.", peerConnectionResult ) );
            ret = -3;
        }
    }
#endif /* ENABLE_SCTP_DATA_CHANNEL */

    /* Set local description. */
    if( ret == 0 )
    {
        memset( &bufferSessionDescription, 0, sizeof( bufferSessionDescription ) );
        bufferSessionDescription.pSdpBuffer = pAppContext->sdpBuffer;
        bufferSessionDescription.sdpBufferLength = PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH;
        bufferSessionDescription.type = SDP_CONTROLLER_MESSAGE_TYPE_OFFER;
        peerConnectionResult = PeerConnection_SetLocalDescription( &pAppSession->peerConnectionSession,
                                                                   &bufferSessionDescription );
        if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
        {
            LogError( ( "Fail to set local description, result: %d", peerConnectionResult ) );
            ret = -4;
        }
    }

    /* Create offer. */
    if( ret == 0 )
    {
        pAppContext->sdpConstructedBufferLength = PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH;
        peerConnectionResult = PeerConnection_CreateOffer( &pAppSession->peerConnectionSession,
                                                           &bufferSessionDescription,
                                                           pAppContext->sdpConstructedBuffer,
                                                           &pAppContext->sdpConstructedBufferLength );
        if( peerConnectionResult != PEER_CONNECTION_RESULT_OK )
        {
            LogError( ( "Fail to create offer, result: %d", peerConnectionResult ) );
            ret = -5;
        }
    }

    if( ret == 0 )
    {
        /* Translate from SDP formal format into signaling event message by replacing newline with "\\n" or "\\r\\n". */
        sdpOfferMessageLength = PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH;
        signalingControllerReturn = SignalingController_SerializeSdpContentNewline( pAppContext->sdpConstructedBuffer,
                                                                                    pAppContext->sdpConstructedBufferLength,
                                                                                    pAppContext->sdpBuffer,
                                                                                    &sdpOfferMessageLength );
        if( signalingControllerReturn != SIGNALING_CONTROLLER_RESULT_OK )
        {
            LogError( ( "Fail to serialize SDP offer newline, result: %d, constructed buffer(%u): %.*s",
                        signalingControllerReturn,
                        pAppContext->sdpConstructedBufferLength,
                        ( int ) pAppContext->sdpConstructedBufferLength,
                        pAppContext->sdpConstructedBuffer ) );
            ret = -6;
        }
    }

    if( ret == 0 )
    {
        signalingMessageSdpOffer.eventContent.correlationIdLength = 0U;
        memset( signalingMessageSdpOffer.eventContent.correlationId, 0, SECRET_ACCESS_KEY_MAX_LEN );
        signalingMessageSdpOffer.eventContent.messageType = SIGNALING_TYPE_MESSAGE_SDP_OFFER;
        signalingMessageSdpOffer.eventContent.pDecodeMessage = pAppContext->sdpBuffer;
        signalingMessageSdpOffer.eventContent.decodeMessageLength = sdpOfferMessageLength;
        memset( signalingMessageSdpOffer.eventContent.remoteClientId, 0, SIGNALING_CONTROLLER_REMOTE_ID_MAX_LENGTH );
        signalingMessageSdpOffer.eventContent.remoteClientIdLength = 0U;

        signalingControllerReturn = SignalingController_SendMessage( &( pAppContext->signalingControllerContext ),
                                                                     &signalingMessageSdpOffer );
        if( signalingControllerReturn != SIGNALING_CONTROLLER_RESULT_OK )
        {
            ret = -7;
            LogError( ( "Send signaling message fail, result: %d", signalingControllerReturn ) );
        }
    }

    return ret;
}

static void Viewer_Task( void * pParameter )
{
    int32_t ret = 0;
    int clientIdLength = 0;
    uint32_t randomClientIdPostfix = 0;

    ( void ) pParameter;

    LogInfo( ( "Start Viewer_Task." ) );

    ret = AppCommon_Init( &appContext, InitTransceiver, &appMediaSourceContext );

    if( ret == 0 )
    {
        ret = InitializeAppMediaSource( &appContext, &appMediaSourceContext );
    }

    if( ret == 0 )
    {
        /* Configure signaling controller with client ID and role type. */
        randomClientIdPostfix = rand();

        clientIdLength = snprintf( &( appContext.signalingControllerClientId[ 0 ] ),
                                   sizeof( appContext.signalingControllerClientId ),
                                   "%s%lu",
                                   SIGNALING_CONTROLLER_VIEWER_CLIENT_ID_PREFIX,
                                   randomClientIdPostfix );
        appContext.signalingControllerClientIdLength = clientIdLength;
        appContext.signalingControllerRole = SIGNALING_ROLE_VIEWER;

        if( clientIdLength < 0 )
        {
            LogError( ( "snprintf return failure, errno: %d", errno ) );
            ret = -1;
        }
    }

    if( ret == 0 )
    {
        /* Launch application with current thread serving as Signaling Controller. */
        ret = AppCommon_StartSignalingController( &appContext );
    }

    if( ret == 0 )
    {
        #if METRIC_PRINT_ENABLED
            Metric_StartEvent( METRIC_EVENT_SENDING_FIRST_FRAME );
        #endif

        ret = SendSdpOffer( &appContext );

        while( appContext.appSessions[0].peerConnectionSession.state >= PEER_CONNECTION_SESSION_STATE_START )
        {
            /* The session is still alive, keep processing. */
            vTaskDelay( pdMS_TO_TICKS( 10000 ) );
        }

        LogInfo( ( "Ending viewer" ) );
    }

    for( ;; )
    {
        vTaskDelay( pdMS_TO_TICKS( 200 ) );
    }
}

void app_example( void )
{
    int ret = 0;

    if( ret == 0 )
    {
        #ifdef BUILD_INFO
        LogInfo( ( "\r\nBuild Info: %s\r\n", BUILD_INFO ) );
        #endif
    }

    if( ret == 0 )
    {
        if( xTaskCreate( Viewer_Task,
                         ( ( const char * ) "ViewerTask" ),
                         16384,
                         NULL,
                         tskIDLE_PRIORITY + 4,
                         NULL ) != pdPASS )
        {
            LogError( ( "xTaskCreate(Viewer_Task) failed" ) );
            ret = -1;
        }
    }
}
