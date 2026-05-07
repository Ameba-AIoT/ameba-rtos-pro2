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

AppContext_t appContext;
AppMediaSourcesContext_t appMediaSourceContext;

static void Master_Task( void * pParameter );

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
    AppMediaSourcesContext_t * pMediaSourceContext = ( AppMediaSourcesContext_t * )pMediaCtx;

    if( ( pMediaCtx == NULL ) || ( pTranceiver == NULL ) )
    {
        LogError( ( "Invalid input, pMediaCtx: %p, pTranceiver: %p", pMediaCtx, pTranceiver ) );
        ret = -1;
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
                LogError( ( "Unknown track kind: %d", trackKind ) );
                ret = -2;
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

static void Master_Task( void * pParameter )
{
    int32_t ret = 0;

    ( void ) pParameter;

    LogInfo( ( "Start Master_Task." ) );

    ret = AppCommon_Init( &appContext, InitTransceiver, &appMediaSourceContext );

    if( ret == 0 )
    {
        ret = InitializeAppMediaSource( &appContext, &appMediaSourceContext );
    }

    if( ret == 0 )
    {
        /* Configure signaling controller with client ID and role type. */
        memcpy( &( appContext.signalingControllerClientId[ 0 ] ), SIGNALING_CONTROLLER_MASTER_CLIENT_ID, SIGNALING_CONTROLLER_MASTER_CLIENT_ID_LENGTH );
        appContext.signalingControllerClientId[ SIGNALING_CONTROLLER_MASTER_CLIENT_ID_LENGTH ] = '\0';
        appContext.signalingControllerClientIdLength = SIGNALING_CONTROLLER_MASTER_CLIENT_ID_LENGTH;
        appContext.signalingControllerRole = SIGNALING_ROLE_MASTER;
    }

    if( ret == 0 )
    {
        /* Launch application with current thread serving as Signaling Controller. */
        ret = AppCommon_StartSignalingController( &appContext );
    }

    if( ret == 0 )
    {
        /* Launch application with current thread serving as Signaling Controller. */
        AppCommon_WaitSignalingControllerStop( &appContext );
    }

    for( ;; )
    {
        vTaskDelay( pdMS_TO_TICKS( 200 ) );
    }
}

void kvs_webrtc_master_entry( void )
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
        if( xTaskCreate( Master_Task,
                         ( ( const char * ) "MasterTask" ),
                         4096,
                         NULL,
                         tskIDLE_PRIORITY + 4,
                         NULL ) != pdPASS )
        {
            LogError( ( "xTaskCreate(Master_Task) failed" ) );
            ret = -1;
        }
    }
}
