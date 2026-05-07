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

#include "demo_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_media_source.h"

#define DEFAULT_TRANSCEIVER_ROLLING_BUFFER_DURACTION_SECOND ( 3 )

/* Considering 1.4 Mbps for 720p (which is what our samples use). This is for H.264.
 * For H.265, we're using a lower bit rate of about 462 Kbps.
 * The value could be different for other codecs. */
#define TRANSCEIVER_H264_VIDEO_BIT_RATE ( 1.4 * 1024 * 1024 )
#define TRANSCEIVER_H265_VIDEO_BIT_RATE ( 462 * 1024 )

/* For opus, the bitrate could be between 6 Kbps to 1000 Kbps  */
#define DEFAULT_TRANSCEIVER_AUDIO_BIT_RATE ( 1000 * 1024 )

#define DEFAULT_TRANSCEIVER_MEDIA_STREAM_ID "myKvsVideoStream"
#define DEFAULT_TRANSCEIVER_VIDEO_TRACK_ID "myVideoTrack"
#define DEFAULT_TRANSCEIVER_AUDIO_TRACK_ID "myAudioTrack"

#define DEMO_TRANSCEIVER_VIDEO_TX_DATA_QUEUE_NAME "/TxVideoMq"
#define DEMO_TRANSCEIVER_AUDIO_TX_DATA_QUEUE_NAME "/TxAudioMq"
#define DEMO_TRANSCEIVER_AUDIO_RX_DATA_QUEUE_NAME "/RxAudioMq"
#define DEMO_TRANSCEIVER_MAX_TX_QUEUE_MSG_NUM ( 10 )
#define DEMO_TRANSCEIVER_MAX_RX_QUEUE_MSG_NUM ( 10 )

static void VideoTx_Task( void * pParameter );
static void AudioTx_Task( void * pParameter );
static int32_t OnFrameReadyToSend( void * pCtx,
                                   MediaFrame_t * pFrame );

static void VideoTx_Task( void * pParameter )
{
    AppMediaSourceContext_t * pVideoContext = ( AppMediaSourceContext_t * )pParameter;
    MessageQueueResult_t retMessageQueue;
    uint8_t skipProcess = 0;
    MediaFrame_t frame;
    size_t frameLength;

    if( pVideoContext == NULL )
    {
        LogError( ( "Invalid input, pVideoContext: %p", pVideoContext ) );
        skipProcess = 1;
    }

    /* Handle event. */
    if( skipProcess == 0 )
    {
        /* Intentional infinite loop to keep addressing media frames from hardware. */
        while( 1 )
        {
            /* Recevied message from data queue. */
            frameLength = sizeof( MediaFrame_t );
            retMessageQueue = MessageQueue_Recv( &pVideoContext->dataTxQueue,
                                                 &frame,
                                                 &frameLength );
            if( retMessageQueue == MESSAGE_QUEUE_RESULT_OK )
            {
                /* Received a media frame. */
                LogVerbose( ( "Video Tx frame(%ld), trackKind: %d, timestamp: %llu, payload: 0x%x 0x%x 0x%x 0x%x", frame.size, frame.trackKind, frame.timestampUs, frame.pData[0], frame.pData[1], frame.pData[2], frame.pData[3] ) );

                if( pVideoContext->pSourcesContext->onMediaSinkHookFunc )
                {
                    ( void ) pVideoContext->pSourcesContext->onMediaSinkHookFunc( pVideoContext->pSourcesContext->pOnMediaSinkHookCustom,
                                                                                  &frame );
                }

                if( frame.freeData )
                {
                    vPortFree( frame.pData );
                }
            }
            else
            {
                LogError( ( " VideoTx_Task: MessageQueue_Recv failed with error %d", retMessageQueue ) );
            }
        }
    }

    for( ;; )
    {
        vTaskDelay( pdMS_TO_TICKS( 200 ) );
    }
}

static void AudioTx_Task( void * pParameter )
{
    AppMediaSourceContext_t * pAudioContext = ( AppMediaSourceContext_t * )pParameter;
    MessageQueueResult_t retMessageQueue;
    uint8_t skipProcess = 0;
    MediaFrame_t frame;
    size_t frameLength;

    if( pAudioContext == NULL )
    {
        LogError( ( "Invalid input, pAudioContext: %p", pAudioContext ) );
        skipProcess = 1;
    }

    /* Handle event. */
    if( skipProcess == 0 )
    {
        /* Intentional infinite loop to keep addressing media frames from hardware. */
        while( 1 )
        {
            /* Recevied message from data queue. */
            frameLength = sizeof( MediaFrame_t );
            retMessageQueue = MessageQueue_Recv( &pAudioContext->dataTxQueue,
                                                 &frame,
                                                 &frameLength );
            if( retMessageQueue == MESSAGE_QUEUE_RESULT_OK )
            {
                /* Received a media frame. */
                LogVerbose( ( "Audio Tx frame(%ld), track kind: %d, timestampUs: %llu", frame.size, frame.trackKind, frame.timestampUs ) );

                if( pAudioContext->pSourcesContext->onMediaSinkHookFunc )
                {
                    ( void ) pAudioContext->pSourcesContext->onMediaSinkHookFunc( pAudioContext->pSourcesContext->pOnMediaSinkHookCustom,
                                                                                  &frame );
                }
                if( frame.freeData )
                {
                    vPortFree( frame.pData );
                }
            }
            else
            {
                LogError( ( " AudioTx_Task: MessageQueue_Recv failed with error %d", retMessageQueue ) );
            }
        }
    }

    for( ;; )
    {
        vTaskDelay( pdMS_TO_TICKS( 200 ) );
    }
}

#if MEDIA_PORT_ENABLE_AUDIO_RECV
    static void AudioRx_Task( void * pParameter )
    {
        AppMediaSourceContext_t * pAudioContext = ( AppMediaSourceContext_t * )pParameter;
        MessageQueueResult_t retMessageQueue;
        uint8_t skipProcess = 0;
        MediaFrame_t frame;
        size_t frameLength;

        if( pAudioContext == NULL )
        {
            LogError( ( "Invalid input, pAudioContext: %p", pAudioContext ) );
            skipProcess = 1;
        }

        /* Handle event. */
        if( skipProcess == 0 )
        {
            while( 1 )
            {
                /* Recevied message from data queue. */
                frameLength = sizeof( MediaFrame_t );
                retMessageQueue = MessageQueue_Recv( &pAudioContext->dataRxQueue,
                                                    &frame,
                                                    &frameLength );
                if( retMessageQueue == MESSAGE_QUEUE_RESULT_OK )
                {
                    /* Received a media frame. */
                    LogVerbose( ( "Audio Rx frame(%ld), timestampUs: %llu", frame.size, frame.timestampUs ) );

                    AppMediaSourcePort_PlayAudioFrame( &frame );

                    if( frame.freeData )
                    {
                        vPortFree( frame.pData );
                    }
                }
                else
                {
                    LogError( ( " AudioRx_Task: MessageQueue_Recv failed with error %d", retMessageQueue ) );
                }
            }
        }

        for( ;; )
        {
            vTaskDelay( pdMS_TO_TICKS( 200 ) );
        }
    }
#endif /* #if MEDIA_PORT_ENABLE_AUDIO_RECV */

static int32_t OnPcEventRemotePeerReady( AppMediaSourceContext_t * pMediaSource )
{
    int32_t ret = 0;

    if( pMediaSource == NULL )
    {
        LogError( ( "Invalid input, pMediaSource: %p", pMediaSource ) );
        ret = -1;
    }

    if( ret == 0 )
    {
        if( xSemaphoreTake( pMediaSource->pSourcesContext->mediaMutex,
                            portMAX_DELAY ) == pdTRUE )
        {
            if( pMediaSource->numReadyPeer < AWS_MAX_VIEWER_NUM )
            {
                pMediaSource->numReadyPeer++;
                pMediaSource->pSourcesContext->totalNumReadyPeer++;
            }

            if( pMediaSource->pSourcesContext->totalNumReadyPeer == 1U )
            {
                /* Start media transmission. */
                ret = AppMediaSourcePort_Start( OnFrameReadyToSend,
                                                &pMediaSource->pSourcesContext->videoContext,
                                                OnFrameReadyToSend,
                                                &pMediaSource->pSourcesContext->audioContext );
            }

            /* We have finished accessing the shared resource.  Release the mutex. */
            xSemaphoreGive( pMediaSource->pSourcesContext->mediaMutex );
            LogInfo( ( "Starting track kind(%d) media, value=%u", pMediaSource->trackKind, pMediaSource->numReadyPeer ) );
        }
        else
        {
            LogError( ( "Failed to lock media mutex, track kind=%d.", pMediaSource->trackKind ) );
            ret = -1;
        }
    }

    return ret;
}

static int32_t OnPcEventRemotePeerClosed( AppMediaSourceContext_t * pMediaSource )
{
    int32_t ret = 0;

    if( pMediaSource == NULL )
    {
        LogError( ( "Invalid input, pMediaSource: %p", pMediaSource ) );
        ret = -1;
    }

    if( ret == 0 )
    {
        if( xSemaphoreTake( pMediaSource->pSourcesContext->mediaMutex,
                            portMAX_DELAY ) == pdTRUE )
        {
            if( pMediaSource->numReadyPeer > 0U )
            {
                pMediaSource->numReadyPeer--;
                pMediaSource->pSourcesContext->totalNumReadyPeer--;
            }

            if( pMediaSource->pSourcesContext->totalNumReadyPeer == 0U )
            {
                /* Stop media transmission. */
                AppMediaSourcePort_Stop();
            }

            /* We have finished accessing the shared resource.  Release the mutex. */
            xSemaphoreGive( pMediaSource->pSourcesContext->mediaMutex );
            LogInfo( ( "Stopping track kind(%d) media, value=%u", pMediaSource->trackKind, pMediaSource->numReadyPeer ) );
        }
        else
        {
            LogError( ( "Failed to lock media mutex, track kind=%d.", pMediaSource->trackKind ) );
            ret = -1;
        }
    }

    return ret;
}

static int32_t HandlePcEventCallback( void * pCustomContext,
                                      TransceiverCallbackEvent_t event,
                                      TransceiverCallbackContent_t * pEventMsg )
{
    int32_t ret = 0;
    AppMediaSourceContext_t * pMediaSource = ( AppMediaSourceContext_t * )pCustomContext;

    if( pMediaSource == NULL )
    {
        LogError( ( "Invalid input, pEventMsg: %p", pEventMsg ) );
        ret = -1;
    }

    if( ret == 0 )
    {
        LogDebug( ( "Receiving peer connection event %d", event ) );
        switch( event )
        {
            case TRANSCEIVER_CB_EVENT_REMOTE_PEER_READY:
                ret = OnPcEventRemotePeerReady( pMediaSource );
                break;
            case TRANSCEIVER_CB_EVENT_REMOTE_PEER_CLOSED:
                ret = OnPcEventRemotePeerClosed( pMediaSource );
                break;
            default:
                LogWarn( ( "Unknown event: 0x%x", event ) );
                break;
        }
    }

    return ret;
}

static int32_t InitializeVideoSource( AppMediaSourceContext_t * pVideoSource )
{
    int32_t ret = 0;
    MessageQueueResult_t retMessageQueue;

    if( pVideoSource == NULL )
    {
        ret = -1;
        LogError( ( "Invalid input, pVideoSource: %p", pVideoSource ) );
    }

    if( ret == 0 )
    {
        retMessageQueue = MessageQueue_Create( &pVideoSource->dataTxQueue,
                                               DEMO_TRANSCEIVER_VIDEO_TX_DATA_QUEUE_NAME,
                                               sizeof( MediaFrame_t ),
                                               DEMO_TRANSCEIVER_MAX_TX_QUEUE_MSG_NUM );
        if( retMessageQueue != MESSAGE_QUEUE_RESULT_OK )
        {
            LogError( ( "Fail to open video transceiver data queue." ) );
            ret = -1;
        }
    }

    if( ret == 0 )
    {
        /* Initialize video transceiver. */
        pVideoSource->trackKind = TRANSCEIVER_TRACK_KIND_VIDEO;
    }

    if( ret == 0 )
    {
        /* Create task for video Tx. */
        if( xTaskCreate( VideoTx_Task,
                         ( ( const char * )"VideoTxTask" ),
                         2048,
                         pVideoSource,
                         tskIDLE_PRIORITY + 2,
                         NULL ) != pdPASS )
        {
            LogError( ( "xTaskCreate(VideoTask) failed" ) );
            ret = -1;
        }
    }

    return ret;
}

static int32_t InitializeAudioSource( AppMediaSourceContext_t * pAudioSource )
{
    int32_t ret = 0;
    MessageQueueResult_t retMessageQueue;

    if( pAudioSource == NULL )
    {
        ret = -1;
        LogError( ( "Invalid input, pAudioSource: %p", pAudioSource ) );
    }

    if( ret == 0 )
    {
        retMessageQueue = MessageQueue_Create( &pAudioSource->dataTxQueue,
                                               DEMO_TRANSCEIVER_AUDIO_TX_DATA_QUEUE_NAME,
                                               sizeof( MediaFrame_t ),
                                               DEMO_TRANSCEIVER_MAX_TX_QUEUE_MSG_NUM );
        if( retMessageQueue != MESSAGE_QUEUE_RESULT_OK )
        {
            LogError( ( "Fail to open audio transceiver data queue." ) );
            ret = -1;
        }
    }

    if( ret == 0 )
    {
        /* Initialize audio transceiver. */
        pAudioSource->trackKind = TRANSCEIVER_TRACK_KIND_AUDIO;
    }

    if( ret == 0 )
    {
        /* Create task for audio Tx. */
        if( xTaskCreate( AudioTx_Task,
                         ( ( const char * )"AudioTxTask" ),
                         2048,
                         pAudioSource,
                         tskIDLE_PRIORITY + 1,
                         NULL ) != pdPASS )
        {
            LogError( ( "xTaskCreate(AudioTxTask) failed" ) );
            ret = -1;
        }
    }

    #if MEDIA_PORT_ENABLE_AUDIO_RECV
    if( ret == 0 )
    {
        retMessageQueue = MessageQueue_Create( &pAudioSource->dataRxQueue,
                                               DEMO_TRANSCEIVER_AUDIO_RX_DATA_QUEUE_NAME,
                                               sizeof( MediaFrame_t ),
                                               DEMO_TRANSCEIVER_MAX_RX_QUEUE_MSG_NUM );
        if( retMessageQueue != MESSAGE_QUEUE_RESULT_OK )
        {
            LogError( ( "Fail to open audio transceiver data queue." ) );
            ret = -1;
        }
    }

    if( ret == 0 )
    {
        /* Create task for audio Rx. */
        if( xTaskCreate( AudioRx_Task,
                         ( ( const char * )"AudioRxTask" ),
                         2048,
                         pAudioSource,
                         tskIDLE_PRIORITY + 1,
                         NULL ) != pdPASS )
        {
            LogError( ( "xTaskCreate(AudioRxTask) failed" ) );
            ret = -1;
        }
    }
    #endif /* MEDIA_PORT_ENABLE_AUDIO_RECV */

    return ret;
}

static int32_t OnFrameReadyToSend( void * pCtx,
                                   MediaFrame_t * pFrame )
{
    int32_t ret = 0;
    AppMediaSourceContext_t * pMediaSource = ( AppMediaSourceContext_t * )pCtx;
    MessageQueueResult_t retMessageQueue;
    MediaFrame_t dropFrame;
    size_t dropFrameSize;

    if( ( pCtx == NULL ) || ( pFrame == NULL ) )
    {
        LogError( ( "Invalid input, pCtx: %p, pFrame: %p", pCtx, pFrame ) );
        ret = -1;
    }

    if( ret == 0 )
    {
        retMessageQueue = MessageQueue_IsFull( &pMediaSource->dataTxQueue );
        /* Drop oldest packet if full. */
        if( retMessageQueue == MESSAGE_QUEUE_RESULT_MQ_IS_FULL )
        {
            dropFrameSize = sizeof( MediaFrame_t );
            ( void ) MessageQueue_Recv( &pMediaSource->dataTxQueue,
                                        &dropFrame,
                                        &dropFrameSize );

            if( dropFrame.freeData )
            {
                vPortFree( dropFrame.pData );
            }
        }
    }

    if( ret == 0 )
    {
        retMessageQueue = MessageQueue_Send( &pMediaSource->dataTxQueue,
                                             pFrame,
                                             sizeof( MediaFrame_t ) );
        if( retMessageQueue != MESSAGE_QUEUE_RESULT_OK )
        {
            LogError( ( "Fail to send frame ready message to queue, error: %d", retMessageQueue ) );
            ret = -1;
        }
    }

    return ret;
}

int32_t AppMediaSource_Init( AppMediaSourcesContext_t * pCtx,
                             AppMediaSourceOnMediaSinkHook onMediaSinkHookFunc,
                             void * pOnMediaSinkHookCustom )
{
    int32_t ret = 0;

    if( pCtx == NULL )
    {
        LogError( ( "Invalid input, pCtx: %p", pCtx ) );
        ret = -1;
    }

    if( ret == 0 )
    {
        memset( pCtx,
                0,
                sizeof( AppMediaSourcesContext_t ) );

        /* Mutex can only be created in executing scheduler. */
        pCtx->mediaMutex = xSemaphoreCreateMutex();
        if( pCtx->mediaMutex == NULL )
        {
            LogError( ( "Fail to create mutex for media source." ) );
            ret = -1;
        }
    }

    if( ret == 0 )
    {
        pCtx->videoContext.pSourcesContext = pCtx;
        ret = InitializeVideoSource( &pCtx->videoContext );
    }

    if( ret == 0 )
    {
        pCtx->audioContext.pSourcesContext = pCtx;
        ret = InitializeAudioSource( &pCtx->audioContext );
    }

    if( ret == 0 )
    {
        pCtx->onMediaSinkHookFunc = onMediaSinkHookFunc;
        pCtx->pOnMediaSinkHookCustom = pOnMediaSinkHookCustom;
    }

    if( ret == 0 )
    {
        ret = AppMediaSourcePort_Init();
    }

    return ret;
}

int32_t AppMediaSource_InitVideoTransceiver( AppMediaSourcesContext_t * pCtx,
                                             Transceiver_t * pVideoTranceiver )
{
    int32_t ret = 0;

    if( ( pCtx == NULL ) || ( pVideoTranceiver == NULL ) )
    {
        LogError( ( "Invalid input, pCtx: %p, pVideoTranceiver: %p", pCtx, pVideoTranceiver ) );
        ret = -1;
    }

    if( ret == 0 )
    {
        /* Initialize video transceiver. */
        memset( pVideoTranceiver,
                0,
                sizeof( Transceiver_t ) );
        pVideoTranceiver->trackKind = TRANSCEIVER_TRACK_KIND_VIDEO;
        pVideoTranceiver->direction = TRANSCEIVER_TRACK_DIRECTION_SENDRECV;
        #if USE_VIDEO_CODEC_H265
        {
            TRANSCEIVER_ENABLE_CODEC( pVideoTranceiver->codecBitMap,
                                      TRANSCEIVER_RTC_CODEC_H265_BIT );
            pVideoTranceiver->rollingbufferBitRate = TRANSCEIVER_H265_VIDEO_BIT_RATE;
        }
        #else
        {
            TRANSCEIVER_ENABLE_CODEC( pVideoTranceiver->codecBitMap,
                                      TRANSCEIVER_RTC_CODEC_H264_PROFILE_42E01F_LEVEL_ASYMMETRY_ALLOWED_PACKETIZATION_BIT );
            pVideoTranceiver->rollingbufferBitRate = TRANSCEIVER_H264_VIDEO_BIT_RATE;
        }
        #endif
        pVideoTranceiver->rollingbufferDurationSec = DEFAULT_TRANSCEIVER_ROLLING_BUFFER_DURACTION_SECOND;
        strncpy( pVideoTranceiver->streamId,
                 DEFAULT_TRANSCEIVER_MEDIA_STREAM_ID,
                 sizeof( pVideoTranceiver->streamId ) );
        pVideoTranceiver->streamIdLength = strlen( DEFAULT_TRANSCEIVER_MEDIA_STREAM_ID );
        strncpy( pVideoTranceiver->trackId,
                 DEFAULT_TRANSCEIVER_VIDEO_TRACK_ID,
                 sizeof( pVideoTranceiver->trackId ) );
        pVideoTranceiver->trackIdLength = strlen( DEFAULT_TRANSCEIVER_VIDEO_TRACK_ID );
        pVideoTranceiver->onPcEventCallbackFunc = HandlePcEventCallback;
        pVideoTranceiver->pOnPcEventCustomContext = &pCtx->videoContext;
    }

    return ret;
}

int32_t AppMediaSource_InitAudioTransceiver( AppMediaSourcesContext_t * pCtx,
                                             Transceiver_t * pAudioTranceiver )
{
    int32_t ret = 0;

    if( ( pCtx == NULL ) || ( pAudioTranceiver == NULL ) )
    {
        LogError( ( "Invalid input, pCtx: %p, pAudioTranceiver: %p", pCtx, pAudioTranceiver ) );
        ret = -1;
    }

    if( ret == 0 )
    {
        /* Initialize audio transceiver. */
        memset( pAudioTranceiver,
                0,
                sizeof( Transceiver_t ) );
        pAudioTranceiver->trackKind = TRANSCEIVER_TRACK_KIND_AUDIO;
        pAudioTranceiver->direction = TRANSCEIVER_TRACK_DIRECTION_SENDRECV;
        #if ( AUDIO_OPUS )
        TRANSCEIVER_ENABLE_CODEC( pAudioTranceiver->codecBitMap,
                                  TRANSCEIVER_RTC_CODEC_OPUS_BIT );
        #elif ( AUDIO_G711_MULAW )
        TRANSCEIVER_ENABLE_CODEC( pAudioTranceiver->codecBitMap,
                                  TRANSCEIVER_RTC_CODEC_MULAW_BIT );
        #elif ( AUDIO_G711_ALAW )
        TRANSCEIVER_ENABLE_CODEC( pAudioTranceiver->codecBitMap,
                                  TRANSCEIVER_RTC_CODEC_ALAW_BIT );
        #endif
        pAudioTranceiver->rollingbufferDurationSec = DEFAULT_TRANSCEIVER_ROLLING_BUFFER_DURACTION_SECOND;
        pAudioTranceiver->rollingbufferBitRate = DEFAULT_TRANSCEIVER_AUDIO_BIT_RATE;
        strncpy( pAudioTranceiver->streamId,
                 DEFAULT_TRANSCEIVER_MEDIA_STREAM_ID,
                 sizeof( pAudioTranceiver->streamId ) );
        pAudioTranceiver->streamIdLength = strlen( DEFAULT_TRANSCEIVER_MEDIA_STREAM_ID );
        strncpy( pAudioTranceiver->trackId,
                 DEFAULT_TRANSCEIVER_AUDIO_TRACK_ID,
                 sizeof( pAudioTranceiver->trackId ) );
        pAudioTranceiver->trackIdLength = strlen( DEFAULT_TRANSCEIVER_AUDIO_TRACK_ID );
        pAudioTranceiver->onPcEventCallbackFunc = HandlePcEventCallback;
        pAudioTranceiver->pOnPcEventCustomContext = &pCtx->audioContext;
    }

    return ret;
}

int32_t AppMediaSource_RecvFrame( AppMediaSourcesContext_t * pCtx,
                                  MediaFrame_t * pFrame )
{
    int32_t ret = 0;

    if( ( pCtx == NULL ) || ( pFrame == NULL ) )
    {
        LogError( ( "Invalid input, pCtx: %p, pFrame: %p", pCtx, pFrame ) );
        ret = -1;
    }

    #ifdef ENABLE_STREAMING_LOOPBACK
        if( ret == 0 )
        {
            if( pCtx->onMediaSinkHookFunc )
            {
                ( void ) pCtx->onMediaSinkHookFunc( pCtx->pOnMediaSinkHookCustom,
                                                    pFrame );
            }
        }
    #else /* ifdef ENABLE_STREAMING_LOOPBACK */
        AppMediaSourceContext_t * pMediaSource = NULL;
        MessageQueueResult_t retMessageQueue;
        MediaFrame_t frame;
        size_t frameSize;
        
        if( ret == 0 )
        {
            #if MEDIA_PORT_ENABLE_AUDIO_RECV
                if( pFrame->trackKind == TRANSCEIVER_TRACK_KIND_AUDIO )
                {
                    pMediaSource = &pCtx->audioContext;
                }
                else
            #endif /* #if MEDIA_PORT_ENABLE_AUDIO_RECV */
            {
                LogDebug( ( "Drop received frame, track kind: %d", pFrame->trackKind ) );
                ret = -1;
            }
        }

        if( ret == 0 )
        {
            retMessageQueue = MessageQueue_IsFull( &pMediaSource->dataRxQueue );

            /* Drop oldest packet if full. */
            if( retMessageQueue == MESSAGE_QUEUE_RESULT_MQ_IS_FULL )
            {
                frameSize = sizeof( MediaFrame_t );
                ( void ) MessageQueue_Recv( &pMediaSource->dataRxQueue,
                                            &frame,
                                            &frameSize );

                if( frame.freeData )
                {
                    vPortFree( frame.pData );
                }
            }
            else if( retMessageQueue != MESSAGE_QUEUE_RESULT_MQ_IS_NOT_FULL )
            {
                LogError( ( "Unexpected return value from Rx queue, result: %d", retMessageQueue ) );
                ret = -1;
            }
            else
            {
                /* Empty else marker. */
            }
        }

        if( ret == 0 )
        {
            /* Allocate memory and handle frame in port layer. */
            frame.pData = pvPortMalloc( pFrame->size );
            if( frame.pData == NULL )
            {
                LogError( ( "Fail to allocate memory for Rx audio frame." ) );
                ret = -1;
            }
            else
            {
                memcpy( frame.pData, pFrame->pData, pFrame->size );
                frame.freeData = 1;
                frame.size = pFrame->size;
                frame.timestampUs = pFrame->timestampUs;
                frame.trackKind = pFrame->trackKind;
            }
        }

        if( ret == 0 )
        {
            retMessageQueue = MessageQueue_Send( &pMediaSource->dataRxQueue,
                                                 &frame,
                                                 sizeof( MediaFrame_t ) );
            if( retMessageQueue != MESSAGE_QUEUE_RESULT_OK )
            {
                LogError( ( "Fail to send frame ready message to queue, error: %d", retMessageQueue ) );
                ret = -1;

                if( frame.freeData )
                {
                    vPortFree( frame.pData );
                }
            }
        }
    #endif /* ifdef ENABLE_STREAMING_LOOPBACK */

    return ret;
}
