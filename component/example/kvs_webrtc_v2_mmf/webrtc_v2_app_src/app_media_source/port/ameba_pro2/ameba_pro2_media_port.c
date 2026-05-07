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

#include "module_kvs_webrtc_v2.h"
#include "logging.h"
#include "demo_config.h"
#include "platform_opts.h"

#include "avcodec.h"
#include "video_api.h"

#include "FreeRTOS.h"

#if METRIC_PRINT_ENABLED
#include "metric.h"
#endif


int32_t AppMediaSourcePort_Init( void )
{
    LogInfo( ( "[WebRTC Port] AppMediaSourcePort_Init called. Hardware is ready." ) );
    
    return 0;
}

int32_t AppMediaSourcePort_Start( OnFrameReadyToSend_t onVideoFrameReadyToSendFunc,
                                  void * pOnVideoFrameReadyToSendCustomContext,
                                  OnFrameReadyToSend_t onAudioFrameReadyToSendFunc,
                                  void * pOnAudioFrameReadyToSendCustomContext )
{
    int32_t ret = 0;

#if METRIC_PRINT_ENABLED
    Metric_StartEvent( METRIC_EVENT_MEDIA_PORT_START );
#endif
    mm_module_ctrl( pWebrtcMmContext,
                    CMD_KVS_WEBRTC_REG_VIDEO_SEND_CALLBACK,
                    ( int ) onVideoFrameReadyToSendFunc );
    mm_module_ctrl( pWebrtcMmContext,
                    CMD_KVS_WEBRTC_REG_VIDEO_SEND_CALLBACK_CUSTOM_CONTEXT,
                    ( int ) pOnVideoFrameReadyToSendCustomContext );
    mm_module_ctrl( pWebrtcMmContext,
                    CMD_KVS_WEBRTC_REG_AUDIO_SEND_CALLBACK,
                    ( int ) onAudioFrameReadyToSendFunc );
    mm_module_ctrl( pWebrtcMmContext,
                    CMD_KVS_WEBRTC_REG_AUDIO_SEND_CALLBACK_CUSTOM_CONTEXT,
                    ( int ) pOnAudioFrameReadyToSendCustomContext );
    mm_module_ctrl( pWebrtcMmContext,
                    CMD_KVS_WEBRTC_START,
                    0 );
#if METRIC_PRINT_ENABLED
    Metric_EndEvent( METRIC_EVENT_MEDIA_PORT_START );
#endif

    return ret;
}

void AppMediaSourcePort_Stop( void )
{
#if METRIC_PRINT_ENABLED
    Metric_StartEvent( METRIC_EVENT_MEDIA_PORT_STOP );
#endif
    mm_module_ctrl( pWebrtcMmContext, CMD_KVS_WEBRTC_STOP, 0);
#if METRIC_PRINT_ENABLED
    Metric_EndEvent( METRIC_EVENT_MEDIA_PORT_STOP );
#endif
}

void AppMediaSourcePort_Destroy( void )
{
    LogInfo( ( "[WebRTC Port]: AppMediaSourcePort_Destroy called." ) );
 
    /* Nothing to destroy here.
     * Module cleanup is handled by the example via mm_module_close(). */
}

void AppMediaSourcePort_PlayAudioFrame( MediaFrame_t * pFrame )
{
    uint8_t skipProcess = 0U;
    mm_queue_item_t *output_item;

    if( pFrame == NULL )
    {
        LogError( ( "Invalid input, pFrame: %p", pFrame ) );
        skipProcess = 1U;
    }
    else if( pFrame->trackKind != TRANSCEIVER_TRACK_KIND_AUDIO )
    {
        LogError( ( "Dropping non-audio frame, track kind: %d", pFrame->trackKind ) );
        skipProcess = 1U;
    }
    else
    {
        /* Empty else marker. */
    }

    if( skipProcess == 0U )
    {
        LogDebug( ( "Playing audio frame with length: %lu", pFrame->size ) );

        if( xQueueReceive( pWebrtcMmContext->output_recycle, &output_item, 0xFFFFFFFF) == pdTRUE )
        {
            memcpy( ( void * )output_item->data_addr, ( void * ) pFrame->pData, pFrame->size );

            #if AUDIO_G711_MULAW
                output_item->type = AV_CODEC_ID_PCMU;
            #elif AUDIO_G711_ALAW
                output_item->type = AV_CODEC_ID_PCMA;
            #elif AUDIO_OPUS
                output_item->type = AV_CODEC_ID_OPUS;
            #else
                #error "Audio codec is not configured."
            #endif

            output_item->size = pFrame->size;
            output_item->timestamp = pFrame->timestampUs;
            xQueueSend( pWebrtcMmContext->output_ready, (void *)&output_item, 0xFFFFFFFF );
        }
        else
        {
            LogWarn( ( "No free output queue item for frame type: %d", AV_CODEC_ID_OPUS ) );
        }
    }
}
