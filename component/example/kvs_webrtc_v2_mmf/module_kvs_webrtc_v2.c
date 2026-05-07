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

#include "logging.h"
#include "demo_config.h"
#include "module_kvs_webrtc_v2.h"
#include "platform_opts.h"

#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_miso.h"

#include "module_video.h"
#include "module_audio.h"
#include "module_g711.h"
#include "module_opusc.h"
#include "module_opusd.h"
#include "opus_defines.h"

#include "avcodec.h"

#include "video_api.h"

#include "FreeRTOS.h"
#include "networking_utils.h"

#if METRIC_PRINT_ENABLED
#include "metric.h"
#endif

/* used to monitor skb resource */
extern int skbbuf_used_num;
extern int skbdata_used_num;
extern int max_local_skb_num;
extern int max_skb_buf_num;

#define MEDIA_PORT_SKB_BUFFER_THRESHOLD ( 64 )
#define MEDIA_PORT_WEBRTC_AUDIO_FRAME_SIZE ( 256 )

extern void kvs_webrtc_master_entry( void );
mm_context_t * pWebrtcMmContext = NULL;

static int HandleModuleFrameHook( void * p, void * input,void * output )
{
    int ret = 0;
    MediaModuleContext_t * pCtx = ( MediaModuleContext_t * )p;
    MediaFrame_t frame;
    mm_queue_item_t * pInputItem = ( mm_queue_item_t * )input;

    ( void ) output;

    if( pCtx->mediaStart != 0 )
    {
        do
        {
            /* Set SKB buffer threshold to manage memory allocation. Reference:
             * https://github.com/Freertos-kvs-LTS/freertos-kvs-LTS/blob/bd0702130e0b8dfa386e011644ce1bc7e0d7fd09/component/example/kvs_webrtc_mmf/webrtc_app_src/AppMediaSrc_AmebaPro2.c#L86-L88 */
            if( ( skbdata_used_num > ( max_skb_buf_num - MEDIA_PORT_SKB_BUFFER_THRESHOLD ) ) ||
                ( skbbuf_used_num > ( max_local_skb_num - MEDIA_PORT_SKB_BUFFER_THRESHOLD ) ) )
            {
                ret = -1;
                break; //skip this frame and wait for skb resource release.
            }

            frame.size = pInputItem->size;
            frame.pData = ( uint8_t * ) pvPortMalloc( frame.size );
            if( !frame.pData )
            {
                LogWarn( ( "Fail to allocate memory for webrtc media frame, size: %lu", frame.size ) );
                ret = -1;
                break;
            }

            memcpy( frame.pData,
                    ( uint8_t * )pInputItem->data_addr,
                    frame.size );
            frame.freeData = 1;
            frame.timestampUs = NetworkingUtils_GetCurrentTimeUs( &pInputItem->timestamp );

            if( ( pInputItem->type == AV_CODEC_ID_H264 ) || ( pInputItem->type == AV_CODEC_ID_H265 ) )
            {
                if( pCtx->onVideoFrameReadyToSendFunc )
                {
                    frame.trackKind = TRANSCEIVER_TRACK_KIND_VIDEO;
                    ( void ) pCtx->onVideoFrameReadyToSendFunc( pCtx->pOnVideoFrameReadyToSendCustomContext,
                                                                &frame );
                }
                else
                {
                    LogError( ( "No available ready to send callback function pointer for video." ) );
                    vPortFree( frame.pData );
                    ret = -1;
                }
            }
            else if( ( pInputItem->type == AV_CODEC_ID_OPUS ) ||
                     ( pInputItem->type == AV_CODEC_ID_PCMU ) )
            {
                if( pCtx->onAudioFrameReadyToSendFunc )
                {
                    frame.trackKind = TRANSCEIVER_TRACK_KIND_AUDIO;
                    ( void ) pCtx->onAudioFrameReadyToSendFunc( pCtx->pOnAudioFrameReadyToSendCustomContext,
                                                                &frame );
                }
                else
                {
                    LogError( ( "No available ready to send callback function pointer for audio." ) );
                    vPortFree( frame.pData );
                    ret = -1;
                }
            }
            else
            {
                LogWarn( ( "Input type cannot be handled: %ld", pInputItem->type ) );
                vPortFree( frame.pData );
                ret = -1;
            }
        } while( pdFALSE );
    }

    return ret;
}

static void kvs_webrtc_v2_main_thread( void *param )
{
    ( void ) param;
 
    LogInfo( ( "[KVS WebRTC v2 module]: === KVS WebRTC V2 Start ===" ) );
    kvs_webrtc_master_entry();
    LogInfo( ( "[KVS WebRTC v2 module]: kvs_webrtc_master_entry called, master task created" ) );
    vTaskDelete( NULL );
}

static int ControlModuleHook( void * p, int cmd, int arg )
{
    MediaModuleContext_t * pCtx = ( MediaModuleContext_t * )p;

    switch( cmd )
    {
        case CMD_KVS_WEBRTC_SET_APPLY:
            if( xTaskCreate( kvs_webrtc_v2_main_thread, "kvs_webrtc_v2_main", 2048, ( void * )pCtx, tskIDLE_PRIORITY + 1,
                             &pCtx->kvs_webrtc_v2_module_main_task  ) != pdPASS )
            {
                LogError( ( "[KVS WebRTC v2 module]: xTaskCreate(kvs_webrtc_v2_main) failed" ) );
            }
            break;
        case CMD_KVS_WEBRTC_START:
            /* If loopback is enabled, we don't need the camera to provide frames.
             * Instead, we loopback the received frames. */
            #ifdef ENABLE_STREAMING_LOOPBACK
            pCtx->mediaStart = 0;
            #else
            pCtx->mediaStart = 1;
            #endif
            break;
        case CMD_KVS_WEBRTC_STOP:
            pCtx->mediaStart = 0;
            break;
        case CMD_KVS_WEBRTC_REG_VIDEO_SEND_CALLBACK:
            pCtx->onVideoFrameReadyToSendFunc = ( OnFrameReadyToSend_t ) arg;
            break;
        case CMD_KVS_WEBRTC_REG_VIDEO_SEND_CALLBACK_CUSTOM_CONTEXT:
            pCtx->pOnVideoFrameReadyToSendCustomContext = ( void * ) arg;
            break;
        case CMD_KVS_WEBRTC_REG_AUDIO_SEND_CALLBACK:
            pCtx->onAudioFrameReadyToSendFunc = ( OnFrameReadyToSend_t ) arg;
            break;
        case CMD_KVS_WEBRTC_REG_AUDIO_SEND_CALLBACK_CUSTOM_CONTEXT:
            pCtx->pOnAudioFrameReadyToSendCustomContext = ( void * ) arg;
            break;
        default:
            LogWarn( ( "Unknown module command: %d", cmd ) );
            break;
    }
    return 0;
}

static void * DestroyModuleHook( void * p )
{
    MediaModuleContext_t * ctx = ( MediaModuleContext_t * )p;
    if( ctx )
    {
        vPortFree( ctx );
    }
    return NULL;
}

static void * CreateModuleHook( void * parent )
{
    MediaModuleContext_t * ctx = pvPortMalloc( sizeof( MediaModuleContext_t ) );

    if(ctx)
    {
        memset( ctx, 0, sizeof( MediaModuleContext_t ) );
        ctx->pParent = parent;
    }

    return ctx;
}

static void * NewModuleItemHook( void * p )
{
    void * pBuffer = pvPortMalloc( MEDIA_PORT_WEBRTC_AUDIO_FRAME_SIZE * 2 );

    ( void ) p;

    if(pBuffer == NULL)
    {
        LogError( ( "Fail to allocate buffer for module item." ) );
    }

    return pBuffer;
}

static void * DeleteModuleItemHook( void * p,
                                    void * d )
{
    ( void ) p;

    if(d != NULL)
    {
        vPortFree(d);
    }

    return NULL;
}

mm_module_t kvs_webrtc_v2_module  = {
    .create = CreateModuleHook,
    .destroy = DestroyModuleHook,
    .control = ControlModuleHook,
    .handle = HandleModuleFrameHook,

    .new_item = NewModuleItemHook,
    .del_item = DeleteModuleItemHook,

    .output_type = MM_TYPE_ASINK, // output for audio sink
    .module_type = MM_TYPE_AVSINK, // module type is video algorithm
    .name = "KVS_WebRTC_V2"
};



