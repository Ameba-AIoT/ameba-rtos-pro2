/*
 * Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */
/******************************************************************************
 * HEADERS
 ******************************************************************************/
#define LOG_CLASS "AppMediaSrc_AmebaPro2"
#include "AppMediaSrc_AmebaPro2.h"
#include "AppCommon.h"
#include "fileio.h"

#include "../sample_config_webrtc.h"
#include "../module_kvs_webrtc.h"
#include "avcodec.h"
#include "sntp/sntp.h"

/******************************************************************************
 * DEFINITIONS
 ******************************************************************************/

/* used to monitor skb resource */
extern int skbbuf_used_num;
extern int skbdata_used_num;
extern int max_local_skb_num;
extern int max_skb_buf_num;


typedef struct {
	RTC_CODEC codec;
	PBYTE pFrameBuffer;
	UINT32 frameBufferSize;
} CodecStreamConf, *PCodecStreamConf;

typedef struct {
	STATUS codecStatus;
	CodecStreamConf videoStream;
	CodecStreamConf audioStream;
} CodecConfiguration, *PCodecConfiguration;

typedef struct {
	// the codec.
	volatile ATOMIC_BOOL bIsRunning;//!< the media source is running or not.
	volatile ATOMIC_BOOL bTerminated;//!< the status of the termination of media sources.
	volatile ATOMIC_BOOL bCodecConfigLatched;

	MUTEX sourceRunnerLock;
	CodecConfiguration codecConfiguration;  //!< the configuration of gstreamer.
	// for meida output.
	PVOID mediaSinkHookUserdata;
	MediaSinkHook mediaSinkHook;
	PVOID mediaEosHookUserdata;
	MediaEosHook mediaEosHook;

} CodecSrcContext, *PCodecSrcContext;

/******************************************************************************
 * FUNCTIONS
 ******************************************************************************/
int app_media_h264_is_i_frame(u8 *frame_buf)
{
	if ((frame_buf[4] & 0x1F) == 7) {
		return 1;
	} else {
		return 0;
	}
}

VOID app_media_sendMediaFrame(PVOID args, PVOID pFrame)
{
	PAppConfiguration pAppConfiguration = (PAppConfiguration)args;
	PCodecSrcContext pCodecSrcContext = (PCodecSrcContext)pAppConfiguration->pMediaContext;

	if (ATOMIC_LOAD_BOOL(&pCodecSrcContext->bIsRunning)) {
		/* wait for skb resource release */
		if ((skbdata_used_num > (max_skb_buf_num - 64)) || (skbbuf_used_num > (max_local_skb_num - 64))) {
			return; //skip this frame and wait for skb resource release.
		}

		if (pCodecSrcContext->mediaSinkHook != NULL) {
			pCodecSrcContext->mediaSinkHook(pCodecSrcContext->mediaSinkHookUserdata, (Frame *)pFrame);
		}
	}
}

static void kvsWebrtcMediaQueueFlushToKeyFrame(QueueHandle_t xQueue)
{
	webrtc_mm_t tmp_mm;
	while (1) {
		if (xQueuePeek(xQueue, &tmp_mm, 50 / portTICK_PERIOD_MS) != pdTRUE) {
			break;
		} else {
			if (app_media_h264_is_i_frame(tmp_mm.pData)) {
				break;
			} else if (xQueueReceive(xQueue, &tmp_mm, 0) == pdTRUE) {
				if (tmp_mm.pData && tmp_mm.bFreeData) {
					free(tmp_mm.pData);
				}
			}
		}
	}
}

static PVOID priv_app_media_sendVideoFrame(PVOID args)
{
	STATUS retStatus = STATUS_SUCCESS;
	PCodecSrcContext pCodecSrcContext = (PCodecSrcContext) args;
	CHK(pCodecSrcContext != NULL, STATUS_MEDIA_NULL_ARG);
	Frame frame;
	UINT32 frameSize;
	STATUS status;
	frame.presentationTs = 0;

	webrtc_mm_t mm;
	DLOGD("The video source is up");

	kvsWebrtcMediaQueueFlushToKeyFrame(pkvsWebrtcMediaQ->VideoSendQueue);

	while (ATOMIC_LOAD_BOOL(&pCodecSrcContext->bIsRunning)) {
		if (xQueueReceive(pkvsWebrtcMediaQ->VideoSendQueue, &mm, 50 / portTICK_PERIOD_MS) != pdTRUE) {
			continue;
		}

		frame.flags = app_media_h264_is_i_frame(mm.pData) ? FRAME_FLAG_KEY_FRAME : FRAME_FLAG_NONE;
		frame.frameData = mm.pData;
		frame.size = mm.size;
		frame.presentationTs = getEpochTimestampInHundredsOfNanos(&mm.timestamp);

		frame.trackId = DEFAULT_VIDEO_TRACK_ID;
		frame.version = FRAME_CURRENT_VERSION;
		frame.decodingTs = frame.presentationTs;

		/* wait for skb resource release */
		if ((skbdata_used_num > (max_skb_buf_num - 64)) || (skbbuf_used_num > (max_local_skb_num - 64))) {
			if (mm.pData != NULL && mm.bFreeData) {
				SAFE_MEMFREE(mm.pData);
			}
			continue; //skip this frame and wait for skb resource release.
		}

		if (pCodecSrcContext->mediaSinkHook != NULL) {
			retStatus = pCodecSrcContext->mediaSinkHook(pCodecSrcContext->mediaSinkHookUserdata, &frame);
		}

		if (mm.pData != NULL && mm.bFreeData) {
			SAFE_MEMFREE(mm.pData);
		}
	}

CleanUp:

	CHK_LOG_ERR(retStatus);

	DLOGD("The video source is down");
	THREAD_EXIT(NULL);
	return (PVOID)(ULONG_PTR) retStatus;
}

static PVOID priv_app_media_sendAudioFrame(PVOID args)
{
	STATUS retStatus = STATUS_SUCCESS;
	PCodecSrcContext pCodecSrcContext = (PCodecSrcContext) args;
	Frame frame;
	UINT32 frameSize;
	STATUS status;

	CHK(pCodecSrcContext != NULL, STATUS_MEDIA_NULL_ARG);

	frame.presentationTs = 0;
	webrtc_mm_t mm;

	DLOGD("The audio source is up");
	while (ATOMIC_LOAD_BOOL(&pCodecSrcContext->bIsRunning)) {
		if (xQueueReceive(pkvsWebrtcMediaQ->AudioSendQueue, &mm, 50 / portTICK_PERIOD_MS) != pdTRUE) {
			continue;
		}

		frame.flags = FRAME_FLAG_KEY_FRAME;
		frame.frameData = mm.pData;
		frame.size = mm.size;
		frame.presentationTs = getEpochTimestampInHundredsOfNanos(&mm.timestamp);

		frame.trackId = DEFAULT_AUDIO_TRACK_ID;
		frame.version = FRAME_CURRENT_VERSION;
		frame.decodingTs = frame.presentationTs;

		// wait for skb resource release
		if ((skbdata_used_num > (max_skb_buf_num - 64)) || (skbbuf_used_num > (max_local_skb_num - 64))) {
			if (mm.pData != NULL && mm.bFreeData) {
				SAFE_MEMFREE(mm.pData);
			}
			//skip this frame and wait for skb resource release.
			continue;
		}

		if (pCodecSrcContext->mediaSinkHook != NULL) {
			retStatus = pCodecSrcContext->mediaSinkHook(pCodecSrcContext->mediaSinkHookUserdata, &frame);
		}

		if (mm.pData != NULL && mm.bFreeData) {
			SAFE_MEMFREE(mm.pData);
		}
	}

CleanUp:

	CHK_LOG_ERR(retStatus);
	DLOGD("The audio source is down");
	THREAD_EXIT(NULL);
	return (PVOID)(ULONG_PTR) retStatus;
}

/**
 * @brief   polling the status of media source.
 * @param[in] pMediaContext the context of the media source.
 *
 * @return STATUS code of the execution. STATUS_SUCCESS on success.
 */
static STATUS app_media_source_isReady(PMediaContext pMediaContext)
{
	STATUS retStatus = STATUS_SUCCESS;
	PCodecSrcContext pCodecSrcContext = (PCodecSrcContext) pMediaContext;
	CHK(pCodecSrcContext != NULL, STATUS_MEDIA_NULL_ARG);

	if (ATOMIC_LOAD_BOOL(&pCodecSrcContext->bCodecConfigLatched)) {
		retStatus = STATUS_SUCCESS;
	} else {
		retStatus = STATUS_MEDIA_NOT_READY;
	}

CleanUp:

	return retStatus;
}

static STATUS app_media_source_isRunning(PMediaContext pMediaContext)
{
	STATUS retStatus = STATUS_SUCCESS;
	PCodecSrcContext pCodecSrcContext = (PCodecSrcContext) pMediaContext;
	CHK(pCodecSrcContext != NULL, STATUS_MEDIA_NULL_ARG);

	if (ATOMIC_LOAD_BOOL(&pCodecSrcContext->bIsRunning)) {
		retStatus = STATUS_SUCCESS;
	} else {
		retStatus = STATUS_MEDIA_NOT_RUNNING;
	}
CleanUp:
	return retStatus;
}
/**
 * @brief   query the video capability of media.
 * @param[in] pMediaContext the context of the media source.
 * @param[in, out] pCodec the codec of the media source.
 *
 * @return STATUS code of the execution. STATUS_SUCCESS on success.
 */
static STATUS app_media_source_queryVideoCap(PMediaContext pMediaContext, RTC_CODEC *pCodec)
{
	STATUS retStatus = STATUS_SUCCESS;
	PCodecSrcContext pCodecSrcContext = (PCodecSrcContext) pMediaContext;
	PCodecStreamConf pVideoStream;
	CHK((pCodecSrcContext != NULL) && (pCodec != NULL), STATUS_MEDIA_NULL_ARG);
	CHK(ATOMIC_LOAD_BOOL(&pCodecSrcContext->bCodecConfigLatched), STATUS_MEDIA_NOT_READY);
	pVideoStream = &pCodecSrcContext->codecConfiguration.videoStream;
	*pCodec = pVideoStream->codec;
CleanUp:
	return retStatus;
}
/**
 * @brief   query the audio capability of media.
 * @param[in] pMediaContext the context of the media source.
 * @param[in, out] pCodec the codec of the media source.
 *
 * @return STATUS code of the execution. STATUS_SUCCESS on success.
 */
static STATUS app_media_source_queryAudioCap(PMediaContext pMediaContext, RTC_CODEC *pCodec)
{
	STATUS retStatus = STATUS_SUCCESS;
	PCodecSrcContext pCodecSrcContext = (PCodecSrcContext) pMediaContext;
	PCodecStreamConf pAudioStream;
	CHK((pCodecSrcContext != NULL), STATUS_MEDIA_NULL_ARG);
	CHK(ATOMIC_LOAD_BOOL(&pCodecSrcContext->bCodecConfigLatched), STATUS_MEDIA_NOT_READY);
	pAudioStream = &pCodecSrcContext->codecConfiguration.audioStream;
	*pCodec = pAudioStream->codec;
CleanUp:
	return retStatus;
}
/**
 * @brief   link the hook function with the media sink.
 *
 *          YOU MUST BE AWARE OF RETURNING ERROR IN THE HOOK CAUSES STREAM TERMINATED.
 *
 * @param[in] pMediaContext the context of the media source.
 * @param[in] mediaSinkHook the function pointer for the hook of media sink.
 * @param[in] udata the user data for the hook.
 *
 * @return STATUS code of the execution. STATUS_SUCCESS on success.
 */
static STATUS app_media_source_linkSinkHook(PMediaContext pMediaContext, MediaSinkHook mediaSinkHook, PVOID udata)
{
	STATUS retStatus = STATUS_SUCCESS;
	PCodecSrcContext pCodecSrcContext = (PCodecSrcContext) pMediaContext;
	CHK(pCodecSrcContext != NULL, STATUS_MEDIA_NULL_ARG);
	pCodecSrcContext->mediaSinkHook = mediaSinkHook;
	pCodecSrcContext->mediaSinkHookUserdata = udata;
CleanUp:
	return retStatus;
}
/**
 * @brief   link the eos hook function with the media source.
 * @param[in] pMediaContext the context of the media source.
 * @param[in] mediaSinkHook the function pointer for the eos hook of media source.
 * @param[in] udata the user data for the hook.
 *
 * @return STATUS code of the execution. STATUS_SUCCESS on success.
 */
static STATUS app_media_source_linkEosHook(PMediaContext pMediaContext, MediaEosHook mediaEosHook, PVOID udata)
{
	STATUS retStatus = STATUS_SUCCESS;
	PCodecSrcContext pCodecSrcContext = (PCodecSrcContext) pMediaContext;
	CHK(pCodecSrcContext != NULL, STATUS_MEDIA_NULL_ARG);
	pCodecSrcContext->mediaEosHook = mediaEosHook;
	pCodecSrcContext->mediaEosHookUserdata = udata;
CleanUp:
	return retStatus;
}
/**
 * @brief   the main thread of media source.
 * @param[in] pArgs the context of the media source.
 *
 * @return STATUS code of the execution. STATUS_SUCCESS on success.
 */
static PVOID app_media_source_run(PVOID pArgs)
{
	STATUS retStatus = STATUS_SUCCESS;
	PCodecSrcContext pCodecSrcContext = (PCodecSrcContext) pArgs;
	TID videoSenderTid = INVALID_TID_VALUE, audioSenderTid = INVALID_TID_VALUE;

	CHK(pCodecSrcContext != NULL, STATUS_MEDIA_NULL_ARG);
	CHK(IS_VALID_MUTEX_VALUE(pCodecSrcContext->sourceRunnerLock), STATUS_MEDIA_INVALID_MUTEX);
	MUTEX_LOCK(pCodecSrcContext->sourceRunnerLock);
	ATOMIC_STORE_BOOL(&pCodecSrcContext->bIsRunning, TRUE);
	ATOMIC_STORE_BOOL(&pCodecSrcContext->bTerminated, FALSE);

	DLOGI("The app media sources are up");
	THREAD_CREATE_EX(&videoSenderTid, APP_MEDIA_VIDEO_SENDER_THREAD_NAME, APP_MEDIA_VIDEO_SENDER_THREAD_SIZE, TRUE, priv_app_media_sendVideoFrame,
					 (PVOID) pCodecSrcContext);
	THREAD_CREATE_EX(&audioSenderTid, APP_MEDIA_AUDIO_SENDER_THREAD_NAME, APP_MEDIA_AUDIO_SENDER_THREAD_SIZE, TRUE, priv_app_media_sendAudioFrame,
					 (PVOID) pCodecSrcContext);

	if (videoSenderTid != INVALID_TID_VALUE) {
		THREAD_JOIN(videoSenderTid, NULL);
	} else {
		DLOGE("The initialization of video sender failed.");
	}

	if (audioSenderTid != INVALID_TID_VALUE) {
		THREAD_JOIN(audioSenderTid, NULL);
	} else {
		DLOGE("The initialization of audio sender failed.");
	}

CleanUp:

	/* free resources */
	if (pCodecSrcContext->mediaEosHook != NULL) {
		retStatus = pCodecSrcContext->mediaEosHook(pCodecSrcContext->mediaEosHookUserdata);
	}
	CHK_LOG_ERR(retStatus);
	ATOMIC_STORE_BOOL(&pCodecSrcContext->bTerminated, TRUE);
	MUTEX_UNLOCK(pCodecSrcContext->sourceRunnerLock);
	DLOGD("The app media sources are down");
	return (PVOID)(ULONG_PTR) retStatus;
}
/**
 * @brief   shutdown the media source and the main thread will be terminated as well.
 * @param[in] pMediaContext the context of the media source.
 *
 * @return STATUS code of the execution. STATUS_SUCCESS on success.
 */
static STATUS app_media_source_shutdown(PMediaContext pMediaContext)
{
	STATUS retStatus = STATUS_SUCCESS;
	PCodecSrcContext pCodecSrcContext = (PCodecSrcContext) pMediaContext;
	DLOGD("Shutdown media source");
	CHK(pCodecSrcContext != NULL, STATUS_MEDIA_NULL_ARG);
	ATOMIC_STORE_BOOL(&pCodecSrcContext->bIsRunning, FALSE);

	// The app media sources are terminated.
	if (IS_VALID_MUTEX_VALUE(pCodecSrcContext->sourceRunnerLock)) {
		MUTEX_LOCK(pCodecSrcContext->sourceRunnerLock);
		MUTEX_UNLOCK(pCodecSrcContext->sourceRunnerLock);
	}
CleanUp:
	return retStatus;
}

static STATUS app_media_source_isShutdown(PMediaContext pMediaContext, PBOOL pShutdown)
{
	STATUS retStatus = STATUS_SUCCESS;
	PCodecSrcContext pCodecSrcContext = (PCodecSrcContext) pMediaContext;

	CHK(pCodecSrcContext != NULL, STATUS_MEDIA_NULL_ARG);
	*pShutdown = !ATOMIC_LOAD_BOOL(&pCodecSrcContext->bIsRunning);
CleanUp:
	return retStatus;
}
/**
 * @brief   destroy the context of media source.
 * @param[in] PMediaContext the context of the media source.
 *
 * @return STATUS code of the execution. STATUS_SUCCESS on success.
 */
static STATUS app_media_source_detroy(PMediaContext *ppMediaContext)
{
	STATUS retStatus = STATUS_SUCCESS;
	PCodecSrcContext pCodecSrcContext;

	DLOGD("Destroy media source");
	CHK(ppMediaContext != NULL, STATUS_MEDIA_NULL_ARG);
	pCodecSrcContext = (PCodecSrcContext) * ppMediaContext;
	CHK(pCodecSrcContext != NULL, STATUS_MEDIA_NULL_ARG);

	if (!ATOMIC_LOAD_BOOL(&pCodecSrcContext->bTerminated)) {
		app_media_source_shutdown(pCodecSrcContext);
	}

	if (IS_VALID_MUTEX_VALUE(pCodecSrcContext->sourceRunnerLock)) {
		MUTEX_FREE(pCodecSrcContext->sourceRunnerLock);
		pCodecSrcContext->sourceRunnerLock = INVALID_MUTEX_VALUE;
	}

	MEMFREE(pCodecSrcContext);
	*ppMediaContext = pCodecSrcContext = NULL;
CleanUp:
	return retStatus;
}
/**
 * @brief   initialize the context of media.
 * @param[in, out] ppMediaContext create the context of the media source, initialize it and return it.
 *
 * @return STATUS code of the execution. STATUS_SUCCESS on success.
 */
static STATUS app_media_source_init(PMediaContext *ppMediaContext)
{
	STATUS retStatus = STATUS_SUCCESS;
	PCodecSrcContext pCodecSrcContext = NULL;
	PCodecConfiguration pGstConfiguration;
	PCodecStreamConf pVideoStream;
	PCodecStreamConf pAudioStream;

	CHK(ppMediaContext != NULL, STATUS_MEDIA_NULL_ARG);
	*ppMediaContext = NULL;
	CHK(NULL != (pCodecSrcContext = (PCodecSrcContext) MEMCALLOC(1, SIZEOF(CodecSrcContext))), STATUS_MEDIA_NOT_ENOUGH_MEMORY);

	ATOMIC_STORE_BOOL(&pCodecSrcContext->bIsRunning, FALSE);
	ATOMIC_STORE_BOOL(&pCodecSrcContext->bTerminated, FALSE);
	ATOMIC_STORE_BOOL(&pCodecSrcContext->bCodecConfigLatched, TRUE);

	pGstConfiguration = &pCodecSrcContext->codecConfiguration;
	pGstConfiguration->codecStatus = STATUS_SUCCESS;
	pVideoStream = &pGstConfiguration->videoStream;
	pAudioStream = &pGstConfiguration->audioStream;
	pVideoStream->codec = RTC_CODEC_H264_PROFILE_42E01F_LEVEL_ASYMMETRY_ALLOWED_PACKETIZATION_MODE;
#if AUDIO_OPUS
	printf("AUDIO_OPUS\r\n");
	pAudioStream->codec = RTC_CODEC_OPUS;
#elif AUDIO_G711_MULAW
	printf("AUDIO_G711_MULAW\r\n");
	pAudioStream->codec = RTC_CODEC_MULAW;
#elif AUDIO_G711_ALAW
	printf("AUDIO_G711_ALAW\r\n");
	pAudioStream->codec = RTC_CODEC_ALAW;
#endif

	pCodecSrcContext->sourceRunnerLock = MUTEX_CREATE(FALSE);
	CHK(IS_VALID_MUTEX_VALUE(pCodecSrcContext->sourceRunnerLock), STATUS_MEDIA_INVALID_MUTEX);
	*ppMediaContext = pCodecSrcContext;

CleanUp:

	if (STATUS_FAILED(retStatus)) {
		if (pCodecSrcContext != NULL) {
			app_media_source_detroy(pCodecSrcContext);
		}
	}

	return retStatus;
}

AppMediaSrc gAppMediaSrc = {
	.app_media_source_init = app_media_source_init,
	.app_media_source_isReady = app_media_source_isReady,
	.app_media_source_queryVideoCap = app_media_source_queryVideoCap,
	.app_media_source_queryAudioCap = app_media_source_queryAudioCap,
	.app_media_source_linkSinkHook = app_media_source_linkSinkHook,
	.app_media_source_linkEosHook = app_media_source_linkEosHook,
	.app_media_source_run = app_media_source_run,
	.app_media_source_shutdown = app_media_source_shutdown,
	.app_media_source_isShutdown = app_media_source_isShutdown,
	.app_media_source_detroy = app_media_source_detroy
};

#ifdef ENABLE_AUDIO_SENDRECV
extern void kvsWebrtcMediaSendToQueue(webrtc_mm_t *pMediaItem, QueueHandle_t xQueue);

static void priv_app_media_frameHandler(uint64_t customData, PFrame pFrame)
{
	UNUSED_PARAM(customData);
	DLOGV("Frame received. TrackId: %" PRIu64 ", Size: %u, Flags %u", pFrame->trackId, pFrame->size, pFrame->flags);

	webrtc_mm_t remote_audio;
	remote_audio.pData = malloc(pFrame->size);
	if (!remote_audio.pData) {
		printf("fail to allocate memory for webrtc receiving video frame\r\n");
		while (1);
	}
	memcpy(remote_audio.pData, pFrame->frameData, pFrame->size);
	remote_audio.size = pFrame->size;
	remote_audio.timestamp = pFrame->presentationTs / HUNDREDS_OF_NANOS_IN_A_MILLISECOND;
	remote_audio.type =  AUDIO_OPUS ? AV_CODEC_ID_OPUS : (AUDIO_G711_MULAW ? AV_CODEC_ID_PCMU : AV_CODEC_ID_PCMA);
	remote_audio.bFreeData = true;

	kvsWebrtcMediaSendToQueue(&remote_audio, pkvsWebrtcMediaQ->AudioRecvQueue);

	PStreamingSession pStreamingSession = (PStreamingSession) customData;
	if (pStreamingSession->firstFrame) {
		pStreamingSession->firstFrame = FALSE;
		pStreamingSession->startUpLatency = (GETTIME() - pStreamingSession->offerReceiveTime) / HUNDREDS_OF_NANOS_IN_A_MILLISECOND;
		DLOGI("Start up latency from offer to first frame: %" PRIu64 "ms\n", pStreamingSession->startUpLatency);
	}
}

PVOID app_media_sink_onFrame(PVOID pArgs)
{
	STATUS retStatus = STATUS_SUCCESS;
	PStreamingSession pStreamingSession = (PStreamingSession) pArgs;

	CHK(pStreamingSession != NULL, STATUS_MEDIA_NULL_ARG);

	CHK(rtp_transceiver_onFrame(pStreamingSession->pAudioRtcRtpTransceiver, (uint64_t) pStreamingSession, priv_app_media_frameHandler) == STATUS_SUCCESS,
		STATUS_MEDIA_NULL_ARG);

CleanUp:
	return (PVOID)(ULONG_PTR) retStatus;
}
#endif /* ENABLE_AUDIO_SENDRECV */

/**
 * @brief return epoch time in hundreds of nanosecond
*/
uint64_t getEpochTimestampInHundredsOfNanos(void *pTick)
{
	uint64_t timestamp;

	long sec;
	long usec;
	unsigned int tick;
	unsigned int tickDiff;

	sntp_get_lasttime(&sec, &usec, &tick);

	if ((void *)pTick == NULL) {
		tickDiff = xTaskGetTickCount() - tick;
	} else {
		tickDiff = (*(PUINT32)pTick) - tick;
	}

	sec += tickDiff / configTICK_RATE_HZ;
	usec += ((tickDiff % configTICK_RATE_HZ) / portTICK_RATE_MS) * 1000;

	if (usec >= 1000000) {
		usec -= 1000000;
		sec ++;
	}

	timestamp = ((uint64_t)sec * 1000 + usec / 1000) * HUNDREDS_OF_NANOS_IN_A_MILLISECOND;

	return timestamp;
}
