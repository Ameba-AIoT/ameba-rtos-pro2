/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/

#include <stdint.h>
#include "avcodec.h"

#include "memory_encoder.h"
#include "mmf2_module.h"
#include "module_p2p_audio.h"
#include "mmf2_dbg.h"

#include "psych.h"

#include "SKYNET_IOTAPI.h"
#include "SKYNET_APP.h"

extern AV_Client gClientInfo[MAX_CLIENT_NUMBER];
static uint8_t audioBuf[640];
static int audioSize = 0;

int p2p_audio_handle(void *p, void *input, void *output)
{
	p2p_audio_ctx_t *ctx = (p2p_audio_ctx_t *)p;
	mm_queue_item_t *input_item = (mm_queue_item_t *)input;
	//mm_queue_item_t *output_item = (mm_queue_item_t *)output;
	//(void *) output;

	/*P2 Part*/
	char drop = 1;
	int i;
	int nHeadLen = sizeof(st_AVStreamIOHead) + sizeof(st_AVFrameHead);
	char *pBufAudio;
	st_AVStreamIOHead *pstStreamIOHead;
	st_AVFrameHead    *pstFrameHead;
	extern unsigned long myGetTickCount(void);
	unsigned long nCurTick = myGetTickCount();
	int UserCount = 0, nRet = 0;

	if (ctx->params.enable_stream) {
		drop = 1;
		for (i = 0 ; i < MAX_CLIENT_NUMBER; i++) {
			if (gClientInfo[i].SID >= 0 && gClientInfo[i].bEnableAudio == 1) {
				drop = 0;
			}
		}
		if (drop == 0) {
			uint8_t *stream_data = (uint8_t *)input_item->data_addr;
			for (int j = 0; j < input_item->size; j++) {
				audioBuf[audioSize++] = stream_data[j];

				if (audioSize == 640) {
					for (i = 0 ; i < MAX_CLIENT_NUMBER; i++) { /* send to multi client */
						if (gClientInfo[i].SID < 0 || gClientInfo[i].bEnableAudio == 0) {
							continue;
						}
						xSemaphoreTake(gClientInfo[i].pBuf_mutex, portMAX_DELAY);
						pBufAudio = &gClientInfo[i].pBuf[nHeadLen];
						pstStreamIOHead = (st_AVStreamIOHead *)gClientInfo[i].pBuf;
						pstFrameHead = (st_AVFrameHead *)&gClientInfo[i].pBuf[sizeof(st_AVStreamIOHead)];

						pstFrameHead->nCodecID  = CODECID_A_PCM;
						pstFrameHead->nTimeStamp = nCurTick;
						pstFrameHead->nDataSize = audioSize;
						UserCount++;
						pstFrameHead->nOnlineNum = UserCount;

						pstStreamIOHead->nStreamIOHead = sizeof(st_AVFrameHead) + audioSize;
						pstStreamIOHead->uionStreamIOHead.nStreamIOType = SIO_TYPE_AUDIO;

						if (ctx->params.sample_rate == 8000) {
							pstFrameHead->flag = (ASAMPLE_RATE_8K << 2) | (ADATABITS_16 << 1) | (ACHANNEL_MONO);
						} else if (ctx->params.sample_rate == 16000) {
							pstFrameHead->flag = (ASAMPLE_RATE_16K << 2) | (ADATABITS_16 << 1) | (ACHANNEL_MONO);
						} else {
							printf("Unsupport sample rate for %d\r\n", ctx->params.sample_rate);
							goto p2p_audio_fail;
						}

						memcpy(pBufAudio, audioBuf, audioSize);

						int send_tx = xTaskGetTickCount();
						nRet = SKYNET_send(gClientInfo[i].SID, gClientInfo[i].pBuf, audioSize + nHeadLen);
						if (xTaskGetTickCount() - send_tx > 20) {
							printf("skynet_send time = %d\r\n", xTaskGetTickCount() - send_tx);
						}


						if (nRet < 0) {
							printf("SKYNET_send Audio %d i:%d SID:%d data_size:%d\r\n", nRet, i, gClientInfo[i].SID, audioSize + nHeadLen);
						}

						xSemaphoreGive(gClientInfo[i].pBuf_mutex);
					}
					audioSize = 0;
				}
			}
		}
	}

	return 0;
p2p_audio_fail:
	return 0;
}

int p2p_audio_control(void *p, int cmd, int arg)
{
	p2p_audio_ctx_t *ctx = (p2p_audio_ctx_t *)p;

	switch (cmd) {
	case CMD_P2P_AUDIO_SET_PARAMS:
		memcpy(&ctx->params, ((p2p_audio_params_t *)arg), sizeof(p2p_audio_params_t));
		break;
	case CMD_P2P_AUDIO_GET_PARAMS:
		memcpy(((p2p_audio_params_t *)arg), &ctx->params, sizeof(p2p_audio_params_t));
		break;
	case CMD_P2P_AUDIO_SAMPLERATE:
		ctx->params.sample_rate = arg;
		break;
	case CMD_P2P_AUDIO_CHANNEL:
		ctx->params.channel = arg;
		break;
	case CMD_P2P_AUDIO_STREAMING:
		if (arg) {
			ctx->params.enable_stream = true;
		} else {
			ctx->params.enable_stream = false;
		}
		break;
	case CMD_P2P_AUDIO_APPLY:
		break;
	}

	return 0;
}

void *p2p_audio_destroy(void *p)
{
	p2p_audio_ctx_t *ctx = (p2p_audio_ctx_t *)p;

	if (ctx) {
		free(ctx);
	}

	return NULL;
}

void *p2p_audio_create(void *parent)
{
	p2p_audio_ctx_t *ctx = malloc(sizeof(p2p_audio_ctx_t));
	if (!ctx) {
		return NULL;
	}
	memset(ctx, 0, sizeof(p2p_audio_ctx_t));
	ctx->parent = parent;

	return ctx;
}

void *p2p_audio_new_item(void *p)
{
	p2p_audio_ctx_t *ctx = (p2p_audio_ctx_t *)p;

	return NULL;
}

void *p2p_audio_del_item(void *p, void *d)
{
	p2p_audio_ctx_t *ctx = (p2p_audio_ctx_t *)p;
	return NULL;
}


mm_module_t p2p_audio_module = {
	.create = p2p_audio_create,
	.destroy = p2p_audio_destroy,
	.control = p2p_audio_control,
	.handle = p2p_audio_handle,

	.new_item = p2p_audio_new_item,
	.del_item = p2p_audio_del_item,
	.rsz_item = NULL,

	.output_type = MM_TYPE_NONE,
	.module_type = MM_TYPE_ASINK,
	.name = "P2PAUDIO"
};
