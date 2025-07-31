/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/

#include <stdint.h>
#include "avcodec.h"

#include "memory_encoder.h"
#include "mmf2_module.h"
#include "module_p2p_aac.h"
#include "mmf2_dbg.h"

#include "psych.h"
#include "faac.h"
#include "faac_api.h"

#include "SKYNET_IOTAPI.h"
#include "SKYNET_APP.h"

#define BIAS        (0x84)

extern AV_Client gClientInfo[MAX_CLIENT_NUMBER];

static short seg_end[8] = {0xFF, 0x1FF, 0x3FF, 0x7FF, 0xFFF, 0x1FFF, 0x3FFF, 0x7FFF};

static char audioBuf[640];
static int audioSize = 0;

//------------------------------------------------------------------------------
static short search(short val, short *table, short size)
{
	short i;
	for (i = 0; i < size; i++) {
		if (val <= *table++) {
			return (i);
		}
	}
	return (size);
}

unsigned char linear2ulaw(short pcm_val)    /* 2's complement (16-bit range) */
{
	short     mask;
	short     seg;
	unsigned char   uval;

	/* Get the sign and the magnitude of the value. */
	if (pcm_val < 0) {
		pcm_val = BIAS - pcm_val;
		mask = 0x7F;
	} else {
		pcm_val += BIAS;
		mask = 0xFF;
	}

	/* Convert the scaled magnitude to segment number. */
	seg = search(pcm_val, seg_end, 8);

	/*
	 * Combine the sign, segment, quantization bits;
	 * and complement the code word.
	 */
	if (seg >= 8) {      /* out of range, return maximum value. */
		return (0x7F ^ mask);
	} else {
		uval = (seg << 4) | ((pcm_val >> (seg + 3)) & 0xF);
		return (uval ^ mask);
	}

}

int p2p_aac_handle(void *p, void *input, void *output)
{
	p2p_aac_ctx_t *ctx = (p2p_aac_ctx_t *)p;
	mm_queue_item_t *input_item = (mm_queue_item_t *)input;
	mm_queue_item_t *output_item = (mm_queue_item_t *)output;

	//printf("p2p_aac_handle input size = %d\r\n",input_item->size);

	/*P2 Part*/
	int size = 0;
	char drop = 1;
	int i;
	int nHeadLen = sizeof(st_AVStreamIOHead) + sizeof(st_AVFrameHead);
	char *pBufVideo;//=&a_buf[nHeadLen];
	st_AVStreamIOHead *pstStreamIOHead;//=(st_AVStreamIOHead *)a_buf;
	st_AVFrameHead    *pstFrameHead;//	  =(st_AVFrameHead *)&a_buf[sizeof(st_AVStreamIOHead)];
	unsigned long nCurTick = myGetTickCount();
	int UserCount = 0, nRet = 0, wsize = 0;
	//char audioBuf[200];
	//int audioSize = 0;
	int frameCnt = 0;

	//int samples_read, frame_size;
	int frame_size = 0;

	output_item->timestamp = input_item->timestamp;
	// set timestamp to 1st sample (cache head)
	output_item->timestamp -= 1000 * (ctx->cache_idx / 2) / ctx->params.sample_rate;

	/*
	if (ctx->params.bit_length==8)
		samples_read = input_item->size;
	else if (ctx->params.bit_length==16)
		samples_read = input_item->size/2;
	*/

#if 1
	drop = 1;
	for (i = 0 ; i < MAX_CLIENT_NUMBER; i++) {
		if (gClientInfo[i].SID >= 0 && gClientInfo[i].bEnableAudio == 1) {
			drop = 0;
		}
	}

	if (drop == 0) {
		short *stream_data = (short *)input_item->data_addr;
		for (i = 0; i < input_item->size / 2; i++) {
			audioBuf[i + audioSize] = linear2ulaw(*(stream_data + i));
		}
		audioSize += input_item->size / 2;

		if (audioSize >= 640) {
			for (i = 0 ; i < MAX_CLIENT_NUMBER; i++) { /* send to multi client */
				if (gClientInfo[i].SID < 0 || gClientInfo[i].bEnableAudio == 0) {
					continue;
				}
				xSemaphoreTake(gClientInfo[i].pBuf_mutex, portMAX_DELAY);
				pBufVideo = &gClientInfo[i].pBuf[nHeadLen];
				pstStreamIOHead = (st_AVStreamIOHead *)gClientInfo[i].pBuf;
				pstFrameHead = (st_AVFrameHead *)&gClientInfo[i].pBuf[sizeof(st_AVStreamIOHead)];

				pstFrameHead->nCodecID  = CODECID_A_G711_U;
				pstFrameHead->nTimeStamp = nCurTick;
				pstFrameHead->nDataSize = audioSize;
				UserCount++;
				pstFrameHead->nOnlineNum = UserCount;

				pstStreamIOHead->nStreamIOHead = sizeof(st_AVFrameHead) + audioSize;
				pstStreamIOHead->uionStreamIOHead.nStreamIOType = SIO_TYPE_AUDIO;

				pstFrameHead->flag = (ASAMPLE_RATE_8K << 2) | (ADATABITS_16 << 1) | (ACHANNEL_MONO);

				memcpy(pBufVideo, audioBuf, audioSize);

				nRet = SKYNET_send(gClientInfo[i].SID, gClientInfo[i].pBuf, audioSize + nHeadLen) ;

				/*
				if(nRet == -11) {
					vTaskDelay(10);
					nRet = SKYNET_send(gClientInfo[i].SID, gClientInfo[i].pBuf, audioSize+nHeadLen) ;
					if(nRet == -11) {
						vTaskDelay(10);
						nRet = SKYNET_send(gClientInfo[i].SID, gClientInfo[i].pBuf, audioSize+nHeadLen) ;
					}
				}
				*/

				if (nRet < 0) {
					printf("SKYNET_send Audio %d i:%d SID:%d data_size:%d\r\n", nRet, i, gClientInfo[i].SID, audioSize + nHeadLen);
				}

				xSemaphoreGive(gClientInfo[i].pBuf_mutex);
			}
			audioSize = 0;
		}
	}
#endif

#if 0
	memcpy(ctx->cache + ctx->cache_idx, (void *)input_item->data_addr, input_item->size);
	ctx->cache_idx += input_item->size;

	if (ctx->cache_idx >= ctx->params.samples_input * 2) {
		frame_size = aac_encode_run(ctx->faac_enc, (void *)ctx->cache, ctx->params.samples_input, (unsigned char *)output_item->data_addr,
									ctx->params.max_bytes_output);
		ctx->cache_idx -= ctx->params.samples_input * 2;
		if (ctx->cache_idx > 0) {
			memmove(ctx->cache, ctx->cache + ctx->params.samples_input * 2, ctx->cache_idx);
		}
	}

	output_item->size = frame_size;
	output_item->type = AV_CODEC_ID_MP4A_LATM;
	output_item->index = 0;
#endif
	return frame_size;
}

int p2p_aac_control(void *p, int cmd, int arg)
{
	p2p_aac_ctx_t *ctx = (p2p_aac_ctx_t *)p;

	switch (cmd) {
	case CMD_P2P_AAC_SET_PARAMS:
		memcpy(&ctx->params, ((p2p_aac_params_t *)arg), sizeof(p2p_aac_params_t));
		break;
	case CMD_P2P_AAC_GET_PARAMS:
		memcpy(((p2p_aac_params_t *)arg), &ctx->params, sizeof(p2p_aac_params_t));
		break;
	case CMD_P2P_AAC_SAMPLERATE:
		ctx->params.sample_rate = arg;
		break;
	case CMD_P2P_AAC_CHANNEL:
		ctx->params.channel = arg;
		break;
	case CMD_P2P_AAC_BITLENGTH:
		ctx->params.bit_length = arg;
		break;
	case CMD_P2P_AAC_MEMORY_SIZE:
		ctx->params.mem_total_size = arg;
		break;
	case CMD_P2P_AAC_BLOCK_SIZE:
		ctx->params.mem_block_size = arg;
		break;
	case CMD_P2P_AAC_MAX_FRAME_SIZE:
		ctx->params.mem_frame_size = arg;
		break;
	case CMD_P2P_AAC_INIT_MEM_POOL:
		ctx->mem_pool = memory_init(ctx->params.mem_total_size, ctx->params.mem_block_size);
		if (ctx->mem_pool == NULL) {
			mm_printf("Can't allocate AAC buffer\r\n");
			while (1);
		}
		break;
	case CMD_P2P_AAC_APPLY:
		aac_encode_init(&ctx->faac_enc, ctx->params.bit_length, ctx->params.output_format, ctx->params.sample_rate, ctx->params.channel, ctx->params.mpeg_version,
						&ctx->params.samples_input, &ctx->params.max_bytes_output);
		ctx->cache = (uint8_t *)malloc(ctx->params.samples_input * 2 + 1500);	// 1500 max audio page size
		if (!ctx->cache) {
			mm_printf("Output memory\n\r");
			while (1);
			// TODO add handing code
		}
		break;
	}

	return 0;
}

void *p2p_aac_destroy(void *p)
{
	p2p_aac_ctx_t *ctx = (p2p_aac_ctx_t *)p;

	if (ctx && ctx->mem_pool) {
		memory_deinit(ctx->mem_pool);
	}
	if (ctx) {
		free(ctx);
	}

	return NULL;
}

void *p2p_aac_create(void *parent)
{
	p2p_aac_ctx_t *ctx = malloc(sizeof(p2p_aac_ctx_t));
	if (!ctx) {
		return NULL;
	}
	memset(ctx, 0, sizeof(p2p_aac_ctx_t));
	ctx->parent = parent;

	return ctx;
}

void *p2p_aac_new_item(void *p)
{
	p2p_aac_ctx_t *ctx = (p2p_aac_ctx_t *)p;

	return memory_alloc(ctx->mem_pool, ctx->params.mem_frame_size);
}

void *p2p_aac_del_item(void *p, void *d)
{
	p2p_aac_ctx_t *ctx = (p2p_aac_ctx_t *)p;

	memory_free(ctx->mem_pool, d);
	return NULL;
}

void *p2p_aac_rsz_item(void *p, void *d, int len)
{
	p2p_aac_ctx_t *ctx = (p2p_aac_ctx_t *)p;
	return memory_realloc(ctx->mem_pool, d, len);
}

mm_module_t p2p_aac_module = {
	.create = p2p_aac_create,
	.destroy = p2p_aac_destroy,
	.control = p2p_aac_control,
	.handle = p2p_aac_handle,

	.new_item = p2p_aac_new_item,
	.del_item = p2p_aac_del_item,
	.rsz_item = p2p_aac_rsz_item,

	.output_type = MM_TYPE_ASINK,
	.module_type = MM_TYPE_ADSP,
	.name = "P2PAAC"
};
