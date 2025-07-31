/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/

#include <stdint.h>
#include "lwipconf.h"
#include "mmf2_module.h"
#include "module_p2p.h"
#include "module_video.h"
#include "mmf2_dbg.h"
//------------------------------------------------------------------------------

#include "SKYNET_IOTAPI.h"
#include "SKYNET_APP.h"
#include <wifi_conf.h>
extern AV_Client gClientInfo[MAX_CLIENT_NUMBER];
int wait_for_i = 1;
int set_force_i = 0;

#define AV_CODEC_ID_MJPEG 0
#define AV_CODEC_ID_H264  1
#define AV_CODEC_ID_PCMU  2
#define AV_CODEC_ID_PCMA  3
#define AV_CODEC_ID_MP4A_LATM 4
#define AV_CODEC_ID_MP4V_ES 5
#define AV_CODEC_ID_PCM_RAW 6
#define AV_CODEC_ID_UNKNOWN -1

#define P2P_DEBUG_SHOW 0

//extern mm_context_t* h264_v2_ctx;

//static void wait_rf_ready(void)
//{
//while (wifi_sta_is_ready_to_transceive() != RTW_SUCCESS) {
//vTaskDelay(100);
//}
//}

uint32_t timer_send_beg;
uint32_t timer_send_end;

static int SKYNET_send_video(int nSocketID, const char *pBuf, int nBufSize)
{
	int nRet = 0;
	int totalSize = 0;
	int offset = 0;

	//wait_rf_ready();
	//printf("SKYNET_send Video %d i:%d SID:%d data_size:%d\r\n",nRet,nSocketID,nBufSize);
	//timer_send_beg = xTaskGetTickCountFromISR();
	//printf("1:%d\r\n",nBufSize);
	nRet = SKYNET_send(nSocketID, pBuf, nBufSize) ;

	if (nRet == -11) {
		//printf("SKYNET_send 1 Video %d i:%d SID:%d data_size:%d\r\n",nRet,nSocketID,nBufSize);
		//wait_rf_ready();
		//timer_send_end = xTaskGetTickCountFromISR();
		//printf("SKYNET_send 1 Video %d i:%d SID:%d data_size:%d sendtime:%d\r\n", nRet, nSocketID, nBufSize, timer_send_end - timer_send_beg);
		vTaskDelay(1);
		//printf("2:%d\r\n",nBufSize);
		nRet = SKYNET_send(nSocketID, pBuf, nBufSize) ;
		if (nRet == -11) {
			//printf("SKYNET_send Video %d i:%d SID:%d data_size:%d\r\n",nRet,nSocketID,nBufSize);
			//wait_rf_ready();
			//timer_send_end = xTaskGetTickCountFromISR();
			//printf("SKYNET_send 2 Video %d i:%d SID:%d data_size:%d sendtime:%d\r\n", nRet, nSocketID, nBufSize, timer_send_end - timer_send_beg);
			vTaskDelay(10);
			//printf("3:%d\r\n",nBufSize);
			nRet = SKYNET_send(nSocketID, pBuf, nBufSize) ;
			if (nRet == -11) {
				//printf("SKYNET_send Video %d SID:%d data_size:%d\r\n", nRet, nSocketID, nBufSize);
				wait_for_i = 1;
				set_force_i = 1;
			}
		}
	}
	//if(nRet < 0)
	//	printf("SKYNET_send Video %d i:%d SID:%d data_size:%d\r\n",nRet,nSocketID,nBufSize);

	return nRet;
}

int p2p_handle(void *p, void *input, void *output)
{
	int ret = 0;
	p2p_ctx_t *ctx = (p2p_ctx_t *)p;

#if P2P_DEBUG_SHOW
	p2p_state_t *state = &ctx->state;
#endif

	mm_queue_item_t *input_item = (mm_queue_item_t *)input;
	(void)output;

	if (input_item->type == AV_CODEC_ID_H264) {
		char drop = 1;
		int nHeadLen = sizeof(st_AVStreamIOHead) + sizeof(st_AVFrameHead);
		char *pBufVideo;
		st_AVStreamIOHead *pstStreamIOHead;
		st_AVFrameHead    *pstFrameHead;
		unsigned long nCurTick = myGetTickCount();
		int UserCount = 0, nRet = 0, wsize = 0;
		int frameCnt = 0;
		int det_result_state;
		int nal_bytes_left = 0;
		int write_size = 0;
		int offset = 0;
		int i = 0;

#if P2P_DEBUG_SHOW
		if (state->timer_2 == 0) {
			state->timer_1 = xTaskGetTickCountFromISR();
			state->timer_2 = state->timer_1;
		} else {
			state->timer_2 = xTaskGetTickCountFromISR();
			if ((state->timer_2 - state->timer_1) >= 5000) {
				RTSP_DBG_INFO("h264 -- drop_farme:%d drop_i_frame:%d drop_p_frame:%d drop_i_frame_size:%d drop_p_frame_size:%d",
							  state->drop_frame, state->drop_i_frame, state->drop_p_frame, state->drop_i_frame_size, state->drop_p_frame_size);

				RTSP_DBG_INFO("h264 -- frame_total:%d iframe_total:%d bitrate_avg:%d iframe_avg:%d",
							  state->frame_total, state->iframe_total, state->bitrate_avg, state->iframe_avg);

				state->timer_2 = 0;
				state->drop_frame = 0;
				state->drop_i_frame = 0;
				state->drop_i_frame_size = 0;
				state->drop_p_frame = 0;
				state->drop_p_frame_size = 0;
			}
		}
#endif

		if (set_force_i == 1) {
			set_force_i = 0;
			//h264_set_force_iframe(h264_v2_ctx);
			extern mm_context_t *video_v1_ctx;
			mm_module_ctrl(video_v1_ctx, CMD_VIDEO_FORCE_IFRAME, NULL);
		}

		for (i = 0 ; i < MAX_CLIENT_NUMBER; i++) {
			if (gClientInfo[i].SID >= 0 && gClientInfo[i].bEnableVideo == 1) {
				drop = 0;
			}
		}

		uint8_t *stream_data = (uint8_t *)input_item->data_addr;


		if (*(stream_data + 4) == 0x27) {
			wait_for_i = 0;
		}

		if ((drop == 0) /*&& (wait_for_i == 0)*/) {

			/*
			    	printf("length:%d data:%x %x %x %x %x %x %x %x\r\n",input_item->size,
			    	*stream_data,*(stream_data+1),*(stream_data+2),*(stream_data+3),
			    	*(stream_data+4),*(stream_data+5),*(stream_data+6),
			    	*(stream_data+7));
			*/

			UserCount = 0;
			for (i = 0 ; i < MAX_CLIENT_NUMBER; i++) { /* send to multi client */
				if (gClientInfo[i].SID < 0 || gClientInfo[i].bEnableVideo == 0) {
					continue;
				}

				xSemaphoreTake(gClientInfo[i].pBuf_mutex, portMAX_DELAY);
				//Video Part
				pBufVideo = &gClientInfo[i].pBuf[nHeadLen];
				pstStreamIOHead = (st_AVStreamIOHead *)gClientInfo[i].pBuf;
				pstFrameHead = (st_AVFrameHead *)&gClientInfo[i].pBuf[sizeof(st_AVStreamIOHead)];

				pstFrameHead->nCodecID  = CODECID_V_H264;
				pstFrameHead->nTimeStamp = nCurTick;
				pstFrameHead->nDataSize = input_item->size;
				UserCount++;
				pstFrameHead->nOnlineNum = UserCount;
				pstFrameHead->top_x = 0x55;

				pstStreamIOHead->nStreamIOHead = sizeof(st_AVFrameHead) + pstFrameHead->nDataSize;
				pstStreamIOHead->uionStreamIOHead.nStreamIOType = SIO_TYPE_VIDEO;


				if (*(stream_data + 4) == 0x27) {
					//printf("Sned I Frame\n");
					pstFrameHead->flag = VFRAME_FLAG_I;
					//frameCnt = 0;
#if P2P_DEBUG_SHOW
					state->iframe_avg = ((state->iframe_avg * state->iframe_total) + pstFrameHead->nDataSize + nHeadLen) / (state->iframe_total + 1);
					state->bitrate_avg = ((state->bitrate_avg * state->frame_total) + pstFrameHead->nDataSize + nHeadLen) / (state->frame_total + 1);
					state->iframe_total++;
					state->frame_total++;

#endif

				} else {
					//printf("Sned P Frame\n");
					pstFrameHead->flag = VFRAME_FLAG_P;
					//frameCnt++;
#if P2P_DEBUG_SHOW
					state->bitrate_avg = ((state->bitrate_avg * state->frame_total) + pstFrameHead->nDataSize + nHeadLen) / (state->frame_total + 1);
					state->frame_total++;
					if (wait_for_i == 1) {
						state->drop_frame_total++;
						state->drop_frame++;

						state->drop_p_frame_size = ((state->drop_p_frame_size * state->drop_p_frame) + pstFrameHead->nDataSize + nHeadLen) / (state->drop_p_frame + 1);
						state->drop_p_frame++;
					}
#endif
				}

				if (wait_for_i == 0) {

					if (pstFrameHead->nDataSize < (MAX_SEND_BUF_SIZE - nHeadLen)) {
						memcpy(pBufVideo, stream_data, pstFrameHead->nDataSize);
					}

					if ((pstFrameHead->nDataSize + nHeadLen) < MAX_SEND_BUF_SIZE) {
#if 0
						if ((nRet = SKYNET_send(gClientInfo[i].SID, gClientInfo[i].pBuf, pstFrameHead->nDataSize + nHeadLen)) < 0) {
							printf("\r\n\r\nSKYNET_send Error Video type:%d nRet:%d i:%d SID:%d data_size:%d\r\n\r\n", pstFrameHead->flag, nRet, i, gClientInfo[i].SID,
								   pstFrameHead->nDataSize + nHeadLen);
							wait_for_i = 1;
							vTaskDelay(200 / portTICK_PERIOD_MS);
						}
#else
						//printf("0:%d\r\n",pstFrameHead->nDataSize + nHeadLen);
						if (SKYNET_send_video(gClientInfo[i].SID, gClientInfo[i].pBuf, pstFrameHead->nDataSize + nHeadLen) == -11) {
#if P2P_DEBUG_SHOW
							state->drop_frame_total++;
							state->drop_frame++;
							if (pstFrameHead->flag == VFRAME_FLAG_I) {
								state->drop_i_frame_size = ((state->drop_i_frame_size * state->drop_i_frame) + pstFrameHead->nDataSize + nHeadLen) / (state->drop_i_frame + 1);
								state->drop_i_frame++;
							} else {
								state->drop_p_frame_size = ((state->drop_p_frame_size * state->drop_p_frame) + pstFrameHead->nDataSize + nHeadLen) / (state->drop_p_frame + 1);
								state->drop_p_frame++;
							}
#endif
						}
#endif
					} else {
						printf("Frame size over P2P buffer!\r\n");
						wait_for_i = 1;
#if P2P_DEBUG_SHOW
						state->drop_frame_total++;
						state->drop_frame++;
						if (pstFrameHead->flag == VFRAME_FLAG_I) {
							state->drop_i_frame_size = ((state->drop_i_frame_size * state->drop_i_frame) + pstFrameHead->nDataSize + nHeadLen) / (state->drop_i_frame + 1);
							state->drop_i_frame++;
						} else {
							state->drop_p_frame_size = ((state->drop_p_frame_size * state->drop_p_frame) + pstFrameHead->nDataSize + nHeadLen) / (state->drop_p_frame + 1);
							state->drop_p_frame++;
						}
#endif
						vTaskDelay(200 / portTICK_PERIOD_MS);
					}
				}

				xSemaphoreGive(gClientInfo[i].pBuf_mutex);
			}
		}
	}

p2p_handle_end:
	return ret;
}


int p2p_control(void *p, int cmd, int arg)
{

	switch (cmd) {
	default:
		break;
	}

	return 0;
}

void *p2p_destroy(void *p)
{
	p2p_ctx_t *ctx = (p2p_ctx_t *)p;

	if (ctx)	{
		free(ctx);
	}

	return NULL;
}

void *p2p_create(void *parent)
{
	int timeout = 10;

	p2p_ctx_t *ctx = malloc(sizeof(p2p_ctx_t));
	if (!ctx) {
		return NULL;
	}
	memset(ctx, 0, sizeof(p2p_ctx_t));
	ctx->parent = parent;

	return ctx;
p2p_create_fail:
	p2p_destroy((void *)ctx);
	return NULL;
}

mm_module_t p2p_module = {
	.create = p2p_create,
	.destroy = p2p_destroy,
	.control = p2p_control,
	.handle = p2p_handle,

	.new_item = NULL,
	.del_item = NULL,

	.output_type = MM_TYPE_NONE,     // no output
	.module_type = MM_TYPE_VSINK,    // module type is video algorithm
	.name = "P2P"
};
