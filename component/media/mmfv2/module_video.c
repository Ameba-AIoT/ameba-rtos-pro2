/******************************************************************************
*
* Copyright(c) 2021 - 2025 Realtek Corporation. All rights reserved.
*
******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <osdep_service.h>
#include "mmf2.h"
#include "mmf2_dbg.h"

#include "module_video.h"
#include "module_rtsp2.h"


#include <math.h>
#include "platform_stdlib.h"

#include <unistd.h>
#include <sys/wait.h>

#include "base_type.h"

#include "cmsis.h"
#include "error.h"


#include "hal.h"
#include "hal_video.h"
#include "isp_ctrl_api.h"

#include "ftl_common_api.h"

#define OSD_ENABLE 1
#define HDR_ENABLE 0

int framecnt = 0;
int jpegcnt = 0;
int ch1framecnt = 0;
int ch2framecnt = 0;
int rgb_lock = 0;

#define MMF_VIDEO_DEBUG 0

///////////////////
#include "fw_img_export.h"
#include "sensor.h"
#include "video_boot.h"
int sensor_id_value = 0;
int isp_get_id(void);
int isp_set_sensor(int sensor_id);
void video_save_sensor_id(int SensorName);
extern video_boot_stream_t *isp_boot;
static int(*sensor_setup)(int status, int sensor_id) = NULL;
//////////////////
static int flash_sensor_id = -1;
static isp_info_t info;

#define CH_NUM 5
static int show_fps = 0;
static int ch_fps_cnt[CH_NUM]   = {0};
static int cb_tick[CH_NUM]   = {0};
static int ch_fps[CH_NUM]   = {0};
static int ch_forcei[CH_NUM]   = {0};
static int video_detect_sensor_id(void);
void video_show_fps(int enable)
{
	show_fps = enable;
}
int video_get_cb_fps(int chn)
{
	if (chn < 0 || chn > 4) {
		printf("[%s] %d is invalid, chn range is 0~4\r\n", __FUNCTION__, chn);
		chn = 0;
	}
	return ch_fps[chn];
}

isp_statis_meta_t _meta;
void video_default_meta_cb(void *parm)
{
	video_meta_t *m_parm = (video_meta_t *)parm;
	m_parm->user_buf = NULL;
	video_sei_write(m_parm);
}

void video_meta_data_process(video_ctx_t *ctx, enc2out_t *enc2out, uint32_t codec_type)
{
	uint32_t meta_offset = 0;
	uint32_t meta_size = 0;
	if (codec_type == AV_CODEC_ID_MJPEG) {
		ctx->meta_data.video_addr = (uint32_t)enc2out->jpg_addr;
		ctx->meta_data.video_len = enc2out->jpg_len;
		meta_offset = enc2out->jpg_meta_offset;
		meta_size = enc2out->jpg_meta_size;
	} else if ((codec_type == AV_CODEC_ID_H264) || (codec_type == AV_CODEC_ID_H265)) {
		ctx->meta_data.video_addr = (uint32_t)enc2out->enc_addr;
		ctx->meta_data.video_len = enc2out->enc_len;
		meta_offset = enc2out->enc_meta_offset;
		meta_size = enc2out->enc_meta_size;
	} else {
		VIDEO_DBG_ERROR("meta data dont support type %d\r\n", codec_type);
		return;
	}
	ctx->meta_data.type = codec_type;
	ctx->meta_data.meta_offset = meta_offset;
	ctx->meta_data.meta_size = meta_size;
	ctx->meta_data.isp_meta_data = &(enc2out->isp_meta_data);
	ctx->meta_data.isp_statis_meta = &(enc2out->statis_data);
	if (ctx->meta_cb) {
		ctx->meta_cb(&(ctx->meta_data));
	}

	//save ae, awb info for pre init
	video_pre_init_save_cur_params(1, &(ctx->meta_data), 0);
}

void video_ch4_delay_release_task(void *param)
{
	while (video_get_stream_info(4)) {
		vTaskDelay(33);
		if (video_ispbuf_release(4, (int)param) == OK) {
			break;
		}
	}
	vTaskDelete(NULL);
}

void video_ch4_delay_release(int ch4_release_fail_add)
{
	if (xTaskCreate(video_ch4_delay_release_task, ((const char *)"ch4_release"), 256, (void *)ch4_release_fail_add, tskIDLE_PRIORITY + 4, NULL) != pdPASS) {
		printf("\r\n video_ch4_delay_release: Create Task Error\n");
	}
}

static void video_bps_cal(int ch, video_bps_stats_t *bps_stats, uint32_t frame_size)
{
	rate_ctrl_s cur_rc;
	video_ctrl(ch, VIDEO_GET_RC_CTRL, (int)&cur_rc);
	//calculate bitrate
	bps_stats->cnt_br++;
	bps_stats->sum_br += frame_size;
	if (bps_stats->cnt_br >= cur_rc.fps) {
		bps_stats->cur_bps = bps_stats->sum_br * 8;
		bps_stats->sum_br = 0;
		bps_stats->cnt_br = 0;
	}
}

void video_frame_complete_cb(void *param1, void  *param2, uint32_t arg)
{
	enc2out_t *enc2out = (enc2out_t *)param1;
	hal_video_adapter_t  *v_adp = (hal_video_adapter_t *)param2;
	commandLine_s *cml = (commandLine_s *)&v_adp->cmd[enc2out->ch];
	video_ctx_t *ctx = (video_ctx_t *)arg;
	mm_context_t *mctx = (mm_context_t *)ctx->parent;
	mm_queue_item_t *output_item;

	//uint32_t timestamp = video_get_system_ts_from_isp_ts(xTaskGetTickCount(), enc2out->time_stamp) + ctx->timestamp_offset;
	uint32_t timestamp = video_get_system_ts_from_isp_ts(mm_read_mediatime_ms(), enc2out->time_stamp, enc2out->ch) + ctx->timestamp_offset;

	int is_output_ready = 0;

#if MMF_VIDEO_DEBUG
	if (enc2out->codec & CODEC_JPEG) {
		VIDEO_DBG_INFO("jpeg in = 0x%X\r\n", enc2out->jpg_addr);
	} else if (enc2out->codec & CODEC_H264 || enc2out->codec & CODEC_HEVC) {
		VIDEO_DBG_INFO("hevc/h264 in = 0x%X\r\n", enc2out->enc_addr);
	} else {
		VIDEO_DBG_INFO("nv12/nv16/rgb in = 0x%X\r\n", enc2out->isp_addr);
	}
#endif

	// VOE status check
	if (enc2out->cmd_status == VOE_OK) {
		// Normal frame output
		//printf("<<test>><%s><%d> %x\n", __func__, __LINE__, enc2out->cmd);
		//force I filter, when force i, it will wait until get i frame
		if (ch_forcei[enc2out->ch] == 1) {
			if (enc2out->codec & (CODEC_H264 | CODEC_HEVC)) {
				if (enc2out->codec == CODEC_H264 || enc2out->codec == CODEC_HEVC || enc2out->codec == (CODEC_H264 | CODEC_JPEG) ||
					enc2out->codec == (CODEC_HEVC | CODEC_JPEG)) {
					uint8_t *ptr = (uint8_t *)enc2out->enc_addr;
					if (ptr[0] != 0 || ptr[1] != 0) {
						VIDEO_DBG_ERROR("\r\nH264 stream error\r\n");
						VIDEO_DBG_ERROR("\r\n(%d/%d) %x %x %x %x\r\n", enc2out->enc_len, enc2out->finish, *ptr, *(ptr + 1), *(ptr + 2), *(ptr + 3));
						video_encbuf_release(enc2out->ch, enc2out->codec, enc2out->enc_len);
						return;
					}
					if (enc2out->codec & (CODEC_H264)) {
						int type = ptr[4] & 0x1F;
						if (ptr[0] == 0 && ptr[1] == 0 && ptr[2] == 0 && ptr[3] == 1 && type == 0x07) {
							ch_forcei[enc2out->ch] = 0;
						} else {
							//printf("release h264\r\n");
							video_encbuf_release(enc2out->ch, enc2out->codec, enc2out->enc_len);
							return;
						}
					}

					if (enc2out->codec & (CODEC_HEVC)) {
						int type = ptr[4];
						if (ptr[0] == 0 && ptr[1] == 0 && ptr[2] == 0 && ptr[3] == 1 && type == 0x40) {
							ch_forcei[enc2out->ch] = 0;
						} else {
							//printf("release h265\r\n");
							video_encbuf_release(enc2out->ch, enc2out->codec, enc2out->enc_len);
							return;
						}
					}
				}
			}
		}
		if(ctx->dbg_ts_info) {
			if(ctx->dbg_ts_info->timestamp_cnt < MMF_VIDEO_DBG_TS_MAX_CNT) {
				ctx->dbg_ts_info->timestamp[ctx->dbg_ts_info->timestamp_cnt] = timestamp;
				ctx->dbg_ts_info->timestamp_cnt++;
			}
		}
		ctx->frame_cnt++;
	} else {
		// Video error handle

		switch (enc2out->cmd_status) {
		case VOE_ENC_BUF_OVERFLOW:
		case VOE_ENC_QUEUE_OVERFLOW:
			VIDEO_DBG_WARNING("VOE CH%d ENC %s full (queue/used/out/rsvd) %d/%dKB%dKB%dKB\n"
							  , enc2out->ch
							  , enc2out->cmd_status == VOE_ENC_BUF_OVERFLOW ? "buff" : "queue"
							  , enc2out->enc_time
							  , enc2out->enc_used >> 10
							  , ctx->params.out_buf_size >> 10
							  , ctx->params.out_rsvd_size >> 10);
			video_encbuf_clean(enc2out->ch, CODEC_H264 | CODEC_HEVC);
			video_ctrl(enc2out->ch, VIDEO_FORCE_IFRAME, 1);
			break;
		case VOE_JPG_BUF_OVERFLOW:
		case VOE_JPG_QUEUE_OVERFLOW:
			VIDEO_DBG_WARNING("VOE CH%d JPG %s full (queue/used/out/rsvd) %d/%dKB\n"
							  , enc2out->ch
							  , enc2out->cmd_status == VOE_JPG_BUF_OVERFLOW ? "buff" : "queue"
							  , enc2out->jpg_time
							  , enc2out->jpg_used >> 10);
			//video_encbuf_clean(enc2out->ch, CODEC_JPEG);
			break;
		default:
			VIDEO_DBG_ERROR("Error CH%d VOE cmd %x status %x\n", enc2out->ch, enc2out->cmd, enc2out->cmd_status);
			break;
		}
		return;
	}

	if(ctx->frame_drop_interval) {
		if(ctx->frame_cnt % ctx->frame_drop_interval != 1) {
			goto direct_output;
		}
	}

	if (ctx->params.direct_output == 1) {
		goto direct_output;
	}

	// Snapshot JPEG
	if (enc2out->codec & CODEC_JPEG && enc2out->jpg_len > 0) { // JPEG
		if (ctx->snapshot_cb != NULL) {
			if (ctx->params.meta_enable == 1 && video_pre_init_get_meta_enable()) {
				video_meta_data_process(ctx, enc2out, AV_CODEC_ID_MJPEG);
			}
			ctx->snapshot_cb((uint32_t)enc2out->jpg_addr, enc2out->jpg_len);
			video_encbuf_release(enc2out->ch, CODEC_JPEG, enc2out->jpg_len);
		} else {
			char *tempaddr = NULL;
			if (ctx->params.use_static_addr == 0) {
				tempaddr = (char *)malloc(enc2out->jpg_len);
				if (tempaddr == NULL) {
					video_encbuf_release(enc2out->ch, CODEC_JPEG, enc2out->jpg_len);
					VIDEO_DBG_ERROR("malloc fail = %d\r\n", enc2out->jpg_len);
					return;
				}
			}

			is_output_ready = xQueueReceive(mctx->output_recycle, (void *)&output_item, 0);
			if (is_output_ready) {
				if (ctx->params.use_static_addr) {
					output_item->data_addr = (uint32_t)enc2out->jpg_addr;
				} else {
					output_item->data_addr = (uint32_t)tempaddr;//malloc(enc2out->jpg_len);
					memcpy((void *)output_item->data_addr, (char *)enc2out->jpg_addr, enc2out->jpg_len);
					video_encbuf_release(enc2out->ch, CODEC_JPEG, enc2out->jpg_len);
				}
				output_item->size = enc2out->jpg_len;
				output_item->timestamp = timestamp;
				output_item->hw_timestamp = enc2out->time_stamp;
				output_item->type = AV_CODEC_ID_MJPEG;
				output_item->priv_data = enc2out->jpg_slot;//JPEG buffer used slot

				if (ctx->params.meta_enable == 1 && video_pre_init_get_meta_enable()) {
					video_meta_data_process(ctx, enc2out, AV_CODEC_ID_MJPEG);
				}

				if (xQueueSend(mctx->output_ready, (void *)&output_item, 0) != pdTRUE) {
					video_encbuf_release(enc2out->ch, CODEC_JPEG, enc2out->jpg_len);
				}

			} else {
				VIDEO_DBG_INFO("\r\n CH %d MMF JPEG Queue full \r\n", enc2out->ch);
				if (ctx->params.use_static_addr == 0) {
					free(tempaddr);
				} else {
					video_encbuf_release(enc2out->ch, CODEC_JPEG, enc2out->jpg_len);
				}
			}
		}
		enc2out->codec = enc2out->codec & (~CODEC_JPEG);
	}

	if (/*enc2out->enc_len > 0 && */(enc2out->codec & CODEC_H264 || enc2out->codec & CODEC_HEVC ||
									 enc2out->codec & CODEC_RGB || enc2out->codec & CODEC_NV12 ||
									 enc2out->codec & CODEC_NV16)) {
		char *tempaddr = NULL;

		is_output_ready = xQueueReceive(mctx->output_recycle, (void *)&output_item, 0);
		if (is_output_ready) {
			if (enc2out->codec == CODEC_H264) {
				output_item->type = AV_CODEC_ID_H264;
				output_item->size = enc2out->enc_len;
				video_bps_cal(enc2out->ch, &(ctx->bps_stats), output_item->size);
			} else if (enc2out->codec == CODEC_HEVC) {
				output_item->type = AV_CODEC_ID_H265;
				output_item->size = enc2out->enc_len;
				video_bps_cal(enc2out->ch, &(ctx->bps_stats), output_item->size);
			} else if (enc2out->codec == CODEC_RGB) {
				output_item->type = AV_CODEC_ID_RGB888;
				output_item->size = enc2out->width * enc2out->height * 3;
			} else if (enc2out->codec == CODEC_NV12) {
				output_item->type = AV_CODEC_ID_NV12;
				output_item->size = enc2out->width * enc2out->height * 3 / 2;
			} else if (enc2out->codec == CODEC_NV16) {
				output_item->type = AV_CODEC_ID_NV16;
				output_item->size = enc2out->width * enc2out->height * 2;
			}

			if (ctx->params.use_static_addr == 0) {
				tempaddr = (char *)malloc(output_item->size);
				if (tempaddr == NULL) {
					VIDEO_DBG_ERROR("malloc fail = %d\r\n", output_item->size);
					if ((enc2out->codec & (CODEC_NV12 | CODEC_RGB | CODEC_NV16)) != 0) {
						video_ispbuf_release(enc2out->ch, (int)enc2out->isp_addr);
					} else {
						video_encbuf_release(enc2out->ch, enc2out->codec, output_item->size);
					}
					return;
				}
			}

			if (enc2out->codec & (CODEC_H264 | CODEC_HEVC)) {
				uint8_t *ptr = (uint8_t *)enc2out->enc_addr;
				if (ptr[0] != 0 || ptr[1] != 0) {
					VIDEO_DBG_ERROR("\r\nH264 stream error\r\n");
					VIDEO_DBG_ERROR("\r\n(%d/%d) %x %x %x %x\r\n", enc2out->enc_len, enc2out->finish, *ptr, *(ptr + 1), *(ptr + 2), *(ptr + 3));
				}
				if ((enc2out->codec & (CODEC_H264)) && (ctx->params.sps_pps_info.enable) && (ctx->params.sps_pps_info.status == 0)) {
					int type = ptr[4] & 0x1F;
					if (ptr[0] == 0 && ptr[1] == 0 && ptr[2] == 0 && ptr[3] == 1 && type == 0x07) {
						int ret = 0;
						ret = video_get_sps_pps(ptr + 4, enc2out->enc_len, enc2out->ch, &ctx->params.sps_pps_info);
						if (ret >= 0) {
							ctx->params.sps_pps_info.status = 1;
							if (ctx->sps_pps_cb) {
								ctx->sps_pps_cb(NULL);
							}
						}
					}
				}

				if ((enc2out->codec & (CODEC_HEVC)) && (ctx->params.sps_pps_info.enable) && (ctx->params.sps_pps_info.status == 0)) {
					int type = ptr[4];
					if (ptr[0] == 0 && ptr[1] == 0 && ptr[2] == 0 && ptr[3] == 1 && type == 0x40) {
						int ret = 0;
						ret = video_get_sps_pps_vps(ptr + 4, enc2out->enc_len, enc2out->ch, &ctx->params.sps_pps_info);
						if (ret >= 0) {
							ctx->params.sps_pps_info.status = 1;
							if (ctx->sps_pps_cb) {
								ctx->sps_pps_cb(NULL);
							}
						}
					}
				}
				if (ctx->params.meta_enable == 1 && video_pre_init_get_meta_enable()) {
					video_meta_data_process(ctx, enc2out, output_item->type);
				}
				/* } */
				if (ctx->params.use_static_addr) {
					output_item->data_addr = (uint32_t)enc2out->enc_addr;
				} else {
					output_item->data_addr = (uint32_t)tempaddr;//malloc(enc2out->enc_len);
					memcpy((void *)output_item->data_addr, (char *)enc2out->enc_addr, output_item->size);
					if (ctx->params.use_static_addr == 0) {
						video_encbuf_release(enc2out->ch, enc2out->codec, output_item->size);
					}
				}

			} else {
				if (ctx->params.use_static_addr) {
					output_item->data_addr = (uint32_t)enc2out->isp_addr;
				} else {
					output_item->data_addr = (uint32_t)tempaddr;//malloc(enc2out->enc_len);
					memcpy((void *)output_item->data_addr, (char *)enc2out->isp_addr, output_item->size);
					video_ispbuf_release(enc2out->ch, (int)enc2out->isp_addr);
				}
			}

			output_item->timestamp = timestamp; //rtp timestamp
			output_item->hw_timestamp = enc2out->time_stamp;
			output_item->priv_data = enc2out->enc_slot;//ENC buffer used slot

			if (show_fps) {
				if (mm_read_mediatime_ms() - cb_tick[enc2out->ch] > 1000) {
					cb_tick[enc2out->ch] = mm_read_mediatime_ms();
					printf("[CH:%d] fps:%d.\r\n", enc2out->ch, ch_fps_cnt[enc2out->ch] - 1);
					ch_fps[enc2out->ch] = ch_fps_cnt[enc2out->ch] - 1;
					ch_fps_cnt[enc2out->ch] = 0;
				}
				ch_fps_cnt[enc2out->ch]++;
			}
			if (voe_boot_fsc_status()) {
				static int queue_len = 0;
				static int queue_timestamp = 0;
				static int queue_initial = 0;
				video_boot_stream_t *isp_fcs_info;
				video_get_fcs_info(&isp_fcs_info);
				//printf("===================voe_boot_fsc_status %d========================\r\n", queue_len);
				if (queue_initial == 0x00) {
					if (isp_fcs_info->video_params[enc2out->ch].fcs) {
						queue_len = output_item->priv_data;
						queue_timestamp = timestamp;
						VIDEO_DBG_INFO("output_item->priv_data %d %d\r\n", output_item->priv_data, queue_timestamp);
						queue_initial = 1;
						video_set_fcs_queue_info(queue_timestamp, queue_timestamp);
					}
				}
				if (queue_len > 0) {
					//output_item->timestamp = queue_timestamp - (queue_len - 1) * (1000 / isp_fcs_info->video_params[enc2out->ch].fps);
					//VIDEO_DBG_INFO("queue_len %d %d\r\n", queue_len, output_item->timestamp);
					VIDEO_DBG_INFO("queue_len %d %d\r\n", queue_len, output_item->timestamp);
					queue_len = queue_len - 1;
				}
			}
			//printf("Video TS => KM: %d en %d, TM: %d, diff: %d\r\n", enc2out->time_stamp, enc2out->enc_time, output_item->timestamp, enc2out->time_stamp - output_item->timestamp);
			if (xQueueSend(mctx->output_ready, (void *)&output_item, 0) != pdTRUE) {
				if (enc2out->codec <= CODEC_JPEG) {
					video_encbuf_release(enc2out->ch, enc2out->codec, output_item->size);
				} else {
					video_ispbuf_release(enc2out->ch, (int)enc2out->isp_addr);
				}
			}

		} else {
			VIDEO_DBG_WARNING("\r\n CH %d MMF ENC Queue full \r\n", enc2out->ch);

			if (enc2out->codec <= CODEC_JPEG) {
				video_encbuf_release(enc2out->ch, enc2out->codec, enc2out->enc_len);
			} else {
				int ret = video_ispbuf_release(enc2out->ch, (int)enc2out->isp_addr);
				if (ret != OK && enc2out->ch == 4) {
					video_ch4_delay_release((int)enc2out->isp_addr);
				}
			}
		}
	}

	return;

direct_output:

	if (enc2out->codec & (CODEC_H264 | CODEC_HEVC)) {
		VIDEO_DBG_INFO("(%s-%s)(0x%X -- %d)(ch%d)(wh=%d x %d) \n"
						, (enc2out->codec & CODEC_H264) != 0 ? "H264" : "HEVC"
						, (enc2out->type == VCENC_INTRA_FRAME) ? "I" : "P"
						, enc2out->enc_addr, enc2out->enc_len, enc2out->ch, enc2out->width, enc2out->height);
	}


	if ((enc2out->codec & (CODEC_H264 | CODEC_HEVC)) != 0) {
		video_encbuf_release(enc2out->ch, enc2out->codec, enc2out->enc_len);
	} else if ((enc2out->codec & (CODEC_NV12 | CODEC_RGB | CODEC_NV16)) != 0) {
		video_ispbuf_release(enc2out->ch, (int)enc2out->isp_addr);
	} else if ((enc2out->codec & CODEC_JPEG) != 0) {
		video_encbuf_release(enc2out->ch, enc2out->codec, enc2out->jpg_len);
	}

	//close output task
	if (enc2out->finish == LAST_FRAME) {

	}
}

int video_control(void *p, int cmd, int arg)
{
	video_ctx_t *ctx = (video_ctx_t *)p;
	mm_context_t *mctx = (mm_context_t *)ctx->parent;
	mm_queue_item_t *tmp_item;
	int ch = ctx->params.stream_id;
	int ret = 0;
	switch (cmd) {
	case CMD_VIDEO_SET_PARAMS:
		memcpy(&ctx->params, (void *)arg, sizeof(video_params_t));
		break;
	case CMD_VIDEO_GET_PARAMS:
		memcpy((void *)arg, &ctx->params, sizeof(video_params_t));
		break;
	case CMD_VIDEO_SET_RCPARAM: {
		ret = video_ctrl(ch, VIDEO_SET_RCPARAM, arg);
	}
	break;
	case CMD_VIDEO_STREAMID:
		ctx->params.stream_id = arg;
		break;
	case CMD_VIDEO_STREAM_STOP: {
		if (video_get_stream_info(ch) == 0) {
			VIDEO_DBG_WARNING("CH %d already close\r\n", ch);
			return NOK;
		}
		ctx->frame_cnt = 0;
		memset(&(ctx->meta_data), 0, sizeof(video_meta_t));
		//video_close will release all voe buffer
		ret = video_close(ch);
		mm_queue_item_t *queue_item;
		if(mctx->output_ready) {
			while(uxQueueMessagesWaiting(mctx->output_ready)) {
				if(xQueueReceive(mctx->output_ready, (void *)&queue_item, 0) == pdTRUE) {
					if (ctx->params.use_static_addr == 0) {
						free((void*)queue_item->data_addr);
					}
					queue_item->data_addr = 0;
					xQueueSend(mctx->output_recycle, (void *)&queue_item, 0);
				}
			}
		}

		//video deinit after all video close, takes 50ms
		if (video_open_status() == 0) {
			video_deinit();
		}

		return ret;
	}
	break;
	case CMD_VIDEO_FORCE_IFRAME: {
		ret = video_ctrl(ch, VIDEO_FORCE_IFRAME, arg);
		ch_forcei[ch] = 1;
	}
	break;
	case CMD_VIDEO_BPS: {
		ret = video_ctrl(ch, VIDEO_BPS, arg);
	}
	break;
	case CMD_VIDEO_GOP: {
		ret = video_ctrl(ch, VIDEO_GOP, arg);
	}
	break;
	case CMD_VIDEO_FPS: {
		ret = video_ctrl(ch, VIDEO_FPS, arg);
	}
	break;
	case CMD_VIDEO_ISPFPS: {
		ret = video_ctrl(ch, VIDEO_ISPFPS, arg);
	}
	break;
	case CMD_VIDEO_SNAPSHOT: {
		ret = video_ctrl(ch, VIDEO_JPEG_OUTPUT, arg);
	}
	break;
	case CMD_VIDEO_YUV: {
		int type = ctx->params.type;
		switch (type) {
		case 0:
			VIDEO_DBG_ERROR("wrong type %d\r\n", type);
			break;
		case 1:
			VIDEO_DBG_ERROR("wrong type %d\r\n", type);
			break;
		case 2:
			VIDEO_DBG_ERROR("wrong type %d\r\n", type);
			break;
		case 3:
			ret = video_ctrl(ch, VIDEO_NV12_OUTPUT, arg);
			break;
		case 4:
			ret = video_ctrl(ch, VIDEO_RGB_OUTPUT, arg);
			break;
		case 5:
			ret = video_ctrl(ch, VIDEO_NV16_OUTPUT, arg);
			break;
		case 6:
			VIDEO_DBG_ERROR("wrong type %d\r\n", type);
			break;
		case 7:
			VIDEO_DBG_ERROR("wrong type %d\r\n", type);
			break;
		}

	}
	break;
	case CMD_ISP_SET_RAWFMT: {
		ret = video_ctrl(ch, VIDEO_ISP_SET_RAWFMT, arg);
	}
	break;
	case CMD_VIDEO_SNAPSHOT_CB:
		ctx->snapshot_cb = (int (*)(uint32_t, uint32_t))arg;
		break;
	case CMD_VIDEO_META_CB:
		if (arg == MMF_VIDEO_DEFAULT_META_CB) {
			ctx->meta_cb = video_default_meta_cb;
		} else {
			ctx->meta_cb = (void (*)(void *))arg;
		}
		break;
	case CMD_VIDEO_GET_META_DATA:
		if(ctx->meta_data.type == 0) {
			VIDEO_DBG_ERROR("ch%d meta data not available\r\n", ctx->params.stream_id);
			return -1;
		}
		memcpy((void *)arg, &(ctx->meta_data), sizeof(video_meta_t));
		break;
	case CMD_VIDEO_UPDATE:

		break;
	case CMD_VIDEO_SET_VOE_HEAP:

		break;
	case CMD_VIDEO_PRINT_INFO: {
		ret = video_ctrl(ch, VIDEO_PRINT_INFO, arg);
	}
	break;
	case CMD_VIDEO_SET_SPS_PPS_INFO: {
		ctx->params.sps_pps_info.status = 0;//clean the before status
		ctx->params.sps_pps_info.enable = 1;//Enable to get the sps pps info
	}
	break;
	case CMD_VIDEO_GET_SPS_PPS_INFO: {
		if (ctx->params.sps_pps_info.status) {
			memcpy((void *)arg, &ctx->params, sizeof(video_params_t));
		}
		return ctx->params.sps_pps_info.status;;
	}
	break;
	case CMD_VIDEO_APPLY: {
		int ch = arg;
		ctx->params.stream_id = ch;
		//video init before first vido open, take 78ms.
		if (video_open_status() == 0) {
			ctx->v_adp = video_init(ctx->iq_addr, ctx->sensor_addr);
			VIDEO_DBG_INFO("ctx->v_adp = 0x%X\r\n", ctx->v_adp);
			if (ctx->v_adp == NULL) {
				VIDEO_DBG_ERROR("video_init fail\r\n");
				return NOK;
			}
		}

		ret = video_open(&ctx->params, video_frame_complete_cb, ctx);

#if MULTI_SENSOR
		if (ret < 0 && video_get_video_sensor_status() == 0) { //Change the sensor procedure
#if NONE_FCS_MODE
			for (int id = 0; id < isp_boot->p_fcs_ld_info.multi_fcs_cnt; id++) {
#else
			for (int id = 1; id < isp_boot->p_fcs_ld_info.multi_fcs_cnt; id++) {
#endif
				if (sensor_setup) {
					ret = sensor_setup(SENSOR_MULTI_SETUP_PROCEDURE, id);
					if (ret < 0) { //Skip the sensor
						continue;
					}
				}

				if (1) {
					video_reset_fw(ch, id);
					ret = video_open(&ctx->params, video_frame_complete_cb, ctx);
					if (ret >= 0) {
						sensor_id_value = id;
						video_save_sensor_id(id);
						flash_sensor_id = id;
						VIDEO_DBG_INFO("It find the correct sesnor %d\r\n", id);
						break;
					}
				}

			}
		} else {
			if (video_get_video_sensor_status() == 0) {
				video_save_sensor_id(sensor_id_value);
			}
		}
#else
		/* 		if ((voe_boot_fsc_status() == 0) && (voe_boot_fsc_id() > 0) && (video_get_video_sensor_status() == 0)) {
					//The fcs mode fail that it need to do the reset.
					VIDEO_DBG_INFO("Reset fcs mode to common mode\r\n");
					video_reset_fw(ch, sensor_id_value);
					ret = video_open(&ctx->params, video_frame_complete_cb, ctx);
				} */
		if (ret < 0 && video_open_status() == 0) { //if video open fail deinit video
			video_deinit();
		}
		if (ret < 0 && video_get_video_sensor_status() == 0) { //Change the sensor procedure
			VIDEO_DBG_ERROR("Please check sensor id first,the id is %d\r\n", sensor_id_value);
			return ret;
		} else {
			if (video_get_video_sensor_status() == 0) {
				video_save_sensor_id(sensor_id_value);
			}
		}
#endif
	}
	break;
	case CMD_VIDEO_SET_SENSOR_ID: {
		int sensor_id = arg;
		if(sensor_id == 0 || sensor_id >= SENSOR_MAX) {
			VIDEO_DBG_ERROR("invalid sensor id %d\r\n", sensor_id);
			return NOK;
		}
		if (video_open_status() != 0) {
			VIDEO_DBG_ERROR("Close streams before switch sensors.\r\n");
			return NOK;
		}
		voe_get_sensor_info(sensor_id, &ctx->iq_addr, &ctx->sensor_addr);
		sensor_id_value = sensor_id;
		info.sensor_fps    = sensor_params[sen_id[sensor_id_value]].sensor_fps;
		info.sensor_width  = sensor_params[sen_id[sensor_id_value]].sensor_width;
		info.sensor_height = sensor_params[sen_id[sensor_id_value]].sensor_height;
		video_set_isp_info(&info);
	}
	break;
	case CMD_VIDEO_SET_TIMESTAMP_OFFSET: {
		ctx->timestamp_offset = arg;
	}
	break;
	case CMD_VIDEO_EN_DBG_TS_INFO: {
		if(arg) {
			if(ctx->dbg_ts_info == NULL) {
				ctx->dbg_ts_info = malloc(sizeof(dbg_ts_info_t));
				if(ctx->dbg_ts_info == NULL) {
					VIDEO_DBG_ERROR("dbg_ts_info malloc failed\r\n");
					return -1;
				}
			}
			ctx->dbg_ts_info->timestamp_cnt = 0; //init timestamp cnt
		} else {
			if(ctx->dbg_ts_info) {
				free(ctx->dbg_ts_info);
				ctx->dbg_ts_info = NULL;
				return 0;
			}
		}
	}
	break;
	case CMD_VIDEO_SHOW_DBG_TS_INFO: {
		if(ctx->dbg_ts_info) {
			printf("ch%d timestamp = ", ch);
			for(int i = 0; i < ctx->dbg_ts_info->timestamp_cnt; i++) {
				printf("%u ", ctx->dbg_ts_info->timestamp[i]);
			}
			printf("\r\n");
		} else {
			VIDEO_DBG_ERROR("dbg_ts_info disable\r\n");
			return -1;
		}
	}
	break;
	case CMD_VIDEO_BPS_STBL_CTRL_EN: {
		ret = video_bps_stbl_ctrl_en(ch, arg);
	}
	break;
	case CMD_VIDEO_SET_BPS_STBL_CTRL_PARAMS: {
		ret = video_set_bps_stbl_ctrl_params(ch, (bps_stbl_ctrl_param_t*)arg, NULL, NULL);
	}
	break;
	case CMD_VIDEO_SET_BPS_STBL_CTRL_FPS_STG: {
		ret = video_set_bps_stbl_ctrl_params(ch, NULL, (uint32_t*)arg, NULL);
	}
	break;
	case CMD_VIDEO_SET_BPS_STBL_CTRL_GOP_STG: {
		ret = video_set_bps_stbl_ctrl_params(ch, NULL, NULL, (uint32_t*)arg);
	}
	break;
	case CMD_VIDEO_GET_CURRENT_BITRATE: {
		*((uint32_t *)arg) = ctx->bps_stats.cur_bps;
	}
	break;
	case CMD_VIDEO_GET_REMAIN_QUEUE_LENGTH: {
		*((uint32_t *)arg) = uxQueueSpacesAvailable(mctx->output_ready);
	}
	break;
	case CMD_VIDEO_GET_MAX_QP: {
		*((uint32_t *)arg) = video_get_maxqp(ctx->params.stream_id);
	}
	break;
	case CMD_VIDEO_SET_MAX_QP: {
		encode_rc_parm_t rc_parm;
		memset(&rc_parm, 0x0, sizeof(encode_rc_parm_t));
		rc_parm.maxQp = *((uint32_t *)arg);
		ret = video_ctrl(ctx->params.stream_id, VIDEO_SET_RCPARAM, (int)&rc_parm);
	}
	break;
	case CMD_VIDEO_SET_PRIVATE_MASK: {
		struct private_mask_s *pmask = (struct private_mask_s *)arg;
		video_set_private_mask(ctx->params.stream_id, pmask);
	}
	break;
	case CMD_VIDEO_SET_MULTI_RCCTRL: {
		rate_ctrl_s *rc_ctrl = (rate_ctrl_s *)arg;
		ret = video_ctrl(ch, VIDEO_RC_CTRL, arg);
	}
	break;
	case CMD_VIDEO_GET_MULTI_RCCTRL: {
		ret = video_ctrl(ch, VIDEO_GET_RC_CTRL, arg);
	}
	break;
	case CMD_VIDEO_SET_EXT_INPUT: {
		return video_ext_in(ch, (uint32_t)arg);
	}
	break;
	case CMD_VIDEO_SPS_CB: {
		ctx->sps_pps_cb = (void (*)(void *))arg;
		break;
	}
	case CMD_VIDEO_PRE_INIT_PARM: {
		video_pre_init_setup_parameters((void *)arg);
		break;
	}
	case CMD_VIDEO_GET_PRE_INIT_PARM: {
		memcpy((void *)arg, (video_pre_init_params_t*)video_get_pre_init_setup_params(), sizeof(video_pre_init_params_t));
		break;
	}
	case CMD_VIDEO_PRE_INIT_LOAD: {
		video_pre_init_load_params(arg);
		break;
	}
	case CMD_VIDEO_PRE_INIT_SAVE: {
		//save_to_flash 0: only save to pre init structure, 1: save to flash, 2: save to sram retention
		video_pre_init_save_cur_params((ctx->params.meta_enable && video_pre_init_get_meta_enable()), &ctx->meta_data, arg);
		break;
	}
	case CMD_VIDEO_SET_CAP_INTVL: {
		ctx->frame_drop_interval = ctx->params.fps * arg;
		break;
	}
}
return ret;
}

int video_handle(void *ctx, void *input, void *output)
{
	return 0;
}

void *video_destroy(void *p)
{
	video_ctx_t *ctx = (video_ctx_t *)p;
	if(ctx->dbg_ts_info) {
		free(ctx->dbg_ts_info);
		ctx->dbg_ts_info = NULL;
	}

	free(ctx);
	return NULL;
}

static int video_detect_sensor_id(void)
{
	int id_value = 0;
	if (MANUAL_SENSOR_IQ > 0 && MANUAL_SENSOR_IQ < SENSOR_MAX) {
		id_value = MANUAL_SENSOR_IQ;
	} else {
		for (int i = 0; i < SENSOR_MAX; i++) {
			if (sen_id[i] == USE_SENSOR) {
				id_value = i;
				break;
			}
		}
	}
	return id_value;
}

void *video_create(void *parent)
{
	video_ctx_t *ctx = malloc(sizeof(video_ctx_t));
	int ret = 0;
	if (!ctx) {
		return NULL;
	}
	memset(ctx, 0, sizeof(video_ctx_t));

	ctx->parent = parent;
	ctx->dbg_ts_info = NULL;


	if (voe_boot_fsc_status()) {
		sensor_id_value = voe_boot_fsc_id();
		voe_get_sensor_info(sensor_id_value, &ctx->iq_addr, &ctx->sensor_addr);
	} else {
#if MULTI_SENSOR
		int sensor_id = isp_get_id();
		flash_sensor_id = sensor_id;
		int sensor_status = 0;
		VIDEO_DBG_INFO("sensor_id %d\r\n", sensor_id);
		if (sensor_id != 0xff && sensor_id < isp_boot->p_fcs_ld_info.multi_fcs_cnt && sensor_id > 0) {
			sensor_id_value = sensor_id;
			sensor_status = SENSOR_MULTI_SAVE_VALUE;
		} else {
			if (sensor_id_value == 0) {
				sensor_id_value = video_detect_sensor_id();
			}
			//sensor_id_value = USE_SENSOR;
			sensor_status = SENSOR_MULTI_DEFAULT_SETUP;
		}
		if (sensor_setup) {
			ret = sensor_setup(sensor_status, sensor_id_value);
			if (ret >= 0) {
				sensor_id_value = ret;
			}
		}
		voe_get_sensor_info(sensor_id_value, &ctx->iq_addr, &ctx->sensor_addr);
#else
		if (!sensor_id_value) { //Use the default sensor, if the value equal to 0
			if (sensor_id_value == 0) {
				sensor_id_value = video_detect_sensor_id();
			}
			//sensor_id_value = USE_SENSOR;
		}
		if (sensor_setup) {
			ret = sensor_setup(SENSOR_SINGLE_DEFAULT_SETUP, sensor_id_value);
			if (ret >= 0) {
				sensor_id_value = ret;
			}
		}
		voe_get_sensor_info(sensor_id_value, &ctx->iq_addr, &ctx->sensor_addr);
#endif
	}
	VIDEO_DBG_INFO("ID %x iq_addr %x sensor_addr %x\r\n", sensor_id_value, ctx->iq_addr, ctx->sensor_addr);

	return ctx;
}

void *video_new_item(void *p)
{
	return NULL;
}

void *video_del_item(void *p, void *d)
{

	video_ctx_t *ctx = (video_ctx_t *)p;
	int ch = ctx->params.stream_id;

	if (ctx->params.use_static_addr == 0) {
		if (d) {
			free(d);
		}
	}
	return NULL;
}

void *video_voe_release_item(void *p, void *d, int length)
{
	video_ctx_t *ctx = (video_ctx_t *)p;
	mm_queue_item_t *free_item = (mm_queue_item_t *)d;
	int ch = ctx->params.stream_id;
	int codec = AV_CODEC_ID_UNKNOWN;
	switch (free_item->type) {
	case AV_CODEC_ID_H265:
		codec = CODEC_HEVC;
		break;
	case AV_CODEC_ID_H264:
		codec = CODEC_H264;
		break;
	case AV_CODEC_ID_MJPEG:
		codec = CODEC_JPEG;
		break;
	case AV_CODEC_ID_RGB888:
		codec = CODEC_RGB;
		break;
	}

	if (ctx->params.use_static_addr == 1) {
		if (video_get_stream_info(ch) != 0) {
			if (free_item->type == AV_CODEC_ID_H264 || free_item->type == AV_CODEC_ID_H265 || free_item->type == AV_CODEC_ID_MJPEG) {
				video_encbuf_release(ch, codec, length);
			} else if (free_item->type == AV_CODEC_ID_RGB888) {
				int ret = video_ispbuf_release(ch, free_item->data_addr);
				if (ret != OK) {
					video_ch4_delay_release((int)free_item->data_addr);
				}
				rgb_lock = 0;
			} else {
				video_ispbuf_release(ch, free_item->data_addr);
			}
		}
	}


	return NULL;
}

int video_voe_presetting(int v1_enable, int v1_w, int v1_h, int v1_bps, int v1_shapshot,
						 int v2_enable, int v2_w, int v2_h, int v2_bps, int v2_shapshot,
						 int v3_enable, int v3_w, int v3_h, int v3_bps, int v3_shapshot,
						 int v4_enable, int v4_w, int v4_h)
{
	int voe_heap_size = 0;

#if MULTI_SENSOR
	info.sensor_fps    = 0;
	info.sensor_width  = 0;
	info.sensor_height = 0;
	for (int i = 0; i < SENSOR_MAX; i++) {
		struct sensor_params_t cur_snr = sensor_params[sen_id[i]];
		if (cur_snr.sensor_width * cur_snr.sensor_height > info.sensor_width * info.sensor_height) {
			info.sensor_width  = cur_snr.sensor_width;
			info.sensor_height = cur_snr.sensor_height;
		}
		if (cur_snr.sensor_fps < info.sensor_fps) {
			info.sensor_fps  = cur_snr.sensor_fps;
		}
	}
#else
	if (sensor_id_value > 0 && sensor_id_value < SENSOR_MAX) {
		info.sensor_fps    = sensor_params[sen_id[sensor_id_value]].sensor_fps;
		info.sensor_width  = sensor_params[sen_id[sensor_id_value]].sensor_width;
		info.sensor_height = sensor_params[sen_id[sensor_id_value]].sensor_height;
	} else {
		info.sensor_fps    = sensor_params[USE_SENSOR].sensor_fps;
		info.sensor_width  = sensor_params[USE_SENSOR].sensor_width;
		info.sensor_height = sensor_params[USE_SENSOR].sensor_height;
	}
#endif
	printf("[%s] fps:%d  w:%d  h:%d   \r\n", __FUNCTION__, info.sensor_fps, info.sensor_width, info.sensor_height);

#if OSD_ENABLE
	info.osd_enable = 1;
#endif

#if HDR_ENABLE
	info.hdr_enable = 1;
#endif

	video_set_isp_info(&info);

	//do 16 alignment
	v1_w = (v1_w + 15) & ~15;
	v2_w = (v2_w + 15) & ~15;
	v3_w = (v3_w + 15) & ~15;
	v4_w = (v4_w + 15) & ~15;

	voe_heap_size =  video_buf_calc(v1_enable, v1_w, v1_h, v1_bps, v1_shapshot,
									v2_enable, v2_w, v2_h, v2_bps, v2_shapshot,
									v3_enable, v3_w, v3_h, v3_bps, v3_shapshot,
									v4_enable, v4_w, v4_h);

	return voe_heap_size;
}

int video_voe_presetting_by_params(const void *v1_params, int v1_jpg_only_shapshot, const void *v2_params, int v2_jpg_only_shapshot, const void *v3_params,
								   int v3_jpg_only_shapshot, const void *v4_params)
{
	int voe_heap_size = 0;

#if MULTI_SENSOR
	info.sensor_fps    = 0;
	info.sensor_width  = 0;
	info.sensor_height = 0;
	for (int i = 0; i < SENSOR_MAX; i++) {
		struct sensor_params_t cur_snr = sensor_params[sen_id[i]];
		if (cur_snr.sensor_width * cur_snr.sensor_height > info.sensor_width * info.sensor_height) {
			info.sensor_width  = cur_snr.sensor_width;
			info.sensor_height = cur_snr.sensor_height;
		}
		if (cur_snr.sensor_fps < info.sensor_fps) {
			info.sensor_fps  = cur_snr.sensor_fps;
		}
	}
#else
	if (sensor_id_value > 0 && sensor_id_value < SENSOR_MAX) {
		info.sensor_fps    = sensor_params[sen_id[sensor_id_value]].sensor_fps;
		info.sensor_width  = sensor_params[sen_id[sensor_id_value]].sensor_width;
		info.sensor_height = sensor_params[sen_id[sensor_id_value]].sensor_height;
	} else {
		info.sensor_fps    = sensor_params[USE_SENSOR].sensor_fps;
		info.sensor_width  = sensor_params[USE_SENSOR].sensor_width;
		info.sensor_height = sensor_params[USE_SENSOR].sensor_height;
	}
#endif
	printf("[%s] fps:%d  w:%d  h:%d   \r\n", __FUNCTION__, info.sensor_fps, info.sensor_width, info.sensor_height);

#if OSD_ENABLE
	info.osd_enable = 1;
#endif

#if HDR_ENABLE
	info.hdr_enable = 1;
#endif

	video_set_isp_info(&info);

	int v1_enable = (v1_params == NULL ? 0 : 1);
	int v2_enable = (v2_params == NULL ? 0 : 1);
	int v3_enable = (v3_params == NULL ? 0 : 1);
	int v4_enable = (v4_params == NULL ? 0 : 1);

	video_params_t *v1_p = (v1_enable == 1 ? (video_params_t *)v1_params : NULL);
	video_params_t *v2_p = (v2_enable == 1 ? (video_params_t *)v2_params : NULL);
	video_params_t *v3_p = (v3_enable == 1 ? (video_params_t *)v3_params : NULL);
	video_params_t *v4_p = (v4_enable == 1 ? (video_params_t *)v4_params : NULL);

	//do 16 alignment
	int v1_w = (v1_enable == 1 ? (v1_p->width + 15) & ~15 : 0);
	int v2_w = (v2_enable == 1 ? (v2_p->width + 15) & ~15 : 0);
	int v3_w = (v3_enable == 1 ? (v3_p->width + 15) & ~15 : 0);
	int v4_w = (v4_enable == 1 ? (v4_p->width + 15) & ~15 : 0);

	int v1_h = (v1_enable == 1 ? v1_p->height : 0);
	int v2_h = (v2_enable == 1 ? v2_p->height : 0);
	int v3_h = (v3_enable == 1 ? v3_p->height : 0);
	int v4_h = (v4_enable == 1 ? v4_p->height : 0);

	int v1_bps = (v1_enable == 1 ? v1_p->bps : 0);
	int v2_bps = (v2_enable == 1 ? v2_p->bps : 0);
	int v3_bps = (v3_enable == 1 ? v3_p->bps : 0);

	int v1_type = (v1_enable == 1 ? v1_p->type : 0);
	int v2_type = (v2_enable == 1 ? v2_p->type : 0);
	int v3_type = (v3_enable == 1 ? v3_p->type : 0);

	voe_heap_size =  video_buf_heap_calc(v1_enable, v1_w, v1_h, v1_bps, v1_type, v1_jpg_only_shapshot, v2_enable, v2_w, v2_h, v2_bps, v2_type, v2_jpg_only_shapshot,
										 v3_enable, v3_w, v3_h, v3_bps, v3_type, v3_jpg_only_shapshot, v4_enable, v4_w, v4_h);

	return voe_heap_size;
}

int video_extra_voe_presetting(int originl_heapsize, int vext_enable, int vext_w, int vext_h, int vext_bps, int vext_shapshot)
{
	int voe_heap_size = 0;

	vext_w = (vext_w + 15) & ~15;

	voe_heap_size =  video_extra_buf_calc(originl_heapsize, vext_enable, vext_w, vext_h, vext_bps, vext_shapshot);

	return voe_heap_size;
}

void video_voe_release(void)
{
	video_buf_release();
}

mm_module_t video_module = {
	.create = video_create,
	.destroy = video_destroy,
	.control = video_control,
	.handle = video_handle,

	.new_item = video_new_item,
	.del_item = video_del_item,
	.rsz_item = NULL,
	.vrelease_item = video_voe_release_item,

	.output_type = MM_TYPE_VDSP,    // output for video algorithm
	.module_type = MM_TYPE_VSRC,    // module type is video source
	.name = "VIDEO"
};
//////////////////for multi sensor/////////////////
int isp_get_id(void)   ////It only for non-fcs settung
{
	unsigned char type[4] = {0};

#if NONE_FCS_MODE
	video_get_fw_isp_info();
#endif

	if (voe_boot_fsc_id() == 0) {
		ftl_common_read(ISP_FW_LOCATION, type, sizeof(type));
		if (type[0] == 'I' && type[1] == 'S' && type[2] == 'P') {
			return type[3];
		} else {
			return 0xff;
		}
	} else {
		return 0xff;
	}
}

int isp_set_sensor(int sensor_id)   //It only for non-fcs settung
{
	int value = 0;
	unsigned char status[4] = {0};
	if (voe_boot_fsc_id() == 0) {
		value = isp_get_id();
		status[0] = 'I';
		status[1] = 'S';
		status[2] = 'P';
		status[3] = sensor_id;
		if (value != sensor_id) {
			ftl_common_write(ISP_FW_LOCATION, status, sizeof(status));
			VIDEO_DBG_INFO("Store the sensor id %d %d\r\n", value, sensor_id);
		} else {
			VIDEO_DBG_INFO("The sensor id is the same\r\n");
		}
		return 0;
	} else {
		return -1;
	}
}

void video_save_sensor_id(int SensorName)
{
	video_fcs_write_sensor_id(SensorName);
	isp_set_sensor(SensorName);

	mult_sensor_info_t multi_sensor_info;
	multi_sensor_info.sensor_index = SensorName;
	multi_sensor_info.sensor_finish = 1;
	video_set_video_snesor_info(&multi_sensor_info);
}

void video_set_sensor_id(int SensorName)
{
	if (MANUAL_SENSOR_IQ > 0 && MANUAL_SENSOR_IQ < SENSOR_MAX) {
		sensor_id_value = SensorName;
	} else {
		VIDEO_DBG_INFO("video_set_sensor_id() is only available for manual mode of Sensor/IQ\r\n");
	}
}

void video_setup_sensor(void *sensor_setup_cb)
{
	sensor_setup = (int(*)(int, int))sensor_setup_cb;
}
