#include "hantro_jpeg_encoder.h"
#include "hantro_h264_encoder.h"
#include "hantro_modify_parm.h"
#include "FreeRTOS.h"
#include "task.h"

#include "../framework/mmf_source_modules/mmf_source_h1v6_nv12.h"

extern struct stream_manage_handle *isp_stm;

static struct mmf_video_modify_parm_context *mmf_video_modify_parm_ctx = NULL;

void mmf_video_modify_parm_init(void)
{
	mmf_video_modify_parm_ctx = malloc(sizeof(struct mmf_video_modify_parm_context));
	memset(mmf_video_modify_parm_ctx, 0, sizeof(struct mmf_video_modify_parm_context));
	if (mmf_video_modify_parm_ctx == NULL) {
		printf("mmf_video_modify_parm_init NULL\r\n");
	} else {
		printf("mmf_video_modify_parm_init success\r\n");
	}
}

void mmf_video_modify_parm_deinit(void)
{
	if (mmf_video_modify_parm_ctx == NULL) {
		printf("mmf_video_modify_parm_init NULL\r\n");
	} else {
		free(mmf_video_modify_parm_ctx);
		mmf_video_modify_parm_ctx = NULL;
	}
}

#if ISP_API_VER==1
void mmf_video_isp_reset_buffer(int streamid)
{
	_irqL 	irqL;
	struct enc_buf_data *enc_data = NULL;
	rtw_enter_critical(&isp_stm->isp_stream[streamid]->enc_list.enc_lock, &irqL);
	do {
		enc_data = enc_get_acti_buffer(&isp_stm->isp_stream[streamid]->enc_list);
		if (enc_data == NULL) {
			break;
		}
		enc_set_idle_buffer(&isp_stm->isp_stream[streamid]->enc_list, enc_data);
	} while (1);
	rtw_exit_critical(&isp_stm->isp_stream[streamid]->enc_list.enc_lock, &irqL);
}


void mmf_video_h264_change_parm_cb(void *ctx)
{
	struct hantro_h264_context *h264_ctx = (struct hantro_h264_context *)ctx;
	if (mmf_video_modify_parm_ctx->h264_parm[h264_ctx->isp_info_value.streamid].config_flag) {
		switch (mmf_video_modify_parm_ctx->h264_parm[h264_ctx->isp_info_value.streamid].cmd_code) {
		case CMD_VIDEO_H264_CHG_FPS:
			isp_stop_stream(h264_ctx->isp_info_value.streamid);
			vTaskDelay(10);
			h264_ctx->h264_parm.ratenum = mmf_video_modify_parm_ctx->h264_parm[h264_ctx->isp_info_value.streamid].fps;
			//mmf_video_isp_reset_buffer(h264_ctx);
			mmf_video_isp_reset_buffer(h264_ctx->isp_info_value.streamid);
			hantro_h264enc_release(h264_ctx);
			hantro_h264enc_initial(h264_ctx, &h264_ctx->h264_parm);

			isp_reset_parm(h264_ctx->isp_info_value.streamid, h264_ctx->h264_parm.ratenum, h264_ctx->h264_parm.width, h264_ctx->h264_parm.height,
						   isp_stm->isp_stream[h264_ctx->isp_info_value.streamid]->format);
			printf("set fps\r\n");
			isp_start(h264_ctx->isp_info_value.streamid);
			break;
		case CMD_VIDEO_H264_CHG_RESOLUTION:
			isp_stop_stream(h264_ctx->isp_info_value.streamid);
			vTaskDelay(10);
			h264_ctx->h264_parm.width = mmf_video_modify_parm_ctx->h264_parm[h264_ctx->isp_info_value.streamid].width;
			h264_ctx->h264_parm.height = mmf_video_modify_parm_ctx->h264_parm[h264_ctx->isp_info_value.streamid].height ;
			//mmf_video_isp_reset_buffer(h264_ctx);
			mmf_video_isp_reset_buffer(h264_ctx->isp_info_value.streamid);
			hantro_h264enc_release(h264_ctx);
			hantro_h264enc_initial(h264_ctx, &h264_ctx->h264_parm);
			isp_reset_parm(h264_ctx->isp_info_value.streamid, h264_ctx->h264_parm.ratenum, h264_ctx->h264_parm.width, h264_ctx->h264_parm.height,
						   isp_stm->isp_stream[h264_ctx->isp_info_value.streamid]->format);
			printf("set resolution w= %d h = %d\r\n", h264_ctx->h264_parm.width, h264_ctx->h264_parm.height);
			isp_start(h264_ctx->isp_info_value.streamid);
			break;
		case CMD_VIDEO_H264_CHG_BPS:
			h264_ctx->h264_ch_config.bps = mmf_video_modify_parm_ctx->h264_parm[h264_ctx->isp_info_value.streamid].bps;
			h264_ctx->h264_ch_config.bps_config = 1;
			break;
		case CMD_VIDEO_H264_CHG_IFRAME:
			h264_ctx->index = h264_ctx->index + (h264_ctx->h264_parm.gopLen - (h264_ctx->index % h264_ctx->h264_parm.gopLen));
			printf("h264_ctx->index = %d\r\n", h264_ctx->index);
			break;
		default:
			printf("THe command is not support");
			break;
		}
		mmf_video_modify_parm_ctx->h264_parm[h264_ctx->isp_info_value.streamid].config_flag = 0;
	}
}
#endif

int mmf_video_h264_change_fps(int streamid, int fps)
{
	if (mmf_video_modify_parm_ctx->h264_parm[streamid].config_flag) {
		return -1;
	}
	mmf_video_modify_parm_ctx->h264_parm[streamid].cmd_code = CMD_VIDEO_H264_CHG_FPS;
	mmf_video_modify_parm_ctx->h264_parm[streamid].fps = fps;
	mmf_video_modify_parm_ctx->h264_parm[streamid].config_flag = 1;
	return 0;
}

int mmf_video_h264_change_resolution(int streamid, int width, int height)
{
	if (mmf_video_modify_parm_ctx->h264_parm[streamid].config_flag) {
		return -1;
	}
	mmf_video_modify_parm_ctx->h264_parm[streamid].cmd_code = CMD_VIDEO_H264_CHG_RESOLUTION;
	mmf_video_modify_parm_ctx->h264_parm[streamid].width = width;
	mmf_video_modify_parm_ctx->h264_parm[streamid].height = height;
	mmf_video_modify_parm_ctx->h264_parm[streamid].config_flag = 1;
	return 0;
}

int mmf_video_h264_change_bps(int streamid, int bps)
{
	if (mmf_video_modify_parm_ctx->h264_parm[streamid].config_flag) {
		return -1;
	}
	mmf_video_modify_parm_ctx->h264_parm[streamid].cmd_code = CMD_VIDEO_H264_CHG_BPS;
	mmf_video_modify_parm_ctx->h264_parm[streamid].bps = bps;
	mmf_video_modify_parm_ctx->h264_parm[streamid].config_flag = 1;
	return 0;
}

int mmf_video_h264_force_iframe(int streamid)
{
	if (mmf_video_modify_parm_ctx->h264_parm[streamid].config_flag) {
		return -1;
	}
	mmf_video_modify_parm_ctx->h264_parm[streamid].cmd_code = CMD_VIDEO_H264_CHG_IFRAME;
	mmf_video_modify_parm_ctx->h264_parm[streamid].config_flag = 1;
	return 0;
}

void mmf_video_nv12_change_parm_cb(void *ctx)
{
	struct h1v6_nv12_context *nv12_ctx = (struct h1v6_nv12_context *)ctx;
	int i = 0;
	u32 y_addr, uv_addr = 0;
	isp_stop_stream(nv12_ctx->isp_info_value.streamid);
	vTaskDelay(10);
	mmf_video_isp_reset_buffer(nv12_ctx->isp_info_value.streamid);
	isp_reset_parm(nv12_ctx->isp_info_value.streamid, nv12_ctx->fps, nv12_ctx->width, nv12_ctx->height, nv12_ctx->format);
	isp_start(nv12_ctx->isp_info_value.streamid);
}