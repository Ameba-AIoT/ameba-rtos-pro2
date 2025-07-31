#include "platform_opts.h"
#include "module_video.h"
#include "video_api.h"
#include "fwfs.h"
#include "vfs.h"
#include "video_snapshot.h"
#include "sensor.h"
#include "ai_glass_media.h"
#include "media_filesystem.h"
#include "ai_glass_dbg.h"
#include "ai_glass_media.h"

// Configure
#define MAXIMUM_FILE_TAG_SIZE   32
#define MAXIMUM_FILE_SIZE       (MAXIMUM_FILE_TAG_SIZE + 32)
#define DROP_FRAME              2

typedef struct {
	uint8_t *output_buffer;
	uint32_t output_size;
} jpeg_buffer_t;

typedef struct {
	_sema snapshot_sema;
	unsigned char *jpeg_buf;
	unsigned int jpeg_len;
	unsigned int take_snapshot;
	unsigned int jpeg_index;
	uint32_t dest_addr;
	uint32_t dest_len;
	uint32_t dest_actual_len;
	_mutex snapshot_mutex;
	video_params_t video_snapshot_params;
	jpeg_buffer_t video_buf;
	mm_context_t *video_snapshot_ctx;
	int (*snapshot_write)(uint8_t *buf, uint32_t len, const char *filename);
} jpeg_aisnapshot_context_t;

// static video_params_t ai_video_params;
static jpeg_aisnapshot_context_t *ai_snap_ctx = NULL;

static video_pre_init_params_t ai_snap_pre_init_param = {
	.video_drop_enable = 1,
	.video_drop_frame = DROP_FRAME,
};

static int video_snapshot_cb(uint32_t jpeg_addr, uint32_t jpeg_len)
{
	ai_snap_ctx->take_snapshot = 1;
	AI_GLASS_MSG("capture_snapshot_cb snapshot size = %lu\n\r", jpeg_len);
	ai_snap_ctx->dest_addr = (uint32_t) malloc(jpeg_len);
	memcpy((void *)ai_snap_ctx->dest_addr, (const void *)jpeg_addr, jpeg_len);
	ai_snap_ctx->dest_actual_len = jpeg_len;
	AI_GLASS_MSG("capture_snapshot_cb snapshot addr = %ld, size = %lu\n\r", ai_snap_ctx->dest_addr, ai_snap_ctx->dest_actual_len);
	rtw_up_sema(&ai_snap_ctx->snapshot_sema);
	return 0;
}

static int video_snapshot_get_buffer(jpeg_buffer_t *video_buf, uint32_t timeout_ms)
{
	if (rtw_down_timeout_sema(&ai_snap_ctx->snapshot_sema, timeout_ms)) {
		video_buf->output_buffer = (uint8_t *) ai_snap_ctx->dest_addr;
		video_buf->output_size = ai_snap_ctx->dest_actual_len;
		AI_GLASS_MSG("video_snapshot_get_buffer size = %p, %lu\n\r", video_buf->output_buffer, video_buf->output_size);
		return 0;
	} else {
		AI_GLASS_ERR("video_snapshot_get_buffer size fail\n\r");
		video_buf->output_buffer = NULL;
		video_buf->output_size = 0;
		return -1;
	}
}

static int video_capture_snapshot(const char *filename)
{
	int ret = -1;
	if (!video_snapshot_get_buffer(&ai_snap_ctx->video_buf, SNAPSHOT_TIMEOUT)) {
		AI_GLASS_MSG("video_snapshot_get_buffer size = %p, %lu\n\r", ai_snap_ctx->video_buf.output_buffer, ai_snap_ctx->video_buf.output_size);
		if (!ai_snap_ctx->snapshot_write(ai_snap_ctx->video_buf.output_buffer, ai_snap_ctx->video_buf.output_size, filename)) {
			ret = 0;
		}
		ai_snap_ctx->take_snapshot = 0;
		AI_GLASS_INFO("get ai snapshot buffer success\r\n");
	} else {
		AI_GLASS_ERR("get ai snapshot buffer failed\r\n");
	}
	return ret;
}

static int aisnapshot_write_picture(uint8_t *buf, uint32_t len, const char *filename)
{
	FILE *m_file = NULL;
	AI_GLASS_MSG("jpeg %s, file len = %lu\r\n", filename, len);
	if (ai_snap_ctx->dest_addr) {
		m_file = ramdisk_fopen(filename, "w");
		if (m_file) {
			ramdisk_fwrite(buf, 1, len, m_file);
			ramdisk_fclose(m_file);
			free((void *)ai_snap_ctx->dest_addr);
			ai_snap_ctx->dest_addr = 0;
			return 0;
		} else {
			free((void *)ai_snap_ctx->dest_addr);
			ai_snap_ctx->dest_addr = 0;
			return -1;
		}
	} else {
		AI_GLASS_ERR("jpeg buffer allocate fail\r\n");
		return -1;
	}
}

int ai_snapshot_initialize(void)
{
	int ret = 0;

	if (ai_snap_ctx == NULL) {
		AI_GLASS_INFO("================AI snapshot start==========================\r\n");
		ai_snap_ctx = (jpeg_aisnapshot_context_t *) malloc(sizeof(jpeg_aisnapshot_context_t));
		memset(ai_snap_ctx, 0x00, sizeof(jpeg_aisnapshot_context_t));

		ai_glass_snapshot_param_t ai_snap_param;
		memset(&ai_snap_param, 0x00, sizeof(ai_glass_snapshot_param_t));
		media_get_ai_snapshot_params(&ai_snap_param);

		ai_snap_ctx->video_snapshot_ctx = mm_module_open(&video_module);
		video_params_t *snapshot_param = &(ai_snap_ctx->video_snapshot_params);
		snapshot_param->stream_id = MAIN_STREAM_ID;
		snapshot_param->type = VIDEO_JPEG;
		snapshot_param->width = ai_snap_param.width;
		snapshot_param->height = ai_snap_param.height;
		snapshot_param->jpeg_qlevel = ai_snap_param.jpeg_qlevel;
		snapshot_param->fps = sensor_params[USE_SENSOR].sensor_fps;
		snapshot_param->roi.xmin = 0;
		snapshot_param->roi.ymin = 0;
		snapshot_param->roi.xmax = 0;
		snapshot_param->roi.ymax = 0;
		snapshot_param->use_static_addr = 1;
		AI_GLASS_MSG("snapshot width = %ld\r\n", snapshot_param->width);
		AI_GLASS_MSG("snapshot height = %ld\r\n", snapshot_param->height);
		AI_GLASS_MSG("snapshot jpeg_qlevel = %ld\r\n", snapshot_param->jpeg_qlevel);

		video_pre_init_params_t ai_glass_pre_init_params = {0};
		rtw_init_sema(&ai_snap_ctx->snapshot_sema, 0);
		rtw_mutex_init(&ai_snap_ctx->snapshot_mutex);
		ai_snap_ctx->snapshot_write = aisnapshot_write_picture;
		if (ai_snap_ctx->video_snapshot_ctx) {
			media_get_preinit_isp_data(&ai_glass_pre_init_params);
			mm_module_ctrl(ai_snap_ctx->video_snapshot_ctx, CMD_VIDEO_PRE_INIT_PARM, (int)&ai_glass_pre_init_params);

			mm_module_ctrl(ai_snap_ctx->video_snapshot_ctx, CMD_VIDEO_SNAPSHOT_CB, (int)video_snapshot_cb);
			mm_module_ctrl(ai_snap_ctx->video_snapshot_ctx, CMD_VIDEO_SET_PARAMS, (int) & (ai_snap_ctx->video_snapshot_params));
			mm_module_ctrl(ai_snap_ctx->video_snapshot_ctx, CMD_VIDEO_PRE_INIT_PARM, (int)&ai_snap_pre_init_param);
			mm_module_ctrl(ai_snap_ctx->video_snapshot_ctx, MM_CMD_SET_QUEUE_LEN, 2);//Default 30
			mm_module_ctrl(ai_snap_ctx->video_snapshot_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
			mm_module_ctrl(ai_snap_ctx->video_snapshot_ctx, CMD_VIDEO_APPLY, ai_snap_ctx->video_snapshot_params.stream_id);
			video_ctrl(0, VIDEO_DEBUG, 0);
		} else {
			ret = -1;
			ai_snapshot_deinitialize();
			AI_GLASS_ERR("AI snapshot open fail\r\n");
			goto endofaisnapshot;
		}
	} else {
		ret = -2;
		AI_GLASS_WARN("AI snapshot is on-going\r\n");
		goto endofaisnapshot;
	}
endofaisnapshot:
	return ret;
}

int ai_snapshot_take(const char *file_name)
{
	AI_GLASS_INFO("================ai_snapshot_take==========================\r\n");
	int ret = -1;
	if (ai_snap_ctx) {
		AI_GLASS_MSG("Sanpshot start\r\n");
		rtw_mutex_get(&ai_snap_ctx->snapshot_mutex);
		AI_GLASS_MSG("ai_snapshot_take %s\r\n", file_name);
		ai_snap_ctx->take_snapshot = 1;
		mm_module_ctrl(ai_snap_ctx->video_snapshot_ctx, CMD_VIDEO_SNAPSHOT, 1);
		ret = video_capture_snapshot(file_name);
		ai_snap_ctx->take_snapshot = 0;
		rtw_mutex_put(&ai_snap_ctx->snapshot_mutex);
	} else {
		AI_GLASS_ERR("The snapshot is not init\r\n");
	}
	return ret;
}

int ai_snapshot_deinitialize(void)
{
	if (ai_snap_ctx) {
		if (ai_snap_ctx->take_snapshot) {
			AI_GLASS_WARN("It is running\r\n");
			return -1;
		} else {
			rtw_free_sema(&ai_snap_ctx->snapshot_sema);
			rtw_mutex_free(&ai_snap_ctx->snapshot_mutex);
			mm_module_ctrl(ai_snap_ctx->video_snapshot_ctx, CMD_VIDEO_STREAM_STOP, 0);
			mm_module_close(ai_snap_ctx->video_snapshot_ctx);
			free(ai_snap_ctx);
			ai_snap_ctx = NULL;
		}
	}
	AI_GLASS_INFO("DEINT WITH SINGLE\r\n");
	AI_GLASS_INFO("ai snapshot deinit\n");
	return 0;
}

