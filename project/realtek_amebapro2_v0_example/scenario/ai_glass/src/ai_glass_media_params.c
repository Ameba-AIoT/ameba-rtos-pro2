#include "ai_glass_media.h"
#include "module_video.h"
#include "sensor.h"
#include "ftl_common_api.h"
#include "video_boot.h"
#include "ai_glass_dbg.h"
#include "isp_ctrl_api.h"

#define OPEN_CHANNEL        0
#define OPEN_STREAM         STREAM_V1

#define MIN_VIDEO_WIDTH     80
#define MIN_VIDEO_HEIGHT    60

#define MIN_RECORD_WIDTH    MIN_VIDEO_WIDTH
#define MIN_RECORD_HEIGHT   MIN_VIDEO_HEIGHT
#define MIN_AISNAP_WIDTH    MIN_VIDEO_WIDTH
#define MIN_AISNAP_HEIGHT   MIN_VIDEO_HEIGHT
#define MIN_LIFESNAP_WIDTH  MIN_VIDEO_WIDTH
#define MIN_LIFESNAP_HEIGHT MIN_VIDEO_HEIGHT

#define IS_VALID_RECORD_TYPE(value) \
	((value) == VIDEO_H264 || \
     (value) == VIDEO_HEVC)

#define IS_VALID_RECORD_WIDTH(value) \
    ((value) >= MIN_RECORD_WIDTH && \
     (value) <= MAX_RECORD_WIDTH)

#define IS_VALID_RECORD_HEIGHT(value) \
    ((value) >= MIN_RECORD_HEIGHT && \
     (value) <= MAX_RECORD_HEIGHT)

#define IS_VALID_RECORD_BPS(value) \
	((value) >= MIN_RECORD_BPS && \
     (value) <= MAX_RECORD_BPS)

#define IS_VALID_RECORD_FPS(value) \
	((value) >= MIN_RECORD_FPS && \
     (value) <= MAX_RECORD_FPS)

#define IS_VALID_RECORD_GOP(value) \
	((value) >= MIN_RECORD_GOP && \
     (value) <= MAX_RECORD_GOP)
#define IS_VALID_RECORD_RCMODE(value) \
	 ((value) == 1 || (value) == 2 )
#define IS_VALID_RECORD_RECTIME(value) \
	((value) >= MIN_RECORD_RECTIME && \
     (value) <= MAX_RECORD_RECTIME)

#define IS_VALID_SNAP_TYPE(value) \
    ((value) == 2 )
#define IS_VALID_AISNAP_WIDTH(value) \
    ((value) >= MIN_AISNAP_WIDTH && \
     (value) <= MAX_AISNAP_WIDTH)
#define IS_VALID_AISNAP_HEIGHT(value) \
    ((value) >= MIN_AISNAP_HEIGHT && \
     (value) <= MAX_AISNAP_HEIGHT)
#define IS_VALID_SNAP_QVALUE(value) \
    ((value) >= 1 && \
     (value) <= 9)
#define IS_VALID_LIFESNAP_WIDTH(value) \
    ((value) >= MIN_LIFESNAP_WIDTH && \
     (value) <= MAX_LIFESNAP_WIDTH)
#define IS_VALID_LIFESNAP_HEIGHT(value) \
    ((value) >= MIN_LIFESNAP_HEIGHT && \
     (value) <= MAX_LIFESNAP_HEIGHT)

static ai_glass_record_param_t record_params = {
	.type = DEFAULT_RECORD_TYPE,
	.width = DEFAULT_RECORD_WIDTH,
	.height = DEFAULT_RECORD_HEIGHT,
	.bps = DEFAULT_RECORD_BPS,
	.fps = DEFAULT_RECORD_FPS,
	.gop = DEFAULT_RECORD_GOP,
	.roi = {
		.xmin = 0,
		.ymin = 0,
		.xmax = 0,
		.ymax = 0,
	},
	.minQp = DEFAULT_RECORD_MINQP,
	.maxQp = DEFAULT_RECORD_MAXQP,
	.rotation = DEFAULT_RECORD_ROTATION,
	.rc_mode = DEFAULT_RECORD_RCMODE,
	.record_length = DEFAULT_RECORD_RECTIME,
};

static ai_glass_snapshot_param_t ai_snapshot_params = {
	.type = DEFAULT_AISNAP_TYPE,
	.width = DEFAULT_AISNAP_WIDTH,
	.height = DEFAULT_AISNAP_HEIGHT,
	.jpeg_qlevel = DEFAULT_AISNAP_QLEVEL,
	.roi = {
		.xmin = 0,
		.ymin = 0,
		.xmax = 0,
		.ymax = 0,
	},
	.minQp = DEFAULT_AISNAP_MINQP,
	.maxQp = DEFAULT_AISNAP_MAXQP,
	.rotation = DEFAULT_AISNAP_ROTATION,
};

static ai_glass_snapshot_param_t life_snapshot_params = {
	.type = DEFAULT_LIFESNAP_TYPE,
	.width = DEFAULT_LIFESNAP_WIDTH,
	.height = DEFAULT_LIFESNAP_HEIGHT,
	.jpeg_qlevel = DEFAULT_LIFESNAP_QLEVEL,
	.roi = {
		.xmin = 0,
		.ymin = 0,
		.xmax = 0,
		.ymax = 0,
	},
	.minQp = DEFAULT_LIFESNAP_MINQP,
	.maxQp = DEFAULT_LIFESNAP_MAXQP,
	.rotation = DEFAULT_LIFESNAP_ROTATION,
};

static mm_context_t *video_fake_ctx = NULL;

static video_params_t video_fake_params = {
	.stream_id = OPEN_CHANNEL,
	.type = VIDEO_H264,
	.width = 176,
	.height = 144,
	.bps = 1024 * 1024,
	.fps = 6,
	.gop = 6,
	.rc_mode = 2,
	.use_static_addr = 1,
	.direct_output = 1
};

static video_pre_init_params_t ai_glass_pre_init_params = {0};

static int record_data_check(const ai_glass_record_param_t *params)
{
	// Todo: check the parameters
	if (params) {
		if (!IS_VALID_RECORD_TYPE(params->type)) {
			return MEDIA_INVALID_VTYPE;
		}
		if (!IS_VALID_RECORD_WIDTH(params->width)) {
			return MEDIA_INVALID_WIDTH;
		}
		if (!IS_VALID_RECORD_HEIGHT(params->height)) {
			return MEDIA_INVALID_HEIGHT;
		}
		if (!IS_VALID_RECORD_BPS(params->bps)) {
			return MEDIA_INVALID_BPS;
		}
		if (!IS_VALID_RECORD_FPS(params->fps)) {
			return MEDIA_INVALID_FPS;
		}
		if (!IS_VALID_RECORD_GOP(params->gop)) {
			return MEDIA_INVALID_GOP;
		}
		if (!IS_VALID_RECORD_RCMODE(params->rc_mode)) {
			return MEDIA_INVALID_RCMODE;
		}
		if (!IS_VALID_RECORD_RECTIME(params->record_length)) {
			return MEDIA_INVALID_RECTIME;
		}
	} else {
		return MEDIA_FAIL;
	}

	return MEDIA_OK;
}

static int ai_snapshot_data_check(const ai_glass_snapshot_param_t *params)
{
	// Todo: check the parameters
	if (params) {
		if (!IS_VALID_SNAP_TYPE(params->type)) {
			return MEDIA_INVALID_SNAP_TYPE;
		}
		if (!IS_VALID_AISNAP_WIDTH(params->width)) {
			return MEDIA_INVALID_WIDTH;
		}
		if (!IS_VALID_AISNAP_HEIGHT(params->height)) {
			return MEDIA_INVALID_HEIGHT;
		}
		if (!IS_VALID_SNAP_QVALUE(params->jpeg_qlevel)) {
			return MEDIA_INVALID_QVALUE;
		}
	} else {
		return MEDIA_FAIL;
	}

	return MEDIA_OK;
}

static int life_snapshot_data_check(const ai_glass_snapshot_param_t *params)
{
	// Todo: check the parameters
	if (params) {
		if (!IS_VALID_SNAP_TYPE(params->type)) {
			return MEDIA_INVALID_SNAP_TYPE;
		}
		if (!IS_VALID_LIFESNAP_WIDTH(params->width)) {
			return MEDIA_INVALID_WIDTH;
		}
		if (!IS_VALID_LIFESNAP_HEIGHT(params->height)) {
			return MEDIA_INVALID_HEIGHT;
		}
		if (!IS_VALID_SNAP_QVALUE(params->jpeg_qlevel)) {
			return MEDIA_INVALID_QVALUE;
		}
	} else {
		return MEDIA_FAIL;
	}

	return MEDIA_OK;
}

static int record_data_update_if_valid(ai_glass_record_param_t *ori_params, const ai_glass_record_param_t *params)
{
	int need_update = 0;
	if (IS_VALID_RECORD_TYPE(params->type) && ori_params->type != params->type) {
		need_update = 1;
		ori_params->type = params->type;
	}
	if (IS_VALID_RECORD_WIDTH(params->width) && IS_VALID_RECORD_HEIGHT(params->height) && (ori_params->width != params->width ||
			ori_params->height != params->height)) {
		need_update = 1;
		ori_params->width = params->width;
		ori_params->height = params->height;
		if ((ori_params->roi.xmax - ori_params->roi.xmin) < params->width) {
			ori_params->roi.xmax = params->width;
			ori_params->roi.xmin = 0;
		}
		if ((ori_params->roi.ymax - ori_params->roi.ymin) < params->height) {
			ori_params->roi.ymax = params->height;
			ori_params->roi.ymin = 0;
		}
	}
	if (IS_VALID_RECORD_WIDTH(params->roi.xmax) && IS_VALID_RECORD_HEIGHT(params->roi.ymax) && params->roi.xmax > params->roi.xmin &&
		params->roi.ymax > params->roi.ymin) {
		if (ori_params->width > (params->roi.xmax - params->roi.xmin) && ori_params->height > (params->roi.ymax - params->roi.ymin)) {
			need_update = 1;
			ori_params->roi.xmax = params->roi.xmax;
			ori_params->roi.xmin = params->roi.xmin;
			ori_params->roi.ymax = params->roi.ymax;
			ori_params->roi.ymin = params->roi.ymin;
		}
	}
	if (IS_VALID_RECORD_BPS(params->bps) && ori_params->bps != params->bps) {
		need_update = 1;
		ori_params->bps = params->bps;
	}
	if (IS_VALID_RECORD_FPS(params->fps) && ori_params->fps != params->fps) {
		need_update = 1;
		ori_params->fps = params->fps;
	}
	if (IS_VALID_RECORD_GOP(params->gop) && ori_params->gop != params->gop) {
		need_update = 1;
		ori_params->gop = params->gop;
	}
	if (IS_VALID_RECORD_RCMODE(params->rc_mode) && ori_params->rc_mode != params->rc_mode) {
		need_update = 1;
		ori_params->rc_mode = params->rc_mode;
	}
	if (IS_VALID_RECORD_RECTIME(params->record_length) && ori_params->record_length != params->record_length) {
		need_update = 1;
		ori_params->record_length = params->record_length;
	}

	if (need_update) {
		return MEDIA_OK;
	} else {
		return MEDIA_NO_NEED_TO_UPDATE;
	}
}

// Note: ROI need to >= picture size
static int ai_snapshot_update_if_valid(ai_glass_snapshot_param_t *ori_params, const ai_glass_snapshot_param_t *params)
{
	int need_update = 0;
	if (IS_VALID_SNAP_TYPE(params->type) && ori_params->type != params->type) {
		need_update = 1;
		ori_params->type = params->type;
	}
	if (IS_VALID_AISNAP_WIDTH(params->width) && IS_VALID_AISNAP_HEIGHT(params->height) && (ori_params->width != params->width ||
			ori_params->height != params->height)) {
		need_update = 1;
		ori_params->width = params->width;
		ori_params->height = params->height;
		if ((ori_params->roi.xmax - ori_params->roi.xmin) < params->width) {
			ori_params->roi.xmax = params->width;
			ori_params->roi.xmin = 0;
		}
		if ((ori_params->roi.ymax - ori_params->roi.ymin) < params->height) {
			ori_params->roi.ymax = params->height;
			ori_params->roi.ymin = 0;
		}
	}

	if (params->roi.xmax <= sensor_params[USE_SENSOR].sensor_width && params->roi.ymax <= sensor_params[USE_SENSOR].sensor_height &&
		params->roi.xmax > params->roi.xmin && params->roi.ymax > params->roi.ymin) {
		if (ori_params->width >= (params->roi.xmax - params->roi.xmin) && ori_params->height >= (params->roi.ymax - params->roi.ymin)) {
			need_update = 1;
			ori_params->roi.xmax = params->roi.xmax;
			ori_params->roi.xmin = params->roi.xmin;
			ori_params->roi.ymax = params->roi.ymax;
			ori_params->roi.ymin = params->roi.ymin;
		}
	}
	if (IS_VALID_SNAP_QVALUE(params->jpeg_qlevel) && ori_params->jpeg_qlevel != params->jpeg_qlevel) {
		need_update = 1;
		ori_params->jpeg_qlevel = params->jpeg_qlevel;
	}

	if (need_update) {
		return MEDIA_OK;
	} else {
		return MEDIA_NO_NEED_TO_UPDATE;
	}
}

// Note: ROI need to >= picture size
static int life_snapshot_update_if_valid(ai_glass_snapshot_param_t *ori_params, const ai_glass_snapshot_param_t *params)
{
	int need_update = 0;

	if (IS_VALID_SNAP_TYPE(params->type) && ori_params->type != params->type) {
		need_update = 1;
		ori_params->type = params->type;
	}
	if (IS_VALID_LIFESNAP_WIDTH(params->width) && IS_VALID_LIFESNAP_HEIGHT(params->height) && (ori_params->width != params->width ||
			ori_params->height != params->height)) {
		need_update = 1;
		ori_params->width = params->width;
		ori_params->height = params->height;

		uint32_t life_time_width = ((params->width > sensor_params[USE_SENSOR].sensor_width) ? sensor_params[USE_SENSOR].sensor_width : params->width);
		uint32_t life_time_height = ((params->height > sensor_params[USE_SENSOR].sensor_height) ? sensor_params[USE_SENSOR].sensor_height : params->height);
		if ((ori_params->roi.xmax - ori_params->roi.xmin) < life_time_width) {
			ori_params->roi.xmax = params->width;
			ori_params->roi.xmin = 0;
		}
		if ((ori_params->roi.ymax - ori_params->roi.ymin) < life_time_height) {
			ori_params->roi.ymax = params->height;
			ori_params->roi.ymin = 0;
		}
	}

	if (params->roi.xmax <= sensor_params[USE_SENSOR].sensor_width && params->roi.ymax <= sensor_params[USE_SENSOR].sensor_height &&
		params->roi.xmax > params->roi.xmin && params->roi.ymax > params->roi.ymin) {
		uint32_t life_time_width = ((ori_params->width > sensor_params[USE_SENSOR].sensor_width) ? sensor_params[USE_SENSOR].sensor_width : ori_params->width);
		uint32_t life_time_height = ((ori_params->height > sensor_params[USE_SENSOR].sensor_height) ? sensor_params[USE_SENSOR].sensor_height : ori_params->height);
		if (life_time_width >= (params->roi.xmax - params->roi.xmin) && life_time_height >= (params->roi.ymax - params->roi.ymin)) {
			need_update = 1;
			ori_params->roi.xmax = params->roi.xmax;
			ori_params->roi.xmin = params->roi.xmin;
			ori_params->roi.ymax = params->roi.ymax;
			ori_params->roi.ymin = params->roi.ymin;
		}
	}
	if (IS_VALID_SNAP_QVALUE(params->jpeg_qlevel) && ori_params->jpeg_qlevel != params->jpeg_qlevel) {
		need_update = 1;
		ori_params->jpeg_qlevel = params->jpeg_qlevel;
	}

	if (need_update) {
		return MEDIA_OK;
	} else {
		return MEDIA_NO_NEED_TO_UPDATE;
	}
}

static int media_set_record_params(const ai_glass_record_param_t *params)
{
	ai_glass_record_param_t temp_params = {0};
	memcpy(&temp_params, &record_params, sizeof(ai_glass_record_param_t));
	int ret = record_data_update_if_valid(&temp_params, params);
	if (ret == MEDIA_OK) {
		memcpy(&record_params, &temp_params, sizeof(ai_glass_record_param_t));
	}
	return ret;
}

static int media_set_ai_snapshot_params(const ai_glass_snapshot_param_t *params)
{
	ai_glass_snapshot_param_t temp_params = {0};
	memcpy(&temp_params, &ai_snapshot_params, sizeof(ai_glass_snapshot_param_t));
	int ret = ai_snapshot_update_if_valid(&temp_params, params);
	if (ret == MEDIA_OK) {
		memcpy(&ai_snapshot_params, &temp_params, sizeof(ai_glass_snapshot_param_t));
	}
	return ret;
}

static int media_set_life_snapshot_params(const ai_glass_snapshot_param_t *params)
{
	ai_glass_snapshot_param_t temp_params = {0};
	memcpy(&temp_params, &life_snapshot_params, sizeof(ai_glass_snapshot_param_t));
	int ret = life_snapshot_update_if_valid(&temp_params, params);
	if (ret == MEDIA_OK) {
		memcpy(&life_snapshot_params, &temp_params, sizeof(ai_glass_snapshot_param_t));
	}
	return ret;
}

static int media_update_record_params_to_flash(const ai_glass_record_param_t *params)
{
	// update data to flash
	unsigned char *record_buf = malloc(FLASH_REC_BLOCK_SIZE); //Allocate a 2KB buffer
	unsigned int flash_addr = 0;

	if (sys_get_boot_sel() == 0) {
		flash_addr = FLASH_REC_BLOCK_BASE;
	} else {
		// Placeholder for NAND FLASH ADDR in future
	}
	if (record_buf == NULL) {
		AI_GLASS_ERR("It can't get the record buffer\r\n");
		return MEDIA_FAIL;
	}

	memset(record_buf, 0x00, FLASH_REC_BLOCK_SIZE);
	record_buf[0] = 'R';  // Add tag for identification (Record params)
	record_buf[1] = 'E';
	record_buf[2] = 'C';
	record_buf[3] = 'D';

	memcpy(record_buf + 4, params, sizeof(ai_glass_record_param_t));

	ftl_common_write(flash_addr, record_buf, FLASH_REC_BLOCK_SIZE);
	memset(record_buf, 0xff, FLASH_REC_BLOCK_SIZE);
	ftl_common_read(flash_addr, record_buf, FLASH_REC_BLOCK_SIZE);
	ai_glass_record_param_t *read_data = (ai_glass_record_param_t *)(record_buf + 4);
	AI_GLASS_MSG("[FLASH]type: %u, width: %u, height:%u, bps: %u, fps: %u, gop: %u, roi_xmin: %u, roi_ymin: %u, roi_xmax: %u, roi_ymax: %u, minQp: %u, maxQp: %u, rotation: %u, rc_mode: %u, record_length:%u\r\n",
				 read_data->type, read_data->width, read_data->height, read_data->bps, read_data->fps, read_data->gop, read_data->roi.xmin, read_data->roi.ymin,
				 read_data->roi.xmax, read_data->roi.ymax, read_data->minQp, read_data->maxQp, read_data->rotation, read_data->rc_mode, read_data->record_length);

	if (record_buf) {
		free(record_buf);
	}
	return MEDIA_OK;
}

static int media_update_ai_snapshot_params_to_flash(const ai_glass_snapshot_param_t *params)
{
	unsigned char *ai_snap_buf = malloc(FLASH_AI_SNAP_BLOCK_SIZE); //Allocate a 2KB buffer
	unsigned int flash_addr = 0;
	if (sys_get_boot_sel() == 0) {
		flash_addr = FLASH_AI_SNAP_BLOCK_BASE;
	} else {
		// Placeholder for NAND FLASH ADDR in future
	}
	if (ai_snap_buf == NULL) {
		AI_GLASS_ERR("It can't get the ai snapshot buffer\r\n");
		return MEDIA_FAIL;
	}

	memset(ai_snap_buf, 0x00, FLASH_AI_SNAP_BLOCK_SIZE);
	ai_snap_buf[0] = 'A';  // Add tag for identification (AI snapshot params)
	ai_snap_buf[1] = 'I';
	ai_snap_buf[2] = 'S';
	ai_snap_buf[3] = 'N';
	ai_snap_buf[4] = 'A';
	ai_snap_buf[5] = 'P';

	memcpy(ai_snap_buf + 6, params, sizeof(ai_glass_snapshot_param_t));
	ftl_common_write(flash_addr, ai_snap_buf, FLASH_AI_SNAP_BLOCK_SIZE);
	memset(ai_snap_buf, 0xff, FLASH_AI_SNAP_BLOCK_SIZE);
	ftl_common_read(flash_addr, ai_snap_buf, FLASH_AI_SNAP_BLOCK_SIZE);
	ai_glass_snapshot_param_t *read_data = (ai_glass_snapshot_param_t *)(ai_snap_buf + 6);
	AI_GLASS_MSG("[FLASH_AISNAP]type: %u, width: %u, height:%u, jpeg_qlevel: %u, roi_xmin: %u, roi_ymin: %u, roi_xmax: %u, roi_ymax: %u, minQp: %u, maxQp: %u, rotation: %u\r\n",
				 read_data->type, read_data->width, read_data->height, read_data->jpeg_qlevel, read_data->roi.xmin, read_data->roi.ymin, read_data->roi.xmax,
				 read_data->roi.ymax, read_data->minQp, read_data->maxQp, read_data->rotation);
	if (ai_snap_buf) {
		free(ai_snap_buf);
	}
	return MEDIA_OK;
}

static int media_update_life_snapshot_params_to_flash(const ai_glass_snapshot_param_t *params)
{
	unsigned char *lifetime_snap_buf = malloc(FLASH_LIFE_SNAP_BLOCK_SIZE); //Allocate a 2KB buffer
	unsigned int flash_addr = 0;
	if (sys_get_boot_sel() == 0) {
		flash_addr = FLASH_LIFE_SNAP_BLOCK_BASE;
	} else {
		// Placeholder for NAND FLASH ADDR in future
	}
	if (lifetime_snap_buf == NULL) {
		AI_GLASS_ERR("It can't get the lifetime snapshot buffer\r\n");
		return MEDIA_FAIL;
	}

	memset(lifetime_snap_buf, 0x00, FLASH_LIFE_SNAP_BLOCK_SIZE);
	lifetime_snap_buf[0] = 'L';  // Add tag for identification (lifetime snapshot params)
	lifetime_snap_buf[1] = 'I';
	lifetime_snap_buf[2] = 'F';
	lifetime_snap_buf[3] = 'E';
	lifetime_snap_buf[4] = 'S';
	lifetime_snap_buf[5] = 'N';
	lifetime_snap_buf[6] = 'A';
	lifetime_snap_buf[7] = 'P';

	memcpy(lifetime_snap_buf + 8, params, sizeof(ai_glass_snapshot_param_t));
	ftl_common_write(flash_addr, lifetime_snap_buf, FLASH_LIFE_SNAP_BLOCK_SIZE);
	memset(lifetime_snap_buf, 0xff, FLASH_LIFE_SNAP_BLOCK_SIZE);
	ftl_common_read(flash_addr, lifetime_snap_buf, FLASH_LIFE_SNAP_BLOCK_SIZE);
	ai_glass_snapshot_param_t *read_data = (ai_glass_snapshot_param_t *)(lifetime_snap_buf + 8);
	AI_GLASS_MSG("[FLASH_LIFESNAP]type: %u, width: %u, height:%u, jpeg_qlevel: %u, roi_xmin: %u, roi_ymin: %u, roi_xmax: %u, roi_ymax: %u, minQp: %u, maxQp: %u, rotation: %u\r\n",
				 read_data->type, read_data->width, read_data->height, read_data->jpeg_qlevel, read_data->roi.xmin, read_data->roi.ymin, read_data->roi.xmax,
				 read_data->roi.ymax, read_data->minQp, read_data->maxQp, read_data->rotation);
	if (lifetime_snap_buf) {
		free(lifetime_snap_buf);
	}
	return MEDIA_OK;
}

static int media_get_record_params_from_flash(ai_glass_record_param_t *params)
{
	if (params == NULL) {
		AI_GLASS_ERR("Input buffer for record params is NULL\r\n");
		return MEDIA_FAIL;
	}

	unsigned char *record_buf = malloc(FLASH_REC_BLOCK_SIZE); //Allocate a 2KB buffer
	unsigned int flash_addr = 0;
	int ret = 0;

	if (sys_get_boot_sel() == 0) {
		flash_addr = FLASH_REC_BLOCK_BASE;
	} else {
		// Placeholder for NAND FLASH ADDR in future
	}
	if (record_buf == NULL) {
		AI_GLASS_ERR("It can't get the record buffer\r\n");
		return MEDIA_FAIL;
	}
	memset(record_buf, 0x00, FLASH_REC_BLOCK_SIZE);
	ftl_common_read(flash_addr, record_buf, FLASH_REC_BLOCK_SIZE);
	if (record_buf[0] == 'R' && record_buf[1] == 'E' && record_buf[2] == 'C' && record_buf[3] == 'D') {
		memcpy(params, record_buf + 4, sizeof(ai_glass_record_param_t));
		ret = MEDIA_OK;
	} else {
		AI_GLASS_WARN("Get Record Param from flash failed\r\n");
		ret = MEDIA_FAIL;
	}

	if (record_buf) {
		free(record_buf);
	}
	return ret;
}

static int media_get_ai_snapshot_params_from_flash(ai_glass_snapshot_param_t *params)
{
	if (params == NULL) {
		AI_GLASS_ERR("Input buffer for ai snapshot params is NULL\r\n");
		return MEDIA_FAIL;
	}

	unsigned char *ai_snap_buf = malloc(FLASH_AI_SNAP_BLOCK_SIZE);
	unsigned int flash_addr = 0;
	int ret = 0;

	if (sys_get_boot_sel() == 0) {
		flash_addr = FLASH_AI_SNAP_BLOCK_BASE;
	} else {
		// Placeholder for NAND FLASH ADDR in future
	}
	if (ai_snap_buf == NULL) {
		AI_GLASS_ERR("It can't get the ai snapshot buffer\r\n");
		return MEDIA_FAIL;
	}
	memset(ai_snap_buf, 0x00, FLASH_AI_SNAP_BLOCK_SIZE);
	ftl_common_read(flash_addr, ai_snap_buf, FLASH_AI_SNAP_BLOCK_SIZE);
	if (ai_snap_buf[0] == 'A' && ai_snap_buf[1] == 'I' && ai_snap_buf[2] == 'S' && ai_snap_buf[3] == 'N' && ai_snap_buf[4] == 'A' && ai_snap_buf[5] == 'P') {
		memcpy(params, ai_snap_buf + 6, sizeof(ai_glass_snapshot_param_t));
		ret = MEDIA_OK;
	} else {
		AI_GLASS_WARN("Get AI Snapshot Param from flash failed\r\n");
		ret = MEDIA_FAIL;
	}

	if (ai_snap_buf) {
		free(ai_snap_buf);
	}
	return ret;
}

static int media_get_life_snapshot_params_from_flash(ai_glass_snapshot_param_t *params)
{
	if (params == NULL) {
		AI_GLASS_ERR("Input buffer for lifetime snapshot params is NULL\r\n");
		return MEDIA_FAIL;
	}

	unsigned char *lifetime_snap_buf = malloc(FLASH_LIFE_SNAP_BLOCK_SIZE);
	unsigned int flash_addr = 0;
	int ret = 0;

	if (sys_get_boot_sel() == 0) {
		flash_addr = FLASH_LIFE_SNAP_BLOCK_BASE;
	} else {
		// Placeholder for NAND FLASH ADDR in future
	}
	if (lifetime_snap_buf == NULL) {
		AI_GLASS_ERR("It can't get the lifetime snapshot buffer\r\n");
		return MEDIA_FAIL;
	}
	memset(lifetime_snap_buf, 0x00, FLASH_LIFE_SNAP_BLOCK_SIZE);
	ftl_common_read(flash_addr, lifetime_snap_buf, FLASH_LIFE_SNAP_BLOCK_SIZE);
	if (lifetime_snap_buf[0] == 'L' && lifetime_snap_buf[1] == 'I' && lifetime_snap_buf[2] == 'F' && lifetime_snap_buf[3] == 'E' && lifetime_snap_buf[4] == 'S' &&
		lifetime_snap_buf[5] == 'N' && lifetime_snap_buf[6] == 'A' && lifetime_snap_buf[7] == 'P') {
		memcpy(params, lifetime_snap_buf + 8, sizeof(ai_glass_snapshot_param_t));
		ret = MEDIA_OK;
	} else {
		AI_GLASS_WARN("Get LifeTime Snapshot Param from flash failed\r\n");
		ret = MEDIA_FAIL;
	}

	if (lifetime_snap_buf) {
		free(lifetime_snap_buf);
	}
	return ret;
}

void print_record_data(const ai_glass_record_param_t *params)
{
	printf("record_params print\r\n");
	printf("type = %u\r\n", params->type);
	printf("width = %u\r\n", params->width);
	printf("height = %u\r\n", params->height);
	printf("bps = %lu\r\n", params->bps);
	printf("fps = %u\r\n", params->fps);
	printf("gop = %u\r\n", params->gop);
	printf("roi.xmin = %lu\r\n", params->roi.xmin);
	printf("roi.ymin = %lu\r\n", params->roi.ymin);
	printf("roi.xmax = %lu\r\n", params->roi.xmax);
	printf("roi.ymax = %lu\r\n", params->roi.ymax);
	printf("minQp = %u\r\n", params->minQp);
	printf("maxQp = %u\r\n", params->maxQp);
	printf("rotation = %u\r\n", params->rotation);
	printf("rc_mode = %u\r\n", params->rc_mode);
	printf("record_length = %u\r\n", params->record_length);
}

void print_snapshot_data(const ai_glass_snapshot_param_t *params)
{
	printf("snapshot_params print\r\n");
	printf("type = %u\r\n", params->type);
	printf("width = %lu\r\n", params->width);
	printf("height = %lu\r\n", params->height);
	printf("jpeg_qlevel = %u\r\n", params->jpeg_qlevel);
	printf("roi.xmin = %lu\r\n", params->roi.xmin);
	printf("roi.ymin = %lu\r\n", params->roi.ymin);
	printf("roi.xmax = %lu\r\n", params->roi.xmax);
	printf("roi.ymax = %lu\r\n", params->roi.ymax);
	printf("minQp = %u\r\n", params->minQp);
	printf("maxQp = %u\r\n", params->maxQp);
	printf("rotation = %u\r\n", params->rotation);
}

int media_get_record_params(ai_glass_record_param_t *params)
{
	if (params) {
		memcpy(params, &record_params, sizeof(ai_glass_record_param_t));
		return MEDIA_OK;
	}
	return MEDIA_FAIL;
}

int media_get_ai_snapshot_params(ai_glass_snapshot_param_t *params)
{
	if (params) {
		memcpy(params, &ai_snapshot_params, sizeof(ai_glass_snapshot_param_t));
		return MEDIA_OK;
	}
	return MEDIA_FAIL;
}

int media_get_life_snapshot_params(ai_glass_snapshot_param_t *params)
{
	if (params) {
		memcpy(params, &life_snapshot_params, sizeof(ai_glass_snapshot_param_t));
		return MEDIA_OK;
	}
	return MEDIA_FAIL;
}

int media_update_record_params(const ai_glass_record_param_t *params)
{
	int ret = media_set_record_params(params);
	if (ret == MEDIA_OK) {
		// update data to flash
		return media_update_record_params_to_flash(params);
	} else if (ret == MEDIA_NO_NEED_TO_UPDATE) {
		return MEDIA_OK;
	}
	return MEDIA_FAIL;
}

int media_update_record_time(uint16_t record_length)
{
	ai_glass_record_param_t temp_record_params = {0};
	media_get_record_params(&temp_record_params);
	temp_record_params.record_length = record_length;
	if (media_update_record_params(&temp_record_params) == MEDIA_OK) {
		// update data to flash has been done by media_update_record_params.
		return MEDIA_OK;
	}

	return MEDIA_FAIL;
}

int media_update_ai_snapshot_params(const ai_glass_snapshot_param_t *params)
{
	int ret = media_set_ai_snapshot_params(params);
	if (ret == MEDIA_OK) {
		// update data to flash
		return media_update_ai_snapshot_params_to_flash(params);
	} else if (ret == MEDIA_NO_NEED_TO_UPDATE) {
		return MEDIA_OK;
	}
	return MEDIA_FAIL;
}

int media_update_life_snapshot_params(const ai_glass_snapshot_param_t *params)
{
	int ret = media_set_life_snapshot_params(params);
	if (ret == MEDIA_OK) {
		// update data to flash
		return media_update_life_snapshot_params_to_flash(params);
	} else if (ret == MEDIA_NO_NEED_TO_UPDATE) {
		return MEDIA_OK;
	}
	return MEDIA_FAIL;
}

void media_update_preinit_isp_data(video_pre_init_params_t *isp_data)
{
	memcpy(&ai_glass_pre_init_params, isp_data, sizeof(video_pre_init_params_t));
}
void media_get_preinit_isp_data(video_pre_init_params_t *isp_data)
{
	memcpy(isp_data, &ai_glass_pre_init_params, sizeof(video_pre_init_params_t));
}
void media_update_preinit_isp_ae(void)
{
	int ae_exposure_time = 0;
	int ae_gain = 0;
	isp_get_exposure_time(&ae_exposure_time);
	isp_get_ae_gain(&ae_gain);
	ai_glass_pre_init_params.isp_ae_init_exposure = ae_exposure_time;
	ai_glass_pre_init_params.isp_ae_init_gain = ae_gain;
}
void media_update_preinit_isp_awb(void)
{
	int awb_rgain = 0;
	int awb_bgain = 0;
	isp_get_red_balance(&awb_rgain);
	isp_get_blue_balance(&awb_bgain);
	ai_glass_pre_init_params.isp_awb_init_rgain = awb_rgain;
	ai_glass_pre_init_params.isp_awb_init_bgain = awb_bgain;
}
void initial_media_parameters(void)
{
	video_boot_stream_t *isp_fcs_info;
	video_get_fcs_info(&isp_fcs_info);//Get the fcs info
	int voe_heap_size = 0;
	video_pre_init_params_t pre_init_params = {0};

	if (isp_fcs_info->fcs_status) {
		// the isp has been set up when fcs, please set up isp in the fcs status
		AI_GLASS_MSG("==================fcs on==============\r\n");
		voe_heap_size = video_voe_presetting(1, isp_fcs_info->video_params[STREAM_V1].width, isp_fcs_info->video_params[STREAM_V1].height,
											 isp_fcs_info->video_params[STREAM_V1].bps, 1, 1, isp_fcs_info->video_params[STREAM_V2].width, isp_fcs_info->video_params[STREAM_V2].height,
											 isp_fcs_info->video_params[STREAM_V2].bps, 1, 0, isp_fcs_info->video_params[STREAM_V3].width, isp_fcs_info->video_params[STREAM_V3].height,
											 isp_fcs_info->video_params[STREAM_V3].bps, 1, 0, 0, 0);
		video_fake_params.bps = isp_fcs_info->video_params[OPEN_STREAM].bps;
		video_fake_params.width = isp_fcs_info->video_params[OPEN_STREAM].width;
		video_fake_params.height = isp_fcs_info->video_params[OPEN_STREAM].height;
		video_fake_params.fps = isp_fcs_info->video_params[OPEN_STREAM].fps;
		video_fake_params.gop = isp_fcs_info->video_params[OPEN_STREAM].gop;
		video_fake_ctx = mm_module_open(&video_module);
		if (video_fake_ctx) {
			mm_module_ctrl(video_fake_ctx, CMD_VIDEO_GET_PRE_INIT_PARM, (int)&pre_init_params);
			pre_init_params.isp_init_enable = 1;
			pre_init_params.init_isp_items.init_brightness = 0;
			pre_init_params.init_isp_items.init_contrast = 50;
			pre_init_params.init_isp_items.init_flicker = 1;
			pre_init_params.init_isp_items.init_hdr_mode = 0;
			pre_init_params.init_isp_items.init_mirrorflip = 0xf0; // flip and mirror
			pre_init_params.init_isp_items.init_saturation = 50;
			pre_init_params.init_isp_items.init_wdr_level = 50;
			pre_init_params.init_isp_items.init_wdr_mode = 2;
			pre_init_params.init_isp_items.init_mipi_mode = 0;
			pre_init_params.voe_dbg_disable = !APP_VOE_LOG_EN;
#if defined(ENABLE_META_INFO)
			unsigned char uuid[16] = {0xc7, 0x98, 0x2c, 0x28, 0x0a, 0xfc, 0x49, 0xe6, 0xaa, 0xe4, 0x7f, 0x8f, 0x64, 0xee, 0x65, 0x01};
			pre_init_params.meta_enable = 1;
#ifdef ENABLE_JPEG_EXIF
			pre_init_params.meta_size = VIDEO_META_USER_SIZE + 0x100;
#else
			pre_init_params.meta_size = VIDEO_META_USER_SIZE;
#endif
			memcpy(pre_init_params.video_meta_uuid, uuid, VIDEO_META_UUID_SIZE);
#endif
			// Since the fcs has open the channl, we do not need to apply the preinit setting again
			media_update_preinit_isp_data(&pre_init_params);

			mm_module_ctrl(video_fake_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_fake_params);
			mm_module_ctrl(video_fake_ctx, MM_CMD_SET_QUEUE_LEN, 60);
			mm_module_ctrl(video_fake_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
			mm_module_ctrl(video_fake_ctx, CMD_VIDEO_APPLY, OPEN_CHANNEL);
		} else {
			AI_GLASS_ERR("video open fail\n\r");
		}
	} else {
		AI_GLASS_MSG("==================fcs off==============\r\n");
		voe_heap_size = video_voe_presetting(1, 176, 144, 1024 * 1024, 0, 1, sensor_params[USE_SENSOR].sensor_width, sensor_params[USE_SENSOR].sensor_height,
											 MAX_RECORD_BPS, 1, 0,
											 sensor_params[USE_SENSOR].sensor_width, sensor_params[USE_SENSOR].sensor_height, 0, 1, 0, 0, 0);
		video_fake_params.bps = isp_fcs_info->video_params[OPEN_STREAM].bps;
		video_fake_params.width = isp_fcs_info->video_params[OPEN_STREAM].width;
		video_fake_params.height = isp_fcs_info->video_params[OPEN_STREAM].height;
		video_fake_params.fps = isp_fcs_info->video_params[OPEN_STREAM].fps;
		video_fake_params.gop = isp_fcs_info->video_params[OPEN_STREAM].gop;
		video_fake_ctx = mm_module_open(&video_module);
		if (video_fake_ctx) {
			mm_module_ctrl(video_fake_ctx, CMD_VIDEO_GET_PRE_INIT_PARM, (int)&pre_init_params);
			pre_init_params.isp_init_enable = 1;
			pre_init_params.init_isp_items.init_brightness = 0;
			pre_init_params.init_isp_items.init_contrast = 50;
			pre_init_params.init_isp_items.init_flicker = 1;
			pre_init_params.init_isp_items.init_hdr_mode = 0;
			pre_init_params.init_isp_items.init_mirrorflip = 0xf0; // no flip and no mirror
			pre_init_params.init_isp_items.init_saturation = 50;
			pre_init_params.init_isp_items.init_wdr_level = 50;
			pre_init_params.init_isp_items.init_wdr_mode = 2;
			pre_init_params.init_isp_items.init_mipi_mode = 0;
			pre_init_params.voe_dbg_disable = !APP_VOE_LOG_EN;
			// Init ISP parameters
			mm_module_ctrl(video_fake_ctx, CMD_VIDEO_PRE_INIT_PARM, (int)&pre_init_params);

			media_update_preinit_isp_data(&pre_init_params);
			mm_module_ctrl(video_fake_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_fake_params);
			mm_module_ctrl(video_fake_ctx, MM_CMD_SET_QUEUE_LEN, 60);
			mm_module_ctrl(video_fake_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
			mm_module_ctrl(video_fake_ctx, CMD_VIDEO_APPLY, OPEN_CHANNEL);
		} else {
			AI_GLASS_ERR("video open fail\n\r");
		}
	}
	AI_GLASS_MSG("\r\n voe heap size = %d\r\n", voe_heap_size);

	// For testing we do not use the temp value
	// Todo: get data from the flash first and store in temp data
	ai_glass_record_param_t temp_record_parames = {0};
	ai_glass_snapshot_param_t temp_ai_snap_parames = {0};
	ai_glass_snapshot_param_t temp_life_snap_parames = {0};

	if (media_get_record_params_from_flash(&temp_record_parames) == MEDIA_OK) {
		AI_GLASS_INFO("Get Record Parameters From Flash Success\r\n");
		record_data_update_if_valid(&record_params, &temp_record_parames);
	}
	media_update_record_params_to_flash(&record_params);
	if (media_get_ai_snapshot_params_from_flash(&temp_ai_snap_parames) == MEDIA_OK) {
		AI_GLASS_INFO("Get AI Snapshot Parameters From Flash Success\r\n");
		ai_snapshot_update_if_valid(&ai_snapshot_params, &temp_ai_snap_parames);
	}
	media_update_ai_snapshot_params_to_flash(&ai_snapshot_params);
	if (media_get_life_snapshot_params_from_flash(&temp_life_snap_parames) == MEDIA_OK) {
		AI_GLASS_INFO("Get LifeTime Snapshot Parameters Success\r\n");
		life_snapshot_update_if_valid(&life_snapshot_params, &temp_life_snap_parames);
	}
	media_update_life_snapshot_params_to_flash(&life_snapshot_params);
}

void deinitial_media(void)
{
	if (video_fake_ctx) {
		// Update AE and AWB when closing the channel
		media_update_preinit_isp_ae();
		media_update_preinit_isp_awb();
		mm_module_ctrl(video_fake_ctx, CMD_VIDEO_STREAM_STOP, OPEN_CHANNEL);
		mm_module_close(video_fake_ctx);
		video_fake_ctx = NULL;
		AI_GLASS_MSG("Close the fake channel used to keep VOE on\r\n");
	} else {
		AI_GLASS_MSG("The Last Channel has been closed\r\n");
	}
}