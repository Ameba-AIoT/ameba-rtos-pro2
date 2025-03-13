/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "platform_opts.h"
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "module_video.h"
#include "video_api.h"
#include "fwfs.h"
#include "vfs.h"
#include "sensor.h"
#include "ai_glass_media.h"
#include "module_filesaver.h"
#include "nv12tojpg.h"
#include "media_filesystem.h"
#include "ai_glass_dbg.h"

// Definition of LIFE SNAPSHOT STATUS
#define LIFESNAP_IDLE    0x00
#define LIFESNAP_START   0x01
#define LIFESNAP_TAKE    0x02
#define LIFESNAP_DONE    0x03
#define LIFESNAP_FAIL    0x04

// Configure for lifetime snapshot
#define JPG_WRITE_SIZE          4096
#define LIFE_SNAP_PRIORITY      5
#define MAXIMUM_FILE_TAG_SIZE   32
#define MAXIMUM_FILE_SIZE       (MAXIMUM_FILE_TAG_SIZE + 32)

static char snapshot_name[MAXIMUM_FILE_SIZE];

typedef struct {
	uint8_t         need_scaleup;
	uint32_t        jpg_width;
	uint32_t        jpg_height;
	uint32_t        jpg_qlevel;
	video_params_t  params;
} jpeg_lifesnapshot_params_t;

//Modules
//ls means lifetime snapshot
static jpeg_lifesnapshot_params_t ls_video_params = {0};
static mm_context_t *ls_snapshot_ctx    = NULL;
static mm_context_t *ls_filesaver_ctx   = NULL;

//linker
static mm_siso_t *ls_siso_snapshot_filesaver = NULL;

static int is_file_saved = LIFESNAP_IDLE;

static void lifetime_snapshot_file_save(char *file_path, uint32_t data_addr, uint32_t data_size)
{
	uint8_t *output_jpg_buf = NULL;
	AI_GLASS_MSG("file_path:%s  data_addr:%ld  data_size:%ld \r\n", file_path, data_addr, data_size);
	if (is_file_saved == LIFESNAP_TAKE) {
		int ret = 0;
		AI_GLASS_MSG("get liftime snapshot frame time %lu\r\n", mm_read_mediatime_ms());
		AI_GLASS_MSG("file_path:%s  data_addr:%ld  data_size:%ld \r\n", file_path, data_addr, data_size);

		uint8_t *input_nv12_buf = NULL;
		uint32_t nv12_size = ls_video_params.jpg_width * ls_video_params.jpg_height / 2 * 3;

		if (ls_video_params.need_scaleup) {
			input_nv12_buf = malloc(nv12_size);
			if (!input_nv12_buf) {
				AI_GLASS_ERR("allocate scale up nv12 buffer size %ld/%u fail\r\n", nv12_size, xPortGetFreeHeapSize());
				ret = -1;
				goto closebuff;
			}
			custom_resize_for_nv12((uint8_t *)data_addr, ls_video_params.params.width, ls_video_params.params.height, input_nv12_buf, ls_video_params.jpg_width,
								   ls_video_params.jpg_height);
		} else {
			input_nv12_buf = (uint8_t *)data_addr;
		}
		AI_GLASS_MSG("get liftime snapshot frame resize done time %lu\r\n", mm_read_mediatime_ms());
		output_jpg_buf = malloc(nv12_size);
		if (!output_jpg_buf) {
			AI_GLASS_ERR("allocate jpg buffer size %ld/%u fail\r\n", nv12_size, xPortGetFreeHeapSize());
			ret = -1;
			goto closebuff;
		}

		FILE *life_snapshot_file = extdisk_fopen(file_path, "wb");
		if (!life_snapshot_file) {
			AI_GLASS_ERR("open jpg file %s fail\r\n", file_path);
			ret = -1;
			goto closebuff;
		}
		uint32_t jpg_size = nv12_size;
		custom_jpegEnc_from_nv12(input_nv12_buf, ls_video_params.jpg_width, ls_video_params.jpg_height, output_jpg_buf, ls_video_params.jpg_qlevel, nv12_size,
								 &jpg_size);
		AI_GLASS_MSG("get liftime snapshot frame encode done time %lu\r\n", mm_read_mediatime_ms());
		//write jpg data
		for (uint32_t i = 0; i < jpg_size; i += JPG_WRITE_SIZE) {
			extdisk_fwrite(output_jpg_buf + i, 1, ((i + JPG_WRITE_SIZE) >= jpg_size) ? (jpg_size - i) : JPG_WRITE_SIZE, life_snapshot_file);
		}

		extdisk_fclose(life_snapshot_file);

closebuff:
		if (ls_video_params.need_scaleup && input_nv12_buf) {
			free(input_nv12_buf);
			input_nv12_buf = NULL;
		}
		if (output_jpg_buf) {
			free(output_jpg_buf);
			output_jpg_buf = NULL;
		}
		if (ret != 0) {
			is_file_saved = LIFESNAP_FAIL;
		} else {
			is_file_saved = LIFESNAP_DONE;
		}
		AI_GLASS_MSG("get liftime snapshot frame all done time %lu\r\n", mm_read_mediatime_ms());
	}
}

int lifetime_snapshot_initialize(void)
{
	int ret = 0;

	if (is_file_saved != LIFESNAP_IDLE) {
		ret = -2;
		goto endoflifesnapshot;
	}

	ls_filesaver_ctx = mm_module_open(&filesaver_module);
	if (ls_filesaver_ctx) {
		mm_module_ctrl(ls_filesaver_ctx, CMD_FILESAVER_SET_TYPE_HANDLER, (int)lifetime_snapshot_file_save);
	} else {
		AI_GLASS_ERR("filesaver open fail\n\r");
		ret = -1;
		goto endoflifesnapshot;
	}

	ai_glass_snapshot_param_t life_snap_param;
	memset(&life_snap_param, 0x00, sizeof(ai_glass_snapshot_param_t));
	media_get_life_snapshot_params(&life_snap_param);
	if (life_snap_param.width <= sensor_params[USE_SENSOR].sensor_width && life_snap_param.height <= sensor_params[USE_SENSOR].sensor_height) {
		ls_video_params.params.width = life_snap_param.width;
		ls_video_params.params.height = life_snap_param.height;
		ls_video_params.jpg_width = life_snap_param.width;
		ls_video_params.jpg_height = life_snap_param.height;
		ls_video_params.need_scaleup = 0;
	} else if (life_snap_param.height <= sensor_params[USE_SENSOR].sensor_height) {
		ls_video_params.params.width = sensor_params[USE_SENSOR].sensor_width;
		ls_video_params.params.height = life_snap_param.height;
		ls_video_params.jpg_width = life_snap_param.width;
		ls_video_params.jpg_height = life_snap_param.height;
		ls_video_params.need_scaleup = 1;
	} else if (life_snap_param.height <= sensor_params[USE_SENSOR].sensor_height) {
		ls_video_params.params.width = life_snap_param.width;
		ls_video_params.params.height = sensor_params[USE_SENSOR].sensor_height;
		ls_video_params.jpg_width = life_snap_param.width;
		ls_video_params.jpg_height = life_snap_param.height;
		ls_video_params.need_scaleup = 1;
	} else {
		ls_video_params.params.width = sensor_params[USE_SENSOR].sensor_width;
		ls_video_params.params.height = sensor_params[USE_SENSOR].sensor_height;
		ls_video_params.jpg_width = life_snap_param.width;
		ls_video_params.jpg_height = life_snap_param.height;
		ls_video_params.need_scaleup = 1;
	}
	ls_video_params.jpg_qlevel = life_snap_param.jpeg_qlevel * 10;
	ls_video_params.params.stream_id = MAIN_STREAM_ID;
	ls_video_params.params.type = VIDEO_NV12;
	ls_video_params.params.fps = 5;
	ls_video_params.params.gop = 5;
	ls_video_params.params.use_static_addr = 1;
	ls_snapshot_ctx = mm_module_open(&video_module);
	if (ls_snapshot_ctx) {
		mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_SET_PARAMS, (int) & (ls_video_params.params));
		mm_module_ctrl(ls_snapshot_ctx, MM_CMD_SET_QUEUE_LEN, 2);//Default 30
		mm_module_ctrl(ls_snapshot_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		AI_GLASS_ERR("video open fail\n\r");
		ret = -1;
		goto endoflifesnapshot;
	}

	ls_siso_snapshot_filesaver = siso_create();
	if (ls_siso_snapshot_filesaver) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(ls_siso_snapshot_filesaver, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(ls_siso_snapshot_filesaver, MMIC_CMD_ADD_INPUT, (uint32_t)ls_snapshot_ctx, 0);
		siso_ctrl(ls_siso_snapshot_filesaver, MMIC_CMD_ADD_OUTPUT, (uint32_t)ls_filesaver_ctx, 0);
		siso_ctrl(ls_siso_snapshot_filesaver, MMIC_CMD_SET_TASKPRIORITY, LIFE_SNAP_PRIORITY, 0);
		siso_start(ls_siso_snapshot_filesaver);
	} else {
		AI_GLASS_ERR("siso_array_filesaver open fail\n\r");
		ret = -1;
		goto endoflifesnapshot;
	}
	mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_APPLY, ls_video_params.params.stream_id);	// start channel 0
	is_file_saved = LIFESNAP_START;
	return ret;
endoflifesnapshot:
	lifetime_snapshot_deinitialize();
	return ret;
}

// Todo: Use semapshore for the process
int lifetime_snapshot_take(const char *file_name)
{
	if (is_file_saved == LIFESNAP_START) {
		AI_GLASS_INFO("================life_snapshot_take==========================\r\n");
		AI_GLASS_INFO("Sanpshot start\r\n");
		snprintf(snapshot_name, MAXIMUM_FILE_SIZE, "%s", file_name);
		AI_GLASS_MSG("life_snapshot_take %s\r\n", snapshot_name);
		mm_module_ctrl(ls_filesaver_ctx, CMD_FILESAVER_SET_SAVE_FILE_PATH, (int)snapshot_name);
		is_file_saved = LIFESNAP_TAKE;
		mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_YUV, 1); // one shot
		while (is_file_saved == LIFESNAP_TAKE) {
			vTaskDelay(1);
		}
		AI_GLASS_INFO("Life snapshot done\r\n");
		return 0;
	}
	return -1;
}

int lifetime_snapshot_deinitialize(void)
{
	if (is_file_saved != LIFESNAP_TAKE) {
		//Pause Linker
		if (ls_siso_snapshot_filesaver) {
			siso_pause(ls_siso_snapshot_filesaver);
		}

		//Stop module
		if (ls_snapshot_ctx) {
			mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_STREAM_STOP, 0);
		}

		//Delete linker
		if (ls_siso_snapshot_filesaver) {
			siso_delete(ls_siso_snapshot_filesaver);
			ls_siso_snapshot_filesaver = NULL;
		}

		//Close module
		if (ls_snapshot_ctx) {
			ls_snapshot_ctx = mm_module_close(ls_snapshot_ctx);
		}
		if (ls_filesaver_ctx) {
			ls_filesaver_ctx = mm_module_close(ls_filesaver_ctx);
		}
		is_file_saved = LIFESNAP_IDLE;
	}
	return 0;
}