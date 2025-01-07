#include "platform_opts.h"
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_miso.h"

#include "module_video.h"
#include "module_audio.h"
#include "module_i2s.h"
#include "module_aac.h"
#include "module_mp4.h"
#include "log_service.h"
#include "sensor.h"
#include "isp_ctrl_api.h"
#include "gyrosensor_api.h"
#include "ai_glass_media.h"
#include "media_filesystem.h"

#define I2S_INTERFACE   0
#define AUDIO_INTERFACE 1

#define ENABLE_GET_GSENSOR_INFO 1
#define AUDIO_SAMPLE_RATE   16000 // 48000
#define AUDIO_SRC           I2S_INTERFACE //AUDIO_INTERFACE
#define AUDIO_I2S_ROLE      I2S_SLAVE //I2S_MASTER

//Modules
//lr means lifetime recording
static mm_context_t *lr_video_ctx  = NULL;
static mm_context_t *lr_audio_ctx = NULL;
static mm_context_t *lr_aac_ctx = NULL;
static mm_context_t *lr_mp4_ctx = NULL;

//Linkers
static mm_siso_t *lr_siso_audio_aac = NULL;
static mm_miso_t *lr_miso_video_aac_mp4 = NULL;

static video_params_t lr_video_params = {
	.stream_id = MAIN_STREAM_ID,
	.use_static_addr = 1
};

#if AUDIO_SRC==AUDIO_INTERFACE
static audio_params_t audio_params;
static audio_sr audio_samplerate2index(int samplerate)
{
	switch (rate) {
	case 8000:
		return ASR_8KHZ;
	case 16000:
		return ASR_16KHZ;
	case 32000:
		return ASR_32KHZ;
	case 44100:
		return ASR_44p1KHZ;
	case 48000:
		return ASR_48KHZ;
	case 88200:
		return ASR_88p2KHZ;
	case 96000:
		return ASR_96KHZ;
	default:
		printf("Invalid audio samplerate %d for audio set to default value ASR_16KHZ\n\r", samplerate);
		return ASR_16KHZ;
	}
}
#elif AUDIO_SRC==I2S_INTERFACE
static i2s_params_t i2s_params;
static int i2s_samplerate2index(int samplerate)
{
	switch (samplerate) {
	case 8000:
		return SR_8KHZ;
	case 16000:
		return SR_16KHZ;
	case 32000:
		return SR_32KHZ;
	case 44100:
		return SR_44p1KHZ;
	case 48000:
		return SR_48KHZ;
	case 88200:
		return SR_88p2KHZ;
	case 96000:
		return SR_96KHZ;
	default:
		printf("Invalid i2s samplerate %d for i2s set to default value SR_16KHZ\n\r", samplerate);
		return SR_16KHZ;
	}
}
#endif

static aac_params_t aac_params = {
	.sample_rate = AUDIO_SAMPLE_RATE,
	.channel = 1,
	.trans_type = AAC_TYPE_ADTS,
	.object_type = AAC_AOT_LC,
	.bitrate = 32000,

	.mem_total_size = 10 * 1024,
	.mem_block_size = 128,
	.mem_frame_size = 1024
};

static mp4_params_t lr_mp4_params = {
	.sample_rate = AUDIO_SAMPLE_RATE,
	.channel     = 1,

	.record_length = 30, //seconds
	.record_type = STORAGE_ALL,
	.record_file_num = 1,
	.record_file_name = "AmebaPro_recording",
	.fatfs_buf_size = 224 * 1024, /* 32kb multiple */
	.append_header = 0,
};

static char life_recording_name[128] = {0};

static const char *example = "ai_glass_lifetime_recording";

#if ENABLE_GET_GSENSOR_INFO
#define GSENSOR_RECORD_FAST 1
typedef struct gyro_data_node_s {
	gyro_data_t data;
	struct gyro_data_node_s *next;
} gyro_data_node_t;

typedef struct gyro_data_list_s {
	gyro_data_node_t *head;
	gyro_data_node_t *tail;
	gyro_data_node_t *read_ptr;
} gyro_data_list_t;

static TimerHandle_t gsensor_timer = NULL;

static gyro_data_list_t gyro_list = {NULL, NULL};

static void append_gyro_data(gyro_data_list_t *list, gyro_data_t new_data)
{
	gyro_data_node_t *new_node = (gyro_data_node_t *)malloc(sizeof(gyro_data_node_t));
	if (new_node == NULL) {
		printf("Memory allocation failed!\n");
		return;
	}

	new_node->data = new_data;
	new_node->next = NULL;

	if (list->head == NULL) {
		list->head = new_node;
		list->tail = new_node;
	} else {
		list->tail->next = new_node;
		list->tail = new_node;
	}
}

static void delete_whole_list(gyro_data_list_t *list)
{
	gyro_data_node_t *current = list->head;
	gyro_data_node_t *temp;

	while (current != NULL) {
		temp = current;
		current = current->next;
		free(temp);
	}

	list->head = NULL;
	list->tail = NULL;
	list->read_ptr = NULL;
}

static void sort_gyro_data_list_by_timestamp(gyro_data_list_t *list)
{
	if (list->head == NULL || list->head->next == NULL) {
		return;
	}

	gyro_data_node_t *sorted = NULL;
	gyro_data_node_t *current = list->head;

	while (current != NULL) {
		gyro_data_node_t *next = current->next;
		if (sorted == NULL || sorted->data.timestamp >= current->data.timestamp) {
			current->next = sorted;
			sorted = current;
		} else {
			gyro_data_node_t *sorted_current = sorted;
			while (sorted_current->next != NULL && sorted_current->next->data.timestamp < current->data.timestamp) {
				sorted_current = sorted_current->next;
			}
			current->next = sorted_current->next;
			sorted_current->next = current;
		}
		current = next;
	}

	list->head = sorted;
	list->tail = sorted;
	while (list->tail->next != NULL) {
		list->tail = list->tail->next;
	}
}

static void insert_gyro_data_sorted(gyro_data_list_t *list, gyro_data_t new_data)
{
	gyro_data_node_t *new_node = (gyro_data_node_t *)malloc(sizeof(gyro_data_node_t));
	if (new_node == NULL) {
		return;
	}
	new_node->data = new_data;
	new_node->next = NULL;

	if (list->head == NULL || list->head->data.timestamp > new_data.timestamp) {
		new_node->next = list->head;
		list->head = new_node;

		if (list->tail == NULL) {
			list->tail = new_node;
		}
	} else {
		gyro_data_node_t *current = list->head;
		while (current->next != NULL && current->next->data.timestamp <= new_data.timestamp) {
			current = current->next;
		}

		new_node->next = current->next;
		current->next = new_node;

		if (new_node->next == NULL) {
			list->tail = new_node;
		}
	}
}

// align the list all node with target timestamp
static void align_gyro_data_by_timestamp(gyro_data_list_t *list, uint32_t target_timestamp)
{
	gyro_data_node_t *current = list->head;
	gyro_data_node_t *temp;

	if (current == NULL) {
		return;
	}

	if (target_timestamp < current->data.timestamp) {
		uint32_t gap = current->data.timestamp - target_timestamp;
		for (uint32_t i = 0; i < gap; i++) {
			gyro_data_t new_data;
			new_data.timestamp = target_timestamp + i;

			for (int j = 0; j < 3; j++) {
#if !IGN_ACC_DATA
				new_data.g[j] = 0.0f;
#endif
				//new_data.g_raw[j] = 0;
				new_data.dps[j] = 0.0f;
				//new_data.dps_raw[j] = 0;
			}

			insert_gyro_data_sorted(list, new_data);
		}
	} else {
		current = list->head;
		while (current != NULL && current->data.timestamp < target_timestamp) {
			temp = current;
			current = current->next;
			free(temp);
		}

		list->head = current;

		if (list->head == NULL) {
			list->tail = NULL;
		}
	}
}


static gyro_data_node_t *read_gyro_data(gyro_data_list_t *list)
{
	gyro_data_node_t *rd_ptr = NULL;
	if (list->read_ptr == NULL) {
		list->read_ptr = list->head;
		rd_ptr = list->read_ptr;
	} else if (list->read_ptr != list->tail) {
		rd_ptr = list->read_ptr->next;
		list->read_ptr = list->read_ptr->next;
	}
	return rd_ptr;
}

static void reset_rdptr_gyro_data(gyro_data_list_t *list)
{
	list->read_ptr = NULL;
}

static void save_gyro_data_to_file(gyro_data_list_t *list, const char *filename)
{
	FILE *file = extdisk_fopen(filename, "wb");
	if (file == NULL) {
		printf("Failed to open file for writing.\n");
		return;
	}

	static const char *csv_header_data =
		"GYROFLOW IMU LOG\n"
		"version,1.3\n"
		"id,amb82\n"
		"orientation,XYz\n"
		"note,development_test\n"
		"fwversion,FIRMWARE_0.1.0\n"
		"timestamp,0\n"
		"vendor,amb82cam\n"
		"videofilename,video.mp4\n"
		"lensprofile,amb82\n"
		"lens_info,wide\n"
		"frame_readout_time,0.0\n"
		"frame_readout_direction,0\n"
		"tscale,0.001\n"
		"gscale,0.0174533\n"
		"ascale,10.0\n"
		"t,gx,gy,gz,ax,ay,az\n";

	extdisk_fwrite((const char *) csv_header_data, strlen(csv_header_data), 1, file);

	gyro_data_node_t *current = list->head;
	char gyro_data_string[128] = {0};
	while (current != NULL) {
#if !IGN_ACC_DATA
		int gyro_str_size = snprintf(gyro_data_string, sizeof(gyro_data_string) - 1, "%lu, %f, %f, %f, %f %f %f\n", current->data.timestamp, current->data.dps[0],
									 current->data.dps[1], current->data.dps[2], current->data.g[0], current->data.g[1], current->data.g[2]);
#else
		int gyro_str_size = snprintf(gyro_data_string, sizeof(gyro_data_string) - 1, "%lu, %f, %f, %f\n", current->data.timestamp, current->data.dps[0],
									 current->data.dps[1], current->data.dps[2]);
#endif
		if (gyro_str_size > sizeof(gyro_data_string) - 1) {
			printf("gyro_str_size = %lu, sizeof(gyro_data_string) = %lu\r\n", gyro_str_size, sizeof(gyro_data_string) - 1);
#if !IGN_ACC_DATA
			printf(gyro_data_string, sizeof(gyro_data_string) - 1, "%lu, %f, %f, %f, %f %f %f\n", current->data.timestamp, current->data.dps[0],
				   current->data.dps[1], current->data.dps[2], current->data.g[0], current->data.g[1], current->data.g[2]);
#else
			printf(gyro_data_string, sizeof(gyro_data_string) - 1, "%lu, %f, %f, %f\n", current->data.timestamp, current->data.dps[0], current->data.dps[1],
				   current->data.dps[2]);
#endif
		} else {
			extdisk_fwrite((const char *) gyro_data_string, gyro_str_size, 1, file);
		}
		current = current->next;
	}

	extdisk_fclose(file);
	printf("Data saved to file: %s\r\n", filename);
}

static void print_gyro_data_list(gyro_data_list_t *list)
{
	gyro_data_node_t *head = list->head;
	gyro_data_node_t *tail = list->tail;
	printf("head Timestamp: %lu\n", head->data.timestamp);
	printf("tail Timestamp: %lu\n", tail->data.timestamp);
}

#define GYRO_SAVE_IDLE          0x00
#define GYRO_SAVE_START         0x01
#define GYRO_SAVE_STOP          0x02
#define GYRO_SAVE_SET_START     0x10
#define GYRO_SAVE_SET_STOP      0x20

static int exdisk_gyro_status = GYRO_SAVE_IDLE;
static void save_gyro_to_exdisk_thread(void *param)
{
	FILE *file = NULL;
	printf("======================save_gyro_to_exdisk_thread=======================\r\n");
	if (!(exdisk_gyro_status & GYRO_SAVE_SET_START)) {
		printf("exdisk_gyro_status 0x%08x is invalid.\r\n", exdisk_gyro_status);
		goto endofsave;
	}
	exdisk_gyro_status = GYRO_SAVE_START;

	char life_gryo_csv[128] = {0};
	snprintf(life_gryo_csv, sizeof(life_gryo_csv), "%s.csv", life_recording_name);

	file = extdisk_fopen((const char *)life_gryo_csv, "w+");
	if (file == NULL) {
		printf("Failed to open file for writing.\r\n");
		goto endofsave;
	}

	static const char *csv_header_data =
		"GYROFLOW IMU LOG\n"
		"version,1.3\n"
		"id,amb82\n"
		"orientation,XYz\n"
		"note,development_test\n"
		"fwversion,FIRMWARE_0.1.0\n"
		"timestamp,0\n"
		"vendor,amb82cam\n"
		"videofilename,video.mp4\n"
		"lensprofile,amb82\n"
		"lens_info,wide\n"
		"frame_readout_time,0.0\n"
		"frame_readout_direction,0\n"
		"tscale,0.001\n"
		"gscale,0.0174533\n"
		"ascale,10.0\n"
		"t,gx,gy,gz,ax,ay,az\n";

	extdisk_fwrite((const char *) csv_header_data, strlen(csv_header_data), 1, file);

	gyro_data_t g_data;
	char gyro_data_string[128] = {0};
	mp4_ctx_t *mp4_module_ctx = (mp4_ctx_t *)(lr_mp4_ctx->priv);
	mp4_context *mp4_muxer = (mp4_context *)(mp4_module_ctx->mp4_muxer);
	while (!mp4_muxer->video_appear_first) {
		if (exdisk_gyro_status & GYRO_SAVE_SET_STOP) {
			goto endofsave;
		}
		vTaskDelay(1);
	}

	uint32_t video_first_ts = mp4_muxer->video_timestamp_first;
	printf("video_first_ts = %u\r\n", video_first_ts);

	while (1) {
		gyro_data_node_t *rd_tpr = read_gyro_data(&gyro_list);
		if (rd_tpr != NULL && rd_tpr->data.timestamp >= video_first_ts) {
			memcpy(&g_data, &(rd_tpr->data), sizeof(gyro_data_t));
#if !IGN_ACC_DATA
			int ret = snprintf(gyro_data_string, sizeof(gyro_data_string), "%lu, %f, %f, %f, %f %f %f\n", g_data.timestamp, g_data.dps[0], g_data.dps[1],
							   g_data.dps[2], g_data.g[0], g_data.g[1], g_data.g[2]);
#else
			int ret = snprintf(gyro_data_string, sizeof(gyro_data_string), "%lu, %f, %f, %f\n", g_data.timestamp, g_data.dps[0], g_data.dps[1],
							   g_data.dps[2]);
#endif
			if (ret > sizeof(gyro_data_string) - 1 || ret < 0) {
				printf("ret = %lu, sizeof(gyro_data_string) = %lu\r\n", ret, sizeof(gyro_data_string) - 1);
#if !IGN_ACC_DATA
				printf("%lu, %f, %f, %f, %f %f %f\n", g_data.timestamp, g_data.dps[0], g_data.dps[1], g_data.dps[2],
					   g_data.g[0], g_data.g[1], g_data.g[2]);
#else
				printf("%lu, %f, %f, %f\n", g_data.timestamp, g_data.dps[0], g_data.dps[1], g_data.dps[2]);
#endif
			} else {
				extdisk_fwrite((const char *) gyro_data_string, ret, 1, file);
			}
		} else if (exdisk_gyro_status & GYRO_SAVE_SET_STOP) {
			break;
		} else {
			//printf("no gyro data is get 0x%08x\r\n", exdisk_gyro_status);
			vTaskDelay(1);
		}
	}

endofsave:
	if (file != NULL) {
		extdisk_fclose(file);
	}
	printf("Data saved to file: %s\r\n", life_gryo_csv);
	exdisk_gyro_status = GYRO_SAVE_STOP;
	vTaskDelete(NULL);
}

static int start_gyro_to_exdisk_process(void)
{
	if (exdisk_gyro_status == GYRO_SAVE_IDLE) {
		exdisk_gyro_status |= GYRO_SAVE_SET_START;
		if (xTaskCreate(save_gyro_to_exdisk_thread, ((const char *)"save_gyro_to_exdisk_thread"), 4096, NULL, tskIDLE_PRIORITY + 4, NULL) != pdPASS) {
			printf("\n\r%s xTaskCreate(save_gyro_to_exdisk_thread) failed\n\r", __FUNCTION__);
			exdisk_gyro_status = GYRO_SAVE_IDLE;
			return -1;
		}
		printf("exdisk_gyro_status is idle\n\r");
		return 0;
	}
	printf("exdisk_gyro_status is not idle\n\r");
	return -1;
}

// G-sensor example => need to update
static gyro_data_t gdata[100] = {0};
static void gyro_read_gsensor(void *parm)
{
	uint32_t cur_timstamp = mm_read_mediatime_ms();
	int read_cnt = gyroscope_fifo_read(gdata, 100);
	uint32_t final_timstamp = 0;
	if (read_cnt > 0) {
		final_timstamp = gdata[read_cnt - 1].timestamp;
	}

	for (int i = 0; i < read_cnt; i++) {
		gdata[i].timestamp = cur_timstamp - (final_timstamp - gdata[i].timestamp);
		append_gyro_data(&gyro_list, gdata[i]);
	}

	printf("Read end G-sensor %lu, read_cnt = %d\r\n", mm_read_mediatime_ms(), read_cnt);
}

static void vGensorTimeCallback(TimerHandle_t xTimer)
{
	gyro_read_gsensor(NULL);
	if (gsensor_timer != NULL) {
		if (xTimerStart(gsensor_timer, 0) != pdPASS) {
			printf("Reload G-sensor read timer\r\n");
		}
	}
}

int lr_gyro_deinit(void)
{
	if (gsensor_timer != NULL) {
		xTimerStop(gsensor_timer, 0);
		if (xTimerDelete(gsensor_timer, 0) == pdPASS) {
			gsensor_timer = NULL;
		}
	}
	if (gyroscope_is_inited()) {
		// Todo: deinit gyroscope
	}
	return 0;
}

int lr_gyro_init(void)
{
	if (gyroscope_fifo_init() != 0) {
		return -1;
	}
	delete_whole_list(&gyro_list);
	gyroscope_reset_fifo();
	if (gsensor_timer == NULL) {
		gsensor_timer = xTimerCreate("gsensor_timer", 20, pdFALSE, &gsensor_timer, vGensorTimeCallback);
	}

	if (gsensor_timer != NULL) {
		if (xTimerStart(gsensor_timer, 0) != pdPASS) {
			printf("Reload G-sensor read timer\r\n");
			return -1;
		}
	} else {
		printf("gsensor_timer create failed\r\n");
		return -1;
	}

	printf("lr_gyro_init success\r\n");
	return 0;
}
#endif

static int get_mp4_video_timestamp(void)
{
	if (lr_mp4_ctx != NULL) {
#if ENABLE_GET_GSENSOR_INFO
		mp4_ctx_t *mp4_module_ctx = (mp4_ctx_t *)(lr_mp4_ctx->priv);
		mp4_context *mp4_muxer = (mp4_context *)(mp4_module_ctx->mp4_muxer);
#if !GSENSOR_RECORD_FAST
		align_gyro_data_by_timestamp(&gyro_list, mp4_muxer->video_timestamp_first);
#endif
		printf("mp4_muxer->video_timestamp_first = %u\r\n", mp4_muxer->video_timestamp_first);
		printf("mp4_muxer->video_timestamp_end = %u\r\n",
			   mp4_muxer->video_timestamp_first + (mp4_muxer->video_timestamp_buffer[mp4_muxer->root.video_len - 1] - mp4_muxer->video_timestamp_buffer[0]) /
			   (mp4_muxer->video_clock_rate / configTICK_RATE_HZ));
#endif
	}
	return 0;
}

MP4State current_state = STATE_IDLE;

extern void mp4_send_response_callback(void *parm);
static int lr_mp4_end_cb(void *parm)
{
	printf("Record end\r\n");
#if GSENSOR_RECORD_FAST
	if (exdisk_gyro_status != GYRO_SAVE_STOP) {
		exdisk_gyro_status |= GYRO_SAVE_SET_STOP;
		while (1) {
			if (exdisk_gyro_status == GYRO_SAVE_STOP) {
				break;
			}
			vTaskDelay(1);
		}
		exdisk_gyro_status = GYRO_SAVE_IDLE;
		delete_whole_list(&gyro_list);
	}
	current_state = STATE_END_RECORDING;
#else
	current_state = STATE_END_RECORDING;
	char life_gryo_csv[128] = {0};
	snprintf(life_gryo_csv, sizeof(life_gryo_csv), "%s.csv", life_recording_name);
	save_gyro_data_to_file(&gyro_list, life_gryo_csv);
	delete_whole_list(&gyro_list);
#endif
	return 0;
}

static int lr_mp4_stop_cb(void *parm)
{
	printf("Record stop\r\n");
	current_state = STATE_IDLE;
#if ENABLE_GET_GSENSOR_INFO
	lr_gyro_deinit();
	print_gyro_data_list(&gyro_list);
	get_mp4_video_timestamp();
#endif
	return 0;
}

static int lr_mp4_error_cb(void *parm)
{
	printf("Lifetime recording error\r\n");
	current_state = STATE_ERROR;
#if ENABLE_GET_GSENSOR_INFO
	lr_gyro_deinit();
#endif
	return 0;
}

void lifetime_recording_initialize(void)
{
	printf("================LifeTime Record start==========================\r\n");
	char *cur_time_str = (char *)media_filesystem_get_current_time_string();
	extdisk_generate_unique_filename("liferecord_", cur_time_str, ".mp4", life_recording_name, 128);
	free(cur_time_str);

	ai_glass_record_param_t *ai_record_param = NULL;
#if ENABLE_GET_GSENSOR_INFO
	// Initial G-sensor
	if (lr_gyro_init()) {
		goto lifetime_recording_initialize_fail;
	}

#if GSENSOR_RECORD_FAST
	if (start_gyro_to_exdisk_process()) {
		goto lifetime_recording_initialize_fail;
	}
#endif
#endif

	ai_record_param = (ai_glass_record_param_t *) malloc(sizeof(ai_glass_record_param_t));
	memset(ai_record_param, 0x00, sizeof(ai_glass_record_param_t));
	media_get_record_params(ai_record_param);

	//Update mp4 params
	lr_mp4_params.fps = ai_record_param->fps;
	lr_mp4_params.gop = ai_record_param->gop;
	lr_mp4_params.width = ai_record_param->width;
	lr_mp4_params.height = ai_record_param->height;
	lr_mp4_params.record_length = ai_record_param->record_length;

	//Update video channel params
	lr_video_params.type = ai_record_param->type;
	lr_video_params.bps = ai_record_param->bps;
	lr_video_params.fps = ai_record_param->fps;
	lr_video_params.width = ai_record_param->width;
	lr_video_params.height = ai_record_param->height;
	lr_video_params.rc_mode = ai_record_param->rc_mode;

#if AUDIO_SRC==AUDIO_INTERFACE
	lr_audio_ctx = mm_module_open(&audio_module);
	memcpy((void *)&audio_params, (void *)&default_audio_params, sizeof(audio_params_t));
	if (lr_audio_ctx) {
		mm_module_ctrl(lr_audio_ctx, CMD_AUDIO_GET_PARAMS, (int)&audio_params);
		audio_params.sample_rate = audio_samplerate2index(AUDIO_SAMPLE_RATE);
		mm_module_ctrl(lr_audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
		mm_module_ctrl(lr_audio_ctx, MM_CMD_SET_QUEUE_LEN, 6); //queue size can be smaller 160ms
		mm_module_ctrl(lr_audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(lr_audio_ctx, CMD_VIDEO_META_CB, (int)video_meta_cb);
	} else {
		printf("audio open fail\n\r");
		goto lifetime_recording_initialize_fail;
	}
#elif AUDIO_SRC==I2S_INTERFACE
	lr_audio_ctx = mm_module_open(&i2s_module);
	if (lr_audio_ctx) {
		mm_module_ctrl(lr_audio_ctx, CMD_I2S_GET_PARAMS, (int)&i2s_params);
		i2s_params.sample_rate = i2s_samplerate2index(AUDIO_SAMPLE_RATE);
		i2s_params.i2s_direction = I2S_RX_ONLY;
		i2s_params.i2s_role = AUDIO_I2S_ROLE;
		//i2s_params.pin_group_num = 0;
		mm_module_ctrl(lr_audio_ctx, CMD_I2S_SET_PARAMS, (int)&i2s_params);
		mm_module_ctrl(lr_audio_ctx, MM_CMD_SET_QUEUE_LEN, 6); //queue size can be smaller 160ms
		mm_module_ctrl(lr_audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
	} else {
		printf("i2s open fail\n\r");
		goto lifetime_recording_initialize_fail;
	}
#endif

	lr_aac_ctx = mm_module_open(&aac_module);
	if (lr_aac_ctx) {
		mm_module_ctrl(lr_aac_ctx, CMD_AAC_SET_PARAMS, (int)&aac_params);
		mm_module_ctrl(lr_aac_ctx, MM_CMD_SET_QUEUE_LEN, 30);
		mm_module_ctrl(lr_aac_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(lr_aac_ctx, CMD_AAC_INIT_MEM_POOL, 0);
		mm_module_ctrl(lr_aac_ctx, CMD_AAC_APPLY, 0);
	} else {
		printf("AAC open fail\n\r");
		goto lifetime_recording_initialize_fail;
	}

	lr_mp4_ctx = mm_module_open(&mp4_module);
	lr_mp4_params.mp4_audio_format = AUDIO_AAC;

	if (lr_mp4_ctx) {
		mm_module_ctrl(lr_mp4_ctx, CMD_MP4_SET_PARAMS, (int)&lr_mp4_params);
		mm_module_ctrl(lr_mp4_ctx, CMD_MP4_LOOP_MODE, 0);
		mm_module_ctrl(lr_mp4_ctx, CMD_MP4_SET_STOP_CB, (int)lr_mp4_stop_cb);
		mm_module_ctrl(lr_mp4_ctx, CMD_MP4_SET_END_CB, (int)lr_mp4_end_cb);
		mm_module_ctrl(lr_mp4_ctx, CMD_MP4_SET_ERROR_CB, (int)lr_mp4_error_cb);
		mm_module_ctrl(lr_mp4_ctx, CMD_MP4_SET_RECORD_FILE_NAME, (int)life_recording_name);
		//char *extdisk_tag = extdisk_get_filesystem_tag_name();
		//if (extdisk_tag) {
		//mm_module_ctrl(lr_mp4_ctx, CMD_MP4_SET_VFS_EXDISK_TAG, (int)extdisk_tag);
		//free(extdisk_tag);
		//}
		mm_module_ctrl(lr_mp4_ctx, CMD_MP4_START, lr_mp4_params.record_file_num);
	} else {
		printf("MP4 open fail\n\r");
		goto lifetime_recording_initialize_fail;
	}

	printf("MP4 opened\n\r");

	lr_video_ctx = mm_module_open(&video_module);
	if (lr_video_ctx) {
		mm_module_ctrl(lr_video_ctx, CMD_VIDEO_SET_PARAMS, (int)&lr_video_params);
		mm_module_ctrl(lr_video_ctx, MM_CMD_SET_QUEUE_LEN, lr_video_params.fps * 10); //Add the queue buffer to avoid to lost data.
		mm_module_ctrl(lr_video_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		printf("video open fail\n\r");
		goto lifetime_recording_initialize_fail;
	}

	lr_siso_audio_aac = siso_create();
	if (lr_siso_audio_aac) {
		siso_ctrl(lr_siso_audio_aac, MMIC_CMD_ADD_INPUT, (uint32_t)lr_audio_ctx, 0);
		siso_ctrl(lr_siso_audio_aac, MMIC_CMD_ADD_OUTPUT, (uint32_t)lr_aac_ctx, 0);
		siso_ctrl(lr_siso_audio_aac, MMIC_CMD_SET_TASKPRIORITY, 4, 0);
		siso_ctrl(lr_siso_audio_aac, MMIC_CMD_SET_STACKSIZE, 44 * 1024, 0);
		siso_start(lr_siso_audio_aac);
	} else {
		printf("lr siso audio aac open fail\n\r");
		goto lifetime_recording_initialize_fail;
	}

	lr_miso_video_aac_mp4 = miso_create();
	if (lr_miso_video_aac_mp4) {
		miso_ctrl(lr_miso_video_aac_mp4, MMIC_CMD_ADD_INPUT0, (uint32_t)lr_video_ctx, 0);
		miso_ctrl(lr_miso_video_aac_mp4, MMIC_CMD_ADD_INPUT1, (uint32_t)lr_aac_ctx, 0);
		miso_ctrl(lr_miso_video_aac_mp4, MMIC_CMD_ADD_OUTPUT0, (uint32_t)lr_mp4_ctx, 0);
		miso_ctrl(lr_miso_video_aac_mp4, MMIC_CMD_SET_TASKPRIORITY, 4, 0);
		miso_start(lr_miso_video_aac_mp4);
	} else {
		printf("miso open fail for video recording\n\r");
		goto lifetime_recording_initialize_fail;
	}

	printf("miso(videochn0_aac_mp4) started\n\r");

	current_state = STATE_RECORDING;
#if AUDIO_SRC==AUDIO_INTERFACE
	mm_module_ctrl(lr_audio_ctx, CMD_AUDIO_APPLY, 0);
#elif AUDIO_SRC==I2S_INTERFACE
	mm_module_ctrl(lr_audio_ctx, CMD_I2S_APPLY, 0);
#endif
	mm_module_ctrl(lr_video_ctx, CMD_VIDEO_APPLY, lr_video_params.stream_id);

	if (ai_record_param) {
		free(ai_record_param);
	}
	return;
lifetime_recording_initialize_fail:
	if (ai_record_param) {
		free(ai_record_param);
	}
	return;
}

void lifetime_recording_deinitialize(void)
{
	//Pause Linker
	//Pause individual audio
	siso_pause(lr_siso_audio_aac);

	//Pause MP4 recording / RTSP Stream
	miso_pause(lr_miso_video_aac_mp4, MM_OUTPUT0);

	//Stop module
	if (lr_video_ctx != NULL) {
		mm_module_ctrl(lr_video_ctx, CMD_VIDEO_STREAM_STOP, lr_video_params.stream_id);
	}
#if AUDIO_SRC==AUDIO_INTERFACE
	if (lr_audio_ctx != NULL) {
		mm_module_ctrl(lr_audio_ctx, CMD_AUDIO_SET_TRX, 0);
	}
#else
	if (lr_audio_ctx != NULL) {
		mm_module_ctrl(lr_audio_ctx, CMD_I2S_SET_TRX, 0);
	}
#endif
	if (lr_aac_ctx != NULL) {
		mm_module_ctrl(lr_aac_ctx, CMD_AAC_STOP, 0);
	}
	if (lr_mp4_ctx != NULL) {
		mm_module_ctrl(lr_mp4_ctx, CMD_MP4_STOP_IMMEDIATELY, 0);
	}
	//Delete linker
	siso_delete(lr_siso_audio_aac);
	lr_siso_audio_aac = NULL;
	miso_delete(lr_miso_video_aac_mp4);
	lr_miso_video_aac_mp4 = NULL;

	//Close module
	mm_module_close(lr_audio_ctx);
	lr_audio_ctx = NULL;
	mm_module_close(lr_aac_ctx);
	lr_aac_ctx = NULL;
	mm_module_close(lr_mp4_ctx);
	lr_mp4_ctx = NULL;
	mm_module_close(lr_video_ctx);
	lr_video_ctx = NULL;

	current_state = STATE_IDLE;
}

static void fUC(void *arg)
{
	static uint32_t user_cmd = 0;

	if (!strcmp(arg, "TD")) {
		lifetime_recording_deinitialize();
		printf("deinit %s\r\n", example);
	} else if (!strcmp(arg, "TSR")) {
		printf("reinit %s\r\n", example);
		lifetime_recording_initialize();
		// sys_reset();
	} else {
		printf("invalid cmd");
	}

	printf("user command 0x%lx\r\n", user_cmd);
}

//audio_pause: 0- pause aac, 1- pause opusc, 2- pause both opusc and aac
void lr_audio_pause(int ai_glass_audio_type)
{
	if (ai_glass_audio_type == 0) {
		siso_pause(lr_siso_audio_aac);
	}
}

void lr_video_pause(void)
{
	miso_pause(lr_miso_video_aac_mp4, MM_OUTPUT0);
}

void lr_video_resume(void)
{
	miso_resume(lr_miso_video_aac_mp4);
}

void lr_audio_resume(void)
{
	siso_resume(lr_siso_audio_aac);
}

void lr_audio_stop(void)
{
	mm_module_ctrl(lr_audio_ctx, CMD_AUDIO_SET_TRX, 0);
}

void lr_audio_start(void)
{
	mm_module_ctrl(lr_audio_ctx, CMD_AUDIO_SET_TRX, 1);
}

static void APAAC(void *arg)
{
	printf("audio_pause_aac %s\r\n", example);
	lr_audio_pause(0);
}

static void AR(void *arg)
{
	printf("audio_resume %s\r\n", example);
	lr_audio_resume();
}

static void ASTOP(void *arg)
{
	printf("audio_stop %s\r\n", example);
	lr_audio_stop();
}

static void ASTART(void *arg)
{
	printf("audio_start %s\r\n", example);
	lr_audio_start();
}

static void VP(void *arg)
{
	lr_video_pause();
}

static void VR(void *arg)
{
	lr_video_resume();
}

static log_item_t userctrl_items[] = {
	{"UC", fUC, },
	{"APAAC", APAAC, },
	{"AR", AR,},
	{"ASTART", ASTART,},
	{"ASTOP", ASTOP,},
	{"VP", VP,},
	{"VR", VR,},
};

void ai_glass_atcmd_lr_userctrl_init(void)
{
	log_service_add_table(userctrl_items, sizeof(userctrl_items) / sizeof(userctrl_items[0]));
}

