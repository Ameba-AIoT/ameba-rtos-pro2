#include "platform_opts.h"
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_miso.h"

#include "module_video.h"
#include "module_audio.h"
#include "module_i2s.h"
#include "module_opusc.h"
#include "module_mp4.h"
#include "log_service.h"
#include "sensor.h"
#include "isp_ctrl_api.h"
#include "gyrosensor_api.h"
#include "ai_glass_media.h"
#include "media_filesystem.h"
#include "ai_glass_dbg.h"
#include "gyrosensor/gyrosensor_data_box.h"

#define AUDIO_SRC               I2S_INTERFACE //I2S_INTERFACE //AUDIO_INTERFACE
#define AUDIO_I2S_ROLE          I2S_SLAVE //I2S_MASTER

#if OPUSC_UPSAMPLE_16K_TO_48K == 1

#define AUDIO_SAMPLE_RATE       48000
#define i2S_SAMPLE_RATE       	16000

#if AUDIO_SRC == AUDIO_INTERFACE

#define AUDIO_I_SAMPLE_RATE     16000

#endif

#else

#define AUDIO_SAMPLE_RATE       16000 

#endif

#if ENABLE_GET_GSENSOR_INFO
#define GSENSOR_RECORD_FAST 1
#define GYRO_USE_MODE 1   // 0: CSV mode, 1: SEI/MP4-mett mode
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
static int exdisk_gyro_status = GYRO_SAVE_IDLE;
static gyro_data_t gdata[100] = {0};
#endif

static char life_recording_name[128] = {0};
static const char *example = "ai_glass_lifetime_recording";

//Modules
//lr means lifetime recording
static mm_context_t *lr_video_ctx  = NULL;
static mm_context_t *lr_audio_ctx = NULL;
static mm_context_t *lr_opusc_ctx = NULL;
static mm_context_t *lr_mp4_ctx = NULL;

//Linkers
static mm_siso_t *lr_siso_audio_opusc = NULL;
static mm_miso_t *lr_miso_video_opusc_mp4 = NULL;

static video_params_t lr_video_params = {
	.stream_id = MAIN_STREAM_ID,
	.use_static_addr = 1
};

#if AUDIO_SRC==AUDIO_INTERFACE
static audio_params_t audio_params;
static audio_sr audio_samplerate2index(int samplerate)
{
	switch (samplerate) {
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
		AI_GLASS_ERR("Invalid audio samplerate %d for audio set to default value ASR_16KHZ\n\r", samplerate);
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
		AI_GLASS_ERR("Invalid i2s samplerate %d for i2s set to default value SR_16KHZ\n\r", samplerate);
		return SR_16KHZ;
	}
}
#endif

static opusc_params_t opusc_rtsp_params = {
	.sample_rate = AUDIO_SAMPLE_RATE,
	.channel = 1,
	.bit_length = 16,			//16 recommand
	.complexity = 5,			//0~10
	.bitrate = 25000,			//default 25000
	.use_framesize = 40,		//needs to the same or bigger than AUDIO_DMA_PAGE_SIZE/(sample_rate/1000)/2 but less than 60
	.enable_vbr = 1,
	.vbr_constraint = 0,
	.packetLossPercentage = 0,
	.opus_application = OPUS_APPLICATION_AUDIO

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

#if ENABLE_GET_GSENSOR_INFO
static void append_gyro_data(gyro_data_list_t *list, gyro_data_t new_data)
{
	gyro_data_node_t *new_node = (gyro_data_node_t *)malloc(sizeof(gyro_data_node_t));
	if (new_node == NULL) {
		AI_GLASS_ERR("Memory allocation failed!\r\n");
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

#if GYRO_USE_MODE == 1
// ============================================================================
// GYRO_USE_MODE == 1 : SEI / MP4 metadata('mett') track implementation.
// G-sensor samples are packed into IMU samples and staged into the MP4
// metadata ('mett') track via mp4_muxer_write_metadata().
//
// One metadata track sample = an array of ImuSample, little-endian, packed
// back-to-back, no padding, no header:
//   struct ImuSample { uint32_t timestamp; float gx, gy, gz; };  // 16 bytes
// ============================================================================
#define GYRO_FIFO_MAX       100   // must match the size of gdata[]
#define IMU_SAMPLE_SIZE     16
#define IMU_MAX_SAMPLES     (MAX_METADATA_SAMPLE_SIZE / IMU_SAMPLE_SIZE)
#define GYRO_ODR_HZ 800

#define IMU_PRE_ROLL_SAMPLES    (GYRO_ODR_HZ * 2)  // 2 seconds @ 800Hz
#define IMU_PRE_ROLL_BYTES      (IMU_PRE_ROLL_SAMPLES * IMU_SAMPLE_SIZE)

static uint8_t  imu_pre_roll[IMU_PRE_ROLL_BYTES];
static uint32_t imu_pre_roll_ts[IMU_PRE_ROLL_SAMPLES];   // ts of each sample
static uint32_t imu_pre_roll_cnt  = 0;                   // # stored (capped at roll size)
static uint32_t imu_pre_roll_head = 0;                   // oldest valid index
static uint32_t last_imu_ts = 0;

static uint8_t  imu_buf[IMU_MAX_SAMPLES * IMU_SAMPLE_SIZE];
static size_t   imu_len = 0;
static uint32_t imu_chunk_first_ts = 0;
static int      imu_prev_mett_ready = 0;

static int imu_mp4_mett_ready(void)
{
      mp4_context *mp4_muxer = NULL;
      if (lr_mp4_ctx != NULL) {
              mp4_ctx_t *mp4_module_ctx = (mp4_ctx_t *)(lr_mp4_ctx->priv);
              mp4_muxer = (mp4_context *)(mp4_module_ctx->mp4_muxer);
      }
      if (mp4_muxer == NULL || mp4_muxer->metadata_data == NULL) {
              return 0;
      }
      return (mp4_muxer->storage_state == STORAGE_WRITE_ALL ||
              mp4_muxer->storage_state == STORAGE_WRITE_VIDEO ||
              mp4_muxer->storage_state == STORAGE_WRITE_AUDIO);
}

/* Store one packed IMU sample into the pre-roll ring (overwrite oldest if full). */
static void imu_pre_roll_put(uint32_t ts, const uint8_t *sample)
{
      uint32_t idx = (imu_pre_roll_head + imu_pre_roll_cnt) % IMU_PRE_ROLL_SAMPLES;
      memcpy(imu_pre_roll + (size_t)idx * IMU_SAMPLE_SIZE, sample, IMU_SAMPLE_SIZE);
      imu_pre_roll_ts[idx] = ts;
      if (imu_pre_roll_cnt < IMU_PRE_ROLL_SAMPLES) {
              imu_pre_roll_cnt++;
      } else {
              imu_pre_roll_head = (imu_pre_roll_head + 1) % IMU_PRE_ROLL_SAMPLES;
      }
}

static void imu_pre_roll_flush(mp4_context *mp4_muxer)
{
      uint8_t imu_buf[IMU_MAX_SAMPLES * IMU_SAMPLE_SIZE];
      uint32_t flushed = 0;

      while (flushed < imu_pre_roll_cnt) {
              uint32_t chunk = imu_pre_roll_cnt - flushed;
              if (chunk > IMU_MAX_SAMPLES) {
                      chunk = IMU_MAX_SAMPLES;
              }
              uint32_t start_idx = (imu_pre_roll_head + flushed) % IMU_PRE_ROLL_SAMPLES;
              uint32_t first_part = IMU_PRE_ROLL_SAMPLES - start_idx;
              uint32_t n0 = (chunk < first_part) ? chunk : first_part;

              memcpy(imu_buf,
                     imu_pre_roll + (size_t)start_idx * IMU_SAMPLE_SIZE,
                     n0 * IMU_SAMPLE_SIZE);
              if (chunk > n0) {
                      memcpy(imu_buf + (size_t)n0 * IMU_SAMPLE_SIZE,
                             imu_pre_roll,
                             (chunk - n0) * IMU_SAMPLE_SIZE);
              }

              uint32_t chunk_first_ts = imu_pre_roll_ts[start_idx];
              mp4_muxer_write_metadata(mp4_muxer, imu_buf, chunk * IMU_SAMPLE_SIZE, chunk_first_ts);
              flushed += chunk;
      }

      imu_pre_roll_cnt  = 0;
      imu_pre_roll_head = 0;
}

imuconfig imu_cfg = {
    .tscale = 0.001,
    .gscale = 0.017453,
    .timestamp_base_sec = 0.0
};

static void imu_pack_sample(uint8_t *buf, size_t *offset, uint32_t ts, const float *dps)
{
      size_t start = *offset;
      uint32_t raw;
      buf[(*offset)++] = (uint8_t)(ts & 0xFF);
      buf[(*offset)++] = (uint8_t)((ts >> 8) & 0xFF);
      buf[(*offset)++] = (uint8_t)((ts >> 16) & 0xFF);
      buf[(*offset)++] = (uint8_t)((ts >> 24) & 0xFF);
      for (int i = 0; i < 3; i++) {
              memcpy(&raw, &dps[i], sizeof(raw));   // IEEE-754 bits, little-endian out
              buf[(*offset)++] = (uint8_t)(raw & 0xFF);
              buf[(*offset)++] = (uint8_t)((raw >> 8) & 0xFF);
              buf[(*offset)++] = (uint8_t)((raw >> 16) & 0xFF);
              buf[(*offset)++] = (uint8_t)((raw >> 24) & 0xFF);
      }
}

void gyro_read_gsensor(void *parm)
{
    static uint32_t last_sys_ts = 0;
	static uint32_t gyro_offset = 0;
    uint32_t current_sys_ts = mm_read_mediatime_ms();

    int read_cnt = gyroscope_fifo_read(gdata, GYRO_FIFO_MAX);
    if (read_cnt <= 0) {
        return;
    }
    if (read_cnt > GYRO_FIFO_MAX) {
        read_cnt = GYRO_FIFO_MAX;
    }

    // Initialize reference point
    if (last_sys_ts == 0) {
        last_sys_ts = current_sys_ts - (read_cnt * 1000 / GYRO_ODR_HZ);
    }
    float actual_interval = (float)(current_sys_ts - last_sys_ts) / read_cnt;

    int mett_ready = imu_mp4_mett_ready();

    mp4_context *mp4_muxer = NULL;
    if (lr_mp4_ctx != NULL) {
        mp4_ctx_t *mp4_module_ctx = (mp4_ctx_t *)(lr_mp4_ctx->priv);
        mp4_muxer = (mp4_context *)(mp4_module_ctx->mp4_muxer);
    }

	// Align gyro start with first video frame
    if (gyro_offset == 0 && video_first_ts > 0) {
        gyro_offset = last_sys_ts;   // reset gyro base to current system time
    }

    for (int i = 0; i < read_cnt; i++) {
    	// gdata[i].timestamp = (float)last_sys_ts + (i * actual_interval);
		float raw_ts = (float)last_sys_ts + (i * actual_interval);
		gdata[i].timestamp = raw_ts - gyro_offset;

        // skip until muxer is ready
        if (!mett_ready) continue;

        double ts_ms = (imu_cfg.timestamp_base_sec + gdata[i].timestamp * imu_cfg.tscale) * 1000.0;
		uint32_t ts = (uint32_t)ts_ms;
		
		double gyro_scale = imu_cfg.gscale * (180.0 / 3.141592653589793);
		float dps_scaled[3];
		// 原始方向（没有旋转）
		dps_scaled[0] = gdata[i].dps[0] * gyro_scale;
		dps_scaled[1] = gdata[i].dps[1] * gyro_scale;
		dps_scaled[2] = gdata[i].dps[2] * gyro_scale;

		// // 绕 X 轴旋转 90°
		// dps_scaled[0] =  gdata[i].dps[0] * gyro_scale;
		// dps_scaled[1] = -gdata[i].dps[2] * gyro_scale;
		// dps_scaled[2] =  gdata[i].dps[1] * gyro_scale;

		// // 绕 Y 轴旋转 90°
		// dps_scaled[0] =  gdata[i].dps[2] * gyro_scale;
		// dps_scaled[1] =  gdata[i].dps[1] * gyro_scale;
		// dps_scaled[2] = -gdata[i].dps[0] * gyro_scale;

		// // 绕 Z 轴旋转 90°
		// dps_scaled[0] = -gdata[i].dps[1] * gyro_scale;
		// dps_scaled[1] =  gdata[i].dps[0] * gyro_scale;
		// dps_scaled[2] =  gdata[i].dps[2] * gyro_scale;

        if (imu_len == 0) {
            imu_chunk_first_ts = ts;
        }
        imu_pack_sample(imu_buf, &imu_len, ts, gdata[i].dps);

        if (imu_len == sizeof(imu_buf)) {
            if (mp4_muxer) {
                mp4_muxer_write_metadata(mp4_muxer, imu_buf, (unsigned int)imu_len, imu_chunk_first_ts);
            }
            imu_len = 0;
        }
    }

    last_sys_ts = current_sys_ts;
}

static void vGensorTimeCallbackMett(TimerHandle_t xTimer)
{
      gyro_read_gsensor(NULL);
      if (gsensor_timer != NULL) {
              if (xTimerStart(gsensor_timer, 0) != pdPASS) {
                      AI_GLASS_INFO("Reload G-sensor read timer\r\n");
              }
      }
}

/* udta_box_cb: build the full udta box (calib only). */
static int camera_calib_set_udta(unsigned char *udta_buf, pmp4_context mp4_ctx)
{
      int pos = 0;
      calib_put32_be(udta_buf, 0); /* size patched at end */
      pos += 4;
      memcpy(udta_buf + 4, "udta", 4);
      pos += 4;

      pos += camera_calib_meta_box(udta_buf + pos); /* calib only, no GPS */

      calib_put32_be(udta_buf, (uint32_t)pos);      /* patch udta size */
      update_udtabox_size(mp4_ctx, udta_buf, pos);
      return pos;
}

#else // GYRO_USE_MODE == 0 : current CSV implementation
// ============================================================================
// GYRO_USE_MODE == 0 : CSV mode (current) - G-sensor data saved to .csv
// ============================================================================

extern uint32_t first_video_frame_ts;
static void save_gyro_data_to_file(gyro_data_list_t *list, const char *filename)
{
	FILE *file = extdisk_fopen(filename, "wb");
	if (file == NULL) {
		AI_GLASS_ERR("Failed to open file for writing.\n");
		return;
	}

	static const char *csv_header_data =
		"GYROFLOW IMU LOG\n"
		"version,1.3\n"
		"id,amb82\n"
		"orientation,Xyz\n"
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
		"t,gx,gy,gz\n";

	extdisk_fwrite((const char *) csv_header_data, strlen(csv_header_data), 1, file);

	gyro_data_node_t *current = list->head;
	char gyro_data_string[128] = {0};
	while (current != NULL) {
#if !IGN_ACC_DATA
		int gyro_str_size = snprintf(gyro_data_string, sizeof(gyro_data_string) - 1, "%.2f, %f, %f, %f, %f, %f, %f\n", current->data.timestamp, current->data.dps[0],
									 current->data.dps[1], current->data.dps[2], current->data.g[0], current->data.g[1], current->data.g[2]);
#else
		int gyro_str_size = snprintf(gyro_data_string, sizeof(gyro_data_string) - 1, "%.2f, %f, %f, %f\n", current->data.timestamp, current->data.dps[0],
									 current->data.dps[1], current->data.dps[2]);
#endif
		if (gyro_str_size > sizeof(gyro_data_string) - 1) {
			AI_GLASS_MSG("gyro_str_size = %lu, sizeof(gyro_data_string) = %lu\r\n", gyro_str_size, sizeof(gyro_data_string) - 1);
#if !IGN_ACC_DATA
			snprintf(gyro_data_string, sizeof(gyro_data_string) - 1, "%.2f, %f, %f, %f, %f %f %f\n", current->data.timestamp, current->data.dps[0],
					 current->data.dps[1], current->data.dps[2], current->data.g[0], current->data.g[1], current->data.g[2]);
#else
			snprintf(gyro_data_string, sizeof(gyro_data_string) - 1, "%.2f, %f, %f, %f\n", current->data.timestamp, current->data.dps[0], current->data.dps[1],
					 current->data.dps[2]);
#endif
		} else {
			extdisk_fwrite((const char *) gyro_data_string, gyro_str_size, 1, file);
		}
		current = current->next;
	}

	extdisk_fclose(file);
	AI_GLASS_MSG("Data saved to file: %s\r\n", filename);
}

static void save_gyro_to_exdisk_thread(void *param)
{
	FILE *file = NULL;
	AI_GLASS_INFO("======================save_gyro_to_exdisk_thread=======================\r\n");
	if (!(exdisk_gyro_status & GYRO_SAVE_SET_START)) {
		AI_GLASS_ERR("exdisk_gyro_status 0x%08x is invalid.\r\n", exdisk_gyro_status);
		goto endofsave;
	}
	exdisk_gyro_status = GYRO_SAVE_START;

	char life_gryo_csv[128] = {0};
	snprintf(life_gryo_csv, sizeof(life_gryo_csv), "%s.csv", life_recording_name);

	file = extdisk_fopen((const char *)life_gryo_csv, "w+");
	if (file == NULL) {
		AI_GLASS_ERR("Failed to open file for writing.\r\n");
		goto endofsave;
	}

	static const char *csv_header_data =
		"GYROFLOW IMU LOG\n"
		"version,1.3\n"
		"id,amb82\n"
		"orientation,xYz\n"
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
	AI_GLASS_MSG("video_first_ts = %u\r\n", video_first_ts);
    // >>> Inject 10 dummy samples before actual video timestamp
    for (int i = 0; i < 10; i++) {
        float dummy_ts = (float)(video_first_ts - (10 - i));
#if !IGN_ACC_DATA
        int ret = snprintf(gyro_data_string, sizeof(gyro_data_string),
                           "%.2f, 0, 0, 0, 0, 0, 0\n", dummy_ts);
#else
        int ret = snprintf(gyro_data_string, sizeof(gyro_data_string),
                           "%.2f, 0, 0, 0\n", dummy_ts);
#endif
        if (ret > 0) {
            extdisk_fwrite((const char *)gyro_data_string, ret, 1, file);
        }
    }
	static int print_first_gyro = 0;

	while (1) {
		gyro_data_node_t *rd_tpr = read_gyro_data(&gyro_list);
		if (print_first_gyro == 0 && rd_tpr != NULL) {
			AI_GLASS_MSG("First gyro timestamp: %f (offset: %d ms)\r\n", rd_tpr->data.timestamp, video_first_ts - (uint32_t)(rd_tpr->data.timestamp));
			print_first_gyro = 1;
		}
		if (rd_tpr != NULL && first_video_frame_ts > 0 && rd_tpr->data.timestamp >= first_video_frame_ts) {
			memcpy(&g_data, &(rd_tpr->data), sizeof(gyro_data_t));
#if !IGN_ACC_DATA
			int ret = snprintf(gyro_data_string, sizeof(gyro_data_string), "%.2f, %f, %f, %f, %f %f %f\n", g_data.timestamp, g_data.dps[0], g_data.dps[1],
							   g_data.dps[2], g_data.g[0], g_data.g[1], g_data.g[2]);
#else
			int ret = snprintf(gyro_data_string, sizeof(gyro_data_string), "%.2f, %f, %f, %f\n", g_data.timestamp, g_data.dps[0], g_data.dps[1],
							   g_data.dps[2]);
#endif
			if (ret > sizeof(gyro_data_string) - 1 || ret < 0) {
				AI_GLASS_MSG("ret = %lu, sizeof(gyro_data_string) = %lu\r\n", ret, sizeof(gyro_data_string) - 1);
#if !IGN_ACC_DATA
				AI_GLASS_MSG("%.2f, %f, %f, %f, %f %f %f\n", g_data.timestamp, g_data.dps[0], g_data.dps[1], g_data.dps[2],
							 g_data.g[0], g_data.g[1], g_data.g[2]);
#else
				AI_GLASS_MSG("%.2f, %f, %f, %f\n", g_data.timestamp, g_data.dps[0], g_data.dps[1], g_data.dps[2]);
#endif
			} else {
				extdisk_fwrite((const char *) gyro_data_string, ret, 1, file);
			}
		} else if (exdisk_gyro_status & GYRO_SAVE_SET_STOP) {
			break;
		} else {
			vTaskDelay(1);
		}
	}

endofsave:
	if (file != NULL) {
		extdisk_fclose(file);
	}
	AI_GLASS_MSG("Data saved to file: %s\r\n", life_gryo_csv);
	exdisk_gyro_status = GYRO_SAVE_STOP;
	vTaskDelete(NULL);
}

static int start_gyro_to_exdisk_process(void)
{
	if (exdisk_gyro_status == GYRO_SAVE_IDLE) {
		exdisk_gyro_status |= GYRO_SAVE_SET_START;
		if (xTaskCreate(save_gyro_to_exdisk_thread, ((const char *)"save_gyro_to_exdisk_thread"), 4096, NULL, tskIDLE_PRIORITY + 4, NULL) != pdPASS) {
			AI_GLASS_ERR("\n\r%s xTaskCreate(save_gyro_to_exdisk_thread) failed\n\r", __FUNCTION__);
			exdisk_gyro_status = GYRO_SAVE_IDLE;
			return -1;
		}
		AI_GLASS_INFO("exdisk_gyro_status is idle\n\r");
		return 0;
	}
	AI_GLASS_WARN("exdisk_gyro_status is not idle\n\r");
	return -1;
}

// G-sensor example => need to update.
// NON-static (global): module_video.c calls gyro_read_gsensor() via extern.
void gyro_read_gsensor(void *parm)
{
	static uint32_t last_sys_ts = 0;
    uint32_t current_sys_ts = mm_read_mediatime_ms();
    
    int read_cnt = gyroscope_fifo_read(gdata, 100);
    if (read_cnt <= 0) return;

	// Initialize reference point
    if (last_sys_ts == 0) {
        last_sys_ts = current_sys_ts - (read_cnt * 1000 / 800); 
    }

	// Calculate the actual "system time interval" occupied by this batch of data
	// This way, no matter how ISP corrects the time, Gyro will follow the System Time
    float actual_interval = (float)(current_sys_ts - last_sys_ts) / read_cnt;

    for (int i = 0; i < read_cnt; i++) {
		// Redistribute timestamps so that the last one exactly equals current_sys_ts
        gdata[i].timestamp = (float)last_sys_ts + (i * actual_interval);
        append_gyro_data(&gyro_list, gdata[i]);
    }

    last_sys_ts = current_sys_ts;
	//AI_GLASS_MSG("Read end G-sensor %lu, read_cnt = %d\r\n", mm_read_mediatime_ms(), read_cnt);
}

static void vGensorTimeCallback(TimerHandle_t xTimer)
{
	gyro_read_gsensor(NULL);
	if (gsensor_timer != NULL) {
		if (xTimerStart(gsensor_timer, 0) != pdPASS) {
			AI_GLASS_INFO("Reload G-sensor read timer\r\n");
		}
	}
}

#endif // GYRO_USE_MODE == 0 (CSV)   /   GYRO_USE_MODE == 1 (SEI/MP4-mett)
#endif

#if ENABLE_GET_GSENSOR_INFO
static void print_gyro_data_list(gyro_data_list_t *list)
{
	gyro_data_node_t *head = list->head;
	gyro_data_node_t *tail = list->tail;
	AI_GLASS_MSG("head Timestamp: %lu\r\n", head->data.timestamp);
	AI_GLASS_MSG("tail Timestamp: %lu\r\n", tail->data.timestamp);
}
#endif

int lr_gyro_deinit(void)
{
#if ENABLE_GET_GSENSOR_INFO
#if GYRO_USE_MODE == 1
	if (gsensor_timer != NULL) {
		xTimerStop(gsensor_timer, 0);
		if (xTimerDelete(gsensor_timer, 0) == pdPASS) {
			gsensor_timer = NULL;
		}
	}
	if (gyroscope_is_inited()) {
		// Todo: deinit gyroscope
	}
#else   /* CSV mode */
	if (gsensor_timer != NULL) {
		xTimerStop(gsensor_timer, 0);
		if (xTimerDelete(gsensor_timer, 0) == pdPASS) {
			gsensor_timer = NULL;
		}
	}
	if (gyroscope_is_inited()) {
		// Todo: deinit gyroscope
	}
#endif
#else
	AI_GLASS_WARN("Please enable ENABLE_GET_GSENSOR_INFO when using lr_gyro_deinit\r\n");
#endif
	return 0;
}

int lr_gyro_init(void)
{
#if ENABLE_GET_GSENSOR_INFO
	if (gyroscope_fifo_init() != 0) {
		return -1;
	}
	delete_whole_list(&gyro_list);
	gyroscope_reset_fifo();
#if GYRO_USE_MODE == 0
	if (gsensor_timer == NULL) {
		gsensor_timer = xTimerCreate("gsensor_timer", 20, pdFALSE, &gsensor_timer, vGensorTimeCallback);
	}

	if (gsensor_timer != NULL) {
		if (xTimerStart(gsensor_timer, 0) != pdPASS) {
			AI_GLASS_INFO("Reload G-sensor read timer\r\n");
			return -1;
		}
	} else {
		AI_GLASS_ERR("gsensor_timer create failed\r\n");
		return -1;
	}
#elif GYRO_USE_MODE == 1
	// Free-running gyro drain: start the IMU readout on its own timer so samples
	// flow BEFORE the first video frame / MP4 start. Pre-muxer samples are kept
	// in the pre-roll ring (imu_pre_roll) and flushed into the mett track the
	// moment the muxer becomes ready.
	imu_pre_roll_cnt  = 0;
	imu_pre_roll_head = 0;
	imu_len = 0;
	imu_chunk_first_ts = 0;
	last_imu_ts = 0;
	imu_prev_mett_ready = 0;
	if (gsensor_timer == NULL) {
		gsensor_timer = xTimerCreate("gsensor_timer", 2, pdFALSE, &gsensor_timer, vGensorTimeCallbackMett);
	}

	if (gsensor_timer != NULL) {
		if (xTimerStart(gsensor_timer, 0) != pdPASS) {
			AI_GLASS_INFO("Reload G-sensor read timer\r\n");
			return -1;
		}
	} else {
		AI_GLASS_ERR("gsensor_timer create failed\r\n");
		return -1;
	}
#endif
	AI_GLASS_INFO("lr_gyro_init success\r\n");
#else
	AI_GLASS_WARN("lr_gyro_init, Please enable ENABLE_GET_GSENSOR_INFO when using lr_gyro_init\r\n");
#endif
	return 0;
}

static int get_mp4_video_timestamp(void)
{
#if ENABLE_GET_GSENSOR_INFO
	if (lr_mp4_ctx != NULL) {
#if ENABLE_GET_GSENSOR_INFO
		mp4_ctx_t *mp4_module_ctx = (mp4_ctx_t *)(lr_mp4_ctx->priv);
		mp4_context *mp4_muxer = (mp4_context *)(mp4_module_ctx->mp4_muxer);
#if !GSENSOR_RECORD_FAST
		align_gyro_data_by_timestamp(&gyro_list, mp4_muxer->video_timestamp_first);
#endif
		AI_GLASS_MSG("mp4_muxer->video_timestamp_first = %u\r\n", mp4_muxer->video_timestamp_first);
		AI_GLASS_MSG("mp4_muxer->video_timestamp_end = %u\r\n",
					 mp4_muxer->video_timestamp_first + (mp4_muxer->video_timestamp_buffer[mp4_muxer->root.video_len - 1] - mp4_muxer->video_timestamp_buffer[0]) /
					 (mp4_muxer->video_clock_rate / configTICK_RATE_HZ));
#endif
	}
#endif
	return 0;
}

MP4State current_state = STATE_IDLE;

extern void mp4_send_response_callback(void *parm);
static int lr_mp4_end_cb(void *parm)
{
	AI_GLASS_INFO("Record end\r\n");
	extdisk_save_file_cntlist();
#if ENABLE_GET_GSENSOR_INFO
#if GYRO_USE_MODE == 1   // --- SEI mode ---
	exdisk_gyro_status = GYRO_SAVE_STOP;
    delete_whole_list(&gyro_list);  // optional cleanup
    current_state = STATE_END_RECORDING;
#elif GSENSOR_RECORD_FAST
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
#else
	current_state = STATE_END_RECORDING;
#endif
	return 0;
}

static int lr_mp4_stop_cb(void *parm)
{
	AI_GLASS_INFO("Record stop\r\n");
#if ENABLE_GET_GSENSOR_INFO && (GYRO_USE_MODE == 1)
    exdisk_gyro_status = GYRO_SAVE_STOP;
    delete_whole_list(&gyro_list);
    current_state = STATE_END_RECORDING;
#else
    current_state = STATE_IDLE;
#endif
#if ENABLE_GET_GSENSOR_INFO
	lr_gyro_deinit();
	print_gyro_data_list(&gyro_list);
	get_mp4_video_timestamp();
#endif
	return 0;
}

static int lr_mp4_error_cb(void *parm)
{
	AI_GLASS_ERR("Lifetime recording error\r\n");
	current_state = STATE_ERROR;
#if ENABLE_GET_GSENSOR_INFO
	lr_gyro_deinit();
#endif
	return 0;
}

int lifetime_recording_initialize(uint8_t record_filename_length, const char *filename)
{
	AI_GLASS_INFO("================LifeTime Record start========================== = %lu\r\n", mm_read_mediatime_ms());
	uint8_t recording_filename_length = record_filename_length;

	if (recording_filename_length == 0) {
		char *cur_time_str = (char *)media_filesystem_get_current_time_string();
		extdisk_generate_unique_filename("VIDEO_0_0_", cur_time_str, ".mp4", life_recording_name, 128);
		free(cur_time_str);
	} else {
		extdisk_generate_unique_filename("", filename, ".mp4", life_recording_name, 128);
	}

	// work around, pre-open the mp4 for the file system to count the mp4 file
	char life_record_name[128] = {0};
	snprintf(life_record_name, sizeof(life_record_name), "%s.mp4", life_recording_name);
	FILE *life_record_mp4 = extdisk_fopen(life_record_name, "w");
	extdisk_fclose(life_record_mp4);

	ai_glass_record_param_t *ai_record_param = NULL;
#if ENABLE_GET_GSENSOR_INFO
	AI_GLASS_MSG("================start initial g-sensor========================== = %lu\r\n", mm_read_mediatime_ms());
	// Initial G-sensor
	if (lr_gyro_init()) {
		goto lifetime_recording_initialize_fail;
	}
	AI_GLASS_MSG("================initial g-sensor done========================== = %lu\r\n", mm_read_mediatime_ms());
#if GSENSOR_RECORD_FAST
	#if GYRO_USE_MODE == 0
	if (start_gyro_to_exdisk_process()) {
		goto lifetime_recording_initialize_fail;
	}
	AI_GLASS_MSG("================start_gyro_to_exdisk_process done========================== = %lu\r\n", mm_read_mediatime_ms());
#endif
#endif
#endif

	AI_GLASS_MSG("================record process start========================== = %lu\r\n", mm_read_mediatime_ms());
	ai_record_param = (ai_glass_record_param_t *) malloc(sizeof(ai_glass_record_param_t));
	memset(ai_record_param, 0x00, sizeof(ai_glass_record_param_t));
	media_get_record_params(ai_record_param);
	if(sensor_idx != -1) {
		deinitial_media(); 
	}
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
	lr_video_params.rotation = ai_record_param->rotation;
	AI_GLASS_MSG("LifeTime Record Max Time = %d\r\n", lr_mp4_params.record_length);

	if (lr_video_params.rotation == 1 || lr_video_params.rotation == 2) {
		lr_mp4_params.width = ai_record_param->height;
		lr_mp4_params.height = ai_record_param->width;
	}
	
#if AUDIO_SRC==AUDIO_INTERFACE
	lr_audio_ctx = mm_module_open(&audio_module);
	memcpy((void *)&audio_params, (void *)&default_audio_params, sizeof(audio_params_t));
	if (lr_audio_ctx) {
		mm_module_ctrl(lr_audio_ctx, CMD_AUDIO_GET_PARAMS, (int)&audio_params);
#if OPUSC_UPSAMPLE_16K_TO_48K == 1

		audio_params.sample_rate = audio_samplerate2index(AUDIO_I_SAMPLE_RATE);

#else

		audio_params.sample_rate = audio_samplerate2index(AUDIO_SAMPLE_RATE);

#endif
		mm_module_ctrl(lr_audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
		mm_module_ctrl(lr_audio_ctx, MM_CMD_SET_QUEUE_LEN, 6); //queue size can be smaller 160ms
		mm_module_ctrl(lr_audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
	} else {
		AI_GLASS_ERR("audio open fail\n\r");
		goto lifetime_recording_initialize_fail;
	}
#elif AUDIO_SRC==I2S_INTERFACE
	lr_audio_ctx = mm_module_open(&i2s_module);
	if (lr_audio_ctx) {
		mm_module_ctrl(lr_audio_ctx, CMD_I2S_GET_PARAMS, (int)&i2s_params);

#if OPUSC_UPSAMPLE_16K_TO_48K == 1

	i2s_params.sample_rate = i2s_samplerate2index(i2S_SAMPLE_RATE);

#else

	i2s_params.sample_rate = i2s_samplerate2index(AUDIO_SAMPLE_RATE);

#endif	
		i2s_params.i2s_direction = I2S_RX_ONLY;
		i2s_params.i2s_role = AUDIO_I2S_ROLE;
		//i2s_params.pin_group_num = 0;
		mm_module_ctrl(lr_audio_ctx, CMD_I2S_SET_PARAMS, (int)&i2s_params);
		mm_module_ctrl(lr_audio_ctx, MM_CMD_SET_QUEUE_LEN, 6); //queue size can be smaller 160ms
		mm_module_ctrl(lr_audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
	} else {
		AI_GLASS_ERR("i2s open fail\n\r");
		goto lifetime_recording_initialize_fail;
	}
#endif

	lr_opusc_ctx = mm_module_open(&opusc_module);
	if (lr_opusc_ctx) {
		mm_module_ctrl(lr_opusc_ctx, CMD_OPUSC_SET_PARAMS, (int)&opusc_rtsp_params);
		mm_module_ctrl(lr_opusc_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(lr_opusc_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(lr_opusc_ctx, CMD_OPUSC_APPLY, 0);
	} else {
		AI_GLASS_ERR("OPUSC open fail\n\r");
		goto lifetime_recording_initialize_fail;
	}

	lr_mp4_ctx = mm_module_open(&mp4_module);
	lr_mp4_params.mp4_audio_format = AUDIO_OPUS;
	lr_mp4_params.mp4_audio_duration = 40;

	if (lr_mp4_ctx) {

#if GYRO_USE_MODE == 1
		/* camera-calib udta buffer (must be set BEFORE SET_PARAMS) */
		lr_mp4_params.udta_buf = (unsigned char *)malloc(CAMERA_CALIB_UDTA_SIZE);
		if (lr_mp4_params.udta_buf == NULL) {
			AI_GLASS_ERR("malloc udta_buf failed\n\r");
			goto lifetime_recording_initialize_fail;
		}
		memset(lr_mp4_params.udta_buf, 0, CAMERA_CALIB_UDTA_SIZE);
		lr_mp4_params.udta_buf_size = CAMERA_CALIB_UDTA_SIZE;

		/* metadata (mett) trak box buffer - the app builds the trak into this
		   buffer (via the meta-trak callback) at stop time, then the muxer
		   memcpy's it into moov (mirrors the udta box). */
		lr_mp4_params.trak_buf = (unsigned char *)malloc(META_TRAK_BOX_SIZE);
		if (lr_mp4_params.trak_buf == NULL) {
			AI_GLASS_ERR("malloc trak_buf failed\n\r");
			goto lifetime_recording_initialize_fail;
		}
		memset(lr_mp4_params.trak_buf, 0, META_TRAK_BOX_SIZE);
		lr_mp4_params.trak_buf_size = META_TRAK_BOX_SIZE;
#endif

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

#if GYRO_USE_MODE == 1
		/* register the udta callback */
		udta_callback_t udta_params;
		memset(&udta_params, 0, sizeof(udta_callback_t));
		udta_params.udta_box_cb = camera_calib_set_udta;
		mm_module_ctrl(lr_mp4_ctx, CMD_MP4_SET_UDAT_CALLBACK, (int)&udta_params);

		/* register the metadata trak callback - build the mett trak box from
		   gyrosensor_data_box.c (trak_box_metadata), replacing the muxer-built
		   one. Runs inside Set_moov_box_all via CMD_MP4_SET_META_TRAK_CALLBACK. */
		meta_trak_callback_t meta_params;
		memset(&meta_params, 0, sizeof(meta_trak_callback_t));
		meta_params.meta_trak_box_cb = trak_box_metadata;
		mm_module_ctrl(lr_mp4_ctx, CMD_MP4_SET_META_TRAK_CALLBACK, (int)&meta_params);
#endif

	} else {
		AI_GLASS_ERR("MP4 open fail\n\r");
		goto lifetime_recording_initialize_fail;
	}

	AI_GLASS_INFO("MP4 opened\n\r");

	video_pre_init_params_t ai_glass_pre_init_params = {0};
	lr_video_ctx = mm_module_open(&video_module);
	if (lr_video_ctx) {
		media_get_preinit_isp_data(&ai_glass_pre_init_params);
		mm_module_ctrl(lr_video_ctx, CMD_VIDEO_PRE_INIT_PARM, (int)&ai_glass_pre_init_params);
		if(sensor_idx != -1) {
			mm_module_ctrl(lr_video_ctx, CMD_VIDEO_SET_SENSOR_ID, sensor_idx);
		}
		mm_module_ctrl(lr_video_ctx, CMD_VIDEO_SET_PARAMS, (int)&lr_video_params);
		mm_module_ctrl(lr_video_ctx, MM_CMD_SET_QUEUE_LEN, lr_video_params.fps * 10); //Add the queue buffer to avoid to lost data.
		mm_module_ctrl(lr_video_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		AI_GLASS_ERR("video open fail\n\r");
		goto lifetime_recording_initialize_fail;
	}

	lr_siso_audio_opusc = siso_create();
	if (lr_siso_audio_opusc) {
		siso_ctrl(lr_siso_audio_opusc, MMIC_CMD_ADD_INPUT, (uint32_t)lr_audio_ctx, 0);
		siso_ctrl(lr_siso_audio_opusc, MMIC_CMD_ADD_OUTPUT, (uint32_t)lr_opusc_ctx, 0);
		siso_ctrl(lr_siso_audio_opusc, MMIC_CMD_SET_TASKPRIORITY, 4, 0);
		siso_ctrl(lr_siso_audio_opusc, MMIC_CMD_SET_STACKSIZE, 24 * 1024, 0);
		siso_start(lr_siso_audio_opusc);
	} else {
		AI_GLASS_ERR("lr siso audio aac open fail\n\r");
		goto lifetime_recording_initialize_fail;
	}

	lr_miso_video_opusc_mp4 = miso_create();
	if (lr_miso_video_opusc_mp4) {
		miso_ctrl(lr_miso_video_opusc_mp4, MMIC_CMD_ADD_INPUT0, (uint32_t)lr_video_ctx, 0);
		miso_ctrl(lr_miso_video_opusc_mp4, MMIC_CMD_ADD_INPUT1, (uint32_t)lr_opusc_ctx, 0);
		miso_ctrl(lr_miso_video_opusc_mp4, MMIC_CMD_ADD_OUTPUT0, (uint32_t)lr_mp4_ctx, 0);
		miso_ctrl(lr_miso_video_opusc_mp4, MMIC_CMD_SET_TASKPRIORITY, 4, 0);
		miso_start(lr_miso_video_opusc_mp4);
	} else {
		AI_GLASS_ERR("miso open fail for video recording\n\r");
		goto lifetime_recording_initialize_fail;
	}

	AI_GLASS_INFO("miso(videochn0_aac_mp4) started\n\r");

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
	return 0;
lifetime_recording_initialize_fail:
	if (ai_record_param) {
		free(ai_record_param);
	}
	lifetime_recording_deinitialize();
	return -1;
}

void lifetime_recording_stop(void)
{
	if (lr_mp4_ctx != NULL) {
		mm_module_ctrl(lr_mp4_ctx, CMD_MP4_STOP_IMMEDIATELY, 0);
	}
}

void lifetime_recording_deinitialize(void)
{
	//Pause Linker
	//Pause individual audio
	siso_pause(lr_siso_audio_opusc);

	//Pause MP4 recording / RTSP Stream
	miso_pause(lr_miso_video_opusc_mp4, MM_OUTPUT0);

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
	if (lr_opusc_ctx != NULL) {
		mm_module_ctrl(lr_opusc_ctx, CMD_OPUSC_STOP, 0);
	}
	if (lr_mp4_ctx != NULL) {
		mm_module_ctrl(lr_mp4_ctx, CMD_MP4_STOP_IMMEDIATELY, 0);
	}
	//Delete linker
	siso_delete(lr_siso_audio_opusc);
	lr_siso_audio_opusc = NULL;
	miso_delete(lr_miso_video_opusc_mp4);
	lr_miso_video_opusc_mp4 = NULL;

	//Close module
	mm_module_close(lr_audio_ctx);
	lr_audio_ctx = NULL;
	mm_module_close(lr_opusc_ctx);
	lr_opusc_ctx = NULL;
	mm_module_close(lr_mp4_ctx);
	lr_mp4_ctx = NULL;
	mm_module_close(lr_video_ctx);
	lr_video_ctx = NULL;

#if GYRO_USE_MODE == 1
	if (lr_mp4_params.udta_buf) {
		free(lr_mp4_params.udta_buf);
		lr_mp4_params.udta_buf = NULL;
		lr_mp4_params.udta_buf_size = 0;
	}
	if (lr_mp4_params.trak_buf) {
		free(lr_mp4_params.trak_buf);
		lr_mp4_params.trak_buf = NULL;
		lr_mp4_params.trak_buf_size = 0;
	}
#endif

	current_state = STATE_IDLE;
	
	if(sensor_idx != -1) {
		sensor_idx = -1;
	}
}