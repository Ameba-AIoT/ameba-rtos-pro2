/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/

#include <stdint.h>
#include <math.h>
#include "platform_stdlib.h"
#include "osdep_service.h"
#include "avcodec.h"
#include "mmf2_module.h"
#include "module_tone.h"

//------------------------------------------------------------------------------
void toneframe_timer_handler(uint32_t hid);

//#define TIMER_FUNCTION
#define MIN_DBFS 128
float dB_fPOW[MIN_DBFS];
static void tone_timer_thread(void *param)
{
	tone_ctx_t *ctx = (tone_ctx_t *)param;
	while (1) {
		if (ctx->audio_timer_process_time < ctx->audio_timer_delay_ms) {
			vTaskDelay(ctx->audio_timer_delay_ms - ctx->audio_timer_process_time);
		} else {

			printf("process %ld too long\r\n", ctx->audio_timer_process_time);
			vTaskDelay(1);
		}
		toneframe_timer_handler((uint32_t)ctx);
	}
}

void tonetimer_task_enable(void *parm)
{
	tone_ctx_t *ctx = (tone_ctx_t *)parm;

	if (xTaskCreate(tone_timer_thread, ((const char *)"tone_timer_thread"), 2048, ctx, tskIDLE_PRIORITY + 5, &ctx->task) != pdPASS) {
		printf("\n\r%s xTaskCreate failed", __FUNCTION__);
	}
}

uint8_t test_pcm_playout [3200];
static int interval_frames = 0;
int number_frames = 0;
int sweep_flag = 0;
void toneframe_timer_handler(uint32_t hid)
{
	tone_ctx_t *ctx = (tone_ctx_t *)hid;
	uint32_t i;
	uint32_t time_sd_start = 0;
	short tx_pattern;
	float temp, n_cnt = 0, m_cnt;
	ctx->audio_timer_process_time = 0;
	if (ctx->stop) {
		return;
	}

	BaseType_t xTaskWokenByReceive = pdFALSE;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

#ifdef TIMER_FUNCTION
	uint32_t timestamp = xTaskGetTickCountFromISR();
#else
	uint32_t timestamp = xTaskGetTickCount();
#endif

	mm_context_t *mctx = (mm_context_t *)ctx->parent;
	mm_queue_item_t *output_item;

	if (ctx->playmode == PLAY_NONE) {
		return;
	}

#ifdef TIMER_FUNCTION
	int is_output_ready = xQueueReceiveFromISR(mctx->output_recycle, &output_item, &xTaskWokenByReceive) == pdTRUE;
#else
	int is_output_ready = xQueueReceive(mctx->output_recycle, &output_item, 1000) == pdTRUE;
#endif
	if (is_output_ready) {
		output_item->type = ctx->params.codec_id;
		output_item->timestamp = timestamp;
		output_item->size = ctx->params.frame_size;
		time_sd_start = xTaskGetTickCount();

		if (ctx->playmode == PLAY_SD_DATA || ctx->playmode == PLAY_RAMDISK_DATA) {

			int br = -1;
			static int total_size = 0;

			br = fread((void *)output_item->data_addr, 1, ctx->params.frame_size, ctx->audio_data);
			total_size += br;
			if (br < ctx->params.frame_size) {
				fseek(ctx->audio_data, 0, SEEK_SET);
				total_size = 0;

			}
		} else if (ctx->playmode == PLAY_TONE) {

			m_cnt = ctx->params.samplerate;

			short *output_buf = (short *)output_item->data_addr;
			if (ctx->sweep_enable) {
				if (ctx->sweep_frequency == 0) {
					if (ctx->pre_sweep_frames == 0) {
						ctx->tone_data_offset = 0;
						ctx->sweep_frequency += 20;
						n_cnt = ctx->sweep_frequency;
					} else {
						ctx->pre_sweep_frames --;
					}
				} else if (ctx->sweep_frequency == ctx->params.samplerate / 2) {
					ctx->sweep_frequency = 0;
					ctx->pre_sweep_frames = PRE_SWEEP_MS / ctx->audio_timer_delay_ms;
				} else {
					if (interval_frames == (ctx->sweep_interval_frames - 1)) {
						if (ctx->tone_data_offset != 0) {
							printf("interval_frames = %d, sweep_frequency = %d, tone_data_offset = %d\r\n", interval_frames, ctx->sweep_frequency, ctx->tone_data_offset);
						}
						ctx->sweep_frequency += 20;
						interval_frames = 0;
						ctx->tone_data_offset = 0;
					} else {
						interval_frames ++;
					}
					n_cnt = ctx->sweep_frequency;
				}
			} else {
				n_cnt = ctx->params.audiotonerate;
			}
			memset(output_buf, 0, ctx->params.frame_size);

			if (ctx->enable_DB_SWEEP) {
				number_frames ++;
				if (number_frames == ctx->DBsweep_frames) {
					if (ctx->target_dB == 57) {
						sweep_flag = 0;
					} else if (ctx->target_dB == 0) {
						sweep_flag = 1;
						//ctx->target_dB = 57;
					}
					if (sweep_flag) {
						ctx->target_dB += 3;
					} else {
						ctx->target_dB -= 3;
					}
					number_frames = 0;
					//printf("ctx->target_dB = %d\r\n", ctx->target_dB);
				}
			}

			//printf("sweep_enable = %d, sweep_frequency = %d, tone_data_offset = %d\r\n", ctx->sweep_enable, ctx->sweep_frequency, ctx->tone_data_offset);
			//printf("output_item->timestamp = %d\r\n", output_item->timestamp);
			if (!(ctx->sweep_enable) || (ctx->sweep_frequency)) {
				for (i = 0; i < (ctx->params.frame_size >> 1); i++) {
					temp = sin((2 * (3.141592653589793f) * (float)n_cnt) / (float)m_cnt * (float)(ctx->tone_data_offset)) * dB_fPOW[ctx->target_dB] *
						   (32767.0f);// / (pow(10, 3.0f/20));
					tx_pattern = (short)temp;
					output_buf[i] = tx_pattern;
					ctx->tone_data_offset++;
					if (((uint32_t)n_cnt * ctx->tone_data_offset) % ctx->params.samplerate == 0) {
						//printf("sweep_frequency = %d, tone_data_offset = %d, interval_frames = %d\r\n", ctx->sweep_frequency, ctx->tone_data_offset, interval_frames);
						ctx->tone_data_offset = 0;
					}
				}
			}
		}
#ifdef TIMER_FUNCTION
		xQueueSendFromISR(mctx->output_ready, (void *)&output_item, &xHigherPriorityTaskWoken);
#else
		xQueueSend(mctx->output_ready, (void *)&output_item, 0);
		ctx->audio_timer_process_time = xTaskGetTickCount() - time_sd_start;

#endif
	} else {
		printf("frames_drop\r\n");
	}
#ifdef TIMER_FUNCTION
	if (xHigherPriorityTaskWoken || xTaskWokenByReceive) {
		taskYIELD();
	}
#endif
}

int tone_control(void *p, int cmd, int arg)
{
	tone_ctx_t *ctx = (tone_ctx_t *)p;

	switch (cmd) {
	case CMD_TONE_SET_PARAMS:
		memcpy(&ctx->params, (void *)arg, sizeof(tone_params_t));
		if (ctx->params.samplerate != 8000 && ctx->params.samplerate != 16000 && ctx->params.samplerate != 32000 && ctx->params.samplerate != 44100 &&
			ctx->params.samplerate != 48000 && ctx->params.samplerate != 88200 && ctx->params.samplerate != 96000) {
			ctx->params.samplerate = 16000;
			mm_printf("Invalid sample rate, Set default sample rate: %d\r\n", ctx->params.audiotonerate);
		}
		if (ctx->params.audiotonerate < 0 || ctx->params.audiotonerate > (ctx->params.samplerate >> 1)) {
			ctx->params.audiotonerate = 1000;
			mm_printf("Invalid audio tone rate, Set default audio tone: %d\r\n", ctx->params.audiotonerate);
		}
		snprintf(ctx->sdcard_filename, sizeof(ctx->sdcard_filename), "%s:/%s", ctx->params.sdcard_tag, ctx->params.audio_filename);
		snprintf(ctx->ramdisk_filename, sizeof(ctx->ramdisk_filename), "%s:/%s", ctx->params.ramdisk_tag, ctx->params.audio_filename);
		break;
	case CMD_TONE_GET_PARAMS:
		memcpy((void *)arg, &ctx->params, sizeof(tone_params_t));
		break;
	case CMD_TONE_SET_AUDIOTONE:
		mm_printf("SET_AUDIOTONE\r\n");
		if (arg > 0 && arg < (ctx->params.samplerate >> 1)) {
			ctx->params.audiotonerate = arg;
			ctx->sweep_enable = 0;
			mm_printf("Set audio tone: %d\r\n", ctx->params.audiotonerate);
		} else {
			ctx->params.audiotonerate = 1000;
			ctx->sweep_enable = 0;
			mm_printf("Invalid audio tone rate for sample rate %d, Set default audio tone: %d\r\n", ctx->params.samplerate, ctx->params.audiotonerate);
		}
		ctx->tone_data_offset = 0;
		break;
	case CMD_TONE_SET_SAMPLERATE:
		mm_printf("SET_SAMPLERATE\r\n");
		if (arg == 8000 || arg == 16000 || arg == 32000 || arg == 44100 || arg == 48000 || arg == 88200 || arg == 96000) {
			ctx->params.samplerate = arg;
		} else {
			ctx->params.samplerate = 16000;
			mm_printf("Invalid sample rate.");
		}
		mm_printf("Set default sample rate: %d\r\n", ctx->params.samplerate);
		if (ctx->params.audiotonerate >= (ctx->params.samplerate >> 1)) {
			ctx->params.audiotonerate = 1000;
			mm_printf("Invalid audio tone rate for sample rate %d, Set default audio tone: %d\r\n", ctx->params.samplerate, ctx->params.audiotonerate);
		}
		ctx->tone_data_offset = 0;
		break;
	case CMD_TONE_RECOUNT_PERIOD:
		if (ctx->params.codec_id == AV_CODEC_ID_PCM_RAW) {
			ctx->frame_timer_period = (int)(1000000 / ((float)ctx->params.samplerate * 2 / ctx->params.frame_size));
		} else {
			return -1;
		}

		if (ctx->frame_timer_period == 0) {
			printf("Error, frame_timer_period can't be 0\n\r");
			return -1;
		}
		printf("Recount frame_timer_period success\n\r");
		ctx->audio_timer_delay_ms = ctx->frame_timer_period / 1000;
		break;
	case CMD_TONE_SWEEP_TONE:
		if (arg > 0) {
			//ctx->sweep_interval_frames = arg / ctx->audio_timer_delay_ms;
			ctx->sweep_interval_frames = SWEEP_INTERVAL_MS / ctx->audio_timer_delay_ms;
			ctx->pre_sweep_frames = PRE_SWEEP_MS / ctx->audio_timer_delay_ms;
			ctx->sweep_frequency = 0;
			if (ctx->sweep_interval_frames) {
				ctx->sweep_enable = 1;
			} else {
				ctx->sweep_enable = 0;
				printf("Sweep interval is too small\n\r");
			}
		} else {
			ctx->sweep_enable = 0;
		}
		break;
	case CMD_TONE_TARGET_DB:
		if (arg >= 0 && arg <= (MIN_DBFS - 1)) {
			ctx->target_dB = arg;
		}
		printf("set tone target dB = %d, value = %f\r\n", -ctx->target_dB, dB_fPOW[ctx->target_dB]);
		break;
	case CMD_TONE_SWEEP_DB:
		if (arg > 0) {
			ctx->DBsweep_frames = arg / ctx->audio_timer_delay_ms;
			ctx->enable_DB_SWEEP = 1;
			printf("Enable sweep DB with %d ms\r\n", ctx->DBsweep_frames * ctx->audio_timer_delay_ms);
		} else {
			ctx->target_dB = 0;
			ctx->DBsweep_frames = 0;
			ctx->enable_DB_SWEEP = 0;
		}
		break;
	case CMD_TONE_SET_PLAY_MODE:
		ctx->playmode = arg;
		if (ctx->playmode == PLAY_SD_DATA) {
			if (ctx->audio_data) {
				fclose(ctx->audio_data);
			}
			ctx->audio_data = fopen((const char *)ctx->sdcard_filename, "rb") ;
			if (!ctx->audio_data) {
				printf("Open SD file %s failed\n\r", ctx->sdcard_filename) ;
				ctx->playmode = PLAY_TONE;
				ctx->audio_data = NULL;
				return -1;
			} else {
				printf("Open SD file %s success\n\r", ctx->sdcard_filename) ;
			}
		} else if (ctx->playmode == PLAY_RAMDISK_DATA) {
			if (ctx->audio_data) {
				fclose(ctx->audio_data);
			}
			extern void fatfs_ram_reset(void);
			fatfs_ram_reset();
			ctx->audio_data = fopen((const char *)ctx->ramdisk_filename, "rb") ;
			if (!ctx->audio_data) {
				printf("Open RAMDISK file %s failed\n\r", ctx->ramdisk_filename) ;
				ctx->playmode = PLAY_TONE;
				ctx->audio_data = NULL;
				return -1;
			} else {
				printf("Open RAMDISK file %s success\n\r", ctx->ramdisk_filename) ;
			}
		} else if (ctx->playmode != PLAY_TONE) {
			ctx->playmode = PLAY_NONE;
			printf("Unkown type set to tone mode\r\n");
		}
		break;
	case CMD_TONE_APPLY:
		for (int i = 0; i < MIN_DBFS; i++) {
			dB_fPOW[i] = pow(10.0f, (float)((float)(-i) / 20.0f));
		}

		if (ctx->playmode == PLAY_RAMDISK_DATA) {
			ctx->audio_data = fopen((const char *)ctx->ramdisk_filename, "rb") ;
			if (!ctx->audio_data) {
				printf("Open RAMDISK file %s failed\n\r", ctx->ramdisk_filename) ;
				ctx->playmode = PLAY_TONE;
			} else {
				printf("Open RAMDISK file %s success\n\r", ctx->ramdisk_filename) ;
			}
		} else if (ctx->playmode == PLAY_SD_DATA) {
			ctx->audio_data = fopen((const char *)ctx->sdcard_filename, "rb") ;
			if (!ctx->audio_data) {
				printf("Open SD file %s failed\n\r", ctx->sdcard_filename) ;
				ctx->playmode = PLAY_TONE;
			} else {
				printf("Open SD file %s success\n\r", ctx->sdcard_filename) ;
			}
		}

		if (ctx->params.codec_id == AV_CODEC_ID_PCM_RAW) {
			ctx->frame_timer_period = (int)(1000000 / ((float)ctx->params.samplerate * 2 / ctx->params.frame_size));
		} else {
			return -1;
		}

		if (ctx->frame_timer_period == 0) {
			printf("Error, frame_timer_period can't be 0\n\r");
			return -1;
		}
#ifdef TIMER_FUNCTION
		gtimer_init(&ctx->frame_timer, 0xff);
#endif
		ctx->audio_timer_delay_ms = ctx->frame_timer_period / 1000;
		break;
	case CMD_TONE_GET_STATE:
		*(int *)arg = ((ctx->stop) ? 0 : 1);
		break;
	case CMD_TONE_STREAMING:
		if (arg == 1) { // stream on
			if (ctx->stop) {
#ifdef TIMER_FUNCTION
				gtimer_start_periodical(&ctx->frame_timer, ctx->frame_timer_period, (void *)toneframe_timer_handler, (uint32_t)ctx);
#else
				if (ctx && ctx->task == NULL) {
					tonetimer_task_enable(ctx);
				}
#endif
				ctx->stop = 0;
				number_frames = 0;
				ctx->tone_data_offset = 0;
				if (ctx->enable_DB_SWEEP) {
					ctx->target_dB = 0;
				}
			}
		} else { // stream off
			if (!ctx->stop) {
#ifdef TIMER_FUNCTION
				gtimer_stop(&ctx->frame_timer);
#endif
				ctx->stop = 1;
			}
		}
		break;
	}
	return 0;
}

int tone_handle(void *ctx, void *input, void *output)
{
	return 0;
}

void *tone_destroy(void *p)
{
	tone_ctx_t *ctx = (tone_ctx_t *)p;

	if (ctx->stop == 0) {
		tone_control((void *)ctx, CMD_TONE_STREAMING, 0);
	}

	if (ctx && ctx->up_sema) {
		rtw_free_sema(&ctx->up_sema);
	}
	if (ctx && ctx->task) {
		vTaskDelete(ctx->task);
	}
	if (ctx) {
		free(ctx);
	}
	return NULL;
}

void *tone_create(void *parent)
{
	tone_ctx_t *ctx = malloc(sizeof(tone_ctx_t));
	if (!ctx) {
		return NULL;
	}
	memset(ctx, 0, sizeof(tone_ctx_t));

	ctx->parent = parent;
	ctx->enable_DB_SWEEP = 0;

	ctx->stop = 1;
	rtw_init_sema(&ctx->up_sema, 0);

	return ctx;

}

void *tone_new_item(void *p)
{
	tone_ctx_t *ctx = (tone_ctx_t *)p;

	return (void *)malloc(ctx->params.frame_size);
}

void *tone_del_item(void *p, void *d)
{
	(void)p;
	if (d) {
		free(d);
	}
	return NULL;
}

mm_module_t tone_module = {
	.create = tone_create,
	.destroy = tone_destroy,
	.control = tone_control,
	.handle = tone_handle,

	.new_item = tone_new_item,
	.del_item = tone_del_item,

	.output_type = MM_TYPE_ASINK | MM_TYPE_ADSP,
	.module_type = MM_TYPE_ASRC,
	.name = "TONE"
};