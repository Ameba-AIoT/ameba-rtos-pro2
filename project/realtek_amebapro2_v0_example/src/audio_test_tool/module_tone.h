#ifndef _MODULE_TONE_H
#define _MODULE_TONE_H

#include <FreeRTOS.h>
#include <freertos_service.h>
#include <task.h>
#include <stdint.h>
#include "timer_api.h"
#include "vfs.h"

#define DEFAULT_TONE_LEN            96000*2 // total frame size 10s, word length = DEFAULT_TONE_LEN / 2
#define PRE_SWEEP_MS                3000    // pre sweep frames
#define SWEEP_INTERVAL_MS           100     // sweep interval ms 

#define TONE_MODE_ONCE      0
#define TONE_MODE_LOOP      1

#define CMD_TONE_SET_PARAMS         MM_MODULE_CMD(0x00)  // set parameter
#define CMD_TONE_GET_PARAMS         MM_MODULE_CMD(0x01)  // get parameter
#define CMD_TONE_SET_AUDIOTONE      MM_MODULE_CMD(0x02)  // set audio tone rate
#define CMD_TONE_SET_SAMPLERATE     MM_MODULE_CMD(0x03)  // set samplerate
#define CMD_TONE_GET_STATE          MM_MODULE_CMD(0x04)
#define CMD_TONE_STREAMING          MM_MODULE_CMD(0x05)
#define CMD_TONE_RECOUNT_PERIOD     MM_MODULE_CMD(0x06)
#define CMD_TONE_SWEEP_TONE         MM_MODULE_CMD(0x07)
#define CMD_TONE_TARGET_DB          MM_MODULE_CMD(0x08)
#define CMD_TONE_SWEEP_DB           MM_MODULE_CMD(0x09)
#define CMD_TONE_SET_PLAY_MODE      MM_MODULE_CMD(0x10)


#define CMD_TONE_APPLY              MM_MODULE_CMD(0x20)  // for hardware module

#define PLAY_NONE           0x00
#define PLAY_TONE           0x01
#define PLAY_SD_DATA        0x02
#define PLAY_RAMDISK_DATA   0x03

typedef struct tone_param_s {
	uint32_t    codec_id;
	uint8_t     mode;
	uint32_t    channel;

	uint32_t    audiotonerate;
	uint32_t    samplerate;
	uint32_t    sample_bit_length;
	uint32_t    frame_size;

	char ramdisk_tag[32];
	char sdcard_tag[32];
	char audio_filename[96];

} tone_params_t;


typedef struct tone_ctx_s {
	void            *parent;
	TaskHandle_t    task;
	_sema           up_sema;
	gtimer_t        frame_timer;
	uint32_t        frame_timer_period; // us
	uint32_t        audio_timer_delay_ms;
	uint32_t        audio_timer_process_time;
	uint32_t        tone_data_offset;
	uint32_t        pre_sweep_frames; 		//pre sweep frames
	uint32_t        sweep_interval_frames; //number of frames to change sweep frequency
	uint32_t        sweep_frequency;
	uint32_t        sweep_enable;
	int32_t         target_dB;
	uint32_t        DBsweep_frames;
	uint8_t         enable_DB_SWEEP;
	uint8_t         playmode;

	FILE            *audio_data;
	char            ramdisk_filename[160];
	char            sdcard_filename[160];

	tone_params_t   params;
	// flag
	int             stop;
} tone_ctx_t;

extern mm_module_t tone_module;
#endif