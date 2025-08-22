#ifndef _MODULE_AFFT_H
#define _MODULE_AFFT_H

#include <stdint.h>
#include "mmf2_module.h"
#define FFT_BK_SIZE        1024
#include "arm_const_structs.h"


#define CMD_AFFT_SET_PARAMS     		MM_MODULE_CMD(0x00)  // set parameter
#define CMD_AFFT_GET_PARAMS     		MM_MODULE_CMD(0x01)  // get parameter
#define CMD_AFFT_SAMPLERATE 			MM_MODULE_CMD(0x02)
#define CMD_AFFT_CHANNEL				MM_MODULE_CMD(0x03)
#define CMD_AFFT_RESET_FFT_RESULT		MM_MODULE_CMD(0x04)
#define CMD_AFFT_SET_OUTPUT				MM_MODULE_CMD(0x05)
#define CMD_AFFT_SHOWN		        	MM_MODULE_CMD(0x0c)

#define CMD_AFFT_APPLY					MM_MODULE_CMD(0x20)  // for hardware module

typedef struct fft_cal_bk_s {
	volatile arm_cfft_instance_f32 *pcfft_instance;  /* ARM CFFT module */
	float max_value;                        /* Max FFT value is stored here */
	float avg_value;                        /* Average FFT value is stored here */
	float real_noise_value;
	float cal_noise_value;
	float s_db;
	float cal_n_db;
	float snr_cal;
	float snr;
	float enob_snr;
	float input_frq;
	float processing_gain;
	float thd;
	float sndr;
	float enob_sndr;
	uint32_t max_index;                          /* Index in Output array where max value is */
	//volatile float window[FFT_SIZE];
	float input[FFT_BK_SIZE * 2]__attribute__((aligned(0x20)));           /* Input[0] = data0_real,   Input[1] = data0_img */
	float output[FFT_BK_SIZE]__attribute__((aligned(0x20)));
	volatile float harmonic[10];
	volatile u32 harmonic_index[10];
} fft_cal_bk_t;

typedef struct afft_param_s {
	uint32_t sample_rate;
	uint32_t channel;
	uint32_t pcm_frame_size;

} afft_params_t;

typedef struct afft_ctx_s {
	void *parent;

	fft_cal_bk_t fft_cal_bk_signal __attribute__((aligned(0x20)));

	afft_params_t params;

	uint8_t *cache;
	uint32_t cache_idx;
	uint32_t stop;
	float accumlated_output[FFT_BK_SIZE];
	uint32_t accumlated_times;
	bool pcm_out_en;
} afft_ctx_t;

extern mm_module_t afft_module;

#endif
