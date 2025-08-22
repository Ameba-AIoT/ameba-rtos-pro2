#ifndef _MODULE_P2P_AAC_H
#define _MODULE_P2P_AAC_H

#include <stdint.h>
#include "mmf2_module.h"

//#include "faac.h"

#define CMD_P2P_AUDIO_SET_PARAMS     		MM_MODULE_CMD(0x00)  // set parameter
#define CMD_P2P_AUDIO_GET_PARAMS     		MM_MODULE_CMD(0x01)  // get parameter
#define CMD_P2P_AUDIO_SAMPLERATE 			MM_MODULE_CMD(0x02)
#define CMD_P2P_AUDIO_CHANNEL				MM_MODULE_CMD(0x03)
#define CMD_P2P_AUDIO_STREAMING				MM_MODULE_CMD(0x04)

#define CMD_P2P_AUDIO_APPLY					MM_MODULE_CMD(0x20)  // for hardware module

typedef struct p2p_audio_param_s {
	uint32_t sample_rate;	// 8000
	uint32_t channel;		// 1

	bool enable_stream;
} p2p_audio_params_t;

typedef struct p2p_audio_ctx_s {
	void *parent;

	p2p_audio_params_t params;

} p2p_audio_ctx_t;

extern mm_module_t p2p_audio_module;

#endif
