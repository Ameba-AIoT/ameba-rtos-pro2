#ifndef _MODULE_P2P_AAC_H
#define _MODULE_P2P_AAC_H

#include <stdint.h>
#include "mmf2_module.h"

#include "faac.h"

#define CMD_P2P_AAC_SET_PARAMS     		MM_MODULE_CMD(0x00)  // set parameter
#define CMD_P2P_AAC_GET_PARAMS     		MM_MODULE_CMD(0x01)  // get parameter
#define CMD_P2P_AAC_SAMPLERATE 		MM_MODULE_CMD(0x02)
#define CMD_P2P_AAC_CHANNEL			MM_MODULE_CMD(0x03)
#define CMD_P2P_AAC_BITLENGTH		MM_MODULE_CMD(0x04)
#define CMD_P2P_AAC_MEMORY_SIZE		MM_MODULE_CMD(0x07)
#define CMD_P2P_AAC_BLOCK_SIZE		MM_MODULE_CMD(0x08)
#define CMD_P2P_AAC_MAX_FRAME_SIZE		MM_MODULE_CMD(0x09)
#define CMD_P2P_AAC_INIT_MEM_POOL		MM_MODULE_CMD(0x0a)

#define CMD_P2P_AAC_APPLY			MM_MODULE_CMD(0x20)  // for hardware module

typedef struct p2p_aac_param_s {
	uint32_t sample_rate;	// 8000
	uint32_t channel;		// 1
	uint32_t bit_length;	// 16
	uint32_t output_format;	// 16
	uint32_t mpeg_version;	// 16

	uint32_t mem_total_size;
	uint32_t mem_block_size;
	uint32_t mem_frame_size;

	int samples_input;
	int max_bytes_output;
	//...
} p2p_aac_params_t;

typedef struct p2p_aac_ctx_s {
	void *parent;

	faacEncHandle faac_enc;

	void *mem_pool;
	p2p_aac_params_t params;

	uint8_t *cache;
	uint32_t cache_idx;
} p2p_aac_ctx_t;

extern mm_module_t p2p_aac_module;

#endif
