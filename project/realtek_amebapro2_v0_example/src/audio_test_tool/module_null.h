#ifndef _MODULE_NULL_H
#define _MODULE_NULL_H

#include "mmf2_module.h"

#define CMD_NULL_SET_PARAMS			MM_MODULE_CMD(0x00)
#define CMD_NULL_GET_PARAMS			MM_MODULE_CMD(0x01)

#define CMD_NULL_SET_APPLY			MM_MODULE_CMD(0x03)

typedef struct null_params_s {
	uint32_t channel;
	uint32_t samplerate;
} null_params_t;

typedef struct null_ctx_s {
	void *parent;

	null_params_t params;
} null_ctx_t;



extern mm_module_t null_module;

#endif