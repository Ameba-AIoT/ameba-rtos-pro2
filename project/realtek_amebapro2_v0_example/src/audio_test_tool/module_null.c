/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/

#include <stdint.h>
#include "platform_stdlib.h"
#include "mmf2_module.h"
#include "module_null.h"
//------------------------------------------------------------------------------

int null_handle(void *p, void *input, void *output)
{
	int ret = 0;
	null_ctx_t *ctx = (null_ctx_t *)p;

	mm_queue_item_t *input_item = (mm_queue_item_t *)input;
	(void)output;

	return ret;
}

int null_control(void *p, int cmd, int arg)
{
	null_ctx_t *ctx = (null_ctx_t *)p;

	switch (cmd) {
	case CMD_NULL_SET_PARAMS:
		//null_params_t *params = (null_params_t *)arg;
		break;
	case CMD_NULL_GET_PARAMS:
		break;
	case CMD_NULL_SET_APPLY:
		break;
	default:
		break;
	}

	return 0;
}

void *null_destroy(void *p)
{
	null_ctx_t *ctx = (null_ctx_t *)p;

	if (ctx) {
		free(ctx);
	}

	return NULL;
}

void *null_create(void *parent)
{
	null_ctx_t *ctx = malloc(sizeof(null_ctx_t));
	if (!ctx) {
		null_destroy((void *)ctx);
		return NULL;
	}
	memset(ctx, 0, sizeof(null_ctx_t));
	ctx->parent = parent;

	return ctx;
}

mm_module_t null_module = {
	.create = null_create,
	.destroy = null_destroy,
	.control = null_control,
	.handle = null_handle,

	.new_item = NULL,
	.del_item = NULL,

	.output_type = MM_TYPE_NONE,     // no output
	.module_type = MM_TYPE_AVSINK,    // module type is video algorithm
	.name = "NULL_MODULE"
};
