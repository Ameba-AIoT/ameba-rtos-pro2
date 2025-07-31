/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include "avcodec.h"

#include "mmf2_module.h"
#include "module_dup.h"

//------------------------------------------------------------------------------
int dup_handle(void *p, void *input, void *output)
{
	dup_ctx_t *ctx = (dup_ctx_t *)p;
	mm_queue_item_t *input_item = (mm_queue_item_t *)input;
	mm_queue_item_t *output_item = (mm_queue_item_t *)output;
	uint8_t *input_buf = (uint8_t *)input_item->data_addr;
	uint8_t *output_buf = (uint8_t *)output_item->data_addr;
	memcpy(output_buf, input_buf, input_item->size);
	output_item->size = input_item->size;
	output_item->timestamp = input_item->timestamp;
	output_item->type = ctx->params.codec_id;
	return output_item->size;
}

int dup_control(void *p, int cmd, int arg)
{
	dup_ctx_t *ctx = (dup_ctx_t *)p;

	switch (cmd) {
	case CMD_DUP_SET_PARAMS:
		memcpy(&ctx->params, ((dup_params_t *)arg), sizeof(dup_params_t));
		break;
	case CMD_DUP_GET_PARAMS:
		memcpy(((dup_params_t *)arg), &ctx->params, sizeof(dup_params_t));
		break;
	case CMD_DUP_CODECID:
		ctx->params.codec_id = arg;
		break;
	case CMD_DUP_LENGTH:
		ctx->params.buf_len = arg;
		break;
	case CMD_DUP_APPLY:
		// nothing to do
		break;
	}

	return 0;
}

void *dup_destroy(void *p)
{
	dup_ctx_t *ctx = (dup_ctx_t *)p;

	if (ctx) {
		free(ctx);
	}

	return NULL;
}

void *dup_create(void *parent)
{
	dup_ctx_t *ctx = (dup_ctx_t *)malloc(sizeof(dup_ctx_t));
	if (!ctx) {
		return NULL;
	}
	memset(ctx, 0, sizeof(dup_ctx_t));
	ctx->parent = parent;

	return ctx;
}

void *dup_new_item(void *p)
{
	dup_ctx_t *ctx = (dup_ctx_t *)p;

	return (void *)malloc(ctx->params.buf_len);
}

void *dup_del_item(void *p, void *d)
{
	(void)p;
	if (d) {
		free(d);
	}
	return NULL;
}


mm_module_t dup_module = {
	.create = dup_create,
	.destroy = dup_destroy,
	.control = dup_control,
	.handle = dup_handle,

	.new_item = dup_new_item,
	.del_item = dup_del_item,

	.output_type = MM_TYPE_ASINK | MM_TYPE_ADSP,
	.module_type = MM_TYPE_ADSP,
	.name = "DUP"
};
