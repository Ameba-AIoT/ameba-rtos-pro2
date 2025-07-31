#ifndef _MODULE_P2P_H
#define _MODULE_P2P_H

#include "mmf2_module.h"

typedef struct p2p_state_s {
	uint32_t timer_1;
	uint32_t timer_2;
	uint32_t drop_frame;
	uint32_t drop_i_frame;
	uint32_t drop_i_frame_size;
	uint32_t drop_p_frame;
	uint32_t drop_p_frame_size;
	uint32_t drop_frame_total;
	uint32_t iframe_total;
	uint32_t frame_total;
	uint32_t iframe_avg;
	uint32_t bitrate_avg;
} p2p_state_t;

typedef struct p2p_ctx_s {
	void *parent;
	p2p_state_t state;
	int (*rate_control_cb)(void *, void *);
} p2p_ctx_t;


extern mm_module_t p2p_module;

#endif