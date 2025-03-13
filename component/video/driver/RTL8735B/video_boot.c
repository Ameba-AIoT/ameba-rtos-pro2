/******************************************************************************
*
* Copyright(c) 2021 - 2025 Realtek Corporation. All rights reserved.
*
******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "platform_stdlib.h"
#include "hal_video.h"
#include "hal_voe.h"
#include "video_boot.h"
#include "diag.h"
#include "rtl8735b_i2c.h"
extern uint8_t __eram_end__[];
extern uint8_t __eram_start__[];
extern int __voe_code_start__[];
static int fcs_flag = 0;//for disable fcs flag
extern video_boot_stream_t video_boot_stream;
static unsigned char video_boot_slot_num[5] = {2, 2, 2, 2, 2};

_WEAK int video_boot_init_sensor_config(void)
{
	return -1;
}

_WEAK int user_load_sensor_boot(void)
{
	return 0;//default not to use
}

extern hal_status_t hal_voe_i2c_init_btldr(u8 idx);
extern void hal_voe_set_kmfw_base_addr(u32 val);
extern void hal_voe_set_kmfw_len(u32 val);
int video_btldr_init_sensor_process(void)
{
	int ret = 0;
	hal_i2c_adapter_t  i2c_master_btldr;
	u8 i2c_idx = 0xff;
	ret = user_load_sensor_boot();
	if (ret <= 0) {
		dbg_printf("It don't do the sensor initial process\r\n");
		return -1;
	} else {
		//dbg_printf("Do the sensor initial process\r\n");
	}

	i2c_master_btldr.pltf_dat.scl_pin = PIN_D12;
	i2c_master_btldr.pltf_dat.sda_pin = PIN_D10;
	i2c_master_btldr.init_dat.index = 3;
	//i2c_slave_addr = 0x30;
	i2c_idx = 3;
	//i2c_addr_len = 2;
	//i2c_data_len = 1;

	hal_i2c_pin_unregister_simple(&i2c_master_btldr);
	hal_i2c_pin_register_simple(&i2c_master_btldr);
	ret = hal_voe_i2c_init_btldr(i2c_idx);
	if (ret != HAL_OK) {
		dbg_printf("hal_voe_i2c_init_btldr fail: %x\n\r", ret);
		hal_i2c_pin_unregister_simple(&i2c_master_btldr);
		return -1;
	} else {
		//dbg_printf("hal_voe_i2c_init_btldr Success\n\r");
	}
	ret = video_boot_init_sensor_config();
	if (ret != 0) {
		dbg_printf("video_boot_init_sensor_config fail\r\n");
		hal_i2c_pin_unregister_simple(&i2c_master_btldr);
	} else {
		//dbg_printf("video_boot_init_sensor_config ok\r\n");
		hal_voe_set_kmfw_base_addr(FCS_RUN_DATA_OK_KM); //set fcs status OK
		hal_voe_set_kmfw_len(0); //clear fcs error_code
	}
	return ret;
}

void video_boot_setup_slot_num(int stream_id, int slot_number)
{
	if ((stream_id >= 0 && stream_id <= 4) && (slot_number >= 2 && slot_number <= 3)) {
		video_boot_slot_num[stream_id] = slot_number;
	} else {
		dbg_printf("The slot number don't support %d %d\r\n", stream_id, slot_number);
	}
}

//Note that heap_4_2.c needs block header (size 0x20) for each block
#define xHeapStructSize 0x20
unsigned int video_boot_malloc(unsigned int size) // alignment 32 byte
{
	unsigned int heap_size = 0;
	unsigned int heap_addr = 0;
	if (size % 32 == 0) {
		heap_size = size;
	} else {
		heap_size = size + (32 - (size % 32));
	}
	//printf("__eram_end__ %x - __eram_start__ %x\r\n",__eram_end__,__eram_start__);
	// allocate the video memory in the end of the heap
	// reverse heapstruct size for the setting of the freertos heap system
	heap_addr = (unsigned int)__eram_end__ - xHeapStructSize - heap_size;
	//printf("heap_addr %x  size %d\r\n", heap_addr, heap_size);
	return heap_addr;
}

void video_boot_set_private_mask(int ch, video_boot_private_mask_t *pmask)
{
	hal_video_reset_mask_status();
	hal_video_set_mask_color(0xff0080);
	for (int id = 0 ; id < PRIVATE_MAX_NUM; id++) {
		if (id == PRIVATE_MASK_GRID) { //GRID MODE
			isp_grid_t grid;
			dbg_printf("[GRID Mode]:\r\n");
			if (pmask->start_x[id] % 2) {
				dbg_printf("[%s] invalid value pmask->start_x=%d", __FUNCTION__, pmask->start_x);
				continue;
			}
			if (pmask->start_y[id] % 2) {
				dbg_printf("[%s] invalid value pmask->start_y=%d", __FUNCTION__, pmask->start_y);
				continue;
			}
			if (pmask->cols % 8) {
				dbg_printf("[%s] invalid value pmask->cols=%d", __FUNCTION__, pmask->cols);
				continue;
			}
			if (pmask->w[id] % 16) {
				int cell_w = pmask->w[id] / pmask->cols;
				if (cell_w % 2) {
					dbg_printf("[%s] invalid value cell_w=%d", __FUNCTION__, cell_w);
					continue;
				}
			}

			if (pmask->en[id] && pmask->cols > 0 && pmask->rows > 0) {
				grid.start_x = pmask->start_x[id];
				grid.start_y = pmask->start_y[id];
				grid.cols = pmask->cols;
				grid.rows = pmask->rows;
				grid.cell_w = pmask->w[id] / grid.cols;
				grid.cell_h = pmask->h[id] / grid.rows;
			}
			hal_video_config_grid_mask(pmask->en[id], grid, (uint8_t *)pmask->bitmap);
		} else { //RECT MODE
			isp_rect_t rect;
			if (pmask->start_x[id] % 2) {
				dbg_printf("[%s] invalid value pmask->start_x=%d", __FUNCTION__, pmask->start_x);
				continue;
			}
			if (pmask->start_y[id] % 2) {
				dbg_printf("[%s] invalid value pmask->start_y=%d", __FUNCTION__, pmask->start_y);
				continue;
			}
			rect.left = pmask->start_x[id];
			rect.top  = pmask->start_y[id];
			rect.right  = pmask->w[id] + rect.left;
			rect.bottom = pmask->h[id] + rect.top;
			hal_video_config_rect_mask(pmask->en[id], id - 1, rect);
		}
	}
	hal_video_fast_enable_mask(ch);
}

int video_boot_buf_calc(video_boot_stream_t vidoe_boot)
{
	int v3dnr_w = 2560;
	int v3dnr_h = 1440;
	int enc_buf_size_len = 0;
	int enc_buf_size = 0;
	int i = 0;

	if (vidoe_boot.isp_info.sensor_width && vidoe_boot.isp_info.sensor_height) {
		v3dnr_w = vidoe_boot.isp_info.sensor_width;
		v3dnr_h = vidoe_boot.isp_info.sensor_height;
	}
	//printf("v3dnr_w %d v3dnr_h %d\r\n", v3dnr_w, v3dnr_h);
	vidoe_boot.voe_heap_size += ((v3dnr_w * v3dnr_h * 3) / 2);

	if (vidoe_boot.isp_info.md_enable) {
		if (vidoe_boot.isp_info.md_buf_size) {
			vidoe_boot.voe_heap_size += vidoe_boot.isp_info.md_buf_size;
		} else {
			vidoe_boot.voe_heap_size += ENABLE_MD_BUF;
		}
	}

	if (vidoe_boot.isp_info.hdr_enable) {
		vidoe_boot.voe_heap_size += ENABLE_HDR_BUF;
	}

	for (i = 0; i < 3; i++) {
		if (vidoe_boot.video_enable[i]) {
			vidoe_boot.voe_heap_size += ((vidoe_boot.video_params[i].width * vidoe_boot.video_params[i].height * 3) / 2) * video_boot_slot_num[i];
			//ISP common
			vidoe_boot.voe_heap_size += ISP_CREATE_BUF;
			//enc ref
			vidoe_boot.voe_heap_size += ((vidoe_boot.video_params[i].width * vidoe_boot.video_params[i].height * 3) / 2) * 2 +
										(vidoe_boot.video_params[i].width * vidoe_boot.video_params[i].height / 16) * 2;
			//enc common
			vidoe_boot.voe_heap_size += ENC_CREATE_BUF;
			//enc buffer
			if (i == 0) {
				enc_buf_size_len = V1_ENC_BUF_SIZE;
			} else if (i == 1) {
				enc_buf_size_len = V2_ENC_BUF_SIZE;
			} else if (i == 2) {
				enc_buf_size_len = V3_ENC_BUF_SIZE;
			}
			enc_buf_size = ((vidoe_boot.video_params[i].width * vidoe_boot.video_params[i].height) / VIDEO_RSVD_DIVISION + (vidoe_boot.video_params[i].bps *
							enc_buf_size_len) / 8);
			vidoe_boot.voe_heap_size += enc_buf_size;
			video_boot_stream.video_params[i].out_buf_size = enc_buf_size;
			//dbg_printf("channel %d size %d\r\n",i,video_boot_stream.video_params[i].out_buf_size);
			//shapshot
			if (vidoe_boot.video_snapshot[i]) {
				vidoe_boot.voe_heap_size += ((vidoe_boot.video_params[i].width * vidoe_boot.video_params[i].height * 3) / 2) + SNAPSHOT_BUF;
			}
			//osd common
			if (vidoe_boot.isp_info.osd_enable) {
				vidoe_boot.voe_heap_size += OSD_CREATE_BUF;
			}
		}
	}
	if (vidoe_boot.video_enable[STREAM_V4]) { //For NN memory
		//ISP buffer
		vidoe_boot.voe_heap_size += vidoe_boot.video_params[i].width * vidoe_boot.video_params[STREAM_V4].height * 3 * video_boot_slot_num[STREAM_ID_V4];
		//ISP common
		vidoe_boot.voe_heap_size += ISP_CREATE_BUF;
	}

	if (vidoe_boot.extra_video_enable) {
		vidoe_boot.voe_heap_size += ((vidoe_boot.extra_video_params.width * vidoe_boot.extra_video_params.height * 3) / 2) * video_boot_slot_num[STREAM_ID_VEXTRA];
		//ISP common
		vidoe_boot.voe_heap_size += ISP_CREATE_BUF;
		//enc ref
		vidoe_boot.voe_heap_size += ((vidoe_boot.extra_video_params.width * vidoe_boot.extra_video_params.height * 3) / 2) * 2 +
									(vidoe_boot.extra_video_params.width * vidoe_boot.extra_video_params.height / 16) * 2;
		//enc common
		vidoe_boot.voe_heap_size += ENC_CREATE_BUF;
		//enc buffer
		enc_buf_size_len = VEXT_ENC_BUF_SIZE;

		enc_buf_size = ((vidoe_boot.extra_video_params.width * vidoe_boot.extra_video_params.height) / VIDEO_RSVD_DIVISION +
						(vidoe_boot.extra_video_params.bps * enc_buf_size_len) / 8);
		vidoe_boot.voe_heap_size += enc_buf_size;
		video_boot_stream.extra_video_params.out_buf_size = enc_buf_size;
		dbg_printf("channel %d size %d\r\n", i, video_boot_stream.extra_video_params.out_buf_size);
		//shapshot
		if (vidoe_boot.extra_video_snapshot) {
			vidoe_boot.voe_heap_size += ((vidoe_boot.extra_video_params.width * vidoe_boot.extra_video_params.height * 3) / 2) + SNAPSHOT_BUF;
		}
	}

	if (vidoe_boot.voe_heap_size % 32 == 0) {
		vidoe_boot.voe_heap_size = vidoe_boot.voe_heap_size;
	} else {
		vidoe_boot.voe_heap_size = vidoe_boot.voe_heap_size + (32 - (vidoe_boot.voe_heap_size % 32));
	}

	return vidoe_boot.voe_heap_size;
}

void video_boot_calcu_meta_size(unsigned int *fcs_meta_size, unsigned int *fcs_meta_extend_size)
{
	int meta_size = 0;
	int meta_loop = 0;
	int enable_extend = 0;
	if (video_boot_stream.extra_fcs_meta_enable_extend) {
		meta_loop = 2;
	} else {
		meta_loop = 1;
	}
	for (int i = 0; i < meta_loop; i++) {
		if (i == 1 && video_boot_stream.extra_fcs_meta_enable_extend) {
			enable_extend = 1;//Insert the extend meta size
		}
		if (enable_extend) {
			meta_size = video_boot_stream.meta_size + sizeof(isp_meta_t) + sizeof(isp_statis_meta_t) + sizeof(af_statis_t) + sizeof(ae_statis_t) + sizeof(awb_statis_t);
		} else {
			meta_size = video_boot_stream.meta_size + sizeof(isp_meta_t) + sizeof(isp_statis_meta_t);
		}
		meta_size = meta_size + meta_size / 2; //Add the extra buffer to dummy bytes
		if (meta_size % 32) { //align 32 byte
			meta_size = meta_size + (32 - (meta_size % 32));
		}
		if (enable_extend) {
			video_boot_stream.extra_fcs_meta_extend_offset = meta_size / 0xff;
			video_boot_stream.extra_fcs_meta_extend_total_size = meta_size;
			if (video_boot_stream.extra_fcs_meta_extend_total_size > VIDEO_BOOT_META_REV_BUF) {
				dbg_printf("Meta size %d is exceed the sei buffer %d\r\n", meta_size, VIDEO_BOOT_META_REV_BUF);
				video_boot_stream.extra_fcs_meta_extend_total_size = VIDEO_BOOT_META_REV_BUF;
				//v_adp->cmd[ch]->userData = VIDEO_BOOT_META_REV_BUF;
				dbg_printf("Setup the meta size as %d\r\n", VIDEO_BOOT_META_REV_BUF);
			}
		} else {
			video_boot_stream.fcs_meta_offset = meta_size / 0xff;
			video_boot_stream.fcs_meta_total_size = meta_size;
			if (video_boot_stream.fcs_meta_total_size > VIDEO_BOOT_META_REV_BUF) {
				dbg_printf("Meta size %d is exceed the sei buffer %d\r\n", meta_size, VIDEO_BOOT_META_REV_BUF);
				video_boot_stream.fcs_meta_total_size = VIDEO_BOOT_META_REV_BUF;
				//v_adp->cmd[ch]->userData = VIDEO_BOOT_META_REV_BUF;
				dbg_printf("Setup the meta size as %d\r\n", VIDEO_BOOT_META_REV_BUF);
			}
		}
	}
	*fcs_meta_size = video_boot_stream.fcs_meta_total_size;
	*fcs_meta_extend_size = video_boot_stream.extra_fcs_meta_extend_total_size;
}

int video_boot_open(int ch_index, video_boot_params_t *v_stream)
{
	int ch = v_stream->stream_id;
	int fcs_v = v_stream->fcs;
	int isp_fps = video_boot_stream.isp_info.sensor_fps;
	int fps = 30;
	int gop = 30;
	int rcMode = 1;
	int bps = 2 * 1024 * 1024;
	int minQp = 0;
	int maxQp = 51;
	int rotation = 0;
	int jpeg_qlevel = 5;
	int type;
	int res = 0;
	int codec = 0;
	int ret = OK;

	int enc_in_w = (v_stream->width + 15) & ~15;  //force 16 aligned
	int enc_in_h = v_stream->height;
	int enc_out_w = v_stream->width;  //will crop enc_in_w to enc_out_w
	int enc_out_h = v_stream->height;
	int enc_out_w_offset = (enc_in_w - enc_out_w) / 2;

	int out_rsvd_size = (enc_in_w * enc_in_h) / VIDEO_RSVD_DIVISION;
	int out_buf_size = 0;
	int jpeg_out_buf_size = out_rsvd_size * 3;
	switch (ch) {
	case 0:
		out_buf_size = (v_stream->bps * V1_ENC_BUF_SIZE) / 8 + out_rsvd_size;
		break;
	case 1:
		out_buf_size = (v_stream->bps * V2_ENC_BUF_SIZE) / 8 + out_rsvd_size;
		break;
	case 2:
		out_buf_size = (v_stream->bps * V3_ENC_BUF_SIZE) / 8 + out_rsvd_size;
		break;
	case 3:
		out_buf_size = (v_stream->bps * VEXT_ENC_BUF_SIZE) / 8 + out_rsvd_size;
		break;
	}

	bps = v_stream->bps;

	if (v_stream->rc_mode) {
		rcMode = v_stream->rc_mode - 1;
		if (rcMode) {
			minQp = 25;
			maxQp = 48;
			bps = v_stream->bps / 2;
		}
	}

	if (v_stream->minQp > 0 && v_stream->minQp <= 51) {
		minQp = v_stream->minQp;
	}
	if (v_stream->maxQp > 0 && v_stream->maxQp <= 51) {
		maxQp = v_stream->maxQp;
	}

	hal_video_adapter_t *v_adp = hal_video_get_adp();
	v_adp->cmd[ch]->lumWidthSrc = enc_in_w;
	v_adp->cmd[ch]->lumHeightSrc = enc_in_h;
	v_adp->cmd[ch]->width = enc_out_w;
	v_adp->cmd[ch]->height = enc_out_h;
	v_adp->cmd[ch]->horOffsetSrc = enc_out_w_offset;
	v_adp->cmd[ch]->rotation = v_stream->rotation;
	v_adp->cmd[ch]->bitPerSecond = bps;//video_boot_stream.video_params[ch].bps;
	v_adp->cmd[ch]->qpMin = minQp;
	v_adp->cmd[ch]->qpMax = maxQp;

	v_adp->cmd[ch]->vbr = rcMode;

	v_adp->cmd[ch]->CodecType = v_stream->type;
	v_adp->cmd[ch]->fcs = 1;
	v_adp->cmd[ch]->EncMode = MODE_QUEUE;
	v_adp->cmd[ch]->voe_dbg = BOOTLOADER_VOE_LOG_EN;

	if (v_stream->type == CODEC_HEVC) {
		v_adp->cmd[ch]->outputFormat     = VCENC_VIDEO_CODEC_HEVC,
						v_adp->cmd[ch]->max_cu_size      = 64;
		v_adp->cmd[ch]->min_cu_size      = 8;
		v_adp->cmd[ch]->max_tr_size      = 16;
		v_adp->cmd[ch]->min_tr_size      = 4;
		v_adp->cmd[ch]->tr_depth_intra   = 2; 							 //mfu =>0
		v_adp->cmd[ch]->tr_depth_inter   = 4;							// (.max_cu_size == 64) ? 4 : 3,
		v_adp->cmd[ch]->level            = VCENC_HEVC_LEVEL_6;
		v_adp->cmd[ch]->profile          = VCENC_HEVC_MAIN_PROFILE;	// default is HEVC MAIN profile
		v_adp->cmd[ch]->outputRateNumer = v_stream->fps;
		v_adp->cmd[ch]->inputRateNumer = isp_fps;
		v_adp->cmd[ch]->intraPicRate = v_stream->gop;
	} else if (v_stream->type == CODEC_NV12) {
		v_adp->cmd[ch]->EncMode = 0;
		v_adp->cmd[ch]->outputFormat     = VCENC_VIDEO_CODEC_NV12;
		v_adp->cmd[ch]->YuvMode 		 = MODE_QUEUE;
		v_adp->cmd[ch]->inputRateNumer = v_stream->fps;
	} else if (v_stream->type == CODEC_RGB) {
		v_adp->cmd[ch]->EncMode = 0;
		v_adp->cmd[ch]->outputFormat     = VCENC_VIDEO_CODEC_RGB;
		v_adp->cmd[ch]->YuvMode 		 = MODE_QUEUE;
		v_adp->cmd[ch]->inputRateNumer = v_stream->fps;
	} else {
		v_adp->cmd[ch]->outputFormat     = VCENC_VIDEO_CODEC_H264;
		v_adp->cmd[ch]->max_cu_size      = 16;
		v_adp->cmd[ch]->min_cu_size      = 8;
		v_adp->cmd[ch]->max_tr_size      = 16;
		v_adp->cmd[ch]->min_tr_size      = 4;
		v_adp->cmd[ch]->tr_depth_intra   = 1;
		v_adp->cmd[ch]->tr_depth_inter   = 2;
		v_adp->cmd[ch]->level            = VCENC_H264_LEVEL_5_1;
		v_adp->cmd[ch]->profile          = VCENC_H264_HIGH_PROFILE;	// default is HEVC HIGH profile
		v_adp->cmd[ch]->outputRateNumer = v_stream->fps;
		v_adp->cmd[ch]->inputRateNumer = isp_fps;
		v_adp->cmd[ch]->intraPicRate = v_stream->gop;
	}
	v_adp->cmd[ch]->byteStream = VCENC_BYTE_STREAM;
	v_adp->cmd[ch]->ch = v_stream->stream_id;
	if (video_boot_stream.isp_info.osd_enable) {
		v_adp->cmd[ch]->osd = 1;
	}

	bool isNormalSnapshotEn = (ch != 3) && video_boot_stream.video_snapshot[ch_index];
	bool isExtraSnapshotEn = (ch == 3) && video_boot_stream.extra_video_snapshot;
	if (isNormalSnapshotEn || isExtraSnapshotEn) {
		v_adp->cmd[ch]->CodecType = v_stream->type | CODEC_JPEG;
		v_adp->cmd[ch]->JpegMode = MODE_SNAPSHOT;
		//Disable the ring buffer with JPEG SNAPSHOT. Setup the jpg_buf_size and jpg_rsvd_size as the same size
		v_adp->cmd[ch]->jpg_buf_size = v_stream->width * v_stream->height * 3 / 2;
		v_adp->cmd[ch]->jpg_rsvd_size = v_stream->width * v_stream->height * 3 / 2;
		v_adp->cmd[ch]->qLevel = v_stream->jpeg_qlevel;
	}

	if (video_boot_stream.fcs_channel == 1 && ch != 0) { //Disable the fcs for channel 1
		v_adp->cmd[0]->fcs = 0;//Remove the fcs setup for channel one
		dcache_clean_invalidate_by_addr((uint32_t *)v_adp->cmd[0], sizeof(commandLine_s));
	}
	v_adp->cmd[ch]->out_buf_size  = out_buf_size;
	v_adp->cmd[ch]->out_rsvd_size = out_rsvd_size;
	v_adp->cmd[ch]->isp_buf_num = video_boot_slot_num[ch];

	v_adp->cmd[ch]->direct_i2c_mode = 1;
	if (video_boot_stream.fcs_isp_ae_enable) {
		v_adp->cmd[ch]->set_AE_init_flag = 1;
		v_adp->cmd[ch]->all_init_iq_set_flag = 1;
		v_adp->cmd[ch]->init_exposure = video_boot_stream.fcs_isp_ae_init_exposure;
		v_adp->cmd[ch]->init_gain = video_boot_stream.fcs_isp_ae_init_gain;
	}
	if (video_boot_stream.fcs_isp_awb_enable) {
		v_adp->cmd[ch]->set_AWB_init_flag = 1;
		v_adp->cmd[ch]->all_init_iq_set_flag = 1;
		v_adp->cmd[ch]->init_r_gain = video_boot_stream.fcs_isp_awb_init_rgain;
		v_adp->cmd[ch]->init_b_gain = video_boot_stream.fcs_isp_awb_init_bgain;
	}

	if (video_boot_stream.fcs_isp_init_daynight_mode) {
		v_adp->cmd[ch]->all_init_iq_set_flag = 1;
		v_adp->cmd[ch]->init_daynight_mode = video_boot_stream.fcs_isp_init_daynight_mode;
	}

	if (video_boot_stream.fcs_isp_gray_mode) {
		v_adp->cmd[ch]->all_init_iq_set_flag = 1;
		v_adp->cmd[ch]->gray_mode = video_boot_stream.fcs_isp_gray_mode;
	}

	int dropFrameNum = (ch == 3) ? video_boot_stream.extra_video_drop_frame : video_boot_stream.video_drop_frame[ch_index];

	if (dropFrameNum) {
		v_adp->cmd[ch]->all_init_iq_set_flag = 1;
		v_adp->cmd[ch]->drop_frame_num = dropFrameNum;
	}

	if (video_boot_stream.voe_scale_up_en == 0) {
		int origin_width = (int)video_boot_stream.isp_info.sensor_width;
		int origin_height = (int)video_boot_stream.isp_info.sensor_height;
		if (v_stream->use_roi) {
			origin_width = v_stream->roi.xmax - v_stream->roi.xmin;
			origin_height = v_stream->roi.ymax - v_stream->roi.ymin;
		}
		if (origin_width <= 0 || origin_height <= 0) {
			dbg_printf("error: invlalid input resolution\r\n");
			ret = NOK;
			goto EXIT;
		}
		if (enc_in_w <= origin_width && enc_in_h <= origin_height) { //scale down
			if (v_stream->use_roi) {
				//dbg_printf("scale down ch%d set_roi (%d,%d,%d,%d)\r\n", ch, v_stream->roi.xmin, v_stream->roi.ymin, origin_width, origin_height);
				hal_video_isp_set_roi(ch, v_stream->roi.xmin, v_stream->roi.ymin, origin_width, origin_height);
			}
		} else if (enc_in_w >= origin_width && enc_in_h >= origin_height) {//scale up
			if (ch != 0) {
				dbg_printf("error: scale up only support ch0\r\n");
				ret = NOK;
				goto EXIT;
			} else if ((enc_in_w >= origin_width * 2) || (enc_in_h >= origin_height * 2)) {
				dbg_printf("error: Encoder width and height should both be less than roi_w/h multi two.\r\n");
				ret = NOK;
				goto EXIT;
			} else if (enc_in_w > 2688 || enc_in_h > 1944) {
				dbg_printf("error: max scale up resolution is 2688x1944 \r\n");
				ret = NOK;
				goto EXIT;
			} else {
				video_boot_stream.voe_scale_up_en = 1;
				hal_video_isp_clk_set(ch, 5, 0);
				if (v_stream->use_roi) {
					//dbg_printf("scale up ch%d set_roi (%d,%d,%d,%d)\r\n", ch, v_stream->roi.xmin, v_stream->roi.ymin, origin_width, origin_height);
					hal_video_isp_set_roi(ch, v_stream->roi.xmin, v_stream->roi.ymin, origin_width, origin_height);
					memcpy(&(video_boot_stream.voe_scale_up_roi), &(v_stream->roi), sizeof(video_boot_roi_t));
				}
			}
		} else {
			dbg_printf("error: invalid resolution %dx%d -> %dx%d\r\n", origin_width, origin_height, enc_in_w, enc_in_h);
			ret = NOK;
			goto EXIT;
		}
	} else if (v_stream->use_roi) {
		if (ch != 0) {
			dbg_printf("It don't support to setup the ROI when scale up is enable\r\n");
			ret = NOK;
			goto EXIT;
		}
	}

	if (v_stream->level) {
		v_adp->cmd[ch]->level = v_stream->level;
	}
	if (v_stream->profile) {
		v_adp->cmd[ch]->profile = v_stream->profile;
	}
	if (v_stream->cavlc) {
		v_adp->cmd[ch]->enableCabac = 0;
	}

	if (video_boot_stream.private_mask.enable) {
		video_boot_set_private_mask(ch, &video_boot_stream.private_mask);
	}
	if (video_boot_stream.meta_enable) {
#if 0
		int meta_size = video_boot_stream.meta_size + sizeof(isp_meta_t) + sizeof(isp_statis_meta_t);

		meta_size = meta_size + meta_size / 4; //Add the extra buffer to dummy bytes
		if (meta_size % 32) { //align 32 byte
			meta_size = meta_size + (32 - (meta_size % 32));
		}
		video_boot_stream.fcs_meta_offset = meta_size / 0xff;
		video_boot_stream.fcs_meta_total_size = meta_size;
		v_adp->cmd[ch]->isp_meta_out = 1;
		v_adp->cmd[ch]->EncuserData = meta_size;
		v_adp->cmd[ch]->IDRuserData = meta_size;
		v_adp->cmd[ch]->IDRuserDataDuration = 1;
#else
		unsigned int fcs_meta_size;
		unsigned int fcs_meta_extend_size;
		video_boot_calcu_meta_size(&fcs_meta_size, &fcs_meta_extend_size);
		v_adp->cmd[ch]->isp_meta_out = 1;
		v_adp->cmd[ch]->EncuserData = fcs_meta_size;
		if (video_boot_stream.extra_fcs_meta_enable_extend) {
			v_adp->cmd[ch]->IDRuserData = fcs_meta_extend_size;
		} else {
			v_adp->cmd[ch]->EncuserData = fcs_meta_size;
		}
		v_adp->cmd[ch]->IDRuserDataDuration = 1;
#endif
		if (isNormalSnapshotEn || isExtraSnapshotEn) {
			v_adp->cmd[ch]->JPGuserData = video_boot_stream.fcs_meta_total_size;
		}
		if (v_adp->cmd[ch]->EncuserData > VIDEO_BOOT_META_REV_BUF) {
			dbg_printf("Meta size %d is exceed the sei buffer %d\r\n", v_adp->cmd[ch]->EncuserData, VIDEO_BOOT_META_REV_BUF);
			v_adp->cmd[ch]->EncuserData = VIDEO_BOOT_META_REV_BUF;
			dbg_printf("Setup the meta size as %d\r\n", VIDEO_BOOT_META_REV_BUF);
		}
	}
	if (video_boot_stream.init_isp_items.enable) {
		video_isp_initial_items_t init_items;
		v_adp->cmd[ch]->all_init_iq_set_flag = 1;
		init_items.init_brightness = video_boot_stream.init_isp_items.init_brightness;
		init_items.init_contrast = video_boot_stream.init_isp_items.init_contrast;
		init_items.init_flicker = video_boot_stream.init_isp_items.init_flicker;
		init_items.init_hdr_mode = video_boot_stream.init_isp_items.init_hdr_mode;;
		init_items.init_mirrorflip = video_boot_stream.init_isp_items.init_mirrorflip;
		init_items.init_saturation = video_boot_stream.init_isp_items.init_saturation;
		init_items.init_wdr_level = video_boot_stream.init_isp_items.init_wdr_level;
		init_items.init_wdr_mode = video_boot_stream.init_isp_items.init_wdr_mode;
		init_items.init_mipi_mode = video_boot_stream.init_isp_items.init_mipi_mode;
		hal_video_set_isp_init_items(ch, &init_items);
	} else {
		video_isp_initial_items_t init_items;
		v_adp->cmd[ch]->all_init_iq_set_flag = 1;
		init_items.init_brightness = 0;
		init_items.init_contrast = 50;
		init_items.init_flicker = 1;
		init_items.init_hdr_mode = 0;
		init_items.init_mirrorflip = 0xf0;
		init_items.init_saturation = 50;
		init_items.init_wdr_level = 50;
		init_items.init_wdr_mode = 2;
		init_items.init_mipi_mode = 0;
		hal_video_set_isp_init_items(ch, &init_items);
	}

	if (v_stream->fcs_vui_disable) {
		v_adp->cmd[ch]->vui_timing_info_enable = 0;
	}

	/* encoder option should br aligned with paramter_table[] in video_api.c */
	if (v_stream->type == CODEC_HEVC || v_stream->type == CODEC_H264) {
		v_adp->cmd[ch]->gopSize = 1;
		v_adp->cmd[ch]->picRc = 1;
		v_adp->cmd[ch]->ctbRc = 1;
		v_adp->cmd[ch]->qpHdr = -1;
		v_adp->cmd[ch]->intraQpDelta = -5;
		v_adp->cmd[ch]->smoothPsnrInGOP = 1;
		v_adp->cmd[ch]->rcQpDeltaRange = 15;
		v_adp->cmd[ch]->picQpDeltaMin = -4;
		v_adp->cmd[ch]->picQpDeltaMax = 6;
		v_adp->cmd[ch]->blockRCSize = 1;
		v_adp->cmd[ch]->compressor = 3; /* enable both luma and chroma compression to save DDR bandwidth */
	}

	dcache_clean_invalidate_by_addr((uint32_t *)v_adp->cmd[ch], sizeof(commandLine_s));

	//v_stream->out_buf_size = out_buf_size;
	//v_stream->out_rsvd_size = out_rsvd_size;
EXIT:
	return ret;
}

/*** KM BOOT LOADER handling ***/
#define WAIT_FCS_DONE_TIMEOUT 	1000000

_WEAK void user_boot_config_init(void *parm)
{

}

_WEAK int user_disable_fcs(void)
{
	return 0;//default not to use
}

static void set_fcs_boottime_information(void)
{
	video_boot_stream.fcs_start_time = hal_read_cur_time() / 1000; // get the time from booloader to fcs OK
	(* ((volatile uint32_t *) 0xe000edfc)) |= (1 << 24);    // DEMCR, bit 24 TRCENA
	(* ((volatile uint32_t *) 0xe0001004)) = 0;             // DWT_CYCCNT
	(* ((volatile uint32_t *) 0xe0001000)) |= 1;            // DWT_CTRL, bit 0 CYCCNTENA
}

extern uint8_t bl4voe_shared_test[];
int video_btldr_process(voe_fcs_load_ctrl_t *pvoe_fcs_ld_ctrl, int *code_start)
{
	int ret = OK;
	unsigned int addr = 0;
	uint8_t *p_fcs_data = NULL, *p_iq_data = NULL, *p_sensor_data = NULL;
	uint8_t fcs_id;
	voe_cpy_t isp_cpy = NULL;
	isp_multi_fcs_ld_info_t *p_fcs_ld_info = NULL;
	int i = 0;
	int video_boot_struct_size = 0;

	if (NULL == pvoe_fcs_ld_ctrl) {
		dbg_printf("voe FCS ld ctrl is NULL \n");
		ret = FCS_CPY_FUNC_ERR;
		return ret;
	} else {
		isp_cpy       = pvoe_fcs_ld_ctrl->isp_cpy;
		p_fcs_ld_info = pvoe_fcs_ld_ctrl->p_fcs_ld_info;
	}

	fcs_id = p_fcs_ld_info->fcs_id;
	p_fcs_data    = (uint8_t *)((p_fcs_ld_info->fcs_hdr_start) + (p_fcs_ld_info->sensor_set[fcs_id].fcs_data_offset));
	p_iq_data     = (uint8_t *)(p_fcs_data + (p_fcs_ld_info->sensor_set[fcs_id].iq_start_addr));
	p_sensor_data = (uint8_t *)(p_fcs_data + (p_fcs_ld_info->sensor_set[fcs_id].sensor_start_addr));
	// this api is use to keep the boot time information
	set_fcs_boottime_information();
	if (hal_voe_fcs_check_OK()) {
		user_boot_config_init(pvoe_fcs_ld_ctrl->p_fcs_para_raw);
		if ((video_boot_stream.fcs_isp_iq_id != 0) && (video_boot_stream.fcs_isp_iq_id < p_fcs_ld_info->multi_fcs_cnt)) {
			p_fcs_data    = (uint8_t *)((p_fcs_ld_info->fcs_hdr_start) + (p_fcs_ld_info->sensor_set[video_boot_stream.fcs_isp_iq_id].fcs_data_offset));
			p_iq_data     = (uint8_t *)(p_fcs_data + (p_fcs_ld_info->sensor_set[video_boot_stream.fcs_isp_iq_id].iq_start_addr));
			p_sensor_data = (uint8_t *)(p_fcs_data + (p_fcs_ld_info->sensor_set[video_boot_stream.fcs_isp_iq_id].sensor_start_addr));
		}
		hal_video_load_iq((voe_cpy_t)isp_cpy, (int *) p_iq_data, (int *) __voe_code_start__);
		hal_video_load_sensor((voe_cpy_t)isp_cpy, (int *) p_sensor_data, (int *) __voe_code_start__);
		int fcs_ch = -1;
		for (i = 0; i < MAX_FCS_CHANNEL; i++) {
			if (video_boot_stream.video_params[i].fcs) {
				if (video_boot_open(i, &video_boot_stream.video_params[i]) != OK) {
					dbg_printf("error: ch%d open fail\r\n", video_boot_stream.video_params[i].stream_id);
					return NOK;
				}
			}
			if ((fcs_ch == -1) && (video_boot_stream.video_params[i].fcs == 1)) { //Get the first start channel
				fcs_ch = i;
			}
		}
		if (video_boot_stream.extra_video_params.fcs) {
			if (video_boot_open(MAX_FCS_CHANNEL, &video_boot_stream.extra_video_params) != OK) {
				dbg_printf("error: ch%d open fail\r\n", video_boot_stream.extra_video_params.stream_id);
				return NOK;
			}
		}
		if ((fcs_ch == -1) && (video_boot_stream.extra_video_params.fcs == 1)) { //Get the first start channel
			fcs_ch = video_boot_stream.extra_video_params.stream_id;
		}
		__DSB();

		pvoe_fcs_peri_info_t fcs_peri_info_for_ram = pvoe_fcs_ld_ctrl->p_fcs_peri_info;


		if (fcs_peri_info_for_ram->i2c_id <= 3) {
			hal_video_isp_set_i2c_id(fcs_ch, fcs_peri_info_for_ram->i2c_id);
		}

		if (fcs_peri_info_for_ram->fcs_data_verion == 0x1) {
			if (fcs_peri_info_for_ram->gpio_cnt > 3) {
				hal_video_isp_set_sensor_gpio(fcs_ch, fcs_peri_info_for_ram->gpio_list[3], fcs_peri_info_for_ram->gpio_list[2], fcs_peri_info_for_ram->gpio_list[0]);
				hal_pinmux_unregister(fcs_peri_info_for_ram->gpio_list[1], PID_GPIO);
				//dbg_printf("snr_pwr 0x%02x pwdn 0x%02x rst 0x%02x i2c_id %d \n", fcs_peri_info_for_ram->gpio_list[0], fcs_peri_info_for_ram->gpio_list[2], fcs_peri_info_for_ram->gpio_list[3], fcs_peri_info_for_ram->i2c_id);
			}
		} else {
			if (fcs_peri_info_for_ram->gpio_cnt > 2) {
				hal_video_isp_set_sensor_gpio(fcs_ch, fcs_peri_info_for_ram->gpio_list[1], fcs_peri_info_for_ram->gpio_list[2], fcs_peri_info_for_ram->gpio_list[0]);
				//dbg_printf("snr_pwr 0x%02x pwdn 0x%02x rst 0x%02x i2c_id %d \n", fcs_peri_info_for_ram->gpio_list[0], fcs_peri_info_for_ram->gpio_list[2], fcs_peri_info_for_ram->gpio_list[1], fcs_peri_info_for_ram->i2c_id);

			}
		}

		int voe_heap_size = video_boot_buf_calc(video_boot_stream);
		addr = video_boot_malloc(voe_heap_size);
		video_boot_stream.voe_heap_addr = addr;
		video_boot_stream.voe_heap_size = voe_heap_size;
		if (video_boot_stream.fcs_start_time) { //Measure the fcs time
			video_boot_stream.fcs_voe_time = (* ((volatile uint32_t *) 0xe0001004)) / (500000) + video_boot_stream.fcs_start_time;
		}
		ret = hal_video_init((long unsigned int *)addr, voe_heap_size);
		video_boot_stream.fcs_status = 1;
	}
	video_boot_stream.fcs_voe_fw_addr = (int)pvoe_fcs_ld_ctrl->fw_addr;
	memcpy(&video_boot_stream.p_fcs_ld_info, p_fcs_ld_info, sizeof(isp_multi_fcs_ld_info_t));
	video_boot_struct_size = sizeof(video_boot_stream_t);
	if (fcs_flag == 1 && video_boot_stream.fcs_status == 0) {
		video_boot_stream.p_fcs_ld_info.fcs_id = 0;
	}
	if (video_boot_struct_size <= VIDEO_BOOT_STRUCT_MAX_SIZE) {
		memcpy(bl4voe_shared_test, &video_boot_stream, sizeof(video_boot_stream_t));
	} else {
		memcpy(bl4voe_shared_test, &video_boot_stream, VIDEO_BOOT_STRUCT_MAX_SIZE);
	}
	return ret;
}

int video_btldr_fcs_terminated(voe_fcs_load_ctrl_t *pvoe_fcs_ld_ctrl)
{

	pvoe_fcs_peri_info_t fcs_peri_info_for_ram = pvoe_fcs_ld_ctrl->p_fcs_peri_info;
	hal_gpio_adapter_t sensor_en_gpio;
	volatile hal_i2c_adapter_t	i2c_master_video;
	hal_status_t ret = 0;

	int terminated_flag = user_disable_fcs();  // Application set terminate or not
	fcs_flag = terminated_flag;

	if (terminated_flag) {
		// disable I2C
		i2c_master_video.pltf_dat.scl_pin = fcs_peri_info_for_ram->i2c_scl;
		i2c_master_video.pltf_dat.sda_pin = fcs_peri_info_for_ram->i2c_sda;
		i2c_master_video.init_dat.index = fcs_peri_info_for_ram->i2c_id;
		ret = hal_i2c_pin_unregister_simple(&i2c_master_video);
		if (ret) {
			dbg_printf("[FCS Terminate] hal_i2c_pin_unregister_simple failed %d \n", ret);
		}

		if (fcs_peri_info_for_ram->gpio_cnt > 0) {

			// unregister pwr_ctrl_pin
			ret = hal_pinmux_unregister(fcs_peri_info_for_ram->gpio_list[0], PID_GPIO);
			if (ret) {
				dbg_printf("[FCS Terminate] hal_pinmux_unregister pwr_ctrl pin failed %d \n", ret);
			} else {
				// turn off sensor power
				ret = hal_gpio_init(&sensor_en_gpio, fcs_peri_info_for_ram->gpio_list[0]);
				if (ret) {
					dbg_printf("[FCS Terminate] hal_gpio_init pwr_ctrl pin failed %d \n", ret);
				} else {
					hal_gpio_set_dir(&sensor_en_gpio, GPIO_OUT);
					hal_gpio_write(&sensor_en_gpio, 0);
					hal_gpio_deinit(&sensor_en_gpio);
				}

			}
			// unregister GPIO
			for (int i = 1;  i < fcs_peri_info_for_ram->gpio_cnt; i++) {
				ret = hal_pinmux_unregister(fcs_peri_info_for_ram->gpio_list[i], PID_GPIO);
				if (ret) {
					dbg_printf("[FCS Terminate] hal_pinmux_unregister GPIO[%d] failed %d \n", i, ret);
				}
			}
			ret = hal_pinmux_unregister(fcs_peri_info_for_ram->snr_clk_pin, PID_SENSOR);
			if (ret) {
				dbg_printf("[FCS Terminate] hal_pinmux_unregister snr_clk_pin failed %d \n", ret);
			}
		}

		fcs_peri_info_for_ram->fcs_OK = 0;
		hal_voe_set_kmfw_base_addr(FCS_RUN_DATA_NG_KM);

	}
	return 0;
}
