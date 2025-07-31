/******************************************************************************
*
* Copyright(c) 2007 - 2021 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "platform_opts.h"
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "module_video.h"
#include "video_api.h"
#include "fwfs.h"
#include "vfs.h"
#include "sensor.h"
#include "ai_glass_media.h"
#include "module_filesaver.h"
#include "nv12tojpg.h"
#include "media_filesystem.h"
#include "ai_glass_dbg.h"
#include "ai_glass_media.h"
#include "isp_ctrl_api.h"

// Definition of LIFE SNAPSHOT STATUS
#define LIFESNAP_IDLE   0x00
#define LIFESNAP_START  0x01
#define LIFESNAP_TAKE   0x02
#define LIFESNAP_GET    0x03
#define LIFESNAP_DONE   0x04
#define LIFESNAP_FAIL   0x05

// Configure for lifetime snapshot
#define JPG_WRITE_SIZE          4096
#define LIFE_SNAP_PRIORITY      5
#define MAXIMUM_FILE_TAG_SIZE   32
#define MAXIMUM_FILE_SIZE       (MAXIMUM_FILE_TAG_SIZE + 32)
#define SNAPSHOT_12M_QLEVEL     91 // can be 0~100, higher means higher quality

static uint32_t nv16_take_time = 0;
static uint32_t nv12_gen_time = 0;
static uint32_t jpeg_enc_time = 0;
static uint32_t emmc_save_time = 0;

static char snapshot_name[MAXIMUM_FILE_SIZE];

typedef struct {
	uint8_t         need_sw_encode;
	uint8_t         is_high_res;
	uint32_t        jpg_width;
	uint32_t        jpg_height;
	uint32_t        jpg_qlevel;
	video_params_t  params;
} jpeg_lifesnapshot_params_t;

//Modules
//ls means lifetime snapshot
static jpeg_lifesnapshot_params_t ls_video_params = {0};
static mm_context_t *ls_snapshot_ctx    = NULL;
static mm_context_t *ls_filesaver_ctx   = NULL;

//linker
static mm_siso_t *ls_siso_snapshot_filesaver = NULL;

static int lfsnap_status = LIFESNAP_IDLE;

#define JPEG_CHANNEL 0
#include "sensor_service.h"
#include "isp_ctrl_api.h"
typedef struct {
	void *virt_addr;
	void *phy_addr;
} rtscam_dma_item_t;
static int get_raw_data = 0;
static uint32_t raw_image_len = 0;
static int image_count = 0;
static uint8_t *hr_nv12_image = NULL;
static rtscam_dma_item_t dma_left, dma_right;
static int x_overlap = 16;
static video_pre_init_params_t init_params = {0};

// Hardware 12M snapshot
static uint8_t *jpeg_nv12_addr = NULL;
static uint32_t jpeg_nv12_len = 0;
static char *file_save_path = NULL;
static SemaphoreHandle_t jpeg_get_sema = NULL;
static int jpeg_encode_done_cb(uint32_t jpeg_addr, uint32_t jpeg_len)
{
	jpeg_nv12_addr = (uint8_t *) jpeg_addr;
	jpeg_nv12_len = jpeg_len;
	FILE *life_snapshot_file = extdisk_fopen(file_save_path, "wb");
	if (!life_snapshot_file) {
		AI_GLASS_ERR("open jpg file %s fail\r\n", file_save_path);
		lfsnap_status = LIFESNAP_FAIL;
		xSemaphoreGive(jpeg_get_sema);
		return 0;
	}
	AI_GLASS_MSG("get liftime snapshot frame encode done time %lu\r\n", mm_read_mediatime_ms());
	jpeg_enc_time = mm_read_mediatime_ms() - jpeg_enc_time;
	emmc_save_time = mm_read_mediatime_ms();
	//write jpg data
	for (uint32_t i = 0; i < jpeg_nv12_len; i += JPG_WRITE_SIZE) {
		extdisk_fwrite((const void *)(jpeg_nv12_addr + i), 1, ((i + JPG_WRITE_SIZE) >= jpeg_nv12_len) ? (jpeg_nv12_len - i) : JPG_WRITE_SIZE, life_snapshot_file);
	}
	xSemaphoreGive(jpeg_get_sema);
	extdisk_fclose(life_snapshot_file);
	emmc_save_time = mm_read_mediatime_ms() - emmc_save_time;
	lfsnap_status = LIFESNAP_DONE;
	return 0;
}
static void lifetime_high_resolution_snapshot_save(char *file_path, uint32_t data_addr, uint32_t data_size)
{
	if (lfsnap_status == LIFESNAP_GET) {
		int ret = 0;
		AI_GLASS_MSG("get liftime snapshot frame time %lu\r\n", mm_read_mediatime_ms());
		AI_GLASS_MSG("file_path:%s  data_addr:%ld  data_size:%ld \r\n", file_path, data_addr, data_size);
		AI_GLASS_MSG("get liftime snapshot frame encode done time %lu\r\n", mm_read_mediatime_ms());
		ls_video_params.params.width = sensor_params[SENSOR_IMX681_12M].sensor_width;
		ls_video_params.params.height = sensor_params[SENSOR_IMX681_12M].sensor_height;
		ls_video_params.params.fps = sensor_params[SENSOR_IMX681_12M].sensor_fps;
		ls_video_params.params.jpeg_qlevel = 9;
		ls_video_params.params.type = VIDEO_JPEG;
		ls_video_params.params.out_mode = MODE_EXT;
		ls_video_params.params.ext_fmt = 1; //NV12
		mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_SET_PARAMS, (int) & (ls_video_params.params));
		mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_SNAPSHOT_CB, (int)jpeg_encode_done_cb);
		video_set_isp_ch_buf(JPEG_CHANNEL, 1);
		mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_APPLY, JPEG_CHANNEL);
		dcache_clean_by_addr((uint32_t *)data_addr, data_size);
		jpeg_nv12_addr = NULL;
		jpeg_nv12_len = 0;
		file_save_path = file_path;
		jpeg_get_sema = xSemaphoreCreateBinary();
		if (mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_SET_EXT_INPUT, (int)data_addr) < 0) {
			AI_GLASS_ERR("fail to set hal_video_ext_in \r\n");
			lfsnap_status = LIFESNAP_FAIL;
		}
		if (xSemaphoreTake(jpeg_get_sema, 3000) != pdTRUE) {
			AI_GLASS_ERR("High resolution jpeg get timeout\r\n");
			lfsnap_status = LIFESNAP_FAIL;
		}
		printf("nv16_take_time = %d, nv12_gen_time = %d, jpeg_enc_time = %d, emmc_save_time = %d\r\n", nv16_take_time, nv12_gen_time, jpeg_enc_time, emmc_save_time);
		AI_GLASS_MSG("get liftime snapshot frame all done time %lu\r\n", mm_read_mediatime_ms());
		mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_STREAM_STOP, JPEG_CHANNEL);
		return;
	}
}

__attribute__((optimize("-O2")))
static int yuv420stitch_step(uint8_t *tiled_yuv, uint8_t *output_buf, int w, int h, int x_overlap, uint32_t *out_size, int is_right)
{
	uint8_t *output_pos = output_buf;
	uint8_t *in_pos = tiled_yuv;
	uint16_t w_tiled = (w / 2 + x_overlap);
	uint16_t w_half = w / 2;
	if (is_right) {
		output_pos += w_half;
	}
	for (int l = 0; l < h * 3 / 2; l++) {
		if (is_right) {
			memcpy(output_pos, in_pos + x_overlap, w_half);
		} else {
			memcpy(output_pos, in_pos, w_half);
		}
		// Move to next line
		output_pos += w_half;
		output_pos += w_half;
		in_pos += w_tiled;
	}
	*out_size += w / 2 * h * 3 / 2;
	return 0;
}
__attribute__((optimize("-O2")))
static void get_remosaiced_cord(uint16_t x, uint16_t y, uint16_t *rm_x, uint16_t *rm_y)
{
	*rm_x = x;
	*rm_y = y;
	switch (x & 0x3) {
	case 1:
		// 1 -> 2
		*rm_x += 1;
		break;
	case 2:
		// 2 -> 1
		*rm_x -= 1;
		break;
	default:
		break;
	}
	switch (y & 0x3) {
	case 1:
		// 1 -> 2
		*rm_y += 1;
		break;
	case 2:
		// 2 -> 1
		*rm_y -= 1;
		break;
	default:
		break;
	}
}
/*
output width = w / 2 + x_overlap
output height = h * 2
input buf => original buf
outbuf => tiled output buf
*/
__attribute__((optimize("-O2")))
static int cap_raw_tiling_with_remosaic(uint8_t *input_buf, uint8_t *outbuf[2], uint16_t in_w, uint16_t in_h, uint16_t x_overlap, uint8_t remosaic_en)
{
	uint16_t tiled_w = (in_w / 2 + x_overlap);
	uint32_t tiled_size = tiled_w * in_h;
	uint32_t x_start_idx;
	uint32_t img_in_size = in_w * in_h;
	uint32_t in_idx = 0;
	uint16_t x, y, rm_x, rm_y;
	uint16_t right_tiled_x;
	uint8_t *left = outbuf[0];
	uint8_t *right = outbuf[1];
	uint8_t pxl_veri_h, pxl_veri_l;
	uint16_t left_only_len;

	for (y = 0; y < in_h; y ++) {
		for (x = 0; x < in_w; x ++) {
			pxl_veri_l = input_buf[in_idx + img_in_size];
			pxl_veri_h = (input_buf[in_idx] & 0xf) << 4 | pxl_veri_l >> 4;
			pxl_veri_l = (pxl_veri_l & 0xf) << 4;

			if (remosaic_en) {
				get_remosaiced_cord(x, y, &rm_x, &rm_y);
			} else {
				rm_x = x;
				rm_y = y;
			}
			x_start_idx = rm_y * tiled_w;
			left_only_len = in_w / 2 - x_overlap;
			// move to out buf (tiled raw)
			// left 0 ~ 2023
			// right
			if (rm_x < left_only_len) { // left
				left[rm_x + x_start_idx] = pxl_veri_h;
				left[rm_x + x_start_idx + tiled_size] = pxl_veri_l;
			} else if (rm_x >= in_w / 2 + x_overlap) { // right
				right_tiled_x = rm_x - (left_only_len);
				right[right_tiled_x + x_start_idx] = pxl_veri_h;
				right[right_tiled_x +  x_start_idx + tiled_size] = pxl_veri_l;
			} else { // both
				left[rm_x + x_start_idx] = pxl_veri_h;
				left[rm_x +  x_start_idx + tiled_size] = pxl_veri_l;
				right_tiled_x = rm_x - (left_only_len);
				right[right_tiled_x + x_start_idx] = pxl_veri_h;
				right[right_tiled_x + x_start_idx + tiled_size] = pxl_veri_l;
			}
			in_idx++;
		}
	}
	return 0;
}
/*
allocate virt addr and free virt addr
dma use phy addr
fill data in phy addr
ret: phy_addr
*/
static void *alloc_dma(rtscam_dma_item_t *dma_item, uint32_t buf_size)
{
	int align_size;
	//Cache line size 2^5 bytes aligned
	uint16_t align_bit = 5;
	align_size = 1 << align_bit;
	dma_item->virt_addr = malloc(buf_size + align_size);
	if (!dma_item->virt_addr) {
		printf("[%s] malloc fail\r\n", __FUNCTION__);
		return NULL;
	}
	if ((int)dma_item->virt_addr & (align_size - 1)) {
		dma_item->phy_addr = (void *)(((int)dma_item->virt_addr + align_size) & (-align_size));
	} else {
		dma_item->phy_addr = (void *)((int)dma_item->virt_addr);
	}
	return dma_item->phy_addr;
}

static void free_dma(rtscam_dma_item_t *dma_item)
{
	if (dma_item->virt_addr) {
		free(dma_item->virt_addr);
	}
	dma_item->phy_addr = 0;
	dma_item->virt_addr = 0;
}
static void config_verification_path_buf(struct verify_ctrl_config *v_cfg, uint32_t img_buf_addr0, uint32_t img_buf_addr1, uint32_t w, uint32_t h,
		uint32_t x_overlap)
{
	uint32_t y_len, uv_len;
	y_len = (w / 2 + x_overlap) * h;
	uv_len = y_len;
	if (v_cfg == NULL) {
		printf("[%s] fail\r\n", __FUNCTION__);
		return;
	}
	v_cfg->verify_addr0 = img_buf_addr0;
	v_cfg->verify_addr1 = img_buf_addr1;
	v_cfg->verify_ylen = y_len;
	v_cfg->verify_uvlen = uv_len;
	//setup right image nlsc center
	v_cfg->verify_r_center.x = x_overlap / 2;
	v_cfg->verify_r_center.y = h / 2;
	v_cfg->verify_g_center.x = x_overlap / 2;
	v_cfg->verify_g_center.y = h / 2;
	v_cfg->verify_b_center.x = x_overlap / 2;
	v_cfg->verify_b_center.y = h / 2;
	dcache_clean_by_addr((uint32_t *)img_buf_addr0, y_len + uv_len);
	dcache_clean_by_addr((uint32_t *)img_buf_addr1, y_len + uv_len);
}
static void save_high_resolution_raw(char *file_path, uint32_t data_addr, uint32_t data_size)
{
	raw_image_len = data_size;
	//split 12M raw to 2 * 6M raw
	uint8_t *tiled_raws[2];
	int tiled_w = ls_video_params.jpg_width / 2 + x_overlap;
	uint32_t tiled_img_size = tiled_w * ls_video_params.jpg_height * 2;
	tiled_raws[0] = alloc_dma(&dma_left, tiled_img_size);
	tiled_raws[1] = alloc_dma(&dma_right, tiled_img_size);
	if (!tiled_raws[0] || !tiled_raws[1]) {
		printf("malloc failed %p %p\n", tiled_raws, tiled_raws);
		goto high_resolution_fail;
	}
#if USE_SENSOR == SENSOR_IMX681
	cap_raw_tiling_with_remosaic((uint8_t *)data_addr, tiled_raws, ls_video_params.jpg_width, ls_video_params.jpg_height, x_overlap, 0);
#else
	cap_raw_tiling_with_remosaic((uint8_t *)data_addr, tiled_raws, ls_video_params.jpg_width, ls_video_params.jpg_height, x_overlap, 1);
#endif
	printf("img_left: %p\n\r", dma_left.phy_addr);
	printf("img_right: %p\n\r", dma_right.phy_addr);
	get_raw_data = 1;
	return;
high_resolution_fail:
	get_raw_data = -1;
	printf("high resolution failed\n\r");
	free_dma(&dma_left);
	free_dma(&dma_right);
	return;
}
static void save_high_resolution_yuv(char *file_path, uint32_t data_addr, uint32_t data_size)
{
	printf("[%s] 0x%lx, data len = %lu\r\n", __FUNCTION__, data_addr, data_size);
	uint8_t *img_buf = (uint8_t *)data_addr;
	uint32_t out_size;
	if (image_count == 0) {
		if (hr_nv12_image == NULL) {
			printf("hr_nv12_image malloc fail\r\n");
			image_count = -1;
			return;
		}
		printf("dma_left raw 0x%lx, data len = %lu\r\n", data_addr, data_size);
		yuv420stitch_step(img_buf, hr_nv12_image, ls_video_params.jpg_width, ls_video_params.jpg_height, x_overlap, &out_size, 0);
		printf("dma_left raw 0x%lx, data len = %lu done\r\n", data_addr, data_size);
		image_count++;
	} else if (image_count == 1) {
		if (hr_nv12_image == NULL) {
			printf("hr_nv12_image malloc fail\r\n");
			image_count = -1;
			return;
		}
		printf("dma_right raw 0x%lx, data len = %lu\r\n", data_addr, data_size);
		yuv420stitch_step(img_buf, hr_nv12_image, ls_video_params.jpg_width, ls_video_params.jpg_height, x_overlap, &out_size, 1);
		printf("dma_right raw 0x%lx, data len = %lu done\r\n", data_addr, data_size);
		image_count++;
	} else {
		image_count = -1;
	}
}

static void high_resolution_snapshot_save(char *file_path)
{
#if USE_VIDEO_HR_FLOW
	int timeout_count = 0;
	nv12_gen_time = mm_read_mediatime_ms();
	mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_STREAM_STOP, JPEG_CHANNEL);
	//switch to verify sequece driver
	int sensor_id = 3;
	mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_SET_SENSOR_ID, sensor_id);
	if (init_params.v_cfg == NULL) {
		init_params.v_cfg = malloc(sizeof(struct verify_ctrl_config));
	}
#if USE_SENSOR == SENSOR_IMX471
	uint8_t zoom_coef[ISP_ZOOM_FILTER_COEF_NUM] = {
		0, 0, 1, 1, 3, 5, 9, 14, 21, 30,
		40, 52, 65, 79, 92, 105, 116, 125, 131, 135
	};
	init_params.zoom_coef = zoom_coef;
#elif USE_SENSOR == SENSOR_IMX681
	uint8_t zoom_coef[ISP_ZOOM_FILTER_COEF_NUM] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 1, 1, 1, 1, 255, 255, 255, 255
	};
	init_params.zoom_coef = zoom_coef;
#endif
	//sent 2 * 6M raw to voe
	config_verification_path_buf(init_params.v_cfg, (uint32_t) dma_left.phy_addr, (uint32_t) dma_right.phy_addr, ls_video_params.jpg_width,
								 ls_video_params.jpg_height, x_overlap);
	init_params.isp_init_raw = 0;
	init_params.isp_raw_mode_tnr_dis = 0;
	init_params.init_isp_items.init_mirrorflip = 0xf0; // not flip when use sequence
	image_count = 0;
	hr_nv12_image = malloc(ls_video_params.jpg_width * ls_video_params.jpg_height * 3 / 2);
	if (!hr_nv12_image) {
		goto snashot_fail;
	}
	//switch to verify sequece driver
	mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_PRE_INIT_PARM, (int)&init_params);
	ls_video_params.params.width = sensor_params[sen_id[sensor_id]].sensor_width;
	ls_video_params.params.height = sensor_params[sen_id[sensor_id]].sensor_height;
	ls_video_params.params.fps = sensor_params[sen_id[sensor_id]].sensor_fps;
	ls_video_params.params.type = VIDEO_NV12;
	mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_SET_PARAMS, (int) & (ls_video_params.params));
	mm_module_ctrl(ls_filesaver_ctx, CMD_FILESAVER_SET_TYPE_HANDLER, (int)save_high_resolution_yuv);
	timeout_count = 0;
	video_set_isp_ch_buf(JPEG_CHANNEL, 2);
	mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_APPLY, JPEG_CHANNEL);
	while (image_count == 0 || image_count == 1) {
		vTaskDelay(1);
		timeout_count++;
		if (timeout_count > 100000) {
			printf("snapshot timeout\r\n");
			goto snashot_fail;
		}
	}
	if (image_count != 2) {
		printf("snapshot failed \r\n");
		goto snashot_fail;
	}
	mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_STREAM_STOP, JPEG_CHANNEL);
	if (init_params.v_cfg) {
		free(init_params.v_cfg);
		init_params.v_cfg = NULL;
	}
	free_dma(&dma_left);
	free_dma(&dma_right);
	AI_GLASS_MSG("yuv genaerate done %lu\r\n", mm_read_mediatime_ms());
	nv12_gen_time = mm_read_mediatime_ms() - nv12_gen_time;
	jpeg_enc_time = mm_read_mediatime_ms();
	lifetime_high_resolution_snapshot_save(file_path, (uint32_t)hr_nv12_image, ls_video_params.jpg_width * ls_video_params.jpg_height * 3 / 2);
	if (hr_nv12_image) {
		free(hr_nv12_image);
		hr_nv12_image = NULL;
	}
	sensor_id = 1;
	mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_SET_SENSOR_ID, sensor_id);
	video_set_isp_ch_buf(JPEG_CHANNEL, 2);
	return;
snashot_fail:
	lfsnap_status = LIFESNAP_FAIL;
	if (hr_nv12_image) {
		free(hr_nv12_image);
		hr_nv12_image = NULL;
	}
	if (init_params.v_cfg) {
		free(init_params.v_cfg);
		init_params.v_cfg = NULL;
	}
	free_dma(&dma_left);
	free_dma(&dma_right);
	lifetime_snapshot_deinitialize();
#endif
	return;
}

static void high_resolution_snapshot_take(char *file_path)
{
#if USE_VIDEO_HR_FLOW
	// get 12M NV16 raw
	get_raw_data = 0;
	int timeout_count = 0;
	video_set_isp_ch_buf(JPEG_CHANNEL, 1);
	if (mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_APPLY, JPEG_CHANNEL) != OK) {
		goto snashot_fail;
	}
	while (!get_raw_data) {
		vTaskDelay(1);
		timeout_count++;
		if (timeout_count > 10000) {
			printf("wait image timeout\r\n");
			goto snashot_fail;
		}
	}
	if (get_raw_data == -1) {
		printf("Err: allocate buffer to process 12M snapshot\r\n");
		goto snashot_fail;
	}
	AI_GLASS_MSG("get 12M NV16 done time %lu\r\n", mm_read_mediatime_ms());
	lfsnap_status = LIFESNAP_GET;
	nv16_take_time = mm_read_mediatime_ms() - nv16_take_time;
	return;
snashot_fail:
	lfsnap_status = LIFESNAP_FAIL;
	if (hr_nv12_image) {
		free(hr_nv12_image);
		hr_nv12_image = NULL;
	}
	if (init_params.v_cfg) {
		free(init_params.v_cfg);
		init_params.v_cfg = NULL;
	}
	free_dma(&dma_left);
	free_dma(&dma_right);
	lifetime_snapshot_deinitialize();
#endif
	return;
}

static void lifetime_normal_snapshot_file_save(char *file_path, uint32_t data_addr, uint32_t data_size)
{
	uint8_t *output_jpg_buf = NULL;
	uint8_t *input_nv12_buf = NULL;
	AI_GLASS_MSG("file_path:%s  data_addr:%ld  data_size:%ld \r\n", file_path, data_addr, data_size);
	if (lfsnap_status == LIFESNAP_TAKE) {
		int ret = 0;
		lfsnap_status = LIFESNAP_GET;
		AI_GLASS_MSG("get liftime snapshot frame time %lu\r\n", mm_read_mediatime_ms());
		AI_GLASS_MSG("file_path:%s  data_addr:%ld  data_size:%ld \r\n", file_path, data_addr, data_size);

		if (ls_video_params.need_sw_encode) {
			uint32_t nv12_size = ls_video_params.jpg_width * ls_video_params.jpg_height / 2 * 3;

			input_nv12_buf = malloc(nv12_size);
			if (!input_nv12_buf) {
				AI_GLASS_ERR("allocate scale up nv12 buffer size %ld/%u fail\r\n", nv12_size, xPortGetFreeHeapSize());
				ret = -1;
				goto closebuff;
			}
			custom_resize_for_nv12((uint8_t *)data_addr, ls_video_params.params.width, ls_video_params.params.height, input_nv12_buf, ls_video_params.jpg_width,
								   ls_video_params.jpg_height);

			AI_GLASS_MSG("get liftime snapshot frame resize done time %lu\r\n", mm_read_mediatime_ms());
			output_jpg_buf = malloc(nv12_size);
			if (!output_jpg_buf) {
				AI_GLASS_ERR("allocate jpg buffer size %ld/%u fail\r\n", nv12_size, xPortGetFreeHeapSize());
				ret = -1;
				goto closebuff;
			}

			FILE *life_snapshot_file = extdisk_fopen(file_path, "wb");
			if (!life_snapshot_file) {
				AI_GLASS_ERR("open jpg file %s fail\r\n", file_path);
				ret = -1;
				goto closebuff;
			}
			uint32_t jpg_size = nv12_size;
			custom_jpegEnc_from_nv12(input_nv12_buf, ls_video_params.jpg_width, ls_video_params.jpg_height, output_jpg_buf, ls_video_params.jpg_qlevel, nv12_size,
									 &jpg_size);
			AI_GLASS_MSG("get liftime snapshot frame encode done time %lu\r\n", mm_read_mediatime_ms());
			//write jpg data
			for (uint32_t i = 0; i < jpg_size; i += JPG_WRITE_SIZE) {
				extdisk_fwrite((const void *)(output_jpg_buf + i), 1, ((i + JPG_WRITE_SIZE) >= jpg_size) ? (jpg_size - i) : JPG_WRITE_SIZE, life_snapshot_file);
			}
			extdisk_fclose(life_snapshot_file);
		} else {
			AI_GLASS_MSG("get liftime snapshot frame time %lu\r\n", mm_read_mediatime_ms());
			AI_GLASS_MSG("get liftime snapshot frame encode done time %lu\r\n", mm_read_mediatime_ms());
			FILE *life_snapshot_file = extdisk_fopen(file_path, "wb");
			if (!life_snapshot_file) {
				AI_GLASS_ERR("open jpg file %s fail\r\n", file_path);
				ret = -1;
				goto closebuff;
			}
			//write jpg data
			for (uint32_t i = 0; i < data_size; i += JPG_WRITE_SIZE) {
				extdisk_fwrite((const void *)(data_addr + i), 1, ((i + JPG_WRITE_SIZE) >= data_size) ? (data_size - i) : JPG_WRITE_SIZE, life_snapshot_file);
			}
			extdisk_fclose(life_snapshot_file);
		}

closebuff:
		if (input_nv12_buf) {
			free(input_nv12_buf);
			input_nv12_buf = NULL;
		}
		if (output_jpg_buf) {
			free(output_jpg_buf);
			output_jpg_buf = NULL;
		}
		if (ret != 0) {
			lfsnap_status = LIFESNAP_FAIL;
		} else {
			lfsnap_status = LIFESNAP_DONE;
		}
		AI_GLASS_MSG("get liftime snapshot frame all done time %lu\r\n", mm_read_mediatime_ms());
	}
}

video_pre_init_params_t ai_glass_pre_init_params = {0};
int lifetime_snapshot_initialize(void)
{
	int ret = 0;
	nv16_take_time = mm_read_mediatime_ms();
	if (lfsnap_status != LIFESNAP_IDLE) {
		ret = -2;
		goto endoflifesnapshot;
	}

#if (USE_SENSOR == SENSOR_IMX681) || (USE_SENSOR == SENSOR_IMX471)
	// Deinitialize fake media channel
	memset(&init_params, 0x00, sizeof(video_pre_init_params_t));
	deinitial_media();
	// Load the AE and AWB data
	media_get_preinit_isp_data(&init_params);
	init_params.isp_ae_enable = 1;
	init_params.isp_awb_enable = 1;
	init_params.init_isp_items.init_mirrorflip = 0xf0;
	// get 12M raw
	int sensor_id = 2;
	init_params.isp_init_raw = 1;
	init_params.isp_raw_mode_tnr_dis = 1;
	init_params.video_drop_enable = 0;
	ls_video_params.params.stream_id = JPEG_CHANNEL;
	ls_video_params.params.type = VIDEO_NV16;
	ls_video_params.params.width = sensor_params[sen_id[sensor_id]].sensor_width;
	ls_video_params.params.height = sensor_params[sen_id[sensor_id]].sensor_height;
	ls_video_params.params.fps = sensor_params[sen_id[sensor_id]].sensor_fps;
	ls_video_params.params.out_mode = 2; //set to contiuous mode
	ls_video_params.params.use_static_addr = 1;
	ls_video_params.jpg_width = sensor_params[sen_id[sensor_id]].sensor_width;
	ls_video_params.jpg_height = sensor_params[sen_id[sensor_id]].sensor_height;
	ls_video_params.jpg_qlevel = SNAPSHOT_12M_QLEVEL;  //life_snap_param.jpeg_qlevel * 10
	ls_video_params.is_high_res = 1;
#if USE_SENSOR == SENSOR_IMX681
	x_overlap = (sensor_params[SENSOR_IMX681_12M_SEQ].sensor_width * 2 - sensor_params[SENSOR_IMX681_12M].sensor_width) / 2;
#elif USE_SENSOR == SENSOR_IMX471
	x_overlap = (sensor_params[SENSOR_IMX471_12M_SEQ].sensor_width * 2 - sensor_params[SENSOR_IMX471_12M].sensor_width) / 2;
#endif
	ls_snapshot_ctx = mm_module_open(&video_module);
	if (ls_snapshot_ctx) {
		mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_SET_SENSOR_ID, sensor_id);
		mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_PRE_INIT_PARM, (int)&init_params);
		mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_SET_PARAMS, (int) & (ls_video_params.params));
		mm_module_ctrl(ls_snapshot_ctx, MM_CMD_SET_QUEUE_LEN, ls_video_params.params.fps);//Default 30
		mm_module_ctrl(ls_snapshot_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		AI_GLASS_ERR("video open fail\n\r");
		ret = -1;
		goto endoflifesnapshot;
	}
	ls_filesaver_ctx = mm_module_open(&filesaver_module);
	if (ls_filesaver_ctx) {
		mm_module_ctrl(ls_filesaver_ctx, CMD_FILESAVER_SET_TYPE_HANDLER, (int)save_high_resolution_raw);
	} else {
		AI_GLASS_ERR("filesaver open fail\n\r");
		ret = -1;
		goto endoflifesnapshot;
	}

	ls_siso_snapshot_filesaver = siso_create();
	if (ls_siso_snapshot_filesaver) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(ls_siso_snapshot_filesaver, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(ls_siso_snapshot_filesaver, MMIC_CMD_ADD_INPUT, (uint32_t)ls_snapshot_ctx, 0);
		siso_ctrl(ls_siso_snapshot_filesaver, MMIC_CMD_ADD_OUTPUT, (uint32_t)ls_filesaver_ctx, 0);
		siso_ctrl(ls_siso_snapshot_filesaver, MMIC_CMD_SET_TASKPRIORITY, LIFE_SNAP_PRIORITY, 0);
		siso_start(ls_siso_snapshot_filesaver);
	} else {
		AI_GLASS_ERR("siso_array_filesaver open fail\n\r");
		ret = -1;
		goto endoflifesnapshot;
	}
#else
	ai_glass_snapshot_param_t life_snap_param;
	memset(&life_snap_param, 0x00, sizeof(ai_glass_snapshot_param_t));
	media_get_life_snapshot_params(&life_snap_param);
	if (life_snap_param.width <= sensor_params[USE_SENSOR].sensor_width && life_snap_param.height <= sensor_params[USE_SENSOR].sensor_height) {
		ls_video_params.params.width = life_snap_param.width;
		ls_video_params.params.height = life_snap_param.height;
		ls_video_params.jpg_width = life_snap_param.width;
		ls_video_params.jpg_height = life_snap_param.height;
		ls_video_params.params.type = VIDEO_JPEG;
		ls_video_params.params.jpeg_qlevel = life_snap_param.jpeg_qlevel;
		ls_video_params.need_sw_encode = 0;
	} else {
		ls_video_params.params.width = life_snap_param.width <= sensor_params[USE_SENSOR].sensor_width ? ls_video_params.params.width :
									   sensor_params[USE_SENSOR].sensor_width;
		ls_video_params.params.height = life_snap_param.height <= sensor_params[USE_SENSOR].sensor_height ? ls_video_params.params.height :
										sensor_params[USE_SENSOR].sensor_height;
		ls_video_params.jpg_width = life_snap_param.width;
		ls_video_params.jpg_height = life_snap_param.height;
		ls_video_params.params.type = VIDEO_NV12;
		ls_video_params.jpg_qlevel = life_snap_param.jpeg_qlevel * 10;
		ls_video_params.need_sw_encode = 1;
	}

	video_pre_init_params_t ai_glass_pre_init_params = {0};
	ls_video_params.params.stream_id = MAIN_STREAM_ID;
	ls_video_params.params.fps = 5;
	ls_video_params.params.gop = 5;
	ls_video_params.params.use_static_addr = 1;
	ls_snapshot_ctx = mm_module_open(&video_module);
	if (ls_snapshot_ctx) {
		media_get_preinit_isp_data(&ai_glass_pre_init_params);
		mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_PRE_INIT_PARM, (int)&ai_glass_pre_init_params);

		mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_SET_PARAMS, (int) & (ls_video_params.params));
		mm_module_ctrl(ls_snapshot_ctx, MM_CMD_SET_QUEUE_LEN, 2);//Default 30
		mm_module_ctrl(ls_snapshot_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		AI_GLASS_ERR("video open fail\n\r");
		ret = -1;
		goto endoflifesnapshot;
	}

	ls_filesaver_ctx = mm_module_open(&filesaver_module);
	if (ls_filesaver_ctx) {
		mm_module_ctrl(ls_filesaver_ctx, CMD_FILESAVER_SET_TYPE_HANDLER, (int)lifetime_normal_snapshot_file_save);
	} else {
		AI_GLASS_ERR("filesaver open fail\n\r");
		ret = -1;
		goto endoflifesnapshot;
	}

	ls_siso_snapshot_filesaver = siso_create();
	if (ls_siso_snapshot_filesaver) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(ls_siso_snapshot_filesaver, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(ls_siso_snapshot_filesaver, MMIC_CMD_ADD_INPUT, (uint32_t)ls_snapshot_ctx, 0);
		siso_ctrl(ls_siso_snapshot_filesaver, MMIC_CMD_ADD_OUTPUT, (uint32_t)ls_filesaver_ctx, 0);
		siso_ctrl(ls_siso_snapshot_filesaver, MMIC_CMD_SET_TASKPRIORITY, LIFE_SNAP_PRIORITY, 0);
		siso_start(ls_siso_snapshot_filesaver);
	} else {
		AI_GLASS_ERR("siso_array_filesaver open fail\n\r");
		ret = -1;
		goto endoflifesnapshot;
	}
	mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_APPLY, ls_video_params.params.stream_id);	// start channel 0
#endif
	lfsnap_status = LIFESNAP_START;
	return ret;
endoflifesnapshot:
	lifetime_snapshot_deinitialize();
	return ret;
}

// Todo: Use semapshore for the process
int lifetime_snapshot_take(const char *file_name)
{
	if (lfsnap_status == LIFESNAP_START) {
		AI_GLASS_MSG("================life_snapshot_take========================== %lu\r\n", mm_read_mediatime_ms());
		AI_GLASS_INFO("Sanpshot start\r\n");
#if (USE_SENSOR == SENSOR_IMX681) || (USE_SENSOR == SENSOR_IMX471)
		AI_GLASS_INFO("Sanpshot start 12M flow\r\n");
		snprintf(snapshot_name, MAXIMUM_FILE_SIZE, "%s", file_name);
		AI_GLASS_MSG("life_snapshot_take %s\r\n", snapshot_name);
		mm_module_ctrl(ls_filesaver_ctx, CMD_FILESAVER_SET_SAVE_FILE_PATH, (int)snapshot_name);
		lfsnap_status = LIFESNAP_TAKE;
		high_resolution_snapshot_take(snapshot_name);
		if (lfsnap_status == LIFESNAP_GET) {
			AI_GLASS_INFO("Get 12M NV16 done\r\n");
			return 0;
		}
#else
		AI_GLASS_INFO("Sanpshot start not 12M flow\r\n");
		snprintf(snapshot_name, MAXIMUM_FILE_SIZE, "%s", file_name);
		AI_GLASS_MSG("life_snapshot_take %s\r\n", snapshot_name);
		mm_module_ctrl(ls_filesaver_ctx, CMD_FILESAVER_SET_SAVE_FILE_PATH, (int)snapshot_name);
		lfsnap_status = LIFESNAP_TAKE;
		if (ls_video_params.need_sw_encode) {
			mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_YUV, 1); // one shot with NV12
		} else {
			mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_SNAPSHOT, 1); // one shot with JPEG
		}
		while (lfsnap_status == LIFESNAP_TAKE) {
			vTaskDelay(1);
		}
		AI_GLASS_INFO("Life snapshot done\r\n");
		return 0;
#endif

	}
	return -1;
}

int lifetime_highres_save(const char *file_name)
{
#if (USE_SENSOR == SENSOR_IMX681) || (USE_SENSOR == SENSOR_IMX471)
	if (lfsnap_status == LIFESNAP_GET) {
		AI_GLASS_MSG("================highres_save========================== %lu\r\n", mm_read_mediatime_ms());
		AI_GLASS_INFO("High res save\r\n");
		high_resolution_snapshot_save((char *)file_name);
		if (lfsnap_status == LIFESNAP_FAIL) {
			AI_GLASS_INFO("Life snapshot save failed\r\n");
			return -1;
		}

		AI_GLASS_INFO("Life snapshot save done\r\n");
		return 0;
	}
	return -1;
#else
	return 0;
#endif
}

int lifetime_snapshot_deinitialize(void)
{
	if (lfsnap_status != LIFESNAP_TAKE && lfsnap_status != LIFESNAP_GET) {
		//Pause Linker
		if (ls_siso_snapshot_filesaver) {
			siso_pause(ls_siso_snapshot_filesaver);
		}

		//Stop module
		if (ls_snapshot_ctx) {
			mm_module_ctrl(ls_snapshot_ctx, CMD_VIDEO_STREAM_STOP, 0);
		}

		//Delete linker
		if (ls_siso_snapshot_filesaver) {
			siso_delete(ls_siso_snapshot_filesaver);
			ls_siso_snapshot_filesaver = NULL;
		}

		//Close module
		if (ls_snapshot_ctx) {
			ls_snapshot_ctx = mm_module_close(ls_snapshot_ctx);
		}
		if (ls_filesaver_ctx) {
			ls_filesaver_ctx = mm_module_close(ls_filesaver_ctx);
		}
		lfsnap_status = LIFESNAP_IDLE;
		return 0;
	}
	return -1;
}