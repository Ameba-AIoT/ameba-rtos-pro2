/******************************************************************************
*
* Copyright(c) 2007 - 2025 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "module_video.h"
#include "module_filesaver.h"
#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"
#include "video_snapshot.h"
#include "log_service.h"
#include "avcodec.h"
#include "isp_ctrl_api.h"
#include "librtsremosaic.h"
#include <ainr.h>
/*
Usage Guide:
1. Please modify sensor driver setting and enable FCS bootup. Only support imx681 and ov13b10.
	project\realtek_amebapro2_v0_example\inc\sensor.h
	#define SENSOR_MAX         4
	static const unsigned char sen_id[SENSOR_MAX] = {
		SENSOR_DUMMY,
		SENSOR_IMX681,
		SENSOR_IMX681_12M,
		SENSOR_IMX681_12M_SEQ
	};
	#define USE_SENSOR      	SENSOR_IMX681
	#define ENABLE_FCS      	1

2. Please enable video high resolution settings.
	component\video\driver\RTL8735B\video_api.h
	#define USE_VIDEO_HR_FLOW 1

3. Disable OSD function
	component\media\mmfv2\module_video.c
	#define OSD_ENABLE 0

4. For FCS mode, set voe heap to 45MB
	component\video\driver\RTL8735B\video_boot.c
	int video_btldr_process(voe_fcs_load_ctrl_t *pvoe_fcs_ld_ctrl, int *code_start)
	{
		...
			int voe_heap_size = video_boot_buf_calc(video_boot_stream);
			voe_heap_size = 45 * 1024 * 1024;
	}

5. To enable burst mode, define BURST_MODE_MAX_COUNT larger than 1. For DDR 128M, maaximun can set to 2.
	#define BURST_MODE_MAX_COUNT 2

	When the heap size required for burst mode is insufficient, it is recommended to reduce the DDR size used by the NN.
	However, if the NN needs to be used concurrently, ensure that the required DDR space is adequate.

	Modify NN DDR size in both rtl8735b_ram.ld and rtl8735b_boot_mp.ld
	project\realtek_amebapro2_v0_example\GCC-RELEASE\application\rtl8735b_ram.ld
	project\realtek_amebapro2_v0_example\GCC-RELEASE\bootloader\rtl8735b_boot_mp.ld
	NN_SIZE = 8;

6. The AINR function is supported only on the IMX681 sensor and requires exposure gain greater than 12x.
	(1) #define ENABLE_AINR 1
	(2) select correct AINR model in amebapro2_fwfs_nn_models.json
		project\realtek_amebapro2_v0_example\GCC-RELEASE\mp\amebapro2_fwfs_nn_models.json
		{
			"msg_level":3,

			"PROFILE":["FWFS"],
			"FWFS":{
				"files":[
					"ainr_mulaw_1024_imx681",
					"planar_to_nchw_blc_1024"
				]
			},
			"ainr_mulaw_1024_imx681":{
				"name" : "ainr_mulaw_1024_imx681.nb",
				"source":"binary",
				"file":"ainr_mulaw_1024_imx681.nb"
			},
			"planar_to_nchw_blc_1024":{
				"name" : "planar_to_nchw_1024_lut.nb",
				"source":"binary",
				"file":"planar_to_nchw_1024_lut.nb"
			}
		}
	(3) AINR requires 12M DDR. With total DDR = 128 MB, burst mode is not supported.


====================================================================
Output 12M JPEG Flow:
1. FCS bringup low resolution video and get converge AE, AWB value.
2. Get 12M raw.
3. Split 12M raw into two 6M raw.
4. Sent two 6M raw into VOE and ouput to two 6M NV12 image.
5. Merge two 6M NV12 into 12M NV12 image.
6. Convert 12M NV12 image to 12M JPEG image.
====================================================================
*/

#if USE_VIDEO_HR_FLOW == 0
void mmf2_video_example_v1_snapshot_hr_init(void)
{
	printf("\r\nPlease modify USE_VIDEO_HR to 1 in video_api.h\r\n\r\n");
}
#else

enum sensor_driver_mode {
	VIDEO_MODE = 1,
	HR_RAW_MODE,
	HR_SEQ_MODE
};

static void atcmd_userctrl_init(void);
static mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *filesaver_ctx			= NULL;
static mm_siso_t *siso_video_filesaver_v1	= NULL;

#define JPEG_CHANNEL 0
#define JPEG_WIDTH	sensor_params[USE_SENSOR].sensor_width
#define JPEG_HEIGHT	sensor_params[USE_SENSOR].sensor_height
#define JPEG_FPS	sensor_params[USE_SENSOR].sensor_fps

//set output resolution to high reesolution
#define OUT_IMG_WIDTH sensor_params[sen_id[2]].sensor_width
#define OUT_IMG_HEIGHT sensor_params[sen_id[2]].sensor_height
#define OUT_IMG_OVERLAP_WIDTH (((sensor_params[sen_id[3]].sensor_width * 2) - sensor_params[sen_id[2]].sensor_width) / 2)
#define OUT_IMG_OVERLAP_HEIGHT (((sensor_params[sen_id[3]].sensor_height * 2) - sensor_params[sen_id[2]].sensor_height) / 2)

static uint8_t *hr_nv12_image = NULL;
static uint32_t hr_nv12_size = OUT_IMG_WIDTH * OUT_IMG_HEIGHT * 3 / 2;
#define SAVE_DBG_IMG 0 //save raw image and NV12 image
#define BURST_MODE_MAX_COUNT 1 //when set to 1, disable burst mode. for DDR 128M, maaximun can set to 2
#if USE_SENSOR == SENSOR_IMX681
#define ENABLE_AINR 1
#else
#define ENABLE_AINR 0
#endif
static int raw_index = 0;
static enum hal_isp_ae_region max_dyn_region_idx = 0; // Data range: 0 ~ 3. 0: upper left, 1: upper right, 2: lower left, 3: lower right.
static int tiled_nv12_cnt = 0;
static video_pre_init_params_t init_params;
static ainr_ctx_t *ainr_ctx = NULL;

/*
allocate virt addr and free virt addr
dma use phy addr
fill data in phy addr
*/
#define SPLIT_RAW_NUM 4
#define ORG_FRAME_NUM (SPLIT_RAW_NUM * 2)
#define DUMMY_FRAME_NUM 2 // Dummy frames based on 8x3M.
#define VERIFY_NUM (ORG_FRAME_NUM + DUMMY_FRAME_NUM)
typedef struct {
	void *virt_addr; //for temporary save 12M raw
	void *phy_addr[SPLIT_RAW_NUM];
} splited_raw_item_t;
static splited_raw_item_t splited_raw_image[BURST_MODE_MAX_COUNT] = {0};

static video_params_t video_v1_params = {
	.stream_id 	= JPEG_CHANNEL,
	.type 		= VIDEO_H264,
	.width 		= JPEG_WIDTH,
	.height 	= JPEG_HEIGHT,
	.fps 		= JPEG_FPS,
	.use_static_addr = 1,
};

enum file_process_status {
	PROCESS_DONE = 0,
	PROCESS_START,
	SPLIT_RAW_IMAGE_START,
	MERGE_UL_NV12_1, // Upper left 1.
	MERGE_UL_NV12_2, // Upper left 2.
	MERGE_UR_NV12_1, // Upper right 1.
	MERGE_UR_NV12_2, // Upper right 2.
	MERGE_LL_NV12_1, // Lower left 1.
	MERGE_LL_NV12_2, // Lower left 2.
	MERGE_LR_NV12_1, // Lower right 1.
	MERGE_LR_NV12_2  // Lower right 2.
};
enum file_process_command {
	DO_NOTHING = 0,
	SPLIT_RAW_IMAGE,		//split 12M raw into 4* 3M raw
	MERGE_NV12_IMAGE,		//skip first 2 frame and merge 4 image
	SAVE_NV12,
};
/*
OUT_IMG_WIDTH => full width
h => full height
*/
__attribute__((optimize("-O2")))
static int yuv420stitch_step_4c(uint8_t *tiled_yuv, uint8_t *output_buf, const uint16_t w, const uint16_t h, const uint16_t overlap_width, const uint16_t overlap_height, uint32_t * const out_size, const enum file_process_status proc_stat)
{
	const uint16_t w_half = w / 2;
	const uint16_t w_tiled = w_half + overlap_width;
	const uint16_t y_half = h / 2;
	const uint16_t uv_half = y_half / 2;
	const uint16_t h_tiled = y_half + overlap_height;

	uint8_t *output_y_pos = output_buf;
	uint8_t *output_uv_pos = output_buf + w * h;
	uint8_t *input_y_pos = tiled_yuv;
	uint8_t *input_uv_pos = tiled_yuv + w_tiled * h_tiled;

	// Initialize start positions of I/O buffers.
	// NOTE: only accepts second images of each position.
	switch (proc_stat) {
	case MERGE_UL_NV12_2:
		break;
	case MERGE_UR_NV12_2:
		output_y_pos += w_half;
		output_uv_pos += w_half;
		input_y_pos += overlap_width;
		input_uv_pos += overlap_width;
		break;
	case MERGE_LL_NV12_2:
		output_y_pos += w * y_half;
		output_uv_pos += w * uv_half;
		input_y_pos += overlap_height * w_tiled;
		input_uv_pos += (overlap_height / 2) * w_tiled;
		break;
	case MERGE_LR_NV12_2:
		output_y_pos += w * y_half + w_half;
		output_uv_pos += w * uv_half + w_half;
		input_y_pos += overlap_height * w_tiled + overlap_width;
		input_uv_pos += (overlap_height / 2) * w_tiled + overlap_width;
		break;
	default:
		printf("[ERROR] Invalid option for function %s: %d\r\n", __FUNCTION__, proc_stat);
		return -1;
	}

	// Copy line by line.
	for (uint16_t l = 0; l < y_half; l++, output_y_pos += w, input_y_pos += w_tiled) {
		memcpy(output_y_pos, input_y_pos, w_half);
	}

	for (uint16_t l = 0; l < uv_half; l++, output_uv_pos += w, input_uv_pos += w_tiled) {
		memcpy(output_uv_pos, input_uv_pos, w_half);
	}

	*out_size += w_half * y_half;
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
		break;
		default:
			break;
    }
}

static void *alloc_split_raw_item(splited_raw_item_t *splited_raw, uint32_t split_raw_size)
{
	// Cache line size 2^5 bytes aligned
	uint16_t align_bit = 5;
	int align_size = 1 << align_bit;
	uint32_t split_raw_size_align = (split_raw_size + align_size - 1) & ~(align_size - 1);
	uint32_t buffer_size = split_raw_size_align * SPLIT_RAW_NUM + align_size;

	splited_raw->virt_addr = malloc(buffer_size);
	if (!splited_raw->virt_addr) {
		printf("[%s] malloc failed\n", __FUNCTION__);
		return NULL;
	}

	uintptr_t base_addr = (uintptr_t)splited_raw->virt_addr;
	uintptr_t aligned_addr = (base_addr + align_size - 1) & ~(uintptr_t)(align_size - 1);
	
	for(int i = 0; i < SPLIT_RAW_NUM; i++) {
		splited_raw->phy_addr[i] = (void*)(aligned_addr + i * split_raw_size_align);
	}

	return splited_raw->virt_addr;
}

static void free_split_raw_item(splited_raw_item_t *splited_raw)
{
	if(splited_raw->virt_addr) {
		free(splited_raw->virt_addr);
		splited_raw->virt_addr = NULL;
	}
	for(int i = 0; i < SPLIT_RAW_NUM; i++) {
		splited_raw->phy_addr[i] = NULL;
	}
}

static void config_verification_path_buf_4c(struct verify_ctrl_config *v_cfg, const uint32_t img_buf_addr[4],
	const uint16_t w, const uint16_t h, const uint16_t overlap_width, const uint16_t overlap_height)
{
	const uint32_t y_len = (w / 2 + overlap_width) * (h / 2 + overlap_height);
	const uint32_t uv_len = y_len;

	if(v_cfg == NULL) {
		printf("[%s] fail\r\n", __FUNCTION__);
		return;
	}

	v_cfg->verify_number = VERIFY_NUM;
	v_cfg->verify_ylen = y_len;
	v_cfg->verify_uvlen = uv_len;

	for (uint8_t i = 0; i < VERIFY_NUM; i++) {
		const uint8_t split_num = i < DUMMY_FRAME_NUM ? 0 : (i - DUMMY_FRAME_NUM) / 2;
		const uint8_t split_id = (split_num + max_dyn_region_idx) % SPLIT_RAW_NUM;

		uint16_t center_x, center_y;
		switch (split_id) {
		case 0:
			center_x = w / 2;
			center_y = h / 2;
			break;
		case 1:
			center_x = overlap_width;
			center_y = h / 2;
			break;
		case 2:
			center_x = w / 2;
			center_y = overlap_height;
			break;
		case 3:
			center_x = overlap_width;
			center_y = overlap_height;
			break;
		default:
			printf("[%s] invalid split_id: %d\r\n", __FUNCTION__, split_id);
			return;
		}

		v_cfg->verify_addr[i] = img_buf_addr[split_id];
		v_cfg->verify_nlsc_center[i].verify_nlsc_rcenter_x = center_x;
		v_cfg->verify_nlsc_center[i].verify_nlsc_rcenter_y = center_y;
		v_cfg->verify_nlsc_center[i].verify_nlsc_gcenter_x = center_x;
		v_cfg->verify_nlsc_center[i].verify_nlsc_gcenter_y = center_y;
		v_cfg->verify_nlsc_center[i].verify_nlsc_bcenter_x = center_x;
		v_cfg->verify_nlsc_center[i].verify_nlsc_bcenter_y = center_y;
	}
	for(int i = 0; i < SPLIT_RAW_NUM; i++) {
		SCB_CleanDCache_by_Addr((uint32_t *)img_buf_addr[i], y_len + uv_len);
	}
}

#define IMG_WRITE_SIZE          4096
static int save_file_to_sd(char* fp, uint8_t *file_buf, uint32_t buf_size)
{
	FILE *m_file;
	if (!file_buf) {
		printf("file buf is empty!!\n");
		return -1;
	}
	m_file = fopen(fp, "w+");
	if (m_file == NULL) {
		printf("Open file to write failed!!! %s\r\n", fp);
		return -1;
	}
    for (uint32_t i = 0; i < buf_size; i += IMG_WRITE_SIZE) {
        fwrite(file_buf + i, 1, ((i + IMG_WRITE_SIZE) >= buf_size) ? (buf_size - i) : IMG_WRITE_SIZE, m_file);
    }
	//int count = fwrite(file_buf, buf_size, 1, m_file);
	fclose(m_file);
	return 0;
}

#if SAVE_DBG_IMG
uint8_t *raw_image = NULL;
uint32_t raw_image_size = 0;
static void raw_reform(uint8_t *pData, uint8_t *pTmp, int dataLen)
{
	int dim = dataLen / 2;
	int nIndex = 0;
	for (int j = 0; j < dim; j++) {
		int nValue = (pTmp[nIndex] << 8) | pTmp[nIndex + dim];

		pData[2 * nIndex] = nValue & 0xff;
		pData[2 * nIndex + 1] = (nValue >> 8) & 0xff;
		nIndex++;
	}
}
#endif

static enum file_process_status file_proc_stat = PROCESS_DONE;
static enum file_process_command file_proc_cmd = DO_NOTHING;
static void file_process(char *file_path, uint32_t data_addr, uint32_t data_size)
{
	//printf("[%s] 0x%x, data len = %lu\r\n",__FUNCTION__, data_addr, data_size);
	uint8_t *img_buf = (uint8_t *)data_addr;
	uint32_t img_buf_size, tiled_img_size;
	img_buf_size = OUT_IMG_WIDTH * OUT_IMG_HEIGHT * 2;
	uint32_t out_size;

	if(file_proc_cmd == SPLIT_RAW_IMAGE) {
		//get 12M raw.
		printf("12M raw 0x%x, data len = %lu\r\n", data_addr, data_size);
#if ENABLE_AINR && (USE_SENSOR == SENSOR_IMX681)
		if(init_params.isp_ae_init_gain > (256 * 12)) {
			// IMX681 AINR flow for exposure gain > 12x
			if (ainr_ctx == NULL) {
				ainr_ctx = ainr_init();
			}
			if(ainr_ctx) {
				uint8_t *ainr_raw_image = splited_raw_image[raw_index].virt_addr;
				uint32_t ainr_raw_image_size = data_size;
				if (ainr_process_frame(ainr_ctx, (const void *)data_addr, ainr_raw_image, ainr_raw_image_size, 256) != OK) {
					printf("ainr_process_frame() failed.\r\n");
					return;
				}
				pack_bayer_to_planar((uint8_t *)data_addr, (const uint16_t *)ainr_raw_image, ainr_raw_image_size);
			} else {
				printf("ainr_init() failed.\r\n");
				return;
			}
		}
#endif

#if SAVE_DBG_IMG
		//temporary use split raw memory for 12M raw buffer
		raw_image = splited_raw_image[raw_index].virt_addr;
		raw_image_size = data_size;
		if(raw_image) {
			raw_reform(raw_image, (uint8_t*)data_addr, raw_image_size);
			char rawfilename[128] = "sd:/12M.raw";
			snprintf(rawfilename, sizeof(rawfilename), "sd:/12M_%d.raw", raw_index);
			save_file_to_sd(rawfilename,  (uint8_t *)raw_image, raw_image_size);
			printf("save %s\r\n", rawfilename);
		} else {
			printf("raw image malloc fail\r\n");
			return; 
		}
#endif
		file_proc_stat = SPLIT_RAW_IMAGE_START;
		//split 12M raw into 4x3M raw.
		uint8_t *tiled_raws[SPLIT_RAW_NUM];
		for (uint8_t i = 0; i < SPLIT_RAW_NUM; i++) {
			tiled_raws[i] = (uint8_t *)splited_raw_image[raw_index].phy_addr[i];
		}
		
		cap_raw_tiling_with_remosaic_4c((uint8_t*)data_addr, tiled_raws, OUT_IMG_WIDTH, OUT_IMG_HEIGHT, OUT_IMG_OVERLAP_WIDTH, OUT_IMG_OVERLAP_HEIGHT, REMOSAIC_DISABLE, REMOSAIC_DIRECT_MODE, GR);
		/*for(int i = 0; i < SPLIT_RAW_NUM; i++) {
			printf("tiled_raws_%d: 0x%x\r\n", i, tiled_raws[i]);
		}*/

	} else if(file_proc_cmd == MERGE_NV12_IMAGE) {
		switch (file_proc_stat)
		{
		case PROCESS_START:
		case MERGE_UL_NV12_1:
		case MERGE_UR_NV12_1:
		case MERGE_LL_NV12_1:
		case MERGE_LR_NV12_1:
			if (tiled_nv12_cnt < DUMMY_FRAME_NUM) {
				//printf("[file_process] Skip %d dummy frame...\r\n", tiled_nv12_cnt + 1);
			} else {
				file_proc_stat++;
			}
			tiled_nv12_cnt++;
			return;
		case MERGE_UL_NV12_2:
		case MERGE_UR_NV12_2:
		case MERGE_LL_NV12_2:
		case MERGE_LR_NV12_2:
			yuv420stitch_step_4c(img_buf, hr_nv12_image, OUT_IMG_WIDTH, OUT_IMG_HEIGHT, OUT_IMG_OVERLAP_WIDTH, OUT_IMG_OVERLAP_HEIGHT, &out_size, file_proc_stat);
			if (tiled_nv12_cnt == VERIFY_NUM - 1) {
				file_proc_stat = PROCESS_DONE;
			} else {
				file_proc_stat = (file_proc_stat + 1 - MERGE_UL_NV12_1) % ORG_FRAME_NUM + MERGE_UL_NV12_1;
			}
			tiled_nv12_cnt++;
			return;
		default:
			printf("Invalid file_proc_stat: %d\r\n", file_proc_stat);
			return;
		}
	}
	file_proc_stat = PROCESS_DONE;
}

static int hr_init_ae_awb(video_pre_init_params_t *init_params, int wait_ae_timeout)
{
	int sen_drv_mode_id = VIDEO_MODE;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_SENSOR_ID, sen_drv_mode_id);
	init_params->isp_init_raw = 0;
	init_params->isp_raw_mode_tnr_dis = 0;
	init_params->video_drop_enable = 0;
	init_params->dyn_iq_mode = 0;
	init_params->init_isp_items.init_wdr_mode = WDR_AUTO;
	init_params->init_max_dyn_region_en = 1;
	init_params->sens_pwr_dis = 0;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_PRE_INIT_PARM, (int)init_params);
 	video_v1_params.direct_output = 1;
	video_v1_params.out_mode = 2; //set to contiuous mode
	video_v1_params.width = sensor_params[sen_id[sen_drv_mode_id]].sensor_width;
	video_v1_params.height = sensor_params[sen_id[sen_drv_mode_id]].sensor_height;
	video_v1_params.fps = sensor_params[sen_id[sen_drv_mode_id]].sensor_fps;
	video_v1_params.type = VIDEO_H264;
	video_v1_params.ext_fmt = 0;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
	//video open in normal resolution, and wait ae converge. (FCS mode can save time.)
	if(mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, JPEG_CHANNEL) != OK) {
		return NOK;
	}
	int last_ae_time = 0, last_ae_gain = 0;
	int ae_time, ae_gain, awb_rgain, awb_bgain;
	isp_get_exposure_time(&ae_time);
	isp_get_ae_gain(&ae_gain);
	int wait_time = 0;
	while((ae_time != last_ae_time) || (ae_gain != last_ae_gain)) {
		vTaskDelay(100);
		wait_time += 100;
		last_ae_time = ae_time;
		last_ae_gain = ae_gain;
		isp_get_exposure_time(&ae_time);
		isp_get_ae_gain(&ae_gain);
		//printf("ae time %d->%d\r\n", last_ae_time, ae_time);
		//printf("ae gain %d->%d\r\n", last_ae_gain, ae_gain);
		if(wait_time >= wait_ae_timeout) {
			printf("wait ae stable timeout\r\n");
			break;	
		}
	}
	isp_get_red_balance(&awb_rgain);
	isp_get_blue_balance(&awb_bgain);

	video_get_max_dyn_region_idx(JPEG_CHANNEL, &max_dyn_region_idx);
	//printf("video_get_dyn_region_idx value %d\r\n", max_dyn_region_idx);
	
	uint8_t direct_wdr_level = 0;
	video_get_dir_wdr_level(JPEG_CHANNEL, &direct_wdr_level);
	init_params->init_isp_items.init_wdr_level = direct_wdr_level;
	//printf("video_get_dir_wdr_level value %u\r\n", direct_wdr_level);

	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, JPEG_CHANNEL);

	//set ae, awb init parameters for high resolution (12M) snapshot
	init_params->isp_ae_enable = 1;
	if(ae_gain >= 1024) { //ae gain >= 4x
		init_params->isp_ae_init_exposure = ae_time << 1;
		init_params->isp_ae_init_gain = ae_gain >> 1;
	}
	else {
		init_params->isp_ae_init_exposure = ae_time;
		init_params->isp_ae_init_gain = ae_gain;
	}
	printf("ae time %d gain %d\r\n", init_params->isp_ae_init_exposure, init_params->isp_ae_init_gain);
	init_params->isp_awb_enable = 1;
	init_params->isp_awb_init_rgain = awb_rgain;
	init_params->isp_awb_init_bgain = awb_bgain;
	return OK;
}

static int hr_raw_capture(video_pre_init_params_t *init_params, int proc_raw_idx)
{
	int ret = OK;
	// get 12M raw
	int sen_drv_mode_id = HR_RAW_MODE;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_SENSOR_ID, sen_drv_mode_id);
	init_params->isp_init_raw = 1;
	init_params->isp_raw_mode_tnr_dis = 1;
	init_params->video_drop_enable = 0;
	init_params->dyn_iq_mode = 0;
	init_params->init_max_dyn_region_en = 0;
	init_params->sens_pwr_dis = 0;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_PRE_INIT_PARM, (int)init_params);
	video_v1_params.direct_output = 0;
	video_v1_params.out_mode = 2; //set to contiuous mode
	video_v1_params.width = sensor_params[sen_id[sen_drv_mode_id]].sensor_width;
	video_v1_params.height = sensor_params[sen_id[sen_drv_mode_id]].sensor_height;
	video_v1_params.fps = sensor_params[sen_id[sen_drv_mode_id]].sensor_fps;
	video_v1_params.type = VIDEO_NV16;
	video_v1_params.ext_fmt = 0;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
	
	//split 12M raw into 2 * 6M raw
	file_proc_cmd = SPLIT_RAW_IMAGE;
	file_proc_stat = PROCESS_START;
	raw_index = proc_raw_idx;
	video_set_isp_ch_buf(JPEG_CHANNEL, 1);
	int timeout_count = 0;
	if(mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, JPEG_CHANNEL) != OK) {
		return NOK;
	}
	while(file_proc_stat != SPLIT_RAW_IMAGE_START) {
		vTaskDelay(1);
		timeout_count++;
		if(timeout_count > 100000) {
			printf("wait hr raw process start timeout\r\n");
			ret = NOK;
			break;
		}
	}
	//process raw image and video close simutaneously.
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, JPEG_CHANNEL);	
	while(file_proc_stat != PROCESS_DONE) {
		vTaskDelay(1);
		timeout_count++;
		if(timeout_count > 100000) {
			printf("wait hr raw timeout\r\n");
			ret = NOK;
			break;
		}
	}
	return ret;
}

static int hr_raw_to_nv12(video_pre_init_params_t *init_params, int proc_raw_idx)
{
	int ret = OK;
	//switch to verify sequece driver
	int sen_drv_mode_id = HR_SEQ_MODE;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_SENSOR_ID, sen_drv_mode_id);
	if(init_params->v_cfg == NULL) {
		init_params->v_cfg = malloc(sizeof(struct verify_ctrl_config));
	}
	//sent 4 * 3M raw to voe
	uint32_t tiled_raws[SPLIT_RAW_NUM];
	for (uint8_t i = 0; i < SPLIT_RAW_NUM; i++) {
		tiled_raws[i] = (uint32_t)splited_raw_image[proc_raw_idx].phy_addr[i];
	}
	config_verification_path_buf_4c(init_params->v_cfg, tiled_raws, OUT_IMG_WIDTH, OUT_IMG_HEIGHT, OUT_IMG_OVERLAP_WIDTH, OUT_IMG_OVERLAP_HEIGHT);
	init_params->isp_init_raw = 0;
	init_params->isp_raw_mode_tnr_dis = 0;
	init_params->dyn_iq_mode = 1;
	init_params->init_isp_items.init_wdr_mode = WDR_DIRECT;
	init_params->init_max_dyn_region_en = 0;
	init_params->sens_pwr_dis = 1;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_PRE_INIT_PARM, (int)init_params);
	video_v1_params.direct_output = 0;
	video_v1_params.out_mode = 2; //set to contiuous mode
	video_v1_params.width = sensor_params[sen_id[sen_drv_mode_id]].sensor_width;
	video_v1_params.height = sensor_params[sen_id[sen_drv_mode_id]].sensor_height;
	video_v1_params.fps = sensor_params[sen_id[sen_drv_mode_id]].sensor_fps;
	video_v1_params.type = VIDEO_NV12;
	video_v1_params.ext_fmt = 0;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);

	//merge output 4 * 3M nv12 to 12M nv12 image
	file_proc_cmd = MERGE_NV12_IMAGE;
	file_proc_stat = MERGE_UL_NV12_1 + max_dyn_region_idx * 2; //decide process order with max_dyn_region_idx
	tiled_nv12_cnt = 0;
	video_set_isp_ch_buf(JPEG_CHANNEL, 2);
	int timeout_count = 0;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, JPEG_CHANNEL);
	while(file_proc_stat != PROCESS_DONE) {
		vTaskDelay(1);
		timeout_count++;
		if(timeout_count > 100000) {
			printf("hr nv12 convert timeout\r\n");
			ret = NOK;
			break;
		}
	}
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, JPEG_CHANNEL);

#if SAVE_DBG_IMG	
	char nv12filename[128] = "sd:/12M.nv12";
	snprintf(nv12filename, sizeof(nv12filename), "sd:/12M_%d.nv12", proc_raw_idx);
	save_file_to_sd(nv12filename,  (uint8_t *)hr_nv12_image, hr_nv12_size);
	printf("save %s\r\n", nv12filename);
#endif

	if(init_params->v_cfg) {
		free(init_params->v_cfg);
		init_params->v_cfg = NULL;
	}
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_PRE_INIT_PARM, (int)init_params);
	return ret;
}

static int jpg_save_done = 0;
static char jpgfilename[128] = "sd:/12M.jpg";
static void hr_jpg_done_cb(uint32_t jpeg_addr, uint32_t jpeg_len)
{
	printf("jpeg addr=%x, len=%lu\r\n", jpeg_addr, jpeg_len);
	save_file_to_sd(jpgfilename, (uint8_t*)jpeg_addr, jpeg_len);
	printf("save %s\r\n", jpgfilename);	
	jpg_save_done = 1;
}
static int hr_nv12_to_jpeg(video_pre_init_params_t *init_params, int jpg_save_timeout, int jpg_idx)
{
	int ret = OK;
	init_params->sens_pwr_dis = 1;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_PRE_INIT_PARM, (int)init_params);
	video_v1_params.direct_output = 0;
	video_v1_params.width = OUT_IMG_WIDTH;
	video_v1_params.height = OUT_IMG_HEIGHT;
	video_v1_params.jpeg_qlevel = 10;
	video_v1_params.type = VIDEO_JPEG;
	video_v1_params.out_mode = MODE_EXT;
	video_v1_params.ext_fmt = 1; //NV12
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SNAPSHOT_CB, (int)hr_jpg_done_cb);
	video_set_isp_ch_buf(JPEG_CHANNEL, 1);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, JPEG_CHANNEL);
	dcache_clean_by_addr((uint32_t *)hr_nv12_image, hr_nv12_size);
	snprintf(jpgfilename, sizeof(jpgfilename), "sd:/12M_%d.jpg", jpg_idx);
	jpg_save_done = 0;
	if (mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_EXT_INPUT, (int)hr_nv12_image) == OK) {
		int jpeg_wait_time = 0;
		while(!jpg_save_done) {
			vTaskDelay(1);
			jpeg_wait_time++;
			if(jpeg_wait_time > jpg_save_timeout) {
				printf("hr jpg save timeout\r\n");
				ret = NOK;
				break;
			}
		}
	} else {
		ret = NOK;
	}
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, JPEG_CHANNEL);
	
	return ret;
}

static void example_deinit(void);
void mmf2_video_example_v1_snapshot_hr_init(void)
{
	atcmd_userctrl_init();
	snapshot_vfs_init();

	int voe_heap_size = video_voe_presetting(1, JPEG_WIDTH, JPEG_HEIGHT, 0, 1,
						0, 0, 0, 0, 0,
						0, 0, 0, 0, 0,
						0, 0, 0);
	//remalloc voe heap to 45M
	voe_heap_size = 45 * 1024 * 1024;
	video_set_voe_heap((int)NULL, voe_heap_size, 1);
	printf("\r\n voe heap size = %d\r\n", voe_heap_size);
	printf("output resolution w=%d, h=%d, overlap_width=%d, overlap_height=%d\r\n", OUT_IMG_WIDTH, OUT_IMG_HEIGHT, OUT_IMG_OVERLAP_WIDTH, OUT_IMG_OVERLAP_HEIGHT);
	printf("Available heap 0x%x\r\n", xPortGetFreeHeapSize());
	
	//prevent memory fragment, allocate hr splited raw buffer
	for(int i = 0; i < BURST_MODE_MAX_COUNT; i++) {
		int tiled_w = OUT_IMG_WIDTH / 2 + OUT_IMG_OVERLAP_WIDTH;
		int tiled_h = OUT_IMG_HEIGHT / 2 + OUT_IMG_OVERLAP_HEIGHT;
		uint32_t tiled_img_size = tiled_w * tiled_h * 2;
		if(alloc_split_raw_item(&(splited_raw_image[i]), tiled_img_size) == NULL) {
			printf("splited raw image malloc failed\n");
			printf("Available heap 0x%x\r\n", xPortGetFreeHeapSize());
			goto mmf2_video_exmaple_v1_shapshot_hr_fail;
		}
	}

	//prevent memory fragment, allocate hr nv12 buffer
	if(hr_nv12_image == NULL) {
		hr_nv12_image = malloc(hr_nv12_size);
	}
	if(hr_nv12_image == NULL) {
		printf("hr_nv12_image malloc fail\r\n");
		printf("Available heap 0x%x\r\n", xPortGetFreeHeapSize());
		goto mmf2_video_exmaple_v1_shapshot_hr_fail;
	}
	//printf("Available heap 0x%x\r\n", xPortGetFreeHeapSize());

	memset(&init_params, 0x00, sizeof(video_pre_init_params_t));
	init_params.isp_init_enable = 1;
	init_params.init_isp_items.init_brightness = 0;
	init_params.init_isp_items.init_contrast = 50;
	init_params.init_isp_items.init_flicker = FLICKER_60HZ;
	init_params.init_isp_items.init_hdr_mode = 0;
	init_params.init_isp_items.init_mirrorflip = MIRRORFLIP_DISABLE;
	init_params.init_isp_items.init_saturation = 50;
	init_params.init_isp_items.init_wdr_mode = WDR_AUTO;
	init_params.init_isp_items.init_wdr_level = 50;
	init_params.init_isp_items.init_mipi_mode = 0;
	init_params.voe_dbg_disable = 1;
	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		video_v1_params.direct_output = 1;
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, JPEG_FPS);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	}

	filesaver_ctx = mm_module_open(&filesaver_module);
	if (filesaver_ctx) {
		mm_module_ctrl(filesaver_ctx, CMD_FILESAVER_SET_TYPE_HANDLER, (int)file_process);
	} else {
		rt_printf("filesaver open fail\n\r");
		goto mmf2_video_exmaple_v1_shapshot_hr_fail;
	}	
	
	siso_video_filesaver_v1 = siso_create();
	if (siso_video_filesaver_v1) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_array_filesaver, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_video_filesaver_v1, MMIC_CMD_ADD_INPUT, (uint32_t)video_v1_ctx, 0);
		siso_ctrl(siso_video_filesaver_v1, MMIC_CMD_ADD_OUTPUT, (uint32_t)filesaver_ctx, 0);
		siso_start(siso_video_filesaver_v1);
	} else {
		rt_printf("siso_array_filesaver open fail\n\r");
		goto mmf2_video_exmaple_v1_shapshot_hr_fail;
	}
	
	if(hr_init_ae_awb(&init_params, 1000) == NOK) {
		goto mmf2_video_exmaple_v1_shapshot_hr_fail;
	}
	
	for(int i = 0; i < BURST_MODE_MAX_COUNT; i++) {
		if(hr_raw_capture(&init_params, i) == NOK) {
			goto mmf2_video_exmaple_v1_shapshot_hr_fail;
		}
	}

	for(int i = 0; i < BURST_MODE_MAX_COUNT; i++) {
		if(hr_raw_to_nv12(&init_params, i) == NOK) {
			goto mmf2_video_exmaple_v1_shapshot_hr_fail;
		}
		if(hr_nv12_to_jpeg(&init_params, 10000, i) == NOK) {
			goto mmf2_video_exmaple_v1_shapshot_hr_fail;
		}
	}
	
mmf2_video_exmaple_v1_shapshot_hr_fail:

	printf("Available heap 0x%x\r\n", xPortGetFreeHeapSize());
	video_voe_release();

	if(ainr_ctx) {
		ainr_deinit(ainr_ctx);
	}
	
	for(int i = 0; i < BURST_MODE_MAX_COUNT; i++) {
		free_split_raw_item(&(splited_raw_image[i]));
	}

	if(hr_nv12_image) {
		free(hr_nv12_image);
		hr_nv12_image = NULL;
	}

	siso_stop(siso_video_filesaver_v1);
	siso_delete(siso_video_filesaver_v1);
	mm_module_close(filesaver_ctx);
	mm_module_close(video_v1_ctx);
	video_voe_release();


	return;
}

static const char *example = "mmf2_video_example_v1_shapshot_hr";
static void example_deinit(void)
{

}

static void fUC(void *arg)
{
	static uint32_t user_cmd = 0;

	if (!strcmp(arg, "TD")) {
		if (user_cmd & USR_CMD_EXAMPLE_DEINIT) {
			printf("invalid state, can not do %s deinit!\r\n", example);
		} else {
			example_deinit();
			user_cmd = USR_CMD_EXAMPLE_DEINIT;
			printf("deinit %s\r\n", example);
		}
	} else if (!strcmp(arg, "TSR")) {
		if (user_cmd & USR_CMD_EXAMPLE_DEINIT) {
			printf("reinit %s\r\n", example);
			sys_reset();
		} else {
			printf("invalid state, can not do %s reinit!\r\n", example);
		}
	} else {
		printf("invalid cmd\r\n");
	}

	printf("user command 0x%lx\r\n", user_cmd);
}

static log_item_t userctrl_items[] = {
	{"UC", fUC, },
};

static void atcmd_userctrl_init(void)
{
	log_service_add_table(userctrl_items, sizeof(userctrl_items) / sizeof(userctrl_items[0]));
}
#endif