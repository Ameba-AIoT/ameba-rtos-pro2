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

/*
Usage Guide:
1. Please modify sensor driver setting and enable FCS bootup. Only support imx681 and imx471.
	project\realtek_amebapro2_v0_example\inc\sensor.h
	static const unsigned char sen_id[SENSOR_MAX] = {
		SENSOR_DUMMY,
		SENSOR_IMX471,
		SENSOR_IMX471_12M,
		SENSOR_IMX471_12M_SEQ,
	};
	#define USE_SENSOR      	SENSOR_IMX471
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

static void atcmd_userctrl_init(void);
static mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *filesaver_ctx			= NULL;
static mm_siso_t *siso_video_filesaver_v1	= NULL;

/*
allocate virt addr and free virt addr
dma use phy addr
fill data in phy addr
ret: phy_addr
*/
typedef struct {
	void *virt_addr;
	void *phy_addr;
} rtscam_dma_item_t;

#define JPEG_CHANNEL 0
#define JPEG_WIDTH	sensor_params[USE_SENSOR].sensor_width
#define JPEG_HEIGHT	sensor_params[USE_SENSOR].sensor_height
#define JPEG_FPS	sensor_params[USE_SENSOR].sensor_fps

//set output resolution to high reesolution
#define OUT_IMG_WIDTH sensor_params[sen_id[2]].sensor_width
#define OUT_IMG_HEIGHT sensor_params[sen_id[2]].sensor_height
#define OUT_IMG_OVERLAP_WIDTH (((sensor_params[sen_id[3]].sensor_width * 2) - sensor_params[sen_id[2]].sensor_width) / 2)
static uint8_t *hr_nv12_image = NULL;
static uint32_t hr_nv12_size = OUT_IMG_WIDTH * OUT_IMG_HEIGHT * 3 / 2;
#define SAVE_DBG_IMG 0
#define SAVE_JPEG 1 // SAVE_JPEG = 0, save NV12 format
#define BURST_MODE_MAX_COUNT 1 //when set to 1, disable burst mode. for DDR 128M, maaximun can set to 2
static int raw_index = 0;
static rtscam_dma_item_t splited_raw_image[BURST_MODE_MAX_COUNT][2] = {0};

static video_params_t video_v1_params = {
	.stream_id 	= JPEG_CHANNEL,
	.type 		= VIDEO_H264,
	.width 		= JPEG_WIDTH,
	.height 	= JPEG_HEIGHT,
	.fps 		= JPEG_FPS,
	.use_static_addr = 1,
};

/*
OUT_IMG_WIDTH => full width
h => full height
*/
__attribute__((optimize("-O2")))
static int yuv420stitch_step(uint8_t *tiled_yuv, uint8_t *output_buf, int w, int h, int overlap_width, uint32_t *out_size, int is_right)
{
	uint32_t yuv420sp_size = w * h * 3 / 2;
	uint8_t *output_pos = output_buf;
	uint8_t *in_pos = tiled_yuv;
	uint16_t w_tiled = (w / 2 + overlap_width);
	uint16_t w_half = w / 2;
	if (is_right) {
		output_pos += w_half;
	} else {
	}
	for (int l = 0; l < h * 3 / 2; l++) {
		if (is_right) {
			memcpy(output_pos, in_pos + overlap_width, w_half);
		}
		else {
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
	if ((int)dma_item->virt_addr & (align_size-1)) {
		dma_item->phy_addr = (void*)(((int)dma_item->virt_addr + align_size) & (-align_size) /*& ~0x80000000*/ );
	} else {
		dma_item->phy_addr = (void*)((int)dma_item->virt_addr /*& ~0x80000000*/ );
	}
	//printf("{%s} 0x%08x \r\n", __func__, (uint32_t)*phy_addr);
	return dma_item->phy_addr;
}

static void free_dma(rtscam_dma_item_t *dma_item)
{
	if(dma_item->virt_addr) {
		free(dma_item->virt_addr);
	}	
	dma_item->phy_addr = 0;
	dma_item->virt_addr = 0;
}

static void config_verification_path_buf(struct verify_ctrl_config *v_cfg, uint32_t img_buf_addr0, uint32_t img_buf_addr1,
	uint32_t w, uint32_t h, uint32_t overlap_width, bool is_right)
{
	uint32_t buf_size = (w / 2 + overlap_width) * h * 2;
	uint32_t y_len, uv_len;
	y_len = (w / 2 + overlap_width) * h;
	uv_len = y_len;

	if(v_cfg == NULL) {
		printf("[%s] fail\r\n", __FUNCTION__);
		return;
	}

	v_cfg->verify_addr0 = img_buf_addr0;
	v_cfg->verify_addr1 = img_buf_addr1;
	v_cfg->verify_ylen = y_len;
	v_cfg->verify_uvlen = uv_len;

	// Setup NLSC center of second image of verification path. Not working for first frame.
	uint32_t center_x, center_y;
	if (is_right) {
		center_x = overlap_width;
	} else {
		center_x = w / 2;
	}
	center_y = h / 2;

	v_cfg->verify_r_center.x = center_x;
	v_cfg->verify_r_center.y = center_y;
	v_cfg->verify_g_center.x = center_x;
	v_cfg->verify_g_center.y = center_y;
	v_cfg->verify_b_center.x = center_x;
	v_cfg->verify_b_center.y = center_y;

	SCB_CleanDCache_by_Addr((uint32_t *)img_buf_addr0, y_len + uv_len);
	SCB_CleanDCache_by_Addr((uint32_t *)img_buf_addr1, y_len + uv_len);
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
enum file_process_option { //1: 12M raw, 2: left 6M nv12, 3: right: right 6M nv12
	FILE_PROCESS_DONE = 0,
	SPLIT_RAW_IMAGE,
	MERGE_LEFT_NV12_SKIP_FIRST,
	MERGE_LEFT_NV12,
	MERGE_RIGHT_NV12_SKIP_FIRST,
	MERGE_RIGHT_NV12
};
static enum file_process_option file_process_option = FILE_PROCESS_DONE;
static void file_process(char *file_path, uint32_t data_addr, uint32_t data_size)
{
	printf("[%s] 0x%x, data len = %lu\r\n",__FUNCTION__, data_addr, data_size);
	TickType_t start_ts, end_ts;
	char *img_buf = (char*)data_addr;
	uint32_t img_buf_size, tiled_img_size;
	img_buf_size = OUT_IMG_WIDTH * OUT_IMG_HEIGHT * 2;
	uint32_t out_size;

	if(file_process_option == MERGE_LEFT_NV12_SKIP_FIRST) {
		file_process_option = MERGE_LEFT_NV12;
		return;
	}
	if(file_process_option == MERGE_RIGHT_NV12_SKIP_FIRST) {
		file_process_option = MERGE_RIGHT_NV12;
		return;
	}

	if(file_process_option == SPLIT_RAW_IMAGE) {
		//get 12M raw.
		printf("12M raw 0x%x, data len = %lu\r\n", data_addr, data_size);
#if SAVE_DBG_IMG
		if(raw_image == NULL) {
			raw_image = malloc(data_size);
		}
		raw_image_size = data_size;
		if(raw_image) {
			memcpy(raw_image, (uint8_t*)data_addr, data_size);
			raw_reform(raw_image, (uint8_t*)data_addr, raw_image_size);
		} else {
			printf("raw image malloc fail\r\n");
			return; 
		}
#endif
		//split 12M raw into two 6M raw.
		uint8_t *tiled_raws[2];
		tiled_raws[0] = splited_raw_image[raw_index][0].phy_addr;
		tiled_raws[1] = splited_raw_image[raw_index][1].phy_addr;
#if USE_SENSOR == SENSOR_IMX681
		cap_raw_tiling_with_remosaic((uint8_t*)data_addr, tiled_raws, OUT_IMG_WIDTH, OUT_IMG_HEIGHT, OUT_IMG_OVERLAP_WIDTH, REMOSAIC_DISABLE, REMOSAIC_DIRECT_MODE, GR);
#else
		cap_raw_tiling_with_remosaic((uint8_t*)data_addr, tiled_raws, OUT_IMG_WIDTH, OUT_IMG_HEIGHT, OUT_IMG_OVERLAP_WIDTH, REMOSAIC_ENABLE, REMOSAIC_DETECT_MODE, GR);
#endif
		printf("img_left: %x\n\r", splited_raw_image[raw_index][0].phy_addr);
		printf("img_right: %x\n\r", splited_raw_image[raw_index][1].phy_addr);
	} else if(file_process_option == MERGE_LEFT_NV12) {
		//deal with left 6M NV12 
		//merge 2 * 6M to 12M NV12 image
		uint8_t *img_pos = (uint8_t *)img_buf;
		uint8_t *output_pos = (uint8_t *)hr_nv12_image;
		yuv420stitch_step(img_pos, hr_nv12_image, OUT_IMG_WIDTH, OUT_IMG_HEIGHT, OUT_IMG_OVERLAP_WIDTH, &out_size, 0);
	} else if(file_process_option == MERGE_RIGHT_NV12) {
		//deal with right 6M NV12
		//merge 2 * 6M to 12M NV12 image
		uint8_t *img_pos = (uint8_t *)img_buf;
		uint8_t *output_pos = (uint8_t *)hr_nv12_image;
		yuv420stitch_step(img_pos, hr_nv12_image, OUT_IMG_WIDTH, OUT_IMG_HEIGHT, OUT_IMG_OVERLAP_WIDTH, &out_size, 1);
	}
	file_process_option = FILE_PROCESS_DONE;
}

static int hr_init_ae_awb(video_pre_init_params_t *init_params, int wait_ae_timeout)
{
	video_v1_params.direct_output = 1;
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
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, JPEG_CHANNEL);

	//set ae, awb init parameters for high resolution (12M) snapshot
	init_params->isp_ae_enable = 1;
	init_params->isp_ae_init_exposure = ae_time;
	init_params->isp_ae_init_gain = ae_gain;
	init_params->isp_awb_enable = 1;
	init_params->isp_awb_init_rgain = awb_rgain;
	init_params->isp_awb_init_bgain = awb_bgain;
	return OK;
}

static int hr_raw_capture(video_pre_init_params_t *init_params, int proc_raw_idx)
{
	int ret = OK;

	// get 12M raw
	int sensor_id = 2;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_SENSOR_ID, sensor_id);
	init_params->isp_init_raw = 1;
	init_params->isp_raw_mode_tnr_dis = 1;
	init_params->video_drop_enable = 0;
	init_params->dyn_iq_mode = 0;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_PRE_INIT_PARM, (int)init_params);
	video_v1_params.direct_output = 0;
	video_v1_params.out_mode = 2; //set to contiuous mode
	video_v1_params.width = sensor_params[sen_id[sensor_id]].sensor_width;
	video_v1_params.height = sensor_params[sen_id[sensor_id]].sensor_height;
	video_v1_params.fps = sensor_params[sen_id[sensor_id]].sensor_fps;
	video_v1_params.type = VIDEO_NV16;
	video_v1_params.ext_fmt = 0;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
	
	//split 12M raw into 2 * 6M raw
	file_process_option = SPLIT_RAW_IMAGE;
	raw_index = proc_raw_idx;
	video_set_isp_ch_buf(JPEG_CHANNEL, 1);
	int timeout_count = 0;
	if(mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, JPEG_CHANNEL) != OK) {
		return NOK;
	}
	while(file_process_option != FILE_PROCESS_DONE) {
		vTaskDelay(1);
		timeout_count++;
		if(timeout_count > 100000) {
			printf("wait hr raw timeout\r\n");
			ret = NOK;
			break;
		}
	}
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, JPEG_CHANNEL);
#if SAVE_DBG_IMG
	if(ret == OK) {
		char rawfilename[] = "sd:/12M.raw";
		save_file_to_sd(rawfilename,  (uint8_t *)raw_image, raw_image_size);
		free(raw_image);
		raw_image = NULL;
		printf("save %s\r\n", rawfilename);
	}
#endif
	return ret;
}

static int hr_raw_to_nv12(video_pre_init_params_t *init_params, int proc_raw_idx)
{
	//switch to verify sequece driver
	int sensor_id = 3;
	int ret = OK;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_SENSOR_ID, sensor_id);
	if(init_params->v_cfg == NULL) {
		init_params->v_cfg = malloc(sizeof(struct verify_ctrl_config));
	}
	//sent 2 * 6M raw to voe
	config_verification_path_buf(init_params->v_cfg, (uint32_t) splited_raw_image[proc_raw_idx][0].phy_addr, (uint32_t) splited_raw_image[proc_raw_idx][0].phy_addr, OUT_IMG_WIDTH, OUT_IMG_HEIGHT, OUT_IMG_OVERLAP_WIDTH, false);
	init_params->isp_init_raw = 0;
	init_params->isp_raw_mode_tnr_dis = 0;
	init_params->dyn_iq_mode = 1;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_PRE_INIT_PARM, (int)init_params);
	video_v1_params.direct_output = 0;
	video_v1_params.out_mode = 2; //set to contiuous mode
	video_v1_params.width = sensor_params[sen_id[sensor_id]].sensor_width;
	video_v1_params.height = sensor_params[sen_id[sensor_id]].sensor_height;
	video_v1_params.fps = sensor_params[sen_id[sensor_id]].sensor_fps;
	video_v1_params.type = VIDEO_NV12;
	video_v1_params.ext_fmt = 0;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);

	//merge output 2 * 6M nv12 to 12M nv12 image
	file_process_option = MERGE_LEFT_NV12_SKIP_FIRST;
	video_set_isp_ch_buf(JPEG_CHANNEL, 2);
	int timeout_count = 0;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, JPEG_CHANNEL);
	while(file_process_option != FILE_PROCESS_DONE) {
		vTaskDelay(1);
		timeout_count++;
		if(timeout_count > 100000) {
			printf("hr nv12 convert timeout\r\n");
			ret = NOK;
			break;
		}
	}
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, JPEG_CHANNEL);

	//right nv12
	config_verification_path_buf(init_params->v_cfg, (uint32_t) splited_raw_image[proc_raw_idx][1].phy_addr, (uint32_t) splited_raw_image[proc_raw_idx][1].phy_addr, OUT_IMG_WIDTH, OUT_IMG_HEIGHT, OUT_IMG_OVERLAP_WIDTH, true);
	init_params->isp_init_raw = 0;
	init_params->isp_raw_mode_tnr_dis = 0;
	init_params->dyn_iq_mode = 1;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_PRE_INIT_PARM, (int)init_params);
	file_process_option = MERGE_RIGHT_NV12_SKIP_FIRST;
	timeout_count = 0;
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, JPEG_CHANNEL);
	while(file_process_option != FILE_PROCESS_DONE) {
		vTaskDelay(1);
		timeout_count++;
		if(timeout_count > 100000) {
			printf("hr nv12 convert timeout\r\n");
			ret = NOK;
			break;
		}
	}
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, JPEG_CHANNEL);

	if(init_params->v_cfg) {
		free(init_params->v_cfg);
		init_params->v_cfg = NULL;
	}
	return ret;
}

#if SAVE_JPEG
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
#endif

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
	printf("output resolution w=%d, h=%d, overlap_width=%d\r\n", OUT_IMG_WIDTH, OUT_IMG_HEIGHT, OUT_IMG_OVERLAP_WIDTH);
	printf("Available heap 0x%x\r\n", xPortGetFreeHeapSize());
	
	//prevent memory fragment, allocate hr splited raw buffer
	for(int i = 0; i < BURST_MODE_MAX_COUNT; i++) {
		uint8_t *tiled_raws[2];
		int tiled_w = OUT_IMG_WIDTH / 2 + OUT_IMG_OVERLAP_WIDTH;
		uint32_t tiled_img_size = tiled_w * OUT_IMG_HEIGHT * 2;
		tiled_raws[0] = alloc_dma(&(splited_raw_image[i][0]), tiled_img_size);
		tiled_raws[1] = alloc_dma(&(splited_raw_image[i][1]), tiled_img_size);
		if(!tiled_raws[0] || !tiled_raws[1]) {
			printf("raw image malloc failed %p %p\n", tiled_raws[0], tiled_raws[1]);
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

	int sensor_id = 0, timeout_count = 0;
	video_pre_init_params_t init_params;
	memset(&init_params, 0x00, sizeof(video_pre_init_params_t));
	init_params.isp_init_enable = 1;
	init_params.init_isp_items.init_brightness = 0;
	init_params.init_isp_items.init_contrast = 50;
	init_params.init_isp_items.init_flicker = 2;
	init_params.init_isp_items.init_hdr_mode = 0;
	init_params.init_isp_items.init_mirrorflip = 0xf0;
	init_params.init_isp_items.init_saturation = 50;
	init_params.init_isp_items.init_wdr_mode = 0; //12M not support WDR, disable WDR.
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
#if SAVE_JPEG == 0
		char yuvfilename[] = "sd:/12M.yuv";
		save_file_to_sd(yuvfilename, (uint8_t *)hr_nv12_image, hr_nv12_size);
		printf("save %s\r\n", yuvfilename);
#else
		if(hr_nv12_to_jpeg(&init_params, 10000, i) == NOK) {
			goto mmf2_video_exmaple_v1_shapshot_hr_fail;
		}
#endif
	}
	
mmf2_video_exmaple_v1_shapshot_hr_fail:

	printf("Available heap 0x%x\r\n", xPortGetFreeHeapSize());
	video_voe_release();
	
	for(int i = 0; i < BURST_MODE_MAX_COUNT; i++) {
		free_dma(&(splited_raw_image[i][0]));
		free_dma(&(splited_raw_image[i][1]));
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