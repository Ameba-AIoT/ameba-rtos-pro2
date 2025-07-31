/******************************************************************************
*
* Copyright(c) 2007 - 2023 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "module_vipnn.h"
#include "module_fileloader.h"
#include "module_filesaver.h"
#include "avcodec.h"
#include "model_scrfd.h"

// NN tester config
#define NN_MODEL_OBJ   scrfd_fwfs           /* fix here to choose model: scrfd_fwfs */
#define TEST_IMG_WIDTH	640                 /* fix here to match model input size */
#define TEST_IMG_HEIGHT	640                 /* fix here to match model input size */
static float nn_confidence_thresh = 0.2;    /* fix here to set score threshold */
static float nn_nms_thresh = 0.7;           /* fix here to set nms threshold */

static nn_data_param_t roi_nn = {
	.img = {
		.width = TEST_IMG_WIDTH,
		.height = TEST_IMG_HEIGHT,
		.roi = {
			.xmin = 0,
			.ymin = 0,
			.xmax = TEST_IMG_WIDTH,
			.ymax = TEST_IMG_HEIGHT,
		}
	},
	.codec_type = AV_CODEC_ID_RGB888
};

static fileloader_params_t test_image_params = {
	.codec_id = AV_CODEC_ID_JPEG       /* Fix me (AV_CODEC_ID_BMP or AV_CODEC_ID_JPEG) */
};

static int ImageDecodeToRGB888planar_ConvertInPlace(void *pbuffer, void *pbuffer_size);
static void nn_save_handler(char *file_name, uint32_t data_addr, uint32_t data_size);
static int sd_save_file(char *file_name, char *data_buf, int data_buf_size);
uint8_t *stbi_load_from_memory(uint8_t const *buffer, int len, int *x, int *y, int *channels_in_file, int desired_channels);
void stbi_image_free(void *retval_from_stbi_load);

static mm_context_t *fileloader_ctx     = NULL;
static mm_context_t *facedet_ctx        = NULL;
static mm_context_t *filesaver_ctx      = NULL;

static mm_siso_t *siso_file_facedet     = NULL;
static mm_siso_t *siso_facedet_filesave = NULL;

#define WIDER_FACE_VAL      0x00
#define FDDB_VAL            0x01
#define TEST_DATA           WIDER_FACE_VAL  /* fix here */

#if TEST_DATA==WIDER_FACE_VAL
#define FACE_DATASET_NUM    3226
#define DATA_FILELIST_NAME  "widerface_names_list.txt"
#elif TEST_DATA==FDDB_VAL
#define FACE_DATASET_NUM    2845
#define DATA_FILELIST_NAME  "fddb_names_list.txt"
#endif

static int get_line_num_in_sdfile(char *file_name)
{
	char line[256];
	memset(line, 0, sizeof(line));
	char file_path[64];
	memset(file_path, 0, sizeof(file_path));
	snprintf(file_path, sizeof(file_path), "%s%s", "sd:/", file_name);
	int count = 0;
	if (access(file_path, F_OK) != 0) {
		printf("file list not exists\r\n");
		return -1;
	}
	FILE *f = fopen(file_path, "r");
	while (fgets(line, sizeof(line), f)) {
		count++;
		//printf("[line %d] %s\r\n", count, line);
		memset(line, 0, sizeof(line));
	}
	fclose(f);

	return count;
}

void mmf2_example_vipnn_facedet_test_init(void)
{
	vfs_init(NULL);
	if (vfs_user_register("sd", VFS_FATFS, VFS_INF_SD) != 0) {
		printf("fail to register SD vfs\r\n");
		return;
	}
	// get test file num on list
	printf("Getting data set image number in list......\r\n");
	uint32_t t0 = xTaskGetTickCount();
	int file_count = get_line_num_in_sdfile((char *)DATA_FILELIST_NAME);
	if (file_count < 0) {
		printf("fail to get line numbers\r\n");
		return;
	}
	printf("The file has %d lines, it take %d ms\r\n", file_count, xTaskGetTickCount() - t0);

	// file loader
	fileloader_ctx = mm_module_open(&fileloader_module);
	if (fileloader_ctx) {
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_PARAMS, (int)&test_image_params);
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_FILE_NUM, (int)file_count);
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_DECODE_PROCESS, (int)ImageDecodeToRGB888planar_ConvertInPlace);
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_READ_MODE, (int)FILELIST_MODE); // FILELIST_MODE, SEQUENCE_MODE
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_FILELIST_NAME, (int)DATA_FILELIST_NAME);

		mm_module_ctrl(fileloader_ctx, MM_CMD_SET_QUEUE_LEN, 1);  //set to 1 when using NN file tester
		mm_module_ctrl(fileloader_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_APPLY, 0);
	} else {
		printf("fileloader open fail\n\r");
		goto mmf2_example_face_detect_file_test_fail;
	}
	printf("fileloader opened\n\r");

	// VIPNN face detection
	facedet_ctx = mm_module_open(&vipnn_module);
	if (facedet_ctx) {
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL_OBJ);
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&roi_nn);
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_RES_SIZE, sizeof(facedetect_res_t));		// result size
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_RES_MAX_CNT, MAX_DETECT_OBJ_NUM);		// result max count
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_CONFIDENCE_THRES, (int)&nn_confidence_thresh);
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_NMS_THRES, (int)&nn_nms_thresh);
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_OUTPUT, 1);  //enable module output
		mm_module_ctrl(facedet_ctx, MM_CMD_SET_QUEUE_LEN, 1);  //set to 1 when using NN file tester
		mm_module_ctrl(facedet_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(facedet_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN open fail\n\r");
		goto mmf2_example_face_detect_file_test_fail;
	}
	printf("VIPNN opened\n\r");

	// file saver
	filesaver_ctx = mm_module_open(&filesaver_module);
	if (filesaver_ctx) {
		mm_module_ctrl(filesaver_ctx, CMD_FILESAVER_SET_TYPE_HANDLER, (int)nn_save_handler);
		mm_module_ctrl(filesaver_ctx, CMD_FILESAVER_APPLY, 0);
	} else {
		printf("filesaver open fail\n\r");
		goto mmf2_example_face_detect_file_test_fail;
	}
	printf("filesaver opened\n\r");


	//--------------Link---------------------------
	siso_file_facedet = siso_create();
	if (siso_file_facedet) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_file_facedet, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_file_facedet, MMIC_CMD_ADD_INPUT, (uint32_t)fileloader_ctx, 0);
		siso_ctrl(siso_file_facedet, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 128, 0);
		siso_ctrl(siso_file_facedet, MMIC_CMD_SET_TASKPRIORITY, 3, 0);
		siso_ctrl(siso_file_facedet, MMIC_CMD_ADD_OUTPUT, (uint32_t)facedet_ctx, 0);
		siso_start(siso_file_facedet);
	} else {
		printf("siso_file_facedet open fail\n\r");
		goto mmf2_example_face_detect_file_test_fail;
	}
	printf("siso_file_facedet started\n\r");

	siso_facedet_filesave = siso_create();
	if (siso_facedet_filesave) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_facedet_filesave, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_facedet_filesave, MMIC_CMD_ADD_INPUT, (uint32_t)facedet_ctx, 0);
		siso_ctrl(siso_facedet_filesave, MMIC_CMD_ADD_OUTPUT, (uint32_t)filesaver_ctx, 0);
		siso_ctrl(siso_facedet_filesave, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 128, 0);
		siso_start(siso_facedet_filesave);
	} else {
		printf("siso_facedet_filesave open fail\n\r");
		goto mmf2_example_face_detect_file_test_fail;
	}
	printf("siso_facedet_filesave started\n\r");

	return;
mmf2_example_face_detect_file_test_fail:

	return;
}

static void set_nn_roi(int w, int h)
{
	roi_nn.img.width = w;
	roi_nn.img.height = h;
	roi_nn.img.roi.xmax = w;
	roi_nn.img.roi.ymax = h;
	mm_module_ctrl(facedet_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&roi_nn);
}

static int ImageDecodeToRGB888planar_ConvertInPlace(void *pbuffer, void *pbuffer_size)
{
	uint8_t *pImageBuf = (uint8_t *)pbuffer;
	uint32_t *pImageSize = (uint32_t *)pbuffer_size;

	int w, h, c;
	int channels = 3;
	uint8_t *im_data = stbi_load_from_memory(pImageBuf, *pImageSize, &w, &h, &c, channels);
	printf("\r\nimage data size: w:%d, h:%d, c:%d\r\n", w, h, c);

	if (c != 1 && c != 3) {
		printf("error: it's not an image file\r\n");
		return -1;
	}

	/* set nn roi according to image size */
	set_nn_roi(w, h);

	/* rgb packed to rgb planar */
	int data_size = w * h * c;
	uint8_t *rgb_planar_buf = (uint8_t *)malloc(data_size);
	for (int k = 0; k < c; k++) {
		for (int j = 0; j < h; j++) {
			for (int i = 0; i < w; i++) {
				int dst_i = i + w * j + w * h * k;
				int src_i = k + c * i + c * w * j;
				rgb_planar_buf[dst_i] = im_data[src_i];
			}
		}
	}
	memcpy(pImageBuf, rgb_planar_buf, data_size);
	*pImageSize = (uint32_t) data_size;

	free(rgb_planar_buf);
	stbi_image_free(im_data);

	return 0;
}

static int (*media_save_file)(char *file_name, char *data_buf, int data_buf_size) = sd_save_file;

static void get_fddb_result_format(void *p, char *src_filename, char *dst_strbuf, int buf_len, int *data_len)
{
	vipnn_out_buf_t *out = (vipnn_out_buf_t *)p;
	facedetect_res_t *fd_res = (facedetect_res_t *)&out->res[0];

	int curr = 0;
	curr += snprintf(&dst_strbuf[curr], buf_len - curr, "%s\n", src_filename);
	curr += snprintf(&dst_strbuf[curr], buf_len - curr, "%d\n", out->res_cnt);

	float im_w = (float)out->input_param->img.width;
	float im_h = (float)out->input_param->img.height;
	for (int i = 0; i < out->res_cnt; i++) {
#if 0
		float top_x = fd_res[i].result[2] * im_w;
		float top_y = fd_res[i].result[3] * im_h;
		float w = fd_res[i].result[4] * im_w - top_x;
		float h = fd_res[i].result[5] * im_h - top_y;
#else
		float top_x = fd_res[i].result[2];
		float top_y = fd_res[i].result[3];
		float w = fd_res[i].result[4] - top_x;
		float h = fd_res[i].result[5] - top_y;
#endif
		float score = fd_res[i].result[1];
		curr += snprintf(&dst_strbuf[curr], buf_len - curr, "%f %f %f %f %f\n", top_x, top_y, w, h, score);
	}

	printf("dst_strbuf: %s\r\n", dst_strbuf);

	*data_len = curr;
}

static void get_widerface_result_format(void *p, char *src_filename, char *dst_strbuf, int buf_len, int *data_len)
{
	//vipnn_out_buf_t *vipnn_res = (vipnn_out_buf_t *)p;
	//facedetect_res_t *fd_res = &vipnn_res->vipnn_res.fd_res;

	vipnn_out_buf_t *out = (vipnn_out_buf_t *)p;
	facedetect_res_t *fd_res = (facedetect_res_t *)&out->res[0];

	int curr = 0;
	curr += snprintf(&dst_strbuf[curr], buf_len - curr, "%s\n", src_filename);
	curr += snprintf(&dst_strbuf[curr], buf_len - curr, "%d\n",  out->res_cnt);

	// scale/resize back to original image
	float im_ori_w = (float)out->input_param->img.width;
	float im_ori_h = (float)out->input_param->img.height;

	int new_w, new_h;
	if (((float)TEST_IMG_WIDTH / im_ori_w) < ((float)TEST_IMG_HEIGHT / im_ori_h)) {
		new_w = TEST_IMG_WIDTH;
		new_h = (im_ori_h * TEST_IMG_WIDTH) / im_ori_w;
	} else {
		new_h = TEST_IMG_HEIGHT;
		new_w = (im_ori_w * TEST_IMG_HEIGHT) / im_ori_h;
	}

	//letterbox padding offset
	float x_off = ((float)TEST_IMG_WIDTH - new_w) / 2;
	float y_off = ((float)TEST_IMG_HEIGHT - new_h) / 2;

	//mapping bbox position back to original image
	for (int i = 0; i < out->res_cnt; i++) {

		float top_x = (fd_res[i].result[2] * TEST_IMG_WIDTH - x_off) / new_w * im_ori_w;
		float top_y = (fd_res[i].result[3] * TEST_IMG_HEIGHT - y_off) / new_h * im_ori_h;
		float bottom_x = (fd_res[i].result[4] * TEST_IMG_WIDTH - x_off) / new_w * im_ori_w;
		float bottom_y = (fd_res[i].result[5] * TEST_IMG_HEIGHT - y_off) / new_h * im_ori_h;
		float w = bottom_x - top_x;
		float h = bottom_y - top_y;

		float score = fd_res[i].result[1];
		curr += snprintf(&dst_strbuf[curr], buf_len - curr, "%.2f %.2f %.2f %.2f %.2f\n", top_x, top_y, w, h, score);
	}

	printf("result:\r\n%s\r\n", dst_strbuf);

	*data_len = curr;
}

static char *strip_filename_extention(char *filename)
{
	char *end = filename + strlen(filename);

	while (end > filename && *end != '.') {
		--end;
	}

	if (end > filename) {
		*end = '\0';
	}
	return filename;
}

static void nn_save_handler(char *file_name, uint32_t data_addr, uint32_t data_size)
{
	//vipnn_out_buf_t pre_tensor_out;
	//memcpy(&pre_tensor_out, data_addr, data_size);

	vipnn_out_buf_t *out = (vipnn_out_buf_t *)data_addr;
	facedetect_res_t *fd_res = (facedetect_res_t *)&out->res[0];

	char nn_fn[256];
	memset(&nn_fn[0], 0x00, sizeof(nn_fn));

	/* set result file name as .txt */
	snprintf(nn_fn, sizeof(nn_fn), "%s.txt", strip_filename_extention(file_name));

	/* get formatted result string */
	char dst_result_buf[32 * 1024];
	int data_len = 0;
#if TEST_DATA==WIDER_FACE_VAL
	get_widerface_result_format((void *)out, file_name, dst_result_buf, sizeof(dst_result_buf), &data_len);
#elif TEST_DATA==FDDB_VAL
	get_fddb_result_format((void *)out, file_name, dst_result_buf, sizeof(dst_result_buf), &data_len);
#endif

	media_save_file(nn_fn, dst_result_buf, data_len);
}

/*-----------------------------------------------------------------------------------*/

static int sd_save_file(char *file_name, char *data_buf, int data_buf_size)
{
	char fn[256];
	snprintf(fn, sizeof(fn), "%s%s", "sd:/", file_name);

	FILE *fp;
	fp = fopen(fn, "wb+");
	if (fp == NULL) {
		printf("fail to open file.\r\n");
		return -1;
	}
	fwrite(data_buf, data_buf_size, 1, fp);
	fclose(fp);

	printf("save file to %s\r\n", fn);

	return 0;
}

/*-----------------------------------------------------------------------------------*/
