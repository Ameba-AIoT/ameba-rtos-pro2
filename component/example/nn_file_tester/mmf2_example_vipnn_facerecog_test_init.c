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
#include "model_mobilefacenet.h"

// NN tester config
#define NN_MODEL_OBJ    mbfacenet_fwfs  /* fix here to choose model: mbfacenet_fwfs */
#define TEST_IMG_WIDTH	112             /* fix here to match model input size */
#define TEST_IMG_HEIGHT	112             /* fix here to match model input size */

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
static mm_context_t *filesaver_ctx      = NULL;
static mm_context_t *vipnn_ctx          = NULL;

static mm_siso_t *siso_fileloader_vipnn = NULL;
static mm_siso_t *siso_vipnn_filesaver  = NULL;

#define LFW_VAL    0x01
#define CFP_VAL    0x02
#define TEST_DATA   LFW_VAL  /* fix here */

#if TEST_DATA==LFW_VAL
#define LFW_FACE_DATASET_NUM    13233
#define DATA_FILELIST_NAME      "lfw_names_list.txt"
#elif TEST_DATA==CFP_VAL
#define LFW_FACE_DATASET_NUM    7000
#define DATA_FILELIST_NAME      "cfp_names_list.txt"
#endif

static int get_line_num_in_sdfile(char *file_name)
{
	char line[128];
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

void mmf2_example_vipnn_facerecog_test_init(void)
{
	// init virtual file system
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
	printf("The file has %d lines, it take %ld ms\r\n", file_count, xTaskGetTickCount() - t0);

	// file loader
	fileloader_ctx = mm_module_open(&fileloader_module);
	if (fileloader_ctx) {
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_PARAMS, (int)&test_image_params);
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_FILE_NUM, (int)file_count);
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_DECODE_PROCESS, (int)ImageDecodeToRGB888planar_ConvertInPlace);
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_FILELIST_NAME, (int)DATA_FILELIST_NAME);
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_READ_MODE, (int)FILELIST_MODE); // FILELIST_MODE, SEQUENCE_MODE

		mm_module_ctrl(fileloader_ctx, MM_CMD_SET_QUEUE_LEN, 1);  //set to 1 when using NN file tester
		mm_module_ctrl(fileloader_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_APPLY, 0);
	} else {
		printf("fileloader open fail\n\r");
		goto mmf2_example_vnn_face_fail;
	}
	printf("fileloader opened\n\r");

	// VIPNN mobilefacenet
	vipnn_ctx = mm_module_open(&vipnn_module);
	if (vipnn_ctx) {
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL_OBJ);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&roi_nn);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_RES_SIZE, sizeof(face_feature_res_t));		// result size
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_RES_MAX_CNT, MAX_DETECT_OBJ_NUM);		// result max count
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_OUTPUT, 1);  //enable module output
		mm_module_ctrl(vipnn_ctx, MM_CMD_SET_QUEUE_LEN, 1);  //set to 1 when using NN file tester
		mm_module_ctrl(vipnn_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN open fail\n\r");
		goto mmf2_example_vnn_face_fail;
	}
	printf("VIPNN opened\n\r");

	// file saver
	filesaver_ctx = mm_module_open(&filesaver_module);
	if (filesaver_ctx) {
		mm_module_ctrl(filesaver_ctx, CMD_FILESAVER_SET_TYPE_HANDLER, (int)nn_save_handler);
		mm_module_ctrl(filesaver_ctx, CMD_FILESAVER_APPLY, 0);
	} else {
		printf("filesaver open fail\n\r");
		goto mmf2_example_vnn_face_fail;
	}
	printf("filesaver opened\n\r");


	//--------------Link---------------------------
	siso_fileloader_vipnn = siso_create();
	if (siso_fileloader_vipnn) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_fileloader_vipnn, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_fileloader_vipnn, MMIC_CMD_ADD_INPUT, (uint32_t)fileloader_ctx, 0);
		siso_ctrl(siso_fileloader_vipnn, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 64, 0);
		siso_ctrl(siso_fileloader_vipnn, MMIC_CMD_SET_TASKPRIORITY, 3, 0);
		siso_ctrl(siso_fileloader_vipnn, MMIC_CMD_ADD_OUTPUT, (uint32_t)vipnn_ctx, 0);
		siso_start(siso_fileloader_vipnn);
	} else {
		printf("siso_fileloader_vipnn open fail\n\r");
		goto mmf2_example_vnn_face_fail;
	}
	printf("siso_fileloader_vipnn started\n\r");

	siso_vipnn_filesaver = siso_create();
	if (siso_vipnn_filesaver) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_vipnn_filesaver, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_vipnn_filesaver, MMIC_CMD_ADD_INPUT, (uint32_t)vipnn_ctx, 0);
		siso_ctrl(siso_vipnn_filesaver, MMIC_CMD_ADD_OUTPUT, (uint32_t)filesaver_ctx, 0);
		siso_start(siso_vipnn_filesaver);
	} else {
		printf("siso_vipnn_filesaver open fail\n\r");
		goto mmf2_example_vnn_face_fail;
	}
	printf("siso_vipnn_filesaver started\n\r");

mmf2_example_vnn_face_fail:

	return;
}

static void set_nn_roi(int w, int h)
{
	roi_nn.img.width = w;
	roi_nn.img.height = h;
	roi_nn.img.roi.xmax = w;
	roi_nn.img.roi.ymax = h;
	mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&roi_nn);
}

static int ImageDecodeToRGB888planar_ConvertInPlace(void *pbuffer, void *pbuffer_size)
{
	uint8_t *pImageBuf = (uint8_t *)pbuffer;
	uint32_t *pImageSize = (uint32_t *)pbuffer_size;

	int w, h, c;
	int channels = 3;
	uint8_t *im_data = stbi_load_from_memory(pImageBuf, *pImageSize, &w, &h, &c, channels);
	printf("\r\nimage data size: w:%d, h:%d, c:%d\r\n", w, h, c);

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

	return 1;
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

static int (*media_save_file)(char *file_name, char *data_buf, int data_buf_size) = sd_save_file;
static void nn_save_handler(char *file_name, uint32_t data_addr, uint32_t data_size)
{
	vipnn_out_buf_t *pre_tensor_out = (vipnn_out_buf_t *)data_addr;
	int face_cnt = pre_tensor_out->res_cnt;
	face_feature_res_t *ff_res = (face_feature_res_t *)&pre_tensor_out->res[0];

	char nn_fn[128];
	memset(&nn_fn[0], 0x00, sizeof(nn_fn));

	snprintf(&nn_fn[0], sizeof(nn_fn), "%s.bin", strip_filename_extention(file_name), "r");

	int ret = media_save_file(nn_fn, (char *)ff_res[0].feature, sizeof(ff_res[0].feature));
	if (ret < 0) {
		printf("Fail to save file\r\n");
		return;
	}

	printf("[nn_save_handler]save feature to %s\r\n", nn_fn);
}

/*-----------------------------------------------------------------------------------*/

static int sd_save_file(char *file_name, char *data_buf, int data_buf_size)
{
	char fn[128];
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
