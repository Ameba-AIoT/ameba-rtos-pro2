/******************************************************************************
*
* Copyright(c) 2007 - 2023 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include <string.h>

#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "module_video.h"
#include "avcodec.h"

#include "module_vipnn.h"
#include "module_fileloader.h"
#include "module_filesaver.h"
#include "log_service.h"

#include "avcodec.h"
#include "vfs.h"
#include "cJSON.h"
#include "FtpClient.h"

#include "model_yolo.h"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_STDIO
#define STBI_ONLY_BMP
#define STBI_ONLY_JPEG
#include "../image/3rdparty/stb/stb_image.h"

static char coco_name[80][20] =
{ "person",    "bicycle",    "car",    "motorbike",    "aeroplane",    "bus",    "train",    "truck",    "boat",    "traffic light",    "fire hydrant",    "stop sign",    "parking meter",    "bench",    "bird",    "cat",    "dog",    "horse",    "sheep",    "cow",    "elephant",    "bear",    "zebra",    "giraffe",    "backpack",    "umbrella",    "handbag",    "tie",    "suitcase",    "frisbee",    "skis",    "snowboard",    "sports ball",    "kite",    "baseball bat",    "baseball glove",    "skateboard",    "surfboard",    "tennis racket",    "bottle",    "wine glass",    "cup",    "fork",    "knife",    "spoon",    "bowl",    "banana",    "apple",    "sandwich",    "orange",    "broccoli",    "carrot",    "hot dog",    "pizza",    "donut",    "cake",    "chair",    "sofa",    "pottedplant",    "bed",    "diningtable",    "toilet",    "tvmonitor",    "laptop",    "mouse",    "remote",    "keyboard",    "cell phone",    "microwave",    "oven",    "toaster",    "sink",    "refrigerator",    "book",    "clock",    "vase",    "scissors",    "teddy bear",    "hair drier",    "toothbrush" };

// NN tester config
#define NN_MODEL_OBJ        yolov4_tiny     /* fix here to choose model: yolov4_tiny, yolov7_tiny */
#define NN_DATASET_LABEL    coco_name       /* fix here to define label */
#define TEST_IMAGE_WIDTH	416             /* fix here to match model input size */
#define TEST_IMAGE_HEIGHT	416             /* fix here to match model input size */
static float nn_confidence_thresh = 0.2;    /* fix here to set score threshold */
static float nn_nms_thresh = 0.3;           /* fix here to set nms threshold */

static int set_remote_host = 0;
static char remote_host[] = "172.21.34.35\0\0\0\0";

static ftp_info_t   ftp_info = {
	.remote_ip = (char *)"172.21.34.35\0                                                  \0",
	.remote_port = 21,
	.remote_dir = (char *)"/dataset\0                                                     \0",
	.remote_user = (char *)"ftpuser\0                                                     \0",
	.remote_pass = (char *)"12345678\0                                                    \0"
};

#define SAVE_COCO_FORMAT    1

nn_data_param_t roi_tester = {
	.img = {
		.width = TEST_IMAGE_WIDTH,
		.height = TEST_IMAGE_HEIGHT,
		.roi = {
			.xmin = 0,
			.ymin = 0,
			.xmax = TEST_IMAGE_WIDTH,
			.ymax = TEST_IMAGE_HEIGHT,
		}
	},
	.codec_type = AV_CODEC_ID_RGB888
};

static fileloader_params_t test_image_params = {
	.codec_id = AV_CODEC_ID_JPEG       /* Fix me (AV_CODEC_ID_BMP or AV_CODEC_ID_JPEG) */
};

static int ImageDecodeToRGB888planar_ConvertInPlace(void *pbuffer, void *pbuffer_size);
static char *nn_get_json_format(void *p, int frame_id, char *file_name);
static void nn_save_handler_for_evaluate(char *file_name, uint32_t data_addr, uint32_t data_size);
static int sd_save_file(char *file_name, char *data_buf, int data_buf_size);
static int ftp_save_file(char *file_name, char *data_buf, int data_buf_size);
static void atcmd_userctrl_init(void);

static mm_context_t *fileloader_ctx			= NULL;
static mm_context_t *filesaver_ctx			= NULL;
static mm_context_t *vipnn_ctx              = NULL;

static mm_siso_t *siso_fileloader_vipnn     = NULL;
static mm_siso_t *siso_vipnn_filesaver      = NULL;

#define MODE_SD     0
#define MODE_FTP    1

/* Configure mode */
#define LOADER_MODE MODE_SD  /* Fix me, MODE_SD, MODE_FTP */
#define SAVER_MODE MODE_SD   /* Fix me, MODE_SD, MODE_FTP */

#define NETWORK_MODE (LOADER_MODE==MODE_FTP && SAVER_MODE==MODE_FTP)

#if LOADER_MODE==MODE_SD
#define FILELIST_NAME   "coco_val2017_list.txt"  /* Fix me */
#elif LOADER_MODE == MODE_FTP
#define TEST_FILE_NUM   5000                     /* Fix me */
#define FILELIST_NAME   "image_list.txt"         /* Fix me */
#endif

#define wifi_wait_time 500 //Here we wait 5 second to wiat the fast connect 
//------------------------------------------------------------------------------
// common code for network connection
//------------------------------------------------------------------------------
#include "wifi_conf.h"
#include "lwip_netconf.h"

#if NETWORK_MODE
static void wifi_common_init(void)
{
	uint32_t wifi_wait_count = 0;

	while (!((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) && (*(u32 *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
		vTaskDelay(10);
		wifi_wait_count++;
		if (wifi_wait_count == wifi_wait_time) {
			printf("\r\nuse ATW0, ATW1, ATWC to make wifi connection\r\n");
			printf("wait for wifi connection...\r\n");
		}
	}
}
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

void mmf2_example_vipnn_objectdet_test_init(void)
{
#if NETWORK_MODE
	wifi_common_init();
#endif

#if LOADER_MODE==MODE_SD
	// init virtual file system
	vfs_init(NULL);
	if (vfs_user_register("sd", VFS_FATFS, VFS_INF_SD) != 0) {
		printf("fail to register SD vfs\r\n");
		return;
	}
	// get test file num on list
	printf("Getting data set image number in list......\r\n");
	uint32_t t0 = xTaskGetTickCount();
	int file_count = get_line_num_in_sdfile((char *)FILELIST_NAME);
	if (file_count < 0) {
		printf("fail to get line numbers\r\n");
		return;
	}
	printf("The file has %d lines, it take %ld ms\r\n", file_count, xTaskGetTickCount() - t0);
#elif LOADER_MODE == MODE_FTP
	int file_count = TEST_FILE_NUM;
#endif
	// file loader
	fileloader_ctx = mm_module_open(&fileloader_module);
	if (fileloader_ctx) {
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_PARAMS, (int)&test_image_params);
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_READ_MODE, (int)FILELIST_MODE);
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_FILELIST_NAME, (int)FILELIST_NAME);
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_FILE_NUM, (int)file_count);
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_DECODE_PROCESS, (int)ImageDecodeToRGB888planar_ConvertInPlace);
#if LOADER_MODE == MODE_FTP
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_FTP_MODE, 1);
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_REMOTE_PARAMS, (int)&ftp_info);
		atcmd_userctrl_init();
		printf("Please Enter Remote IP...\r\n");
		while (!set_remote_host) {
			vTaskDelay(1000);
		}
#endif

		mm_module_ctrl(fileloader_ctx, MM_CMD_SET_QUEUE_LEN, 1);  //set to 1 when using NN file tester
		mm_module_ctrl(fileloader_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_APPLY, 0);
	} else {
		printf("fileloader open fail\n\r");
		goto mmf2_example_file_vipnn_tester_fail;
	}
	printf("fileloader opened\n\r");

	// VIPNN
	vipnn_ctx = mm_module_open(&vipnn_module);
	if (vipnn_ctx) {
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL_OBJ);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&roi_tester);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_RES_SIZE, sizeof(objdetect_res_t));		// result size
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_RES_MAX_CNT, MAX_DETECT_OBJ_NUM);		// result max count
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_CONFIDENCE_THRES, (int)&nn_confidence_thresh);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_NMS_THRES, (int)&nn_nms_thresh);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_OUTPUT, 1);  //enable module output
		mm_module_ctrl(vipnn_ctx, MM_CMD_SET_QUEUE_LEN, 1);  //set to 1 when using NN file tester
		mm_module_ctrl(vipnn_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(vipnn_ctx, CMD_VIPNN_APPLY, 0);
	} else {
		printf("VIPNN open fail\n\r");
		goto mmf2_example_file_vipnn_tester_fail;
	}
	printf("VIPNN opened\n\r");

	// file saver
	filesaver_ctx = mm_module_open(&filesaver_module);
	if (filesaver_ctx) {
		mm_module_ctrl(filesaver_ctx, CMD_FILESAVER_SET_TYPE_HANDLER, (int)nn_save_handler_for_evaluate);
		mm_module_ctrl(filesaver_ctx, CMD_FILESAVER_APPLY, 0);
	} else {
		printf("filesaver open fail\n\r");
		goto mmf2_example_file_vipnn_tester_fail;
	}
	printf("filesaver opened\n\r");

	//--------------Link---------------------------

	siso_fileloader_vipnn = siso_create();
	if (siso_fileloader_vipnn) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_fileloader_vipnn, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_fileloader_vipnn, MMIC_CMD_ADD_INPUT, (uint32_t)fileloader_ctx, 0);
		siso_ctrl(siso_fileloader_vipnn, MMIC_CMD_ADD_OUTPUT, (uint32_t)vipnn_ctx, 0);
		siso_ctrl(siso_fileloader_vipnn, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 128, 0);
		siso_start(siso_fileloader_vipnn);
	} else {
		printf("siso_fileloader_vipnn open fail\n\r");
		goto mmf2_example_file_vipnn_tester_fail;
	}
	printf("siso_fileloader_vipnn started\n\r");

	siso_vipnn_filesaver = siso_create();
	if (siso_vipnn_filesaver) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_vipnn_filesaver, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_vipnn_filesaver, MMIC_CMD_ADD_INPUT, (uint32_t)vipnn_ctx, 0);
		siso_ctrl(siso_vipnn_filesaver, MMIC_CMD_ADD_OUTPUT, (uint32_t)filesaver_ctx, 0);
		siso_ctrl(siso_vipnn_filesaver, MMIC_CMD_SET_STACKSIZE, (uint32_t)1024 * 128, 0);
		siso_start(siso_vipnn_filesaver);
	} else {
		printf("siso_vipnn_filesaver open fail\n\r");
		goto mmf2_example_file_vipnn_tester_fail;
	}
	printf("siso_vipnn_filesaver started\n\r");

	return;
mmf2_example_file_vipnn_tester_fail:

	return;
}

/*-----------------------------------------------------------------------------------*/

static void set_nn_roi(int w, int h)
{
	roi_tester.img.width = w;
	roi_tester.img.height = h;
	roi_tester.img.roi.xmax = w;
	roi_tester.img.roi.ymax = h;
	mm_module_ctrl(vipnn_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&roi_tester);
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

static char *nn_get_json_format(void *p, int frame_id, char *file_name)
{

	vipnn_out_buf_t *out = (vipnn_out_buf_t *)p;

	objdetect_res_t *od_res = (objdetect_res_t *)&out->res[0];

	/**** cJSON ****/
	cJSON_Hooks memoryHook;
	memoryHook.malloc_fn = malloc;
	memoryHook.free_fn = free;
	cJSON_InitHooks(&memoryHook);

	cJSON *nnJSObject = NULL, *nn_obj_JSObject = NULL;
	cJSON *nn_coor_JSObject = NULL, *nn_obj_JSArray = NULL;
	cJSON *nnJSArray = NULL, *nn_coor_JSArray = NULL;
	char *nn_json_string = NULL;
#if SAVE_COCO_FORMAT
	nnJSArray = cJSON_CreateArray();
#else
	nnJSObject = cJSON_CreateObject();
	cJSON_AddItemToObject(nnJSObject, "frame_id", cJSON_CreateNumber(frame_id));
	cJSON_AddItemToObject(nnJSObject, "filename", cJSON_CreateString(file_name));
	cJSON_AddItemToObject(nnJSObject, "objects", nn_obj_JSArray = cJSON_CreateArray());
#endif

	int im_w = roi_tester.img.width;
	int im_h = roi_tester.img.height;

	printf("object num = %d\r\n", out->res_cnt);
	if (out->res_cnt > 0) {
		for (int i = 0; i < out->res_cnt; i++) {

			int top_x = (int)(od_res[i].result[2] * im_w) < 0 ? 0 : (int)(od_res[i].result[2] * im_w);
			int top_y = (int)(od_res[i].result[3] * im_h) < 0 ? 0 : (int)(od_res[i].result[3] * im_h);
			int bottom_x = (int)(od_res[i].result[4] * im_w) > im_w ? im_w : (int)(od_res[i].result[4] * im_w);
			int bottom_y = (int)(od_res[i].result[5] * im_h) > im_h ? im_h : (int)(od_res[i].result[5] * im_h);
			int class_id = (int)(od_res[i].result[0]);
			float probability = od_res[i].result[1];

			printf("%d,c%d,s%lf:(x0 y0 w h)%d %d %d %d\n\r", i, class_id, probability, top_x, top_y, bottom_x, bottom_y);
#if SAVE_COCO_FORMAT
			int w = bottom_x - top_x;
			int h = bottom_y - top_y;

			cJSON_AddItemToArray(nnJSArray, nn_obj_JSObject = cJSON_CreateObject());

			cJSON_AddItemToObject(nn_obj_JSObject, "image_id", cJSON_CreateNumber(frame_id));
			cJSON_AddItemToObject(nn_obj_JSObject, "category_id", cJSON_CreateNumber(class_id));
			cJSON_AddItemToObject(nn_obj_JSObject, "bbox", nn_coor_JSArray = cJSON_CreateArray());
			cJSON_AddItemToObject(nn_obj_JSObject, "score", cJSON_CreateNumber(probability));
			cJSON_AddItemToArray(nn_coor_JSArray, cJSON_CreateNumber(top_x));
			cJSON_AddItemToArray(nn_coor_JSArray, cJSON_CreateNumber(top_y));
			cJSON_AddItemToArray(nn_coor_JSArray, cJSON_CreateNumber(w));
			cJSON_AddItemToArray(nn_coor_JSArray, cJSON_CreateNumber(h));
#else
			cJSON_AddItemToArray(nn_obj_JSArray, nn_obj_JSObject = cJSON_CreateObject());
			cJSON_AddItemToObject(nn_obj_JSObject, "class_id", cJSON_CreateNumber(class_id));
			cJSON_AddItemToObject(nn_obj_JSObject, "name", cJSON_CreateString(NN_DATASET_LABEL[class_id]));
			cJSON_AddItemToObject(nn_obj_JSObject, "relative_coordinates", nn_coor_JSObject = cJSON_CreateObject());
			cJSON_AddItemToObject(nn_coor_JSObject, "top_x", cJSON_CreateNumber(top_x));
			cJSON_AddItemToObject(nn_coor_JSObject, "top_y", cJSON_CreateNumber(top_y));
			cJSON_AddItemToObject(nn_coor_JSObject, "bottom_x", cJSON_CreateNumber(bottom_x));
			cJSON_AddItemToObject(nn_coor_JSObject, "bottom_y", cJSON_CreateNumber(bottom_y));

			cJSON_AddItemToObject(nn_obj_JSObject, "probability", cJSON_CreateNumber(probability));
#endif
		}
	}
#if SAVE_COCO_FORMAT
	nn_json_string = cJSON_Print(nnJSArray);
	cJSON_Delete(nnJSArray);
#else
	nn_json_string = cJSON_Print(nnJSObject);
	cJSON_Delete(nnJSObject);
#endif
	return nn_json_string;
}

static int saver_count = 0;

#if SAVER_MODE==MODE_SD
static int (*media_save_file)(char *file_name, char *data_buf, int data_buf_size) = sd_save_file;
#elif SAVER_MODE==MODE_FTP
static int (*media_save_file)(char *file_name, char *data_buf, int data_buf_size) = ftp_save_file;
#endif

//char *str1 = "coco_val2017_pro2/image-0001.jpg";  --> return 1
//char *str2 = "coco_val2017_pro2/000000425131.jpg";  --> return 425131
static int get_id_in_filename(char *str)
{
	int pos_slash = strrchr(str, '/') - str;
	int pos_dash = strrchr(str, '-') - str;
	int start_pos = pos_dash > pos_slash ? pos_dash + 1 : pos_slash + 1;

	int pos_dot = strrchr(str, '.') - str;
	int len = pos_dot - start_pos;

	char image_id[32];
	memset(&image_id[0], 0x00, sizeof(image_id));
	strncpy(image_id, &str[start_pos], len);

	printf("image_id = %s\n", image_id);

	return (int)strtol(image_id, NULL, 10);
}

//char *str1 = "coco_val2017_pro2/000000425131.jpg";  --> return "coco_val2017_pro2/000000425131"
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

static void nn_save_handler_for_evaluate(char *file_name, uint32_t data_addr, uint32_t data_size)
{

	vipnn_out_buf_t *out = (vipnn_out_buf_t *)data_addr;
	vipnn_out_tensor_t *tensor_out = &out->tensors;

	//memcpy(&pre_tensor_out, (vipnn_out_buf_t *)data_addr, data_size);

	char nn_fn[128];
	memset(&nn_fn[0], 0x00, sizeof(nn_fn));

	int image_id = get_id_in_filename(file_name);

	/* save yolo json result */
	snprintf(nn_fn, sizeof(nn_fn), "%s.json", strip_filename_extention(file_name));
	char *json_format_out = nn_get_json_format((void *)out, image_id, nn_fn);
	//printf("\r\njson_format_out: %s\r\n", json_format_out);
	media_save_file(nn_fn, json_format_out, strlen(json_format_out));

	/* save tensor */
#if 0
	for (int i = 0; i < tensor_out->vipnn_out_tensor_num; i++) {

		/* save raw tensor */
		memset(&nn_fn[0], 0x00, sizeof(nn_fn));
		snprintf(nn_fn, sizeof(nn_fn), "%s/nn_result/nn_out_tensor%d_uint8_%d.bin", folder_name, i, image_id);
		media_save_file(nn_fn, (char *)tensor_out->vipnn_out_tensor[i], tensor_out->vipnn_out_tensor_size[i]); /* raw tensor*/

		/* save float32 tensor */
		memset(&nn_fn[0], 0x00, sizeof(nn_fn));
		snprintf(nn_fn, sizeof(nn_fn), "%s/nn_result/nn_out_tensor%d_float32_%d.bin", folder_name, i, image_id);
		float *float_tensor;
		switch (tensor_out->quant_format[i]) {
		case VIP_BUFFER_QUANTIZE_TF_ASYMM:   /* uint8 --> float32 */
			float_tensor = (float *)malloc((int)(tensor_out->vipnn_out_tensor_size[i] * sizeof(float)));
			for (int k = 0; k < tensor_out->vipnn_out_tensor_size[i]; k++) {
				float_tensor[k] = (*((uint8_t *)tensor_out->vipnn_out_tensor[i] + k) - tensor_out->quant_data[i].affine.zeroPoint) *
								  tensor_out->quant_data[i].affine.scale;
			}
			media_save_file(nn_fn, (char *)float_tensor, tensor_out->vipnn_out_tensor_size[i] * sizeof(float));
			break;
		case VIP_BUFFER_QUANTIZE_DYNAMIC_FIXED_POINT:   /* int16 --> float32 */
			float_tensor = (float *)malloc((int)(tensor_out->vipnn_out_tensor_size[i] * sizeof(float) / sizeof(int16_t)));
			for (int k = 0; k < (tensor_out->vipnn_out_tensor_size[i] / sizeof(int16_t)); k++) {
				float_tensor[k] = (float)(*((int16_t *)tensor_out->vipnn_out_tensor[i] + k)) / ((float)(1 << tensor_out->quant_data[i].dfp.fixed_point_pos));
			}
			media_save_file(nn_fn, (char *)float_tensor, tensor_out->vipnn_out_tensor_size[i] * sizeof(float) / sizeof(int16_t));
			break;
		default:   /* float16 --> float32 */
			float_tensor = (float *)malloc(tensor_out->vipnn_out_tensor_size[i] * sizeof(float) / sizeof(__fp16));
			for (int k = 0; k < (tensor_out->vipnn_out_tensor_size[i] / sizeof(__fp16)); k++) {
				float_tensor[k] = (float)(*((__fp16 *)tensor_out->vipnn_out_tensor[i] + k));
			}
			media_save_file(nn_fn, (char *)float_tensor, tensor_out->vipnn_out_tensor_size[i] * sizeof(float) / sizeof(__fp16));
		}
		free(float_tensor);
	}
#endif

	saver_count++;
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

/*-----------------------------------------------------------------------------------*/

static FtpClient *g_client = NULL;
static NetBuf_t *ftpClientNetBuf = NULL;
static int ftp_save_file(char *filename, char *data_buf, int data_buf_size)
{

	if (g_client == NULL) {
		printf("init receiver ftp\n\r");
		g_client = getFtpClient();


		printf("connecting to remote %s port %d\n\r", ftp_info.remote_ip, ftp_info.remote_port);
		int connect = g_client->ftpClientConnect(ftp_info.remote_ip, ftp_info.remote_port, &ftpClientNetBuf);

		if (connect == 0) {
			printf("FTP server connect fail");
			goto ftp_save_err;
		}

		printf("user %s pass %s\n\r", ftp_info.remote_user, ftp_info.remote_pass);
		int login = g_client->ftpClientLogin(ftp_info.remote_user, ftp_info.remote_pass, ftpClientNetBuf);
		if (login == 0) {
			printf("FTP server login fail");
			goto ftp_save_err;
		}

		printf("change remote dir %s\n\r", ftp_info.remote_dir);
		int chdir = g_client->ftpClientChangeDir(ftp_info.remote_dir, ftpClientNetBuf);
		if (chdir == 0) {
			printf("FTP server change dir fail");
			goto ftp_save_err;
		}
	}

	printf("upload file : %s, buf %p, len %d\n\r", filename, data_buf, data_buf_size);

	int ret = g_client->ftpClientPutBuf(data_buf, &data_buf_size, filename, FTP_CLIENT_BINARY, ftpClientNetBuf);

	if (ret == 0) {
		printf("FTP server upload fail");
		goto ftp_save_err;
	}

	printf("upload complete\n\r");
	//g_client->ftpClientClose(ftpClientNetBuf);
	return 0;
ftp_save_err:
	//g_client->ftpClientClose(ftpClientNetBuf);
	return -1;
}

static void fFTP(void *arg)
{
	printf("enter remote ip = %s\n\r", (char *)arg);
	strncpy(remote_host, (char *)arg, strlen(arg));

	mm_module_ctrl(fileloader_ctx, CMD_FILELOADER_SET_REMOTE_IP, (int)remote_host);

	strcpy(ftp_info.remote_ip, remote_host);

	set_remote_host = 1;
}

static log_item_t userctrl_items[] = {
	{"FTP", fFTP, },
};

static void atcmd_userctrl_init(void)
{
	log_service_add_table(userctrl_items, sizeof(userctrl_items) / sizeof(userctrl_items[0]));
}

/*-----------------------------------------------------------------------------------*/
