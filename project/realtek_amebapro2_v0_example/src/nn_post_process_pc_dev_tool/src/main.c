#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "nn_utils/sigmoid.h"
#include "nn_utils/iou.h"
#include "nn_utils/nms.h"
#include "nn_utils/quantize.h"
#include "nn_utils/tensor.h"

#include "model_yolo_sim.h"
#include "model_yolov9_sim.h"
#include "model_nanodet_sim.h"
#include "model_scrfd_sim.h"
#include "img_render.h"
#include "nbg_reader.h"

#define CONFIG_PARAM_FROM_NB_FILE   1  /* it is recommended to configure the model parameter from network binary file */
#define NON_QUANT_MODE              1

static void yolo_pc_configure_tensor_param(nn_tensor_param_t *input_param, nn_tensor_param_t *output_param)
{
#if CONFIG_PARAM_FROM_NB_FILE

	/* Configure the model parameter from nb file */
	char *nbg_filename = "../../test_model/model_nb/yolov4_tiny.nb";
	config_param_from_nb_file(nbg_filename, input_param, output_param);

#else  /* CONFIG_PARAM_FROM_NB_FILE */

	/* Configure the model parameter manually */
	// input
	input_param->count = 1;
	input_param->dim[0].size[0] = 416;
	input_param->dim[0].size[1] = 416;

	// output
	output_param->count = 2;
	output_param->dim[0].num = 4;
	output_param->dim[0].size[0] = 13;
	output_param->dim[0].size[1] = 13;
	output_param->dim[0].size[2] = 255;
	output_param->dim[0].size[3] = 1;
#if NON_QUANT_MODE
	output_param->format[0].buf_type = VIP_BUFFER_FORMAT_FP32;
	output_param->format[0].type = VIP_BUFFER_QUANTIZE_NONE;
#else
	output_param->format[0].buf_type = VIP_BUFFER_FORMAT_UINT8;
	output_param->format[0].type = VIP_BUFFER_QUANTIZE_TF_ASYMM;
	output_param->format[0].scale = 0.137945;
	output_param->format[0].zero_point = 178;
#endif

	output_param->dim[1].num = 4;
	output_param->dim[1].size[0] = 26;
	output_param->dim[1].size[1] = 26;
	output_param->dim[1].size[2] = 255;
	output_param->dim[1].size[3] = 1;
#if NON_QUANT_MODE
	output_param->format[1].buf_type = VIP_BUFFER_FORMAT_FP32;
	output_param->format[1].type = VIP_BUFFER_QUANTIZE_NONE;
#else
	output_param->format[1].buf_type = VIP_BUFFER_FORMAT_UINT8;
	output_param->format[1].type = VIP_BUFFER_QUANTIZE_TF_ASYMM;
	output_param->format[1].scale = 0.131652;
	output_param->format[1].zero_point = 193;
#endif

#endif  /* CONFIG_PARAM_FROM_NB_FILE */
}

static void yolov9_pc_configure_tensor_param(nn_tensor_param_t *input_param, nn_tensor_param_t *output_param)
{
#if CONFIG_PARAM_FROM_NB_FILE

	/* Configure the model parameter from nb file */
	char *nbg_filename = "../../test_model/model_nb/yolov9_tiny_hybrid.nb";
	config_param_from_nb_file(nbg_filename, input_param, output_param);

#else  /* CONFIG_PARAM_FROM_NB_FILE */

	/* Configure the model parameter manually */
	// input
	input_param->count = 1;
	input_param->dim[0].size[0] = 416;
	input_param->dim[0].size[1] = 416;

	output_param->count = 2;
	output_param->dim[0].num = 3;
	output_param->dim[0].size[0] = 3549;
	output_param->dim[0].size[1] = 4;
	output_param->dim[0].size[2] = 1;
#if NON_QUANT_MODE
	output_param->format[0].buf_type = VIP_BUFFER_FORMAT_FP32;
	output_param->format[0].type = VIP_BUFFER_QUANTIZE_NONE;
#else
	output_param->format[0].buf_type = VIP_BUFFER_FORMAT_UINT8;
	output_param->format[0].type = VIP_BUFFER_QUANTIZE_TF_ASYMM;
	output_param->format[0].scale = 1.624492;
	output_param->format[0].zero_point = 0;
#endif

	output_param->dim[1].num = 3;
	output_param->dim[1].size[0] = 3549;
	output_param->dim[1].size[1] = 80;
	output_param->dim[1].size[2] = 1;
#if NON_QUANT_MODE
	output_param->format[1].buf_type = VIP_BUFFER_FORMAT_FP32;
	output_param->format[1].type = VIP_BUFFER_QUANTIZE_NONE;
#else
	output_param->format[1].buf_type = VIP_BUFFER_FORMAT_INT16;
	output_param->format[1].type = VIP_BUFFER_QUANTIZE_DYNAMIC_FIXED_POINT;
	output_param->format[1].fix_point_pos = 15;
#endif
	
#endif  /* CONFIG_PARAM_FROM_NB_FILE */
}

static void nanodet_pc_configure_tensor_param(nn_tensor_param_t *input_param, nn_tensor_param_t *output_param)
{
	char *nbg_filename = "../../test_model/model_nb/nanodet_plus_m_416_uint8.nb";
	config_param_from_nb_file(nbg_filename, input_param, output_param);
}

static void scrfd_pc_configure_tensor_param(nn_tensor_param_t *input_param, nn_tensor_param_t *output_param)
{
	char *nbg_filename = "../../test_model/model_nb/scrfd_500m_bnkps_576x320_u8.nb";
	config_param_from_nb_file(nbg_filename, input_param, output_param);
}

static void acuity_output_tensor_conversion(char **pp_file_name, void **pp_tensor, nn_tensor_param_t *output_param)
{
	int tmp_cnt = 0;
	char line[256];
	memset(line, 0, sizeof(line));
	int tensor_len = 1;
	FILE *fp = NULL;
	uint8_t *tmp_u8 = NULL;
	int8_t *tmp_s8 = NULL;
	int16_t *tmp_s16 = NULL;
	__fp16 *tmp_f16 = NULL;
	float *tmp_f32 = NULL;

	char **tensor_name = pp_file_name;
	for (int i = 0; i < output_param->count; i++) {
		tensor_len = 1;
		for (int k = 0; k < output_param->dim[i].num; k++) {
			tensor_len *= output_param->dim[i].size[k];
		}
		pp_tensor[i] = malloc(tensor_len * get_element_size(&output_param->format[i]));
		if (!pp_tensor[i]) {
			printf("fail to allocate tensor\r\n");
			return;
		}
		fp = fopen(tensor_name[i], "r+");
		if (fp == NULL) {
			printf("fail to open the file\r\n");
			return;
		}

		if (output_param->format[i].buf_type == VIP_BUFFER_FORMAT_UINT8) {
			tmp_u8 = (uint8_t *)pp_tensor[i];
		} else if (output_param->format[i].buf_type == VIP_BUFFER_FORMAT_INT8) {
			tmp_s8 = (int8_t *)pp_tensor[i];
		} else if (output_param->format[i].buf_type == VIP_BUFFER_FORMAT_INT16) {
			tmp_s16 = (int16_t *)pp_tensor[i];
		} else if (output_param->format[i].buf_type == VIP_BUFFER_FORMAT_FP16 || output_param->format[i].buf_type == VIP_BUFFER_FORMAT_BFP16) {
			tmp_f16 = (__fp16 *)pp_tensor[i];
		} else {
			tmp_f32 = (float *)pp_tensor[i];
		}

		tmp_cnt = 0;
		while (fgets(line, sizeof(line), fp)) {
			if (output_param->format[i].buf_type == VIP_BUFFER_FORMAT_UINT8) {
				tmp_u8[tmp_cnt] = f_to_u8(atof(line), output_param->format[i].zero_point, output_param->format[i].scale);
			} else if (output_param->format[i].buf_type == VIP_BUFFER_FORMAT_INT8) {
				tmp_s8[tmp_cnt] = f_to_s8(atof(line), output_param->format[i].fix_point_pos);
			} else if (output_param->format[i].buf_type == VIP_BUFFER_FORMAT_INT16) {
				tmp_s16[tmp_cnt] = f_to_s16(atof(line), output_param->format[i].fix_point_pos);
			} else if (output_param->format[i].buf_type == VIP_BUFFER_FORMAT_FP16 || output_param->format[i].buf_type == VIP_BUFFER_FORMAT_BFP16) {
				tmp_f16[tmp_cnt] = f_to_bf16(atof(line));
			} else {
				tmp_f32[tmp_cnt] = atof(line);
			}
			memset(line, 0, sizeof(line));
			tmp_cnt++;
			//printf("%f %d\r\n", atof(line), tmp_cnt);
		}
		fclose(fp);
		printf("tensor%d length = %d\r\n", i, tmp_cnt);
	}
}

int yolo_simulation(void)
{
	// configure tensor param
	nn_tensor_param_t input_param, output_param;
	yolo_pc_configure_tensor_param(&input_param, &output_param); // >>> user need to configure this function!!!

	// set anchor box and model input size
	yolov4_get_network_filename_init();
	yolo_preprocess(0, 0, 0, &input_param);

	// prepare Acuity pre-generated output tensor from file
	char *acuity_tensor_name[16]; // >>> user need to configure acuity tensor path!!!
	acuity_tensor_name[0] = "../data/yolo_data/iter_0_output_30_65_out0_1_255_13_13.tensor";
	acuity_tensor_name[1] = "../data/yolo_data/iter_0_output_37_76_out0_1_255_26_26.tensor";
	void *pp_tensor_out[16];
	memset(pp_tensor_out, 0, sizeof(pp_tensor_out));
	acuity_output_tensor_conversion(acuity_tensor_name, pp_tensor_out, &output_param);

	// prepare result buffer for decode
	vipnn_out_buf_t *res_buf = (vipnn_out_buf_t *)malloc(sizeof(vipnn_out_buf_t) + sizeof(objdetect_res_t) * MAX_DETECT_OBJ_NUM);
	if (!res_buf) {
		printf("fail to allocate memory \r\n");
		return -1;
	}
	res_buf->res_size = sizeof(objdetect_res_t);
	res_buf->res_max_cnt = MAX_DETECT_OBJ_NUM;
	vipnn_out_buf_t *out = (vipnn_out_buf_t *)res_buf;
	out->res_cnt = 0;

	// post-process
	int post_res_cnt = yolo_postprocess((void *)pp_tensor_out, &output_param, (void *)((uint8_t *)&out->res[0] + out->res_cnt * res_buf->res_size));
	out->res_cnt += post_res_cnt;

	// rendering
	image_rendering(out, "../data/yolo_data/horses_416x416.jpg", "../data/yolo_data/prediction.jpg", OBJDECT_TYPE);

	// release
	for (int i = 0; i < (sizeof(pp_tensor_out) / sizeof(void *)); i++) {
		if (pp_tensor_out[i]) {
			free(pp_tensor_out[i]);
		}
	}
	free(res_buf);
}

int yolov9_simulation(void)
{
	// configure tensor param
	nn_tensor_param_t input_param, output_param;
	yolov9_pc_configure_tensor_param(&input_param, &output_param); // >>> user need to configure this function!!!

	// set anchor box and model input size
	yolov9_get_network_filename_init();
	yolov9_preprocess(0, 0, 0, &input_param);
	// prepare Acuity pre-generated output tensor from file
	char *acuity_tensor_name[16]; // >>> user need to configure acuity tensor path!!!
	acuity_tensor_name[0] = "../data/yolov9_data/iter_0_attach_Mul__model.22_Mul_out0_0_out0_1_4_3549.tensor";
	acuity_tensor_name[1] = "../data/yolov9_data/iter_0_attach_Sigmoid__model.22_Sigmoid_out0_1_out0_1_80_3549.tensor";
	void *pp_tensor_out[16];
	memset(pp_tensor_out, 0, sizeof(pp_tensor_out));
	acuity_output_tensor_conversion(acuity_tensor_name, pp_tensor_out, &output_param);
	// prepare result buffer for decode
	vipnn_out_buf_t *res_buf = (vipnn_out_buf_t *)malloc(sizeof(vipnn_out_buf_t) + sizeof(objdetect_res_t) * MAX_DETECT_OBJ_NUM);
	if (!res_buf) {
		printf("fail to allocate memory \r\n");
		return -1;
	}
	res_buf->res_size = sizeof(objdetect_res_t);
	res_buf->res_max_cnt = MAX_DETECT_OBJ_NUM;
	vipnn_out_buf_t *out = (vipnn_out_buf_t *)res_buf;
	out->res_cnt = 0;

	// post-process
	int post_res_cnt = yolov9_postprocess((void *)pp_tensor_out, &output_param, (void *)((uint8_t *)&out->res[0] + out->res_cnt * res_buf->res_size));
	out->res_cnt += post_res_cnt;
	printf("out_cnt: %d\n", out->res_cnt);
	// rendering
	image_rendering(out, "../data/yolov9_data/horses_416x416.jpg", "../data/yolov9_data/prediction.jpg", OBJDECT_TYPE);

	// release
	for (int i = 0; i < (sizeof(pp_tensor_out) / sizeof(void *)); i++) {
		if (pp_tensor_out[i]) {
			free(pp_tensor_out[i]);
		}
	}
	free(res_buf);
}

int nanodet_simulation(void)
{
	// configure tensor param
	nn_tensor_param_t input_param, output_param;
	nanodet_pc_configure_tensor_param(&input_param, &output_param); // >>> user need to configure this function!!!

	// set anchor box and model input size
	nanodet_plus_m_get_network_filename();
	nanodet_preprocess(0, 0, 0, &input_param);

	// prepare Acuity pre-generated output tensor from file
	char *acuity_tensor_name[16]; // >>> user need to configure acuity tensor path!!!
	acuity_tensor_name[0] = "../data/nanodet_data/iter_0_attach_Transpose_Transpose_526_out0_0_out0_1_3598_112.tensor";
	void *pp_tensor_out[16];
	memset(pp_tensor_out, 0, sizeof(pp_tensor_out));
	acuity_output_tensor_conversion(acuity_tensor_name, pp_tensor_out, &output_param);

	// prepare result buffer for decode
	vipnn_out_buf_t *res_buf = (vipnn_out_buf_t *)malloc(sizeof(vipnn_out_buf_t) + sizeof(objdetect_res_t) * MAX_DETECT_OBJ_NUM);
	if (!res_buf) {
		printf("fail to allocate memory \r\n");
		return -1;
	}
	res_buf->res_size = sizeof(objdetect_res_t);
	res_buf->res_max_cnt = MAX_DETECT_OBJ_NUM;
	vipnn_out_buf_t *out = (vipnn_out_buf_t *)res_buf;
	out->res_cnt = 0;

	// post-process
	int post_res_cnt = nanodet_postprocess((void *)pp_tensor_out, &output_param, (void *)((uint8_t *)&out->res[0] + out->res_cnt * res_buf->res_size));
	out->res_cnt += post_res_cnt;

	// rendering
	image_rendering(out, "../data/nanodet_data/horses_416x416.jpg", "../data/nanodet_data/prediction.jpg", OBJDECT_TYPE);

	// release
	for (int i = 0; i < (sizeof(pp_tensor_out) / sizeof(void *)); i++) {
		if (pp_tensor_out[i]) {
			free(pp_tensor_out[i]);
		}
	}
	free(res_buf);
	nanodet_release();
}

int scrfd_simulation(void)
{
	// configure tensor param
	nn_tensor_param_t input_param, output_param;
	scrfd_pc_configure_tensor_param(&input_param, &output_param); // >>> user need to configure this function!!!

	// set anchor box and model input size
	scrfd_get_network_filename();
	scrfd_preprocess(0, 0, 0, &input_param);

	// prepare Acuity pre-generated output tensor from file
	char *acuity_tensor_name[16]; // >>> user need to configure acuity tensor path!!!
	acuity_tensor_name[0] = "../data/scrfd_data/iter_0_attach_Sigmoid_Sigmoid_128_out0_0_out0_1_5760_1.tensor";
	acuity_tensor_name[1] = "../data/scrfd_data/iter_0_attach_Sigmoid_Sigmoid_155_out0_1_out0_1_1440_1.tensor";
	acuity_tensor_name[2] = "../data/scrfd_data/iter_0_attach_Sigmoid_Sigmoid_182_out0_2_out0_1_360_1.tensor";
	acuity_tensor_name[3] = "../data/scrfd_data/iter_0_attach_Reshape_Reshape_132_out0_3_out0_1_5760_4.tensor";
	acuity_tensor_name[4] = "../data/scrfd_data/iter_0_attach_Reshape_Reshape_159_out0_4_out0_1_1440_4.tensor";
	acuity_tensor_name[5] = "../data/scrfd_data/iter_0_attach_Reshape_Reshape_186_out0_5_out0_1_360_4.tensor";
	acuity_tensor_name[6] = "../data/scrfd_data/iter_0_attach_Reshape_Reshape_136_out0_6_out0_1_5760_10.tensor";
	acuity_tensor_name[7] = "../data/scrfd_data/iter_0_attach_Reshape_Reshape_163_out0_7_out0_1_1440_10.tensor";
	acuity_tensor_name[8] = "../data/scrfd_data/iter_0_attach_Reshape_Reshape_190_out0_8_out0_1_360_10.tensor";
	void *pp_tensor_out[16];
	memset(pp_tensor_out, 0, sizeof(pp_tensor_out));
	acuity_output_tensor_conversion(acuity_tensor_name, pp_tensor_out, &output_param);

	// prepare result buffer for decode
	vipnn_out_buf_t *res_buf = (vipnn_out_buf_t *)malloc(sizeof(vipnn_out_buf_t) + sizeof(facedetect_res_t) * MAX_DETECT_OBJ_NUM);
	if (!res_buf) {
		printf("fail to allocate memory \r\n");
		return -1;
	}
	res_buf->res_size = sizeof(facedetect_res_t);
	res_buf->res_max_cnt = MAX_DETECT_OBJ_NUM;
	vipnn_out_buf_t *out = (vipnn_out_buf_t *)res_buf;
	out->res_cnt = 0;

	// post-process
	int post_res_cnt = scrfd_postprocess((void *)pp_tensor_out, &output_param, (void *)((uint8_t *)&out->res[0] + out->res_cnt * res_buf->res_size));
	out->res_cnt += post_res_cnt;

	// rendering
	image_rendering(out, "../data/scrfd_data/wls.jpg", "../data/scrfd_data/prediction.jpg", FACEDECT_TYPE);

	// release
	for (int i = 0; i < (sizeof(pp_tensor_out) / sizeof(void *)); i++) {
		if (pp_tensor_out[i]) {
			free(pp_tensor_out[i]);
		}
	}
	free(res_buf);
}

int main()
{
	yolo_simulation();
	yolov9_simulation();
	nanodet_simulation();
	scrfd_simulation();

	return 0;
}

