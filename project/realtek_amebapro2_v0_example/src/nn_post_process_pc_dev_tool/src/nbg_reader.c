#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "module_vipnn.h"
#include "nbg_reader.h"

#include "NBGParser/nbg_parser.h"
#include "NBGParser/gc_vip_nbg_format.h"

#define ENUM_NAME_SIMPLE_MAP(_ENUM) case _ENUM: return #_ENUM

void print_line_of_stars(int length)
{
	int star;
	for (star = 0; star < length; star++) {
		printf("*");
	}
	printf("\n");
}

char *get_data_format_name(nbg_uint32_t data_format)
{
	switch (data_format) {
		ENUM_NAME_SIMPLE_MAP(NBG_BUFFER_FORMAT_FP32);
		ENUM_NAME_SIMPLE_MAP(NBG_BUFFER_FORMAT_FP16);
		ENUM_NAME_SIMPLE_MAP(NBG_BUFFER_FORMAT_UINT8);
		ENUM_NAME_SIMPLE_MAP(NBG_BUFFER_FORMAT_INT8);
		ENUM_NAME_SIMPLE_MAP(NBG_BUFFER_FORMAT_UINT16);
		ENUM_NAME_SIMPLE_MAP(NBG_BUFFER_FORMAT_INT16);
		ENUM_NAME_SIMPLE_MAP(NBG_BUFFER_FORMAT_CHAR);
		ENUM_NAME_SIMPLE_MAP(NBG_BUFFER_FORMAT_BFP16);
		ENUM_NAME_SIMPLE_MAP(NBG_BUFFER_FORMAT_INT32);
		ENUM_NAME_SIMPLE_MAP(NBG_BUFFER_FORMAT_UINT32);
		ENUM_NAME_SIMPLE_MAP(NBG_BUFFER_FORMAT_INT64);
		ENUM_NAME_SIMPLE_MAP(NBG_BUFFER_FORMAT_UINT64);
		ENUM_NAME_SIMPLE_MAP(NBG_BUFFER_FORMAT_FP64);
	default:
		return "UNKNOWN_NBG_BUFFER_FORMAT";
	}
}

char *get_data_type_name(nbg_uint32_t data_type)
{
	switch (data_type) {
		ENUM_NAME_SIMPLE_MAP(NBG_BUFFER_TYPE_TENSOR);
		ENUM_NAME_SIMPLE_MAP(NBG_BUFFER_TYPE_IMAGE);
		ENUM_NAME_SIMPLE_MAP(NBG_BUFFER_TYPE_ARRAY);
		ENUM_NAME_SIMPLE_MAP(NBG_BUFFER_TYPE_SCALAR);
	default:
		return "UNKNOWN_NBG_BUFFER_TYPE";
	}
}

char *get_quantize_format_name(nbg_uint32_t quantization_format)
{
	switch (quantization_format) {
		ENUM_NAME_SIMPLE_MAP(NBG_BUFFER_QUANTIZE_NONE);
		ENUM_NAME_SIMPLE_MAP(NBG_BUFFER_QUANTIZE_DYNAMIC_FIXED_POINT);
		ENUM_NAME_SIMPLE_MAP(NBG_BUFFER_QUANTIZE_AFFINE_ASYMMETRIC);
	default:
		return "UNKNOWN_FORMAT";
	}
}

nbg_uint32_t get_file_size(const char *name)
{
	FILE *fp = fopen(name, "rb");
	nbg_uint32_t size = 0;

	if (fp != NULL) {
		fseek(fp, 0, SEEK_END);
		size = ftell(fp);

		fclose(fp);
	} else {
		printf("Checking file %s failed.\n", name);
	}

	return size;
}

nbg_uint32_t read_file(const char *name, void *dst)
{
	FILE *fp = fopen(name, "rb");
	nbg_uint32_t size = 0;
	int ret = 0;

	if (fp != NULL) {
		fseek(fp, 0, SEEK_END);
		size = ftell(fp);

		fseek(fp, 0, SEEK_SET);
		ret = fread(dst, size, 1, fp);

		fclose(fp);
	}

	return size;
}

int get_binary_data(char *file_name, nbg_uint32_t *file_size, void **buffer)
{
	uint8_t *data = NULL;

	*file_size = get_file_size((const char *)file_name);
	if (*file_size == 0) {
		printf("failed to get binary data, the size of file is 0\n");
		return -1;
	}

	data = (uint8_t *)malloc(*file_size * sizeof(uint8_t));

	read_file(file_name, (void *)data);

	if (buffer != NBG_NULL) {
		*buffer = (void *)data;
	} else {
		printf("failed to get binary data, buffer is NULL\n");
		return -1;
	}

	return 0;
}

void config_param_from_nb_file(char *nb_file_name, nn_tensor_param_t *input_param, nn_tensor_param_t *output_param)
{
	uint8_t *buffer = NULL;
	nbg_parser_data nbg = NULL;
	nbg_status_e status = NBG_SUCCESS;
	uint32_t nbg_size = 0;
	int ret = 0;

	ret = get_binary_data(nb_file_name, &nbg_size, (void **)&buffer);
	if (ret < 0) {
		printf("failed to read NBG file %s\n", nb_file_name);
		return;
	}

	status = nbg_parser_init(buffer, nbg_size, &nbg);
	if (status != NBG_SUCCESS) {
		printf("failed to initialize nbg parser\n");
		return;
	}

	if (buffer != NULL) {
		free(buffer);
		buffer = NULL;
	}

	// input
	printf("\n");
	print_line_of_stars(80);
	printf("Input Table\n");
	print_line_of_stars(80);
	nbg_uint32_t input_count = 0;
	nbg_parser_query_network(nbg, NBG_PARSER_NETWORK_INPUT_COUNT, &input_count, sizeof(nbg_uint32_t));
	nbg_uint32_t dim_count;
	input_param->count = dim_count;  //
	nbg_uint32_t dim_size[MAX_NUM_DIMS];
	for (int i = 0; i < input_count; i++) {
		nbg_parser_query_input(nbg, i, NBG_PARSER_BUFFER_PROP_NUM_OF_DIMENSION,
							   &dim_count, sizeof(nbg_uint32_t));
		nbg_parser_query_input(nbg, i, NBG_PARSER_BUFFER_PROP_DIMENSIONS, dim_size,
							   sizeof(nbg_uint32_t) * dim_count);
		printf("Input %d\n", i);
		printf("%-40s%40u\n", "Dim Count:", dim_count);
		for (int j = 0; j < dim_count; j++) {
			nbg_char_t buffer[100];
			sprintf(buffer, "Size of Dim[%d]:", j);
			printf("%-40s%40u\n", buffer, dim_size[j]);
			input_param->dim[i].size[j] = dim_size[j];  //
		}
	}
	print_line_of_stars(80);

	// output
	printf("\n");
	print_line_of_stars(80);
	printf("Output Table\n");
	print_line_of_stars(80);
	nbg_uint32_t output_count = 0;
	nbg_uint32_t data_format;
	nbg_uint32_t quan_format;
	nbg_uint32_t fixed_pos;
	nbg_float_t tf_scale;
	nbg_uint32_t tf_zerop;
	nbg_parser_query_network(nbg, NBG_PARSER_NETWORK_OUTPUT_COUNT, &output_count, sizeof(nbg_uint32_t));
	output_param->count = output_count; //
	for (int i = 0; i < output_count; i++) {
		nbg_parser_query_output(nbg, i, NBG_PARSER_BUFFER_PROP_QUANT_FORMAT, &quan_format,
								sizeof(nbg_uint32_t));
		nbg_parser_query_output(nbg, i, NBG_PARSER_BUFFER_PROP_NUM_OF_DIMENSION, &dim_count,
								sizeof(nbg_uint32_t));
		nbg_parser_query_output(nbg, i, NBG_PARSER_BUFFER_PROP_DIMENSIONS, dim_size,
								sizeof(nbg_uint32_t) * dim_count);
		nbg_parser_query_output(nbg, i, NBG_PARSER_BUFFER_PROP_DATA_FORMAT, &data_format,
								sizeof(nbg_uint32_t));
		nbg_parser_query_output(nbg, i, NBG_PARSER_BUFFER_PROP_FIXED_POINT_POS, &fixed_pos,
								sizeof(nbg_uint32_t));
		nbg_parser_query_output(nbg, i, NBG_PARSER_BUFFER_PROP_SCALE, &tf_scale,
								sizeof(nbg_float_t));
		nbg_parser_query_output(nbg, i, NBG_PARSER_BUFFER_PROP_ZERO_POINT, &tf_zerop,
								sizeof(nbg_uint32_t));

		output_param->dim[i].num = dim_count;  //
		printf("Output %d\n", i);
		printf("%-40s%40u\n", "Dim Count:", dim_count);
		for (int j = 0; j < dim_count; j++) {
			nbg_char_t buffer[100];
			sprintf(buffer, "Size of Dim[%d]:", j);
			printf("%-40s%40u\n", buffer, dim_size[j]);
			output_param->dim[i].size[j] = dim_size[j];  //
		}
		output_param->format[i].buf_type = data_format;
		output_param->format[i].type = quan_format;
		output_param->format[i].scale = tf_scale;
		output_param->format[i].zero_point = tf_zerop;
		output_param->format[i].fix_point_pos = fixed_pos;
	}
	print_line_of_stars(80);

	if (nbg != NBG_NULL) {
		nbg_parser_destroy(nbg);
		nbg = NBG_NULL;
	}
}
