#ifndef MODEL_YOLOV9_SIM_H
#define MODEL_YOLOV9_SIM_H

void *yolov9_get_network_filename_init(void);

int yolov9_preprocess(void *data_in, nn_data_param_t *data_param, void *tensor_in, nn_tensor_param_t *tensor_param);
int yolov9_postprocess(void *tensor_out, nn_tensor_param_t *param, void *res);

#endif