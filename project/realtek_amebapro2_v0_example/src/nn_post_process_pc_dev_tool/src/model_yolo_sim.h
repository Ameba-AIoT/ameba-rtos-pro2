#ifndef MODEL_YOLO_SIM_H
#define MODEL_YOLO_SIM_H

void *yolov3_get_network_filename_init(void);
void *yolov4_get_network_filename_init(void);
void *yolov7_get_network_filename_init(void);

int yolo_preprocess(void *data_in, nn_data_param_t *data_param, void *tensor_in, nn_tensor_param_t *tensor_param);
int yolo_postprocess(void *tensor_out, nn_tensor_param_t *param, void *res);

#endif