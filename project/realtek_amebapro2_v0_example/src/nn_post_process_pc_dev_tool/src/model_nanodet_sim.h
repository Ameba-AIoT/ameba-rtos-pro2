#ifndef MODEL_NANODET_SIM_H
#define MODEL_NANODET_SIM_H

void *nanodet_plus_m_get_network_filename(void);

int nanodet_preprocess(void *data_in, nn_data_param_t *data_param, void *tensor_in, nn_tensor_param_t *tensor_param);
int nanodet_postprocess(void *tensor_out, nn_tensor_param_t *param, void *res);
void nanodet_release(void);

#endif