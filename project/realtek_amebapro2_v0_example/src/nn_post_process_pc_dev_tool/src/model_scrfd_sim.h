#ifndef MODEL_SCRFD_SIM_H
#define MODEL_SCRFD_SIM_H

#include "module_vipnn.h"

void *scrfd_get_network_filename(void);
int scrfd_preprocess(void *data_in, nn_data_param_t *data_param, void *tensor_in, nn_tensor_param_t *tensor_param);
int scrfd_postprocess(void *tensor_out, nn_tensor_param_t *param, void *res);

#endif /* MODEL_SCRFD_SIM_H */