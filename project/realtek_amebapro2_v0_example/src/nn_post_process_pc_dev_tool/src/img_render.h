#ifndef __IMG_RENDER_H__
#define __IMG_RENDER_H__

typedef enum {
	OBJDECT_TYPE,
	FACEDECT_TYPE
} nn_res_t;

void image_rendering(void *post_res, char *imge_name_in, char *imge_name_out, nn_res_t type);

#endif /* __IMG_RENDER_H__ */