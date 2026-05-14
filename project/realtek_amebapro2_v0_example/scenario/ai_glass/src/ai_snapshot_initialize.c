#include "platform_opts.h"
#include "module_video.h"
#include "video_api.h"
#include "fwfs.h"
#include "vfs.h"
#include "video_snapshot.h"
#include "sensor.h"
#include "ai_glass_media.h"
#include "media_filesystem.h"
#include "ai_glass_dbg.h"

// VIPNN and object detection includes
#include "../../../../../component/media/mmfv2/module_vipnn.h"
#include "../../../../../component/media/mmfv2/mmf2_link.h"
#include "../../../../../component/media/mmfv2/mmf2_siso.h"
#include "../../../../src/test_model/model_yolo.h"
#include "../../../../../component/media/rtp_codec/avcodec.h"
#include "../../../../src/test_model/model_yolo.c"

// STB image for JPEG decoding
#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_STDIO
#define STBI_ONLY_JPEG
#include "../../../../../component/image/3rdparty/stb/stb_image.h"
#define JPG_WRITE_SIZE          4096

// Configure
#define MAXIMUM_FILE_TAG_SIZE   32
#define MAXIMUM_FILE_SIZE       (MAXIMUM_FILE_TAG_SIZE + 32)
#define DROP_FRAME              2

// YOLOv4 Object Detection Configuration
#define NN_MODEL_OBJ        yolov4_tiny
#define TEST_IMAGE_WIDTH	416
#define TEST_IMAGE_HEIGHT	416
#define NN_CONFIDENCE_THRESH  0.2f
#define NN_NMS_THRESH         0.3f

// COCO class names for object detection
static const char *coco_name[80] = {
    "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
};

typedef struct {
	uint8_t *output_buffer;
	uint32_t output_size;
} jpeg_buffer_t;

typedef struct {
    _sema snapshot_sema;
    unsigned char *jpeg_buf;
    unsigned int jpeg_len;
    unsigned int take_snapshot;
    unsigned int jpeg_index;
    uint32_t dest_addr;
    uint32_t dest_len;
    uint32_t dest_actual_len;
    _mutex snapshot_mutex;
    video_params_t video_snapshot_params;
    jpeg_buffer_t video_buf;
    mm_context_t *video_snapshot_ctx;
    
    // VIPNN object detection context
    mm_context_t *vipnn_ctx;
    mm_siso_t *siso_vipnn;
    nn_data_param_t nn_input_param;
    float nn_confidence_thresh;
    float nn_nms_thresh;
    
    int (*snapshot_write)(uint8_t *buf, uint32_t len, const char *filename);
    
    // Detection result callback
    void (*detection_callback)(detected_object_t *objects, int num_objects, uint32_t image_addr, uint32_t image_size);
} jpeg_aisnapshot_context_t;

// Global context
static jpeg_aisnapshot_context_t *ai_snap_ctx = NULL;

static video_pre_init_params_t ai_snap_pre_init_param;

// Function prototypes
static int video_snapshot_cb(uint32_t jpeg_addr, uint32_t jpeg_len);
static int video_snapshot_get_buffer(jpeg_buffer_t *video_buf, uint32_t timeout_ms);
static int video_capture_snapshot(const char *filename);
static int aisnapshot_write_picture(uint8_t *buf, uint32_t len, const char *filename);
static int jpeg_to_rgb888_planar(uint8_t *jpeg_data, uint32_t jpeg_size, uint8_t **rgb_data, uint32_t *rgb_size, int *width, int *height);
static void nn_detection_callback(void *p, void *img_param);
static int run_object_detection(uint32_t jpeg_addr, uint32_t jpeg_len, detected_object_t *objects, int max_objects);

// Public API for object detection
int ai_snapshot_obj_detect(const char *file_name, detected_object_t *objects, int max_objects);
// int ai_snapshot_detect(detected_object_t *objects, int max_objects);

static int video_snapshot_cb(uint32_t jpeg_addr, uint32_t jpeg_len)
{
	ai_snap_ctx->take_snapshot = 1;
	AI_GLASS_MSG("capture_snapshot_cb snapshot size = %lu\n\r", jpeg_len);
	ai_snap_ctx->dest_addr = (uint32_t) malloc(jpeg_len);
	memcpy((void *)ai_snap_ctx->dest_addr, (const void *)jpeg_addr, jpeg_len);
	ai_snap_ctx->dest_actual_len = jpeg_len;
	AI_GLASS_MSG("capture_snapshot_cb snapshot addr = %ld, size = %lu\n\r", ai_snap_ctx->dest_addr, ai_snap_ctx->dest_actual_len);
	rtw_up_sema(&ai_snap_ctx->snapshot_sema);
	return 0;
}

static int video_snapshot_get_buffer(jpeg_buffer_t *video_buf, uint32_t timeout_ms)
{
	if (rtw_down_timeout_sema(&ai_snap_ctx->snapshot_sema, timeout_ms)) {
		video_buf->output_buffer = (uint8_t *) ai_snap_ctx->dest_addr;
		video_buf->output_size = ai_snap_ctx->dest_actual_len;
		AI_GLASS_MSG("video_snapshot_get_buffer size = %p, %lu\n\r", video_buf->output_buffer, video_buf->output_size);
		return 0;
	} else {
		AI_GLASS_ERR("video_snapshot_get_buffer size fail\n\r");
		video_buf->output_buffer = NULL;
		video_buf->output_size = 0;
		return -1;
	}
}

static int video_capture_snapshot(const char *filename)
{
	int ret = -1;
	if (!video_snapshot_get_buffer(&ai_snap_ctx->video_buf, SNAPSHOT_TIMEOUT)) {
		AI_GLASS_MSG("video_snapshot_get_buffer size = %p, %lu\n\r", ai_snap_ctx->video_buf.output_buffer, ai_snap_ctx->video_buf.output_size);
		if (!ai_snap_ctx->snapshot_write(ai_snap_ctx->video_buf.output_buffer, ai_snap_ctx->video_buf.output_size, filename)) {
			ret = 0;
		}
		ai_snap_ctx->take_snapshot = 0;
		AI_GLASS_INFO("get ai snapshot buffer success\r\n");
	} else {
		AI_GLASS_ERR("get ai snapshot buffer failed\r\n");
	}
	return ret;
}

// Convert JPEG to RGB888 planar format for NN input
static int jpeg_to_rgb888_planar(uint8_t *jpeg_data, uint32_t jpeg_size, uint8_t **rgb_data, uint32_t *rgb_size, int *width, int *height)
{
    int w, h, c;
    int channels = 3;
    
    // Decode JPEG using stb_image
    uint8_t *im_data = stbi_load_from_memory(jpeg_data, jpeg_size, &w, &h, &c, channels);
    AI_GLASS_MSG("JPEG decoded: w:%d, h:%d, c:%d\n\r", w, h, c);
    
    if (im_data == NULL) {
        AI_GLASS_ERR("Failed to decode JPEG image\n\r");
        return -1;
    }
    
    if (c != 1 && c != 3) {
        AI_GLASS_ERR("Invalid image channels: %d\n\r", c);
        stbi_image_free(im_data);
        return -1;
    }
    
    // Allocate buffer for RGB planar format (R plane + G plane + B plane)
    int data_size = w * h * 3;
    uint8_t *rgb_planar_buf = (uint8_t *)malloc(data_size);
    if (rgb_planar_buf == NULL) {
        AI_GLASS_ERR("Failed to allocate RGB planar buffer\n\r");
        stbi_image_free(im_data);
        return -1;
    }
    
    // Convert RGB packed to RGB planar
    for (int k = 0; k < 3; k++) {
        for (int j = 0; j < h; j++) {
            for (int i = 0; i < w; i++) {
                int dst_i = i + w * j + w * h * k;
                int src_i = k + c * i + c * w * j;
                rgb_planar_buf[dst_i] = im_data[src_i];
            }
        }
    }
    
    *rgb_data = rgb_planar_buf;
    *rgb_size = (uint32_t)data_size;
    *width = w;
    *height = h;
    
    stbi_image_free(im_data);
    return 0;
}

// NN detection callback - prints detected objects
static void nn_detection_callback(void *p, void *img_param)
{
    vipnn_out_buf_t *out = (vipnn_out_buf_t *)p;
    objdetect_res_t *od_res = (objdetect_res_t *)&out->res[0];
    
    nn_data_param_t *im_param = (nn_data_param_t *)img_param;
    int im_w = im_param->img.width;
    int im_h = im_param->img.height;
    
    AI_GLASS_MSG("Object detection results: %d objects detected\n\r", out->res_cnt);
    
    if (out->res_cnt > 0) {
        for (int i = 0; i < out->res_cnt; i++) {
            int class_id = (int)(od_res[i].result[0]);
            float probability = od_res[i].result[1];
            
            int top_x = (int)(od_res[i].result[2] * im_w);
            int top_y = (int)(od_res[i].result[3] * im_h);
            int bottom_x = (int)(od_res[i].result[4] * im_w);
            int bottom_y = (int)(od_res[i].result[5] * im_h);
            
            // Clamp coordinates
            if (top_x < 0) top_x = 0;
            if (top_y < 0) top_y = 0;
            if (bottom_x > im_w) bottom_x = im_w;
            if (bottom_y > im_h) bottom_y = im_h;
            
            const char *class_name = "unknown";
            if (class_id >= 0 && class_id < 80) {
                class_name = coco_name[class_id];
            }
            
            AI_GLASS_MSG("  [%d] %s: %.2f%% (x1:%d, y1:%d, x2:%d, y2:%d)\n\r",
                         i, class_name, probability * 100, top_x, top_y, bottom_x, bottom_y);
        }
    }
}

// Run object detection on a JPEG image and populate results
// Note: This function uses the SISO pipeline for VIPNN inference
static int run_object_detection(uint32_t jpeg_addr, uint32_t jpeg_len, detected_object_t *objects, int max_objects)
{
    if (ai_snap_ctx->vipnn_ctx == NULL) {
        AI_GLASS_ERR("VIPNN module not initialized\n\r");
        return -1;
    }
    
    // Decode JPEG to RGB888 planar
    uint8_t *rgb_data = NULL;
    uint32_t rgb_size = 0;
    int img_width = 0, img_height = 0;
    
    if (jpeg_to_rgb888_planar((uint8_t *)jpeg_addr, jpeg_len, &rgb_data, &rgb_size, &img_width, &img_height) != 0) {
        AI_GLASS_ERR("Failed to convert JPEG to RGB888\n\r");
        return -1;
    }
    
    AI_GLASS_MSG("Running object detection on %dx%d image\n\r", img_width, img_height);
    
    // Update NN input parameters with actual image size
    ai_snap_ctx->nn_input_param.img.width = img_width;
    ai_snap_ctx->nn_input_param.img.height = img_height;
    ai_snap_ctx->nn_input_param.img.roi.xmin = 0;
    ai_snap_ctx->nn_input_param.img.roi.ymin = 0;
    ai_snap_ctx->nn_input_param.img.roi.xmax = img_width;
    ai_snap_ctx->nn_input_param.img.roi.ymax = img_height;
    
    // Apply updated input parameters
    mm_module_ctrl(ai_snap_ctx->vipnn_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&ai_snap_ctx->nn_input_param);
    
    // Allocate output buffer for VIPNN results
    uint32_t output_buf_size = sizeof(vipnn_out_buf_t) + MAX_DETECT_OBJ_NUM * sizeof(objdetect_res_t);
    uint32_t output_buf_addr = (uint32_t)malloc(output_buf_size);
    if (output_buf_addr == 0) {
        AI_GLASS_ERR("Failed to allocate output buffer\n\r");
        free(rgb_data);
        return -1;
    }
    
    // Create input queue item with RGB data
    mm_queue_item_t input_item = {0};
    input_item.data_addr = (uint32_t)rgb_data;
    input_item.size = rgb_size;
    input_item.type = AV_CODEC_ID_RGB888;
    input_item.flag = MMQI_FLAG_READY;
    
    mm_queue_item_t output_item = {0};
    output_item.data_addr = output_buf_addr;
    output_item.size = output_buf_size;
    
    // Call VIPNN module's handle function directly through the module context
    // The handle function signature: int (*handle)(void *ctx, void *input, void *output)
    int result = 0;
    if (ai_snap_ctx->vipnn_ctx->module && ai_snap_ctx->vipnn_ctx->module->handle) {
        result = ai_snap_ctx->vipnn_ctx->module->handle(ai_snap_ctx->vipnn_ctx->priv, &input_item, &output_item);
    }
    
    if (result > 0 || (result == 0 && output_item.data_addr != 0)) {
        // Parse detection results
        vipnn_out_buf_t *out = (vipnn_out_buf_t *)output_item.data_addr;
        objdetect_res_t *od_res = (objdetect_res_t *)&out->res[0];
        int num_detected = out->res_cnt;
        
        AI_GLASS_MSG("Object detection results: %d objects detected\n\r", num_detected);
        
        // Copy results to output array
        int copy_count = (num_detected < max_objects) ? num_detected : max_objects;
        for (int i = 0; i < copy_count; i++) {
            objects[i].class_id = (int)(od_res[i].result[0]);
            objects[i].confidence = od_res[i].result[1];
            
            // Calculate bounding box coordinates
            objects[i].x1 = (int)(od_res[i].result[2] * img_width);
            objects[i].y1 = (int)(od_res[i].result[3] * img_height);
            objects[i].x2 = (int)(od_res[i].result[4] * img_width);
            objects[i].y2 = (int)(od_res[i].result[5] * img_height);
            
            // Clamp coordinates
            if (objects[i].x1 < 0) objects[i].x1 = 0;
            if (objects[i].y1 < 0) objects[i].y1 = 0;
            if (objects[i].x2 > img_width) objects[i].x2 = img_width;
            if (objects[i].y2 > img_height) objects[i].y2 = img_height;
            
            // Get class name
            if (objects[i].class_id >= 0 && objects[i].class_id < 80) {
                strncpy(objects[i].class_name, coco_name[objects[i].class_id], 31);
                objects[i].class_name[31] = '\0';
            } else {
                strncpy(objects[i].class_name, "unknown", 31);
                objects[i].class_name[31] = '\0';
            }
            
            // Print each detection
            AI_GLASS_MSG("  [%d] %s: %.2f%% (x1:%d, y1:%d, x2:%d, y2:%d)\n\r",
                         i, objects[i].class_name, objects[i].confidence * 100,
                         objects[i].x1, objects[i].y1, objects[i].x2, objects[i].y2);
        }
        
        // Free output buffer
        free((void *)output_buf_addr);
    } else {
        AI_GLASS_ERR("VIPNN processing failed, result=%d\n\r", result);
        if (output_buf_addr) free((void *)output_buf_addr);
    }
    
    // Free the RGB data
    free(rgb_data);
    
    AI_GLASS_MSG("Object detection completed\n\r");
    return 0;
}

// Save JPEG buffer to file without freeing the buffer (used for detect + save workflow)
static int aisnapshot_save_picture_only(uint8_t *buf, uint32_t len, const char *filename)
{
	FILE *m_file = NULL;
	AI_GLASS_MSG("jpeg %s, file len = %lu\r\n", filename, len);
	if (buf && len > 0) {
		m_file = ramdisk_fopen(filename, "w");
		if (m_file) {
			ramdisk_fwrite(buf, 1, len, m_file);
			ramdisk_fclose(m_file);
			AI_GLASS_MSG("Snapshot saved to %s\r\n", filename);
			return 0;
		} else {
			AI_GLASS_ERR("Failed to open file %s for writing\r\n", filename);
			return -1;
		}
	} else {
		AI_GLASS_ERR("Invalid buffer or length\r\n");
		return -1;
	}
}

// Original write function: save and free buffer (used for take-only workflow)
static int aisnapshot_write_picture(uint8_t *buf, uint32_t len, const char *filename)
{
	FILE *m_file = NULL;
	AI_GLASS_MSG("jpeg %s, file len = %lu\r\n", filename, len);
	if (ai_snap_ctx->dest_addr) {
		m_file = ramdisk_fopen(filename, "w");
		if (m_file) {
			ramdisk_fwrite(buf, 1, len, m_file);
			ramdisk_fclose(m_file);
			free((void *)ai_snap_ctx->dest_addr);
			ai_snap_ctx->dest_addr = 0;
			return 0;
		} else {
			free((void *)ai_snap_ctx->dest_addr);
			ai_snap_ctx->dest_addr = 0;
			return -1;
		}
	} else {
		AI_GLASS_ERR("jpeg buffer allocate fail\r\n");
		return -1;
	}
}

// Helper to free the snapshot buffer after use
static void aisnapshot_free_buffer(void)
{
	if (ai_snap_ctx->dest_addr) {
		free((void *)ai_snap_ctx->dest_addr);
		ai_snap_ctx->dest_addr = 0;
	}
}

// Initialize object detection for AI snapshot
int ai_snapshot_initialize(void)
{
	int ret = 0;

	if (ai_snap_ctx == NULL) {
		AI_GLASS_INFO("================AI snapshot start==========================\r\n");
		ai_snap_ctx = (jpeg_aisnapshot_context_t *) malloc(sizeof(jpeg_aisnapshot_context_t));
		memset(ai_snap_ctx, 0x00, sizeof(jpeg_aisnapshot_context_t));

		ai_glass_snapshot_param_t ai_snap_param;
		memset(&ai_snap_param, 0x00, sizeof(ai_glass_snapshot_param_t));
		media_get_ai_snapshot_params(&ai_snap_param);

		ai_snap_ctx->video_snapshot_ctx = mm_module_open(&video_module);
		video_params_t *snapshot_param = &(ai_snap_ctx->video_snapshot_params);
		snapshot_param->stream_id = MAIN_STREAM_ID;
		snapshot_param->type = VIDEO_H264_JPEG;
		snapshot_param->width = ai_snap_param.width;
		snapshot_param->height = ai_snap_param.height;
		snapshot_param->rotation = ai_snap_param.rotation;
		snapshot_param->jpeg_qlevel = ai_snap_param.jpeg_qlevel;
		snapshot_param->fps = sensor_params[current_sensor_id].sensor_fps;
		snapshot_param->roi.xmin = 0;
		snapshot_param->roi.ymin = 0;
		snapshot_param->roi.xmax = 0;
		snapshot_param->roi.ymax = 0;
		snapshot_param->use_static_addr = 1;
		AI_GLASS_MSG("snapshot width = %ld\r\n", snapshot_param->width);
		AI_GLASS_MSG("snapshot height = %ld\r\n", snapshot_param->height);
		AI_GLASS_MSG("snapshot jpeg_qlevel = %ld\r\n", snapshot_param->jpeg_qlevel);

		video_pre_init_params_t ai_glass_pre_init_params = {0};
		rtw_init_sema(&ai_snap_ctx->snapshot_sema, 0);
		rtw_mutex_init(&ai_snap_ctx->snapshot_mutex);
		ai_snap_ctx->snapshot_write = aisnapshot_write_picture;
		
		// Initialize VIPNN for object detection
		ai_snap_ctx->vipnn_ctx = mm_module_open(&vipnn_module);
		if (ai_snap_ctx->vipnn_ctx) {
			// Set YOLOv4_tiny model
			mm_module_ctrl(ai_snap_ctx->vipnn_ctx, CMD_VIPNN_SET_MODEL, (int)&NN_MODEL_OBJ);
			
			// Set input parameters (416x416 RGB888)
			ai_snap_ctx->nn_input_param.img.width = TEST_IMAGE_WIDTH;
			ai_snap_ctx->nn_input_param.img.height = TEST_IMAGE_HEIGHT;
			ai_snap_ctx->nn_input_param.img.rgb = 0;
			ai_snap_ctx->nn_input_param.img.roi.xmin = 0;
			ai_snap_ctx->nn_input_param.img.roi.ymin = 0;
			ai_snap_ctx->nn_input_param.img.roi.xmax = TEST_IMAGE_WIDTH;
			ai_snap_ctx->nn_input_param.img.roi.ymax = TEST_IMAGE_HEIGHT;
			ai_snap_ctx->nn_input_param.codec_type = AV_CODEC_ID_RGB888;
			mm_module_ctrl(ai_snap_ctx->vipnn_ctx, CMD_VIPNN_SET_IN_PARAMS, (int)&ai_snap_ctx->nn_input_param);
			
			// Set detection thresholds
			ai_snap_ctx->nn_confidence_thresh = NN_CONFIDENCE_THRESH;
			ai_snap_ctx->nn_nms_thresh = NN_NMS_THRESH;
			mm_module_ctrl(ai_snap_ctx->vipnn_ctx, CMD_VIPNN_SET_CONFIDENCE_THRES, (int)&ai_snap_ctx->nn_confidence_thresh);
			mm_module_ctrl(ai_snap_ctx->vipnn_ctx, CMD_VIPNN_SET_NMS_THRES, (int)&ai_snap_ctx->nn_nms_thresh);
			
			// Set result parameters
			mm_module_ctrl(ai_snap_ctx->vipnn_ctx, CMD_VIPNN_SET_RES_SIZE, sizeof(objdetect_res_t));
			mm_module_ctrl(ai_snap_ctx->vipnn_ctx, CMD_VIPNN_SET_RES_MAX_CNT, MAX_DETECT_OBJ_NUM);
			
			// Set display callback for printing results
			mm_module_ctrl(ai_snap_ctx->vipnn_ctx, CMD_VIPNN_SET_DISPPOST, (int)nn_detection_callback);
			
			// Enable module output
			mm_module_ctrl(ai_snap_ctx->vipnn_ctx, CMD_VIPNN_SET_OUTPUT, 1);
			
			// Set queue length
			mm_module_ctrl(ai_snap_ctx->vipnn_ctx, MM_CMD_SET_QUEUE_LEN, 2);
			mm_module_ctrl(ai_snap_ctx->vipnn_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
			
			// Apply VIPNN configuration
			mm_module_ctrl(ai_snap_ctx->vipnn_ctx, CMD_VIPNN_APPLY, 0);
			AI_GLASS_INFO("VIPNN object detection initialized with YOLOv4_tiny\n\r");
		} else {
			AI_GLASS_WARN("VIPNN module open failed, object detection disabled\n\r");
		}
		
		if (ai_snap_ctx->video_snapshot_ctx) {
			media_get_preinit_isp_data(&ai_glass_pre_init_params);
			ai_glass_pre_init_params.video_drop_enable = 1;
			ai_glass_pre_init_params.video_drop_frame = DROP_FRAME;
			mm_module_ctrl(ai_snap_ctx->video_snapshot_ctx, CMD_VIDEO_PRE_INIT_PARM, (int)&ai_glass_pre_init_params);
			mm_module_ctrl(ai_snap_ctx->video_snapshot_ctx, CMD_VIDEO_SNAPSHOT_CB, (int)video_snapshot_cb);
			mm_module_ctrl(ai_snap_ctx->video_snapshot_ctx, CMD_VIDEO_SET_PARAMS, (int) & (ai_snap_ctx->video_snapshot_params));
			mm_module_ctrl(ai_snap_ctx->video_snapshot_ctx, CMD_VIDEO_PRE_INIT_PARM, (int)&ai_snap_pre_init_param);
			mm_module_ctrl(ai_snap_ctx->video_snapshot_ctx, MM_CMD_SET_QUEUE_LEN, 2);//Default 30
			mm_module_ctrl(ai_snap_ctx->video_snapshot_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
			mm_module_ctrl(ai_snap_ctx->video_snapshot_ctx, CMD_VIDEO_APPLY, ai_snap_ctx->video_snapshot_params.stream_id);
			// video_ctrl(0, VIDEO_DEBUG, 0);
		} else {
			ret = -1;
			ai_snapshot_deinitialize();
			AI_GLASS_ERR("AI snapshot open fail\r\n");
			goto endofaisnapshot;
		}
	} else {
		ret = -2;
		AI_GLASS_WARN("AI snapshot is on-going\r\n");
		goto endofaisnapshot;
	}
endofaisnapshot:
	return ret;
}

// Take snapshot, run object detection, (optional)save to file
int ai_snapshot_obj_detect(const char *file_name, detected_object_t *objects, int max_objects)
{
	AI_GLASS_INFO("================ai_obj_detect==========================\r\n");
	int ret = -1;
	
	if (ai_snap_ctx) {
		AI_GLASS_MSG("Snapshot, save, and detection start\r\n");
		rtw_mutex_get(&ai_snap_ctx->snapshot_mutex);
		
		// Take snapshot
		ai_snap_ctx->take_snapshot = 1;
		mm_module_ctrl(ai_snap_ctx->video_snapshot_ctx, CMD_VIDEO_SNAPSHOT, 1);
		
		// Wait for snapshot buffer
		if (!video_snapshot_get_buffer(&ai_snap_ctx->video_buf, SNAPSHOT_TIMEOUT)) {
			uint8_t *jpeg_buf = ai_snap_ctx->video_buf.output_buffer;
			uint32_t jpeg_len = ai_snap_ctx->video_buf.output_size;
			
			AI_GLASS_MSG("Snapshot captured: %p, %lu\n\r", jpeg_buf, jpeg_len);
			
			/* Step 1: Run object detection FIRST (before buffer is freed) */
			if (ai_snap_ctx->vipnn_ctx != NULL && objects && max_objects > 0) {
				AI_GLASS_MSG("Running object detection on captured image...\r\n");
				int detect_result = run_object_detection(
					(uint32_t)jpeg_buf,
					jpeg_len,
					objects,
					max_objects
				);
				if (detect_result != 0) {
					AI_GLASS_WARN("Object detection failed, continuing to save\r\n");
				}
			} else if (ai_snap_ctx->vipnn_ctx == NULL) {
				AI_GLASS_WARN("VIPNN not available, skipping detection\n\r");
			}
		
			// Save to image to ram
			if (aisnapshot_save_picture_only(jpeg_buf, jpeg_len, file_name) == 0) {
				AI_GLASS_MSG("Snapshot saved successfully to %s\r\n", file_name);
			} else {
				AI_GLASS_ERR("Failed to save snapshot to %s\r\n", file_name);
			}
		
		#if AI_SNAPSHOT_SAVE_DETECT_IMAGE
			// Optionally save to external disk as well
			ai_glass_init_external_disk();
			FILE *life_snapshot_file = extdisk_fopen(file_name, "wb");

			// write jpg data
			for (uint32_t i = 0; i < jpeg_len; i += JPG_WRITE_SIZE) {
				extdisk_fwrite((const void *)(jpeg_buf + i), 1, ((i + JPG_WRITE_SIZE) >= jpeg_len) ? (jpeg_len - i) : JPG_WRITE_SIZE, life_snapshot_file);
			}
			extdisk_fclose(life_snapshot_file);
		#endif

			// Free the buffer manually after detection and saving are done
			aisnapshot_free_buffer();
			
			ai_snap_ctx->take_snapshot = 0;
			AI_GLASS_INFO("Snapshot saved and detection completed\r\n");
			ret = 0;
		} else {
			AI_GLASS_ERR("Failed to capture snapshot\r\n");
			ai_snap_ctx->take_snapshot = 0;
		}
		
		rtw_mutex_put(&ai_snap_ctx->snapshot_mutex);
	} else {
		AI_GLASS_ERR("The snapshot is not initialized\r\n");
	}
	
	return ret;
}

// Take snapshot only (without detection)
int ai_snapshot_take(const char *file_name)
{
	AI_GLASS_INFO("================ai_snapshot_take==========================\r\n");
	int ret = -1;
	if (ai_snap_ctx) {
		AI_GLASS_MSG("Snapshot start\r\n");
		rtw_mutex_get(&ai_snap_ctx->snapshot_mutex);
		AI_GLASS_MSG("ai_snapshot_take %s\r\n", file_name);
		ai_snap_ctx->take_snapshot = 1;
		mm_module_ctrl(ai_snap_ctx->video_snapshot_ctx, CMD_VIDEO_SNAPSHOT, 1);
		ret = video_capture_snapshot(file_name);
		ai_snap_ctx->take_snapshot = 0;
		rtw_mutex_put(&ai_snap_ctx->snapshot_mutex);
	} else {
		AI_GLASS_ERR("The snapshot is not init\r\n");
	}
	return ret;
}

int ai_snapshot_deinitialize(void)
{
	if (ai_snap_ctx) {
		if (ai_snap_ctx->take_snapshot) {
			AI_GLASS_WARN("It is running\r\n");
			return -1;
		} else {
			rtw_free_sema(&ai_snap_ctx->snapshot_sema);
			rtw_mutex_free(&ai_snap_ctx->snapshot_mutex);
			
			// Clean up VIPNN module
			if (ai_snap_ctx->vipnn_ctx) {
				mm_module_close(ai_snap_ctx->vipnn_ctx);
				ai_snap_ctx->vipnn_ctx = NULL;
				AI_GLASS_INFO("VIPNN module closed\n\r");
			}
			
			// Clean up SISO link if created
			if (ai_snap_ctx->siso_vipnn) {
				siso_pause(ai_snap_ctx->siso_vipnn);
				siso_delete(ai_snap_ctx->siso_vipnn);
				ai_snap_ctx->siso_vipnn = NULL;
				AI_GLASS_INFO("SISO link deleted\n\r");
			}
			
			mm_module_ctrl(ai_snap_ctx->video_snapshot_ctx, CMD_VIDEO_STREAM_STOP, 0);
			mm_module_close(ai_snap_ctx->video_snapshot_ctx);
			free(ai_snap_ctx);
			ai_snap_ctx = NULL;
		}
	}
	AI_GLASS_INFO("DEINT WITH SINGLE\r\n");
	AI_GLASS_INFO("ai snapshot deinit\n");
	return 0;
}

