#include <string.h>
#include <stdio.h>
#include <math.h>
#include "basic_types.h"
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "module_video.h"
#include "module_rtsp2.h"
#include "mmf2_pro2_video_config.h"
#include "log_service.h"
#include "sensor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "vl53l5cx_api.h"
#include "tof_sens_ctrl_api.h"
#include "osd_api.h"
#include "osd_render.h"

#define V1_CHANNEL 0
#define V1_RESOLUTION VIDEO_FHD
#define V1_BPS 2*1024*1024
#define V1_RCMODE 2 // 1: CBR, 2: VBR
#define USE_H265 0
#if USE_H265
#include "sample_h265.h"
#define VIDEO_TYPE VIDEO_HEVC
#define VIDEO_CODEC AV_CODEC_ID_H265
#else
#include "sample_h264.h"
#define VIDEO_TYPE VIDEO_H264
#define VIDEO_CODEC AV_CODEC_ID_H264
#endif

#if V1_RESOLUTION == VIDEO_VGA
#define V1_WIDTH	640
#define V1_HEIGHT	480
#elif V1_RESOLUTION == VIDEO_HD
#define V1_WIDTH	1280
#define V1_HEIGHT	720
#elif V1_RESOLUTION == VIDEO_FHD
#define V1_WIDTH	1920
#define V1_HEIGHT	1080
#endif

#define CHANGE_FONT 0
#define USE_CUSTOM_1BPP 0
#if CHANGE_FONT
#if USE_CUSTOM_1BPP
#include "custom_font_1bpp.h"
#else
#include "custom_font_argb4444.h"
#endif
#endif

// Set what to show on VideoStream
#define DIST_ARR_EN				1
#define NEAREST_DETECT_EN		1
#define REGION_RECT_EN			1

// I2C
#define MBED_I2C_MTR_SDA 		PE_4
#define MBED_I2C_MTR_SCL 		PE_3
#define MBED_I2C_BUS_CLK		400000
#define I2C_DATA_LENGTH			128
#define I2C_DEVICE_ADDRESS		0x29
#define I2C_BUFFER_SIZE			32

// OSD block index
#define DIST_ARR_IDX_0 			0 
#define DIST_ARR_IDX_1 			1
#define RECT_IDX				2
#define LABEL_IDX				3

// ToF position offset
#define TOF_DX_MM           	0  // Left/right offset
#define TOF_DY_MM          	 	0 // Up/down offset
#define TOF_DZ_MM           	0  // Forward/backward offset

// Tof and Camera FOV
#define TOF_HFOV_DEG 			48
#define TOF_VFOV_DEG 			48
#define CAM_HFOV_DEG 			110
#define CAM_VFOV_DEG 			70

#define TRACKING_BOX_W 			60
#define TRACKING_BOX_H 			60
#define DIST_FIELD_WIDTH        4 // digits per distance
#define CHAR_WIDTH 				16
#define CHAR_HEIGHT 			28
#define H_GAP					50
#define V_GAP					10
#define START_X					20
#define START_Y					20
#define ROWS_PER_BLOCK 			4

#define ALIGN16(x) (((x) + 15) & ~15)

static char nearest_label_buf[32] = {0};

static void atcmd_userctrl_init(void);
static mm_context_t *video_v1_ctx			= NULL;
static mm_context_t *rtsp2_v1_ctx			= NULL;
static mm_siso_t *siso_video_rtsp_v1			= NULL;

static video_params_t video_v1_params = {
	.stream_id = V1_CHANNEL,
	.type = VIDEO_TYPE,
	.bps = V1_BPS,
	.rc_mode = V1_RCMODE,
	.use_static_addr = 1
};

static rtsp2_params_t rtsp2_v1_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = VIDEO_CODEC,
			.bps      = V1_BPS
		}
	}
};

tof_sens_ctx_t tof_sens_osd_ctx = {
	.tof_i2c_cfg.bus_clk_hz = MBED_I2C_BUS_CLK,
	.tof_i2c_cfg.i2c_addr = I2C_DEVICE_ADDRESS,
	.tof_i2c_cfg.i2c_data_length = I2C_DATA_LENGTH,
	.tof_i2c_cfg.i2c_buffer_size = I2C_BUFFER_SIZE,
	.tof_i2c_cfg.i2c_max_packet_size = I2C_DATA_LENGTH,
	.tof_i2c_cfg.sda = MBED_I2C_MTR_SDA,
	.tof_i2c_cfg.scl = MBED_I2C_MTR_SCL,
	.image_res = TOF_SENS_RESOLUTION_8X8,
};

static void tof_set_osd(void) {
	bool status;
	int tof_image_width = 0;
	int tof_image_height = 0;

	char dist_str[10] = {0};
	int dist_width = DIST_FIELD_WIDTH * CHAR_WIDTH;  // 4 digits per number
	int blk_idx = 0;

	printf("Initializing sensor board. This can take up to 10s. Please wait.\r\n");
	if (tof_sens_init(&tof_sens_osd_ctx, &tof_sens_osd_ctx.vl53l5cx_sensor) == false) {
		printf("Sensor not found - check your wiring. Freezing\r\n");
		while (1)
			;
	}
	vTaskDelay(pdMS_TO_TICKS(100));

	tof_image_width = sqrt(tof_sens_osd_ctx.image_res);
	tof_image_height = tof_image_width;

	status = tof_sens_start_ranging(&tof_sens_osd_ctx.vl53l5cx_sensor);
	if (!status) {
		printf("Start ranging failed\r\n");
		goto tof_sens_fail;
	}
	printf("Start ranging\r\n");

	// Calculate constant values once at initialization
	float tof_hfov_rad = TOF_HFOV_DEG * 0.5 * M_PI / 180.0;
	float tof_vfov_rad = TOF_VFOV_DEG * 0.5 * M_PI / 180.0;
	float cam_hfov_rad = CAM_HFOV_DEG * 0.5 * M_PI / 180.0;
	float cam_vfov_rad = CAM_VFOV_DEG * 0.5 * M_PI / 180.0;

	float scale_w = tanf(tof_hfov_rad) / tanf(cam_hfov_rad);
	float scale_h = tanf(tof_vfov_rad) / tanf(cam_vfov_rad);

	if (scale_w > 1.0) {
		scale_w = 1.0;
	}
	if (scale_h > 1.0) {
		scale_h = 1.0;
	}

	int region_w = (int)(video_v1_params.width * scale_w);
	int region_h = (int)(video_v1_params.height * scale_h);
	int region_x0 = (video_v1_params.width - region_w) / 2;
	int region_y0 = (video_v1_params.height - region_h) / 2;

	int zone_w = region_w / tof_image_width;
	int zone_h = region_h / tof_image_height;

	while (1) {
		// Poll sensor for new data
		int data_is_ready = 0;
		data_is_ready = tof_sens_data_ready(&tof_sens_osd_ctx.vl53l5cx_sensor);
		if (data_is_ready) {
			if (tof_sens_get_ranging_data(&tof_sens_osd_ctx.vl53l5cx_sensor, &tof_sens_osd_ctx.vl53l5cx_data)) { // Read distance data into array
#if DIST_ARR_EN
				canvas_create_bitmap(V1_CHANNEL, DIST_ARR_IDX_0, RTS_OSD2_BLK_FMT_1BPP);
				canvas_create_bitmap(V1_CHANNEL, DIST_ARR_IDX_1, RTS_OSD2_BLK_FMT_1BPP);
#endif

#if NEAREST_DETECT_EN || REGION_RECT_EN
				canvas_create_bitmap(V1_CHANNEL, RECT_IDX, RTS_OSD2_BLK_FMT_RGBA2222);
#endif

#if NEAREST_DETECT_EN
				canvas_create_bitmap(V1_CHANNEL, LABEL_IDX, RTS_OSD2_BLK_FMT_1BPP);
#endif
				int col_nearest = -1, row_nearest = -1;
				int d_nearest = 99999; // nearest valid distance, start from large number

				// Display all 8 rows x 8 columns inside the region
				for (int row = 0; row < tof_image_height; row++) {
					for (int col = 0; col < tof_image_width; col++) {
						int dist_mm = tof_sens_osd_ctx.vl53l5cx_data.distance_mm[row * tof_image_width + col];
						snprintf(dist_str, sizeof(dist_str), "%4d", dist_mm);

						// Find nearest valid distance
						if (dist_mm > 0 && dist_mm <= 4000) {
							if (dist_mm < d_nearest) {
								d_nearest = dist_mm;
								col_nearest = col;
								row_nearest = row;
							}
						}

						// Position array inside the region rectangle
						int x = region_x0 + col * zone_w + TOF_DX_MM;
						int y = region_y0 + row * zone_h + TOF_DY_MM;

						// Use block 0 for rows 0-3, block 1 for rows 4-7
						if (row < ROWS_PER_BLOCK) {
							blk_idx = DIST_ARR_IDX_0;
						} else {
							blk_idx = DIST_ARR_IDX_1;
						}
#if DIST_ARR_EN
						canvas_set_text(V1_CHANNEL, blk_idx, x, y, dist_str, COLOR_BLUE);
#endif
					}
				}
#if DIST_ARR_EN				
				canvas_update(V1_CHANNEL, DIST_ARR_IDX_0, 1);
				canvas_update(V1_CHANNEL, DIST_ARR_IDX_1, 1);
#endif

#if NEAREST_DETECT_EN || REGION_RECT_EN
				// Process nearest distance with tracking box
				if (col_nearest >= 0 && row_nearest >= 0) {
					// Map nearest ToF zone into camera pixel center (use pre-calculated values)
					int px = region_x0 + col_nearest * zone_w + zone_w / 2;
					int py = region_y0 + row_nearest * zone_h + zone_h / 2;

					int start_x = px - TRACKING_BOX_W / 2;
					int start_y = py - TRACKING_BOX_H / 2;

					if (start_x < 0) {
						start_x = 0;
					}

					if (start_y < 0) {
						start_y = 0;
					}

					if (start_x + TRACKING_BOX_W >= video_v1_params.width) {
						start_x = video_v1_params.width - TRACKING_BOX_W - 1;
					}

					if (start_y + TRACKING_BOX_H >= video_v1_params.height) {
						start_y = video_v1_params.height - TRACKING_BOX_H - 1;
					}

#if REGION_RECT_EN
					int x0 = region_x0;
					int y0 = region_y0;
					int x1 = region_x0 + region_w;
					int y1 = region_y0 + region_h;

					if (x0 < 0) {
						x0 = 0;
					}
					if (y0 < 0) {
						y0 = 0;
					}
					if (x1 > video_v1_params.width - 1) {
						x1 = video_v1_params.width - 1;
					}
					if (y1 > video_v1_params.height - 1) {
						y1 = video_v1_params.height - 1;
					}

					x0 = ALIGN16(x0);
					y0 = ALIGN16(y0);

					x1 = ALIGN16(x1);
					y1 = ALIGN16(y1);

					if (x1 > x0 && y1 > y0) {
						canvas_set_rect(V1_CHANNEL, RECT_IDX, x0, y0, x1, y1, 2, COLOR_GREEN);
					}
#endif

#if NEAREST_DETECT_EN
					int x_min = start_x;
					int y_min = start_y;
					int x_max = start_x + TRACKING_BOX_W;
					int y_max = start_y + TRACKING_BOX_H;
					if (x_max > x_min && y_max > y_min) {
						canvas_set_rect(V1_CHANNEL, RECT_IDX, 
										x_min, y_min, 
										x_max, y_max, 
										2, COLOR_RED);
					}

					// Update nearest distance label
					snprintf(nearest_label_buf, sizeof(nearest_label_buf), "%dmm", d_nearest);

					int label_x = start_x;
					int label_y = start_y - 30;
					if (label_y < 0) {
						label_y = 0;
					}

					canvas_set_text(V1_CHANNEL, LABEL_IDX, label_x, label_y, nearest_label_buf, COLOR_RED);
#endif
				}	
#if NEAREST_DETECT_EN || REGION_RECT_EN
				canvas_update(V1_CHANNEL, RECT_IDX, 1);
#endif

#if NEAREST_DETECT_EN
				canvas_update(V1_CHANNEL, LABEL_IDX, 1);
#endif
#endif
			}
		}
		vTaskDelay(pdMS_TO_TICKS(10));  // Poll every 10ms
	}
	return;
tof_sens_fail:
	return;
}

void tof_video_osd_task(void *param)
{
	atcmd_userctrl_init();

	/*sensor capacity check & video parameter setting*/
	video_v1_params.resolution = VIDEO_FHD;
	video_v1_params.width = sensor_params[USE_SENSOR].sensor_width;
	video_v1_params.height = sensor_params[USE_SENSOR].sensor_height;
	video_v1_params.fps = sensor_params[USE_SENSOR].sensor_fps;
	video_v1_params.gop = sensor_params[USE_SENSOR].sensor_fps;
	/*rtsp parameter setting*/
	rtsp2_v1_params.u.v.fps = sensor_params[USE_SENSOR].sensor_fps;
#if (USE_UPDATED_VIDEO_HEAP == 0)
	int voe_heap_size = video_voe_presetting(1, video_v1_params.width, video_v1_params.height, V1_BPS, 0,
						0, 0, 0, 0, 0,
						0, 0, 0, 0, 0,
						0, 0, 0);
#else
	int voe_heap_size = video_voe_presetting_by_params(&video_v1_params, 0, NULL, 0, NULL, 0, NULL);
#endif
	printf("\r\n voe heap size = %d\r\n", voe_heap_size);
	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, video_v1_params.fps * 3);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
	} else {
		rt_printf("video open fail\n\r");
		goto tof_sens_example_fail;
	}

	rtsp2_v1_ctx = mm_module_open(&rtsp2_module);
	if (rtsp2_v1_ctx) {
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SELECT_STREAM, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_PARAMS, (int)&rtsp2_v1_params);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_APPLY, 0);
		mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, ON);
	} else {
		rt_printf("RTSP2 open fail\n\r");
		goto tof_sens_example_fail;
	}

	siso_video_rtsp_v1 = siso_create();
	if (siso_video_rtsp_v1) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		siso_ctrl(siso_video_rtsp_v1, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		siso_ctrl(siso_video_rtsp_v1, MMIC_CMD_ADD_INPUT, (uint32_t)video_v1_ctx, 0);
		siso_ctrl(siso_video_rtsp_v1, MMIC_CMD_ADD_OUTPUT, (uint32_t)rtsp2_v1_ctx, 0);
		siso_start(siso_video_rtsp_v1);
	} else {
		rt_printf("siso2 open fail\n\r");
		goto tof_sens_example_fail;
	}

	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);	// start channel 0
	int ch_enable[3] = {1, 0, 0};
	int char_resize_w[3] = {CHAR_WIDTH, 0, 0}, char_resize_h[3] = {CHAR_HEIGHT, 0, 0};
	int ch_width[3] = {V1_WIDTH, 0, 0}, ch_height[3] = {V1_HEIGHT, 0, 0};
	osd_render_dev_init(ch_enable, char_resize_w, char_resize_h);
	osd_render_task_start(ch_enable, ch_width, ch_height);
	tof_set_osd();

	return;
tof_sens_example_fail:
	return;
}

static const char *example = "tof_sens_example_osd_nearest_dist_detect_init";
static void example_deinit(void)
{
	osd_render_task_stop();
	osd_render_dev_deinit_all();

	//Pause Linker
	siso_pause(siso_video_rtsp_v1);

	//Stop module
	mm_module_ctrl(rtsp2_v1_ctx, CMD_RTSP2_SET_STREAMMING, OFF);
	mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, V1_CHANNEL);

	//Delete linker
	siso_delete(siso_video_rtsp_v1);

	//Close module
	mm_module_close(rtsp2_v1_ctx);
	mm_module_close(video_v1_ctx);

	video_voe_release();
}

static void fUC(void *arg)
{
	static uint32_t user_cmd = 0;

	if (!strcmp(arg, "TD")) {
		if (user_cmd & USR_CMD_EXAMPLE_DEINIT) {
			printf("invalid state, can not do %s deinit!\r\n", example);
		} else {
			example_deinit();
			user_cmd = USR_CMD_EXAMPLE_DEINIT;
			printf("deinit %s\r\n", example);
		}
	} else if (!strcmp(arg, "TSR")) {
		if (user_cmd & USR_CMD_EXAMPLE_DEINIT) {
			printf("reinit %s\r\n", example);
			sys_reset();
		} else {
			printf("invalid state, can not do %s reinit!\r\n", example);
		}
	} else {
		printf("invalid cmd");
	}

	printf("user command 0x%lx\r\n", user_cmd);
}

static log_item_t userctrl_items[] = {
	{"UC", fUC, },
};

static void atcmd_userctrl_init(void)
{
	log_service_add_table(userctrl_items, sizeof(userctrl_items) / sizeof(userctrl_items[0]));
}

void tof_sens_example_osd_init(void)
{
	if (xTaskCreate(tof_video_osd_task, "tof_sensor_isp_osd_task", 10 * 1024, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate failed", __FUNCTION__);
	}
}