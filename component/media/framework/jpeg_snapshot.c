#include "jpeg_encoder.h"
#include "jpeg_snapshot.h"
#include "FreeRTOS.h"
#include "task.h"
#include "isp_api.h"
#include "hal_cache.h"
#include "sensor.h"

struct isp_snapshot_context {
	isp_stream_t *stream;
	_sema isp_napshot_sema;
	isp_buf_t isp_buf;
	isp_info_t isp_info;
	isp_cfg_t cfg;
};

struct snapshot_context {
	struct jpeg_context *jpeg_ctx;
	uint8_t take_snapshot;
	uint8_t snapshot_processing;
	_sema snapshot_sema;
	struct isp_snapshot_context *isp_snapshot_ctx;
	uint32_t dest_addr;
	uint32_t dest_len;
	uint32_t dest_actual_len;
	int32_t (*cb_jpeg_snapshot)(void *);
	uint32_t current_time;
};

static struct snapshot_context *snapshot_ctx = NULL;

#define JPEG_SNAPSHOT_THRESHOLD 0

void jpeg_snapshot_initial_with_instance(void *jpeg_ctx, uint32_t dest_addr, uint32_t dest_len, uint8_t snapshot_during_jpeg_streaming)
{
	snapshot_ctx = (struct snapshot_context *) malloc(sizeof(struct snapshot_context));
	if (snapshot_ctx == NULL) {
		printf("[Error] Allocate snapshot_ctx fail\n\r");
		while (1);
	}

	memset(snapshot_ctx, 0, sizeof(struct snapshot_context));

	snapshot_ctx->jpeg_ctx = (struct jpeg_context *) jpeg_ctx;
	if (!snapshot_during_jpeg_streaming) {
		snapshot_ctx->jpeg_ctx->dest_addr = dest_addr;
		snapshot_ctx->jpeg_ctx->dest_len = dest_len;
	}

	snapshot_ctx->dest_addr = dest_addr;
	snapshot_ctx->dest_len = dest_len;

	snapshot_ctx->snapshot_processing = 0;
	snapshot_ctx->take_snapshot = 0;
	rtw_init_sema(&snapshot_ctx->snapshot_sema, 0);
}

void jpeg_snapshot_initial(uint32_t width, uint32_t height, uint32_t fps, uint32_t level, uint32_t dest_addr, uint32_t dest_len)
{
	isp_init_cfg_t isp_init_cfg;

	memset(&isp_init_cfg, 0, sizeof(isp_init_cfg));
	isp_init_cfg.pin_idx = ISP_PIN_IDX;
	isp_init_cfg.clk = SENSOR_CLK_USE;
	isp_init_cfg.ldc = LDC_STATE;
	isp_init_cfg.fps = SENSOR_FPS;
	video_subsys_init(&isp_init_cfg);

	struct jpeg_context *jpeg_ctx = (struct jpeg_context *) jpeg_open();
	if (jpeg_ctx == NULL) {
		printf("[Error] Allocate jpeg_ctx fail\n\r");
		while (1);
	}

	jpeg_ctx->jpeg_parm.height = height;
	jpeg_ctx->jpeg_parm.width = width;
	jpeg_ctx->jpeg_parm.qLevel = level;
	jpeg_ctx->jpeg_parm.ratenum = fps;
	jpeg_initial(jpeg_ctx, &jpeg_ctx->jpeg_parm);

	jpeg_snapshot_initial_with_instance((void *)jpeg_ctx, dest_addr, dest_len, 0);
}

int jpeg_snapshot_encode_cb(uint32_t y_addr, uint32_t uv_addr)
{
	if ((!snapshot_ctx->take_snapshot) || (snapshot_ctx->snapshot_processing)) {
		return 0;
	}

	int ret = 0;

	snapshot_ctx->jpeg_ctx->source_addr = y_addr;
	snapshot_ctx->jpeg_ctx->y_addr = y_addr;
	snapshot_ctx->jpeg_ctx->uv_addr = uv_addr;//source_addr+snapshot_ctx->jpeg_ctx->jpeg_parm.width*snapshot_ctx->jpeg_ctx->jpeg_parm.height;

	ret = jpeg_encode(snapshot_ctx->jpeg_ctx);
	snapshot_ctx->dest_actual_len = snapshot_ctx->jpeg_ctx->dest_actual_len;
	snapshot_ctx->dest_addr = snapshot_ctx->jpeg_ctx->dest_addr;


	if (ret < 0) {
		printf("snapshot_enc_ret = %d\r\n", ret);
	} else {
		snapshot_ctx->take_snapshot = 0;
		printf("snapshot size=%d\n\r", snapshot_ctx->jpeg_ctx->dest_actual_len);
		rtw_up_sema(&snapshot_ctx->snapshot_sema);
	}
	return ret;
}

void jpeg_snapshot_cb(void)
{
	if ((!snapshot_ctx->take_snapshot) || (snapshot_ctx->snapshot_processing)) {
		return;
	}

	snapshot_ctx->dest_actual_len = snapshot_ctx->jpeg_ctx->dest_actual_len;
	memcpy((u8 *)snapshot_ctx->dest_addr, (u8 *)snapshot_ctx->jpeg_ctx->dest_addr, snapshot_ctx->dest_actual_len);

	snapshot_ctx->take_snapshot = 0;
	printf("snapshot size=%d\n\r", snapshot_ctx->jpeg_ctx->dest_actual_len);
	rtw_up_sema(&snapshot_ctx->snapshot_sema);
}

void isp_snapshot_frame_complete_cb(void *p)
{
	struct isp_snapshot_context *ctx = (struct isp_snapshot_context *)p;
	isp_info_t *info = &ctx->stream->info;
	int is_output_ready = 0;

	if (info->isp_overflow_flag == 0) {
		is_output_ready = 1;
	} else {
		info->isp_overflow_flag = 0;
	}

	if (abs(mm_read_mediatime_ms() - snapshot_ctx->current_time) < JPEG_SNAPSHOT_THRESHOLD) {
		is_output_ready = 0;
	}

	if (is_output_ready) {
		isp_buf_t buf;
		isp_handle_buffer(ctx->stream, &buf, MODE_SNAPSHOT);
		//printf("finish\r\n");
		rtw_up_sema_from_isr(&ctx->isp_napshot_sema);
	} else {
		isp_handle_buffer(ctx->stream, NULL, MODE_SKIP);
	}
}

int jpeg_snapshot_isp_config(int streamid)
{
	unsigned char *isp_buffer = NULL;
	//isp_cfg_t cfg;

	if (snapshot_ctx == NULL) {
		return -1;
	}

	snapshot_ctx->isp_snapshot_ctx = (struct isp_snapshot_context *) malloc(sizeof(struct isp_snapshot_context));
	if (snapshot_ctx->isp_snapshot_ctx == NULL) {
		printf("[Error] Allocate isp_snapshot_ctx fail\n\r");
		while (1);
	}
	memset(snapshot_ctx->isp_snapshot_ctx, 0, sizeof(struct isp_snapshot_context));
	rtw_init_sema(&snapshot_ctx->isp_snapshot_ctx->isp_napshot_sema, 0);

	snapshot_ctx->isp_snapshot_ctx->cfg.format = ISP_FORMAT_YUV420_SEMIPLANAR;
	snapshot_ctx->isp_snapshot_ctx->cfg.fps = snapshot_ctx->jpeg_ctx->jpeg_parm.ratenum;
	snapshot_ctx->isp_snapshot_ctx->cfg.height = snapshot_ctx->jpeg_ctx->jpeg_parm.height;
	snapshot_ctx->isp_snapshot_ctx->cfg.width = snapshot_ctx->jpeg_ctx->jpeg_parm.width;
	snapshot_ctx->isp_snapshot_ctx->cfg.hw_slot_num = 1;
	snapshot_ctx->isp_snapshot_ctx->cfg.isp_id = streamid;
	snapshot_ctx->isp_snapshot_ctx->cfg.boot_mode = 0;

	snapshot_ctx->isp_snapshot_ctx->stream = isp_stream_create(&snapshot_ctx->isp_snapshot_ctx->cfg);//(&cfg);

	if (snapshot_ctx->isp_snapshot_ctx->stream->cfg.format == ISP_FORMAT_YUV420_SEMIPLANAR) {
		isp_buffer = malloc((int)(snapshot_ctx->isp_snapshot_ctx->stream->cfg.width * snapshot_ctx->isp_snapshot_ctx->stream->cfg.height * 1.5));
	} else {
		isp_buffer = malloc(snapshot_ctx->isp_snapshot_ctx->stream->cfg.width * snapshot_ctx->isp_snapshot_ctx->stream->cfg.height * 2);
	}

	snapshot_ctx->isp_snapshot_ctx->isp_buf.slot_id = 0;
	snapshot_ctx->isp_snapshot_ctx->isp_buf.y_addr = (unsigned int)isp_buffer;
	snapshot_ctx->isp_snapshot_ctx->isp_buf.uv_addr = snapshot_ctx->isp_snapshot_ctx->isp_buf.y_addr + snapshot_ctx->isp_snapshot_ctx->stream->cfg.width *
			snapshot_ctx->isp_snapshot_ctx->stream->cfg.height;
	isp_handle_buffer(snapshot_ctx->isp_snapshot_ctx->stream, &snapshot_ctx->isp_snapshot_ctx->isp_buf, MODE_SETUP);
	snapshot_ctx->isp_snapshot_ctx->stream->frame_complete_arg = (void *)snapshot_ctx->isp_snapshot_ctx;
	snapshot_ctx->isp_snapshot_ctx->stream->frame_complete = isp_snapshot_frame_complete_cb;

	isp_stream_set_complete_callback(snapshot_ctx->isp_snapshot_ctx->stream, isp_snapshot_frame_complete_cb, (void *)snapshot_ctx->isp_snapshot_ctx);
	isp_stream_apply(snapshot_ctx->isp_snapshot_ctx->stream);
	printf("jpeg_snapshot_isp_config finish\r\n");
	return 0;
}

int	jpeg_snapshot_get_buffer(VIDEO_BUFFER *video_buf, uint32_t timeout_ms)
{
	if (rtw_down_timeout_sema(&snapshot_ctx->snapshot_sema, timeout_ms)) {
		video_buf->output_buffer = (u8 *) snapshot_ctx->dest_addr;
		video_buf->output_buffer_size = snapshot_ctx->dest_len;
		video_buf->output_size = snapshot_ctx->dest_actual_len;
		return 1;
	} else {
		video_buf->output_buffer = NULL;
		video_buf->output_buffer_size = 0;
		video_buf->output_size = 0;
		return 0;
	}
}

void jpeg_snapshot_set_processing(uint8_t snapshot_processing)
{
	snapshot_ctx->snapshot_processing = snapshot_processing;
}

u8 jpeg_snapshot_get_processing()
{
	return snapshot_ctx->snapshot_processing;
}

void jpeg_snapshot_isp_callback(int arg)
{
	snapshot_ctx->cb_jpeg_snapshot = (int (*)(void *))arg;
}

int jpeg_snapshot_isp()
{
	int ret = 0;
	if (snapshot_ctx->snapshot_processing) {
		printf("the data is ongoing\r\n");
		return -1;
	}
	if (snapshot_ctx == NULL) {
		printf("[Error] jpeg_snapshot_isp fail, snapshot_ctx == NULL\n\r");
		return -1;
	}
	if (snapshot_ctx->isp_snapshot_ctx == NULL) {
		printf("[Error] jpeg_snapshot_isp fail, snapshot_ctx->isp_snapshot_ctx == NULL\n\r");
		return -1;
	}
	snapshot_ctx->take_snapshot = 1;
	isp_stream_start(snapshot_ctx->isp_snapshot_ctx->stream);
	snapshot_ctx->current_time = rtw_get_current_time();
	if (snapshot_ctx->cb_jpeg_snapshot) {
		snapshot_ctx->cb_jpeg_snapshot(NULL);
	}
	rtw_down_sema(&snapshot_ctx->isp_snapshot_ctx->isp_napshot_sema);
	isp_stream_stop(snapshot_ctx->isp_snapshot_ctx->stream);
	snapshot_ctx->jpeg_ctx->y_addr = snapshot_ctx->isp_snapshot_ctx->isp_buf.y_addr;
	snapshot_ctx->jpeg_ctx->uv_addr = snapshot_ctx->isp_snapshot_ctx->isp_buf.uv_addr;
	ret = jpeg_encode(snapshot_ctx->jpeg_ctx);
	snapshot_ctx->dest_actual_len = snapshot_ctx->jpeg_ctx->dest_actual_len;
	snapshot_ctx->dest_addr = snapshot_ctx->jpeg_ctx->dest_addr;

	if (ret < 0) {
		printf("jpeg encode fail ret = %d\r\n", ret);
	} else {
		snapshot_ctx->take_snapshot = 0;
		printf("snapshot size=%d\n\r", snapshot_ctx->jpeg_ctx->dest_actual_len);
		rtw_up_sema(&snapshot_ctx->snapshot_sema);
	}
	isp_handle_buffer(snapshot_ctx->isp_snapshot_ctx->stream, &snapshot_ctx->isp_snapshot_ctx->isp_buf, MODE_SETUP);
	snapshot_ctx->take_snapshot = 0;
	return ret;
}

void jpeg_snapshot_isp_deinit(void)//This is only for snapshot from isp
{
	if (snapshot_ctx->isp_snapshot_ctx) {
		jpeg_release(snapshot_ctx->jpeg_ctx);
		rtw_free_sema(&snapshot_ctx->snapshot_sema);
		free(snapshot_ctx->jpeg_ctx);
	}

	if (snapshot_ctx->isp_snapshot_ctx) {
		snapshot_ctx->isp_snapshot_ctx->stream = isp_stream_destroy(snapshot_ctx->isp_snapshot_ctx->stream);
		rtw_free_sema(&snapshot_ctx->isp_snapshot_ctx->isp_napshot_sema);
		free((void *)snapshot_ctx->isp_snapshot_ctx->isp_buf.y_addr);
		free((void *)snapshot_ctx->isp_snapshot_ctx);
		snapshot_ctx->isp_snapshot_ctx = NULL;
	}

	free(snapshot_ctx);
	snapshot_ctx = NULL;
}

int jpeg_snapshot_isp_change_resolution(int width, int height) //This is only for snapshot from isp
{
	if (snapshot_ctx->take_snapshot || snapshot_ctx->isp_snapshot_ctx == NULL || snapshot_ctx->jpeg_ctx == NULL) {
		printf("The snapshot is ongoing or snapshot_ctx is not initial\r\n");
		return -1;
	}
	if (snapshot_ctx->jpeg_ctx) {
		jpeg_release(snapshot_ctx->jpeg_ctx);
		snapshot_ctx->jpeg_ctx->jpeg_parm.height = height;
		snapshot_ctx->jpeg_ctx->jpeg_parm.width = width;
		jpeg_initial(snapshot_ctx->jpeg_ctx, &snapshot_ctx->jpeg_ctx->jpeg_parm);
	}

	if (snapshot_ctx->isp_snapshot_ctx) {
		snapshot_ctx->isp_snapshot_ctx->stream = isp_stream_destroy(snapshot_ctx->isp_snapshot_ctx->stream);
		free((void *)snapshot_ctx->isp_snapshot_ctx->isp_buf.y_addr);

		unsigned char *isp_buffer = NULL;
		//isp_cfg_t cfg;
		snapshot_ctx->isp_snapshot_ctx->cfg.width = width;
		snapshot_ctx->isp_snapshot_ctx->cfg.height = height;

		snapshot_ctx->isp_snapshot_ctx->stream = isp_stream_create(&snapshot_ctx->isp_snapshot_ctx->cfg);

		if (snapshot_ctx->isp_snapshot_ctx->stream->cfg.format == ISP_FORMAT_YUV420_SEMIPLANAR) {
			isp_buffer = malloc((int)(snapshot_ctx->isp_snapshot_ctx->stream->cfg.width * snapshot_ctx->isp_snapshot_ctx->stream->cfg.height * 1.5));
		} else {
			isp_buffer = malloc(snapshot_ctx->isp_snapshot_ctx->stream->cfg.width * snapshot_ctx->isp_snapshot_ctx->stream->cfg.height * 2);
		}

		snapshot_ctx->isp_snapshot_ctx->isp_buf.slot_id = 0; //snapshot_ctx->isp_snapshot_ctx->isp_buf.slot_id ;
		snapshot_ctx->isp_snapshot_ctx->isp_buf.y_addr = (unsigned int)isp_buffer;
		snapshot_ctx->isp_snapshot_ctx->isp_buf.uv_addr = snapshot_ctx->isp_snapshot_ctx->isp_buf.y_addr + snapshot_ctx->isp_snapshot_ctx->stream->cfg.width *
				snapshot_ctx->isp_snapshot_ctx->stream->cfg.height;
		isp_handle_buffer(snapshot_ctx->isp_snapshot_ctx->stream, &snapshot_ctx->isp_snapshot_ctx->isp_buf, MODE_SETUP);
		snapshot_ctx->isp_snapshot_ctx->stream->frame_complete_arg = (void *)snapshot_ctx->isp_snapshot_ctx;
		snapshot_ctx->isp_snapshot_ctx->stream->frame_complete = isp_snapshot_frame_complete_cb;

		isp_stream_set_complete_callback(snapshot_ctx->isp_snapshot_ctx->stream, isp_snapshot_frame_complete_cb, (void *)snapshot_ctx->isp_snapshot_ctx);
		isp_stream_apply(snapshot_ctx->isp_snapshot_ctx->stream);
		printf("jpeg_snapshot_isp_change_resolution finish\r\n");
	}
}

void jpeg_snapshot_stream()
{
	if (snapshot_ctx == NULL) {
		printf("[Error] jpeg_snapshot_stream fail, snapshot_ctx == NULL\n\r");
	} else {
		snapshot_ctx->take_snapshot = 1;
	}
}

int yuv_snapshot_isp_config(int width, int height, int fps, int streamid)
{
	isp_init_cfg_t isp_init_cfg;
	isp_cfg_t cfg;
	uint32_t before, after = 0;
	unsigned char *isp_buffer = NULL;

	if (snapshot_ctx) {
		printf("It has been initiated\r\n");
		return -1;
	}

	memset(&isp_init_cfg, 0, sizeof(isp_init_cfg));
	isp_init_cfg.pin_idx = ISP_PIN_IDX;
	isp_init_cfg.clk = SENSOR_CLK_USE;
	isp_init_cfg.ldc = LDC_STATE;
	isp_init_cfg.fps = SENSOR_FPS;
	isp_init_cfg.isp_fw_location = ISP_FW_LOCATION;
	video_subsys_init(&isp_init_cfg);

	snapshot_ctx = (struct snapshot_context *) malloc(sizeof(struct snapshot_context));
	if (snapshot_ctx == NULL) {
		printf("[Error] Allocate snapshot_ctx fail\n\r");
		return -1;
	}

	memset(snapshot_ctx, 0, sizeof(struct snapshot_context));

	cfg.format = ISP_FORMAT_YUV420_SEMIPLANAR;
	cfg.fps = fps;
	cfg.height = height;
	cfg.width = width;
	cfg.hw_slot_num = 1;
	cfg.isp_id = streamid;
	cfg.boot_mode = 0;//ISP_NORMAL_BOOT

	snapshot_ctx->isp_snapshot_ctx = (struct isp_snapshot_context *) malloc(sizeof(struct isp_snapshot_context));
	if (snapshot_ctx->isp_snapshot_ctx == NULL) {
		printf("[Error] Allocate isp_snapshot_ctx fail\n\r");
		return -1;
	}

	memset(snapshot_ctx->isp_snapshot_ctx, 0, sizeof(struct isp_snapshot_context));

	rtw_init_sema(&snapshot_ctx->isp_snapshot_ctx->isp_napshot_sema, 0);

	snapshot_ctx->isp_snapshot_ctx->stream = isp_stream_create(&cfg);

	if (snapshot_ctx->isp_snapshot_ctx->stream->cfg.format == ISP_FORMAT_YUV420_SEMIPLANAR) {
		isp_buffer = malloc((int)(snapshot_ctx->isp_snapshot_ctx->stream->cfg.width * snapshot_ctx->isp_snapshot_ctx->stream->cfg.height * 1.5));
	} else {
		isp_buffer = malloc(snapshot_ctx->isp_snapshot_ctx->stream->cfg.width * snapshot_ctx->isp_snapshot_ctx->stream->cfg.height * 2);
	}
	snapshot_ctx->isp_snapshot_ctx->isp_buf.slot_id = 0;
	snapshot_ctx->isp_snapshot_ctx->isp_buf.y_addr = (unsigned int)isp_buffer;
	snapshot_ctx->isp_snapshot_ctx->isp_buf.uv_addr = snapshot_ctx->isp_snapshot_ctx->isp_buf.y_addr + snapshot_ctx->isp_snapshot_ctx->stream->cfg.width *
			snapshot_ctx->isp_snapshot_ctx->stream->cfg.height;
	isp_handle_buffer(snapshot_ctx->isp_snapshot_ctx->stream, &snapshot_ctx->isp_snapshot_ctx->isp_buf, MODE_SETUP);
	snapshot_ctx->isp_snapshot_ctx->stream->frame_complete_arg = (void *)snapshot_ctx->isp_snapshot_ctx;
	snapshot_ctx->isp_snapshot_ctx->stream->frame_complete = isp_snapshot_frame_complete_cb;

	isp_stream_set_complete_callback(snapshot_ctx->isp_snapshot_ctx->stream, isp_snapshot_frame_complete_cb, (void *)snapshot_ctx->isp_snapshot_ctx);
	isp_stream_apply(snapshot_ctx->isp_snapshot_ctx->stream);

	return 0;
}

int yuv_snapshot_isp(unsigned char **raw_y)
{
	int ret = 0;
	if (snapshot_ctx == NULL) {
		printf("[Error] snapshot_isp fail, snapshot_ctx == NULL\n\r");
		return -1;
	}
	if (snapshot_ctx->isp_snapshot_ctx == NULL) {
		printf("[Error] jpeg_snapshot_isp fail, snapshot_ctx->isp_snapshot_ctx == NULL\n\r");
		return -1;
	}
	isp_stream_start(snapshot_ctx->isp_snapshot_ctx->stream);
	rtw_down_sema(&snapshot_ctx->isp_snapshot_ctx->isp_napshot_sema);
	isp_stream_stop(snapshot_ctx->isp_snapshot_ctx->stream);
	if (snapshot_ctx->isp_snapshot_ctx->stream->cfg.format == ISP_FORMAT_YUV420_SEMIPLANAR) {
		dcache_invalidate_by_addr((uint32_t *)snapshot_ctx->isp_snapshot_ctx->isp_buf.y_addr,
								  snapshot_ctx->isp_snapshot_ctx->stream->cfg.height * snapshot_ctx->isp_snapshot_ctx->stream->cfg.width * 1.5);
	} else {
		dcache_invalidate_by_addr((uint32_t *)snapshot_ctx->isp_snapshot_ctx->isp_buf.y_addr,
								  snapshot_ctx->isp_snapshot_ctx->stream->cfg.height * snapshot_ctx->isp_snapshot_ctx->stream->cfg.width * 2);
	}
	*raw_y = (unsigned char *)snapshot_ctx->isp_snapshot_ctx->isp_buf.y_addr;

	isp_handle_buffer(snapshot_ctx->isp_snapshot_ctx->stream, &snapshot_ctx->isp_snapshot_ctx->isp_buf, MODE_SETUP);
	return ret;
}

int yuv_snapshot_isp_deinit()
{
	if (snapshot_ctx) {
		snapshot_ctx->isp_snapshot_ctx->stream = isp_stream_destroy(snapshot_ctx->isp_snapshot_ctx->stream);
		rtw_free_sema(&snapshot_ctx->isp_snapshot_ctx->isp_napshot_sema);
		free((void *)snapshot_ctx->isp_snapshot_ctx->isp_buf.y_addr);
		free((void *)snapshot_ctx->isp_snapshot_ctx);
		free((void *)snapshot_ctx);
		snapshot_ctx->isp_snapshot_ctx = NULL;
		snapshot_ctx = NULL;
		return 0;
	} else {
		return -1;
	}
}