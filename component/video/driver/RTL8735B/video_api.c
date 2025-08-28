/******************************************************************************
*
* Copyright(c) 2021 - 2025 Realtek Corporation. All rights reserved.
*
******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <osdep_service.h>
#include <math.h>
#include "platform_stdlib.h"
#include <unistd.h>
#include <sys/wait.h>
#include "base_type.h"
#include "cmsis.h"
#include "error.h"
#include "hal.h"
#include "hal_video.h"
#include "video_api.h"
#include "boot_retention.h"
#include "platform_opts.h"
#include "hal_voe_nsc.h"
#include "video_boot.h"
#include "sensor.h"
#include "fwfs.h"
#include "hal_sys_ctrl.h"
#include "osd_api.h"
#include "isp_osd_example.h"
#include "device_lock.h"
#include "avcodec.h"
#include "isp_ctrl_api.h"
static isp_info_t isp_info;
static voe_info_t voe_info = {0};
static int voe_info_init = 0;
static int g_enc_buff_size[5] = {0, 0, 0, 0, 0};

static int calib_iq_en = 1;
#define CALIB_IQ_LOAD_FLASH 1
#define IQ_CALI_EN_AWB  (0x1 << 0)
#define IQ_CALI_EN_MLSC (0x1 << 1)
#define IQ_CALI_EN_NLSC (0x1 << 2)
static unsigned char *video_load_cal_iq(unsigned int cal_iq_start_addr);
u8 voe_ver[24] = {0};
u8 snr_voe_ver[24] = {0};
u8 snr_timestamp[8] = {0};
unsigned short fcs_version = 0xFFFF;
u8 iq_timestamp[8] = {0};
u8 iq_cus_ver[8] = {0};

#define FASTOSD_EN 0

extern enc2out_t tm_enc2out_log[3];

#if NONE_FCS_MODE
static int voe_load_sesnor_init = 0;
int video_get_fw_isp_info(void);
#endif

volatile video_peri_info_t g_video_peri_info = {0};

hal_gpio_adapter_t sensor_en_gpio;
int sensor_en_pin_delay_init = 0;
static int fcs_queue_start_time = 0;
static int fcs_queue_end_time = 0;
static video_pre_init_params_t video_pre_init_param = {
	.voe_dbg_disable = !APP_VOE_LOG_EN,
#if USE_VIDEO_HR_FLOW
	.v_cfg = NULL,
	.zoom_coef = NULL,
#endif
};

//Video boot config//
#include "fw_img_export.h"
#include "video_boot.h"
#include "ftl_common_api.h"
#include "../fwfs/fwfs.h"
extern BL4VOE_INFO_T bl4voe_shared;
extern uint8_t __eram_heap_start__[];
extern uint8_t __eram_bss_end__[];
extern uint8_t __eram_end__[];
extern void *pvPortInsertPreAlloc(void *addr, size_t xWantedSize);
video_boot_stream_t *isp_boot = (video_boot_stream_t *)bl4voe_shared.data;

__attribute__((section(".retention.table"))) boot_retention_table_t retention_table;
#ifdef USE_ISP_RETENTION_DATA
__attribute__((section(".retention.data"))) isp_retention_data_t isp_retention_data __attribute__((aligned(32)));
#endif

static mult_sensor_info_t multi_sensor_info = {0};
int video_isp_memcpy(void *dst, const void *src, u32 size);
int video_voe_memcpy(void *dst, const void *src, u32 size);
int video_load(int sensor_index);
void *video_fw_deinit(void);
void video_set_isp_info(isp_info_t *info);
int video_reset_fw(int ch, int id);
const unsigned int crc32_result[10] = {0xd202ef8d, 0xa505df1b, 0x3c0c8ea1, 0x4b0bbe37, 0xd56f2b94, 0xa2681b02, 0x3b614ab8, 0x4c667a2e, 0xdcd967bf, 0xabde5729};
#define FCS_ID  0x04
#define FCS_CRC 0x08
extern int __voe_code_start__[];
int voe_fw_reload = 0;
int video_load_fw(unsigned int sensor_start_addr);
unsigned char *video_load_sensor(unsigned int sensor_start_addr);
unsigned char *video_load_iq(unsigned int iq_start_addr);
static void isp_set_dn_initial_mode(int ch, int mode);
int video_get_voe_version(void);
static void video_rc_callback(int ch, bps_stbl_ctrl_t *bps_stbl_ctrl, uint32_t frame_size);

/////////////////////////////////
SECTION(".sdram.bss")
unsigned char sensor_buf[FW_SENSOR_SIZE];

SECTION(".sdram.bss")
unsigned char iq_buf[FW_IQ_SIZE];

SECTION(".sdram.bss")
unsigned char cal_iq_buf[FW_CAL_IQ_SIZE];

SECTION(".sdram.bss")
unsigned char video_sei_buf[VIDEO_META_REV_BUF];

static af_statis_t af_result;
static ae_statis_t ae_result;
static awb_statis_t awb_result;

SECTION(".sdram.bss")
ExifWorkspace jpeg_exif_wsp;

int video_dbg_level = VIDEO_LOG_MSG;

#define video_dprintf(level, ...) if(level >= video_dbg_level) printf(__VA_ARGS__)

const char *fmt_table[] = {"hevc", "h264", "jpeg", "nv12", "rgb", "nv16", "hevc", "h264"};
const char *paramter_table[] = {
	"-l 1 --gopSize=1 -U1 -u1 -q-1 -A-5 --smoothPsnrInGOP=1 --rcQpDeltaRange=15 --picQpDeltaRange=-4:6 --blockRCSize=1 --compressor=3", // HEVC
	"-l 1 --gopSize=1 -U1 -u1 -q-1 -A-5 --smoothPsnrInGOP=1 --rcQpDeltaRange=15 --picQpDeltaRange=-4:6 --blockRCSize=1 --compressor=3", // H264
	"-g 1 -b 1",			// JPEG
	"",						// NV12
	"",						// RGB
	"",						// NV16
};

static int isp_ch_buf_num[5] = {2, 2, 2, 2, 2};

static uint32_t last_system_ts_ms[5] = {0};
static uint32_t last_video_ts_ms[5] = {0};
static int offset_ms[5] = {0};
static uint32_t isp_overflow_times[5] = {0};
static uint32_t last_isp_ts[5] = {0};
static uint32_t isp_ts_initialed_flag[5] = {0};
static _mutex video_open_close_mutex = NULL;
#if VIDEO_MPU_VOE_HEAP
#include "cmsis.h"
#include "mpu.h"
void setup_mpu(uint32_t start, uint32_t end, int index, int enable)
{
	video_dprintf(VIDEO_LOG_MSG, "the mpu start %x end %x index %x enable %x\r\n", start, end, index, enable);
	MPU->MAIR0 = 0;
	MPU->RNR = index;// region 1
	MPU->RBAR = (start & (~0x1F)) | (0 << 3) | (0x2 << 1) | 0; // non-sharable readonly no exectualbe
	if (enable) {
		MPU->RLAR = (end & (~0x1F)) | (0 << 4) | (2 << 2) | 1;    // use attr2, EN=1 M
	} else {
		MPU->RLAR = (end & (~0x1F)) | (0 << 4) | (2 << 2) | 0;
	}
	MPU->CTRL |= MPU_CTRL_ENABLE_Msk;
}

void video_mpu_trigger(void)
{
	unsigned char *ptr = (unsigned char *)voe_info.voe_heap_addr;
	ptr[0] = 0x00;
	ptr[1] = 0xff;
}
#endif

void video_set_isp_ch_buf(int ch, int slot_num)
{
	video_dprintf(VIDEO_LOG_INF, "modify ch%d slot num %d\r\n", ch, slot_num);
	isp_ch_buf_num[ch] = slot_num;
}

int video_get_maxqp(int ch)
{
	if (voe_info.ch_info[ch].rc_info == NULL) {
		video_dprintf(VIDEO_LOG_ERR, "[%s] ch%d rc_info not init\r\n", __FUNCTION__, ch);
		return NOK;
	}
	return voe_info.ch_info[ch].rc_info->rc_ctrl.maxqp;
}

unsigned char *video_get_iq_buf(void)
{
	return iq_buf;
}
void video_set_debug_level(int value)
{
	if (value <= VIDEO_LOG_OFF) {
		video_dbg_level = value;
	} else {
		video_dprintf(VIDEO_LOG_ERR, "The maximu is %d\r\n", VIDEO_LOG_OFF);
	}
}

int video_set_framerate(int fps)
{
	int max_fps = fps;
	int min_fps = fps;
	int ret = 0;
	ret = hal_video_isp_ctrl(0xF021, 1, &min_fps);
	ret = hal_video_isp_ctrl(0xF022, 1, &max_fps);
	return ret;
}

int video_get_buffer_info(int ch, int *enc_size, int *out_buf_size, int *out_rsvd_size)
{
	if (ch <= 2) {
		hal_video_adapter_t *v_adp = hal_video_get_adp();
		*enc_size = tm_enc2out_log[ch].enc_used;
		*out_buf_size =	v_adp->cmd[ch]->out_buf_size;
		*out_rsvd_size = v_adp->cmd[ch]->out_rsvd_size;
		return (int)&tm_enc2out_log[ch];
	} else {
		return 0;
	}
}

// Output stream callback function
static void temp_output_cb(void *param1, void  *param2, uint32_t arg)
{
	enc2out_t *enc2out = (enc2out_t *)param1;
	//hal_video_adapter_t  *v_adp = (hal_video_adapter_t *)param2;
	//commandLine_s *cml = (commandLine_s *)&v_adp->cmd[enc2out->ch];

	video_dprintf(VIDEO_LOG_INF, "ch = %d, type = %d\r\n", enc2out->ch, enc2out->codec);

	// VOE status check
	if (enc2out->cmd_status != VOE_OK) {
		switch (enc2out->cmd_status) {
		case VOE_ENC_BUF_OVERFLOW:
		case VOE_ENC_QUEUE_OVERFLOW:
		case VOE_JPG_BUF_OVERFLOW:
		case VOE_JPG_QUEUE_OVERFLOW:
			video_dprintf(VIDEO_LOG_MSG, "VOE CH%d ENC/BUF overflow\n", enc2out->ch);
			break;
		default:
			video_dprintf(VIDEO_LOG_MSG, "Error CH%d VOE status %x\n", enc2out->ch, enc2out->cmd_status);
			break;
		}
		return;
	}


	if ((enc2out->codec & (CODEC_H264 | CODEC_HEVC)) != 0) {
		hal_video_release(enc2out->ch, (enc2out->codec & (CODEC_H264 | CODEC_HEVC)), 0);
	}

	if ((enc2out->codec & CODEC_JPEG) != 0) {
		hal_video_release(enc2out->ch, (enc2out->codec & CODEC_JPEG), 0);
	}

	if ((enc2out->codec & (CODEC_NV12 | CODEC_RGB | CODEC_NV16)) != 0) {
		hal_video_isp_buf_release(enc2out->ch, (uint32_t)enc2out->isp_addr);
	}


	if (enc2out->finish == LAST_FRAME) {

	}

}

static void video_output_cb(void *param1, void  *param2, uint32_t arg)
{
	enc2out_t *enc2out = (enc2out_t *)param1;
	hal_video_adapter_t  *v_adp = (hal_video_adapter_t *)param2;
	int ch = enc2out->ch;
	voe_info.ch_info[ch].incb = 1;

	if (voe_info.ch_info[ch].video_output_cb) {
		voe_info.ch_info[ch].video_output_cb(param1, param2, arg);
	} else {
		temp_output_cb(param1, param2, arg);
	}

	if (enc2out->codec & CODEC_H264 || enc2out->codec & CODEC_HEVC) {
		video_rc_callback(ch, voe_info.ch_info[ch].bps_stbl_ctrl, enc2out->enc_len);
	} else if (enc2out->codec & CODEC_JPEG) {
		video_rc_callback(ch, voe_info.ch_info[ch].bps_stbl_ctrl, enc2out->jpg_len);
	}
	voe_info.ch_info[ch].incb = 0;
}

int isp_ctrl_cmd(int argc, char **argv)
{
	int id;
	int set_flag;
	int set_value;
	int read_value;
	int ret;
	if (argc >= 2) {
		set_flag = atoi(argv[0]);
		id = atoi(argv[1]);
		if (set_flag == 0) {
			ret = hal_video_isp_ctrl(id, set_flag, &read_value);
			if (ret == 0) {
				video_dprintf(VIDEO_LOG_INF, "result 0x%08x %d \r\n", read_value, read_value);
			} else {
				video_dprintf(VIDEO_LOG_ERR, "isp_ctrl get error\r\n");
			}
		} else {

			if (argc >= 3) {
				set_value = atoi(argv[2]);

				ret = hal_video_isp_ctrl(id, set_flag, &set_value);
				if (ret != 0) {
					video_dprintf(VIDEO_LOG_ERR, "isp_ctrl set error\r\n");
				}
			} else {
				video_dprintf(VIDEO_LOG_ERR, "isp_ctrl set error : need 3 argument: set_flag id  set_value\r\n");
			}
		}

	} else {
		video_dprintf(VIDEO_LOG_ERR, "isp_ctrl  error : need 2~3 argument: set_flag id  [set_value] \r\n");
	}
	return OK;
}

int iq_tuning_cmd(int argc, char **argv)
{
	int ccmd;
	uint32_t cmd_data;
	struct isp_tuning_cmd *iq_cmd;
	if (argc < 1) {	// usage
		video_dprintf(VIDEO_LOG_INF, "iqtun cmd\r\n");
		video_dprintf(VIDEO_LOG_INF, "      0: rts_isp_tuning_get_iq\n");
		video_dprintf(VIDEO_LOG_INF, "      1 : rts_isp_tuning_set_iq\r\n");
		video_dprintf(VIDEO_LOG_INF, "      2 : rts_isp_tuning_get_statis\r\n");
		video_dprintf(VIDEO_LOG_INF, "      3 : rts_isp_tuning_get_param\r\n");
		video_dprintf(VIDEO_LOG_INF, "      4 : rts_isp_tuning_set_param\r\n");
		video_dprintf(VIDEO_LOG_INF, "      5 offset lens : rts_isp_tuning_read_vreg \r\n");
		video_dprintf(VIDEO_LOG_INF, "      6 offset lens value1 value2: rts_isp_tuning_write_vreg\r\n");
		return NOK;
	}
	ccmd = atoi(argv[0]);

	cmd_data = (uint32_t)malloc(IQ_CMD_DATA_SIZE + 32); // for cache alignment
	if ((void *)cmd_data == NULL) {
		video_dprintf(VIDEO_LOG_ERR, "malloc cmd buf fail\r\n");
		return NOK;
	}
	iq_cmd = (struct isp_tuning_cmd *)((cmd_data + 31) & ~31); // for cache alignment

	if (ccmd == 0) {
		iq_cmd->addr = ISP_TUNING_IQ_TABLE_ALL;
		hal_video_isp_tuning(VOE_ISP_TUNING_GET_IQ, iq_cmd);
	} else if (ccmd == 1) {
		iq_cmd->addr = ISP_TUNING_IQ_TABLE_ALL;
		hal_video_isp_tuning(VOE_ISP_TUNING_SET_IQ, iq_cmd);
	} else if (ccmd == 2) {
		iq_cmd->addr = ISP_TUNING_STATIS_ALL;
		hal_video_isp_tuning(VOE_ISP_TUNING_GET_STATIS, iq_cmd);
	} else if (ccmd == 3) {
		iq_cmd->addr = ISP_TUNING_PARAM_ALL;
		hal_video_isp_tuning(VOE_ISP_TUNING_GET_PARAM, iq_cmd);
	} else if (ccmd == 4) {
		iq_cmd->addr = ISP_TUNING_PARAM_ALL;
		hal_video_isp_tuning(VOE_ISP_TUNING_SET_PARAM, iq_cmd);
	} else if (ccmd == 5) {

		iq_cmd->addr = atoi(argv[1]);
		iq_cmd->len = atoi(argv[2]);
		hal_video_isp_tuning(VOE_ISP_TUNING_READ_VREG, iq_cmd);
		uint32_t *r_data = (uint32_t *)iq_cmd->data;
		for (int i = 0; i < (iq_cmd->len / 4); i++) {
			video_dprintf(VIDEO_LOG_INF, "vreg 0x%08x = 0x%08x \n", iq_cmd->addr + i * 4, r_data[i]);
		}

	} else if (ccmd == 6) {
		if (argc < 4) {
			video_dprintf(VIDEO_LOG_ERR, "      6 offset lens value1 [value2]: rts_isp_tuning_write_vreg\r\n");
		} else {
			iq_cmd->addr = atoi(argv[1]);
			iq_cmd->len = atoi(argv[2]);

			uint32_t *w_data = (uint32_t *)iq_cmd->data;
			w_data[0] = atoi(argv[3]);
			if (argc > 4) {
				for (int i = 0; i < (argc - 4); i++) {
					w_data[i + 1] = atoi(argv[4 + i]);
				}
			}
			hal_video_isp_tuning(VOE_ISP_TUNING_WRITE_VREG, iq_cmd);
		}
	}

	if (cmd_data) {
		free((void *)cmd_data);
	}
	return OK;
}

int video_i2c_cmd(int argc, char **argv)
{
	int rd = 0;
	struct rts_isp_i2c_reg reg = {0};
	if (argc < 1) {	// usage
		video_dprintf(VIDEO_LOG_INF, "i2cs r/w reg data  \r\n");
		video_dprintf(VIDEO_LOG_INF, "      r/w: r/w function 0:write 1:read\n");
		video_dprintf(VIDEO_LOG_INF, "      reg  : sensor reg\r\n");
		video_dprintf(VIDEO_LOG_INF, "      data : sensor data\r\n");
		return 0;
	}
	if (argc > 0) {
		rd = atoi(argv[0]);
		reg.addr = atoi(argv[1]);
		reg.data = atoi(argv[2]);
		if (rd == 0) {
			hal_video_i2c_write(&reg);
		} else {
			hal_video_i2c_read(&reg);
		}
		video_dprintf(VIDEO_LOG_INF, "i2c reg[%x] = %x\n\r", reg.addr, reg.data);
	}
	return 0;
}

int video_encbuf_clean(int ch, int codec)
{
	int ret = 0;
	ret = hal_video_release(ch, codec, 1);
	return ret;
}

int video_encbuf_release(int ch, int codec, int mode)
{
	int ret = 0;
	//printf("hevc/h264/jpeg release = %d\r\n",mode);
	ret = hal_video_release(ch, codec, 0);
	if (ret != OK) {
		//retry release again
		ret = hal_video_release(ch, codec, 0);
	}
	return ret;
}

int video_ispbuf_release(int ch, int addr)
{
	int ret = 0;
	//printf("nv12/nv16/rgb release = 0x%X\r\n",addr);
	ret = hal_video_isp_buf_release(ch, addr);
	if (ret != OK) {
		//retry release again
		ret = hal_video_isp_buf_release(ch, addr);
	}
	return ret;
}

int video_ctrl(int ch, int cmd, int arg)
{
	int ret = 0;
	switch (cmd) {
	case VIDEO_SET_RCPARAM: {
		rate_ctrl_s rc_ctrl;
		encode_rc_parm_t *rc_param = (encode_rc_parm_t *)arg;
		memset(&rc_ctrl, 0x0, sizeof(rate_ctrl_s));

		if (rc_param->maxQp) {
			rc_ctrl.maxqp = rc_param->maxQp;
		}

		if (rc_param->minQp) {
			rc_ctrl.minqp = rc_param->minQp;
		}

		if (rc_param->maxIQp) {
			rc_ctrl.qpMaxI = rc_param->maxIQp;
		}

		if (rc_param->minIQp) {
			rc_ctrl.qpMinI = rc_param->minIQp;
		}

		ret = video_set_rc(ch, &rc_ctrl);
	}
	break;
	case VIDEO_FORCE_IFRAME: {
		ret = hal_video_force_i(ch);
	}
	break;
	case VIDEO_BPS: {
		rate_ctrl_s rc_ctrl;
		memset(&rc_ctrl, 0x0, sizeof(rate_ctrl_s));
		rc_ctrl.bps = arg;
		ret = video_set_rc(ch, &rc_ctrl);
	}
	break;
	case VIDEO_GOP: {
		rate_ctrl_s rc_ctrl;
		memset(&rc_ctrl, 0x0, sizeof(rate_ctrl_s));
		rc_ctrl.gop = arg;
		ret = video_set_rc(ch, &rc_ctrl);
	}
	break;
	case VIDEO_ISPFPS: {
		int video_type = voe_info.ch_info[ch].param->type;
		ret = hal_video_set_isp_stream_fps(ch, arg);
		if (ret == OK) {
			voe_info.ch_info[ch].isp_fps = arg;
		}
		if (video_type != VIDEO_NV12 && video_type != VIDEO_NV16 && video_type != VIDEO_RGB) {
			//sync isp fps to rc control
			rate_ctrl_s rc_ctrl;
			memset(&rc_ctrl, 0x0, sizeof(rate_ctrl_s));
			rc_ctrl.isp_fps = arg;
			video_set_rc(ch, &rc_ctrl);
		}
	}
	break;
	case VIDEO_FPS: {
		rate_ctrl_s rc_ctrl;
		memset(&rc_ctrl, 0x0, sizeof(rate_ctrl_s));
		rc_ctrl.fps = arg;
		ret = video_set_rc(ch, &rc_ctrl);
	}
	break;
	case VIDEO_RC_CTRL: {
		ret = video_set_rc(ch, (rate_ctrl_s *)arg);
	}
	break;
	case VIDEO_GET_RC_CTRL: {
		ret = video_get_rc(ch, (rate_ctrl_s *)arg);
	}
	break;
	//type 0:HEVC 1:H264 2:JPEG 3:NV12 4:RGB 5:NV16
	case VIDEO_HEVC_OUTPUT: {
		ret = hal_video_out_mode(ch, 0, arg);
	}
	break;
	case VIDEO_H264_OUTPUT: {
		ret = hal_video_out_mode(ch, 1, arg);
	}
	break;
	case VIDEO_JPEG_OUTPUT: {
		ret = hal_video_out_mode(ch, 2, arg);
	}
	break;
	case VIDEO_NV12_OUTPUT: {
		hal_video_out_mode(ch, 3, arg);
	}
	break;
	case VIDEO_RGB_OUTPUT: {
		ret = hal_video_out_mode(ch, 4, arg);
	}
	break;
	case VIDEO_NV16_OUTPUT: {
		ret = hal_video_out_mode(ch, 5, arg);
	}
	break;
	case VIDEO_ISP_SET_RAWFMT: {
		ret = hal_video_isp_set_rawfmt(ch, arg);
	}
	break;
	case VIDEO_PRINT_INFO: {
		ret = hal_video_mem_info();
		ret = hal_video_buf_info();
	}
	break;
	case VIDEO_DEBUG: {
		video_pre_init_param.voe_dbg_disable = ((arg == 1) ? 0 : 1);
		if (video_open_status() == 0) {
			ret = OK;
		} else {
			ret = hal_video_print(arg);
		}
	}
	break;
	}
	// Reset the time offset when the video output is stopped
	if (arg == 0 && cmd >= VIDEO_HEVC_OUTPUT && cmd <= VIDEO_NV16_OUTPUT) {
		last_system_ts_ms[ch] = 0;
		last_video_ts_ms[ch] = 0;
		offset_ms[ch] = 0;
		isp_overflow_times[ch] = 0;
		last_isp_ts[ch] = 0;
		isp_ts_initialed_flag[ch] = 0;
	}

	return ret;
}

static int video_bps_stbl_ctrl_deinit(int ch)
{
	if (voe_info.ch_info[ch].bps_stbl_ctrl) {
		free(voe_info.ch_info[ch].bps_stbl_ctrl);
		voe_info.ch_info[ch].bps_stbl_ctrl = NULL;
	}
	return OK;
}

static int video_bps_stbl_ctrl_init(int ch)
{
	voe_info.ch_info[ch].bps_stbl_ctrl = malloc(sizeof(bps_stbl_ctrl_t));
	if (voe_info.ch_info[ch].bps_stbl_ctrl == NULL) {
		video_dprintf(VIDEO_LOG_ERR, "ch%d failed to create bps_stbl_ctrl\r\n", ch);
		return NOK;
	}
	memset(voe_info.ch_info[ch].bps_stbl_ctrl, 0, sizeof(bps_stbl_ctrl_t));
	return OK;
}

int video_bps_stbl_ctrl_en(int ch, int enable)
{
	if (voe_info.ch_info[ch].bps_stbl_ctrl == NULL) {
		video_dprintf(VIDEO_LOG_ERR, "ch%d set bps stbl parameter before enable\r\n", ch);
		return NOK;
	}
	bps_stbl_ctrl_t *bps_stbl_ctrl = voe_info.ch_info[ch].bps_stbl_ctrl;
	if (enable) {
		bps_stbl_ctrl->en = 1;
		video_dprintf(VIDEO_LOG_INF, "fps stage %u, %u, %u\r\n", bps_stbl_ctrl->fps_stage[0], bps_stbl_ctrl->fps_stage[1], bps_stbl_ctrl->fps_stage[2]);
		video_dprintf(VIDEO_LOG_INF, "gop stage %u, %u, %u\r\n", bps_stbl_ctrl->gop_stage[0], bps_stbl_ctrl->gop_stage[1], bps_stbl_ctrl->gop_stage[2]);
	} else {
		bps_stbl_ctrl->en = 0;
	}
	return OK;
}

int video_set_bps_stbl_ctrl_params(int ch, bps_stbl_ctrl_param_t *bps_stbl_ctrl_param, uint32_t *fps_stage, uint32_t *gop_stage)
{
	if (voe_info.ch_info[ch].bps_stbl_ctrl == NULL) {
		if (video_bps_stbl_ctrl_init(ch) != OK) {
			video_dprintf(VIDEO_LOG_ERR, "ch%d failed to create bps_stbl_ctrl\r\n", ch);
			return NOK;
		}
	}
	bps_stbl_ctrl_t *bps_stbl_ctrl = voe_info.ch_info[ch].bps_stbl_ctrl;
	if (bps_stbl_ctrl_param) {
		int rcMode = voe_info.ch_info[ch].param->rc_mode - 1;
		if (rcMode) { //VBR
			bps_stbl_ctrl->params.minimum_bitrate = bps_stbl_ctrl_param->minimum_bitrate / 2;
			bps_stbl_ctrl->params.maximun_bitrate = bps_stbl_ctrl_param->maximun_bitrate / 2;
			bps_stbl_ctrl->params.target_bitrate = bps_stbl_ctrl_param->target_bitrate / 2;
			bps_stbl_ctrl->params.sampling_time = bps_stbl_ctrl_param->sampling_time;
		} else { //CBR
			memcpy(&(bps_stbl_ctrl->params), (void *)bps_stbl_ctrl_param, sizeof(bps_stbl_ctrl_param_t));
		}
		if ((bps_stbl_ctrl_param->minimum_bitrate == 0) || (bps_stbl_ctrl_param->minimum_bitrate >= bps_stbl_ctrl_param->maximun_bitrate)) {
			bps_stbl_ctrl_param->minimum_bitrate = bps_stbl_ctrl_param->target_bitrate;
		}
		bps_stbl_ctrl->stats_info.sum_sr = 0;
		bps_stbl_ctrl->stats_info.cnt_sr = 0;
	}
	if (fps_stage) {
		memcpy(bps_stbl_ctrl->fps_stage, fps_stage, sizeof(bps_stbl_ctrl->fps_stage));
	}
	if (gop_stage) {
		memcpy(bps_stbl_ctrl->gop_stage, gop_stage, sizeof(bps_stbl_ctrl->gop_stage));
	}
	return OK;
}

static int video_bps_stbl_ctrl_check_fps(int fps)
{
	int isp_max_fps = 0;
	isp_get_max_fps(&isp_max_fps);
	if (isp_max_fps > 0 && isp_max_fps < fps) {
		return isp_max_fps;
	}
	// Since the min fps of VOE is not related to the isp min fps, user only needs to prevent the fps is 0
	if (fps <= 0) {
		return 1;
	}
	return fps;
}

static void video_bps_stbl_ctrl_update_fps_gop(int ch, int fps, int gop)
{
	fps = video_bps_stbl_ctrl_check_fps(fps);
	voe_info.ch_info[ch].rc_info->temp_rc_ctrl.fps = fps;
	voe_info.ch_info[ch].rc_info->temp_rc_ctrl.gop = gop;
	atomic_store(&(voe_info.ch_info[ch].rc_info->update_flag), true);
}

static int video_bps_stbl_ctrl_get_fps_stage(bps_stbl_ctrl_t *bps_stbl_ctrl)
{
	int cur_fps = bps_stbl_ctrl->current_framerate;
	for (int i = 0; i < BPS_STBL_CTRL_STG_CNT; i++) {
		if (cur_fps == bps_stbl_ctrl->fps_stage[i]) {
			return i;
		}
	}
	return -1;
}

#define BPS_STBL_CTRL_DEBOUNCE	1
static void video_bps_stbl_ctrl_process(int ch, bps_stbl_ctrl_t *bps_stbl_ctrl, uint32_t frame_size)
{
	int cur_fps = voe_info.ch_info[ch].rc_info->rc_ctrl.fps;
	int fps = 0;
	int gop = 0;
	rate_ctrl_s rc_ctrl;
	bps_stbl_ctrl_param_t *bps_stbl_ctrl_param = &(bps_stbl_ctrl->params);

	if (frame_size == 0) {
		return;
	}
	bps_stbl_ctrl->current_framerate = cur_fps;
	//calculate sample bitrate
	bps_stbl_ctrl->stats_info.cnt_sr++;
	bps_stbl_ctrl->stats_info.sum_sr += frame_size;
	uint32_t sampling_frame_count = bps_stbl_ctrl->params.sampling_time * cur_fps / 1000;
	if (bps_stbl_ctrl->stats_info.cnt_sr < sampling_frame_count) {
		return;
	}
	bps_stbl_ctrl->sample_bitrate = bps_stbl_ctrl->stats_info.sum_sr * 8 / bps_stbl_ctrl->stats_info.cnt_sr * bps_stbl_ctrl->current_framerate;
	bps_stbl_ctrl->stats_info.sum_sr = 0;
	bps_stbl_ctrl->stats_info.cnt_sr = 0;

	//check if fps not manually change
	if (atomic_load(&voe_info.ch_info[ch].rc_info->update_flag) && (voe_info.ch_info[ch].rc_info->temp_rc_ctrl.fps != voe_info.ch_info[ch].rc_info->rc_ctrl.fps)) {
		video_dprintf(VIDEO_LOG_INF, "[%s] ch%d fps gop manual change, skip bps stable control\r\n", __FUNCTION__, ch);
		return;
	}

	//get fps stage index
	bps_stbl_ctrl->fps_stage_idx = video_bps_stbl_ctrl_get_fps_stage(bps_stbl_ctrl);
	if (bps_stbl_ctrl->fps_stage_idx < 0) {
		video_dprintf(VIDEO_LOG_INF, "[%s] ch%d get fps stage fail %d\r\n", __FUNCTION__, ch, cur_fps);
		return;
	}

	if (bps_stbl_ctrl->sample_bitrate > bps_stbl_ctrl_param->maximun_bitrate) {
		//if not min fps (fps stage 0~1), cut fps, raise fps stage
		if ((0 <= bps_stbl_ctrl->fps_stage_idx) && (bps_stbl_ctrl->fps_stage_idx <= (BPS_STBL_CTRL_STG_CNT - 2))) {
			bps_stbl_ctrl->switch_fps_down++;
			if (bps_stbl_ctrl->switch_fps_down >= BPS_STBL_CTRL_DEBOUNCE) {
				bps_stbl_ctrl->fps_stage_idx++;
				fps = (int)(bps_stbl_ctrl->fps_stage[bps_stbl_ctrl->fps_stage_idx]);
				gop = (int)(bps_stbl_ctrl->gop_stage[bps_stbl_ctrl->fps_stage_idx]);
				video_bps_stbl_ctrl_update_fps_gop(ch, fps, gop);
				video_dprintf(VIDEO_LOG_INF, "\r\nch = %d sample rate = %ld	maximun bitrate = %ld fps = %d gop = %d\r\n",
							  ch, bps_stbl_ctrl->sample_bitrate, bps_stbl_ctrl_param->maximun_bitrate, fps, gop);
				bps_stbl_ctrl->switch_fps_down = 0;
			}
		}
	} else if (bps_stbl_ctrl->sample_bitrate < bps_stbl_ctrl_param->minimum_bitrate) {
		//if not max fps (fps stage 1~2), add fps, lower fps stage
		if ((1 <= bps_stbl_ctrl->fps_stage_idx) && (bps_stbl_ctrl->fps_stage_idx <= (BPS_STBL_CTRL_STG_CNT - 1))) {
			bps_stbl_ctrl->switch_fps_up++;
			if (bps_stbl_ctrl->switch_fps_up >= BPS_STBL_CTRL_DEBOUNCE) {
				bps_stbl_ctrl->fps_stage_idx--;
				fps = (int)(bps_stbl_ctrl->fps_stage[bps_stbl_ctrl->fps_stage_idx]);
				gop = (int)(bps_stbl_ctrl->gop_stage[bps_stbl_ctrl->fps_stage_idx]);
				video_bps_stbl_ctrl_update_fps_gop(ch, fps, gop);
				video_dprintf(VIDEO_LOG_INF, "\r\nch = %d sample rate = %ld	minimum bitrate = %ld fps = %d gop = %d\r\n",
							  ch, bps_stbl_ctrl->sample_bitrate, bps_stbl_ctrl_param->minimum_bitrate, fps, gop);
				bps_stbl_ctrl->switch_fps_up = 0;
			}
		}
	} else {
		bps_stbl_ctrl->switch_fps_up = 0;
		bps_stbl_ctrl->switch_fps_down = 0;
	}
}

static int video_rc_update_param(int ch, rate_ctrl_s *dis_rc, rate_ctrl_s *src_rc)
{
	if (src_rc->isp_fps) {
		dis_rc->isp_fps = src_rc->isp_fps;
	}
	if (src_rc->fps) {
		dis_rc->fps = src_rc->fps;
	}
	if (src_rc->gop) {
		dis_rc->gop = src_rc->gop;
	}
	if (src_rc->bps) {
		dis_rc->bps = src_rc->bps;
	}
	if (src_rc->maxqp) {
		dis_rc->maxqp = src_rc->maxqp;
	}
	if (src_rc->minqp) {
		dis_rc->minqp = src_rc->minqp;
	}
	if (src_rc->fixedIntraQp) {
		dis_rc->fixedIntraQp = src_rc->fixedIntraQp;
	}
	if (src_rc->intraQpDelta) {
		dis_rc->intraQpDelta = src_rc->intraQpDelta;
	}
	if (src_rc->qpMaxI) {
		dis_rc->qpMaxI = src_rc->qpMaxI;
	}
	if (src_rc->qpMinI) {
		dis_rc->qpMinI = src_rc->qpMinI;
	}
	if (src_rc->forcei) {
		dis_rc->forcei = src_rc->forcei;
	}
	return OK;
}

static int video_rc_init(video_params_t *video_param)
{
	static char rc_version[] = "RC_v1";
	video_dprintf(VIDEO_LOG_MSG, "rc_version %s\n", rc_version);
	int ch = video_param->stream_id;
	if (!voe_info.ch_info[ch].rc_info) {
		voe_info.ch_info[ch].rc_info = malloc(sizeof(video_rc_info_t));
		if (!voe_info.ch_info[ch].rc_info) {
			video_dprintf(VIDEO_LOG_ERR, "rc_info malloc fail\n");
			return NOK;
		}
		memset(voe_info.ch_info[ch].rc_info, 0, sizeof(video_rc_info_t));
		atomic_store(&(voe_info.ch_info[ch].rc_info->update_flag), false);
		voe_info.ch_info[ch].rc_info->rc_ctrl.isp_fps = voe_info.ch_info[ch].isp_fps;
		voe_info.ch_info[ch].rc_info->rc_ctrl.fps = video_param->fps;
		if (video_param->rc_mode == 2) { //VBR
			voe_info.ch_info[ch].rc_info->rc_ctrl.bps = video_param->bps / 2;
		} else { //CBR
			voe_info.ch_info[ch].rc_info->rc_ctrl.bps = video_param->bps;
		}
		voe_info.ch_info[ch].rc_info->rc_ctrl.maxqp = video_param->maxQp;
		voe_info.ch_info[ch].rc_info->rc_ctrl.minqp = video_param->minQp;
		voe_info.ch_info[ch].rc_info->rc_ctrl.gop = video_param->gop;
	}
	return OK;
}

static int video_rc_deinit(int ch)
{
	if (voe_info.ch_info[ch].rc_info) {
		free(voe_info.ch_info[ch].rc_info);
		voe_info.ch_info[ch].rc_info = NULL;
	}
	return OK;
}

static void video_rc_callback(int ch, bps_stbl_ctrl_t *bps_stbl_ctrl, uint32_t frame_size)
{
	if (voe_info.ch_info[ch].rc_info == NULL) {
		return;
	}
	int cur_fps = voe_info.ch_info[ch].rc_info->rc_ctrl.fps;

	//bps stable control
	if (bps_stbl_ctrl && bps_stbl_ctrl->en) {
		video_bps_stbl_ctrl_process(ch, bps_stbl_ctrl, frame_size);
	}

	uint64_t rc_update_duration = (hal_read_systime_us() - voe_info.ch_info[ch].rc_info->update_time) / 1000;
	if ((rc_update_duration > (1000 / cur_fps)) && atomic_exchange(&(voe_info.ch_info[ch].rc_info->update_flag), false)) {
		if (voe_info.ch_info[ch].rc_info->temp_rc_ctrl.isp_fps != 0 &&
			(voe_info.ch_info[ch].rc_info->temp_rc_ctrl.fps > voe_info.ch_info[ch].rc_info->temp_rc_ctrl.isp_fps)) {
			video_dprintf(VIDEO_LOG_MSG, "[%s] invalid fps(%d) > ispfps(%d)\r\n", __FUNCTION__, voe_info.ch_info[ch].rc_info->temp_rc_ctrl.fps,
						  voe_info.ch_info[ch].rc_info->temp_rc_ctrl.isp_fps);
			voe_info.ch_info[ch].rc_info->temp_rc_ctrl.fps = 0;
		}
		// if update_flag = ture, update rc parameters and set update_flag to false
		int ret = hal_video_set_rc(&(voe_info.ch_info[ch].rc_info->temp_rc_ctrl), ch);
		if (ret == OK) {
			voe_info.ch_info[ch].rc_info->update_time = hal_read_systime_us();
			// update the rc info
			hal_video_set_fps(voe_info.ch_info[ch].rc_info->temp_rc_ctrl.fps, ch);
			video_rc_update_param(ch, &(voe_info.ch_info[ch].rc_info->rc_ctrl), &(voe_info.ch_info[ch].rc_info->temp_rc_ctrl));
			video_dprintf(VIDEO_LOG_INF, "[%s] ch %d update rc fps = %d, isp_fps = %d, gop = %d, bps = %d\r\n", __FUNCTION__, ch, voe_info.ch_info[ch].rc_info->rc_ctrl.fps,
						  voe_info.ch_info[ch].rc_info->rc_ctrl.isp_fps, voe_info.ch_info[ch].rc_info->rc_ctrl.gop, voe_info.ch_info[ch].rc_info->rc_ctrl.bps);
		} else {
			//if set rc fail
			video_dprintf(VIDEO_LOG_ERR, "ch%d rc fail:isp_fps%d,fps%d,gop%d,bps%d,maxqp%d,minqp%d\r\n", ch, voe_info.ch_info[ch].rc_info->temp_rc_ctrl.isp_fps,
						  voe_info.ch_info[ch].rc_info->temp_rc_ctrl.fps, voe_info.ch_info[ch].rc_info->temp_rc_ctrl.gop, voe_info.ch_info[ch].rc_info->temp_rc_ctrl.bps,
						  voe_info.ch_info[ch].rc_info->temp_rc_ctrl.maxqp, voe_info.ch_info[ch].rc_info->temp_rc_ctrl.minqp);
			memset(&(voe_info.ch_info[ch].rc_info->temp_rc_ctrl), 0, sizeof(rate_ctrl_s));
		}
		voe_info.ch_info[ch].rc_info->update_status = ret;
	}
}

int video_get_rc(int ch, rate_ctrl_s *rc_ctrl)
{
	if (voe_info.ch_info[ch].rc_info == NULL) {
		video_dprintf(VIDEO_LOG_ERR, "[%s] ch%d rc_info not init\r\n", __FUNCTION__, ch);
		return NOK;
	}
	memcpy(rc_ctrl, &(voe_info.ch_info[ch].rc_info->rc_ctrl), sizeof(rate_ctrl_s));
	return OK;
}

int video_set_rc(int ch, rate_ctrl_s *rc_ctrl)
{
	int ret = OK;
	if (voe_info.ch_info[ch].rc_info == NULL) {
		video_dprintf(VIDEO_LOG_ERR, "[%s] ch%d rc_info not init\r\n", __FUNCTION__, ch);
		return NOK;
	}

	video_rc_update_param(ch, &(voe_info.ch_info[ch].rc_info->temp_rc_ctrl), rc_ctrl);
	if (rc_ctrl->bps) {
		int rcMode = voe_info.ch_info[ch].param->rc_mode - 1;
		int bps;
		if (rcMode) { //VBR
			bps = rc_ctrl->bps / 2;
		} else { //CBR
			bps = rc_ctrl->bps;
		}
		if (bps >= 10000) { //encoder HW limit
			voe_info.ch_info[ch].rc_info->temp_rc_ctrl.bps = bps;
		} else {
			video_dprintf(VIDEO_LOG_MSG, "[%s] ch%d invlid bps setting\r\n", __FUNCTION__, ch);
		}
	}
	//video_dprintf(VIDEO_LOG_MSG, "[%s] rc fps = %d, isp_fps = %d, gop = %d, bps = %d\r\n", __FUNCTION__, voe_info.ch_info[ch].rc_info->temp_rc_ctrl.fps,
	//		voe_info.ch_info[ch].rc_info->temp_rc_ctrl.isp_fps, voe_info.ch_info[ch].rc_info->temp_rc_ctrl.gop, voe_info.ch_info[ch].rc_info->temp_rc_ctrl.bps);
	atomic_store(&(voe_info.ch_info[ch].rc_info->update_flag), true);
	return OK;
}

int video_get_realfps(int ch, int *isp_fps, int *enc_fps)
{
	int ispfpsx100, encfpsx100;
	int ret = hal_video_get_realfps(ch, &ispfpsx100, &encfpsx100); //FPS are multiplied with 100
	if (ret == OK) {
		*isp_fps = ispfpsx100 / 100;
		*enc_fps = encfpsx100 / 100;
	}
	return ret;
}

int video_wait_target_fps(int ch, int target_fps, int timeout)
{
	int ispfps, encfps, wait_time = 0, delay = 67;
	while (wait_time < timeout) {
		video_get_realfps(ch, &ispfps, &encfps);
		if (ispfps == target_fps) {
			return OK;
		}
		vTaskDelay(delay);
		wait_time += delay;
	}
	video_dprintf(VIDEO_LOG_MSG, "[%s] ch%d wait target fps fail\r\n", __FUNCTION__, ch);
	return NOK;
}

int video_set_roi_region(int ch, int x, int y, int width, int height, int value)
{
	int result = 0;
	result = hal_video_roi_region(ch, x, y, width, height, value);
	return result;
}

void video_set_isp_info(isp_info_t *info)
{
	isp_info.sensor_width = info->sensor_width;
	isp_info.sensor_height = info->sensor_height;
	isp_info.sensor_fps = info->sensor_fps;
	isp_info.osd_enable = info->osd_enable;
	isp_info.md_enable = info->md_enable;
	isp_info.hdr_enable = info->hdr_enable;
}

int video_set_voe_heap(int heap_addr, int heap_size, int use_malloc)
{
#if 0
	if (use_malloc) {
		voe_info.voe_heap_addr = malloc(heap_size);
	} else {
		voe_info.voe_heap_addr = heap_addr;
	}

	if (voe_info.voe_heap_addr) {
		voe_info.voe_heap_size = heap_size;
		return 0;
	} else {
		video_dprintf(VIDEO_LOG_ERR, "malloc voe buffer fail\r\n");
		return -1;
	}
#else
	voe_info.voe_heap_size = heap_size;
	return 0;
#endif
}

static int video_get_codec_type(int video_type)
{
	int codec = 0;
	switch (video_type) {
	case VIDEO_HEVC:
		codec = CODEC_HEVC;
		break;
	case VIDEO_H264:
		codec = CODEC_H264;
		break;
	case VIDEO_JPEG:
		codec = CODEC_JPEG;
		break;
	case VIDEO_NV12:
		codec = CODEC_NV12;
		break;
	case VIDEO_RGB:
		codec = CODEC_RGB;
		break;
	case VIDEO_NV16:
		codec = CODEC_NV16;
		break;
	case VIDEO_HEVC_JPEG:
		codec = CODEC_HEVC | CODEC_JPEG;
		break;
	case VIDEO_H264_JPEG:
		codec = CODEC_H264 | CODEC_JPEG;
		break;
	}
	return codec;
}

static int video_alighment(int num, int align)
{
	return ((num + align - 1) / align) * align;
}

int video_buf_calc(int v1_enable, int v1_w, int v1_h, int v1_bps, int v1_shapshot,
				   int v2_enable, int v2_w, int v2_h, int v2_bps, int v2_shapshot,
				   int v3_enable, int v3_w, int v3_h, int v3_bps, int v3_shapshot,
				   int v4_enable, int v4_w, int v4_h)
{
	int voe_heap_size = 0;
	int v3dnr_w = 2560;
	int v3dnr_h = 1440;

	if (isp_info.sensor_width && isp_info.sensor_height) {
		v3dnr_w = isp_info.sensor_width;
		v3dnr_h = isp_info.sensor_height;
	}

	//3dnr
	voe_heap_size += ((v3dnr_w * v3dnr_h * 3) / 2);
	video_dprintf(VIDEO_LOG_INF, "3dnr = %d,%d,%d\r\n", v3dnr_w, v3dnr_h, voe_heap_size);

	if (isp_info.md_enable) {
		if (isp_info.md_buf_size) {
			voe_heap_size += isp_info.md_buf_size;
		} else {
			voe_heap_size += ENABLE_MD_BUF;
		}
	}

	if (isp_info.hdr_enable) {
		voe_heap_size += ENABLE_HDR_BUF;
	}

	if (v1_enable) {
		//ISP buffer
		voe_heap_size += ((v1_w * v1_h * 3) / 2) * isp_ch_buf_num[0];
		//ISP common
		voe_heap_size += ISP_CREATE_BUF;
		//enc ref
		voe_heap_size += ((v1_w * v1_h * 3) / 2) * 2 + (v1_w * v1_h / 16) * 2;
		//enc common
		voe_heap_size += ENC_CREATE_BUF;
		//enc buffer
		voe_heap_size += ((v1_w * v1_h) / VIDEO_RSVD_DIVISION + (v1_bps * V1_ENC_BUF_SIZE) / 8);
		g_enc_buff_size[0] = ((v1_w * v1_h) / VIDEO_RSVD_DIVISION + (v1_bps * V1_ENC_BUF_SIZE) / 8);
		//shapshot
		if (v1_shapshot) {
			voe_heap_size += ((v1_w * v1_h * 3) / 2) + SNAPSHOT_BUF;
		}
		//osd common
		if (isp_info.osd_enable) {
			voe_heap_size += OSD_CREATE_BUF;
		}
	}

	video_dprintf(VIDEO_LOG_INF, "v1 = %d,%d,%d\r\n", v1_w, v1_h, voe_heap_size);

	if (v2_enable) {
		//ISP buffer
		voe_heap_size += ((v2_w * v2_h * 3) / 2) * isp_ch_buf_num[1];
		//ISP common
		voe_heap_size += ISP_CREATE_BUF;
		//enc ref
		voe_heap_size += ((v2_w * v2_h * 3) / 2) * 2 + (v2_w * v2_h / 16) * 2;
		//enc common
		voe_heap_size += ENC_CREATE_BUF;
		//enc buffer
		voe_heap_size += ((v2_w * v2_h) / VIDEO_RSVD_DIVISION + (v2_bps * V2_ENC_BUF_SIZE) / 8);
		g_enc_buff_size[1] = ((v2_w * v2_h) / VIDEO_RSVD_DIVISION + (v2_bps * V2_ENC_BUF_SIZE) / 8);
		//shapshot
		if (v2_shapshot) {
			voe_heap_size += ((v2_w * v2_h * 3) / 2) + SNAPSHOT_BUF;
		}
		//osd common
		if (isp_info.osd_enable) {
			voe_heap_size += OSD_CREATE_BUF;
		}
	}

	video_dprintf(VIDEO_LOG_INF, "v2 = %d,%d,%d\r\n", v2_w, v2_h, voe_heap_size);

	if (v3_enable) {
		//ISP buffer
		voe_heap_size += ((v3_w * v3_h * 3) / 2) * isp_ch_buf_num[2];
		//ISP common
		voe_heap_size += ISP_CREATE_BUF;
		//enc ref
		voe_heap_size += ((v3_w * v3_h * 3) / 2) * 2 + (v3_w * v3_h / 16) * 2;
		//enc common
		voe_heap_size += ENC_CREATE_BUF;
		//enc buffer
		voe_heap_size += ((v3_w * v3_h) / VIDEO_RSVD_DIVISION + (v3_bps * V3_ENC_BUF_SIZE) / 8);
		g_enc_buff_size[2] = ((v3_w * v3_h) / VIDEO_RSVD_DIVISION + (v3_bps * V3_ENC_BUF_SIZE) / 8);
		//shapshot
		if (v3_shapshot) {
			voe_heap_size += ((v3_w * v3_h * 3) / 2) + SNAPSHOT_BUF;
		}
		//osd common
		if (isp_info.osd_enable) {
			voe_heap_size += OSD_CREATE_BUF;
		}
	}

	video_dprintf(VIDEO_LOG_INF, "v3 = %d,%d,%d\r\n", v3_w, v3_h, voe_heap_size);

	if (v4_enable) {
		//ISP buffer
		voe_heap_size += v4_w * v4_h * 3 * isp_ch_buf_num[4];
		//ISP common
		voe_heap_size += ISP_CREATE_BUF;
	}

	video_dprintf(VIDEO_LOG_INF, "v4 = %d,%d,%d\r\n", v4_w, v4_h, voe_heap_size);

	video_set_voe_heap((int)NULL, voe_heap_size, 1);

	return voe_heap_size;
}

static int video_nv12_channel_heapsize(int ch_id, int ch_codec, int ch_w, int ch_h, int ch_bps, int ch_buf_size, int ch_jpg_only_shapshot, int *isp_main,
									   int *enc_main, int *jpg_main, int *sw_margin)
{
	int ch_heap_size = 0;
	//ISP buffer
	ch_heap_size += ((ch_w * ch_h * 3) / 2) * isp_ch_buf_num[ch_id];
	//ISP common
	if (*isp_main) {
		ch_heap_size += ISP_COMMON_MAIN_BUF;
		*sw_margin -= ISP_COMMON_MAIN_SW_MARGIN;
		*isp_main = 0;
	} else {
		ch_heap_size += ISP_COMMON_EXTRA_BUF;
		*sw_margin -= ISP_COMMON_EXTRA_SW_MARGIN;
	}

	if (ch_codec & (CODEC_HEVC | CODEC_H264)) {
		//ENC ref
		//Count the align 64 of width and height
		uint32_t ch_w_64 = video_alighment(ch_w, 64);
		uint32_t ch_h_64 = video_alighment(ch_h, 64);
		uint32_t ch_enc_ref = 0;
		ch_enc_ref += video_alighment(ch_w_64 * ch_h_64 + ch_w_64 * ch_h_64 / 16 + 16, 4096) + 4096;
		ch_enc_ref += video_alighment(ch_w_64 * ch_h_64 / 2, 4096) + 4096;
		ch_enc_ref = ch_enc_ref * 2;
		ch_heap_size += ch_enc_ref;
		//HEVC or H264 ENC common
		if (*enc_main) {
			ch_heap_size += ENC_COMMON_MAIN_BUF;
			*sw_margin -= ENC_COMMON_MAIN_SW_MARGIN;
			*enc_main = 0;
		} else {
			ch_heap_size += ENC_COMMON_EXTRA_BUF;
			*sw_margin -= ENC_COMMON_EXTRA_SW_MARGIN;
		}
		//HEVC or H264 ENC output buffer
		g_enc_buff_size[ch_id] = ((ch_w * ch_h) / VIDEO_RSVD_DIVISION + (ch_bps * ch_buf_size) / 8);
		ch_heap_size += g_enc_buff_size[ch_id];
		//HEVC or H264 ENC ROI buffer
		//roiMapDeltaQpEnable == 1 needed
		uint32_t ch_w_16 = video_alighment(ch_w, 16);
		uint32_t ch_h_16 = video_alighment(ch_h, 16);
		uint32_t ch_roiMapDeltaQpUnit = 0;
		uint32_t ch_roiMapDeltaQpParameter = 1;
		for (int i = 1; i <= ch_roiMapDeltaQpUnit; i++) {
			ch_roiMapDeltaQpParameter *= 2;
		}
		ch_heap_size += (ch_w_16 * ch_h_16 * ((64 / ch_roiMapDeltaQpParameter) + 1)) / ((64 / ch_roiMapDeltaQpParameter) * (64 / ch_roiMapDeltaQpParameter)) + 1536;
	}

	if (ch_codec & CODEC_JPEG) {
		//JPG ENC common
		if (*jpg_main) {
			ch_heap_size += JPG_COMMON_MAIN_BUF;
			*jpg_main = 0;
		} else {
			ch_heap_size += JPG_COMMON_EXTRA_BUF;
		}
		//JPG ENC output buffer
		if (ch_jpg_only_shapshot) {
			//JPEG for snapshot
			ch_heap_size += ((ch_w * ch_h * 3) / 2) + SNAPSHOT_BUF;
		} else {
			//JPEG for streaming
			ch_heap_size += ((ch_w * ch_h * 3) / 2) * JPG_BUF_SIZE;
		}
	}

	//osd common
	if (isp_info.osd_enable) {
		ch_heap_size += OSD_CREATE_BUF;
	}
	video_dprintf(VIDEO_LOG_INF, "v%d = %d,%d,%d\r\n", ch_id + 1, ch_w, ch_h, ch_heap_size);
	return ch_heap_size;
}

static int video_rgb_channel_heapsize(int ch_id, int ch_w, int ch_h, int *isp_main, int *sw_margin)
{
	int ch_heap_size = 0;
	//ISP buffer
	//RGB (channel for only support RGB)
	ch_heap_size += ch_w * ch_h * 3 * isp_ch_buf_num[ch_id];
	//ISP common
	if (*isp_main) {
		ch_heap_size += ISP_COMMON_MAIN_BUF;
		*sw_margin -= ISP_COMMON_MAIN_SW_MARGIN;
		*isp_main = 0;
	} else {
		ch_heap_size += ISP_COMMON_EXTRA_BUF;
		*sw_margin -= ISP_COMMON_EXTRA_SW_MARGIN;
	}
	video_dprintf(VIDEO_LOG_INF, "v%d = %d,%d,%d\r\n", ch_id + 1, ch_w, ch_h, ch_heap_size);
	return ch_heap_size;
}

int video_buf_heap_calc(int v1_enable, int v1_w, int v1_h, int v1_bps, int v1_enctype, int v1_jpg_only_shapshot,
						int v2_enable, int v2_w, int v2_h, int v2_bps, int v2_enctype, int v2_jpg_only_shapshot,
						int v3_enable, int v3_w, int v3_h, int v3_bps, int v3_enctype, int v3_jpg_only_shapshot,
						int v4_enable, int v4_w, int v4_h)
{
	//ISP Common: only the first channel need 800KiB, others 100 KiB
	int voe_heap_size = 0;
	int v3dnr_w = 2560;
	int v3dnr_h = 1440;
	int isp_main = 1;
	int enc_main = 1;
	int jpg_main = 1;
	int sw_margin = 2 * 1024 * 1024;

	if (isp_info.sensor_width && isp_info.sensor_height) {
		v3dnr_w = isp_info.sensor_width;
		v3dnr_h = isp_info.sensor_height;
	}

	//3dnr
	voe_heap_size += ((v3dnr_w * v3dnr_h * 3) / 2);
	video_dprintf(VIDEO_LOG_INF, "3dnr = %d,%d,%d\r\n", v3dnr_w, v3dnr_h, voe_heap_size);

	//MD, HDR margin use the same buffer
	if (isp_info.md_enable || isp_info.hdr_enable) {
		if (isp_info.md_buf_size > ENABLE_HDR_MD_BUF) {
			voe_heap_size += isp_info.md_buf_size;
		} else {
			voe_heap_size += ENABLE_HDR_MD_BUF;
		}
	}

	if (v1_enable) {
		int v1_codec = video_get_codec_type(v1_enctype);
		voe_heap_size += video_nv12_channel_heapsize(0, v1_codec, v1_w, v1_h, v1_bps, V1_ENC_BUF_SIZE, v1_jpg_only_shapshot, &isp_main, &enc_main, &jpg_main,
						 &sw_margin);
	}

	if (v2_enable) {
		int v2_codec = video_get_codec_type(v2_enctype);
		voe_heap_size += video_nv12_channel_heapsize(1, v2_codec, v2_w, v2_h, v2_bps, V2_ENC_BUF_SIZE, v2_jpg_only_shapshot, &isp_main, &enc_main, &jpg_main,
						 &sw_margin);
	}

	if (v3_enable) {
		int v3_codec = video_get_codec_type(v3_enctype);
		voe_heap_size += video_nv12_channel_heapsize(2, v3_codec, v3_w, v3_h, v3_bps, V3_ENC_BUF_SIZE, v3_jpg_only_shapshot, &isp_main, &enc_main, &jpg_main,
						 &sw_margin);
	}

	if (v4_enable) {
		voe_heap_size += video_rgb_channel_heapsize(4, v4_w, v4_h, &isp_main, &sw_margin);
	}

	// Add software margin
	if (sw_margin > 0) {
		voe_heap_size += sw_margin;
	}

	video_set_voe_heap((int)NULL, voe_heap_size, 1);

	return voe_heap_size;
}

int video_extra_buf_calc(int originl_heapsize, int vext_enable, int vext_w, int vext_h, int vext_bps, int vext_shapshot)
{
	int voe_heap_size = originl_heapsize;
	if (vext_enable) {
		//ISP buffer
		voe_heap_size += ((vext_w * vext_h * 3) / 2) * isp_ch_buf_num[3];
		//ISP common
		voe_heap_size += ISP_CREATE_BUF;
		//enc ref
		voe_heap_size += ((vext_w * vext_h * 3) / 2) * 2 + (vext_w * vext_h / 16) * 2;
		//enc common
		voe_heap_size += ENC_CREATE_BUF;
		//enc buffer (may need modify by codec type)
		voe_heap_size += ((vext_w * vext_h) / VIDEO_RSVD_DIVISION + (vext_bps * VEXT_ENC_BUF_SIZE) / 8);
		g_enc_buff_size[3] = ((vext_w * vext_h) / VIDEO_RSVD_DIVISION + (vext_bps * VEXT_ENC_BUF_SIZE) / 8);
		//snapshot
		if (vext_shapshot) {
			voe_heap_size += ((vext_w * vext_h * 3) / 2) + SNAPSHOT_BUF;
		}
		//osd common
		if (isp_info.osd_enable) {
			voe_heap_size += OSD_CREATE_BUF;
		}
	}

	video_dprintf(VIDEO_LOG_INF, "vext = %d,%d,%d\r\n", vext_w, vext_h, voe_heap_size);

	video_set_voe_heap((int)NULL, voe_heap_size, 1);

	return voe_heap_size;
}

void video_buf_release(void)
{
	if (video_open_status() != 0) {
		video_dprintf(VIDEO_LOG_ERR, "[%s] video not closed\n", __FUNCTION__);
		return;
	}
	if ((void *)(voe_info.voe_heap_addr) != NULL) {
#if VIDEO_MPU_VOE_HEAP
		setup_mpu((uint32_t)voe_info.voe_heap_addr, (uint32_t)voe_info.voe_heap_addr + voe_info.voe_heap_size - 1, 1, 0);
#endif
		free((void *)voe_info.voe_heap_addr);
		voe_info.voe_heap_addr = (uint32_t)NULL;
	}
}

void video_init_peri(void)
{
	int peri_update_with_fcs = 0;


	volatile hal_i2c_adapter_t  i2c_master_video;


	g_video_peri_info.scl_pin = PIN_D12;
	g_video_peri_info.sda_pin = PIN_D10;
	g_video_peri_info.i2c_id = 3;

	g_video_peri_info.rst_pin = PIN_E0;
	g_video_peri_info.pwdn_pin = PIN_D11;
	g_video_peri_info.pwr_ctrl_pin = PIN_A5;
	g_video_peri_info.snr_clk_pin = PIN_D13;

	peri_update_with_fcs = hal_video_peri_update_with_fcs((video_peri_info_t *)&g_video_peri_info);
	fcs_peri_info_ram_t pfcs_peri_info;
	pfcs_peri_info.fcs_peri_valid = 1;
	hal_video_get_fcs_peri_info(&pfcs_peri_info);
	fcs_version = pfcs_peri_info.fcs_data_id;

	if (!hal_video_check_fcs_OK()) {

		if (peri_update_with_fcs) {
			hal_pinmux_unregister(g_video_peri_info.pwr_ctrl_pin, PID_GPIO); //sensor clock
			hal_pinmux_unregister(g_video_peri_info.rst_pin, PID_GPIO); //reset pin
			hal_pinmux_unregister(g_video_peri_info.pwdn_pin, PID_GPIO); //power down pin
			hal_pinmux_unregister(g_video_peri_info.snr_clk_pin, PID_SENSOR); //sensor clock

			i2c_master_video.pltf_dat.scl_pin = g_video_peri_info.scl_pin;
			i2c_master_video.pltf_dat.sda_pin = g_video_peri_info.sda_pin;
			i2c_master_video.init_dat.index = g_video_peri_info.i2c_id;
			hal_i2c_pin_unregister_simple(&i2c_master_video);

			//Disable the sensor power
			video_dprintf(VIDEO_LOG_INF, "unregister sensor pwr 0x%02x \n", g_video_peri_info.pwr_ctrl_pin);
			hal_gpio_init(&sensor_en_gpio, g_video_peri_info.pwr_ctrl_pin);
			hal_gpio_set_dir(&sensor_en_gpio, GPIO_OUT);

			hal_gpio_write(&sensor_en_gpio, 0);
			hal_gpio_deinit(&sensor_en_gpio);
			vTaskDelay(50);
		}


		if (IS_CUT_TEST(hal_sys_get_rom_ver())) {
			g_video_peri_info.pwr_ctrl_pin = PIN_A5;
			g_video_peri_info.scl_pin	= PIN_D14;
			g_video_peri_info.sda_pin	= PIN_D12;
			g_video_peri_info.rst_pin	= PIN_D13;
			g_video_peri_info.pwdn_pin = PIN_D11;
			g_video_peri_info.snr_clk_pin = PIN_D10;
			g_video_peri_info.i2c_id = 3;
		} else {
			g_video_peri_info.pwr_ctrl_pin = PIN_A5;
			g_video_peri_info.scl_pin	= PIN_D12;
			g_video_peri_info.sda_pin	= PIN_D10;
			g_video_peri_info.rst_pin	= PIN_E0;
			g_video_peri_info.pwdn_pin = PIN_D11;
			g_video_peri_info.snr_clk_pin = PIN_D13;
			g_video_peri_info.i2c_id = 3;
		}

		// Enable Sensor PWR
		hal_gpio_init(&sensor_en_gpio, g_video_peri_info.pwr_ctrl_pin);
		hal_gpio_set_dir(&sensor_en_gpio, GPIO_OUT);
		hal_gpio_write(&sensor_en_gpio, 1);
		video_dprintf(VIDEO_LOG_INF, "set sensor pwr 0x%02x \n", g_video_peri_info.pwr_ctrl_pin);
		// Enable GPIO
		hal_sys_peripheral_en(GPIO_SYS, ENABLE);
		hal_pinmux_register(g_video_peri_info.rst_pin, PID_GPIO); //reset pin
		hal_pinmux_register(g_video_peri_info.pwdn_pin, PID_GPIO); //power down pin
		hal_pinmux_register(g_video_peri_info.snr_clk_pin, PID_SENSOR); //sensor clock
		video_dprintf(VIDEO_LOG_INF, "register pin rst 0x%02x pwdn 0x%02x snr_clk 0x%02x \n", g_video_peri_info.rst_pin, g_video_peri_info.pwdn_pin,
					  g_video_peri_info.snr_clk_pin);

		// Enable I2C
		i2c_master_video.pltf_dat.scl_pin = g_video_peri_info.scl_pin;
		i2c_master_video.pltf_dat.sda_pin = g_video_peri_info.sda_pin;
		i2c_master_video.init_dat.index = g_video_peri_info.i2c_id;//1;
		hal_i2c_pin_register_simple(&i2c_master_video);
		video_dprintf(VIDEO_LOG_INF, "i2c init scl 0x%02x sda 0x%02x id %d \n", g_video_peri_info.scl_pin, g_video_peri_info.sda_pin, g_video_peri_info.i2c_id);

	} else {
		sensor_en_pin_delay_init = 0x1;  // set flag for reset during de-init
		hal_video_reset_fcs_OK();
	}

}

void video_deinit_peri(void)
{
	volatile hal_i2c_adapter_t	i2c_master_video;

	// Disable I2C
	i2c_master_video.pltf_dat.scl_pin = g_video_peri_info.scl_pin;
	i2c_master_video.pltf_dat.sda_pin = g_video_peri_info.sda_pin;
	i2c_master_video.init_dat.index = g_video_peri_info.i2c_id;
	hal_i2c_pin_unregister_simple(&i2c_master_video);

	if (sensor_en_pin_delay_init) {    // for fcs OK flow, unregister and re-init
		sensor_en_pin_delay_init = 0;
		hal_pinmux_unregister(g_video_peri_info.pwr_ctrl_pin, PID_GPIO);
		video_dprintf(VIDEO_LOG_INF, "unregister sensor pwr 0x%02x \n", g_video_peri_info.pwr_ctrl_pin);
		hal_gpio_init(&sensor_en_gpio, g_video_peri_info.pwr_ctrl_pin);
		hal_gpio_set_dir(&sensor_en_gpio, GPIO_OUT);
	}
	hal_gpio_write(&sensor_en_gpio, 0);
	hal_gpio_deinit(&sensor_en_gpio);
	//unregister GPIO

	hal_pinmux_unregister(g_video_peri_info.rst_pin, PID_GPIO);//reset pin
	hal_pinmux_unregister(g_video_peri_info.pwdn_pin, PID_GPIO);//power down pin
	hal_pinmux_unregister(g_video_peri_info.snr_clk_pin, PID_SENSOR);//reset pin

}

void video_pre_init_setup_parameters(video_pre_init_params_t *parm)
{
	if (video_open_status() == 0) {
		memcpy(&video_pre_init_param, parm, sizeof(video_pre_init_params_t));
	} else {
		video_dprintf(VIDEO_LOG_MSG, "It need to close all chaneel to setup the initial parameters\r\n");
	}
}

video_pre_init_params_t *video_get_pre_init_setup_params(void)
{
	return &video_pre_init_param;
}

void video_pre_init_load_params(enum isp_init_option save_option)
{
	if (save_option == SAVE_TO_FLASH) {
		/*load from flash takes 3ms*/
		int fcs_buf_size = 2048;
		unsigned char *fcs_buf = malloc(fcs_buf_size);
		if (fcs_buf == NULL) {
			video_dprintf(VIDEO_LOG_ERR, "[%s] malloc failed\r\n", __func__);
			return;
		}
		//check fcs_data addr
		unsigned int flash_addr = 0;
		if (sys_get_boot_sel() == 0) {
			flash_addr = NOR_FLASH_FCS;
		} else {
			flash_addr = NAND_FLASH_FCS;
		}

		video_boot_stream_t *fcs_data = NULL;
		if (ftl_common_read(flash_addr, fcs_buf, fcs_buf_size) >= 0) {
			fcs_data = (video_boot_stream_t *)(fcs_buf + 8);

			if (fcs_buf[0] == 'F' && fcs_buf[1] == 'C' && fcs_buf[2] == 'S' && fcs_buf[3] == 'D') {
				//if fcs data are already writen, only modify ae, awb settings
				unsigned int checksum_tag = fcs_buf[4] | fcs_buf[5] << 8 | fcs_buf[6] << 16 | fcs_buf[7] << 24;
				unsigned int checksum_value = 0;
				for (int i = 0; i < sizeof(video_boot_stream_t); i++) {
					checksum_value += fcs_buf[i + 8];
				}
				if (checksum_tag == checksum_value) {
					memcpy(&(video_pre_init_param.isp_ae_enable), &(fcs_data->fcs_isp_ae_enable), sizeof(uint32_t) * 6);
					video_dprintf(VIDEO_LOG_MSG, "ISP pre init params load success\r\n");
					video_dprintf(VIDEO_LOG_INF, "ae exposure time %d, ae gain %d\r\n", video_pre_init_param.isp_ae_init_exposure, video_pre_init_param.isp_ae_init_gain);
					video_dprintf(VIDEO_LOG_INF, "awb rgain %d, bgain %d\r\n", video_pre_init_param.isp_awb_init_rgain, video_pre_init_param.isp_awb_init_bgain);
				} else {
					video_dprintf(VIDEO_LOG_ERR, "ISP pre init check sum fail\r\n");
				}
			} else {
				video_dprintf(VIDEO_LOG_ERR, "Can't find %x %x %x %x\r\n", fcs_buf[0], fcs_buf[1], fcs_buf[2], fcs_buf[3]);
			}
		}
		if (fcs_buf) {
			free(fcs_buf);
		}
#ifdef USE_ISP_RETENTION_DATA
	} else if (save_option == SAVE_TO_RETENTION) {

		uint8_t *checksum_array = (uint8_t *) &isp_retention_data.ae_exposure;
		uint32_t checksum_value = 0;
		for (int i = 0; i < (sizeof(isp_retention_data_t) - sizeof(isp_retention_data.checksum)); i++) {
			checksum_value += checksum_array[i];
		}
		video_dprintf(VIDEO_LOG_MSG, "load isp retention %d %d\r\n", isp_retention_data.checksum, checksum_value);
		if (isp_retention_data.checksum == checksum_value) {
			video_pre_init_param.isp_ae_enable = 1;
			video_pre_init_param.isp_ae_init_exposure = isp_retention_data.ae_exposure;
			video_pre_init_param.isp_ae_init_gain = isp_retention_data.ae_gain;
			video_pre_init_param.isp_awb_enable = 1;
			video_pre_init_param.isp_awb_init_rgain = isp_retention_data.awb_rgain;
			video_pre_init_param.isp_awb_init_bgain = isp_retention_data.awb_bgain;
		} else {
			video_dprintf(VIDEO_LOG_ERR, "ISP pre init check sum fail\r\n");
		}
#endif
	}
}

void video_pre_init_save_cur_params(int meta_enable, video_meta_t *meta_data, enum isp_init_option save_option)
{
	int ae_exposure_time, ae_gain, awb_rgain, awb_bgain;
	/*get ae, awb info*/
	if (meta_enable) {
		ae_exposure_time = meta_data->isp_statis_meta->exposure_h;
		ae_gain = meta_data->isp_statis_meta->gain_h;
		awb_rgain = meta_data->isp_statis_meta->wb_r_gain;
		awb_bgain = meta_data->isp_statis_meta->wb_b_gain;
	} else {
		/*get ae, awb info from isp cmd takes 90ms*/
		isp_get_exposure_time(&ae_exposure_time);
		isp_get_ae_gain(&ae_gain);
		isp_get_red_balance(&awb_rgain);
		isp_get_blue_balance(&awb_bgain);
	}

	//video_dprintf(VIDEO_LOG_INF, "ae exposure time %d, ae gain %d\r\n", ae_exposure_time, ae_gain);
	video_pre_init_param.isp_ae_enable = 1;
	video_pre_init_param.isp_ae_init_exposure = ae_exposure_time;
	video_pre_init_param.isp_ae_init_gain = ae_gain;
	//video_dprintf(VIDEO_LOG_INF, "awb rgain %d, bgain %d\r\n", awb_rgain, awb_bgain);
	video_pre_init_param.isp_awb_enable = 1;
	video_pre_init_param.isp_awb_init_rgain = awb_rgain;
	video_pre_init_param.isp_awb_init_bgain = awb_bgain;

	/*save to flash takes 60ms*/
	if (save_option == SAVE_TO_FLASH) {
		int fcs_buf_size = 2048;
		unsigned char *fcs_buf = malloc(fcs_buf_size);
		if (fcs_buf == NULL) {
			video_dprintf(VIDEO_LOG_ERR, "[%s] malloc failed\r\n", __func__);
			return;
		}
		//check fcs_data addr
		unsigned int flash_addr = 0;
		if (sys_get_boot_sel() == 0) {
			flash_addr = NOR_FLASH_FCS;
		} else {
			flash_addr = NAND_FLASH_FCS;
		}

		video_boot_stream_t *fcs_data = NULL;
		if (ftl_common_read(flash_addr, fcs_buf, fcs_buf_size) >= 0) {
			fcs_data = (video_boot_stream_t *)(fcs_buf + 8);
			unsigned char *ptr = (unsigned char *)(fcs_buf + 8);

			if (fcs_buf[0] == 'F' && fcs_buf[1] == 'C' && fcs_buf[2] == 'S' && fcs_buf[3] == 'D') {
				//if fcs data are already writen, only modify ae, awb settings
				unsigned int checksum_tag = fcs_buf[4] | fcs_buf[5] << 8 | fcs_buf[6] << 16 | fcs_buf[7] << 24;
				unsigned int checksum_value = 0;
				for (int i = 0; i < sizeof(video_boot_stream_t); i++) {
					checksum_value += fcs_buf[i + 8];
				}
				if (checksum_tag != checksum_value) {
					//if fcs data not writen, set other parameters to 0.
					memset(fcs_data, 0x00, sizeof(video_boot_stream_t));
				} else {
					video_dprintf(VIDEO_LOG_INF, "FCS DATA exist\r\n");
				}
			} else {
				//if fcs data not writen, set other parameters to 0.
				memset(fcs_data, 0x00, sizeof(video_boot_stream_t));
			}

			fcs_buf[0] = 'F';
			fcs_buf[1] = 'C';
			fcs_buf[2] = 'S';
			fcs_buf[3] = 'D';
			fcs_data->fcs_isp_ae_enable = 1;
			fcs_data->fcs_isp_ae_init_exposure = ae_exposure_time;
			fcs_data->fcs_isp_ae_init_gain = ae_gain;
			fcs_data->fcs_isp_awb_enable = 1;
			fcs_data->fcs_isp_awb_init_rgain = awb_rgain;
			fcs_data->fcs_isp_awb_init_bgain = awb_bgain;
			unsigned int checksum_tag = 0;
			for (int i = 0; i < sizeof(video_boot_stream_t); i++) {
				checksum_tag += ptr[i];
			}
			fcs_buf[4] = (checksum_tag) & 0xff;
			fcs_buf[5] = (checksum_tag >> 8) & 0xff;
			fcs_buf[6] = (checksum_tag >> 16) & 0xff;
			fcs_buf[7] = (checksum_tag >> 24) & 0xff;
			if (ftl_common_write(flash_addr, fcs_buf, fcs_buf_size) >= 0) {
				video_dprintf(VIDEO_LOG_MSG, "ISP pre init params save success\r\n");
			}
		}
		if (fcs_buf) {
			free(fcs_buf);
		}
#ifdef USE_ISP_RETENTION_DATA
	} else if (save_option == SAVE_TO_RETENTION) {
		//video_dprintf(VIDEO_LOG_MSG, "retention table address 0x%x\r\n", &retention_table.isp_init_data.tag);
		isp_retention_data.ae_exposure = ae_exposure_time;
		isp_retention_data.ae_gain = ae_gain;
		isp_retention_data.awb_rgain = awb_rgain;
		isp_retention_data.awb_bgain = awb_bgain;

		uint8_t *checksum_array = (uint8_t *) &isp_retention_data.ae_exposure;
		uint32_t checksum_value = 0;
		for (int i = 0; i < (sizeof(isp_retention_data_t) - sizeof(isp_retention_data.checksum)); i++) {
			checksum_value += checksum_array[i];
		}
		isp_retention_data.checksum = checksum_value;
		video_dprintf(VIDEO_LOG_MSG, "save isp retention %d %d\r\n", isp_retention_data.checksum, checksum_value);
		dcache_clean_invalidate_by_addr((uint32_t *)&isp_retention_data, sizeof(isp_retention_data_t));

		uint8_t *tag = (uint8_t *)&retention_table.isp_init_data.tag;
		tag[0] = 'I';
		tag[1] = 'S';
		tag[2] = 'P';
		tag[3] = 'D';
		retention_table.isp_init_data.data_len = sizeof(isp_retention_data_t);
		retention_table.isp_init_data.address = (uint32_t)&isp_retention_data;
		retention_table.isp_init_data.checksum = checksum_value;
		dcache_clean_invalidate_by_addr((uint32_t *)&retention_table, sizeof(retention_table));
#endif
	}

}


int video_pre_init_get_meta_enable(void)
{
	if (isp_boot->fcs_status) {
		return isp_boot->meta_enable;
	} else {
		return video_pre_init_param.meta_enable;
	}
}

void video_calcu_meta_size(video_pre_init_params_t *parm)
{
	int meta_size = 0;
	int meta_loop = 0;
	int enable_extend = 0;
	if (parm->meta_enable_extend) {
		meta_loop = 2;
	} else {
		meta_loop = 1;
	}
	for (int i = 0; i < meta_loop; i++) {
		if (i == 1 && parm->meta_enable_extend) {
			enable_extend = 1;//Insert the extend meta size
		}
		if (enable_extend) {
			meta_size = parm->meta_size + sizeof(isp_meta_t) + sizeof(isp_statis_meta_t) + sizeof(af_statis_t) + sizeof(ae_statis_t) + sizeof(awb_statis_t);
		} else {
			meta_size = parm->meta_size + sizeof(isp_meta_t) + sizeof(isp_statis_meta_t);
		}
		meta_size = meta_size + meta_size / 2; //Add the extra buffer to dummy bytes
		if (meta_size % 32) { //align 32 byte
			meta_size = meta_size + (32 - (meta_size % 32));
		}
		if (enable_extend) {
			parm->video_meta_extend_offset = meta_size / 0xff;
			parm->video_meta_extend_total_size = meta_size;
			if (parm->video_meta_extend_total_size > VIDEO_META_REV_BUF) {
				video_dprintf(VIDEO_LOG_MSG, "Meta size %d is exceed the sei buffer %d\r\n", meta_size, VIDEO_META_REV_BUF);
				parm->video_meta_extend_total_size = VIDEO_META_REV_BUF;
				//v_adp->cmd[ch]->userData = VIDEO_META_REV_BUF;
				video_dprintf(VIDEO_LOG_MSG, "Setup the meta size as %d\r\n", VIDEO_META_REV_BUF);
			}
		} else {
			parm->video_meta_offset = meta_size / 0xff;
			parm->video_meta_total_size = meta_size;
			if (parm->video_meta_total_size > VIDEO_META_REV_BUF) {
				video_dprintf(VIDEO_LOG_ERR, "Meta size %d is exceed the sei buffer %d\r\n", meta_size, VIDEO_META_REV_BUF);
				parm->video_meta_total_size = VIDEO_META_REV_BUF;
				//v_adp->cmd[ch]->userData = VIDEO_META_REV_BUF;
				video_dprintf(VIDEO_LOG_ERR, "Setup the meta size as %d\r\n", VIDEO_META_REV_BUF);
			}
		}
	}
}

void video_pre_init_procedure(int ch, video_pre_init_params_t *parm)
{
	video_dprintf(VIDEO_LOG_MSG, "[%s] START\r\n", __FUNCTION__);
	hal_video_adapter_t *v_adp = hal_video_get_adp();
	unsigned char *cal_iq_addr = video_load_cal_iq(CALI_IQ_FW);
	int *calib_iq_size = (int *)(cal_iq_addr + 4);
	struct isp_iq_cali *piq_cali_data = (struct isp_iq_cali *)(cal_iq_addr + 8);
	video_dprintf(VIDEO_LOG_INF, "[hal_video_load_cali_iq] enable:%d fun-en:%d %c %c %c %c %d \r\n", calib_iq_en, piq_cali_data->enable, cal_iq_addr[0],
				  cal_iq_addr[1], cal_iq_addr[2], cal_iq_addr[3], *calib_iq_size);
	if (calib_iq_en && cal_iq_addr[0] == 'C' && cal_iq_addr[1] == 'A' && cal_iq_addr[2] == 'I' && cal_iq_addr[3] == 'Q' &&
		*calib_iq_size == sizeof(struct isp_iq_cali)) {
		hal_video_enable_load_cali_iq(ch, calib_iq_en);
		hal_video_load_cali_iq((voe_cpy_t)hal_voe_cpy, (int *)(piq_cali_data), __voe_code_start__, sizeof(struct isp_iq_cali));
		video_dprintf(VIDEO_LOG_MSG, "[%s] apply calibration data.\r\n", __FUNCTION__);
	}

	if (parm->fast_mask_en) {
		video_set_private_mask(ch, &parm->fast_mask);
		hal_video_fast_enable_mask(ch);
	}

	if (parm->isp_init_enable) {
		v_adp->cmd[ch]->all_init_iq_set_flag = 1;
		hal_video_set_isp_init_items(ch, &parm->init_isp_items);
	} else {
		v_adp->cmd[ch]->all_init_iq_set_flag = 1;
		video_isp_initial_items_t init_items;
		init_items.init_brightness = 0;
		init_items.init_contrast = 50;
		init_items.init_flicker = 1;
		init_items.init_hdr_mode = 0;
		init_items.init_mirrorflip = 0xf0;
		init_items.init_saturation = 50;
		init_items.init_wdr_level = 50;
		init_items.init_wdr_mode = 2;
		init_items.init_mipi_mode = 0;
		hal_video_set_isp_init_items(ch, &init_items);
	}

	if (parm->dn_init_enable) {
		isp_set_dn_initial_mode(ch, parm->dn_init_mode);
		video_dprintf(VIDEO_LOG_INF, "Set night mode\r\n");
	}

	if (parm->meta_enable) {
		video_calcu_meta_size(parm);
	}
	if (parm->voe_dbg_disable) {
		v_adp->cmd[0]->voe_dbg = 0;
	} else {
		v_adp->cmd[0]->voe_dbg = 1;
	}
	if (parm->isp_ae_enable) {
		v_adp->cmd[ch]->set_AE_init_flag = 1;
		v_adp->cmd[ch]->all_init_iq_set_flag = 1;
		v_adp->cmd[ch]->direct_i2c_mode = 1;
		//init exposure time min limit /*to fix*/
		if (parm->isp_ae_init_exposure < 300) {
			video_dprintf(VIDEO_LOG_MSG, "modify ae exposure time to 300\r\n");
			v_adp->cmd[ch]->init_exposure = 300;
		} else {
			v_adp->cmd[ch]->init_exposure = parm->isp_ae_init_exposure;
		}
		v_adp->cmd[ch]->init_gain = parm->isp_ae_init_gain;
	}
	if (parm->isp_awb_enable) {
		v_adp->cmd[ch]->set_AWB_init_flag = 1;
		v_adp->cmd[ch]->all_init_iq_set_flag = 1;
		v_adp->cmd[ch]->init_r_gain = parm->isp_awb_init_rgain;
		v_adp->cmd[ch]->init_b_gain = parm->isp_awb_init_bgain;
	}
	if (parm->video_drop_enable) {
		v_adp->cmd[ch]->all_init_iq_set_flag = 1;
		v_adp->cmd[ch]->drop_frame_num = parm->video_drop_frame;
	}

#if USE_VIDEO_HR_FLOW
	if (parm->isp_init_raw) {
		//for trigger first raw
		video_dprintf(VIDEO_LOG_INF, "hal_video_isp_init_raw(%d, 1)\r\n", ch);
		hal_video_isp_init_raw(ch, 1);
	}

	if (parm->isp_raw_mode_tnr_dis) {
		video_dprintf(VIDEO_LOG_INF, "hal_video_isp_tnr_dis(%d, 1)\r\n", ch);
		hal_video_isp_raw_mode_tnr_dis(ch, 1);
	}

	if (parm->v_cfg) {
		video_dprintf(VIDEO_LOG_INF, "hal_isp_set_verify_info(%d, %x)\r\n", ch, parm->v_cfg);
		hal_video_isp_verify_info(ch, *(parm->v_cfg));
	}

	if (parm->zoom_coef) {
		video_dprintf(VIDEO_LOG_INF, "hal_video_isp_zoom_filter_coef_init(%d, %x)\r\n", ch, parm->zoom_coef);
		video_dprintf(VIDEO_LOG_INF, "%d %d %d %d\r\n", parm->zoom_coef[ISP_ZOOM_FILTER_COEF_NUM - 1], parm->zoom_coef[ISP_ZOOM_FILTER_COEF_NUM - 2],
					  parm->zoom_coef[ISP_ZOOM_FILTER_COEF_NUM - 3], parm->zoom_coef[ISP_ZOOM_FILTER_COEF_NUM - 4]);
		hal_video_isp_zoom_filter_coef_init(ch, &(parm->zoom_coef[0]));
	}

	hal_video_isp_init_dyn_iq_mode(ch, parm->dyn_iq_mode);

#endif
}

int video_open_status(void)//0:No video open 1:video open
{
	int status = 0;
	for (int i = 0; i < MAX_CHANNEL; i++) {
		if (voe_info.ch_info[i].stream_is_open) {
			status = 1;
			break;
		}
	}
	return status;
}

int video_check_fcs(int stream_id)
{
	int i = 0;
	int ret = 0;
	for (i = 0; i < 4; i++) {
		if (isp_boot->video_params[i].stream_id == stream_id) {
			if (isp_boot->video_params[i].fcs) {
				ret = 1;
			}
			break;
		}
	}
	return ret;
}

int video_open(video_params_t *v_stream, output_callback_t output_cb, void *ctx)
{
	int ch = v_stream->stream_id;
	int fcs_v = v_stream->fcs;
	int isp_fps = isp_info.sensor_fps;
	int fps = 30;
	int gop = 30;
	int rcMode = 1;
	int bps = 1024 * 1024;
	int minQp = 0;
	int maxQp = 51;
	int rotation = 0;
	int jpeg_qlevel = 5;

	char cmd1[384];
	char cmd2[384];
	char cmd3[384];
	char fps_cmd1[64];
	char fps_cmd2[64];
	char fps_cmd3[64];
	int type;
	int res = 0;
	int codec = 0;

	int enc_in_w = (v_stream->width + 15) & ~15;  //force 16 aligned
	int enc_in_h = (v_stream->height + 3) & ~3;  //force 4 aligned
	int enc_out_w = v_stream->width;  //will crop enc_in_w to enc_out_w
	int enc_out_h = v_stream->height; //will crop enc_in_h to enc_out_h
	int enc_out_w_offset = ((enc_in_w - enc_out_w) / 2) & ~1; //force 2 aligned
	int enc_out_h_offset = ((enc_in_h - enc_out_h) / 2) & ~1; //force 2 aligned

	int out_rsvd_size = (enc_in_w * enc_in_h) / VIDEO_RSVD_DIVISION;
	int out_buf_size = 0;
	int jpeg_out_buf_size = (enc_in_w * enc_in_h * 3) / 2; //jpeg max size = NV12 size
	int jpeg_out_rsvd_size = jpeg_out_buf_size; //set buf_size==rsvd_size to disable ring buffer
	unsigned char *cal_iq_addr;
	struct isp_iq_cali *piq_cali_data;
	int status = OK;
	rtw_mutex_get(&video_open_close_mutex);

	voe_info.ch_info[ch].param = v_stream;
	voe_info.ch_info[ch].video_output_cb = output_cb;
	switch (ch) {
	case 0:
		out_buf_size = (v_stream->bps * V1_ENC_BUF_SIZE) / 8 + out_rsvd_size;
		break;
	case 1:
		out_buf_size = (v_stream->bps * V2_ENC_BUF_SIZE) / 8 + out_rsvd_size;
		break;
	case 2:
		out_buf_size = (v_stream->bps * V3_ENC_BUF_SIZE) / 8 + out_rsvd_size;
		break;
	case 3:
		out_buf_size = (v_stream->bps * VEXT_ENC_BUF_SIZE) / 8 + out_rsvd_size;
		break;
	}

	type = v_stream->type;

	codec = video_get_codec_type(type);

	video_dprintf(VIDEO_LOG_INF, "video w = %d, video h = %d\r\n", v_stream->width, v_stream->height);
	if ((codec & (CODEC_HEVC | CODEC_H264 | CODEC_JPEG)) != 0) {
		video_dprintf(VIDEO_LOG_INF, "encoder w_in,h_in=%d,%d w_out,h_out=%d,%d\r\n", enc_in_w, enc_in_h, enc_out_w, enc_out_h);
	}

	memset(fps_cmd1, 0x0, sizeof(fps_cmd1));
	memset(fps_cmd2, 0x0, sizeof(fps_cmd2));
	memset(fps_cmd3, 0x0, sizeof(fps_cmd3));
	memset(cmd1, 0x0, sizeof(cmd1));
	memset(cmd2, 0x0, sizeof(cmd2));
	memset(cmd3, 0x0, sizeof(cmd3));
	res = v_stream->resolution;

	if (v_stream->fps) {
		fps = v_stream->fps;
		hal_video_set_fps(fps, ch);

		if (v_stream->out_mode == MODE_SYNC || v_stream->out_mode == MODE_EXT) {
			isp_fps = fps;
		}
	}

	if (v_stream->bps) {
		bps = v_stream->bps;
	}

	if (v_stream->gop) {
		gop = v_stream->gop;
	}

	if (v_stream->rc_mode) {
		rcMode = v_stream->rc_mode - 1;
		if (rcMode) {
			minQp = 25;
			maxQp = 48;
			bps = v_stream->bps / 2;
		}
	}

	if (v_stream->minQp > 0 && v_stream->minQp <= 51) {
		minQp = v_stream->minQp;
	}
	if (v_stream->maxQp > 0 && v_stream->maxQp <= 51) {
		maxQp = v_stream->maxQp;
	}

	if (v_stream->rotation) {
		rotation = v_stream->rotation;
	}

	if (v_stream->jpeg_qlevel) {
		jpeg_qlevel = v_stream->jpeg_qlevel;
	}

	if (isp_boot->fcs_status == 1 && isp_boot->fcs_setting_done == 0 && video_check_fcs(v_stream->stream_id) == 1) {
		hal_video_set_fps(isp_boot->video_params[ch].fps, ch); // for count the enc offset of the fcs channel
		if (hal_video_out_cb(video_output_cb, 4096, (uint32_t)ctx, ch) != OK) {
			video_dprintf(VIDEO_LOG_ERR, "hal_video_cb_register fail\n");
			//return -1;
			status = NOK;
			goto EXIT;
		}
		static int count = 0;
		count++;
		video_dprintf(VIDEO_LOG_INF, "start fcs video\r\n");
		if (codec & CODEC_H264) {
			voe_info.ch_info[ch].isp_fps = isp_boot->isp_info.sensor_fps;
			hal_video_out_mode(ch, TYPE_H264, MODE_ENABLE);
		} else if (codec & CODEC_NV12) {
			voe_info.ch_info[ch].isp_fps = isp_boot->video_params[ch].fps;
			hal_video_out_mode(ch, TYPE_NV12, MODE_ENABLE);
		} else if (codec & CODEC_HEVC) {
			voe_info.ch_info[ch].isp_fps = isp_boot->isp_info.sensor_fps;
			hal_video_out_mode(ch, TYPE_HEVC, MODE_ENABLE);
		} else if (codec & CODEC_RGB) {
			voe_info.ch_info[ch].isp_fps = isp_boot->video_params[ch].fps;
			hal_video_out_mode(ch, TYPE_RGB, MODE_ENABLE);
		}
		if ((codec & (CODEC_HEVC | CODEC_H264)) != 0) {
			if (video_rc_init(v_stream) != OK) {
				video_dprintf(VIDEO_LOG_ERR, "ch%d rc task init fail\r\n", ch);
				status = NOK;
				goto EXIT;
			}
		}
		hal_video_fcs_ch(count);
		if (count == isp_boot->fcs_channel) {
			isp_boot->fcs_setting_done = 1;
			video_dprintf(VIDEO_LOG_MSG, "The fcs setup is finished\r\n");
		}

		if (voe_info_init == 0) {
			voe_info_init = 1;
		}
		voe_info.ch_info[ch].stream_is_open = 1;
		if (isp_boot->fcs_start_time) { //If it enable the fcs mode that it will show the fcs info.
			hal_video_print(APP_VOE_FCS_INFO_EN);
			video_time_info_t video_time;
			hal_video_time_info(1, &video_time);
			isp_info.frame_done_time = isp_boot->fcs_voe_time + video_time.frame_done / 1000;
			video_dprintf(VIDEO_LOG_MSG, " fcs_start_time %d fcs_voe_time %d frame_done_time %d\r\n", isp_boot->fcs_start_time, isp_boot->fcs_voe_time,
						  isp_info.frame_done_time);
			hal_video_print((video_pre_init_param.voe_dbg_disable == 0) ? 1 : 0);
		}
		for (int i = 0; i <= 2; i++) {
			g_enc_buff_size[i] = isp_boot->video_params[i].out_buf_size;
			video_dprintf(VIDEO_LOG_MSG, "g_enc_buff_size %d = %d\r\n", i, g_enc_buff_size[i]);
		}
		g_enc_buff_size[3] = isp_boot->extra_video_params.out_buf_size;

		v_stream->out_buf_size  = g_enc_buff_size[ch];
		v_stream->out_rsvd_size = out_rsvd_size;
		//return OK;
		goto EXIT;
	}

	int ret = 0;
	if ((codec & (CODEC_HEVC | CODEC_H264)) != 0) {
		ret = snprintf(fps_cmd1, sizeof(fps_cmd1), "-f %d -j %d -R %d -B %d --vbr %d -n %d -m %d", fps, isp_fps, gop, bps, rcMode, minQp, maxQp);
		if (ret < 0 || ret >= sizeof(fps_cmd1)) {
			printf("fail to generate fps_cmd1\r\n");
			//return -1;
			status = NOK;
			goto EXIT;
		}
		ret = snprintf(cmd1, sizeof(cmd1), "%s %d %s -w %d -h %d -x %d -X %d -y %d -Y %d -r %d --mode %d --codecFormat %d %s -i isp"  // -x %d -y %d
					   , fmt_table[(codec & (CODEC_HEVC | CODEC_H264)) - 1]
					   , ch
					   , fps_cmd1
					   , enc_in_w
					   , enc_in_h
					   , enc_out_w
					   , enc_out_w_offset
					   , enc_out_h
					   , enc_out_h_offset
					   , rotation
					   , 2
					   , (codec & (CODEC_HEVC | CODEC_H264)) - 1
					   , paramter_table[(codec & (CODEC_HEVC | CODEC_H264)) - 1]);
		if (ret < 0 || ret >= sizeof(cmd1)) {
			printf("fail to generate cmd1\r\n");
			//return -1;
			status = NOK;
			goto EXIT;
		}
	}

	if ((codec & CODEC_JPEG) != 0) {
		ret = snprintf(fps_cmd2, sizeof(fps_cmd2), "-j %d -n %d", isp_fps, fps);
		if (ret < 0 || ret >= sizeof(fps_cmd2)) {
			printf("fail to generate fps_cmd2\r\n");
			//return -1;
			status = NOK;
			goto EXIT;
		}
		ret = snprintf(cmd2, sizeof(cmd2), "%s %d %s -w %d -h %d -x %d -X %d -y %d -Y %d -G %d -q %d --mode %d --codecFormat %d %s -i isp"
					   , fmt_table[2]
					   , ch
					   , fps_cmd2
					   , enc_in_w
					   , enc_in_h
					   , enc_out_w
					   , enc_out_w_offset
					   , enc_out_h
					   , enc_out_h_offset
					   , rotation
					   , jpeg_qlevel
					   , 0
					   , 2
					   , paramter_table[2]);
		if (ret < 0 || ret >= sizeof(cmd2)) {
			printf("fail to generate cmd2\r\n");
			//return -1;
			status = NOK;
			goto EXIT;
		}
	}

	if ((codec & (CODEC_NV12 | CODEC_RGB | CODEC_NV16)) != 0) {
		isp_fps = fps;
		int value;
		if ((codec & CODEC_NV12) != 0) {
			value = 3;
		} else if ((codec & CODEC_RGB) != 0) {
			value = 4;
		} else {
			value = 5;
		}
		ret = snprintf(fps_cmd3, sizeof(fps_cmd3), "-j %d", fps);
		if (ret < 0 || ret >= sizeof(fps_cmd3)) {
			printf("fail to generate fps_cmd3\r\n");
			//return -1;
			status = NOK;
			goto EXIT;
		}
		ret = snprintf(cmd3, sizeof(cmd3), "%s %d %s -w %d -h %d --mode %d --codecFormat %d %s -i isp"
					   , fmt_table[value]
					   , ch
					   , fps_cmd3
					   , v_stream->width
					   , v_stream->height
					   , (v_stream->out_mode ? v_stream->out_mode : 0) //set out mode if not 0
					   , 3
					   , paramter_table[value]);
		if (ret < 0 || ret >= sizeof(cmd3)) {
			printf("fail to generate cmd3\r\n");
			//return -1;
			status = NOK;
			goto EXIT;
		}
		video_dprintf(VIDEO_LOG_INF, "str3 %s\n", cmd3);

	}
	voe_info.ch_info[ch].isp_fps = isp_fps;

	video_dprintf(VIDEO_LOG_INF, "%s\n", cmd1);
	video_dprintf(VIDEO_LOG_INF, "%s\n", cmd2);
	video_dprintf(VIDEO_LOG_INF, "%s\n", cmd3);

	video_dprintf(VIDEO_LOG_INF, "hal_video_str2cmd\r\n");
	//string to command
	if (hal_video_str2cmd(cmd1, cmd2, cmd3) != OK) {
		video_dprintf(VIDEO_LOG_ERR, "hal_video_str2cmd fail\n");
		//return -1;
		status = NOK;
		goto EXIT;
	}


	hal_video_isp_set_sensor_gpio(ch, g_video_peri_info.rst_pin, g_video_peri_info.pwdn_pin, g_video_peri_info.pwr_ctrl_pin);
	hal_video_isp_set_i2c_id(ch, g_video_peri_info.i2c_id);

	if ((codec & (CODEC_HEVC | CODEC_H264)) != 0) {
		if (video_rc_init(v_stream) != OK) {
			video_dprintf(VIDEO_LOG_ERR, "ch%d rc task init fail\r\n", ch);
			status = NOK;
			goto EXIT;
		}
	}
	video_dprintf(VIDEO_LOG_INF, "set video callback\r\n");
	if (hal_video_out_cb(video_output_cb, 4096, (uint32_t)ctx, ch) != OK) {
		video_dprintf(VIDEO_LOG_ERR, "hal_video_cb_register fail\n");
		status = NOK;
		goto EXIT;
	}
#if FASTOSD_EN
	if (v_stream->fast_osd_en) {
		voe_info.ch_info[ch].stream_is_open = 2;
		rts_osd_fast_start_enable(ch, 1);
		isp_osd_lite(ch);
	}
#endif
	video_dprintf(VIDEO_LOG_INF, "hal_video_isp_buf_num\r\n");
	hal_video_isp_buf_num(ch, isp_ch_buf_num[ch]);

	if (codec & (CODEC_HEVC | CODEC_H264)) {
		//hal_video_enc_buf(ch, out_buf_size, out_rsvd_size);
		hal_video_enc_buf(ch, g_enc_buff_size[ch], out_rsvd_size);
		v_stream->out_buf_size = g_enc_buff_size[ch];
		v_stream->out_rsvd_size = out_rsvd_size;
	}
	if (codec & CODEC_JPEG) {
		hal_video_jpg_buf(ch, jpeg_out_buf_size, jpeg_out_rsvd_size);
	}

	hal_video_adapter_t *v_adp = hal_video_get_adp();
	/* Encoder initialization */
	if (v_adp) {
		if (isp_info.osd_enable) {
			v_adp->cmd[ch]->osd	= 1;
			v_adp->cmd[ch]->osd_block_num = 24;
		} else {
			v_adp->cmd[ch]->osd	= 0;
			v_adp->cmd[ch]->osd_block_num = 0;
		}
	}

	if (video_open_status() == 0) {
		voe_info.voe_scale_up_en = 0;
	}

	if (voe_info.voe_scale_up_en == 0) {
		int origin_width = (int)isp_info.sensor_width;
		int origin_height = (int)isp_info.sensor_height;
		if (v_stream->use_roi) {
			origin_width = (int)v_stream->roi.xmax - (int)v_stream->roi.xmin;
			origin_height = (int)v_stream->roi.ymax - (int)v_stream->roi.ymin;
		}
		if (origin_width <= 0 || origin_height <= 0) {
			video_dprintf(VIDEO_LOG_ERR, "error: invlalid input resolution\r\n");
			status = NOK;
			goto EXIT;
		}
		if (v_stream->out_mode == MODE_EXT) {
			video_dprintf(VIDEO_LOG_INF, "ch%d ext mode skip roi check\r\n", ch);
		} else if (enc_in_w <= origin_width && enc_in_h <= origin_height) { //scale down
			if (v_stream->use_roi) {
				video_dprintf(VIDEO_LOG_INF, "ch%d set_roi (%d,%d,%d,%d)\r\n", ch, v_stream->roi.xmin, v_stream->roi.ymin, origin_width, origin_height);
				hal_video_isp_set_roi(ch, v_stream->roi.xmin, v_stream->roi.ymin, origin_width, origin_height);
			}
		} else if (enc_in_w >= origin_width && enc_in_h >= origin_height) {//scale up
			if (ch != 0) {
				video_dprintf(VIDEO_LOG_ERR, "error: scale up only support ch0\r\n");
				status = NOK;
				goto EXIT;
			} else if ((enc_in_w >= origin_width * 2) || (enc_in_h >= origin_height * 2)) {
				video_dprintf(VIDEO_LOG_ERR, "error: Encoder width and height should both be less than roi_w/h multi two.\r\n");
				status = NOK;
				goto EXIT;
			} else if (enc_in_w > 2688 || enc_in_h > 1944) {
				video_dprintf(VIDEO_LOG_ERR, "error: max scale up resolution is 2688x1944 \r\n");
				status = NOK;
				goto EXIT;
			} else {
				voe_info.voe_scale_up_en = 1;
				hal_video_isp_clk_set(ch, 5, 0);
				if (v_stream->use_roi) {
					video_dprintf(VIDEO_LOG_INF, "ch%d set_roi (%d,%d,%d,%d)\r\n", ch, v_stream->roi.xmin, v_stream->roi.ymin, origin_width, origin_height);
					hal_video_isp_set_roi(ch, v_stream->roi.xmin, v_stream->roi.ymin, origin_width, origin_height);
					memcpy(&(voe_info.voe_scale_up_roi), &(v_stream->roi), sizeof(video_roi_t));
				}
			}
		} else {
			video_dprintf(VIDEO_LOG_ERR, "error: invalid resolution %dx%d -> %dx%d\r\n", origin_width, origin_height, enc_in_w, enc_in_h);
			status = NOK;
			goto EXIT;
		}
	} else if (v_stream->use_roi) {
		if (ch == 0) {
			video_dprintf(VIDEO_LOG_INF, "It don't need to setup ch0 again\r\n");
			//ch0 should maintain the roi settings, while reopening /* to fix */
			int roi_w = (int)voe_info.voe_scale_up_roi.xmax - (int)voe_info.voe_scale_up_roi.xmin;
			int roi_h = (int)voe_info.voe_scale_up_roi.ymax - (int)voe_info.voe_scale_up_roi.ymin;
			hal_video_isp_set_roi(ch, voe_info.voe_scale_up_roi.xmin, voe_info.voe_scale_up_roi.ymin, roi_w, roi_h);
		} else {
			video_dprintf(VIDEO_LOG_ERR, "It don't support to setup the ROI when scale up is enable\r\n");
			status = NOK;
			goto EXIT;
		}
	}

	if ((codec & (CODEC_HEVC | CODEC_H264)) != 0) {
		if (v_stream->level) {
			v_adp->cmd[ch]->level = v_stream->level;
		}
		if (v_stream->profile) {
			v_adp->cmd[ch]->profile = v_stream->profile;
		}
		if (v_stream->cavlc) {
			v_adp->cmd[ch]->enableCabac = 0;
		}
	}

	if (v_stream->out_mode == MODE_SYNC || v_stream->out_mode == MODE_EXT) {
		if (codec & (CODEC_HEVC | CODEC_H264)) {
			v_adp->cmd[ch]->EncMode = v_stream->out_mode;
		}
		if (codec & CODEC_JPEG) {
			v_adp->cmd[ch]->JpegMode = v_stream->out_mode;
		}
	}
	if (v_stream->out_mode == MODE_EXT) {
		v_adp->cmd[ch]->inputFormat = v_stream->ext_fmt;
	}
	video_dprintf(VIDEO_LOG_INF, "[ch%d] video mode: ENC(%d) JPG(%d) YUV(%d)\n", ch, v_adp->cmd[ch]->EncMode, v_adp->cmd[ch]->JpegMode, v_adp->cmd[ch]->YuvMode);

	/* if (v_adp) {
		v_adp->cmd[ch]->roiMapDeltaQpBlockUnit = 2; //0:64x64, 1:32x32, 2:16x16
		v_adp->cmd[ch]->roiMapDeltaQpEnable = 1;
	} */

	if (voe_info_init == 0 || video_open_status() == 0) {
		video_pre_init_procedure(ch, &video_pre_init_param);
		voe_info_init = 1;
	} else if (ch == 0) {
		// hal_video_str2cmd default enable voe log
		v_adp->cmd[0]->voe_dbg = ((video_pre_init_param.voe_dbg_disable == 0) ? 1 : 0);
	}

	if (v_stream->vui_disable) {
		video_dprintf(VIDEO_LOG_INF, "disable the vui\r\n");
		v_adp->cmd[ch]->vui_timing_info_enable = 0;
	}
	if (isp_boot->fcs_status == 1) {
		if (v_stream->meta_enable == 1 && isp_boot->meta_enable == 1) {
			if (codec & CODEC_JPEG) {
				v_adp->cmd[ch]->JPGuserData = isp_boot->fcs_meta_total_size;
			} else {
				v_adp->cmd[ch]->EncuserData = isp_boot->fcs_meta_total_size;
			}
			if (isp_boot->extra_fcs_meta_enable_extend == 1) {
				v_adp->cmd[ch]->IDRuserData = isp_boot->extra_fcs_meta_extend_total_size;
			} else {
				v_adp->cmd[ch]->IDRuserData = isp_boot->fcs_meta_total_size;
			}
			hal_video_isp_set_isp_meta_out(ch, 1);
			if (codec & (CODEC_HEVC | CODEC_H264)) {
				v_adp->cmd[ch]->IDRuserDataDuration = 1;
			}
		} else {
			v_adp->cmd[ch]->EncuserData = 0;
		}
	} else {
		if (v_stream->meta_enable == 1 && video_pre_init_param.meta_enable == 1) {
			if (codec & (CODEC_HEVC | CODEC_H264)) {
				v_adp->cmd[ch]->EncuserData = video_pre_init_param.video_meta_total_size;
				if (video_pre_init_param.meta_enable_extend) {
					v_adp->cmd[ch]->IDRuserData = video_pre_init_param.video_meta_extend_total_size;
				} else {
					v_adp->cmd[ch]->IDRuserData = video_pre_init_param.video_meta_total_size;
				}
				v_adp->cmd[ch]->IDRuserDataDuration = 1;

				video_dprintf(VIDEO_LOG_INF, "EncuserData %d IDRuserData %d IDRuserDataDuration %d\r\n", v_adp->cmd[ch]->EncuserData, v_adp->cmd[ch]->IDRuserData,
							  v_adp->cmd[ch]->IDRuserDataDuration);
			}
			if (codec & CODEC_JPEG) {
				v_adp->cmd[ch]->JPGuserData = video_pre_init_param.video_meta_total_size;
			}
			hal_video_isp_set_isp_meta_out(ch, 1);
		} else {
			v_adp->cmd[ch]->EncuserData = 0;
		}
	}
	if ((codec & CODEC_JPEG) && v_stream->jpeg_crop_parm.enable) {
		int jpeg_horOffsetSrc = (v_stream->jpeg_crop_parm.xmin + 1) & ~1;//force 2 align
		int jpeg_verOffsetSrc = (v_stream->jpeg_crop_parm.ymin + 1) & ~1;//force 2 akign
		int jpeg_width  = ((v_stream->jpeg_crop_parm.xmax - v_stream->jpeg_crop_parm.xmin) + 15) & ~15; //force 16 aligned
		int jpeg_height = ((v_stream->jpeg_crop_parm.ymax - v_stream->jpeg_crop_parm.ymin) + 3) & ~3; //force 4 aligned
		v_adp->cmd[ch]->horOffsetSrc = jpeg_horOffsetSrc;
		v_adp->cmd[ch]->verOffsetSrc = jpeg_verOffsetSrc;
		v_adp->cmd[ch]->width = jpeg_width;
		v_adp->cmd[ch]->height = jpeg_height;
		if (((v_stream->jpeg_crop_parm.xmin % 2) != 0) || ((v_stream->jpeg_crop_parm.ymin % 2) != 0)) {
			video_dprintf(VIDEO_LOG_MSG, "The starting point needs to be 2 alignment\r\n");
		}
		if ((jpeg_width % 16) != 0 || (jpeg_height % 4)) {
			video_dprintf(VIDEO_LOG_MSG, "The width needs to align to 16 and the height to 4\r\n");
		}
	}
	video_dprintf(VIDEO_LOG_INF, "hal_video_open\r\n");
	status = hal_video_open(ch);
	if (status != OK) {
		video_dprintf(VIDEO_LOG_ERR, "hal_video_open fail ret=%x, group=%d\r\n", status, video_get_error_group(status));
		goto EXIT;
	}

	voe_info.ch_info[ch].stream_is_open = 1;
	video_get_voe_version();
#if FASTOSD_EN
	if (v_stream->fast_osd_en) {
		rts_osd_fast_start_enable(ch, 0);
	}
#endif
EXIT:

	if (status != OK) {
		video_rc_deinit(ch);
	}

	rtw_mutex_put(&video_open_close_mutex);
	return status;
}



int video_close(int ch)
{
	while (voe_info.ch_info[ch].incb) {
		vTaskDelay(1);
	}
	rtw_mutex_get(&video_open_close_mutex);
	int status = OK;
	voe_info.ch_info[ch].stream_is_open = 0;
	voe_info.ch_info[ch].param = NULL;
	hal_video_set_fps(0, ch);
	video_dprintf(VIDEO_LOG_INF, "hal_video_close\r\n");
	status = hal_video_close(ch);
	if (status != OK) {
		video_dprintf(VIDEO_LOG_ERR, "hal_video_close fail ret=%x, group=%d\r\n", status, video_get_error_group(status));
		goto EXIT;
	}

	osDelay(10); // To idle task clean task usage memory
	// Reset the time offset when the video output is stopped
	last_system_ts_ms[ch] = 0;
	last_video_ts_ms[ch] = 0;
	offset_ms[ch] = 0;
	isp_overflow_times[ch] = 0;
	last_isp_ts[ch] = 0;
	isp_ts_initialed_flag[ch] = 0;
	video_rc_deinit(ch);
	video_bps_stbl_ctrl_deinit(ch);
EXIT:
	rtw_mutex_put(&video_open_close_mutex);
	return status;
}

int video_get_stream_info(int id)
{
	return voe_info.ch_info[id].stream_is_open;
}

#include <stdio.h>
#include "vfs.h"
#define TUNING_USB_MODE //If you need to load from sd card, please disable the marco. 
#define TUNING_IQ     0X00
#define TUNING_SENSOR 0X01
static unsigned char *g_uvcd_iq = NULL;
static unsigned char *g_uvcd_sensor = NULL;
void video_set_uvcd_iq(unsigned int addr)
{
	g_uvcd_iq = (unsigned char *)addr;
}
void video_set_uvcd_sensor(unsigned int addr)
{
	g_uvcd_sensor = (unsigned char *)addr;
}
int sd_load_iq_sensor(int index, unsigned char *buf) //Index 0 iq, index 1 sensor
{
	FILE     *m_file;
	struct stat fstat;
	int br = 0;
	vfs_init(NULL);
	vfs_user_register("sd", VFS_FATFS, VFS_INF_SD);
	const char *path = NULL;
	if (index == 0) {
		path = "sd:/iq.bin";
	} else {
		path = "sd:/sensor.bin";
	}
	m_file = fopen(path, "r");
	if (!m_file) {
		video_dprintf(VIDEO_LOG_ERR, "open file %s fail from sd card\r\n", path);
		goto EXIT;
	}
	if (stat(path, &fstat) < 0) {
		video_dprintf(VIDEO_LOG_ERR, "get file %s status fail\r\n", path);
		fclose(m_file);
		goto EXIT;
	}

	br = fread(buf, 1, fstat.st_size, m_file);
	if (br != fstat.st_size) {
		video_dprintf(VIDEO_LOG_ERR, "The length is not correct %d %d\r\n", br, fstat.st_size);
	}
	fclose(m_file);
	return 0;
EXIT:
	return -1;
}

#define SLICE_SIZE (2 * 1024 * 1024)
static void video_clean_invalidate_heap(uint32_t *heap_addr, uint32_t heap_size)
{
	//dcache_clean_invalidate_by_addr((uint32_t *)heap_addr, heap_size);
	uint32_t remain_heap_size = heap_size;
	//clean the slice the heap buffer for clean and invalidate to prevent a long blocking of irq
	while (1) {
		uint32_t cleaned_size = (heap_size - remain_heap_size) / sizeof(uint32_t);
		if (remain_heap_size <= SLICE_SIZE) {
			dcache_clean_invalidate_by_addr((uint32_t *)(heap_addr + cleaned_size), remain_heap_size);
			break;
		} else {
			dcache_clean_invalidate_by_addr((uint32_t *)(heap_addr + cleaned_size), SLICE_SIZE);
			remain_heap_size -= SLICE_SIZE;
		}
	}
}

hal_video_adapter_t *video_init(int iq_start_addr, int sensor_start_addr)
{
	// HW enable & allocation adapter memory
	int res = 0;
	if (video_open_close_mutex == NULL) {
		rtw_mutex_init(&video_open_close_mutex);
	}
	if (hal_voe_ready() != OK) {
		extern int __voe_code_start__[];
		if (!hal_voe_fcs_check_OK()) {
			unsigned char *iq_addr = NULL;
			unsigned char *sensor_addr = NULL;
			iq_addr = video_load_iq(iq_start_addr);
			sensor_addr = video_load_sensor(sensor_start_addr);
#ifdef TUNING_USB_MODE
			if (g_uvcd_iq) {
				if (g_uvcd_iq[0] == 0 && g_uvcd_iq[1] == 0) {
					int *dtmp = (int *)iq_addr;
					memcpy(g_uvcd_iq, iq_addr, FW_IQ_SIZE);
					video_dprintf(VIDEO_LOG_MSG, "[%s] Load default IQ.size:%d 0x%X,	0x%X 0x%X 0x%X.\r\n", __FUNCTION__, *dtmp, *dtmp, dtmp[1], dtmp[2], dtmp[3]);
				} else {
					int *dtmp = (int *)g_uvcd_iq;
					memcpy(iq_addr, g_uvcd_iq, FW_IQ_SIZE);
					video_dprintf(VIDEO_LOG_MSG, "[%s] Load user IQ.size:%d 0x%X,	0x%X 0x%X 0x%X.\r\n", __FUNCTION__, *dtmp, *dtmp, dtmp[1], dtmp[2], dtmp[3]);
				}
			} else {
				video_dprintf(VIDEO_LOG_MSG, "[%s] uvcd iq is null, use default.\r\n", __FUNCTION__);
			}

			if (g_uvcd_sensor) {
				if (g_uvcd_sensor[0] == 0 && g_uvcd_sensor[1] == 0) {
					int *dtmp = (int *)sensor_addr;
					memcpy(g_uvcd_sensor, sensor_addr, FW_SENSOR_SIZE);
					video_dprintf(VIDEO_LOG_MSG, "[%s] Load default SNR.size:%d 0x%X,	0x%X 0x%X 0x%X.\r\n", __FUNCTION__, *dtmp, *dtmp, dtmp[1], dtmp[2], dtmp[3]);
				} else {
					int *dtmp = (int *)g_uvcd_sensor;
					memcpy(sensor_addr, g_uvcd_sensor, FW_SENSOR_SIZE);
					video_dprintf(VIDEO_LOG_MSG, "[%s] Load user SNR.size:%d 0x%X,	0x%X 0x%X 0x%X.\r\n", __FUNCTION__, *dtmp, *dtmp, dtmp[1], dtmp[2], dtmp[3]);
				}
			} else {
				video_dprintf(VIDEO_LOG_MSG, "[%s] uvcd SNR is null, use default.\r\n", __FUNCTION__);
			}
#else
			res = sd_load_iq_sensor(TUNING_IQ, iq_buf);
			if (res < 0) {
				video_dprintf(VIDEO_LOG_MSG, "Use the original IQ\r\n");
			} else {
				iq_addr = iq_buf;
			}
			res = sd_load_iq_sensor(TUNING_SENSOR, sensor_buf);
			if (res < 0) {
				video_dprintf(VIDEO_LOG_MSG, "Use the original SENSOR\r\n");
			} else {
				sensor_addr = sensor_buf;
			}
#endif
			hal_video_load_iq((voe_cpy_t)hal_voe_cpy, (int *)iq_addr, __voe_code_start__);
			hal_video_load_sensor((voe_cpy_t)hal_voe_cpy, (int *)sensor_addr, __voe_code_start__);
			memcpy(snr_voe_ver, sensor_addr, 24);
			video_dprintf(VIDEO_LOG_MSG, "sensor timestamp: %04d/%02d/%02d\r\n", *(unsigned short *)(sensor_addr + 0), sensor_addr[2], sensor_addr[3]);
			video_dprintf(VIDEO_LOG_MSG, "iq timestamp: %04d/%02d/%02d %02d:%02d:%02d\r\n", *(unsigned short *)(iq_addr + 12), iq_addr[14], iq_addr[15], iq_addr[16],
						  iq_addr[17], *(unsigned short *)(iq_addr + 18));
			memcpy(iq_timestamp, iq_addr + 12, 8);
			memcpy(iq_cus_ver, iq_addr + 23, 8);
		}

		if (voe_info_init == 0) {
			memset(voe_info.ch_info, 0, sizeof(voe_info.ch_info));
		}

		if (voe_fw_reload == 1) {
			hal_status_t v_ret;
			video_reld_img_ctrl_info_t reld_info;
			v_ret = hal_sys_get_video_img_ld_offset(&reld_info, VIDEO_VOE_OFFSET);
			if (HAL_OK == v_ret) {
				video_dprintf(VIDEO_LOG_INF, "fwin(%x),enc_en(%x),VOE_OFFSET = 0x%x\r\n", reld_info.fwin, reld_info.enc_en, reld_info.data_start_offset);
				video_load_fw(reld_info.data_start_offset);
			} else {
				video_dprintf(VIDEO_LOG_ERR, "It can't load the VOE fw\r\n");
			}
			voe_fw_reload = 0;
		}

		if (!voe_info.voe_heap_addr) {
			voe_info.voe_heap_addr = (uint32_t)malloc(voe_info.voe_heap_size);
			video_dprintf(VIDEO_LOG_MSG, "voe_heap malloc 0x%x, size %d\n", voe_info.voe_heap_addr, voe_info.voe_heap_size);
			if (!voe_info.voe_heap_addr) {
				video_dprintf(VIDEO_LOG_ERR, "voe_heap malloc fail\n");
				return NULL;
			}
		}

		video_init_peri();

		video_clean_invalidate_heap((uint32_t *)voe_info.voe_heap_addr, voe_info.voe_heap_size);
		res = hal_video_init((uint32_t *)voe_info.voe_heap_addr, voe_info.voe_heap_size);
		if (res != OK) {
			video_dprintf(VIDEO_LOG_ERR, "hal_video_init fail\n");
			return NULL;
		}

		video_dprintf(VIDEO_LOG_INF, "!hal_voe_ready\r\n");

	}
	if (isp_boot->fcs_status == 1 && video_open_status() == 0) {
		unsigned char *sensor_addr = NULL;
		unsigned char *iq_addr = NULL;
		sensor_addr = video_load_sensor(sensor_start_addr);
		memcpy(snr_voe_ver, sensor_addr, 24);
		iq_addr = video_load_iq(iq_start_addr);
		video_dprintf(VIDEO_LOG_MSG, "sensor timestamp: %04d/%02d/%02d\r\n", *(unsigned short *)(sensor_addr + 0), sensor_addr[2], sensor_addr[3]);
		video_dprintf(VIDEO_LOG_MSG, "iq timestamp: %04d/%02d/%02d %02d:%02d:%02d\r\n", *(unsigned short *)(iq_addr + 12), iq_addr[14], iq_addr[15], iq_addr[16],
					  iq_addr[17], *(unsigned short *)(iq_addr + 18));
		memcpy(iq_timestamp, iq_addr + 12, 8);
		memcpy(iq_cus_ver, iq_addr + 23, 8);
		voe_info.voe_heap_addr = isp_boot->voe_heap_addr;
		voe_info.voe_heap_size = isp_boot->voe_heap_size;
		video_dprintf(VIDEO_LOG_INF, "voe_info.voe_heap_addr %x voe_info.voe_heap_size %x\r\n", voe_info.voe_heap_addr, voe_info.voe_heap_size);
		video_init_peri();
	}
#if VIDEO_MPU_VOE_HEAP
	setup_mpu((uint32_t)voe_info.voe_heap_addr, (uint32_t)voe_info.voe_heap_addr + voe_info.voe_heap_size - 1, 1, 1);
#endif
	hal_video_adapter_t *v_adp = hal_video_get_adp();

	return 	v_adp;
}

void *video_deinit(void)
{
	if (hal_voe_ready() == OK) {
		video_deinit_peri();

		video_dprintf(VIDEO_LOG_INF, "video_deinit\r\n");
		hal_video_deinit();

		rtw_mdelay_os(50);
		voe_fw_reload = 1;

		voe_info_init = 0;

		isp_boot->fcs_status = 0;
	}

	return NULL;
}

void *video_poweroff(void)
{
	if (hal_voe_ready() == OK) {
		dbg_printf("video_poweroff start\r\n");
		video_deinit_peri();
		dbg_printf("video_poweroff end\r\n");
	}

	return NULL;
}

//////////////////////////////////////////////
void voe_set_iq_sensor_fw(int id)
{
	extern int __voe_code_start__[];

	int iq_data = 0;
	int sensor_data = 0;
	voe_get_sensor_info(id, &iq_data, &sensor_data);
	video_dprintf(VIDEO_LOG_INF, "ch %d iq_data %x sensor_data %x\r\n", id, iq_data, sensor_data);
	unsigned char *iq_addr = NULL;
	unsigned char *sensor_addr = NULL;
	iq_addr = video_load_iq(iq_data);
	sensor_addr = video_load_sensor(sensor_data);
	if (iq_addr == NULL || sensor_addr == NULL) {
		video_dprintf(VIDEO_LOG_ERR, "It can't allocate the buffer\r\n");
		return;
	}

#ifdef TUNING_USB_MODE
	if (g_uvcd_iq) {
		if (g_uvcd_iq[0] == 0 && g_uvcd_iq[1] == 0) {
			int *dtmp = (int *)iq_addr;
			memcpy(g_uvcd_iq, iq_addr, FW_IQ_SIZE);
			video_dprintf(VIDEO_LOG_MSG, "[%s] Load default IQ.size:%d 0x%X,	0x%X 0x%X 0x%X.\r\n", __FUNCTION__, *dtmp, *dtmp, dtmp[1], dtmp[2], dtmp[3]);
		} else {
			int *dtmp = (int *)g_uvcd_iq;
			memcpy(iq_addr, g_uvcd_iq, FW_IQ_SIZE);
			video_dprintf(VIDEO_LOG_MSG, "[%s] Load user IQ.size:%d 0x%X,	0x%X 0x%X 0x%X.\r\n", __FUNCTION__, *dtmp, *dtmp, dtmp[1], dtmp[2], dtmp[3]);
		}
		//hal_video_load_iq((voe_cpy_t)hal_voe_cpy, iq_addr, __voe_code_start__);
	} else {
		video_dprintf(VIDEO_LOG_MSG, "[%s] uvcd iq is null, use default.\r\n", __FUNCTION__);
	}
	if (g_uvcd_sensor) {
		if (g_uvcd_sensor[0] == 0 && g_uvcd_sensor[1] == 0) {
			int *dtmp = (int *)sensor_addr;
			memcpy(g_uvcd_sensor, sensor_addr, FW_SENSOR_SIZE);
			video_dprintf(VIDEO_LOG_MSG, "[%s] Load default SNR.size:%d 0x%X,	0x%X 0x%X 0x%X.\r\n", __FUNCTION__, *dtmp, *dtmp, dtmp[1], dtmp[2], dtmp[3]);
		} else {
			int *dtmp = (int *)g_uvcd_sensor;
			memcpy(sensor_addr, g_uvcd_sensor, FW_SENSOR_SIZE);
			video_dprintf(VIDEO_LOG_MSG, "[%s] Load user SNR.size:%d 0x%X,	0x%X 0x%X 0x%X.\r\n", __FUNCTION__, *dtmp, *dtmp, dtmp[1], dtmp[2], dtmp[3]);
		}
	} else {
		video_dprintf(VIDEO_LOG_MSG, "[%s] uvcd SNR is null, use default.\r\n", __FUNCTION__);
	}
#else
	int res = 0;
	res = sd_load_iq_sensor(TUNING_IQ, iq_buf);
	if (res < 0) {
		video_dprintf(VIDEO_LOG_MSG, "Use the original IQ\r\n");
	} else {
		iq_addr = iq_buf;
	}
	res = sd_load_iq_sensor(TUNING_SENSOR, sensor_buf);
	if (res < 0) {
		video_dprintf(VIDEO_LOG_MSG, "Use the original SENSOR\r\n");
	} else {
		sensor_addr = sensor_buf;
	}
#endif
	hal_video_load_iq((voe_cpy_t)hal_voe_cpy, (int *)iq_addr, __voe_code_start__);
	hal_video_load_sensor((voe_cpy_t)hal_voe_cpy, (int *)sensor_addr, __voe_code_start__);
	memcpy(snr_voe_ver, sensor_addr, 24);
	video_dprintf(VIDEO_LOG_MSG, "sensor timestamp: %04d/%02d/%02d\r\n", *(unsigned short *)(sensor_addr + 0), sensor_addr[2], sensor_addr[3]);
	video_dprintf(VIDEO_LOG_MSG, "iq timestamp: %04d/%02d/%02d %02d:%02d:%02d\r\n", *(unsigned short *)(iq_addr + 12), iq_addr[14], iq_addr[15], iq_addr[16],
				  iq_addr[17], *(unsigned short *)(iq_addr + 18));
	memcpy(iq_timestamp, iq_addr + 12, 8);
	memcpy(iq_cus_ver, iq_addr + 23, 8);
}

int voe_get_sensor_info(int id, int *iq_data, int *sensor_data)
{
	int ret = 0;
	int sensor_id = id + 1;
#if NONE_FCS_MODE
	video_get_fw_isp_info();
#endif

	if (id >= isp_boot->p_fcs_ld_info.multi_fcs_cnt) {
		ret = -1;
		video_dprintf(VIDEO_LOG_ERR, "sensor id is bigger than isp_boot->p_fcs_ld_info.multi_fcs_cnt\r\n");
	} else {
		hal_status_t v_ret;
		video_reld_img_ctrl_info_t reld_info;
		for (int i = 1; i <= 2; i++) {
			v_ret = hal_sys_get_video_img_ld_offset(&reld_info, ((sensor_id << 4) | i));
			if (HAL_OK == v_ret) {
				if (VIDEO_ISP_SENSOR_IQ == i) {
					video_dprintf(VIDEO_LOG_MSG, "fwin(%x),enc_en(%x),IQ_OFFSET = 0x%x\r\n ", reld_info.fwin, reld_info.enc_en, reld_info.data_start_offset);
					*iq_data = reld_info.data_start_offset;
				} else if (VIDEO_ISP_SENSOR_DATA == i) {
					video_dprintf(VIDEO_LOG_MSG, "fwin(%x),enc_en(%x),SENSOR_OFFSET = 0x%x\r\n", reld_info.fwin, reld_info.enc_en, reld_info.data_start_offset);
					*sensor_data  = reld_info.data_start_offset;
				}
			} else {
				video_dprintf(VIDEO_LOG_ERR, "It can't load the sensor fw\r\n");
				goto EXIT;
			}
		}
		video_dprintf(VIDEO_LOG_MSG, "sensor id %d iq_data %x sensor_data %x\r\n", id, *iq_data, *sensor_data);
	}
EXIT:
	return ret;
}

void voe_dump_isp_info(void)
{
	//video_boot_stream_t *isp_boot = (video_boot_stream_t *)bl4voe_shared.data;
	int i, j = 0;
	unsigned int p_fcs_data, p_iq_data, p_sensor_data = 0;
	video_dprintf(VIDEO_LOG_INF, "isp_boot->fcs_voe_fw_addr %x\r\n", isp_boot->fcs_voe_fw_addr);
	video_dprintf(VIDEO_LOG_INF, "fcs_id %d\r\n", isp_boot->p_fcs_ld_info.fcs_id);
	video_dprintf(VIDEO_LOG_INF, "multi_fcs_cnt %d\r\n", isp_boot->p_fcs_ld_info.multi_fcs_cnt);
	video_dprintf(VIDEO_LOG_INF, "magic %x\r\n", isp_boot->p_fcs_ld_info.magic);
	video_dprintf(VIDEO_LOG_INF, "version %d\r\n", isp_boot->p_fcs_ld_info.version);
	video_dprintf(VIDEO_LOG_INF, "wait_km_init_timeout_us %d\r\n", isp_boot->p_fcs_ld_info.wait_km_init_timeout_us);
	video_dprintf(VIDEO_LOG_INF, "fcs_hdr_start %x\r\n", isp_boot->p_fcs_ld_info.fcs_hdr_start);
	//video_dprintf(VIDEO_LOG_INF, "ispiq_img_offset %x\r\n", isp_boot->p_fcs_ld_info.ispiq_img_offset);
	for (i = 0; i < isp_boot->p_fcs_ld_info.multi_fcs_cnt; i++) {
		video_dprintf(VIDEO_LOG_INF, "sesnor %d fcs_data_size %x\r\n", i, isp_boot->p_fcs_ld_info.sensor_set[i].fcs_data_size);
		video_dprintf(VIDEO_LOG_INF, "sesnor %d fcs_data_offset %x\r\n", i, isp_boot->p_fcs_ld_info.sensor_set[i].fcs_data_offset);
		video_dprintf(VIDEO_LOG_INF, "sesnor %d iq_start_addr %x\r\n", i, isp_boot->p_fcs_ld_info.sensor_set[i].iq_start_addr);
		video_dprintf(VIDEO_LOG_INF, "sesnor %d iq_data_size %x\r\n", i, isp_boot->p_fcs_ld_info.sensor_set[i].iq_data_size);
		video_dprintf(VIDEO_LOG_INF, "sesnor %d sensor_start_addr %x\r\n", i, isp_boot->p_fcs_ld_info.sensor_set[i].sensor_start_addr);
		video_dprintf(VIDEO_LOG_INF, "sesnor %d sensor_data_size %x\r\n", i, isp_boot->p_fcs_ld_info.sensor_set[i].sensor_data_size);
		p_fcs_data    = (unsigned int)(isp_boot->p_fcs_ld_info.fcs_hdr_start + (isp_boot->p_fcs_ld_info.sensor_set[i].fcs_data_offset));
		p_iq_data     = (unsigned int)(p_fcs_data + (isp_boot->p_fcs_ld_info.sensor_set[i].iq_start_addr));
		p_sensor_data = (unsigned int)(p_fcs_data + (isp_boot->p_fcs_ld_info.sensor_set[i].sensor_start_addr));
		video_dprintf(VIDEO_LOG_INF, "sensor %d p_fcs_data %x p_iq_data %x p_sensor_data %x\r\n", i, p_fcs_data, p_iq_data, p_sensor_data);
	}
	video_dprintf(VIDEO_LOG_INF, "isp_boot->fcs_voe_fw_addr %x\r\n", isp_boot->fcs_voe_fw_addr);

	hal_status_t v_ret;
	video_reld_img_ctrl_info_t reld_info;
	for (j = 0; j < isp_boot->p_fcs_ld_info.multi_fcs_cnt; j++) {
		video_dprintf(VIDEO_LOG_MSG, "sensor_set(%d)\r\n", j);
		for (i = 1; i <= 2; i++) {
			v_ret = hal_sys_get_video_img_ld_offset(&reld_info, ((j << 4) | i));
			if (HAL_OK == v_ret) {
				if (VIDEO_ISP_SENSOR_IQ == i) {
					video_dprintf(VIDEO_LOG_MSG, "fwin(%x),enc_en(%x),IQ_OFFSET = 0x%x\r\n ", reld_info.fwin, reld_info.enc_en, reld_info.data_start_offset);
				} else if (VIDEO_ISP_SENSOR_DATA == i) {
					video_dprintf(VIDEO_LOG_MSG, "fwin(%x),enc_en(%x),SENSOR_OFFSET = 0x%x\r\n", reld_info.fwin, reld_info.enc_en, reld_info.data_start_offset);
				}
			}
		}
	}
	v_ret = hal_sys_get_video_img_ld_offset(&reld_info, VIDEO_VOE_OFFSET);
	if (HAL_OK == v_ret) {
		video_dprintf(VIDEO_LOG_MSG, "fwin(%x),enc_en(%x),VOE_OFFSET = 0x%x\r\n", reld_info.fwin, reld_info.enc_en, reld_info.data_start_offset);
	}

}

void voe_t2ff_prealloc(void)
{
	//video_boot_stream_t *isp_boot = (video_boot_stream_t *)bl4voe_shared.data;
	unsigned char *ptr = NULL;
	if (isp_boot->fcs_status) {
		//printf("isp_boot %d %d %d\r\n",isp_boot->video_params[0].width,isp_boot->video_params[0].height,isp_boot->fcs_status);
		video_dprintf(VIDEO_LOG_INF, "heap addr %x heap size %x\r\n", isp_boot->voe_heap_addr, isp_boot->voe_heap_size);
		if (isp_boot->voe_heap_size > (__eram_end__ - __eram_heap_start__)) {
			video_dprintf(VIDEO_LOG_ERR, "The voe buffer is not enough\r\n");
			abort();
		} else {
			ptr = (unsigned char *) pvPortInsertPreAlloc((void *)isp_boot->voe_heap_addr, isp_boot->voe_heap_size);
			if (ptr == NULL) {
				video_dprintf(VIDEO_LOG_ERR, "It can't be allocate buffer\r\n");
				abort();
			}
		}
	}
	//voe_dump_isp_info();
}

int voe_boot_fsc_status(void)// 1 : Inited 0: Non-Inited
{
	//video_dprintf(VIDEO_LOG_INF, "isp_boot->fcs_status %d\r\n", isp_boot->fcs_status);
	return isp_boot->fcs_status;
}

int voe_boot_fsc_id(void)//
{
	video_dprintf(VIDEO_LOG_INF, "isp_boot->fcs_id %d\r\n", isp_boot->p_fcs_ld_info.fcs_id);
	return isp_boot->p_fcs_ld_info.fcs_id;
}

uint8_t isp_i2c_read_byte(int addr)
{
	struct rts_isp_i2c_reg reg;
	int ret = 0;
	reg.addr = addr;
	reg.data = 0;
	ret = hal_video_i2c_read(&reg);
	return reg.data;
}

int video_set_fcs_id(int SensorName)
{
	int ret = -1;
	unsigned int crc32_result[10] = {0xd202ef8d, 0xa505df1b, 0x3c0c8ea1, 0x4b0bbe37, 0xd56f2b94, 0xa2681b02, 0x3b614ab8, 0x4c667a2e, 0xdcd967bf, 0xabde5729};
	pfw_init();

	void *fp = pfw_open_by_typeid(0x89CE, M_MANI_UNPT);
	uint8_t *ref = malloc(32);
	uint8_t *tmp = malloc(32);
	if (fp == NULL || ref == NULL || tmp == NULL) {
		video_dprintf(VIDEO_LOG_ERR, "It can't get the memory\r\n");
		goto exit_set_fcs_id;
	}
	pfw_read_unpt(fp, ref, 12);

	ref[4] = SensorName;
	*(uint32_t *)&ref[8] = crc32_result[ref[4]];

	pfw_write_unpt(fp, ref, 12);

	pfw_read_unpt(fp, tmp, 12);

	ret = 0;
exit_set_fcs_id:
	if (fp) {
		pfw_close(fp);
	}
	if (ref) {
		free(ref);
	}
	if (tmp) {
		free(tmp);
	}
	return ret;
}

int video_fcs_write_sensor_id(int SensorName)
{
	if (isp_boot->p_fcs_ld_info.fcs_id == 0) {
		return -1;
	} else {
		if (SensorName != isp_boot->p_fcs_ld_info.fcs_id) {
			//if(video_set_fcs_id(SensorName) < 0){ //Don't support now
			//	return -1;
			//}
		}
		return 0;
	}
}

void video_get_video_snesor_info(mult_sensor_info_t *info)
{
	info->sensor_index = multi_sensor_info.sensor_index;
	info->sensor_finish =  multi_sensor_info.sensor_finish;
}

void video_set_video_snesor_info(mult_sensor_info_t *info)
{
	multi_sensor_info.sensor_index = info->sensor_index;
	multi_sensor_info.sensor_finish = info->sensor_finish;
}

int video_get_video_sensor_status(void)
{
	return multi_sensor_info.sensor_finish;
}

int video_get_video_sensor_index(void)
{
	return multi_sensor_info.sensor_index;
}

int video_load(int sensor_index)
{


	/* 	if (!hal_voe_fcs_check_OK()) {
			// Enable ISP PWR
			hal_gpio_init(&sensor_en_gpio, PIN_A5);
			hal_gpio_set_dir(&sensor_en_gpio, GPIO_OUT);
			hal_gpio_write(&sensor_en_gpio, 1);
		} */
	video_dprintf(VIDEO_LOG_INF, "sensor_index %d\r\n", sensor_index);
	voe_set_iq_sensor_fw(sensor_index);
	// extern int _binary_voe_bin_start[];			// VOE binary address
	// extern int __voe_code_start__[];
	// int *voe_bin_addr = (int *)isp_boot->fcs_voe_fw_addr;
	hal_status_t v_ret;
	video_reld_img_ctrl_info_t reld_info;
	v_ret = hal_sys_get_video_img_ld_offset(&reld_info, VIDEO_VOE_OFFSET);
	if (HAL_OK == v_ret) {
		video_dprintf(VIDEO_LOG_INF, "fwin(%x),enc_en(%x),VOE_OFFSET = 0x%x\r\n", reld_info.fwin, reld_info.enc_en, reld_info.data_start_offset);
		video_load_fw(reld_info.data_start_offset);
	} else {
		video_dprintf(VIDEO_LOG_ERR, "It can't find the voe fw\r\n");
		return -1;
	}
	//hal_video_load_fw((voe_cpy_t)hal_voe_cpy, _binary_voe_bin_start, __voe_code_start__);
	video_dprintf(VIDEO_LOG_INF, "voe_info.voe_heap_addr %x voe_heap_size %x\r\n", voe_info.voe_heap_addr, voe_info.voe_heap_size);
	video_init_peri();
	if (hal_video_init((uint32_t *)voe_info.voe_heap_addr, voe_info.voe_heap_size) != OK) {
		video_dprintf(VIDEO_LOG_ERR, "hal_video_init fail\n");
		return -1;
	}

	return OK;
}

void *video_fw_deinit(void)
{
	if (hal_voe_ready() == OK) {
		video_dprintf(VIDEO_LOG_INF, "video_deinit\r\n");
		hal_gpio_write(&sensor_en_gpio, 0);

		video_deinit_peri();
		hal_video_deinit();
		// Disable ISP PWR
		//hal_gpio_write(&sensor_en_gpio, 0);
		//hal_gpio_deinit(&sensor_en_gpio);
		rtw_mdelay_os(50);
	}
	return NULL;
}

int video_reset_fw(int ch, int id)
{
	int ret = 0;
	video_close(ch);
	video_fw_deinit();
	video_load(id);
	return ret;
}

unsigned char *video_load_iq(unsigned int iq_start_addr)
{
	unsigned int iq_full_size = 0;
	void *fp = NULL;
	if (hal_sys_get_ld_fw_idx() == FW_1) {
		fp = pfw_open("FW1", M_RAW);
		if (!fp) {
			video_dprintf(VIDEO_LOG_ERR, "cannot open file FW1\n\r");
			return NULL;
		}
	} else if (hal_sys_get_ld_fw_idx() == FW_2) {
		fp = pfw_open("FW2", M_RAW);
		if (!fp) {
			video_dprintf(VIDEO_LOG_ERR, "cannot open file FW2\n\r");
			return NULL;
		}
	} else {
		return NULL;
	}

	iq_full_size = FW_IQ_SIZE;//default iq size
	pfw_seek(fp, iq_start_addr, SEEK_SET);
	pfw_read(fp, iq_buf, iq_full_size);
	pfw_close(fp);
	return iq_buf;
}

unsigned char *video_load_sensor(unsigned int sensor_start_addr)
{
	unsigned int sensor_size = 0;
	void *fp = NULL;
	if (hal_sys_get_ld_fw_idx() == FW_1) {
		fp = pfw_open("FW1", M_RAW);
		if (!fp) {
			video_dprintf(VIDEO_LOG_ERR, "cannot open file FW1\n\r");
			return NULL;
		}
	} else if (hal_sys_get_ld_fw_idx() == FW_2) {
		fp = pfw_open("FW2", M_RAW);
		if (!fp) {
			video_dprintf(VIDEO_LOG_ERR, "cannot open file FW2\n\r");
			return NULL;
		}
	} else {
		return NULL;
	}

	sensor_size = FW_SENSOR_SIZE;//Default size for sensor fw
	pfw_seek(fp, sensor_start_addr, SEEK_SET);
	pfw_read(fp, sensor_buf, sensor_size);
	pfw_close(fp);
	return sensor_buf;
}

static unsigned char *video_load_cal_iq(unsigned int cal_iq_start_addr)
{
	int *calib_iq_size = (int *)(cal_iq_buf + 4);
	struct isp_iq_cali *piq_cali_data = (struct isp_iq_cali *)(cal_iq_buf + 8);
#if CALIB_IQ_LOAD_FLASH
	ftl_common_read(cal_iq_start_addr, cal_iq_buf, 8 + sizeof(struct isp_iq_cali));
#else
	cal_iq_buf[0] = 'C';
	cal_iq_buf[1] = 'A';
	cal_iq_buf[2] = 'I';
	cal_iq_buf[3] = 'Q';
	*calib_iq_size = sizeof(struct isp_iq_cali);
	piq_cali_data->enable = IQ_CALI_EN_AWB | IQ_CALI_EN_MLSC | IQ_CALI_EN_NLSC;
	piq_cali_data->awb.x_offset = -2;
	piq_cali_data->awb.y_offset = 2;
	piq_cali_data->nlsc.r_center.x = 952;
	piq_cali_data->nlsc.r_center.y = 532;
	piq_cali_data->nlsc.g_center.x = 960;
	piq_cali_data->nlsc.g_center.y = 540;
	piq_cali_data->nlsc.b_center.x = 968;
	piq_cali_data->nlsc.b_center.y = 548;

	memset((void *) & (piq_cali_data->mlsc.matrix_r[0]), 0x84, 1536);
	memset((void *) & (piq_cali_data->mlsc.matrix_g[0]), 0x80, 1536);
	memset((void *) & (piq_cali_data->mlsc.matrix_b[0]), 0x88, 1536);
#endif
	return cal_iq_buf;
}

int video_get_voe_version()
{
	unsigned int fw_size = 0;
	void *fp = NULL;
	video_reld_img_ctrl_info_t reld_info;
	hal_status_t v_ret;
	v_ret = hal_sys_get_video_img_ld_offset(&reld_info, VIDEO_VOE_OFFSET);

	if (hal_sys_get_ld_fw_idx() == FW_1) {
		fp = pfw_open("FW1", M_RAW);
		if (!fp) {
			video_dprintf(VIDEO_LOG_ERR, "cannot open file FW1\n\r");
			return -1;
		}
	} else if (hal_sys_get_ld_fw_idx() == FW_2) {
		fp = pfw_open("FW2", M_RAW);
		if (!fp) {
			video_dprintf(VIDEO_LOG_ERR, "cannot open file FW2\n\r");
			return -1;
		}
	} else {
		return -1;
	}

	fw_size = 32; //Default voe fw size
	unsigned char *buf = malloc(fw_size);
	if (buf == NULL) {
		video_dprintf(VIDEO_LOG_ERR, "It don't the enough memory %s\r\n", __FUNCTION__);
		return -1;
	}
	pfw_seek(fp, reld_info.data_start_offset, SEEK_SET);
	pfw_read(fp, buf, fw_size);
	pfw_close(fp);
	dcache_clean_invalidate_by_addr((uint32_t *)buf, (int32_t)fw_size);

	memcpy(voe_ver, buf + 4, 24);
	if (buf) {
		free(buf);
	}
	return 0;
}

void video_get_version()
{
	char version[] = {0xff, 0xff, 0xff, 0xff};
	video_get_voe_version();

	version[0] = voe_ver[13] - 48;
	version[1] = voe_ver[15] - 48;
	version[2] = voe_ver[17] - 48;
	version[3] = voe_ver[19] - 48;
	printf("voe_ver: %d.%d.%d.%d \r\n", version[0], version[1], version[2], version[3]);

	version[0] = snr_voe_ver[17] - 48;
	version[1] = snr_voe_ver[19] - 48;
	version[2] = snr_voe_ver[21] - 48;
	version[3] = snr_voe_ver[23] - 48;
	printf("sensor_voe_ver: %d.%d.%d.%d \r\n", version[0], version[1], version[2], version[3]);
	printf("sensor_timestamp: %04d/%02d/%02d \r\n", *(unsigned short *)(snr_voe_ver + 0), snr_voe_ver[2], snr_voe_ver[3]);

	printf("fcs_version: 0x%04X \r\n", fcs_version);
	printf("iq_timestamp: %04d/%02d/%02d %02d:%02d:%02d\r\n", *(unsigned short *)(iq_timestamp + 0), iq_timestamp[2], iq_timestamp[3], iq_timestamp[4],
		   iq_timestamp[5], *(unsigned short *)(iq_timestamp + 6));
	printf("iq_cus_ver: 0x%02X \r\n", iq_cus_ver[0]);
}

int video_load_fw(unsigned int sensor_start_addr)
{
	unsigned int fw_size = 0;
	void *fp = NULL;
	if (hal_sys_get_ld_fw_idx() == FW_1) {
		fp = pfw_open("FW1", M_RAW);
		if (!fp) {
			video_dprintf(VIDEO_LOG_ERR, "cannot open file FW1\n\r");
			return -1;
		}
	} else if (hal_sys_get_ld_fw_idx() == FW_2) {
		fp = pfw_open("FW2", M_RAW);
		if (!fp) {
			video_dprintf(VIDEO_LOG_ERR, "cannot open file FW2\n\r");
			return -1;
		}
	} else {
		return -1;
	}

	fw_size = FW_VOE_SIZE; //Default voe fw size
	unsigned char *buf = malloc(fw_size);
	if (buf == NULL) {
		video_dprintf(VIDEO_LOG_ERR, "It don't the enough memory %s\r\n", __FUNCTION__);
		return -1;
	}
	pfw_seek(fp, sensor_start_addr, SEEK_SET);
	pfw_read(fp, buf, fw_size);
	pfw_close(fp);
	dcache_clean_invalidate_by_addr((uint32_t *)buf, (int32_t)fw_size);
	hal_video_load_fw((voe_cpy_t)hal_voe_cpy, (int *)buf, __voe_code_start__);
	if (buf) {
		free(buf);
	}
	return 0;
}

void video_get_fcs_info(void *isp_fcs_info)
{
	video_boot_stream_t **_isp_fcs_info = (video_boot_stream_t **)isp_fcs_info;
	*_isp_fcs_info = isp_boot;
}

void video_set_fcs_queue_info(int start_time, int end_time)
{
	fcs_queue_start_time = start_time;
	fcs_queue_end_time = end_time;
	video_dprintf(VIDEO_LOG_INF, "fcs start_time %d end_time %d\r\n", fcs_queue_start_time, fcs_queue_end_time);
}

void video_get_fcs_queue_info(int *start_time, int *end_time)
{
	*start_time = fcs_queue_start_time;
	*end_time = fcs_queue_end_time;
	video_dprintf(VIDEO_LOG_INF, "fcs start_time %d end_time %d\r\n", fcs_queue_start_time, fcs_queue_end_time);
}

int video_get_fcs_cost_time(void)
{
	return isp_info.frame_done_time;
}

static void isp_set_dn_initial_mode(int ch, int mode) //0 day mode 1 night mode
{
	/*
	---------------------------------------------
	|  description | mode  |  iq index | color  |
	---------------------------------------------
	|    RGB mode  |   0   |     0     |  Yes   |
	---------------------------------------------
	|    IR mode   |   1   |     1     |  No    |
	---------------------------------------------
	|    SP mode   |   2   |     2     |  Yes   |
	---------------------------------------------
	init_iq_mode.
	0 - disable these initial iq parameters setting
	1 - Valid with stream open after all streams closed.
	2 - Only valid 1st steam open after voe turn on
	*/
	if (isp_boot->fcs_status && isp_boot->fcs_setting_done == 0) {
		video_dprintf(VIDEO_LOG_ERR, "It is fcs mode, don't support\r\n");
	} else {
		if ((mode <= 2) && (mode >= 0)) {
			hal_video_isp_set_init_iq_mode(ch, 1);
			hal_video_isp_set_dn_mode(ch, mode);
			if (mode == 1) {
				hal_video_isp_set_gray_mode(ch, 1);
			} else {
				hal_video_isp_set_gray_mode(ch, 0);
			}
			video_dprintf(VIDEO_LOG_INF, "Set isp mode %d\r\n", mode);
		} else {
			video_dprintf(VIDEO_LOG_ERR, "Don't support mode %d\r\n", mode);
		}
	}
}

char *video_base64_encode(unsigned char *data_in, int input_length, int *output_length, char *data_out, int data_out_len);
int video_get_sps_pps(unsigned char *frame_buf, unsigned int frame_size, int ch, video_sps_pps_info_t *info)
{
	unsigned int sps_offset = 0;
	unsigned int pps_offset = 0;
	unsigned char type = 0;
	unsigned int frame_offset = 0;
	unsigned int nal_len = 4;
	hal_video_adapter_t *v_adp = hal_video_get_adp();
	for (int i = 0; i < frame_size ; i++) {
		if (frame_buf[i] == 0x00 && frame_buf[i + 1] == 0x00) {
			if ((frame_buf[i + 2] == 0x00 && frame_buf[i + 3] == 0x01)) {
				type = frame_buf[i + nal_len] & 0x1f;
				if (type == 0x08) {
					pps_offset = i;
					i += nal_len;
				} else {
					frame_offset = i;
					break;
				}
			}
		}
	}

	info->sps_len = pps_offset - sps_offset;
	info->pps_len = frame_offset - pps_offset - nal_len;
	if (info->sps_len <= sizeof(info->sps)) {
		memcpy((char *)info->sps, (char *)(frame_buf), info->sps_len);
	} else {
		video_dprintf(VIDEO_LOG_ERR, "The sps length exceed the frame_buf\r\n");
		return -1;
	}
	if (info->pps_len <= sizeof(info->pps)) {
		memcpy((char *)info->pps, (char *)(frame_buf + pps_offset + nal_len), info->pps_len);
	} else {
		video_dprintf(VIDEO_LOG_ERR, "The sps length exceed the frame_buf\r\n");
		return -1;
	}
	int output_len = 0;
	memset(info->sps_base64, 0x00, sizeof(info->sps_base64));
	memset(info->pps_base64, 0x00, sizeof(info->pps_base64));
	memset(info->profile_level_id, 0x00, sizeof(info->profile_level_id));
	video_base64_encode(info->sps, info->sps_len, &output_len, info->sps_base64, sizeof(info->sps_base64));
	video_base64_encode(info->pps, info->pps_len, &output_len, info->pps_base64, sizeof(info->pps_base64));
	sprintf(info->profile_level_id, "%02x%02x%02x", info->sps[1], info->sps[2], info->sps[3]);
	video_dprintf(VIDEO_LOG_INF, "sps %s pps %s profile %s\r\n", info->sps_base64, info->pps_base64, info->profile_level_id);
	return 0;
}

int video_get_sps_pps_vps(unsigned char *frame_buf, unsigned int frame_size, int ch, video_sps_pps_info_t *info)
{
	unsigned int sps_offset = 0;
	unsigned int pps_offset = 0;
	unsigned int vps_offset = 0;
	unsigned char type = 0;
	unsigned int nal_len = 4;
	for (unsigned int i = 0; i < frame_size ; i++) {
		if (frame_buf[i] == 0x00 && frame_buf[i + 1] == 0x00) {
			if ((frame_buf[i + 2] == 0x00 && frame_buf[i + 3] == 0x01)) {
				type = frame_buf[i + nal_len];
				if (type == 0x42) {	//SPS
					vps_offset = i;
					i += nal_len;
				} else if (type == 0x44) { //PPS
					sps_offset = i;
					i += nal_len;
				} else {
					pps_offset = i;
					break;
				}
			}
		}
	}
	info->vps_len = vps_offset;
	info->sps_len = sps_offset - vps_offset - nal_len;
	info->pps_len = pps_offset - sps_offset - nal_len;

	if (info->vps_len <= sizeof(info->vps)) {
		memcpy((char *)info->vps, (char *)(frame_buf), info->vps_len);
	} else {
		video_dprintf(VIDEO_LOG_ERR, "The vps length exceed the buffer\r\n");
		return -1;
	}
	if (info->sps_len <= sizeof(info->sps)) {
		memcpy((char *)info->sps, (char *)(frame_buf) + vps_offset + nal_len, info->sps_len);
	} else {
		video_dprintf(VIDEO_LOG_ERR, "The sps length exceed the buffer\r\n");
		return -1;
	}
	if (info->pps_len <= sizeof(info->pps)) {
		memcpy((char *)info->pps, (char *)(frame_buf) + sps_offset + nal_len, info->pps_len);
	} else {
		video_dprintf(VIDEO_LOG_ERR, "The pps length exceed the buffer\r\n");
		return -1;
	}

	int output_len = 0;
	memset(info->vps_base64, 0x00, sizeof(info->vps_base64));
	memset(info->sps_base64, 0x00, sizeof(info->sps_base64));
	memset(info->pps_base64, 0x00, sizeof(info->pps_base64));
	memset(info->profile_level_id, 0x00, sizeof(info->profile_level_id));
	video_base64_encode(info->vps, info->vps_len, &output_len, info->vps_base64, sizeof(info->vps_base64));
	video_base64_encode(info->sps, info->sps_len, &output_len, info->sps_base64, sizeof(info->sps_base64));
	video_base64_encode(info->pps, info->pps_len, &output_len, info->pps_base64, sizeof(info->pps_base64));
	sprintf(info->profile_level_id, "%02x%02x%02x", info->sps[1], info->sps[2], info->sps[3]);
	video_dprintf(VIDEO_LOG_INF, "vps %s sps %s pps %s profile %s\r\n", info->vps_base64, info->sps_base64, info->pps_base64, info->profile_level_id);
	return 0;
}

#if NONE_FCS_MODE
#define FW_IMG_ISP_ID           0xCAA6
#define FW_IMG_VOE_ID           0xC2A7

int video_get_fw_isp_info(void)
{
	if (voe_load_sesnor_init) {
		return 0;
	}

	isp_multi_fcs_hdr_t multi_fcs_hdr;
	int psensor_set_start = 0;
	iq_info_t iq_start;
	sensor_info_t sensor_start;
	uint32_t i;
	fw_img_hdr_t img_hdr;
	int ptr_isp_multi_sensor, p_data = 0;
	ptr_isp_multi_sensor = 0X1080;

	isp_multi_fcs_ld_info_t *p_isp_sensor_ld_info = malloc(sizeof(isp_multi_fcs_ld_info_t));

	memset(p_isp_sensor_ld_info, 0x00, sizeof(isp_multi_fcs_ld_info_t));

	int offset = 0;
	void *fp = NULL;

	printf("video_get_fw_isp_info\r\n");

	if (hal_sys_get_ld_fw_idx() == FW_1) {
		fp = pfw_open("FW1", M_RAW);
		if (!fp) {
			video_dprintf(VIDEO_LOG_ERR, "cannot open file FW1\n\r");
			goto EXIT;
		}
	} else if (hal_sys_get_ld_fw_idx() == FW_2) {
		fp = pfw_open("FW2", M_RAW);
		if (!fp) {
			video_dprintf(VIDEO_LOG_ERR, "cannot open file FW2\n\r");
			goto EXIT;
		}
	} else {
		goto EXIT;
	}


	if (!fp) {
		video_dprintf(VIDEO_LOG_ERR, "It cannot open file FW\n\r");
		goto EXIT;
	}


	pfw_seek(fp, 0X1000, SEEK_SET);

	pfw_read(fp, &img_hdr, sizeof(fw_img_hdr_t));

	video_dprintf(VIDEO_LOG_INF, "imglen %x nxtoffset %x type_id %x nxt_type_id %x\r\n", img_hdr.imglen, img_hdr.nxtoffset, img_hdr.type_id, img_hdr.nxt_type_id);

	if (img_hdr.type_id == FW_IMG_ISP_ID) {
		pfw_seek(fp, 0X2100, SEEK_SET);//Skip the manifest
		pfw_read(fp, &multi_fcs_hdr, sizeof(isp_multi_fcs_hdr_t));
		offset = 0x1080;
		p_isp_sensor_ld_info->fcs_hdr_start = 0x1080 * 2;
		p_isp_sensor_ld_info->magic         = (multi_fcs_hdr.magic);
		p_isp_sensor_ld_info->version       = (multi_fcs_hdr.version);
		p_isp_sensor_ld_info->multi_fcs_cnt = (multi_fcs_hdr.multi_fcs_cnt);
		p_isp_sensor_ld_info->wait_km_init_timeout_us = (multi_fcs_hdr.wait_km_init_timeout_us);
	} else {
		video_dprintf(VIDEO_LOG_ERR, "It IS NOT FW_IMG_ISP_ID\r\n");
		goto EXIT;
	}


	for (i = 0; i < MULTI_FCS_MAX; i++) {
		if ((MULTI_SENSOR_SET_INVALID_PTN1 != (multi_fcs_hdr.fcs_data_offset[i])) &&
			(MULTI_SENSOR_SET_INVALID_PTN2 != (multi_fcs_hdr.fcs_data_offset[i])) &&
			(MULTI_SENSOR_SET_INVALID_PTN1 != (multi_fcs_hdr.fcs_data_size[i])) &&
			(MULTI_SENSOR_SET_INVALID_PTN2 != (multi_fcs_hdr.fcs_data_size[i]))) {
			p_isp_sensor_ld_info->sensor_set[i].fcs_data_offset = (multi_fcs_hdr.fcs_data_offset[i]);
			p_isp_sensor_ld_info->sensor_set[i].fcs_data_size   = (multi_fcs_hdr.fcs_data_size[i]);
			video_dprintf(VIDEO_LOG_INF, "index %d fcs_data_offset %x fcs_data_size %x\r\n", i, p_isp_sensor_ld_info->sensor_set[i].fcs_data_offset,
						  p_isp_sensor_ld_info->sensor_set[i].fcs_data_size);
			psensor_set_start = (ptr_isp_multi_sensor + (multi_fcs_hdr.fcs_data_offset[i]));
			p_data = (psensor_set_start + (multi_fcs_hdr.fcs_data_size[i])) + offset;
			video_dprintf(VIDEO_LOG_INF, "index %d p_data %x\r\n", i, p_data);
			//load_op_f(&iq_start, p_data, sizeof(iq_info_t));
			pfw_seek(fp, p_data, SEEK_SET);
			pfw_read(fp, &iq_start, sizeof(iq_info_t));
			p_data = (psensor_set_start + (multi_fcs_hdr.fcs_data_size[i]) + sizeof(iq_info_t)) + offset;
			video_dprintf(VIDEO_LOG_INF, "index %d p_data %x\r\n", i, p_data);
			//load_op_f(&sensor_start, p_data, sizeof(sensor_info_t));
			pfw_seek(fp, p_data, SEEK_SET);
			pfw_read(fp, &sensor_start, sizeof(sensor_info_t));
			p_isp_sensor_ld_info->sensor_set[i].iq_start_addr     = iq_start.start_addr;
			p_isp_sensor_ld_info->sensor_set[i].iq_data_size      = iq_start.data_size;
			p_isp_sensor_ld_info->sensor_set[i].sensor_start_addr = sensor_start.start_addr;
			p_isp_sensor_ld_info->sensor_set[i].sensor_data_size  = sensor_start.data_size;
			video_dprintf(VIDEO_LOG_INF, "index %d iq_start_addr %x iq_data_size %x\r\n", i, p_isp_sensor_ld_info->sensor_set[i].iq_start_addr,
						  p_isp_sensor_ld_info->sensor_set[i].iq_data_size);
			video_dprintf(VIDEO_LOG_INF, "index %d sensor_start_addr %x sensor_data_size %x\r\n", i, p_isp_sensor_ld_info->sensor_set[i].sensor_start_addr,
						  p_isp_sensor_ld_info->sensor_set[i].sensor_data_size);
		}
	}
	int p_fcs_data, p_iq_data, p_sensor_data = 0;
	for (i = 0; i < multi_fcs_hdr.multi_fcs_cnt; i++) {
		video_dprintf(VIDEO_LOG_INF, "sesnor %d fcs_data_size %x\r\n", i, p_isp_sensor_ld_info->sensor_set[i].fcs_data_size);
		video_dprintf(VIDEO_LOG_INF, "sesnor %d fcs_data_offset %x\r\n", i, p_isp_sensor_ld_info->sensor_set[i].fcs_data_offset);
		video_dprintf(VIDEO_LOG_INF, "sesnor %d iq_start_addr %x\r\n", i, p_isp_sensor_ld_info->sensor_set[i].iq_start_addr);
		video_dprintf(VIDEO_LOG_INF, "sesnor %d iq_data_size %x\r\n", i, p_isp_sensor_ld_info->sensor_set[i].iq_data_size);
		video_dprintf(VIDEO_LOG_INF, "sesnor %d sensor_start_addr %x\r\n", i, p_isp_sensor_ld_info->sensor_set[i].sensor_start_addr);
		video_dprintf(VIDEO_LOG_INF, "sesnor %d sensor_data_size %x\r\n", i, p_isp_sensor_ld_info->sensor_set[i].sensor_data_size);
		p_fcs_data    = 0x1080 + 0x1080 + (p_isp_sensor_ld_info->sensor_set[i].fcs_data_offset);
		p_iq_data     = p_fcs_data + (p_isp_sensor_ld_info->sensor_set[i].iq_start_addr);
		p_sensor_data = p_fcs_data + (p_isp_sensor_ld_info->sensor_set[i].sensor_start_addr);
		video_dprintf(VIDEO_LOG_INF, "sensor %d p_fcs_data %x p_iq_data %x p_sensor_data %x\r\n", i, p_fcs_data, p_iq_data, p_sensor_data);
	}
	/* isp_boot->p_fcs_ld_info.fcs_hdr_start = 0x1080*2;
	isp_boot->p_fcs_ld_info.multi_fcs_cnt = multi_fcs_hdr.multi_fcs_cnt; */
	int fcs_id = isp_boot->p_fcs_ld_info.fcs_id;
	memcpy(&isp_boot->p_fcs_ld_info, p_isp_sensor_ld_info, sizeof(isp_multi_fcs_ld_info_t));
	isp_boot->p_fcs_ld_info.fcs_id = fcs_id;
	if (fp) {
		pfw_close(fp);
	}
	if (p_isp_sensor_ld_info) {
		free(p_isp_sensor_ld_info);
	}



	video_dprintf(VIDEO_LOG_INF, "Get the ISP FW location\r\n");

	//voe_dump_isp_info();

	voe_load_sesnor_init = 1;

	return 0;
EXIT:
	return -1;
}
#endif
int video_set_private_mask(int ch, struct private_mask_s *pmask)
{
	hal_video_reset_mask_status();
	uint32_t *en = pmask->en;
	if (en[0] || en[1] || en[2] || en[3] || en[4]) {
		hal_video_set_mask_color(pmask->color);
	}
	for (int id = 0 ; id < MASK_MAX_NUM; id++) {
		if (id == MASK_GRID) { //GRID MODE
			isp_grid_t grid = {0};
			video_dprintf(VIDEO_LOG_MSG, "[GRID Mode]: %s\r\n", (en[id] > 0 ? "used" : "unused"));
			if (pmask->start_x[id] % 2) {
				video_dprintf(VIDEO_LOG_MSG, "[%s] invalid value pmask->start_x=%u.\r\n", __FUNCTION__, pmask->start_x);
				continue;
			}
			if (pmask->start_y[id] % 2) {
				video_dprintf(VIDEO_LOG_MSG, "[%s] invalid value pmask->start_y=%u.\r\n", __FUNCTION__, pmask->start_y);
				continue;
			}
			if (pmask->cols % 8) {
				video_dprintf(VIDEO_LOG_MSG, "[%s] invalid value pmask->cols=%u.\r\n", __FUNCTION__, pmask->cols);
				continue;
			}
			if (pmask->w[id] % 16) {
				unsigned int cell_w = pmask->w[id] / pmask->cols;
				if (cell_w % 2) {
					video_dprintf(VIDEO_LOG_MSG, "[%s] invalid value cell_w=%u.\r\n", __FUNCTION__, cell_w);
					continue;
				}
			}

			if (pmask->en[id] && pmask->cols > 0 && pmask->rows > 0) {
				grid.start_x = pmask->start_x[id];
				grid.start_y = pmask->start_y[id];
				grid.cols = pmask->cols;
				grid.rows = pmask->rows;
				grid.cell_w = pmask->w[id] / grid.cols;
				grid.cell_h = pmask->h[id] / grid.rows;
			}
			hal_video_config_grid_mask(pmask->en[id], grid, (uint8_t *)pmask->bitmap);
		} else { //RECT MODE
			isp_rect_t rect;
			video_dprintf(VIDEO_LOG_MSG, "[RECT Mode]: %s\r\n", (en[id] > 0 ? "used" : "unused"));
			if (pmask->start_x[id] % 2) {
				video_dprintf(VIDEO_LOG_MSG, "[%s] invalid value pmask->start_x=%u.\r\n", __FUNCTION__, pmask->start_x);
				continue;
			}
			if (pmask->start_y[id] % 2) {
				video_dprintf(VIDEO_LOG_MSG, "[%s] invalid value pmask->start_y=%u.\r\n", __FUNCTION__, pmask->start_y);
				continue;
			}
			rect.left = pmask->start_x[id];
			rect.top  = pmask->start_y[id];
			rect.right  = pmask->w[id] + rect.left;
			rect.bottom = pmask->h[id] + rect.top;
			hal_video_config_rect_mask(pmask->en[id], id - 1, rect);
		}
	}
	int is_video_not_open = 1;
	for (int i = 0; i < MAX_CHANNEL; i++) {
		if (voe_info.ch_info[i].stream_is_open) {
			is_video_not_open = 0;
			break;
		}
	}
	return hal_video_set_mask(ch, is_video_not_open);
}
void video_set_private_mask_single(int ch, private_mask_single_t *pmask)
{
	video_dprintf(VIDEO_LOG_MSG, "enable: %d, id: %d\r\nstart_x: %d, start_y: %d\r\ncols: %d, rows: %d\r\nw: %d, h: %d\r\n"
				  , pmask->en, pmask->id, pmask->start_x, pmask->start_y, pmask->cols, pmask->rows, pmask->w, pmask->h);
	if (pmask->en < 0 || pmask->en > 1) {
		video_dprintf(VIDEO_LOG_MSG, "[%s] invalid value pmask->en=%d", __FUNCTION__, pmask->en);
		return;
	}
	if (pmask->grid_mode < 0 || pmask->grid_mode > 1) {
		video_dprintf(VIDEO_LOG_MSG, "[%s] invalid value pmask->grid_mode=%d", __FUNCTION__, pmask->grid_mode);
		return;
	}
	if (pmask->id < 0 || pmask->id > 3) {
		video_dprintf(VIDEO_LOG_MSG, "[%s] invalid value pmask->id=%d", __FUNCTION__, pmask->id);
		return;
	}
	if (pmask->start_x % 2) {
		video_dprintf(VIDEO_LOG_MSG, "[%s] invalid value pmask->start_x=%d", __FUNCTION__, pmask->start_x);
		return;
	}
	if (pmask->start_y % 2) {
		video_dprintf(VIDEO_LOG_MSG, "[%s] invalid value pmask->start_y=%d", __FUNCTION__, pmask->start_y);
		return;
	}
	if (pmask->grid_mode == 1 && pmask->cols % 8) {
		video_dprintf(VIDEO_LOG_MSG, "[%s] invalid value pmask->cols=%d", __FUNCTION__, pmask->cols);
		return;
	}
	if (pmask->grid_mode == 1 && pmask->w % 16) {
		int cell_w = pmask->w / pmask->cols;
		if (cell_w % 2) {
			video_dprintf(VIDEO_LOG_MSG, "[%s] invalid value cell_w=%d", __FUNCTION__, cell_w);
			return;
		}
	}
	hal_video_reset_mask_status();
	if (pmask->en) {
		hal_video_set_mask_color(pmask->color);
	}


	if (pmask->grid_mode) {
		video_dprintf(VIDEO_LOG_MSG, "[GRID Mode]:\r\n");
		isp_grid_t grid;
		grid.start_x = pmask->start_x;
		grid.start_y = pmask->start_y;
		grid.cols = pmask->cols;
		grid.rows = pmask->rows;
		grid.cell_w = pmask->w / grid.cols;
		grid.cell_h = pmask->h / grid.rows;
		hal_video_config_grid_mask(pmask->en, grid, (uint8_t *)pmask->bitmap);
	} else {
		video_dprintf(VIDEO_LOG_MSG, "[RECT Mode]:\r\n");
		isp_rect_t rect;
		rect.left = pmask->start_x;
		rect.top  = pmask->start_y;
		rect.right  = pmask->w + rect.left;
		rect.bottom = pmask->h + rect.top;
		hal_video_config_rect_mask(pmask->en, pmask->id, rect);
	}
	int is_video_not_open = 1;
	for (int i = 0; i < MAX_CHANNEL; i++) {
		if (voe_info.ch_info[i].stream_is_open) {
			is_video_not_open = 0;
			break;
		}
	}
	hal_video_set_mask(ch, is_video_not_open);
}

static int voe_cmd_min_timeout = 100, voe_cmd_max_timeout = 1000;
int voe_cmd_timout_setting(uint32_t cmd)
{
	switch (cmd) {
	case VOE_OPEN_CMD:
		return voe_cmd_max_timeout;
	case VOE_CLOSE_CMD:
		return voe_cmd_max_timeout;
	default: // other voe command
		return voe_cmd_min_timeout;
	}
}

static uint32_t voe_error_cmd = 0;
static int voe_error_timeout = 0;
void voe_cmd_timout_callback(u32 cur_cmd, u32 voe_cmd, int timeout)
{
	voe_error_cmd = cur_cmd;
	voe_error_timeout = timeout;
	printf("VOE command wait ACK timeout 0x%x != 0x%x\n", voe_cmd, cur_cmd);
	printf("VOE cmd 0x%x ACK timeout %dms.\n", voe_error_cmd, voe_error_timeout);
	return;
}

void voe_get_cmd_timout_info(uint32_t *cmd, int *timeout)
{
	*cmd = voe_error_cmd;
	*timeout = voe_error_timeout;
	printf("VOE cm 0x%x ACK timeout %dms.\n", voe_error_cmd, voe_error_timeout);
	return;
}

void voe_set_cmd_timout(int min_timeout, int max_timeout)
{
	voe_cmd_min_timeout = min_timeout;
	voe_cmd_max_timeout = max_timeout;
	return;
}

uint32_t video_get_video_timer_cur_time(void)
{
	return hal_video_get_video_timer_cur_time();
}

//hal_read_curtime_us; => system
//hal_video_get_video_timer_cur_time(); => video
uint32_t video_get_system_ts_from_isp_ts(uint32_t cur_system_ts, uint32_t cur_isp_ts, int channel)
{
	// convert the isp time stamp to the current time
	//printf("video input channel %d\r\n", channel);
	uint32_t cur_system_ts_ms = cur_system_ts;
	uint32_t cur_video_ts_us = hal_video_get_video_timer_cur_time();
	uint32_t diff_system_ts_ms = 0;
	uint32_t diff_video_ts_ms = 0;
	// correct the by <isp_timestamp> * 0.992 (124/125)
	// when last_isp_ts[channel] < cur_isp_ts means the overflow happened
	if (last_isp_ts[channel] > cur_isp_ts) {
		isp_overflow_times[channel] ++;
	}
	last_isp_ts[channel] = cur_isp_ts;
	cur_isp_ts += isp_overflow_times[channel] * (UINT32_MAX / 1000) + (isp_overflow_times[channel] * (UINT32_MAX % 1000)) / 1000;

	// to prevent overflow use a - a/125 to replace a * 124 / 125
	cur_isp_ts -= cur_isp_ts / 125;
	cur_video_ts_us -= cur_video_ts_us / 125;

	// Get cur_video_ts_ms by rounding the cur_video_ts_us
	uint32_t cur_video_ts_ms;
	cur_video_ts_ms = (cur_video_ts_us + 500) / 1000;

	diff_system_ts_ms = cur_system_ts_ms - last_system_ts_ms[channel];
	// Intialize the offset between video and audio in the first time or the video pause time >= max time
	if (isp_ts_initialed_flag[channel] == 0 || diff_system_ts_ms >= (UINT32_MAX / 1000 - UINT32_MAX / 1000 / 125)) {
		isp_ts_initialed_flag[channel] = 1;
		isp_overflow_times[channel] = 0;
		offset_ms[channel] = cur_system_ts_ms - cur_video_ts_ms;
		last_video_ts_ms[channel] = cur_video_ts_ms;
		last_system_ts_ms[channel] = cur_system_ts_ms;
		goto count_ts;
	}

	// calcaute the difference between video and system clock
	// the following process is just to prevent some unexpected situation to prvent one clock stop templetely
	if (cur_video_ts_ms <= last_video_ts_ms[channel]) {// system or video overflow
		last_video_ts_ms[channel] = cur_video_ts_ms;
		last_system_ts_ms[channel] = cur_system_ts_ms;
		goto count_ts;
	}
	diff_video_ts_ms = cur_video_ts_ms - last_video_ts_ms[channel];

	// Check every 1 sec
	if (diff_system_ts_ms < 1000) {
		goto count_ts;
	}

	// If video_diff > system_diff count negative offset
	int this_ts_diff_ms = diff_system_ts_ms - diff_video_ts_ms;

	// Change the original offset if needed
	offset_ms[channel] += this_ts_diff_ms;

	// Record the current time for the next time check
	//printf("video last: %d, cur %d\r\n", last_video_ts_ms, cur_video_ts_ms);
	//printf("system last: %d, cur %d\r\n", last_system_ts_ms, cur_system_ts_ms);
	last_video_ts_ms[channel] = cur_video_ts_ms;
	last_system_ts_ms[channel] = cur_system_ts_ms;
count_ts:

	return (uint32_t)((int)cur_isp_ts + offset_ms[channel]);
}

int video_ext_in(int ch, uint32_t addr)
{
	return hal_video_ext_in(ch, addr);
}

static char video_encoding_table[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
									  'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
									  'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
									  'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
									  'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
									  'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
									  'w', 'x', 'y', 'z', '0', '1', '2', '3',
									  '4', '5', '6', '7', '8', '9', '+', '/'
									 };
static char *decoding_table = NULL;
static int video_mod_table[] = {0, 2, 1};

char *video_base64_encode(unsigned char *data_in, int input_length, int *output_length, char *data_out, int data_out_len)
{

	*output_length = 4 * ((input_length + 2) / 3);

	/* char *encoded_data = malloc(*output_length);
	if (encoded_data == NULL) return NULL; */
	if (data_out_len < *output_length) {
		return NULL;
	}
	for (int i = 0, j = 0; i < input_length;) {

		uint32_t octet_a = i < input_length ? (unsigned char)data_in[i++] : 0;
		uint32_t octet_b = i < input_length ? (unsigned char)data_in[i++] : 0;
		uint32_t octet_c = i < input_length ? (unsigned char)data_in[i++] : 0;

		uint32_t triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;

		data_out[j++] = video_encoding_table[(triple >> 3 * 6) & 0x3F];
		data_out[j++] = video_encoding_table[(triple >> 2 * 6) & 0x3F];
		data_out[j++] = video_encoding_table[(triple >> 1 * 6) & 0x3F];
		data_out[j++] = video_encoding_table[(triple >> 0 * 6) & 0x3F];
	}

	for (int i = 0; i < video_mod_table[input_length % 3]; i++) {
		data_out[*output_length - 1 - i] = '=';
	}
	return data_out;
}
//ISO/IEC 14496-10 7.4.1 emulation_prevention_three_byte
static int video_check_sei_dummy_bytes(video_meta_t *m_parm, unsigned int length)
{
	int zeroCount = 0;
	int dummy_bytes_length = 0;
	int value = 0;
	for (int i = 0; i < length; i++) {
		value = video_sei_buf[i];
		if (zeroCount == 2 && value <= 3) {
			zeroCount = 0;
			dummy_bytes_length++;
		}

		if (value == 0) {
			zeroCount++;
		} else {
			zeroCount = 0;
		}
	}
	return dummy_bytes_length;
}

static void video_insert_sei_dummy_three_bytes(unsigned char *video_output, unsigned int length, unsigned int extra_dummy_byte, unsigned int meta_length)
{
	int outputAmount = 0;
	int zeroCount = 0;
	int value = 0;

	for (int i = 0; i < length; i++) {
		value = video_sei_buf[i];
		if (outputAmount >= meta_length) {
			video_dprintf(VIDEO_LOG_ERR, "outputAmount %d bigger than meta_size %d\r\n", outputAmount, meta_length);
			break;
		}
		if (zeroCount == 2 && value <= 3) {
			video_output[outputAmount++] = VIDEO_START_CODE_DUMMY;
			zeroCount = 0;
		}

		if (value == 0) {
			zeroCount++;
		} else {
			zeroCount = 0;
		}
		video_output[outputAmount++] = value;
	}
	if (zeroCount > 0 && outputAmount == (meta_length - 1)) { //00 00 -> 00 03
		if (outputAmount >= meta_length) {
			video_dprintf(VIDEO_LOG_ERR, "outputAmount %d bigger than meta_size %d\r\n", outputAmount, meta_length);
		} else {
			video_output[outputAmount++] = VIDEO_START_CODE_DUMMY;
		}
	}
	if (extra_dummy_byte) {
		if (outputAmount + 10 < meta_length) {
			video_output[outputAmount++] = 0x10;
			video_output[outputAmount++] = 0x00;
			video_output[outputAmount++] = 0x00;
			video_output[outputAmount++] = 0x03;
			video_output[outputAmount++] = 0x00;
			video_output[outputAmount++] = 0x00;
			video_output[outputAmount++] = 0x03;
		} else {
			video_dprintf(VIDEO_LOG_ERR, "It can't insert the dummy bytes %d %d\r\n", outputAmount, meta_length);
		}
	}
}
static int video_meta_size_count(int size)
{
	int value = 0;
	if (size % 0xff == 0) {
		value = size / 0xff;
	} else {
		value = size / 0xff + 1;
	}
	return value;
}
/*
Layout [UUID-16][ISP_STATIS_META][ISP_META_DATA]
*/
/*
T-REC-H.265
Since the payload size of an SEI message is specified in RBSP bytes, the quantity of emulation prevention bytes is not included in the size payloadSize of an SEI payload.
*/
#include "isp_ctrl_api.h"
#include "isp_api.h"
int video_meta_copy_info(video_meta_t *m_parm)
{
	int isp_3a_enable = 0;
	int length = 0;
	int ret = 0;
	unsigned char tag[VIDEO_META_3A_TAG_SIZE] = {0x33, 0x41, 0X44, 0X45};//"3ADB"

	if (isp_boot->fcs_status) {
		if (m_parm->meta_size > isp_boot->fcs_meta_total_size) {
			isp_3a_enable = 1;
		}
	} else {
		if (m_parm->meta_size > video_pre_init_param.video_meta_total_size) {
			isp_3a_enable = 1;
		}
	}
	if (isp_3a_enable) {
		length =  sizeof(isp_statis_meta_t) + sizeof(isp_meta_t) + sizeof(af_statis_t) + sizeof(ae_statis_t) + sizeof(awb_statis_t) + VIDEO_META_UUID_SIZE + sizeof(
					  tag);
		ret = isp_get_AF_statis(&af_result);
		ret = isp_get_AE_statis(&ae_result, AE_STATIS_HIST);
		ret = isp_get_AWB_statis(&awb_result);
	} else {
		length = sizeof(isp_statis_meta_t) + sizeof(isp_meta_t) + VIDEO_META_UUID_SIZE;
	}

	if (length > VIDEO_META_REV_BUF) {
		video_dprintf(VIDEO_LOG_MSG, "The length %d bigger than reserved %d size\r\n", length, VIDEO_META_REV_BUF);
		return -1;
	}

	memcpy(video_sei_buf, video_pre_init_param.video_meta_uuid, VIDEO_META_UUID_SIZE);
	memcpy(video_sei_buf + VIDEO_META_UUID_SIZE, m_parm->isp_statis_meta, sizeof(isp_statis_meta_t));
	memcpy(video_sei_buf + VIDEO_META_UUID_SIZE + sizeof(isp_statis_meta_t), m_parm->isp_meta_data, sizeof(isp_meta_t));
	if (isp_3a_enable) {
		memcpy(video_sei_buf + VIDEO_META_UUID_SIZE + sizeof(isp_statis_meta_t) + sizeof(isp_meta_t), tag, sizeof(tag));
		memcpy(video_sei_buf + VIDEO_META_UUID_SIZE + sizeof(tag) + sizeof(isp_statis_meta_t) + sizeof(isp_meta_t), (unsigned char *)&af_result,
			   sizeof(af_result));
		memcpy(video_sei_buf + VIDEO_META_UUID_SIZE + sizeof(tag) + sizeof(isp_statis_meta_t) + sizeof(isp_meta_t) + sizeof(af_result),
			   (unsigned char *)&ae_result, sizeof(ae_result));
		memcpy(video_sei_buf + VIDEO_META_UUID_SIZE + sizeof(tag) + sizeof(isp_statis_meta_t) + sizeof(isp_meta_t) + sizeof(af_result) + sizeof(ae_result),
			   (unsigned char *)&awb_result, sizeof(awb_result));
	}
	if (m_parm->user_buf != NULL) {
		memcpy(video_sei_buf + length, m_parm->user_buf, m_parm->user_buf_len);
		length += m_parm->user_buf_len;
	}
	return length;
}
//////////////////////EXIF///////////////////////
static void video_exif_degree2dms(double v, uint32_t *out)
{
	double a = fabs(v);
	out[0] = (int)a;
	out[1] = 1;
	double m = (a - out[0]) * 60.0;
	out[2] = (int)m;
	out[3] = 1;
	double s = (m - out[2]) * 60.0;
	out[4] = (uint32_t)(s * 10000 + 0.5);
	out[5] = 10000;
}
static void video_exif_float2rational(float v, uint32_t *num, uint32_t *den)
{
	*den = 10000;
	*num = (uint32_t)(fabs(v) * (*den) + 0.5);
}
static void video_exif_double2rational(double v, uint32_t *num, uint32_t *den)
{
	*den = 10000;
	*num = (uint32_t)(fabs(v) * (*den) + 0.5);
}

static void video_exif_write_u16(uint8_t *b, uint16_t v)
{
	b[0] = v & 0xFF;
	b[1] = v >> 8;
}
static void video_exif_write_u32(uint8_t *b, uint32_t v)
{
	b[0] = v & 0xFF;
	b[1] = (v >> 8) & 0xFF;
	b[2] = (v >> 16) & 0xFF;
	b[3] = (v >> 24) & 0xFF;
}

static void video_exif_write_entry(uint8_t *entry, ExifTag *tag, uint32_t value_offset, uint8_t *data_area, uint32_t *data_offset, uint32_t tiff_header_off)
{
	video_exif_write_u16(entry, tag->tag);
	video_exif_write_u16(entry + 2, tag->type);
	video_exif_write_u32(entry + 4, tag->count);

	if (tag->type == TYPE_ASCII) {
		uint32_t len = tag->count;
		if (len <= 4) {
			for (uint32_t i = 0; i < len; ++i) {
				entry[8 + i] = tag->data.bytes[i];
			}
			for (uint32_t i = len; i < 4; ++i) {
				entry[8 + i] = 0;
			}
		} else {
			video_exif_write_u32(entry + 8, (uint32_t)(*data_offset - tiff_header_off));
			memcpy(data_area + *data_offset, tag->data.bytes, len);
			*data_offset += len;
		}
	} else if (tag->type == TYPE_BYTE) {
		if (tag->count <= 4) {
			for (uint32_t i = 0; i < tag->count; ++i) {
				entry[8 + i] = tag->data.bytes[i];
			}
			for (uint32_t i = tag->count; i < 4; ++i) {
				entry[8 + i] = 0;
			}
		} else {
			video_exif_write_u32(entry + 8, (uint32_t)(*data_offset - tiff_header_off));
			memcpy(data_area + *data_offset, tag->data.bytes, tag->count);
			*data_offset += tag->count;
		}
	} else if (tag->type == TYPE_SHORT) {
		if (tag->count == 1) {
			entry[8] = tag->data.short_val & 0xFF;
			entry[9] = (tag->data.short_val >> 8) & 0xFF;
			entry[10] = 0;
			entry[11] = 0;
		} else {
			video_exif_write_u32(entry + 8, (uint32_t)(*data_offset - tiff_header_off));
			memcpy(data_area + *data_offset, tag->data.short_arr, sizeof(uint16_t)*tag->count);
			*data_offset += sizeof(uint16_t) * tag->count;
		}
	} else if (tag->type == TYPE_LONG) {
		video_exif_write_u32(entry + 8, tag->data.long_val);
	} else if (tag->type == TYPE_RATIONAL) {
		if (tag->count == 1) {
			video_exif_write_u32(entry + 8, (uint32_t)(*data_offset - tiff_header_off));
			video_exif_write_u32(data_area + *data_offset, tag->data.rational.num);
			video_exif_write_u32(data_area + *data_offset + 4, tag->data.rational.den);
			*data_offset += 8;
		} else {
			video_exif_write_u32(entry + 8, (uint32_t)(*data_offset - tiff_header_off));
			for (uint32_t i = 0; i < tag->count; ++i) {
				uint32_t num = tag->data.rational_arr[i * 2];
				uint32_t den = tag->data.rational_arr[i * 2 + 1];
				video_exif_write_u32(data_area + *data_offset, num);
				video_exif_write_u32(data_area + *data_offset + 4, den);
				*data_offset += 8;
			}
		}
	}
}

int video_create_exif_tags(uint8_t *buf, uint32_t video_len)
{
	ExifTag *main_ifd_tags = jpeg_exif_wsp.main_tags;
	int main_count = jpeg_exif_wsp.exif_count;
	ExifTag *exif_ifd_tags = jpeg_exif_wsp.exif_tags;
	int exif_count = jpeg_exif_wsp.exif_count;
	ExifTag *gps_ifd_tags = jpeg_exif_wsp.gps_tags;
	int gps_count = jpeg_exif_wsp.gps_count;

	uint32_t exif_header = 6, tiff_header = 8;
	uint32_t offset = exif_header + tiff_header;
	uint32_t ifd0_offset = offset;
	uint32_t ifd0_size = (main_ifd_tags && main_count > 0) ? 2 + main_count * 12 + 4 : 0;
	offset += ifd0_size;

	uint32_t exif_ifd_offset = offset;
	uint32_t exif_ifd_size = (exif_ifd_tags && exif_count > 0) ? 2 + exif_count * 12 + 4 : 0;
	offset += exif_ifd_size;

	uint32_t gps_ifd_offset = offset;
	uint32_t gps_ifd_size = (gps_ifd_tags && gps_count > 0) ? 2 + gps_count * 12 + 4 : 0;
	offset += gps_ifd_size;

	uint32_t extra = 0;
	int i;
	// count extra for all IFDs: only if tags && count>0
	for (i = 0; (main_ifd_tags && i < main_count); ++i) {
		ExifTag *tag = &main_ifd_tags[i];
		if (tag->type == TYPE_ASCII) {
			extra += tag->count;
		} else if (tag->type == TYPE_RATIONAL) {
			extra += tag->count * 8;
		} else if (tag->type == TYPE_BYTE && tag->count > 4) {
			extra += tag->count;
		} else if (tag->type == TYPE_SHORT && tag->count > 1) {
			extra += tag->count * 2;
		}
	}
	for (i = 0; (exif_ifd_tags && i < exif_count); ++i) {
		ExifTag *tag = &exif_ifd_tags[i];
		if (tag->type == TYPE_ASCII) {
			extra += tag->count;
		} else if (tag->type == TYPE_RATIONAL) {
			extra += tag->count * 8;
		} else if (tag->type == TYPE_BYTE && tag->count > 4) {
			extra += tag->count;
		} else if (tag->type == TYPE_SHORT && tag->count > 1) {
			extra += tag->count * 2;
		}
	}
	for (i = 0; (gps_ifd_tags && i < gps_count); ++i) {
		ExifTag *tag = &gps_ifd_tags[i];
		if (tag->type == TYPE_ASCII) {
			extra += tag->count;
		} else if (tag->type == TYPE_RATIONAL) {
			extra += tag->count * 8;
		} else if (tag->type == TYPE_BYTE && tag->count > 4) {
			extra += tag->count;
		} else if (tag->type == TYPE_SHORT && tag->count > 1) {
			extra += tag->count * 2;
		}
	}

	uint32_t data_start = offset;
	uint32_t total_size = offset + extra;
	if (ifd0_size == 0) { //If it don't have the main tag, it won't do it.
		video_dprintf(VIDEO_LOG_MSG, "It don't have the main tag\r\n");
		return -1;
	}
	if (total_size > video_len) {
		video_dprintf(VIDEO_LOG_MSG, "The exif size %d is bigger than meta len %d\r\n", total_size, video_len);
		return -1;
	}

	memset(buf, 0x00, video_len);

	memcpy(buf, "Exif\0\0", 6);

	// (1) TIFF header (offset 6)
	video_exif_write_u16(buf + 6, 0x4949);  // II
	video_exif_write_u16(buf + 8, 42);
	video_exif_write_u32(buf + 10, 8);      // offset to IFD0: always=8 from TIFF header

	// (2) 0th IFD
	uint8_t *ifd0 = buf + ifd0_offset;
	video_exif_write_u16(ifd0, main_count);

	// (3) 0th IFD entry/fields
	uint32_t data_offset = data_start;
	for (i = 0; i < main_count; ++i) {
		uint8_t *entry = ifd0 + 2 + 12 * i;
		uint32_t tagid = main_ifd_tags[i].tag;
		// Pointer to EXIF/GPS IFD entry
		if (tagid == 0x8769) { // ExifIFD pointer
			video_exif_write_u16(entry, main_ifd_tags[i].tag);
			video_exif_write_u16(entry + 2, TYPE_LONG);
			video_exif_write_u32(entry + 4, 1);
			// pointer only when exif_ifd_tags exists & count>0 else 0
			if (exif_ifd_tags && exif_count > 0) {
				video_exif_write_u32(entry + 8, (uint32_t)(exif_ifd_offset - 6));
			} else {
				video_exif_write_u32(entry + 8, 0);
			}
		} else if (tagid == 0x8825) { // GPSIFD pointer
			video_exif_write_u16(entry, main_ifd_tags[i].tag);
			video_exif_write_u16(entry + 2, TYPE_LONG);
			video_exif_write_u32(entry + 4, 1);
			if (gps_ifd_tags && gps_count > 0) {
				video_exif_write_u32(entry + 8, (uint32_t)(gps_ifd_offset - 6));
			} else {
				video_exif_write_u32(entry + 8, 0);
			}
		} else {
			video_exif_write_entry(entry, &main_ifd_tags[i], (uint32_t)(data_offset - 6), buf, &data_offset, 6);
		}
	}
	video_exif_write_u32(ifd0 + 2 + 12 * main_count, 0);

	// (4) Exif IFD
	if (exif_ifd_tags && exif_count > 0) {
		uint8_t *exif_ifd = buf + exif_ifd_offset;
		video_exif_write_u16(exif_ifd, exif_count);
		for (i = 0; i < exif_count; ++i) {
			uint8_t *entry = exif_ifd + 2 + 12 * i;
			video_exif_write_entry(entry, &exif_ifd_tags[i], (uint32_t)(data_offset - 6), buf, &data_offset, 6);
		}
		video_exif_write_u32(exif_ifd + 2 + 12 * exif_count, 0);
	}
	// (5) GPS IFD
	if (gps_ifd_tags && gps_count > 0) {
		uint8_t *gps_ifd = buf + gps_ifd_offset;
		video_exif_write_u16(gps_ifd, gps_count);
		for (i = 0; i < gps_count; ++i) {
			uint8_t *entry = gps_ifd + 2 + 12 * i;
			video_exif_write_entry(entry, &gps_ifd_tags[i], (uint32_t)(data_offset - 6), buf, &data_offset, 6);
		}
		video_exif_write_u32(gps_ifd + 2 + 12 * gps_count, 0);
	}
	return 0;
}

int video_insert_jpeg_exif(video_meta_t *m_parm)
{
	uint8_t *video_output = NULL;
	int meta_length = m_parm->meta_size;
	video_output = (uint8_t *)(m_parm->video_addr) + m_parm->meta_offset;
	int ret = video_create_exif_tags(video_output + VIDEO_JPEG_META_OFFSET, meta_length);

	if (ret < 0) {
		return -1;
	} else {
		//Modify the app1 to store the exif
		video_output[0] = 0xFF;
		video_output[1] = 0XE1;
	}
	return 0;
}

void video_fill_exif_tags_from_struct(const ExifParams *params)
{
	int m = 0, e = 0, g = 0;
	memset(&jpeg_exif_wsp, 0x00, sizeof(ExifWorkspace));
	ExifWorkspace *wsp = &jpeg_exif_wsp;
	// ==== Main IFD (Image File Directory) ====
	// Check if the manufacturer is available and fill the main tags
	if (params->make)
		wsp->main_tags[m++] = (ExifTag) {
		0x010F, TYPE_ASCII, strlen(params->make) + 1, .data.bytes = (const uint8_t *)params->make
	};

	// Check if the model is available and fill the main tags
	if (params->model)
		wsp->main_tags[m++] = (ExifTag) {
		0x0110, TYPE_ASCII, strlen(params->model) + 1, .data.bytes = (const uint8_t *)params->model
	};

	// Check if the datetime is available and fill the main tags
	if (params->datetime)
		wsp->main_tags[m++] = (ExifTag) {
		0x0132, TYPE_ASCII, 20, .data.bytes = (const uint8_t *)params->datetime
	};

	// Add the ExifIFD pointer tag (the pointer to the EXIF data)
	wsp->main_tags[m++] = (ExifTag) {
		0x8769, TYPE_LONG, 1, .data.long_val = 0
	};

	// If GPS data is present, add the GPS pointer tag
	if (params->has_gps)
		wsp->main_tags[m++] = (ExifTag) {
		0x8825, TYPE_LONG, 1, .data.long_val = 0
	};

	// Set the count of main tags filled
	wsp->main_count = m;

	// ==== Exif IFD (EXIF Information) ====
	// If datetime is present, fill EXIF datetime tags
	if (params->datetime) {
		wsp->exif_tags[e++] = (ExifTag) {
			0x9003, TYPE_ASCII, 20, .data.bytes = (const uint8_t *)params->datetime
		};
		wsp->exif_tags[e++] = (ExifTag) {
			0x9004, TYPE_ASCII, 20, .data.bytes = (const uint8_t *)params->datetime
		};
	}

	// If exposure time is valid (greater than a small threshold), store it as a rational number (numerator/denominator)
	if (params->exposure_time > 1e-6) {
		uint32_t num, den;
		video_exif_double2rational(params->exposure_time, &num, &den);
		wsp->exif_tags[e++] = (ExifTag) {
			0x829A, TYPE_RATIONAL, 1, .data.rational = {num, den}
		};
	}

	// If f-number is valid, store it as a rational number (numerator/denominator)
	if (params->fnumber > 1e-6) {
		uint32_t num, den;
		video_exif_double2rational(params->fnumber, &num, &den);
		wsp->exif_tags[e++] = (ExifTag) {
			0x829D, TYPE_RATIONAL, 1, .data.rational = {num, den}
		};
	}

	// If focal length is valid, store it as a rational number (numerator/denominator)
	if (params->focal_length > 1e-6) {
		uint32_t num, den;
		video_exif_double2rational(params->focal_length, &num, &den);
		wsp->exif_tags[e++] = (ExifTag) {
			0x920A, TYPE_RATIONAL, 1, .data.rational = {num, den}
		};
	}

	// If white balance is available, store the tag
	if (params->white_balance >= 0)
		wsp->exif_tags[e++] = (ExifTag) {
		0xA403, TYPE_SHORT, 1, .data.short_val = (uint16_t)params->white_balance
	};

	// If ISO is valid, store the ISO value
	if (params->iso > 0)
		wsp->exif_tags[e++] = (ExifTag) {
		0x8827, TYPE_SHORT, 1, .data.short_val = (uint16_t)params->iso
	};

	// Set the count of EXIF tags filled
	wsp->exif_count = e;

	// ==== GPS IFD (GPS Information) ====
	// If GPS data is present, fill GPS-related tags
	if (params->has_gps) {
		const uint8_t gps_ver[4] = {2, 3, 0, 0};

		// Set latitude and longitude reference (North/South and East/West)
		wsp->latref_buf[0] = (params->gps_latitude >= 0) ? 'N' : 'S';
		wsp->latref_buf[1] = 0;
		wsp->lonref_buf[0] = (params->gps_longitude >= 0) ? 'E' : 'W';
		wsp->lonref_buf[1] = 0;

		// Convert latitude and longitude from degrees to DMS (degrees, minutes, seconds)
		video_exif_degree2dms(params->gps_latitude, wsp->gps_lat_arr);
		video_exif_degree2dms(params->gps_longitude, wsp->gps_lon_arr);

		// Set altitude reference and store altitude in a rational format (height in meters * 100)
		wsp->altref_buf[0] = (params->gps_altitude >= 0) ? 0 : 1;
		wsp->gps_alt_arr[0] = (uint32_t)fabs(params->gps_altitude * 100);
		wsp->gps_alt_arr[1] = 100;

		// Add GPS tags with appropriate values
		wsp->gps_tags[g++] = (ExifTag) {
			0x0000, TYPE_BYTE, 4, .data.bytes = gps_ver
		};
		wsp->gps_tags[g++] = (ExifTag) {
			0x0001, TYPE_ASCII, 2, .data.bytes = wsp->latref_buf
		};
		wsp->gps_tags[g++] = (ExifTag) {
			0x0002, TYPE_RATIONAL, 3, .data.rational_arr = wsp->gps_lat_arr
		};
		wsp->gps_tags[g++] = (ExifTag) {
			0x0003, TYPE_ASCII, 2, .data.bytes = wsp->lonref_buf
		};
		wsp->gps_tags[g++] = (ExifTag) {
			0x0004, TYPE_RATIONAL, 3, .data.rational_arr = wsp->gps_lon_arr
		};
		wsp->gps_tags[g++] = (ExifTag) {
			0x0005, TYPE_BYTE, 1, .data.bytes = wsp->altref_buf
		};
		wsp->gps_tags[g++] = (ExifTag) {
			0x0006, TYPE_RATIONAL, 1, .data.rational = {wsp->gps_alt_arr[0], wsp->gps_alt_arr[1]}
		};
	}

	// Set the count of GPS tags filled
	wsp->gps_count = g;
}

void video_sei_write(video_meta_t *m_parm)
{
	int length = 0;
	int meta_length = m_parm->meta_size;
	unsigned char *video_output = NULL;

	if (m_parm->type == AV_CODEC_ID_MJPEG) {
		video_output = (unsigned char *)(m_parm->video_addr) + m_parm->meta_offset + VIDEO_JPEG_META_OFFSET;
	} else if (m_parm->type == AV_CODEC_ID_H264) {
		video_output = (unsigned char *)(m_parm->video_addr) + m_parm->meta_offset + VIDEO_H264_META_OFFSET + video_get_meta_offset(meta_length);
	} else if (m_parm->type == AV_CODEC_ID_H265) {
		video_output = (unsigned char *)(m_parm->video_addr) + m_parm->meta_offset + VIDEO_HEVC_META_OFFSET + video_get_meta_offset(meta_length);
	} else {
		video_dprintf(VIDEO_LOG_MSG, "It don't support type %d\r\n", m_parm->type);
		return;
	}

	length = video_meta_copy_info(m_parm);

	if (length < 0) {
		video_dprintf(VIDEO_LOG_MSG, "The length %d bigger than reserved %d size\r\n", length, VIDEO_META_REV_BUF);
		return;
	}

	if (m_parm->type == AV_CODEC_ID_MJPEG) {
		memcpy(video_output, video_sei_buf, length);
	} else if (m_parm->type == AV_CODEC_ID_H265 || m_parm->type == AV_CODEC_ID_H264) {
		int enc_meta_size = 0;
		int enc_size_interval = 0;
		int dummp_byte_enable = 0;
		int dummy_bytes_length = 0;
		if (m_parm->type == AV_CODEC_ID_H264) {
			video_output = (unsigned char *)(m_parm->video_addr) + m_parm->meta_offset + VIDEO_H264_META_OFFSET - 1; //Get the start offset of the sei payload.
		} else {
			video_output = (unsigned char *)(m_parm->video_addr) + m_parm->meta_offset + VIDEO_HEVC_META_OFFSET - 1; //Get the start offset of the sei payload.
		}
		dummy_bytes_length = video_check_sei_dummy_bytes(m_parm, length);
		enc_meta_size = meta_length - dummy_bytes_length;
		if (enc_meta_size % 0xff == 0) { //insert the dummy byte
			dummp_byte_enable = 1;
			enc_meta_size = enc_meta_size + (video_meta_size_count(meta_length) - video_meta_size_count(meta_length - dummy_bytes_length)) - 2;
		} else {
			enc_meta_size = enc_meta_size + (video_meta_size_count(meta_length) - video_meta_size_count(meta_length - dummy_bytes_length));
		}
		enc_size_interval = video_meta_size_count(enc_meta_size);

		for (int i = 0; i < enc_size_interval; i++) { //Modify the meta size
			if (i == (enc_size_interval - 1)) {
				video_output[i] = enc_meta_size % 0xff;
			} else {
				video_output[i] = 0xff;
			}
		}
		if (m_parm->type == AV_CODEC_ID_H264) {
			video_output = (unsigned char *)(m_parm->video_addr) + m_parm->meta_offset + VIDEO_H264_META_OFFSET + (enc_size_interval - 1);
		} else {
			video_output = (unsigned char *)(m_parm->video_addr) + m_parm->meta_offset + VIDEO_HEVC_META_OFFSET + (enc_size_interval - 1);
		}
		video_insert_sei_dummy_three_bytes(video_output, length, dummp_byte_enable, meta_length);
	}
}

void video_sei_read(video_meta_read_t *meta_read, unsigned char *video_input, video_meta_t *m_parm)
{
	int outputAmount = 0;
	int zeroCount = 0;
	int value = 0;
	int meta_store_size = 0;
	int meta_length = m_parm->meta_size;

	int isp_3a_enable = 0;

	if (isp_boot->fcs_status) {
		if (m_parm->meta_size > isp_boot->fcs_meta_total_size) {
			isp_3a_enable = 1;
		}
	} else {
		if (m_parm->meta_size > video_pre_init_param.video_meta_total_size) {
			isp_3a_enable = 1;
		}
	}
	if (isp_3a_enable) {
		meta_store_size =  sizeof(isp_statis_meta_t) + sizeof(isp_meta_t) + sizeof(af_statis_t) + sizeof(ae_statis_t) + sizeof(
							   awb_statis_t) + VIDEO_META_UUID_SIZE + VIDEO_META_3A_TAG_SIZE + meta_read->user_length;
	} else {
		meta_store_size = sizeof(isp_statis_meta_t) + sizeof(isp_meta_t) + VIDEO_META_UUID_SIZE + meta_read->user_length;
	}

	for (int i = 0; i < meta_length; i++) {
		value = video_input[i];
		if (zeroCount == 2 && value == VIDEO_START_CODE_DUMMY) {
			zeroCount = 0;
		} else {
			video_sei_buf[outputAmount++] = value;
		}

		if (value == 0) {
			zeroCount++;
		} else {
			zeroCount = 0;
		}
		if (outputAmount > meta_store_size) {
			break;
		}
	}
	memcpy(meta_read->uuid, video_sei_buf, VIDEO_META_UUID_SIZE);
	memcpy(&meta_read->isp_statis_meta, video_sei_buf + VIDEO_META_UUID_SIZE, sizeof(isp_statis_meta_t));
	memcpy(&meta_read->isp_meta_data, video_sei_buf + VIDEO_META_UUID_SIZE + sizeof(isp_statis_meta_t), sizeof(isp_meta_t));

	if (isp_3a_enable) {
		memcpy(meta_read->magic_num, video_sei_buf + VIDEO_META_UUID_SIZE + sizeof(isp_statis_meta_t) + sizeof(isp_meta_t), VIDEO_META_3A_TAG_SIZE);

		memcpy(&meta_read->af_result, video_sei_buf + VIDEO_META_UUID_SIZE + sizeof(isp_statis_meta_t) + sizeof(isp_meta_t) + VIDEO_META_3A_TAG_SIZE,
			   sizeof(af_statis_t));

		memcpy(&meta_read->ae_result, video_sei_buf + VIDEO_META_UUID_SIZE + sizeof(isp_statis_meta_t) + sizeof(isp_meta_t) + VIDEO_META_3A_TAG_SIZE + sizeof(
				   af_statis_t), sizeof(ae_statis_t));

		memcpy(&meta_read->awb_result, video_sei_buf + VIDEO_META_UUID_SIZE + sizeof(isp_statis_meta_t) + sizeof(isp_meta_t) + VIDEO_META_3A_TAG_SIZE + sizeof(
				   af_statis_t) + sizeof(ae_statis_t), sizeof(awb_statis_t));
	}
	if (meta_read->user_input && meta_read->user_length > 0) {
		memcpy(meta_read->user_input, video_sei_buf + meta_store_size - meta_read->user_length, meta_read->user_length);
	}
}

int video_get_meta_offset(int meta_size)
{
	if (isp_boot->fcs_status) {
		if (meta_size == isp_boot->fcs_meta_total_size) {
			return isp_boot->fcs_meta_offset;
		} else {
			return isp_boot->extra_fcs_meta_extend_offset;
		}
	} else {
		if (meta_size == video_pre_init_param.video_meta_total_size) {
			return video_pre_init_param.video_meta_offset;
		} else {
			return video_pre_init_param.video_meta_extend_offset;
		}
	}
}

void device_voe_lock(void)
{
	device_mutex_lock(RT_DEV_LOCK_VOE);
}

void device_voe_unlock(void)
{
	device_mutex_unlock(RT_DEV_LOCK_VOE);
}

int video_get_encoder_nalu_payload_info(unsigned char *frame_buf, unsigned int frame_size, int codec_type, video_encoder_nalu_payload_info_t *info)
{
	unsigned char type = 0;
	unsigned int frame_offset = 0;
	unsigned int nal_len = 4;
	int ret = -1;
	int iframe_type = 0;
	int pframe_type = 0;
	info->nalu_count = 0;
	if (codec_type == VIDEO_H264) {
		iframe_type = 0x05;
		pframe_type = 0x01;
	} else if (codec_type == VIDEO_HEVC) {
		iframe_type = 0x13;
		pframe_type = 0x01;
	} else {
		ret = -1;
		goto EXIT;
	}
	for (int i = 0; i < frame_size - nal_len; i++) {
		if (frame_buf[i] == 0x00 && frame_buf[i + 1] == 0x00) {
			if ((frame_buf[i + 2] == 0x00 && frame_buf[i + 3] == 0x01)) {
				if (codec_type == VIDEO_H264) {
					type = frame_buf[i + nal_len] & 0x1f;
				} else {
					type = (frame_buf[i + nal_len] & 0x7f) >> 1;
				}
				frame_offset = i;
				if (info->nalu_count >= NALU_PAYLOAD_MAX_SIZE) {
					video_dprintf(VIDEO_LOG_MSG, "info->nalu_count bigger %d than NALU_PAYLOAD_MAX_SIZE %d\r\n", info->nalu_count, NALU_PAYLOAD_MAX_SIZE);
					ret = -1;
					break;
				}
				info->nalu_info[info->nalu_count].offset = frame_offset;
				info->nalu_info[info->nalu_count].type = type;
				if (type == iframe_type || type == pframe_type) { //I or P frame, the last frame
					if (info->nalu_count == 0) {
						info->nalu_info[info->nalu_count].size = frame_size - frame_offset;
					} else {
						info->nalu_info[info->nalu_count - 1].size = frame_offset - info->nalu_info[info->nalu_count - 1].offset;
						info->nalu_info[info->nalu_count].size = frame_size - frame_offset;
					}
					ret = 0;
					info->nalu_count++;
					i += nal_len;
					break;
				} else {
					if (info->nalu_count > 0) {
						info->nalu_info[info->nalu_count - 1].size = frame_offset - info->nalu_info[info->nalu_count - 1].offset;
					}
					info->nalu_count++;
					i += nal_len;
				}

			}
		}
	}
EXIT:
	return ret;
}

int video_get_error_group(int error_id)
{
	//1:VOE, 2:ISP flow, 3: Driver, 4: Mod, 5: OSD
	return error_id >> 27 & 0xF;
}
