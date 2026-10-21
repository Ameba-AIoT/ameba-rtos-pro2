/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright 2026 AmebaPro2 WebRTC Contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Local WebRTC MMF Example — AmebaPro2
 *
 * This example demonstrates direct WebRTC streaming from AmebaPro2
 * to a browser without any cloud service or intermediary PC.
 *
 * Architecture:
 *   Camera → H.264 Encoder → MMF Mux → WebRTC Peer Connection → Browser
 *   Mic    → Opus Encoder  → MMF Mux ↗
 *
 * The browser connects directly to the AmebaPro2 via:
 *   - HTTP for signaling (SDP offer/answer exchange)
 *   - WebRTC (ICE/DTLS/SRTP) for media
 *
 * No AWS KVS, no KVS credentials, no cloud services required.
 */

#include "platform_opts.h"
#include "log_service.h"

#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_miso.h"

#include "avcodec.h"
#include "module_video.h"
#include "module_audio.h"
#include "module_opusc.h"

#include "mmf2_pro2_video_config.h"
#include "video_example_media_framework.h"

#include "example_local_webrtc_mmf.h"
#include "module_local_webrtc.h"

/*
 * Local WebRTC configuration
 *
 * ISP channel : 0
 * Video type  : H.264
 * Audio codec : OPUS
 */
#define V1_CHANNEL      0
#define V1_RESOLUTION   VIDEO_FHD
#define V1_FPS          15
#define V1_GOP          15
#define V1_BPS          (512 * 1024)
#define V1_RCMODE       2 /* 1: CBR, 2: VBR */

#define VIDEO_TYPE      VIDEO_H264
#define VIDEO_CODEC     AV_CODEC_ID_H264

#if V1_RESOLUTION == VIDEO_VGA
#define V1_WIDTH    640
#define V1_HEIGHT   480
#elif V1_RESOLUTION == VIDEO_HD
#define V1_WIDTH    1280
#define V1_HEIGHT   720
#elif V1_RESOLUTION == VIDEO_FHD
#define V1_WIDTH    1920
#define V1_HEIGHT   1080
#endif

static mm_context_t *video_v1_ctx           = NULL;
static mm_context_t *audio_ctx              = NULL;
static mm_context_t *local_webrtc_ctx       = NULL;

static mm_context_t *opusc_ctx              = NULL;

static mm_siso_t *siso_audio_a1             = NULL;
static mm_miso_t *miso_local_webrtc         = NULL;

static video_params_t video_v1_params = {
	.stream_id = V1_CHANNEL,
	.type = VIDEO_TYPE,
	.resolution = V1_RESOLUTION,
	.width = V1_WIDTH,
	.height = V1_HEIGHT,
	.bps = V1_BPS,
	.fps = V1_FPS,
	.gop = V1_GOP,
	.rc_mode = V1_RCMODE,
	.use_static_addr = 1
};

static audio_params_t audio_params = {
	.sample_rate = ASR_8KHZ,
	.word_length = WL_16BIT,
	.mic_gain    = MIC_0DB,
	.dmic_l_gain    = DMIC_BOOST_24DB,
	.dmic_r_gain    = DMIC_BOOST_24DB,
	.use_mic_type   = USE_AUDIO_AMIC,
	.channel     = 1,
	.mix_mode = 0,
	.enable_record = 0
};

static opusc_params_t opusc_params  = {
	.sample_rate = 8000,
	.channel = 1,
	.bit_length = 16,
	.complexity = 5,
	.bitrate = 25000,
	.use_framesize = 40,
	.enable_vbr = 1,
	.vbr_constraint = 0,
	.packetLossPercentage = 0,
	.opus_application = OPUS_APPLICATION_AUDIO
};

#include "wifi_conf.h"
#include "lwip_netconf.h"

#define WIFI_WAIT_TIME_MS 500
static void wifi_common_init(void)
{
	uint32_t wifi_wait_count = 0;

	while (!((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) &&
			 (*(uint32_t *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
		vTaskDelay(10);
		wifi_wait_count++;
		if (wifi_wait_count == WIFI_WAIT_TIME_MS) {
			printf("\r\nuse ATW0, ATW1, ATWC to make wifi connection\r\n");
			printf("wait for wifi connection...\r\n");
		}
	}
}

static void atcmd_local_webrtc_init(void);
static SemaphoreHandle_t local_webrtc_stop_sem;

static void example_local_webrtc_mmf_thread(void *param)
{
	(void)param;
	local_webrtc_stop_sem = xSemaphoreCreateBinary();
	atcmd_local_webrtc_init();

	if (!voe_boot_fsc_status()) {
		if (!((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) &&
			  (*(uint32_t *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
			wifi_common_init();
		}
	}

	/* ---- Open Local WebRTC module ---- */
	local_webrtc_ctx = mm_module_open(&local_webrtc_module);
	if (local_webrtc_ctx) {
		pWebrtcMmContext = local_webrtc_ctx;
		mm_module_ctrl(local_webrtc_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(local_webrtc_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(local_webrtc_ctx, CMD_LOCAL_WEBRTC_SET_APPLY, 0);
	} else {
		printf("Local WebRTC open fail\r\n");
		goto cleanup;
	}

	/* ---- Video ---- */
	{
		int voe_heap_size;
		voe_heap_size = video_voe_presetting(1, V1_WIDTH, V1_HEIGHT, V1_BPS, 0,
											 0, 0, 0, 0, 0,
											 0, 0, 0, 0, 0,
											 0, 0, 0);
		(void)voe_heap_size;
	}

	video_v1_ctx = mm_module_open(&video_module);
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_SET_PARAMS, (int)&video_v1_params);
		mm_module_ctrl(video_v1_ctx, MM_CMD_SET_QUEUE_LEN, V1_FPS * 3);
		mm_module_ctrl(video_v1_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_DYNAMIC);
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_APPLY, V1_CHANNEL);
	} else {
		printf("Video open fail\r\n");
		goto cleanup;
	}

	/* ---- Audio ---- */
	audio_ctx = mm_module_open(&audio_module);
	if (audio_ctx) {
		mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_PARAMS, (int)&audio_params);
		mm_module_ctrl(audio_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(audio_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(audio_ctx, CMD_AUDIO_APPLY, 0);
	} else {
		printf("Audio open fail\r\n");
		goto cleanup;
	}

	/* ---- Opus Encoder ---- */
	opusc_ctx = mm_module_open(&opusc_module);
	if (opusc_ctx) {
		mm_module_ctrl(opusc_ctx, CMD_OPUSC_SET_PARAMS, (int)&opusc_params);
		mm_module_ctrl(opusc_ctx, MM_CMD_SET_QUEUE_LEN, 6);
		mm_module_ctrl(opusc_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
		mm_module_ctrl(opusc_ctx, CMD_OPUSC_APPLY, 0);
	} else {
		printf("Opus encoder open fail\r\n");
		goto cleanup;
	}

	/* ---- SISO: Audio → Opus Encoder ---- */
	siso_audio_a1 = siso_create();
	if (siso_audio_a1) {
		siso_ctrl(siso_audio_a1, MMIC_CMD_ADD_INPUT, (uint32_t)audio_ctx, 0);
		siso_ctrl(siso_audio_a1, MMIC_CMD_ADD_OUTPUT, (uint32_t)opusc_ctx, 0);
		siso_ctrl(siso_audio_a1, MMIC_CMD_SET_STACKSIZE, 24 * 1024, 0);
		if (siso_start(siso_audio_a1) != 0) {
			printf("SISO audio start fail\r\n");
			goto cleanup;
		}
	} else {
		printf("SISO audio open fail\r\n");
		goto cleanup;
	}

	/* ---- MISO: Video + Audio → WebRTC ---- */
	miso_local_webrtc = miso_create();
	if (miso_local_webrtc) {
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
		miso_ctrl(miso_local_webrtc, MMIC_CMD_SET_SECURE_CONTEXT, 1, 0);
#endif
		miso_ctrl(miso_local_webrtc, MMIC_CMD_ADD_INPUT0, (uint32_t)video_v1_ctx, 0);
		miso_ctrl(miso_local_webrtc, MMIC_CMD_ADD_INPUT1, (uint32_t)opusc_ctx, 0);
		miso_ctrl(miso_local_webrtc, MMIC_CMD_ADD_OUTPUT, (uint32_t)local_webrtc_ctx, 0);
		if (miso_start(miso_local_webrtc) != 0) {
			printf("MISO local webrtc start fail\r\n");
			goto cleanup;
		}
	} else {
		printf("MISO local webrtc open fail\r\n");
		goto cleanup;
	}

	{
		/* LwIP stores IP in network byte order (MSB first) */
		uint8_t *ip_bytes = (uint8_t *)LwIP_GetIP(0);
		printf("\r\n========================================\r\n");
		printf("  AmebaPro2 Local WebRTC Started\r\n");
		printf("  Open a browser and navigate to:\r\n");
		printf("  http://%u.%u.%u.%u\r\n",
			   ip_bytes[0], ip_bytes[1], ip_bytes[2], ip_bytes[3]);
		printf("========================================\r\n");
	}

	/*
	 * Note: WebRTC media pipeline is NOT started here.
	 * It will be started automatically by AppMediaSourcePort_Start()
	 * when a browser connects and the WebRTC DTLS handshake completes.
	 */

	/* Wait for stop signal */
	xSemaphoreTake(local_webrtc_stop_sem, portMAX_DELAY);

cleanup:
	/* Stop modules */
	if (siso_audio_a1) {
		siso_pause(siso_audio_a1);
	}
	if (miso_local_webrtc) {
		miso_pause(miso_local_webrtc, MM_OUTPUT);
	}

	if (local_webrtc_ctx) {
		mm_module_ctrl(local_webrtc_ctx, CMD_LOCAL_WEBRTC_STOP, 0);
	}
	if (video_v1_ctx) {
		mm_module_ctrl(video_v1_ctx, CMD_VIDEO_STREAM_STOP, V1_CHANNEL);
	}
	if (audio_ctx) {
		mm_module_ctrl(audio_ctx, CMD_AUDIO_SET_TRX, 0);
	}

	/* Delete modules */
	if (siso_audio_a1) {
		siso_audio_a1 = siso_delete(siso_audio_a1);
	}
	if (miso_local_webrtc) {
		miso_local_webrtc = miso_delete(miso_local_webrtc);
	}

	if (local_webrtc_ctx) {
		local_webrtc_ctx = mm_module_close(local_webrtc_ctx);
		pWebrtcMmContext = NULL;
	}
	if (video_v1_ctx) {
		video_v1_ctx = mm_module_close(video_v1_ctx);
	}
	if (audio_ctx) {
		audio_ctx = mm_module_close(audio_ctx);
	}
	if (opusc_ctx) {
		mm_module_ctrl(opusc_ctx, CMD_OPUSC_STOP, 0);
		opusc_ctx = mm_module_close(opusc_ctx);
	}

	/* video_deinit(): calls hal_video_deinit() which has an internal state guard,
	 * so it is safe even though we initialized video via video_voe_presetting()
	 * and mm_module_open(&video_module) rather than video_init(). */
	video_deinit();

	if (local_webrtc_stop_sem) {
		vSemaphoreDelete(local_webrtc_stop_sem);
	}
	local_webrtc_stop_sem = NULL;

	vTaskDelete(NULL);
}

static void fLocalWebrtcStop(void *arg)
{
	(void)arg;
	if (local_webrtc_stop_sem != NULL) {
		xSemaphoreGive(local_webrtc_stop_sem);
	}
}

static log_item_t local_webrtc_items[] = {
	{"LWST", fLocalWebrtcStop},
};

static void atcmd_local_webrtc_init(void)
{
	log_service_add_table(local_webrtc_items,
						  sizeof(local_webrtc_items) / sizeof(local_webrtc_items[0]));
}

void example_local_webrtc_mmf(void)
{
	if (xTaskCreate(example_local_webrtc_mmf_thread,
					((const char *)"example_local_webrtc_mmf_thread"),
					8192, NULL,
					tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\r\nexample_local_webrtc_mmf_thread: Create Task Error\n");
	}
}
