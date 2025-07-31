#if 0
/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include "example_media_framework.h"

isp_params_t isp_v1_params = {
	.width    = V1_WIDTH,
	.height   = V1_HEIGHT,
	.fps      = V1_FPS,
	.slot_num = V1_HW_SLOT,
	.buff_num = V1_SW_SLOT,
	.format   = ISP_FORMAT_YUV420_SEMIPLANAR
};

isp_params_t isp_v2_params = {
	.width    = V2_WIDTH,
	.height   = V2_HEIGHT,
	.fps      = V2_FPS,
	.slot_num = V2_HW_SLOT,
	.buff_num = V2_SW_SLOT,
	.format   = ISP_FORMAT_YUV420_SEMIPLANAR
};

h264_params_t h264_v1_params = {
	.width          = V1_WIDTH,
	.height         = V1_HEIGHT,
	.bps            = V1_BITRATE,
	.fps            = V1_FPS,
	.gop            = V1_FPS,
	.rc_mode        = V1_H264_RCMODE,
	.rotation       = 0,
	.mem_total_size = V1_BUFFER_SIZE,
	.mem_block_size = V1_BLOCK_SIZE,
	.mem_frame_size = V1_FRAME_SIZE
};

rtsp2_params_t rtsp2_v1_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = AV_CODEC_ID_H264,
			.fps      = V1_FPS,
			.bps      = V1_BITRATE
		}
	}
};

#if ISP_BOOT_MODE_ENABLE
#define CINIT_DATA_SECTION SECTION(".cinit.data")
CINIT_DATA_SECTION isp_boot_stream_t isp_boot_stream = {
	.width = V1_WIDTH,
	.height = V1_HEIGHT,
	.isp_id = 0,
	.hw_slot_num = V1_HW_SLOT,
	.fps = V1_FPS,
	.format = ISP_FORMAT_YUV420_SEMIPLANAR,
	.pin_idx = ISP_PIN_IDX,
	.mode = ISP_FAST_BOOT,
	.interface = ISP_INTERFACE_MIPI,
	.clk = SENSOR_CLK_USE,
	.sensor_fps = SENSOR_FPS,
	.isp_fw_location = ISP_FW_LOCATION,
	.wake_mode = WAKE_FROM_BOOT,//WAKE_FROM_GPIO//WAKE_FROM_WLAN
#if SENSOR_USE == SENSOR_ALL
	.isp_multi_sensor = SENSOR_DEFAULT
#else
	.isp_multi_sensor = 0
#endif
};
#endif

#if CONFIG_EXAMPLE_DOORBELL
h264_params_t h264_v2_params = {
	.width          = V2_WIDTH,
	.height         = V2_HEIGHT,
	.bps            = V2_BITRATE,
	.fps            = V2_FPS,
	.gop            = V2_FPS,
	.rc_mode        = V2_H264_RCMODE,
	//.auto_qp        =1,
	.mem_total_size = V2_BUFFER_SIZE,
	.mem_block_size = V2_BLOCK_SIZE,
	.mem_frame_size = V2_FRAME_SIZE
};
#else
h264_params_t h264_v2_params = {
	.width          = V2_WIDTH,
	.height         = V2_HEIGHT,
	.bps            = V2_BITRATE,
	.fps            = V2_FPS,
	.gop            = V2_FPS,
	.rc_mode        = V2_H264_RCMODE,
	.mem_total_size = V2_BUFFER_SIZE,
	.mem_block_size = V2_BLOCK_SIZE,
	.mem_frame_size = V2_FRAME_SIZE
};
#endif

rtsp2_params_t rtsp2_v2_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = AV_CODEC_ID_H264,
			.fps      = V2_FPS,
			.bps      = V2_BITRATE
		}
	}
};

isp_params_t isp_v3_params = {
	.width    = V3_WIDTH,
	.height   = V3_HEIGHT,
	.fps      = V3_FPS,
	.slot_num = V3_HW_SLOT,
	.buff_num = V3_SW_SLOT,
	.format   = ISP_FORMAT_YUV420_SEMIPLANAR
};

jpeg_params_t jpeg_v3_params = {
	.width          = V3_WIDTH,
	.height         = V3_HEIGHT,
	.level          = V3_JPEG_LEVEL,
	.fps            = V3_FPS,
	.mem_total_size = V3_BUFFER_SIZE,
	.mem_block_size = V3_BLOCK_SIZE,
	.mem_frame_size = V3_FRAME_SIZE
};

rtsp2_params_t rtsp2_v3_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = AV_CODEC_ID_MJPEG,
			.fps      = V3_FPS,
		}
	}
};

rtsp2_params_t rtsp2_a_params = {
	.type = AVMEDIA_TYPE_AUDIO,
	.u = {
		.a = {
			.codec_id   = AV_CODEC_ID_MP4A_LATM,
			.channel    = 1,
			.samplerate = 8000
		}
	}
};

rtsp2_params_t rtsp2_a_pcmu_params = {
	.type = AVMEDIA_TYPE_AUDIO,
	.u = {
		.a = {
			.codec_id   = AV_CODEC_ID_PCMU,
			.channel    = 1,
			.samplerate = 8000
		}
	}
};


#if USING_I2S_MIC

i2s_params_t i2s_params = {
	.sample_rate = SR_32KHZ,
	.word_length = WL_24b,
	.out_sample_rate = SR_8KHZ,
	.out_word_length = WL_16b,
	.mic_gain    = MIC_40DB,
	.channel     = 2,
	.out_channel = 1,
	.enable_aec  = 0
};

audio_params_t audio_params = {
	.sample_rate = ASR_8KHZ,
	.word_length = WL_16BIT,
	.mic_gain    = MIC_40DB,
	.channel     = 1,
};

#else
audio_params_t audio_params = {
	.sample_rate = ASR_8KHZ,
	.word_length = WL_16BIT,
	.mic_gain    = MIC_40DB,
	.channel     = 1,
};
#endif

aac_params_t aac_params = {
	.sample_rate = 8000,
	.channel = 1,
	.bit_length = FAAC_INPUT_16BIT,
	.output_format = 1,
	.mpeg_version = MPEG4,
	.mem_total_size = 10 * 1024,
	.mem_block_size = 128,
	.mem_frame_size = 1024
};

opusc_params_t opusc_params = {
	//voice	8000/12000/16000/24000/48000
	//audio	8000/16000/24000/48000
	.sample_rate = 8000,//16000,//
	.channel = 1,
	.bit_length = 16, //16 recommand
	.complexity = 5,  //0~10
	.bitrate = 25000, //default 25000
	.use_framesize = 0,  //set 0 only when using audio loop
	.enable_vbr = 1,
	.vbr_constraint = 0,
	.packetLossPercentage = 0,
	.opus_application = OPUS_APPLICATION_AUDIO

};

opusc_params_t opusc_rtsp_params = {
	//voice	8000/12000/16000/24000/48000
	//audio	8000/16000/24000/48000
	.sample_rate = 8000,//16000,//
	.channel = 1,
	.bit_length = 16,     //16 recommand
	.complexity = 5,      //0~10
	.bitrate = 25000,     //default 25000
	.use_framesize = 20,//10,// //needs to the same or bigger than AUDIO_DMA_PAGE_SIZE/(sample_rate/1000)/2 but less than 60
	.enable_vbr = 1,
	.vbr_constraint = 0,
	.packetLossPercentage = 0,
	.opus_application = OPUS_APPLICATION_AUDIO

};

rtsp2_params_t rtsp2_a_opus_params = {
	.type = AVMEDIA_TYPE_AUDIO,
	.u = {
		.a_opus = {
			.codec_id   = AV_CODEC_ID_OPUS,
			.channel    = 1,
			.samplerate = 8000,//16000,//
			.frame_size = 20//10//        //equal to use_framesize in opusc_rtsp_params
		}
	}
};

opusd_params_t opusd_params = {
	.sample_rate = 8000,//16000,//
	.channel = 1,
	.bit_length = 16,         //16 recommand
	.frame_size_in_msec = 10, //will not be uused
	.with_opus_enc = 1,       //enable semaphore if the application with opus encoder
	.opus_application = OPUS_APPLICATION_AUDIO
};

opusd_params_t opusd_only_params = {
	.sample_rate = 8000,//16000,//
	.channel = 1,
	.bit_length = 16,         //16 recommand
	.frame_size_in_msec = 10, //will not be uused
	.with_opus_enc = 0,       //enable semaphore if the application with opus encoder
	.opus_application = OPUS_APPLICATION_AUDIO
};
#if 0
#if ISP_BOOT_MODE_ENABLE
mp4_params_t mp4_params = {
	.width          = V1_WIDTH,
	.height         = V1_HEIGHT,
	.fps            = V1_FPS,
	.gop            = V1_FPS,

	.sample_rate = 8000,
	.channel = 0,

	.record_length = 10, //seconds
	.record_type = STORAGE_ALL,
	.record_file_num = 1,
	.record_file_name = "AmebaPro_recording",
	.fatfs_buf_size = 224 * 1024, /* 32kb multiple */
};
#else
mp4_params_t mp4_params = {
	.width          = V1_WIDTH,
	.height         = V1_HEIGHT,
	.fps            = V1_FPS,
	.gop            = V1_FPS,

	.sample_rate = 8000,
	.channel = 1,

	.record_length = 30, //seconds
	.record_type = STORAGE_ALL,
	.record_file_num = 3,
	.record_file_name = "AmebaPro_recording",
	.fatfs_buf_size = 224 * 1024, /* 32kb multiple */
};
#endif
mp4_params_t mp4_params = {
	.width          = V1_WIDTH,
	.height         = V1_HEIGHT,
	.fps            = V1_FPS,
	.gop            = V1_FPS,

	.sample_rate = 8000,
	.channel = 1,

	.record_length = 30, //seconds
	.record_type = STORAGE_ALL,
	.record_file_num = 3,
	.record_file_name = "AmebaPro_recording",
	.fatfs_buf_size = 224 * 1024, /* 32kb multiple */
};
#endif

mp4_params_t mp4_v1_params = {
	.width          = V1_WIDTH,
	.height         = V1_HEIGHT,
	.fps            = V1_FPS,
	.gop            = V1_FPS,

	.sample_rate = 8000,
	.channel = 1,

	.record_length = 30, //seconds
	.record_type = STORAGE_ALL,
	.record_file_num = 3,
	.record_file_name = "AmebaPro_recording",
	.fatfs_buf_size = 224 * 1024, /* 32kb multiple */
};
mp4_params_t mp4_v2_params = {
	.width          = V2_WIDTH,
	.height         = V2_HEIGHT,
	.fps            = V2_FPS,
	.gop            = V2_FPS,

	.sample_rate = 8000,
	.channel = 1,

	.record_length = 30, //seconds
	.record_type = STORAGE_ALL,
	.record_file_num = 3,
	.record_file_name = "AmebaPro_recording",
	.fatfs_buf_size = 224 * 1024, /* 32kb multiple */
};
mp4_params_t mp4_v3_params = {
	.width          = V3_WIDTH,
	.height         = V3_HEIGHT,
	.fps            = V3_FPS,
	.gop            = V3_FPS,

	.sample_rate = 8000,
	.channel = 1,

	.record_length = 30, //seconds
	.record_type = STORAGE_ALL,
	.record_file_num = 3,
	.record_file_name = "AmebaPro_recording",
	.fatfs_buf_size = 224 * 1024, /* 32kb multiple */
};

g711_params_t g711e_params = {
	.codec_id = AV_CODEC_ID_PCMU,
	.buf_len = 2048,
	.mode     = G711_ENCODE
};

g711_params_t g711d_params = {
	.codec_id = AV_CODEC_ID_PCMU,
	.buf_len = 2048,
	.mode     = G711_DECODE
};

aad_params_t aad_params = {
	.sample_rate = 8000,
	.channel = 1,
	.type = TYPE_ADTS
};

aad_params_t aad_rtp_params = {
	.sample_rate = 8000,
	.channel = 1,
	.type = TYPE_RTP_RAW
};

rtp_params_t rtp_aad_params = {
	.valid_pt = 0xFFFFFFFF,
	.port = 16384,
	.frame_size = 1500,
	.cache_depth = 6
};

rtp_params_t rtp_opusd_params = {
	.valid_pt = 0xFFFFFFFF,
	.port = 16384,
	.frame_size = 1500,
	.cache_depth = 15
};

rtp_params_t rtp_g711d_params = {
	.valid_pt = 0xFFFFFFFF,
	.port = 16384,
	.frame_size = 200,
	.cache_depth = 64
};

array_params_t doorbell_pcmu_array_params = {
	.type = AVMEDIA_TYPE_AUDIO,
	.codec_id = AV_CODEC_ID_PCMU,
	.mode = ARRAY_MODE_ONCE,
	.u = {
		.a = {
			.channel    = 1,
			.samplerate = 8000,
			.frame_size = 160,
		}
	}
};

array_params_t pcmu_array_params = {
	.type = AVMEDIA_TYPE_AUDIO,
	.codec_id = AV_CODEC_ID_PCMU,
	.mode = ARRAY_MODE_LOOP,
	.u = {
		.a = {
			.channel    = 1,
			.samplerate = 8000,
			.frame_size = 160,
		}
	}
};

array_params_t aac_array_params = {
	.type = AVMEDIA_TYPE_AUDIO,
	.codec_id = AV_CODEC_ID_MP4A_LATM,
	.mode = ARRAY_MODE_LOOP,
	.u = {
		.a = {
			.channel    = 1,
			.samplerate = 44100,
			.sample_bit_length = 16,
		}
	}
};

rtsp2_params_t rtsp2_aac_array_params = {
	.type = AVMEDIA_TYPE_AUDIO,
	.u = {
		.a = {
			.codec_id   = AV_CODEC_ID_MP4A_LATM,
			.channel    = 1,
			.samplerate = 44100,
		}
	}
};

rtsp2_params_t rtsp2_array_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.u = {
		.v = {
			.codec_id = AV_CODEC_ID_H264,
			.fps      = 25,
			.bps      = 0, // if unknown
		}
	}
};

array_params_t h264_array_params = {
	.type = AVMEDIA_TYPE_VIDEO,
	.codec_id = AV_CODEC_ID_H264,
	.mode = ARRAY_MODE_LOOP,
	.u = {
		.v = {
			.fps    = 25,
			.h264_nal_size = 4,
		}
	}
};

httpfs_params_t httpfs_params = {
	.fileext = "mp4",
	.filedir = "VIDEO",
	.request_string = "/video_get.mp4",
	.fatfs_buf_size = 1024
};

dup_params_t dup_params = {
	.buf_len = 320,
	.codec_id = 0
};

mm_context_t *isp_v1_ctx           = NULL;
mm_context_t *h264_v1_ctx          = NULL;
mm_context_t *rtsp2_v1_ctx         = NULL;
mm_siso_t *siso_isp_h264_v1        = NULL;
mm_siso_t *siso_h264_rtsp_v1       = NULL;

mm_context_t *isp_v2_ctx           = NULL;
mm_context_t *h264_v2_ctx          = NULL;
mm_context_t *rtsp2_v2_ctx         = NULL;
mm_siso_t *siso_isp_h264_v2        = NULL;
mm_siso_t *siso_h264_rtsp_v2       = NULL;

mm_context_t *isp_v3_ctx           = NULL;
mm_context_t *jpeg_v3_ctx          = NULL;
mm_context_t *rtsp2_v3_ctx         = NULL;
mm_siso_t *siso_isp_jpeg_v3        = NULL;
mm_siso_t *siso_jpeg_rtsp_v3       = NULL;

mm_simo_t *simo_h264_rtsp_v2_v3    = NULL;


mm_context_t *audio_ctx            = NULL;
mm_context_t *i2s_ctx             = NULL;
mm_context_t *aac_ctx              = NULL;
mm_context_t *rtsp2_ctx            = NULL;
mm_siso_t *siso_audio_aac          = NULL;
mm_siso_t *siso_aac_rtsp           = NULL;

mm_miso_t *miso_h264_aac_rtsp      = NULL;
mm_mimo_t *mimo_2v_1a_rtsp         = NULL;

mm_context_t *rtsp2_v4_ctx         = NULL;

mm_context_t *mp4_ctx              = NULL;
mm_miso_t *miso_h264_aac_mp4       = NULL;
mm_mimo_t *mimo_2v_1a_rtsp_mp4     = NULL;

mm_mimo_t *mimo_1v_1a_rtsp_mp4     = NULL;

mm_context_t *array_ctx            = NULL;
mm_miso_t *miso_h264_g711_rtsp     = NULL;
mm_siso_t *siso_rtp_g711d          = NULL;
mm_miso_t *miso_rtp_array_g711d    = NULL;

mm_siso_t *siso_array_rtsp         = NULL;

mm_siso_t *siso_audio_loop       = NULL;

mm_context_t *g711e_ctx        = NULL;
mm_context_t *g711d_ctx        = NULL;
mm_siso_t *siso_audio_g711e      = NULL;
mm_siso_t *siso_g711e_rtsp      = NULL;
mm_siso_t *siso_g711_e2d       = NULL;
mm_siso_t *siso_g711d_audio      = NULL;

mm_context_t *aad_ctx        = NULL;
mm_siso_t *siso_aac_e2d      = NULL;
mm_siso_t *siso_aad_audio      = NULL;

mm_context_t *rtp_ctx        = NULL;
mm_siso_t *siso_rtp_aad      = NULL;

mm_context_t *httpfs_ctx        = NULL;

mm_context_t *opusc_ctx           = NULL;
mm_context_t *opusd_ctx           = NULL;
mm_siso_t *siso_audio_opusc       = NULL;
mm_siso_t *siso_opusd_audio       = NULL;
mm_siso_t *siso_opus_e2d          = NULL;
mm_siso_t *siso_opusc_rtsp        = NULL;
mm_siso_t *siso_rtp_opusd         = NULL;
#endif