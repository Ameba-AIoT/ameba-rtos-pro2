#include <stdio.h>
#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
#include "diag.h"
#include <stdlib.h>
#include "platform_opts.h"
#include <fatfs_ext/inc/ff_driver.h>
#include <disk_if/inc/sdcard.h>
#include "section_config.h"
#include "platform_stdlib.h"
#include "ameba_soc.h"
#include "audio_track.h"
#include "ff.h"

#include "haac/aacdec.h"

#ifndef SDRAM_BSS_SECTION
#define SDRAM_BSS_SECTION       SECTION(".sdram.bss")
#endif

//------------------------------------- ---CONFIG Parameters-----------------------------------------------//
#define INPUT_FRAME_SIZE 170*1024
#define AUDIO_DMA_PAGE_SIZE  2048

//#include "sr48000_br320_stereo.c"
#define FILE_NAME "160.aac"

static SDRAM_BSS_SECTION uint8_t AAC_Buf[INPUT_FRAME_SIZE];
SDRAM_BSS_SECTION static uint8_t decodebuf[ AAC_MAX_NCHANS * AAC_MAX_NSAMPS * sizeof(int16_t) ];

static uint32_t audio_tx_pcm_cache_len = 0;
static int16_t audio_tx_pcm_cache[AUDIO_DMA_PAGE_SIZE / 2];


static struct RTAudioTrack *g_audio_track = NULL ;
#if 1
static void audio_aac_play_pcm(int16_t *buf, uint32_t len)
{
	static int index = 0;
	for (int i = 0; i < len; i++) {
		audio_tx_pcm_cache[audio_tx_pcm_cache_len++] = buf[i];
		if (audio_tx_pcm_cache_len == AUDIO_DMA_PAGE_SIZE / 2) {
			printf("%s Line %d:index=%d\n", __FILE__, __LINE__, index++);
			RTAudioTrack_Write(g_audio_track, ((u8 *)audio_tx_pcm_cache), AUDIO_DMA_PAGE_SIZE, true);
			audio_tx_pcm_cache_len = 0;
		}
	}
}
#endif

static void initialize_audio_track(uint8_t ch_num, int sample_rate)
{
	RTAudioTrackConfig  track_config;
	track_config.sample_rate = sample_rate;
	track_config.format = RTAUDIO_FORMAT_PCM_16_BIT;///check this value
	track_config.channel_count = ch_num;
	track_config.category_type = RTAUDIO_CATEGORY_MEDIA;
	track_config.buffer_bytes = RTAudioTrack_GetMinBufferBytes(g_audio_track, RTAUDIO_CATEGORY_MEDIA, sample_rate, RTAUDIO_FORMAT_PCM_16_BIT, ch_num) * 4;

	RTAudioTrack_Init(g_audio_track, &track_config);
	printf("%s Line %d:ch_num=%d rate=%d\n", __FILE__, __LINE__, ch_num, sample_rate);
	RTAudioTrack_Start(g_audio_track);
}

void audio_play_sd_aac(u8 *filename)
{
	HAACDecoder	hAACDecoder;
	AACFrameInfo frameInfo;

	int drv_num = 0;
	int frame_size = 0;
	u32 read_length = 0;
	uint8_t first_frame = 1;
	FRESULT res;
	FATFS 	m_fs;
	FIL		m_file;
	char	logical_drv[4]; //root diretor
	char abs_path[32]; //Path to input file
	DWORD bytes_left;
	DWORD file_size;
	int channel_sel = 1;
	volatile u32 tim1 = 0;
	volatile u32 tim2 = 0;
	int ret = 0;
	int i;
	int decodebytesLeft;

	//WavHeader pwavHeader;
	//u32 wav_length = 0;
	//u32 wav_offset = 0;
	int *ptx_buf;
	printf("emter audio_play_sd_aac ......\n");
	drv_num = FATFS_RegisterDiskDriver(&SD_disk_Driver);
	if (drv_num < 0) {
		printf("Rigester disk driver to FATFS fail.\n");
		return;
	} else {
		logical_drv[0] = drv_num + '0';
		logical_drv[1] = ':';
		logical_drv[2] = '/';
		logical_drv[3] = 0;
	}

	if (f_mount(&m_fs, logical_drv, 1) != FR_OK) {
		printf("FATFS mount logical drive fail, please format DISK to FAT16/32.\n");
		goto unreg;
	}
	memset(abs_path, 0x00, sizeof(abs_path));
	strcpy(abs_path, logical_drv);
	sprintf(&abs_path[strlen(abs_path)], "%s", filename);

	//Open source file
	res = f_open(&m_file, abs_path, FA_OPEN_EXISTING | FA_READ); // open read only file
	if (res != FR_OK) {
		printf("Open source file %s fail.\n", abs_path);
		goto umount;
	}

	file_size = f_size(&m_file);
	bytes_left = file_size;
	printf("File size is %ld\n", file_size);

	hAACDecoder = AACInitDecoder();
	if (!hAACDecoder) {
		printf("aac context create fail\n");
		goto exit;
	}
	tim1 = rtw_get_current_time();
	printf("cur time is %ld\n", tim1);
	/* Read a block */
	if (bytes_left >= INPUT_FRAME_SIZE) {
		decodebytesLeft = INPUT_FRAME_SIZE;
		res = f_read(&m_file, AAC_Buf, INPUT_FRAME_SIZE, (UINT *)&read_length);
	} else if (bytes_left > 0) {
		decodebytesLeft = bytes_left;
		res = f_read(&m_file, AAC_Buf, bytes_left, (UINT *)&read_length);
	}
	if ((res != FR_OK)) {
		printf("Wav play done !\n");
		return;
	}
	uint8_t *srcbuf = AAC_Buf ;
	do {
		printf("%s Line %d: decodebytesLeft=%d \n", __FILE__, __LINE__, decodebytesLeft);
//		printf("%s Line %d: %x-%x-%x-%x/0x%x \n", __FILE__, __LINE__, srcbuf[0], srcbuf[1], srcbuf[2], srcbuf[3],AAC_Buf);
		ret = AACDecode(hAACDecoder, &srcbuf, &decodebytesLeft, (void *)decodebuf);
		printf("%s Line %d: decodebytesLeft=%d \n", __FILE__, __LINE__, decodebytesLeft);
		if (!ret && decodebytesLeft > 0) {
			AACGetLastFrameInfo(hAACDecoder, &frameInfo);
			if (first_frame) {
				initialize_audio_track(frameInfo.nChans, frameInfo.sampRateOut);
				first_frame = 0;
			}
#if 0
			if (frameInfo.nChans == 2) {
				if (channel_sel == 0) {
					for (i = 0; i < frameInfo.outputSamps / 2; i++) {
						decodebuf[i] = decodebuf[i * 2];
					}
				} else if (channel_sel == 1) {
					for (i = 0; i < frameInfo.outputSamps / 2; i++) {
						decodebuf[i] = decodebuf[i * 2 + 1];
					}
				}
				audio_aac_play_pcm((void *)decodebuf, frameInfo.outputSamps / 2);
			} else
#endif
			{
				audio_aac_play_pcm((void *)decodebuf, frameInfo.outputSamps);
			}
		}  else {
			if (decodebytesLeft != 0) {
				printf("error: %d\r\n", ret);
			}
			break;
		}
	} while (1);
	tim2 = rtw_get_current_time();
	printf("Decode time = %dms\n", (tim2 - tim1));
	printf("PCM done\n");

exit:
	// close source file
	res = f_close(&m_file);
	if (res) {
		printf("close file (%s) fail.\n", filename);
	}

umount:
	if (f_unmount(logical_drv) != FR_OK) {
		printf("FATFS unmount logical drive fail.\n");
	}

unreg:
	SD_DeInit();
	if (FATFS_UnRegisterDiskDriver(drv_num)) {
		printf("Unregister disk driver from FATFS fail.\n");
	}

}

void example_audio_aac_thread(void *param)
{
	printf("Audio codec demo begin enter example_audio_mp3_thread ......\n");
	struct RTAudioTrack *audio_track;
	char file[16] = FILE_NAME;
	audio_track = RTAudioTrack_Create();
	if (!audio_track) {
		printf("new RTAudioTrack failed");
		goto exit;
	}
	printf("%s\nLine %d:\n", __FILE__, __LINE__);
	g_audio_track = audio_track ;

	//printf("%s Line %d:\n", __FILE__, __LINE__);
	//audio_play_ac3(sr48000_br320_stereo_aac, sr48000_br320_stereo_aac_len);
	audio_play_sd_aac(file);
	//printf("%s Line %d:\n", __FILE__, __LINE__);
	RTAudioTrack_Stop(audio_track);
	//printf("%s Line %d:\n", __FILE__, __LINE__);
	//RTAudioTrack_Release(audio_track);
	//printf("%s Line %d:\n", __FILE__, __LINE__);
	RTAudioTrack_Destroy(audio_track);
	//printf("%s Line %d:\n", __FILE__, __LINE__);

exit:
	vTaskDelete(NULL);
}


void example_audio_aac(void)
{
	if (xTaskCreate(example_audio_aac_thread, ((const char *)"example_audio_aac_thread"), 2000, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(example_audio_mp3_thread) failed", __FUNCTION__);
	}
}


