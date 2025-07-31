#include <stdint.h>
#include "mp4_demuxer.h"
#include "fatfs_sdcard_api.h"
#include <stdio.h>
static FIL     w_file;
//#define VFS_ENABLE
#ifdef VFS_ENABLE
#include "vfs.h"
static FILE     *m_file;
#endif
static void mp4_demuxer_thread(void *param)
{
#ifdef VFS_ENABLE
	int i = 0;
	int size = 0;
	int bw = 0;
	unsigned char key_frame = 0;
	unsigned int video_timestamp = 0;
	unsigned int audio_timestamp = 0;
	unsigned int video_duration = 0;
	unsigned int audio_duration = 0;
	unsigned char *video_buf = NULL;
	unsigned char *audio_buf = NULL;
	fatfs_sd_params_t *fatfs_params = NULL;
	char *file_name = NULL;
	const char *mp4_name = "AMEBA_PRO.mp4";
	int ret = 0;

	mp4_demux *mp4_demuxer_ctx = NULL;
	mp4_demuxer_ctx = (mp4_demux *)malloc(sizeof(mp4_demux));
	if (mp4_demuxer_ctx == NULL) {
		printf("It can't be allocated the buffer\r\n");
		goto mp4_create_fail;
	} else {
		memset(mp4_demuxer_ctx, 0, sizeof(mp4_demux));
	}

	vfs_init(NULL);
	const char *tag = "sd:/";
	vfs_user_register("sd", VFS_FATFS, VFS_INF_SD);
	set_mp4_demuxer_vfs_enable(mp4_demuxer_ctx);
	file_name = malloc(128);
	if (file_name == NULL) {
		printf("It can't be allocated the buffer\r\n");
		goto mp4_create_fail;
	}
	memset(file_name, 0, 128);
	strcpy(file_name, tag);
	sprintf(file_name + strlen(file_name), "%s", mp4_name);

	mp4_demuxer_open(mp4_demuxer_ctx, file_name);

	m_file = fopen("sd:/ameba_video.h264", "w");
	if (m_file == NULL) {
		printf("Can't open file\r\n");
		goto mp4_create_fail;
	}

	video_buf = (unsigned char *)malloc(mp4_demuxer_ctx->video_max_size);
	if (video_buf == NULL) {
		printf("It can't be allocated the buffer\r\n");
		goto mp4_create_fail;
	}
	for (i = 0; i < mp4_demuxer_ctx->video_len; i++) {
		size = get_video_frame(mp4_demuxer_ctx, video_buf, i, &key_frame, &video_duration, &video_timestamp);
		fwrite(video_buf, 1, size, m_file);
		/* if(key_frame){
			printf("Key Video %d timestamp %d video_duration %d ms %d\r\n",i,video_timestamp,video_duration,(video_timestamp*1000)/90000);
		}else{
			printf("Video %d timestamp %d video_duration %d ms %d\r\n",i,video_timestamp,video_duration,(video_timestamp*1000)/90000);
		} */
	}
	printf("Write video done\r\n");
	fclose(m_file);

	m_file = fopen("sd:/ameba_audio.aac", "w");
	if (m_file == NULL) {
		printf("Can't open file\r\n");
		goto mp4_create_fail;
	}

	audio_buf = (unsigned char *)malloc(mp4_demuxer_ctx->audio_max_size);
	if (audio_buf == NULL) {
		printf("It can't be allocated the buffer\r\n");
		goto mp4_create_fail;
	}
	for (i = 0; i < mp4_demuxer_ctx->audio_len; i++) {
		size = get_audio_frame(mp4_demuxer_ctx, audio_buf, i, &audio_duration, &audio_timestamp);
		fwrite(audio_buf, 1, size, m_file);
		/* if(mp4_demuxer_ctx->audio_format_type == AUDIO_AAC){
			printf("Audio %d timestamp %d audio_duration %d ms %d\r\n",i,audio_timestamp,audio_duration,(audio_timestamp*1024)/mp4_demuxer_ctx->audio_sample_rate);
		}else{
			printf("Audio %d timestamp %d audio_duration %d ms %d\r\n",i,audio_timestamp,audio_duration,(audio_timestamp*1024)/mp4_demuxer_ctx->audio_sample_rate);
		} */
	}
	printf("Write audio done\r\n");

	int audio_index, video_index = 0;
	mp4_demuxer_seek(mp4_demuxer_ctx, 5000, &video_index, &audio_index);

	fclose(m_file);
	printf("audio foramt %u\r\n", mp4_demuxer_ctx->audio_format_type);
	if (mp4_demuxer_ctx->audio_format_type == AUDIO_ULAW) {
		ret = access("sd:/ameba_audio.ulaw", W_OK);
		if (ret >= 0) {
			remove("sd:/ameba_audio.ulaw");
			printf("Remove the exit sd:/ameba_audio.ulaw file\r\n");
		}
		rename("sd:/ameba_audio.aac", "sd:/ameba_audio.ulaw");
	} else if (mp4_demuxer_ctx->audio_format_type == AUDIO_ALAW) {
		rename("sd:/ameba_audio.aac", "sd:/ameba_audio.alaw");
	}
	printf("mp4_demuxer_close\r\n");
	mp4_demuxer_close(mp4_demuxer_ctx);
	vfs_user_unregister("sd", VFS_FATFS, VFS_INF_SD);
#else
	int i = 0;
	int size = 0;
	unsigned int bw = 0;
	unsigned char key_frame = 0;
	unsigned int video_timestamp = 0;
	unsigned int audio_timestamp = 0;
	unsigned int video_duration = 0;
	unsigned int audio_duration = 0;
	unsigned char *video_buf = NULL;
	unsigned char *audio_buf = NULL;
	char *file_name = NULL;
	const char *mp4_name = "AMEBA_PRO.mp4";
	int ret = 0;
	fatfs_sd_params_t *fatfs_params = NULL;

	mp4_demux *mp4_demuxer_ctx = NULL;
	mp4_demuxer_ctx = (mp4_demux *)malloc(sizeof(mp4_demux));
	if (mp4_demuxer_ctx == NULL) {
		printf("It can't be allocated the buffer\r\n");
		goto mp4_create_fail;
	} else {
		memset(mp4_demuxer_ctx, 0, sizeof(mp4_demux));
	}


	fatfs_params = malloc(sizeof(fatfs_sd_params_t));
	if (fatfs_params == NULL) {
		printf("It can't be allocated the buffer\r\n");
		goto mp4_create_fail;
	}
	memset(fatfs_params, 0x00, sizeof(fatfs_sd_params_t));
	if (fatfs_sd_init() < 0) {
		goto mp4_create_fail;
	}
	fatfs_sd_get_param(fatfs_params);
	set_mp4_demuxer_fatfs_param(mp4_demuxer_ctx, fatfs_params);


	file_name = malloc(128);
	if (file_name == NULL) {
		printf("It can't be allocated the buffer\r\n");
		goto mp4_create_fail;
	}
	memset(file_name, 0, 128);

	strcpy(file_name, fatfs_params->drv);
	sprintf(file_name + strlen(file_name), "%s", mp4_name);
	printf("mp4_demuxer->filename = %s\r\n", file_name);

	mp4_demuxer_open(mp4_demuxer_ctx, file_name);

	ret = f_open(&w_file, "0:/ameba_video.h264", FA_OPEN_ALWAYS | FA_READ | FA_WRITE); // res = f_open(&m_file, path, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	if (ret != 0) {
		printf("Can't open file\r\n");
		goto mp4_create_fail;
	}

	video_buf = (unsigned char *)malloc(mp4_demuxer_ctx->video_max_size);
	if (video_buf == NULL) {
		printf("It can't be allocated the buffer\r\n");
		goto mp4_create_fail;
	}
	for (i = 0; i < mp4_demuxer_ctx->video_len; i++) {
		size = get_video_frame(mp4_demuxer_ctx, video_buf, i, &key_frame, &video_duration, &video_timestamp);
		f_write(&w_file, video_buf, size, (UINT *)&bw);
		//printf("Write video data = %d key = %d video_timestamp = %d\r\n",size,key_frame,video_timestamp);
	}
	printf("Write video done\r\n");
	f_close(&w_file);

	ret = f_open(&w_file, "0:/ameba_audio.aac", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	if (ret != 0) {
		printf("Can't open file\r\n");
		goto mp4_create_fail;
	}

	audio_buf = (unsigned char *)malloc(mp4_demuxer_ctx->audio_max_size);
	if (audio_buf == NULL) {
		printf("It can't be allocated the buffer\r\n");
		goto mp4_create_fail;
	}
	for (i = 0; i < mp4_demuxer_ctx->audio_len; i++) {
		size = get_audio_frame(mp4_demuxer_ctx, audio_buf, i, &audio_duration, &audio_timestamp);
		f_write(&w_file, audio_buf, size, (UINT *)&bw);
		//printf("Write audio data %d timestamp = %d\r\n",size,audio_timestamp);
	}
	printf("Write audio done\r\n");
	f_close(&w_file);
	printf("audio foramt %u\r\n", mp4_demuxer_ctx->audio_format_type);
	if (mp4_demuxer_ctx->audio_format_type == AUDIO_ULAW) {
		f_rename("ameba_audio.aac", "ameba_audio.ulaw");
	} else if (mp4_demuxer_ctx->audio_format_type == AUDIO_ALAW) {
		f_rename("ameba_audio.aac", "ameba_audio.alaw");
	}
	printf("mp4_demuxer_close\r\n");
	mp4_demuxer_close(mp4_demuxer_ctx);
#endif
mp4_create_fail:
	if (fatfs_params != NULL) {
		free(fatfs_params);
	}
	if (video_buf) {
		free(video_buf);
	}
	if (audio_buf) {
		free(audio_buf);
	}
	if (file_name) {
		free(file_name);
	}
	if (mp4_demuxer_ctx) {
		free(mp4_demuxer_ctx);
	}
	vTaskDelete(NULL);
}

void example_media_mp4_demuxer(void)
{
	if (xTaskCreate(mp4_demuxer_thread, ((const char *)"mp4_demuxer_thread"), 1024, NULL, 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(mp4_demuxer_thread) failed\n\r", __FUNCTION__);
	}
	return;
}