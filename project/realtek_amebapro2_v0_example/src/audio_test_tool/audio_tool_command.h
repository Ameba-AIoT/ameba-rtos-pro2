#ifndef __AUDIO_SAVETOOL_H__
#define __AUDIO_SAVETOOL_H__
#include <platform_stdlib.h>
#include "vfs.h"
#include "module_audio.h"
#include "module_null.h"
#include "module_array.h"
#include "module_tone.h"
#include "module_afft.h"
#include "module_p2p_audio.h"
#include "mmf2_link.h"
#include "mmf2_siso.h"
#include "mmf2_mimo.h"
#include "avcodec.h"
#include <semphr.h>
//#include "pcm16K_std1ktone.h"
//#include "pcm8K_std1ktone.h"
#include "pcm16K_music.h"
#include "pcm8K_music.h"
#include "tftp/tftp.h"

#define	P2P_ENABLE  1

#define SD_SAVE_EN          0x01
#define SD_SAVE_START       0x02
#define TFTP_UPLOAD_EN      0x04
#define TFTP_UPLOAD_START   0x08

#define FRAME_LEN           320     //each frame contain 320 * (word_lengh / 8) bytes, 8K 40ms 16K 20 ms
#define RECORD_WORDS        80      //record 80 words (160 bytes) each time for SD card

#define RECORD_MIN          0x01
#define RECORD_RX_DATA      0x01
#define RECORD_TX_DATA      0x02
#define RECORD_ASP_DATA     0x04
#define RECORD_TXASP_DATA   0x08
#define RECORD_MAX          0x08

#define AUDIO_TFTP_MODE             "octet"
#define AUDIO_TFTP_HOST_IP_ADDR     "192.168.1.129"
#define AUDIO_TFTP_HOST_PORT        69

#define DEFAULT_AUDIO_MIC   USE_AUDIO_LEFT_DMIC //USE_AUDIO_AMIC

#define TXNOPLAY        0
#define TXPLAYTONE      1
#define TXPLAYBACK      2
#define TXPLAYMUSIC     3
#define TXPLAYSPEECH    4
#define TXPLAYRAMDISK   5

typedef   struct {
	char            fccID[4];
	unsigned long   dwSize;
	unsigned short  wFormatTag;
	unsigned short  wChannels;
	unsigned long   dwSamplesPerSec;
	unsigned long   dwAvgBytesPerSec;
	unsigned short  wBlockAlign;
	unsigned short  uiBitsPerSample;
} WAVE_FMT; //Format Chunk

typedef   struct {
	char            fccID[4];
	unsigned long   dwSize;
} WAVE_DATA; //Data Chunk

typedef   struct {
	char            fccID[4];
	unsigned long   dwSize;
	char            fccType[4];
} WAVE_HEAD; //RIFF WAVE Chunk

typedef struct {
	WAVE_HEAD   w_header;
	WAVE_FMT    w_fmt;
	WAVE_DATA   w_data;
} WAVE_HEADER;

extern  mm_context_t    *audio_save_ctx;
extern  mm_context_t    *null_save_ctx;
extern  mm_context_t    *array_pcm_ctx;
extern  mm_context_t    *pcm_tone_ctx;
extern  mm_context_t    *afft_test_ctx;
extern  mm_context_t    *p2p_audio_ctx;
extern  mm_siso_t       *siso_audio_afft;
extern  mm_mimo_t       *mimo_aarray_audio;

extern  audio_params_t      audio_save_params;
extern  array_params_t      pcm16k_array_params;
extern  tone_params_t       pcm_tone_params;
extern  afft_params_t       afft_test_params;
extern  p2p_audio_params_t  p2p_audio_params;

extern  int     reset_flag;
extern  int     record_state;
extern  int     frame_count;
extern  int     playing_sample_rate;
extern  char    file_name[20];
extern  int     record_frame_count;
extern  int     record_type;
extern  uint8_t audiocopy_status;
extern  int     audio_tftp_port;
extern  char    audio_tftp_ip[16];
extern  void    audio_save_log_init(void);
extern  RX_cfg_t    rx_asp_params;
extern  TX_cfg_t    tx_asp_params;
#endif