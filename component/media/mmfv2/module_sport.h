#ifndef _MODULE_SPORT_H
#define _MODULE_SPORT_H

#include "mmf2_module.h"
#include "sport_api.h"
#include "hal_sport.h"

#define SPORT_DMA_PAGE_NUM 4
#define SPORT_DMA_PAGE_SIZE (640 * 4 * 4)	// 640 samples * 4 channels * 4 bytes (32-bit slot)
#define SPORT_ISR_POOL_SIZE 8				// Pre-allocated buffer pool for ISR use

#define CMD_SPORT_SET_PARAMS            MM_MODULE_CMD(0x00) // set parameter
#define CMD_SPORT_GET_PARAMS            MM_MODULE_CMD(0x01) // get parameter
#define CMD_SPORT_SET_SAMPLERATE        MM_MODULE_CMD(0x02)
#define CMD_SPORT_SET_RX                MM_MODULE_CMD(0x03)
#define CMD_SPORT_SET_RESET             MM_MODULE_CMD(0x04)
#define CMD_SPORT_FORCE_DEINIT          MM_MODULE_CMD(0x05)
#define CMD_SPORT_SET_FORMAT            MM_MODULE_CMD(0x06)
#define CMD_SPORT_SET_ROLE              MM_MODULE_CMD(0x07)
#define CMD_SPORT_SET_CHANNEL_SEL       MM_MODULE_CMD(0x08)
#define CMD_SPORT_SET_TIMESTAMP_OFFSET  MM_MODULE_CMD(0x09)
#define CMD_SPORT_DUMP_STATUS           MM_MODULE_CMD(0x0A) // print RX debug status to log
#define CMD_SPORT_SET_TX                MM_MODULE_CMD(0x0B) // start/stop TX test-pattern stream
#define CMD_SPORT_SET_TEST_DATA         MM_MODULE_CMD(0x0C) // inject synthetic test data (val=1 start, val=0 stop)
#define CMD_SPORT_SET_RTSP_CTX    		MM_MODULE_CMD(0x0D) // pick an unused ID
#define CMD_SPORT_APPLY                 MM_MODULE_CMD(0x0E) // for hardware module
#define CMD_SPORT_SET_CHANNEL_SWAP      MM_MODULE_CMD(0x0F) // perform de-interleave

// Channel selection flags for 4-channel TDM
#define SPORT_CH_SEL_ALL    0xFF    // Output all 4 channels
#define SPORT_CH_SEL_0      (1<<0)
#define SPORT_CH_SEL_1      (1<<1)
#define SPORT_CH_SEL_2      (1<<2)
#define SPORT_CH_SEL_3      (1<<3)

// Channel count enum (maps to sport_ch_t from hal_sport.h)
#define SPORT_CH_MONO       CH_1_MONO
#define SPORT_CH_STEREO     CH_2_STEREO
#define SPORT_CH_4          CH_4
#define SPORT_CH_6          CH_6
#define SPORT_CH_8          CH_8

// Queue item: holds a pointer to dynamically allocated audio data
// This avoids putting large arrays on the stack
typedef struct sport_rx_s {
	uint32_t        timestamp;
	uint32_t        hw_timestamp;
	uint8_t         *data;		// pointer to buffer from ISR pool
	uint32_t        data_size;
} sport_rx_t;

// ISR-safe buffer pool entry
typedef struct sport_isr_buf_s {
	uint8_t         *buf;
	volatile int    in_use;		// 0 = free, 1 = in use by ISR, 2 = in queue
} sport_isr_buf_t;

typedef struct sport_rx_cache_s {
	xQueueHandle    queue;
	uint16_t        idx;
	uint8_t         *dma_rx0_buf;	// DMA RX0 buffer (malloc'd, passed to HAL)
	uint8_t         *dma_rx1_buf;	// DMA RX1 buffer (malloc'd, passed to HAL)
	uint8_t         *dma_tx0_buf;	// DMA TX0 buffer (loopback test pattern)
	uint8_t         *temp_buf;		// temp buffer for assembling 4ch interleaved data
	sport_isr_buf_t isr_pool[SPORT_ISR_POOL_SIZE];	// pre-allocated ISR-safe buffers
} sport_rx_cache_t;

typedef struct sport_param_s {
	uint32_t        sample_rate;            // 48000
	uint8_t         sport_word_length;      // SPORT data word length (16, 24, 32)
	uint8_t         rx_word_length;         // Output word length (16, 24, 32)
	uint8_t         tx_word_length;         // TX word length
	sport_format_t  sport_format;           // SPORT_I2S, SPORT_LEFT_JUST, etc.
	sport_dev_mode_t sport_role;            // SPORT_MASTER_MODE or SPORT_SLAVE_MODE
	sport_ch_t      sport_ch_num;           // CH_4 for 4-channel TDM
	sport_cl_t      sport_ch_len;           // SPORT_CL_32BIT for 32-bit channel length
	sport_dl_t      sport_data_len;         // SPORT_DL_24BIT for 24-bit data
	uint8_t         rx_channel_select;      // Which channels to output (SPORT_CH_SEL_ALL)
	uint8_t         rx_byte_swap;           // Byte swap enable
	uint8_t         tx_byte_swap;           // TX byte swap
	uint8_t         pin_group_num;          // 0 or 1
	uint32_t        sport_timestamp_offset;
	uint8_t 		rx_channel_swap;
} sport_params_t;

typedef struct sport_ctx_s {
	void                    *parent;
	sport_t                 *sport_obj;
	sport_params_t          params;
	uint8_t                 sport_inited;
	uint32_t                sample_rate;
	uint8_t                 sport_word_length;
	uint8_t                 rx_word_length;
	uint8_t                 tx_word_length;
	sport_ch_t              sport_ch_num;
	sport_cl_t              sport_ch_len;
	sport_dl_t              sport_data_len;
	uint8_t                 rx_channel_select;
	uint8_t                 pin_group_num;
	uint8_t 				rx_channel_swap;
	uint32_t (*rxbyteProc32)(uint32_t);
	uint16_t (*rxbyteProc16)(uint16_t);
	uint32_t                sport_timestamp_offset;
	sport_rx_cache_t        sport_rx_cache;
	TaskHandle_t            sport_rx_task;
	mm_context_t 			*rtsp_ctx;
	mm_context_t    *sport_audio_ctx;
} sport_ctx_t;

typedef struct {
    uint32_t sample_rate;
    uint32_t channels;
    uint32_t frame_samples;
    uint32_t slot_bytes;
    uint32_t data_bytes;
    uint32_t pattern[8];       // up to 8 channels
    uint32_t output_codec_id;
} sport_pattern_params_t;

extern sport_params_t default_sport_params;
extern mm_module_t sport_module;
#endif