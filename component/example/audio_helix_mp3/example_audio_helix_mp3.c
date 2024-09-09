#include <platform_opts.h>
#include "platform_stdlib.h"
#include "section_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "example_audio_helix_mp3.h"

#include "wifi_conf.h"
#if defined(CONFIG_PLATFORM_8195BHP) || defined(CONFIG_PLATFORM_8735B)
#if defined(CONFIG_PLATFORM_8735B)
#include "wifi_conf.h"
#include "lwip_netconf.h"
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
#include "osdep_service.h"
#endif
#endif
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include "audio_api.h"
#else
#include "lwip_netconf.h"
#include "i2s_api.h"
#endif

#include "mp3dec.h"

#ifndef SDRAM_BSS_SECTION
#define SDRAM_BSS_SECTION                        \
        SECTION(".sdram.bss")
#endif

#define AUDIO_SOURCE_BINARY_ARRAY (1)
#define ADUIO_SOURCE_HTTP_FILE    (0)

#if AUDIO_SOURCE_BINARY_ARRAY
#include "audio_helix_mp3/sr48000_br320_stereo.c"
#endif

#if defined(CONFIG_PLATFORM_8195BHP) || defined(CONFIG_PLATFORM_8735B)
#define UPSAMPLE_48K (0)
#endif
#if ADUIO_SOURCE_HTTP_FILE
#define AUDIO_SOURCE_HTTP_FILE_HOST         "192.168.2.126"
#define AUDIO_SOURCE_HTTP_FILE_PORT         (8080)
#define AUDIO_SOURCE_HTTP_FILE_NAME         "/audio.mp3"
#define BUFFER_SIZE                         (1500)

#define MP3_MAX_FRAME_SIZE (1600)
#define MP3_DATA_CACHE_SIZE ( BUFFER_SIZE + 2 * MP3_MAX_FRAME_SIZE )
SDRAM_BSS_SECTION static uint8_t mp3_data_cache[MP3_DATA_CACHE_SIZE];
static uint32_t mp3_data_cache_len = 0;
#endif

#if ADUIO_SOURCE_HTTP_FILE
#define AUDIO_PKT_BEGIN (1)
#define AUDIO_PKT_DATA  (2)
#define AUDIO_PKT_END   (3)
typedef struct _audio_pkt_t {
	uint8_t type;
	uint8_t *data;
	uint32_t data_len;
} audio_pkt_t;

#define AUDIO_PKT_QUEUE_LENGTH (50)
static xQueueHandle audio_pkt_queue;
#endif

#if defined(CONFIG_PLATFORM_8195BHP) || defined(CONFIG_PLATFORM_8735B)
SDRAM_BSS_SECTION static uint8_t decodebuf[ 8192 ];

#define AUDIO_TX_PCM_QUEUE_LENGTH (10)
#define AUDIO_DMA_PAGE_SIZE (2304)	// 20ms: 160 samples, 16bit
#define AUDIO_DMA_PAGE_NUM 4

#define TX_PAGE_SIZE 	AUDIO_DMA_PAGE_SIZE //64*N bytes, max: 4095. 128, 4032 
#define TX_PAGE_NUM 	AUDIO_DMA_PAGE_NUM
#define RX_PAGE_SIZE 	AUDIO_DMA_PAGE_SIZE //64*N bytes, max: 4095. 128, 4032 
#define RX_PAGE_NUM 	AUDIO_DMA_PAGE_NUM

static xQueueHandle audio_tx_pcm_queue = NULL;

static uint32_t audio_tx_pcm_cache_len = 0;
static int16_t audio_tx_pcm_cache[AUDIO_DMA_PAGE_SIZE * 3 / 4];
SDRAM_BSS_SECTION static int16_t audio_tx_pcm_cache_temp[AUDIO_DMA_PAGE_SIZE];
static int16_t audio_tx_pcm_last_end = 0;

static uint8_t dma_txdata[TX_PAGE_SIZE * TX_PAGE_NUM]__attribute__((aligned(0x20)));
static uint8_t dma_rxdata[RX_PAGE_SIZE * RX_PAGE_NUM]__attribute__((aligned(0x20)));
static audio_t g_taudio;
#else
#define I2S_DMA_PAGE_SIZE  2304
#define I2S_DMA_PAGE_NUM      4   // Vaild number is 2~4

SDRAM_BSS_SECTION static uint8_t i2s_tx_buf[ I2S_DMA_PAGE_SIZE * I2S_DMA_PAGE_NUM ];
SDRAM_BSS_SECTION static uint8_t i2s_rx_buf[ I2S_DMA_PAGE_SIZE * I2S_DMA_PAGE_NUM ];

#define I2S_SCLK_PIN	PC_1
#define I2S_WS_PIN		PC_0
#define I2S_SD_PIN		PC_2

static i2s_t i2s_obj;

SDRAM_BSS_SECTION static uint8_t decodebuf[ 4096 ];

#define I2S_TX_PCM_QUEUE_LENGTH (10)
static xQueueHandle i2s_tx_pcm_queue = NULL;

static uint32_t i2s_tx_pcm_cache_len = 0;
static int16_t i2s_tx_pcm_cache[I2S_DMA_PAGE_SIZE / 2];

static xSemaphoreHandle i2s_tx_done_sema = NULL;
#endif

#if defined(CONFIG_PLATFORM_8195BHP) || defined(CONFIG_PLATFORM_8735B)
static void audio_tx_complete(uint32_t arg, uint8_t *pbuf)
{
	uint8_t *ptx_buf;

	ptx_buf = (uint8_t *)audio_get_tx_page_adr(&g_taudio);
	if (xQueueReceiveFromISR(audio_tx_pcm_queue, ptx_buf, NULL) != pdPASS) {
		memset(ptx_buf, 0, AUDIO_DMA_PAGE_SIZE);
	}
	audio_set_tx_page(&g_taudio, (uint8_t *)ptx_buf);
}

static void audio_rx_complete(uint32_t arg, uint8_t *pbuf)
{
	audio_t *obj = (audio_t *)arg;
	audio_set_rx_page(obj);
}

static void initialize_audio(int sample_rate)
{
	uint8_t smpl_rate_idx = ASR_16KHZ;

	switch (sample_rate) {
	case 8000:
		smpl_rate_idx = ASR_8KHZ;
		break;
	case 16000:
		smpl_rate_idx = ASR_16KHZ;
		break;
	case 32000:
		smpl_rate_idx = ASR_32KHZ;
		break;
	case 44100:
		smpl_rate_idx = ASR_44p1KHZ;
		break;
	case 48000:
		smpl_rate_idx = ASR_48KHZ;
		break;
	case 88200:
		smpl_rate_idx = ASR_88p2KHZ;
		break;
	case 96000:
		smpl_rate_idx = ASR_96KHZ;
		break;
	default:
		break;
	}

	audio_init(&g_taudio, OUTPUT_SINGLE_EDNED, MIC_SINGLE_EDNED, AUDIO_CODEC_2p8V);
	audio_dac_digital_vol(&g_taudio, 0xAF / 2);

	//Init TX dma
#if defined(CONFIG_PLATFORM_8735B)
	audio_set_dma_buffer(&g_taudio, dma_txdata, dma_rxdata, AUDIO_DMA_PAGE_SIZE, AUDIO_DMA_PAGE_NUM);
#else
	audio_set_tx_dma_buffer(&g_taudio, dma_txdata, TX_PAGE_SIZE);
#endif

#if defined(CONFIG_PLATFORM_8735B)
	//Init RX dma
	audio_rx_irq_handler(&g_taudio, (audio_irq_handler)audio_rx_complete, (uint32_t *)&g_taudio);

	//Init TX dma
	audio_tx_irq_handler(&g_taudio, (audio_irq_handler)audio_tx_complete, (uint32_t *)&g_taudio);
#else
	//Init RX dma
	audio_rx_irq_handler(&g_taudio, (audio_irq_handler)audio_rx_complete, (uint32_t)&g_taudio);

	//Init TX dma
	audio_tx_irq_handler(&g_taudio, (audio_irq_handler)audio_tx_complete, (uint32_t)&g_taudio);
#endif

#if UPSAMPLE_48K
	smpl_rate_idx = ASR_48KHZ;
#endif
	audio_set_param(&g_taudio, smpl_rate_idx, WL_16BIT);  // ASR_8KHZ, ASR_16KHZ //ASR_48KHZ
	audio_mic_analog_gain(&g_taudio, 1, AUDIO_MIC_40DB); // default 0DB

#if defined(CONFIG_PLATFORM_8735B)
	/* Use (DMA page count -1) because occur RX interrupt in first */
	for (int i = 0; i < (AUDIO_DMA_PAGE_NUM - 1); i++) {
		uint8_t *ptx_buf = audio_get_tx_page_adr(&g_taudio);
		if (ptx_buf) {
			memset(ptx_buf, 0x0, AUDIO_DMA_PAGE_SIZE);
			audio_set_tx_page(&g_taudio, ptx_buf);
		}
		audio_set_rx_page(&g_taudio);
	}
#endif

	audio_trx_start(&g_taudio);
}

static void audio_play_pcm(int16_t *buf, uint32_t len)
{
#if UPSAMPLE_48K
	for (int i = 0; i < 3 * len; i++) {
		audio_tx_pcm_cache[audio_tx_pcm_cache_len++] = audio_tx_pcm_cache_temp[i];
		if (audio_tx_pcm_cache_len == AUDIO_DMA_PAGE_SIZE / 2) {
			xQueueSend(audio_tx_pcm_queue, audio_tx_pcm_cache, portMAX_DELAY);
			audio_tx_pcm_cache_len = 0;
		}
	}
#else
	for (int i = 0; i < len; i++) {
		audio_tx_pcm_cache[audio_tx_pcm_cache_len++] = buf[i];
		if (audio_tx_pcm_cache_len == AUDIO_DMA_PAGE_SIZE / 2) {
			xQueueSend(audio_tx_pcm_queue, audio_tx_pcm_cache, portMAX_DELAY);
			audio_tx_pcm_cache_len = 0;
		}
	}
#endif
}

#if AUDIO_SOURCE_BINARY_ARRAY
void audio_play_binary_array(uint8_t *srcbuf, uint32_t len)
{
	uint8_t *inbuf;
	int bytesLeft;

	int ret;
	HMP3Decoder	hMP3Decoder;
	MP3FrameInfo frameInfo;

	uint8_t first_frame = 1;
	int ch_sel = 0;

	hMP3Decoder = MP3InitDecoder();
	MP3DecodeSetChannel(ch_sel);

	inbuf = srcbuf;
	bytesLeft = len;

	while (1) {
		if (bytesLeft > 6) {
			ret = MP3Decode(hMP3Decoder, &inbuf, &bytesLeft, (short *)decodebuf, 0);
		} else {
			ret = ERR_MP3_INDATA_UNDERFLOW;
		}
		if (!ret) {
			MP3GetLastFrameInfo(hMP3Decoder, &frameInfo);
			short *wTemp = (short *)decodebuf;
			if (frameInfo.nChans == 2) {
				for (int i = 0; i < 2048; i++) {
					wTemp[i] = wTemp[2 * i + ch_sel % 2];
				}
				frameInfo.outputSamps /= 2;
			}
			printf("inbuf:%d, bytesLeft:%d, %d %d %d %d %d %d %d .\r\n", (int)inbuf, bytesLeft, frameInfo.bitrate, frameInfo.nChans, frameInfo.samprate,
				   frameInfo.bitsPerSample, frameInfo.outputSamps, frameInfo.layer, frameInfo.version);
			if (first_frame) {
				initialize_audio(frameInfo.samprate);
				first_frame = 0;
			}
#if UPSAMPLE_48K
			MP3PCMUpSample3X(audio_tx_pcm_cache_temp, (short *)decodebuf, frameInfo.outputSamps, &audio_tx_pcm_last_end);
#endif
			audio_play_pcm((int16_t *)decodebuf, frameInfo.outputSamps);
		} else {
			if (ret != ERR_MP3_INDATA_UNDERFLOW) {
				printf("err:%d\r\n", ret);
			}
			break;
		}
	}

	printf("decoding finished\r\n");
}
#endif
#else
static void i2s_tx_complete(void *data, char *pbuf)
{
	uint8_t *ptx_buf;

	ptx_buf = (uint8_t *)i2s_get_tx_page(&i2s_obj);
	if (xQueueReceiveFromISR(i2s_tx_pcm_queue, ptx_buf, NULL) != pdPASS) {
		memset(ptx_buf, 0, I2S_DMA_PAGE_SIZE);
	}
	i2s_send_page(&i2s_obj, (uint32_t *)ptx_buf);

	xSemaphoreGiveFromISR(i2s_tx_done_sema, NULL);
}

static void i2s_rx_complete(void *data, char *pbuf) {}

static void initialize_audio(uint8_t ch_num, int sample_rate)
{
	printf("ch:%d sr:%d\r\n", ch_num, sample_rate);

	uint8_t smpl_rate_idx = SR_16KHZ;
	uint8_t ch_num_idx = I2S_CH_STEREO;

	switch (ch_num) {
	case 1:
		ch_num_idx = I2S_CH_MONO;
		break;
	case 2:
		ch_num_idx = I2S_CH_STEREO;
		break;
	default:
		break;
	}

	switch (sample_rate) {
	case 8000:
		smpl_rate_idx = SR_8KHZ;
		break;
	case 16000:
		smpl_rate_idx = SR_16KHZ;
		break;
	case 22050:
		smpl_rate_idx = SR_22p05KHZ;
		break;
	case 24000:
		smpl_rate_idx = SR_24KHZ;
		break;
	case 32000:
		smpl_rate_idx = SR_32KHZ;
		break;
	case 44100:
		smpl_rate_idx = SR_44p1KHZ;
		break;
	case 48000:
		smpl_rate_idx = SR_48KHZ;
		break;
	default:
		break;
	}

	i2s_obj.channel_num = ch_num_idx;
	i2s_obj.sampling_rate = smpl_rate_idx;
	i2s_obj.word_length = WL_16b;
	i2s_obj.direction = I2S_DIR_TXRX;
	i2s_init(&i2s_obj, I2S_SCLK_PIN, I2S_WS_PIN, I2S_SD_PIN);
	i2s_set_dma_buffer(&i2s_obj, (char *)i2s_tx_buf, (char *)i2s_rx_buf, I2S_DMA_PAGE_NUM, I2S_DMA_PAGE_SIZE);
	i2s_tx_irq_handler(&i2s_obj, (i2s_irq_handler)i2s_tx_complete, (uint32_t)&i2s_obj);
	i2s_rx_irq_handler(&i2s_obj, (i2s_irq_handler)i2s_rx_complete, (uint32_t)&i2s_obj);

	/* rx need clock, let tx out first */
	i2s_send_page(&i2s_obj, (uint32_t *)i2s_get_tx_page(&i2s_obj));
	i2s_recv_page(&i2s_obj);
}

static void audio_play_pcm(int16_t *buf, uint32_t len)
{
	for (int i = 0; i < len; i++) {
		i2s_tx_pcm_cache[i2s_tx_pcm_cache_len++] = buf[i];
		if (i2s_tx_pcm_cache_len == I2S_DMA_PAGE_SIZE / 2) {
			xQueueSend(i2s_tx_pcm_queue, i2s_tx_pcm_cache, portMAX_DELAY);
			i2s_tx_pcm_cache_len = 0;
		}
	}
}

#if AUDIO_SOURCE_BINARY_ARRAY
void audio_play_binary_array(uint8_t *srcbuf, uint32_t len)
{
	uint8_t *inbuf;
	int bytesLeft;

	int ret;
	HMP3Decoder	hMP3Decoder;
	MP3FrameInfo frameInfo;

	uint8_t first_frame = 1;

	hMP3Decoder = MP3InitDecoder();

	inbuf = srcbuf;
	bytesLeft = len;

	i2s_tx_done_sema = xSemaphoreCreateBinary();

	while (1) {
		ret = MP3Decode(hMP3Decoder, &inbuf, &bytesLeft, decodebuf, 0);
		if (!ret) {
			MP3GetLastFrameInfo(hMP3Decoder, &frameInfo);
			if (first_frame) {
				initialize_audio(frameInfo.nChans, frameInfo.samprate);
				first_frame = 0;
			}
			audio_play_pcm(decodebuf, frameInfo.outputSamps);
		} else {
			printf("error: %d\r\n", ret);
			break;
		}
	}

	printf("decoding finished\r\n");
}
#endif
#endif

#if ADUIO_SOURCE_HTTP_FILE
void file_download_thread(void *param)
{
	int n, server_fd = -1;
	struct sockaddr_in server_addr;
	struct hostent *server_host;
	char *buf = NULL;

	audio_pkt_t pkt_data;

#if defined(CONFIG_PLATFORM_8195BHP)
	while (wifi_is_ready_to_transceive(RTW_STA_INTERFACE) != RTW_SUCCESS) {
		vTaskDelay(1000);
	}
#else
	while (!((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) && (*(u32 *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
		vTaskDelay(1000);
	}
#endif
	do {
		server_fd = socket(AF_INET, SOCK_STREAM, 0);
		if (server_fd < 0) {
			printf("ERROR: socket\r\n");
			break;
		}

		server_host = gethostbyname(AUDIO_SOURCE_HTTP_FILE_HOST);
		server_addr.sin_port = htons(AUDIO_SOURCE_HTTP_FILE_PORT);
		server_addr.sin_family = AF_INET;
		memcpy((void *) &server_addr.sin_addr, (void *) server_host->h_addr, server_host->h_length);
		if (connect(server_fd, (struct sockaddr *) &server_addr, sizeof(server_addr)) != 0) {
			printf("ERROR: connect\r\n");
			break;
		}

		setsockopt(server_fd, SOL_SOCKET, SO_RCVTIMEO, &(int) {
			5000
		}, sizeof(int));

		buf = (char *) malloc(BUFFER_SIZE);
		if (buf == NULL) {
			printf("ERROR: malloc\r\n");
			break;
		}

		sprintf(buf, "GET %s HTTP/1.1\r\nHost: %s\r\n\r\n", AUDIO_SOURCE_HTTP_FILE_NAME, AUDIO_SOURCE_HTTP_FILE_HOST);
		write(server_fd, buf, strlen(buf));

		audio_pkt_t pkt_begin = { .type = AUDIO_PKT_BEGIN, .data = NULL, .data_len = 0 };
		xQueueSend(audio_pkt_queue, (void *) &pkt_begin, portMAX_DELAY);

		n = read(server_fd, buf, BUFFER_SIZE);

		while ((n = read(server_fd, buf, BUFFER_SIZE)) > 0) {
			pkt_data.type = AUDIO_PKT_DATA;
			pkt_data.data_len = n;
			pkt_data.data = (uint8_t *) malloc(n);
			while (pkt_data.data == NULL) {
				vTaskDelay(100);
				pkt_data.data = (uint8_t *) malloc(n);
			}
			memcpy(pkt_data.data, buf, n);
			xQueueSend(audio_pkt_queue, (void *) &pkt_data, portMAX_DELAY);
		}

		printf("exit download\r\n");
	} while (0);

	if (buf != NULL) {
		free(buf);
	}

	if (server_fd >= 0) {
		close(server_fd);
	}

	audio_pkt_t pkt_end = { .type = AUDIO_PKT_END, .data = NULL, .data_len = 0 };
	xQueueSend(audio_pkt_queue, (void *) &pkt_end, portMAX_DELAY);

	vTaskDelete(NULL);
}

void audio_play_http_file(void)
{
	audio_pkt_t pkt;

	uint8_t *inbuf;
	int bytesLeft;

	int ret;
	HMP3Decoder	hMP3Decoder;
	MP3FrameInfo frameInfo;

	uint8_t first_frame = 1;

	hMP3Decoder = MP3InitDecoder();

	while (1) {
		if (xQueueReceive(audio_pkt_queue, &pkt, portMAX_DELAY) != pdTRUE) {
			continue;
		}

		if (pkt.type == AUDIO_PKT_BEGIN) {
			vTaskDelay(5000); // wait 5 seconds for buffering
		}

		if (pkt.type == AUDIO_PKT_DATA) {
			if (mp3_data_cache_len + pkt.data_len >= MP3_DATA_CACHE_SIZE) {
				printf("mp3 data cache overflow %d %d\r\n", mp3_data_cache_len, pkt.data_len);
				free(pkt.data);
				break;
			}

			memcpy(mp3_data_cache + mp3_data_cache_len, pkt.data, pkt.data_len);
			mp3_data_cache_len += pkt.data_len;
			free(pkt.data);

			inbuf = mp3_data_cache;
			bytesLeft = mp3_data_cache_len;

			ret = MP3FindSyncWord(inbuf, bytesLeft);
			if (ret > 0) {
				inbuf += ret;
				bytesLeft -= ret;
			}

			ret = 0;
			while (ret == 0) {
#if defined(CONFIG_PLATFORM_8195BHP) || defined(CONFIG_PLATFORM_8735B)
				if (bytesLeft > 6) {
					ret = MP3Decode(hMP3Decoder, &inbuf, &bytesLeft, (short *)decodebuf, 0);
				} else {
					ret = ERR_MP3_INDATA_UNDERFLOW;
				}
				if (ret == 0) {
					MP3GetLastFrameInfo(hMP3Decoder, &frameInfo);
					if (frameInfo.nChans == 2) {
						short *wTemp = (short *)decodebuf;
						for (int i = 0; i < 2048; i++) {
							wTemp[i] = wTemp[2 * i];
						}
						frameInfo.outputSamps /= 2;
					}
					printf("cache_len=%d  bytesLeft=%d  pkt.data_len=%d  %d %d %d %d %d %d %d .\r\n", mp3_data_cache_len, bytesLeft, pkt.data_len, frameInfo.bitrate,
						   frameInfo.nChans, frameInfo.samprate, frameInfo.bitsPerSample, frameInfo.outputSamps, frameInfo.layer, frameInfo.version);
					if (first_frame) {
						initialize_audio(frameInfo.samprate);
						first_frame = 0;
					}
#if UPSAMPLE_48K
					MP3PCMUpSample3X(audio_tx_pcm_cache_temp, (short *)decodebuf, frameInfo.outputSamps, &audio_tx_pcm_last_end);
#endif
					audio_play_pcm((int16_t *)decodebuf, frameInfo.outputSamps);
				} else {
					if (ret != ERR_MP3_INDATA_UNDERFLOW) {
						printf("err:%d\r\n", ret);
					}
					break;
				}
#else
				ret = MP3Decode(hMP3Decoder, &inbuf, &bytesLeft, decodebuf, 0);
				if (ret == 0) {
					MP3GetLastFrameInfo(hMP3Decoder, &frameInfo);
					if (first_frame) {
						initialize_audio(frameInfo.nChans, frameInfo.samprate);
						first_frame = 0;
					}
					audio_play_pcm(decodebuf, frameInfo.outputSamps);
				} else {
					if (ret != ERR_MP3_INDATA_UNDERFLOW) {
						printf("err:%d\r\n", ret);
					}
					break;
				}
#endif
			}
			if (bytesLeft > 0) {
				memmove(mp3_data_cache, mp3_data_cache + mp3_data_cache_len - bytesLeft, bytesLeft);
				mp3_data_cache_len = bytesLeft;
			} else {
				mp3_data_cache_len = 0;
			}
		}

		if (pkt.type == AUDIO_PKT_END) {
			printf("play finished\r\n");
		}
	}

	vTaskDelete(NULL);
}
#endif



void example_audio_helix_mp3_thread(void *param)
{
#if defined(CONFIG_PLATFORM_8195BHP) || defined(CONFIG_PLATFORM_8735B)
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1) && defined(CONFIG_PLATFORM_8735B)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif
	audio_tx_pcm_queue = xQueueCreate(AUDIO_TX_PCM_QUEUE_LENGTH, AUDIO_DMA_PAGE_SIZE);
#else
	i2s_tx_pcm_queue = xQueueCreate(I2S_TX_PCM_QUEUE_LENGTH, I2S_DMA_PAGE_SIZE);
#endif

#if AUDIO_SOURCE_BINARY_ARRAY
	audio_play_binary_array(sr48000_br320_stereo_mp3, sr48000_br320_stereo_mp3_len);
#endif

#if ADUIO_SOURCE_HTTP_FILE
	audio_pkt_queue = xQueueCreate(AUDIO_PKT_QUEUE_LENGTH, sizeof(audio_pkt_t));

	if (xTaskCreate(file_download_thread, ((const char *)"file_download_thread"), 768, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(file_download_thread) failed", __FUNCTION__);
	}

	audio_play_http_file();
#endif

	vTaskDelete(NULL);
}

#define EXAMPLE_AUDIO_HELIX_MP3_HEAP_SIZE (768)
static uint8_t example_audio_helix_mp3_heap[EXAMPLE_AUDIO_HELIX_MP3_HEAP_SIZE * sizeof(StackType_t)];

void example_audio_helix_mp3(void)
{
#if defined(CONFIG_PLATFORM_8195BHP) || defined(CONFIG_PLATFORM_8735B)
	if (xTaskCreate(example_audio_helix_mp3_thread, "example_audio_helix_mp3_thread", EXAMPLE_AUDIO_HELIX_MP3_HEAP_SIZE, (void *) NULL, tskIDLE_PRIORITY + 1,
					NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(example_audio_helix_mp3_thread) failed", __FUNCTION__);
	}
#else
	if (xTaskGenericCreate(example_audio_helix_mp3_thread, "example_audio_helix_mp3_thread", EXAMPLE_AUDIO_HELIX_MP3_HEAP_SIZE, (void *) NULL, tskIDLE_PRIORITY + 1,
						   NULL, (void *)example_audio_helix_mp3_heap, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(example_audio_helix_mp3_thread) failed", __FUNCTION__);
	}
#endif
}

