/******************************************************************************
*
* Copyright(c) 2007 - 2024 Realtek Corporation. All rights reserved.
*
* Example: Verify 4-channel TDM audio reception from BT SoC 8773/AP via SPORT
*
* Purpose: Simple verification test - receive 10 frames of audio data 
		   and dump the first 32 bytes of each frame to UART log.
*
* Hardware setup:
*   8773/AP (Master)  -->  8735 (Wi-Fi Slave)
*     BCK             -->   I2S0_CLK    (PF_13)
*     WS              -->   I2S0_WS     (PF_15)
*     SD              -->   I2S0_SD_RX  (PF_12)
*
* Audio format:
*   - Sample rate: 48 kHz
*   - Channels: 4 (TDM)
*   - Data length: 24-bit
*   - Channel/slot length: 32-bit
*   - Format: I2S
*
* The 8735 SPORT peripheral receives the data in slave mode.
* module_sport interleaves RX0 (ch0/ch1) and RX1 (ch2/ch3) into a
* single 4-channel interleaved buffer.
******************************************************************************/
#include "mmf2_link.h"
#include "mmf2_siso.h"

#include "module_sport.h"

// 1: self-test - 8735B is TDM master and generates BCLK/WS itself, so the
//    RX path (FIFO/DMA/ISR/queue) can be verified with nothing connected,
//    and PF_13 (6.144 MHz) / PF_15 (48 kHz) become outputs you can scope
//    to positively identify the header pins.
// 0: normal mode - 8735B is slave, BCLK/WS must come from the AP / 8773.
#define SPORT_SELF_TEST_MASTER 0

static mm_context_t *sport_ctx = NULL;
static sport_params_t sport_params;

void mmf2_example_sport_audio_init(void)
{
	mm_queue_item_t *item = NULL;
	uint32_t frame_count = 0;
	uint32_t idle_ticks = 0;
	int ret;

	printf("\n========================================\n");
	printf("SPORT 4-Channel Audio RX Verification\n");
	printf("Format: 48kHz, 4ch TDM, 24-bit in 32-bit slot\n");
#if SPORT_SELF_TEST_MASTER
	printf("SPORT role: MASTER SELF-TEST (TX pattern loopback)\n");
	printf("*** 8735B drives BCLK/WS itself - no external source needed ***\n");
	printf("*** jumper PF_14 (TXD) -> PF_12 (RXD) ***\n");
	printf("*** scope PF_13 = 6.144 MHz, PF_15 = 48 kHz ***\n");
	printf("*** expected frame data: 11 11 11 00 22 22 22 00 33 33 33 00 44 44 44 00 ... ***\n");
#else
	printf("SPORT role: Slave\n");
	printf("Waiting for external TDM master clocks...\n");
#endif
	printf("========================================\n\n");

	// ====== Open and configure SPORT module ======
	sport_ctx = mm_module_open(&sport_module);
	if (!sport_ctx) {
		printf("SPORT open FAILED!\n");
		goto example_fail;
	}

	mm_module_ctrl(sport_ctx, CMD_SPORT_GET_PARAMS, (int)&sport_params);

	// Configure for 8773 output format:
	sport_params.sample_rate       = 48000;          // 48 kHz
	sport_params.sport_ch_num      = CH_4;           // 4-channel TDM
	sport_params.sport_ch_len      = SPORT_CL_32BIT; // 32-bit slot length
	sport_params.sport_data_len    = SPORT_DL_24BIT; // 24-bit data
	sport_params.sport_word_length = 32;             // word length
	sport_params.sport_format      = SPORT_I2S;      // I2S format
#if SPORT_SELF_TEST_MASTER
	sport_params.sport_role        = SPORT_MASTER_MODE; // self-test: generate clocks
#else
	sport_params.sport_role        = SPORT_SLAVE_MODE; // 8735 is slave
#endif
	sport_params.rx_channel_select = SPORT_CH_SEL_ALL; // output all 4 channels
	sport_params.rx_byte_swap      = DISABLE;
	sport_params.tx_byte_swap      = DISABLE;
	sport_params.pin_group_num     = 0;              // default pin group

	mm_module_ctrl(sport_ctx, CMD_SPORT_SET_PARAMS, (int)&sport_params);
	mm_module_ctrl(sport_ctx, MM_CMD_SET_QUEUE_LEN, 30);
	mm_module_ctrl(sport_ctx, MM_CMD_INIT_QUEUE_ITEMS, MMQI_FLAG_STATIC);
	mm_module_ctrl(sport_ctx, CMD_SPORT_APPLY, 0);

	printf("SPORT module opened and configured\n");

	// ====== Allocate receive buffer ======
	item = (mm_queue_item_t *)sport_module.new_item(sport_ctx->priv);
	if (item == NULL) {
		printf("Failed to allocate receive buffer\n");
		goto example_fail;
	}

	// ====== Start SPORT RX (and pattern TX in self-test mode) ======
#if SPORT_SELF_TEST_MASTER
	mm_module_ctrl(sport_ctx, CMD_SPORT_SET_TX, 1);
#endif
	mm_module_ctrl(sport_ctx, CMD_SPORT_SET_RX, 1);
	printf("SPORT RX started. Receiving 10 frames...\n\n");
	mm_module_ctrl(sport_ctx, CMD_SPORT_DUMP_STATUS, 0);

	// ====== Receive and dump 10 frames ======
	while (frame_count < 10) {
		ret = sport_module.handle(sport_ctx->priv, NULL, item);
		if (ret > 0) {
			uint32_t i;
			uint32_t dump_len = (item->size < 32) ? item->size : 32;

			printf("Frame %2lu [%lu bytes]: ", frame_count + 1, item->size);
			for (i = 0; i < dump_len; i += 4) {
				// 24-bit data is left-aligned in 32-bit slot (upper 24 bits).
				// In little-endian memory: [padding][LSB][mid][MSB]
				// Print only the 3 valid bytes MSB-first: byte[3] byte[2] byte[1]
				printf("%02x %02x %02x ",
					((uint8_t *)item->data_addr)[i + 3],  // MSB of 24-bit data
					((uint8_t *)item->data_addr)[i + 2],
					((uint8_t *)item->data_addr)[i + 1]); // LSB of 24-bit data
			}
			printf("\n");

			frame_count++;
		} else {
			vTaskDelay(1);
			// Heartbeat roughly every 10 s while nothing arrives:
			if ((++idle_ticks % 10000) == 0) {
				printf("[no frame yet, %lu ticks]\n", idle_ticks);
				mm_module_ctrl(sport_ctx, CMD_SPORT_DUMP_STATUS, 0);
			}
		}
	}

	// ====== Stop SPORT and cleanup ======
	mm_module_ctrl(sport_ctx, CMD_SPORT_SET_RX, 0);
	sport_module.del_item(sport_ctx->priv, item);
	mm_module_close(sport_ctx);

	printf("\n========================================\n");
	printf("Verification complete - 10 frames received\n");
	printf("Check the hex dump above to confirm valid audio data\n");
	printf("========================================\n");
	return;

example_fail:
	printf("SPORT verification FAILED!\n");
	return;
}