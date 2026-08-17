#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "mmf2_module.h"
#include "module_sport.h"
#include "sport_api.h"
#include "hal_sport.h"
#include "avcodec.h"
#include "module_rtsp2.h"
#include "media_filesystem.h"

// Default parameters: 48kHz, 4-channel TDM, 24-bit data in 32-bit slot, slave mode
sport_params_t default_sport_params = {
	.sample_rate            = 48000,
	.sport_word_length      = 32,
	.rx_word_length         = 32,
	.tx_word_length         = 32,
	.sport_format           = SPORT_I2S,
	.sport_role             = SPORT_SLAVE_MODE,
	.sport_ch_num           = CH_4,
	.sport_ch_len           = SPORT_CL_32BIT,
	.sport_data_len         = SPORT_DL_24BIT,
	.rx_channel_select      = SPORT_CH_SEL_ALL,
	.rx_byte_swap           = DISABLE,
	.tx_byte_swap           = DISABLE,
	.pin_group_num          = 0,
	.sport_timestamp_offset = 0,
	.rx_channel_swap           = 0,
};

// static sport_pattern_params_t sport_pattern_params = {
//     .sample_rate      = 48000,
//     .channels         = 4,
//     .frame_samples    = 80,
//     .slot_bytes       = 4,
//     .data_bytes       = 3,
//     .pattern          = {0x111111, 0x222222, 0x333333, 0x444444},
//     .output_codec_id  = AV_CODEC_ID_PCM_RAW
// };
static sport_pattern_params_t sport_pattern_params = {
    .sample_rate      = 48000,
    .channels         = 4,
    .frame_samples    = 640,
    .slot_bytes       = 4,
    .data_bytes       = 3,
    .output_codec_id  = AV_CODEC_ID_PCM_RAW
};
// Debug counters for RX bring-up diagnosis (printed via CMD_SPORT_DUMP_STATUS)
static volatile uint32_t sport_dbg_rx0_isr_cnt = 0;
static volatile uint32_t sport_dbg_rx1_isr_cnt = 0;
static volatile uint32_t sport_dbg_tx0_isr_cnt = 0;
static volatile uint32_t sport_dbg_pool_miss_cnt = 0;

// Test data injector state
static volatile int sport_test_data_running = 0;
static TaskHandle_t sport_test_data_task = NULL;

// Map sample rate to SPORT rate enum
static sport_rate_t sample_rate_to_sport_rate(uint32_t sample_rate)
{
	switch (sample_rate) {
	case 8000:
		return R_8KHZ;
	case 11025:
		return R_11p025KHZ;
	case 12000:
		return R_12KHZ;
	case 16000:
		return R_16KHZ;
	case 22050:
		return R_22p05KHZ;
	case 24000:
		return R_24KHZ;
	case 32000:
		return R_32KHZ;
	case 44100:
		return R_44p1KHZ;
	case 48000:
		return R_48KHZ;
	case 88200:
		return R_88p2KHZ;
	case 96000:
		return R_96KHZ;
	case 192000:
		return R_192KHZ;
	default:
		return R_48KHZ;
	}
}

// Map data length to sport_dl_t
static sport_dl_t word_len_to_sport_dl(uint8_t word_len)
{
	switch (word_len) {
	case 16:
		return SPORT_DL_16BIT;
	case 20:
		return SPORT_DL_20BIT;
	case 24:
		return SPORT_DL_24BIT;
	case 32:
		return SPORT_DL_32BIT;
	default:
		return SPORT_DL_24BIT;
	}
}

// Map channel length to sport_cl_t
static sport_cl_t word_len_to_sport_cl(uint8_t word_len)
{
	switch (word_len) {
	case 16:
		return SPORT_CL_16BIT;
	case 20:
		return SPORT_CL_20BIT;
	case 24:
		return SPORT_CL_24BIT;
	case 32:
		return SPORT_CL_32BIT;
	default:
		return SPORT_CL_32BIT;
	}
}

// Calculate page size based on channel config
static uint32_t calc_page_size(sport_ctx_t *ctx)
{
	uint32_t samples_per_page = 640;	// fixed by DMA design
	uint32_t bytes_per_slot = 4;		// 32-bit slot always
	uint32_t num_channels;

	switch (ctx->sport_ch_num) {
	case CH_1_MONO:
		num_channels = 1;
		break;
	case CH_2_STEREO:
		num_channels = 2;
		break;
	case CH_4:
		num_channels = 4;
		break;
	case CH_6:
		num_channels = 6;
		break;
	case CH_8:
		num_channels = 8;
		break;
	default:
		num_channels = 2;
		break;
	}

	return samples_per_page * num_channels * bytes_per_slot;
}

// Get a free buffer from the ISR-safe pool
// Returns NULL if no buffer available (frame will be dropped)
static uint8_t *isr_pool_get(sport_ctx_t *ctx)
{
	int i;
	for (i = 0; i < SPORT_ISR_POOL_SIZE; i++) {
		if (ctx->sport_rx_cache.isr_pool[i].in_use == 0) {
			ctx->sport_rx_cache.isr_pool[i].in_use = 1;
			return ctx->sport_rx_cache.isr_pool[i].buf;
		}
	}
	sport_dbg_pool_miss_cnt++;
	return NULL;	// pool exhausted, drop frame
}

// Mark a buffer as queued (in use by queue, not by ISR anymore)
static void isr_pool_mark_queued(sport_ctx_t *ctx, uint8_t *buf)
{
	int i;
	for (i = 0; i < SPORT_ISR_POOL_SIZE; i++) {
		if (ctx->sport_rx_cache.isr_pool[i].buf == buf) {
			ctx->sport_rx_cache.isr_pool[i].in_use = 2;
			return;
		}
	}
}

// Free a buffer back to the pool (called from task context after consumer reads it)
static void isr_pool_free(sport_ctx_t *ctx, uint8_t *buf)
{
	int i;
	for (i = 0; i < SPORT_ISR_POOL_SIZE; i++) {
		if (ctx->sport_rx_cache.isr_pool[i].buf == buf) {
			ctx->sport_rx_cache.isr_pool[i].in_use = 0;
			return;
		}
	}
}

// ============================================================
// Synthetic test data injector (for testing without SPORT HW)
// ============================================================

// Generate synthetic 4-channel test data: each channel gets a distinct pattern
// so we can verify channel mapping in the RTP stream.
// Ch1: 440Hz square wave, Ch2: 523Hz square wave, Ch3: 659Hz square wave, Ch4: 784Hz square wave
// Data is 24-bit left-justified in 32-bit slots (same as real SPORT output)
// Uses integer arithmetic only (no math.h dependency)
// Generate synthetic 4-channel test data (safe + correct signed 24-bit)
// static void sport_generate_test_data(int32_t *buf,
//                                      uint32_t num_frames,
//                                      uint32_t sample_count)
// {
//     for (uint32_t f = 0; f < num_frames; f++) {
//         uint32_t idx = sample_count + f;

//         // Square waves at different frequencies, 24‑bit left‑justified in 32‑bit slot
//         int32_t ch1 = ((idx * 440 / 48000) & 1) ?  0x00400000 : -0x00400000;
//         int32_t ch2 = ((idx * 523 / 48000) & 1) ?  0x00400000 : -0x00400000;
//         int32_t ch3 = ((idx * 659 / 48000) & 1) ?  0x00400000 : -0x00400000;
//         int32_t ch4 = ((idx * 784 / 48000) & 1) ?  0x00400000 : -0x00400000;

//         buf[f * 4 + 0] = ch1;
//         buf[f * 4 + 1] = ch2;
//         buf[f * 4 + 2] = ch3;
//         buf[f * 4 + 3] = ch4;
//     }
// }

// Task that periodically injects synthetic test data into sport_rx_cache.queue
// (same queue the ISR would use, so sport_rx_handle_thread picks it up)
// static void sport_test_data_task_func(void *arg)
// {
//     sport_ctx_t *ctx = (sport_ctx_t *)arg;
//     uint32_t page_size = calc_page_size(ctx);
//     uint32_t frames_per_page = page_size / (4 * sizeof(int32_t));
//     uint32_t sample_count = 0;

//     while (sport_test_data_running) {
//         mm_queue_item_t *item = (mm_queue_item_t *)sport_module.new_item(ctx);
//         if (item) {
//             sport_generate_test_data((int32_t *)item->data_addr,
//                                      frames_per_page,
//                                      sample_count);
//             sample_count += frames_per_page;

//             item->size      = page_size;
//             item->timestamp = mm_read_mediatime_ms();
//             item->type      = AV_CODEC_ID_PCM_RAW;

//             // Direct push into RTSP2
//            if (ctx->rtsp_ctx) {
// 				printf("[SPORT TEST] pushed PCM_RAW frame size=%u\n", page_size);
// 				rtsp2_module.handle(ctx->rtsp_ctx->priv, item, NULL);
// 			}
            
//         }
//         vTaskDelay(pdMS_TO_TICKS(20));
//     }
// }

// static void sport_pattern_feeder_task(void *arg)
// {
//     sport_ctx_t *ctx = (sport_ctx_t *)arg;
//     int count = 0;

//     while (sport_test_data_running) {
//         mm_queue_item_t *item = (mm_queue_item_t *)sport_module.new_item(ctx);
//         if (item) {
//             // Fill buffer with fixed per‑channel pattern
//             uint8_t *ptr = (uint8_t *)item->data_addr;
//             for (uint32_t s = 0; s < sport_pattern_params.frame_samples; s++) {
//                 for (uint32_t c = 0; c < sport_pattern_params.channels; c++) {
//                     uint32_t val = sport_pattern_params.pattern[c] << 8; // left‑align 24‑bit
//                     memcpy(ptr, &val, sport_pattern_params.slot_bytes);
//                     ptr += sport_pattern_params.slot_bytes;
//                 }
//             }

//             item->size      = sport_pattern_params.frame_samples *
//                               sport_pattern_params.channels *
//                               sport_pattern_params.slot_bytes;
//             item->timestamp = mm_read_mediatime_ms();
//             item->type      = sport_pattern_params.output_codec_id;

//             if ((++count % 100) == 1) {
//                 printf("[SPORT PATTERN] frame %d size=%lu ts=%lu\n",
//                        count, (unsigned long)item->size,
//                        (unsigned long)item->timestamp);
//             }

//             if (ctx->rtsp_ctx) {
//                 rtsp2_module.handle(ctx->rtsp_ctx->priv, item, NULL);
//             }

//             sport_module.del_item(ctx, item);
//         }

//         // *** Throttle so video frames can get through ***
//         vTaskDelay(pdMS_TO_TICKS(20));   // ~50 fps audio injection
//     }
// }

// static void sport_pattern_feeder_task(void *arg)
// {
//     sport_ctx_t *ctx = (sport_ctx_t *)arg;
//     int count = 0;
//     static uint32_t voice_idx = 0; 

//     while (sport_test_data_running) {
//         mm_queue_item_t *item = (mm_queue_item_t *)sport_module.new_item(ctx);
//         if (item) {
//             uint8_t *ptr = (uint8_t *)item->data_addr;
            
//             // Loop through all 640 samples per frame block (640 * 4 * 4 = 10240 bytes)
//             for (uint32_t s = 0; s < sport_pattern_params.frame_samples; s++) {
                
//                 // Read the pre-processed Big Endian value directly from the Python header array
//                 uint32_t big_endian_val = hello_voice_data[voice_idx];
                
//                 // Duplicate across all 4 tracks
//                 for (uint32_t c = 0; c < sport_pattern_params.channels; c++) {
//                     memcpy(ptr, &big_endian_val, sport_pattern_params.slot_bytes);
//                     ptr += sport_pattern_params.slot_bytes;
//                 }
                
//                 voice_idx++;
//                 if (voice_idx >= HELLO_VOICE_LEN) {
//                     voice_idx = 0; // Loop seamlessly
//                 }
//             }

//             // Sets total payload block boundaries to exactly 10,240 bytes
//             item->size      = sport_pattern_params.frame_samples *
//                               sport_pattern_params.channels *
//                               sport_pattern_params.slot_bytes;
//             item->timestamp = mm_read_mediatime_ms();
//             item->type      = sport_pattern_params.output_codec_id;

//             if ((++count % 100) == 1) {
//                 printf("[SPORT PATTERN] Streaming Dummy Voice: frame %d size=%lu\n", 
//                        count, (unsigned long)item->size);
//             }

//             if (ctx->rtsp_ctx) {
//                 rtsp2_module.handle(ctx->rtsp_ctx->priv, item, NULL);
//             }

//             sport_module.del_item(ctx, item);
//         }

//         vTaskDelay(pdMS_TO_TICKS(20));   
//     }
// }

static void sport_pattern_feeder_task(void *arg)
{
    sport_ctx_t *ctx = (sport_ctx_t *)arg;
    int count = 0;

    // Open the packed PCM file you dumped earlier
    FILE *pcm_file = extdisk_fopen("sport_dump_s32.pcm", "rb");
    if (!pcm_file) {
        printf("Failed to open sport_dump_s32.pcm\n");
        return;
    }

    while (sport_test_data_running) {
        mm_queue_item_t *item = (mm_queue_item_t *)sport_module.new_item(ctx);
        if (item) {
            uint8_t *ptr = (uint8_t *)item->data_addr;

            // Read one frame block (e.g. 10240 bytes) directly from file
            size_t bytes_read = extdisk_fread(ptr, 1,
                                              sport_pattern_params.frame_samples *
                                              sport_pattern_params.channels *
                                              sport_pattern_params.slot_bytes,
                                              pcm_file);

            if (bytes_read == 0) {
                // Loop back to start of file when EOF
                extdisk_fseek(pcm_file, 0, SEEK_SET);
                continue;
            }

            item->size      = bytes_read;
            item->timestamp = mm_read_mediatime_ms();
            item->type      = sport_pattern_params.output_codec_id;

            if ((++count % 100) == 1) {
                printf("[SPORT PATTERN] Streaming PCM file: frame %d size=%lu\n",
                       count, (unsigned long)item->size);
            }

            if (ctx->rtsp_ctx) {
                rtsp2_module.handle(ctx->rtsp_ctx->priv, item, NULL);
            }

            sport_module.del_item(ctx, item);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    extdisk_fclose(pcm_file);
}

// ============================================================
// End of test data injector
// ============================================================

// ============================================================
// RX handle thread: bridges sport_rx_cache.queue to MMF2 pipeline
// (same pattern as i2s_rx_handle_thread in module_i2s.c)
// ============================================================
// static void sport_rx_handle_thread(void *param)
// {
//     sport_ctx_t *ctx = (sport_ctx_t *)param;
//     mm_context_t *mctx = (mm_context_t *)ctx->parent;
//     mm_queue_item_t *output_item;

//     while (1) {
//         vTaskDelay(1);
//         // Read from internal queue (filled by ISR or test data task)
//         sport_rx_t rx_item;
//         if (xQueueReceive(ctx->sport_rx_cache.queue, &rx_item, 40) != pdTRUE) {
//             continue;
//         }
//         if (rx_item.data == NULL || rx_item.data_size == 0) {
//             continue;
//         }

//         // Get an output buffer from the MMF2 pipeline
//         if (xQueueReceive(mctx->output_recycle, &output_item, 0xFFFFFFFF) == pdTRUE) {
//             uint32_t copy_size = (rx_item.data_size < output_item->size) ?
//                                   rx_item.data_size : output_item->size;
//             memcpy((void *)output_item->data_addr, rx_item.data, copy_size);
//             output_item->size = copy_size;
//             output_item->timestamp = rx_item.timestamp;
//             output_item->hw_timestamp = rx_item.hw_timestamp;
//             output_item->type = AV_CODEC_ID_PCM_RAW;

//             // Debug print
//             int32_t *samples = (int32_t *)rx_item.data;
//             printf("[SPORT->RTSP] C1=%d C2=%d C3=%d C4=%d\r\n",
//                    samples[0], samples[1], samples[2], samples[3]);

//             // Return ISR pool buffer
//             isr_pool_free(ctx, rx_item.data);

//             // Push to MMF2 pipeline - MISO framework will read this and call sport_handle()
//             xQueueSend(mctx->output_ready, (void *)&output_item, 0xFFFFFFFF);
//         } else {
//             // No output buffer available, drop frame
//             isr_pool_free(ctx, rx_item.data);
//         }
//     }
//     printf("[SPORT RX Handle] Task Jump Out!!\r\n");
//     vTaskDelete(NULL);
// }
// Simple per-channel filter chain: DC block + low-pass + normalize

static int32_t filter_sample(int ch, int32_t x) {
    // --- DC blocker ---
    static int32_t prev_x[4] = {0};
    static int32_t prev_y[4] = {0};
    int32_t y = x - prev_x[ch] + (int32_t)(0.999f * prev_y[ch]);
    prev_x[ch] = x;
    prev_y[ch] = y;

    // --- Moving average low-pass ---
    #define LPF_TAPS 4
    static int32_t lpf_buf[4][LPF_TAPS];
    static int idx[4] = {0};
    lpf_buf[ch][idx[ch]] = y;
    idx[ch] = (idx[ch] + 1) % LPF_TAPS;
    int64_t sum = 0;
    for (int i = 0; i < LPF_TAPS; i++) sum += lpf_buf[ch][i];
    int32_t lpf_out = (int32_t)(sum / LPF_TAPS);

    return lpf_out;
}

// Q15 fixed‑point amplifier for 24‑bit PCM
// gain_q15: 32768 = 1.0x, 65536 = 2.0x, etc.
static inline int32_t amplify_pcm24_q15(int32_t sample, int32_t gain_q15) {
    int64_t temp = (int64_t)sample * gain_q15;
    temp >>= 15; // scale back
    if (temp > 0x7FFFFF) temp = 0x7FFFFF;
    if (temp < -0x800000) temp = -0x800000;
    return (int32_t)temp;
}

// Soft limiter for 24-bit PCM
// threshold: set near max (e.g. 0x700000)
// softness: controls curve (e.g. 4, higher = softer)
static inline int32_t soft_limit_pcm24(int32_t sample, int32_t threshold, int32_t softness) {
    if (sample > threshold) {
        int32_t excess = sample - threshold;
        sample = threshold + (excess / softness);
    } else if (sample < -threshold) {
        int32_t excess = (-threshold) - sample;
        sample = -threshold - (excess / softness);
    }
    return sample;
}
// working code qc tested
// static void sport_rx_handle_thread(void *param)
// {
//     sport_ctx_t *ctx = (sport_ctx_t *)param;
//     sport_rx_t rx_item;
//     int count = 0;

//     printf("[SPORT RX] Task started, waiting for data... queue=%p\n", (void *)ctx->sport_rx_cache.queue);

//     while (1) {
//         // Wait for SPORT RX data from ISR queue
//         if (xQueueReceive(ctx->sport_rx_cache.queue, &rx_item, portMAX_DELAY) == pdTRUE) {
//             if (rx_item.data && rx_item.data_size > 0) {
//                 // Wrap into MMF2 item
//                 mm_queue_item_t *item = (mm_queue_item_t *)sport_module.new_item(ctx);
//                 if (item) {
//                     memcpy((void *)item->data_addr, rx_item.data, rx_item.data_size);

// 					// --- Debug: print original SPORT slots ---
//                     printf("Original first 32 bytes: ");
//                     for (int i = 0; i < 32 && i < rx_item.data_size; i++) {
//                         printf("%02X ", ((uint8_t*)item->data_addr)[i]);
//                     }
//                     printf("\n");

//                     item->size      = rx_item.data_size;
//                     item->timestamp = mm_read_mediatime_ms();
//                     item->type      = AV_CODEC_ID_PCM_RAW;

// 					// // --- Apply filter to all samples ---
//                     // int32_t *samples = (int32_t *)item->data_addr;
//                     // int total_samples = rx_item.data_size / sizeof(int32_t);
//                     // for (int i = 0; i < total_samples; i++) {
//                     //     int ch = i % 4; // channel index
//                     //     samples[i] = filter_sample(ch, samples[i]);
//                     // }

// 					// // --- Filter + Amplify ---
//                     // int32_t *samples = (int32_t *)item->data_addr;
//                     // int total_samples = rx_item.data_size / sizeof(int32_t);
//                     // for (int i = 0; i < total_samples; i++) {
//                     //     int ch = i % 4;
//                     //     int32_t s = filter_sample(ch, samples[i]);
//                     //     s = amplify_pcm24_q15(s, 500000);
// 					// 	s = soft_limit_pcm24(s, 0x700000, 4);
//                     //     samples[i] = s;
//                     // }

// 					// if (pcm_file && (mm_read_mediatime_ms() - start_ms <= 20000)) {
//                     //     extdisk_fwrite(item->data_addr, 1, item->size, pcm_file);
//                     // } else if (pcm_file && (mm_read_mediatime_ms() - start_ms > 20000)) {
//                     //     extdisk_fclose(pcm_file);
//                     //     pcm_file = NULL;
//                     //     printf("PCM dump complete (5s)\n");
//                     // }

//                     if ((++count % 100) == 1) {
//                         int32_t *samples = (int32_t *)rx_item.data;
//                         printf("[SPORT->RTSP] frame %d size=%u ts=%lu C1=%d C2=%d C3=%d C4=%d\n",
//                                count, rx_item.data_size,
//                                (unsigned long)item->timestamp,
//                                samples[0], samples[1], samples[2], samples[3]);
//                     }

//                     // Direct push into RTSP2
//                     if (ctx->rtsp_ctx) {
//                         rtsp2_module.handle(ctx->rtsp_ctx->priv, item, NULL);
//                     }

//                     sport_module.del_item(ctx, item);
//                 }

//                 // Return buffer to ISR pool
//                 isr_pool_free(ctx, rx_item.data);
//             }
//         }
//     }
// }

static void sport_rx_handle_thread(void *param)
{
    sport_ctx_t *ctx = (sport_ctx_t *)param;
    sport_rx_t rx_item;
    int count = 0;

	// Open PCM file for writing (raw dump)
    // FILE *pcm_file = extdisk_fopen("sport_dump_s32.pcm", "wb");
    // uint64_t start_ms = mm_read_mediatime_ms();

    while (1) {
        // Wait for SPORT RX data from ISR queue
        if (xQueueReceive(ctx->sport_rx_cache.queue, &rx_item, portMAX_DELAY) == pdTRUE) {
            if (rx_item.data && rx_item.data_size > 0) {
                // Wrap into MMF2 item
                mm_queue_item_t *item = (mm_queue_item_t *)sport_module.new_item(ctx);
                if (item) {
                    memcpy((void *)item->data_addr, rx_item.data, rx_item.data_size);
                    item->size      = rx_item.data_size;
                    item->timestamp = mm_read_mediatime_ms();
                    item->type      = AV_CODEC_ID_PCM_RAW;

					// // --- Apply filter to all samples ---
                    // int32_t *samples = (int32_t *)item->data_addr;
                    // int total_samples = rx_item.data_size / sizeof(int32_t);
                    // for (int i = 0; i < total_samples; i++) {
                    //     int ch = i % 4; // channel index
                    //     samples[i] = filter_sample(ch, samples[i]);
                    // }

					// // --- Filter + Amplify ---
                    // int32_t *samples = (int32_t *)item->data_addr;
                    // int total_samples = rx_item.data_size / sizeof(int32_t);
                    // for (int i = 0; i < total_samples; i++) {
                    //     int ch = i % 4;
                    //     int32_t s = filter_sample(ch, samples[i]);
                    //     s = amplify_pcm24_q15(s, 500000);
					// 	s = soft_limit_pcm24(s, 0x700000, 4);
                    //     samples[i] = s;
                    // }

					// if (pcm_file) {
                    //     extdisk_fwrite(item->data_addr, 1, item->size, pcm_file);
					// }

                    if ((++count % 100) == 1) {
                        int32_t *samples = (int32_t *)rx_item.data;
                        printf("[SPORT->RTSP] frame %d size=%u ts=%lu C1=%d C2=%d C3=%d C4=%d\n",
                               count, rx_item.data_size,
                               (unsigned long)item->timestamp,
                               samples[0], samples[1], samples[2], samples[3]);
                    }

                    // Direct push into RTSP2
                    if (ctx->rtsp_ctx) {
                        rtsp2_module.handle(ctx->rtsp_ctx->priv, item, NULL);
                    }

                    sport_module.del_item(ctx, item);
                }

                // Return buffer to ISR pool
                isr_pool_free(ctx, rx_item.data);
            }
        }
		// Stop after 5 seconds
        // if (mm_read_mediatime_ms() - start_ms > 5000) {
        //     if (pcm_file) {
        //         extdisk_fclose(pcm_file);
        //         pcm_file = NULL;
        //         printf("PCM dump complete (5s)\n");
        //     }
        //     break;
		// }
    }
}

// static void sport_rx_handle_thread(void *param)
// {
//     sport_ctx_t *ctx = (sport_ctx_t *)param;
//     sport_rx_t rx_item;
//     int count = 0;

//     FILE *pcm_file = extdisk_fopen("sport_dump_s24.pcm", "wb");
//     uint64_t start_ms = mm_read_mediatime_ms();

//     while (1) {
//         if (xQueueReceive(ctx->sport_rx_cache.queue, &rx_item, portMAX_DELAY) == pdTRUE) {
//             if (rx_item.data && rx_item.data_size > 0) {
//                 mm_queue_item_t *item = (mm_queue_item_t *)sport_module.new_item(ctx);
//                 if (item) {
//                     memcpy((void *)item->data_addr, rx_item.data, rx_item.data_size);

//                     // --- Debug: print original SPORT slots ---
//                     printf("Original first 32 bytes: ");
//                     for (int i = 0; i < 32 && i < rx_item.data_size; i++) {
//                         printf("%02X ", ((uint8_t*)item->data_addr)[i]);
//                     }
//                     printf("\n");

//                     // --- Repack SPORT 32-bit slots into packed 24-bit PCM ---
//                     int total_samples = rx_item.data_size / 4;
//                     int packed_size   = total_samples * 3;

//                     uint8_t *src = (uint8_t *)rx_item.data;
//                     uint8_t *dst = (uint8_t *)item->data_addr;

//                     for (int i = 0; i < total_samples; i++) {
//                         dst[i*3 + 0] = src[i*4 + 1]; // LSB
//                         dst[i*3 + 1] = src[i*4 + 2];
//                         dst[i*3 + 2] = src[i*4 + 3]; // MSB
//                     }

//                     item->size      = packed_size;
//                     item->timestamp = mm_read_mediatime_ms();
//                     item->type      = AV_CODEC_ID_PCM_RAW;

//                     // --- Debug: print repacked PCM ---
//                     printf("Repacked first 32 bytes: ");
//                     for (int i = 0; i < 32 && i < item->size; i++) {
//                         printf("%02X ", ((uint8_t*)item->data_addr)[i]);
//                     }
//                     printf("\n");

//                     // --- Save packed 24-bit PCM to file ---
//                     if (pcm_file && (mm_read_mediatime_ms() - start_ms <= 20000)) {
//                         extdisk_fwrite(item->data_addr, 1, item->size, pcm_file);
//                     } else if (pcm_file && (mm_read_mediatime_ms() - start_ms > 20000)) {
//                         extdisk_fclose(pcm_file);
//                         pcm_file = NULL;
//                         printf("PCM dump complete (20s)\n");
//                     }

//                     if ((++count % 100) == 1) {
//                         int32_t *samples = (int32_t *)rx_item.data;
//                         printf("[SPORT->RTSP] frame %d orig_size=%u packed_size=%u ts=%lu C1=%d C2=%d C3=%d C4=%d\n",
//                                count, rx_item.data_size, item->size,
//                                (unsigned long)item->timestamp,
//                                samples[0], samples[1], samples[2], samples[3]);
//                     }

//                     if (ctx->rtsp_ctx) {
//                         rtsp2_module.handle(ctx->rtsp_ctx->priv, item, NULL);
//                     }

//                     sport_module.del_item(ctx, item);
//                 }

//                 isr_pool_free(ctx, rx_item.data);
//             }
//         }
//     }
// }


// RX0 DMA callback - handles channels 0/1
static void sport_rx0_irq_cb(u32 *arg, u8 *pbuf)
{
	sport_ctx_t *ctx = (sport_ctx_t *)arg;
	uint32_t page_size = calc_page_size(ctx);
	uint32_t half_page = page_size / 2;	// RX0 gets first half (ch 0/1)
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	sport_dbg_rx0_isr_cnt++;

	if (pbuf != NULL) {
		// Copy RX0 data (channels 0/1) to temp buffer
		memcpy(ctx->sport_rx_cache.temp_buf, pbuf, half_page);

		ctx->sport_rx_cache.idx++;

		// When both RX0 and RX1 have arrived, get pool buffer and send to queue
		if (ctx->sport_rx_cache.idx >= 2) {
			ctx->sport_rx_cache.idx = 0;

			// Get a buffer from the ISR-safe pool (no malloc!)
			uint8_t *out_buf = isr_pool_get(ctx);
			if (out_buf) {
				memcpy(out_buf, ctx->sport_rx_cache.temp_buf, page_size);

				sport_rx_t rx_item;
				rx_item.timestamp = 0;
				rx_item.hw_timestamp = 0;
				rx_item.data = out_buf;
				rx_item.data_size = page_size;

				if (xQueueSendFromISR(ctx->sport_rx_cache.queue, &rx_item, &xHigherPriorityTaskWoken) == pdTRUE) {
					isr_pool_mark_queued(ctx, out_buf);
				} else {
					// Queue full, return buffer to pool
					isr_pool_free(ctx, out_buf);
				}
			}
		}
	}

	// Return page to hardware AFTER all processing, so the HAL ISR
	// can finish clearing the interrupt and advancing its page index first.
	hal_sport_rx0_page_recv(&ctx->sport_obj->sport_adapter);
}

// RX1 DMA callback - handles channels 2/3
static void sport_rx1_irq_cb(u32 *arg, u8 *pbuf)
{
	sport_ctx_t *ctx = (sport_ctx_t *)arg;
	uint32_t page_size = calc_page_size(ctx);
	uint32_t half_page = page_size / 2;	// RX1 gets second half (ch 2/3)
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	sport_dbg_rx1_isr_cnt++;

	if (pbuf != NULL) {
		// Interleave RX1 data (channels 2/3) into temp_buf at positions 2/3
		uint8_t *dst = ctx->sport_rx_cache.temp_buf;
		uint8_t *src = (uint8_t *)pbuf;
		uint32_t slot_size = 4;	// 32-bit slots
		uint32_t num_samples = half_page / (2 * slot_size);	// samples per channel pair
		uint32_t i;

		for (i = 0; i < num_samples; i++) {
			// ch2 at position 2
			memcpy(dst + (i * 4 * slot_size) + (2 * slot_size),
				src + (i * 2 * slot_size), slot_size);
			// ch3 at position 3
			memcpy(dst + (i * 4 * slot_size) + (3 * slot_size),
				src + (i * 2 * slot_size) + slot_size, slot_size);
		}

		ctx->sport_rx_cache.idx++;

		// When both RX0 and RX1 have arrived, get pool buffer and send to queue
		if (ctx->sport_rx_cache.idx >= 2) {
			ctx->sport_rx_cache.idx = 0;

			// Get a buffer from the ISR-safe pool (no malloc!)
			uint8_t *out_buf = isr_pool_get(ctx);
			if (out_buf) {
				memcpy(out_buf, ctx->sport_rx_cache.temp_buf, page_size);

				sport_rx_t rx_item;
				rx_item.timestamp = 0;
				rx_item.hw_timestamp = 0;
				rx_item.data = out_buf;
				rx_item.data_size = page_size;

				if (xQueueSendFromISR(ctx->sport_rx_cache.queue, &rx_item, &xHigherPriorityTaskWoken) == pdTRUE) {
					isr_pool_mark_queued(ctx, out_buf);
				} else {
					isr_pool_free(ctx, out_buf);
				}
			}
		}
	}

	// Return page to hardware AFTER all processing, so the HAL ISR
	// can finish clearing the interrupt and advancing its page index first.
	hal_sport_rx1_page_recv(&ctx->sport_obj->sport_adapter);
}

// RX0 DMA callback for 1/2/4-channel mode (TDM4: all four channels arrive via RX0)
static void sport_rx0_2ch_irq_cb(u32 *arg, u8 *pbuf)
{
    sport_ctx_t *ctx = (sport_ctx_t *)arg;
    uint32_t page_size = calc_page_size(ctx);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    sport_dbg_rx0_isr_cnt++;

    if (pbuf != NULL) {
        // Get a buffer from the ISR-safe pool
        uint8_t *out_buf = isr_pool_get(ctx);
        if (out_buf) {
            if (ctx->rx_channel_swap && ctx->sport_ch_num == CH_4) {
                // De‑interleave [s1,s3,s2,s4] back to [s1,s2,s3,s4]
                uint32_t *src = (uint32_t *)pbuf;
                uint32_t *dst = (uint32_t *)out_buf;
                uint32_t frames = page_size / 16;
                for (uint32_t f = 0; f < frames; f++) {
                    dst[0] = src[0];
                    dst[1] = src[2];
                    dst[2] = src[1];
                    dst[3] = src[3];
                    src += 4;
                    dst += 4;
                }
            } else {
                memcpy(out_buf, pbuf, page_size);
            }

            sport_rx_t rx_item;
            rx_item.timestamp = mm_read_mediatime_ms();
            rx_item.hw_timestamp = 0;
            rx_item.data = out_buf;
            rx_item.data_size = page_size;

            if (xQueueSendFromISR(ctx->sport_rx_cache.queue, &rx_item, &xHigherPriorityTaskWoken) == pdTRUE) {
                isr_pool_mark_queued(ctx, out_buf);
            } else {
                isr_pool_free(ctx, out_buf);
            }
        }
    }

    // Return page to hardware AFTER all processing, so the HAL ISR
    // can finish clearing the interrupt and advancing its page index first.
    hal_sport_rx0_page_recv(&ctx->sport_obj->sport_adapter);
}

// RX1 DMA callback for 2-channel mode
static void sport_rx1_2ch_irq_cb(u32 *arg, u8 *pbuf)
{
	sport_ctx_t *ctx = (sport_ctx_t *)arg;
	uint32_t page_size = calc_page_size(ctx);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (pbuf != NULL) {
		uint8_t *out_buf = isr_pool_get(ctx);
		if (out_buf) {
			memcpy(out_buf, pbuf, page_size);

			sport_rx_t rx_item;
			rx_item.timestamp = 0;
			rx_item.hw_timestamp = 0;
			rx_item.data = out_buf;
			rx_item.data_size = page_size;

			if (xQueueSendFromISR(ctx->sport_rx_cache.queue, &rx_item, &xHigherPriorityTaskWoken) == pdTRUE) {
				isr_pool_mark_queued(ctx, out_buf);
			} else {
				isr_pool_free(ctx, out_buf);
			}
		}
	}

	// Return page to hardware AFTER all processing, so the HAL ISR
	// can finish clearing the interrupt and advancing its page index first.
	hal_sport_rx1_page_recv(&ctx->sport_obj->sport_adapter);
}

// TX0 DMA callback - the pattern is static, so just hand the completed
// page straight back to hardware to keep the stream running
static void sport_tx0_irq_cb(u32 *arg, u8 *pbuf)
{
	sport_ctx_t *ctx = (sport_ctx_t *)arg;

	sport_dbg_tx0_isr_cnt++;
	if (pbuf != NULL) {
		sport_tx0_page_send(ctx->sport_obj, (u32 *)pbuf);
	}
}

// Allocate DMA-safe buffers and ISR pool
static int sport_allocate_dma_buffers(sport_ctx_t *ctx)
{
	uint32_t page_size = calc_page_size(ctx);
	uint32_t buf_size = page_size * SPORT_DMA_PAGE_NUM;
	int i;

	// Allocate RX0 DMA buffer
	ctx->sport_rx_cache.dma_rx0_buf = (uint8_t *)malloc(buf_size);
	if (ctx->sport_rx_cache.dma_rx0_buf == NULL) {
		return -1;
	}
	memset(ctx->sport_rx_cache.dma_rx0_buf, 0, buf_size);

	// Allocate RX1 DMA buffer
	ctx->sport_rx_cache.dma_rx1_buf = (uint8_t *)malloc(buf_size);
	if (ctx->sport_rx_cache.dma_rx1_buf == NULL) {
		free(ctx->sport_rx_cache.dma_rx0_buf);
		ctx->sport_rx_cache.dma_rx0_buf = NULL;
		return -1;
	}
	memset(ctx->sport_rx_cache.dma_rx1_buf, 0, buf_size);

	// Allocate temp buffer for 4-channel interleaving
	ctx->sport_rx_cache.temp_buf = (uint8_t *)malloc(page_size);
	if (ctx->sport_rx_cache.temp_buf == NULL) {
		free(ctx->sport_rx_cache.dma_rx0_buf);
		ctx->sport_rx_cache.dma_rx0_buf = NULL;
		free(ctx->sport_rx_cache.dma_rx1_buf);
		ctx->sport_rx_cache.dma_rx1_buf = NULL;
		return -1;
	}
	memset(ctx->sport_rx_cache.temp_buf, 0, page_size);

	// Allocate TX0 DMA buffer, pre-filled with the per-slot test pattern
	// (used by CMD_SPORT_SET_TX for loopback verification)
	ctx->sport_rx_cache.dma_tx0_buf = (uint8_t *)malloc(buf_size);
	if (ctx->sport_rx_cache.dma_tx0_buf == NULL) {
		free(ctx->sport_rx_cache.temp_buf);
		ctx->sport_rx_cache.temp_buf = NULL;
		free(ctx->sport_rx_cache.dma_rx0_buf);
		ctx->sport_rx_cache.dma_rx0_buf = NULL;
		free(ctx->sport_rx_cache.dma_rx1_buf);
		ctx->sport_rx_cache.dma_rx1_buf = NULL;
		return -1;
	}

	memset(ctx->sport_rx_cache.dma_tx0_buf, 0, buf_size);

	// Allocate ISR-safe buffer pool (pre-allocated, no malloc in ISR!)
	for (i = 0; i < SPORT_ISR_POOL_SIZE; i++) {
		ctx->sport_rx_cache.isr_pool[i].buf = (uint8_t *)malloc(page_size);
		if (ctx->sport_rx_cache.isr_pool[i].buf == NULL) {
			// Free previously allocated pool buffers
			int j;
			for (j = 0; j < i; j++) {
				free(ctx->sport_rx_cache.isr_pool[j].buf);
				ctx->sport_rx_cache.isr_pool[j].buf = NULL;
			}
			free(ctx->sport_rx_cache.temp_buf);
			ctx->sport_rx_cache.temp_buf = NULL;
			free(ctx->sport_rx_cache.dma_rx0_buf);
			ctx->sport_rx_cache.dma_rx0_buf = NULL;
			free(ctx->sport_rx_cache.dma_rx1_buf);
			ctx->sport_rx_cache.dma_rx1_buf = NULL;
			return -1;
		}
		memset(ctx->sport_rx_cache.isr_pool[i].buf, 0, page_size);
		ctx->sport_rx_cache.isr_pool[i].in_use = 0;
	}

	return 0;
}

static void sport_free_dma_buffers(sport_ctx_t *ctx)
{
	int i;

	if (ctx->sport_rx_cache.dma_rx0_buf) {
		free(ctx->sport_rx_cache.dma_rx0_buf);
		ctx->sport_rx_cache.dma_rx0_buf = NULL;
	}
	if (ctx->sport_rx_cache.dma_rx1_buf) {
		free(ctx->sport_rx_cache.dma_rx1_buf);
		ctx->sport_rx_cache.dma_rx1_buf = NULL;
	}
	if (ctx->sport_rx_cache.temp_buf) {
		free(ctx->sport_rx_cache.temp_buf);
		ctx->sport_rx_cache.temp_buf = NULL;
	}
	if (ctx->sport_rx_cache.dma_tx0_buf) {
		free(ctx->sport_rx_cache.dma_tx0_buf);
		ctx->sport_rx_cache.dma_tx0_buf = NULL;
	}
	for (i = 0; i < SPORT_ISR_POOL_SIZE; i++) {
		if (ctx->sport_rx_cache.isr_pool[i].buf) {
			free(ctx->sport_rx_cache.isr_pool[i].buf);
			ctx->sport_rx_cache.isr_pool[i].buf = NULL;
		}
		ctx->sport_rx_cache.isr_pool[i].in_use = 0;
	}
}

// Initialize SPORT hardware
static int sport_hw_init(sport_ctx_t *ctx)
{
	sport_t *obj = ctx->sport_obj;
	hal_sport_adapter_t *psport_adapter = &obj->sport_adapter;
	uint32_t page_size = calc_page_size(ctx);
	sport_rate_t rate = sample_rate_to_sport_rate(ctx->sample_rate);
	sport_dl_t data_len = word_len_to_sport_dl(ctx->sport_data_len);
	sport_cl_t ch_len = word_len_to_sport_cl(ctx->sport_ch_len);

	// Initialize SPORT (includes pinmux)
	sport_init(obj);

	// Set master/slave mode
	sport_set_master(obj, ctx->params.sport_role);

	// select slave
	if (ctx->params.sport_role == SPORT_SLAVE_MODE) {
		SPORT_TypeDef *psport_reg = (SPORT_TypeDef *)psport_adapter->base_addr;
		psport_reg->SPORT_SP_CON |= SPORT_BIT_SP_SLAVE_CLK_SEL;
	}

	// Set format (I2S, left-justified, etc.)
	sport_set_format(obj, ctx->params.sport_format);

	// Configure TX parameters
	sport_tx_params(obj,
		ctx->sport_ch_num,	// channel number (CH_4)
		ch_len,			// channel length (32-bit)
		data_len,		// TX0 data length
		data_len,		// TX1 data length
		rate);			// sample rate

	// Configure RX parameters
	sport_rx_params(obj,
		ctx->sport_ch_num,	// channel number (CH_4)
		ch_len,			// channel length (32-bit)
		data_len,		// RX0 data length
		data_len,		// RX1 data length
		rate);			// sample rate

	// Configure TDM mode for 4-channel
	if (ctx->sport_ch_num >= CH_4) {
		sport_tdm_t tdm_mode;
		switch (ctx->sport_ch_num) {
		case CH_4:
			tdm_mode = SPORT_4_TDM;
			break;
		case CH_6:
			tdm_mode = SPORT_6_TDM;
			break;
		case CH_8:
			tdm_mode = SPORT_8_TDM;
			break;
		default:
			tdm_mode = SPORT_NO_TDM;
			break;
		}
		hal_sport_sel_tx_tdm(psport_adapter, tdm_mode);
		hal_sport_sel_rx_tdm(psport_adapter, tdm_mode);

		// Configure FIFO routing for 4-channel:
		// FIFO_0: channels 0/1, FIFO_1: channels 2/3
		hal_sport_tx_fifo_0(psport_adapter, ENABLE, ENABLE);	// ch0ch1, ch2ch3
		hal_sport_tx_fifo_1(psport_adapter, DISABLE, DISABLE);	// ch4ch5, ch6ch7 unused
		hal_sport_rx_fifo_0(psport_adapter, ENABLE, ENABLE);	// ch0ch1, ch2ch3
		hal_sport_rx_fifo_1(psport_adapter, DISABLE, DISABLE);	// ch4ch5, ch6ch7 unused
	}

	// Set byte swap
	sport_byte_swap(obj,
		ctx->params.tx_byte_swap, ctx->params.tx_byte_swap,
		ctx->params.rx_byte_swap, ctx->params.rx_byte_swap);

	// Set DMA buffers - pass the malloc'd DMA buffers directly to HAL
	sport_dma_buffer(obj,
		ctx->sport_rx_cache.dma_tx0_buf,	// TX0 (loopback test pattern)
		NULL,					// TX1 (not used)
		ctx->sport_rx_cache.dma_rx0_buf,	// RX0
		ctx->sport_rx_cache.dma_rx1_buf,	// RX1
		SPORT_DMA_PAGE_NUM,
		page_size);

	hal_sport_dma_reset(&obj->sport_adapter);

	// Register TX DMA callback (keeps the static pattern streaming)
	sport_tx0_dma_cb_handler(obj,
		(sport_irq_cb_t)sport_tx0_irq_cb, (void *)ctx);

	// Register RX DMA callbacks.
	// Per the FIFO_EN register layout, FIFO_0 carries channels 0-3 and
	// FIFO_1 carries channels 4-7: in TDM4 every channel arrives through
	// RX0 DMA already interleaved, and RX1 never fires.
	if (ctx->sport_ch_num == CH_6 || ctx->sport_ch_num == CH_8) {
		// 6/8 channel mode: ch4-7 arrive via RX1, interleave the halves
		sport_rx0_dma_cb_handler(obj,
			(sport_irq_cb_t)sport_rx0_irq_cb, (void *)ctx);
		sport_rx1_dma_cb_handler(obj,
			(sport_irq_cb_t)sport_rx1_irq_cb, (void *)ctx);
	} else {
		// up to 4 channels: RX0 pages are complete frames, push directly
		sport_rx0_dma_cb_handler(obj,
			(sport_irq_cb_t)sport_rx0_2ch_irq_cb, (void *)ctx);
		sport_rx1_dma_cb_handler(obj,
			(sport_irq_cb_t)sport_rx1_2ch_irq_cb, (void *)ctx);
	}

	ctx->sport_inited = 1;

	return 0;
}

// Module create
static void *sport_create(void *arg)
{
	sport_ctx_t *ctx;
	sport_params_t *params = (sport_params_t *)arg;

	ctx = (sport_ctx_t *)malloc(sizeof(sport_ctx_t));
	if (ctx == NULL) {
		return NULL;
	}
	memset(ctx, 0, sizeof(sport_ctx_t));

	// Allocate SPORT object
	ctx->sport_obj = (sport_t *)malloc(sizeof(sport_t));
	if (ctx->sport_obj == NULL) {
		free(ctx);
		return NULL;
	}
	memset(ctx->sport_obj, 0, sizeof(sport_t));

	// Set default parameters
	if (params) {
		memcpy(&ctx->params, params, sizeof(sport_params_t));
	} else {
		memcpy(&ctx->params, &default_sport_params, sizeof(sport_params_t));
	}

	// Apply parameters to context
	ctx->sample_rate = ctx->params.sample_rate;
	ctx->sport_word_length = ctx->params.sport_word_length;
	ctx->rx_word_length = ctx->params.rx_word_length;
	ctx->tx_word_length = ctx->params.tx_word_length;
	ctx->sport_ch_num = ctx->params.sport_ch_num;
	ctx->sport_ch_len = ctx->params.sport_ch_len;
	ctx->sport_data_len = ctx->params.sport_data_len;
	ctx->rx_channel_select = ctx->params.rx_channel_select;
	ctx->pin_group_num = ctx->params.pin_group_num;
	ctx->sport_timestamp_offset = ctx->params.sport_timestamp_offset;
	ctx->rx_channel_swap = ctx->params.rx_channel_swap;

	// Create RX queue - each item is a small sport_rx_t (16 bytes: 2 timestamps + pointer + size)
	ctx->sport_rx_cache.queue = xQueueCreate(SPORT_DMA_PAGE_NUM, sizeof(sport_rx_t));
	if (ctx->sport_rx_cache.queue == NULL) {
		free(ctx->sport_obj);
		free(ctx);
		return NULL;
	}

	// Allocate DMA buffers and ISR pool
	if (sport_allocate_dma_buffers(ctx) != 0) {
		vQueueDelete(ctx->sport_rx_cache.queue);
		free(ctx->sport_obj);
		free(ctx);
		return NULL;
	}

	// NOTE: sport_rx_handle_thread is NOT created here.
	// It is created in CMD_SPORT_APPLY after MM_CMD_INIT_QUEUE_ITEMS has been called,
	// because the handle thread needs mctx->output_recycle to be valid.

	ctx->sport_inited = 0;

	return ctx;
}

// Module destroy
static void *sport_destroy(void *arg)
{
	sport_ctx_t *ctx = (sport_ctx_t *)arg;

	if (ctx) {
		// Stop test data injector
		sport_test_data_running = 0;
		if (sport_test_data_task) {
			vTaskDelete(sport_test_data_task);
			sport_test_data_task = NULL;
		}

		// Stop SPORT
		if (ctx->sport_inited) {
			sport_rx_stop(ctx->sport_obj);
			sport_tx_stop(ctx->sport_obj);
			sport_deinit(ctx->sport_obj);
		}

		// Delete RX handle thread
		if (ctx->sport_rx_task) {
			vTaskDelete(ctx->sport_rx_task);
			ctx->sport_rx_task = NULL;
		}

		// Free DMA buffers and ISR pool
		sport_free_dma_buffers(ctx);

		// Flush and delete queue (free any pending buffers)
		if (ctx->sport_rx_cache.queue) {
			sport_rx_t rx_item;
			while (xQueueReceive(ctx->sport_rx_cache.queue, &rx_item, 0) == pdTRUE) {
				if (rx_item.data) {
					isr_pool_free(ctx, rx_item.data);
				}
			}
			vQueueDelete(ctx->sport_rx_cache.queue);
		}

		// Free SPORT object
		if (ctx->sport_obj) {
			free(ctx->sport_obj);
		}

		free(ctx);
	}

	return NULL;
}

// Module control
static int sport_control(void *arg, int cmd, int val)
{
	sport_ctx_t *ctx = (sport_ctx_t *)arg;
	int ret = 0;

	switch (cmd) {
	case CMD_SPORT_SET_PARAMS: {
		sport_params_t *params = (sport_params_t *)val;
		if (params) {
			memcpy(&ctx->params, params, sizeof(sport_params_t));
			ctx->sample_rate = ctx->params.sample_rate;
			ctx->sport_word_length = ctx->params.sport_word_length;
			ctx->rx_word_length = ctx->params.rx_word_length;
			ctx->tx_word_length = ctx->params.tx_word_length;
			ctx->sport_ch_num = ctx->params.sport_ch_num;
			ctx->sport_ch_len = ctx->params.sport_ch_len;
			ctx->sport_data_len = ctx->params.sport_data_len;
			ctx->rx_channel_select = ctx->params.rx_channel_select;
			ctx->pin_group_num = ctx->params.pin_group_num;
			ctx->sport_timestamp_offset = ctx->params.sport_timestamp_offset;
		}
		break;
	}

	case CMD_SPORT_GET_PARAMS: {
		sport_params_t *params = (sport_params_t *)val;
		if (params) {
			memcpy(params, &ctx->params, sizeof(sport_params_t));
		}
		break;
	}

	case CMD_SPORT_SET_SAMPLERATE:
		ctx->sample_rate = (uint32_t)val;
		ctx->params.sample_rate = ctx->sample_rate;
		break;

	case CMD_SPORT_SET_RX:
		if (val) {
			// Start RX
			if (!ctx->sport_inited) {
				sport_hw_init(ctx);
			}
			// Hand all RX DMA pages to hardware before starting
			for (int i = 0; i < SPORT_DMA_PAGE_NUM; i++) {
				hal_sport_rx0_page_recv(&ctx->sport_obj->sport_adapter);
				hal_sport_rx1_page_recv(&ctx->sport_obj->sport_adapter);
			}
			sport_rx_start(ctx->sport_obj);
			hal_sport_rx_start(&ctx->sport_obj->sport_adapter, ENABLE);
			// Debug: print SPORT register status after starting RX
			{
				SPORT_TypeDef *psport_reg = (SPORT_TypeDef *)ctx->sport_obj->sport_adapter.base_addr;
				uint32_t con = psport_reg->SPORT_SP_CON;
				uint32_t dma_con = psport_reg->SPORT_SP_DMA_CON;
				uint32_t fifo_cnt = psport_reg->SPORT_SP_FIFO_CNT_STA;
				printf("[SPORT RX START] SP_CON=0x%08x DMA_CON=0x%08x FIFO_CNT=0x%08x (rx0_depth=%u rx1_depth=%u, 32=empty)\n",
					(unsigned int)con, (unsigned int)dma_con, (unsigned int)fifo_cnt,
					(unsigned int)((fifo_cnt & SPORT_MASK_SP_RX_DEPTH_CNT_0) >> SPORT_SHIFT_SP_RX_DEPTH_CNT_0),
					(unsigned int)((fifo_cnt & SPORT_MASK_SP_RX_DEPTH_CNT_1) >> SPORT_SHIFT_SP_RX_DEPTH_CNT_1));
				printf("[SPORT RX START] rx0_isr=%u rx1_isr=%u tx0_isr=%u\n",
					(unsigned int)sport_dbg_rx0_isr_cnt, (unsigned int)sport_dbg_rx1_isr_cnt,
					(unsigned int)sport_dbg_tx0_isr_cnt);
			}
		} else {
			// Stop RX
			hal_sport_rx_start(&ctx->sport_obj->sport_adapter, DISABLE);
			sport_rx_stop(ctx->sport_obj);
		}
		break;

	case CMD_SPORT_SET_TX:
		if (val) {
			// Start TX (loopback test pattern from dma_tx0_buf)
			if (!ctx->sport_inited) {
				sport_hw_init(ctx);
			}
			// Hand all pre-filled TX pages to hardware before starting
			for (int i = 0; i < SPORT_DMA_PAGE_NUM; i++) {
				int *pg = sport_get_tx0_page(ctx->sport_obj);
				if (pg) {
					sport_tx0_page_send(ctx->sport_obj, (u32 *)pg);
				} else {
					printf("[SPORT] TX page %d handover FAILED\r\n", i);
				}
			}
			printf("[SPORT] TX started, TX_PAGE_OWN=0x%08x\r\n",
				   (unsigned int)((SPORT_TypeDef *)ctx->sport_obj->sport_adapter.base_addr)->SPORT_SP_TX_PAGE_OWN);
			sport_tx_start(ctx->sport_obj);
			hal_sport_tx_start(&ctx->sport_obj->sport_adapter, ENABLE);
		} else {
			// Stop TX
			hal_sport_tx_start(&ctx->sport_obj->sport_adapter, DISABLE);
			sport_tx_stop(ctx->sport_obj);
		}
		break;

	case CMD_SPORT_SET_RESET:
		if (ctx->sport_inited) {
			sport_reset(ctx->sport_obj);
		}
		break;

	case CMD_SPORT_FORCE_DEINIT:
		if (ctx->sport_inited) {
			sport_rx_stop(ctx->sport_obj);
			sport_tx_stop(ctx->sport_obj);
			sport_deinit(ctx->sport_obj);
			ctx->sport_inited = 0;
		}
		break;

	case CMD_SPORT_SET_FORMAT:
		ctx->params.sport_format = (sport_format_t)val;
		break;

	case CMD_SPORT_SET_ROLE:
		ctx->params.sport_role = (sport_dev_mode_t)val;
		break;

	case CMD_SPORT_SET_CHANNEL_SEL:
		ctx->rx_channel_select = (uint8_t)val;
		ctx->params.rx_channel_select = ctx->rx_channel_select;
		break;

	case CMD_SPORT_SET_TIMESTAMP_OFFSET:
		ctx->sport_timestamp_offset = (uint32_t)val;
		ctx->params.sport_timestamp_offset = ctx->sport_timestamp_offset;
		break;

	case CMD_SPORT_DUMP_STATUS: {
		SPORT_TypeDef *psport_reg = (SPORT_TypeDef *)ctx->sport_obj->sport_adapter.base_addr;
		uint32_t con, dma_con;

		if (!ctx->sport_inited || psport_reg == NULL) {
			printf("[SPORT DBG] not initialized (inited=%d base=%p)\r\n",
				   ctx->sport_inited, (void *)psport_reg);
			break;
		}
		con = psport_reg->SPORT_SP_CON;
		dma_con = psport_reg->SPORT_SP_DMA_CON;
		printf("[SPORT DBG] SP_CON=0x%08x (START_RX=%d RX_DIS=%d SLV_CLK=%d SLV_DATA=%d)\r\n",
			   (unsigned int)con,
			   !!(con & SPORT_BIT_SP_START_RX), !!(con & SPORT_BIT_SP_RX_DISABLE),
			   !!(con & SPORT_BIT_SP_SLAVE_CLK_SEL), !!(con & SPORT_BIT_SP_SLAVE_DATA_SEL));
		printf("[SPORT DBG] SP_CON TX: START_TX=%d TX_DIS=%d\r\n",
			   !!(con & SPORT_BIT_SP_START_TX), !!(con & SPORT_BIT_SP_TX_DISABLE));
		printf("[SPORT DBG] SP_DMA_CON=0x%08x (RX_DMA_EN=%d TX_DMA_EN=%d) RX_PAGE_OWN=0x%08x TX_PAGE_OWN=0x%08x\r\n",
			   (unsigned int)dma_con, !!(dma_con & SPORT_BIT_SP_RX_DMA_EN),
			   !!(dma_con & SPORT_BIT_SP_TX_DMA_EN),
			   (unsigned int)psport_reg->SPORT_SP_RX_PAGE_OWN,
			   (unsigned int)psport_reg->SPORT_SP_TX_PAGE_OWN);
		printf("[SPORT DBG] rx0_isr=%u rx1_isr=%u tx0_isr=%u pool_miss=%u q_wait=%u dma_err=0x%02x\r\n",
			   (unsigned int)sport_dbg_rx0_isr_cnt, (unsigned int)sport_dbg_rx1_isr_cnt,
			   (unsigned int)sport_dbg_tx0_isr_cnt,
			   (unsigned int)sport_dbg_pool_miss_cnt,
			   (unsigned int)uxQueueMessagesWaiting(ctx->sport_rx_cache.queue),
			   (unsigned int)ctx->sport_obj->sport_adapter.dma_err_sta);
		{
			uint32_t fifo_en = psport_reg->SPORT_SP_FIFO_EN_CLK_CON;
			uint32_t fifo_cnt = psport_reg->SPORT_SP_FIFO_CNT_STA;
			// RX depth counters read 32 when the FIFO is EMPTY; any other
			// value means bits are being clocked in from the pins
			printf("[SPORT DBG] FIFO_EN=0x%08x (RXF0_01=%d RXF0_23=%d) FIFO_CNT=0x%08x (rx0_depth=%u rx1_depth=%u, 32=empty)\r\n",
				   (unsigned int)fifo_en,
				   !!(fifo_en & SPORT_BIT_SP_RX_FIFO_0_CH0_1_EN),
				   !!(fifo_en & SPORT_BIT_SP_RX_FIFO_0_CH2_3_EN),
				   (unsigned int)fifo_cnt,
				   (unsigned int)((fifo_cnt & SPORT_MASK_SP_RX_DEPTH_CNT_0) >> SPORT_SHIFT_SP_RX_DEPTH_CNT_0),
				   (unsigned int)((fifo_cnt & SPORT_MASK_SP_RX_DEPTH_CNT_1) >> SPORT_SHIFT_SP_RX_DEPTH_CNT_1));
			printf("[SPORT DBG] ERR_STA=0x%08x RX0_INT_EN=0x%08x RX1_INT_EN=0x%08x\r\n",
				   (unsigned int)psport_reg->SPORT_SP_TX_RX_ERR_STA,
				   (unsigned int)psport_reg->SPORT_SP_RX0_DMA_INT_EN,
				   (unsigned int)psport_reg->SPORT_SP_RX1_DMA_INT_EN);
		}
		{
			// PON pinmux SEL readback
			// 5 = I2S0 function, 15 = plain GPIO (reset default),
			// 6 = UART1, 8 = SWD/JTAG.
			// PF_12/13 @ PON+0x68 (SEL bits 3:0 / 19:16), PF_14/15 @ PON+0x6C
			uint32_t mux_12_13 = *(volatile uint32_t *)(0x40009800UL + 0x68);
			uint32_t mux_14_15 = *(volatile uint32_t *)(0x40009800UL + 0x6C);
			printf("[SPORT DBG] pinmux SEL: PF12=%u PF13=%u PF14=%u PF15=%u (expect 5=I2S0; 15=GPIO 6=UART1 8=SWD)\r\n",
				   (unsigned int)(mux_12_13 & 0xF),
				   (unsigned int)((mux_12_13 >> 16) & 0xF),
				   (unsigned int)(mux_14_15 & 0xF),
				   (unsigned int)((mux_14_15 >> 16) & 0xF));
		}
		break;
	}

	case CMD_SPORT_SET_TEST_DATA:
    if (!sport_test_data_running) {
        sport_test_data_running = 1;
        if (xTaskCreate(sport_pattern_feeder_task, "sport_test_feed", 4096,
                        ctx, tskIDLE_PRIORITY+4, &sport_test_data_task) != pdPASS) {
            printf("[SPORT] test feeder task create failed!\n");
            sport_test_data_running = 0;
            return -1;
        }
        printf("[SPORT] test feeder task started\n");
    }
    break;


	case CMD_SPORT_APPLY:
		// Re-initialize hardware with current parameters
		if (ctx->sport_inited) {
			sport_rx_stop(ctx->sport_obj);
			sport_tx_stop(ctx->sport_obj);
			sport_deinit(ctx->sport_obj);
			ctx->sport_inited = 0;
		}
		// Re-allocate DMA buffers if needed
		sport_free_dma_buffers(ctx);
		if (sport_allocate_dma_buffers(ctx) != 0) {
			ret = -1;
		}
		if (sport_hw_init(ctx) != 0) {
			ret = -1;
		}
		// Create RX handle thread AFTER MMF2 queues are initialized
		// (MM_CMD_INIT_QUEUE_ITEMS must have been called before CMD_SPORT_APPLY)
		if (ctx->sport_rx_task == NULL) {
			if (xTaskCreate(sport_rx_handle_thread, ((const char *)"sport_rx"), 1024, (void *)ctx, tskIDLE_PRIORITY + 5, &ctx->sport_rx_task) != pdPASS) {
				printf("\r\n[SPORT Apply Err] sport_rx_handle_thread: Create Task Error\n");
				ret = -1;
			}
		}
		break;
	case CMD_SPORT_SET_RTSP_CTX:
    	ctx->rtsp_ctx = (mm_context_t *)val;
		printf("[SPORT] RTSP ctx set to %p\n", ctx->rtsp_ctx);
    	break;
	case CMD_SPORT_SET_CHANNEL_SWAP:
		ctx->rx_channel_swap = (mm_context_t *)val;
	default:
		break;
	}

	return ret;
}

// Module handle (process data)
static int sport_handle(void *arg, void *input, void *output)
{
	sport_ctx_t *ctx = (sport_ctx_t *)arg;
	mm_queue_item_t *output_item = (mm_queue_item_t *)output;

	if (ctx == NULL) {
		return -1;
	}

	// RX path: receive data from SPORT and pass to output
	if (output_item != NULL) {
		sport_rx_t rx_item;

		// Receive from queue (small struct, no stack overflow)
		if (xQueueReceive(ctx->sport_rx_cache.queue, &rx_item, 0) == pdTRUE) {
			if (rx_item.data && rx_item.data_size > 0) {
				// Copy data to output item
				uint32_t copy_size = (rx_item.data_size < output_item->size) ?
					rx_item.data_size : output_item->size;

				memcpy((void *)output_item->data_addr, rx_item.data, copy_size);
				output_item->size = copy_size;
				output_item->timestamp = rx_item.timestamp;
				output_item->type = ctx->sport_ch_num;

				// Return buffer to ISR pool (no free!)
				isr_pool_free(ctx, rx_item.data);
				rx_item.data = NULL;

				return copy_size;
			}
		}
	}

	return 0;
}

// Module new item
static void *sport_new_item(void *arg)
{
	sport_ctx_t *ctx = (sport_ctx_t *)arg;
	uint32_t page_size = calc_page_size(ctx);
	mm_queue_item_t *item = (mm_queue_item_t *)malloc(sizeof(mm_queue_item_t));

	if (item) {
		memset(item, 0, sizeof(mm_queue_item_t));
		item->data_addr = (uint32_t)malloc(page_size);
		if (item->data_addr == 0) {
			free(item);
			return NULL;
		}
		item->size = page_size;
		item->type = ctx->sport_ch_num;
	}

	return item;
}

// Module delete item
static void *sport_del_item(void *arg, void *item)
{
	mm_queue_item_t *del_item = (mm_queue_item_t *)item;

	if (del_item) {
		if (del_item->data_addr) {
			free((void *)del_item->data_addr);
		}
		free(del_item);
	}

	return NULL;
}

// Module resize item
static void *sport_rsz_item(void *arg, void *item, int new_size)
{
	mm_queue_item_t *rsz_item = (mm_queue_item_t *)item;

	if (rsz_item && rsz_item->data_addr) {
		void *new_buf = realloc((void *)rsz_item->data_addr, new_size);
		if (new_buf) {
			rsz_item->data_addr = (uint32_t)new_buf;
			rsz_item->size = new_size;
		}
	}

	return item;
}

// Module release item
static void *sport_vrelease_item(void *arg, void *item, int reserved)
{
	return sport_del_item(arg, item);
}

// Module definition
mm_module_t sport_module = {
	.create      = sport_create,
	.destroy     = sport_destroy,
	.control     = sport_control,
	.handle      = sport_handle,

	.new_item    = sport_new_item,
	.del_item    = sport_del_item,
	.rsz_item    = sport_rsz_item,
	.vrelease_item = sport_vrelease_item,

	.output_type = MM_TYPE_ASRC,	// Audio source
	.module_type = MM_TYPE_ASRC,
	.name        = "SPORT",
};