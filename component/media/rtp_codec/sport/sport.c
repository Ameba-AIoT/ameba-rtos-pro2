// #include "rtsp/rtsp_api.h"
// // #include "rtp_api.h"
// #include "sport/sport.h"
// #include "mmf2_dbg.h"
// #include <string.h>
// #include <stdlib.h>
// #include <unistd.h>
// #include <arpa/inet.h>
// #include "lwipconf.h"

// #define PCM_PACKET_BUF_SIZE  1500   // max RTP packet size (MTU safe)
// #define PCM_MAX_PAYLOAD      (PCM_PACKET_BUF_SIZE - RTP_HDR_SZ)

// // Send one RTP fragment of PCM data (UDP or TCP interleaved)
// static int rtp_pcm_send_fragment(struct stream_context *stream_ctx, struct rtp_object *pObject,
//                                  uint8_t *data, int data_len, int is_last)
// {
//     struct rtsp_context *rtsp_ctx = stream_ctx->parent;
//     int cast_mode = rtsp_ctx->transport[stream_ctx->index].castMode;
//     int socket, ret;
//     uint8_t packet_buf[PCM_PACKET_BUF_SIZE];
//     uint8_t *ptr, *rtp_hdr_pos;

//     socket = pObject->connect_ctx.socket_id;

//     if (cast_mode == UNICAST_UDP_MODE) {
//         struct sockaddr_in adr_cs;
//         adr_cs.sin_family = AF_INET;
//         adr_cs.sin_addr.s_addr = *(uint32_t *)pObject->connect_ctx.remote_ip;
//         adr_cs.sin_port = htons(pObject->connect_ctx.remote_port);
//         ptr = packet_buf;

//         memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ);
//         if (is_last) {
//             ((rtp_hdr_t *)ptr)->m = 1;  // set marker on last fragment
//         }
//         ptr += RTP_HDR_SZ;

//         memcpy(ptr, data, data_len);

//         ret = sendto(socket, packet_buf, RTP_HDR_SZ + data_len, 0,
//                      (struct sockaddr *)&adr_cs, sizeof(adr_cs));
//     } else { // TCP interleaved
//         packet_buf[0] = '$';
//         packet_buf[1] = rtsp_ctx->transport[stream_ctx->index].interleaved_low;
//         ptr = packet_buf + 4;

//         memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ);
//         if (is_last) {
//             ((rtp_hdr_t *)ptr)->m = 1;
//         }
//         ptr += RTP_HDR_SZ;

//         memcpy(ptr, data, data_len);

//         uint16_t rtp_len = RTP_HDR_SZ + data_len;
//         *(uint16_t *)(packet_buf + 2) = htons(rtp_len);

//         ret = write(socket, packet_buf, 4 + RTP_HDR_SZ + data_len);
//     }

//     return (ret < 0) ? -EAGAIN : 0;
// }

// // Handler for PCM_RAW (L24 RTP payload format)
// // Fragments large frames into MTU-sized RTP packets
// int rtp_o_sport_handler(struct stream_context *stream_ctx, struct rtp_object *payload)
// {
//     struct rtsp_context *rtsp_ctx = stream_ctx->parent;
//     uint8_t *data = payload->data;
//     int remaining = payload->len;
//     int ret = 0;
//     int frag_nr = 0;

//     if (payload->rtphdr == NULL) {
//         payload->rtphdr = malloc(RTP_HDR_SZ);
//         if (!payload->rtphdr) return -ENOMEM;
//     }

//     printf("[SPORT] handler called: data_len=%d, ts_in=%lu\n\r",
//            payload->len, (unsigned long)payload->timestamp);

//     // Convert ms timestamp to RTP timestamp (L24 clock rate = samplerate)
//     uint32_t frame_tick_cnt = payload->timestamp;
//     payload->timestamp = rtsp_get_timestamp(stream_ctx, frame_tick_cnt);

//     printf("[SPORT] RTP ts=%lu (converted from %lu ms, rate=%lu)\n\r",
//            (unsigned long)payload->timestamp,
//            (unsigned long)frame_tick_cnt,
//            (unsigned long)stream_ctx->codec->clock_rate);

//     // Fill RTP header once — same timestamp for all fragments.
//     // PT must match what the SDP advertises: codec->pt + stream_id
//     rtp_fill_header(payload->rtphdr, 2, 0, 0, 0, 0,
//                     stream_ctx->codec->pt + stream_ctx->stream_id,
//                     rtsp_ctx->rtpseq[stream_ctx->index],
//                     payload->timestamp,
//                     stream_ctx->stream_id);

//     printf("[SPORT] PT=%d stream_id=%d codec->pt=%d initial_seq=%d\n\r",
//            stream_ctx->codec->pt + stream_ctx->stream_id,
//            stream_ctx->stream_id,
//            stream_ctx->codec->pt,
//            rtsp_ctx->rtpseq[stream_ctx->index]);

//     // Fragment: send in PCM_MAX_PAYLOAD-sized chunks
//     while (remaining > 0) {
//         int chunk = (remaining > PCM_MAX_PAYLOAD) ? PCM_MAX_PAYLOAD : remaining;
//         int is_last = (chunk == remaining) ? 1 : 0;

//         printf("[SPORT] frag %d: offset=%d chunk=%d remaining=%d is_last=%d seq=%d\n\r",
//                frag_nr,
//                (int)(data - (uint8_t *)payload->data),
//                chunk, remaining, is_last,
//                rtsp_ctx->rtpseq[stream_ctx->index]);

//         ret = rtp_pcm_send_fragment(stream_ctx, payload, data, chunk, is_last);
//         if (ret < 0) {
//             printf("[SPORT] sendto failed: %d\n\r", ret);
//             return -EAGAIN;
//         }

//         data += chunk;
//         remaining -= chunk;
//         frag_nr++;

//         // Increment sequence number for each fragment except the last
//         // (the caller increments once more after the final fragment)
//         if (!is_last) {
//             rtsp_ctx->rtpseq[stream_ctx->index]++;
//             // Update sequence number in the shared header
//             rtp_fill_header(payload->rtphdr, 2, 0, 0, 0, 0,
//                             stream_ctx->codec->pt + stream_ctx->stream_id,
//                             rtsp_ctx->rtpseq[stream_ctx->index],
//                             payload->timestamp,
//                             stream_ctx->stream_id);
//         }
//     }

//     printf("[SPORT] done: %d fragments sent, final_seq=%d\n\r",
//            frag_nr, rtsp_ctx->rtpseq[stream_ctx->index] + 1);

//     // Final seq increment (the caller expects this)
//     rtsp_ctx->rtpseq[stream_ctx->index]++;
//     return 0;
// }

// no test
// #include "FreeRTOS.h"
// #include <platform_stdlib.h>
// #include "platform_opts.h"

// #include "rtsp/rtsp_api.h"
// #if defined(CONFIG_PLATFORM_8195BHP) || defined(CONFIG_PLATFORM_8721D) || defined(CONFIG_PLATFORM_8735B)
// #include "mmf2_dbg.h"
// #include "mmf2_mediatime_8735b.h"
// #endif


// #include "sport/sport.h"
// // #include "mmf2_dbg.h"
// // #include <string.h>
// // #include <stdlib.h>
// // #include <unistd.h>
// // #include <arpa/inet.h>
// #include "lwipconf.h"

// #define PCM_PACKET_BUF_SIZE  1500   // max RTP packet size (MTU safe)
// #define PCM_MAX_PAYLOAD      (PCM_PACKET_BUF_SIZE - RTP_HDR_SZ - 4)  // -4 for TCP interleaved framing ($ + channel + 2-byte length)

// // Send one RTP fragment of PCM data (UDP or TCP interleaved)
// int rtp_pcm_send_unicast(struct stream_context *stream_ctx, struct rtp_object *pObject,
//                                  uint8_t *data, int data_len, int is_last)
// {
//     struct rtsp_context *rtsp_ctx = stream_ctx->parent;
//     int cast_mode = rtsp_ctx->transport[stream_ctx->index].castMode;
//     int socket, ret;
//     uint8_t packet_buf[PCM_PACKET_BUF_SIZE];
//     uint8_t *ptr;

//     socket = pObject->connect_ctx.socket_id;

//     if (cast_mode == UNICAST_UDP_MODE) {
//         struct sockaddr_in adr_cs;
//         adr_cs.sin_family = AF_INET;
//         adr_cs.sin_addr.s_addr = *(uint32_t *)pObject->connect_ctx.remote_ip;
//         adr_cs.sin_port = htons(pObject->connect_ctx.remote_port);
//         ptr = packet_buf;

//         memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ);
//         if (is_last) {
//             ((rtp_hdr_t *)ptr)->m = 1;  // set marker on last fragment
//         }
//         ptr += RTP_HDR_SZ;

//         memcpy(ptr, data, data_len);

//         printf("[RTP SEND UDP] First 32 bytes: ");
//         for (int i = 0; i < 32 && i < (RTP_HDR_SZ + data_len); i++) {
//             printf("%02X ", packet_buf[i]);
//         }
//         printf("\n");

//         ret = sendto(socket, packet_buf, RTP_HDR_SZ + data_len, 0,
//                      (struct sockaddr *)&adr_cs, sizeof(adr_cs));
//     } else { // TCP interleaved
//         packet_buf[0] = '$';
//         packet_buf[1] = rtsp_ctx->transport[stream_ctx->index].interleaved_low;
//         ptr = packet_buf + 4;

//         memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ);
//         if (is_last) {
//             ((rtp_hdr_t *)ptr)->m = 1;
//         }
//         ptr += RTP_HDR_SZ;

//         memcpy(ptr, data, data_len);

//         uint16_t rtp_len = RTP_HDR_SZ + data_len;
//         *(uint16_t *)(packet_buf + 2) = htons(rtp_len);
//         // uint16_t rtp_len = RTP_HDR_SZ + data_len;
//         // uint16_t net_len = htons(rtp_len);
//         // memcpy(packet_buf + 2, &net_len, sizeof(net_len)); // safe, no unaligned access

//         // Debug: show TCP packet
//         printf("[RTP SEND TCP] First 32 bytes: ");
//         for (int i = 0; i < 32 && i < (4 + RTP_HDR_SZ + data_len); i++) {
//             printf("%02X ", packet_buf[i]);
//         }
//         printf("\n");

//         ret = write(socket, packet_buf, 4 + RTP_HDR_SZ + data_len);
//     }

//     return (ret < 0) ? -EAGAIN : 0;
// }

// // Handler processing live left-aligned customer audio blocks (10240 bytes)
// int rtp_o_sport_handler(struct stream_context *stream_ctx, struct rtp_object *payload)
// {
//     struct rtsp_context *rtsp_ctx = stream_ctx->parent;
//     uint8_t *src = (uint8_t *)payload->data;
//     uint8_t *dst = (uint8_t *)payload->data; // Overwrite IN-PLACE safely to prevent stack errors
//     int src_remaining = payload->len;         // 10240 bytes
//     int ret = 0;
//     int frag_nr = 0;

//     if (payload->rtphdr == NULL) {
//         payload->rtphdr = malloc(RTP_HDR_SZ);
//         if (!payload->rtphdr) return -ENOMEM;
//     }

//     // --- 1. Convert 32-bit little-endian slots to big-endian for network ---
//     int total_slots = src_remaining / 4; 
//     for (int i = 0; i < total_slots; i++) {
//         uint32_t host_val;
//         memcpy(&host_val, src, 4); // Read 32-bit slot sequentially from pre-sorted ISR buffer
        
//         // Keep full 32-bit data in big-endian order for VLC
//         *dst++ = (uint8_t)((host_val >> 24) & 0xFF); // Byte 3 (MSB)
//         *dst++ = (uint8_t)((host_val >> 16) & 0xFF); // Byte 2
//         *dst++ = (uint8_t)((host_val >> 8)  & 0xFF); // Byte 1
//         *dst++ = (uint8_t)(host_val & 0xFF);         // Byte 0 (LSB)

//         src += 4;

//     }
//     // int total_samples = src_remaining / 3;
//     // for (int i = 0; i < total_samples; i++) {
//     //     uint8_t b0 = src[0]; // LSB
//     //     uint8_t b1 = src[1]; // middle
//     //     uint8_t b2 = src[2]; // MSB

//     //     // Repack to big-endian order
//     //     dst[0] = b2;
//     //     dst[1] = b1;
//     //     dst[2] = b0;

//     //     src += 3;
//     //     dst += 3;
//     // }

//     // Update packed network track specs (10240 bytes dropped down to 7680 packed bytes)
//     int packed_len = dst - (uint8_t *)payload->data;
//     uint8_t *data = (uint8_t *)payload->data;
//     int remaining = packed_len;

//     // Convert baseline hardware clock milliseconds over to Network Audio clock base
//     uint32_t current_rtp_ts = rtsp_get_timestamp(stream_ctx, payload->timestamp);

//     // --- 2. TIMELINE PACING LOOP ---
//     while (remaining > 0) {
//         int chunk = (remaining > PCM_MAX_PAYLOAD) ? PCM_MAX_PAYLOAD : remaining;
//         int is_last = (chunk == remaining) ? 1 : 0;

//         // Build the current fragment RTP header parameters
//         rtp_fill_header(payload->rtphdr, 2, 0, 0, 0, 0,
//                         stream_ctx->codec->pt + stream_ctx->stream_id,
//                         rtsp_ctx->rtpseq[stream_ctx->index],
//                         current_rtp_ts,
//                         stream_ctx->stream_id);
//         printf("[rtp_o_sport_handler] Packed PCM first 32 bytes: ");
//         for (int i = 0; i < 32 && i < packed_len; i++) {
//             printf("%02X ", ((uint8_t*)payload->data)[i]);
//         }
//         printf("\n");
//         ret = rtp_pcm_send_unicast(stream_ctx, payload, data, chunk, is_last);
//         if (ret < 0) {
//             return -EAGAIN;
//         }

//         data += chunk;
//         remaining -= chunk;
//         frag_nr++;

//         // 1484 bytes / 4 bytes per sample / 4 channels = 92 clock frames progress per chunk
//         int samples_sent = chunk / (4 * 4); 

//         rtsp_ctx->rtpseq[stream_ctx->index]++;
//         current_rtp_ts += samples_sent; // Move timeline forward cleanly

//         // Let the system rest between fragments so video frames can pass through
//         vTaskDelay(pdMS_TO_TICKS(1)); 
//     }

//     return 0;
// }

// //no test
// // Handler processing live left-aligned customer audio blocks (10240 bytes)
// int rtp_o_sport_handler(struct stream_context *stream_ctx, struct rtp_object *payload)
// {
//     struct rtsp_context *rtsp_ctx = stream_ctx->parent;
//     uint8_t *src = (uint8_t *)payload->data;
//     uint8_t *dst = (uint8_t *)payload->data; // Overwrite IN-PLACE safely to prevent stack errors
//     int src_remaining = payload->len;         // 10240 bytes
//     int ret = 0;
//     int frag_nr = 0;

//     if (payload->rtphdr == NULL) {
//         payload->rtphdr = malloc(RTP_HDR_SZ);
//         if (!payload->rtphdr) return -ENOMEM;
//     }

//     // --- 1. LEFT-ALIGNED EXTRACTION: Pack 4-byte slots into 3-byte network L24 ---
//     int total_slots = src_remaining / 4; 
//     for (int i = 0; i < total_slots; i++) {
//         uint32_t host_val;
//         memcpy(&host_val, src, 4); // Read 32-bit slot sequentially from pre-sorted ISR buffer
        
//         // Extract real active speech values from upper bytes, stripping trailing 0x00 padding byte
//         *dst++ = (uint8_t)((host_val >> 24) & 0xFF); // High Byte (Most Significant Speech bits)
//         *dst++ = (uint8_t)((host_val >> 16) & 0xFF); // Mid Byte
//         *dst++ = (uint8_t)((host_val >> 8)  & 0xFF); // Low Byte (Least Significant Speech bits)

//         src += 4; 
//     }

//     // Update packed network track specs (10240 bytes dropped down to 7680 packed bytes)
//     int packed_len = dst - (uint8_t *)payload->data;
//     uint8_t *data = (uint8_t *)payload->data;
//     int remaining = packed_len;

//     // Convert baseline hardware clock milliseconds over to Network Audio clock base
//     uint32_t current_rtp_ts = rtsp_get_timestamp(stream_ctx, payload->timestamp);

//     // --- 2. TIMELINE PACING LOOP ---
//     while (remaining > 0) {
//         int chunk = (remaining > PCM_MAX_PAYLOAD) ? PCM_MAX_PAYLOAD : remaining;
//         int is_last = (chunk == remaining) ? 1 : 0;

//         // Build the current fragment RTP header parameters
//         rtp_fill_header(payload->rtphdr, 2, 0, 0, 0, 0,
//                         stream_ctx->codec->pt + stream_ctx->stream_id,
//                         rtsp_ctx->rtpseq[stream_ctx->index],
//                         current_rtp_ts,
//                         stream_ctx->stream_id);

        
//         printf("[rtp_o_sport_handler] Packed PCM first 32 bytes: ");
//         for (int i = 0; i < 32 && i < packed_len; i++) {
//             printf("%02X ", ((uint8_t*)payload->data)[i]);
//         }
//         printf("\n");

//         ret = rtp_pcm_send_unicast(stream_ctx, payload, data, chunk, is_last);
//         if (ret < 0) {
//             return -EAGAIN;
//         }
//         data += chunk;
//         remaining -= chunk;
//         frag_nr++;

//         // 1488 bytes / 3 bytes per sample / 4 channels = 124 clock frames progress per chunk
//         int samples_sent = chunk / (3 * 4); 

//         rtsp_ctx->rtpseq[stream_ctx->index]++;
//         current_rtp_ts += samples_sent; // Move timeline forward cleanly

//         // Let the system rest between fragments so video frames can pass through
//         vTaskDelay(pdMS_TO_TICKS(1)); 
//     }

//     return 0;
// }




// // Handler processing live left-aligned customer audio blocks (10240 bytes)
// int rtp_o_sport_handler(struct stream_context *stream_ctx, struct rtp_object *payload)
// {
//     struct rtsp_context *rtsp_ctx = stream_ctx->parent;
//     uint8_t *data = (uint8_t *)payload->data;
//     int remaining = payload->len;   // keep full 10240 bytes
//     int ret = 0;
//     int frag_nr = 0;

//     if (payload->rtphdr == NULL) {
//         payload->rtphdr = malloc(RTP_HDR_SZ);
//         if (!payload->rtphdr) return -ENOMEM;
//     }

//     // Convert baseline hardware clock milliseconds over to Network Audio clock base
//     uint32_t current_rtp_ts = rtsp_get_timestamp(stream_ctx, payload->timestamp);

//     // --- TIMELINE PACING LOOP ---
//     while (remaining > 0) {
//         int chunk = (remaining > PCM_MAX_PAYLOAD) ? PCM_MAX_PAYLOAD : remaining;
//         int is_last = (chunk == remaining);

//         // Build the current fragment RTP header parameters
//         rtp_fill_header(payload->rtphdr, 2, 0, 0, 0, 0,
//                         stream_ctx->codec->pt + stream_ctx->stream_id,
//                         rtsp_ctx->rtpseq[stream_ctx->index],
//                         current_rtp_ts,
//                         stream_ctx->stream_id);

//         ret = rtp_pcm_send_unicast(stream_ctx, payload, data, chunk, is_last);
//         if (ret < 0) {
//             return -EAGAIN;
//         }

//         data += chunk;
//         remaining -= chunk;
//         frag_nr++;

//         // Timeline advance: still based on 24‑bit samples per channel
//         int samples_sent = chunk / (3 * 4); 
//         rtsp_ctx->rtpseq[stream_ctx->index]++;
//         current_rtp_ts += samples_sent;

//         vTaskDelay(pdMS_TO_TICKS(1));
//     }

//     return 0;
// }


















//13/8/26

#include "rtsp/rtsp_api.h"
// #include "rtp_api.h"
#include "sport/sport.h"
#include "mmf2_dbg.h"
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include "lwipconf.h"

#define PCM_PACKET_BUF_SIZE  1500   // max RTP packet size (MTU safe)
#define PCM_MAX_PAYLOAD      (PCM_PACKET_BUF_SIZE - RTP_HDR_SZ)

// Send one RTP fragment of PCM data (UDP or TCP interleaved)
static int rtp_pcm_send_fragment(struct stream_context *stream_ctx, struct rtp_object *pObject,
                                 uint8_t *data, int data_len, int is_last)
{
    struct rtsp_context *rtsp_ctx = stream_ctx->parent;
    int cast_mode = rtsp_ctx->transport[stream_ctx->index].castMode;
    int socket, ret;
    uint8_t packet_buf[PCM_PACKET_BUF_SIZE];
    uint8_t *ptr;

    socket = pObject->connect_ctx.socket_id;

    if (cast_mode == UNICAST_UDP_MODE) {
        struct sockaddr_in adr_cs;
        adr_cs.sin_family = AF_INET;
        adr_cs.sin_addr.s_addr = *(uint32_t *)pObject->connect_ctx.remote_ip;
        adr_cs.sin_port = htons(pObject->connect_ctx.remote_port);
        ptr = packet_buf;

        memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ);
        if (is_last) {
            ((rtp_hdr_t *)ptr)->m = 1;  // set marker on last fragment
        }
        ptr += RTP_HDR_SZ;

        memcpy(ptr, data, data_len);
        printf("[RTP SEND UDP] First 32 bytes: ");
        for (int i = 0; i < 32 && i < (RTP_HDR_SZ + data_len); i++) {
            printf("%02X ", packet_buf[i]);
        }
        printf("\n");
        ret = sendto(socket, packet_buf, RTP_HDR_SZ + data_len, 0,
                     (struct sockaddr *)&adr_cs, sizeof(adr_cs));
    } else { // TCP interleaved
        packet_buf[0] = '$';
        packet_buf[1] = rtsp_ctx->transport[stream_ctx->index].interleaved_low;
        ptr = packet_buf + 4;

        memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ);
        if (is_last) {
            ((rtp_hdr_t *)ptr)->m = 1;
        }
        ptr += RTP_HDR_SZ;

        memcpy(ptr, data, data_len);

        uint16_t rtp_len = RTP_HDR_SZ + data_len;
        uint16_t net_len = htons(rtp_len);
        memcpy(packet_buf + 2, &net_len, sizeof(net_len)); // safe, no unaligned access
        printf("[RTP SEND TCP] First 32 bytes: ");
        for (int i = 0; i < 32 && i < (4 + RTP_HDR_SZ + data_len); i++) {
            printf("%02X ", packet_buf[i]);
        }
        printf("\n");
        ret = write(socket, packet_buf, 4 + RTP_HDR_SZ + data_len);
    }

    return (ret < 0) ? -EAGAIN : 0;
}

// Handler processing live left-aligned customer audio blocks (10240 bytes)
int rtp_o_sport_handler(struct stream_context *stream_ctx, struct rtp_object *payload)
{
    struct rtsp_context *rtsp_ctx = stream_ctx->parent;
    uint8_t *src = (uint8_t *)payload->data;
    uint8_t *dst = (uint8_t *)payload->data; // Overwrite IN-PLACE safely to prevent stack errors
    int src_remaining = payload->len;         // 10240 bytes
    int ret = 0;
    int frag_nr = 0;

    if (payload->rtphdr == NULL) {
        payload->rtphdr = malloc(RTP_HDR_SZ);
        if (!payload->rtphdr) return -ENOMEM;
    }

    // --- 1. LEFT-ALIGNED EXTRACTION: Pack 4-byte slots into 3-byte network L24 ---
    int total_slots = src_remaining / 4; 
    for (int i = 0; i < total_slots; i++) {
        uint32_t host_val;
        memcpy(&host_val, src, 4); // Read 32-bit slot sequentially from pre-sorted ISR buffer
        
        // Extract real active speech values from upper bytes, stripping trailing 0x00 padding byte
        *dst++ = (uint8_t)((host_val >> 24) & 0xFF); // High Byte (Most Significant Speech bits)
        *dst++ = (uint8_t)((host_val >> 16) & 0xFF); // Mid Byte
        *dst++ = (uint8_t)((host_val >> 8)  & 0xFF); // Low Byte (Least Significant Speech bits)

        src += 4; 
    }

    // Update packed network track specs (10240 bytes dropped down to 7680 packed bytes)
    int packed_len = dst - (uint8_t *)payload->data;
    uint8_t *data = (uint8_t *)payload->data;
    int remaining = packed_len;

    // Convert baseline hardware clock milliseconds over to Network Audio clock base
    uint32_t current_rtp_ts = rtsp_get_timestamp(stream_ctx, payload->timestamp);

    // --- 2. TIMELINE PACING LOOP ---
    while (remaining > 0) {
        int chunk = (remaining > PCM_MAX_PAYLOAD) ? PCM_MAX_PAYLOAD : remaining;
        int is_last = (chunk == remaining) ? 1 : 0;

        // Build the current fragment RTP header parameters
        rtp_fill_header(payload->rtphdr, 2, 0, 0, 0, 0,
                        stream_ctx->codec->pt + stream_ctx->stream_id,
                        rtsp_ctx->rtpseq[stream_ctx->index],
                        current_rtp_ts,
                        stream_ctx->stream_id);
        printf("[rtp_o_sport_handler] Packed PCM first 32 bytes: ");
        for (int i = 0; i < 32 && i < packed_len; i++) {
            printf("%02X ", ((uint8_t*)payload->data)[i]);
        }
        printf("\n");
        ret = rtp_pcm_send_fragment(stream_ctx, payload, data, chunk, is_last);
        if (ret < 0) {
            return -EAGAIN;
        }

        data += chunk;
        remaining -= chunk;
        frag_nr++;

        // 1488 bytes / 3 bytes per sample / 4 channels = 124 clock frames progress per chunk
        int samples_sent = chunk / (3 * 4); 

        rtsp_ctx->rtpseq[stream_ctx->index]++;
        current_rtp_ts += samples_sent; // Move timeline forward cleanly

        // Let the system rest between fragments so video frames can pass through
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }

    return 0;
}


// #include "FreeRTOS.h"
// #include <platform_stdlib.h>
// #include "platform_opts.h"

// #include "rtsp/rtsp_api.h"
// #include "sport/sport.h"
// #include "mmf2_dbg.h"
// #include <string.h>
// #include <stdlib.h>
// #include <unistd.h>
// #include <arpa/inet.h>
// #include "lwipconf.h"

// #if defined(CONFIG_PLATFORM_8195BHP) || defined(CONFIG_PLATFORM_8721D) || defined(CONFIG_PLATFORM_8735B)
// #include "mmf2_mediatime_8735b.h"
// #endif

// #define WRITE_SIZE 1460              // safe MTU size for TCP
// #define PCM_PACKET_BUF_SIZE 1500     // max RTP packet size for UDP
// #define PCM_MAX_PAYLOAD (PCM_PACKET_BUF_SIZE - RTP_HDR_SZ)

// extern int max_skb_buf_num;
// extern int skbdata_used_num;
// extern uint8_t flag_show_ts_diff;

// // Send one RTP fragment of PCM data (UDP or TCP interleaved)
// static int rtp_pcm_send_unicast(struct stream_context *stream_ctx,
//                                 struct rtp_object *pObject,
//                                 uint8_t *data, int data_len, int is_last)
// {
//     struct rtsp_context *rtsp_ctx = stream_ctx->parent;
//     int cast_mode = rtsp_ctx->transport[stream_ctx->index].castMode;
//     int socket = pObject->connect_ctx.socket_id;
//     int ret;
//     uint8_t udp_packet_buf[PCM_PACKET_BUF_SIZE];
//     uint8_t tcp_packet_buf[WRITE_SIZE];
//     uint8_t *ptr;

//     if (cast_mode == UNICAST_UDP_MODE) {
//         struct sockaddr_in adr_cs;
//         adr_cs.sin_family = AF_INET;
//         memcpy(&adr_cs.sin_addr.s_addr, pObject->connect_ctx.remote_ip, 4);
//         adr_cs.sin_port = htons(pObject->connect_ctx.remote_port);
//         ptr = udp_packet_buf;

//         memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ);
//         if (is_last) ((rtp_hdr_t *)ptr)->m = 1;
//         ptr += RTP_HDR_SZ;

//         memcpy(ptr, data, data_len);
//         printf("[RTP SEND UDP] First 32 bytes: ");
//         for (int i = 0; i < 32 && i < (RTP_HDR_SZ + data_len); i++) {
//             printf("%02X ", udp_packet_buf[i]);
//         }
//         printf("\n");
//         ret = sendto(socket, udp_packet_buf, RTP_HDR_SZ + data_len, 0,
//                      (struct sockaddr *)&adr_cs, sizeof(adr_cs));
//     } else { // TCP interleaved
//         tcp_packet_buf[0] = '$';
//         tcp_packet_buf[1] = rtsp_ctx->transport[stream_ctx->index].interleaved_low;
//         ptr = tcp_packet_buf + 4;

//         memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ);
//         if (is_last) ((rtp_hdr_t *)ptr)->m = 1;
//         ptr += RTP_HDR_SZ;

//         memcpy(ptr, data, data_len);

//         uint16_t rtp_len = RTP_HDR_SZ + data_len;
//         uint16_t net_len = htons(rtp_len);
//         memcpy(tcp_packet_buf + 2, &net_len, sizeof(net_len)); // safe
//         printf("[RTP SEND TCP] First 32 bytes: ");
//         for (int i = 0; i < 32 && i < (4 + RTP_HDR_SZ + data_len); i++) {
//             printf("%02X ", tcp_packet_buf[i]);
//         }
//         printf("\n");
//         ret = write(socket, tcp_packet_buf, 4 + RTP_HDR_SZ + data_len);
//     }

//     return (ret < 0) ? -EAGAIN : 0;
// }

// // Handler processing SPORT PCM_RAW (L24) blocks
// int rtp_o_sport_handler(struct stream_context *stream_ctx, struct rtp_object *payload)
// {
//     struct rtsp_context *rtsp_ctx = stream_ctx->parent;
//     uint8_t *src = (uint8_t *)payload->data;
//     uint8_t *dst = (uint8_t *)payload->data;
//     int src_remaining = payload->len;
//     int ret = 0;

//     if (payload->rtphdr == NULL) {
//         payload->rtphdr = malloc(RTP_HDR_SZ);
//         if (!payload->rtphdr) return -ENOMEM;
//     }

//     // Pack 32-bit slots into 3-byte L24
//     int total_slots = src_remaining / 4;
//     for (int i = 0; i < total_slots; i++) {
//         uint32_t host_val;
//         memcpy(&host_val, src, 4);
//         *dst++ = (host_val >> 24) & 0xFF;
//         *dst++ = (host_val >> 16) & 0xFF;
//         *dst++ = (host_val >> 8)  & 0xFF;
//         src += 4;
//     }

//     int packed_len = dst - (uint8_t *)payload->data;
//     uint8_t *data = (uint8_t *)payload->data;
//     int remaining = packed_len;

//     uint32_t frame_tick_cnt = payload->timestamp;
//     payload->timestamp = rtsp_get_timestamp(stream_ctx, frame_tick_cnt);

//     if (flag_show_ts_diff) {
//         printf("[SPORT][%d] ts += %lu\n\r", rtsp_ctx->id,
//                (unsigned long)(payload->timestamp - stream_ctx->periodic_report.last_timestamp));
//     }
//     stream_ctx->periodic_report.last_timestamp = payload->timestamp;

//     uint32_t current_tick   = mm_read_mediatime_ms() + stream_ctx->time_offset;
//     uint32_t current_rtp_ts = rtsp_get_timestamp(stream_ctx, frame_tick_cnt);
//     uint32_t tick_diff      = (current_tick > frame_tick_cnt) ? current_tick - frame_tick_cnt : 0;

//     if ((!stream_ctx->framecontrol.drop_frame_enable) ||
//         (tick_diff <= stream_ctx->framecontrol.rtp_drop_threshold)) {
//         // Outer transport switch like AAC/Opus
//         switch (rtsp_ctx->transport[stream_ctx->index].castMode) {
//         case UNICAST_UDP_MODE:
//             while (remaining > 0) {
//                 int chunk   = (remaining > PCM_MAX_PAYLOAD) ? PCM_MAX_PAYLOAD : remaining;
//                 int is_last = (chunk == remaining);

//                 rtp_fill_header(payload->rtphdr, 2, 0, 0, 0, 0,
//                                 stream_ctx->codec->pt + stream_ctx->stream_id,
//                                 rtsp_ctx->rtpseq[stream_ctx->index],
//                                 current_rtp_ts,
//                                 stream_ctx->stream_id);

//                 if (is_last) ((rtp_hdr_t*)payload->rtphdr)->m = 1;
//                 printf("[rtp_o_sport_handler UNICAST_UDP_MODE] Packed PCM first 32 bytes: ");
//                 for (int i = 0; i < 32 && i < packed_len; i++) {
//                     printf("%02X ", ((uint8_t*)payload->data)[i]);
//                 }
//                 printf("\n");
//                 ret = rtp_pcm_send_unicast(stream_ctx, payload, data, chunk, is_last);
//                 if (ret < 0) return -EAGAIN;

//                 data      += chunk;
//                 remaining -= chunk;
//                 rtsp_ctx->rtpseq[stream_ctx->index]++;
//                 current_rtp_ts += chunk / (3 * 4);

//                 vTaskDelay(pdMS_TO_TICKS(1));
//             }
//             break;

//         case UNICAST_TCP_MODE:
//             while (remaining > 0) {
//                 int chunk   = (remaining > (WRITE_SIZE - RTP_HDR_SZ)) ? (WRITE_SIZE - RTP_HDR_SZ) : remaining;
//                 int is_last = (chunk == remaining);

//                 rtp_fill_header(payload->rtphdr, 2, 0, 0, 0, 0,
//                                 stream_ctx->codec->pt + stream_ctx->stream_id,
//                                 rtsp_ctx->rtpseq[stream_ctx->index],
//                                 current_rtp_ts,
//                                 stream_ctx->stream_id);

//                 if (is_last) ((rtp_hdr_t*)payload->rtphdr)->m = 1;
//                 printf("[rtp_o_sport_handler UNICAST_TCP_MODE] Packed PCM first 32 bytes: ");
//                 for (int i = 0; i < 32 && i < packed_len; i++) {
//                     printf("%02X ", ((uint8_t*)payload->data)[i]);
//                 }
//                 printf("\n");
//                 ret = rtp_pcm_send_unicast(stream_ctx, payload, data, chunk, is_last);
//                 if (ret < 0) return -EAGAIN;

//                 data      += chunk;
//                 remaining -= chunk;
//                 rtsp_ctx->rtpseq[stream_ctx->index]++;
//                 current_rtp_ts += chunk / (3 * 4);

//                 vTaskDelay(pdMS_TO_TICKS(1));
//             }
//             break;

//         case MULTICAST_MODE:
//             return -EINVAL;
//         default:
//             return -EINVAL;
//         }
//     } else {
//         // Drop frame path
//         stream_ctx->periodic_report.drop_frame++;
//         rtp_object_set_fs(payload, 0);
//         rtp_object_set_fe(payload, 0);
//         rtp_object_set_fk(payload, 0);
//         rtp_object_set_fd(payload, 0);
//         payload->data = NULL;
//         payload->len  = 0;
//         return -EAGAIN;
//     }

//     stream_ctx->statistics.do_start_check = 0;
//     payload->data = NULL;
//     payload->len  = 0;
//     return ret;
// }
