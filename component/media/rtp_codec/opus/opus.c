#include "FreeRTOS.h"
#include <platform_stdlib.h>
#include "platform_opts.h"

#include "rtsp/rtsp_api.h"
#include "opus/opus.h"
//#include "sockets.h"
#include "lwip/netif.h"
#if defined(CONFIG_PLATFORM_8195BHP) || defined(CONFIG_PLATFORM_8721D) || defined(CONFIG_PLATFORM_8735B)
#include "mmf2_dbg.h"
#include "mmf2_mediatime_8735b.h"
#endif

#define WRITE_SIZE 1460
#define RETRY_TIMES 3
extern int max_skb_buf_num;
extern int skbdata_used_num;

//extern uint32_t rtp_drop_threshold;
extern uint8_t flag_show_ts_diff;

static int safe_memcpy(void *dst, void *src, int len, void *dst_end)
{
	int rest_size = (int)dst_end - (int)dst;

	if (rest_size < len) {
		return -1;
	}

	memcpy(dst, src, len);

	return 0;
}

int rtp_opus_send_unicast(struct stream_context *stream_ctx, struct rtp_object *pObject)
{
	struct rtsp_context *rtsp_ctx = stream_ctx->parent;
	int cast_mode = rtsp_ctx->transport[stream_ctx->index].castMode;

	if (stream_ctx->periodic_report.timer1 == 0) {
		stream_ctx->periodic_report.timer1 = rtw_get_current_time();
	}

	rtp_hdr_t *rtphdr;
	int socket;
	int ret;
	uint8_t packet_buf[WRITE_SIZE];
	uint8_t *ptr, *rtp_hdr_pos;
	uint8_t *ptr_end = packet_buf + WRITE_SIZE;
	//int bytes_left = pObject->len;
	pObject->rtphdr->seq = htons(rtsp_ctx->rtpseq[stream_ctx->index]);
	int j, retry_cnt, header_len, data_len;
	uint32_t _write_size;

	uint8_t *data_entry = pObject->data;

	// for UDP
	struct sockaddr_in adr_cs;
	int len_cs = 0;

	socket = pObject->connect_ctx.socket_id;

	if (cast_mode == UNICAST_UDP_MODE) {
		//remote socket info
		adr_cs.sin_family = AF_INET;
		adr_cs.sin_addr.s_addr = *(uint32_t *)pObject->connect_ctx.remote_ip;
		adr_cs.sin_port = htons(pObject->connect_ctx.remote_port);
		len_cs = sizeof(adr_cs);
		rtp_hdr_pos = packet_buf;
		_write_size = WRITE_SIZE;
	} else { // UNICAST_TCP_MODE
		packet_buf[0] = '$';
		packet_buf[1] = rtsp_ctx->transport[stream_ctx->index].interleaved_low; // channel ID in SETUP
		rtp_hdr_pos = packet_buf + 4;
		_write_size = WRITE_SIZE - 4; // 4 bytes for interleaved header
	}

	ptr = (cast_mode == UNICAST_TCP_MODE) ? (packet_buf + 4) : packet_buf;

	if (pObject->rtphdr->cc > 0) { //has CSRC
		if (safe_memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ + 4 * pObject->rtphdr->cc, ptr_end) < 0) {
			RTP_DBG_ERROR("[OPUS] OVERRUN1\n\r");
			return -EAGAIN;
		}
		//memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ + 4 * pObject->rtphdr->cc);
		rtphdr = (rtp_hdr_t *)ptr;
		ptr += RTP_HDR_SZ + 4 * pObject->rtphdr->cc;
	} else { //no CSRC
		if (safe_memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ, ptr_end) < 0) {
			RTP_DBG_ERROR("[OPUS] OVERRUN2\n\r");
			return -EAGAIN;
		}
		//memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ); //prepare header
		rtphdr = (rtp_hdr_t *)ptr;
		ptr += RTP_HDR_SZ;
	}

	header_len = ptr - rtp_hdr_pos;
	data_len = pObject->len;

	if (safe_memcpy(ptr, data_entry, data_len, ptr_end) < 0) {
		RTP_DBG_ERROR("[OPUS] OVERRUN3\n\r");
		return -EAGAIN;
	}
	//memcpy(ptr, data_entry, data_len); //prepare data

	if (cast_mode == UNICAST_TCP_MODE) {
		// fill lenght in interleaved header
		uint16_t rtp_len = header_len + data_len;
		uint16_t *len_ptr = (uint16_t *)(packet_buf + 2);
		*len_ptr = htons(rtp_len);
	}

	// send
	if (cast_mode == UNICAST_TCP_MODE) {
		rtw_mutex_get(&rtsp_ctx->socket_lock);
		ret = write(socket, packet_buf, 4 + header_len + data_len);	// 4 bytes for interleaved header
		rtw_mutex_put(&rtsp_ctx->socket_lock);
	} else { //cast_mode == UNICAST_UDP_MODE
		while (skbdata_used_num > (max_skb_buf_num - 3)) {
			vTaskDelay(1);
		}
		retry_cnt = stream_ctx->framecontrol.packet_retry;
		ret = sendto(socket, packet_buf, header_len + data_len, 0, (struct sockaddr *)&adr_cs, len_cs);
		if (ret < 0) {
			j = 1;
			do {
				while (skbdata_used_num > (max_skb_buf_num - 3)) {
					vTaskDelay(1);
				}
				//retry
				ret = sendto(socket, packet_buf, header_len + data_len, 0, (struct sockaddr *)&adr_cs, len_cs);
				j++;
				retry_cnt--;
			} while ((ret < 0) && (retry_cnt > 0));
		}
	}
	if (ret < 0) {
		RTP_DBG_ERROR("[OPUS] RTP send fail\n\r");
		return -EAGAIN;
	}
	stream_ctx->periodic_report.bytes += (header_len + data_len);
	rtsp_ctx->rtpseq[stream_ctx->index]++;
	rtphdr->seq = htons(rtsp_ctx->rtpseq[stream_ctx->index]);
	stream_ctx->periodic_report.timer2 = rtw_get_current_time();
	stream_ctx->periodic_report.send_frame++;
	if ((stream_ctx->periodic_report.timer2 - stream_ctx->periodic_report.timer1) >= stream_ctx->periodic_report.period) {
		rtp_report(stream_ctx);
	}
	return 0;
}

int rtp_o_opus_handler(struct stream_context *stream_ctx, struct rtp_object *payload)
{
	struct rtsp_context *rtsp_ctx = stream_ctx->parent;
	uint32_t temp_timer = 0;
	int i, stream_nb = 0;
	int ret;

	uint32_t frame_tick_cnt = payload->timestamp;
	// convert mm_read_mediatime_ms to timestamp here
	// no matter the sample rate, the time stamp of opus is fixed as creasing 960
	// stream_ctx->use_rtp_tick_inc = 1;
	payload->timestamp = rtsp_get_timestamp(stream_ctx, frame_tick_cnt);
	if (flag_show_ts_diff) {
		printf("[G][%d] ts += %d\n\r", rtsp_ctx->id, payload->timestamp - stream_ctx->periodic_report.last_timestamp);
	}
	stream_ctx->periodic_report.last_timestamp = payload->timestamp;

	if (stream_ctx->statistics.do_start_check) {
		for (i = 0; i < rtsp_ctx->nb_streams; i ++) {
			if (rtsp_ctx->stream_ctx[i].stream_id >= 0) {
				stream_nb ++;
			}
		}
		/* initialize rtp object header part */
		if (payload->rtphdr != NULL) {
			free(payload->rtphdr);
		}
		payload->rtphdr = malloc(RTP_HDR_SZ + sizeof(uint32_t) * stream_nb);
		if (payload->rtphdr == NULL) {
			OPUS_ERROR("\n\rallocate rtp header failed");
			return -ENOMEM;
		}
	}
	//stream_ctx->statistics.rtp_tick += stream_ctx->statistics.rtp_tick_inc;
	//rtp_fill_header(payload->rtphdr, 2, 0, 0, 0, 0, stream_ctx->codec->pt, rtsp_ctx->rtpseq, stream_ctx->statistics.rtp_tick, stream_ctx->stream_id);
	rtp_fill_header(payload->rtphdr, 2, 0, 0, 0, 0, stream_ctx->codec->pt + stream_ctx->stream_id, rtsp_ctx->rtpseq[stream_ctx->index], payload->timestamp,
					stream_ctx->stream_id);
	if (stream_ctx->statistics.do_start_check) {
		stream_ctx->statistics.delta_timer = 0;
		stream_ctx->statistics.timer2 = rtw_get_current_time();
	}
	stream_ctx->statistics.timer1 = rtw_get_current_time();
	uint32_t current_tick = mm_read_mediatime_ms() + stream_ctx->time_offset;
	uint32_t tick_diff = (current_tick > frame_tick_cnt) ? current_tick - frame_tick_cnt : 0;
	/*
	if (!stream_ctx->framecontrol.drop_frame_enable) {
		goto send_payload;
	}
	*/

	if ((!stream_ctx->framecontrol.drop_frame_enable) || (tick_diff <= stream_ctx->framecontrol.rtp_drop_threshold)) {
//send_payload:
		/* sending rtp payload */
		switch (rtsp_ctx->transport[stream_ctx->index].castMode) {
		case (UNICAST_UDP_MODE):
		case (UNICAST_TCP_MODE):
			ret = rtp_opus_send_unicast(stream_ctx, payload);
			break;
		case (MULTICAST_MODE):
			//to add...
			//break;
			return -EINVAL;
		default:
			return -EINVAL;
		}
		/* END of sending rtp payload */
		temp_timer = rtw_get_current_time();
		stream_ctx->statistics.delta_timer = temp_timer - stream_ctx->statistics.timer2;
		//printf("\n\r%dms-%dms", stream_ctx->statistics.delta_timer, (stream_ctx->statistics.timer1-stream_ctx->statistics.timer2));
#if 0
		if (stream_ctx->statistics.delta_timer < stream_ctx->statistics.delay_threshold) {
			vTaskDelay(stream_ctx->statistics.delay_threshold - stream_ctx->statistics.delta_timer);
			//printf("\n\r-");
		}
#endif
		stream_ctx->statistics.timer2 = rtw_get_current_time();
	} else {
		stream_ctx->periodic_report.drop_frame++;
#if 0
		if (tick_diff >= rtp_drop_threshold) {
			RTP_DBG_INFO("[%d][G] Drop %d ms old data\n\r", rtsp_ctx->id, tick_diff);
		} else {
			RTP_DBG_INFO("[%d][G] Delay %d ms\n\r", rtsp_ctx->id,
						 ((stream_ctx->statistics.timer1 - stream_ctx->statistics.timer2) - stream_ctx->statistics.delay_threshold));
		}
#endif
		temp_timer = rtw_get_current_time();
		stream_ctx->statistics.delta_timer = temp_timer - stream_ctx->statistics.timer2;
		stream_ctx->statistics.timer2 = temp_timer;
		//stream_ctx->statistics.do_start_check = 1;
		rtp_object_set_fs(payload, 0);
		rtp_object_set_fe(payload, 0);
		rtp_object_set_fk(payload, 0);
		rtp_object_set_fd(payload, 0);
		payload->data = NULL;
		payload->len = 0;
		return -EAGAIN;
	}
	//successfully sent packet out, set do start check to 0
	stream_ctx->statistics.do_start_check = 0;
	payload->data = NULL;
	payload->len = 0;
	return ret;
}
