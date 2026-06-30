
#include "FreeRTOS.h"
#include <platform_stdlib.h>
#include "platform_opts.h"

#include "rtsp/rtsp_api.h"
#if defined(CONFIG_PLATFORM_8195BHP) || defined(CONFIG_PLATFORM_8721D) || defined(CONFIG_PLATFORM_8735B)
#include "mmf2_dbg.h"
#include "mmf2_mediatime_8735b.h"
#endif
#include "aac/aac.h"

#include "lwipconf.h"

#define RTP_MAX_AU_NUM 1
#define WRITE_SIZE 1460
extern int max_skb_buf_num;
extern int skbdata_used_num;
extern unsigned char aac_flag_adjust;
//extern uint32_t rtp_drop_threshold;
extern uint8_t flag_show_ts_diff;

static void parse_aac_header(uint8_t *au, int len, struct rtp_aac_obj *aac_obj)
{
	int au_cnt = 0;
	struct rtp_au_hdr *au_hdr = (struct rtp_au_hdr *)(aac_obj + 1);
	uint8_t *ptr = au;
	uint8_t *end = au + len;

	while (ptr < end) {
		if ((*ptr == 0xff && (*(ptr + 1) >> 4) == 0x0f)) { //syncword 0xfff
			if (au_cnt >= RTP_MAX_AU_NUM) {
				aac_obj->au_header_num = RTP_MAX_AU_NUM;
				aac_obj->au_headers_len = 2 * RTP_MAX_AU_NUM;
				return;
			}
			unsigned char *pptr = ptr;
			au_cnt++;
			ptr++;
			au_hdr->au_header_len = (*ptr & 0x01) ? 7 : 9; //does it have 2 byte CRC?
			ptr += 2;
			au_hdr->au_size = ((*ptr & 0x03) << 11) | (*(ptr + 1) << 3) | ((*(ptr + 2) & 0xe0) >> 5); //big endian
			ptr = pptr + au_hdr->au_size - 1;
			au_hdr ++;
		}
		ptr++;
	}
	aac_obj->au_header_num = au_cnt;
	aac_obj->au_headers_len = au_cnt * 2;
}

static int safe_memcpy(void *dst, void *src, int len, void *dst_end)
{
	int rest_size = (int)dst_end - (int)dst;

	if (rest_size < len) {
		return -1;
	}

	memcpy(dst, src, len);

	return 0;
}

int rtp_aac_send_unicast(struct stream_context *stream_ctx, struct rtp_object *pObject)
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
	int bytes_left = pObject->len;
	pObject->rtphdr->seq = htons(rtsp_ctx->rtpseq[stream_ctx->index]);
	int i, j, retry_cnt, header_len, data_len, offset = 0;
	uint32_t _write_size;

	struct rtp_aac_obj *aac_obj = (struct rtp_aac_obj *)pObject->extra;
	struct rtp_au_hdr *au_hdr = (struct rtp_au_hdr *)(aac_obj + 1); //sizeof(*aac_obj));

	uint8_t *temp;
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
			RTP_DBG_ERROR("[AAC] OVERRUN1\n\r");
			return -EAGAIN;
		}

		//memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ + 4 * pObject->rtphdr->cc);
		rtphdr = (rtp_hdr_t *)ptr;
		ptr += RTP_HDR_SZ + 4 * pObject->rtphdr->cc;
	} else { //no CSRC
		if (safe_memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ, ptr_end) < 0) {
			RTP_DBG_ERROR("[AAC] OVERRUN2\n\r");
			return -EAGAIN;
		}

		//memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ); //prepare header
		rtphdr = (rtp_hdr_t *)ptr;
		ptr += RTP_HDR_SZ;
	}

	header_len = ptr - rtp_hdr_pos;

	while (bytes_left > 0) {
		if (offset == 0) {
			header_len += (2 + aac_obj->au_headers_len);
			data_len = _write_size - header_len;
			temp = ptr;
			*temp = (aac_obj->au_headers_len * 8) >> 8;
			*(temp + 1) = (aac_obj->au_headers_len * 8) & 0x00ff;
			temp += 2;
			for (i = 0; i < aac_obj->au_header_num; i++) {
				uint32_t datasize = au_hdr->au_size - au_hdr->au_header_len;
				//translate in bits
				*temp = (datasize * 8) >> 8;
				*(temp + 1) = (datasize * 8) & 0x00ff;
				au_hdr++;
				temp += 2;
			}
			au_hdr = (struct rtp_au_hdr *)(aac_obj + 1);//sizeof(*aac_obj));
			offset += au_hdr->au_header_len;
			bytes_left -= au_hdr->au_header_len;
		} else {
			header_len = ptr - rtp_hdr_pos;
			data_len = _write_size - header_len;
		}
//edit by Ian -- This is only workable with one AU. Will replace ...
#if 1
		if (bytes_left <= data_len) {
			data_len = bytes_left;
			rtphdr->m = 1;
		}

		if (safe_memcpy(rtp_hdr_pos + header_len, data_entry + offset, data_len, ptr_end) < 0) {
			RTP_DBG_ERROR("[AAC] OVERRUN3\n\r");
			return -EAGAIN;
		}

		//memcpy(rtp_hdr_pos + header_len, data_entry + offset, data_len);

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
check_skb:
			if (skbdata_used_num > (max_skb_buf_num - 3)) {
				vTaskDelay(1);
				goto check_skb;
			} else {
				retry_cnt = stream_ctx->framecontrol.packet_retry;
				ret = sendto(socket, packet_buf, header_len + data_len, 0, (struct sockaddr *)&adr_cs, len_cs);
				if (ret < 0) {
					//j = 1;
					do {
						//rtw_mdelay_os(j);
						//printf("\n\r+");
						//retry
						while (skbdata_used_num > (max_skb_buf_num - 3)) {
							vTaskDelay(1);
						}
						ret = sendto(socket, packet_buf, header_len + data_len, 0, (struct sockaddr *)&adr_cs, len_cs);
						//j++;
						retry_cnt--;
					} while ((ret < 0) && (retry_cnt > 0));
				}
			}
		}
		if (ret < 0) {
			RTP_DBG_ERROR("[AAC] RTP send fail\n\r");
			return -EAGAIN;
		}
		offset += data_len;
		bytes_left -= data_len;
#endif
		stream_ctx->periodic_report.bytes += (header_len + data_len);
		rtsp_ctx->rtpseq[stream_ctx->index]++;
		rtphdr->seq = htons(rtsp_ctx->rtpseq[stream_ctx->index]);
	}
	stream_ctx->periodic_report.timer2 = rtw_get_current_time();
	stream_ctx->periodic_report.send_frame++;
	if ((stream_ctx->periodic_report.timer2 - stream_ctx->periodic_report.timer1) >= stream_ctx->periodic_report.period) {
		rtp_report(stream_ctx);
	}
	return 0;
}

int rtp_o_aac_handler(struct stream_context *stream_ctx, struct rtp_object *payload)
{
	struct rtsp_context *rtsp_ctx = stream_ctx->parent;
	struct rtp_aac_obj *aac_obj;
	uint32_t temp_timer = 0;
	int i, ret, stream_nb = 0;

	uint32_t frame_tick_cnt = payload->timestamp;
	// convert mm_read_mediatime_ms to timestamp here
	payload->timestamp = rtsp_get_timestamp(stream_ctx, frame_tick_cnt);
	if (flag_show_ts_diff) {
		printf("[A][%d] ts += %d\n\r", rtsp_ctx->id, payload->timestamp - stream_ctx->periodic_report.last_timestamp);
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
		if (payload->extra != NULL) {
			free(payload->extra);
		}
		payload->rtphdr = malloc(RTP_HDR_SZ + sizeof(uint32_t) * stream_nb);
		if (payload->rtphdr == NULL) {
			AAC_ERROR("\n\rallocate rtp header failed");
			return -ENOMEM;
		}
		payload->extra = malloc(sizeof(struct rtp_aac_obj) + sizeof(struct rtp_au_hdr) * RTP_MAX_AU_NUM);
		if (payload->extra == NULL) {
			AAC_ERROR("\n\rallocate rtp extra memory failed");
			free(payload->rtphdr);
			return -ENOMEM;
		}

	}
	//stream_ctx->statistics.rtp_tick += stream_ctx->statistics.rtp_tick_inc;
	//rtp_fill_header(payload->rtphdr, 2, 0, 0, 0, 0, stream_ctx->codec->pt + stream_ctx->stream_id, rtsp_ctx->rtpseq, stream_ctx->statistics.rtp_tick, stream_ctx->stream_id);
	rtp_fill_header(payload->rtphdr, 2, 0, 0, 0, 0, stream_ctx->codec->pt + stream_ctx->stream_id, rtsp_ctx->rtpseq[stream_ctx->index], payload->timestamp,
					stream_ctx->stream_id);
	aac_obj = (struct rtp_aac_obj *)payload->extra;
	parse_aac_header(payload->data, payload->len, aac_obj);

	if (stream_ctx->statistics.do_start_check) {
		stream_ctx->statistics.delta_timer = 0;
		stream_ctx->statistics.timer2 = rtw_get_current_time();
	}
	stream_ctx->statistics.timer1 = rtw_get_current_time();
	uint32_t current_tick = mm_read_mediatime_ms() + stream_ctx->time_offset;
	uint32_t tick_diff = (current_tick > frame_tick_cnt) ? current_tick - frame_tick_cnt : 0;

	if (!stream_ctx->framecontrol.drop_frame_enable) {
		goto send_payload;
	}

	if (/*((stream_ctx->statistics.timer1-stream_ctx->statistics.timer2) < stream_ctx->statistics.delay_threshold * 3)
	    &&*/ (tick_diff <= stream_ctx->framecontrol.rtp_drop_threshold)) {
send_payload:
		/* sending rtp payload */
		switch (rtsp_ctx->transport[stream_ctx->index].castMode) {
		case (UNICAST_UDP_MODE):
		case (UNICAST_TCP_MODE):
			ret = rtp_aac_send_unicast(stream_ctx, payload);
			break;
		case (MULTICAST_MODE):
			//to add..., remove error return after implement
			return -EINVAL;
			break;
		default:
			return -EINVAL;
		}
		/* END of sending rtp payload */
		temp_timer = rtw_get_current_time();
		stream_ctx->statistics.delta_timer = temp_timer - stream_ctx->statistics.timer2;
		//printf("\n\r%dms-%dms", stream_ctx->statistics.delta_timer, (stream_ctx->statistics.timer1-stream_ctx->statistics.timer2));
#if 0
		if (aac_flag_adjust) {
			if (stream_ctx->statistics.delta_timer < stream_ctx->statistics.delay_threshold) {
				vTaskDelay(stream_ctx->statistics.delay_threshold - stream_ctx->statistics.delta_timer);
				//printf("\n\r-");
			}
		}
#endif
		stream_ctx->statistics.timer2 = rtw_get_current_time();
	} else {
		stream_ctx->periodic_report.drop_frame++;
#if 0
		if (tick_diff >= rtp_drop_threshold) {
			RTP_DBG_INFO("[%d][A] Drop %d ms old data\n\r", rtsp_ctx->id, tick_diff);
		} else {
			RTP_DBG_INFO("[%d][A] Delay %d ms\n\r", rtsp_ctx->id,
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
