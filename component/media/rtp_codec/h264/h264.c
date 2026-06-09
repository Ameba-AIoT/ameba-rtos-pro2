#include "FreeRTOS.h"
#include <platform_stdlib.h>
#include "platform_opts.h"

#include "rtsp/rtsp_api.h"
//#include "mmf_dbg.h"
#include "mmf2_dbg.h"
#include "mmf2_mediatime_8735b.h"
#include "h264/h264.h"
#include "lwipconf.h"

#define WRITE_SIZE 1460

extern int max_skb_buf_num;
extern int skbdata_used_num;

int dropP = 0;

char base64_sps[128];
char base64_pps[32];
char plid[7]; //3 bytes value in 6 byte string
//return nal start code len.
extern unsigned char h264_flag_adjust;

//extern uint32_t rtp_drop_threshold;
extern uint8_t flag_show_ts_diff;

static int h264_is_i_frame(u8 *frame_buf)
{
	if ((frame_buf[4] & 0x1F) == 7) {
		return 1;
	} else {
		return 0;
	}
}


static int nal_start_code_len(uint8_t *nal)
{
	int len = 0;
	if (nal[0] == 0 && nal[1] == 0) {
		if (nal[2] == 1) {
			len = 3;
		} else if (nal[2] == 0 && nal[3] == 1) {
			len = 4;
		} else {
			len = 0;
		}
	}
	return len;
}

static void parse_h264_header(uint8_t *h264_data, int len, struct rtp_h264_obj *h264_obj)
{
	uint8_t *ptr = h264_data;
	int start_code_len = 0;
	int count = 0;
	while (ptr < h264_data + len - 3) {
		if (ptr[0] == 0 && ptr[1] == 0) {
			if (ptr[2] == 0 || ptr[2] == 1) {
				//NAL FOUND
				h264_obj->num_nal++;
				if (h264_obj->num_nal > MAX_NUM_NAL_PER_FRM) {
					H264_ERROR("\n\rtoo many NALs!");
					return;
				}
				start_code_len = nal_start_code_len(ptr);
				h264_obj->nal_obj[h264_obj->num_nal - 1].start_code_len = start_code_len;
				h264_obj->nal_obj[h264_obj->num_nal - 1].offset = ptr - h264_data;
				h264_obj->nal_obj[h264_obj->num_nal - 1].nal_header = ptr[start_code_len];

				if ((h264_obj->nal_obj[h264_obj->num_nal - 1].nal_header & 0x60) == 96) {
					h264_obj->nal_obj[h264_obj->num_nal - 1].must_not_drop = 1;
				} else {
					h264_obj->nal_obj[h264_obj->num_nal - 1].must_not_drop = 0;
				}
				//if(h264_obj->num_nal == 0x02)
				//goto EXIT;

				//h264_obj->nal_obj[h264_obj->num_nal-1].must_not_drop = 1;
				switch (h264_obj->nal_obj[h264_obj->num_nal - 1].nal_header & 0x1f) {
				case (1):
				case (2):
				case (3):
				case (4):
				case (5):
				case (6):
				case (10):
				case (11):
				case (12):
				case (7):
				case (8):
					h264_obj->nal_obj[h264_obj->num_nal - 1].do_not_send = 0;
					return;
					//goto EXIT;
					break;
				case (9):
				default:
					h264_obj->nal_obj[h264_obj->num_nal - 1].do_not_send = 1;
					break;
				}
				ptr += start_code_len; //skip start code
			}
		}
		ptr++;
		count++;
	}
//EXIT:
	//printf("count = %d\n",count);
	//printf("start_code_len = %d offset = %d header = %x num = %d\n",h264_obj->nal_obj[h264_obj->num_nal-1].start_code_len,h264_obj->nal_obj[h264_obj->num_nal-1].offset,h264_obj->nal_obj[h264_obj->num_nal-1].nal_header,h264_obj->num_nal);
	return;
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

int rtp_h264_send_unicast(struct stream_context *stream_ctx, struct rtp_object *pObject)
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
	int bytes_left;
	pObject->rtphdr->seq = htons(rtsp_ctx->rtpseq[stream_ctx->index]);
	int i, j, retry_cnt, header_len, data_len, offset = 0;
	uint32_t _write_size;

	struct rtp_h264_obj *h264_obj = (struct rtp_h264_obj *)pObject->extra;
	int nal_bytes_left;
	int use_fu = FALSE;

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

	bytes_left = (pObject->fd) ? pObject->fd : pObject->len;

	if (pObject->rtphdr->cc > 0) { //has CSRC
		if (safe_memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ + 4 * pObject->rtphdr->cc, ptr_end) < 0) {
			RTP_DBG_ERROR("[H264] OVERRUN1\n\r");
			return -EAGAIN;
		}
		//memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ + 4 * pObject->rtphdr->cc);
		rtphdr = (rtp_hdr_t *)ptr;
		ptr += RTP_HDR_SZ + 4 * pObject->rtphdr->cc;
	} else { //no CSRC
		if (safe_memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ, ptr_end) < 0) {
			RTP_DBG_ERROR("[H264] OVERRUN2\n\r");
			return -EAGAIN;
		}
		//memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ); //prepare header
		rtphdr = (rtp_hdr_t *)ptr;
		ptr += RTP_HDR_SZ;
	}

	header_len = ptr - rtp_hdr_pos;

	for (i = 0; i < h264_obj->num_nal; i++) {
		if (h264_obj->nal_obj[i].do_not_send) {
			continue;
		}
		rtphdr->m = 0;
		offset = 0;
		use_fu = FALSE;
		if ((i + 1) >= h264_obj->num_nal) {
			nal_bytes_left = bytes_left - h264_obj->nal_obj[i].offset;
		} else {
			nal_bytes_left = h264_obj->nal_obj[i + 1].offset - h264_obj->nal_obj[i].offset;
		}
		if (pObject->fs) {
			if (h264_obj->nal_obj[i].start_code_len > 0 && h264_obj->nal_obj[i].nal_header) {
				offset = h264_obj->nal_obj[i].start_code_len + 1;    // skip starting sequence and NAL header
			}
			nal_bytes_left -= offset;
			h264_obj->nal_obj[i].is_fu_start = TRUE;
			h264_obj->nal_obj[i].is_fu_end = FALSE;
		}

		// FU enable?
		if ((nal_bytes_left > (_write_size - header_len)) || (!pObject->fs)) {
			use_fu = TRUE;
			header_len += 2;	// append FU header
		} else {
			header_len += 1;	// append NAL header
		}

		while (nal_bytes_left > 0) {

			data_len = _write_size - header_len;
			if (data_len >= nal_bytes_left) {
				data_len = nal_bytes_left;
				if (pObject->fe) {
					rtphdr->m = 1;
					h264_obj->nal_obj[i].is_fu_end = TRUE;
				}
			}

			// fill FU header
			if (use_fu) {
				ptr[0] = (h264_obj->nal_obj[i].nal_header & 0x60) | 28; // FU indicator  FU-A
				ptr[1] = (h264_obj->nal_obj[i].nal_header & 0x1f); // FU header

				if (h264_obj->nal_obj[i].is_fu_start) {
					ptr[1] |= 0x80; // first FU
					h264_obj->nal_obj[i].is_fu_start = FALSE;
				}
				if (h264_obj->nal_obj[i].is_fu_end) {
					ptr[1] |= 0x40; // final FU
					h264_obj->nal_obj[i].is_fu_end = FALSE;
				}
				if (safe_memcpy(ptr + 2, pObject->data + h264_obj->nal_obj[i].offset + offset, data_len, ptr_end) < 0) {
					RTP_DBG_ERROR("[H264] OVERRUN3\n\r");
					return -EAGAIN;
				}
				//memcpy(ptr + 2, pObject->data + h264_obj->nal_obj[i].offset + offset, data_len);
			} else {
				ptr[0] = h264_obj->nal_obj[i].nal_header;
				if (safe_memcpy(ptr + 1, pObject->data + h264_obj->nal_obj[i].offset + offset, data_len, ptr_end) < 0) {
					RTP_DBG_ERROR("[H264] OVERRUN4\n\r");
					return -EAGAIN;
				}
				//memcpy(ptr + 1, pObject->data + h264_obj->nal_obj[i].offset + offset, data_len);
			}

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
						do {
							//retry
							while (skbdata_used_num > (max_skb_buf_num - 3)) {
								vTaskDelay(1);
							}
							ret = sendto(socket, packet_buf, header_len + data_len, 0, (struct sockaddr *)&adr_cs, len_cs);
							if (!h264_obj->nal_obj[i].must_not_drop) {
								retry_cnt--;
							}
						} while (((ret < 0) && (pObject->fk)) || ((ret < 0) && (retry_cnt > 0)));
					}
				}
			}
			if (ret < 0) {
				RTP_DBG_ERROR("[H264] RTP send fail\n\r");
				return -EAGAIN;
			}
			offset += data_len;
			nal_bytes_left -= data_len;
			stream_ctx->periodic_report.bytes += (header_len + data_len);
			rtsp_ctx->rtpseq[stream_ctx->index]++;
			rtphdr->seq = htons(rtsp_ctx->rtpseq[stream_ctx->index]);
		}
	}
	stream_ctx->periodic_report.timer2 = rtw_get_current_time();
	stream_ctx->periodic_report.send_frame++;
	if ((stream_ctx->periodic_report.timer2 - stream_ctx->periodic_report.timer1) >= stream_ctx->periodic_report.period) {
		rtp_report(stream_ctx);
	}
	return 0;
}

int rtp_o_h264_handler(struct stream_context *stream_ctx, struct rtp_object *payload)
{
	struct rtsp_context *rtsp_ctx = stream_ctx->parent;
	struct rtp_h264_obj *h264_obj;
	uint32_t temp_timer = 0;
	int ret;

	uint32_t frame_tick_cnt = payload->timestamp;
	// convert mm_read_mediatime_ms to timestamp here
	payload->timestamp = rtsp_get_timestamp(stream_ctx, frame_tick_cnt);
	if (flag_show_ts_diff) {
		printf("[H][%d] ts += %d\n\r", rtsp_ctx->id, payload->timestamp - stream_ctx->periodic_report.last_timestamp);
	}
	stream_ctx->periodic_report.last_timestamp = payload->timestamp;

	/* if using fragmentation service */
	if (payload->fs) {
		/* calculate mux stream numbers */
		int i, stream_nb = 0;
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
				RTP_DBG_ERROR("\n\rallocate rtp header failed");
				return -ENOMEM;
			}
			payload->extra = malloc(sizeof(struct rtp_h264_obj));
			if (payload->extra == NULL) {
				RTP_DBG_ERROR("\n\rallocate rtp extra memory failed");
				return -ENOMEM;
			}
		}
		//stream_ctx->statistics.rtp_tick += stream_ctx->statistics.rtp_tick_inc;
		//rtp_fill_header(payload->rtphdr, 2, 0, 0, 0, 0, stream_ctx->codec->pt + stream_ctx->stream_id, rtsp_ctx->rtpseq, stream_ctx->statistics.rtp_tick, stream_ctx->stream_id);
		rtp_fill_header(payload->rtphdr, 2, 0, 0, 0, 0, stream_ctx->codec->pt + stream_ctx->stream_id, rtsp_ctx->rtpseq[stream_ctx->index], payload->timestamp,
						stream_ctx->stream_id);
		h264_obj = (struct rtp_h264_obj *)payload->extra;
		memset(h264_obj, 0, sizeof(struct rtp_h264_obj)); //clear h264_obj
		parse_h264_header(payload->data, payload->len, h264_obj);
	} else {
		if (stream_ctx->statistics.do_start_check) {
			rtp_object_set_fs(payload, 0);
			rtp_object_set_fe(payload, 0);
			rtp_object_set_fk(payload, 0);
			rtp_object_set_fd(payload, 0);
			payload->data = NULL;
			payload->len = 0;
			return -EAGAIN;
		}
	}
	if (stream_ctx->statistics.do_start_check) {
		stream_ctx->statistics.delta_timer = 0;
		stream_ctx->statistics.timer2 = rtw_get_current_time();
	}
	stream_ctx->statistics.timer1 = rtw_get_current_time();
	uint32_t current_tick = mm_read_mediatime_ms() + stream_ctx->time_offset;
	uint32_t tick_diff = (current_tick > frame_tick_cnt) ? current_tick - frame_tick_cnt : 0;

#if 0
	//change rate control
	if (stream_ctx->framecontrol.h264_change_rate_control) {
		if (current_tick - stream_ctx->framecontrol.change_rate_timer > stream_ctx->framecontrol.change_rate_frequency) {
			stream_ctx->framecontrol.change_rate_timer =  current_tick;
			//if(stream_ctx->framecontrol.drop_frame_count > stream_ctx->framecontrol.change_rate_threshold)
			if (stream_ctx->framecontrol.send_frame_count < stream_ctx->framecontrol.change_rate_threshold1) {
				if (stream_ctx->framecontrol.current_bps == 0
					|| stream_ctx->framecontrol.current_bps == stream_ctx->bitrate) {
					h264_change_bitrate(stream_ctx->framecontrol.h264_ctx, stream_ctx->bitrate / 2);
					stream_ctx->framecontrol.current_bps = stream_ctx->bitrate / 2;
				}
			}
			//else if(stream_ctx->framecontrol.drop_frame_count == 0)
			else if (stream_ctx->framecontrol.send_frame_count > stream_ctx->framecontrol.change_rate_threshold2) {
				if (stream_ctx->framecontrol.current_bps == stream_ctx->bitrate / 2) {
					h264_change_bitrate(stream_ctx->framecontrol.h264_ctx, stream_ctx->bitrate);
					stream_ctx->framecontrol.current_bps = stream_ctx->bitrate;
				}
			}
			stream_ctx->framecontrol.send_frame_count = 0;
		}
	}
#endif

	if (stream_ctx->framecontrol.start_drop_frame == 1) {
		if (h264_is_i_frame((u8 *) payload->data)) {
			printf("drop %d Pframe\r\n", dropP);
			stream_ctx->framecontrol.start_drop_frame = 0;
			dropP = 0;
		} else {
			dropP++;
			goto drop_frame;
		}
	}

	if (!stream_ctx->framecontrol.drop_frame_enable) {
		goto send_payload;
	}

#if defined(CONFIG_PLATFORM_8735B)
	if (1) {
#else
	if (/*((stream_ctx->statistics.timer1-stream_ctx->statistics.timer2) < stream_ctx->statistics.delay_threshold * 3)
	    && */(tick_diff <= stream_ctx->framecontrol.rtp_drop_threshold)) {
#endif
send_payload:
		/* sending rtp payload */
		switch (rtsp_ctx->transport[stream_ctx->index].castMode) {
		case (UNICAST_UDP_MODE):
		case (UNICAST_TCP_MODE):
			ret = rtp_h264_send_unicast(stream_ctx, payload);
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
#if 1
		if (h264_flag_adjust) {
			if ((stream_ctx->statistics.delta_timer < stream_ctx->statistics.delay_threshold) && (payload->fe)) {
				vTaskDelay(stream_ctx->statistics.delay_threshold - stream_ctx->statistics.delta_timer);
				//printf("\n\r-");
			}
		}
#endif
		if (payload->fe) {
			stream_ctx->statistics.timer2 = rtw_get_current_time();
		}
		if ((payload->fs) && (payload->fd) && (ret == 0)) {
			//if using fragmentation and already sent first fragment then keep the rest of this frame sending out
			rtp_object_set_fk(payload, 1);
		}
	} else {
drop_frame:
		stream_ctx->periodic_report.drop_frame++;
		stream_ctx->framecontrol.drop_frame_count++;
		if (stream_ctx->framecontrol.start_drop_frame == 0) {
			stream_ctx->framecontrol.start_drop_frame = 1;

			printf("drop frame = %d\r\n", stream_ctx->periodic_report.drop_frame);
			printf("tick_diff = %d\r\n", tick_diff);
			if (stream_ctx->framecontrol.drop_frame_use_forcei) {
				if (stream_ctx->framecontrol.h264_ctx != NULL) {
#if defined(CONFIG_PLATFORM_8195BHP)
					h264_set_force_iframe(stream_ctx->framecontrol.h264_ctx);
#endif
				}
			}
		}

#if 0
		if (tick_diff >= stream_ctx->framecontrol.rtp_drop_threshold) {
			RTP_DBG_INFO("[%d][H] Drop %d ms old data\n\r", rtsp_ctx->id, tick_diff);
		} else {
			RTP_DBG_INFO("[%d][H] Delay %d ms\n\r", rtsp_ctx->id,
						 ((stream_ctx->statistics.timer1 - stream_ctx->statistics.timer2) - stream_ctx->statistics.delay_threshold));
		}
#endif
		temp_timer = rtw_get_current_time();
		stream_ctx->statistics.delta_timer = temp_timer - stream_ctx->statistics.timer2;
		stream_ctx->statistics.timer2 = temp_timer;
		if (payload->fd) {
			stream_ctx->statistics.do_start_check = 1;
		}

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
