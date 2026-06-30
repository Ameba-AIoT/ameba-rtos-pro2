
#include "FreeRTOS.h"
#include <platform_stdlib.h>
#include "platform_opts.h"

#include "rtsp/rtsp_api.h"
#if defined(CONFIG_PLATFORM_8195BHP) || defined(CONFIG_PLATFORM_8721D) || defined(CONFIG_PLATFORM_8735B)
#include "mmf2_dbg.h"
#include "mmf2_mediatime_8735b.h"
#endif
#include "mjpeg/mjpeg.h"
#include "lwipconf.h"

#define WRITE_SIZE 1460
extern int max_skb_buf_num;
extern int skbdata_used_num;

//extern uint32_t rtp_drop_threshold;
extern uint8_t flag_show_ts_diff;

static void parse_jpeg_header(uint8_t *jpeg_data, int len, int *width, int *height, uint8_t *type, u16 *dri, uint8_t *precision, uint8_t *lqt, uint8_t *cqt,
							  int *hdr_len)
{
	uint8_t *ptr;
	uint8_t *start;
	if (jpeg_data == NULL) {
		printf("\n\rnull jpeg data!\n\r");
		return;
	}
	ptr = start = jpeg_data;
	uint8_t i;
	u16 rec; //header length record

	while ((ptr - start) < len) {
		if (*ptr == 0xff) {
			switch (*(ptr + 1)) {
			case (0xdb):			//parse quantization table
				ptr += 4;
				*precision = (*ptr) >> 4;
				i = (*ptr) & (0x0f);
				ptr ++;
				if (*precision != 0) { //16 bit precision
					if (i == 0) {
						memcpy(lqt, ptr, 128);
					} else {
						memcpy(cqt, ptr, 128);
					}
					ptr += 128;
				} else {		//8 bit precision
					if (i == 0) {
						memcpy(lqt, ptr, 64);
					} else {
						memcpy(cqt, ptr, 64);
					}
					ptr += 64;
				}
				break;
			case (0xc4):		//parse DHT - skip here
				ptr += 2;
				rec = (*ptr << 8) | *(ptr + 1);
				ptr += rec;
				break;
			case (0xc0):		//parse SOF0
				ptr += 5;
				*height = *ptr << 8 | *(ptr + 1);
				ptr += 2;
				*width = *ptr << 8 | *(ptr + 1);
				ptr += 2;
				if (*ptr == 3) { //parse component
					ptr++;
					if (*(ptr + 1) == 0x21) {
						*type = 0;
					} else if (*(ptr + 1) == 0x22) {
						*type = 1;
					}
					ptr += 9;
				}
				break;
			case (0xc1):		//parse SOF1 - skip here
				ptr++;
				break;
			case (0xdd):		//parse dri
				ptr += 4;
				*dri = *ptr << 8 | *(ptr + 1);
				ptr += 2;
				break;
			/*end of parsing condition*/
			case (0xda):		//parse SOS and return
				ptr += 2;
				rec = (*ptr << 8) | *(ptr + 1);
				ptr += rec;
				*hdr_len = ptr - start;
			case (0xd9):
				return;
			default:
				ptr++;
			}
		} else {
			ptr++;
		}
	}
}

static void fillJpegHeader(struct jpeghdr *jpghdr, uint8_t type, uint8_t typespec, int width, int height, u16 dri, uint8_t q)
{
	jpghdr->tspec = typespec;
	jpghdr->type = type | ((dri != 0) ? RTP_JPEG_RESTART : 0);
	jpghdr->q = q;
	jpghdr->width = (uint8_t)(width / 8);
	jpghdr->height = (uint8_t)(height / 8);
}

static void fillRstHeader(struct jpeghdr_rst *rsthdr, u16 dri)
{
	rsthdr->dri = htons(dri);
	if (dri != 0) {
		rsthdr->f = 1;			/* This code does not align RIs */
		rsthdr->l = 1;
		rsthdr->count = 0x3fff;
	}
}

static void fillqtable(struct jpeghdr_qtable *qtable, uint8_t precision)
{
	qtable->mbz = 0;
	qtable->precision = precision;
	if (precision != 0) {
		qtable->length = htons(256); // 2*128 quantization table length in network byte order
	} else {
		qtable->length = htons(128); // 2*64 quantization table length in network byte order
	}
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

int rtp_jpeg_send_unicast(struct stream_context *stream_ctx, struct rtp_object *pObject)
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
	int j, retry_cnt, header_len, data_len, offset = 0;
	uint32_t _write_size;

	struct rtp_jpeg_obj *jpeg_object = (struct rtp_jpeg_obj *)pObject->extra;
	struct jpeghdr *jpghdr;
	uint8_t *tmp;
	int dqt_len;
	uint8_t *data_entry = pObject->data;
	header_len = dqt_len = data_len = offset = 0;

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
			RTP_DBG_ERROR("[JPEG] OVERRUN1\n\r");
			return -EAGAIN;
		}
		//memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ + 4 * pObject->rtphdr->cc);
		rtphdr = (rtp_hdr_t *)ptr;
		ptr += RTP_HDR_SZ + 4 * pObject->rtphdr->cc;
	} else { //no CSRC
		if (safe_memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ, ptr_end) < 0) {
			RTP_DBG_ERROR("[JPEG] OVERRUN2\n\r");
			return -EAGAIN;
		}
		//memcpy(ptr, pObject->rtphdr, RTP_HDR_SZ); //prepare header
		rtphdr = (rtp_hdr_t *)ptr;
		ptr += RTP_HDR_SZ;
	}

	if (safe_memcpy(ptr, &jpeg_object->jpghdr, sizeof(jpeg_object->jpghdr), ptr_end) < 0) {
		RTP_DBG_ERROR("[JPEG] OVERRUN3\n\r");
		return -EAGAIN;
	}
	//memcpy(ptr, &jpeg_object->jpghdr, sizeof(jpeg_object->jpghdr));
	jpghdr = (struct jpeghdr *)ptr;
	ptr += sizeof(jpeg_object->jpghdr);
	if (jpeg_object->frame_offset == 0) {
		jpghdr->off = 0;
	}
	if (jpeg_object->rsthdr.dri > 0) {
		if (safe_memcpy(ptr, &jpeg_object->rsthdr, sizeof(jpeg_object->rsthdr), ptr_end) < 0) {
			RTP_DBG_ERROR("[JPEG] OVERRUN4\n\r");
			return -EAGAIN;
		}
		//memcpy(ptr, &jpeg_object->rsthdr, sizeof(jpeg_object->rsthdr));
		ptr += sizeof(jpeg_object->rsthdr);
	} else {
		//to fix logitech c160 no dri bug
		jpghdr->q = 0;
	}

	header_len = ptr - rtp_hdr_pos;

	while (bytes_left > 0) {
		if (offset == 0) {
			if (jpghdr->q >= 128) {
				//printf("\n\rdri:%d", jpeg_object->rsthdr.dri);
				tmp = ptr;
				if (safe_memcpy(tmp, &jpeg_object->qtable, sizeof(jpeg_object->qtable), ptr_end) < 0) {
					RTP_DBG_ERROR("[JPEG] OVERRUN5\n\r");
					return -EAGAIN;
				}
				//memcpy(tmp, &jpeg_object->qtable, sizeof(jpeg_object->qtable));
				tmp += sizeof(jpeg_object->qtable);
				if (jpeg_object->qtable.precision != 0) {
					if (safe_memcpy(tmp, jpeg_object->lqt, 128, ptr_end) < 0) {
						RTP_DBG_ERROR("[JPEG] OVERRUN6\n\r");
						return -EAGAIN;
					}
					//memcpy(tmp, jpeg_object->lqt, 128);
					tmp += 128;
					if (safe_memcpy(tmp, jpeg_object->cqt, 128, ptr_end) < 0) {
						RTP_DBG_ERROR("[JPEG] OVERRUN7\n\r");
						return -EAGAIN;
					}
					//memcpy(tmp, jpeg_object->cqt, 128);
					tmp += 128;
				} else {
					if (safe_memcpy(tmp, jpeg_object->lqt, 64, ptr_end) < 0) {
						RTP_DBG_ERROR("[JPEG] OVERRUN8\n\r");
						return -EAGAIN;
					}
					//memcpy(tmp, jpeg_object->lqt, 64);
					tmp += 64;
					if (safe_memcpy(tmp, jpeg_object->cqt, 64, ptr_end) < 0) {
						RTP_DBG_ERROR("[JPEG] OVERRUN9\n\r");
						return -EAGAIN;
					}
					//memcpy(tmp, jpeg_object->cqt, 64);
					tmp += 64;
				}
				dqt_len = tmp - ptr;
				header_len += (tmp - ptr);
				data_entry += jpeg_object->hdr_len;
				bytes_left -= jpeg_object->hdr_len;
			} else {
				//jpghdr->q = 0;
				dqt_len = 0;
			}
		} else {
			header_len = ptr - rtp_hdr_pos;
			dqt_len = 0;
		}
		data_len = _write_size - header_len;
		if (data_len >= bytes_left) {
			data_len = bytes_left;
			if (pObject->fe == 1) {
				rtphdr->m = 1;
			}
		}

		if (safe_memcpy(ptr + dqt_len, data_entry + offset, data_len, ptr_end) < 0) {
			RTP_DBG_ERROR("[JPEG] OVERRUN10\n\r");
			return -EAGAIN;
		}
		//memcpy(ptr + dqt_len, data_entry + offset, data_len);

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
					j = 1;
					do {
						while (skbdata_used_num > (max_skb_buf_num - 3)) {
							vTaskDelay(1);
						}
						//retry
						ret = sendto(socket, packet_buf, header_len + data_len, 0, (struct sockaddr *)&adr_cs, len_cs);
						j++;
						retry_cnt--;
					} while (((ret < 0) && (pObject->fk)) || ((ret < 0) && (retry_cnt > 0)));
				}
			}
		}
		if (ret < 0) {
			RTP_DBG_ERROR("[JPEG] RTP write fail\n\r");
			return -EAGAIN;
		}
		offset += data_len;
		jpeg_object->frame_offset += data_len;
		//change offset to network byte order
		jpghdr->off = (((jpeg_object->frame_offset & 0xff) << 16) | (jpeg_object->frame_offset & 0xff00) | ((jpeg_object->frame_offset & 0xff0000UL) >> 16));
		bytes_left -= data_len;
		stream_ctx->periodic_report.bytes += (header_len + data_len);
		rtsp_ctx->rtpseq[stream_ctx->index]++;
		rtphdr->seq = htons(rtsp_ctx->rtpseq[stream_ctx->index]);
		/*for debugging message*/
		//dumpRtpHeader(rtphdr);
		//dumpJpegHeader(jpghdr);
		//dumpRstDeader(&jpeg_object->rsthdr);
	}
	stream_ctx->periodic_report.timer2 = rtw_get_current_time();
	stream_ctx->periodic_report.send_frame++;
	if ((stream_ctx->periodic_report.timer2 - stream_ctx->periodic_report.timer1) >= stream_ctx->periodic_report.period) {
		rtp_report(stream_ctx);
	}
	return 0;
}

int rtp_o_mjpeg_handler(struct stream_context *stream_ctx, struct rtp_object *payload)
{
	struct rtsp_context *rtsp_ctx = stream_ctx->parent;
	struct rtp_jpeg_obj *jpeg_obj;
	uint32_t temp_timer = 0;
	int ret;

	uint32_t frame_tick_cnt = payload->timestamp;
	// convert mm_read_mediatime_ms to timestamp here
	payload->timestamp = rtsp_get_timestamp(stream_ctx, frame_tick_cnt);
	if (flag_show_ts_diff) {
		printf("[M][%d] ts += %d\n\r", rtsp_ctx->id, payload->timestamp - stream_ctx->periodic_report.last_timestamp);
	}
	stream_ctx->periodic_report.last_timestamp = payload->timestamp;

	/* if using fragmentation service */
	if (payload->fs) {
		/* calculate mux stream numbers */
		int i, stream_nb = 0;
		uint8_t type = 0;
		uint8_t precision = 0;
		u16 dri = 0;
		int rtp_width = 0;
		int rtp_height = 0;
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
				MJPEG_ERROR("\n\rallocate rtp header failed");
				return -ENOMEM;
			}
			payload->extra = malloc(sizeof(struct rtp_jpeg_obj));
			if (payload->extra == NULL) {
				MJPEG_ERROR("\n\rallocate rtp extra memory failed");
				return -ENOMEM;
			}
		}
		//stream_ctx->statistics.rtp_tick += stream_ctx->statistics.rtp_tick_inc;
		//rtp_fill_header(payload->rtphdr, 2, 0, 0, 0, 0, stream_ctx->codec->pt, rtsp_ctx->rtpseq, stream_ctx->statistics.rtp_tick, stream_ctx->stream_id);
		rtp_fill_header(payload->rtphdr, 2, 0, 0, 0, 0, stream_ctx->codec->pt, rtsp_ctx->rtpseq[stream_ctx->index], payload->timestamp, stream_ctx->stream_id);
		jpeg_obj = (struct rtp_jpeg_obj *)payload->extra;
		parse_jpeg_header(payload->data, payload->len, &rtp_width, &rtp_height, &type, &dri, &precision, jpeg_obj->lqt, jpeg_obj->cqt, &jpeg_obj->hdr_len);

		//In RFC 2435 for rtp jpg, the width and height should be less than 2040 (8bits 255 * 8 = 2040)
		if (rtp_width > 2040) {
			RTP_DBG_WARNING("[JPEG] RTP image width (%d) is exceeded the normal format (2040)\n\r", rtp_width);
			rtp_width = 0;
		}
		if (rtp_height > 2040) {
			RTP_DBG_WARNING("[JPEG] RTP image width (%d) is exceeded the normal format (2040)\n\r", rtp_height);
			rtp_height = 0;
		}
		fillJpegHeader(&jpeg_obj->jpghdr, type, /*typespec*/0, rtp_width, rtp_height, dri, /*q*/USE_EXPLICIT_DQT);
		fillRstHeader(&jpeg_obj->rsthdr, dri);
		fillqtable(&jpeg_obj->qtable, precision);
		jpeg_obj->frame_offset = 0;
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
			ret = rtp_jpeg_send_unicast(stream_ctx, payload);
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
		if ((stream_ctx->statistics.delta_timer < stream_ctx->statistics.delay_threshold) && (payload->fe)) {
			vTaskDelay(stream_ctx->statistics.delay_threshold - stream_ctx->statistics.delta_timer);
			//printf("\n\r-");
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
		stream_ctx->periodic_report.drop_frame++;
#if 0
		if (tick_diff >= rtp_drop_threshold) {
			RTP_DBG_INFO("[%d][M] Drop %d ms old data\n\r", rtsp_ctx->id, tick_diff);
		} else {
			RTP_DBG_INFO("[%d][M] Delay %d ms\n\r", rtsp_ctx->id,
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

#if MJPEG_DEBUG
void dumpJpegHeader(struct jpeghdr *jpghdr)
{
	printf("\n\rJpeg header info:");
	printf("\n\rid of jpeg decoder params:%d", jpghdr->type);
	printf("\n\rquantization factor (or table id):%d", jpghdr->q);
	printf("\n\rframe width in 8 pixel blocks:%d", jpghdr->width);
	printf("\n\rframe height in 8 pixel blocks:%d", jpghdr->height);
}

void dumpRstDeader(struct jpeghdr_rst *rsthdr)
{
	printf("\n\rRestart header info:");
	printf("\n\rRestart interval:%d", ntohs(rsthdr->dri));
	printf("\n\rRestart first bit set:%d", rsthdr->f);
	printf("\n\rRestart last bit set:%d", rsthdr->l);
	printf("\n\rRestart Count:%d", rsthdr->count);
}
#else
void dumpJpegHeader(struct jpeghdr *jpghdr)
{
}

void dumpRstDeader(struct jpeghdr_rst *rsthdr)
{
}
#endif
