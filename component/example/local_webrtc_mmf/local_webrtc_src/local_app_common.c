/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright 2026 AmebaPro2 WebRTC Contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "sys_api.h"
#include "sntp/sntp.h"
#include "wifi_conf.h"
#include "lwip_netconf.h"
#include "lwip/sockets.h"
#include "srtp.h"

#include "logging.h"
#include "local_app_common.h"
#include "networking_utils.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <errno.h>
#include "string_utils.h"

/* Binary memory search - forward declaration */
static char *memstr(const char *haystack, size_t haystack_len,
					const char *needle, size_t needle_len);
#include "app_media_source.h"
#include "peer_connection.h"
/* peer_connection_sdp.h is included implicitly via peer_connection.h */

/* FreeRTOS event group bits */
#define APP_COMMON_SIGNALING_BARRIER_BIT   (1 << 0)

/* Forward declarations */
extern int crypto_init(void);
extern int platform_set_malloc_free(void *(*malloc_func)(size_t),
									void (*free_func)(void *));

/* ========================================================================== */
/*  Platform Initialization                                                    */
/* ========================================================================== */

static void wifi_common_init(void)
{
	uint32_t wifi_wait_count = 0;
	const uint32_t wifi_wait_time = 500; /* ~5 seconds */

	while (!((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) &&
			 (*(uint32_t *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
		vTaskDelay(10);
		wifi_wait_count++;
		if (wifi_wait_count == wifi_wait_time) {
			printf("\r\nuse ATW0, ATW1, ATWC to make wifi connection\r\n");
			printf("wait for wifi connection...\r\n");
			wifi_wait_count = 0;
		}
	}
}

/* platform_set_malloc_free expects void *(*)(size_t), but calloc has
 * signature void *(size_t, size_t). On ARM the second argument ends up
 * in R1 and is ignored, making (void *(*))calloc behave as calloc(1, size)
 * in practice on this platform. A wrapper was attempted but shifted the
 * heap layout and triggered a latent use-after-free elsewhere at boot.
 * The direct cast matches the SDK's existing kvs_webrtc_v2_mmf example. */
static void platform_init(void)
{
	long long sec;

	/* mbedTLS init */
	crypto_init();
	platform_set_malloc_free((void *(*))calloc,
							 (void (*)(void *))free);

	/* Show backtrace if exception. */
	sys_backtrace_enable();

	/* Block until network ready. */
	wifi_common_init();

	/* Try to get NTP time via SNTP, but don't block forever.
	 * DTLS-SRTP does not require wall-clock time; the epoch is only
	 * used for STUN/ICE logging and timestamp presentation. On a
	 * LAN-only deployment (no NTP server), skip after 30 seconds. */
	{
		int sntp_retries = 150; /* 150 * 200ms = 30 seconds */
		sntp_init();
		while ((sec = NetworkingUtils_GetCurrentTimeSec(NULL)) < 1000000000ULL) {
			vTaskDelay(pdMS_TO_TICKS(200));
			if (--sntp_retries <= 0) {
				LogWarn(("SNTP time not acquired after 30s, continuing without wall-clock time"));
				break;
			}
			LogInfo(("waiting get epoch timer"));
		}
	}
}

/* ========================================================================== */
/*  Callbacks                                                                 */
/* ========================================================================== */

static const char *GetCandidateTypeString(IceCandidateType_t candidateType)
{
	switch (candidateType) {
	case ICE_CANDIDATE_TYPE_HOST:
		return "host";
	case ICE_CANDIDATE_TYPE_PEER_REFLEXIVE:
		return "prflx";
	case ICE_CANDIDATE_TYPE_SERVER_REFLEXIVE:
		return "srflx";
	case ICE_CANDIDATE_TYPE_RELAY:
		return "relay";
	default:
		return "unknown";
	}
}

static void HandleLocalCandidateReady(void *pCustomContext,
									  PeerConnectionIceLocalCandidate_t *pIceLocalCandidate)
{
	AppContext_t *pAppContext = (AppContext_t *)pCustomContext;
	int written;

	if (pAppContext == NULL || pIceLocalCandidate == NULL) {
		LogWarn(("Invalid local candidate ready callback"));
		return;
	}

	if (pIceLocalCandidate->pLocalCandidate == NULL) {
		LogWarn(("Local candidate payload is NULL, skipping"));
		return;
	}

	if (pAppContext->localCandidateCount >= LOCAL_WEBRTC_MAX_CANDIDATES) {
		LogWarn(("Too many local candidates, dropping"));
		return;
	}

	if (pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.family == STUN_ADDRESS_IPv4) {
		written = snprintf(
					  pAppContext->localCandidates[pAppContext->localCandidateCount],
					  DEMO_JSON_CANDIDATE_MAX_LENGTH,
					  DEMO_ICE_CANDIDATE_JSON_IPV4_TEMPLATE,
					  pIceLocalCandidate->localCandidateIndex,
					  pIceLocalCandidate->pLocalCandidate->priority,
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[0],
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[1],
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[2],
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[3],
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.port,
					  GetCandidateTypeString(pIceLocalCandidate->pLocalCandidate->candidateType));
	} else {
		written = snprintf(
					  pAppContext->localCandidates[pAppContext->localCandidateCount],
					  DEMO_JSON_CANDIDATE_MAX_LENGTH,
					  DEMO_ICE_CANDIDATE_JSON_IPV6_TEMPLATE,
					  pIceLocalCandidate->localCandidateIndex,
					  pIceLocalCandidate->pLocalCandidate->priority,
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[0],
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[1],
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[2],
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[3],
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[4],
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[5],
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[6],
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[7],
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[8],
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[9],
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[10],
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[11],
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[12],
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[13],
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[14],
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.address[15],
					  pIceLocalCandidate->pLocalCandidate->endpoint.transportAddress.port,
					  GetCandidateTypeString(pIceLocalCandidate->pLocalCandidate->candidateType));
	}

	if (written > 0) {
		pAppContext->localCandidateCount++;
		LogInfo(("Collected local candidate %u: %s",
				 (unsigned int)pAppContext->localCandidateCount,
				 pAppContext->localCandidates[pAppContext->localCandidateCount - 1]));
	}
}

static PeerConnectionResult_t HandleRxVideoFrame(void *pCustomContext,
		PeerConnectionFrame_t *pFrame)
{
	AppContext_t *pAppContext = (AppContext_t *)pCustomContext;
	MediaFrame_t frame;

	if (pFrame != NULL) {
		memset(&frame, 0, sizeof(MediaFrame_t));
		frame.trackKind = TRANSCEIVER_TRACK_KIND_VIDEO;
		frame.pData = pFrame->pData;
		frame.size = pFrame->dataLength;
		frame.freeData = 0U;
		frame.timestampUs = pFrame->presentationUs;
		(void)AppMediaSource_RecvFrame(pAppContext->pAppMediaSourcesContext, &frame);
	}

	return PEER_CONNECTION_RESULT_OK;
}

static PeerConnectionResult_t HandleRxAudioFrame(void *pCustomContext,
		PeerConnectionFrame_t *pFrame)
{
	AppContext_t *pAppContext = (AppContext_t *)pCustomContext;
	MediaFrame_t frame;

	if (pFrame != NULL) {
		memset(&frame, 0, sizeof(MediaFrame_t));
		frame.trackKind = TRANSCEIVER_TRACK_KIND_AUDIO;
		frame.pData = pFrame->pData;
		frame.size = pFrame->dataLength;
		frame.freeData = 0U;
		frame.timestampUs = pFrame->presentationUs;
		(void)AppMediaSource_RecvFrame(pAppContext->pAppMediaSourcesContext, &frame);
	}

	return PEER_CONNECTION_RESULT_OK;
}

/* ========================================================================== */
/*  Session Management                                                         */
/* ========================================================================== */

static int32_t InitializeAppSession(AppSession_t *pAppSession)
{
	int32_t ret = 0;
	PeerConnectionResult_t peerConnectionResult;
	PeerConnectionSessionConfiguration_t pcConfig;

	memset(&pcConfig, 0, sizeof(PeerConnectionSessionConfiguration_t));
	pcConfig.canTrickleIce = 1U;
	pcConfig.natTraversalConfigBitmap = ICE_CANDIDATE_NAT_TRAVERSAL_CONFIG_ALLOW_ALL;

	peerConnectionResult = PeerConnection_Init(&pAppSession->peerConnectionSession, &pcConfig);
	if (peerConnectionResult != PEER_CONNECTION_RESULT_OK) {
		LogWarn(("PeerConnection_Init fail, result: %d", peerConnectionResult));
		ret = -1;
	}

	return ret;
}

int LocalAppCommon_StartPeerConnection(AppContext_t *pAppContext,
									   AppSession_t *pAppSession)
{
	int32_t ret = 0;
	PeerConnectionResult_t peerConnectionResult;
	PeerConnectionSessionConfiguration_t pcConfig;
	Transceiver_t *pTransceiver = NULL;

	memset(&pcConfig, 0, sizeof(PeerConnectionSessionConfiguration_t));
	pcConfig.iceServersCount = 0; /* No STUN/TURN for LAN */
	pcConfig.canTrickleIce = 1U;
	pcConfig.natTraversalConfigBitmap = ICE_CANDIDATE_NAT_TRAVERSAL_CONFIG_ALLOW_ALL;

	/* Add ICE server config (none for LAN — host candidates only) */
	peerConnectionResult = PeerConnection_AddIceServerConfig(&pAppSession->peerConnectionSession, &pcConfig);
	if (peerConnectionResult != PEER_CONNECTION_RESULT_OK) {
		LogWarn(("PeerConnection_AddIceServerConfig fail, result: %d", peerConnectionResult));
		ret = -1;
	}

	/* Set local candidate callback */
	if (ret == 0) {
		peerConnectionResult = PeerConnection_SetOnLocalCandidateReady(
								   &pAppSession->peerConnectionSession,
								   HandleLocalCandidateReady,
								   pAppContext);
		if (peerConnectionResult != PEER_CONNECTION_RESULT_OK) {
			LogWarn(("PeerConnection_SetOnLocalCandidateReady fail, result: %d", peerConnectionResult));
			ret = -1;
		}
	}

	/* Add video transceiver */
	if (ret == 0) {
		pTransceiver = &pAppSession->transceivers[DEMO_TRANSCEIVER_MEDIA_INDEX_VIDEO];
		ret = pAppContext->initTransceiverFunc(pAppContext->pAppMediaSourcesContext,
											   TRANSCEIVER_TRACK_KIND_VIDEO,
											   pTransceiver);
		if (ret != 0) {
			LogError(("Fail to get video transceiver."));
		} else {
			peerConnectionResult = PeerConnection_AddTransceiver(&pAppSession->peerConnectionSession,
								   pTransceiver);
			if (peerConnectionResult != PEER_CONNECTION_RESULT_OK) {
				LogError(("Fail to add video transceiver, result = %d.", peerConnectionResult));
				ret = -1;
			}
		}
	}

	/* Add audio transceiver */
	if (ret == 0) {
		pTransceiver = &pAppSession->transceivers[DEMO_TRANSCEIVER_MEDIA_INDEX_AUDIO];
		ret = pAppContext->initTransceiverFunc(pAppContext->pAppMediaSourcesContext,
											   TRANSCEIVER_TRACK_KIND_AUDIO,
											   pTransceiver);
		if (ret != 0) {
			LogError(("Fail to get audio transceiver."));
		} else {
			peerConnectionResult = PeerConnection_AddTransceiver(&pAppSession->peerConnectionSession,
								   pTransceiver);
			if (peerConnectionResult != PEER_CONNECTION_RESULT_OK) {
				LogError(("Fail to add audio transceiver, result = %d.", peerConnectionResult));
				ret = -1;
			}
		}
	}

	/* Start the peer connection session */
	if (ret == 0) {
		peerConnectionResult = PeerConnection_Start(&pAppSession->peerConnectionSession);
		if (peerConnectionResult != PEER_CONNECTION_RESULT_OK) {
			LogError(("Fail to start peer connection, result = %d.", peerConnectionResult));
			ret = -1;
		}
	}

	return ret;
}

/* ========================================================================== */
/*  SDP Offer/Answer Handling                                                  */
/* ========================================================================== */

int LocalAppCommon_HandleSdpOffer(AppContext_t *pAppContext,
								  const char *pSdpOffer,
								  size_t sdpOfferLength,
								  char *pAnswerSdp,
								  size_t *pAnswerSdpLength)
{
	int ret = 0;
	PeerConnectionResult_t peerConnectionResult;
	PeerConnectionBufferSessionDescription_t bufferSessionDescription;
	AppSession_t *pAppSession = NULL;

	if (pAppContext == NULL || pSdpOffer == NULL || pAnswerSdp == NULL || pAnswerSdpLength == NULL) {
		LogError(("Invalid input"));
		return -1;
	}

	/* Get the first available peer connection session.
	 * If a session is already active on this slot, close it first to avoid
	 * leaking ICE, DTLS, and SRTP resources from the previous connection. */
	pAppSession = &pAppContext->appSessions[0];
	PeerConnection_CloseSession(&pAppSession->peerConnectionSession);

	/* Initialize and start the peer connection */
	ret = LocalAppCommon_StartPeerConnection(pAppContext, pAppSession);
	if (ret != 0) {
		LogError(("Fail to start peer connection"));
		return -1;
	}

	/* Reset candidate collection */
	pAppContext->localCandidateCount = 0;
	/* Set remote description (browser's SDP offer) */
	if (ret == 0) {
		memset(&bufferSessionDescription, 0, sizeof(bufferSessionDescription));
		bufferSessionDescription.pSdpBuffer = (char *)pSdpOffer;
		bufferSessionDescription.sdpBufferLength = sdpOfferLength;
		bufferSessionDescription.type = SDP_CONTROLLER_MESSAGE_TYPE_OFFER;

		peerConnectionResult = PeerConnection_SetRemoteDescription(&pAppSession->peerConnectionSession,
							   &bufferSessionDescription);
		if (peerConnectionResult != PEER_CONNECTION_RESULT_OK) {
			LogWarn(("PeerConnection_SetRemoteDescription fail, result: %d", peerConnectionResult));
			ret = -1;
		}
	}

	/* Set video/audio receive callbacks */
	if (ret == 0) {
		peerConnectionResult = PeerConnection_SetVideoOnFrame(&pAppSession->peerConnectionSession,
							   HandleRxVideoFrame,
							   pAppContext);
		if (peerConnectionResult != PEER_CONNECTION_RESULT_OK) {
			LogWarn(("PeerConnection_SetVideoOnFrame fail, result: %d.", peerConnectionResult));
			ret = -1;
		}
	}

	if (ret == 0) {
		peerConnectionResult = PeerConnection_SetAudioOnFrame(&pAppSession->peerConnectionSession,
							   HandleRxAudioFrame,
							   pAppContext);
		if (peerConnectionResult != PEER_CONNECTION_RESULT_OK) {
			LogWarn(("PeerConnection_SetAudioOnFrame fail, result: %d.", peerConnectionResult));
			ret = -1;
		}
	}

	/* Set local description (prepares for answer) */
	if (ret == 0) {
		memset(&bufferSessionDescription, 0, sizeof(bufferSessionDescription));
		bufferSessionDescription.pSdpBuffer = pAppContext->sdpBuffer;
		bufferSessionDescription.sdpBufferLength = PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH;
		peerConnectionResult = PeerConnection_SetLocalDescription(&pAppSession->peerConnectionSession,
							   &bufferSessionDescription);
		if (peerConnectionResult != PEER_CONNECTION_RESULT_OK) {
			LogWarn(("PeerConnection_SetLocalDescription fail, result: %d.", peerConnectionResult));
			ret = -1;
		}
	}

	/* Create answer */
	if (ret == 0) {
		pAppContext->sdpConstructedBufferLength = PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH;
		memset(&bufferSessionDescription, 0, sizeof(bufferSessionDescription));
		bufferSessionDescription.pSdpBuffer = pAppContext->sdpBuffer;
		bufferSessionDescription.sdpBufferLength = PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH;

		peerConnectionResult = PeerConnection_CreateAnswer(&pAppSession->peerConnectionSession,
							   &bufferSessionDescription,
							   pAppContext->sdpConstructedBuffer,
							   &pAppContext->sdpConstructedBufferLength);
		if (peerConnectionResult != PEER_CONNECTION_RESULT_OK) {
			LogWarn(("PeerConnection_CreateAnswer fail, result: %d.", peerConnectionResult));
			ret = -1;
		}
	}

	/*
	 * Wait a short time for local host candidates to arrive via the
	 * HandleLocalCandidateReady callback. On LAN with host-only candidates,
	 * this is typically sub-second.
	 */
	if (ret == 0) {
		TickType_t startTick = xTaskGetTickCount();
		TickType_t timeoutTicks = pdMS_TO_TICKS(3000); /* 3-second timeout */

		while (pAppContext->localCandidateCount == 0 &&
			   (xTaskGetTickCount() - startTick) < timeoutTicks) {
			vTaskDelay(pdMS_TO_TICKS(100));
		}

		/* Wait a bit more to collect additional candidates */
		if (pAppContext->localCandidateCount > 0) {
			vTaskDelay(pdMS_TO_TICKS(500));
		}

		LogInfo(("ICE candidate collection complete: %u candidate(s)",
				 (unsigned int)pAppContext->localCandidateCount));
	}

	/*
	 * Build the final answer SDP.
	 * The CreateAnswer output is JSON: {"type":"answer","sdp":"<SDP>\r\n"}
	 * We extract the raw SDP, append ICE candidates, and output the raw SDP.
	 * The caller will JSON-escape and wrap it.
	 */
	if (ret == 0) {
		size_t i;
		/* Find the SDP content within the CreateAnswer JSON output.
		 * Format: {"type":"answer","sdp":"<raw_sdp>\r\n"} */
		const char *sdp_key_nospace = "\"sdp\":\"";
		const char *sdp_key_space   = "\"sdp\": \"";
		const char *sdp_key = NULL;
		char *sdp_start = strstr(pAppContext->sdpConstructedBuffer, sdp_key_nospace);
		if (sdp_start != NULL) {
			sdp_key = sdp_key_nospace;
		} else {
			sdp_start = strstr(pAppContext->sdpConstructedBuffer, sdp_key_space);
			if (sdp_start != NULL) {
				sdp_key = sdp_key_space;
			}
		}
		if (sdp_start == NULL) {
			LogError(("Cannot find sdp field in CreateAnswer output"));
			ret = -1;
		} else {
			sdp_start += strlen(sdp_key); /* skip past the key to the SDP value */
			char *sdp_close = strstr(sdp_start, "\"}");
			if (sdp_close == NULL) {
				LogError(("Cannot find end of sdp in CreateAnswer output"));
				ret = -1;
			} else {
				size_t sdp_raw_len = sdp_close - sdp_start;

				/* Compute worst-case candidate text size:
				 * Each candidate: "a=<candidate>\r\n" where candidate is up to
				 * DEMO_JSON_CANDIDATE_MAX_LENGTH bytes. */
				size_t cand_max = pAppContext->localCandidateCount *
								  (2 + DEMO_JSON_CANDIDATE_MAX_LENGTH) +
								  sizeof("a=end-of-candidates\r\n") + 1;

				/* Copy raw SDP to output */
				if (sdp_raw_len + cand_max < *pAnswerSdpLength) {
					memcpy(pAnswerSdp, sdp_start, sdp_raw_len);

					/* Build candidate text */
					char *cand_buf = (char *)pvPortMalloc(cand_max);
					if (cand_buf == NULL) {
						LogError(("Cannot allocate candidate buffer"));
						ret = -1;
					} else {
						size_t cand_len = 0;
						for (i = 0; i < pAppContext->localCandidateCount; i++) {
							cand_len += snprintf(cand_buf + cand_len, cand_max - cand_len,
												 "a=%s\r\n", pAppContext->localCandidates[i]);
						}
						cand_len += snprintf(cand_buf + cand_len, cand_max - cand_len,
											 "a=end-of-candidates\r\n");

						/* Place candidates INSIDE the video section (before m=audio),
						 * so Chrome associates them with the BUNDLE ICE transport. */
						char *audio_pos = memstr(pAnswerSdp, sdp_raw_len, "\nm=audio", 8);
						if (audio_pos == NULL) {
							audio_pos = memstr(pAnswerSdp, sdp_raw_len, "m=audio", 7);
						}
						if (audio_pos) {
							audio_pos++; /* point to 'm' in m=audio */
							size_t video_part_len = audio_pos - pAnswerSdp;
							size_t audio_part_len = sdp_raw_len - video_part_len;
							/* Build output: video part + candidates + audio part */
							size_t out_len = 0;
							memmove(pAnswerSdp + out_len, pAnswerSdp, video_part_len);
							out_len += video_part_len;
							/* Shift audio part right to make room */
							memmove(pAnswerSdp + out_len + cand_len, pAnswerSdp + out_len, audio_part_len);
							memcpy(pAnswerSdp + out_len, cand_buf, cand_len);
							out_len += cand_len + audio_part_len;
							*pAnswerSdpLength = out_len;
							vPortFree(cand_buf);
							cand_buf = NULL;
						} else {
							/* Fallback: append at end */
							size_t outLen = sdp_raw_len;
							for (i = 0; i < pAppContext->localCandidateCount; i++) {
								outLen += snprintf(pAnswerSdp + outLen, *pAnswerSdpLength - outLen,
												   "a=%s\r\n", pAppContext->localCandidates[i]);
							}
							outLen += snprintf(pAnswerSdp + outLen, *pAnswerSdpLength - outLen,
											   "a=end-of-candidates\r\n");
							*pAnswerSdpLength = outLen;
							vPortFree(cand_buf);
						}
						LogInfo(("Raw SDP length: %u with %u candidates",
								 (unsigned int)*pAnswerSdpLength, (unsigned int)pAppContext->localCandidateCount));
					}
				} else {
					ret = -1;
				}
			}
		}
	}

	return ret;
}

/* ========================================================================== */
/*  HTTP Signaling Server                                                      */
/* ========================================================================== */

/*
 * Simple HTTP signaling server.
 *
 * Endpoints:
 *   GET  /               → Serves the browser viewer HTML page
 *   POST /webrtc/offer   → Receives SDP offer (JSON), returns SDP answer (JSON)
 *
 * JSON format for offer/answer:
 *   {"type":"offer","sdp":"..."}
 *   {"type":"answer","sdp":"..."}
 */

#define HTTP_PORT         (80)
#define JSON_RESPONSE_MAX_LENGTH     ( PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH * 6 + 256 )
#define HTTP_BUF_SIZE     (16384)
#define MAX_REQUEST_SIZE  (16384)

/* Binary memory search (no null termination needed) */
static char *memstr(const char *haystack, size_t haystack_len,
					const char *needle, size_t needle_len)
{
	size_t i;
	if (needle_len == 0) {
		return (char *)haystack;
	}
	if (haystack_len < needle_len) {
		return NULL;
	}
	for (i = 0; i <= haystack_len - needle_len; i++) {
		if (memcmp(&haystack[i], needle, needle_len) == 0) {
			return (char *)&haystack[i];
		}
	}
	return NULL;
}

static const char *get_content_type(const char *path)
{
	const char *ext = strrchr(path, '.');
	if (ext) {
		if (strcmp(ext, ".html") == 0 || strcmp(ext, ".htm") == 0) {
			return "text/html";
		}
		if (strcmp(ext, ".js") == 0) {
			return "application/javascript";
		}
		if (strcmp(ext, ".css") == 0) {
			return "text/css";
		}
		if (strcmp(ext, ".json") == 0) {
			return "application/json";
		}
	}
	return "text/html";
}

static const char web_page[] =
	"<!DOCTYPE html>\n"
	"<html>\n"
	"<head>\n"
	"<meta charset=\"utf-8\">\n"
	"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
	"<title>AmebaPro2 WebRTC</title>\n"
	"<style>\n"
	"body { font-family: Arial, sans-serif; margin: 20px; background: #111; color: #eee; }\n"
	"video { width: 100%%; max-width: 800px; background: #000; border-radius: 8px; }\n"
	".status { margin: 10px 0; padding: 8px; border-radius: 4px; }\n"
	".connecting { background: #664; color: #ff0; }\n"
	".connected { background: #262; color: #0f0; }\n"
	".error { background: #622; color: #f00; }\n"
	"h1 { color: #0af; }\n"
	"</style>\n"
	"</head>\n"
	"<body>\n"
	"<h1>AmebaPro2 WebRTC</h1>\n"
	"<div id=\"status\" class=\"status connecting\">Initializing...</div>\n"
	"<video id=\"video\" autoplay playsinline muted controls></video>\n"
	"<audio id=\"audio\" autoplay controls></audio>\n"
	"<script>\n"
	"async function waitForIceGathering(pc) {\n"
	"  if (pc.iceGatheringState === \"complete\") return;\n"
	"  await new Promise((resolve) => {\n"
	"    const listener = () => {\n"
	"      if (pc.iceGatheringState === \"complete\") {\n"
	"        pc.removeEventListener(\"icegatheringstatechange\", listener);\n"
	"        resolve();\n"
	"      }\n"
	"    };\n"
	"    pc.addEventListener(\"icegatheringstatechange\", listener);\n"
	"  });\n"
	"}\n"
	"async function start() {\n"
	"  const status = document.getElementById('status');\n"
	"  try {\n"
	"    const pc = new RTCPeerConnection({ iceServers: [] });\n"
	"    pc.addTransceiver(\"video\", { direction: \"recvonly\" });\n"
	"    pc.addTransceiver(\"audio\", { direction: \"recvonly\" });\n"
	"    pc.ontrack = (event) => {\n"
	"      if (event.track.kind === \"video\") {\n"
	"        document.getElementById(\"video\").srcObject = event.streams[0];\n"
	"      } else if (event.track.kind === \"audio\") {\n"
	"        document.getElementById(\"audio\").srcObject = event.streams[0];\n"
	"      }\n"
	"      status.textContent = 'Connected - receiving ' + event.track.kind;\n"
	"      status.className = 'status connected';\n"
	"    };\n"
	"    pc.oniceconnectionstatechange = () => {\n"
	"      status.textContent = 'ICE: ' + pc.iceConnectionState;\n"
	"    };\n"
	"    status.textContent = 'Creating offer...';\n"
	"    const offer = await pc.createOffer();\n"
	"    await pc.setLocalDescription(offer);\n"
	"    status.textContent = 'Gathering ICE candidates...';\n"
	"    await waitForIceGathering(pc);\n"
	"    status.textContent = 'Sending offer to camera...';\n"
	"    const response = await fetch('/webrtc/offer', {\n"
	"      method: 'POST',\n"
	"      headers: { 'Content-Type': 'application/json' },\n"
	"      body: JSON.stringify(pc.localDescription)\n"
	"    });\n"
	"    if (!response.ok) throw new Error('Offer failed: ' + response.status);\n"
	"    const answer = await response.json();\n"
	"    status.textContent = 'Applying answer...';\n"
	"    await pc.setRemoteDescription(answer);\n"
	"    status.textContent = 'Connecting...';\n"
	"  } catch (e) {\n"
	"    status.textContent = 'Error: ' + e.message;\n"
	"    status.className = 'status error';\n"
	"    console.error(e);\n"
	"  }\n"
	"}\n"
	"start();\n"
	"</script>\n"
	"</body>\n"
	"</html>\n";

static int tcp_send_all(int fd, const char *data, size_t len)
{
	while (len > 0) {
		int sent = write(fd, data, len);
		if (sent < 0) {
			/* Retry on EAGAIN/EWOULDBLOCK — buffer full, try again */
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				vTaskDelay(pdMS_TO_TICKS(10));
				continue;
			}
			return -1;
		}
		if (sent == 0) {
			return -1;
		}
		data += sent;
		len -= sent;
	}
	return 0;
}

static int http_send_response(int client_fd, int status_code,
							  const char *status_text,
							  const char *content_type,
							  const char *body, size_t body_len)
{
	char header[512];
	int hlen = snprintf(header, sizeof(header),
						"HTTP/1.1 %d %s\r\n"
						"Content-Type: %s\r\n"
						"Content-Length: %u\r\n"
						"Connection: close\r\n"
						"Access-Control-Allow-Origin: *\r\n"
						"\r\n",
						status_code, status_text,
						content_type ? content_type : "text/plain",
						(unsigned int)body_len);

	if (hlen <= 0) {
		return -1;
	}
	if (tcp_send_all(client_fd, header, hlen) < 0) {
		return -1;
	}
	if (body && body_len > 0) {
		if (tcp_send_all(client_fd, body, body_len) < 0) {
			return -1;
		}
	}
	return 0;
}

static int http_serve_static(int client_fd, const char *path)
{
	(void)path;
	return http_send_response(client_fd, 200, "OK",
							  "text/html; charset=utf-8",
							  web_page, strlen(web_page));
}

/*
 * Parse JSON SDP message.
 * Expects:  {"type":"offer","sdp":"..."}
 * Extracts: sdp_value and type_value
 */
static int parse_json_sdp(const char *json, size_t json_len,
						  char *sdp_out, size_t sdp_out_size,
						  size_t *sdp_out_len)
{
	const char *sdp_key = "\"sdp\":\"";
	const char *sdp_start, * sdp_end;
	const char *end_marker = "\"}";
	size_t sdp_len;

	/* Find sdp value */
	sdp_start = strstr(json, sdp_key);
	if (sdp_start == NULL) {
		return -1;
	}
	sdp_start += strlen(sdp_key);

	/* Find end of sdp value */
	sdp_end = strstr(sdp_start, end_marker);
	if (sdp_end == NULL) {
		/* Try just closing quote */
		sdp_end = strchr(sdp_start, '"');
		if (sdp_end == NULL) {
			return -1;
		}
	}

	sdp_len = sdp_end - sdp_start;

	/* Handle escaped characters (very basic: unescape \\n and \\r\\n) */
	{
		size_t src = 0, dst = 0;
		while (src < sdp_len && dst < sdp_out_size - 1) {
			if (sdp_start[src] == '\\' && src + 1 < sdp_len) {
				if (sdp_start[src + 1] == 'n') {
					sdp_out[dst++] = '\n';
					src += 2;
				} else if (sdp_start[src + 1] == 'r') {
					sdp_out[dst++] = '\r';
					src += 2; /* preserve \r, let next iteration output \n */
				} else {
					sdp_out[dst++] = sdp_start[src++];
				}
			} else {
				sdp_out[dst++] = sdp_start[src++];
			}
		}
		sdp_out[dst] = '\0';
		*sdp_out_len = dst;
	}

	return 0;
}

static void http_server_task(void *param)
{
	AppContext_t *pAppContext = (AppContext_t *)param;
	int server_fd;
	struct sockaddr_in server_addr;

	/* Create TCP socket */
	server_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (server_fd < 0) {
		LogError(("Failed to create HTTP server socket"));
		vTaskDelete(NULL);
		return;
	}

	/* Allow address reuse */
	{
		int opt = 1;
		setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
	}

	/* Bind to port 80 */
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = INADDR_ANY;
	server_addr.sin_port = htons(HTTP_PORT);

	if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
		LogError(("Failed to bind HTTP server to port %d", HTTP_PORT));
		close(server_fd);
		vTaskDelete(NULL);
		return;
	}

	/* Listen */
	if (listen(server_fd, 1) < 0) {
		LogError(("Failed to listen on HTTP server"));
		close(server_fd);
		vTaskDelete(NULL);
		return;
	}

	LogInfo(("HTTP signaling server listening on port %d", HTTP_PORT));

	for (;;) {
		int client_fd;
		struct sockaddr_in client_addr;
		socklen_t client_len = sizeof(client_addr);
		client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_len);
		if (client_fd < 0) {
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}

		/* Enable linger so close() waits for pending data to be sent */
		{
			struct linger ling = {1, 3}; /* linger for 3 seconds */
			setsockopt(client_fd, SOL_SOCKET, SO_LINGER, &ling, sizeof(ling));
		}

		/* Allocate receive buffer on the heap and zero it */
		char *recv_buf = (char *)pvPortMalloc(MAX_REQUEST_SIZE);
		if (recv_buf) {
			memset(recv_buf, 0, MAX_REQUEST_SIZE);
		}
		if (recv_buf == NULL) {
			close(client_fd);
			continue;
		}

		/* Receive request — read headers + body in a loop */
		int total_recv = 0;
		while (total_recv < MAX_REQUEST_SIZE - 1) {
			int n = read(client_fd, recv_buf + total_recv, MAX_REQUEST_SIZE - 1 - total_recv);
			if (n <= 0) {
				break;
			}
			total_recv += n;
			recv_buf[total_recv] = '\0'; /* for strchr below */
			/* Check if we have the full HTTP headers (end of headers) */
			if (total_recv >= 4 &&
				memstr(recv_buf, total_recv, "\r\n\r\n", 4) != NULL) {
				/* Headers received. Check Content-Length for body. */
				char *cl = memstr(recv_buf, total_recv, "Content-Length: ", 16);
				if (cl == NULL) {
					cl = memstr(recv_buf, total_recv, "content-length: ", 16);
				}
				if (cl != NULL) {
					cl += 16; /* skip "Content-Length: " */
					size_t content_length = 0;
					while (*cl >= '0' && *cl <= '9') {
						content_length = content_length * 10 + (*cl - '0');
						cl++;
					}
					/* Check if we have the full body:
					 * total_recv must be >= (headers_end + 4 + content_length) */
					char *body_start = NULL;
					size_t body_offset = 0;
					body_start = memstr(recv_buf, total_recv, "\r\n\r\n", 4);
					if (body_start != NULL) {
						body_offset = 4;
					} else {
						body_start = memstr(recv_buf, total_recv, "\n\n", 2);
						if (body_start != NULL) {
							body_offset = 2;
						}
					}
					if (body_start != NULL) {
						body_start += body_offset;
						size_t body_received = total_recv - (body_start - recv_buf);
						if (body_received >= content_length) {
							break; /* Full request received */
						}
					}
				} else {
					break; /* No Content-Length, assume all data received */
				}
			}
		}
		recv_buf[total_recv] = '\0';

		/* Parse HTTP method and path */
		if (strncmp(recv_buf, "GET ", 4) == 0) {
			char *path_start = recv_buf + 4;
			char *path_end = strchr(path_start, ' ');
			if (path_end) {
				*path_end = '\0';
			}

			http_serve_static(client_fd, path_start);

		} else if (strncmp(recv_buf, "POST ", 5) == 0) {
			/* Check for /webrtc/offer */
			char *path_start = recv_buf + 5;
			char *path_end = strchr(path_start, ' ');
			if (path_end) {
				*path_end = '\0';
			}

			if (strcmp(path_start, "/webrtc/offer") == 0) {
				/* Find the HTTP body (after \r\n\r\n) using memstr (binary-safe) */
				char *body = memstr(recv_buf, total_recv, "\r\n\r\n", 4);
				size_t body_len;

				if (body == NULL) {
					LogError(("HTTP body not found (no \\r\\n\\r\\n in %d bytes)", total_recv));
					/* Dump first 200 bytes as hex to see the raw data */
					{
						int dbg_i;
						char hexbuf[1200] = "";
						int hlen = 0;
						for (dbg_i = 0; dbg_i < 200 && dbg_i < total_recv; dbg_i++) {
							hlen += snprintf(hexbuf + hlen, sizeof(hexbuf) - hlen,
											 "%02x ", (unsigned char)recv_buf[dbg_i]);
							if (hlen >= (int)sizeof(hexbuf) - 10) {
								break;
							}
						}
						LogError(("Raw hex (first 200 bytes): %s", hexbuf));
					}
					http_send_response(client_fd, 400, "Bad Request", NULL, NULL, 0);
					vPortFree(recv_buf);
					close(client_fd);
					continue;
				}
				body += 4;
				body_len = total_recv - (body - recv_buf);
				LogInfo(("HTTP body received: %u bytes", (unsigned int)body_len));
				if (body_len > 0 && body_len < 200) {
					LogInfo(("Body content: %.*s", (int)body_len, body));
				}

				/* Allocate SDP buffers on the heap */
				char *sdp_offer = (char *)pvPortMalloc(PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH);
				char *answer_sdp = (char *)pvPortMalloc(PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH);
				char *json_response = (char *)pvPortMalloc(JSON_RESPONSE_MAX_LENGTH);

				if (sdp_offer == NULL || answer_sdp == NULL || json_response == NULL) {
					if (sdp_offer) {
						vPortFree(sdp_offer);
					}
					if (answer_sdp) {
						vPortFree(answer_sdp);
					}
					if (json_response) {
						vPortFree(json_response);
					}
					http_send_response(client_fd, 500, "Internal Server Error", NULL, NULL, 0);
					vPortFree(recv_buf);
					close(client_fd);
					continue;
				}

				size_t sdp_offer_len = 0;
				if (parse_json_sdp(body, body_len,
								   sdp_offer, PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH,
								   &sdp_offer_len) != 0) {
					LogError(("JSON parse failed on %u bytes body: %.*s",
							  (unsigned int)body_len,
							  (int)(body_len < 256 ? body_len : 256), body));
					http_send_response(client_fd, 400, "Bad Request",
									   "text/plain", "Invalid SDP JSON", 15);
					vPortFree(sdp_offer);
					vPortFree(answer_sdp);
					vPortFree(json_response);
					vPortFree(recv_buf);
					close(client_fd);
					continue;
				}

				/*
				 * Replace mDNS hostnames (.local) in ICE candidates with the
				 * browser's actual IP address from the TCP connection.
				 * Chrome uses mDNS to hide IPs; the browser's TCP source IP
				 * is the same as its ICE interface IP on a LAN.
				 */
				{
					/* Get browser IP from TCP connection (network byte order) */
					uint8_t *ip_bytes = (uint8_t *)&client_addr.sin_addr.s_addr;
					char ip_str[16];
					snprintf(ip_str, sizeof(ip_str), "%u.%u.%u.%u",
							 ip_bytes[0], ip_bytes[1], ip_bytes[2], ip_bytes[3]);

					/* Scan SDP for .local hostnames and replace with browser IP */
					int replaced = 0;
					char *scan = sdp_offer;
					while (scan && (scan - sdp_offer) < (int)sdp_offer_len - 6) {
						scan = memstr(scan, sdp_offer_len - (scan - sdp_offer), ".local", 6);
						if (scan == NULL) {
							break;
						}
						/* Find the start of the hostname (previous space) */
						char *host_start = scan;
						while (host_start > sdp_offer && *(host_start - 1) != ' ') {
							host_start--;
						}
						size_t host_len = (scan + 6) - host_start;
						size_t ip_len = strlen(ip_str);
						if (ip_len <= host_len) {
							memcpy(host_start, ip_str, ip_len);
							size_t remaining = host_len - ip_len;
							if (remaining > 0) {
								memmove(host_start + ip_len, scan + 6,
										sdp_offer_len - (scan + 6 - sdp_offer));
								sdp_offer_len -= remaining;
							}
							sdp_offer[sdp_offer_len] = '\0';
							scan = host_start + ip_len;
							replaced++;
						} else {
							scan += 6;
						}
					}
					if (replaced > 0) {
						LogInfo(("Replaced %d .local hostname(s) with IP: %s", replaced, ip_str));
					}
				}

				LogInfo(("Received SDP offer (%u bytes)", (unsigned int)sdp_offer_len));

				size_t answer_sdp_len = PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH;
				int result = LocalAppCommon_HandleSdpOffer(pAppContext,
							 sdp_offer, sdp_offer_len,
							 answer_sdp, &answer_sdp_len);

				if (result != 0) {
					http_send_response(client_fd, 500, "Internal Server Error",
									   "text/plain", "Failed to create SDP answer", 29);
				} else {
					int json_len = snprintf(json_response,
											JSON_RESPONSE_MAX_LENGTH,
											"{\"type\":\"answer\",\"sdp\":\"");
					size_t i;
					for (i = 0; i < answer_sdp_len &&
						 json_len < (int)(JSON_RESPONSE_MAX_LENGTH) - 4; i++) {
						if (answer_sdp[i] == '\n') {
							json_response[json_len++] = '\\';
							json_response[json_len++] = 'n';
						} else if (answer_sdp[i] == '\r') {
							json_response[json_len++] = '\\';
							json_response[json_len++] = 'r';
						} else if (answer_sdp[i] == '"') {
							json_response[json_len++] = '\\';
							json_response[json_len++] = '"';
						} else if (answer_sdp[i] == '\\') {
							json_response[json_len++] = '\\';
							json_response[json_len++] = '\\';
						} else if (answer_sdp[i] < 0x20) {
							/* Escape other control chars as \uXXXX (6 bytes per char).
							 * Check buffer space before writing to avoid overflow. */
							if (json_len >= (int)(JSON_RESPONSE_MAX_LENGTH) - 6) {
								break;
							}
							static const char hex[] = "0123456789abcdef";
							json_response[json_len++] = '\\';
							json_response[json_len++] = 'u';
							json_response[json_len++] = '0';
							json_response[json_len++] = '0';
							json_response[json_len++] = hex[(answer_sdp[i] >> 4) & 0x0F];
							json_response[json_len++] = hex[answer_sdp[i] & 0x0F];
						} else {
							json_response[json_len++] = answer_sdp[i];
						}
					}
					json_response[json_len++] = '"';
					json_response[json_len++] = '}';
					json_response[json_len] = '\0';

					int http_ret = http_send_response(client_fd, 200, "OK",
													  "application/json",
													  json_response, json_len);
					LogInfo(("SDP answer: json_len=%u, send_ret=%d, json_begins_with=%.32s",
							 (unsigned int)json_len, http_ret,
							 json_len > 32 ? json_response : "[too short]"));
				}
				vPortFree(sdp_offer);
				vPortFree(answer_sdp);
				vPortFree(json_response);
			} else {
				http_send_response(client_fd, 404, "Not Found", NULL, NULL, 0);
			}
		} else {
			http_send_response(client_fd, 405, "Method Not Allowed", NULL, NULL, 0);
		}

		vPortFree(recv_buf);
		close(client_fd);
	}

	close(server_fd);
	vTaskDelete(NULL);
}

/* ========================================================================== */
/*  Initialization                                                             */
/* ========================================================================== */

int LocalAppCommon_Init(AppContext_t *pAppContext,
						InitTransceiverFunc_t initTransceiverFunc,
						void *pMediaContext)
{
	int ret = 0;
	int i;

	if (pAppContext == NULL) {
		LogError(("Invalid input, AppContext is NULL"));
		return -1;
	}

	platform_init();

	memset(pAppContext, 0, sizeof(AppContext_t));

	srtp_init();

	/* Seed random. */
	srand(NetworkingUtils_GetCurrentTimeSec(NULL));

	/* Store context pointers */
	pAppContext->initTransceiverFunc = initTransceiverFunc;
	pAppContext->pAppMediaSourcesContext = (AppMediaSourcesContext_t *)pMediaContext;

	/* Initialize peer connection sessions */
	for (i = 0; i < LOCAL_WEBRTC_MAX_VIEWER_NUM; i++) {
		ret = InitializeAppSession(&(pAppContext->appSessions[i]));
		if (ret != 0) {
			LogError(("Fail to initialize peer connection session %d", i));
			break;
		}
	}

	return ret;
}

void LocalAppCommon_StartHttpServer(AppContext_t *pAppContext)
{
	if (xTaskCreate(http_server_task, "HttpSignaling",
					16384, pAppContext,
					tskIDLE_PRIORITY + 3,
					&pAppContext->httpServerTaskHandle) != pdPASS) {
		LogError(("Failed to create HTTP server task"));
	} else {
		LogInfo(("HTTP signaling server task created"));
	}
}
