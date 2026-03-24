/*
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PEER_CONNECTION_DATA_TYPES_H
#define PEER_CONNECTION_DATA_TYPES_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Standard includes. */
#include <stdint.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "transport_dtls_mbedtls.h"
#include "rtcp_twcc_manager.h"

#include "message_queue.h"
#include "ice_controller.h"
#include "transceiver_data_types.h"
#include "sdp_controller_data_types.h"

#include "srtp.h"

#if ENABLE_SCTP_DATA_CHANNEL
    #include "sctp_utils.h"
#endif /* ENABLE_SCTP_DATA_CHANNEL */

#include "rtp_data_types.h"
#include "rtp_pkt_queue.h"
#include "rtcp_data_types.h"

#define PEER_CONNECTION_TRANSCEIVER_MAX_COUNT ( 2 )
#define PEER_CONNECTION_USER_NAME_LENGTH ( 32 )
#define PEER_CONNECTION_PASSWORD_LENGTH ( 32 )
#define PEER_CONNECTION_CNAME_LENGTH ( 40 )
#define PEER_CONNECTION_CERTIFICATE_FINGERPRINT_LENGTH ( CERTIFICATE_FINGERPRINT_LENGTH )
#define PEER_CONNECTION_JITTER_BUFFER_MAX_ENTRY_NUM ( 1000 )
#define PEER_CONNECTION_FRAME_BUFFER_SIZE ( 16384 )

#define PEER_CONNECTION_FRAME_CURRENT_VERSION ( 0 )

#define PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH ( 10000 )

#define PEER_CONNECTION_RTCP_TWCC_MAX_ARRAY ( 100 )

#define PEER_CONNECTION_MAX_DTLS_DECRYPTED_DATA_LENGTH ( 2048 )

#define MAX_SCTP_DATA_CHANNELS          4
#define PEER_CONNECTION_MAX_SCTP_DATA_CHANNELS_PER_PEER 2

#define PEER_CONNECTION_TWCC_BITRATE_ADJUSTMENT_INTERVAL_US        1000 * 10000  //1,000,000 microseconds.
#define PEER_CONNECTION_MIN_VIDEO_BITRATE_KBPS                     512     // Unit kilobits/sec. Value could change based on codec.
#define PEER_CONNECTION_MAX_VIDEO_BITRATE_KBPS                     2048000 // Unit kilobits/sec. Value could change based on codec.
#define PEER_CONNECTION_MIN_AUDIO_BITRATE_BPS                      4000    // Unit bits/sec. Value could change based on codec.
#define PEER_CONNECTION_MAX_AUDIO_BITRATE_BPS                      650000  // Unit bits/sec. Value could change based on codec.

#define PEER_CONNECTION_WAIT_SDP_MESSAGE_TIMEOUT_MS    ( 24000 )
#define PEER_CONNECTION_INACTIVE_CONNECTION_TIMEOUT_MS ( 30000 )
#define PEER_CONNECTION_DTLS_HANDSHAKING_TIMEOUT_MS    ( 24000 )

#define PEER_CONNECTION_START_UP_BARRIER_BIT ( 1 << 0 )

typedef enum PeerConnectionResult
{
    PEER_CONNECTION_RESULT_OK = 0,
    PEER_CONNECTION_RESULT_CLOSING,
    PEER_CONNECTION_RESULT_BAD_PARAMETER,
    PEER_CONNECTION_RESULT_NO_FREE_TRANSCEIVER,
    PEER_CONNECTION_RESULT_FAIL_CREATE_TASK_ICE_CONTROLLER,
    PEER_CONNECTION_RESULT_FAIL_CREATE_TASK_ICE_SOCK_LISTENER,
    PEER_CONNECTION_RESULT_FAIL_CREATE_STARTUP_BARRIER,
    PEER_CONNECTION_RESULT_FAIL_SIGNAL_STARTUP_BARRIER,
    PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_INIT,
    PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_START,
    PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_ADD_REMOTE_CANDIDATE,
    PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_PROCESS_CANDIDATES_AND_PAIRS,
    PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_PERIOD_CONNECTION_CHECK,
    PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_ADDRESS_CLOSING,
    PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_DESTROY,
    PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_DESERIALIZE_CANDIDATE,
    PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_SEND_RTP_PACKET,
    PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_SEND_RTCP_PACKET,
    PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_RESEND_RTP_PACKET,
    PEER_CONNECTION_RESULT_FAIL_ICE_CONTROLLER_ADD_ICE_SERVER_CONFIG,
    PEER_CONNECTION_RESULT_FAIL_CREATE_CERT_AND_KEY,
    PEER_CONNECTION_RESULT_FAIL_CREATE_CERT_FINGERPRINT,
    PEER_CONNECTION_RESULT_FAIL_MQ_INIT,
    PEER_CONNECTION_RESULT_FAIL_MQ_SEND,
    PEER_CONNECTION_RESULT_FAIL_CREATE_SRTP_RX_SESSION,
    PEER_CONNECTION_RESULT_FAIL_CREATE_SRTP_TX_SESSION,
    PEER_CONNECTION_RESULT_FAIL_ENCRYPT_SRTP_RTP_PACKET,
    PEER_CONNECTION_RESULT_FAIL_ENCRYPT_SRTP_RTCP_PACKET,
    PEER_CONNECTION_RESULT_FAIL_DECRYPT_SRTP_RTP_PACKET,
    PEER_CONNECTION_RESULT_FAIL_RTP_INIT,
    PEER_CONNECTION_RESULT_FAIL_RTP_SERIALIZE,
    PEER_CONNECTION_RESULT_FAIL_RTP_DESERIALIZE,
    PEER_CONNECTION_RESULT_FAIL_RTP_RX_NO_MATCHING_SSRC,
    PEER_CONNECTION_RESULT_FAIL_RTCP_INIT,
    PEER_CONNECTION_RESULT_FAIL_RTCP_DESERIALIZE,
    PEER_CONNECTION_RESULT_FAIL_RTCP_PARSE_REMB,
    PEER_CONNECTION_RESULT_FAIL_RTCP_PARSE_NACK,
    PEER_CONNECTION_RESULT_FAIL_RTCP_PARSE_PLI,
    PEER_CONNECTION_RESULT_FAIL_RTCP_PARSE_SLI,
    PEER_CONNECTION_RESULT_FAIL_RTCP_PARSE_FIR,
    PEER_CONNECTION_RESULT_FAIL_RTCP_PARSE_SENDER_REPORT,
    PEER_CONNECTION_RESULT_FAIL_RTCP_SERIALIZE_SENDER_REPORT,
    PEER_CONNECTION_RESULT_FAIL_RTCP_PARSE_RECEIVER_REPORT,
    PEER_CONNECTION_RESULT_FAIL_RTCP_TWCC_INIT,
    PEER_CONNECTION_RESULT_FAIL_CREATE_TWCC_MUTEX,
    PEER_CONNECTION_RESULT_FAIL_TAKE_TWCC_MUTEX,
    PEER_CONNECTION_RESULT_FAIL_TIMER_INIT,
    PEER_CONNECTION_RESULT_FAIL_TIMER_RESET,
    PEER_CONNECTION_RESULT_FAIL_RTCP_PARSE_TWCC,
    PEER_CONNECTION_RESULT_FAIL_RTCP_HANDLE_TWCC,
    PEER_CONNECTION_RESULT_FAIL_CREATE_SENDER_MUTEX,
    PEER_CONNECTION_RESULT_FAIL_TAKE_SENDER_MUTEX,
    PEER_CONNECTION_RESULT_FAIL_CREATE_SRTP_MUTEX,
    PEER_CONNECTION_RESULT_FAIL_INIT_DTLS_SESSION,
    PEER_CONNECTION_RESULT_FAIL_TAKE_SRTP_MUTEX,
    PEER_CONNECTION_RESULT_FAIL_PACKET_INFO_NO_ENOUGH_MEMORY,
    PEER_CONNECTION_RESULT_FAIL_RTP_PACKET_QUEUE_INIT,
    PEER_CONNECTION_RESULT_FAIL_RTP_PACKET_QUEUE_RETRIEVE,
    PEER_CONNECTION_RESULT_FAIL_RTP_PACKET_ENQUEUE,
    PEER_CONNECTION_RESULT_FAIL_RTP_PACKET_NOT_FOUND,
    PEER_CONNECTION_RESULT_FAIL_PACKETIZER_INIT,
    PEER_CONNECTION_RESULT_FAIL_PACKETIZER_ADD_FRAME,
    PEER_CONNECTION_RESULT_FAIL_PACKETIZER_GET_PACKET,
    PEER_CONNECTION_RESULT_FAIL_DEPACKETIZER_INIT,
    PEER_CONNECTION_RESULT_FAIL_DEPACKETIZER_GET_PROPERTIES,
    PEER_CONNECTION_RESULT_FAIL_DEPACKETIZER_ADD_PACKET,
    PEER_CONNECTION_RESULT_FAIL_DEPACKETIZER_GET_FRAME,
    PEER_CONNECTION_RESULT_FAIL_JITTER_BUFFER_SEQ_NOT_FOUND,
    PEER_CONNECTION_RESULT_FAIL_SDP_DESERIALIZE_OFFER,
    PEER_CONNECTION_RESULT_FAIL_SDP_GET_PAYLOAD_TYPES,
    PEER_CONNECTION_RESULT_FAIL_SDP_SET_PAYLOAD_TYPE,
    PEER_CONNECTION_RESULT_FAIL_SDP_POPULATE_SINGLE_MEDIA_DESCRIPTION,
    PEER_CONNECTION_RESULT_FAIL_SDP_POPULATE_SESSION_DESCRIPTION,
    PEER_CONNECTION_RESULT_INVALID_REMOTE_USERNAME,
    PEER_CONNECTION_RESULT_INVALID_REMOTE_PASSWORD,
    PEER_CONNECTION_RESULT_UNKNOWN_SRTP_PROFILE,
    PEER_CONNECTION_RESULT_UNKNOWN_TX_CODEC,
    PEER_CONNECTION_RESULT_UNKNOWN_SSRC,
    PEER_CONNECTION_RESULT_UNKNOWN_SDP_TYPE,
    PEER_CONNECTION_RESULT_UNKNOWN_SDP_TRACK_KIND,
    PEER_CONNECTION_RESULT_UNKNOWN_CODEC,
    PEER_CONNECTION_RESULT_UNKNOWN_TRANSCEIVER,
    PEER_CONNECTION_RESULT_PACKET_OUTDATED,
    PEER_CONNECTION_RESULT_FAIL_SCTP_WRITE,
    PEER_CONNECTION_RESULT_FAIL_SCTP_READ,
    PEER_CONNECTION_RESULT_FAIL_SCTP_CLOSE,
} PeerConnectionResult_t;

/*
 * SDP relates data structures.
 */
typedef struct PeerConnectionBufferSessionDescription
{
    char * pSdpBuffer;
    size_t sdpBufferLength;
    SdpControllerMessageType_t type;
    SdpControllerSdpDescription_t sdpDescription;
} PeerConnectionBufferSessionDescription_t;

/*
 * On candidate ready data structures.
 */
typedef IceControllerLocalCandidateReadyMsg_t PeerConnectionIceLocalCandidate_t;

typedef void (* OnIceCandidateReadyCallback_t)( void * pCustomContext,
                                                PeerConnectionIceLocalCandidate_t * pIceLocalCandidate );

#if ENABLE_TWCC_SUPPORT
    typedef void ( * OnBandwidthEstimationCallback_t )( void * pCustomContext,
                                                        TwccBandwidthInfo_t * pTwccBandwidthInfo );
#endif

typedef void ( * OnPictureLossIndicationCallback_t )( void * pCustomContext,
                                                      RtcpPliPacket_t * pRtcpPliPacket );

/*
 * Media relates data structures.
 */
typedef struct PeerConnectionFrame
{
    uint32_t version;
    uint8_t * pData;
    size_t dataLength;
    uint64_t presentationUs;
} PeerConnectionFrame_t;

typedef struct PeerConnectionJitterBufferPacket PeerConnectionJitterBufferPacket_t;
typedef struct PeerConnectionJitterBuffer PeerConnectionJitterBuffer_t;

typedef PeerConnectionResult_t (* OnFrameReadyCallback_t)( void * pCustomContext,
                                                           PeerConnectionFrame_t * pFrame );
typedef PeerConnectionResult_t (* OnJitterBufferFrameReadyCallback_t)( void * pCustomContext,
                                                                       uint16_t startSequence,
                                                                       uint16_t endSequence );
typedef PeerConnectionResult_t (* OnJitterBufferFrameDropCallback_t)( void * pCustomContext,
                                                                      uint16_t startSequence,
                                                                      uint16_t endSequence );
typedef PeerConnectionResult_t (* GetPacketPropertyFunc_t)( PeerConnectionJitterBufferPacket_t * pPacket,
                                                            uint8_t * pIsStartPacket );
typedef PeerConnectionResult_t (* FillFrameFunc_t)( PeerConnectionJitterBuffer_t * pJitterBuffer,
                                                    uint16_t rtpSeqStart,
                                                    uint16_t rtpSeqEnd,
                                                    uint8_t * pOutBuffer,
                                                    size_t * pOutBufferLength,
                                                    uint32_t * pRtpTimestamp );

typedef struct PeerConnectionRollingBufferPacket
{
    RtpPacket_t rtpPacket;
    uint32_t twccExtensionPayload;
    uint8_t * pPacketBuffer;
    size_t packetBufferLength;
} PeerConnectionRollingBufferPacket_t;

typedef struct PeerConnectionRollingBuffer
{
    uint8_t isInit;
    RtpPacketQueue_t packetQueue;
    size_t maxSizePerPacket;
    size_t capacity; /* Buffer duration * highest expected bitrate (in bps) / 8 / maxPacketSize. */
} PeerConnectionRollingBuffer_t;

typedef struct PeerConnectionJitterBufferPacket
{
    uint8_t isPushed;
    uint16_t sequenceNumber;
    uint32_t rtpTimestamp;
    TickType_t receiveTick;
    uint8_t * pPacketBuffer;
    size_t packetBufferLength;
} PeerConnectionJitterBufferPacket_t;

typedef struct PeerConnectionJitterBuffer
{
    uint8_t isInit;
    uint8_t isStart; /* The jitter buffer starts to receive packet or not. */
    size_t capacity; /* The total number of packets that packet queue can store. */
    uint32_t clockRate; /* The clock rate based on the codec. For example: the clock rate is 90000 if the chosen RTP is H264/90000. */
    uint32_t codec; /* The codec. For example: the codec is set to H264 if the chosen RTP is H264/90000. */
    uint32_t tolerenceRtpTimeStamp; /* The buffer time in RTP time stamp format. */
    uint32_t lastPopRtpTimestamp; /* The timestamp in last pop RTP packet. */
    TickType_t lastPopTick; /* The receive time ticks in last pop RTP packet. */
    uint16_t lastPopSequenceNumber; /* The RTP sequence number in last pop RTP packet. */
    uint16_t oldestReceivedSequenceNumber; /* The oldest RTP sequence number that received in the packet queue. */
    uint16_t newestReceivedSequenceNumber; /* The newest RTP sequence number that received in the packet queue. */
    uint32_t newestReceivedTimestamp; /* The newest timestamp in packet queue. */
    PeerConnectionJitterBufferPacket_t rtpPackets[ PEER_CONNECTION_JITTER_BUFFER_MAX_ENTRY_NUM ]; /* The buffer for packet queue. */

    /* Callback functions & custom contexts. */
    OnJitterBufferFrameReadyCallback_t onFrameReadyCallbackFunc;
    void * pOnFrameReadyCallbackContext;
    OnJitterBufferFrameDropCallback_t onFrameDropCallbackFunc;
    void * pOnFrameDropCallbackContext;
    GetPacketPropertyFunc_t getPacketPropertyFunc;
    FillFrameFunc_t fillFrameFunc;
} PeerConnectionJitterBuffer_t;

/*
 * Session relates data structures.
 */
typedef enum PeerConnectionSessionRequestType
{
    PEER_CONNECTION_SESSION_REQUEST_TYPE_NONE = 0,
    PEER_CONNECTION_SESSION_REQUEST_TYPE_ADD_REMOTE_CANDIDATE,
    PEER_CONNECTION_SESSION_REQUEST_TYPE_PROCESS_ICE_CANDIDATES_AND_PAIRS,
    PEER_CONNECTION_SESSION_REQUEST_TYPE_PERIOD_CONNECTION_CHECK,
    PEER_CONNECTION_SESSION_REQUEST_TYPE_RTCP_SENDER_REPORT,
    PEER_CONNECTION_SESSION_REQUEST_TYPE_ICE_CLOSING,
    PEER_CONNECTION_SESSION_REQUEST_TYPE_ICE_CLOSED,
    PEER_CONNECTION_SESSION_REQUEST_TYPE_PEER_CONNECTION_CLOSE,
    PEER_CONNECTION_SESSION_REQUEST_TYPE_PEER_CONNECTION_CLOSE_NO_ICE_FLOW,
} PeerConnectionSessionRequestType_t;

typedef struct PeerConnectionSessionRequestMessage
{
    PeerConnectionSessionRequestType_t requestType;

    /* Decode the request message based on request type. */
    union
    {
        IceControllerCandidate_t remoteCandidate; /* PEER_CONNECTION_SESSION_REQUEST_TYPE_ADD_REMOTE_CANDIDATE */
        struct {
            uint64_t currentTimeUs;               /* PEER_CONNECTION_SESSION_REQUEST_TYPE_RTCP_SENDER_REPORT */
            const Transceiver_t * pTransceiver;
        } rtcpContent;
    } peerConnectionSessionRequestContent;
} PeerConnectionSessionRequestMessage_t;

typedef enum PeerConnectionSessionState
{
    PEER_CONNECTION_SESSION_STATE_NONE = 0,
    PEER_CONNECTION_SESSION_STATE_CLOSING,
    PEER_CONNECTION_SESSION_STATE_INITED,
    PEER_CONNECTION_SESSION_STATE_START,
    PEER_CONNECTION_SESSION_STATE_FIND_CONNECTION,
    PEER_CONNECTION_SESSION_STATE_P2P_CONNECTION_FOUND,
    PEER_CONNECTION_SESSION_STATE_CONNECTION_READY,
} PeerConnectionSessionState_t;

typedef struct PeerConnectionRtpConfig
{
    uint8_t isVideoCodecPayloadSet;
    uint8_t isAudioCodecPayloadSet;
    uint16_t videoSequenceNumber;
    uint16_t audioSequenceNumber;
    uint32_t videoCodecPayload;
    uint32_t audioCodecPayload;
    uint32_t videoCodecRtxPayload;
    uint32_t audioCodecRtxPayload;
    uint16_t videoRtxSequenceNumber;
    uint16_t audioRtxSequenceNumber;

    uint16_t twccId;
    uint16_t twccSequence;

    uint32_t remoteVideoSsrc;
    uint32_t remoteAudioSsrc;
} PeerConnectionRtpConfig_t;

typedef struct PeerConnectionSrtpSender
{
    /* RTP Tx rolling buffer. */
    PeerConnectionRollingBuffer_t txRollingBuffer;

    /* Mutex to protect sender info like rolling buffer. */
    SemaphoreHandle_t senderMutex;
    uint8_t isSenderMutexInit;
} PeerConnectionSrtpSender_t;

typedef struct PeerConnectionSrtpReceiver
{
    /* RTP Rx jitter buffer. */
    PeerConnectionJitterBuffer_t rxJitterBuffer;
    uint8_t frameBuffer[ PEER_CONNECTION_FRAME_BUFFER_SIZE ];

    OnFrameReadyCallback_t onFrameReadyCallbackFunc;
    void * pOnFrameReadyCallbackCustomContext;
} PeerConnectionSrtpReceiver_t;

#if ENABLE_TWCC_SUPPORT
    typedef struct PeerConnectionTwccMetaData
    {
        /* Mutex to protect updated Bitrate's because we might read the updated bitrate in between of updating the bitrate. */
        SemaphoreHandle_t twccBitrateMutex;
        uint64_t lastAdjustmentTimeUs;
        uint64_t currentVideoBitrate;
        uint64_t currentAudioBitrate;
        uint64_t updatedVideoBitrate;
        uint64_t updatedAudioBitrate;
        double averagePacketLoss;
    } PeerConnectionTwccMetaData_t;
#endif

typedef struct PeerConnectionContext PeerConnectionContext_t;
typedef struct PeerConnectionSession PeerConnectionSession_t;
typedef struct PeerConnectionDataChannel PeerConnectionDataChannel_t;

typedef void (* OnDataChannelMessageReceived_t)( PeerConnectionDataChannel_t * pDataChannel,
                                                 uint8_t isBinary,
                                                 uint8_t * pMessage,
                                                 uint32_t pMessageLen );

#if ENABLE_SCTP_DATA_CHANNEL
typedef struct PeerConnectionDataChannel
{
    uint8_t ucChannelActive;
    uint8_t ucChannelOpen;
    char ucDataChannelName[MAX_DATA_CHANNEL_NAME_LEN + 1];
    PeerConnectionSession_t * pPeerConnection;
    SctpDataChannel_t dataChannel;
    SctpDataChannelInitInfo_t dataChannelInitInfo;
    void * onMessageCustomData;
    void * onOpenCustomData;
    OnDataChannelMessageReceived_t onDataChannelMessage;
    struct PeerConnectionDataChannel * pxNext;
} PeerConnectionDataChannel_t;
#endif /* ENABLE_SCTP_DATA_CHANNEL */

typedef struct PeerConnectionSession
{
    volatile PeerConnectionSessionState_t state;

    TaskHandle_t * pTaskHandler;

    /* Task synchronization using eventgroups to block peer connection session until SetRemoteDescription completes.
     * That ensures ICE Controller processes candidates only after remote description is set, as ICE credentials
     * (username/password) are obtained from SDP. */
    EventGroupHandle_t startupBarrier;

    /* The remote user name, representing the remote peer, from SDP message. */
    char remoteUserName[ PEER_CONNECTION_USER_NAME_LENGTH + 1 ];
    /* The remote password, representing password of the remote peer, from SDP message. */
    char remotePassword[ PEER_CONNECTION_PASSWORD_LENGTH + 1 ];
    /* The combine name to respond back in SDP message.
     * Reserve 1 space for NULL terminator, the other one is for ':' between remote username & local username */
    char combinedName[ ( PEER_CONNECTION_USER_NAME_LENGTH << 1 ) + 2 ];
    /* The remote cert fingerprint from SDP message. */
    char remoteCertFingerprint[ PEER_CONNECTION_CERTIFICATE_FINGERPRINT_LENGTH + 1 ];
    size_t remoteCertFingerprintLength;

    IceControllerContext_t iceControllerContext;
    OnIceCandidateReadyCallback_t onIceCandidateReadyCallbackFunc;
    void * pOnLocalCandidateReadyCallbackCustomContext;

    /* Request queue. */
    MessageQueueHandler_t requestQueue;

    /* DTLS session. */
    DtlsSession_t dtlsSession;
    /* SRTP sessions. */
    SemaphoreHandle_t srtpSessionMutex;
    srtp_t srtpTransmitSession;
    srtp_t srtpReceiveSession;
    /* RTP config. */
    PeerConnectionRtpConfig_t rtpConfig;
    /* Store the original transceiver setting. */
    const Transceiver_t * pTransceivers[ PEER_CONNECTION_TRANSCEIVER_MAX_COUNT ];
    uint32_t transceiverCount;
    /* Store the transceiver sequence to match m-lines. */
    const Transceiver_t * pMLinesTransceivers[ PEER_CONNECTION_TRANSCEIVER_MAX_COUNT ];
    uint32_t mLinesTransceiverCount;
    /* Remote SDP description. */
    char remoteSdpBuffer[ PEER_CONNECTION_SDP_DESCRIPTION_BUFFER_MAX_LENGTH ];
    PeerConnectionBufferSessionDescription_t remoteSessionDescription;

    /* PLI callback and context */
    OnPictureLossIndicationCallback_t onPictureLossIndicationCallback;
    void * pPictureLossIndicationUserContext;

    #if ENABLE_SCTP_DATA_CHANNEL
    uint8_t ucEnableDataChannelLocal;
    uint8_t ucEnableDataChannelRemote;
    /* SCTP Session */
    SctpSession_t sctpSession;
    /* Data channel configs */
    PeerConnectionDataChannel_t * pDataChannels;
    uint32_t uKvsDataChannelCount;
    #endif /* ENABLE_SCTP_DATA_CHANNEL */

    PeerConnectionSrtpSender_t videoSrtpSender;
    PeerConnectionSrtpSender_t audioSrtpSender;
    PeerConnectionSrtpReceiver_t videoSrtpReceiver;
    PeerConnectionSrtpReceiver_t audioSrtpReceiver;

    TimerHandler_t rtcpAudioSenderReportTimer;
    TimerHandler_t rtcpVideoSenderReportTimer;
    TimerHandler_t closeSessionTimer;

    uint64_t dtlsHandshakingTimeoutMs;
    uint64_t inactiveConnectionTimeoutMs;

    #if ENABLE_TWCC_SUPPORT
    PeerConnectionTwccMetaData_t twccMetaData;
    #endif
    /* Pointer that points to peer connection context. */
    PeerConnectionContext_t * pCtx;
} PeerConnectionSession_t;

typedef struct PeerConnectionSessionConfiguration
{
    uint8_t canTrickleIce;
    IceControllerNatTraversalConfig_t natTraversalConfigBitmap;

    /* Provide Ice server list for peer connection. Note that the index 0 is for default STUN server,
     * and the following 5 for maximum Ice server list from SIGNALING_CONTROLLER_ICE_SERVER_MAX_CONFIG_COUNT. */
    IceControllerIceServer_t iceServers[ ICE_CONTROLLER_MAX_ICE_SERVER_COUNT ];
    size_t iceServersCount;

    char * pRootCaPath;
    size_t rootCaPathLength;
    char * pRootCaPem;
    size_t rootCaPemLength;
} PeerConnectionSessionConfiguration_t;

/*
 * Peer connection general instances.
 */
typedef struct PeerConnectionDtlsContext
{
    uint8_t isInitialized;
    mbedtls_x509_crt localCert;
    mbedtls_pk_context localKey;
    char localCertFingerprint[CERTIFICATE_FINGERPRINT_LENGTH];
    unsigned char privateKeyPcsPem[PRIVATE_KEY_PCS_PEM_SIZE];
} PeerConnectionDtlsContext_t;

typedef struct PeerConnectionContext
{
    uint8_t isInited;

    char localUserName[ PEER_CONNECTION_USER_NAME_LENGTH + 1 ];
    char localPassword[ PEER_CONNECTION_PASSWORD_LENGTH + 1 ];
    char localCname[ PEER_CONNECTION_CNAME_LENGTH + 1 ];

    /* DTLS cert/key/fingerprint. */
    PeerConnectionDtlsContext_t dtlsContext;
    RtpContext_t rtpContext;
    RtcpContext_t rtcpContext;

    #if ENABLE_TWCC_SUPPORT
    RtcpTwccManager_t rtcpTwccManager;
    TwccPacketInfo_t twccPacketInfo[ PEER_CONNECTION_RTCP_TWCC_MAX_ARRAY ];

    /* Callback for bandwidth estimation updates */
    OnBandwidthEstimationCallback_t onBandwidthEstimationCallback;
    void * pOnBandwidthEstimationCallbackContext;
    #endif /* ENABLE_TWCC_SUPPORT */
} PeerConnectionContext_t;

#ifdef __cplusplus
}
#endif

#endif /* PEER_CONNECTION_DATA_TYPES_H */
