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

#ifndef SDP_CONTROLLER_DATA_TYPES_H
#define SDP_CONTROLLER_DATA_TYPES_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Standard includes. */
#include <stdint.h>
#include "transceiver_data_types.h"

#define SDP_CONTROLLER_MAX_SDP_SESSION_TIMING_COUNT ( 2 )
#define SDP_CONTROLLER_MAX_SDP_SESSION_TIMEZONE_COUNT ( 2 )
#define SDP_CONTROLLER_MAX_SDP_ATTRIBUTES_COUNT ( 255 )
#define SDP_CONTROLLER_MAX_SDP_MEDIA_DESCRIPTIONS_COUNT ( 5 )

typedef enum SdpControllerResult
{
    SDP_CONTROLLER_RESULT_OK = 0,
    SDP_CONTROLLER_RESULT_BAD_PARAMETER,
    SDP_CONTROLLER_RESULT_SDP_FAIL_DESERIALIZER_INIT,
    SDP_CONTROLLER_RESULT_SDP_FAIL_DESERIALIZER_PARSE_ATTRIBUTE,
    SDP_CONTROLLER_RESULT_SDP_FAIL_SERIALIZER_INIT,
    SDP_CONTROLLER_RESULT_SDP_FAIL_SERIALIZER_ADD,
    SDP_CONTROLLER_RESULT_SDP_FAIL_SNPRINTF,
    SDP_CONTROLLER_RESULT_SDP_FAIL_NO_FINGERPRINT_FOUND,
    SDP_CONTROLLER_RESULT_SDP_FAIL_NO_ICE_UFRAG_FOUND,
    SDP_CONTROLLER_RESULT_SDP_FAIL_NO_ICE_PWD_FOUND,
    SDP_CONTROLLER_RESULT_SDP_FAIL_CODEC_NOT_SUPPORT,
    SDP_CONTROLLER_RESULT_SDP_FAIL_ADD_SESSION_ATTRIBUTE_GROUP,
    SDP_CONTROLLER_RESULT_SDP_FAIL_ADD_SESSION_ATTRIBUTE_ICE_OPTIONS,
    SDP_CONTROLLER_RESULT_SDP_FAIL_ADD_SESSION_ATTRIBUTE_MSID_SEMANTIC,
    SDP_CONTROLLER_RESULT_SDP_SESSION_ATTRIBUTE_MAX_EXCEDDED,
    SDP_CONTROLLER_RESULT_SDP_MEDIA_ATTRIBUTE_MAX_EXCEDDED,
    SDP_CONTROLLER_RESULT_SDP_INVALID_VERSION,
    SDP_CONTROLLER_RESULT_SDP_INVALID_TWCC_ID,
    SDP_CONTROLLER_RESULT_SDP_CONVERTED_BUFFER_TOO_SMALL,
    SDP_CONTROLLER_RESULT_SDP_POPULATE_BUFFER_TOO_SMALL,
} SdpControllerResult_t;

typedef enum SdpControllerDtlsRole
{
    SDP_CONTROLLER_DTLS_ROLE_NONE = 0,
    SDP_CONTROLLER_DTLS_ROLE_ACTIVE,
    SDP_CONTROLLER_DTLS_ROLE_ACTPASS,
} SdpControllerDtlsRole_t;

/*
 * c=<nettype> <addrtype> <connection-address>
 * https://tools.ietf.org/html/rfc4566#section-5.7
 */
typedef struct SdpControllerConnectionInformation
{
    const char * pNetworkType;
    size_t networkTypeLength;
    const char * pAddressType;
    size_t addressTypeLength;
    const char * pConnectionAddress;
    size_t connectionAddressLength;
} SdpControllerConnectionInformation_t;

/*
 * o=<username> <sess-id> <sess-version> <nettype> <addrtype> <unicast-address>
 * https://tools.ietf.org/html/rfc4566#section-5.2
 */
typedef struct SdpControllerOrigin
{
    const char * pUserName;
    size_t userNameLength;
    uint64_t sessionId;
    uint64_t sessionVersion;
    SdpControllerConnectionInformation_t sdpConnectionInformation;
} SdpControllerOrigin_t;

/*
 * https://tools.ietf.org/html/rfc4566#section-5.9
 * https://tools.ietf.org/html/rfc4566#section-5.10
 */
typedef struct SdpControllerTiming
{
    uint64_t startTime;
    uint64_t stopTime;
} SdpControllerTiming_t;

/*
 * z=<adjustment time> <offset> <adjustment time> <offset> ...
 * https://tools.ietf.org/html/rfc4566#section-5.11
 */
typedef struct SdpControllerTimeZone
{
    uint64_t adjustmentTime;
    const char * pOffset;
    size_t offsetLength;
} SdpControllerTimeZone_t;

/*
 * a=<attribute>
 * a=<attribute>:<value>
 * https://tools.ietf.org/html/rfc4566#section-5.13
 */
typedef struct SdpControllerAttributes
{
    const char * pAttributeName;
    size_t attributeNameLength;
    const char * pAttributeValue;
    size_t attributeValueLength;
} SdpControllerAttributes_t;

typedef struct SdpControllerMediaDescription
{
    // m=<media> <port>/<number of ports> <proto> <fmt> ...
    // https://tools.ietf.org/html/rfc4566#section-5.14
    const char * pMediaName;
    size_t mediaNameLength;

    // i=<session description>
    // https://tools.ietf.org/html/rfc4566#section-5.4. Given these are free-form strings, the length could be anything.
    // Although our SDK parses this information, the SDK does not use it. Leaving this attribute in if SDK uses it in
    // the future
    const char * pMediaTitle;
    size_t mediaTitleLength;

    SdpControllerConnectionInformation_t connectionInformation;

    SdpControllerAttributes_t attributes[ SDP_CONTROLLER_MAX_SDP_ATTRIBUTES_COUNT ];

    uint8_t mediaAttributesCount;
} SdpControllerMediaDescription_t;

typedef struct SdpControllerQuickAccess
{
    const char * pFingerprint;
    size_t fingerprintLength;
    SdpControllerDtlsRole_t dtlsRole;
    uint8_t isIceTrickle;
    const char * pIceUfrag;
    size_t iceUfragLength;
    const char * pIcePwd;
    size_t icePwdLength;
    uint32_t twccExtId;
    uint8_t isVideoCodecPayloadSet;
    uint8_t isAudioCodecPayloadSet;
    uint32_t videoCodecPayload;
    uint32_t audioCodecPayload;
    uint32_t videoCodecRtxPayload;
    uint32_t audioCodecRtxPayload;
    uint32_t videoSsrc;
    uint32_t audioSsrc;
    const char * pRemoteCandidates[ SDP_CONTROLLER_MAX_SDP_ATTRIBUTES_COUNT ];
    size_t remoteCandidateLengths[ SDP_CONTROLLER_MAX_SDP_ATTRIBUTES_COUNT ];
    uint8_t remoteCandidateCount;
} SdpControllerQuickAccess_t;

typedef struct SdpControllerSdpOffer
{
    // https://tools.ietf.org/html/rfc4566#section-5.1
    uint32_t version;

    SdpControllerOrigin_t origin;

    // s=<session name>
    // https://tools.ietf.org/html/rfc4566#section-5.3
    const char * pSessionName;
    size_t sessionNameLength;

    // i=<session description>
    // https://tools.ietf.org/html/rfc4566#section-5.4
    const char * pSessionInformation;
    size_t sessionInformationLength;

    // u=<uri>
    // https://tools.ietf.org/html/rfc4566#section-5.5
    const char * pUri;
    size_t uriLength;

    // e=<email-address>
    // https://tools.ietf.org/html/rfc4566#section-5.6
    const char * pEmailAddress;
    size_t emailAddressLength;

    // p=<phone-number>
    // https://tools.ietf.org/html/rfc4566#section-5.6
    const char * pPhoneNumber;
    size_t phoneNumberLength;

    SdpControllerConnectionInformation_t connectionInformation;

    SdpControllerTiming_t timingDescription;

    SdpControllerAttributes_t attributes[ SDP_CONTROLLER_MAX_SDP_ATTRIBUTES_COUNT ];

    SdpControllerMediaDescription_t mediaDescriptions[ SDP_CONTROLLER_MAX_SDP_MEDIA_DESCRIPTIONS_COUNT ];

    uint16_t sessionAttributesCount;

    uint16_t mediaCount;

    /* Below is extra info to accerlate SDP creation and provide some info for peer connection creation. */
    SdpControllerQuickAccess_t quickAccess;
} SdpControllerSdpDescription_t;

typedef enum SdpControllerMessageType
{
    SDP_CONTROLLER_MESSAGE_TYPE_NONE = 0,
    SDP_CONTROLLER_MESSAGE_TYPE_OFFER,
    SDP_CONTROLLER_MESSAGE_TYPE_ANSWER,
} SdpControllerMessageType_t;

typedef struct SdpControllerPopulateMediaConfiguration
{
    /* Basic configurations. */
    uint8_t isOffer; /* 0 for answer, 1 for offer. */
    uint8_t canTrickleIce;

    /* ICE information. */
    const char * pCname;
    size_t cnameLength;
    const char * pUserName; /* For ice-ufrag in SDP attributes */
    size_t userNameLength;
    const char * pPassword; /* For ice-pwd in SDP attributes */
    size_t passwordLength;

    /* Transceiver. */
    const Transceiver_t * pTransceiver;
    uint32_t payloadType;
    uint32_t rtxPayloadType;

    /* Fingerprint. */
    const char * pLocalFingerprint;
    size_t localFingerprintLength;

    /* TWCC EXT ID */
    uint16_t twccExtId;
} SdpControllerPopulateMediaConfiguration_t;

typedef struct SdpControllerPopulateSessionConfiguration
{
    /* Basic configurations. */
    uint8_t isOffer; /* 0 for answer, 1 for offer. */
    uint8_t canTrickleIce;

    /* Start/Stop time. */
    SdpControllerTiming_t timingDescription;
} SdpControllerPopulateSessionConfiguration_t;

#ifdef __cplusplus
}
#endif

#endif /* SDP_CONTROLLER_DATA_TYPES_H */
