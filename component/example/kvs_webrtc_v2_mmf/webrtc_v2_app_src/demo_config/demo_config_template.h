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

#ifndef DEMO_CONFIG_H
#define DEMO_CONFIG_H

#define AWS_REGION "us-west-2"

#define AWS_KVS_CHANNEL_NAME ""

#define AWS_KVS_AGENT_NAME "AWS-SDK-KVS"

#ifndef ENABLE_TWCC_SUPPORT
#define ENABLE_TWCC_SUPPORT 1U
#endif

/* Uncomment to use fetching credentials by IoT Role-alias for Authentication. */
/* Please add the AWS_IOT_THING_CERT and AWS_IOT_THING_PRIVATE_KEY similar to the AWS_CA_CERT_PEM. */

// #define AWS_CREDENTIALS_ENDPOINT ""
// #define AWS_IOT_THING_NAME ""
// #define AWS_IOT_THING_ROLE_ALIAS ""
// #define AWS_IOT_THING_CERT ""
// #define AWS_IOT_THING_PRIVATE_KEY ""

/* Uncomment to use AWS Access Key Credentials for Authentication. */
// #define AWS_ACCESS_KEY_ID ""
// #define AWS_SECRET_ACCESS_KEY ""
// #define AWS_SESSION_TOKEN ""

#if defined( AWS_ACCESS_KEY_ID ) && defined( AWS_IOT_THING_ROLE_ALIAS )
#error "Configuration Error: AWS_ACCESS_KEY_ID and AWS_IOT_THING_ROLE_ALIAS are mutually exclusive authentication methods. Please define only one of them."
#endif /* #if defined( AWS_ACCESS_KEY_ID ) && defined( AWS_IOT_THING_ROLE_ALIAS ). */

#define AWS_CA_CERT_PEM \
    "-----BEGIN CERTIFICATE-----\n"\
    "MIID7zCCAtegAwIBAgIBADANBgkqhkiG9w0BAQsFADCBmDELMAkGA1UEBhMCVVMx\n"\
    "EDAOBgNVBAgTB0FyaXpvbmExEzARBgNVBAcTClNjb3R0c2RhbGUxJTAjBgNVBAoT\n"\
    "HFN0YXJmaWVsZCBUZWNobm9sb2dpZXMsIEluYy4xOzA5BgNVBAMTMlN0YXJmaWVs\n"\
    "ZCBTZXJ2aWNlcyBSb290IENlcnRpZmljYXRlIEF1dGhvcml0eSAtIEcyMB4XDTA5\n"\
    "MDkwMTAwMDAwMFoXDTM3MTIzMTIzNTk1OVowgZgxCzAJBgNVBAYTAlVTMRAwDgYD\n"\
    "VQQIEwdBcml6b25hMRMwEQYDVQQHEwpTY290dHNkYWxlMSUwIwYDVQQKExxTdGFy\n"\
    "ZmllbGQgVGVjaG5vbG9naWVzLCBJbmMuMTswOQYDVQQDEzJTdGFyZmllbGQgU2Vy\n"\
    "dmljZXMgUm9vdCBDZXJ0aWZpY2F0ZSBBdXRob3JpdHkgLSBHMjCCASIwDQYJKoZI\n"\
    "hvcNAQEBBQADggEPADCCAQoCggEBANUMOsQq+U7i9b4Zl1+OiFOxHz/Lz58gE20p\n"\
    "OsgPfTz3a3Y4Y9k2YKibXlwAgLIvWX/2h/klQ4bnaRtSmpDhcePYLQ1Ob/bISdm2\n"\
    "8xpWriu2dBTrz/sm4xq6HZYuajtYlIlHVv8loJNwU4PahHQUw2eeBGg6345AWh1K\n"\
    "Ts9DkTvnVtYAcMtS7nt9rjrnvDH5RfbCYM8TWQIrgMw0R9+53pBlbQLPLJGmpufe\n"\
    "hRhJfGZOozptqbXuNC66DQO4M99H67FrjSXZm86B0UVGMpZwh94CDklDhbZsc7tk\n"\
    "6mFBrMnUVN+HL8cisibMn1lUaJ/8viovxFUcdUBgF4UCVTmLfwUCAwEAAaNCMEAw\n"\
    "DwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMCAQYwHQYDVR0OBBYEFJxfAN+q\n"\
    "AdcwKziIorhtSpzyEZGDMA0GCSqGSIb3DQEBCwUAA4IBAQBLNqaEd2ndOxmfZyMI\n"\
    "bw5hyf2E3F/YNoHN2BtBLZ9g3ccaaNnRbobhiCPPE95Dz+I0swSdHynVv/heyNXB\n"\
    "ve6SbzJ08pGCL72CQnqtKrcgfU28elUSwhXqvfdqlS5sdJ/PHLTyxQGjhdByPq1z\n"\
    "qwubdQxtRbeOlKyWN7Wg0I8VRw7j6IPdj/3vQQF3zCepYoUz8jcI73HPdwbeyBkd\n"\
    "iEDPfUYd/x7H4c7/I9vG+o1VTqkC50cRRj70/b17KSa7qWFiNyi2LSr2EIZkyXCn\n"\
    "0q23KXB56jzaYyWf/Wi3MOxw+3WKt21gZ7IeyLnp2KhvAotnDU0mV3HaIPzBSlCN\n"\
    "sSi6\n"\
    "-----END CERTIFICATE-----\n"

#define AWS_MAX_VIEWER_NUM ( 2 )

/* Audio codec setting. */
#define AUDIO_G711_MULAW 1
#define AUDIO_G711_ALAW 0
#define AUDIO_OPUS 0
#if ( AUDIO_G711_MULAW + AUDIO_G711_ALAW + AUDIO_OPUS ) != 1
#error only one of audio format should be set
#endif
/* Enable audio receive flow to deliver received audio frames to output. */
#define MEDIA_PORT_ENABLE_AUDIO_RECV ( 1 )

/* Video codec setting. */
#define USE_VIDEO_CODEC_H264 1
#define USE_VIDEO_CODEC_H265 0
#if ( USE_VIDEO_CODEC_H264 + USE_VIDEO_CODEC_H265 ) != 1
    #error only one of video codec should be set
#endif

/* Join Storage Session setting. */
#ifndef JOIN_STORAGE_SESSION
    #define JOIN_STORAGE_SESSION 0
#endif

#endif /* DEMO_CONFIG_H */
