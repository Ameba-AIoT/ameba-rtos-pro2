/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include <FreeRTOS.h>
#include <task.h>
#include "platform_opts.h"
#include <platform_stdlib.h>

#ifndef CONFIG_PLATFORM_8735B
#include "platform_autoconf.h"
#endif
#include <cJSON.h>
#include "audio_cjson_generater.h"

static void Update_ItemInObject(cJSON *JSObject, const char *key_string, cJSON *item)
{
	if (cJSON_GetObjectItem(JSObject, key_string)) {
		cJSON_ReplaceItemInObject(JSObject, key_string, item);
	} else {
		cJSON_AddItemToObject(JSObject, key_string, item);
	}
}

static const char *samplerate2string(audio_sr samplerate)
{
	switch (samplerate) {
	case ASR_8KHZ:
		return "ASR_8KHZ";
		break;
	case ASR_16KHZ:
		return "ASR_16KHZ";
		break;
	case ASR_32KHZ:
		return "ASR_32KHZ";
		break;
	case ASR_48KHZ:
		return "ASR_48KHZ";
		break;
	case ASR_88p2KHZ:
		return "ASR_88p2KHZ";
		break;
	case ASR_96KHZ:
		return "ASR_96KHZ";
		break;
	default:
		return "UNKNOWN";
		break;
	}
}

static const char *wordlength2string(audio_wl wordlength)
{
	switch (wordlength) {
	case WL_16BIT:
		return "WL_16BIT";
		break;
	case WL_24BIT:
		return "WL_24BIT";
		break;
	default:
		return "UNKNOWN";
		break;
	}
}

static const char *micgain2string(audio_mic_gain micgain)
{
	switch (micgain) {
	case MIC_0DB:
		return "MIC_0DB";
		break;
	case MIC_20DB:
		return "MIC_20DB";
		break;
	case MIC_30DB:
		return "MIC_30DB";
		break;
	case MIC_40DB:
		return "MIC_40DB";
		break;
	default:
		return "UNKNOWN";
		break;
	}
}

static const char *dmicgain2string(audio_dmic_gain dmicgain)
{
	switch (dmicgain) {
	case DMIC_BOOST_0DB:
		return "DMIC_BOOST_0DB";
		break;
	case DMIC_BOOST_12DB:
		return "DMIC_BOOST_12DB";
		break;
	case DMIC_BOOST_24DB:
		return "DMIC_BOOST_24DB";
		break;
	case DMIC_BOOST_36DB:
		return "DMIC_BOOST_36DB";
		break;
	default:
		return "UNKNOWN";
		break;
	}
}

static const char *usemictype2string(audio_mic_type usemictype)
{

	switch (usemictype) {
	case USE_AUDIO_AMIC:
		return "USE_AUDIO_AMIC";
		break;
	case USE_AUDIO_LEFT_DMIC:
		return "USE_AUDIO_LEFT_DMIC";
		break;
	case USE_AUDIO_RIGHT_DMIC:
		return "USE_AUDIO_RIGHT_DMIC";
		break;
	case USE_AUDIO_STEREO_DMIC:
		return "USE_AUDIO_STEREO_DMIC";
		break;
	default:
		return "UNKNOWN";
		break;
	}
}

static void Int2HexString(char *hex, int number, int buflen)
{
	memset(hex, 0, buflen);
	snprintf(hex, buflen, "0x%02x", number);
}

static void Int2DecString(char *dec, int number, int buflen)
{
	memset(dec, 0, buflen);
	snprintf(dec, buflen, "%d", number);
}

#define EQ_NAME_LEN 16
#define EQ_HEX_LEN 16
static cJSON *EQArraytoJSItem(eq_cof_t EQ_Array)
{
	cJSON *EQItemJSArray = NULL;
	if (EQ_Array.eq_enable) {
		char *EQ_hex = malloc(EQ_HEX_LEN);
		EQItemJSArray = cJSON_CreateArray();

		cJSON_AddItemToArray(EQItemJSArray, cJSON_CreateNumber(EQ_Array.eq_enable));
		Int2HexString(EQ_hex, EQ_Array.eq_h0, EQ_HEX_LEN);
		cJSON_AddItemToArray(EQItemJSArray, cJSON_CreateString((const char *)EQ_hex));
		Int2HexString(EQ_hex, EQ_Array.eq_b1, EQ_HEX_LEN);
		cJSON_AddItemToArray(EQItemJSArray, cJSON_CreateString((const char *)EQ_hex));
		Int2HexString(EQ_hex, EQ_Array.eq_b2, EQ_HEX_LEN);
		cJSON_AddItemToArray(EQItemJSArray, cJSON_CreateString((const char *)EQ_hex));
		Int2HexString(EQ_hex, EQ_Array.eq_a1, EQ_HEX_LEN);
		cJSON_AddItemToArray(EQItemJSArray, cJSON_CreateString((const char *)EQ_hex));
		Int2HexString(EQ_hex, EQ_Array.eq_a2, EQ_HEX_LEN);
		cJSON_AddItemToArray(EQItemJSArray, cJSON_CreateString((const char *)EQ_hex));
	}

	return EQItemJSArray;
}

static void UpdateEQ(cJSON *AUDPARAM_JSObject, const char *EQ_String, eq_cof_t *EQ_Array, int numofarray)
{
	char *EQ_Name = malloc(EQ_NAME_LEN);
	cJSON *EQItemJSArray = NULL;

	for (int i = 0; i < numofarray; i++) {
		EQItemJSArray = EQArraytoJSItem(EQ_Array[i]);
		if (EQItemJSArray) {
			memset(EQ_Name, 0, EQ_NAME_LEN);
			snprintf(EQ_Name, EQ_NAME_LEN, "%s[%d]", EQ_String, i);
			Update_ItemInObject(AUDPARAM_JSObject, (const char *)EQ_Name, EQItemJSArray);
		}
	}
	free(EQ_Name);
}

#define HEX_BUF_LEN 12
#define DEC_BUF_LEN 12
static void Update_AudioParam(cJSON *AUDPARAM_JSObject, audio_params_t audio_save_params)
{
	if (AUDPARAM_JSObject) {
		Update_ItemInObject(AUDPARAM_JSObject, "sample_rate", cJSON_CreateString(samplerate2string(audio_save_params.sample_rate)));
		Update_ItemInObject(AUDPARAM_JSObject, "word_length", cJSON_CreateString(wordlength2string(audio_save_params.word_length)));
		Update_ItemInObject(AUDPARAM_JSObject, "mic_gain", cJSON_CreateString(micgain2string(audio_save_params.mic_gain)));
		Update_ItemInObject(AUDPARAM_JSObject, "dmic_l_gain", cJSON_CreateString(dmicgain2string(audio_save_params.dmic_l_gain)));
		Update_ItemInObject(AUDPARAM_JSObject, "dmic_r_gain", cJSON_CreateString(dmicgain2string(audio_save_params.dmic_r_gain)));
		Update_ItemInObject(AUDPARAM_JSObject, "use_mic_type", cJSON_CreateString(usemictype2string(audio_save_params.use_mic_type)));

		Update_ItemInObject(AUDPARAM_JSObject, "channel", cJSON_CreateNumber(audio_save_params.channel));
		Update_ItemInObject(AUDPARAM_JSObject, "mix_mode", cJSON_CreateNumber(audio_save_params.mix_mode));
		Update_ItemInObject(AUDPARAM_JSObject, "mic_bias", cJSON_CreateNumber(audio_save_params.mic_bias));
		Update_ItemInObject(AUDPARAM_JSObject, "hpf_set", cJSON_CreateNumber(audio_save_params.hpf_set));

		char *hex_buf = malloc(HEX_BUF_LEN);
		Int2HexString(hex_buf, audio_save_params.ADC_gain, HEX_BUF_LEN);
		Update_ItemInObject(AUDPARAM_JSObject, "ADC_gain", cJSON_CreateString((const char *)hex_buf));
		Int2HexString(hex_buf, audio_save_params.DAC_gain, HEX_BUF_LEN);
		Update_ItemInObject(AUDPARAM_JSObject, "DAC_gain", cJSON_CreateString((const char *)hex_buf));
		Int2HexString(hex_buf, audio_save_params.ADC_mute, HEX_BUF_LEN);
		Update_ItemInObject(AUDPARAM_JSObject, "ADC_mute", cJSON_CreateString((const char *)hex_buf));
		Int2HexString(hex_buf, audio_save_params.DAC_mute, HEX_BUF_LEN);
		Update_ItemInObject(AUDPARAM_JSObject, "DAC_mute", cJSON_CreateString((const char *)hex_buf));
		free(hex_buf);

		UpdateEQ(AUDPARAM_JSObject, "mic_l_eq", audio_save_params.mic_l_eq, 5);
		UpdateEQ(AUDPARAM_JSObject, "mic_r_eq", audio_save_params.mic_r_eq, 5);
		UpdateEQ(AUDPARAM_JSObject, "spk_l_eq", audio_save_params.spk_l_eq, 5);
	}
}

#if defined(CONFIG_PLATFORM_8735B) && defined(CONFIG_NEWAEC) && CONFIG_NEWAEC
static const char *AGCMode2string(CT_AGC_MODE AGCMode)
{

	switch (AGCMode) {
	case CT_ALC:
		return "CT_ALC";
		break;
	case CT_LIMITER:
		return "CT_LIMITER";
		break;
	default:
		return "UNKNOWN";
		break;
	}
}

static cJSON *CreateAGCJSArray(int16_t *Ratio, int numofratio)
{
	cJSON *AGCJSArray = cJSON_CreateArray();
	if (AGCJSArray) {
		for (int i = 0; i < numofratio; i++) {
			cJSON_AddItemToArray(AGCJSArray, cJSON_CreateNumber(Ratio[i]));
		}
	}
	return AGCJSArray;
}

static cJSON *CreateAECJSObject(CTAEC_cfg_t aec_cfg)
{
	cJSON *AECJSObject = cJSON_CreateObject();
	if (AECJSObject) {
		Update_ItemInObject(AECJSObject, "AEC_EN", cJSON_CreateNumber(aec_cfg.AEC_EN));
		Update_ItemInObject(AECJSObject, "EchoTailLen", cJSON_CreateNumber(aec_cfg.EchoTailLen));
		Update_ItemInObject(AECJSObject, "CNGEnable", cJSON_CreateNumber(aec_cfg.CNGEnable));
		Update_ItemInObject(AECJSObject, "PPLevel", cJSON_CreateNumber(aec_cfg.PPLevel));
		Update_ItemInObject(AECJSObject, "DTControl", cJSON_CreateNumber(aec_cfg.DTControl));
		Update_ItemInObject(AECJSObject, "ConvergenceTime", cJSON_CreateNumber(aec_cfg.ConvergenceTime));
	}
	return AECJSObject;
}

static cJSON *CreateAGCJSObject(CTAGC_cfg_t agc_cfg)
{
	cJSON *AGCJSObject = cJSON_CreateObject();
	if (AGCJSObject) {
		Update_ItemInObject(AGCJSObject, "AGC_EN", cJSON_CreateNumber(agc_cfg.AGC_EN));
		Update_ItemInObject(AGCJSObject, "AGCMode", cJSON_CreateString(AGCMode2string(agc_cfg.AGCMode)));
		Update_ItemInObject(AGCJSObject, "ReferenceLvl", cJSON_CreateNumber(agc_cfg.ReferenceLvl));
		Update_ItemInObject(AGCJSObject, "RatioFormat", cJSON_CreateNumber(agc_cfg.RatioFormat));
		Update_ItemInObject(AGCJSObject, "AttackTime", cJSON_CreateNumber(agc_cfg.AttackTime));
		Update_ItemInObject(AGCJSObject, "ReleaseTime", cJSON_CreateNumber(agc_cfg.ReleaseTime));

		Update_ItemInObject(AGCJSObject, "Ratio", CreateAGCJSArray(agc_cfg.Ratio, 3));
		Update_ItemInObject(AGCJSObject, "Threshold", CreateAGCJSArray(agc_cfg.Threshold, 3));

		Update_ItemInObject(AGCJSObject, "KneeWidth", cJSON_CreateNumber(agc_cfg.KneeWidth));
		Update_ItemInObject(AGCJSObject, "NoiseFloorAdaptEnable", cJSON_CreateNumber(agc_cfg.NoiseFloorAdaptEnable));
		Update_ItemInObject(AGCJSObject, "RMSDetectorEnable", cJSON_CreateNumber(agc_cfg.RMSDetectorEnable));
		Update_ItemInObject(AGCJSObject, "MaxGainLimit", cJSON_CreateNumber(agc_cfg.MaxGainLimit));
	}
	return AGCJSObject;
}

static cJSON *CreateNSJSObject(CTNS_cfg_t ns_cfg)
{
	cJSON *NSJSObject = cJSON_CreateObject();
	if (NSJSObject) {
		Update_ItemInObject(NSJSObject, "NS_EN", cJSON_CreateNumber(ns_cfg.NS_EN));
		Update_ItemInObject(NSJSObject, "NSLevel", cJSON_CreateNumber(ns_cfg.NSLevel));
		Update_ItemInObject(NSJSObject, "HPFEnable", cJSON_CreateNumber(ns_cfg.HPFEnable));
		Update_ItemInObject(NSJSObject, "NSSlowConvergence", cJSON_CreateNumber(ns_cfg.NSSlowConvergence));
	}
	return NSJSObject;
}
#else
static const char *aec_core2string(AEC_CORE aec_core)
{
	switch (aec_core) {
	case SPEEX_AEC:
		return "SPEEX_AEC";
		break;
	case WEBRTC_AEC:
		return "WEBRTC_AEC";
		break;
	case WEBRTC_AECM:
		return "WEBRTC_AECM";
		break;
	default:
		return "UNKNOWN";
		break;
	}
}

static cJSON *CreateAECJSObject(WebrtcAEC_cfg_t aec_cfg)
{
	cJSON *AECJSObject = cJSON_CreateObject();
	if (AECJSObject) {
		//AEC parameters
		Update_ItemInObject(AECJSObject, "AEC_EN", cJSON_CreateNumber(aec_cfg.AEC_EN));
		Update_ItemInObject(AECJSObject, "aec_core", cJSON_CreateString(aec_core2string(aec_cfg.aec_core)));
		Update_ItemInObject(AECJSObject, "FilterLength", cJSON_CreateNumber(aec_cfg.FilterLength));
		Update_ItemInObject(AECJSObject, "AECLevel", cJSON_CreateNumber(aec_cfg.AECLevel));
		Update_ItemInObject(AECJSObject, "CNGEnable", cJSON_CreateNumber(aec_cfg.CNGEnable));
		//AGC parameters for AEC
		Update_ItemInObject(AECJSObject, "AGC_EN", cJSON_CreateNumber(aec_cfg.AGC_EN));
		Update_ItemInObject(AECJSObject, "AGCMode", cJSON_CreateNumber(aec_cfg.AGCMode));
		Update_ItemInObject(AECJSObject, "TargetLevelDbfs", cJSON_CreateNumber(aec_cfg.TargetLevelDbfs));
		Update_ItemInObject(AECJSObject, "CompressionGaindB", cJSON_CreateNumber(aec_cfg.CompressionGaindB));
		Update_ItemInObject(AECJSObject, "LimiterEnable", cJSON_CreateNumber(aec_cfg.LimiterEnable));
		//NS parameters
		Update_ItemInObject(AECJSObject, "NS_EN", cJSON_CreateNumber(aec_cfg.NS_EN));
		Update_ItemInObject(AECJSObject, "NSLevel", cJSON_CreateNumber(aec_cfg.NSLevel));

		//howling
		Update_ItemInObject(AECJSObject, "HOWL_EN", cJSON_CreateNumber(aec_cfg.HOWL_EN));
		//AGC parameters for howling
		Update_ItemInObject(AECJSObject, "HOWL_AGC_EN", cJSON_CreateNumber(aec_cfg.HOWL_AGC_EN));
		Update_ItemInObject(AECJSObject, "HOWL_AGCMode", cJSON_CreateNumber(aec_cfg.HOWL_AGCMode));
		Update_ItemInObject(AECJSObject, "HOWL_TargetLevelDbfs", cJSON_CreateNumber(aec_cfg.HOWL_TargetLevelDbfs));
		Update_ItemInObject(AECJSObject, "HOWL_CompressionGaindB", cJSON_CreateNumber(aec_cfg.HOWL_CompressionGaindB));
		Update_ItemInObject(AECJSObject, "HOWL_LimiterEnable", cJSON_CreateNumber(aec_cfg.HOWL_LimiterEnable));
		//NS parameters for howling
		Update_ItemInObject(AECJSObject, "HOWL_NS_EN", cJSON_CreateNumber(aec_cfg.HOWL_NS_EN));
		Update_ItemInObject(AECJSObject, "HOWL_NSLevel", cJSON_CreateNumber(aec_cfg.HOWL_NSLevel));
	}
	return AECJSObject;
}

static cJSON *CreateAGCJSObject(WebrtcAGC_cfg_t agc_cfg)
{
	cJSON *AGCJSObject = cJSON_CreateObject();
	if (AGCJSObject) {
		Update_ItemInObject(AGCJSObject, "AGC_EN", cJSON_CreateNumber(agc_cfg.AGC_EN));
		Update_ItemInObject(AGCJSObject, "AGCMode", cJSON_CreateNumber(agc_cfg.AGCMode));
		Update_ItemInObject(AGCJSObject, "TargetLevelDbfs", cJSON_CreateNumber(agc_cfg.TargetLevelDbfs));
		Update_ItemInObject(AGCJSObject, "CompressionGaindB", cJSON_CreateNumber(agc_cfg.CompressionGaindB));
		Update_ItemInObject(AGCJSObject, "LimiterEnable", cJSON_CreateNumber(agc_cfg.LimiterEnable));
	}
	return AGCJSObject;
}

static cJSON *CreateNSJSObject(WebrtcNS_cfg_t ns_cfg)
{
	cJSON *NSJSObject = cJSON_CreateObject();
	if (NSJSObject) {
		Update_ItemInObject(NSJSObject, "NS_EN", cJSON_CreateNumber(ns_cfg.NS_EN));
		Update_ItemInObject(NSJSObject, "NSLevel", cJSON_CreateNumber(ns_cfg.NSLevel));
#if defined(CONFIG_PLATFORM_8735B) && defined(CONFIG_NEWAEC) && CONFIG_NEWAEC
		Update_ItemInObject(NSJSObject, "HPFEnable", cJSON_CreateNumber(ns_cfg.HPFEnable));
		Update_ItemInObject(NSJSObject, "QuickConvergenceEnable", cJSON_CreateNumber(ns_cfg.QuickConvergenceEnable));
#endif
	}
	return NSJSObject;
}

static cJSON *CreateVADJSObject(WebrtcVAD_cfg_t vad_cfg)
{
	cJSON *VADJSObject = cJSON_CreateObject();
	if (VADJSObject) {
		Update_ItemInObject(VADJSObject, "VAD_EN", cJSON_CreateNumber(vad_cfg.VAD_EN));
		Update_ItemInObject(VADJSObject, "VadMode", cJSON_CreateNumber(vad_cfg.VadMode));
	}
	return VADJSObject;
}
#endif
static void Update_TxConfig(cJSON *TXCONFIG_JSObject, TX_cfg_t tx_asp_params)
{
	if (TXCONFIG_JSObject) {
		Update_ItemInObject(TXCONFIG_JSObject, "agc_cfg", CreateAGCJSObject(tx_asp_params.agc_cfg));
		Update_ItemInObject(TXCONFIG_JSObject, "ns_cfg", CreateNSJSObject(tx_asp_params.ns_cfg));
		Update_ItemInObject(TXCONFIG_JSObject, "post_mute", cJSON_CreateNumber(tx_asp_params.post_mute));
	}
}
static void Update_RxConfig(cJSON *RXCONFIG_JSObject, RX_cfg_t rx_asp_params)
{
	if (RXCONFIG_JSObject) {
		Update_ItemInObject(RXCONFIG_JSObject, "aec_cfg", CreateAECJSObject(rx_asp_params.aec_cfg));
		Update_ItemInObject(RXCONFIG_JSObject, "agc_cfg", CreateAGCJSObject(rx_asp_params.agc_cfg));
		Update_ItemInObject(RXCONFIG_JSObject, "ns_cfg", CreateNSJSObject(rx_asp_params.ns_cfg));
#if !(defined(CONFIG_PLATFORM_8735B) && defined(CONFIG_NEWAEC) && CONFIG_NEWAEC)
		Update_ItemInObject(RXCONFIG_JSObject, "vad_cfg", CreateVADJSObject(rx_asp_params.vad_cfg));
#endif
		Update_ItemInObject(RXCONFIG_JSObject, "post_mute", cJSON_CreateNumber(rx_asp_params.post_mute));
	}
}

char *Get_Audio_CJSON(audio_params_t audio_save_params, RX_cfg_t rx_asp_params, TX_cfg_t tx_asp_params)
{
	cJSON *AUDJSObject = NULL, *AUDPARAM_JSObject = NULL, *TXCONFIG_JSObject = NULL, *RXCONFIG_JSObject = NULL;
	char *audio_json = NULL;
	cJSON_Hooks memoryHook;

	memoryHook.malloc_fn = malloc;
	memoryHook.free_fn = free;
	cJSON_InitHooks(&memoryHook);

	AUDJSObject = cJSON_CreateObject();
	AUDPARAM_JSObject = cJSON_CreateObject();
	TXCONFIG_JSObject = cJSON_CreateObject();
	RXCONFIG_JSObject = cJSON_CreateObject();

	cJSON_AddItemToObject(AUDJSObject, "audio_save_params", AUDPARAM_JSObject);
	cJSON_AddItemToObject(AUDJSObject, "tx_asp_params", TXCONFIG_JSObject);
	cJSON_AddItemToObject(AUDJSObject, "rx_asp_params", RXCONFIG_JSObject);
	Update_AudioParam(AUDPARAM_JSObject, audio_save_params);
	Update_TxConfig(TXCONFIG_JSObject, tx_asp_params);
	Update_RxConfig(RXCONFIG_JSObject, rx_asp_params);


	audio_json = cJSON_Print(AUDJSObject);
	cJSON_Delete(AUDJSObject);
	return audio_json;
}
