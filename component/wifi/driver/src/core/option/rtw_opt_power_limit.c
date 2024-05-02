/******************************************************************************
*                           TXPWR_LMT.TXT
******************************************************************************/

typedef enum _ODM_PW_LMT_REGULATION_TYPE {
	PW_LMT_REGU_FCC = 0,
	PW_LMT_REGU_ETSI = 1,
	PW_LMT_REGU_MKK = 2,
	PW_LMT_REGU_WW13 = 3
} ODM_PW_LMT_REGULATION_TYPE;

typedef enum _ODM_PW_LMT_BAND_TYPE {
	PW_LMT_BAND_2_4G = 0,
	PW_LMT_BAND_5G = 1
} ODM_PW_LMT_BAND_TYPE;

typedef enum _ODM_PW_LMT_BANDWIDTH_TYPE {
	PW_LMT_BW_20M = 0,
	PW_LMT_BW_40M = 1,
	PW_LMT_BW_80M = 2
} ODM_PW_LMT_BANDWIDTH_TYPE;

typedef enum _ODM_PW_LMT_RATESECTION_TYPE {
	PW_LMT_RS_CCK = 0,
	PW_LMT_RS_OFDM = 1,
	PW_LMT_RS_HT = 2,
	PW_LMT_RS_VHT = 3
} ODM_PW_LMT_RATESECTION_TYPE;

typedef enum _ODM_PW_LMT_RFPATH_TYPE {
	PW_LMT_PH_1T = 0,
	PW_LMT_PH_2T = 1,
	PW_LMT_PH_3T = 2,
	PW_LMT_PH_4T = 3
} ODM_PW_LMT_RFPATH_TYPE;

typedef unsigned char			u1Byte;

// Note: power index = power*2
// => ex: if power = 16dBm, set power index = 32

const u1Byte Array_MP_8195A_TXPWR_LMT[] = {
	// regulation, band, bandwidth, rate, path, channel, power index
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	1,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	1,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	1,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	2,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	2,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	2,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	3,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	3,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	3,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	4,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	4,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	4,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	5,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	5,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	5,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	6,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	6,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	6,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	7,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	7,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	7,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	8,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	8,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	8,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	9,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	9,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	9,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	10,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	10,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	10,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	11,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	11,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	11,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	12,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	12,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	12,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	13,	26,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	13,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	13,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	14,	63,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	14,	63,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	14,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	1,	28,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	1,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	1,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	2,	28,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	2,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	2,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	3,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	3,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	3,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	4,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	4,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	4,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	5,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	5,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	5,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	6,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	6,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	6,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	7,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	7,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	7,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	8,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	8,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	8,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	9,	28,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	9,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	9,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	10,	28,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	10,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	10,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	11,	28,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	11,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	11,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	12,	24,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	12,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	12,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	13,	12,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	13,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	13,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	14,	63,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	14,	63,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	14,	63,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	1,	28,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	1,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	1,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	2,	28,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	2,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	2,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	3,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	3,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	3,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	4,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	4,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	4,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	5,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	5,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	5,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	6,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	6,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	6,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	7,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	7,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	7,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	8,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	8,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	8,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	9,	28,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	9,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	9,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	10,	28,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	10,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	10,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	11,	28,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	11,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	11,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	12,	24,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	12,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	12,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	13,	12,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	13,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	13,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	14,	63,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	14,	63,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	14,	63,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	1,	63,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	1,	63,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	1,	63,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	2,	63,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	2,	63,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	2,	63,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	3,	26,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	3,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	3,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	4,	26,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	4,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	4,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	5,	26,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	5,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	5,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	6,	26,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	6,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	6,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	7,	26,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	7,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	7,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	8,	26,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	8,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	8,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	9,	26,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	9,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	9,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	10,	24,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	10,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	10,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	11,	6,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	11,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	11,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	12,	63,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	12,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	12,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	13,	63,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	13,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	13,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	14,	63,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	14,	63,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	14,	63
};

const u1Byte Array_MP_8711B_TXPWR_LMT[] = {
	/* regulation, band, bandwidth, rateSection, rfPath, chnl, value */
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	1,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	1,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	1,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	2,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	2,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	2,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	3,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	3,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	3,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	4,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	4,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	4,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	5,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	5,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	5,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	6,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	6,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	6,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	7,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	7,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	7,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	8,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	8,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	8,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	9,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	9,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	9,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	10,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	10,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	10,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	11,	32,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	11,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	11,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	12,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	12,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	12,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	13,	26,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	13,	28,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	13,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	14,	63,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	14,	63,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_CCK,	PW_LMT_PH_1T,	14,	32,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	1,	28,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	1,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	1,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	2,	28,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	2,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	2,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	3,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	3,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	3,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	4,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	4,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	4,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	5,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	5,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	5,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	6,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	6,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	6,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	7,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	7,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	7,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	8,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	8,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	8,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	9,	28,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	9,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	9,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	10,	28,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	10,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	10,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	11,	28,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	11,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	11,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	12,	24,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	12,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	12,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	13,	12,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	13,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	13,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	14,	63,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	14,	63,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_OFDM,	PW_LMT_PH_1T,	14,	63,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	1,	28,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	1,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	1,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	2,	28,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	2,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	2,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	3,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	3,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	3,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	4,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	4,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	4,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	5,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	5,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	5,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	6,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	6,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	6,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	7,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	7,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	7,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	8,	30,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	8,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	8,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	9,	28,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	9,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	9,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	10,	28,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	10,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	10,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	11,	28,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	11,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	11,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	12,	24,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	12,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	12,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	13,	12,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	13,	30,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	13,	30,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	14,	63,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	14,	63,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_20M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	14,	63,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	1,	63,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	1,	63,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	1,	63,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	2,	63,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	2,	63,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	2,	63,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	3,	26,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	3,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	3,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	4,	26,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	4,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	4,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	5,	26,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	5,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	5,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	6,	26,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	6,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	6,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	7,	26,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	7,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	7,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	8,	26,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	8,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	8,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	9,	26,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	9,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	9,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	10,	24,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	10,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	10,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	11,	6,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	11,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	11,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	12,	63,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	12,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	12,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	13,	63,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	13,	26,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	13,	26,
	PW_LMT_REGU_FCC,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	14,	63,
	PW_LMT_REGU_ETSI,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	14,	63,
	PW_LMT_REGU_MKK,	PW_LMT_BAND_2_4G,	PW_LMT_BW_40M,	PW_LMT_RS_HT,	PW_LMT_PH_1T,	14,	63
};
