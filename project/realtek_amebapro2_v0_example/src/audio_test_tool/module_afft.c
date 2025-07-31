/******************************************************************************
*
* Copyright(c) 2007 - 2018 Realtek Corporation. All rights reserved.
*
******************************************************************************/

#include <stdint.h>
#include "platform_stdlib.h"
#include "avcodec.h"

#include "mmf2_module.h"
#include "module_afft.h"
#include "mmf2_dbg.h"

#include <math.h>
#include "arm_math.h" /* arm_cfft_f32, arm_cmplx_mag_f32, arm_max_f32, arm_cfft_sR_f32_len1024 */

float audio_rxdata_bk[FFT_BK_SIZE]__attribute__((aligned(0x20)));
// fft table for 1024 windows
float window_table_bk[1024] = {
	0.080000,  0.080009,  0.080035,  0.080078,  0.080139,  0.080217,  0.080312,  0.080425,
	0.080555,  0.080703,  0.080867,  0.081049,  0.081249,  0.081466,  0.081700,  0.081951,
	0.082219,  0.082505,  0.082808,  0.083129,  0.083466,  0.083821,  0.084193,  0.084582,
	0.084989,  0.085412,  0.085853,  0.086311,  0.086786,  0.087278,  0.087787,  0.088313,
	0.088856,  0.089416,  0.089993,  0.090588,  0.091199,  0.091827,  0.092472,  0.093134,
	0.093812,  0.094508,  0.095220,  0.095950,  0.096695,  0.097458,  0.098237,  0.099033,
	0.099846,  0.100675,  0.101521,  0.102383,  0.103262,  0.104157,  0.105069,  0.105997,
	0.106942,  0.107903,  0.108880,  0.109873,  0.110883,  0.111909,  0.112951,  0.114009,
	0.115083,  0.116173,  0.117279,  0.118402,  0.119540,  0.120693,  0.121863,  0.123049,
	0.124250,  0.125467,  0.126699,  0.127947,  0.129211,  0.130490,  0.131785,  0.133095,
	0.134420,  0.135761,  0.137117,  0.138488,  0.139874,  0.141276,  0.142692,  0.144123,
	0.145570,  0.147031,  0.148507,  0.149998,  0.151503,  0.153023,  0.154558,  0.156107,
	0.157671,  0.159249,  0.160842,  0.162449,  0.164070,  0.165705,  0.167355,  0.169018,
	0.170696,  0.172387,  0.174092,  0.175811,  0.177544,  0.179291,  0.181051,  0.182824,
	0.184611,  0.186412,  0.188226,  0.190053,  0.191893,  0.193747,  0.195613,  0.197493,
	0.199385,  0.201290,  0.203208,  0.205139,  0.207082,  0.209038,  0.211007,  0.212988,
	0.214981,  0.216986,  0.219004,  0.221033,  0.223075,  0.225129,  0.227195,  0.229272,
	0.231361,  0.233462,  0.235574,  0.237698,  0.239833,  0.241980,  0.244137,  0.246306,
	0.248486,  0.250677,  0.252879,  0.255092,  0.257316,  0.259550,  0.261794,  0.264050,
	0.266315,  0.268591,  0.270877,  0.273174,  0.275480,  0.277797,  0.280123,  0.282459,
	0.284805,  0.287160,  0.289525,  0.291900,  0.294284,  0.296677,  0.299079,  0.301490,
	0.303910,  0.306340,  0.308778,  0.311224,  0.313680,  0.316144,  0.318616,  0.321097,
	0.323586,  0.326083,  0.328588,  0.331101,  0.333623,  0.336151,  0.338688,  0.341232,
	0.343784,  0.346343,  0.348909,  0.351483,  0.354063,  0.356651,  0.359246,  0.361847,
	0.364455,  0.367070,  0.369691,  0.372319,  0.374953,  0.377593,  0.380240,  0.382892,
	0.385550,  0.388215,  0.390884,  0.393560,  0.396241,  0.398927,  0.401619,  0.404316,
	0.407018,  0.409726,  0.412438,  0.415154,  0.417876,  0.420602,  0.423333,  0.426068,
	0.428807,  0.431551,  0.434299,  0.437050,  0.439806,  0.442565,  0.445328,  0.448095,
	0.450865,  0.453638,  0.456415,  0.459195,  0.461977,  0.464763,  0.467552,  0.470343,
	0.473137,  0.475934,  0.478733,  0.481534,  0.484337,  0.487143,  0.489951,  0.492760,
	0.495571,  0.498384,  0.501199,  0.504014,  0.506832,  0.509650,  0.512470,  0.515291,
	0.518112,  0.520935,  0.523758,  0.526582,  0.529406,  0.532231,  0.535056,  0.537881,
	0.540706,  0.543532,  0.546357,  0.549182,  0.552006,  0.554830,  0.557654,  0.560477,
	0.563299,  0.566120,  0.568940,  0.571759,  0.574577,  0.577394,  0.580209,  0.583023,
	0.585835,  0.588645,  0.591454,  0.594260,  0.597065,  0.599867,  0.602667,  0.605465,
	0.608260,  0.611053,  0.613843,  0.616630,  0.619414,  0.622196,  0.624974,  0.627749,
	0.630521,  0.633289,  0.636054,  0.638815,  0.641573,  0.644326,  0.647076,  0.649821,
	0.652563,  0.655300,  0.658033,  0.660762,  0.663485,  0.666205,  0.668919,  0.671629,
	0.674333,  0.677033,  0.679727,  0.682417,  0.685100,  0.687779,  0.690451,  0.693118,
	0.695780,  0.698435,  0.701084,  0.703728,  0.706365,  0.708996,  0.711620,  0.714238,
	0.716850,  0.719455,  0.722053,  0.724644,  0.727228,  0.729805,  0.732375,  0.734938,
	0.737493,  0.740041,  0.742581,  0.745114,  0.747639,  0.750156,  0.752665,  0.755167,
	0.757660,  0.760145,  0.762621,  0.765089,  0.767549,  0.770000,  0.772442,  0.774876,
	0.777301,  0.779717,  0.782124,  0.784521,  0.786910,  0.789289,  0.791658,  0.794019,
	0.796369,  0.798710,  0.801041,  0.803363,  0.805674,  0.807976,  0.810267,  0.812548,
	0.814819,  0.817079,  0.819329,  0.821569,  0.823798,  0.826016,  0.828223,  0.830420,
	0.832605,  0.834780,  0.836943,  0.839095,  0.841236,  0.843366,  0.845484,  0.847590,
	0.849685,  0.851768,  0.853840,  0.855900,  0.857947,  0.859983,  0.862007,  0.864018,
	0.866017,  0.868005,  0.869979,  0.871941,  0.873891,  0.875828,  0.877752,  0.879664,
	0.881563,  0.883449,  0.885322,  0.887182,  0.889029,  0.890862,  0.892683,  0.894490,
	0.896284,  0.898064,  0.899831,  0.901584,  0.903324,  0.905050,  0.906762,  0.908461,
	0.910145,  0.911815,  0.913472,  0.915114,  0.916742,  0.918356,  0.919956,  0.921542,
	0.923113,  0.924669,  0.926211,  0.927739,  0.929251,  0.930750,  0.932233,  0.933702,
	0.935155,  0.936594,  0.938018,  0.939427,  0.940821,  0.942199,  0.943563,  0.944911,
	0.946244,  0.947562,  0.948864,  0.950151,  0.951423,  0.952679,  0.953919,  0.955144,
	0.956353,  0.957546,  0.958724,  0.959886,  0.961031,  0.962162,  0.963276,  0.964374,
	0.965456,  0.966522,  0.967572,  0.968606,  0.969624,  0.970626,  0.971611,  0.972580,
	0.973533,  0.974469,  0.975389,  0.976292,  0.977179,  0.978050,  0.978904,  0.979742,
	0.980563,  0.981367,  0.982154,  0.982925,  0.983680,  0.984417,  0.985138,  0.985842,
	0.986529,  0.987199,  0.987853,  0.988489,  0.989109,  0.989712,  0.990297,  0.990866,
	0.991418,  0.991952,  0.992470,  0.992971,  0.993454,  0.993921,  0.994370,  0.994802,
	0.995217,  0.995615,  0.995995,  0.996359,  0.996705,  0.997034,  0.997345,  0.997640,
	0.997917,  0.998177,  0.998420,  0.998645,  0.998853,  0.999044,  0.999217,  0.999373,
	0.999512,  0.999633,  0.999738,  0.999824,  0.999894,  0.999946,  0.999981,  0.999998,
	0.999998,  0.999981,  0.999946,  0.999894,  0.999824,  0.999738,  0.999633,  0.999512,
	0.999373,  0.999217,  0.999044,  0.998853,  0.998645,  0.998420,  0.998177,  0.997917,
	0.997640,  0.997345,  0.997034,  0.996705,  0.996359,  0.995995,  0.995615,  0.995217,
	0.994802,  0.994370,  0.993921,  0.993454,  0.992971,  0.992470,  0.991952,  0.991418,
	0.990866,  0.990297,  0.989712,  0.989109,  0.988489,  0.987853,  0.987199,  0.986529,
	0.985842,  0.985138,  0.984417,  0.983680,  0.982925,  0.982154,  0.981367,  0.980562,
	0.979742,  0.978904,  0.978050,  0.977179,  0.976292,  0.975389,  0.974469,  0.973532,
	0.972580,  0.971611,  0.970625,  0.969624,  0.968606,  0.967572,  0.966522,  0.965456,
	0.964374,  0.963276,  0.962162,  0.961031,  0.959885,  0.958724,  0.957546,  0.956353,
	0.955144,  0.953919,  0.952679,  0.951423,  0.950151,  0.948864,  0.947562,  0.946244,
	0.944911,  0.943563,  0.942199,  0.940821,  0.939427,  0.938018,  0.936594,  0.935155,
	0.933702,  0.932233,  0.930750,  0.929251,  0.927739,  0.926211,  0.924669,  0.923113,
	0.921542,  0.919956,  0.918356,  0.916742,  0.915114,  0.913472,  0.911815,  0.910145,
	0.908461,  0.906762,  0.905050,  0.903324,  0.901584,  0.899831,  0.898064,  0.896284,
	0.894490,  0.892683,  0.890862,  0.889029,  0.887182,  0.885322,  0.883449,  0.881563,
	0.879664,  0.877752,  0.875828,  0.873891,  0.871941,  0.869979,  0.868004,  0.866017,
	0.864018,  0.862007,  0.859983,  0.857947,  0.855899,  0.853840,  0.851768,  0.849685,
	0.847590,  0.845484,  0.843365,  0.841236,  0.839095,  0.836943,  0.834780,  0.832605,
	0.830420,  0.828223,  0.826016,  0.823798,  0.821569,  0.819329,  0.817079,  0.814819,
	0.812548,  0.810267,  0.807976,  0.805674,  0.803363,  0.801041,  0.798710,  0.796369,
	0.794019,  0.791658,  0.789289,  0.786909,  0.784521,  0.782123,  0.779717,  0.777301,
	0.774876,  0.772442,  0.770000,  0.767549,  0.765089,  0.762621,  0.760145,  0.757660,
	0.755167,  0.752665,  0.750156,  0.747639,  0.745114,  0.742581,  0.740041,  0.737493,
	0.734938,  0.732375,  0.729805,  0.727228,  0.724644,  0.722052,  0.719454,  0.716850,
	0.714238,  0.711620,  0.708996,  0.706365,  0.703728,  0.701084,  0.698435,  0.695780,
	0.693118,  0.690451,  0.687779,  0.685100,  0.682417,  0.679727,  0.677033,  0.674333,
	0.671629,  0.668919,  0.666205,  0.663485,  0.660761,  0.658033,  0.655300,  0.652563,
	0.649821,  0.647076,  0.644326,  0.641572,  0.638815,  0.636054,  0.633289,  0.630521,
	0.627749,  0.624974,  0.622196,  0.619414,  0.616630,  0.613843,  0.611053,  0.608260,
	0.605465,  0.602667,  0.599867,  0.597065,  0.594260,  0.591454,  0.588645,  0.585835,
	0.583023,  0.580209,  0.577394,  0.574577,  0.571759,  0.568940,  0.566120,  0.563299,
	0.560476,  0.557654,  0.554830,  0.552006,  0.549182,  0.546357,  0.543532,  0.540706,
	0.537881,  0.535056,  0.532231,  0.529406,  0.526582,  0.523758,  0.520935,  0.518112,
	0.515291,  0.512470,  0.509650,  0.506832,  0.504014,  0.501198,  0.498384,  0.495571,
	0.492760,  0.489950,  0.487143,  0.484337,  0.481534,  0.478733,  0.475934,  0.473137,
	0.470343,  0.467552,  0.464763,  0.461977,  0.459194,  0.456415,  0.453638,  0.450865,
	0.448095,  0.445328,  0.442565,  0.439806,  0.437050,  0.434298,  0.431551,  0.428807,
	0.426068,  0.423333,  0.420602,  0.417876,  0.415154,  0.412437,  0.409725,  0.407018,
	0.404316,  0.401619,  0.398928,  0.396241,  0.393560,  0.390884,  0.388214,  0.385550,
	0.382892,  0.380240,  0.377593,  0.374953,  0.372319,  0.369691,  0.367070,  0.364455,
	0.361847,  0.359246,  0.356651,  0.354063,  0.351483,  0.348909,  0.346343,  0.343784,
	0.341232,  0.338688,  0.336151,  0.333622,  0.331101,  0.328588,  0.326083,  0.323586,
	0.321097,  0.318616,  0.316144,  0.313680,  0.311224,  0.308778,  0.306340,  0.303910,
	0.301490,  0.299079,  0.296677,  0.294283,  0.291900,  0.289525,  0.287160,  0.284805,
	0.282459,  0.280123,  0.277797,  0.275480,  0.273174,  0.270877,  0.268591,  0.266315,
	0.264050,  0.261794,  0.259550,  0.257316,  0.255092,  0.252879,  0.250677,  0.248486,
	0.246306,  0.244137,  0.241980,  0.239833,  0.237698,  0.235574,  0.233462,  0.231361,
	0.229272,  0.227195,  0.225129,  0.223075,  0.221033,  0.219004,  0.216986,  0.214981,
	0.212988,  0.211007,  0.209038,  0.207082,  0.205139,  0.203208,  0.201290,  0.199385,
	0.197493,  0.195613,  0.193747,  0.191893,  0.190053,  0.188226,  0.186412,  0.184611,
	0.182824,  0.181051,  0.179290,  0.177544,  0.175811,  0.174092,  0.172387,  0.170695,
	0.169018,  0.167355,  0.165705,  0.164070,  0.162449,  0.160842,  0.159249,  0.157671,
	0.156107,  0.154558,  0.153023,  0.151503,  0.149998,  0.148507,  0.147031,  0.145570,
	0.144123,  0.142692,  0.141276,  0.139874,  0.138488,  0.137117,  0.135761,  0.134420,
	0.133095,  0.131785,  0.130490,  0.129211,  0.127947,  0.126699,  0.125467,  0.124250,
	0.123049,  0.121863,  0.120693,  0.119540,  0.118402,  0.117279,  0.116173,  0.115083,
	0.114009,  0.112951,  0.111909,  0.110883,  0.109873,  0.108880,  0.107903,  0.106942,
	0.105997,  0.105069,  0.104157,  0.103262,  0.102383,  0.101521,  0.100675,  0.099846,
	0.099033,  0.098237,  0.097458,  0.096695,  0.095949,  0.095220,  0.094508,  0.093812,
	0.093134,  0.092472,  0.091827,  0.091199,  0.090588,  0.089993,  0.089416,  0.088856,
	0.088313,  0.087787,  0.087278,  0.086785,  0.086311,  0.085853,  0.085412,  0.084989,
	0.084582,  0.084193,  0.083821,  0.083466,  0.083129,  0.082808,  0.082505,  0.082219,
	0.081951,  0.081700,  0.081466,  0.081249,  0.081049,  0.080867,  0.080703,  0.080555,
	0.080425,  0.080312,  0.080217,  0.080139,  0.080078,  0.080035,  0.080009,  0.080000,
};
//------------------------------------------------------------------------------
//caculate thd value
/* point_range needs to be odd */
void thd_cal_bk(fft_cal_bk_t *pfft_cal, uint32_t point_range, uint32_t point_num, uint32_t cal_point_num)
{
	uint32_t max_index, i, j, point_index, index_bf;
	float max_temp, max_bf, harmonic_total;
	float harmonic, thd_tp;

	max_index = pfft_cal->max_index;

	for (j = 0; j < point_num; j++) {

		point_index = max_index * (2 + j);

		point_index = point_index - ((point_range - 1) >> 1);

		max_bf = 0;
		index_bf = 0;
		for (i = point_index; i < (point_index + point_range); i++) {
			max_temp = pfft_cal->output[i];
			if (max_temp > max_bf) {
				max_bf = max_temp;
				index_bf = i;
			}
		}
		pfft_cal->harmonic[j] = max_bf;
		pfft_cal->harmonic_index[j] = index_bf;
		//dbg_printf("j: %d, harmonic_index: %d, harmonic: %f \r\n", j, pfft_cal->harmonic_index[j], pfft_cal->harmonic[j]);
	}

	harmonic_total = 0;
	for (j = 0; j < cal_point_num; j++) {
		harmonic = pfft_cal->harmonic[j];
		harmonic_total = harmonic_total + harmonic * harmonic;
	}

	thd_tp = sqrtf(harmonic_total) / pfft_cal->max_value;
	pfft_cal->thd = 20 * log10f(thd_tp);

}

//caculate noise value
/* thd_cnt_range needs to be odd */
void noise_cal_bk(fft_cal_bk_t *pfft_cal, uint32_t data_shift, uint32_t thd_cnt, uint32_t thd_cnt_range, float limit_scale)
{
	float total_value, cal_tp, harmonic_temp;
	uint32_t i, j, data_cnt;
	uint32_t harmonic_index;

	data_cnt = 0;
	total_value = 0;
	for (i = 0; i < (FFT_BK_SIZE >> 1); i++) {
		if ((i > (pfft_cal->max_index + data_shift)) || (i < (pfft_cal->max_index - data_shift))) {
			cal_tp = (pfft_cal->output[i] / limit_scale); //Limit Scale
			data_cnt++;
		} else {
			cal_tp = 0;
		}

		total_value = total_value + cal_tp;
	}
	//dbg_printf("total_value: %f, data_cnt: %d \r\n", total_value, data_cnt);

	for (i = 0; i < thd_cnt; i++) {

		harmonic_temp = 0;
		harmonic_index = pfft_cal->harmonic_index[i];
		harmonic_index = harmonic_index - ((thd_cnt_range - 1) >> 1);
		for (j = 0; j < thd_cnt_range; j++) {
			harmonic_temp = harmonic_temp + (pfft_cal->output[harmonic_index + j] / limit_scale);
			//dbg_printf("harmonic_index + j: %d, harmonic_temp: %f \r\n", (harmonic_index + j), pfft_cal->output[harmonic_index + j]);
		}

		total_value = total_value - harmonic_temp;
	}
	//dbg_printf("total_value: %f, harmonic_temp: %d \r\n", total_value, harmonic_temp);

	data_cnt = data_cnt - (thd_cnt * thd_cnt_range);

	pfft_cal->cal_noise_value = ((total_value / data_cnt) * limit_scale);
	pfft_cal->cal_n_db = 20 * log10f(pfft_cal->cal_noise_value);

}

uint32_t get_cfft_instance(uint32_t fft_size)
{
	uint32_t s = 1024;

	switch (fft_size) {
	case 16: //bit 4
		s = (uint32_t)&arm_cfft_sR_f32_len16;
		break;
	case 32:
		s = (uint32_t)&arm_cfft_sR_f32_len32;
		break;
	case 64:
		s = (uint32_t)&arm_cfft_sR_f32_len64;
		break;
	case 128:
		s = (uint32_t)&arm_cfft_sR_f32_len128;
		break;
	case 256:
		s = (uint32_t)&arm_cfft_sR_f32_len256;
		break;
	case 512:
		s = (uint32_t)&arm_cfft_sR_f32_len512;
		break;
	case 1024:
		s = (uint32_t)&arm_cfft_sR_f32_len1024;
		break;
	case 2048:
		s = (uint32_t)&arm_cfft_sR_f32_len2048;
		break;
	case 4096: //bit 12
		s = (uint32_t)&arm_cfft_sR_f32_len4096;
		break;

	default:
		dbg_printf("input_init Error !!! \r\n");
		break;
	}

	return s;
}


void afft_input_init(float *pinput, float *pdata, uint32_t fft_size)
{
	uint32_t i, index;

	for (i = 0; i < fft_size; i++) {
		index = (i << 1);
		*(pinput + index) = (float) * (pdata + i);
		*(pinput + index + 1) = 0;
	}
}

//caculate the fft
void fft_cal_bk(float *ptest_data, fft_cal_bk_t *pfft_cal, uint32_t sample_rate, float *accumlated_output, uint32_t *accumlated_times)
{
	//window
	for (uint32_t i = 0; i < FFT_BK_SIZE; i++) {
		*(ptest_data + i) = *(ptest_data + i) * window_table_bk[i];
	}

	afft_input_init(pfft_cal->input, ptest_data, FFT_BK_SIZE);

	pfft_cal->pcfft_instance = (arm_cfft_instance_f32 *)get_cfft_instance(FFT_BK_SIZE);

	/* Process the data through the CFFT/CIFFT module */
	arm_cfft_f32(&arm_cfft_sR_f32_len1024, pfft_cal->input, 0, 1);

	/* Process the data through the Complex Magniture Module for calculating the magnitude at each bin */
	arm_cmplx_mag_f32(pfft_cal->input, pfft_cal->output, FFT_BK_SIZE);

	/* only the front half elements are needed and the magnitude  */
	for (uint32_t j = 0; j < (FFT_BK_SIZE >> 1); j++) {
		if (j != 0 && j != (FFT_BK_SIZE >> 1) - 1) {
			pfft_cal->output[j] = 2 * pfft_cal->output[j];
		}
		pfft_cal->output[j] = pfft_cal->output[j] / FFT_BK_SIZE;
	}

	/* Calculates maxValue and returns corresponding value */
	/* only use the front half size to do calculate */
	arm_max_f32(pfft_cal->output, (FFT_BK_SIZE >> 1), &pfft_cal->max_value, &pfft_cal->max_index);

	//dbg_printf("max_value: %f, max_index: %d \r\n", pfft_cal->max_value, pfft_cal->max_index);

	/* Calculates performance: Audio Codec */
	//thd_cal_bk(pfft_cal, 5, 10, 5);
	//noise_cal_bk(pfft_cal, 4, 10, 5, 1);

	pfft_cal->input_frq = ((float)sample_rate / FFT_BK_SIZE) * pfft_cal->max_index;

#if 0

	/* output print */
	for (int i = 0; i < (FFT_BK_SIZE >> 1); i++) {
		if (accumlated_output && accumlated_times) {
			accumlated_times++;
			//printf("%f \r\n", accumlated_output[i]);
			accumlated_output[i] += pfft_cal->output[i];
			//printf("frequency: %f, average magnitude: %f, magnitude: %f\r\n", ((float)sample_rate / FFT_BK_SIZE) * i, 20*log10f(accumlated_output[i] / (*accumlated_times)), 20*log10f(pfft_cal->output[i]));
		} else {
			//printf("frequency: %f, magnitude: %f\r\n", ((float)sample_rate / FFT_BK_SIZE) * i, 20*log10f(pfft_cal->output[i]));
		}
	}
#endif

}

int afft_handle(void *p, void *input, void *output)
{
	afft_ctx_t *ctx = (afft_ctx_t *)p;
	mm_queue_item_t *input_item = (mm_queue_item_t *)input;
	mm_queue_item_t *output_item = (mm_queue_item_t *)output;

	//int samples_read, frame_size;
	//int frame_size = 0;

	//output_item->timestamp = input_item->timestamp;
	// set timestamp to 1st sample (cache head)
	//output_item->timestamp -= 1000 * (ctx->cache_idx / 2) / ctx->params.sample_rate;
	//printf("AFFT Handle\r\n");
	memcpy(ctx->cache + ctx->cache_idx, (void *)input_item->data_addr, input_item->size);
	ctx->cache_idx += input_item->size;

	if (ctx->cache_idx >= FFT_BK_SIZE * 2) {
		if (!ctx->stop) {
			for (int i = 0; i < FFT_BK_SIZE; i++) {
				audio_rxdata_bk[i] = (float)((short)((ctx->cache[(i << 1) + 1] << 8) | ctx->cache[(i << 1)])) / (32767.0f);
			}
			fft_cal_bk(audio_rxdata_bk, &(ctx->fft_cal_bk_signal), ctx->params.sample_rate, ctx->accumlated_output, &(ctx->accumlated_times));
			mm_printf("\r\n ============== Audio Info =============== \r\n");
			mm_printf("sample_rate: %d \r\n", ctx->params.sample_rate);
			mm_printf("max_index: %d \r\n", ctx->fft_cal_bk_signal.max_index);
			mm_printf("input_frq: %f \r\n", ctx->fft_cal_bk_signal.input_frq);
			mm_printf("input_frq megnitude: %f \r\n", 20 * log10f(ctx->fft_cal_bk_signal.output[ctx->fft_cal_bk_signal.max_index]));
		}
		ctx->cache_idx -= FFT_BK_SIZE * 2;
		if (ctx->cache_idx >= 0) {
			memmove(ctx->cache, ctx->cache + FFT_BK_SIZE * 2, ctx->cache_idx);
		}
	}
	if (ctx->pcm_out_en) {
		//printf("AFFT Output\r\n");
		output_item->size = input_item->size;
		output_item->timestamp = input_item->timestamp;
		output_item->type = input_item->type;
		memcpy((void *) output_item->data_addr, (void *) input_item->data_addr, input_item->size);

		return output_item->size;
	}
	return 0;
}

int afft_control(void *p, int cmd, int arg)
{
	afft_ctx_t *ctx = (afft_ctx_t *)p;

	switch (cmd) {
	case CMD_AFFT_SET_PARAMS:
		memcpy(&ctx->params, ((afft_params_t *)arg), sizeof(afft_params_t));
		break;
	case CMD_AFFT_GET_PARAMS:
		memcpy(((afft_params_t *)arg), &ctx->params, sizeof(afft_params_t));
		break;
	case CMD_AFFT_SAMPLERATE:
		ctx->params.sample_rate = arg;
		break;
	case CMD_AFFT_CHANNEL:
		ctx->params.channel = arg;
		break;
	case CMD_AFFT_RESET_FFT_RESULT:
		ctx->accumlated_times = 0;
		memset(ctx->accumlated_output, 0, sizeof(float) * FFT_BK_SIZE);
		break;
	case CMD_AFFT_SET_OUTPUT:
		if (arg) {
			ctx->pcm_out_en = (bool)arg;
			((mm_context_t *)ctx->parent)->module->output_type = MM_TYPE_ASINK;
			((mm_context_t *)ctx->parent)->module->module_type = MM_TYPE_ADSP;
		} else {
			ctx->pcm_out_en = (bool)arg;
			((mm_context_t *)ctx->parent)->module->output_type = MM_TYPE_NONE;
			((mm_context_t *)ctx->parent)->module->module_type = MM_TYPE_ASINK;
		}
		break;
	case CMD_AFFT_SHOWN:
		if (arg) {
			ctx->cache_idx = 0;
			ctx->stop = 0;
			ctx->accumlated_times = 0;
		} else {
			ctx->stop = 1;
		}
		break;
	case CMD_AFFT_APPLY:
		ctx->accumlated_times = 0;
		memset(ctx->accumlated_output, 0, sizeof(float) * FFT_BK_SIZE);
		ctx->cache = (uint8_t *)malloc(FFT_BK_SIZE * 2 + 1500); // 1500 max audio page size
		if (!ctx->cache) {
			mm_printf("Output memory\n\r");
			while (1);
		}
		break;
	}

	return 0;
}

void *afft_destroy(void *p)
{
	afft_ctx_t *ctx = (afft_ctx_t *)p;

	if (ctx) {
		if (ctx->cache) {
			free(ctx->cache);
		}
		ctx->cache_idx = 0;
		free(ctx);
	}

	return NULL;
}

void *afft_create(void *parent)
{
	afft_ctx_t *ctx = malloc(sizeof(afft_ctx_t));
	if (!ctx) {
		return NULL;
	}
	memset(ctx, 0, sizeof(afft_ctx_t));
	ctx->parent = parent;

	return ctx;
}

void *afft_new_item(void *p)
{
	afft_ctx_t *ctx = (afft_ctx_t *)p;

	return (void *)malloc(ctx->params.pcm_frame_size);
}

void *afft_del_item(void *p, void *d)
{
	(void)p;
	if (d) {
		free(d);
	}
	return NULL;
}

mm_module_t afft_module = {
	.create = afft_create,
	.destroy = afft_destroy,
	.control = afft_control,
	.handle = afft_handle,

	.new_item = afft_new_item,
	.del_item = afft_del_item,
	.rsz_item = NULL,

	.output_type = MM_TYPE_NONE,
	.module_type = MM_TYPE_ASINK,
	.name = "AFFT"
};
