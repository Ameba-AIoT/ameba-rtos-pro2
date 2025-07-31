#include <FreeRTOS.h>
#include <task.h>
#include <platform_stdlib.h>
#include <arm_math.h>

#define MAX_DBFS 10
#define COM_DBFS 5
static float dB_fPOW[MAX_DBFS] = {1}; //compensate for LPF set to 1 if not set it

void init_comp_dB(void)
{
	for (int i = 0; i < MAX_DBFS; i++) {
		dB_fPOW[i] = pow(10.0f, (float)((float)(i) / 20.0f));
	}
}

/*
y[n] = y[n-1] + (a * (x[n] - y[n-1]))
where
a = ts / (rc + ts), rc = 1 / 2 * PI * cutoff_frequency
ts is the time interval between each samples
*/
// a simple first-order LPF
void simple_first_lpf(const short *input, short *output, int input_samples, int cutoff_fr, int sample_rate, float *y_n1)
{
	float rc = 1.0 / ((float)cutoff_fr * 2.0 * PI);
	float ts = 1.0 / (float)sample_rate;
	float alpha = ts / (rc + ts);
	float temp = 0;
	short output_pattern;

	if (input && output && (cutoff_fr < sample_rate / 2)) {
		for (int i = 0; i < input_samples; i++) {
			// LPF result
			// temp ~ output[i - 1]
			temp = (float)temp + (alpha * ((float)input[i] - (float)(*y_n1)));
			// covert float to short
			output_pattern = (short)(temp * dB_fPOW[COM_DBFS]);
			//printf("rc = %f, ts = %f, alpha = %f, output = %f\r\n", rc, ts, alpha, temp);
			output[i] = output_pattern;
			*y_n1 = temp;
		}
	}
}

/*
s domain
  Y(s)             1
 ----- = -----------------------
  X(s)    (s/wn)^2+2*(L/wn)*s+1
s domain to z domain
         1-z^(-1)
s ~ K * ----------
         1+z^(-1)
where
                   2*PI*fc
wn = -------------------------------------
      [(1-2*L^2)+(4*L^4-4*L^2+2)^0.5]^0.5
K = 2 * sample_rate
fc is the cutoff frequency
*/
// a simple second-order LPF //only with poles, no zeros and the damping factor (lamda) is fixed in 0.707 to prevent the overfitting
void simple_second_lpf(const short *input, short *output, int input_samples, int cutoff_fr, int sample_rate, float *y_n1, float *y_n2, short *x_n1, short *x_n2)
{
	// s domain parameters
	float small_factor = 0.00000001; // to prvent devide by 0
	float lambda = 0.707; // prevent overfiiting
	float wc = (float)cutoff_fr * 2.0 * PI;
	float conj = 0;
	arm_sqrt_f32(4 * (lambda * lambda * lambda * lambda - lambda * lambda) + 2, &conj); // conjugate
	float fn = 0;
	arm_sqrt_f32(1 - 2 * lambda * lambda + conj, &fn);
	fn = cutoff_fr / (fn + small_factor);
	float wn = 2.0 * PI * fn;
	float b0 = 0, b1 = 0, b2 = wn * wn;
	float a0 = 1;
	float a1 = 2.0 * wn * lambda;
	float a2 = wn * wn;
	float K = 2.0 * (float)sample_rate;

	// z domain paramters
	float d = a0 * K * K + a1 * K + a2 + small_factor;
	float x0 = (b0 * K * K + b1 * K + b2) / d;
	float x1 = (2.0 * b2 - 2.0 * b0 * K * K) / d;
	float x2 = (b0 * K * K - b1 * K + b2) / d;
	float y1 = -1.0 * (2.0 * a2 - 2.0 * a0 * K * K) / d;
	float y2 = -1.0 * (a0 * K * K - a1 * K + a2) / d;

	float y_n = 0;
	//static float y_n1 = 0, y_n2 = 0;
	//static short x_n1 = 0, x_n2 = 0;
	short output_pattern;

	if (input && output && (cutoff_fr < sample_rate / 2)) {
		for (int i = 0; i < input_samples; i++) {
			// LPF result
			y_n = x0 * (float)input[i] + x1 * (float)(*x_n1) + x2 * (float)(*x_n2) + y1 * (*y_n1) + y2 * (*y_n2);
			// covert float to short
			output_pattern = (short)(y_n * dB_fPOW[COM_DBFS]);
			//printf("x0 = %f, x1 = %f, x2 = %f, y1 = %f, y2 = %f, y_n = %f\r\n", x0, x1, x2, y1, y2, y_n);
			output[i] = output_pattern;

			*y_n2 = *y_n1;
			*y_n1 = *y_n;
			*x_n2 = *x_n1;
			*x_n1 = input[i];
		}
	}
}

void pcm_downsample_2x(const short *input, short *output, int input_samples, int sample_rate, int lpf_order, float *y_n1, float *y_n2, short *x_n1, short *x_n2)
{
	short *lpf_data = malloc(input_samples * sizeof(short));
	static float y_n1, y_n2 = 0;
	static short x_n1, x_n2 = 0;
	if (lpf_data) {
		// apply low pass filter
		// filter the signal larger than the BW of sample_rate / 2
		if (lpf_order == 2) {
			simple_second_lpf(input, lpf_data, input_samples, sample_rate / 2 / 2, sample_rate, y_n1, y_n2, x_n1, x_n2);
		} else if (lpf_order == 1) {
			simple_first_lpf(input, lpf_data, input_samples, sample_rate / 2 / 2, sample_rate, y_n1);
		} else {
			memcpy(lpf_data, input, input_samples * sizeof(short));
		}
		for (int i = 0; i < input_samples / 2; i++) {
			output[i] = lpf_data[2 * i];
		}
		free(lpf_data);
	}
}

int pcm_upsample_2x(short *outbuf, const short *inbuf, int input_samples, int sample_rate, int lpf_order, float *y_n1, float *y_n2, short *x_n1, short *x_n2)
{
	int doutlen = 0;
	short *prelpf_data = malloc(2 * input_samples * sizeof(short));
	if (outbuf == NULL || inbuf == NULL || input_samples <= 0 || prelpf_data == NULL) {
		return -1;
	}

	for (int i = 0; i < input_samples; i++) {
		prelpf_data[doutlen++] = inbuf[i];
		prelpf_data[doutlen++] = 0;
	}

	if (lpf_order == 2) {
		simple_second_lpf(prelpf_data, outbuf, 2 * input_samples, 2 * sample_rate / 2 / 2, 2 * sample_rate, y_n1, y_n2, x_n1, x_n2);
	} else if (lpf_order == 1) {
		simple_first_lpf(prelpf_data, outbuf, 2 * input_samples, 2 * sample_rate / 2 / 2, 2 * sample_rate, y_n1);
	} else {
		memcpy(outbuf, prelpf_data, 2 * input_samples * sizeof(short));
	}
	free(prelpf_data);

	return 0;
}
