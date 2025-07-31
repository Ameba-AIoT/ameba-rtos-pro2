#ifndef AUDIO_TOOL_FILTERBANK_H
#define AUDIO_TOOL_FILTERBANK_H

// Init the gain dBFS table
void init_comp_dB(void);

// a simple 1 order lpf //
// input [in]: the input data to do first order LPF process
// output [in/out]: the output buffer to save the result
// input_samples [in]: the length in sample of the input data
// cutoff_fr [in]: the cutoff frequency of the LPF filter
// sample_rate [in]: the samples rate of the input data
// y_n1 [in/out]: y[n-1], the last data of the previous process
void simple_first_lpf(const short *input, short *output, int input_samples, int cutoff_fr, int sample_rate, float *y_n1);

// a simple second-order LPF //only with poles, no zeros and the damping factor (lamda) is fixed in 0.707 to prevent the overfitting //
// input [in]: the input data to do first order LPF process
// output [in/out]: the output buffer to save the result
// input_samples [in]: the length in sample of the input data
// cutoff_fr [in]: the cutoff frequency of the LPF filter
// sample_rate [in]: the samples rate of the input data
// y_n1 [in/out]: y[n-1], the last result from the previous process, will be updated in each time you call this function
// y_n2 [in/out]: y[n-2], the previous data of the last output result from the previous process, will be updated in each time you call this function
// x_n1 [in/out]: x[n-1], the last input data from the previous process, will be updated in each time you call this function
// x_n2 [in/out]: x[n-2], the previous input data of the last data from the previous process, will be updated in each time you call this function
void simple_second_lpf(const short *input, short *output, int input_samples, int cutoff_fr, int sample_rate, float *y_n1, float *y_n2, short *x_n1,
					   short *x_n2);

// downsapling with half sample rate
// input: the input singal data will be doing downsampling
// output: the output signal buffer to store the downsampling result
// input_samples: the samples in input data (1 sample = 16 bits)
// sample_rate: the sample rate of the inout data
// lpf_order: the order of the LPF before doensampling, support 0, 1, 2 now
// y_n1 [in/out]: y[n-1], the last result from the previous process, will be updated in each time you call this function
// y_n2 [in/out]: y[n-2], the previous data of the last output result from the previous process, will be updated in each time you call this function
// x_n1 [in/out]: x[n-1], the last input data from the previous process, will be updated in each time you call this function
// x_n2 [in/out]: x[n-2], the previous input data of the last data from the previous process, will be updated in each time you call this function
void pcm_downsample_2x(const short *input, short *output, int input_samples, int sample_rate, int lpf_order, float *y_n1, float *y_n2, short *x_n1,
					   short *x_n2);

// downsapling with half sample rate
// input: the input singal data will be doing upsampling
// output: the output signal buffer to store the upsampling result
// input_samples: the samples in input data (1 sample = 16 bits)
// sample_rate: the sample rate of the inout data
// lpf_order: the order of the LPF after upsampling, support 0, 1, 2 now
// y_n1 [in/out]: y[n-1], the last result from the previous process, will be updated in each time you call this function
// y_n2 [in/out]: y[n-2], the previous data of the last output result from the previous process, will be updated in each time you call this function
// x_n1 [in/out]: x[n-1], the last input data from the previous process, will be updated in each time you call this function
// x_n2 [in/out]: x[n-2], the previous input data of the last data from the previous process, will be updated in each time you call this function
int pcm_upsample_2x(short *outbuf, const short *inbuf, int input_samples, int sample_rate, int lpf_order, float *y_n1, float *y_n2, short *x_n1, short *x_n2);

#endif /* AUDIO_TOOL_FILTERBANK_H */
