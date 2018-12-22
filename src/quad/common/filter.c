
#include "filter.h"
#include "utils.h"
#include <math.h>

#define M_PI_FLOAT				3.14159265358979323846f

#define BIQUAD_Q				1.0f / sqrtf(2.0f)				/* quality factor - butterworth */

/* NULL filter function */
float nullFilterApply(void *filter, float input)
{
	UNUSED(filter);
	return input;
}

/* biquad filter functions */
void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate)
{
	biquadFilterInit(filter, filterFreq, refreshRate, BIQUAD_Q, FILTER_LPF);
}

/* Biquad 2nd-order low pass filter or notch filter
 * Referenced by <Filter_Design_Equations.pdf> under D:\Kent\PhD\3-QUADCOPTER\STM32\Filters
 */
void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType)
{
	/* setup variables */
	const float sampleRate = 1 / ((float)refreshRate * 0.000001f);	// refreshRate = gyro.targetLooptime = 500 us for example, sampleRate (fs) is frequency in Hz
	const float omega = 2 * M_PI_FLOAT * filterFreq / sampleRate;	// filterFreq (fc) is filter cutoff frequency
	const float sn = sinf(omega);
	const float cs = cosf(omega);
	const float alpha = sn / (2 * Q);
	
	float b0 = 0, b1 = 0, b2 = 0, a0 = 0, a1 = 0, a2 = 0;
	
	switch (filterType) {
		case FILTER_LPF:
			b0 = (1 - cs) / 2;
			b1 = 1 - cs;
			b2 = (1 - cs) / 2;
			a0 = 1 + alpha;
			a1 = -2 * cs;
			a2 = 1 - alpha;
			break;
		
		case FILTER_NOTCH:
			b0 = 1;
			b1 = -2 * cs;
			b2 = 1;
			a0 = 1 + alpha;
			a1 = -2 * cs;
			a2 = 1 - alpha;
			break;
	}
	
	/* precompute(normalise) the coefficients */
	filter->b0 = b0 / a0;
	filter->b1 = b1 / a0;
	filter->b2 = b2 / a0;
	filter->a1 = a1 / a0;
	filter->a2 = a2 / a0;
	
	/* clear (zero) initial samples */
	filter->d1 = filter->d2 = 0;
}

/* computes a biquadFilter_t filter on a sample
 * Referenced by https://en.wikipedia.org/wiki/Digital_biquad_filter#Direct_form_2 (direct form 2 normalised difference equation)
 */
float biquadFilterApply(biquadFilter_t *filter, float input)
{
	const float result = filter->b0 * input + filter->d1;
	filter->d1 = filter->b1 * input - filter->a1 * result + filter->d2;
	filter->d2 = filter->b2 * input - filter->a2 * result;
	return result;
}

/* N (float octaves) = log10y / log102 = log2y = log2(f2 / f1) = log2((f0^2 / f1) / f1) = log2((f0^2 / f1^2)) = log2((f0 / f1)^2)
 * =2*log2(f0 / f1)
 */
float filterGetNotchQ(uint16_t centerFreq, uint16_t cutoff)
{
	float octaves = log2f((float)centerFreq / (float)cutoff) * 2;
	return sqrtf(powf(2, octaves)) / (powf(2, octaves) - 1);
}

/* PT1 filter functions
 * Referenced by https://en.wikipedia.org/wiki/Low-pass_filter, pseudocode part
 */
void pt1FilterInit(pt1Filter_t *filter, uint8_t f_cut, float dT)
{
	filter->RC = 1.0f / (2.0f * M_PI_FLOAT * f_cut);
	filter->dT = dT;
	filter->k = filter->dT / (filter->RC + filter->dT);
}

/*
 * for i from 1 to n
 *     y[i] := y[i-1] + k * (x[i] - y[i-1])
 *
 * where y[i] is the current output
 *		 y[i-1] is the previous output
 *       k is the coefficient
 *       x[i] is the input value
 */
float pt1FilterApply(pt1Filter_t *filter, float input)
{
	filter->state = filter->state + filter->k * (input - filter->state);
	return filter->state;
}

float pt1FilterApply4(pt1Filter_t *filter, float input, uint8_t f_cut, float dT)
{
	/* pre calculate and store RC */
	if (!filter->RC) {
		filter->RC = 1.0f / (2.0f * M_PI_FLOAT * f_cut);
		filter->dT = dT;
		filter->k = filter->dT / (filter->RC + filter->dT);
	}
	
	filter->state = filter->state + filter->k * (input - filter->state);
	return filter->state;
}
