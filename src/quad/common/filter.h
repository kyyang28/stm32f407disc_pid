#ifndef __FILTER_H
#define __FILTER_H

#include <stdint.h>

/* Referenced by https://en.wikipedia.org/wiki/Low-pass_filter, pseudocode part */
typedef struct pt1Filter_s {
	float state;
	float k;		// k := dT / (RC + dT)
	float RC;		// time constant RC
	float dT;		// time interval dt
}pt1Filter_t;

/* this  holds the data requied to update samples thru a filter */
typedef struct biquadFilter_s {
	float b0, b1, b2, a1, a2;
	float d1, d2;
}biquadFilter_t;

/* TODO: leave FIR filter for now */

typedef enum {
	FILTER_PT1 = 0,
	FILTER_BIQUAD,
	FILTER_FIR
}filterType_e;

typedef enum {
	FILTER_LPF,
	FILTER_NOTCH
}biquadFilterType_e;

typedef float (*filterApplyFnPtr)(void *filter, float input);

/* NULL filter function */
float nullFilterApply(void *filter, float input);

/* biquad filter functions */
void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate);
void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType);
float biquadFilterApply(biquadFilter_t *filter, float input);
float filterGetNotchQ(uint16_t centerFreq, uint16_t cutoff);

/* PT1 filter functions */
void pt1FilterInit(pt1Filter_t *filter, uint8_t f_cut, float dT);
float pt1FilterApply(pt1Filter_t *filter, float input);
float pt1FilterApply4(pt1Filter_t *filter, float input, uint8_t f_cut, float dT);

#endif	// __FILTER_H
