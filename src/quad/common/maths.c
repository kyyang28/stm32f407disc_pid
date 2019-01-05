
#include <stdio.h>

#include "maths.h"

#if defined(FAST_MATH) || defined(VERY_FAST_MATH)
#if defined(VERY_FAST_MATH)
/* VERY_FAST_MATH: order 7 approximation using Remez Algorithm
 * Remez algorithm seeks the minimax polynomial that approximates a given function in a given interval
 *
 * sin_approx maximum absolute error = 2.305023e-06
 * cos_approx maximum absolute error = 2.857298e-06
 */
#define sinPolyCoef3			-1.666568107e-1f
#define sinPolyCoef5			8.312366210e-3f
#define sinPolyCoef7			-1.849218155e-4f
#define sinPolyCoef9			0
#else
/* FAST_MATH: order 9 approximation using Remez Algorithm */
#define sinPolyCoef3			-1.666665710e-1f                       // Double: -1.666665709650470145824129400050267289858e-1
#define sinPolyCoef5			8.333017292e-3f                        // Double:  8.333017291562218127986291618761571373087e-3
#define sinPolyCoef7			-1.980661520e-4f                       // Double: -1.980661520135080504411629636078917643846e-4
#define sinPolyCoef9			2.600054768e-6f                        // Double:  2.600054767890361277123254766503271638682e-6
#endif

float fastInvSqrt(float x)
{
	float x_half = 0.5f * x;
	int intF = *(int *)&x;
	
	intF = 0x5f3759df - (intF >> 1);		// Magic happens here!!!!!!
	
	x = *(float *)&intF;
	
	x = x * (1.5f - x_half * x * x);		// 1st-order Newton's iteration
//	x = x * (1.5f - x_half * x * x);		// 2nd-order Newton's iteration
	
	return x;
}

//int add(int i, int j)
//{
//  int res = 0;
//  __asm ("ADD %[result], %[input_i], %[input_j]"
//    : [result] "=r" (res)
//    : [input_i] "r" (i), [input_j] "r" (j)
//  );
//  return res;
//	
//	__asm ("ADD R0, %[input_i], %[input_j]"
//    :  /* This is an empty output operand list */
//    : [input_i] "r" (i), [input_j] "r" (j)
//    : "r5","r6","cc","memory" /*Use "r5" instead of "R5" */
//  );
//}

//float invSqrtAsm(float x)
//{
//	__asm {
//		"MOVF R0, %[input_x]"
//		: /* empty output operand list */
//		: [input_x] "r" (x)
//		
//	};
//}

float sin_approx(float x)
{
//	printf("xsin: %f\r\n", x);
	int32_t x_int = x;
	
	/* return 0.0f if x_int is approximately 5 * 360 deg = 1800 */
	if (x_int < -32 || x_int > 32) {
		return 0.0f;
	}
	
	/* Clamp input float value x into -PI ~ +PI */
	while (x > M_PIf) {
		x -= (2.0f * M_PIf);
	}
	
	while (x < -M_PIf) {
		x += (2.0f * M_PIf);
	}
	
	/* Clamp input float value further into -90 ~ +90 degree */
	if (x > (0.5f * M_PIf)) {
		x = (0.5f * M_PIf) - (x - (0.5f * M_PIf));
	} else if (x < -(0.5f * M_PIf)) {
		x = -(0.5f * M_PIf) - (x + (0.5f * M_PIf));
	}
	
	float x2 = x * x;
	
	return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

float cos_approx(float x)
{
//	printf("xcos: %f\r\n", x + (0.5f * M_PIf));
}

float atan2_approx(float y, float x)
{
	
}

float acos_approx(float x)
{
	
}
#endif


/* +-------------------------- Standard Deviation helper functions --------------------------+ */
void devClear(stdev_t *dev)
{
	dev->m_n = 0;
}

void devPush(stdev_t *dev, float x)
{
	dev->m_n++;
	if (dev->m_n == 1) {
		dev->m_oldM = dev->m_newM = x;
		dev->m_oldS = 0.0f;
	}else {
		dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
		dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
		dev->m_oldM = dev->m_newM;
		dev->m_oldS = dev->m_newS;
	}
}

float devVariance(stdev_t *dev)
{
	return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

float devStandardDeviation(stdev_t *dev)
{
	return sqrtf(devVariance(dev));
}
/* +-------------------------- Standard Deviation helper functions --------------------------+ */

int scaleRange(int x, int srcMin, int srcMax, int destMin, int destMax)
{
	long int a = ((long int) destMax - (long int) destMin) * ((long int) x - (long int) srcMin);
	long int b = (long int) srcMax - (long int) srcMin;
	return ((a / b) - (destMax - destMin)) + destMax;
}

float degreesToRadians(int16_t degrees)
{
	return degrees * RAD;			// RAD = ((M_PIf) / 180.0f), M_PIf = 3.14159265358979323846f
}

void buildRotationMatrix(fp_angles_t *delta, float matrix[3][3])
{
//	printf("rollRad: %f\r\n", delta->angles.roll);
//	printf("pitchRad: %f\r\n", delta->angles.pitch);
//	printf("yawRad: %f\r\n", delta->angles.yaw);
	
	float cosx, sinx, cosy, siny, cosz, sinz;
	float coszcosx, sinzcosx, coszsinx, sinzsinx;
	
	/* 
	 * delta->angles.roll = 0
	 * delta->angles.ptich = 0
	 * delta->angles.yaw = 90
	 */
	cosx = cos_approx(delta->angles.roll);
	sinx = sin_approx(delta->angles.roll);

	
}
