
#include <stdio.h>

#include "maths.h"
#include "axis.h"

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

/* fastInvSqrt does the same performance as built-in sqrtf */
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

float sinApprox(float x)
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

float cosApprox(float x)
{
//	printf("xcos: %f\r\n", x + (0.5f * M_PIf));
	return sinApprox(x + (0.5f * M_PIf));
}

/* Max absolute error 0,000027 degree
 * atan2Approx maximum absolute error = 7.152557e-07 rads (4.098114e-05 degree)
 *
 * http://www.dsprelated.com/showthread/comp.dsp/21872-1.php
 */
float atan2Approx(float y, float x)
{
	#define atanPolyCoef1  3.14551665884836e-07f
	#define atanPolyCoef2  0.99997356613987f
	#define atanPolyCoef3  0.14744007058297684f
	#define atanPolyCoef4  0.3099814292351353f
	#define atanPolyCoef5  0.05030176425872175f
	#define atanPolyCoef6  0.1471039133652469f
	#define atanPolyCoef7  0.6444640676891548f

    float res, absX, absY;
    absX = fabsf(x);
    absY = fabsf(y);

    res  = MAX(absX, absY);

    if (res) {
		res = MIN(absX, absY) / res;
	} else {
		res = 0.0f;
	}
	
    res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);
    
	if (absY > absX) {
		res = (M_PIf / 2.0f) - res;
	}
	
    if (x < 0) {
		res = M_PIf - res;
	}
	
    if (y < 0) {
		res = -res;
	}
	
    return res;
}

/* http://developer.download.nvidia.com/cg/acos.html
 * Handbook of Mathematical Functions
 * M. Abramowitz and I.A. Stegun, Ed.
 * acos_approx maximum absolute error = 6.760856e-05 rads (3.873685e-03 degree)
 */
float acosApprox(float x)
{
	float xTmp = fabsf(x);
	
	float result = sqrtf(1.0f - xTmp) * (1.5707288f + xTmp * (-0.2121144f + xTmp * (0.0742610f + (-0.0187293f * xTmp))));
	
	if (x < 0.0f) {
		return M_PIf - result;
	} else {
		return result;
	}
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

/* Build Rotation Matrix, order: M_roll * M_pitch * M_yaw */
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
	 *
	 * Roll resides along the X-axis
	 * Pitch resides along the Y-axis
	 * Yaw resides along the Z-axis
	 */
	cosx = cosApprox(delta->angles.roll);
	sinx = sinApprox(delta->angles.roll);
	cosy = cosApprox(delta->angles.pitch);
	siny = sinApprox(delta->angles.pitch);
	cosz = cosApprox(delta->angles.yaw);
	sinz = sinApprox(delta->angles.yaw);
	
//	printf("cosx: %f\r\n", cosx);
//	printf("sinx: %f\r\n", sinx);
//	printf("cosy: %f\r\n", cosy);
//	printf("siny: %f\r\n", siny);
//	printf("cosz: %f\r\n", cosz);
//	printf("sinz: %f\r\n", sinz);
	
	coszcosx = cosz * cosx;
	sinzcosx = sinz * cosx;
	coszsinx = sinx * cosz;
	sinzsinx = sinx * sinz;
//	printf("coszcosx: %f\r\n", coszcosx);
//	printf("sinzcosx: %f\r\n", sinzcosx);
//	printf("coszsinx: %f\r\n", coszsinx);
//	printf("sinzsinx: %f\r\n", sinzsinx);
	
#if defined(RzRyRx)
	/* Coordinate-based rotation matrix, RzRyRx, first Roll axis, then Pitch axis, lastly Yaw axis */

	/*
						+-																		    -+ +-   -+
						|	cosycosz		cosxsinz + coszsinysinx			sinxsinz - cosxcoszsiny  | |  x  |
		(RzRyRx)vec  =  |  -cosysinz		cosxcosz - sinxsinysinz			coszsinx + cosxsinysinz  | |  y  |
						|	  siny				   -cosysinx					   cosycosx          | |  z  |
						+-																		    -+ +-   -+	
	*/
	matrix[0][X] = cosz * cosy;
	matrix[0][Y] = (sinzcosx) + (coszsinx * siny);
	matrix[0][Z] = (sinzsinx) - (coszcosx * siny);
	matrix[1][X] = -cosy * sinz;
	matrix[1][Y] = (coszcosx) - (sinzsinx * siny);
	matrix[1][Z] = (coszsinx) + (sinzcosx * siny);
	matrix[2][X] = siny;
	matrix[2][Y] = -sinx * cosy;
	matrix[2][Z] = cosy * cosx;
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
#else
	/* BF-3.1.7 way, weired */
	matrix[0][X] = cosz * cosy;
	matrix[0][Y] = -cosy * sinz;
	matrix[0][Z] = siny;
	matrix[1][X] = (sinzcosx) + (coszsinx * siny);
	matrix[1][Y] = (coszcosx) - (sinzsinx * siny);
	matrix[1][Z] = -sinx * cosy;
	matrix[2][X] = (sinzsinx) - (coszcosx * siny);
	matrix[2][Y] = (coszsinx) + (sinzcosx * siny);
	matrix[2][Z] = cosy * cosx;
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
#endif	
//	printf("matrix[0][X]: %f\r\n", matrix[0][X]);
//	printf("matrix[0][Y]: %f\r\n", matrix[0][Y]);
//	printf("matrix[0][Z]: %f\r\n", matrix[0][Z]);
//	printf("matrix[1][X]: %f\r\n", matrix[1][X]);
//	printf("matrix[1][Y]: %f\r\n", matrix[1][Y]);
//	printf("matrix[1][Z]: %f\r\n", matrix[1][Z]);
//	printf("matrix[2][X]: %f\r\n", matrix[2][X]);
//	printf("matrix[2][Y]: %f\r\n", matrix[2][Y]);
//	printf("matrix[2][Z]: %f\r\n", matrix[2][Z]);
}
