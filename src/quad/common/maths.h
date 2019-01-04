#ifndef __MATHS_H
#define __MATHS_H

#include <math.h>
#include <stdint.h>

/* comment these two lines to use glibc sinf and cosf.
 * uncomment these to use fast and very fast sin and cos approximations
 */
#define FAST_MATH							// approximation under order 9
#define VERY_FAST_MATH						// approximation under order 7

// Use floating point M_PI instead explicitly.
#define M_PIf       		(3.14159265358979323846f)

#define RAD    				((M_PIf) / 180.0f)

#define MIN(a, b) 			((a) < (b) ? (a) : (b))
#define MAX(a, b) 			((a) > (b) ? (a) : (b))
#define ABS(x) 				((x) > 0 ? (x) : -(x))

#define POWER3(x)			((x) * (x) * (x))

typedef struct fp_angles {
	float roll;
	float pitch;
	float yaw;
}fp_angles_def;

typedef union {
	float raw[3];
	fp_angles_def angles;
}fp_angles_t;

typedef struct stdev_s {
	float m_oldM, m_newM, m_oldS, m_newS;
	int m_n;
}stdev_t;

static inline int constrain(int amt, int low, int high)
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}

static inline float constrainf(float amt, float low, float high)
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}

void devClear(stdev_t *dev);
void devPush(stdev_t *dev, float x);
float devVariance(stdev_t *dev);
float devStandardDeviation(stdev_t *dev);

int scaleRange(int x, int srcMin, int srcMax, int destMin, int destMax);

#if defined(FAST_MATH) || defined(VERY_FAST_MATH)
float sin_approx(float x);
float cos_approx(float x);
float atan2_approx(float y, float x);
float acos_approx(float x);
#else
#define sin_approx(x)				sinf(x)
#define cos_approx(x)				cosf(x)
#define atan2_approx(y,x)			atan2f(y,x)
#define acos_approx(x)				acosf(x)
#define tan_approx(x)				tanf(x)
#endif

float degreesToRadians(int16_t degrees);
void buildRotationMatrix(fp_angles_t *delta, float matrix[3][3]);


#endif	// __MATHS_H
