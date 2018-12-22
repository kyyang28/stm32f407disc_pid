
#include "maths.h"

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
