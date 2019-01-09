#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>
#include "time.h"

typedef struct imuRuntimeConfig_s {
	float dcm_kp;								// DCM filter proportional gain ( x 10000)
	float dcm_ki;								// DCM filter integral gain ( x 10000)
	uint8_t smallAngle;							// smallAngle for determining whether arming or not
	uint8_t accUnarmedcal;						// turn automatic acc compensation on/off
}imuRuntimeConfig_t;

void imuInit(void);
void taskIMUUpdateAttitude(timeUs_t currentTimeUs);																	

#endif	// __IMU_H
