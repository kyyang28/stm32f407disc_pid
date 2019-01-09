#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>
#include "time.h"
#include "pid.h"

typedef struct accDeadband_s {
	uint8_t xy;									// set the acc deadband for xy-axis
	uint8_t z;									// set the acc deadband for z-axis, this ignores small accelerations
}accDeadband_t;

typedef struct imuConfig_s {
	uint16_t dcm_kp;							// DCM (Direction Cosine Matrix) filter integral gain ( x 10000)
	uint16_t dcm_ki;							// DCM (Direction Cosine Matrix) filter proportional gain ( x 10000)
	uint8_t smallAngle;							// smallAngle for determining whether arming or not
	uint8_t accUnarmedcal;						// turn automatic acc compensation on/off
	accDeadband_t accDeadband;
}imuConfig_t;

typedef struct imuRuntimeConfig_s {
	float dcm_kp;
	float dcm_ki;
	uint8_t smallAngle;							// smallAngle for determining whether arming or not
	uint8_t accUnarmedcal;
	accDeadband_t accDeadband;
}imuRuntimeConfig_t;

typedef struct throttleCorrectionConfig_s {
	uint16_t throttleCorrectionAngle;			// the angle when the throttle correction is maximal in 0.1 degrees, e.g. 225 = 22.5; 450 = 45.0 deg
	uint16_t throttleCorrectionValue;			// the correction that will be applied at throttleCorrectionAngle
}throttleCorrectionConfig_t;

void imuInit(void);
void imuConfigure(imuConfig_t *imuConfig, struct pidProfile_s *initialPidProfile, uint16_t throttleCorrectionAngle);
void taskIMUUpdateAttitude(timeUs_t currentTimeUs);																	

#endif	// __IMU_H
