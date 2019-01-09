
#include <stdio.h>

#include "imu.h"
#include "runtime_config.h"
#include "sensors.h"
#include "acceleration.h"
#include "maths.h"

float smallAngleCosZ = 0;

static imuRuntimeConfig_t imuRuntimeConfig;

void imuInit(void)
{
	/* smallAngle for determining whether the quad is able to ARM or not */
//	smallAngleCosZ = cosApprox(degreesToRadians(imuRuntimeConfig));
}

static void imuCalculateEstimatedAttitude(timeUs_t currentTimeUs)
{
	
}

void taskIMUUpdateAttitude(timeUs_t currentTimeUs)
{
	if (sensors(SENSOR_ACC) && acc.isAccelUpdatedAtLeastOnce) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		imuCalculateEstimatedAttitude(currentTimeUs);
	} else {
		acc.accSmooth[X] = 0;
		acc.accSmooth[Y] = 0;
		acc.accSmooth[Z] = 0;
	}
}
