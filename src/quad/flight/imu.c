
#include <stdio.h>

#include "imu.h"
#include "runtime_config.h"
#include "sensors.h"
#include "acceleration.h"

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
