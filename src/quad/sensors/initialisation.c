
#include "initialisation.h"		// includes gyro.h, acceleration.h, barometer.h, compass.h
//#include "gyro.h"
//#include "acceleration.h"
#include "sensors.h"
#include <stdio.h>

uint8_t detectedSensors[SENSOR_INDEX_COUNT] = { GYRO_NONE, ACC_NONE, BARO_NONE, MAG_NONE };

bool sensorsAutodetect(const gyroConfig_t *gyroConfig, const accelerometerConfig_t *accelerometerConfig)
{
	/* gyro must be initialised before accelerometer */
	if (!gyroInit(gyroConfig)) {
		return false;
	}
	
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//	accInit(accelerometerConfig, gyro.targetLooptime);
	
	if (gyroConfig->gyro_align != ALIGN_DEFAULT) {
		gyro.dev.gyroAlign = gyroConfig->gyro_align;
	}
	
//    if (accelerometerConfig->acc_align != ALIGN_DEFAULT) {
//        acc.dev.accAlign = accelerometerConfig->acc_align;
//    }
//    if (compassConfig->mag_align != ALIGN_DEFAULT) {
//        mag.dev.magAlign = compassConfig->mag_align;
//    }
	
	return true;
}
