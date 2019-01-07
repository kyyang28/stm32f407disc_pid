
#include <stdio.h>

#include "initialisation.h"		// includes gyro.h, acceleration.h, barometer.h, compass.h
//#include "gyro.h"
//#include "acceleration.h"
#include "sensors.h"
#include "target.h"

uint8_t detectedSensors[SENSOR_INDEX_COUNT] = { GYRO_NONE, ACC_NONE, BARO_NONE, MAG_NONE };

bool sensorsAutodetect(const gyroConfig_t *gyroConfig, const accelerometerConfig_t *accelerometerConfig)
{
	/* gyro must be initialised before accelerometer */
	if (!gyroInit(gyroConfig)) {
		return false;
	}

#if defined(F450_QUAD_30A_1000KVMOTOR)
	/* gyroInit() configure gyro.targetLooptime to 125 us, reset it to 1000 us for F450 quad */
	gyro.targetLooptime = 1000;					// F450 1000KV Motor utilises 1000 us of gyro.targetLooptime
#endif
	
//	printf("gyro.taragetLooptime: %u\r\n", gyro.targetLooptime);
	accInit(accelerometerConfig, gyro.targetLooptime);		// gyro.targetLooptime = 1000 us for F450 quad, gyro.targetLooptime = 125 us for F210 quad
	
	if (gyroConfig->gyro_align != ALIGN_DEFAULT) {
		gyro.dev.gyroAlign = gyroConfig->gyro_align;
	}
	
    if (accelerometerConfig->acc_align != ALIGN_DEFAULT) {
        acc.dev.accAlign = accelerometerConfig->acc_align;
    }

//    if (compassConfig->mag_align != ALIGN_DEFAULT) {
//        mag.dev.magAlign = compassConfig->mag_align;
//    }
	
	return true;
}
