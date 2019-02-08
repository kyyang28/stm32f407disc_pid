#ifndef __COMPASS_H
#define __COMPASS_H

#include <stdint.h>
#include "compassDev.h"
#include "axis.h"

typedef enum {
	MAG_DEFAULT = 0,
	MAG_NONE = 1,
	MAG_HMC5883 = 2,
	MAG_AK8975 = 3,
	MAG_AK8963 = 4
}magSensor_e;

typedef struct mag_s {
	magDev_t dev;
	int32_t magADC[XYZ_AXIS_COUNT];
	float magneticDeclination;
}mag_t;

extern mag_t mag;

#endif	// __COMPASS_H
