#ifndef __COMPASS_H
#define __COMPASS_H

typedef enum {
	MAG_DEFAULT = 0,
	MAG_NONE = 1,
	MAG_HMC5883 = 2,
	MAG_AK8975 = 3,
	MAG_AK8963 = 4
}magSensor_e;

#endif	// __COMPASS_H
