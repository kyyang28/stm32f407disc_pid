#ifndef __COMPASSDEV_H
#define __COMPASSDEV_H

#include "sensor.h"

typedef struct magDev_s {
	sensorInitFuncPtr init;						// mag initialise function pointer
	sensorReadFuncPtr read;						// mag read function pointer
	sensor_align_e magAlign;
}magDev_t;

#endif	// __COMPASSDEV_H
