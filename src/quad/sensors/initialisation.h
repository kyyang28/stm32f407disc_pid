#ifndef __INITIALISATION_H
#define __INITIALISATION_H

#include <stdbool.h>
#include "gyro.h"
#include "acceleration.h"
#include "barometer.h"
#include "compass.h"

bool sensorsAutodetect(const gyroConfig_t *gyroConfig, const accelerometerConfig_t *accelerometerConfig);

#endif	// __INITIALISATION_H
