#ifndef __SENSOR_H
#define __SENSOR_H

#include <stdint.h>
#include <stdbool.h>
//#include "accgyro.h"

typedef bool (*sensorInitFuncPtr)(void);                    // sensor init prototype
typedef bool (*sensorReadFuncPtr)(int16_t *data);           // sensor read and align prototype
typedef bool (*sensorInterruptFuncPtr)(void);
struct accDev_s;
typedef void (*sensorAccInitFuncPtr)(struct accDev_s *acc);
typedef bool (*sensorAccReadFuncPtr)(struct accDev_s *acc);
struct gyroDev_s;
typedef void (*sensorGyroInitFuncPtr)(struct gyroDev_s *gyro);
typedef bool (*sensorGyroReadFuncPtr)(struct gyroDev_s *gyro);
typedef bool (*sensorGyroUpdateFuncPtr)(struct gyroDev_s *gyro);
typedef bool (*sensorGyroReadDataFuncPtr)(struct gyroDev_s *gyro, int16_t *data);
typedef bool (*sensorGyroInterruptStatusFuncPtr)(struct gyroDev_s *gyro);

typedef enum {
	ALIGN_DEFAULT = 0,                                      // driver-provided alignment
	CW0_DEG = 1,
	CW90_DEG = 2,
	CW180_DEG = 3,
	CW270_DEG = 4,
	CW0_DEG_FLIP = 5,
	CW90_DEG_FLIP = 6,
	CW180_DEG_FLIP = 7,
	CW270_DEG_FLIP = 8
} sensor_align_e;

#endif	// __SENSOR_H
