#ifndef __GYRO_H
#define __GYRO_H

#include <stdint.h>
#include <stdbool.h>
#include "sensor.h"
#include "axis.h"
#include "accgyro.h"

typedef enum {
	GYRO_NONE = 0,
	GYRO_DEFAULT,
	GYRO_MPU6050,
	GYRO_MPU9250,
	GYRO_FAKE
}gyroSensor_e;

typedef struct gyroConfig_s {
	sensor_align_e gyro_align;						// gyro alignment
	uint8_t gyroMovementCalibrationThreshold;		// people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
	uint8_t gyro_sync_denom;						// gyro sample divider
	uint8_t gyro_lpf;								// gyro LPF settings - values are driver specific, in case of invalid number, a reasonable default ~30-40 Hz is chosen
	uint8_t gyro_soft_lpf_type;
	uint8_t gyro_soft_lpf_hz;
	bool gyro_isr_update;
	bool gyro_use_32khz;
	uint16_t gyro_soft_notch_hz_1;
	uint16_t gyro_soft_notch_cutoff_1;
	uint16_t gyro_soft_notch_hz_2;
	uint16_t gyro_soft_notch_cutoff_2;
}gyroConfig_t;

typedef struct gyro_s {
	gyroDev_t dev;
	uint32_t targetLooptime;
	float gyroADCf[XYZ_AXIS_COUNT];
}gyro_t;

extern gyro_t gyro;
//extern int32_t gyroADC[XYZ_AXIS_COUNT];			// TODO: testing for now, remove later

bool gyroInit(const gyroConfig_t *gyroConfigToUse);
void gyroSetCalibrationCycles(void);
bool isGyroCalibrationComplete(void);
void gyroInitFilters(void);
void gyroUpdate(void);

#endif	// __GYRO_H
