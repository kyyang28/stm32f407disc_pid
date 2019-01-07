#ifndef __ACCELERATION_H
#define __ACCELERATION_H

#include "sensor.h"
#include "sensors.h"
#include "accgyro.h"

/* Type of accelerometer used/detected */
typedef enum {
	ACC_DEFAULT,			// 0
	ACC_NONE,				// 1
	ACC_MPU6050,			// 2
	ACC_MPU6500,			// 3
	ACC_MPU9250,			// 4
	ACC_FAKE				// 5
}accelerationSensor_e;

typedef struct acc_s {
	accDev_t dev;
	uint32_t accSamplingInterval;
	int32_t accSmooth[XYZ_AXIS_COUNT];
	bool isAccelUpdatedAtLeastOnce;
}acc_t;

extern acc_t acc;

typedef struct rollAndPitchTrims_s {
	int16_t roll;
	int16_t pitch;
}rollAndPitchTrims_t_def;

typedef union rollAndPitchTrims_u {
	int16_t raw[2];
	rollAndPitchTrims_t_def values;
}rollAndPitchTrims_t;

typedef struct accelerometerConfig_s {
	uint16_t acc_lpf_hz;		// cutoff frequency for the low pass filter used on the acc z-axis for althold in Hz
	sensor_align_e acc_align;	// acc alignment
	uint8_t acc_hardware;		// which acc hardware to use on boards with more than one device
	flightDynamicsTrims_t accZero;
	rollAndPitchTrims_t accelerometerTrims;
}accelerometerConfig_t;

bool accInit(const accelerometerConfig_t *accelerometerConfig, uint32_t gyroSamplingInverval);
void accUpdate(rollAndPitchTrims_t *rollAndPitchTrims);

void ResetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims);
void setAccelerationTrims(flightDynamicsTrims_t *accelerationTrimsToUse);
void setAccelerationFilter(uint16_t initialAccLpfCutHz);

#endif	// __ACCELERATION_H
