
#include <string.h>
#include "gyro.h"
#include "accgyro_mpu6050.h"
#include "accgyro_i2c_mpu9250.h"
#include "acceleration.h"
#include "sensors.h"
#include "runtime_config.h"
#include "target.h"

acc_t acc;				// acc access functions

bool accDetect(accDev_t *dev, accelerationSensor_e accHardwareToUse)
{
	accelerationSensor_e accHardware;
	
retry:	
	dev->accAlign = ALIGN_DEFAULT;
	
	switch (accHardwareToUse) {
		case ACC_DEFAULT:
			;		// fallthrough
		case ACC_MPU6050:
#ifdef USE_ACC_MPU6050
			if (mpu6050AccDetect(dev)) {
#ifdef ACC_MPU6050_ALIGN
				dev->accAlign = ACC_MPU6050_ALIGN;
#endif
				accHardware = ACC_MPU6050;
				break;
			}
#endif
			;
		case ACC_MPU6500:
		case ACC_MPU9250:
#ifdef USE_ACC_SPI_MPU9250
//		if (mpu9250AccDetect(dev) || mpu9250SpiAccDetect(dev))
#elif defined(USE_ACC_I2C_MPU9250)
		if (mpu9250I2CAccDetect(dev))
#endif
		{
#ifdef ACC_MPU9250_ALIGN
			dev->accAlign = ACC_MPU9250_ALIGN;
#endif
			switch (dev->mpuDetectionResult.sensor) {
				case MPU_9250_I2C:
					accHardware = ACC_MPU9250;
					break;
				default:
					accHardware = ACC_MPU6500;
			}
			break;
		}
		
		;
		case ACC_FAKE:
			break;
		
		;	// fallthrough
		case ACC_NONE:		// disable ACC
			accHardware = ACC_NONE;
			break;
	}
	
	/* Found anything? Check if error or ACC is really missing */
	if (accHardware == ACC_NONE && accHardwareToUse != ACC_DEFAULT && accHardwareToUse != ACC_NONE) {
		/* Nothing was found and we have a forced sensor that isn't present */
		accHardwareToUse = ACC_DEFAULT;
		goto retry;
	}
	
	if (accHardware == ACC_NONE) {
		return false;
	}
	
	detectedSensors[SENSOR_INDEX_ACC] = accHardware;
	sensorSet(SENSOR_ACC);
	
	return true;
}

bool accInit(const accelerometerConfig_t *accelerometerConfig, uint32_t gyroSamplingInverval)
{
	memset(&acc, 0, sizeof(acc));
	
	/* copy over the common gyro mpu settings */
	acc.dev.mpuConfiguration = gyro.dev.mpuConfiguration;
	acc.dev.mpuDetectionResult = gyro.dev.mpuDetectionResult;
	if (!accDetect(&acc.dev, accelerometerConfig->acc_hardware)) {
		return false;
	}
		
	return true;
}

void accUpdate(rollAndPitchTrims_t *rollAndPitchTrims)
{
	if (!acc.dev.read(&acc.dev)) {
		return;
	}
}

void ResetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims)
{
    rollAndPitchTrims->values.roll = 0;
    rollAndPitchTrims->values.pitch = 0;
}
