
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "gyro.h"
#include "accgyro_mpu6050.h"
//#include "accgyro_i2c_mpu9250.h"
#include "accgyro_spi_mpu9250.h"
#include "acceleration.h"
#include "sensors.h"
#include "runtime_config.h"
#include "target.h"
#include "filter.h"
#include "axis.h"
#include "boardAlignment.h"

acc_t acc;				// acc access functions

static flightDynamicsTrims_t *accelerationTrims;
static uint16_t accLpfCutHz = 0;
static biquadFilter_t accFilter[XYZ_AXIS_COUNT];

/* Calibration is processed in the main loop, which decreases at each cycle down to 0, then we step into the normal mode */
static uint16_t calibratingA = 0;

bool isAccelerationCalibrationComplete(void)
{
	return calibratingA == 0;
}

static bool isOnFirstAccelerationCalibrationCycle(void)
{
	return calibratingA == CALIBRATING_ACC_CYCLES;
}

static bool isOnFinalAccelerationCalibrationCycle(void)
{
	return calibratingA == 1;
}

void ResetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims)
{
    rollAndPitchTrims->values.roll = 0;
    rollAndPitchTrims->values.pitch = 0;
}

void setAccelerationTrims(flightDynamicsTrims_t *accelerationTrimsToUse)
{
	accelerationTrims = accelerationTrimsToUse;
}

void setAccelerationFilter(uint16_t initialAccLpfCutHz)
{
	accLpfCutHz = initialAccLpfCutHz;
	
	if (acc.accSamplingInterval) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
			biquadFilterInitLPF(&accFilter[axis], accLpfCutHz, acc.accSamplingInterval);
		}
	}
}

static void performAccelerationCalibration(rollAndPitchTrims_t *rollAndPitchTrims)
{
	static int32_t a[3];
	
	for (int axis = 0; axis < 3; axis++) {
		/* Reset a[axis] at start of calibration */
		if (isOnFirstAccelerationCalibrationCycle()) {
			a[axis] = 0;
		}
		
		/* Sum up CALIBRATING_ACC_CYCLES readings */
		a[axis] += acc.accSmooth[axis];
		
		/* Reset global variables to prevent other code from using un-calibrated data */
		acc.accSmooth[axis] = 0;
		accelerationTrims->raw[axis] = 0;
	}
	
	if (isOnFinalAccelerationCalibrationCycle()) {
		/* Calculate average, shift Z down by acc_1G and store values in EEPROM at the end of calibration */
		accelerationTrims->raw[X] = (a[X] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
		accelerationTrims->raw[Y] = (a[Y] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
		accelerationTrims->raw[Z] = (a[Z] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES - acc.dev.acc_1G;	// acc.dev.acc_1G = 4096 (AFS_SEL = 2, +/- 8g)

		/* Reset roll and pitch trims */
		ResetRollAndPitchTrims(rollAndPitchTrims);
		
		/* Write contents to EEPROM and beep to notify */
//		saveConfigAndNotify();
	}
	
	/* ACC calibration decreasing counter */
	calibratingA--;
}

static void applyAccelerationTrims(const flightDynamicsTrims_t *accelerationTrims)
{
	/* raw[X-Y-Z] share the same memory space as accelerationTrims->values->roll, pitch, yaw */
	acc.accSmooth[X] -= accelerationTrims->raw[X];
	acc.accSmooth[Y] -= accelerationTrims->raw[Y];
	acc.accSmooth[Z] -= accelerationTrims->raw[Z];
}

bool accDetect(accDev_t *dev, accelerationSensor_e accHardwareToUse)
{
	accelerationSensor_e accHardware;
	
retry:
//	printf("accAlign: %d\r\n", dev->accAlign);
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
		if (mpu9250SpiAccDetect(dev))
//		if (mpu9250AccDetect(dev) || mpu9250SpiAccDetect(dev))
#elif defined(USE_ACC_I2C_MPU9250)
		if (mpu9250I2CAccDetect(dev))
#endif
		{
#ifdef ACC_MPU9250_ALIGN
			dev->accAlign = ACC_MPU9250_ALIGN;
#endif
			switch (dev->mpuDetectionResult.sensor) {
				case MPU_9250_SPI:
					accHardware = ACC_MPU9250;
					break;
					
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
	
	acc.dev.acc_1G = 256;		// set acc_1G to default
//	printf("acc_1G before: %u\r\n", acc.dev.acc_1G);		// acc.dev.acc_1G = 256
	acc.dev.init(&acc.dev);		// set acc_1G to 4096
//	printf("acc_1G after: %u\r\n", acc.dev.acc_1G);			// acc.dev.acc_1G = 4096
	
	/* Set the acc sampling interval based on the gyro sampling interval
	 *
	 * gyroSamplingInverval = gyro.targetLooptime = 125 for F210 quad
	 * gyroSamplingInverval = gyro.targetLooptime = 1000 for F450 quad
	 */
	switch (gyroSamplingInverval) {
		case 500:
		case 375:
		case 250:
		case 125:
			acc.accSamplingInterval = 1000;
			break;
		
		case 1000:
		default:
			acc.accSamplingInterval = 1000;
	}
	
//	printf("accSamplingInterval: %u\r\n", acc.accSamplingInterval);		// acc.accSamplingInterval = 1000
	
	/* Initialise ACC biquad LPF */
	if (accLpfCutHz) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
			biquadFilterInitLPF(&accFilter[axis], accLpfCutHz, acc.accSamplingInterval);
		}
	}
	
	return true;
}

void accUpdate(rollAndPitchTrims_t *rollAndPitchTrims)
{
	if (!acc.dev.read(&acc.dev)) {
		return;
	}
	
	acc.isAccelUpdatedAtLeastOnce = true;
	
	for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
		acc.accSmooth[axis] = acc.dev.ADCRaw[axis];
//		printf("%u\t", acc.accSmooth[axis]);
//		if (axis == 2) printf("\r\n");
	}
	
//	printf("accLpfCutHz: %u\r\n", accLpfCutHz);
	
	/* accLpfCutHz = 10.0 Hz */
	if (accLpfCutHz) {
		for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
			acc.accSmooth[axis] = lrintf(biquadFilterApply(&accFilter[axis], (float)acc.accSmooth[axis]));
		}
	}
	
//	printf("accAlign: %u\r\n", acc.dev.accAlign);		// accAlign = ALIGN_DEFAULT
	
	/* Align accelerometer sensor */
	alignSensors(acc.accSmooth, acc.dev.accAlign);
	
	/* Calibrate ACC */
	if (!isAccelerationCalibrationComplete()) {
		performAccelerationCalibration(rollAndPitchTrims);
	}
	
	/* TODO: Might need to implement inflight acc calibration */
	
	
	/* Adjust acc.accSmooth[X,Y,Z] by subtracting acceleration trims */
	applyAccelerationTrims(accelerationTrims);
	
//	for (int axis = 0; axis < 3; axis++) {
//		printf("%d\t", acc.accSmooth[axis]);
//		if (axis == 2) printf("\r\n");
//	}
}
