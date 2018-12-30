
#include <string.h>
#include "gyro.h"
#include "exti.h"
#include "target.h"
#include "accgyro_mpu6050.h"
#include "accgyro_i2c_mpu9250.h"			// mpu9250 I2C implementation
#include "accgyro_spi_mpu9250.h"			// mpu9250 SPI implementation
#include "gyro_sync.h"
#include "sensors.h"
#include "runtime_config.h"
#include "maths.h"
#include "led.h"
#include "filter.h"

#include <stdio.h>			// printf

gyro_t gyro;				// gyro access functions

//int32_t gyroADC[XYZ_AXIS_COUNT];				// TODO: testing for now, remove later
static int32_t gyroADC[XYZ_AXIS_COUNT];		// TODO: uncomment back later
static int32_t gyroZero[XYZ_AXIS_COUNT] = { 0, 0, 0 };

static uint16_t calibratingG = 0;
static const gyroConfig_t *gyroConfig;

static filterApplyFnPtr softLpfFilterApplyFn;
static void *softLpfFilter[3];
static filterApplyFnPtr notchFilter1ApplyFn;
static void *notchFilter1[3];
static filterApplyFnPtr notchFilter2ApplyFn;
static void *notchFilter2[3];

static const extiConfig_t *selectMPUIntExtiConfig(void)
{
#if defined(MPU_INT_EXTI)
	static const extiConfig_t mpuIntExtiConfig = { .tag = IO_TAG(MPU_INT_EXTI) };
	return &mpuIntExtiConfig;
#endif
}

static bool gyroDetect(gyroDev_t *dev)
{
	gyroSensor_e gyroHardware = GYRO_DEFAULT;
	
	dev->gyroAlign = ALIGN_DEFAULT;
	
	switch (gyroHardware) {
		case GYRO_DEFAULT:
#ifdef USE_GYRO_MPU6050
		case GYRO_MPU6050:
			if (mpu6050GyroDetect(dev)) {
				gyroHardware = GYRO_MPU6050;			// GYRO_MPU6050 = 2
//#ifdef GYRO_MPU6050_ALIGN
//				dev->gyroAlign = GYRO_MPU6050_ALIGN;
//#endif
				break;
			}
#endif
			
#ifdef USE_GYRO_I2C_MPU9250				// defined in target.h
		case GYRO_MPU9250:
//			printf("USE_GYRO_I2C_MPU9250: %s, %d\r\n", __FUNCTION__, __LINE__);
			if (mpu9250I2CGyroDetect(dev)) {
				gyroHardware = GYRO_MPU9250;			// GYRO_MPU9250 = 3
//#ifdef GYRO_MPU9250_ALIGN
//				dev->gyroAlign = GYRO_MPU9250_ALIGN;
//#endif
				break;
			}
#endif
			
#ifdef USE_GYRO_SPI_MPU9250
		case GYRO_MPU9250:
//			printf("USE_GYRO_SPI_MPU9250: %s, %d\r\n", __FUNCTION__, __LINE__);
			if (mpu9250SpiGyroDetect(dev)) {
				gyroHardware = GYRO_MPU9250;
#ifdef GYRO_MPU9250_ALIGN
				dev->gyroAlign = GYRO_MPU9250_ALIGN;
#endif
				break;
			}
#endif
			
		default:
			gyroHardware = GYRO_NONE;					// GYRO_NONE = 0
	}
	
	if (gyroHardware == GYRO_NONE) {
		return false;
	}
	
//	printf("gyroHardware: %d, %s, %d\r\n", gyroHardware, __FUNCTION__, __LINE__);
	detectedSensors[SENSOR_INDEX_GYRO] = gyroHardware;
//	printf("detectedSensors[%d]: %d, %s, %d\r\n", SENSOR_INDEX_GYRO, detectedSensors[SENSOR_INDEX_GYRO], __FUNCTION__, __LINE__);
	sensorSet(SENSOR_GYRO);
	
	return true;
}

bool gyroInit(const gyroConfig_t *gyroConfigToUse)
{
	gyroConfig = gyroConfigToUse;
	memset(&gyro, 0, sizeof(gyro));

#if defined(USE_GYRO_MPU6050) || defined(USE_GYRO_I2C_MPU9250) || defined(USE_GYRO_SPI_MPU9250)
	gyro.dev.mpuIntExtiConfig = selectMPUIntExtiConfig();
	mpuDetect(&gyro.dev);
	mpuReset = gyro.dev.mpuConfiguration.reset;
#endif
	
	if (!gyroDetect(&gyro.dev)) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		return false;
	}
	
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	
	switch (detectedSensors[SENSOR_INDEX_GYRO]) {
		default:
			/* gyro does not support 32khz
		     * cast away constness, legitimate as this is cross-validation
		     */
			((gyroConfig_t *)gyroConfig)->gyro_use_32khz = false;
			break;
		
//		case GYRO_MPU6500:
		case GYRO_MPU9250:
			/* do nothing, as gyro supports 32khz */
			break;
	}
	
	/* Must set gyro sample rate before initialisation
	 *
	 * gyroConfig->gyro_lpf = GYRO_LPF_256HZ = 0
	 * gyroConfig->gyro_sync_denom = 1
	 */
	gyro.targetLooptime = gyroSetSampleRate(&gyro.dev, gyroConfig->gyro_lpf, gyroConfig->gyro_sync_denom, gyroConfig->gyro_use_32khz);
	//	printf("gyro.targetLooptime: %u, %s, %d\r\n", gyro.targetLooptime, __FUNCTION__, __LINE__);		// gyro.targetLooptime = 500 us
	gyro.dev.lpf = gyroConfig->gyro_lpf;
	gyro.dev.init(&gyro.dev);				// gyro.dev.init function is initialised inside mpu6050GyroDetect() function
	gyro.dev.calibrationFlag = false;		// added by YANG for testing purpose
	
	gyroInitFilters();
	return true;
}

void gyroInitFilters(void)
{
	static biquadFilter_t gyroFilterLPF[XYZ_AXIS_COUNT];
	static pt1Filter_t gyroFilterPt1[XYZ_AXIS_COUNT];
//	static firFilterDenoise_t gyroDenoiseState[XYZ_AXIS_COUNT];		// TODO: leave FIR filter for now
	static biquadFilter_t gyroFilterNotch_1[XYZ_AXIS_COUNT];
	static biquadFilter_t gyroFilterNotch_2[XYZ_AXIS_COUNT];
	
	softLpfFilterApplyFn = nullFilterApply;
	notchFilter1ApplyFn = nullFilterApply;
	notchFilter2ApplyFn = nullFilterApply;
	
	/*
	 * gyro.targetLooptime is the gyro sampling period in microseconds
	 * gyro.targetLooptime * 0.000001f converts the gyro sampling period to seconds
     * (1.0f / (gyro.targetLooptime * 0.000001f)) represents the sampling frequency of the gyro
	 * (1.0f / (gyro.targetLooptime * 0.000001f)) / 2 represents the Nyquist frequency of the gyro, which is half of the sampling frequency.
	 */
	uint32_t gyroFrequencyNyquist = (1.0f / (gyro.targetLooptime * 0.000001f)) / 2;			// no rounding needed
	
	/* Initialise gyro lowpass filter, could be either BIQUAD or PT1, FIR will be implemented later */
	if (gyroConfig->gyro_soft_lpf_hz && gyroConfig->gyro_soft_lpf_hz <= gyroFrequencyNyquist) {	// initialisation needs to happen once sampling rate is known
		if (gyroConfig->gyro_soft_lpf_type == FILTER_BIQUAD) {
			softLpfFilterApplyFn = (filterApplyFnPtr)biquadFilterApply;
			for (int axis = 0; axis < 3; axis++) {
				softLpfFilter[axis] = &gyroFilterLPF[axis];
				biquadFilterInitLPF(softLpfFilter[axis], gyroConfig->gyro_soft_lpf_hz, gyro.targetLooptime);
			}
		}else if (gyroConfig->gyro_soft_lpf_type == FILTER_PT1) {
			softLpfFilterApplyFn = (filterApplyFnPtr)pt1FilterApply;
			const float gyroDt = (float)gyro.targetLooptime * 0.000001f;
			for (int axis = 0; axis < 3; axis++) {
				softLpfFilter[axis] = &gyroFilterPt1[axis];
				pt1FilterInit(softLpfFilter[axis], gyroConfig->gyro_soft_lpf_hz, gyroDt);
			}
		}else {
			/* TODO: FIR filter implementation */
		}
	}
	
	/* Initialise gyro notch 1 filter */
	if (gyroConfig->gyro_soft_notch_hz_1 && gyroConfig->gyro_soft_notch_hz_1 <= gyroFrequencyNyquist) {
		notchFilter1ApplyFn = (filterApplyFnPtr)biquadFilterApply;
		const float gyroSoftNotchQ1 = filterGetNotchQ(gyroConfig->gyro_soft_notch_hz_1, gyroConfig->gyro_soft_notch_cutoff_1);
		for (int axis = 0; axis < 3; axis++) {
			notchFilter1[axis] = &gyroFilterNotch_1[axis];
			biquadFilterInit(notchFilter1[axis], gyroConfig->gyro_soft_notch_hz_1, gyro.targetLooptime, gyroSoftNotchQ1, FILTER_NOTCH);
		}
	}
	
	/* Initialise gyro notch 2 filter */
	if (gyroConfig->gyro_soft_notch_hz_2 && gyroConfig->gyro_soft_notch_hz_2 <= gyroFrequencyNyquist) {
		notchFilter2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
		const float gyroSoftNotchQ2 = filterGetNotchQ(gyroConfig->gyro_soft_notch_hz_2, gyroConfig->gyro_soft_notch_cutoff_2);
		for (int axis = 0; axis < 3; axis++) {
			notchFilter2[axis] = &gyroFilterNotch_2[axis];
			biquadFilterInit(notchFilter2[axis], gyroConfig->gyro_soft_notch_hz_2, gyro.targetLooptime, gyroSoftNotchQ2, FILTER_NOTCH);
		}
	}
}

bool isGyroCalibrationComplete(void)
{
	return calibratingG == 0;
}

static uint16_t gyroCalculateCalibratingCycles(void)
{
//	printf("gyro.targetLooptime: %u, %s, %d\r\n", gyro.targetLooptime, __FUNCTION__, __LINE__);		// 125(1/8k)us * 4(gyro_sync_denom) = 500us for MPU6050, 8K gyro sampling freq
	
	/* CALIBRATING_GYRO_CYCLES = 1000; gyro.targetLooptime = 125
	 *
	 * gyroCalibrationCycles = (CALIBRATING_GYRO_CYCLES / gyro.targetLooptime) * CALIBRATING_GYRO_CYCLES
	 *					     = (1000 / 125) * 1000
	 *						 = 8 * 1000
	 *						 = 8000
	 */
	return (CALIBRATING_GYRO_CYCLES / gyro.targetLooptime) * CALIBRATING_GYRO_CYCLES;
}

static bool isOnFirstGyroCalibrationCycle(void)
{
	return calibratingG == gyroCalculateCalibratingCycles();
}

static bool isOnFinalGyroCalibrationCycle(void)
{
	return calibratingG == 1;
}

void gyroSetCalibrationCycles(void)
{
	calibratingG = gyroCalculateCalibratingCycles();
}

static void performGyroCalibration(uint8_t gyroMovementCalibrationThreshold)
{
	static int32_t g[3];
	static stdev_t var[3];
	
	for (int axis = 0; axis < 3; axis++) {
		
		/* Reset g[axis] at start of calibration */
		if (isOnFirstGyroCalibrationCycle()) {
//			printf("%s, %d\r\n", __FUNCTION__, __LINE__);
			g[axis] = 0;
			devClear(&var[axis]);
		}
		
		/* Sum up CALIBRATING_GYRO_CYCLES readings */
		g[axis] += gyroADC[axis];
		devPush(&var[axis], gyroADC[axis]);
//		printf("g[%d]: %d, %s, %d\r\n", axis, g[axis], __FUNCTION__, __LINE__);
		
		/* Reset global variables to prevent other code from using un-calibrated data */
		gyroADC[axis] = 0;
		gyroZero[axis] = 0;
		
//		printf("calibratingG: %u\r\n", calibratingG);
		if (isOnFinalGyroCalibrationCycle()) {
			float dev = devStandardDeviation(&var[axis]);		// use standard deviation value to determine if the gyro is moved during the power up process, if so, re-calibrate the gyro again
//			printf("dev[%d]: %f, moron threshold: %u, %s, %d\r\n", axis, dev, gyroMovementCalibrationThreshold, __FUNCTION__, __LINE__);
			/* check deviation and startover in case the model was moved */
			if (gyroMovementCalibrationThreshold && dev > gyroMovementCalibrationThreshold) {		// gyroMovementCalibrationThreshold = 48
				gyroSetCalibrationCycles();			// reset the calibration cycle so that the gyro can be calibrated again
				return;
			}
			
			gyroZero[axis] = (g[axis] + (gyroCalculateCalibratingCycles() / 2)) / gyroCalculateCalibratingCycles();
//			printf("gyroZero[%d]: %d, %s, %d\r\n", axis, gyroZero[axis], __FUNCTION__, __LINE__);
		}
	}
	
	if (isOnFinalGyroCalibrationCycle()) {
//		schedulerResetTaskStatistics(TASK_SELF);		// so calibration cycles do not pollute tasks statistics
//		beeper(BEEPER_GYRO_CALIBRATED);
		LED3_OFF;			// turn off LED3 when GYRO calibration process is finished
	}
	
	calibratingG--;
}

void gyroUpdate(void)
{
	/* range: +/- 2048 LSB/g; +/- 2000 deg/sec */
	if (gyro.dev.update) {
		/* if the gyro update function is set then return, since the gyro is read in gyroUpdateISR */
		return;
	}
	
	if (!gyro.dev.read(&gyro.dev)) {
		return;
	}
	
	gyro.dev.dataReady = false;
	
	/* move gyro data into 32-bit variables to avoid overflows in calculations */
	gyroADC[X] = gyro.dev.gyroADCRaw[X];
	gyroADC[Y] = gyro.dev.gyroADCRaw[Y];
	gyroADC[Z] = gyro.dev.gyroADCRaw[Z];
//	printf("MPU9250 (SPI) data - gyroADCRaw[X]: %d, gyroADCRaw[Y]: %d, gyroADCRaw[Z]: %d\r\n", gyroADC[X], gyroADC[Y], gyroADC[Z]);

//	alignSensors(gyroADC, gyro.dev.gyroAlign);			// TODO: implement this function later

	const bool calibrationComplete = isGyroCalibrationComplete();
//	printf("calibrationComplete: %d, %s, %d\r\n", calibrationComplete, __FUNCTION__, __LINE__);
	if (calibrationComplete) {
//		printf("gyro calibration completed!, %s, %d\r\n", __FUNCTION__, __LINE__);

		gyro.dev.calibrationFlag = true;        // For testing only
		LED4_ON;

#if defined(GYRO_USES_SPI) && defined(USE_MPU_DATA_READY_SIGNAL)
		/* SPI-based gyro so can read and update in ISR */
//		if (gyroConfig->gyro_isr_update) {
//			mpuGyroSetIsrUpdate(&gyro.dev, gyroUpdateISR);
//			return;
//		}
#endif
	} else {
		/* Toggle LED3 (Orange LED) when GYRO calibration process is started */
//		LED3_TOGGLE;
		LED4_OFF;
		performGyroCalibration(gyroConfig->gyroMovementCalibrationThreshold);
	}

	/* debug the gyroZero array */
//	if (calibrationComplete) {
//		printf("gyroZero[%d]: %d, %s, %d\r\n", X, gyroZero[X], __FUNCTION__, __LINE__);
//		printf("gyroZero[%d]: %d, %s, %d\r\n", Y, gyroZero[Y], __FUNCTION__, __LINE__);
//		printf("gyroZero[%d]: %d, %s, %d\r\n", Z, gyroZero[Z], __FUNCTION__, __LINE__);
//	}
	
	for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
//		printf("gyroADC[%d] BEFORE cali: %d, %s, %d\r\n", axis, gyroADC[axis], __FUNCTION__, __LINE__);
		gyroADC[axis] -= gyroZero[axis];
//		printf("gyroADC[%d] AFTER cali: %d, %s, %d\r\n", axis, gyroADC[axis], __FUNCTION__, __LINE__);
		
		/* scale gyro output to degrees per second
         *
         * gyroADCf is in DEGREE PER SECOND
         */
		float gyroADCf = (float)gyroADC[axis] * gyro.dev.scale;			// gyro.dev.scale = 1.0 / 16.4 for +/-2000 gyro configuration
		
		/* Apply LPF */
		gyroADCf = softLpfFilterApplyFn(softLpfFilter[axis], gyroADCf);
		
		/* Apply Notch filter 1 */
		gyroADCf = notchFilter1ApplyFn(notchFilter1[axis], gyroADCf);
		
		/* Apply Notch filter 2 */
		gyroADCf = notchFilter2ApplyFn(notchFilter2[axis], gyroADCf);
		
		/* store filtered gyro data to gyroADCf array */
		gyro.gyroADCf[axis] = gyroADCf;
	}
    
    /* The reason doing the division of gyro.dev.scale is to convert the gyroADC value from DEGREE PER SECOND back to the raw data in order to do the calibration process
     *
     * As float gyroADCf = (float)gyroADC[axis] * gyro.dev.scale from above code has converted the gyro raw data to the value in DEGREE PER SECOND, we need to convert it back.
     */
    if (!calibrationComplete) {
        gyroADC[X] = lrintf(gyro.gyroADCf[X] / gyro.dev.scale);     // Convert gyro X-axis data from DEGREE PER SECOND back to raw data as the calibration is NOT finished
        gyroADC[Y] = lrintf(gyro.gyroADCf[Y] / gyro.dev.scale);     // Convert gyro Y-axis data from DEGREE PER SECOND back to raw data as the calibration is NOT finished
        gyroADC[Z] = lrintf(gyro.gyroADCf[Z] / gyro.dev.scale);     // Convert gyro Z-axis data from DEGREE PER SECOND back to raw data as the calibration is NOT finished
    }
}
