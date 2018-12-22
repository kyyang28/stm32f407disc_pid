
#include <stdint.h>
#include <stdbool.h>
#include "accgyro.h"		// accgyro_mpu.h is included
#include "accgyro_spi_mpu9250.h"
#include "bus_i2c.h"
#include "system.h"
#include "target.h"
#include "utils.h"
#include "axis.h"
#include "nvic.h"

#ifndef MPU_I2C_INSTANCE
#define MPU_I2C_INSTANCE 			I2C_DEVICE				// I2C_DEVICE = I2CDEV_2 = 1 using I2C2 (SCL: PB10, SDA: PB11)
#endif

#define MPU_ADDRESS             	0x68

/* WHO_AM_I register contents for MPU6050, MPU9250, MPU6500 */
#define MPUx0x0_WHO_AM_I_CONST		(0x68)				// MPU6050
#define MPU9250_WHO_AM_I_CONST		(0x71)				// MPU9250
#define MPU6500_WHO_AM_I_CONST		(0x70)				// MPU6500

#define MPU_INQUIRY_MASK			0x7E					// 0111 1110

/* Debugging purposes */
#include <stdio.h>

mpuResetFuncPtr mpuReset;

#ifdef USE_I2C
static bool mpuReadRegisterI2C(uint8_t reg, uint8_t length, uint8_t *data);
static bool mpuWriteRegisterI2C(uint8_t reg, uint8_t data);
#endif

#ifdef USE_SPI
static bool detectSPISensorsAndUpdateDetectionResult(gyroDev_t *gyro)
{
#ifdef USE_GYRO_SPI_MPU9250
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	if (mpu9250SpiDetect()) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		gyro->mpuDetectionResult.sensor = MPU_9250_SPI;
		gyro->mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
		gyro->mpuConfiguration.read = mpu9250ReadRegister;
		gyro->mpuConfiguration.slowRead = mpu9250SlowReadRegister;
		gyro->mpuConfiguration.write = mpu9250WriteRegister;
		gyro->mpuConfiguration.verifyWrite = verifyMPU9250WriteRegister;
		gyro->mpuConfiguration.reset = mpu9250ResetGyro;
		return true;
	}
#endif
	
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	
	UNUSED(gyro);
	return false;
}
#endif

mpuDetectionResult_t *mpuDetect(gyroDev_t *gyro)
{
	bool ack;
	uint8_t sig;
	
	/* Chapter 3.1 Table 1, Gyroscope Startup Time: TYP 35 ms from <MPU9250REV1.0_ProductSpecification.pdf> */
	delay(35);

#ifndef USE_I2C			// SPI mode
	ack = false;
	sig = 0;
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
#else
	ack = mpuReadRegisterI2C(MPU_RA_WHO_AM_I, 1, &sig);
#endif
//	printf("ack: %d, %s, %d\r\n", ack, __FUNCTION__, __LINE__);
	if (ack) {
#ifdef USE_I2C
		gyro->mpuConfiguration.read = mpuReadRegisterI2C;
		gyro->mpuConfiguration.write = mpuWriteRegisterI2C;
		printf("MPU9250 I2C read and write initialisation done, %s, %d\r\n", __FUNCTION__, __LINE__);
#endif
	}else {
#ifdef USE_SPI
		// TODO: MPU9250 SPI implementation
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		bool detectedSpiSensor = detectSPISensorsAndUpdateDetectionResult(gyro);
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		UNUSED(detectedSpiSensor);
		
		return &gyro->mpuDetectionResult;
#endif
	}
	
	gyro->mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
	
//	printf("sig before mask: 0x%x, %s, %d\r\n", sig, __FUNCTION__, __LINE__);			// sig = 0x73

//	sig &= MPU_INQUIRY_MASK;		// MPU9250 WHO_AM_I returns 0x73 stored in sig in my case, sig &= MPU_INQUIRY_MASK will give 0x72
	
//	printf("sig after mask: 0x%x, %s, %d\r\n", sig, __FUNCTION__, __LINE__);			// sig = 0x72 after masking
	
	if (sig == MPUx0x0_WHO_AM_I_CONST) {
		printf("MPU6050 I2C is detected! %s, %d\r\n", __FUNCTION__, __LINE__);
		gyro->mpuDetectionResult.sensor = MPU_60x0;
//		mpu6050FindRevision(gyro);
	}else if ((sig == MPU9250_WHO_AM_I_CONST) || (sig == MPU9250_WHO_AM_I_CONST_ALT)) {
		printf("MPU9250 I2C is detected! %s, %d\r\n", __FUNCTION__, __LINE__);
		gyro->mpuDetectionResult.sensor = MPU_9250_I2C;
	}
	
	return &gyro->mpuDetectionResult;
}

#ifdef USE_I2C
static bool mpuReadRegisterI2C(uint8_t reg, uint8_t length, uint8_t *data)
{
	bool ack = i2cRead(MPU_ADDRESS, reg, length, data);
//	bool ack = i2cRead(MPU_I2C_INSTANCE, MPU_ADDRESS, reg, length, data);
	return ack;
}

static bool mpuWriteRegisterI2C(uint8_t reg, uint8_t data)
{
	bool ack = i2cWrite(MPU_ADDRESS, reg, data);
	return ack;
}
#endif

/* Gyro interrupt service routine */
#if defined(MPU_INT_EXTI)
static void mpuIntExtiHandler(extiCallbackRec_t *cb)
{
#ifdef DEBUG_MPU_DATA_READY_INTERRUPT
	static uint32_t lastCalledAtUs = 0;
	const uint32_t nowUs = micros();
	printf("nowUs - lastCalledAtUs: %u, %s, %d\r\n", (uint16_t)(nowUs - lastCalledAtUs));
	lastCalledAtUs = nowUs;
#endif
	gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
	gyro->dataReady = true;
	if (gyro->update) {
		gyro->update(gyro);
	}
#ifdef DEBUG_MPU_DATA_READY_INTERRUPT
	const uint32_t now2Us = micros();
	printf("now2Us - nowUs: %u, %s, %d\r\n", (uint16_t)(now2Us - nowUs));
#endif
}
#endif

static void mpuIntExtiInit(gyroDev_t *gyro)
{
#if defined(MPU_INT_EXTI)
	if (!gyro->mpuIntExtiConfig) {
		return;
	}
	
	IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiConfig->tag);
	
#ifdef ENSURE_MPU_DATA_READY_IS_LOW
	uint8_t status = IORead(mpuIntIO);
	if (status) {
		return;
	}
#endif
	
	/* For STM32F4 boards */
	IOInit(mpuIntIO, OWNER_MPU_EXTI, 0);
	IOConfigGPIO(mpuIntIO, IOCFG_IN_FLOATING);		// TODO: maybe pullup or pulldown?
	
	EXTIHandlerInit(&gyro->exti, mpuIntExtiHandler);
	EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, EXTI_Trigger_Rising);
	EXTIEnable(mpuIntIO, true);
	
#else
	UNUSED(gyro);
#endif
}

void mpuGyroInit(struct gyroDev_s *gyro)
{
	mpuIntExtiInit(gyro);
}

bool mpuGyroRead(gyroDev_t *gyro)
{
	uint8_t data[6];
	
	const bool ack = gyro->mpuConfiguration.read(gyro->mpuConfiguration.gyroReadXRegister, 6, data);
//	printf("ack: %d, %s, %d\r\n", ack, __FUNCTION__, __LINE__);		// returns 1 which is correct
	if (!ack) {
		return false;
	}

//	for (int i = 0; i < 6; i++) {
//		printf("data[%d]: %u ", i, data[i]);
//	}
//	printf("\r\n");
	
	/* X = 0, Y = 1, Z = 2 */
	gyro->gyroADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
	gyro->gyroADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
	gyro->gyroADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);
	
//	if (gyro->calibrationFlag) {
//		printf("collectedAt: %u\r\n", micros());
//	}

	return true;
}

bool mpuCheckDataReady(gyroDev_t *gyro)
{
	bool ret;
	
	if (gyro->dataReady) {
		ret = true;
		gyro->dataReady = false;
	}else {
		ret = false;
	}
	
	return ret;
}

bool mpuAccRead(accDev_t *acc)
{
	uint8_t data[6];
	
	bool ack = acc->mpuConfiguration.read(MPU_RA_ACCEL_XOUT_H, 6, data);
	if (!ack) {
		return false;
	}
	
	acc->ADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
	acc->ADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
	acc->ADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);
	
	return true;
}
