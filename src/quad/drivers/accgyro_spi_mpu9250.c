
#include "accgyro_mpu.h"
#include "accgyro_spi_mpu9250.h"
#include "gyro_sync.h"
#include "target.h"
#include "io.h"
#include "bus_spi.h"
#include "system.h"

#include <stdio.h>

#ifdef USE_SPI

static IO_t mpuSpi9250CsPin = IO_NONE;

static bool mpuSpi9250InitDone = false;

#define ENABLE_MPU9250			IOLo(mpuSpi9250CsPin);
#define DISABLE_MPU9250			IOHi(mpuSpi9250CsPin);

void mpu9250ResetGyro(void)
{
	/* Device reset */
	mpu9250WriteRegister(MPU_RA_PWR_MGMT_1, MPU9250_BIT_RESET);
	delay(150);			// delay 150 ms
}

bool mpu9250WriteRegister(uint8_t reg, uint8_t data)
{
//	printf("reg: 0x%x, data: %u, %s, %d\r\n", reg, data, __FUNCTION__, __LINE__);
	ENABLE_MPU9250;				// CS lo
	delayMicroseconds(1);
	spiTransferByte(MPU9250_SPI_INSTANCE, reg);
	spiTransferByte(MPU9250_SPI_INSTANCE, data);
	DISABLE_MPU9250;			// CS high
	delayMicroseconds(1);
	
	return true;
}

bool mpu9250ReadRegister(uint8_t reg, uint8_t length, uint8_t *data)
{
	ENABLE_MPU9250;
	spiTransferByte(MPU9250_SPI_INSTANCE, reg | 0x80);		// read transaction
	spiTransfer(MPU9250_SPI_INSTANCE, data, NULL, length);
	DISABLE_MPU9250;
	
	return true;
}

bool mpu9250SlowReadRegister(uint8_t reg, uint8_t length, uint8_t *data)
{
	ENABLE_MPU9250;
	delayMicroseconds(1);
	spiTransferByte(MPU9250_SPI_INSTANCE, reg | 0x80);		// read transaction
	spiTransfer(MPU9250_SPI_INSTANCE, data, NULL, length);
	DISABLE_MPU9250;
	delayMicroseconds(1);
	
	return true;
}

bool verifyMPU9250WriteRegister(uint8_t reg, uint8_t data)
{
	uint8_t in;
	uint8_t attemptsRemaining = 20;
	
//	printf("reg: 0x%x, %d\r\n", reg, __LINE__);
//	printf("reg: 0x%x, data: %u, %s, %d\r\n", reg, data, __FUNCTION__, __LINE__);
	mpu9250WriteRegister(reg, data);
	delayMicroseconds(100);
	
	do {
//		printf("reg (slowRead): 0x%x, %s, %d\r\n", reg, __FUNCTION__, __LINE__);
		mpu9250SlowReadRegister(reg, 1, &in);
//		printf("in: %u\r\n", in);
		if (in == data) {
			return true;
		}else {
//			printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//			printf("reg: 0x%x, %d\r\n", reg, __LINE__);
//			printf("reg: 0x%x, data: %u\r\n", reg, data);
			mpu9250WriteRegister(reg, data);
			delayMicroseconds(100);
		}
	} while (attemptsRemaining--);
	
	return false;
}

static void mpu9250AccAndGyroInit(gyroDev_t *gyro)
{
	if (mpuSpi9250InitDone) {
		return;
	}
	
	spiSetDivisor(MPU9250_SPI_INSTANCE, SPI_CLOCK_INITIALISATION);		// low speed for writing to slow registers
	
	/* Config Power Management 1 register (Reg 107) */
	mpu9250WriteRegister(MPU_RA_PWR_MGMT_1, MPU9250_BIT_RESET);
	delay(50);															// according to MPU9250 product specification
	
	/* Config Power Management 1 register (Reg 107) */
	if (!verifyMPU9250WriteRegister(MPU_RA_PWR_MGMT_1, INV_CLK_PLL)) {
		printf("Failed to config CLK SOURCE of IMU!, %s, %d\r\n", __FUNCTION__, __LINE__);
	}

#if 1
	/* Fchoice_b defaults to 00 which makes fchoice 11
	 * 
	 * Notes:
	 * FCB_DISABLED = 00, FCB stands for fchoice_b, which means fchoice = 11 (inverted of fchoice_b)
	 * The DLPF_CFG = 7, Gyro bandwidth = 3600 Hz, delay = 0.17 ms, Fs = 8 KHz, Temperature bandwidth = 4000 Hz, delay = 0.04 ms
	 * according to MPU-9250A Register Map and Descriptions-00-v1.6.pdf
	 *
	 * This will setup to use 8 kHz gyro sampling frequency with bandwidth 3600 Hz (delay 0.17 ms)
	 */
	const uint8_t raGyroConfigData = gyro->gyroRateKHz > GYRO_RATE_8_kHz ? (INV_FSR_2000DPS << 3 | FCB_3600_32) : (INV_FSR_2000DPS << 3 | FCB_DISABLED);
	/* Config Gyroscope configuration register (Reg 27) */
	if (!verifyMPU9250WriteRegister(MPU_RA_GYRO_CONFIG, raGyroConfigData)) {
//		printf("Failed to config Gyroscope configuration register!, %s, %d\r\n", __FUNCTION__, __LINE__);
	}
	
//	printf("gyro->lpf: %u, %s, %d\r\n", gyro->lpf, __FUNCTION__, __LINE__);
	
	/* Config Configuration register (Reg 26) */
	if (gyro->lpf == 4) {
		if (!verifyMPU9250WriteRegister(MPU_RA_CONFIG, 1)) {		// 1 kHz, 184 DLPF
			printf("Failed to config Configuration register!, %s, %d\r\n", __FUNCTION__, __LINE__);
		}
	}else if (gyro->lpf < 4) {
		if (!verifyMPU9250WriteRegister(MPU_RA_CONFIG, 7)) {		// 8 kHz, 3600 Hz (bandwidth) DLPF
//			printf("Failed to config Configuration register!, %s, %d\r\n", __FUNCTION__, __LINE__);
		}
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	}else if (gyro->lpf > 4) {
		if (!verifyMPU9250WriteRegister(MPU_RA_CONFIG, 0)) {		// 8 kHz, 250 Hz (bandwidth) DLPF
			printf("Failed to config Configuration register!, %s, %d\r\n", __FUNCTION__, __LINE__);
		}
	}
	
	/* Config Sample Rate Divider register (Reg 25) */
	if (!verifyMPU9250WriteRegister(MPU_RA_SMPLRT_DIV, gyroMPU6xxxGetDividerDrops(gyro))) {
		printf("Failed to config Sample Rate Divider register!, %s, %d\r\n", __FUNCTION__, __LINE__);
	}
	
	/* Config Accelerometer Configuration register (Reg 28) */
	if (!verifyMPU9250WriteRegister(MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3)) {
//		printf("Failed to config Accelerometer Configuration register!, %s, %d\r\n", __FUNCTION__, __LINE__);
	}
	
	/* Config INT Pin / Bypass Enable Configuration register (Reg 55) */
	if (!verifyMPU9250WriteRegister(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0)) {	// INT_ANYRD_2CLEAR, BYPASS_EN
//		printf("Failed to config INT Pin / Bypass Enable Configuration register!, %s, %d\r\n", __FUNCTION__, __LINE__);
	}

#if defined(USE_MPU_DATA_READY_SIGNAL)
	/* Config Interrupt Enable register (Reg 56) */
	if (!verifyMPU9250WriteRegister(MPU_RA_INT_ENABLE, 0x01)) {	// this resets register MPU_RA_PWR_MGMT_1 and won't read back correctly.
//		printf("Failed to config Interrupt Enable register!, %s, %d\r\n", __FUNCTION__, __LINE__);
	}
#endif

#endif
	spiSetDivisor(MPU9250_SPI_INSTANCE, SPI_CLOCK_FAST);	// SPI_CLOCK_FAST = 4, APB2 clock freq = 84 MHz, SPI clock freq = APB2 clock freq / SPI_CLOCK_FAST = 84 MHz / 4 = 21 MHz

	mpuSpi9250InitDone = true;		// init done
}

void mpu9250SpiGyroInit(gyroDev_t *gyro)
{
	mpuGyroInit(gyro);
	
	mpu9250AccAndGyroInit(gyro);

	spiResetErrorCounter(MPU9250_SPI_INSTANCE);

	spiSetDivisor(MPU9250_SPI_INSTANCE, SPI_CLOCK_FAST);		// high speed now that we don't need to write to the slow registers

	mpuGyroRead(gyro);

	if ((((int8_t)gyro->gyroADCRaw[1]) == -1 && ((int8_t)gyro->gyroADCRaw[0]) == -1) || spiGetErrorCounter(MPU9250_SPI_INSTANCE) != 0) {
		spiResetErrorCounter(MPU9250_SPI_INSTANCE);
		printf("mpuGyroRead data is failed!, %s, %d\r\n", __FUNCTION__, __LINE__);
	}
}

bool mpu9250SpiDetect(void)
{
	uint8_t in;
	uint8_t attemptsRemaining = 20;
	
#ifdef MPU9250_CS_PIN
	mpuSpi9250CsPin = IOGetByTag(IO_TAG(MPU9250_CS_PIN));
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
#endif
//	ioRec_t *ioRecMPUSpi9250Cs = IO_Rec(mpuSpi9250CsPin);
//	printf("ioRecMPUSpi9250Cs->gpio: 0x%x\r\n", (uint32_t)ioRecMPUSpi9250Cs->gpio);		// gpio = 0x40021000 (GPIOE)
//	printf("ioRecMPUSpi9250Cs->pin: %u\r\n", ioRecMPUSpi9250Cs->pin);					// pin = 16 = 0x10 (1 << 4)
	
	IOInit(mpuSpi9250CsPin, OWNER_MPU_CS, 0);
	IOConfigGPIO(mpuSpi9250CsPin, SPI_IO_CS_CFG);
	
//	printf("ioRecMPUSpi9250Cs->owner: %u\r\n", ioRecMPUSpi9250Cs->owner);				// owner = 11
//	printf("ioRecMPUSpi9250Cs->index: %u\r\n", ioRecMPUSpi9250Cs->index);				// index = 0	
	
	spiSetDivisor(MPU9250_SPI_INSTANCE, SPI_CLOCK_INITIALISATION);		// low speed
	
	mpu9250WriteRegister(MPU_RA_PWR_MGMT_1, MPU9250_BIT_RESET);
	
	do {
		delay(150);
		
		mpu9250ReadRegister(MPU_RA_WHO_AM_I, 1, &in);
		printf("WHO_AM_I reg value: 0x%x\r\n", in);
		/* MPU9250_WHO_AM_I_CONST = 0x71
		 * MPU9250_WHO_AM_I_CONST_ALT = 0x73
		 */
		if ((in == MPU9250_WHO_AM_I_CONST) || (in == MPU9250_WHO_AM_I_CONST_ALT)) {
			break;
		}
		
		if (!attemptsRemaining) {
			return false;
		}
	} while (attemptsRemaining--);
	
	/* MPU 9250 found, set the SPI clock to 21.00000 MHz  */
	spiSetDivisor(MPU9250_SPI_INSTANCE, SPI_CLOCK_FAST);
	
	return true;
}

bool mpu9250SpiGyroDetect(gyroDev_t *gyro)
{
	if (gyro->mpuDetectionResult.sensor != MPU_9250_SPI) {
		return false;
	}
	
	gyro->init = mpu9250SpiGyroInit;
	gyro->read = mpuGyroRead;			// mpuGyroRead function takes 30 us (8K sampling rate) or 35 us (1K sampling rate) to sample the gyro data
	gyro->intStatus = mpuCheckDataReady;
	
	/* 16.4 dps/lsb scale factor */
	gyro->scale = 1.0f / 16.4f;
	
	return true;
}
#endif
