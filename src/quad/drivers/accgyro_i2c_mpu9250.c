
#include <stdbool.h>
#include "accgyro_i2c_mpu9250.h"
#include "accgyro_spi_mpu9250.h"
#include "accgyro_mpu.h"
#include "gyro_sync.h"
#include "system.h"

#include <stdio.h>				// printf

static void mpu9250I2CGyroInit(gyroDev_t *gyro)
{
	/* Config interrupt pin */
	mpuGyroInit(gyro);
	
	gyro->mpuConfiguration.write(MPU_RA_PWR_MGMT_1, 0x80);	// PWR_MGMT_1	-- DEVICE_RESET 1
	delay(100);
	gyro->mpuConfiguration.write(MPU_RA_PWR_MGMT_1, 0x03);	// PWR_MGMT_1   -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
	gyro->mpuConfiguration.write(MPU_RA_SMPLRT_DIV, gyroMPU6xxxGetDividerDrops(gyro));	// SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV), gyroMPU6xxxGetDividerDrops(gyro) returns 3, Gyro output freq = 8khz, sample rate = 8khz / (1+3) = 2khz
	delay(15);		// PLL settling time when changing CLKSEL is max 10ms. Use 15ms to be sure
	gyro->mpuConfiguration.write(MPU_RA_CONFIG, gyro->lpf);	// CONFIG       -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz), gyro->lpf = 0 meaning uses gyro bandwidth 256hz, gyro output frequency = 8khz
	gyro->mpuConfiguration.write(MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);	// GYRO_CONFIG  -- FS_SEL = 3: Full scale set to 2000 deg/sec

	/*
	 * ACC init stuff
	 * Accel scale 16g (2048 LSB/g)
	 */
	gyro->mpuConfiguration.write(MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);
	
	gyro->mpuConfiguration.write(MPU_RA_INT_PIN_CFG, 
			0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);	// INT_PIN_CFG  -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS

#ifdef USE_MPU_DATA_READY_SIGNAL
	gyro->mpuConfiguration.write(MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
#endif
}

bool mpu9250I2CGyroDetect(gyroDev_t *gyro)
{
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	if (gyro->mpuDetectionResult.sensor != MPU_9250_I2C) {
		return false;
	}
	
	gyro->init = mpu9250I2CGyroInit;
	gyro->read = mpuGyroRead;
	gyro->intStatus = mpuCheckDataReady;
	
	/* 16.4 dps/lsb scale factor */
	gyro->scale = 1.0f / 16.4f;
	
	return true;
}

void mpu9250AccInit(accDev_t *acc)
{
	acc->acc_1G = 512 * 4;
}

bool mpu9250I2CAccDetect(accDev_t *acc)
{
	if (acc->mpuDetectionResult.sensor != MPU_9250_I2C) {
		return false;
	}
	
	acc->init = mpu9250AccInit;
	acc->read = mpuAccRead;
	return true;
}
