
#include "gyro_sync.h"
//#include <stdio.h>

bool gyroSyncCheckUpdate(gyroDev_t *gyro)
{
	if (!gyro->intStatus) {
		return false;
	}
	
	return gyro->intStatus(gyro);
}

uint8_t gyroMPU6xxxGetDividerDrops(const gyroDev_t *gyro)
{
	return gyro->mpuDividerDrops;
}

uint32_t gyroSetSampleRate(gyroDev_t *gyro, uint8_t lpf, uint8_t gyroSyncDenominator, bool gyro_use_32khz)
{
	float gyroSamplePeriod;
	
	if (lpf == GYRO_LPF_256HZ || lpf == GYRO_LPF_NONE) {		// when GYRO_LPF_256HZ = 0 or GYRO_LPF_NONE = 7, DLPF is disabled, gyro uses 32khz or 8khz sampling frequency
		if (gyro_use_32khz) {
			gyro->gyroRateKHz = GYRO_RATE_32_kHz;
			gyroSamplePeriod = 31.5f;			// should be 32.25f?	in microseconds
//			gyroSamplePeriod = 31.25f;
		}else {
			gyro->gyroRateKHz = GYRO_RATE_8_kHz;
			gyroSamplePeriod = 125.0f;			// in microseconds
		}
	} else {
		gyro->gyroRateKHz = GYRO_RATE_1_kHz;
		gyroSamplePeriod = 1000.0f;				// in microseconds
		gyroSyncDenominator = 1;				// always full sampling frequency 1khz
	}
	
	/* calculate gyro divider and targetLooptime (expected cycleTime) */
//	printf("gyroSyncDenominator: %u, %s, %d\r\n", gyroSyncDenominator, __FUNCTION__, __LINE__);
	gyro->mpuDividerDrops = gyroSyncDenominator - 1;
	const uint32_t targetLooptime = (uint32_t)(gyroSyncDenominator * gyroSamplePeriod);
	return targetLooptime;
}
