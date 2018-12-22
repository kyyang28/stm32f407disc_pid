#ifndef __GYRO_SYNC_H
#define __GYRO_SYNC_H

#include <stdint.h>
#include <stdbool.h>
#include "accgyro.h"

bool gyroSyncCheckUpdate(gyroDev_t *gyro);
uint8_t gyroMPU6xxxGetDividerDrops(const gyroDev_t *gyro);
uint32_t gyroSetSampleRate(gyroDev_t *gyro, uint8_t lpf, uint8_t gyroSyncDenominator, bool gyro_use_32khz);

#endif	// __GYRO_SYNC_H
