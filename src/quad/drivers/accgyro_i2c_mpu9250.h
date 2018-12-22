#ifndef __ACCGYRO_I2C_MPU9250_H
#define __ACCGYRO_I2C_MPU9250_H

#include "accgyro.h"

bool mpu9250I2CGyroDetect(gyroDev_t *gyro);
bool mpu9250I2CAccDetect(accDev_t *acc);

#endif	// __ACCGYRO_I2C_MPU9250_H
