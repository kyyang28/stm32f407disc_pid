#ifndef __ACCGYRO_MPU6050_H
#define __ACCGYRO_MPU6050_H

#include "accgyro.h"

bool mpu6050GyroDetect(gyroDev_t *gyro);
bool mpu6050AccDetect(accDev_t *acc);

#endif	// __ACCGYRO_MPU6050_H
