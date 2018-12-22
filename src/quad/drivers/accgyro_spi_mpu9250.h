#ifndef __ACCGYRO_SPI_MPU9250_H
#define __ACCGYRO_SPI_MPU9250_H

#include <stdbool.h>
#include "accgyro.h"

#define MPU9250_WHO_AM_I_CONST              (0x71)
#define MPU9250_WHO_AM_I_CONST_ALT			(0x73)

/* <MPU9250A-00-v1.6_RegisterMap.pdf> p40 PWR_MGMT_1 [7] H_RESET, Write a 1 to set the reset, the bit will auto clear. */
#define MPU9250_BIT_RESET                   (0x80)

/* RF = Register Flag */
#define MPU_RF_DATA_RDY_EN 					(1 << 0)

bool mpu9250SpiDetect(void);
bool mpu9250SpiGyroDetect(gyroDev_t *gyro);
bool mpu9250WriteRegister(uint8_t reg, uint8_t data);
bool mpu9250ReadRegister(uint8_t reg, uint8_t length, uint8_t *data);
bool mpu9250SlowReadRegister(uint8_t reg, uint8_t length, uint8_t *data);
bool verifyMPU9250WriteRegister(uint8_t reg, uint8_t data);
void mpu9250ResetGyro(void);

#endif	// __ACCGYRO_SPI_MPU9250_H
