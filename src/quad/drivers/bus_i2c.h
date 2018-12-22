#ifndef __BUS_I2C_H
#define __BUS_I2C_H

#include <stdint.h>
#include <stdbool.h>

//IO_t scl;
//IO_t sda;

//#define SCL_H					IOHi(scl)
//#define SCL_L					IOLo(scl)

//#define SDA_H					IOHi(sda)
//#define SDA_L					IOLo(sda)

//#define SCL_read				IORead(scl)
//#define SDA_read				IORead(sda)

typedef enum I2CDevice {
	I2CINVALID = -1,
	I2CDEV_1 = 0,
	I2CDEV_2,
	I2CDEV_3,
	I2CDEV_COUNT
}I2CDevice;

//void I2C_delay(void);
//bool I2C_Start(void);
//void I2C_Stop(void);
//void I2C_Ack(void);
//void I2C_NoAck(void);
//bool I2C_WaitAck(void);
//void I2C_SendByte(uint8_t byte);
void i2cInit(I2CDevice device);
bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data);
//bool i2cWrite(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t data);
bool i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
//int i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);			// only use in inv_mpu.c and inv_mpu_dmp_motion_driver.c
//bool i2cWriteBuffer(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
//int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
bool i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
//bool i2cRead(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
uint16_t i2cGetErrorCounter(void);

#endif	// __BUS_I2C_H
