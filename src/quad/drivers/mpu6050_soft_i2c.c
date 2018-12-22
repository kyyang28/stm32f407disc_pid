
#include <stdio.h>				// for debugging purpose

#include "bus_i2c.h"
#include "mpu6050_soft_i2c.h"
//#include "mpu6050.h"
#include "target.h"
#include "system.h"
#include "stm32f4xx_usart.h"
#include "rxSerial3Test.h"

#ifdef USE_I2C

/* 	Set gyro full scale range (FSR)
 *	fsr - 0: ±250dps; 1:±500dps; 2:±1000dps; 3:±2000dps
 */
uint8_t MPU6050_Set_Gyro_FSR(uint8_t fsr)
{
	uint8_t data = fsr << 3;
	return i2cWriteBuffer(MPU_ADDR_AD0, MPU_GYRO_CFG_REG, 1, &data);
//	return i2cWrite(MPU_ADDR_AD0, MPU_GYRO_CFG_REG, fsr << 3);
//	return i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_GYRO_CFG_REG, fsr << 3);
}

/*
 * Set accelerometer full scale range
* fsr - 0:±2g; 1:±4g; 2:±8g; 3:±16g
 */
uint8_t MPU6050_Set_Accel_FSR(uint8_t fsr)
{
	uint8_t data = fsr << 3;
	return i2cWriteBuffer(MPU_ADDR_AD0, MPU_ACCEL_CFG_REG, 1, &data);
//	return i2cWrite(MPU_ADDR_AD0, MPU_ACCEL_CFG_REG, fsr << 3);
//	return i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_ACCEL_CFG_REG, fsr << 3);
}

uint8_t MPU6050_Set_LPF(uint16_t lpf)
{
	uint8_t data = 0;
	
	if (lpf >= 188) {
		data = 1;
	}else if (lpf >= 98) {
		data = 2;
	}else if (lpf >= 42) {
		data = 3;
	}else if (lpf >= 20) {
		data = 4;
	}else if (lpf >= 10) {
		data = 5;
	}else {
		data = 6;
	}

	return i2cWriteBuffer(MPU_ADDR_AD0, MPU_CFG_REG, 1, &data);
//	return i2cWrite(MPU_ADDR_AD0, MPU_CFG_REG, data);	// setup lpf
//	return i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_CFG_REG, data);	// setup lpf
}

uint8_t MPU6050_Set_Rate(uint16_t rate)
{
	uint8_t data;
	
	if (rate > 1000) {
		rate = 1000;
	}
	
	if (rate < 4) {
		rate = 4;
	}
	
	data = 1000 / rate - 1;
	
	i2cWriteBuffer(MPU_ADDR_AD0, MPU_SAMPLE_RATE_REG, 1, &data);
//	data = i2cWrite(MPU_ADDR_AD0, MPU_SAMPLE_RATE_REG, data);
//	data = i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_SAMPLE_RATE_REG, data);
	
	return MPU6050_Set_LPF(rate / 2);
}

int MPU6050_Soft_I2C_Init(void)
{
	uint8_t res;
	uint8_t data;
	
	data = 0x80;
	i2cWriteBuffer(MPU_ADDR_AD0, MPU_PWR_MGMT1_REG, 1, &data);
//	i2cWrite(MPU_ADDR_AD0, MPU_PWR_MGMT1_REG, 0x80);			// reset MPU6050
//	i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_PWR_MGMT1_REG, 0x80);			// reset MPU6050
	
	delay(100);
	
	data = 0x00;
	i2cWriteBuffer(MPU_ADDR_AD0, MPU_PWR_MGMT1_REG, 1, &data);
//	i2cWrite(MPU_ADDR_AD0, MPU_PWR_MGMT1_REG, 0x00);			// wake up MPU6050
//	i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_PWR_MGMT1_REG, 0x00);			// wake up MPU6050
	
	MPU6050_Set_Gyro_FSR(3);												// ±2000dps
	
	MPU6050_Set_Accel_FSR(0);												// ±2g
	
	MPU6050_Set_Rate(50);													// sampling rate 50Hz
	
	data = 0x00;
	i2cWriteBuffer(MPU_ADDR_AD0, MPU_INT_EN_REG, 1, &data);
//	i2cWrite(MPU_ADDR_AD0, MPU_INT_EN_REG, 0x00);				// disable all interrupt
//	i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_INT_EN_REG, 0x00);				// disable all interrupt
	
	data = 0x00;
	i2cWriteBuffer(MPU_ADDR_AD0, MPU_USER_CTRL_REG, 1, &data);
//	i2cWrite(MPU_ADDR_AD0, MPU_USER_CTRL_REG, 0x00);			// disable I2C master mode
//	i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_USER_CTRL_REG, 0x00);			// disable I2C master mode
	
	data = 0x00;
	i2cWriteBuffer(MPU_ADDR_AD0, MPU_FIFO_EN_REG, 1, &data);
//	i2cWrite(MPU_ADDR_AD0, MPU_FIFO_EN_REG, 0x00);				// disable FIFO
//	i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_FIFO_EN_REG, 0x00);				// disable FIFO
	
	data = 0x80;
	i2cWriteBuffer(MPU_ADDR_AD0, MPU_INTBP_CFG_REG, 1, &data);
//	i2cWrite(MPU_ADDR_AD0, MPU_INTBP_CFG_REG, 0x80);			// INT low voltage level
//	i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_INTBP_CFG_REG, 0x80);			// INT low voltage level
	
	i2cRead(MPU_ADDR_AD0, MPU_DEVICE_ID_REG, 1, &res);
//	i2cRead(I2C_DEVICE, MPU_ADDR_AD0, MPU_DEVICE_ID_REG, 1, &res);
	
	printf("MPU_ADDR_res: 0x%x, %s, %d\r\n", res, __FUNCTION__, __LINE__);
	
	if (res == MPU_ADDR_AD0) {
		printf("MPU6050 is initialised successfully!, %s, %d\r\n", __FUNCTION__, __LINE__);
	}else {
		printf("Failed to initialise MPU6050!, %s, %d\r\n", __FUNCTION__, __LINE__);
		return -1;
	}
	
	return 0;
}

uint8_t MPU6050_Get_Gyroscope_Data(short *gx, short *gy, short *gz)
{
	uint8_t buf[6], res;
	
	res = i2cRead(MPU_ADDR_AD0, MPU_GYRO_XOUTH_REG, 6, buf);
//	printf("res: %d, %s, %d\r\n", res, __FUNCTION__, __LINE__);
//	res = i2cRead(I2C_DEVICE, MPU_ADDR_AD0, MPU_GYRO_XOUTH_REG, 6, buf);
	if (res == 1) {
		*gx = ((uint16_t)buf[0] << 8) | buf[1];
//		printf("*gx: %d, %s, %d\r\n", *gx, __FUNCTION__, __LINE__);
		*gy = ((uint16_t)buf[2] << 8) | buf[3];
//		printf("*gy: %d, %s, %d\r\n", *gy, __FUNCTION__, __LINE__);
		*gz = ((uint16_t)buf[4] << 8) | buf[5];
//		printf("*gz: %d, %s, %d\r\n", *gz, __FUNCTION__, __LINE__);
	}
	
	return res;
}

uint8_t MPU6050_Get_Accelerometer_Data(short *ax, short *ay, short *az)
{
	uint8_t buf[6], res;
	
	res = i2cRead(MPU_ADDR_AD0, MPU_ACCEL_XOUTH_REG, 6, buf);
//	res = i2cRead(I2C_DEVICE, MPU_ADDR_AD0, MPU_ACCEL_XOUTH_REG, 6, buf);
	if (res == 1) {
		*ax = ((uint16_t)buf[0] << 8) | buf[1];
		*ay = ((uint16_t)buf[2] << 8) | buf[3];
		*az = ((uint16_t)buf[4] << 8) | buf[5];
	}
	
	return res;
}

float MPU6050_Get_Temperature_Data(void)
{
	uint8_t buf[2];
	short raw;
	float temp;
	
	i2cRead(MPU_ADDR_AD0, MPU_TEMP_OUTH_REG, 2, buf);
//	i2cRead(I2C_DEVICE, MPU_ADDR_AD0, MPU_TEMP_OUTH_REG, 2, buf);
	
	raw = ((uint16_t)buf[0] << 8) | buf[1];
	temp = ((double)raw)/340 + 36.53;
	
	return temp;
}

//串口3发送1个字符 
//c:要发送的字符
void usart3_send_char(uint8_t c)
{
//	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET) {
//		USART_SendData(USART3, c);
//	}
	rxSerial3TestWrite(c);
}

//传送数据给匿名四轴上位机软件(V2.6版本)
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
void usart3_ano_report(uint8_t fun, uint8_t *data, uint8_t len)
{
	uint8_t send_buf[32];
	uint8_t i;
	
	if (len > 28)							// 最多28字节数据
		return;
	
	send_buf[len+3] = 0;					// 校验数置零
	send_buf[0] = 0x88;						// 帧头
	send_buf[1] = fun;						// 功能字
	send_buf[2] = len;						// 数据长度
	for (i = 0; i < len; i++) {
		send_buf[3+i] = data[i];			// 复制数据
	}
	
	for (i = 0; i < len+3; i++) {
		send_buf[len+3] += send_buf[i];		// 计算校验和
	}
	
	for (i = 0; i < len+4; i++) {
		usart3_send_char(send_buf[i]);		// 发送数据到串口3
	}
}

//发送加速度传感器数据和陀螺仪数据
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
void mpu6050_send_data(short accx, short accy, short accz, short gyrox, short gyroy, short gyroz)
{
	uint8_t tbuf[12];
	uint8_t i;
	
	for (i = 0; i < 13; i++) {
		tbuf[i] = 0;
	}
	
	tbuf[0] = (accx >> 8) & 0xff;
	tbuf[1] = accx & 0xff;
	tbuf[2] = (accy >> 8) & 0xff;
	tbuf[3] = accy & 0xff;
	tbuf[4] = (accz >> 8) & 0xff;
	tbuf[5] = accz & 0xff;
	tbuf[6] = (gyrox >> 8) & 0xff;
	tbuf[7] = gyrox & 0xff;
	tbuf[8] = (gyroy >> 8) & 0xff;
	tbuf[9] = gyroy & 0xff;
	tbuf[10] = (gyroz >> 8) & 0xff;
	tbuf[11] = gyroz & 0xff;
	usart3_ano_report(0xA1, tbuf, 12);		// 自定义帧,0xA1
}

//通过串口1上报结算后的姿态数据给电脑
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
void usart3_report_imu(short accx, short accy, short accz, short gyrox, short gyroy, short gyroz, short roll, short pitch, short yaw)
{
	uint8_t tbuf[28];
	uint8_t i;
	
	for (i = 0; i < 28; i++) {
		tbuf[i] = 0;
	}
	
	tbuf[0] = (accx >> 8) & 0xff;
	tbuf[1] = accx & 0xff;
	tbuf[2] = (accy >> 8) & 0xff;
	tbuf[3] = accy & 0xff;
	tbuf[4] = (accz >> 8) & 0xff;
	tbuf[5] = accz & 0xff;
	tbuf[6] = (gyrox >> 8) & 0xff;
	tbuf[7] = gyrox & 0xff;
	tbuf[8] = (gyroy >> 8) & 0xff;
	tbuf[9] = gyroy & 0xff;
	tbuf[10] = (gyroz >> 8) & 0xff;
	tbuf[11] = gyroz & 0xff;
	tbuf[18] = (roll >> 8) & 0xff;
	tbuf[19] = roll & 0xff;
	tbuf[20] = (pitch >> 8) & 0xff;
	tbuf[21] = pitch & 0xff;
	tbuf[22] = (yaw >> 8) & 0xff;
	tbuf[23] = yaw & 0xff;
	usart3_ano_report(0xAF, tbuf, 28);
}

#endif	// end of USE_I2C
