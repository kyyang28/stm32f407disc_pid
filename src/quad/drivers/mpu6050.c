
#include "mpu6050.h"
#include "bitband_i2c_soft.h"
#include "system.h"
#include "stm32f4xx_usart.h"
#include "rxSerial3Test.h"
#include <stdio.h>				// debugging purposes

// IIC写一个字节 

// reg:寄存器地址

// data:数据

// 返回值:0,正常

// 其他,错误代码
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data)
{
	IIC_Start();
	IIC_Send_Byte((MPU_ADDR_AD0 << 1) | 0);			// 发送设备地址 + 写命令
	
	if (IIC_Wait_Ack()) {						// 等待应答
		IIC_Stop();								// I2C传输停止
		return 1;
	}
	
	IIC_Send_Byte(reg);							// 写寄存器地址
	IIC_Wait_Ack();								// 等待应答
	IIC_Send_Byte(data);						// 发送数据
	
	if (IIC_Wait_Ack()) {						// 等待Ack
		IIC_Stop();								// I2C传输停止
		return 1;
	}
	
	IIC_Stop();									// I2C传输停止
	return 0;
}

//IIC读一个字节 

//reg:寄存器地址 

//返回值:读到的数据
uint8_t MPU_Read_Byte(uint8_t reg)
{
	uint8_t res;
	
	IIC_Start();							// 开始IIC传输
	
	IIC_Send_Byte((MPU_ADDR_AD0 << 1) | 0);		// 发送设备地址 + 写命令
	
	IIC_Wait_Ack();							// 等待应答
	
	IIC_Send_Byte(reg);						// 发送寄存器地址
	
	IIC_Wait_Ack();							// 等待应答
	
	IIC_Start();							// 再次开始IIC传输
	
	IIC_Send_Byte((MPU_ADDR_AD0 << 1) | 1);		// 发送设备地址 + 读命令
	
	IIC_Wait_Ack();							// 等待应答
	
	res = IIC_Read_Byte(0);					// 读取数据， 发送nAck
		
	IIC_Stop();								// 产生一个停止条件
	
	return res;
}

//设置MPU6050陀螺仪传感器满量程范围

//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps

//返回值:0,设置成功

//    其他,设置失败 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3);		// 设置陀螺仪满量程范围
}

//设置MPU6050加速度传感器满量程范围

//fsr:0,±2g;1,±4g;2,±8g;3,±16g

//返回值:0,设置成功

//    其他,设置失败 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3);		// 设置加速度计传感器满量程范围
}

//设置MPU6050的数字低通滤波器

//lpf:数字低通滤波频率(Hz)

//返回值:0,设置成功

//    其他,设置失败 
uint8_t MPU_Set_LPF(uint16_t lpf)
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
	
	return MPU_Write_Byte(MPU_CFG_REG, data);				// 设置数字低通滤波器
}

//设置MPU6050的采样率(假定Fs=1KHz)

//rate:4~1000(Hz)

//返回值:0,设置成功

//    其他,设置失败 
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	
	if (rate > 1000) {
		rate = 1000;
	}
	
	if (rate < 4) {
		rate = 4;
	}
	
	data = 1000 / rate - 1;
	
	data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data);		// 设置数字低通滤波器
	
	return MPU_Set_LPF(rate / 2);							// 自动设置LPF为采样率的一半
}

uint8_t MPU_Set_Fifo(uint8_t sens)
{
	return 0;
}

//IIC连续写 (Burst write)
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	uint8_t i;
	
	IIC_Start();
	IIC_Send_Byte((addr << 1) | 0);			// send device address + write cmd
	
	if (IIC_Wait_Ack()) {					// wait ack
		IIC_Stop();
		return 1;
	}
	
	IIC_Send_Byte(reg);						// send register address
	IIC_Wait_Ack();							// wait ack
	
	for (i = 0; i < len; i++) {
		IIC_Send_Byte(buf[i]);				// send data one at a time
		
		if (IIC_Wait_Ack()) {				// wait ack
			IIC_Stop();
			return 1;
		}
	}
	
	IIC_Stop();
	return 0;
}

//IIC连续读 (Burst read)
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	IIC_Start();							// start IIC
	IIC_Send_Byte((addr << 1) | 0);			// send device address + write cmd
	if (IIC_Wait_Ack()) {					// wait response
		IIC_Stop();
		return 1;
	}
	
	IIC_Send_Byte(reg);						// send register address
	IIC_Wait_Ack();							// wait ack
	
	IIC_Start();							// start IIC
	IIC_Send_Byte((addr << 1) | 1);			// send device address + read cmd
	IIC_Wait_Ack();							// wait ack
	
	while (len) {
		if (len == 1) {
			*buf = IIC_Read_Byte(0);		// read data, send nAck
		}else {
			*buf = IIC_Read_Byte(1);		// read data, send ack
		}
		
		len--;
		buf++;
	}
	
	IIC_Stop();								// generate a IIC stop signal
	return 0;
}

// 初始化MPU6050
uint8_t MPU_Init(void)
{
	uint8_t resID;
	
//	IIC_Init();									// 初始化IIC总线
	
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x80);	// 复位MPU6050
//	i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_PWR_MGMT1_REG, 0x80);			// reset MPU6050
	
	delay(30);									// 延迟30 ms
//	delay(100);									// 延迟100 ms
	
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x00);	// 唤醒MPU6050
//	i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_PWR_MGMT1_REG, 0x00);			// wake up MPU6050
	
	MPU_Set_Gyro_Fsr(3);						// 设置陀螺仪传感器, ±2000dps
//	MPU6050_Set_Gyro_FSR(3);												// ±2000dps
	
	MPU_Set_Accel_Fsr(0);						// 设置加速度计传感器, ±2g
//	MPU6050_Set_Accel_FSR(0);												// ±2g
		
	MPU_Set_Rate(50);							// 设置采样率50Hz
//	MPU6050_Set_Rate(50);													// sampling rate 50Hz
	
	MPU_Write_Byte(MPU_INT_EN_REG, 0x00);		// 关闭所有中断
//	i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_INT_EN_REG, 0x00);				// disable all interrupt
	
	MPU_Write_Byte(MPU_USER_CTRL_REG, 0x00);	// I2C主模式关闭
//	i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_USER_CTRL_REG, 0x00);			// disable I2C master mode
	
	MPU_Write_Byte(MPU_FIFO_EN_REG, 0x00);		// 关闭FIFO
//	i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_FIFO_EN_REG, 0x00);				// disable FIFO
	
	MPU_Write_Byte(MPU_INTBP_CFG_REG, 0x80);	// INT引脚低电平有效
//	i2cWrite(I2C_DEVICE, MPU_ADDR_AD0, MPU_INTBP_CFG_REG, 0x80);			// INT low voltage level
	
	resID = MPU_Read_Byte(MPU_DEVICE_ID_REG);
//	i2cRead(I2C_DEVICE, MPU_ADDR_AD0, MPU_DEVICE_ID_REG, 1, &resID);
	
	printf("MPU_ADDR_resID: 0x%x, %s, %d\r\n", resID, __FUNCTION__, __LINE__);
	
	if (resID == MPU_ADDR_AD0) {						// 设备ID正确
		printf("MPU6050 is initialised successfully!, %s, %d\r\n", __FUNCTION__, __LINE__);
		MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x01);	// 设置CLKSEL, PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0x00);	// 加速度计与陀螺仪都工作
		MPU_Set_Rate(50);							// 设置采样率50Hz
	}else {
		printf("Failed to initialise MPU6050!, %s, %d\r\n", __FUNCTION__, __LINE__);
		return 1;
	}
	
	return 0;
}

//得到温度值
//返回值:温度值
float MPU_Get_Temperature(void)
//short MPU_Get_Temperature(void)
{
	uint8_t buf[2];
	short raw;
	float temp;
	
	MPU_Read_Len(MPU_ADDR_AD0, MPU_TEMP_OUTH_REG, 2, buf);
	
	raw = ((uint16_t)buf[0] << 8) | buf[1];
	temp = ((double)raw)/340 + 36.53;
	
	return temp;
//	return temp*100;
}

//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
	uint8_t buf[6], res;
	
	res = MPU_Read_Len(MPU_ADDR_AD0, MPU_GYRO_XOUTH_REG, 6, buf);
	if (res == 0) {
		*gx = ((uint16_t)buf[0] << 8) | buf[1];
		*gy = ((uint16_t)buf[2] << 8) | buf[3];
		*gz = ((uint16_t)buf[4] << 8) | buf[5];
	}
	
	return res;
}

//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
	uint8_t buf[6], res;
	
	res = MPU_Read_Len(MPU_ADDR_AD0, MPU_ACCEL_XOUTH_REG, 6, buf);
	if (res == 0) {
		*ax = ((uint16_t)buf[0] << 8) | buf[1];
		*ay = ((uint16_t)buf[2] << 8) | buf[3];
		*az = ((uint16_t)buf[4] << 8) | buf[5];
	}
	
	return res;
}

#if 0
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
#endif
