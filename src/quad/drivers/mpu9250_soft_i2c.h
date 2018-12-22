#ifndef __MPU9250_SOFT_I2C_H
#define __MPU9250_SOFT_I2C_H

#include <stdint.h>

//#define MPU_ACCEL_OFFS_REG		0x06	//accel_offs寄存器,可读取版本号,寄存器手册未提到
//#define MPU_PROD_ID_REG			0x0C	//prod id寄存器,在寄存器手册未提到
#define MPU9250_SELF_TESTX_GYRO_REG	 0x00	//自检寄存器X GYRO
#define MPU9250_SELF_TESTY_GYRO_REG	 0x01	//自检寄存器Y GYRO
#define MPU9250_SELF_TESTZ_GYRO_REG	 0x02	//自检寄存器Z GYRO
#define MPU9250_SELF_TESTX_ACCEL_REG 0x0D	//自检寄存器X ACCEL
#define MPU9250_SELF_TESTY_ACCEL_REG 0x0E	//自检寄存器Y ACCEL
#define MPU9250_SELF_TESTZ_ACCEL_REG 0x0F	//自检寄存器Z ACCEL
#define MPU9250_SAMPLE_RATE_REG		 0x19	//采样频率分频器
#define MPU9250_CFG_REG				 0x1A	//配置寄存器
#define MPU9250_GYRO_CFG_REG		 0x1B	//陀螺仪配置寄存器
#define MPU9250_ACCEL_CFG_REG		 0x1C	//加速度计配置寄存器
#define MPU9250_MOTION_DET_REG		 0x1F	//运动检测阀值设置寄存器
#define MPU9250_FIFO_EN_REG			 0x23	//FIFO使能寄存器
#define MPU9250_I2CMST_CTRL_REG		 0x24	//IIC主机控制寄存器
#define MPU9250_I2CSLV0_ADDR_REG	 0x25	//IIC从机0器件地址寄存器
#define MPU9250_I2CSLV0_REG			 0x26	//IIC从机0数据地址寄存器
#define MPU9250_I2CSLV0_CTRL_REG	 0x27	//IIC从机0控制寄存器
#define MPU9250_I2CSLV1_ADDR_REG	 0x28	//IIC从机1器件地址寄存器
#define MPU9250_I2CSLV1_REG			 0x29	//IIC从机1数据地址寄存器
#define MPU9250_I2CSLV1_CTRL_REG	 0x2A	//IIC从机1控制寄存器
#define MPU9250_I2CSLV2_ADDR_REG	 0x2B	//IIC从机2器件地址寄存器
#define MPU9250_I2CSLV2_REG			 0x2C	//IIC从机2数据地址寄存器
#define MPU9250_I2CSLV2_CTRL_REG	 0x2D	//IIC从机2控制寄存器
#define MPU9250_I2CSLV3_ADDR_REG	 0x2E	//IIC从机3器件地址寄存器
#define MPU9250_I2CSLV3_REG			 0x2F	//IIC从机3数据地址寄存器
#define MPU9250_I2CSLV3_CTRL_REG	 0x30	//IIC从机3控制寄存器
#define MPU9250_I2CSLV4_ADDR_REG	 0x31	//IIC从机4器件地址寄存器
#define MPU9250_I2CSLV4_REG			 0x32	//IIC从机4数据地址寄存器
#define MPU9250_I2CSLV4_DO_REG		 0x33	//IIC从机4写数据寄存器
#define MPU9250_I2CSLV4_CTRL_REG	 0x34	//IIC从机4控制寄存器
#define MPU9250_I2CSLV4_DI_REG		 0x35	//IIC从机4读数据寄存器

#define MPU9250_I2CMST_STA_REG		 0x36	//IIC主机状态寄存器
#define MPU9250_INTBP_CFG_REG		 0x37	//中断/旁路设置寄存器
#define MPU9250_INT_EN_REG			 0x38	//中断使能寄存器
#define MPU9250_INT_STA_REG			 0x3A	//中断状态寄存器

#define MPU9250_ACCEL_XOUTH_REG		 0x3B	//加速度值,X轴高8位寄存器
#define MPU9250_ACCEL_XOUTL_REG		 0x3C	//加速度值,X轴低8位寄存器
#define MPU9250_ACCEL_YOUTH_REG		 0x3D	//加速度值,Y轴高8位寄存器
#define MPU9250_ACCEL_YOUTL_REG		 0x3E	//加速度值,Y轴低8位寄存器
#define MPU9250_ACCEL_ZOUTH_REG		 0x3F	//加速度值,Z轴高8位寄存器
#define MPU9250_ACCEL_ZOUTL_REG		 0x40	//加速度值,Z轴低8位寄存器

#define MPU9250_TEMP_OUTH_REG		 0x41	//温度值高八位寄存器
#define MPU9250_TEMP_OUTL_REG		 0x42	//温度值低8位寄存器

#define MPU9250_GYRO_XOUTH_REG		 0x43	//陀螺仪值,X轴高8位寄存器
#define MPU9250_GYRO_XOUTL_REG		 0x44	//陀螺仪值,X轴低8位寄存器
#define MPU9250_GYRO_YOUTH_REG		 0x45	//陀螺仪值,Y轴高8位寄存器
#define MPU9250_GYRO_YOUTL_REG		 0x46	//陀螺仪值,Y轴低8位寄存器
#define MPU9250_GYRO_ZOUTH_REG		 0x47	//陀螺仪值,Z轴高8位寄存器
#define MPU9250_GYRO_ZOUTL_REG		 0x48	//陀螺仪值,Z轴低8位寄存器

#define MPU9250_I2CSLV0_DO_REG		 0x63	//IIC从机0数据寄存器
#define MPU9250_I2CSLV1_DO_REG		 0x64	//IIC从机1数据寄存器
#define MPU9250_I2CSLV2_DO_REG		 0x65	//IIC从机2数据寄存器
#define MPU9250_I2CSLV3_DO_REG		 0x66	//IIC从机3数据寄存器

#define MPU9250_I2CMST_DELAY_REG	 0x67	//IIC主机延时管理寄存器
#define MPU9250_SIGPATH_RST_REG		 0x68	//信号通道复位寄存器
#define MPU9250_MDETECT_CTRL_REG	 0x69	//运动检测控制寄存器
#define MPU9250_USER_CTRL_REG		 0x6A	//用户控制寄存器
#define MPU9250_PWR_MGMT1_REG		 0x6B	//电源管理寄存器1
#define MPU9250_PWR_MGMT2_REG		 0x6C	//电源管理寄存器2 
#define MPU9250_FIFO_CNTH_REG		 0x72	//FIFO计数寄存器高八位
#define MPU9250_FIFO_CNTL_REG		 0x73	//FIFO计数寄存器低八位
#define MPU9250_FIFO_RW_REG			 0x74	//FIFO读写寄存器
#define MPU9250_DEVICE_ID_REG		 0x75	//器件ID寄存器
 
//如果AD0脚(9脚)接地,IIC地址为0x68(不包含最低位).
//如果接V3.3,则IIC地址为0x69(不包含最低位).
#define MPU9250_ADDR_AD0			 0x68
#define MPU9250_ADDR_AD1			 0x69

uint8_t MPU9250_Set_Gyro_FSR(uint8_t fsr);
uint8_t MPU9250_Set_Accel_FSR(uint8_t fsr);
uint8_t MPU9250_Set_LPF(uint16_t lpf);
uint8_t MPU9250_Set_Rate(uint16_t rate);

int MPU9250_Soft_I2C_Init(void);
uint8_t MPU9250_Get_Gyroscope_Data(short *gx, short *gy, short *gz);
uint8_t MPU9250_Get_Accelerometer_Data(short *ax, short *ay, short *az);
float MPU9250_Get_Temperature_Data(void);

void mpu9250_send_data(short accx, short accy, short accz, short gyrox, short gyroy, short gyroz);
void MPU9250_usart3_report_imu(short accx, short accy, short accz, short gyrox, short gyroy, short gyroz, short roll, short pitch, short yaw);

#endif	// __MPU9250_SOFT_I2C_H
