
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "bitband_i2c_soft.h"
#include "system.h"

void IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	// enable GPIOB clock
	
	/* Initialise GPIOB 8 and 9 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;			// Open-drain
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			// Push-pull
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;						// Low speed
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		// Just for testing
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	IIC_SCL = 1;		// PB8
	IIC_SDA = 1;		// PB9
}

void IIC_Start(void)
{
	SDA_OUT();			// SDA output mode
	IIC_SDA = 1;
	IIC_SCL = 1;
//	delay(500);			// For testing using oscilloscope
	delayMicroseconds(4);
	IIC_SDA = 0;		// START: when CLK line is high, DATA line changes from high to low
//	delay(500);			// For testing using oscilloscope
	delayMicroseconds(4);
	IIC_SCL = 0;
}

void IIC_Stop(void)
{
	SDA_OUT();
	IIC_SCL = 0;
	IIC_SDA = 0;		// STOP: when CLK line is high, DATA line changes from low to high
//	delay(500);			// For testing using oscilloscope
	delayMicroseconds(4);
	IIC_SCL = 1;
	IIC_SDA = 1;		// Send I2C terminal signal
//	delay(500);			// For testing using oscilloscope
	delayMicroseconds(4);
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime = 0;
	SDA_IN();			// SDA设置为输入
	IIC_SDA = 1;
	delayMicroseconds(1);
	IIC_SCL = 1;
	delayMicroseconds(1);
	while (READ_SDA) {
		ucErrTime++;
		if (ucErrTime > 250) {
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL = 0;		// 时钟输出0
	return 0;
}

void IIC_Ack(void)
{
	IIC_SCL = 0;
	SDA_OUT();
	IIC_SDA = 0;
//	delay(200);			// For testing using oscilloscope
	delayMicroseconds(2);
	IIC_SCL = 1;
//	delay(200);			// For testing using oscilloscope
	delayMicroseconds(2);
	IIC_SCL = 0;
}

void IIC_NAck(void)
{
	IIC_SCL = 0;
	SDA_OUT();
	IIC_SDA = 1;
//	delay(200);			// For testing using oscilloscope
	delayMicroseconds(2);
	IIC_SCL = 1;
//	delay(200);			// For testing using oscilloscope
	delayMicroseconds(2);
	IIC_SCL = 0;
}

// IIC发送一个字节
// 返回从机有无应答
// 1，有应答
// 0，无应答		
void IIC_Send_Byte(uint8_t txd)
{
	uint8_t i;
	SDA_OUT();

	/* All SDA changes should take place when SCL is low, with the exception of start and stop conditions. */
	IIC_SCL = 0;		// 拉低时钟开始数据传输

	for (i = 0; i < 8; i++) {
		IIC_SDA = (txd & 0x80) >> 7;
		txd <<= 1;
		delayMicroseconds(2);
		IIC_SCL = 1;
		delayMicroseconds(2);
		IIC_SCL = 0;
		delayMicroseconds(2);
	}
}

// 读1个字节，ack=1时，发送ACK，ack=0，发送nACK
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i, receive = 0;
	SDA_IN();			// SDA设置为输入
	for (i = 0; i < 8; i++) {
		IIC_SCL = 0;
		delayMicroseconds(2);
		IIC_SCL = 1;
		receive <<= 1;
		if (READ_SDA)
			receive++;
		delayMicroseconds(1);
	}
	if (!ack) {
		IIC_NAck();		// 发送NAck
	}else {
		IIC_Ack();		// 发送Ack
	}
	
	return receive;
}

uint8_t IIC_Read_Byte2(void)
{
	unsigned char i, receive = 0;
	SDA_IN();			// SDA设置为输入
	for (i = 0; i < 8; i++) {
		IIC_SCL = 0;
		delayMicroseconds(2);
		IIC_SCL = 1;
		receive <<= 1;
		if (READ_SDA)
			receive++;
		delayMicroseconds(1);
	}
	
	return receive;
}

//void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr, uint8_t data)
//{
//	
//}

//uint8_t IIC_Read_One_Byte(uint8_t daddr, uint8_t addr)
//{
//	return 0;
//}
