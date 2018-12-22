#ifndef __BITBAND_I2C_SOFT_H
#define __BITBAND_I2C_SOFT_H

#include <stdint.h>
#include "stm32f4xx.h"

//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).M4同M3类似,只是寄存器地址变了.
//IO口操作宏定义
#define BITBAND(addr, bitnum) 			((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  				*((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   		MEM_ADDR(BITBAND(addr, bitnum)) 

#define GPIOB_ODR_Addr    (GPIOB_BASE+20) 			//0x40020414
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) 			//0x40020410

//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PBout(n)   			BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    			BIT_ADDR(GPIOB_IDR_Addr,n)  //输入

/* configuration of IO direction */
#define SDA_IN()  { GPIOB->MODER &= ~(3<<(9*2)); GPIOB->MODER |= 0<<9*2; }	// Input mode of PB9
#define SDA_OUT() { GPIOB->MODER &= ~(3<<(9*2)); GPIOB->MODER |= 1<<9*2;} 	// Output mode of PB9

/* IO operations */
#define IIC_SCL			PBout(8)			// SCL
#define IIC_SDA			PBout(9)			// SDA
#define READ_SDA		PBin(9)				// SDA input

//IIC所有操作函数
void IIC_Init(void);                		//初始化IIC的IO口				 
void IIC_Start(void);						//发送IIC开始信号
void IIC_Stop(void);	  					//发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);	//IIC读取一个字节
uint8_t IIC_Read_Byte2(void);
uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);							//IIC发送ACK信号
void IIC_NAck(void);						//IIC不发送ACK信号

//void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr, uint8_t data);
//uint8_t IIC_Read_One_Byte(uint8_t daddr, uint8_t addr);

#endif	// __BITBAND_I2C_SOFT_H
