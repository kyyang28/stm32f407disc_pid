
#include "stm32f4xx_i2c.h"
#include "bus_i2c.h"
#include "utils.h"
#include "io.h"

#include "system.h"			// delay, delayMicroseconds
#include "stdio.h"

#include "bitband_i2c_soft.h"

// Software I2C driver, using same pins as hardware I2C, with hw i2c module disabled.
// Can be configured for I2C2 pinout (SCL: PB10, SDA: PB11) or I2C1 pinout (SCL: PB6, SDA: PB7)

#ifdef USE_I2C

static IO_t scl;
static IO_t sda;
static volatile uint16_t i2cErrorCount = 0;

#define SCL_H					IOHi(scl)
#define SCL_L					IOLo(scl)

#define SDA_H					IOHi(sda)
#define SDA_L					IOLo(sda)

#define SCL_read				IORead(scl)
#define SDA_read				IORead(sda)

static void I2C_delay(void)
{
	volatile int i = 7;
	while (i) {
		i--;
	}
}

//bool I2C_Start(void)
static bool I2C_Start(void)
{
	SDA_H;
	SCL_H;
	I2C_delay();
//	delay(500);				// for testing purpose
	if (!SDA_read) {
		return false;
	}
	SDA_L;
	I2C_delay();
//	delay(500);				// for testing purpose
	if (SDA_read) {
		return false;
	}
	SDA_L;
//	SCL_L;
	I2C_delay();
//	delay(500);				// for testing purpose
	return true;
}

//void I2C_Stop(void)
static void I2C_Stop(void)
{
	SCL_L;
	I2C_delay();
//	delay(500);
	SDA_L;
	I2C_delay();
//	delay(500);
	SCL_H;
	I2C_delay();
//	delay(500);
	SDA_H;
	I2C_delay();
//	delay(500);
}

//void I2C_Ack(void)
static void I2C_Ack(void)
{
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}

//void I2C_NoAck(void)
static void I2C_NoAck(void)
{
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}

//bool I2C_WaitAck(void)
static bool I2C_WaitAck(void)
{
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
//	if (!SDA_read) {		// WARNING: SDA_read should be LOW here, do not use !SDA_read
//	printf("SDA_read value: %d, %s, %d\r\n", SDA_read, __FUNCTION__, __LINE__);
	if (SDA_read) {
//		printf("SDA_read: %s, %d\r\n", __FUNCTION__, __LINE__);
		SCL_L;
		return false;
	}
	SCL_L;
	return true;
}

//void I2C_SendByte(uint8_t byte)
static void I2C_SendByte(uint8_t byte)
{
	uint8_t i = 8;
	while (i--) {
		SCL_L;
		I2C_delay();
//		delay(200);
		if (byte & 0x80) {
			SDA_H;
		}else {
			SDA_L;
		}
		byte <<= 1;
		I2C_delay();
//		delay(200);
		SCL_H;
		I2C_delay();
//		delay(200);
	}
	SCL_L;
}

static uint8_t I2C_ReceiveByte(void)
{
	uint8_t i = 8;
	uint8_t byte = 0;
	
	SDA_H;
	while (i--) {
		byte <<= 1;
		SCL_L;
		I2C_delay();
		SCL_H;
		I2C_delay();
		if (SDA_read) {
			byte |= 0x01;
		}
	}
	SCL_L;
	return byte;
}

void i2cInit(I2CDevice device)
{
	UNUSED(device);
	
	scl = IOGetByTag(IO_TAG(SOFT_I2C_SCL));
	sda = IOGetByTag(IO_TAG(SOFT_I2C_SDA));
//	ioRec_t *ioRecSCL = IO_Rec(scl);
//	printf("ioRecSCL->gpio: 0x%x\r\n", (uint32_t)ioRecSCL->gpio);		// gpio = 0x40020400 (GPIOB)
//	printf("ioRecSCL->pin: %u\r\n", ioRecSCL->pin);						// pin = 256 = 0x100 (1 << 8)
//	ioRec_t *ioRecSDA = IO_Rec(sda);
//	printf("ioRecSDA->gpio: 0x%x\r\n", (uint32_t)ioRecSDA->gpio);		// gpio = 0x40020400 (GPIOB)
//	printf("ioRecSDA->pin: %u\r\n", ioRecSDA->pin);						// pin = 512 = 0x200 (1 << 9)
	
//	IOConfigGPIO(scl, IOCFG_OUT_PP);			// Push-pull
//	IOConfigGPIO(sda, IOCFG_OUT_PP);			// Push-pull
//	IOConfigGPIO(scl, IOCFG_OUT_PP_UP);
//	IOConfigGPIO(sda, IOCFG_OUT_PP_UP);
	IOConfigGPIO(scl, IOCFG_OUT_OD);			// Open-drain
	IOConfigGPIO(sda, IOCFG_OUT_OD);			// Open-drain
}

bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
//bool i2cWrite(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t data)
{
//	UNUSED(device);
	
	if (!I2C_Start()) {
		return false;
	}
	
	I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
	if (!I2C_WaitAck()) {
		I2C_Stop();
		i2cErrorCount++;
		return false;
	}
	
	I2C_SendByte(reg);
	I2C_WaitAck();
	I2C_SendByte(data);
	I2C_WaitAck();
	I2C_Stop();
	return true;
}

#if 1
bool i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
//bool i2cWriteBuffer(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
//	UNUSED(device);
	int i;
	
	if (!I2C_Start()) {
		i2cErrorCount++;
		return false;
	}
	
	I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
	
	if (!I2C_WaitAck()) {
		I2C_Stop();
		return false;
	}
	
	I2C_SendByte(reg);
	I2C_WaitAck();
	
	for (i = 0; i < len; i++) {
		I2C_SendByte(data[i]);
		if (!I2C_WaitAck()) {
			I2C_Stop();
			i2cErrorCount++;
			return false;
		}
	}
	
	I2C_Stop();
	return true;
}
#else
int i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
//bool i2cWriteBuffer(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
//	UNUSED(device);
	int i;
	
	if (!I2C_Start()) {
		i2cErrorCount++;
		return -1;
	}
	
	I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
	
	if (!I2C_WaitAck()) {
		I2C_Stop();
		return -1;
	}
	
	I2C_SendByte(reg);
	I2C_WaitAck();
	
	for (i = 0; i < len; i++) {
		I2C_SendByte(data[i]);
		if (!I2C_WaitAck()) {
			I2C_Stop();
			i2cErrorCount++;
			return -1;
		}
	}
	
	I2C_Stop();
	return 0;
}
#endif

#if 1
bool i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
//bool i2cRead(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
//	UNUSED(device);
	
	if (!I2C_Start()) {
		printf("I2C_Start failed: %s, %d\r\n", __FUNCTION__, __LINE__);
		return false;
	}
	I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
	if (!I2C_WaitAck()) {
		printf("I2C_WaitAck failed: %s, %d\r\n", __FUNCTION__, __LINE__);
		I2C_Stop();
		i2cErrorCount++;
		return false;
	}
	I2C_SendByte(reg);
	I2C_WaitAck();
	I2C_Start();
	I2C_SendByte(addr << 1 | I2C_Direction_Receiver);
	I2C_WaitAck();
	while (len) {
		*buf = I2C_ReceiveByte();
		if (len == 1) {
			I2C_NoAck();
		}else {
			I2C_Ack();
		}
		buf++;
		len--;
	}
	I2C_Stop();
	return true;
}
#else
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
//bool i2cRead(I2CDevice device, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
//	UNUSED(device);
	
	if (!I2C_Start()) {
		printf("I2C_Start failed: %s, %d\r\n", __FUNCTION__, __LINE__);
		return -1;
	}
	I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
	if (!I2C_WaitAck()) {
		printf("I2C_WaitAck failed: %s, %d\r\n", __FUNCTION__, __LINE__);
		I2C_Stop();
		i2cErrorCount++;
		return -1;
	}
	I2C_SendByte(reg);
	I2C_WaitAck();
	I2C_Start();
	I2C_SendByte(addr << 1 | I2C_Direction_Receiver);
	I2C_WaitAck();
	while (len) {
		*buf = I2C_ReceiveByte();
		if (len == 1) {
			I2C_NoAck();
		}else {
			I2C_Ack();
		}
		buf++;
		len--;
	}
	I2C_Stop();
	return 0;
}
#endif

uint16_t i2cGetErrorCounter(void)
{
	return i2cErrorCount;
}

#endif		// end of USE_I2C
