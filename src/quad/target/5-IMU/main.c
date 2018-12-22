
#include "system_stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"
#include "nvic.h"
#include "system.h"
#include "led.h"
#include "sound_beeper.h"
#include "config.h"
#include "exti.h"
#include "button.h"
#include "serial.h"
//#include "msp_serial.h"
//#include "printf.h"
#include "gps.h"
#include "rxSerial1Test.h"
#include "rxSerial3Test.h"
#include "rxSerial6Test.h"
#include <stdio.h>
//#include "stdio.h"
#include "bus_i2c.h"
#include "bitband_i2c_soft.h"					// self-implemented i2c protocol
#include "bus_spi.h"
#include "initialisation.h"
#include "accgyro_spi_mpu9250.h"
#include "gyro.h"
#include "acceleration.h"
#include "maths.h"

//#include "mpu6050.h"
//#include "mpu6050_soft_i2c.h"					// for MPU6050 testing purposes
#include "mpu9250_soft_i2c.h"					// for MPU9250 testing purposes
//#include "inv_mpu.h"							// for MPU6050/MPU9250 DMP testing purposes
//#include "inv_mpu_dmp_motion_driver.h"			// for MPU6050/MPU9250 DMP testing purposes

//#define GPIO_PA1_PIN				PA1
//#define GPIO_PB8_PIN				PB8

//uint32_t counter = 0;

//uint32_t currentTimeUs;

//uint32_t base = 1;
//uint32_t sub = 2;			// build = 0x00000050
//uint32_t build;
//uint32_t tmp1, tmp2, tmp3, tmp4, tmp5, tmp6, tmp7, tmp8;

//uint8_t __g_basepri_save, __g_basepri_save2, __g_basepri_save3;
//uint8_t __g_ToDo, __g_ToDo2;
//uint8_t __basepriRestoreMem_inter;
//uint8_t basepri_val;
//int forCnt = 0;

//static IO_t g_scl;
//static IO_t g_sda;

typedef enum {
	SYSTEM_STATE_INITIALISING			= 0,
	SYSTEM_STATE_CONFIG_LOADED			= (1 << 0),
	SYSTEM_STATE_SENSORS_READY			= (1 << 1),
	SYSTEM_STATE_MOTORS_READY			= (1 << 2),
	SYSTEM_STATE_TRANSPONDER_ENABLED	= (1 << 3),
	SYSTEM_STATE_ALL_READY				= (1 << 7)
}systemState_e;

//int g_val = 0;

uint8_t systemState = SYSTEM_STATE_INITIALISING;
//IO_t PA1_PIN, PB8_PIN;


void systemInit(void);
//void gpioPA1Init(void);
//void gpioPA1Ops(void);
//void gpioPB8Init(void);
//void gpioPB8Ops(void);

int i_cnt = 0;
double f_cnt = 0.0;

#if 1
struct __FILE
{
    int dummy;
};

FILE __stdout;

int fputc(int ch, FILE *f)
{
    /* Send byte to USART */
//	gpsWrite(ch);
//	rxSerial1TestWrite(ch);
	rxSerial3TestWrite(ch);
//	rxSerial6TestWrite(ch);
    
    /* If everything is OK, you have to return character written */
    return ch;
    /* If character is not correct, you can return EOF (-1) to stop writing */
    //return -1;
}
#else
#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	rxSerial3TestWrite(ch);
	return ch;
}
#endif

//static IO_t mpuSpi9250CsPin = IO_NONE;

bool sdaFlag = false;

int main(void)
{
//	printfSupportInit();
	
	systemInit();
	
	/* Initialise IO (needed for all IO operations) */
	IOGlobalInit();
	
	/* Initialise EEPROM */
//	InitEEPROM();
	
	/* Check if EEPROM contains valid data */
	CheckEEPROMContainsValidData();

	/* Initialise serial port usage list */
	serialInit(SerialConfig());
	
	/* Initialise MSP (MCU Support Package) serial */
//	mspSerialInit();

//	gpsInit();
	
//	rxSerial1TestInit();
	rxSerial3TestInit();
//	rxSerial6TestInit();
	
//	gpsSetPrintfSerialPort();
	
	/* Read the contents of EEPROM */
//	ReadEEPROM();
	
	systemState |= SYSTEM_STATE_CONFIG_LOADED;
	
	/* Initialise leds */
	LedInit(LedStatusConfig());
	
	/* Initialise external interrupt */
	EXTIInit();
	
//	LedSet(LED3_NUM, true);		// LED3_NUM = 0
//	LedSet(LED4_NUM, true);		// LED4_NUM = 1
//	LedSet(LED5_NUM, true);		// LED5_NUM = 2
//	LedSet(LED6_NUM, true);		// LED6_NUM = 3

//	LED3_ON;
//	LED4_ON;
//	LED5_ON;
//	LED6_ON;

//	userBtnPollInit();
	
	/* Initialisae button with interrupt */
//	buttonInit();

#ifdef USE_SPI
	#ifdef USE_SPI_DEVICE_1
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		spiInit(SPIDEV_1);		// SPIDEV_1 = 0
	#endif
#endif

#ifdef BEEPER
	beeperInit(BeeperConfig());
#endif

#ifdef USE_I2C
	/* Initialise I2C device */
	i2cInit(I2C_DEVICE);
#endif

#if 0
//	IIC_Init();
#endif
	
//	g_scl = IOGetByTag(IO_TAG(SOFT_I2C_SCL));				// PB10
//	g_sda = IOGetByTag(IO_TAG(SOFT_I2C_SDA));				// PB11
//	ioRec_t *ioRecSDA = IO_Rec(g_sda);
//	printf("IO_GPIO(g_scl): 0x%x\r\n", (uint32_t)IO_GPIO(g_scl));		// gpio = 0x40021000 (GPIOE)
//	printf("IO_Pin(g_scl): %u\r\n", IO_Pin(g_scl));					// pin = 16 = 0x10 (1 << 4)
//	printf("IO_GPIO(g_sda): 0x%x\r\n", (uint32_t)IO_GPIO(g_sda));		// gpio = 0x40021000 (GPIOE)
//	printf("IO_Pin(g_sda): %u\r\n", IO_Pin(g_sda));					// pin = 16 = 0x10 (1 << 4)
	
//	IOConfigGPIO(g_scl, IOCFG_OUT_PP);
//	IOConfigGPIO(g_sda, IOCFG_OUT_PP);
//	IOConfigGPIO(g_scl, IOCFG_OUT_OD);
//	IOConfigGPIO(g_sda, IOCFG_OUT_OD);

//#define YELLOW_LED		PE4
//	IO_t yellowLedPin = IO_NONE;
//	yellowLedPin = IOGetByTag(IO_TAG(YELLOW_LED));
//	IOInit(yellowLedPin, OWNER_LED, 0);
//	IOConfigGPIO(yellowLedPin, IOCFG_OUT_PP);

//	mpuSpi9250CsPin = IOGetByTag(IO_TAG(MPU9250_CS_PIN));
	
//	ioRec_t *ioRecMPUSpi9250Cs = IO_Rec(IOGetByTag(IO_TAG(MPU9250_CS_PIN)));
//	printf("ioRecMPUSpi9250Cs->gpio: 0x%x\r\n", (uint32_t)ioRecMPUSpi9250Cs->gpio);		// gpio = 0x40021000 (GPIOE)
//	printf("ioRecMPUSpi9250Cs->pin: %u\r\n", ioRecMPUSpi9250Cs->pin);					// pin = 16 = 0x10 (1 << 4)

//	IOInit(mpuSpi9250CsPin, OWNER_MPU_CS, 0);

//	printf("ioRecMPUSpi9250Cs->owner: %u\r\n", ioRecMPUSpi9250Cs->owner);				// owner = 11
//	printf("ioRecMPUSpi9250Cs->index: %u\r\n", ioRecMPUSpi9250Cs->index);				// index = 0
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);		// ok
//	printf("IO_GPIOPortIdx(mpuSpi9250CsPin): %d\r\n", IO_GPIOPortIdx(mpuSpi9250CsPin));		// IO_GPIOPortIdx(mpuSpi9250CsPin) = 4
//	printf("DEFIO_IO_USED_COUNT: %d\r\n", DEFIO_IO_USED_COUNT);		// DEFIO_IO_USED_COUNT = 80
//	IOConfigGPIO(mpuSpi9250CsPin, SPI_IO_CS_CFG);
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);		// ok
	
#if 1
	if (!sensorsAutodetect(GyroConfig(), AccelerometerConfig())) {
		//failureMode();
		printf("Failed to initialise IMU!, %s, %d\r\n", __FUNCTION__, __LINE__);
		while (1) {
			/* BLUE LED */
			LED6_ON;
			delay(100);
			LED6_OFF;
			delay(100);
		}
	}
#endif
	
//	printf("Setup completed!\r\n");
	
//	gpioPA1Init();
//	gpioPB8Init();

#if 0
	short gyro_x, gyro_y, gyro_z;
	short acc_x, acc_y, acc_z;
	float temp = 0.0;
	float roll, pitch, yaw;				// Euler angles
#endif
	
#if 0
	uint8_t res = MPU_Init();
	printf("MPU_Init result: %u, %s, %d\r\n", res, __FUNCTION__, __LINE__);
#endif

#if 0
	while (mpu_dmp_init()) {
		printf("MPU6050 initialisation error!, %s, %d\r\n", __FUNCTION__, __LINE__);
		delay(200);
	}
	
	printf("MPU6050 initialisation is done using mpu_dmp_init @%s, %d\r\n", __FUNCTION__, __LINE__);
#endif

#if 0
	if (!MPU6050_Soft_I2C_Init()) {
		printf("MPU6050 initialisation is done @%s, %d\r\n", __FUNCTION__, __LINE__);
	}else {
		printf("Failed to initialise MPU6050 @%s, %d\r\n", __FUNCTION__, __LINE__);
		return -1;
	}
#endif

#if 0
	if (!MPU9250_Soft_I2C_Init()) {
		printf("MPU9250 initialisation is done @%s, %d\r\n", __FUNCTION__, __LINE__);
	}else {
		printf("Failed to initialise MPU9250 @%s, %d\r\n", __FUNCTION__, __LINE__);
		return -1;
	}
#endif
	
//	bool flag = false;

#if 0
	/* Board power on beeper */
	for (int i = 0; i < 10; i++) {
		delay(25);
		BEEP_ON;
		delay(25);
		BEEP_OFF;
	}
#endif
	
//	delay(3000);
#if 0
	/* Testing standard deviation functions */
	stdev_t var;
	float dataset[] = {727.7, 1086.5, 1091.0, 1361.3, 1490.5, 1956.1};
	devClear(&var);
	for (int i = 0; i < sizeof(dataset)/sizeof(dataset[0]); i++) {
		devPush(&var, dataset[i]);
	}
	float dev = devStandardDeviation(&var);
	printf("dev: %f, %s, %d\r\n", dev, __FUNCTION__, __LINE__);					// 420.962463
	printf("devint: %ld, %s, %d\r\n", lrintf(dev), __FUNCTION__, __LINE__);		// 421
#endif

	/* set gyro calibration cycles */
	gyroSetCalibrationCycles();
		
	while (1) {

#ifdef BEEPER
//		BEEP_ON;		// turn beeper on
//		delay(200);
//		BEEP_OFF;		// turn beeper off
//		delay(200);

	for (int i = 0; i < 10; i++) {
		LED3_TOGGLE;
		LED4_TOGGLE;
		delay(25);
		BEEP_ON;
		delay(25);
		BEEP_OFF;
	}
	delay(2000);
#endif
		
//		gyroUpdate();													// read gyro data
//		delay(100);
#if 1
		gyroUpdate();													// read gyro data
//		accUpdate(&AccelerometerConfig()->accelerometerTrims);			// read acc data
		if (gyro.dev.mpuDetectionResult.sensor == MPU_60x0 && gyro.dev.calibrationFlag) {
//			printf("MPU6050 data - gyroADCRaw[X]: %d, gyroADCRaw[Y]: %d, gyroADCRaw[Z]: %d, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyro.dev.gyroADCRaw[X], gyro.dev.gyroADCRaw[Y], gyro.dev.gyroADCRaw[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
//			printf("MPU6050 data - gyroADCCali[X]: %d, gyroADCCali[Y]: %d, gyroADCCali[Z]: %d, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyroADC[X], gyroADC[Y], gyroADC[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
//		}else if (gyro.dev.mpuDetectionResult.sensor == MPU_9250_I2C) {
		}else if (gyro.dev.mpuDetectionResult.sensor == MPU_9250_I2C && gyro.dev.calibrationFlag) {
			printf("MPU9250 (I2C) data - gyroADCRaw[X]: %d, gyroADCRaw[Y]: %d, gyroADCRaw[Z]: %d, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyro.dev.gyroADCRaw[X], gyro.dev.gyroADCRaw[Y], gyro.dev.gyroADCRaw[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
//			printf("MPU9250 (I2C) data - gyroADCCali[X]: %d, gyroADCCali[Y]: %d, gyroADCCali[Z]: %d, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyroADC[X], gyroADC[Y], gyroADC[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
//			printf("MPU9250 (I2C) data - gyroADCfiltered[X]: %.4f, gyroADCfiltered[Y]: %.4f, gyroADCfiltered[Z]: %.4f, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyro.gyroADCf[X], gyro.gyroADCf[Y], gyro.gyroADCf[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
		}else if (gyro.dev.mpuDetectionResult.sensor == MPU_9250_SPI && gyro.dev.calibrationFlag) {
//			printf("MPU9250 (SPI) data - gyroADCRaw[X]: %d, gyroADCRaw[Y]: %d, gyroADCRaw[Z]: %d, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyro.dev.gyroADCRaw[X], gyro.dev.gyroADCRaw[Y], gyro.dev.gyroADCRaw[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
//			printf("MPU9250 (SPI) data - gyroADCCali[X]: %d, gyroADCCali[Y]: %d, gyroADCCali[Z]: %d, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyroADC[X], gyroADC[Y], gyroADC[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
			printf("MPU9250 (SPI) data - gyroADCfiltered[X]: %.4f, gyroADCfiltered[Y]: %.4f, gyroADCfiltered[Z]: %.4f, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyro.gyroADCf[X], gyro.gyroADCf[Y], gyro.gyroADCf[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
		}
//		printf("gyro.dev.gyroADCRaw[X]: %d, %s, %d\r\n", gyro.dev.gyroADCRaw[X], __FUNCTION__, __LINE__);
//		printf("gyro.dev.gyroADCRaw[Y]: %d, %s, %d\r\n", gyro.dev.gyroADCRaw[Y], __FUNCTION__, __LINE__);
//		printf("gyro.dev.gyroADCRaw[Z]: %d, %s, %d\r\n", gyro.dev.gyroADCRaw[Z], __FUNCTION__, __LINE__);
		delay(100);
#endif
		
#if 0
		if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0) {
			/* Get raw data of MPU6050 (gyroscope, accelerometer, temperature) */
			MPU6050_Get_Gyroscope_Data(&gyro_x, &gyro_y, &gyro_z);
			MPU6050_Get_Accelerometer_Data(&acc_x, &acc_y, &acc_z);
			temp = MPU6050_Get_Temperature_Data();
			/* Send data to ANO flight controller software */
			mpu6050_send_data(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
			usart3_report_imu(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, (int)(roll*100), (int)(pitch*100), (int)(yaw*10));
			/* Display on serial terminal */
//			printf("MPU6050 data - gyro_x: %d, gyro_y: %d, gyro_z: %d, acc_x: %d, acc_y: %d, acc_z: %d, temp: %.2f\r\n", gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temp);
//			printf("Euler angles - roll: %.4f, pitch: %.4f, yaw: %.4f\r\n\r\n", roll, pitch, yaw);
//			delay(100);
		}else {
//			printf("mpu_dmp_get_data failed, %s, %d\r\n", __FUNCTION__, __LINE__);
//			delay(100);
		}
#endif

#if 0
		MPU6050_Get_Gyroscope_Data(&gyro_x, &gyro_y, &gyro_z);
		MPU6050_Get_Accelerometer_Data(&acc_x, &acc_y, &acc_z);
		temp = MPU6050_Get_Temperature_Data();
//		printf("MPU6050 gyro data: gyro_x: %d, gyro_y: %d, gyro_z: %d\r\n", gyro_x, gyro_y, gyro_z);
//		printf("MPU6050 temp data: %.2f\r\n", temp);
		printf("MPU6050 data - gyro_x: %d, gyro_y: %d, gyro_z: %d, acc_x: %d, acc_y: %d, acc_z: %d, temp: %.2f\r\n", gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temp);		
		delay(100);
#endif

#if 0
		MPU9250_Get_Gyroscope_Data(&gyro_x, &gyro_y, &gyro_z);
		MPU9250_Get_Accelerometer_Data(&acc_x, &acc_y, &acc_z);
		temp = MPU9250_Get_Temperature_Data();
//		printf("MPU6050 gyro data: gyro_x: %d, gyro_y: %d, gyro_z: %d\r\n", gyro_x, gyro_y, gyro_z);
//		printf("MPU6050 temp data: %.2f\r\n", temp);
		printf("MPU9250 data - gyro_x: %d, gyro_y: %d, gyro_z: %d, acc_x: %d, acc_y: %d, acc_z: %d, temp: %.2f\r\n", gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temp);		
		delay(100);
#endif
		
		/* +----------------------------------------------------------------------------------------+ */
		/* +------------------------------- bus_i2c_soft API testing -------------------------------+ */
		/* +----------------------------------------------------------------------------------------+ */
//		flag = I2C_Start();			// test ok
//		printf("I2C_Start flag: %d, %s, %d\r\n", flag, __FUNCTION__, __LINE__);
//		I2C_Stop();					// test ok
//		I2C_Ack();					// test ok
//		I2C_NoAck();				// test ok
//		flag = I2C_WaitAck();		// test ok
//		printf("I2C_WaitAck flag: %d, %s, %d\r\n", flag, __FUNCTION__, __LINE__);
//		I2C_SendByte(0x59);			// test ok, sends 'Y'
		/* +----------------------------------------------------------------------------------------+ */
		/* +------------------------------- bus_i2c_soft API testing -------------------------------+ */
		/* +----------------------------------------------------------------------------------------+ */
		
//		IIC_Start();
//		delay(1000);

//		IIC_Stop();
//		delay(1000);

//		IIC_Ack();
//		delay(1000);
//		IIC_NAck();
//		delay(1000);
		
//		IIC_SCL = 0;
//		delay(500);
//		IIC_SCL = 1;
//		delay(500);
		
//		IIC_SDA = 0;
//		delay(800);
//		IIC_SDA = 1;
//		delay(800);

//		printf("STM32F407 by QUADYANG using USART3 Port\r\n");
//		delay(1000);
		
//		IOHi(g_sda);
//		IOHi(g_scl);
////		I2C_delay();
//		delay(500);
//		printf("sda: %d\r\n", IORead(g_sda));
		
//		IOHi(g_scl);
//		IOHi(g_sda);
//		delay(500);
//		printf("scl: %d\r\n", IORead(g_scl));
//		printf("sda: %d\r\n", IORead(g_sda));
//		IOLo(g_scl);
//		IOLo(g_sda);
//		delay(500);
//		printf("scl: %d\r\n", IORead(g_scl));
//		printf("sda: %d\r\n", IORead(g_sda));

//		IOHi(g_sda);
//		delay(500);
//		printf("sda: %d\r\n", IORead(g_sda));

//		IOLo(g_sda);
//		delay(500);
//		printf("sda: %d\r\n", IORead(g_sda));		
		
//		IOHi(g_sda);
//		IOHi(g_scl);
////		I2C_delay();
//		delay(1000);
//		sdaFlag = IORead(g_sda);
//		printf("sdaFlag: %d\r\n", sdaFlag);
		
//		I2C_Start();
//		delayMicroseconds(3);
//		I2C_Stop();
//		delayMicroseconds(3);

//		printf("STM32F407 by QUADYANG using USART1 Port\r\n");
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//		f_cnt += 0.01;
//		printf("i_cnt: %d, f_cnt: %.4f\r\n", i_cnt++, f_cnt);
//		delay(1000);

//		IOWrite(yellowLedPin, false);		// high
//		delay(1000);
//		IOWrite(yellowLedPin, true);		// low
//		delay(1000);

//		mpu9250WriteRegister(0x1B, 0x10);
//		delay(1000);
//		uint8_t in;
//		mpu9250ReadRegister(0x1B, 1, &in);
//		printf("in: 0x%x\r\n", in);
		
//		IOLo(mpuSpi9250CsPin);
//		delayMicroseconds(1);
////		delay(1000);
//		spiTransferByte(MPU9250_SPI_INSTANCE, 0x1B);
//		IOHi(mpuSpi9250CsPin);
//		delayMicroseconds(1);

//		IOLo(mpuSpi9250CsPin);
//		delay(800);
//		IOHi(mpuSpi9250CsPin);
//		delay(800);
		
//		printf("QUADYANG\n");
//		delay(1000);
		//counter++;
				
//		userBtnPollOps();

//		gpioPA1Ops();
//		gpioPB8Ops();
		
//		LED3_ON;
//		delay(1000);
//		LED3_OFF;
//		delay(1000);
		
//		LED4_ON;
//		delay(700);
//		LED4_OFF;
//		delay(700);

//		LED5_ON;
//		delay(400);
//		LED5_OFF;
//		delay(400);

//		LED6_ON;
//		delay(100);
//		LED6_OFF;
//		delay(100);
		
//		msElapse = millis();
//		currentTimeUs = micros();
	}
//	return 0;
}

void EnableGPIOClk(void)
{
	/* AHB1 clocks enable */
	RCC_AHB1PeriphClockCmd(
		RCC_AHB1Periph_GPIOA 	|
		RCC_AHB1Periph_GPIOB 	|
		RCC_AHB1Periph_GPIOC 	|
		RCC_AHB1Periph_GPIOD 	|
		RCC_AHB1Periph_GPIOE 	|
		RCC_AHB1Periph_GPIOH 	|
		RCC_AHB1Periph_CRC	 	|
		RCC_AHB1Periph_FLITF 	|
		RCC_AHB1Periph_SRAM1	|
		RCC_AHB1Periph_SRAM2	|
		RCC_AHB1Periph_BKPSRAM	|
		RCC_AHB1Periph_DMA1		|
		RCC_AHB1Periph_DMA2		|
		0, ENABLE
	);
	
	/* AHB2 clocks enable */
	RCC_AHB2PeriphClockCmd(0, ENABLE);
	
	/* APB1 clocks enable */
	RCC_APB1PeriphClockCmd(
		RCC_APB1Periph_PWR		|
//		RCC_APB1Periph_I2C3		|	
//		RCC_APB1Periph_I2C2		|	
//		RCC_APB1Periph_I2C1		|	
		RCC_APB1Periph_USART2	|	
		RCC_APB1Periph_SPI3		|	
		RCC_APB1Periph_SPI2		|	
		RCC_APB1Periph_WWDG		|	
		RCC_APB1Periph_TIM5		|	
		RCC_APB1Periph_TIM4		|	
		RCC_APB1Periph_TIM3		|	
		RCC_APB1Periph_TIM2		|	
		0, ENABLE
	);
	
	/* APB2 clocks enable */
	RCC_APB2PeriphClockCmd(
		RCC_APB2Periph_ADC1		|
		RCC_APB2Periph_SPI5		|
		RCC_APB2Periph_TIM11	|
		RCC_APB2Periph_TIM10	|
		RCC_APB2Periph_TIM9		|
		RCC_APB2Periph_TIM1		|
		RCC_APB2Periph_SYSCFG	|
		RCC_APB2Periph_SPI4		|
		RCC_APB2Periph_SPI1		|
		RCC_APB2Periph_SDIO		|		
		RCC_APB2Periph_USART6	|		
		RCC_APB2Periph_USART1	|
		0, ENABLE
	);
	
	/* Initialise GPIOA */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_PuPd		= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin			= GPIO_Pin_All;
//	GPIO_InitStructure.GPIO_Pin			&= ~(GPIO_Pin_11 | GPIO_Pin_12);		// leave USB D+/D- alone
	GPIO_InitStructure.GPIO_Pin			&= ~(GPIO_Pin_13 | GPIO_Pin_14);		// leave JTAG pins alone
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Initialise GPIOB */
	GPIO_InitStructure.GPIO_Pin			= GPIO_Pin_All;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Initialise GPIOC */
	GPIO_InitStructure.GPIO_Pin			= GPIO_Pin_All;
	GPIO_Init(GPIOC, &GPIO_InitStructure);			// Board ID, AUDIO MCLK, SCLK, SDIN, OTG_FS_PowerSwitchOn
	GPIO_Init(GPIOD, &GPIO_InitStructure);			// 4 USER LEDS, AUDIO, OTG_FS
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_Init(GPIOH, &GPIO_InitStructure);
}

void systemInit(void)
{
	/* Configure the system clock */
	SetSysClock();

	/* Configure NVIC preempt/priority groups */
	NVIC_PriorityGroupConfig(NVIC_PRIORITY_GROUPING);	// SCB->AIRCR, [10:8] = 101, 2 bits of preemption priority, 2 bits of subpriority (response priority)

	/* Clear the reset flags */
	RCC_ClearFlag();		// RCC->CSR |= RCC_CSR_RMVF;
	
	/* Enable AHB, APB1, APB2 Peripheral clocks and configure GPIOx, where x = A, B, C, D, E, H */
	EnableGPIOClk();
	
	/* Initialise SysTick counter */
	SysTick_Init();
	
	/* Configure SysTick in milliseconds time base */
	SysTick_Config(SystemCoreClock / 1000);
}


//void gpioPA1Init(void)
//{
//	PA1_PIN = IOGetByTag(IO_TAG(GPIO_PA1_PIN));
//	IOInit(PA1_PIN, OWNER_SYSTEM, 0);
//	IOConfigGPIO(PA1_PIN, IOCFG_OUT_PP);
//}

//void gpioPA1Ops(void)
//{
//	IOWrite(PA1_PIN, false);
//	delay(800);
//	IOWrite(PA1_PIN, true);
//	delay(800);	
//}

//void gpioPB8Init(void)
//{
//	PB8_PIN = IOGetByTag(IO_TAG(GPIO_PB8_PIN));
//	IOInit(PB8_PIN, OWNER_SYSTEM, 0);
//	IOConfigGPIO(PB8_PIN, IOCFG_OUT_PP);
//}

//void gpioPB8Ops(void)
//{
//	IOWrite(PB8_PIN, false);
//	delay(400);
//	IOWrite(PB8_PIN, true);
//	delay(400);	
//}

#if 0	// so far so good
	__g_ToDo = __basepriSetMemRetVal(0x10);		// basepri can be set to 0x10 only
	__g_basepri_save = __get_BASEPRI();			// __get_BASEPRI() returns 0x10
	
	__set_BASEPRI(0x00);		// reset basepri to 0x00
	__g_basepri_save2 = __get_BASEPRI();		// __get_BASEPRI() returns 0x00

	__g_ToDo2 = __basepriSetMemRetVal(0x10);		// basepri can be set to 0x10 only
	__g_basepri_save3 = __get_BASEPRI();			// __get_BASEPRI() returns 0x10

	basepri_val = 0x10;
	__basepriRestoreMem(&basepri_val);

	basepri_val = 0x00;
	__basepriRestoreMem(&basepri_val);
#endif

#if 0
	{
//		uint8_t __basepri_save = __get_BASEPRI();
		uint8_t __basepri_save __attribute__ ((__cleanup__(__basepriRestoreMem))) = __get_BASEPRI();
		__g_basepri_save = __basepri_save;
		uint8_t __ToDo = __basepriSetMemRetVal(0x10);
		__g_basepri_save2 = __get_BASEPRI();
		forCnt++;
	}
#endif
	
#if 0
	for ( uint8_t __basepri_save __attribute__ ((__cleanup__(__basepriRestoreMem))) = __get_BASEPRI(), __ToDo = __basepriSetMemRetVal(0x10); __ToDo ; __ToDo = 0 ) {
//	for ( uint8_t __ToDo = __basepriSetMemRetVal(0x10); __ToDo ; __ToDo = 0 ) {
		__g_basepri_save2 = __get_BASEPRI();
		//__set_BASEPRI(0x00);
	}
	//__set_BASEPRI(0x00);
	__g_basepri_save3 = __get_BASEPRI();	// WTF?! __g_basepri_save2 should be 0x00, but it has 0x10??!!
#endif

#if 0
void clean_up(int *final_value)
{
	g_val = *final_value;
}
#endif
	
#if 0
	{
		int avar __attribute__ ((__cleanup__(clean_up))) = 1;
	}
#endif
