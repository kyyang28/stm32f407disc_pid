
#include "system_stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"
#include "nvic.h"
#include "system.h"
#include "led.h"
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
//#include "bus_i2c.h"
//#include "bus_spi.h"
//#include "initialisation.h"
//#include "accgyro_spi_mpu9250.h"
//#include "timer.h"
#include "pwm_output.h"			// including timer.h ledTimer.h
#include "rx_pwm.h"
#include "feature.h"

//#include "timerLedsTesting.h"					// Testing timer leds flashing (Done successfully)
//#include "timerUserBtnInputTesting.h"			// Testing timer input capture for user button (PA0) ONLY (Done successfully)

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

//bool sdaFlag = false;

uint32_t TIM_ARR = 0;
int16_t ledDutyCycle[LED_NUMBER];

extern uint8_t TIM_CAPTURE_STATUS;			// status of timer input capture

int main(void)
{
//	long long temp = 0;
	
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

//	gpsInit();			// USART2 initialisation
	
//	rxSerial1TestInit();
	rxSerial3TestInit();
//	rxSerial6TestInit();
	
	/* LedTimerConfig()->ioTags[xxx] initialisation done correctly */
//	printf("LedTimerConfig()->ioTags[0]: 0x%x\r\n", LedTimerConfig()->ioTags[0]);	// LED4, PD12, ioTag_t 0x4C
//	printf("LedTimerConfig()->ioTags[1]: 0x%x\r\n", LedTimerConfig()->ioTags[1]);	// LED3, PD13, ioTag_t 0x4D
//	printf("LedTimerConfig()->ioTags[2]: 0x%x\r\n", LedTimerConfig()->ioTags[2]);	// LED5, PD14, ioTag_t 0x4E
//	printf("LedTimerConfig()->ioTags[3]: 0x%x\r\n", LedTimerConfig()->ioTags[3]);	// LED6, PD15, ioTag_t 0x4F
	
	/* PwmConfig()->ioTags[xxx] initialisation done correctly */
//	printf("PwmConfig()->ioTags[0]: 0x%x\r\n", PwmConfig()->ioTags[0]);
//	printf("PwmConfig()->ioTags[1]: 0x%x\r\n", PwmConfig()->ioTags[1]);
//	printf("PwmConfig()->ioTags[2]: 0x%x\r\n", PwmConfig()->ioTags[2]);
//	printf("PwmConfig()->ioTags[3]: 0x%x\r\n", PwmConfig()->ioTags[3]);
//	printf("PwmConfig()->ioTags[4]: 0x%x\r\n", PwmConfig()->ioTags[4]);
//	printf("PwmConfig()->ioTags[5]: 0x%x\r\n", PwmConfig()->ioTags[5]);
//	printf("PwmConfig()->ioTags[6]: 0x%x\r\n", PwmConfig()->ioTags[6]);
//	printf("PwmConfig()->ioTags[7]: 0x%x\r\n", PwmConfig()->ioTags[7]);
	
	/* Read the contents of EEPROM */
//	ReadEEPROM();
	
	systemState |= SYSTEM_STATE_CONFIG_LOADED;
	
	/* Latch active features to be used for feature() in the remainder of init() */
	latchActiveFeatures();
//	printf("masterConfig.enabledFeatures: 0x%x, %s, %d\r\n", masterConfig.enabledFeatures, __FUNCTION__, __LINE__);		// 0x2000 (1 << 13) FEATURE_RX_PARALLEL_PWM
	
	/* Initialise leds */
	LedInit(LedStatusConfig());			// Normal Leds initialisation, no usage of timer (TIM4), will be overriden by ledTimerInit 
	
//	LedSet(LED3_NUM, true);		// LED3_NUM = 0
//	LedSet(LED4_NUM, true);		// LED4_NUM = 1
//	LedSet(LED5_NUM, true);		// LED5_NUM = 2
//	LedSet(LED6_NUM, true);		// LED6_NUM = 3

//	LED3_ON;
//	LED4_ON;
//	LED5_ON;
//	LED6_ON;

	/* Initialise external interrupt */
	EXTIInit();

//	userBtnPollInit();
	
	/* Initialisae button with interrupt */
//	buttonInit();

#if 1
	/* Timer must be initialised before any channel is allocated */
	timerInit();

//	uint16_t idlePulse = LedTimerConfig()->mincommand;
	uint16_t idlePulse = 0;
	uint32_t KhzGain = 500;
//	printf("idlePulse: %u, %s, %d\r\n", idlePulse, __FUNCTION__, __LINE__);		// idlePulse = 1000 (mincommand)
	ledTimerInit(LedTimerConfig(), idlePulse, LED_NUMBER, KhzGain);
	
	TIM_ARR = 10 * KhzGain;	// KhzGain = 500, TIM_ARR = 10*500 = 5000 (5000 / 10000 = 0.5s = 500ms)
	
//	printf("TIM_ARR: %u\r\n", TIM_ARR);
	
	ledDutyCycle[0] = TIM_ARR * 0.75;		// TIM4 CH1 LED4 PD12
	ledDutyCycle[1] = TIM_ARR * 0.50;		// TIM4 CH2 LED3 PD13
	ledDutyCycle[2] = TIM_ARR * 0.25;		// TIM4 CH3 LED5 PD14
	ledDutyCycle[3] = TIM_ARR * 0.20;		// TIM4 CH4 LED6 PD15
	
	for (int i = 0; i < LED_NUMBER; i++) {
		pwmWriteLed(i, ledDutyCycle[i]);
	}
#endif
	
#if defined(USE_PWM)
	if (feature(FEATURE_RX_PARALLEL_PWM)) {
		pwmRxInit(PwmConfig());
	}
	
//	IO_t pwmIO = IOGetByTag(PwmConfig()->ioTags[0]);					// IOGetByTag() converts ioTag_t to IO_t
//	printf("IO_GPIO(pwmIO): 0x%x\r\n", (uint32_t)IO_GPIO(pwmIO));		// gpio = 0x40020000 (GPIOA)
//	printf("IO_Pin(pwmIO): %u\r\n", IO_Pin(pwmIO));						// pin = 1 = 0x1 (1 << 0)
#endif
	
	/* Testing USER button (PA0) high voltage level time period using TIM5 Input Capture mode */
//	TIM5_CH1_Cap_Init(0xFFFFFFFF, 84 - 1);	// 1MHz counter frequency, period (ARR) = 0xFFFFFFFF, psc = 84 - 1 = 83 (1MHz, 1us)
////	TIM5_CH1_Cap_Init(0x10000, 84 - 1);	// 1MHz counter frequency, period (ARR) = 0x10000, psc = 84 - 1 = 83 (1MHz, 1us), 0x10000 is not a proper value to measure the user button high voltage level
	
	/* +--------- TIMER LED TESTING PURPOSES ONLY ---------+ */
//	board_leds_init();
//	timer_clock_init();
//	timer_pwm_init();
//	timer_start();
////	flash_green_led_forever();
	/* +--------- TIMER LED TESTING PURPOSES ONLY ---------+ */

	/* Initialise MSP (MCU Support Package) serial */
//	mspSerialInit();
	
//	gpsSetPrintfSerialPort();

//#ifdef USE_SPI
//	#ifdef USE_SPI_DEVICE_1
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		//spiInit(SPIDEV_1);		// SPIDEV_3 = 2
//	#endif
//#endif

	//i2cInit(I2CDEV_2);
	
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
	
#if 0
	if (!sensorsAutodetect(GyroConfig())) {
		//failureMode();
		printf("Failed to initialise IMU!\r\n");
		while (1) {
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
	
//	float f = 0.0;
	
	while (1) {
		
		/* Testing USER button (PA0) high voltage level time period using TIM5 Input Capture mode
		 * Make sure to use the following uint32_t type for captures
		 * 	-- uint32_t captures[PWM_PORTS_OR_PPM_CAPTURE_COUNT];
		 *  -- static void pwmEdgeCallback(timerCCHandlerRec_t *cbRec, timCCR_t capture)
		 *  -- static void pwmOverflowCallback(timerOvrHandlerRec_t *cbRec, timCCR_t capture)
		 *  -- 	timCCR_t rise;				// timCCR_t is uint32_t
		 *  -- 	timCCR_t fall;				// timCCR_t is uint32_t
		 *  -- 	timCCR_t capture;				// timCCR_t is uint32_t
		 *  -- 	//	captureCompare_t rise;		// captureCompare_t is uint16_t
		 *  -- 	//	captureCompare_t fall;
		 *  -- 	//	captureCompare_t capture;
		 */
		delay(10);
		if (TIM_CAPTURE_STATUS & 0x80) {			// check if the time period between the rising and falling edge of the signal is captured.
			/* print the time period */
			printf("The period of signal high: %u us\r\n", pwmRead(TIM_Channel_1));		// TIM_Channel_1 = 0x0
			TIM_CAPTURE_STATUS = 0x0;				// clear the status to zero for the next timer input capture
		}
		
		/* Testing USER button (PA0) high voltage level time period using TIM5 Input Capture mode */
//		delay(10);
//		if (TIM5_CH1_CAPTURE_STATUS & 0x80) {		// 0x80: 1 << 7 (capture successful)
//			printf("TIM5_CH1_CAPTURE_VAL: %u\r\n", TIM5_CH1_CAPTURE_VAL);
////			temp = TIM5_CH1_CAPTURE_STATUS & 0x3F;
////			temp *= 0xFFFFFFFF;						// sum of overflow value
//			temp += TIM5_CH1_CAPTURE_VAL;			// retrieve total time period of high voltage level
//			printf("HIGH: %lld us\r\n", temp);
//			TIM5_CH1_CAPTURE_STATUS = 0;			// clear capture status to enable the next input capture
//		}
	/* Testing USER button (PA0) high voltage level time period using TIM5 Input Capture mode */
		
//		printf("float val: %.2f\r\n", f);
//		f += 0.1f;
//		delay(500);
		
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
	__g_basepri_save3 = __get_BASEPRI();	// WTF?! __g_basepri_save2 should be 0x00, but it is 0x10??!!
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
