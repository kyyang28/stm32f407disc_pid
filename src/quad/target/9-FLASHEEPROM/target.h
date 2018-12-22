#ifndef __TARGET_H
#define __TARGET_H

#include "stm32f4xx_exti.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "bus_i2c.h"
#include "feature.h"

#define TARGET_BOARD_IDENTIFIER "STMDISF4" 				// STM32 Discovery F4 board for FLASH EEPROM storage

#define TARGET_IO_PORTA			0xFFFF
#define TARGET_IO_PORTB			0xFFFF
#define TARGET_IO_PORTC			0xFFFF
#define TARGET_IO_PORTD			0xFFFF
#define TARGET_IO_PORTE			0xFFFF

#define LED3					PD13
#define LED4					PD12
#define LED5					PD14
#define LED6					PD15

//#define BEEPER					PA1		// UNCOMMENT this line to use BEEPER
//#define BEEPER					PE2		// UNCOMMENT this line to use BEEPER

#define USE_EXTI
#define BTN_INT_EXTI
#define MPU_INT_EXTI            PA4
#define USE_MPU_DATA_READY_SIGNAL
//#define DEBUG_MPU_DATA_READY_INTERRUPT
//#define BTN_POLL

//#define USE_VCP
#define SERIAL_PORT_COUNT		3	// UART1, UART3, UART6
//#define SERIAL_PORT_COUNT		4	// UART1, UART2, UART3, UART6

#define USE_UART1
#ifdef USE_UART1
#define UART1_RX_PIN        	PB7		// RX line for SBUS data of FrSKY X4R-SB receiver
#define UART1_TX_PIN        	PB6
#endif

//#define USE_UART2				// uncomment this line to use UART2
#ifdef USE_UART2
#define UART2_RX_PIN        	PA3			// NOTE: PA3 is for TIM2, Channel 4
#define UART2_TX_PIN        	PA2			// NOTE: PA2 is for TIM2, Channel 3
#endif

#define USE_UART3
#ifdef USE_UART3
#define UART3_RX_PIN        	PB11		// share with SOFT_I2C_SDA
#define UART3_TX_PIN        	PB10		// share with SPI2 SCK line and SOFT_I2C_SCL
#endif

#define USE_UART6
#ifdef USE_UART6
#define UART6_RX_PIN        	PC7
#define UART6_TX_PIN        	PC6
#endif

/* IMU related macros */
//#define USE_IMU								// IMPORTANT: uncomment this line to compile IMU related codes

/* +-----------------------------------------------------------------------------------+ */
/* +--------------------------------------- I2C ---------------------------------------+ */
/* +-----------------------------------------------------------------------------------+ */
// Software I2C driver, using same pins as hardware I2C, with hw i2c module disabled.
// Can be configured for I2C2 pinout (SCL: PB10, SDA: PB11) or I2C1 pinout (SCL: PB8, SDA: PB9)

//#define USE_I2C							// uncomment this line to use I2C interface

#ifdef USE_I2C
//#define SOFT_I2C_SCL			PB6
#define SOFT_I2C_SCL			PB8
#define SOFT_I2C_SDA			PB9
//#define SOFT_I2C_SCL			PB10
//#define SOFT_I2C_SDA			PB11
#define I2C_DEVICE              (I2CDEV_2) // PB6/SCL, PB7/SDA

//#define USE_GYRO_MPU6050			// uncomment this line when uses the MPU6050 GYRO with I2C interface
//#define USE_ACC_MPU6050				// uncomment this line when uses the MPU6050 ACC with I2C interface

#define USE_GYRO_I2C_MPU9250		// uncomment this line when uses MPU9250 with I2C interfacce
#define USE_ACC_I2C_MPU9250			// uncomment this line when uses the MPU9250 ACC with I2C interface
#endif
/* +-----------------------------------------------------------------------------------+ */
/* +--------------------------------------- I2C ---------------------------------------+ */
/* +-----------------------------------------------------------------------------------+ */

/* +-----------------------------------------------------------------------------------+ */
/* +--------------------------------------- SPI ---------------------------------------+ */
/* +-----------------------------------------------------------------------------------+ */
#define USE_SPI						// uncomment this line to use SPI interface

#ifdef USE_SPI
#define USE_SPI_DEVICE_1			// SPI1 for MPU9250 MEMS sensor
//#define USE_SPI_DEVICE_2			// SPI2 for SDCARD interface
//#define USE_SPI_DEVICE_3

/* USED BY onboard accelerometer sensor LIS302DL. Now we use it for MPU9250 IMU */
//#define SPI1_NSS_PIN			PE4
#define SPI1_SCK_PIN			PA5			// SCL/SCLK on MPU9250
#define SPI1_MISO_PIN			PA6			// ADO/SDO on MPU9250
#define SPI1_MOSI_PIN			PA7			// SDA/SDI on MPU9250

/* SHOULD be fine */
//#define SPI2_NSS_PIN			PE5
//#define SPI2_SCK_PIN			PB13		// 45
//#define SPI2_MISO_PIN			PB14		// 46
//#define SPI2_MOSI_PIN			PB15		// 47

/* PB3 is shared with T_SWO (USB ST LINK), DO NOT reconfigure PB3, otherwise the usb upload function is not working any more */
//#define SPI3_NSS_PIN			PA15
//#define SPI3_SCK_PIN			PB3			// PC10
//#define SPI3_MISO_PIN			PB4			// PC11
//#define SPI3_MOSI_PIN			PB5			// PC12

#define USE_GYRO_SPI_MPU9250
#define MPU9250_SPI_INSTANCE	SPI1
#define MPU9250_CS_PIN          PC4			// NCS on MPU9250
#endif
/* +-----------------------------------------------------------------------------------+ */
/* +--------------------------------------- SPI ---------------------------------------+ */
/* +-----------------------------------------------------------------------------------+ */

//#define USE_DSHOT							// uncomment this line to use TIMER MOTORS DMA protocol

/* +-------------------------------------------------------------------------------------+ */
/* +--------------------------------------- TIMER ---------------------------------------+ */
/* +-------------------------------------------------------------------------------------+ */
/* 14 channels: 4 for timer leds(output compare mode), 6 for timer RC PWM inputs(input capture mode), 4 for timer PWM motor outputs(PWM generator mode) */
#define USABLE_TIMER_CHANNEL_COUNT 			14
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8) | TIM_N(10) )		// BITCOUNT(USED_TIMERS) = 6
//#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) )		// BITCOUNT(USED_TIMERS) = 6

#define USE_LEDTIMER			// uncomment this line to use on-board timer LEDS (LED3, 4, 5, 6)
/* +-------------------------------------------------------------------------------------+ */
/* +--------------------------------------- TIMER ---------------------------------------+ */
/* +-------------------------------------------------------------------------------------+ */


/* +-------------------------------------------------------------------------------------+ */
/* +--------------------------------------- Radio ---------------------------------------+ */
/* +-------------------------------------------------------------------------------------+ */
/*
 * Radio transmitter and receiver protocols (i.e. PWM, SBUS or CRSF)
 */

//#define USE_PWM					// IMPORTANT: uncomment this line to use TIMER PWM for Frsky PWM receiver

/* uncomment both the following two lines to ENABLE sbus or crsf protocols for radio receiver */
#define DEFAULT_RX_FEATURE		FEATURE_RX_SERIAL
#define SERIAL_RX

#ifdef SERIAL_RX
#define USE_SERIALRX_SBUS
#endif

#ifndef DEFAULT_FEATURES
#define DEFAULT_FEATURES		0			// could be members of features_e
#endif

#ifndef DEFAULT_RX_FEATURE
#define DEFAULT_RX_FEATURE		FEATURE_RX_PARALLEL_PWM			// define DEFAULT_RX_FEATURE to PWM protocol for radio receiver
#endif

/* +-------------------------------------------------------------------------------------+ */
/* +--------------------------------------- Radio ---------------------------------------+ */
/* +-------------------------------------------------------------------------------------+ */

#define FLASH_SIZE						1024			// TODO: FLASH_SIZE will be defined in Makefile later
#define CONFIG_START_FLASH_ADDRESS 		(0x08080000) 	// 0x08080000 to 0x0809FFFF (0x080A0000) (FLASH_Sector_8)

#endif	// __TARGET_H
