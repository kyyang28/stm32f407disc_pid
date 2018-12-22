#ifndef __TARGET_H
#define __TARGET_H

#include "stm32f4xx_exti.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "bus_i2c.h"
#include "feature.h"

#define TARGET_IO_PORTA			0xFFFF
#define TARGET_IO_PORTB			0xFFFF
#define TARGET_IO_PORTC			0xFFFF
#define TARGET_IO_PORTD			0xFFFF
#define TARGET_IO_PORTE			0xFFFF

#define LED3					PD13		// ORANGE LED, ioTag_t: DEFIO_TAG__PD13 (0x4D)
#define LED4					PD12		// GREEN LED, ioTag_t: DEFIO_TAG__PD12	(0x4C)
#define LED5					PD14		// RED LED, ioTag_t: DEFIO_TAG__PD14	(0x4E)
#define LED6					PD15		// BLUE LED, ioTag_t: DEFIO_TAG__PD15	(0x4F)

#define USE_EXTI
#define BTN_INT_EXTI
#define MPU_INT_EXTI            PC13
//#define BTN_POLL

//#define USE_VCP
#define SERIAL_PORT_COUNT		4

#define USE_UART1
#define UART1_RX_PIN        	PB7
#define UART1_TX_PIN        	PB6

#define USE_UART2
#define UART2_RX_PIN        	PA3
#define UART2_TX_PIN        	PA2

#define USE_UART3
#define UART3_RX_PIN        	PB11		// share with SOFT_I2C_SDA
#define UART3_TX_PIN        	PB10		// share with SPI2 SCK line and SOFT_I2C_SCL

#define USE_UART6
#define UART6_RX_PIN        	PC7
#define UART6_TX_PIN        	PC6

/* IMU related macros */
//#define USE_SPI
//#define USE_SPI_DEVICE_1
//#define USE_SPI_DEVICE_2
//#define USE_SPI_DEVICE_3

// Software I2C driver, using same pins as hardware I2C, with hw i2c module disabled.
// Can be configured for I2C2 pinout (SCL: PB10, SDA: PB11) or I2C1 pinout (SCL: PB6, SDA: PB7)
//#define USE_I2C
//#define SOFT_I2C_SCL			PB6
//#define SOFT_I2C_SDA			PB9
//#define SOFT_I2C_SCL			PB10
//#define SOFT_I2C_SDA			PB11
//#define I2C_DEVICE              (I2CDEV_2) // PB6/SCL, PB7/SDA

#define USE_GYRO_MPU6050

#define SPI1_NSS_PIN			PE4
#define SPI1_SCK_PIN			PA5
#define SPI1_MISO_PIN			PA6
#define SPI1_MOSI_PIN			PA7

#define SPI2_NSS_PIN			PE5
#define SPI2_SCK_PIN			PB13		// 45
#define SPI2_MISO_PIN			PB14		// 46
#define SPI2_MOSI_PIN			PB15		// 47

#define SPI3_NSS_PIN			PA15
#define SPI3_SCK_PIN			PB3			// PC10
#define SPI3_MISO_PIN			PB4			// PC11
#define SPI3_MOSI_PIN			PB5			// PC12

#define USE_GYRO_SPI_MPU9250
#define MPU9250_SPI_INSTANCE	SPI1
#define MPU9250_CS_PIN          SPI1_NSS_PIN			// SPI2 CS	(PE4)

#define USE_PWM
#define USE_DSHOT

#define USABLE_TIMER_CHANNEL_COUNT 7
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) )		// BITCOUNT(USED_TIMERS) = 6

//#define DEFAULT_RX_FEATURE		FEATURE_RX_SERIAL		// defined in feature.h

#endif	// __TARGET_H
