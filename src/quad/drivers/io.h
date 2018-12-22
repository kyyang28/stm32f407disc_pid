#ifndef __IO_H
#define __IO_H

#include <stdbool.h>
#include <stdint.h>
#include "iodef.h"
#include "ioImpl.h"
#include "resource.h"
#include "stm32f4xx_gpio.h"
#include "IOTypes.h"

typedef uint8_t ioTag_t;	/* package tag to specify IO pin */
typedef void *IO_t;			/* type specifying IO pin. Currently ioRec_t pointer */

// both ioTag_t and IO_t are guarantied to be zero if pinid is NONE (no pin)
// this simplifies initialization (globals are zeroed on start) and allows
//  omitting unused fields in structure initializers.
// it is also possible to use IO_t and ioTag_t as boolean value
//   TODO - this may conflict with requirement to generate warning/error on IO_t - ioTag_t assignment
//   IO_t being pointer is only possibility I know of ..

// pin config handling
// pin config is packed into ioConfig_t to decrease memory requirements
// IOCFG_x macros are defined for common combinations for all CPUs; this
//  helps masking CPU differences
typedef uint8_t ioConfig_t;		// packed IO configuration

/* 
 *	preprocessor is used to convert pinid to requested C data value
 * 	compile-time error is generated if requested pin is not available (not set in TARGET_IO_PORTx)
 * 	ioTag_t and IO_t is supported, but ioTag_t is preferred
 */

/* Expand pinID to ioTag_t */
#define IO_TAG(pinID)		DEFIO_TAG(pinID)

#define IO_CONFIG(mode, speed, otype, pupd)			((mode) | ((speed) << 2) | ((otype) << 4) | ((pupd) << 5))
#define IOCFG_OUT_PP								IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define IOCFG_OUT_PP_UP								IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_PP, GPIO_PuPd_UP)
#define IOCFG_OUT_OD								IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL)
#define IOCFG_OUT_OD_UP								IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_OD, GPIO_PuPd_UP)
#define IOCFG_IPD									IO_CONFIG(GPIO_Mode_IN, 0, 0, GPIO_PuPd_DOWN)
#define IOCFG_IPU									IO_CONFIG(GPIO_Mode_IN, 0, 0, GPIO_PuPd_UP)
#define IOCFG_AF_PP          						IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define IOCFG_AF_PP_PD       						IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_PP, GPIO_PuPd_DOWN)
#define IOCFG_AF_PP_UP       						IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_PP, GPIO_PuPd_UP)
#define IOCFG_AF_OD          						IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_OD, GPIO_PuPd_NOPULL)
//#define IOCFG_OUT_PP								IO_CONFIG(GPIO_Mode_OUT, GPIO_Fast_Speed, GPIO_OType_PP, GPIO_PuPd_UP)
#define IOCFG_IN_FLOATING							IO_CONFIG(GPIO_Mode_IN,  0, 0, GPIO_PuPd_NOPULL)

void IOGlobalInit(void);
IO_t IOGetByTag(ioTag_t tag);
void IOInit(IO_t io, resourceOwner_e owner, uint8_t index);
void IOConfigGPIO(IO_t io, ioConfig_t cfg);
void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af);
int IO_GPIOPortIdx(IO_t io);
int IO_GPIOPinIdx(IO_t io);
int IO_EXTI_PortSourceGPIO(IO_t io);
int IO_EXTI_PinSource(IO_t io);
uint32_t IO_EXTI_Line(IO_t io);
void IOWrite(IO_t io, bool hi);
bool IORead(IO_t io);
void IOHi(IO_t io);
void IOLo(IO_t io);
void IOToggle(IO_t io);
ioRec_t * IO_Rec(IO_t io);
uint16_t IO_Pin(IO_t io);
GPIO_TypeDef * IO_GPIO(IO_t io);

#endif	// __IO_H
