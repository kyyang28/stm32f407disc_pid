
#include <stddef.h>
#include "io.h"
#include "ioImpl.h"
#include "RCCTypes.h"
#include "rcc.h"
//#include "stdio.h"

typedef struct ioPortDef_s {
	RccPeriphTag_t rcc;
}ioPortDef_t;

static const uint16_t ioDefUsedMask[DEFIO_PORT_USED_COUNT] = { DEFIO_PORT_USED_LIST };
static const uint8_t ioDefUsedOffset[DEFIO_PORT_USED_COUNT] = { DEFIO_PORT_OFFSET_LIST };
ioRec_t ioRecs[DEFIO_IO_USED_COUNT];		// DEFIO_IO_USED_COUNT = 0x10 + 0x10 + 0x10 + 0x10 + 0x10 = 0x50 = 80

/* <RM0090-STM32F407-Reference_manual.pdf> p65, Table 1. STM32F4xx register boundary addresses (continued) */
const struct ioPortDef_s ioPortDefs[] = {
	{ RCC_AHB1(GPIOA) },
	{ RCC_AHB1(GPIOB) },
	{ RCC_AHB1(GPIOC) },
	{ RCC_AHB1(GPIOD) },
    { RCC_AHB1(GPIOE) },	// RCC_AHB1ENR_GPIOEEN = ((uint32_t)0x00000010)
    { RCC_AHB1(GPIOF) },
};

/* 
 *	Initialise all ioRec_t structures from ROM
 *	currently only bitmask is used, this may change in the future
 */
void IOGlobalInit(void)
{
	ioRec_t *ioRec = ioRecs;
	
	for (unsigned port = 0; port < ARRAYLEN(ioDefUsedMask); port++) {
		for (unsigned pin = 0; pin < sizeof(ioDefUsedMask[0]) * 8; pin++) {
			if (ioDefUsedMask[port] & (1 << pin)) {
				ioRec->gpio = (GPIO_TypeDef *)(GPIOA_BASE + (port << 10));		// ports are 0x400 apart
				ioRec->pin = 1 << pin;
				ioRec++;
			}
		}
	}
}

ioRec_t * IO_Rec(IO_t io)
{
	return io;
}

void IOInit(IO_t io, resourceOwner_e owner, uint8_t index)
{
	ioRec_t *ioRec = IO_Rec(io);
	ioRec->owner = owner;
	ioRec->index = index;
}

void IORelease(IO_t io)
{
	ioRec_t *ioRec = IO_Rec(io);
	ioRec->owner = OWNER_FREE;
}

GPIO_TypeDef * IO_GPIO(IO_t io)
{
	ioRec_t *ioRec = IO_Rec(io);
	return ioRec->gpio;
}

uint16_t IO_Pin(IO_t io)
{
	ioRec_t *ioRec = IO_Rec(io);
	return ioRec->pin;
}

/* port index, GPIOA = 0 */
int IO_GPIOPortIdx(IO_t io)
{
	if (!io)
		return -1;
	
	return (((size_t)IO_GPIO(io) - GPIOA_BASE) >> 10);		// ports are 0x400 apart
}

int IO_GPIOPinIdx(IO_t io)
{
	if (!io)
		return -1;
	/* __builtin_clz is a little bit faster than __builtin_ffs */
	return 31 - __builtin_clz(IO_Pin(io));
}

int IO_GPIO_PinSource(IO_t io)
{
    return IO_GPIOPinIdx(io);
}

int IO_EXTI_PortSourceGPIO(IO_t io)
{
	return IO_GPIOPortIdx(io);
}

int IO_EXTI_PinSource(IO_t io)
{
	return IO_GPIOPinIdx(io);
}

uint32_t IO_EXTI_Line(IO_t io)
{
	if (!io) {
		return 0;
	}
	
	return 1 << IO_GPIOPinIdx(io);		// STM32F4
}

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
	if (!io)
		return;
	
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	
	/* GPIOD clock is already ininitialised in systemInit() --> EnableGPIOClk()
	 * we don't really need to initialise it again here.
     */
	RccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
//	printf("rcc: 0x%x\r\n", rcc);
	RCC_ClockCmd(rcc, ENABLE);
	
	GPIO_InitTypeDef init = {
		.GPIO_Pin 		= IO_Pin(io),
		.GPIO_Mode      = (cfg >> 0) & 0x03,
		.GPIO_Speed		= (cfg >> 2) & 0x03,
		.GPIO_OType		= (cfg >> 4) & 0x01,
		.GPIO_PuPd		= (cfg >> 5) & 0x03,
	};
	
	GPIO_Init(IO_GPIO(io), &init);
}

void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af)
{
    if (!io)
        return;

    RccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);
    GPIO_PinAFConfig(IO_GPIO(io), IO_GPIO_PinSource(io), af);

    GPIO_InitTypeDef init = {
        .GPIO_Pin = IO_Pin(io),
        .GPIO_Mode = (cfg >> 0) & 0x03,
        .GPIO_Speed = (cfg >> 2) & 0x03,
        .GPIO_OType = (cfg >> 4) & 0x01,
        .GPIO_PuPd = (cfg >> 5) & 0x03,
    };
    GPIO_Init(IO_GPIO(io), &init);
}

IO_t IOGetByTag(ioTag_t tag)
{
	int portIDx = DEFIO_TAG_GPIOID(tag);		// portIDx = 0x3
	int pinIDx = DEFIO_TAG_PIN(tag);			// pinIDx = 0xD(13), 0xC(12), 0xE(14), 0xF(15)
	
	if (portIDx >= DEFIO_PORT_USED_COUNT)
		return NULL;
	
	/* Check if pin exists */
	if (!(ioDefUsedMask[portIDx] & (1 << pinIDx)))
		return NULL;
	
	/* count bits before this pin on single port
	 * (1 << pinIDx) = 0x2000, 0x1000, 0x4000, 0x8000
	 * (1 << pinIDx) - 1 = 0x1FFF, 0x0FFF, 0x3FFF, 0x7FFF
	 * ioDefUsedMask[portIDx] = 0xFFFF
	 * ((1 << pinIDx) - 1) & ioDefUsedMask[portIDx]) = 0x1FFF, 0x0FFF, 0x3FFF, 0x7FFF
	 * __builtin_popcount(0x1FFF) = 13
	 * __builtin_popcount(0x0FFF) = 12
	 * __builtin_popcount(0x3FFF) = 14
	 * __builtin_popcount(0x7FFF) = 15
	 *
	 * offset = 13 or 12 or 14 or 15
	 */
	int offset = __builtin_popcount(((1 << pinIDx) - 1) & ioDefUsedMask[portIDx]);
	
	/* add port offset */
	offset += ioDefUsedOffset[portIDx];		// ioDefUsedOffset[portIDx] = 48 (PORTD), offset = 13+48=61, 12+48=60, 14+48=62, 15+48=63
	
	return ioRecs + offset;		// ioRecs[61], ioRecs[60], ioRecs[62], ioRecs[63]
}

void IOWrite(IO_t io, bool hi)
{
	if (!io)
		return;
	
	IO_GPIO(io)->BSRR = IO_Pin(io) << (hi ? 16 : 0);
}

bool IORead(IO_t io)
{
	if (!io)
		return false;
	
	return !! (IO_GPIO(io)->IDR & IO_Pin(io));
}

void IOHi(IO_t io)
{
	if (!io)
		return;

	IO_GPIO(io)->BSRR = IO_Pin(io);				// Bits 15:0  Port set bit (1)
}

void IOLo(IO_t io)
{
	if (!io)
		return;
	
	IO_GPIO(io)->BSRR = IO_Pin(io) << 16;		// Bits 31:16 Port reset bit (0)
}

void IOToggle(IO_t io)
{
	if (!io)
		return;
	
	uint32_t mask = IO_Pin(io);

    // Read pin state from ODR but write to BSRR because it only changes the pins
    // high in the mask value rather than all pins. XORing ODR directly risks
    // setting other pins incorrectly because it change all pins' state.
	if (IO_GPIO(io)->ODR & mask)
		mask <<= 16;			// bit is set, shift mask to reset half
	
	IO_GPIO(io)->BSRR = mask;
}
