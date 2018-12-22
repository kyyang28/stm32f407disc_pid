
#include "bus_spi.h"
#include "rcc.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"

/* Testing purposes */
#include "stdio.h"
#include "utils.h"
#include "ioImpl.h"

#ifdef USE_SPI
static spiDevice_t spiHardwareMap[] = {
	// RCC_APB1ENR_SPI2EN
	{ .dev = SPI1, .nss = (uint8_t)IO_NONE, .sck = IO_TAG(SPI1_SCK_PIN), .miso = IO_TAG(SPI1_MISO_PIN), .mosi = IO_TAG(SPI1_MOSI_PIN), .rcc = RCC_APB2(SPI1), .af = GPIO_AF_SPI1, false },
    { .dev = SPI2, .nss = (uint8_t)IO_NONE, .sck = IO_TAG(SPI2_SCK_PIN), .miso = IO_TAG(SPI2_MISO_PIN), .mosi = IO_TAG(SPI2_MOSI_PIN), .rcc = RCC_APB1(SPI2), .af = GPIO_AF_SPI2, false },
//    { .dev = SPI3, .nss = IO_NONE, .sck = IO_TAG(SPI3_SCK_PIN), .miso = IO_TAG(SPI3_MISO_PIN), .mosi = IO_TAG(SPI3_MOSI_PIN), .rcc = RCC_APB1(SPI3), .af = GPIO_AF_SPI3, false }
//	{ .dev = SPI1, .nss = IO_TAG(SPI1_NSS_PIN), .sck = IO_TAG(SPI1_SCK_PIN), .miso = IO_TAG(SPI1_MISO_PIN), .mosi = IO_TAG(SPI1_MOSI_PIN), .rcc = RCC_APB2(SPI1), .af = GPIO_AF_SPI1, false },
//    { .dev = SPI2, .nss = IO_TAG(SPI2_NSS_PIN), .sck = IO_TAG(SPI2_SCK_PIN), .miso = IO_TAG(SPI2_MISO_PIN), .mosi = IO_TAG(SPI2_MOSI_PIN), .rcc = RCC_APB1(SPI2), .af = GPIO_AF_SPI2, false },
//    { .dev = SPI3, .nss = IO_TAG(SPI3_NSS_PIN), .sck = IO_TAG(SPI3_SCK_PIN), .miso = IO_TAG(SPI3_MISO_PIN), .mosi = IO_TAG(SPI3_MOSI_PIN), .rcc = RCC_APB1(SPI3), .af = GPIO_AF_SPI3, false }
};

SPIDevice spiDeviceByInstance(SPI_TypeDef *instance)
{
	if (instance == SPI1)
		return SPIDEV_1;
	
	if (instance == SPI2)
		return SPIDEV_2;
	
	if (instance == SPI3)
		return SPIDEV_3;
	
	return SPIINVALID;
}

void spiInitDevice(SPIDevice device)
{
	spiDevice_t *spi = &(spiHardwareMap[device]);

#if 0
	printf("spi->dev: 0x%x\r\n", (uint32_t)spi->dev);		// spi-dev = 0x40013000 (SPI1)
	printf("spi->nss: %u\r\n", spi->nss);					// spi->nss = 0
	printf("spi->sck: %u\r\n", spi->sck);					// spi->sck = 21 (DECIMAL)
	printf("spi->miso: %u\r\n", spi->miso);					// spi->miso = 22 (DECIMAL)
	printf("spi->mosi: %u\r\n", spi->mosi);					// spi->mosi = 23 (DECIMAL)
	printf("RCC_APB2 << 5: 0x%x\r\n", (uint32_t)(2 << 5));
	printf("LOG2_32BIT(mask): 0x%x\r\n", (uint32_t)LOG2_32BIT(0x4000));
	printf("spi->rcc: 0x%x\r\n", spi->rcc);					// spi->rcc = 0x4E
	printf("spi->af: %u\r\n", spi->af);						// spi->af = 0x05
#endif
	
	/* Enable SPI clock */
	RCC_ClockCmd(spi->rcc, ENABLE);		// spi->rcc = 0x4e for SPI2
	RCC_ResetCmd(spi->rcc, ENABLE);		// spi->rcc = 0x4e for SPI2

#if 0
	ioRec_t *ioRecSpiSck = IO_Rec(IOGetByTag(spi->sck));
	printf("ioRecSpiSck->gpio: 0x%x\r\n", (uint32_t)ioRecSpiSck->gpio);	// gpio = 0x40020000 (GPIOA)
	printf("ioRecSpiSck->pin: %u\r\n", ioRecSpiSck->pin);				// pin = 32 = 0x20 (1 << 5)

	ioRec_t *ioRecSpiMiso = IO_Rec(IOGetByTag(spi->miso));
	printf("ioRecSpiMiso->gpio: 0x%x\r\n", (uint32_t)ioRecSpiMiso->gpio);	// gpio = 0x40020000 (GPIOA)
	printf("ioRecSpiMiso->pin: %u\r\n", ioRecSpiMiso->pin);					// pin = 64 = 0x40 (1 << 6)

	ioRec_t *ioRecSpiMosi = IO_Rec(IOGetByTag(spi->mosi));
	printf("ioRecSpiMosi->gpio: 0x%x\r\n", (uint32_t)ioRecSpiMosi->gpio);	// gpio = 0x40020000 (GPIOA)
	printf("ioRecSpiMosi->pin: %u\r\n", ioRecSpiMosi->pin);					// pin = 128 = 0x80 (1 << 7)
#endif

	IOInit(IOGetByTag(spi->sck), OWNER_SPI_SCK, RESOURCE_INDEX(device));
	IOInit(IOGetByTag(spi->miso), OWNER_SPI_MISO, RESOURCE_INDEX(device));
	IOInit(IOGetByTag(spi->mosi), OWNER_SPI_MOSI, RESOURCE_INDEX(device));

#if 0
	printf("ioRecSpiSck->owner: %u\r\n", ioRecSpiSck->owner);				// owner = 8 (OWNER_SPI_SCK)
	printf("ioRecSpiSck->index: %u\r\n", ioRecSpiSck->index);				// index = 1
	printf("ioRecSpiMiso->owner: %u\r\n", ioRecSpiMiso->owner);				// owner = 9 (OWNER_SPI_MISO)
	printf("ioRecSpiMiso->index: %u\r\n", ioRecSpiMiso->index);				// index = 1
	printf("ioRecSpiMosi->owner: %u\r\n", ioRecSpiMosi->owner);				// owner = 10 (OWNER_SPI_MOSI)
	printf("ioRecSpiMosi->index: %u\r\n", ioRecSpiMosi->index);				// index = 1

	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
#endif

//#if defined(STM32F4)
	IOConfigGPIOAF(IOGetByTag(spi->sck), SPI_IO_AF_CFG, spi->af);
	IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_MISO_CFG, spi->af);
	IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_MOSI_CFG, spi->af);
//	IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_CFG, spi->af);
//	IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->af);

//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);

	if (spi->nss) {
		IOInit(IOGetByTag(spi->nss), OWNER_SPI_CS, RESOURCE_INDEX(device));
		IOConfigGPIOAF(IOGetByTag(spi->nss), SPI_IO_CS_CFG, spi->af);
		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	}
//#endif
	
	/* Initialise SPI hardware */
	SPI_I2S_DeInit(spi->dev);
	
	SPI_InitTypeDef spiInit;
	spiInit.SPI_Mode = SPI_Mode_Master;
	spiInit.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spiInit.SPI_DataSize = SPI_DataSize_8b;
	spiInit.SPI_NSS = SPI_NSS_Soft;
	spiInit.SPI_FirstBit = SPI_FirstBit_MSB;
	spiInit.SPI_CRCPolynomial = 7;
	spiInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	
//	printf("spiInit.SPI_Mode: %u\r\n", spiInit.SPI_Mode);
//	printf("spiInit.SPI_Direction: %u\r\n", spiInit.SPI_Direction);
//	printf("spiInit.SPI_DataSize: %u\r\n", spiInit.SPI_DataSize);
//	printf("spiInit.SPI_NSS: %u\r\n", spiInit.SPI_NSS);
//	printf("spiInit.SPI_FirstBit: %u\r\n", spiInit.SPI_FirstBit);
//	printf("spiInit.SPI_CRCPolynomial: %u\r\n", spiInit.SPI_CRCPolynomial);
//	printf("spiInit.SPI_BaudRatePrescaler: %u\r\n", spiInit.SPI_BaudRatePrescaler);
	
	if (spi->leadingEdge) {
		spiInit.SPI_CPOL = SPI_CPOL_Low;
		spiInit.SPI_CPHA = SPI_CPHA_1Edge;
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	}else {
		spiInit.SPI_CPOL = SPI_CPOL_High;
		spiInit.SPI_CPHA = SPI_CPHA_2Edge;
//		printf("spiInit.SPI_CPOL = SPI_CPOL_High, spiInit.SPI_CPHA = SPI_CPHA_2Edge, %s, %d\r\n", __FUNCTION__, __LINE__);
	}
	
	SPI_Init(spi->dev, &spiInit);
	SPI_Cmd(spi->dev, ENABLE);
	
	if (spi->nss) {
		/* Drive NSS high to disable connected SPI device */
//		ioRec_t *ioRecSpiNss = IO_Rec(IOGetByTag(spi->nss));
//		printf("ioRecSpiNss->gpio: 0x%x\r\n", (uint32_t)ioRecSpiNss->gpio);
//		printf("ioRecSpiNss->pin: %u\r\n", ioRecSpiNss->pin);
//		printf("ioRecSpiNss->owner: %u\r\n", ioRecSpiNss->owner);
//		printf("ioRecSpiNss->index: %u\r\n", ioRecSpiNss->index);
		IOHi(IOGetByTag(spi->nss));
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	}
}

bool spiInit(SPIDevice device)
{
	switch (device) {
		case SPIINVALID:
			return false;
		case SPIDEV_1:
#ifdef USE_SPI_DEVICE_1
//			printf("%s, %d\r\n", __FUNCTION__, __LINE__);
			spiInitDevice(device);
			return true;
#else
			break;
#endif
		case SPIDEV_2:
#ifdef USE_SPI_DEVICE_2
//			printf("%s, %d\r\n", __FUNCTION__, __LINE__);
			spiInitDevice(device);
			return true;
#else
			break;
#endif
		case SPIDEV_3:
#ifdef USE_SPI_DEVICE_3
//			printf("%s, %d\r\n", __FUNCTION__, __LINE__);
			spiInitDevice(device);
			return true;
#else
			break;
#endif
		case SPIDEV_4:
#ifdef USE_SPI_DEVICE_4
//			spiInitDevice(device);
			return true;
#else
			break;
#endif
	}
	
	return false;
}

void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor)
{
#define BR_CLEAR_MASK	0xFFC7	// 1111 1111 1100 0111 (BaudRatePrescaler set to fPCLK / 2)

	uint16_t tempRegister;
	
	/* Disable SPI */
	SPI_Cmd(instance, DISABLE);
	
//	printf("instance: 0x%x\r\n", (uint32_t)instance);					// SPI2: 0x40003800 
//	printf("instance->CR1: 0x%x\r\n", (uint32_t)&(instance->CR1));		// SPI2->CR1: 0x40003800
	tempRegister = instance->CR1;
//	printf("tempRegister: 0x%x\r\n", tempRegister);						// 0x317 (SPI disable)
	
	switch (divisor) {
		case 2:
			tempRegister &= BR_CLEAR_MASK;
			tempRegister |= SPI_BaudRatePrescaler_2;
			break;

		case 4:
			tempRegister &= BR_CLEAR_MASK;
			tempRegister |= SPI_BaudRatePrescaler_4;
			break;

		case 8:
			tempRegister &= BR_CLEAR_MASK;
			tempRegister |= SPI_BaudRatePrescaler_8;
			break;

		case 16:
			tempRegister &= BR_CLEAR_MASK;
			tempRegister |= SPI_BaudRatePrescaler_16;
			break;

		case 32:
			tempRegister &= BR_CLEAR_MASK;
			tempRegister |= SPI_BaudRatePrescaler_32;
			break;

		case 64:
			tempRegister &= BR_CLEAR_MASK;
			tempRegister |= SPI_BaudRatePrescaler_64;
			break;

		case 128:
			tempRegister &= BR_CLEAR_MASK;
			tempRegister |= SPI_BaudRatePrescaler_128;
			break;

		case 256:
			tempRegister &= BR_CLEAR_MASK;
			tempRegister |= SPI_BaudRatePrescaler_256;
			break;
	}
	
	instance->CR1 = tempRegister;
	
	SPI_Cmd(instance, ENABLE);
//	printf("instance->CR1: 0x%x\r\n", (uint32_t)instance->CR1);		// Content of SPI2->CR1: 0x37F (BaudRatePrescaler set to 256)
}

uint32_t spiTimeoutUserCallback(SPI_TypeDef *instance)
{
	SPIDevice device = spiDeviceByInstance(instance);
	if (device == SPIINVALID)
		return -1;
	spiHardwareMap[device].errorCount++;
	return spiHardwareMap[device].errorCount;
}

/**
 * Return true if the bus is currently in the middle of a transmission
 */
bool spiIsBusBusy(SPI_TypeDef *instance)
{
	return SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET || SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_BSY) == SET;
}

uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t data)
{
	uint16_t spiTimeout = 1000;
	
//	printf("instance: 0x%x, %s, %d\r\n", (uint32_t)instance, __FUNCTION__, __LINE__);
	
	/* RESET indicates that the Tx buffer is not empty, waiting for sending the data out */
	while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET) {
		/* Tx buffer not empty (RESET) */
		if ((spiTimeout--) == 0)
			return spiTimeoutUserCallback(instance);
	}
	
//	printf("instance: 0x%x, %s, %d\r\n", (uint32_t)instance, __FUNCTION__, __LINE__);
//	printf("data: 0x%x\r\n", data);
	SPI_I2S_SendData(instance, data);

	spiTimeout = 1000;
	/* RESET indicates that the Rx buffer is empty, not ready to read */
	while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET) {
		if ((spiTimeout--) == 0)
			return spiTimeoutUserCallback(instance);
	}
	
	return ((uint8_t)SPI_I2S_ReceiveData(instance));		// Clearing the RXNE bit is performed by reading the SPI_DR register
}

/* <RM0090-STM32F407-Reference_manual.pdf> figure 253 */
bool spiTransfer(SPI_TypeDef *instance, uint8_t *out, const uint8_t *in, int len)
{
	uint16_t spiTimeout = 1000;
	uint8_t b;
	
	instance->DR;			// Clearing the RXNE bit is performed by reading the SPI_DR register
	while (len--) {
		b = in ? *(in++) : 0xFF;
		
		/* TXE flag should be '1'(SET) before any attempt to write the Tx buffer is made
		 *
		 * When TXE flag is set, this flag indicates that the Tx buffer is empty and the next data to be
		 * transmitted can be loaded into the buffer. The TXE flag is cleared when writing to the
		 * SPI_DR register.
		 */
		while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET) {
			if ((spiTimeout--) == 0) {
				printf("TX timeout, %s, %d\r\n", __FUNCTION__, __LINE__);
				return spiTimeoutUserCallback(instance);
			}
		}
		
		SPI_I2S_SendData(instance, b);		// Clearing the TXE bit is performed by writing to the SPI_DR register
		
		spiTimeout = 1000;
		
		/*
		 * When set, RXNE flag indicates that there are valid received data in the Rx buffer. It is cleared
		 * when SPI_DR is read.
		 */
		while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET) {
			if ((spiTimeout--) == 0) {
				printf("RX timeout, %s, %d\r\n", __FUNCTION__, __LINE__);
				return spiTimeoutUserCallback(instance);
			}
		}
		
		b = SPI_I2S_ReceiveData(instance);
		
		if (out)
			*(out++) = b;
	}
	
	return true;
}

uint16_t spiGetErrorCounter(SPI_TypeDef *instance)
{
	SPIDevice device = spiDeviceByInstance(instance);
	if (device == SPIINVALID)
		return 0;
	
	return spiHardwareMap[device].errorCount;
}

void spiResetErrorCounter(SPI_TypeDef *instance)
{
	SPIDevice device = spiDeviceByInstance(instance);
	if (device != SPIINVALID) {
		spiHardwareMap[device].errorCount = 0;
	}
}
#endif
