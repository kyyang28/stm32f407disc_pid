
#include <stdio.h>				// testing purposes
#include <string.h>
#include "configMaster.h"		// including platform.h, which including stm32f4xx_conf.h, which including stm32f4xx_flash.h
#include "utils.h"
#include "config_eeprom.h"
//#include "platform.h"

#if FLASH_SIZE > 128
#define FLASH_TO_RESERVE_FOR_CONFIG				0x1000					// 0x1000 = 4096
#endif

#ifndef FLASH_PAGE_SIZE
#if defined(STM32F40_41xxx)
#define FLASH_PAGE_SIZE							((uint32_t)0x20000)
#endif
#endif

void initEEPROM(void)
{
}

static uint8_t calculateChecksum(const uint8_t *data, uint32_t length)
{
	uint8_t checksum = 0;
	const uint8_t *byteOffset;
	
	for (byteOffset = data; byteOffset < (data + length); byteOffset++)
		checksum ^= *byteOffset;
	
	return checksum;
}

bool isEEPROMContentValid(void)
{
	const master_t *temp = (const master_t *)CONFIG_START_FLASH_ADDRESS;
	uint8_t checksum = 0;
	
	/* Step 1: check version number */
//	printf("EEPROM_CONF_VERSION: %u; temp->version: %u\r\n", EEPROM_CONF_VERSION, temp->version);	// EEPROM_CONF_VERSION = 157; temp->version: 157
	if (EEPROM_CONF_VERSION != temp->version) {
		printf("EEPROM configuration version conflicts!!!\r\n");
		return false;
	}
	
	/* Step 2: check size and magic numbers */
//	printf("temp->size: %u; sizeof(master_t): %u\r\n", temp->size, sizeof(master_t));	// temp->size: 876; sizeof(master_t): 876
//	printf("temp->magic_be: 0x%x; temp->magic_ef: 0x%x\r\n", temp->magic_be, temp->magic_ef);
	if (temp->size != sizeof(master_t) || temp->magic_be != 0xBE || temp->magic_ef != 0xEF) {
		printf("EEPROM configuration size, magic_be and magic_ef conflicts!!!\r\n");
		return false;
	}
	
//	printf("temp->boardIdentifier: %s\r\n", temp->boardIdentifier);				// temp->boardIdentifier = STMDISF4
//	printf("TARGET_BOARD_IDENTIFIER: %s\r\n", TARGET_BOARD_IDENTIFIER);			// TARGET_BOARD_IDENTIFIER = STMDISF4
	
	if (strncasecmp(temp->boardIdentifier, TARGET_BOARD_IDENTIFIER, sizeof(TARGET_BOARD_IDENTIFIER))) {
		printf("EEPROM configuration board identifier conflicts\r\n");
		return false;
	}
	
	/* Step 3: verify integrity of temporary copy */
	checksum = calculateChecksum((const uint8_t *)temp, sizeof(master_t));
//	printf("checksum: %u\r\n", checksum);										// checksum = 
	if (checksum != 0) {
		printf("EEPROM configuration invalid checksum\r\n");
		return false;
	}

//	printf("temp->chk: %u\r\n", temp->chk);													// temp->chk: 0
//	printf("temp->motorConfig.minthrottle: %u\r\n", temp->motorConfig.minthrottle);			// temp->motorConfig.minthrottle = 0
//	printf("temp->motorConfig.maxthrottle: %u\r\n", temp->motorConfig.maxthrottle);			// temp->motorConfig.maxthrottle = 0
//	printf("temp->motorConfig.mincommand: %u\r\n", temp->motorConfig.mincommand);			// temp->motorConfig.mincommand = 0
//	printf("temp->motorConfig.motorPwmProtocol: %u\r\n", temp->motorConfig.motorPwmProtocol);	// temp->motorConfig.motorPwmProtocol = 0
//	printf("temp->motorConfig.motorPwmRate: %u\r\n\r\n", temp->motorConfig.motorPwmRate);	// temp->motorConfig.motorPwmRate = 0
	
	/* looks good, let's roll! */
	return true;
}

/*
 * Sector 0    0x08000000 - 0x08003FFF 16 Kbytes
 * Sector 1    0x08004000 - 0x08007FFF 16 Kbytes
 * Sector 2    0x08008000 - 0x0800BFFF 16 Kbytes
 * Sector 3    0x0800C000 - 0x0800FFFF 16 Kbytes
 * Sector 4    0x08010000 - 0x0801FFFF 64 Kbytes
 * Sector 5    0x08020000 - 0x0803FFFF 128 Kbytes
 * Sector 6    0x08040000 - 0x0805FFFF 128 Kbytes
 * Sector 7    0x08060000 - 0x0807FFFF 128 Kbytes
 * Sector 8    0x08080000 - 0x0809FFFF 128 Kbytes
 * Sector 9    0x080A0000 - 0x080BFFFF 128 Kbytes
 * Sector 10   0x080C0000 - 0x080DFFFF 128 Kbytes
 * Sector 11   0x080E0000 - 0x080FFFFF 128 Kbytes
 */
static uint32_t getFLASHSectorForEEPROM(void)
{
	if (CONFIG_START_FLASH_ADDRESS <= 0x08003FFF)
		return FLASH_Sector_0;
	
	if (CONFIG_START_FLASH_ADDRESS <= 0x08007FFF)
		return FLASH_Sector_1;
	
	if (CONFIG_START_FLASH_ADDRESS <= 0x0800BFFF)
		return FLASH_Sector_2;
	
	if (CONFIG_START_FLASH_ADDRESS <= 0x0800FFFF)
		return FLASH_Sector_3;
	
	if (CONFIG_START_FLASH_ADDRESS <= 0x0801FFFF)
		return FLASH_Sector_4;
	
	if (CONFIG_START_FLASH_ADDRESS <= 0x0803FFFF)
		return FLASH_Sector_5;

	if (CONFIG_START_FLASH_ADDRESS <= 0x0805FFFF)
		return FLASH_Sector_6;

	if (CONFIG_START_FLASH_ADDRESS <= 0x0807FFFF)
		return FLASH_Sector_7;

	if (CONFIG_START_FLASH_ADDRESS <= 0x0809FFFF)
		return FLASH_Sector_8;

	if (CONFIG_START_FLASH_ADDRESS <= 0x080BFFFF)
		return FLASH_Sector_9;

	if (CONFIG_START_FLASH_ADDRESS <= 0x080DFFFF)
		return FLASH_Sector_10;

	if (CONFIG_START_FLASH_ADDRESS <= 0x080FFFFF)
		return FLASH_Sector_11;
	
	while (1) {
//		printf("Wrong FLASH sector for EEPROM!!\r\n");
	}
}

void writeEEPROM(void)
{
	/* Generate the compilation error if the config does not fit in the reserved area of flash */
//	printf("sizeof(master_t): %u\r\n", sizeof(master_t));
//	printf("FLASH_TO_RESERVE_FOR_CONFIG: 0x%x\r\n", FLASH_TO_RESERVE_FOR_CONFIG);	// FLASH_TO_RESERVE_FOR_CONFIG = 4096 (0x1000)
//	BUILD_BUG_ON();
	FLASH_Status status = 0;
	uint32_t wordOffset;
	int8_t attemptsRemaining = 3;
	
//	suspendRxSignal();			// TODO: add suspendRxSignal() later
	
	/* Step 1: prepare checksum/version constants */
	masterConfig.version = EEPROM_CONF_VERSION;
	masterConfig.size = sizeof(master_t);
	masterConfig.magic_be = 0xBE;
	masterConfig.magic_ef = 0xEF;
	masterConfig.chk = 0;				// erase checksum before recalculation
	masterConfig.chk = calculateChecksum((const uint8_t *)&masterConfig, sizeof(master_t));
	
	/* can not call the printf as the debugger serial 3 has not been initialised yet at this moment */
//	printf("masterConfig.version: %u, %s, %d\r\n", masterConfig.version, __FUNCTION__, __LINE__);			// masterConfig.version: 157
//	printf("masterConfig.size: %u, %s, %d\r\n", masterConfig.size, __FUNCTION__, __LINE__);					// masterConfig.size: 876
//	printf("masterConfig.magic_be: 0x%x, %s, %d\r\n", masterConfig.magic_be, __FUNCTION__, __LINE__);		// masterConfig.magic_be: 0xbe
//	printf("masterConfig.magic_ef: 0x%x, %s, %d\r\n", masterConfig.magic_ef, __FUNCTION__, __LINE__);		// masterConfig.magic_ef: 0xef
//	printf("masterConfig.chk: %u, %s, %d\r\n", masterConfig.chk, __FUNCTION__, __LINE__);					// masterConfig.chk: 244

	/* Step 2: write it */
	FLASH_Unlock();
	while (attemptsRemaining--) {
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
		
		for (wordOffset = 0; wordOffset < sizeof(master_t); wordOffset += 4) {
			if (wordOffset % FLASH_PAGE_SIZE == 0) {
				status = FLASH_EraseSector(getFLASHSectorForEEPROM(), VoltageRange_3);
				if (status != FLASH_COMPLETE) {
					break;
				}
			}
			
			status = FLASH_ProgramWord(CONFIG_START_FLASH_ADDRESS + wordOffset, *(uint32_t *)((char *)&masterConfig + wordOffset));
			if (status != FLASH_COMPLETE) {
				break;
			}
		}
		
		if (status == FLASH_COMPLETE) {
			break;
		}
	}
	
	FLASH_Lock();
	
//	printf("status: %d, %s, %d\r\n", status, __FUNCTION__, __LINE__);
	
	/* Step 3: flash write failed, just die now */
	if (status != FLASH_COMPLETE || !isEEPROMContentValid()) {
		printf("FAILURE_FLASH_WRITE_FAILED\r\n");
//		for (;;);
	}
	
//	resumeRxSignal();			// TODO: add resumeRxSignal() later
}

static master_t g_masterConfig;

void readEEPROM(void)
{
#if 0	
	/* Sanity check */
	if (!isEEPROMContentValid()) {
		printf("FLASH EEPROM contains invalid information!!\r\n");
	}else {
//		printf("FLASH EEPROM looks good\r\n");
	}
#endif
	
//	suspendRxSignal();
	
	/* Read flash */
//	printf("g_masterConfig.version: %u, %s, %d\r\n", g_masterConfig.version, __FUNCTION__, __LINE__);			// g_masterConfig.version: 0
//	printf("g_masterConfig.size: %u, %s, %d\r\n", g_masterConfig.size, __FUNCTION__, __LINE__);				// g_masterConfig.size: 0
//	printf("g_masterConfig.magic_be: 0x%x, %s, %d\r\n", g_masterConfig.magic_be, __FUNCTION__, __LINE__);		// g_masterConfig.magic_be: 0
//	printf("g_masterConfig.magic_ef: 0x%x, %s, %d\r\n", g_masterConfig.magic_ef, __FUNCTION__, __LINE__);		// g_masterConfig.magic_ef: 0
//	printf("g_masterConfig.chk: %u, %s, %d\r\n", g_masterConfig.chk, __FUNCTION__, __LINE__);					// g_masterConfig.chk: 0
//	printf("g_masterConfig.motorConfig.minthrottle: %u\r\n", g_masterConfig.motorConfig.minthrottle);			// g_masterConfig.motorConfig.minthrottle = 0
//	printf("g_masterConfig.motorConfig.maxthrottle: %u\r\n", g_masterConfig.motorConfig.maxthrottle);			// g_masterConfig.motorConfig.maxthrottle = 0
//	printf("g_masterConfig.motorConfig.mincommand: %u\r\n", g_masterConfig.motorConfig.mincommand);			// g_masterConfig.motorConfig.mincommand = 0
//	printf("g_masterConfig.motorConfig.motorPwmProtocol: %u\r\n", g_masterConfig.motorConfig.motorPwmProtocol);	// g_masterConfig.motorConfig.motorPwmProtocol = 0
//	printf("g_masterConfig.motorConfig.motorPwmRate: %u\r\n\r\n", g_masterConfig.motorConfig.motorPwmRate);	// g_masterConfig.motorConfig.motorPwmRate = 0

#if 0	
	/* copy contents from CONFIG_START_FLASH_ADDRESS(0x08080000) to g_masterConfig */
	memcpy(&g_masterConfig, (char *)CONFIG_START_FLASH_ADDRESS, sizeof(master_t));
#endif
	
//	printf("g_masterConfig.version: %u, %s, %d\r\n", g_masterConfig.version, __FUNCTION__, __LINE__);			// g_masterConfig.version: 157
//	printf("g_masterConfig.size: %u, %s, %d\r\n", g_masterConfig.size, __FUNCTION__, __LINE__);				// g_masterConfig.size: 876
//	printf("g_masterConfig.magic_be: 0x%x, %s, %d\r\n", g_masterConfig.magic_be, __FUNCTION__, __LINE__);		// g_masterConfig.magic_be: 0xbe
//	printf("g_masterConfig.magic_ef: 0x%x, %s, %d\r\n", g_masterConfig.magic_ef, __FUNCTION__, __LINE__);		// g_masterConfig.magic_ef: 0xef
//	printf("g_masterConfig.chk: %u, %s, %d\r\n", g_masterConfig.chk, __FUNCTION__, __LINE__);					// g_masterConfig.chk: 244
//	printf("g_masterConfig.motorConfig.minthrottle: %u\r\n", g_masterConfig.motorConfig.minthrottle);			// g_masterConfig.motorConfig.minthrottle = 1070
//	printf("g_masterConfig.motorConfig.maxthrottle: %u\r\n", g_masterConfig.motorConfig.maxthrottle);			// g_masterConfig.motorConfig.minthrottle = 2000
//	printf("g_masterConfig.motorConfig.mincommand: %u\r\n", g_masterConfig.motorConfig.mincommand);			// g_masterConfig.motorConfig.minthrottle = 1000
//	printf("g_masterConfig.motorConfig.motorPwmProtocol: %u\r\n", g_masterConfig.motorConfig.motorPwmProtocol);	// g_masterConfig.motorConfig.minthrottle = 0
//	printf("g_masterConfig.motorConfig.motorPwmRate: %u\r\n\r\n", g_masterConfig.motorConfig.motorPwmRate);	// g_masterConfig.motorConfig.minthrottle = 480
		
	/* set profile to the first one */
	setProfile(masterConfig.current_profile_index);
	
	validateAndFixConfig();
	activateConfig();
	
//	resumeRxSignal();
}
