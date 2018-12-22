#ifndef __CONFIG_H
#define __CONFIG_H

#include "led.h"

#define MAX_PROFILE_COUNT		3
#define MAX_RATEPROFILES		3

void resetEEPROM(void);
void checkEEPROMContainsValidData(void);
void ResetLedStatusConfig(LedStatusConfig_t *ledStatusConfig);

void validateAndFixConfig(void);
void activateConfig(void);

void beeperOffSet(uint32_t mask);
void beeperOffSetAll(uint8_t beeperCount);
void beeperOffClear(uint32_t mask);
void beeperOffClearAll(void);
uint32_t getBeeperOffMask(void);
void setBeeperOffMask(uint32_t mask);
uint32_t getPreferredBeeperOffMask(void);
void setPreferredBeeperOffMask(uint32_t mask);

void setProfile(uint8_t profileIndex);

struct master_s;			// cheat on compiler, to avoid cross including of configMaster.h, ie config.h inclues configMaster.h and configMaster.h includes config.h
void CreateDefaultConfig(struct master_s *config);

#endif	// __CONFIG_H
