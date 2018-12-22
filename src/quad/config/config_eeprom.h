#ifndef __CONFIG_EEPROM_H
#define __CONFIG_EEPROM_H

#include <stdbool.h>

#define EEPROM_CONF_VERSION				157

void initEEPROM(void);
void writeEEPROM(void);
void readEEPROM(void);
bool isEEPROMContentValid(void);

#endif	// __CONFIG_EEPROM_H
