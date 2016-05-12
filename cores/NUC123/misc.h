#ifndef __MISC_H
#define __MISC_H

#include <Arduino.h>

u8 eeprom_read_byte(u8 *addr);
void eeprom_write_byte(u8* addr,u8 value);

#endif
