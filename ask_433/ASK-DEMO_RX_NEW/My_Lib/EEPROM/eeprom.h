#ifndef         _EEPROM_H_
#define         _EEPROM_H_

#include "ALL_Includes.h"

#define         KEY_ONE         0X56
#define         KEY_TWO         0XAE

#define         EE_ADDR0        0X4000
#define         EE_ADDR1        0X4004

void EEPROM_Byte_Write(unsigned int address , unsigned char date);
unsigned char EEPROM_Byte_Read(unsigned int addr);
void EEPROM_EREASE();

#endif
