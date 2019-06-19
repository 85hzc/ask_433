#ifndef         _EEPROM_H_
#define         _EEPROM_H_

#include "ALL_Includes.h"

#define         KEY_ONE         0X56
#define         KEY_TWO         0XAE

#define         EE_ADDR0        0X4000
#define         EE_ADDR1        0X4004

#define         FM24C_COMM_W        0xA0        //FM24C的I2C地址,写
#define         FM24C_COMM_R        0xA1        //FM24C的I2C地址,读    
#define         FM24C_STORE_ADDR    0x00        //存储地址
#define         PCODE_NUM           2           //Pcode数目


void EEPROM_Byte_Write(unsigned int address , unsigned char date);
unsigned char EEPROM_Byte_Read(unsigned int addr);
void EEPROM_EREASE();

#endif
