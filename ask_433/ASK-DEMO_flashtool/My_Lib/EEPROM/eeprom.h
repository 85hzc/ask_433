#ifndef         _EEPROM_H_
#define         _EEPROM_H_

#include "ALL_Includes.h"

#define         KEY_ONE         0X56
#define         KEY_TWO         0XAE

#define         EE_ADDR0            0X4000
#define         EE_ADDR1            0X4002
#define         EE_Duty             0X4004
#define         EE_ADDR_InsNum      0X4008

#define         EE_ADDR_DevType     0X4010
#define         EE_ADDR_DevAddr     0X4012
#define         EE_ADDR_KeyVal      0X4014

#define         FM24C_COMM_W        0xA0        //FM24C的I2C地址,写
#define         FM24C_COMM_R        0xA1        //FM24C的I2C地址,读
#define         FM24C_STORE_ADDR    0x00        //存储地址
#define         FM24C_PAGE_SIZE     8

#define         MAX_DEV_NUM         8          //遥控支持的最大设备数目

#define         CARDTYPE_OFFSET     0
#define         IDCODE_OFFSET       1
#define         INSTNUM_OFFSET      3
#define         DEV_TYPE_OFFSET     (0x10)

void EEPROM_Byte_Write(unsigned int address , unsigned char date);
unsigned char EEPROM_Byte_Read(unsigned int addr);
void EEPROM_EREASE();

typedef struct {
  unsigned char keyType;
  unsigned char scene;
}FM24C_Key_S;

typedef struct {
  unsigned char devType;
  unsigned char devAddr;
  //unsigned char Reserved[6];
  FM24C_Key_S   key[8];
} FM24C_DevData_S;

typedef struct {
  unsigned char cardType;
  unsigned char instNum;
  unsigned char assoAddr[2];
  //unsigned char Reserved[4];
  FM24C_DevData_S dev;
} FM24C_Data_S;

#endif
