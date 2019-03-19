#include "eeprom.h"
#if 0
unsigned char Unlock_eeprom()
{
  FLASH_DUKR = KEY_ONE;
  FLASH_DUKR = KEY_TWO;
  return Is_Unlock_Ok();
}

unsigned char Is_Unlock_Ok()
{
  if(FLASH_IAPSR->DUL)
    return 1;
  else return 0;
}

void Lock_eeprom()
{
  FLASH_IAPSR_DUL = 0;
}

unsigned char Write_ee_byte(unsigned char addr, unsigned char dat)
{
  
}

unsigned char Read_ee_byte(unsigned char addr)
{
  
}

#endif

/*******************************************************************************
**函数名称：void EEPROM_Byte_Write(unsigned int address , unsigned char date)
**功能描述：向EEPROM中固定地址写入一个字节数据
**入口参数：unsigned int address , unsigned char date
            address  ：要写入数据的存储地址
              date   ：一个字节数据
**输出：无
*******************************************************************************/
void EEPROM_Byte_Write(unsigned int address , unsigned char date)
{
  FLASH_CR1_bit.FIX = 1;              //设定编程时间为标准编程时间
  
  //MASS 密钥，解除EEPROM的保护
  FLASH_DUKR = 0x56;                  //先写0X56
  FLASH_DUKR = 0xAE;                  //后写0XAE
  
 *((unsigned char *)address) = date;  //把数据写入相应的存储地址
 
 while(FLASH_IAPSR_bit.EOP == 1);     //等待编程结束
}

unsigned char EEPROM_Byte_Read(unsigned int addr)
{
  unsigned char ee_dat;
  ee_dat = (*(unsigned char*)addr);      //先读出EEPROM一个字节内容
  return ee_dat;
}

void EEPROM_EREASE()
{
  EEPROM_Byte_Write(EE_ADDR0 , 0X00);
  delay_ms(1);
  EEPROM_Byte_Write(EE_ADDR1 , 0X00);
}