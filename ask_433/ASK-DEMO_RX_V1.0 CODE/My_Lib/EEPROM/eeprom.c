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
**�������ƣ�void EEPROM_Byte_Write(unsigned int address , unsigned char date)
**������������EEPROM�й̶���ַд��һ���ֽ�����
**��ڲ�����unsigned int address , unsigned char date
            address  ��Ҫд�����ݵĴ洢��ַ
              date   ��һ���ֽ�����
**�������
*******************************************************************************/
void EEPROM_Byte_Write(unsigned int address , unsigned char date)
{
  FLASH_CR1_bit.FIX = 1;              //�趨���ʱ��Ϊ��׼���ʱ��
  
  //MASS ��Կ�����EEPROM�ı���
  FLASH_DUKR = 0x56;                  //��д0X56
  FLASH_DUKR = 0xAE;                  //��д0XAE
  
 *((unsigned char *)address) = date;  //������д����Ӧ�Ĵ洢��ַ
 
 while(FLASH_IAPSR_bit.EOP == 1);     //�ȴ���̽���
}

unsigned char EEPROM_Byte_Read(unsigned int addr)
{
  unsigned char ee_dat;
  ee_dat = (*(unsigned char*)addr);      //�ȶ���EEPROMһ���ֽ�����
  return ee_dat;
}

void EEPROM_EREASE()
{
  EEPROM_Byte_Write(EE_ADDR0 , 0X00);
  delay_ms(1);
  EEPROM_Byte_Write(EE_ADDR1 , 0X00);
}