/*********************************************************
//说明：ASK无线编码码程序，模拟EV1527编码，最小脉宽为400us。
//单片机：STM8S003F3P6
//晶振：内部16Mhz
//作者：少凯同学，2168916131@qq.com
//时间：20170708
***********************************************************/
#include"ASK.h"

unsigned char Ask_send_buf[ASK_SEND_LEN]={0};
extern FM24C_Data_S fm24c_data;

FM24C_Data_S EE_dev_data;

void rf_delay_long()
{//实测1.2ms
    unsigned char i, j;
    for(i=0;i<167;i++)
    {
        for(j=0;j<21;j++)
            ;
    }
}

void rf_delay_short()
{	//实测396~404us
    unsigned char i, j;
    for(i=0;i<55;i++)
    {
        for(j=0;j<21;j++)
            ;
    }
}

unsigned long gRfDelay = 0;
void rf_delay_15ms()
{
    gRfDelay = GetTimer();
    while(SpanTime(gRfDelay) < 15)
    {
        ;
    } 
}

void send_one()
{
    ASK=1;
    rf_delay_long();
    ASK=0;
    rf_delay_short();
}

void send_zero()
{
    ASK=1;
    rf_delay_short();
    ASK=0;
    rf_delay_long();
}

void send_byte(unsigned char da)
{
    unsigned char i;
    for(i=8;i>0;i--)
    {
        if(da & 0x80)
        {
            send_one();
        }
        else
        {
            send_zero();
        }
        da = da<<1;
    }
}

void ask_send(unsigned char datt[], unsigned char len)
{
     unsigned char i;
     for(i=0;i<len;i++)
     {
        send_byte(datt[i]);
     }
     send_one();
     rf_delay_15ms();
}

void Ask_IO_Init()
{
  PD_DDR_DDR3 =1;
  PD_CR1_C13  =1;
}

void Ask_process()
{
  unsigned char i,j;
  unsigned char key_value=KEY_NULL;

  Ask_IO_Init();
  ReadSelfAddr();

  while(1)
  {
    key_value=key_scan();
    if(key_value != KEY_NULL)
    {
      #ifdef READ_REALTIME
      FM24C_ReadDevInfo(0);
      
      Ask_send_buf[0]=fm24c_data.assoAddr[0];
      Ask_send_buf[1]=fm24c_data.assoAddr[1];

      for(i=0;i<fm24c_data.instNum;i++)
      {
        for(j=0;j<8;j++)
        {
          //if(fm24c_data.dev[i].key[j].keyType==key_value)
          {
            //Ask_send_buf[2] = fm24c_data.dev[i].devAddr;
            Ask_send_buf[2] = 0xf0 | key_value;//fm24c_data.dev[i].key[j].keyType;
            //Ask_send_buf[4] = fm24c_data.dev[i].key[j].scene;
            Uart_Sendbyte(Ask_send_buf[2]);
            ask_send(Ask_send_buf, ASK_SEND_LEN);
            ask_send(Ask_send_buf, ASK_SEND_LEN);
            ask_send(Ask_send_buf, ASK_SEND_LEN);
            ask_send(Ask_send_buf, ASK_SEND_LEN);
            break;
          }
        }
      }

      #else
      Ask_send_buf[0]=EE_dev_data.assoAddr[0];
      Ask_send_buf[1]=EE_dev_data.assoAddr[1];

      #ifdef TJD
      Ask_send_buf[0]=NETID;
      #endif
      #if 1
      Ask_send_buf[1] = 0;
      Ask_send_buf[2] = key_value;
  
      ask_send(Ask_send_buf, ASK_SEND_LEN);
      ask_send(Ask_send_buf, ASK_SEND_LEN);
      ask_send(Ask_send_buf, ASK_SEND_LEN);
      ask_send(Ask_send_buf, ASK_SEND_LEN);

      #else
      for(i=0;i<EE_dev_data.instNum;i++)
      {
        for(j=0;j<8;j++)
        {
          if(EE_dev_data.dev[i].key[j].keyType==key_value)
          {
            Ask_send_buf[2] = EE_dev_data.dev[i].devAddr;
            Ask_send_buf[3] = EE_dev_data.dev[i].key[j].keyType;
            Ask_send_buf[4] = EE_dev_data.dev[i].key[j].scene;

            ask_send(Ask_send_buf, ASK_SEND_LEN);
            ask_send(Ask_send_buf, ASK_SEND_LEN);
            ask_send(Ask_send_buf, ASK_SEND_LEN);
            ask_send(Ask_send_buf, ASK_SEND_LEN);
            break;
          }
        }
      }
      #endif
      #endif
      #ifndef THERMOELECTRIC_SENSOR
      Led_off_all();
      #endif
      delay_ms(10);
    }
  }
}

///////////////////读取ID函数///////////////////////
void ReadSelfAddr()
{
  uint8_t i,j;

  EE_dev_data.assoAddr[0]= EEPROM_Byte_Read(EE_ADDR0);
  EE_dev_data.assoAddr[1]= EEPROM_Byte_Read(EE_ADDR1);
  EE_dev_data.instNum = EEPROM_Byte_Read(EE_ADDR_InsNum);

  for(j=0;j<EE_dev_data.instNum && j<MAX_DEV_NUM;j++)
  {
    delay_ms(1);
    EE_dev_data.dev[j].devType = EEPROM_Byte_Read(EE_ADDR_DevType+j*0x10);
    delay_ms(1);
    EE_dev_data.dev[j].devAddr = EEPROM_Byte_Read(EE_ADDR_DevAddr+j*0x10);

    for(i=0;i<8;i++)
    {
      delay_ms(1);
      EE_dev_data.dev[j].key[i].keyType = EEPROM_Byte_Read(EE_ADDR_KeyVal+j*0x10+i*4);
      EE_dev_data.dev[0].key[i].scene = EEPROM_Byte_Read(EE_ADDR_KeyVal+j*0x10+i*4+2);
    }
  }
}

///////////////////清除对码函数///////////////////////
void Write_Coder()
{
  uint8_t i,j;

  EEPROM_EREASE();

  delay_ms(1);
  EEPROM_Byte_Write(EE_ADDR0, fm24c_data.assoAddr[0]);
  delay_ms(1);
  EEPROM_Byte_Write(EE_ADDR1, fm24c_data.assoAddr[1]);
  delay_ms(1);
  EEPROM_Byte_Write(EE_ADDR_InsNum, fm24c_data.instNum);

  for(j=0;j<MAX_DEV_NUM;j++)
  {
    delay_ms(1);
    EEPROM_Byte_Write(EE_ADDR_DevAddr+j*0x10, fm24c_data.dev[j].devAddr);
  
    for(i=0;i<8;i++)
    {
      delay_ms(1);
      EEPROM_Byte_Write(EE_ADDR_KeyVal+j*0x10+i*4, fm24c_data.dev[j].key[i].keyType);
      EEPROM_Byte_Write(EE_ADDR_KeyVal+j*0x10+i*4+2, fm24c_data.dev[j].key[i].scene);
    }
  }
}


