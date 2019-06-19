/*********************************************************
//说明：ASK无线编码码程序，模拟EV1527编码，最小脉宽为400us。
//单片机：STM8S003F3P6
//晶振：内部16Mhz
//作者：少凯同学，2168916131@qq.com
//时间：20170708
***********************************************************/
#include"ASK.h"

unsigned char Ask_send_buf[3]={0};
extern unsigned char stm8s_id[12];

unsigned char SelfAddr[2]={0, 0};

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
  unsigned char i;
  unsigned char key_value=0;

  Ask_IO_Init();
  ReadSelfAddr();

  while(1)
  {
    key_value=key_scan();
    if(key_value != 0)
    {

      Ask_send_buf[0]=SelfAddr[0];
      Ask_send_buf[1]=SelfAddr[1];
      Ask_send_buf[2] = key_value;

      Uart_Sendbyte(Ask_send_buf[2]);
      ask_send(Ask_send_buf, 3);
      ask_send(Ask_send_buf, 3);
      ask_send(Ask_send_buf, 3);
      ask_send(Ask_send_buf, 3);
      Led_off_all();
      delay_ms(10);
    }
  }
}

///////////////////读取ID函数///////////////////////
void ReadSelfAddr()
{
  SelfAddr[0]= EEPROM_Byte_Read(EE_ADDR0);
  SelfAddr[1]= EEPROM_Byte_Read(EE_ADDR1);
  
  Uart_Sendbyte(SelfAddr[0]);
  Uart_Sendbyte(SelfAddr[1]);  
}

///////////////////清除对码函数///////////////////////
void Write_Coder(unsigned char a,unsigned char b)
{
  EEPROM_EREASE();
  delay_ms(1);
  Uart_Sendbyte(3);

  Uart_Sendbyte(a);
  Uart_Sendbyte(b);
  
  EEPROM_Byte_Write(EE_ADDR0, a);
  delay_ms(1);
  EEPROM_Byte_Write(EE_ADDR1, b);
  
  Uart_Sendbyte(a);
  Uart_Sendbyte(b);
}


