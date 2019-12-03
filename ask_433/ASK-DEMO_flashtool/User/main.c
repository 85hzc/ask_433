/*********************************************************
//˵����ASK���߽�����򣬽�EV1527���룬��С����Ϊ300~500us��ʹ��ǰ��Ҫ���룬����һ�°������ɿ���
//      4��LEDȫ����Ϩ��ֻ��LED2��LED3������5s���ڰ���ң���������Է���LED��������˸���κ�Ϩ�𼴿���ɶ��롣
//      �Ժ��ң�����Ϳ��ԶԸ�ASK_Demo�����ң�ء���ASK_Demo���յ������źź���������ڣ�Ȼ���ж�ID��ok�������Ӧ��LED��100ms��Ϩ��     
//��Ƭ����STM8S003F3P6
//�����ڲ�16Mhz
//���ߣ��ٿ�ͬѧ��2168916131@qq.com
//ʱ�䣺20170708
***********************************************************/
#include "ALL_Includes.h"
//����CPU�ڲ�ʱ��
#define  SYS_CLOCK    16

extern uint8_t fm24c_store_addr;
extern FM24C_Data_S EE_dev_data;
extern unsigned char rxlen,rxBuff[64],txBuff[64],uart_flag;

void CLOCK_Config(u8 SYS_CLK);
void All_Congfig(void);
void Pwrup_Indicate();
void Ask_process();
void FM24C_SetDevInfo(uint8_t devId);
void FM24C_ReadDevInfo(uint8_t devId);

FM24C_Data_S fm24c_data;

int main(void)
{
  unsigned char i;

  All_Congfig();        //���еĻ������ã�����ASK��
  __enable_interrupt(); //�����ж�
  Pwrup_Indicate();     //����ָʾ

  I2C_Init();
#if 0
  //FM24C_Reset();
  FM24C_SetDevInfo(0);
#endif
#if 0
  FM24C_ReadDevInfo(0);
  ReadSelfAddr();

  if(!((fm24c_data.assoAddr[0]==EE_dev_data.assoAddr[0]) &&
     (fm24c_data.assoAddr[1]==EE_dev_data.assoAddr[1]))&&fm24c_data.assoAddr[0]&&fm24c_data.assoAddr[1])
  {
      Write_Coder();
      for(i=0;i<5;i++)
      {
        Led_on(1);
        delay_ms(300);
        Led_off(1);
        delay_ms(300);
      }
  }

  Ask_process();
#endif

  while(1)
  {
    if(uart_flag)
    {
      Uart_Sendbyte(0x88);
      for(i=0;i<5;i++)
      {
         Uart_Sendbyte(rxBuff[i]);
      }
        //read eeprom info
        if(rxBuff[0]==0xf1 &&rxBuff[1]==0xf2)
        {
            FM24C_ReadDevInfo(0);
            Uart_sendEEpromInfo();
        }
        else if(rxBuff[0]==0xf1 &&rxBuff[1]==0xf3)
        {
            Uart_SetDevInfo();
            Uart_Sendbyte(0);//return ok
        }
        uart_flag = 0;
    }
    
    Led_on(1);
    delay_ms(200);
    Led_off(1);
    delay_ms(200);
  }
}

void All_Congfig(void)
{
    CLOCK_Config(SYS_CLOCK);//ϵͳʱ�ӳ�ʼ��
    Led_Init();
    Key_Init();
    Pwm_Init();
    Uart_Init(16, 9600);
    Tim4_Init();
    //Tim1_Init();
    TIM1_PWM_SET();
}


/*********************************************
�������ܣ�ϵͳ�ڲ�ʱ������
���������SYS_CLK : 2��4��8��16
�����������
��    ע��ϵͳ����Ĭ���ڲ�2�ͣȣ�
*********************************************/
void CLOCK_Config(u8 SYS_CLK)
{
   //ʱ������Ϊ�ڲ�RC��16M
   CLK_CKDIVR &=~(BIT(4)|BIT(3));
  
   switch(SYS_CLK)
   {
      case 2: CLK_CKDIVR |=((1<<4)|(1<<3)); break;
      case 4: CLK_CKDIVR |=(1<<4); break;
      case 8: CLK_CKDIVR |=(1<<3); break;
   }
}

void Pwrup_Indicate()
{
  unsigned char i;
  for(i=0; i<3; i++)
  {
    Led_on_all();
    delay_ms(200);
    Led_off_all();
    delay_ms(200);
  }
}