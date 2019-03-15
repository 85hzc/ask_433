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

void SysClock_Init(u8 SYS_CLK);
void CLOCK_Config(u8 SYS_CLK);
void All_Congfig(void);
void Pwrup_Indicate();

unsigned char table[]={"I like study\n"};

int main(void)
{     
  //unsigned char i;    
  All_Congfig();        //���еĻ������ã�����ASK��
  //asm("rim");
  __enable_interrupt(); //�����ж�
  Pwrup_Indicate();     //����ָʾ
  Ask_process();
  
  while(1)
  {         
    Led_on(1);
    delay_ms(200);
    Led_off(1);
    delay_ms(200);
  }        
}

void All_Congfig(void)
{
    //CLOCK_Config(SYS_CLOCK);//ϵͳʱ�ӳ�ʼ��  
    SysClock_Init(SYS_CLOCK);

    Led_Init();
    Pwm_Init();
    Key_Init();
    //Uart_Init(16, 9600);
    Tim4_Init();
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
      case 2:
        CLK_CKDIVR |=((1<<4)|(1<<3));
        break;
      case 4:
        CLK_CKDIVR |=(1<<4);
        break;
      case 8:
        CLK_CKDIVR |=(1<<3);
        break;
   }
}

void SysClock_Init(u8 SYS_CLK)
{
/*
    CLK_ICKCR |= 0x01;               //�����ڲ�HSI
    while(!(CLK_ICKCR&0x02));       //HSI׼��������ȡCLK_ICKCR bit1
    CLK_SWR = 0x01;                 //HSIΪ��ʱ��Դ
    CLK_CKDIVR = 0X04;             //16��Ƶ
    //CLK_ICKCR |=(1 << 2);           //open LSI clock 
*/

#if 0
   CLK_CKDIVR = 0X03;
  
   switch(SYS_CLK)
   {
      case 2: CLK_CKDIVR |=((1<<4)|(1<<3)); break;
      case 4: CLK_CKDIVR |=(1<<4); break;
      case 8: CLK_CKDIVR |=(1<<3); break;
   }
#else
    CLK_CKDIVR = 0x0; //8分频 0x03
    CLK_ICKCR  = 0x1; //启用内部RC=16mhz
    //CLK_ICKCR  = 0x11; //启用内部RC=16mhz
    while((CLK_ICKCR & 0x02)==0); //等待时钟稳定
      CLK_PCKENR1 = 0xff;         //定义，时钟提供给设备
    //CLK_PCKENR2 |= 0b00111111;  //定义，时钟提供给设备
#endif
  
}

void Pwrup_Indicate()
{
  unsigned char i;
  for(i=0; i<3; i++)
  {
    Led_on_all();
    delay_ms(100);
    Led_off_all();
    delay_ms(100);
  }
}

