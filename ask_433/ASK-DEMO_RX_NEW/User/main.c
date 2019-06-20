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

extern unsigned char SelfAddr[2];

void CLOCK_Config(u8 SYS_CLK);
void All_Congfig(void);
void Pwrup_Indicate();

unsigned char table[]={"I like study high technology !\n"};

int main(void)
{
  unsigned char code[PCODE_NUM];
  unsigned char i;
  All_Congfig();        //���еĻ������ã�����ASK��
  __enable_interrupt(); //�����ж�
  Pwrup_Indicate();     //����ָʾ
  Uart_Sendbyte(1);

  I2C_Init();
  /*
  //FM24C_Reset();
  delay_ms(500);
  code[0] = 0xA;
  code[1] = 0xB;
  FM24C_WriteData(code);
  delay_ms(500);
*/
  memset(code, 0, sizeof(code));
  FM24C_ReadData(code);
  Uart_Sendbyte(code[0]);
  Uart_Sendbyte(code[1]);
  Uart_Sendbyte(2);
  ReadSelfAddr();

  if(!((code[0]==SelfAddr[0]) &&
     (code[1]==SelfAddr[1]))&&code[0]&&code[1])
  {
      Write_Coder(code[0], code[1]);
      for(i=0;i<5;i++)
      {
        Led_on(1);
        delay_ms(300);
        Led_off(1);
        delay_ms(300);
      }
  }

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
    CLOCK_Config(SYS_CLOCK);//ϵͳʱ�ӳ�ʼ��
    Led_Init();
    Key_Init();
    Pwm_Init();
    Uart_Init(16, 9600);
    Tim4_Init();
    Tim1_Init();
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

