/*********************************************************
//˵����ASK���߱��������ģ��EV1527���룬��С����Ϊ400us��
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
//unsigned char stm8s_id[12]={0};
int main(void)
{
  unsigned char code[PCODE_NUM];
  unsigned char i;
  All_Congfig();        //���еĻ������ã�����ASK��
  __enable_interrupt(); //�����ж�
  Pwrup_Indicate();     //����ָʾ
  Uart_Sendbyte(1);

  I2C_Init();
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
    Uart_Init(16, 9600);
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
      case 2: CLK_CKDIVR |=((1<<4)|(1<<3)); break;
      case 4: CLK_CKDIVR |=(1<<4); break;
      case 8: CLK_CKDIVR |=(1<<3); break;
   }
}

void Pwrup_Indicate()
{
  unsigned char i;
  /*
  for(i=0; i<12; i++)
  {
    stm8s_id[i]=*((unsigned char *)0x4865+i);
    Uart_Sendbyte(stm8s_id[i]);
  }*/
  //00 14 00 0C 0A 47 37 30 33 30 30 37
  //00 15 00 01 0A 47 37 30 33 30 30 37 
  //��ӡ�������ֵ�1��2��3���ֽڲ�һ����������һ����
  for(i=0; i<3; i++)
  {
    Led_on_all();
    delay_ms(200);
    Led_off_all();
    delay_ms(200);
  }
}

