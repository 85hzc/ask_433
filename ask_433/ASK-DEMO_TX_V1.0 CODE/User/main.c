/*********************************************************
//说明：ASK无线编码码程序，模拟EV1527编码，最小脉宽为400us。
//单片机：STM8S003F3P6
//晶振：内部16Mhz
//作者：少凯同学，2168916131@qq.com
//时间：20170708
***********************************************************/
#include "ALL_Includes.h"
//定义CPU内部时钟
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
  All_Congfig();        //所有的基本配置，除了ASK的
  __enable_interrupt(); //开总中断
  Pwrup_Indicate();     //开机指示
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
    CLOCK_Config(SYS_CLOCK);//系统时钟初始化
    Led_Init();
    Key_Init();
    Uart_Init(16, 9600);
    Tim4_Init();
}


/*********************************************
函数功能：系统内部时钟配置
输入参数：SYS_CLK : 2、4、8、16
输出参数：无
备    注：系统启动默认内部2ＭＨＺ
*********************************************/
void CLOCK_Config(u8 SYS_CLK)
{
   //时钟配置为内部RC，16M
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
  //打印出来发现第1、2、3个字节不一样，其他都一样。
  for(i=0; i<3; i++)
  {
    Led_on_all();
    delay_ms(200);
    Led_off_all();
    delay_ms(200);
  }
}

