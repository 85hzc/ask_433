/*********************************************************
//说明：ASK无线解码程序，解EV1527编码，最小脉宽为300~500us。使用前需要对码，即按一下按键后松开，
//      4个LED全亮后熄灭只留LED2，LED3常亮，5s钟内按下遥控器，可以发现LED分两组闪烁三次后熄灭即可完成对码。
//      以后该遥控器就可以对该ASK_Demo板进行遥控。该ASK_Demo板收到无线信号后，先输出串口，然后判断ID，ok后点亮对应的LED，100ms后熄灭。     
//单片机：STM8S003F3P6
//晶振：内部16Mhz
//作者：少凯同学，2168916131@qq.com
//时间：20170708
***********************************************************/
#include "ALL_Includes.h"
//定义CPU内部时钟
#define  SYS_CLOCK    16

void SysClock_Init(u8 SYS_CLK);
void CLOCK_Config(u8 SYS_CLK);
void All_Congfig(void);
void Pwrup_Indicate();

unsigned char table[]={"I like study\n"};

int main(void)
{     
  //unsigned char i;    
  All_Congfig();        //所有的基本配置，除了ASK的
  //asm("rim");
  __enable_interrupt(); //开总中断
  Pwrup_Indicate();     //开机指示
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
    //CLOCK_Config(SYS_CLOCK);//系统时钟初始化  
    SysClock_Init(SYS_CLOCK);

    Led_Init();
    Pwm_Init();
    Key_Init();
    //Uart_Init(16, 9600);
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
    CLK_ICKCR |= 0x01;               //开启内部HSI
    while(!(CLK_ICKCR&0x02));       //HSI准备就绪读取CLK_ICKCR bit1
    CLK_SWR = 0x01;                 //HSI为主时钟源
    CLK_CKDIVR = 0X04;             //16分频
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
    CLK_CKDIVR = 0x0; //8鍒嗛 0x03
    CLK_ICKCR  = 0x1; //鍚敤鍐呴儴RC=16mhz
    //CLK_ICKCR  = 0x11; //鍚敤鍐呴儴RC=16mhz
    while((CLK_ICKCR & 0x02)==0); //绛夊緟鏃堕挓绋冲畾
      CLK_PCKENR1 = 0xff;         //瀹氫箟锛屾椂閽熸彁渚涚粰璁惧
    //CLK_PCKENR2 |= 0b00111111;  //瀹氫箟锛屾椂閽熸彁渚涚粰璁惧
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

