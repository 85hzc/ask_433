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

extern uint8_t fm24c_store_addr;
extern FM24C_Data_S EE_dev_data;

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

  All_Congfig();        //所有的基本配置，除了ASK的
  __enable_interrupt(); //开总中断
  Pwrup_Indicate();     //开机指示

  I2C_Init();
#if 0
  //FM24C_Reset();
  FM24C_SetDevInfo(0);
#endif

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
    Pwm_Init();
    Uart_Init(16, 9600);
    Tim4_Init();
    //Tim1_Init();
    TIM1_PWM_SET();
    #ifdef PROJECT_SPOTLIGHT
    Drv_MOTOR_Init();
    #endif
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
  for(i=0; i<3; i++)
  {
    Led_on_all();
    delay_ms(200);
    Led_off_all();
    delay_ms(200);
  }
}

