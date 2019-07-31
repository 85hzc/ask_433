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
extern unsigned char pwm_duty;
extern FM24C_Data_S EE_dev_data;

void CLOCK_Config(u8 SYS_CLK);
void All_Congfig(void);
void Pwrup_Indicate();

unsigned char page[FM24C_PAGE_SIZE];
FM24C_Data_S fm24c_data;

int main(void)
{
  unsigned char i;

  All_Congfig();        //所有的基本配置，除了ASK的
  __enable_interrupt(); //开总中断
  Pwrup_Indicate();     //开机指示
  Uart_Sendbyte(1);

  I2C_Init();
#if 0
  //FM24C_Reset();
  memset(&fm24c_data, 0, sizeof(FM24C_Data_S));
  delay_ms(20);
  fm24c_data.cardType = 0x11;
  fm24c_data.instNum = 1;
  fm24c_data.assoAddr[0] = 0xA;
  fm24c_data.assoAddr[1] = 0xB;
  fm24c_store_addr=0;
  FM24C_WriteData(&fm24c_data);

  delay_ms(20);
  fm24c_data.dev[0].devType = 0x01;
  fm24c_data.dev[0].devAddr = 0x28;
  fm24c_store_addr=0x10;
  FM24C_WriteData(&fm24c_data.dev[0]);

  delay_ms(20);
  fm24c_data.dev[0].keyValue[0] = 0x10|LIGHT_ON;
  fm24c_data.dev[0].keyValue[1] = 0x20|LIGHT_OFF;
  fm24c_data.dev[0].keyValue[2] = 0x30|LIGHT_UP;
  fm24c_data.dev[0].keyValue[3] = 0x40|LIGHT_DOWN;
  fm24c_store_addr=0x18;
  FM24C_WriteData(&fm24c_data.dev[0].keyValue[0]);
#endif

  memset(&fm24c_data, 0, sizeof(FM24C_Data_S));
  delay_ms(10);

  fm24c_store_addr=0;
  FM24C_ReadData(page);
  fm24c_data.cardType = page[0];
  fm24c_data.instNum = page[1];
  fm24c_data.assoAddr[0] = page[2];
  fm24c_data.assoAddr[1] = page[3];
  
  delay_ms(20);
  fm24c_store_addr=0x10;
  FM24C_ReadData(page);
  fm24c_data.dev[0].devType = page[0];
  fm24c_data.dev[0].devAddr = page[1];

  delay_ms(20);
  fm24c_store_addr=0x18;
  FM24C_ReadData(page);
  fm24c_data.dev[0].keyValue[0] = page[0];
  fm24c_data.dev[0].keyValue[1] = page[1];
  fm24c_data.dev[0].keyValue[2] = page[2];
  fm24c_data.dev[0].keyValue[3] = page[3];
  fm24c_data.dev[0].keyValue[4] = page[4];
  fm24c_data.dev[0].keyValue[5] = page[5];
  fm24c_data.dev[0].keyValue[6] = page[6];
  fm24c_data.dev[0].keyValue[7] = page[7];

  ReadSelfAddr();
  Set_Pwm(pwm_duty);

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
    Tim1_Init();
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

