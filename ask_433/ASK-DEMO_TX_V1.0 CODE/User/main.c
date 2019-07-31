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

extern uint8_t fm24c_store_addr;
extern FM24C_Data_S EE_dev_data;

void CLOCK_Config(u8 SYS_CLK);
void All_Congfig(void);
void Pwrup_Indicate();

unsigned char page[FM24C_PAGE_SIZE];
FM24C_Data_S fm24c_data;

int main(void)
{
  unsigned char i,j;

  All_Congfig();        //所有的基本配置，除了ASK的
  __enable_interrupt(); //开总中断
  Pwrup_Indicate();     //开机指示
  Uart_Sendbyte(1);

  I2C_Init();

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

  for(i=0;i<fm24c_data.instNum && fm24c_data.instNum<MAX_DEV_NUM;i++)
  {
    fm24c_store_addr=0x10*(i+1) | 0x08;
    FM24C_ReadData(page);

    for(j=0;j<8;j++)
    {
      fm24c_data.dev[i].keyValue[j] = page[j];
    }
  }

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

