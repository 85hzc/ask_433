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

  All_Congfig();        //���еĻ������ã�����ASK��
  __enable_interrupt(); //�����ж�
  Pwrup_Indicate();     //����ָʾ
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

