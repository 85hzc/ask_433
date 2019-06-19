#include "ALL_Includes.h"

void Key_Init()
{
  PA_DDR_DDR1=0;        //KEY1输入
  PA_CR1_C11=0;         //带上拉电阻输入
  PA_CR2_C21=0;         //禁止外部中断
}

unsigned char key_scan()
{
  unsigned char i;
  unsigned char key_value=0x00;
  unsigned long key_time=0x00;
  unsigned char t_over=0;
  if(SW_K1==1)
  {
    delay_ms(10);
    if(SW_K1==1)
    {
      key_value=0x01;
      Led_on_all();
      key_time=GetTimer();
      while(SW_K1==1)
      {
        if(SpanTime(key_time)>3000)
        {
          if(t_over==0)
          {
            t_over=1;
            /*for(i=0;i<3;i++)
            {
              Led_off_all();
              delay_ms(100);
              Led_on_all();
              delay_ms(100);
            }*/
            key_value=0x02;
          }
        }
      }
      Led_off_all();
    }
  }
  return key_value;
}
