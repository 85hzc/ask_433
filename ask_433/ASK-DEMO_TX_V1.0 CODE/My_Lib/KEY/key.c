#include "ALL_Includes.h"

void Key_Init()
{
#if 1
  PD_DDR_DDR4=0;        //KEY1输入
  PD_CR1_C14=0;         //带上拉电阻输入(PD_CR1_C14=1;)//浮空输入（PD_CR1_C14=0）
  PD_CR2_C24=0;         //禁止外部中断
  
  PA_DDR_DDR1=0;        //KEY2输入
  PA_CR1_C11=0;         //带上拉电阻输入
  PA_CR2_C21=0;         //禁止外部中断
  
  PC_DDR_DDR5=0;        //KEY3输入
  PC_CR1_C15=0;         //带上拉电阻输入
  PC_CR2_C25=0;         //禁止外部中断
  
  PA_DDR_DDR2=0;        //KEY4输入
  PA_CR1_C12=0;         //带上拉电阻输入
  PA_CR2_C22=0;         //禁止外部中断

  ///////////key 5678//////////
  PA_DDR_DDR3=0;        //KEY5输入
  PA_CR1_C13=0;         //带上拉电阻输入
  PA_CR2_C23=0;         //禁止外部中断
  
  PC_DDR_DDR6=0;        //KEY6输入
  PC_CR1_C16=0;         //带上拉电阻输入
  PC_CR2_C26=0;         //禁止外部中断
  
  PC_DDR_DDR3=0;        //KEY7输入
  PC_CR1_C13=0;         //带上拉电阻输入
  PC_CR2_C23=0;         //禁止外部中断
  
  PC_DDR_DDR4=0;        //KEY8输入
  PC_CR1_C14=0;         //带上拉电阻输入
  PC_CR2_C24=0;         //禁止外部中断
#else
  PD_DDR_DDR4=0;        //KEY1输入
  PD_CR1_C14=1;         //带上拉电阻输入(PD_CR1_C14=1;)//浮空输入（PD_CR1_C14=0）
  PD_CR2_C24=0;         //禁止外部中断
  
  PA_DDR_DDR1=0;        //KEY2输入
  PA_CR1_C11=1;         //带上拉电阻输入
  PA_CR2_C21=0;         //禁止外部中断
  
  PC_DDR_DDR5=0;        //KEY3输入
  PC_CR1_C15=1;         //带上拉电阻输入
  PC_CR2_C25=0;         //禁止外部中断
  
  PA_DDR_DDR2=0;        //KEY4输入
  PA_CR1_C12=1;         //带上拉电阻输入
  PA_CR2_C22=0;         //禁止外部中断

  ///////////key 5678//////////
  PA_DDR_DDR3=0;        //KEY5输入
  PA_CR1_C13=1;         //带上拉电阻输入
  PA_CR2_C23=0;         //禁止外部中断
  
  PC_DDR_DDR6=0;        //KEY6输入
  PC_CR1_C16=1;         //带上拉电阻输入
  PC_CR2_C26=0;         //禁止外部中断
  
  PC_DDR_DDR3=0;        //KEY7输入
  PC_CR1_C13=1;         //带上拉电阻输入
  PC_CR2_C23=0;         //禁止外部中断
  
  PC_DDR_DDR4=0;        //KEY8输入
  PC_CR1_C14=1;         //带上拉电阻输入
  PC_CR2_C24=0;         //禁止外部中断
#endif
}

unsigned char key_scan()
{
  unsigned char key_value=0x00;

  if(SW_K1==1)
  {
    delay_ms(10);
    if(SW_K1==1)
    {
      key_value=0x01;
      Led_on(1);
    }
  }
  else if(SW_K2==1)
  {
    delay_ms(10);
    if(SW_K2==1)
    {
      key_value=0x02;
      Led_on(2);
    }
  }
  else if(SW_K3==1)
  {
    delay_ms(10);
    if(SW_K3==1)
    {
      key_value=0x03;
      Led_on(3);
    }
  }
  else if(SW_K4==1)
  {
    delay_ms(10);
    if(SW_K4==1)
    {
      key_value=0x04;
      Led_on(4);
    }
  }
  else if(SW_K5==1)
  {
    delay_ms(10);
    if(SW_K5==1)
    {
      key_value=0x05;
      Led_on(5);
    }
  }
  else if(SW_K6==1)
  {
    delay_ms(10);
    if(SW_K6==1)
    {
      key_value=0x06;
      Led_on(6);
    }
  }
  else if(SW_K7==1)
  {
    delay_ms(10);
    if(SW_K7==1)
    {
      key_value=0x07;
      Led_on(7);
    }
  }
  else if(SW_K8==1)
  {
    delay_ms(10);
    if(SW_K8==1)
    {
      key_value=0x08;
      Led_on(8);
    }
  }
  return key_value;
}
