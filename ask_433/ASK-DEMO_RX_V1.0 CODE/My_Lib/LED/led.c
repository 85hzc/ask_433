#include "ALL_Includes.h"

/***********************
函数功能：初始化LED
输入参数：无
输出参数：无
备    注：LED_L1, LED_L2, LED_L3, LED_L4分别为PC3, PA1，PA2， PA3。
***********************/
void Led_Init(void)
{
    PB_DDR_DDR4 =1;
    PB_CR1_C14  =1;
    /*
    PA_DDR_DDR1 =1;
    PA_CR1_C11  =1;

    PA_DDR_DDR2 =1;
    PA_CR1_C12  =1; 
    
    PA_DDR_DDR3 =1;
    PA_CR1_C13  =1; 
    */
    Led_off_all();
}

void Pwm_Init(void)
{
    PB_DDR_DDR2 =1;
    PB_CR1_C12  =1;

    PB_DDR_DDR1 =1;
    PB_CR1_C11  =1;
}


void Led_on(unsigned char m)
{
  switch(m)
  {
  case 1: LED_L1=0;
    break;
    /*
  case 2: LED_L2=1;
    break;
  case 3: LED_L3=1;
    break;
  case 4: LED_L4=1;
    break;
    */
  default:
    break;
  }
}

void Led_off(unsigned char m)
{
  switch(m)
  {
  case 1: LED_L1=1;
    break;
    /*
  case 2: LED_L2=0;
    break;
  case 3: LED_L3=0;
    break;
  case 4: LED_L4=0;
    break;
    */
  default:
    break;
  }
}

void Led_on_all()
{
  LED_L1=0;
  //LED_L2=1;
  //LED_L3=1;
  //LED_L4=1;
}

void Led_off_all()
{
  LED_L1=1;
  //LED_L2=0;
  //LED_L3=0;
  //LED_L4=0;
}

