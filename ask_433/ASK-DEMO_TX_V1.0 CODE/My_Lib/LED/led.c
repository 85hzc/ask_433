#include "ALL_Includes.h"

/***********************
函数功能：初始化LED
输入参数：无
输出参数：无
备    注：LED_L1, LED_L2, LED_L3, LED_L4分别为PC3, PA1，PA2， PA3。
***********************/
void Led_Init(void)
{
#ifdef DEMO_EVB
    PA_DDR_DDR3 =1;
    PA_CR1_C13  =1;
    
    PB_DDR_DDR4 =1;
    PB_CR1_C14  =1;

    PB_DDR_DDR5 =1;
    PB_CR1_C15  =1;
    
    PC_DDR_DDR3 =1;
    PC_CR1_C13  =1;
#else
    PD_DDR_DDR2 =1;
    PD_CR1_C12  =1;
    
    PC_DDR_DDR7 =1;
    PC_CR1_C17  =1;
#endif
    Led_off_all();
}

void Led_on(unsigned char m)
{
  switch(m)
  {
      case 1: 
        LED_L5=1;
        LED_L6=1;
        break;
      case 2: 
        LED_L5=1;
        LED_L6=1;
        break;
      case 3: 
        LED_L5=1;
        LED_L6=1;
        break;
      case 4: 
        LED_L5=1;
        LED_L6=1;
        break;
      case 5: 
        LED_L5=1;
        LED_L6=1;
        break;
      case 6: 
        LED_L5=1;
        LED_L6=1;
        break;
      case 7: 
        LED_L5=1;
        LED_L6=1;
        break;
      case 8: 
        LED_L5=1;
        LED_L6=1;
        break;
      default:
        break;
  }
}

void Led_off(unsigned char m)
{
  switch(m)
  {
      case 1: 
        LED_L5=0;
        LED_L6=0;
        break;
      case 2: 
        LED_L5=0;
        LED_L6=0;
        break;
      case 3: 
        LED_L5=0;
        LED_L6=0;
        break;
      case 4: 
        LED_L5=0;
        LED_L6=0;
        break;
      case 5: 
        LED_L5=0;
        LED_L6=0;
        break;
      case 6: 
        LED_L5=0;
        LED_L6=0;
        break;
      case 7: 
        LED_L5=0;
        LED_L6=0;
        break;
      case 8: 
        LED_L5=0;
        LED_L6=0;
        break;

      default:
        break;
  }
}

void Led_on_all()
{
#ifdef DEMO_EVB
  LED_L1=0;
  LED_L2=0;
  LED_L3=0;
  LED_L4=0;
#else
  LED_L5=1;
  LED_L6=1;
#endif
}

void Led_off_all()
{
#ifdef DEMO_EVB
  LED_L1=1;
  LED_L2=1;
  LED_L3=1;
  LED_L4=1;
#else
  LED_L5=0;
  LED_L6=0;
#endif
}

