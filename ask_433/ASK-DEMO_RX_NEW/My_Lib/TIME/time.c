#include "time.h"

unsigned long Timer4Count = 0;
unsigned char timer_4_count = 0;
unsigned char timer_4_countover = 0;

void Tim4_Init()
{
#if 1
  TIM4_PSCR=5;          //32分频得到500khz的定时器时钟，定时器加1就是2us
  TIM4_ARR=24;          //理论25时是50us，实测为52us，改为24时实测50us
  TIM4_IER_UIE=1;
  TIM4_CR1_ARPE=1;
  TIM4_CR1_CEN=1;  
#endif
  //tim4.PSCR=6;
  //tim4.ARR=0xfa;
  //tim4.IER |= 0x01;
  //tim4.CR1 |= 0x81;
}


//#pragma vector = 25
#pragma vector = TIM4_OVR_UIF_vector
__interrupt __root void TIM4_UIF_HANDLER()
{
  static unsigned char tt4=0;
  //PA_ODR_ODR2=~PA_ODR_ODR2;
  tt4++;
  if(tt4>=20)
  {//1ms
    tt4=0;
    Timer4Count++; 
    //PA_ODR_ODR3=~PA_ODR_ODR3;
  }   
  timer_4_count++;
  if(timer_4_count == 0)
    timer_4_countover++;
  
  TIM4_SR_UIF=0;
}



