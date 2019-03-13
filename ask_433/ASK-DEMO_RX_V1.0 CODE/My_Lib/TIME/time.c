#include "time.h"

unsigned long Timer4Count = 0;
unsigned char timer_4_count = 0;
unsigned char timer_4_countover = 0;

void Tim4_Init()
{
#if 1
  TIM4_PSCR=5;          //32��Ƶ�õ�500khz�Ķ�ʱ��ʱ�ӣ���ʱ����1����2us
  TIM4_ARR=24;          //����25ʱ��50us��ʵ��Ϊ52us����Ϊ24ʱʵ��50us
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



