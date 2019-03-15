#include "time.h"

unsigned long Timer4Count = 0;
unsigned char timer_4_count = 0;
unsigned char timer_4_countover = 0;

#if 1
void Tim4_Init(void)
{
#if 1
    /*很多人都是在这里装填0xFF，其实是为了让PSC尽快生效，
　　*对于PSC的设置，需要在下一个更新事件时才会生效*/
    TIM4_CNTR=0;    /*计数器值*/
    TIM4_ARR=12;//0xFA;  /*自动重装寄存器 250，产生125次定时1S*/
    TIM4_PSCR=0x05; /*预分频系数为16(0x07 128) */
    TIM4_EGR=0x01;  /*手动产生一个更新事件，用于PSC生效。注意，是手动更新*/
    TIM4_IER_UIE=1;  /*更新事件中断使能*/
    TIM4_CR1_ARPE=1;  /*使能计时器，TIM4_CR0停止计时器*/
    TIM4_CR1_URS=1;
    TIM4_CR1_CEN=1;
    TIM4_EGR_UG=1;
#endif

}

#else
//#define TIM4_PSCR_PSC_stm8l ((uint8_t)0x0f) /*!< Prescaler Value  mask. */

void Tim4_Init(void)
{
  //-------打开TIM4外设时钟-------

  //CLK_PCKENR1_PCKEN12=1;//打开定时器1时钟
  CLK_PCKENR1 = 0xff;
  //----禁用预装载寄存器----

  TIM4_CR1_ARPE =0;//不经过缓存,分频值和重装值立即被写入.
  //除非需要频繁的在定时器运行时,改变分频值和重装值,否则没必要使用预装载寄存器
  //------设置TIM4时钟分频值------
  //TIM4_PSCR_PSC_stm8l=10;//分频值  2M/2^10=2M/1024=1953.125Hz
  TIM4_PSCR=0x07;

  //-重装值,TIM4从0计数到此值,发生溢出-
  TIM4_ARR=200;//自动重装值    1953.125Hz/200=9.76HZ

  //1000ms/9.76=102ms
  //TIM4定时器每隔102ms进入一次中断
  //本程序,每次进入中断后会反转接LED的IO电平,所以LED闪烁周期为2*102ms=204ms
  //LED闪烁频率为 1000ms/204ms=4.9Hz
  //读者可以改变ARR值和PSC值,根据上面计算过程,验证定时器的这种功能

  TIM4_CR1_URS=1;//仅当计数器溢出时才发生中断请求
  TIM4_CR1_UDIS=0;//允许更新中断
  TIM4_CR1_CEN=1;//开启计数器
  TIM4_IER_UIE=1;// update interrupt enable
}
#endif

//#pragma vector = 25 //stm8l051
#pragma vector = TIM4_UIF_vector
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

  PWM2 = ~PWM2;
  TIM4_SR1_UIF=0;
}
