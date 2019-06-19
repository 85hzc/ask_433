
#include "All_Includes.h"

/***********************
函数功能：us延时
输入参数：无
输出参数：无
备    注：粗略延时
***********************/
void delay_nop(void)
{ 
    asm("nop"); //一个asm("nop")函数经过示波器测试代表100ns
    asm("nop");
    asm("nop");
    asm("nop"); 
}

/***********************
函数功能：ms延时
输入参数：无
输出参数：无
备    注：粗略延时
***********************/
void delay_ms(unsigned int time)
{
    unsigned int i;
    while(time--)  
    for(i=900;i>0;i--)
    delay_nop(); 
}

/***********************
函数功能：us延时
输入参数：无
输出参数：无
备    注：粗略延时
***********************/
void delay_us(/*unsigned int time*/)
{
    unsigned int i;
    //while(time--)
    for(i=4;i>0;i--)
    delay_nop(); 
}
///////////////////////////////////////////////////////
extern unsigned long Timer4Count;       //系统计时用

/********获取当前时间函数*********/

unsigned long GetTimer()
{
  unsigned long Timer0Temp;
  //ET0 = 0;
  TIM4_IER_UIE=0;
  Timer0Temp = Timer4Count;
  //ET0 = 1;
  TIM4_IER_UIE=1;
  return Timer0Temp;
}

/********扫描时间间隔函数*********/
unsigned long SpanTime(unsigned long OldTime)
{
  unsigned long Timer0Temp;
  //ET0 = 0;  //关T0中断
  TIM4_IER_UIE=0;
  Timer0Temp = Timer4Count; //取当前定时计数器的计数值。
  //ET0 = 1;  //使能T0中断
  TIM4_IER_UIE=1;
  if(Timer0Temp >= OldTime)
    return Timer0Temp - OldTime;  //返回两次计数器的差值
  else //如果新的值小于老的值，说明计数器已经至少走过了一轮了
    return (unsigned long)-1 - OldTime + Timer0Temp;
}

/********精确m毫秒延时函数*********/
void DelayMS(unsigned long ms)
{
  unsigned long __DelayTimerMs;
  __DelayTimerMs = GetTimer();
  while(SpanTime(__DelayTimerMs) < ms);
    //WDT_CountClear();

}

