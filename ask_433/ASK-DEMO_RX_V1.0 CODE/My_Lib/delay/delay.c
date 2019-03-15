
#include "All_Includes.h"

/***********************
�������ܣ�us��ʱ
�����������
�����������
��    ע��������ʱ
***********************/
void delay_us(void)
{ 
    asm("nop"); //һ��asm("nop")��������ʾ�������Դ���100ns
    asm("nop");
    asm("nop");
    asm("nop");
}

/***********************
�������ܣ�ms��ʱ
�����������
�����������
��    ע��������ʱ
***********************/
void delay_ms(unsigned int time)
{
    unsigned int i;
    
    while(time--)
        for(i=900;i>0;i--)
            delay_us(); 
}
///////////////////////////////////////////////////////
extern unsigned long Timer4Count;       //ϵͳ��ʱ��

/********��ȡ��ǰʱ�亯��*********/

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

/********ɨ��ʱ��������*********/
unsigned long SpanTime(unsigned long OldTime)
{
  unsigned long Timer0Temp;
  //ET0 = 0;  //��T0�ж�
  TIM4_IER_UIE=0;
  Timer0Temp = Timer4Count; //ȡ��ǰ��ʱ�������ļ���ֵ��
  //ET0 = 1;  //ʹ��T0�ж�
  TIM4_IER_UIE=1;
  if(Timer0Temp >= OldTime)
    return Timer0Temp - OldTime;  //�������μ������Ĳ�ֵ
  else //����µ�ֵС���ϵ�ֵ��˵���������Ѿ������߹���һ����
    return (unsigned long)-1 - OldTime + Timer0Temp;
}

/********��ȷm������ʱ����*********/
void DelayMS(unsigned long ms)
{
  unsigned long __DelayTimerMs;
  __DelayTimerMs = GetTimer();
  while(SpanTime(__DelayTimerMs) < ms);
    //WDT_CountClear();

}

