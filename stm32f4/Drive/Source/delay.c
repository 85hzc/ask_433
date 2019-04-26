/***
	***************************************************************************
	*	@file  	delay.c
	*	@brief   delay�ӿ���غ���
   ***************************************************************************
   *  @description
	*
	*  SysTick��ʱ������Ϊ1ms�жϣ�ʵ�ֺ�����ʱ
	* 	
	***************************************************************************
***/

#include "delay.h"

static u32 TimingDelay;  //��������

#if 1
#define delay(a) Delay_ms(a)
//#else
void delay_ns(u32 time)
{
	//u32 tt = time;
	while(time--) {
		__ASM("nop");
		__ASM("nop");
		__ASM("nop");
		__ASM("nop");
	}
}
#endif

//	��������ʱ��ʼ��
//	˵�������� SysTick Ϊ1ms�жϣ���������ʱ��
//
void Delay_Init(void)
{
	SysTick_Config(SystemCoreClock / (1000*1000));  //����SysTickʱ��Ϊ1ms�ж�
}

//	��������ʱ����
//	˵������ SysTick �жϷ������ﱻ����
//
void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0)
	{ 
		TimingDelay--;
	}
}

//	������������ʱ
// ������nTime - ��ʱʱ�䣬��λms
//	˵����ÿ�ε��ö������¸�TimingDelay��ֵ��ʵ�� n �������ʱ�������ʱ 4294967295 ms��	
//
void Delay_ms(u32 nTime)
{ 
	TimingDelay = nTime;

	while(TimingDelay != 0);
}




