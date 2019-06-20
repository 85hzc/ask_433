/*********************************************************
//˵����pwm�����������
***********************************************************/
#include "ALL_Includes.h"


/****************************************************************************************
*  ��    �ƣ�void TIMER1_Init(void)
*  ��    �ܣ�stm8��ʱ����ʼ��
*  ��ڲ�������
*  ���ڲ�������
*  ˵    ����ͨ��1���pwm��
*  ��    ������
****************************************************************************************/
void Tim1_Init(void)
{
    //����stm8l��Ĭ��ʱ��Դ�ǹرյģ���Ҫ�ȴ򿪣��������üĴ���
   //CLK_PCKENR2_PCKEN21=1;//�򿪶�ʱ��1ʱ��
   CLK_PCKENR2 = 1;

   TIM1_CCMR1_OC1M=0x06;//0b110 PWMģʽ1
   TIM1_CCER1_CC1P=0;//�ߵ�ƽ��Ч
   TIM1_CCER1_CC1E=1;//OC1�ź��������Ӧ���ţ�PD2ΪOC1�������

   //����Ԥװ�ؼĴ�����TIM1CCR can be wiriten at any time
   TIM1_CCMR1_OC1PE=0;

   TIM1_CR1_CMS=0;//��������������ȡ����TIM1_CR1_DIR
   TIM1_CR1_DIR=0;//counter used as up-counter

   //��ʼ��ɲ���Ĵ����е�MOE��ֻ�����ô�λ����ʹTIM1��OC1���pwm
   TIM1_BKR_MOE=1;

   TIM1_PSCRH = 0;// Set the Prescaler value
   TIM1_PSCRL = 0;// Set the Prescaler value
   TIM1_ARRH = 0;// ��ʼ���Զ�װ�ؼĴ���������PWM ������Ƶ�� 
   TIM1_ARRL = 256;// ��ʼ���Զ�װ�ؼĴ���������PWM ������Ƶ�� 

   TIM1_CCR1H=0;
   TIM1_CCR1L=0;

   TIM1_CR1_CEN=1;//����������
}

void Pwm_Init()
{
   //PD2�������PWM��
    PD_DDR_DDR2 =1;//����Ϊ���
    PD_CR1_C12 =1;//�������
}

void Set_Pwm(unsigned char dutyCycle)
{
    TIM1_CCR1H = 0;
    TIM1_CCR1L = dutyCycle;
}

