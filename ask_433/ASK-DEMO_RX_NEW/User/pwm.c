/*********************************************************
//说明：pwm输出操作函数
***********************************************************/
#include "ALL_Includes.h"


/****************************************************************************************
*  名    称：void TIMER1_Init(void)
*  功    能：stm8定时器初始化
*  入口参数：无
*  出口参数：无
*  说    明：通道1输出pwm波
*  范    例：无
****************************************************************************************/
extern int  pwm_duty, pwm_duty_last, lighting_status;

void CH3_PWM_SET(int Duty_CH3);

void delay(unsigned int z) 
{
  unsigned int i,j;
  while (z--)
  {
    for(i=0;i<20;i++)
      for(j=0;j<5;j++);
  }
}

void Tim1_Init(void)
{
    //对于stm8l，默认时钟源是关闭的，需要先打开，才能配置寄存器
   //CLK_PCKENR2_PCKEN21=1;//打开定时器1时钟
   CLK_PCKENR2 = 1;

   TIM1_CCMR1_OC1M=0x06;//0b110 PWM模式1
   TIM1_CCER1_CC1P=0;//高电平有效
   TIM1_CCER1_CC1E=1;//OC1信号输出到对应引脚，PD2为OC1输出引脚

   //禁用预装载寄存器，TIM1CCR can be wiriten at any time
   TIM1_CCMR1_OC1PE=0;

   TIM1_CR1_CMS=0;//计数器计数方向取决于TIM1_CR1_DIR
   TIM1_CR1_DIR=0;//counter used as up-counter

   //初始化刹车寄存器中的MOE，只有设置此位才能使TIM1的OC1输出pwm
   TIM1_BKR_MOE=1;

   TIM1_PSCRH = 0;// Set the Prescaler value
   TIM1_PSCRL = 0;// Set the Prescaler value
   TIM1_ARRH = 0;// 初始化自动装载寄存器，决定PWM 方波的频率 
   TIM1_ARRL = 256;// 初始化自动装载寄存器，决定PWM 方波的频率 

   TIM1_CCR1H=0;
   TIM1_CCR1L=0;

   TIM1_CR1_CEN=1;//开启计数器
}
//pc   3:pwm3   4:pwm2    5: pwm1
void Pwm_Init()
{
   //PD2引脚输出PWM波
    //PD_DDR_DDR2 =1;//设置为输出
    //PD_CR1_C12 =1;//推挽输出
    PC_DDR_DDR3 =1;//设置为输出
    PC_CR1_C13 =1;//推挽输出
}

void On_Pwm()
{
  unsigned int i,tmp;
  
  for(i=1; i<=pwm_duty_last; i++)
  {
      tmp = i;
      TIM1_CCR3H=((unsigned int)(tmp))/256;
      TIM1_CCR3L=((unsigned int)(tmp))%256;
      delay(1);
  }
}

void CH1_PWM_SET(unsigned int SET_CH1,float Duty_CH1)//改变占空比
{
  float a;
  a=Duty_CH1*SET_CH1;
  TIM1_CCR1H=((unsigned int)(a))/256;
  TIM1_CCR1L=((unsigned int)(a))%256;
  TIM1_CCMR1|=0x60;
  TIM1_CCER1&=0xfd;
  TIM1_CCER1|=0x01;
  TIM1_OISR|=0x01;
}

void CH2_PWM_SET(unsigned int SET_CH2,float Duty_CH2)//改变占空比
{
  float a;
  a=Duty_CH2*SET_CH2;
  TIM1_CCR2H=((unsigned int)(a))/256;
  TIM1_CCR2L=((unsigned int)(a))%256;
  TIM1_CCMR2|=0x60;
  TIM1_CCER1&=0xdf;
  TIM1_CCER1|=0x10;
  TIM1_OISR|=0x04;
}

void CH3_PWM_SET(int a)//改变占空比
{
  int i,tmp;
  
  if(pwm_duty_last > a)//-
  {
    for(i=1; i<=pwm_duty_last-a; i++)
    {
        tmp = pwm_duty_last-i;
        TIM1_CCR3H=((unsigned int)(tmp))/256;
        TIM1_CCR3L=((unsigned int)(tmp))%256;
        delay(1);
    }
  }
  else if(pwm_duty_last < a)//+
  {
    for(i=1; i<=a-pwm_duty_last; i++)
    {
        tmp = pwm_duty_last+i;
        TIM1_CCR3H=((unsigned int)(tmp))/256;
        TIM1_CCR3L=((unsigned int)(tmp))%256;
        delay(1);
    }
  }

  /*
  TIM1_CCMR3|=0x60;
  TIM1_CCER2&=0x3d;
  TIM1_CCER2|=0x01;
  TIM1_OISR|=0x10;*/
}

void TIM1_PWM_SET()//改变周期
{
  //PWM_SET=(PWM_SET/2); //改变和边沿对齐一样的频率
  TIM1_ARRH=PWM_SET/256;
  TIM1_ARRL=PWM_SET%256;
  TIM1_CR1|=0x60;
  //CH1_PWM_SET(PWM_SET,0.2);
  //CH2_PWM_SET(PWM_SET,0.4);
  CH3_PWM_SET(pwm_duty);
  pwm_duty_last = pwm_duty;
  lighting_status = 1;

  TIM1_PSCRH =0;//定时器分频
  
  //ch3 config
  TIM1_CCMR3|=0x60;
  TIM1_CCER2&=0x3d;
  TIM1_CCER2|=0x01;
  TIM1_OISR|=0x10;

  TIM1_CR1|=0x01;
  TIM1_BKR|=0x80;
}

