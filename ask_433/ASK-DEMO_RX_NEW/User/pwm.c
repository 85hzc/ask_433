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

void Pwm_Init()
{
   //PD2引脚输出PWM波
    PD_DDR_DDR2 =1;//设置为输出
    PD_CR1_C12 =1;//推挽输出
}

void Set_Pwm(unsigned char dutyCycle)
{
    TIM1_CCR1H = 0;
    TIM1_CCR1L = dutyCycle;
}

