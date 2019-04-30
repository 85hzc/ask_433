/***
	***************************************************************************
	*	@file  	main.c
	*	@version V1.0	
	*  @date    2019.1.30
	*	@author  反客科技	
	*	@brief   按键控制LED的亮灭
   ***************************************************************************
   *  @description
	*
	*	实验平台：反客STM32F407ZGT6核心板(型号：FK407M2)
	*	淘宝地址：https://shop212360197.taobao.com
	*	QQ交流群：536665479
	*
	*	功能说明：
	*
	*  1.按键每按下一次就改变LED的亮灭状态
	*	2.串口初始化时打印信息到串口助手
	* 	
	***************************************************************************
***/

#include "stm32f4xx.h"
#include "led.h"
#include "delay.h"
#include "key.h"
#include "usart.h"
#include "gpio.h"

uint8_t circuit=0;

int main(void)
{
	uint8_t key_flag = 0;	//按键标志
	
	SystemInit();
	
	Delay_Init();		//延时函数初始化
	LED_Init();			//LED初始化
	KEY_Init();			//按键IO口初始化
	Usart_Config();	// USART初始化函数
	MBI_GPIO_Init();//初始化MBI驱动pin
	
	printf("system start.\r\n");
	
	SystemCoreClockUpdate();
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//系统中断优先级分组2
#if 0
	Pulse_output(1000,8000);//1KHZ,8000个
#else
#if 0
	TIM14_PWM_Init(100-1,21-1);//168M/42=4Mhz的计数频率,重装载值100，所以PWM频率为 4M/100=40Khz.
	TIM_SetCompare1(TIM14,50);
#else
	//TIM1_PWM_Init(100-1,14-1);//(75-1,6-1); OK
	TIM1_PWM_Init(100-1,6-1); //OK
	TIM_SetCompare1(TIM1,50);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
#endif
#endif

	//printf("befor reset\r\n");
	//Delay_ms(5000);
	///soft_reset();
	//printf("after reset\r\n");
	//Delay_ms(5000);
	MBI_Init();
	printf("after mbi init\r\n");
	//Delay_ms(2000);
	//printf("mbi init reset\r\n");
	//circuit++;

	//vsync();
	//MBI5153();
	
	while (1)
	{

		//soft_reset();
		//MBI_Init();
		//circuit = (circuit != 0)?0:15;
		//printf("circuit:%d\r\n",circuit);
		MBI5153();
		printf("circuit:%d\r\n",circuit%16);
		circuit++;
		Delay_ms(50);
		/*
		LED1_ON;
		LED2_OFF;
		Delay_ms(500);
		LED1_OFF;
		LED2_ON;
		Delay_ms(500);*/
#if 0
		//每次按键按下对标志进行取反
		if( KEY_Scan() == KEY_ON )
		{
			key_flag = ~key_flag;
		}
		
		//根据按键标志进行LED的亮灭操作
		if(key_flag == 0)
		{
			LED1_ON;
		}
		else
		{
			LED1_OFF;
		}
#endif
	}
}

