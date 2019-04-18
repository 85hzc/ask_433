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

int main(void)
{
	u8 key_flag = 0;	//按键标志
	

	Delay_Init();		//延时函数初始化
	LED_Init();			//LED初始化
	KEY_Init();			//按键IO口初始化
	Usart_Config();	// USART初始化函数
	
	printf("FK407M2核心板测试\r\n");
	
	while (1)
	{	
		Delay_ms(500);
		printf("while\r\n");
	  //printf("FK407M2核心板测试\r\n");
		/*
		LED1_ON;
		LED2_OFF;
		Delay_ms(500);
		LED1_OFF;
		LED2_ON;
		Delay_ms(500);*/
		
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
	}	
}





