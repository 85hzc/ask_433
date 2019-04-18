#ifndef __KEY_H
#define __KEY_H

#include "stm32f4xx.h"
#include "delay.h"

#define	KEY_ON	 1		//按键按下
#define	KEY_OFF	 0		//按键放开

/*---------------------- 按键配置宏 ------------------------*/

#define KEY_PIN           GPIO_Pin_4        		 // KEY 引脚      
#define KEY_PORT          GPIOE                     // KEY GPIO端口     
#define KEY_CLK           RCC_AHB1Periph_GPIOE	    // KEY GPIO端口时钟

/*---------------------- 函数声明 ----------------------------*/

void 	KEY_Init(void);	//按键IO口初始化函数
u8		KEY_Scan(void);   //按键扫描

#endif //__KEY_H


