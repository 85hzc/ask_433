#ifndef __LED_H
#define __LED_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

/*---------------------- LED配置宏 ------------------------*/

#define LED1_PIN             GPIO_Pin_9       		 // LED1 引脚
#define LED2_PIN             GPIO_Pin_10       		 // LED1 引脚     
#define LED_PORT            GPIOF                  // LED1 GPIO端口     
#define LED_CLK             RCC_AHB1Periph_GPIOF	 // LED1 GPIO端口时钟

/*---------------------- LED控制宏 ------------------------*/
					
#define LED1_ON 	  GPIO_ResetBits(LED_PORT,LED1_PIN);	// 输出低电平，点亮LED1	
#define LED1_OFF 	  GPIO_SetBits(LED_PORT,LED1_PIN);		// 输出高电平，关闭LED1	
#define LED2_ON 	  GPIO_ResetBits(LED_PORT,LED2_PIN);	// 输出低电平，点亮LED1	
#define LED2_OFF 	  GPIO_SetBits(LED_PORT,LED2_PIN);		// 输出高电平，关闭LED1	

/*---------------------- 函数声明 ----------------------------*/

void LED_Init(void);	//LED初始化函数


#endif //__LED_H

