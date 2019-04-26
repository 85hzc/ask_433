#ifndef __LED_H
#define __LED_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

/*---------------------- LED���ú� ------------------------*/

#define LED1_PIN             GPIO_Pin_9       		 // LED1 ����
#define LED2_PIN             GPIO_Pin_10       		 // LED1 ����     
#define LED_PORT            GPIOF                  // LED1 GPIO�˿�     
#define LED_CLK             RCC_AHB1Periph_GPIOF	 // LED1 GPIO�˿�ʱ��

/*---------------------- LED���ƺ� ------------------------*/
					
#define LED1_ON 	  GPIO_ResetBits(LED_PORT,LED1_PIN);	// ����͵�ƽ������LED1	
#define LED1_OFF 	  GPIO_SetBits(LED_PORT,LED1_PIN);		// ����ߵ�ƽ���ر�LED1	
#define LED2_ON 	  GPIO_ResetBits(LED_PORT,LED2_PIN);	// ����͵�ƽ������LED1	
#define LED2_OFF 	  GPIO_SetBits(LED_PORT,LED2_PIN);		// ����ߵ�ƽ���ر�LED1	

/*---------------------- �������� ----------------------------*/

void LED_Init(void);	//LED��ʼ������


#endif //__LED_H

