#ifndef __KEY_H
#define __KEY_H

#include "stm32f4xx.h"
#include "delay.h"

#define	KEY_ON	 1		//��������
#define	KEY_OFF	 0		//�����ſ�

/*---------------------- �������ú� ------------------------*/

#define KEY_PIN           GPIO_Pin_4        		 // KEY ����      
#define KEY_PORT          GPIOE                     // KEY GPIO�˿�     
#define KEY_CLK           RCC_AHB1Periph_GPIOE	    // KEY GPIO�˿�ʱ��

/*---------------------- �������� ----------------------------*/

void 	KEY_Init(void);	//����IO�ڳ�ʼ������
u8		KEY_Scan(void);   //����ɨ��

#endif //__KEY_H


