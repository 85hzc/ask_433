/***
	***************************************************************************
	*	@file  	main.c
	*	@version V1.0	
	*  @date    2019.1.30
	*	@author  ���ͿƼ�	
	*	@brief   ��������LED������
   ***************************************************************************
   *  @description
	*
	*	ʵ��ƽ̨������STM32F407ZGT6���İ�(�ͺţ�FK407M2)
	*	�Ա���ַ��https://shop212360197.taobao.com
	*	QQ����Ⱥ��536665479
	*
	*	����˵����
	*
	*  1.����ÿ����һ�ξ͸ı�LED������״̬
	*	2.���ڳ�ʼ��ʱ��ӡ��Ϣ����������
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
	uint8_t key_flag = 0;	//������־
	
	SystemInit();
	
	Delay_Init();		//��ʱ������ʼ��
	LED_Init();			//LED��ʼ��
	KEY_Init();			//����IO�ڳ�ʼ��
	Usart_Config();	// USART��ʼ������
	MBI_GPIO_Init();//��ʼ��MBI����pin
	
	printf("system start.\r\n");
	
	SystemCoreClockUpdate();
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//ϵͳ�ж����ȼ�����2
#if 0
	Pulse_output(1000,8000);//1KHZ,8000��
#else
#if 0
	TIM14_PWM_Init(100-1,21-1);//168M/42=4Mhz�ļ���Ƶ��,��װ��ֵ100������PWMƵ��Ϊ 4M/100=40Khz.
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
		//ÿ�ΰ������¶Ա�־����ȡ��
		if( KEY_Scan() == KEY_ON )
		{
			key_flag = ~key_flag;
		}
		
		//���ݰ�����־����LED���������
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

