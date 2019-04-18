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

int main(void)
{
	u8 key_flag = 0;	//������־
	

	Delay_Init();		//��ʱ������ʼ��
	LED_Init();			//LED��ʼ��
	KEY_Init();			//����IO�ڳ�ʼ��
	Usart_Config();	// USART��ʼ������
	
	printf("FK407M2���İ����\r\n");
	
	while (1)
	{	
		Delay_ms(500);
		printf("while\r\n");
	  //printf("FK407M2���İ����\r\n");
		/*
		LED1_ON;
		LED2_OFF;
		Delay_ms(500);
		LED1_OFF;
		LED2_ON;
		Delay_ms(500);*/
		
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
	}	
}





