/***
	***************************************************************************
	*	@file  	gpio.c
	*	@version V1.0.0
	*	@brief   LED�ӿ���غ���
   ***************************************************************************
   *  @description
	*
	*  ��ʼ��GPIO�ڣ�GCLK��DCLK��SDI��LE
	* 	
	***************************************************************************
***/

#include "gpio.h"

// ������IO��ʼ��
void MBI_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; //����ṹ��
	RCC_AHB1PeriphClockCmd (MBI_CLK, ENABLE); 	//��ʼ��GPIOGʱ��

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;   	//���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  	//�������
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;  	//
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	//����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 	//�ٶ�ѡ��
	
	//��ʼ������
	GPIO_InitStructure.GPIO_Pin = LE_PIN|SDI_PIN|DCLK_PIN;//|GCLK_PIN;
	GPIO_Init(MBI_PORT, &GPIO_InitStructure);
	
	GPIO_ResetBits(MBI_PORT,LE_PIN|SDI_PIN|DCLK_PIN);//|GCLK_PIN);  //����͵�ƽ
}

