/**********************************************************
* @ File name -> led.c
* @ Version   -> V1.0
* @ Date      -> 10-31-2013
* @ Brief     -> LED������غ���

 V1.*
* @ Revise    ->
**********************************************************/

#include "led.h"

/**********************************************************
* �������� ---> LED�ӿڳ�ʼ��
* ��ڲ��� ---> none
* ���ز��� ---> none 
* ����˵�� ---> none
**********************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOE, ENABLE);	//����GPIOB and GPIOE����ʱ��	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	//��ʼ��GPIOx.5---------->LEO0---->PB.5
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//GPIO��ת�ٶ�Ϊ50MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//����Ϊ�������

	GPIO_Init(GPIOB, &GPIO_InitStructure);	//��ʼ��GPIO��ؽṹ��

	GPIO_SetBits(GPIOB, GPIO_Pin_5);	//PB.5����ߵ�ƽ

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	//��ʼ��GPIOx.5---------->LEO1---->PE.5

	GPIO_Init(GPIOE, &GPIO_InitStructure);	//��ʼ��GPIO��ؽṹ��

	GPIO_SetBits(GPIOE, GPIO_Pin_5);	//PE.5����ߵ�ƽ
}

