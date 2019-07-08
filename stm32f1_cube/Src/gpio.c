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
    GPIO_InitTypeDef GPIO_InitStruct;

    //��ʼ������
    /*Configure GPIO pins :  */
    GPIO_InitStruct.Pin = LE_PIN|SDI_PIN|DCLK_PIN|GCLK_PIN|AG_CLK_PIN|AG_DIN_PIN|AG_OE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MBI_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(MBI_PORT, LE_PIN|SDI_PIN|DCLK_PIN|GCLK_PIN|AG_CLK_PIN|AG_DIN_PIN|AG_OE_PIN, GPIO_PIN_SET);
}

