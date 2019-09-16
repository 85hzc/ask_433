/***
	***************************************************************************
	*	@file  	gpio.c
	*	@version V1.0.0
	*	@brief   LED接口相关函数
   ***************************************************************************
   *  @description
	*
	*  初始化GPIO口，GCLK、DCLK、SDI、LE
	* 	
	***************************************************************************
***/
#include "main.h"
#include "gpio.h"


//LED init
void LED_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /*Configure GPIO pins : LED_LEFT_Pin LED_DOWN_Pin LED_UP_Pin LED_RIGHT_Pin 
                           LED1_Pin */
    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

// 函数：IO初始化
void MBI_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    //初始化引脚 MBI5153
    /*Configure GPIO pins :  */
    GPIO_InitStruct.Pin = LE_PIN|SDI_PIN|DCLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MBI_PORT, &GPIO_InitStruct);

    //初始化引脚 MBI5153
    /*Configure GPIO pins :  */
    GPIO_InitStruct.Pin = GCLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MBI_GCLK_PORT, &GPIO_InitStruct);

    //初始化引脚 MBI5153
    /*Configure GPIO pins :  */
    GPIO_InitStruct.Pin = AG_CLK_PIN|AG_DIN_PIN|AG_OE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(AG_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(MBI_GCLK_PORT, GCLK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MBI_PORT, LE_PIN|SDI_PIN|DCLK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AG_PORT, AG_CLK_PIN|AG_DIN_PIN|AG_OE_PIN, GPIO_PIN_RESET);
}

// 函数：IO初始化
void WS2801_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    //初始化引脚 MBI5153
    /*Configure GPIO pins :  */
    GPIO_InitStruct.Pin = LE_PIN|SDI_PIN|DCLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MBI_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(MBI_PORT, LE_PIN|SDI_PIN|DCLK_PIN, GPIO_PIN_RESET);
}

void OSRAM_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /*Configure GPIO pins*/
    GPIO_InitStruct.Pin = QUADRANT_EN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(QUADRANT_EN_PORT, &GPIO_InitStruct);

    /*Configure GPIO pins :  */
    GPIO_InitStruct.Pin = QT_CLK|Q0_SI|Q1_SI|Q2_SI|Q3_SI|QT_UPD;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OSRAM_PORT, &GPIO_InitStruct);
}

void MCU_GPIO_Init(void)
{

}

