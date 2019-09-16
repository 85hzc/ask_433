/**********************************************************
* @ File name -> led.c
* @ Version   -> V1.0
* @ Date      -> 10-31-2013
* @ Brief     -> LED控制相关函数

 V1.*
* @ Revise    ->
**********************************************************/

#include "led.h"

/**********************************************************
* 函数功能 ---> LED接口初始化
* 入口参数 ---> none
* 返回参数 ---> none 
* 功能说明 ---> none
**********************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOE, ENABLE);	//开启GPIOB and GPIOE外设时钟	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	//初始化GPIOx.5---------->LEO0---->PB.5
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//GPIO翻转速度为50MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//设置为推挽输出

	GPIO_Init(GPIOB, &GPIO_InitStructure);	//初始化GPIO相关结构体

	GPIO_SetBits(GPIOB, GPIO_Pin_5);	//PB.5输出高电平

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	//初始化GPIOx.5---------->LEO1---->PE.5

	GPIO_Init(GPIOE, &GPIO_InitStructure);	//初始化GPIO相关结构体

	GPIO_SetBits(GPIOE, GPIO_Pin_5);	//PE.5输出高电平
}

