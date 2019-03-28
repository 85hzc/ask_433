//=============================================================================
//文件名称：main.c
//功能概要：STM32F051R8例程—LED测试程序
//版权所有：源地工作室www.vcc-gnd.com
//淘宝网店：http://vcc-gnd.taobao.com
//更新时间：2013-11-20
//调试方式：J-Link OB ARM SWD
//=============================================================================

//头文件
#include "stm32f0xx.h"
#include "BSP_LED.h"			// LED驱动模块头文件

//变量定义


//函数声明
//void GPIO_Configuration(void);
//=============================================================================
//文件名称：Delay
//功能概要：延时函数
//参数说明：无
//函数返回：无
//=============================================================================
void  Delay (uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}


//=============================================================================
//文件名称：main
//功能概要：主函数
//参数说明：无
//函数返回：int
//=============================================================================
int main(void)
{
//	unsigned char i;
	 /* LED初始化 */
  LED_Init();

// 	GPIO_SetBits(GPIOA,GPIO_Pin_1);
// 	GPIO_SetBits(GPIOA,GPIO_Pin_2);
// 	 
// 	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
//   for(i=0;i<6;i++) Delay(0xfffff);
// 		GPIO_SetBits(GPIOC,GPIO_Pin_13);
//   for(i=0;i<6;i++) Delay(0xfffff);
	
  while(1)
  {     	
 //  LED1_TURN;
//   LED2_TURN; 
 //  LED3_TURN;
 //  LED4_TURN;
 //  LED5_TURN;
 //  LED6_TURN;
// 		GPIO_SetBits(GPIOC,GPIO_Pin_13);
//    Delay(0xfffff);
// 		GPIO_ResetBits(GPIOC,GPIO_Pin_13);
//    Delay(0xfffff);
		
		GPIO_Write(GPIOA,0XFFFF);
		GPIO_Write(GPIOB,0XFFFF);
		GPIO_Write(GPIOC,0XFFFF);
		GPIO_Write(GPIOD,0XFFFF);
		GPIO_Write(GPIOF,0XFFFF);
		Delay(0x200000);
		
		GPIO_Write(GPIOA,0X0000);
		GPIO_Write(GPIOB,0X0000);
		GPIO_Write(GPIOC,0X0000);
		GPIO_Write(GPIOD,0X0000);
		GPIO_Write(GPIOF,0X0000);
		Delay(0x200000); 
    }
}


/*****END OF FILE****/
