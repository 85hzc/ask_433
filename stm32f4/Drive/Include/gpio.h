#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

/*---------------------- GPIO配置宏 ------------------------*/

#define DCLK_PIN          GPIO_Pin_5             // LED1 引脚
#define GCLK_PIN          GPIO_Pin_9             // LED1 引脚
#define LE_PIN            GPIO_Pin_7             // LED1 引脚
#define SDI_PIN           GPIO_Pin_8             // LED1 引脚
#define MBI_PORT          GPIOE                  // MBI驱动 GPIO端口
#define MBI_CLK           RCC_AHB1Periph_GPIOE   // LED1 GPIO端口时钟

/*---------------------- GPIO控制宏 ------------------------*/

#define DCLK_PIN_H        MBI_PORT->BSRRL = DCLK_PIN;		// 输出高电平
#define DCLK_PIN_L        MBI_PORT->BSRRH = DCLK_PIN;		// 输出低电平
#define LE_PIN_H          MBI_PORT->BSRRL = LE_PIN;			// 输出高电平
#define LE_PIN_L          MBI_PORT->BSRRH = LE_PIN; 		// 输出低电平
#define SDI_PIN_H         MBI_PORT->BSRRL = SDI_PIN;		// 输出高电平
#define SDI_PIN_L         MBI_PORT->BSRRH = SDI_PIN;		// 输出低电平

#define GCLK_PIN_H        MBI_PORT->BSRRL = GCLK_PIN;		// 输出高电平
#define GCLK_PIN_L        MBI_PORT->BSRRH = GCLK_PIN;		// 输出低电平


/*---------------------- 函数声明 ----------------------------*/

void MBI_GPIO_Init(void);	//LED初始化函数


#endif //__GPIO_H

