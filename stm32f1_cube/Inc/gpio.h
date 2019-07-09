#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f1xx.h"
#include "stm32f1xx_hal_gpio.h"

/*---------------------- GPIO���ú� ------------------------*/

#define LED_LEFT_Pin        GPIO_PIN_0
#define LED_LEFT_GPIO_Port  GPIOA
#define LED_DOWN_Pin        GPIO_PIN_6
#define LED_DOWN_GPIO_Port  GPIOA
#define LED_UP_Pin          GPIO_PIN_1
#define LED_UP_GPIO_Port    GPIOA
#define LED_RIGHT_Pin       GPIO_PIN_7
#define LED_RIGHT_GPIO_Port GPIOA

#define INT_Pin             GPIO_PIN_5
#define INT_GPIO_Port       GPIOA
/*
#define USART1_GPIO_Port    GPIOA
#define USART1_TX_Pin       GPIO_PIN_9
#define USART1_RX_Pin       GPIO_PIN_10
*/

#define SCL_GPIO_Port       GPIOC
#define SDA_GPIO_Port       GPIOC
#define SCL1_Pin            GPIO_PIN_10
#define SDA1_Pin            GPIO_PIN_11
#define SCL2_Pin            GPIO_PIN_8
#define SDA2_Pin            GPIO_PIN_9
#define SCL3_Pin            GPIO_PIN_6
#define SDA3_Pin            GPIO_PIN_7



#define DCLK_PIN          GPIO_PIN_5             // MBI dclk ����
#define GCLK_PIN          GPIO_PIN_1             // MBI gclk ����
#define LE_PIN            GPIO_PIN_7             // MBI LE ����
#define SDI_PIN           GPIO_PIN_8             // MBI SDI ����
#define MBI_PORT          GPIOB                  // MBI���� GPIO�˿�

#define AG_OE_PIN         GPIO_PIN_2             // AG OE ����
#define AG_CLK_PIN        GPIO_PIN_3             // AG Clk ����
#define AG_DIN_PIN        GPIO_PIN_4             // AG Din ����


/*---------------------- GPIO���ƺ� ------------------------*/

#define DCLK_PIN_H        MBI_PORT->BSRR = DCLK_PIN;       // ����ߵ�ƽ
#define DCLK_PIN_L        MBI_PORT->BSRR = (uint32_t)DCLK_PIN << 16;       // ����͵�ƽ
#define LE_PIN_H          MBI_PORT->BSRR = LE_PIN;         // ����ߵ�ƽ
#define LE_PIN_L          MBI_PORT->BSRR = (uint32_t)LE_PIN << 16;         // ����͵�ƽ
#define SDI_PIN_H         MBI_PORT->BSRR = SDI_PIN;        // ����ߵ�ƽ
#define SDI_PIN_L         MBI_PORT->BSRR = (uint32_t)SDI_PIN << 16;        // ����͵�ƽ

#define GCLK_PIN_H        MBI_PORT->BSRR = GCLK_PIN;       // ����ߵ�ƽ
#define GCLK_PIN_L        MBI_PORT->BSRR = (uint32_t)GCLK_PIN << 16;       // ����͵�ƽ

#define AG_CLK_PIN_H      MBI_PORT->BSRR = AG_CLK_PIN;     // ����ߵ�ƽ
#define AG_CLK_PIN_L      MBI_PORT->BSRR = (uint32_t)AG_CLK_PIN << 16;     // ����͵�ƽ
#define AG_DIN_PIN_H      MBI_PORT->BSRR = AG_DIN_PIN;     // ����ߵ�ƽ
#define AG_DIN_PIN_L      MBI_PORT->BSRR = (uint32_t)AG_DIN_PIN << 16;     // ����͵�ƽ
#define AG_OE_PIN_H       MBI_PORT->BSRR = AG_OE_PIN;      // ����ߵ�ƽ
#define AG_OE_PIN_L       MBI_PORT->BSRR = (uint32_t)AG_OE_PIN << 16;      // ����͵�ƽ

/*---------------------- �������� ----------------------------*/

void MBI_GPIO_Init(void);	//LED��ʼ������


#endif //__GPIO_H

