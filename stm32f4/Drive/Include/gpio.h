#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

/*---------------------- GPIO���ú� ------------------------*/

#define DCLK_PIN          GPIO_Pin_5             // LED1 ����
#define GCLK_PIN          GPIO_Pin_9             // LED1 ����
#define LE_PIN            GPIO_Pin_7             // LED1 ����
#define SDI_PIN           GPIO_Pin_8             // LED1 ����
#define MBI_PORT          GPIOE                  // MBI���� GPIO�˿�
#define MBI_CLK           RCC_AHB1Periph_GPIOE   // LED1 GPIO�˿�ʱ��

/*---------------------- GPIO���ƺ� ------------------------*/

#define DCLK_PIN_H        MBI_PORT->BSRRL = DCLK_PIN;		// ����ߵ�ƽ
#define DCLK_PIN_L        MBI_PORT->BSRRH = DCLK_PIN;		// ����͵�ƽ
#define LE_PIN_H          MBI_PORT->BSRRL = LE_PIN;			// ����ߵ�ƽ
#define LE_PIN_L          MBI_PORT->BSRRH = LE_PIN; 		// ����͵�ƽ
#define SDI_PIN_H         MBI_PORT->BSRRL = SDI_PIN;		// ����ߵ�ƽ
#define SDI_PIN_L         MBI_PORT->BSRRH = SDI_PIN;		// ����͵�ƽ

#define GCLK_PIN_H        MBI_PORT->BSRRL = GCLK_PIN;		// ����ߵ�ƽ
#define GCLK_PIN_L        MBI_PORT->BSRRH = GCLK_PIN;		// ����͵�ƽ


/*---------------------- �������� ----------------------------*/

void MBI_GPIO_Init(void);	//LED��ʼ������


#endif //__GPIO_H

