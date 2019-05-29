#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

/*---------------------- GPIO���ú� ------------------------*/

#define DCLK_PIN          GPIO_Pin_5             // MBI dclk ����
#define GCLK_PIN          GPIO_Pin_9             // MBI gclk ����
#define LE_PIN            GPIO_Pin_7             // MBI LE ����
#define SDI_PIN           GPIO_Pin_8             // MBI SDI ����
#define MBI_PORT          GPIOE                  // MBI���� GPIO�˿�
#define MBI_CLK           RCC_AHB1Periph_GPIOE   // LED1 GPIO�˿�ʱ��

#define AG_OE_PIN         GPIO_Pin_1             // AG OE ����
#define AG_CLK_PIN        GPIO_Pin_2             // AG Clk ����
#define AG_DIN_PIN        GPIO_Pin_3             // AG Din ����


/*---------------------- GPIO���ƺ� ------------------------*/

#define DCLK_PIN_H        MBI_PORT->BSRRL = DCLK_PIN;       // ����ߵ�ƽ
#define DCLK_PIN_L        MBI_PORT->BSRRH = DCLK_PIN;       // ����͵�ƽ
#define LE_PIN_H          MBI_PORT->BSRRL = LE_PIN;         // ����ߵ�ƽ
#define LE_PIN_L          MBI_PORT->BSRRH = LE_PIN;         // ����͵�ƽ
#define SDI_PIN_H         MBI_PORT->BSRRL = SDI_PIN;        // ����ߵ�ƽ
#define SDI_PIN_L         MBI_PORT->BSRRH = SDI_PIN;        // ����͵�ƽ

#define GCLK_PIN_H        MBI_PORT->BSRRL = GCLK_PIN;       // ����ߵ�ƽ
#define GCLK_PIN_L        MBI_PORT->BSRRH = GCLK_PIN;       // ����͵�ƽ

#define AG_CLK_PIN_H      MBI_PORT->BSRRL = AG_CLK_PIN;     // ����ߵ�ƽ
#define AG_CLK_PIN_L      MBI_PORT->BSRRH = AG_CLK_PIN;     // ����͵�ƽ
#define AG_DIN_PIN_H      MBI_PORT->BSRRL = AG_DIN_PIN;     // ����ߵ�ƽ
#define AG_DIN_PIN_L      MBI_PORT->BSRRH = AG_DIN_PIN;     // ����͵�ƽ
#define AG_OE_PIN_H       MBI_PORT->BSRRL = AG_OE_PIN;      // ����ߵ�ƽ
#define AG_OE_PIN_L       MBI_PORT->BSRRH = AG_OE_PIN;      // ����͵�ƽ

/*---------------------- �������� ----------------------------*/

void MBI_GPIO_Init(void);	//LED��ʼ������


#endif //__GPIO_H

