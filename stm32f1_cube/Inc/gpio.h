#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f1xx.h"
#include "stm32f1xx_hal_gpio.h"

/*---------------------- GPIO���ú� ------------------------*/

#define LED_Pin           GPIO_PIN_0
#define LED_GPIO_Port     GPIOA

#define INT_Pin           GPIO_PIN_5
#define INT_GPIO_Port     GPIOC

#define IR_IN_Pin         GPIO_PIN_1
#define IR_IN_GPIO_Port   GPIOB



#define SPI1_SCK          GPIO_PIN_5
#define SPI1_MISO         GPIO_PIN_6
#define SPI1_MOSI         GPIO_PIN_7
#define SPI1_PORT         GPIOB


/////MBI GPIO
#define DCLK_PIN          GPIO_PIN_5             // MBI dclk ����
#define GCLK_PIN          GPIO_PIN_1             // MBI gclk ����
#define LE_PIN            GPIO_PIN_7             // MBI LE ����
#define SDI_PIN           GPIO_PIN_8             // MBI SDI ����
#define MBI_PORT          GPIOB                  // MBI���� GPIO�˿�

#define AG_OE_PIN         GPIO_PIN_2             // AG OE ����
#define AG_CLK_PIN        GPIO_PIN_3             // AG Clk ����
#define AG_DIN_PIN        GPIO_PIN_4             // AG Din ����

/////OSRAM GPIO
#define QUADRANT_EN       GPIO_PIN_1
#define QUADRANT_EN_PORT  GPIOA

#define SCL_GPIO_Port     GPIOB
#define SDA_GPIO_Port     GPIOB
#define SCL_Pin           GPIO_PIN_6
#define SDA_Pin           GPIO_PIN_7

#define Q0_SI             GPIO_PIN_1
#define Q0_UPD            GPIO_PIN_2
#define Q1_SI             GPIO_PIN_3
#define Q1_UPD            GPIO_PIN_4
#define Q2_SI             GPIO_PIN_8
#define Q2_UPD            GPIO_PIN_9
#define Q3_SI             GPIO_PIN_10
#define QT_CLK            GPIO_PIN_11
#define Q3_UPD            GPIO_PIN_12
#define OSRAM_PORT        GPIOB


/*---------------------- GPIO���ƺ� ------------------------*/

#define DCLK_PIN_H        MBI_PORT->BSRR = DCLK_PIN;                       // ����ߵ�ƽ
#define DCLK_PIN_L        MBI_PORT->BSRR = (uint32_t)DCLK_PIN << 16;       // ����͵�ƽ
#define LE_PIN_H          MBI_PORT->BSRR = LE_PIN;                         // ����ߵ�ƽ
#define LE_PIN_L          MBI_PORT->BSRR = (uint32_t)LE_PIN << 16;         // ����͵�ƽ
#define SDI_PIN_H         MBI_PORT->BSRR = SDI_PIN;                        // ����ߵ�ƽ
#define SDI_PIN_L         MBI_PORT->BSRR = (uint32_t)SDI_PIN << 16;        // ����͵�ƽ

#define GCLK_PIN_H        MBI_PORT->BSRR = GCLK_PIN;                       // ����ߵ�ƽ
#define GCLK_PIN_L        MBI_PORT->BSRR = (uint32_t)GCLK_PIN << 16;       // ����͵�ƽ

#define AG_CLK_PIN_H      MBI_PORT->BSRR = AG_CLK_PIN;                     // ����ߵ�ƽ
#define AG_CLK_PIN_L      MBI_PORT->BSRR = (uint32_t)AG_CLK_PIN << 16;     // ����͵�ƽ
#define AG_DIN_PIN_H      MBI_PORT->BSRR = AG_DIN_PIN;                     // ����ߵ�ƽ
#define AG_DIN_PIN_L      MBI_PORT->BSRR = (uint32_t)AG_DIN_PIN << 16;     // ����͵�ƽ
#define AG_OE_PIN_H       MBI_PORT->BSRR = AG_OE_PIN;                      // ����ߵ�ƽ
#define AG_OE_PIN_L       MBI_PORT->BSRR = (uint32_t)AG_OE_PIN << 16;      // ����͵�ƽ


#define Q0_SI_H           OSRAM_PORT->BSRR = Q0_SI;                        // ����ߵ�ƽ
#define Q0_SI_L           OSRAM_PORT->BSRR = (uint32_t)Q0_SI << 16;        // ����͵�ƽ  
#define Q0_UPD_H          OSRAM_PORT->BSRR = Q0_UPD;                       // ����ߵ�ƽ
#define Q0_UPD_L          OSRAM_PORT->BSRR = (uint32_t)Q0_UPD << 16;       // ����͵�ƽ 
#define Q1_SI_H           OSRAM_PORT->BSRR = Q1_SI;                        // ����ߵ�ƽ
#define Q1_SI_L           OSRAM_PORT->BSRR = (uint32_t)Q1_SI << 16;        // ����͵�ƽ 
#define Q1_UPD_H          OSRAM_PORT->BSRR = Q1_UPD;                       // ����ߵ�ƽ
#define Q1_UPD_L          OSRAM_PORT->BSRR = (uint32_t)Q1_UPD << 16;       // ����͵�ƽ  
#define Q2_SI_H           OSRAM_PORT->BSRR = Q2_SI;                        // ����ߵ�ƽ
#define Q2_SI_L           OSRAM_PORT->BSRR = (uint32_t)Q2_SI << 16;        // ����͵�ƽ  
#define Q2_UPD_H          OSRAM_PORT->BSRR = Q2_UPD;                       // ����ߵ�ƽ
#define Q2_UPD_L          OSRAM_PORT->BSRR = (uint32_t)Q2_UPD << 16;       // ����͵�ƽ 
#define Q3_SI_H           OSRAM_PORT->BSRR = Q3_SI;                        // ����ߵ�ƽ
#define Q3_SI_L           OSRAM_PORT->BSRR = (uint32_t)Q3_SI << 16;        // ����͵�ƽ 
#define Q3_UPD_H          OSRAM_PORT->BSRR = Q3_UPD;                       // ����ߵ�ƽ
#define Q3_UPD_L          OSRAM_PORT->BSRR = (uint32_t)Q3_UPD << 16;       // ����͵�ƽ 
#define QT_CLK_H          OSRAM_PORT->BSRR = QT_CLK;                       // ����ߵ�ƽ
#define QT_CLK_L          OSRAM_PORT->BSRR = (uint32_t)QT_CLK << 16;       // ����͵�ƽ 


/*---------------------- �������� ----------------------------*/

void MBI_GPIO_Init(void);                                                  //GPIO��ʼ������
void OSRAM_GPIO_Init(void);                                                //GPIO��ʼ������


#endif //__GPIO_H

