#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f1xx.h"
#include "stm32f1xx_hal_gpio.h"
#include "main.h"

/*---------------------- GPIO���ú� ------------------------*/

#define LED_Pin           GPIO_PIN_1
#define LED_GPIO_Port     GPIOB

#define PWM_Pin           GPIO_PIN_11
#define PWM_GPIO_Port     GPIOA

#define FAN_Pin           GPIO_PIN_0
#define FAN_GPIO_Port     GPIOB

#define IR_IN_Pin         GPIO_PIN_7  //TIM2 CH1
#define IR_IN_GPIO_Port   GPIOA

#define UART_ble_TX       GPIO_PIN_10
#define UART_ble_RX       GPIO_PIN_11
#define UART_ble_Port     GPIOB
#define UART_wifi_TX      GPIO_PIN_9
#define UART_wifi_RX      GPIO_PIN_10
#define UART_wifi_Port    GPIOA
#define UART_Debug_TX     GPIO_PIN_2
#define UART_Debug_RX     GPIO_PIN_3
#define UART_Debug_Port   GPIOA

#define SD_DET_GPIO       GPIO_PIN_8
#define SD_DET_PORT       GPIOA

#define SD_CS_GPIO        GPIO_PIN_12
#define SD_CS_PORT        GPIOB

#define SPI1_SCK          GPIO_PIN_5
#define SPI1_MISO         GPIO_PIN_6
#define SPI1_MOSI         GPIO_PIN_7
#define SPI1_PORT         GPIOA

#define SPI2_SCK          GPIO_PIN_13
#define SPI2_MISO         GPIO_PIN_14
#define SPI2_MOSI         GPIO_PIN_15
#define SPI2_PORT         GPIOB

//#if(PROJECTOR_MBI5153)
/////MBI GPIO
#define DCLK_PIN          GPIO_PIN_6             // MBI dclk ����
#define LE_PIN            GPIO_PIN_7             // MBI LE ����
#define SDI_PIN           GPIO_PIN_5             // MBI SDI ����
#define MBI_PORT          GPIOA                  // MBI���� GPIO�˿�

#define GCLK_PIN          GPIO_PIN_0             // MBI gclk ����
#define MBI_GCLK_PORT     GPIOB                  // MBI���� GPIO�˿�

#define AG_OE_PIN         GPIO_PIN_15             // AG OE ����
#define AG_CLK_PIN        GPIO_PIN_14             // AG Clk ����
#define AG_DIN_PIN        GPIO_PIN_13             // AG Din ����
#define AG_PORT           GPIOC                   // AG���� GPIO�˿�


#define DCLK_PIN_H        MBI_PORT->BSRR = DCLK_PIN;                       // ����ߵ�ƽ
#define DCLK_PIN_L        MBI_PORT->BSRR = (uint32_t)DCLK_PIN << 16;       // ����͵�ƽ
#define LE_PIN_H          MBI_PORT->BSRR = LE_PIN;                         // ����ߵ�ƽ
#define LE_PIN_L          MBI_PORT->BSRR = (uint32_t)LE_PIN << 16;         // ����͵�ƽ
#define SDI_PIN_H         MBI_PORT->BSRR = SDI_PIN;                        // ����ߵ�ƽ
#define SDI_PIN_L         MBI_PORT->BSRR = (uint32_t)SDI_PIN << 16;        // ����͵�ƽ

#define GCLK_PIN_H        MBI_GCLK_PORT->BSRR = GCLK_PIN;                       // ����ߵ�ƽ
#define GCLK_PIN_L        MBI_GCLK_PORT->BSRR = (uint32_t)GCLK_PIN << 16;       // ����͵�ƽ

#define AG_CLK_PIN_H      AG_PORT->BSRR = AG_CLK_PIN;                     // ����ߵ�ƽ
#define AG_CLK_PIN_L      AG_PORT->BSRR = (uint32_t)AG_CLK_PIN << 16;     // ����͵�ƽ
#define AG_DIN_PIN_H      AG_PORT->BSRR = AG_DIN_PIN;                     // ����ߵ�ƽ
#define AG_DIN_PIN_L      AG_PORT->BSRR = (uint32_t)AG_DIN_PIN << 16;     // ����͵�ƽ
#define AG_OE_PIN_H       AG_PORT->BSRR = AG_OE_PIN;                      // ����ߵ�ƽ
#define AG_OE_PIN_L       AG_PORT->BSRR = (uint32_t)AG_OE_PIN << 16;      // ����͵�ƽ


//#elif(PROJECTOR_OSRAM)
/////OSRAM GPIO

#define SCL_Pin           GPIO_PIN_6
#define SDA_Pin           GPIO_PIN_7
#define SCL_GPIO_Port     GPIOB
#define SDA_GPIO_Port     GPIOB

#define OSRAM_EN          GPIO_PIN_13
#define OSRAM_EN_PORT     GPIOC

#define QT_CLK            GPIO_PIN_15
#define OSRAM_PORT_CLK    GPIOC

#define QT_UPD            GPIO_PIN_0
#define OSRAM_PORT_UPD    GPIOD

#define Q0_SI             GPIO_PIN_14
#define OSRAM_PORT_SI0    GPIOC
#define Q1_SI             GPIO_PIN_0
#define OSRAM_PORT_SI1    GPIOA
#define Q2_SI             GPIO_PIN_5
#define OSRAM_PORT_SI2    GPIOA
#define Q3_SI             GPIO_PIN_6
#define OSRAM_PORT_SI3    GPIOA


#define Q0_SI_H           OSRAM_PORT_SI0->BSRR = Q0_SI;                        // ����ߵ�ƽ
#define Q0_SI_L           OSRAM_PORT_SI0->BSRR = (uint32_t)Q0_SI << 16;        // ����͵�ƽ 
#define Q1_SI_H           OSRAM_PORT_SI1->BSRR = Q1_SI;                        // ����ߵ�ƽ
#define Q1_SI_L           OSRAM_PORT_SI1->BSRR = (uint32_t)Q1_SI << 16;        // ����͵�ƽ
#define Q2_SI_H           OSRAM_PORT_SI2->BSRR = Q2_SI;                        // ����ߵ�ƽ
#define Q2_SI_L           OSRAM_PORT_SI2->BSRR = (uint32_t)Q2_SI << 16;        // ����͵�ƽ
#define Q3_SI_H           OSRAM_PORT_SI3->BSRR = Q3_SI;                        // ����ߵ�ƽ
#define Q3_SI_L           OSRAM_PORT_SI3->BSRR = (uint32_t)Q3_SI << 16;        // ����͵�ƽ
#define QT_UPD_H          OSRAM_PORT_UPD->BSRR = QT_UPD;                       // ����ߵ�ƽ
#define QT_UPD_L          OSRAM_PORT_UPD->BSRR = (uint32_t)QT_UPD << 16;       // ����͵�ƽ
#define QT_CLK_H          OSRAM_PORT_CLK->BSRR = QT_CLK;                       // ����ߵ�ƽ
#define QT_CLK_L          OSRAM_PORT_CLK->BSRR = (uint32_t)QT_CLK << 16;       // ����͵�ƽ
#define EN_H              OSRAM_EN_PORT->BSRR = OSRAM_EN;                      // ����ߵ�ƽ
#define EN_L              OSRAM_EN_PORT->BSRR = (uint32_t)OSRAM_EN << 16;      // ����͵�ƽ

#define MOTOR_IN_Pin      GPIO_PIN_8
#define MOTOR_IN_Port     GPIOB
#define MOTOR_OUT_Pin     GPIO_PIN_9
#define MOTOR_OUT_Port    GPIOB

#define MOTOR_AIN1_Pin    GPIO_PIN_15
#define MOTOR_AIN1_Port   GPIOA
#define MOTOR_AIN2_Pin    GPIO_PIN_3
#define MOTOR_AIN2_Port   GPIOB
#define MOTOR_BIN1_Pin    GPIO_PIN_4
#define MOTOR_BIN1_Port   GPIOB
#define MOTOR_BIN2_Pin    GPIO_PIN_5
#define MOTOR_BIN2_Port   GPIOB

//#elif(PROJECTOR_MCUGPIO)

#define ROW_0             GPIO_PIN_0
#define ROW_1             GPIO_PIN_1
#define ROW_2             GPIO_PIN_2
#define ROW_3             GPIO_PIN_3
#define ROW_4             GPIO_PIN_4
#define ROW_5             GPIO_PIN_5
#define ROW_6             GPIO_PIN_6
#define ROW_7             GPIO_PIN_7
#define ROW_8             GPIO_PIN_8
#define ROW_9             GPIO_PIN_9
#define ROW_10            GPIO_PIN_10
#define ROW_11            GPIO_PIN_11
#define ROW_12            GPIO_PIN_12
#define ROW_13            GPIO_PIN_13
#define ROW_14            GPIO_PIN_14
#define ROW_15            GPIO_PIN_15
#define ROW_PORT          GPIOD

#define ROW_PULL_UP(a)    ROW_PORT->BSRR = a;                          // ����ߵ�ƽ
#define ROW_PULL_DOWN(a)  ROW_PORT->BSRR = (uint32_t)a << 16;          // ����͵�ƽ

#define COLUMN_0          GPIO_PIN_0
#define COLUMN_1          GPIO_PIN_1
#define COLUMN_2          GPIO_PIN_2
#define COLUMN_3          GPIO_PIN_3
#define COLUMN_4          GPIO_PIN_4
#define COLUMN_5          GPIO_PIN_5
#define COLUMN_6          GPIO_PIN_6
#define COLUMN_7          GPIO_PIN_7
#define COLUMN_8          GPIO_PIN_8
#define COLUMN_9          GPIO_PIN_9
#define COLUMN_10         GPIO_PIN_10
#define COLUMN_11         GPIO_PIN_11
#define COLUMN_12         GPIO_PIN_12
#define COLUMN_13         GPIO_PIN_13
#define COLUMN_14         GPIO_PIN_14
#define COLUMN_15         GPIO_PIN_15
#define COL_PORT          GPIOC

#define COL_PULL_UP(a)    COL_PORT->BSRR = a;                          // ����ߵ�ƽ
#define COL_PULL_DOWN(a)  COL_PORT->BSRR = (uint32_t)a << 16;          // ����͵�ƽ


//#if(PROJECTOR_CUBE)

#define CU_P1_PIN         GPIO_PIN_7
#define CU_P2_PIN         GPIO_PIN_0
#define CU_P3_PIN         GPIO_PIN_6
#define CU_P4_PIN         GPIO_PIN_5
#define WS_PORT           GPIOA

#define CU_P1_PIN_H       WS_PORT->BSRR = CU_P1_PIN;                        // ����ߵ�ƽ
#define CU_P1_PIN_L       WS_PORT->BSRR = (uint32_t)CU_P1_PIN << 16;        // ����͵�ƽ 
#define CU_P2_PIN_H       WS_PORT->BSRR = CU_P2_PIN;                        // ����ߵ�ƽ
#define CU_P2_PIN_L       WS_PORT->BSRR = (uint32_t)CU_P2_PIN << 16;        // ����͵�ƽ 
#define CU_P3_PIN_H       WS_PORT->BSRR = CU_P3_PIN;                        // ����ߵ�ƽ
#define CU_P3_PIN_L       WS_PORT->BSRR = (uint32_t)CU_P3_PIN << 16;        // ����͵�ƽ 
#define CU_P4_PIN_H       WS_PORT->BSRR = CU_P4_PIN;                        // ����ߵ�ƽ
#define CU_P4_PIN_L       WS_PORT->BSRR = (uint32_t)CU_P4_PIN << 16;        // ����͵�ƽ 

//#elif(CUBEPLT_SLAVE)

#define P0_PIN            GPIO_PIN_0
#define P1_PIN            GPIO_PIN_8
#define P2_PIN            GPIO_PIN_7
#define P3_PIN            GPIO_PIN_6
#define P4_PIN            GPIO_PIN_5

#define WS_PORT           GPIOA

#define P0_PIN_H          GPIOA->BSRR = P0_PIN;                        // ����ߵ�ƽ
#define P0_PIN_L          GPIOA->BSRR = (uint32_t)P0_PIN << 16;        // ����͵�ƽ 
#define P1_PIN_H          GPIOA->BSRR = P1_PIN;                        // ����ߵ�ƽ
#define P1_PIN_L          GPIOA->BSRR = (uint32_t)P1_PIN << 16;        // ����͵�ƽ 
#define P2_PIN_H          GPIOA->BSRR = P2_PIN;                        // ����ߵ�ƽ
#define P2_PIN_L          GPIOA->BSRR = (uint32_t)P2_PIN << 16;        // ����͵�ƽ 
#define P3_PIN_H          GPIOA->BSRR = P3_PIN;                        // ����ߵ�ƽ
#define P3_PIN_L          GPIOA->BSRR = (uint32_t)P3_PIN << 16;        // ����͵�ƽ 
#define P4_PIN_H          GPIOA->BSRR = P4_PIN;                        // ����ߵ�ƽ
#define P4_PIN_L          GPIOA->BSRR = (uint32_t)P4_PIN << 16;        // ����͵�ƽ 
#define P5_PIN_H          GPIOA->BSRR = P0_PIN;                        // ����ߵ�ƽ
#define P5_PIN_L          GPIOA->BSRR = (uint32_t)P0_PIN << 16;        // ����͵�ƽ 
#define P6_PIN_H          GPIOA->BSRR = P1_PIN;                        // ����ߵ�ƽ
#define P6_PIN_L          GPIOA->BSRR = (uint32_t)P1_PIN << 16;        // ����͵�ƽ 
#define P7_PIN_H          GPIOA->BSRR = P2_PIN;                        // ����ߵ�ƽ
#define P7_PIN_L          GPIOA->BSRR = (uint32_t)P2_PIN << 16;        // ����͵�ƽ 

#define P8_PIN_H          GPIOA->BSRR = P3_PIN;                        // ����ߵ�ƽ
#define P8_PIN_L          GPIOA->BSRR = (uint32_t)P3_PIN << 16;        // ����͵�ƽ 
#define P9_PIN_H          GPIOA->BSRR = P4_PIN;                        // ����ߵ�ƽ
#define P9_PIN_L          GPIOA->BSRR = (uint32_t)P4_PIN << 16;        // ����͵�ƽ 
#define P10_PIN_H         GPIOA->BSRR = P0_PIN;                        // ����ߵ�ƽ
#define P10_PIN_L         GPIOA->BSRR = (uint32_t)P0_PIN << 16;        // ����͵�ƽ 
#define P11_PIN_H         GPIOA->BSRR = P1_PIN;                        // ����ߵ�ƽ
#define P11_PIN_L         GPIOA->BSRR = (uint32_t)P1_PIN << 16;        // ����͵�ƽ 
#define P12_PIN_H         GPIOA->BSRR = P2_PIN;                        // ����ߵ�ƽ
#define P12_PIN_L         GPIOA->BSRR = (uint32_t)P2_PIN << 16;        // ����͵�ƽ 
#define P13_PIN_H         GPIOA->BSRR = P3_PIN;                        // ����ߵ�ƽ
#define P13_PIN_L         GPIOA->BSRR = (uint32_t)P3_PIN << 16;        // ����͵�ƽ 
#define P14_PIN_H         GPIOA->BSRR = P4_PIN;                        // ����ߵ�ƽ
#define P14_PIN_L         GPIOA->BSRR = (uint32_t)P4_PIN << 16;        // ����͵�ƽ 
#define P15_PIN_H         GPIOA->BSRR = P0_PIN;                        // ����ߵ�ƽ
#define P15_PIN_L         GPIOA->BSRR = (uint32_t)P0_PIN << 16;        // ����͵�ƽ 

//#endif


/*---------------------- GPIO���ƺ� ------------------------*/




/*---------------------- �������� ----------------------------*/

void MBI_GPIO_Init(void);                                                  //GPIO��ʼ������
void OSRAM_GPIO_Init(void);                                                //GPIO��ʼ������


#endif //__GPIO_H

