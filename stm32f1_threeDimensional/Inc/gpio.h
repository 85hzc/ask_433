#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f1xx.h"
#include "stm32f1xx_hal_gpio.h"

/*---------------------- GPIO配置宏 ------------------------*/

#define LED_Pin           GPIO_PIN_1
#define LED_GPIO_Port     GPIOB

//#define INT_Pin           GPIO_PIN_1
//#define INT_GPIO_Port     GPIOA

#define IR_IN_Pin         GPIO_PIN_0  //TIM2 CH1
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
#define DCLK_PIN          GPIO_PIN_6             // MBI dclk 引脚
#define LE_PIN            GPIO_PIN_7             // MBI LE 引脚
#define SDI_PIN           GPIO_PIN_5             // MBI SDI 引脚
#define MBI_PORT          GPIOA                  // MBI驱动 GPIO端口

#define GCLK_PIN          GPIO_PIN_0             // MBI gclk 引脚
#define MBI_GCLK_PORT     GPIOB                  // MBI驱动 GPIO端口

#define AG_OE_PIN         GPIO_PIN_15             // AG OE 引脚
#define AG_CLK_PIN        GPIO_PIN_14             // AG Clk 引脚
#define AG_DIN_PIN        GPIO_PIN_13             // AG Din 引脚
#define AG_PORT           GPIOC                   // AG驱动 GPIO端口


#define DCLK_PIN_H        MBI_PORT->BSRR = DCLK_PIN;                       // 输出高电平
#define DCLK_PIN_L        MBI_PORT->BSRR = (uint32_t)DCLK_PIN << 16;       // 输出低电平
#define LE_PIN_H          MBI_PORT->BSRR = LE_PIN;                         // 输出高电平
#define LE_PIN_L          MBI_PORT->BSRR = (uint32_t)LE_PIN << 16;         // 输出低电平
#define SDI_PIN_H         MBI_PORT->BSRR = SDI_PIN;                        // 输出高电平
#define SDI_PIN_L         MBI_PORT->BSRR = (uint32_t)SDI_PIN << 16;        // 输出低电平

#define GCLK_PIN_H        MBI_GCLK_PORT->BSRR = GCLK_PIN;                       // 输出高电平
#define GCLK_PIN_L        MBI_GCLK_PORT->BSRR = (uint32_t)GCLK_PIN << 16;       // 输出低电平

#define AG_CLK_PIN_H      AG_PORT->BSRR = AG_CLK_PIN;                     // 输出高电平
#define AG_CLK_PIN_L      AG_PORT->BSRR = (uint32_t)AG_CLK_PIN << 16;     // 输出低电平
#define AG_DIN_PIN_H      AG_PORT->BSRR = AG_DIN_PIN;                     // 输出高电平
#define AG_DIN_PIN_L      AG_PORT->BSRR = (uint32_t)AG_DIN_PIN << 16;     // 输出低电平
#define AG_OE_PIN_H       AG_PORT->BSRR = AG_OE_PIN;                      // 输出高电平
#define AG_OE_PIN_L       AG_PORT->BSRR = (uint32_t)AG_OE_PIN << 16;      // 输出低电平


//#elif(PROJECTOR_OSRAM)
/////OSRAM GPIO
#define QUADRANT_EN       GPIO_PIN_1
#define QUADRANT_EN_PORT  GPIOA

#define SCL_Pin           GPIO_PIN_6
#define SDA_Pin           GPIO_PIN_7
#define SCL_GPIO_Port     GPIOB
#define SDA_GPIO_Port     GPIOB

#define Q0_SI             GPIO_PIN_5
#define Q1_SI             GPIO_PIN_3
#define Q2_SI             GPIO_PIN_8
#define Q3_SI             GPIO_PIN_10
#define QT_UPD            GPIO_PIN_0
#define QT_CLK            GPIO_PIN_11
#define OSRAM_PORT        GPIOB

#define Q0_SI_H           OSRAM_PORT->BSRR = Q0_SI;                        // 输出高电平
#define Q0_SI_L           OSRAM_PORT->BSRR = (uint32_t)Q0_SI << 16;        // 输出低电平 
#define Q1_SI_H           OSRAM_PORT->BSRR = Q1_SI;                        // 输出高电平
#define Q1_SI_L           OSRAM_PORT->BSRR = (uint32_t)Q1_SI << 16;        // 输出低电平
#define Q2_SI_H           OSRAM_PORT->BSRR = Q2_SI;                        // 输出高电平
#define Q2_SI_L           OSRAM_PORT->BSRR = (uint32_t)Q2_SI << 16;        // 输出低电平
#define Q3_SI_H           OSRAM_PORT->BSRR = Q3_SI;                        // 输出高电平
#define Q3_SI_L           OSRAM_PORT->BSRR = (uint32_t)Q3_SI << 16;        // 输出低电平
#define QT_UPD_H          OSRAM_PORT->BSRR = Q3_UPD;                       // 输出高电平
#define QT_UPD_L          OSRAM_PORT->BSRR = (uint32_t)Q3_UPD << 16;       // 输出低电平
#define QT_CLK_H          OSRAM_PORT->BSRR = QT_CLK;                       // 输出高电平
#define QT_CLK_L          OSRAM_PORT->BSRR = (uint32_t)QT_CLK << 16;       // 输出低电平

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

#define ROW_PULL_UP(a)    ROW_PORT->BSRR = a;                          // 输出高电平
#define ROW_PULL_DOWN(a)  ROW_PORT->BSRR = (uint32_t)a << 16;          // 输出低电平

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

#define COL_PULL_UP(a)    COL_PORT->BSRR = a;                          // 输出高电平
#define COL_PULL_DOWN(a)  COL_PORT->BSRR = (uint32_t)a << 16;          // 输出低电平

//#endif

/*---------------------- GPIO控制宏 ------------------------*/




/*---------------------- 函数声明 ----------------------------*/

void MBI_GPIO_Init(void);                                                  //GPIO初始化函数
void OSRAM_GPIO_Init(void);                                                //GPIO初始化函数


#endif //__GPIO_H

