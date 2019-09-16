/*--------------File Info-------------------------------------------------------
** 文   件   名:  Config.h
** 最后修改日期:  2008.3.28
** 版        本:  V1.0
** 描        述:  配置文件、裁剪不需要的功能
**------------------------------------------------------------------------------
** Created   by:
** Created date:
*******************************************************************************/
#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "stm32f1xx.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_usart.h"
#include "integer.h"

#include "SPI_SD_driver.h"

#include "ff.h"
#include "diskio.h"

#define MAX_FILE_NUM        4       //256 must be % comfort
#define MAX_FILE_SIZE       (1024)       //256 must be % comfort

#define CHIP_SIZE           64
#define IO_SIZE             16

#define RX_LEN              (IO_SIZE*CHIP_SIZE*3)
#define TX_LEN              (IO_SIZE*CHIP_SIZE*3)

#define SW_period_2ms                               1
#define SW_period_20ms                              10
#define SW_period_200ms                             100
#define GRAY                                        10

typedef enum
{
    RGB_B_minus = 1,
    RGB_B_plus,
    RGB_G_minus,
    RGB_G_plus,
    RGB_R_minus,
    RGB_R_plus,

    RGB_G1_minus,
    RGB_B1_minus,
    RGB_G1_plus,
    RGB_R1_minus,
    RGB_B1_plus,
    RGB_R1_plus,

}RGB_Switch_E;

typedef enum
{
    G = 1<<0,
    R = 1<<1,
    B = 1<<2,
    RGB = G|R|B,
}RGB_Type_E;

typedef struct
{
    uint8_t RX_flag:1;          //IDLE receive flag
    uint16_t RX_Size;           //receive length
    uint8_t RX_pData[RX_LEN];   //DMA receive buffer
}USART_RECEIVETYPE;

typedef struct
{
    uint8_t TX_flag:1;          //IDLE receive flag
    uint16_t TX_Size;           //receive length
    uint8_t TX_pData[TX_LEN];   //DMA receive buffer
}USART_TransmiteTYPE;

#endif

