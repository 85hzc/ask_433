/*--------------File Info-------------------------------------------------------
** ��   ��   ��:  config.h
** ����޸�����:  2008.3.28
** ��        ��:  V1.0
** ��        ��:  �����ļ����ü�����Ҫ�Ĺ���
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

#define     true    1
#define     false   0

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
    DWORD    filesize;
    uint16_t framNum;
    uint8_t  filename[FILE_NAME_LEN];
    uint8_t  foldername[FILE_NAME_LEN];
}FILE_INFO_S;

#endif

