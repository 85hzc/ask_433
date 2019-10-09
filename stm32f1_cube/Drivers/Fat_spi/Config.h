/*--------------File Info-------------------------------------------------------
** ��   ��   ��:  Config.h
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

#define MAX_FILE_NUM        32

#if(PROJECTOR_OSRAM)
#define MAX_FILE_SIZE       2048       //256 must be % comfort
#elif(PROJECTOR_WS2801)
#define MAX_FILE_SIZE       4096//(4096*4)       //256 must be % comfort
#endif

#define MAX_FILM_FRAME      64
#define MAX_FILM_FOLDER     8

#define FILE_NAME_LEN       16
#define FILE_PATH_LEN       64

#define MATRIX_SIZE         32
#define CUBE_ROW_SIZE       24
#define CUBE_COL_SIZE       12
#define CUBE_PAGE_SIZE      4


#define CHIP_SIZE           (CUBE_ROW_SIZE*CUBE_COL_SIZE)
#define IO_SIZE             16

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

#endif

