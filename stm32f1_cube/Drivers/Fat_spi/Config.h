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
#define MAX_FILE_SIZE       2200       //256 must be % comfort
#define MAX_FILM_FRAME      64
#define MAX_FILM_FOLDER     8

#define FILE_NAME_LEN       16
#define FILE_PATH_LEN       64

#define MATRIX_SIZE         32

#endif

