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

#define MAX_FILE_NUM        50       //256 must be % comfort
#define MAX_FILE_SIZE       (1024)       //256 must be % comfort

#endif

