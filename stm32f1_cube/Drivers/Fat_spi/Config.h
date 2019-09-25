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

#define MAX_FILE_NUM        50       //256 must be % comfort
#define MAX_FILE_SIZE       (1024)       //256 must be % comfort

#endif

