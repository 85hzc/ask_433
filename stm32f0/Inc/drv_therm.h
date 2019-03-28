/**
 ******************************************************************************
 * @file    drv_therm.c
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_therm source file
 ******************************************************************************
 */

#ifndef __DRV_THERM_H
#define __DRV_THERM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Type prototypes -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/


/**
  * @brief  The application entry point.
  */
void Drv_THERM_Init(void);
void Drv_THERM_CMD_Proc(void);

int8_t drv_therm_get_value(void);

#endif


