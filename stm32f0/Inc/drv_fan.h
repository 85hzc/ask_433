/**
 ******************************************************************************
 * @file    drv_fan.h
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_fan source file
 ******************************************************************************
 */

#ifndef __DRV_FAN_H
#define __DRV_FAN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Type prototypes -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void Drv_FAN_Init(void);
void Drv_FAN_Proc(void);

void drv_fan_speed(uint16_t param);
void drv_fan_on(void);
void drv_fan_off(uint32_t delay);

#endif

