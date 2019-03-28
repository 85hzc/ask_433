/**
 ******************************************************************************
 * @file    drv_motor.h
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_dlpc source file
 ******************************************************************************
 */

#ifndef __DRV_MOTOR_H
#define __DRV_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Type prototypes -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  The application entry point.
  */
void Drv_MOTOR_Init(void);
void Drv_MOTOR_CMD_Proc(void);

void drv_motor_move_forward(uint8_t steps);
void drv_motor_move_reverse(uint8_t steps);
void drv_motor_reset(void);

#endif

