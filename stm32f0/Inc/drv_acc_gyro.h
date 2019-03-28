/**
 ******************************************************************************
 * @file    drv_acc_gyro.c
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_acc_gyro source file
 ******************************************************************************
 */

#ifndef __DRV_ACC_GYRO_H
#define __DRV_ACC_GYRO_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Type prototypes -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  The application entry point.
  */
void Drv_ACC_Init(void);
void Drv_ACC_Proc(void);
#endif

