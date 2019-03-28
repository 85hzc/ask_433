/**
 ******************************************************************************
 * @file    drv_au_amp.c
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_au_amp source file
 ******************************************************************************
 */

#ifndef __DRV_AU_AMP_H
#define __DRV_AU_AMP_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c2;

/* Private function prototypes -----------------------------------------------*/


/**
  * @brief  The application entry point.
  */
void Drv_AU_AMP_Init(void);

#endif

