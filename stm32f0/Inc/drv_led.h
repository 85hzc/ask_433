/**
 ******************************************************************************
 * @file    drv_led.h
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_led source file
 ******************************************************************************
 */

#ifndef __DRV_LED_H
#define __DRV_LED_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

#define DRV_LED_ON  \
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET)
  
#define DRV_LED_OFF \
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET)

#define DRV_LED_TOGGLE \
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2)

/* Private function prototypes -----------------------------------------------*/
void Drv_LED_Init(void);
void Drv_LED_Proc(void);

#endif

