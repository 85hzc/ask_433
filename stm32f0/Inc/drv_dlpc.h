/**
 ******************************************************************************
 * @file    drv_dlpc.h
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_dlpc source file
 ******************************************************************************
 */

#ifndef __DRV_DLPC_H
#define __DRV_DLPC_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Type prototypes -----------------------------------------------------------*/
#define DLPC_PROJ_STATUS()  \
    HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  The application entry point.
  */
void Drv_DLPC_Init(void);
void Drv_DLPC_CMD_Proc(void);

int8_t drv_dlpc_set_current(uint16_t param);
int8_t drv_dlpc_set_keystone(uint8_t d);
int8_t drv_dlpc_reset_keystone(void);
int8_t drv_dlpc_proj_ctrl(uint16_t param);
int8_t drv_dlpc_set_input(uint16_t param);
int8_t drv_dlpc_set_orient(void);
int8_t drv_dlpc_switch_test_pattern(void);
int8_t drv_dlpc_sw(void);

#endif

