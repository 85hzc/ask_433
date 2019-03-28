/**
 ******************************************************************************
 * @file    drv_i2c_peripheral.c
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_i2c_peripheral source file
 ******************************************************************************
 */

#ifndef __DRV_HDMI_RCVR_H
#define __DRV_HDMI_RCVR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Type prototypes -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  The drv_hdmi_rcvr init.
  */
void Drv_HDMI_RCVR_Init(void);
void Drv_HDMI_RCVR_Proc(void);
void drv_hdmi_get_p0_status(void);

#endif

