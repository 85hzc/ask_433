/**
 ******************************************************************************
 * @file    drv_led.c
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_led source file
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_led.h"

/* Private variables ---------------------------------------------------------*/
static uint32_t tickstart;

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  The drv_led init.
  */
void Drv_LED_Init(void)
{
  DRV_LED_ON;
  
  tickstart = HAL_GetTick();  
}

void Drv_LED_Proc(void)
{
  if((HAL_GetTick() - tickstart) >= 1000)
  {
    tickstart = HAL_GetTick();
    DRV_LED_TOGGLE;
  }
}

