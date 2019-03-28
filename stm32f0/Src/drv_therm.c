/**
 ******************************************************************************
 * @file    drv_therm.c
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_therm source file
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_serial.h"
#include "drv_therm.h"
    
/* Private variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef hadc;

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  The application entry point.
  */
void Drv_THERM_Init(void)
{
  HAL_ADC_Start(&hadc);
}

void Drv_THERM_CMD_Proc(void)
{
  
}

int8_t drv_therm_get_value(void)
{
  uint32_t temp;
  
  temp=HAL_ADC_GetValue(&hadc);
  Drv_SERIAL_Log("Temperature: %d, %f", temp, temp*3.3f/4096);

  return 0;
}
