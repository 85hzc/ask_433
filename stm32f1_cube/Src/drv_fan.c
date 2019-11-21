/**
 ******************************************************************************
 * @file    drv_fan.c
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_fan source file
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;

static uint32_t tickstart = 0;
static uint32_t tickexpire = 0;
static uint8_t turn_off = 0;

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  The drv_led init.
  */
void Drv_FAN_Init(void)
{
    #if (PROJECTOR_CUBE)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  
    tickstart = HAL_GetTick();
    #else
    HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);
    #endif
}

void Drv_FAN_Proc(void)
{
    if (turn_off)
    {
        if((HAL_GetTick() - tickstart) > tickexpire)
        {
            turn_off = 0;
            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, (uint32_t)0);
        }
    }
}

void drv_fan_speed(uint16_t param)
{
    param=param>100?100:param;
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, (uint32_t)param);
}

void drv_fan_on(void)
{
    turn_off = 0;
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, 100);
}

void drv_fan_off(uint32_t delay)
{
    turn_off = 1;
    tickexpire = delay;
    tickstart = HAL_GetTick();
}

