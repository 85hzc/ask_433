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
#include "drv_fan.h"
#include "app.h"

#define PWM_STEP 20
/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
static uint32_t tickstart;
static uint32_t tickexpire;
static uint8_t turn_off = 0;

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  The drv_led init.
  */
void Drv_FAN_Init(void)
{  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  

  tickstart = HAL_GetTick();
}

void Drv_FAN_Proc(void)
{
  if (turn_off)
  {
    if((HAL_GetTick() - tickstart) > tickexpire)
    {
      turn_off = 0;
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, (uint32_t)0);
    }
  }
}

void drv_fan_speed(uint16_t param)
{
  param=param>100?100:param;
  
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, (uint32_t)param);
}

void drv_pwm_speed_rgb(uint16_t param)
{
  uint8_t times = 20;
  static uint8_t speed = 0, pre_speed = 0;

  param=param>100?100:param;

  switch(param) {

    case GES_LEVEL2_UP:
        speed += PWM_STEP;
        if(speed > 100)
            speed = 100;
        break;
    case GES_LEVEL2_DOWN:
        if(speed <= PWM_STEP)
            speed = 0;
        else
            speed -= PWM_STEP;
        break;
    case GES_LEVEL2_LEFT:
        break;
    case GES_LEVEL2_RIGHT:
        break;
    case GES_LEVEL2_HOLD:
        times = 100;
        if(0==speed)
            speed = 40;
        else
            speed = 0;
        break;
    case GES_LEVEL2_DCLK:
        break;
    default:
        break;
  }

  //Drv_SERIAL_Log("speed=%d",speed);

  if(speed <= pre_speed) {

      for(int i=0; i < times && speed > 0 && speed < 100; i++) {
          //Drv_SERIAL_Log("speed=%d",speed);

          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, --speed);
          HAL_Delay(25);
      }
  } else {

      for(int i=0; i < times && speed < 100; i++) {
  
          //Drv_SERIAL_Log("speed=%d",speed);
          __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, ++speed);
          HAL_Delay(25);
      }
  }

  pre_speed = speed;
  //__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, (uint32_t)param);
  //__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, (uint32_t)param);
}

void drv_fan_on(void)
{
  turn_off = 0;

  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, (uint32_t)100);
}

void drv_fan_off(uint32_t delay)
{
  turn_off = 1;
  tickexpire = delay;
  tickstart = HAL_GetTick();
}

