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
#include "config.h"

/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;

static uint32_t tickstart = 0;
static uint32_t tickexpire = 0;
static uint8_t turn_off = 0;

#if (PROJECTOR_CUBE)
extern BOOL              lightingStatus;
extern uint16_t          brightness;
#endif
/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  The drv_led init.
  */
void Drv_PWM_Init(void)
{
    #if (PROJECTOR_CUBE)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  
    tickstart = HAL_GetTick();
    #elif (PROJECTOR_OSRAM)
    HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);
    #endif
}

void Drv_PWM_Proc(void)
{
		uint16_t i;
	
    if (turn_off)
    {
        if((HAL_GetTick() - tickstart) > tickexpire)
        {
            turn_off = 0;
            
            #if (PROJECTOR_CUBE)
            for( i=brightness; i>0; i-- )
            {
                HAL_Delay(10);
                __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, i-1);
            }
            #endif
        }
    }
}

void drv_pwm_speed(uint16_t param)
{
    param=param>1000?1000:param;
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, (uint32_t)param);

    #if (PROJECTOR_CUBE)
    if(param)
        lightingStatus = true;
    else
        lightingStatus = false;
    #endif
}

void drv_pwm_speed_stepup(uint16_t param)
{
    uint16_t i;

    #if (PROJECTOR_CUBE)
    param=param+brightness>1000?1000:param+brightness;

    for( i=brightness; i<param; i++ )
    {
        HAL_Delay(10);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, i+1);
    }

    brightness = param;
    #endif
}

void drv_pwm_speed_stepdown(uint16_t param)
{
    #if (PROJECTOR_CUBE)
    uint16_t i=brightness;

    if(brightness>param)
    {
        param=brightness-param<100?100:brightness-param;
        brightness = param;
    }

    while(i>param)
    {
        HAL_Delay(10);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, --i);
    }
    #endif
}

void drv_pwm_on(void)
{
    uint16_t i;
    turn_off = 0;

    #if (PROJECTOR_CUBE)
    if(lightingStatus)
        return;
    lightingStatus = true;
    
    for( i=0; i<brightness; i++ )
    {
        HAL_Delay(10);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, i+1);
    }
    #endif
}

void drv_pwm_off(uint32_t delay)
{
    #if (PROJECTOR_CUBE)
    if(!lightingStatus)
        return;
    lightingStatus = false;
    #endif

    turn_off = 1;
    tickexpire = delay;
    tickstart = HAL_GetTick();
}

