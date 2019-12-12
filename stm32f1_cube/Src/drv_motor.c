/**
 ******************************************************************************
 * @file    drv_motor.c
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_motor source file
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "delay.h"
#include "drv_serial.h"
#include "drv_motor.h"

#define MOVE_CYCLE   4

#define MOVE_STEP_1  0
#define MOVE_STEP_2  1
#define MOVE_STEP_3  2
#define MOVE_STEP_4  3
#define MOVE_STEP_S  4

#if (PROJECTOR_OSRAM)
#define MOTOR_MOVE_STEP1 \
    HAL_GPIO_WritePin(MOTOR_AIN2_Port, MOTOR_AIN2_Pin, GPIO_PIN_SET);  \
    HAL_GPIO_WritePin(MOTOR_AIN1_Port, MOTOR_AIN1_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_BIN1_Port, MOTOR_BIN1_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_BIN2_Port, MOTOR_BIN2_Pin, GPIO_PIN_SET);

#define MOTOR_MOVE_STEP2 \
    HAL_GPIO_WritePin(MOTOR_AIN2_Port, MOTOR_AIN2_Pin, GPIO_PIN_SET);  \
    HAL_GPIO_WritePin(MOTOR_AIN1_Port, MOTOR_AIN1_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_BIN1_Port, MOTOR_BIN1_Pin, GPIO_PIN_SET);  \
    HAL_GPIO_WritePin(MOTOR_BIN2_Port, MOTOR_BIN2_Pin, GPIO_PIN_RESET);
    
#define MOTOR_MOVE_STEP3 \
    HAL_GPIO_WritePin(MOTOR_AIN2_Port, MOTOR_AIN2_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_AIN1_Port, MOTOR_AIN1_Pin, GPIO_PIN_SET);  \
    HAL_GPIO_WritePin(MOTOR_BIN1_Port, MOTOR_BIN1_Pin, GPIO_PIN_SET);  \
    HAL_GPIO_WritePin(MOTOR_BIN2_Port, MOTOR_BIN2_Pin, GPIO_PIN_RESET);
    
#define MOTOR_MOVE_STEP4 \
    HAL_GPIO_WritePin(MOTOR_AIN2_Port, MOTOR_AIN2_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_AIN1_Port, MOTOR_AIN1_Pin, GPIO_PIN_SET);  \
    HAL_GPIO_WritePin(MOTOR_BIN1_Port, MOTOR_BIN1_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_BIN2_Port, MOTOR_BIN2_Pin, GPIO_PIN_SET);

#define MOTOR_MOVE_STOP \
    HAL_GPIO_WritePin(MOTOR_AIN2_Port, MOTOR_AIN2_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_AIN1_Port, MOTOR_AIN1_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_BIN1_Port, MOTOR_BIN1_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_BIN2_Port, MOTOR_BIN2_Pin, GPIO_PIN_RESET);
#elif (PROJECTOR_FOCUS)
#define MOTOR_MOVE_STEP1 \
    HAL_GPIO_WritePin(MOTOR_Focus_AIN1_Port, MOTOR_Focus_AIN1_Pin, GPIO_PIN_SET);  \
    HAL_GPIO_WritePin(MOTOR_Focus_AIN2_Port, MOTOR_Focus_AIN2_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_Focus_BIN1_Port, MOTOR_Focus_BIN1_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_Focus_BIN2_Port, MOTOR_Focus_BIN2_Pin, GPIO_PIN_RESET);

#define MOTOR_MOVE_STEP2 \
    HAL_GPIO_WritePin(MOTOR_Focus_AIN1_Port, MOTOR_Focus_AIN1_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_Focus_AIN2_Port, MOTOR_Focus_AIN2_Pin, GPIO_PIN_SET);  \
    HAL_GPIO_WritePin(MOTOR_Focus_BIN1_Port, MOTOR_Focus_BIN1_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_Focus_BIN2_Port, MOTOR_Focus_BIN2_Pin, GPIO_PIN_RESET);
    
#define MOTOR_MOVE_STEP3 \
    HAL_GPIO_WritePin(MOTOR_Focus_AIN1_Port, MOTOR_Focus_AIN1_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_Focus_AIN2_Port, MOTOR_Focus_AIN2_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_Focus_BIN1_Port, MOTOR_Focus_BIN1_Pin, GPIO_PIN_SET);  \
    HAL_GPIO_WritePin(MOTOR_Focus_BIN2_Port, MOTOR_Focus_BIN2_Pin, GPIO_PIN_RESET);
    
#define MOTOR_MOVE_STEP4 \
    HAL_GPIO_WritePin(MOTOR_Focus_AIN1_Port, MOTOR_Focus_AIN1_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_Focus_AIN2_Port, MOTOR_Focus_AIN2_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_Focus_BIN1_Port, MOTOR_Focus_BIN1_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_Focus_BIN2_Port, MOTOR_Focus_BIN2_Pin, GPIO_PIN_SET);

#define MOTOR_MOVE_STOP \
    HAL_GPIO_WritePin(MOTOR_Focus_AIN1_Port, MOTOR_Focus_AIN1_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_Focus_AIN2_Port, MOTOR_Focus_AIN2_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_Focus_BIN1_Port, MOTOR_Focus_BIN1_Pin, GPIO_PIN_RESET);  \
    HAL_GPIO_WritePin(MOTOR_Focus_BIN2_Port, MOTOR_Focus_BIN2_Pin, GPIO_PIN_RESET);
#endif

/* Private variables ---------------------------------------------------------*/
static uint8_t motor_steps[MOVE_CYCLE] = {MOVE_STEP_1 ,MOVE_STEP_2 ,MOVE_STEP_3, MOVE_STEP_4};
static uint8_t step = 0;

/* Private function prototypes -----------------------------------------------*/
static void drv_motor_move_execute(uint8_t step);

/**
  * @brief  The application entry point.
  */
void Drv_MOTOR_Init(void)
{
    //uint8_t sta;

    #if(PROJECTOR_OSRAM)
    HAL_GPIO_WritePin(MOTOR_IN_Port, MOTOR_IN_Pin, GPIO_PIN_RESET);
    MOTOR_MOVE_STOP;
    //sta = HAL_GPIO_ReadPin(MOTOR_OUT_Port, MOTOR_OUT_Pin);
    //Drv_SERIAL_Log("MOTOR OUTPUT %d", sta);
    #endif
}

void Drv_MOTOR_CMD_Proc(void)
{
  
}

void drv_motor_reset(void)
{
  
}
#if (PROJECTOR_FOCUS)
void motor_loop_forward()
{
    step = (step + 1) % MOVE_CYCLE;
    printf("Fstep %d\r\n",motor_steps[step]);
    drv_motor_move_execute(motor_steps[step]);
    Delay_ms(1);
}

void motor_loop_reverse()
{  
    step = (step + MOVE_CYCLE - 1) % MOVE_CYCLE;
    printf("Rstep %d\r\n",motor_steps[step]);
    drv_motor_move_execute(motor_steps[step]);
    Delay_ms(1);
}

void motor_loop_stop()
{
    drv_motor_move_execute(MOVE_STEP_S);
}
#endif

#if(PROJECTOR_OSRAM)
void drv_motor_move_forward(uint8_t steps)
{
    uint8_t i;

    #if 0
    for (i=0,step=0;i<steps;i++)
    #else
    for (i=0;i<1;i++)
    #endif
    {
        step = (step + 1) % MOVE_CYCLE;
        printf("Fstep %d\r\n",motor_steps[step]);
        drv_motor_move_execute(motor_steps[step]);
        HAL_Delay(3);
    }
    drv_motor_move_execute(MOVE_STEP_S);
}

void drv_motor_move_reverse(uint8_t steps)
{  
    uint8_t i;

    #if 0
    for (i=0,step=0;i<400;i++)
    #else
    for (i=0;i<1;i++)
    #endif
    {
        step = (step + MOVE_CYCLE - 1) % MOVE_CYCLE;
        printf("Rstep %d\r\n",motor_steps[step]);
        drv_motor_move_execute(motor_steps[step]);
        HAL_Delay(3);
    }
    drv_motor_move_execute(MOVE_STEP_S);
}
#endif

#if(PROJECTOR_OSRAM || PROJECTOR_FOCUS)
static void drv_motor_move_execute(uint8_t step)
{
    switch (step)
    {
        case MOVE_STEP_1:
            MOTOR_MOVE_STEP1;
            break;
        case MOVE_STEP_2:
            MOTOR_MOVE_STEP2;
            break;
        case MOVE_STEP_3:
            MOTOR_MOVE_STEP3;
            break;
        case MOVE_STEP_4:
            MOTOR_MOVE_STEP4;
            break;
        case MOVE_STEP_S:
            MOTOR_MOVE_STOP;
            break;
    }
    #if(PROJECTOR_OSRAM)
    uint8_t sta = HAL_GPIO_ReadPin(MOTOR_OUT_Port, MOTOR_OUT_Pin);
    printf("MOTOR move step %d, OUTPUT %d\r\n", step, sta);
    #endif
}
#endif

