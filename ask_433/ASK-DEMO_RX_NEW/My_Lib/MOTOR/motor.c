#include "ALL_Includes.h"


#define MOVE_CYCLE   4

#define MOVE_STEP_1  0
#define MOVE_STEP_2  1
#define MOVE_STEP_3  2
#define MOVE_STEP_4  3
#define MOVE_STEP_S  4

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


/* Private variables ---------------------------------------------------------*/
static uint8_t motor_steps[MOVE_CYCLE] = {MOVE_STEP_1 ,MOVE_STEP_2 ,MOVE_STEP_3, MOVE_STEP_4};
static uint8_t step = 0;

void Drv_MOTOR_Init(void)
{
    //uint8_t sta;

    HAL_GPIO_WritePin(MOTOR_IN_Port, MOTOR_IN_Pin, GPIO_PIN_RESET);
    MOTOR_MOVE_STOP;
    //sta = HAL_GPIO_ReadPin(MOTOR_OUT_Port, MOTOR_OUT_Pin);
    //Drv_SERIAL_Log("MOTOR OUTPUT %d", sta);
}

void drv_motor_move_forward(uint8_t steps)
{
    uint8_t i;
    unsigned char st[128];

    //for (i=0,step=0;i<steps;i++)
    for (i=0;i<1;i++)
    {
        step = (step + 1) % MOVE_CYCLE;
        sprintf(st,"Fstep %d\r\n",motor_steps[step]);
        Uart_Senddata(st, strlen(st));
        drv_motor_move_execute(motor_steps[step]);
        DelayMS(5);
    }
    drv_motor_move_execute(MOVE_STEP_S);
}

void drv_motor_move_reverse(uint8_t steps)
{  
    uint8_t i;

    //for (i=0,step=0;i<steps;i++)
    for (i=0;i<1;i++)
    {
        step = (step + MOVE_CYCLE - 1) % MOVE_CYCLE;
        sprintf(st,"Rstep %d\r\n",motor_steps[step]);
        Uart_Senddata(st, strlen(st));
        drv_motor_move_execute(motor_steps[step]);
        DelayMS(5);
    }
    drv_motor_move_execute(MOVE_STEP_S);
}

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

    uint8_t sta = HAL_GPIO_ReadPin(MOTOR_OUT_Port, MOTOR_OUT_Pin);
    printf("MOTOR move step %d, OUTPUT %d\r\n", step, sta);
}

int8_t Drv_MOTOR_CMD_Handler(uint8_t code, uint16_t param)
{
    int8_t rc = 0;

    switch (code)
    {
        case CMD_OP_MOTOR_SET_FORWARD:
            drv_motor_move_forward(4);
            break;
        case CMD_OP_MOTOR_SET_BACKWARD:
            drv_motor_move_reverse(4);
            break;
    }

    return rc;
}


