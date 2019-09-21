/**
 ******************************************************************************
 * @file    drv_ir.c
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_ir source file
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "drv_ir.h"
#include "drv_serial.h"

#define TIME_GAP(s1,s2,max) (s2>=s1?s2-s1:max-s1+s2)

/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;


static uint16_t IR_data16L, IR_data16H;


/* Private function prototypes -----------------------------------------------*/
static void Drv_IR_MI_Decode(uint32_t cap, uint32_t max);
static int Drv_IR_NEC_Decode(uint32_t cap, uint32_t max);

/**
  * @brief  The drv_ir init.
  */
void Drv_IR_Init(void)
{
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
}

void Drv_IR_Proc(void)
{
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    Drv_IR_MI_Decode(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2), __HAL_TIM_GET_AUTORELOAD(htim));
  }
}

static void Drv_IR_MI_Decode(uint32_t cap, uint32_t max)
{
  uint8_t         firstFlag = 0;
  uint32_t        time_gap = 0;
  static uint8_t  IR_index = 0, IR_repeat = 0;
  static uint16_t IR_code = 0;
  static uint32_t IR_captured = 0;
  static uint32_t IR_data = 0, IR_data_repeat = 0;

  if (!Drv_IR_NEC_Decode(cap, max)) {
    //nec IR signal handler,escape MI
    return;
  }

  time_gap = TIME_GAP(IR_captured,cap,max); 

  //if ((time_gap >= 125 && time_gap <= 140)){
  if (time_gap >= 500) {
    IR_data_repeat = 0;
  }
  else if (time_gap >= 100){
    IR_index = 0;

    IR_data16L = 0;
    IR_data16H = 0;
  }
  else if (time_gap >= 10 && time_gap <= 15){
    IR_data16L &= ~(1 << IR_index);
    IR_index++;
  }
  else if (time_gap > 15 && time_gap < 20){
    IR_data16L |= (1 << IR_index);
    IR_index++;
  }
  else if (time_gap >= 20 && time_gap <= 25){
    IR_data16H |= (1 << IR_index);
    IR_index++;
  }

  if (IR_index >= 11){
    IR_data = IR_data16L | IR_data16H<<16;
    IR_index = 0;
    
    if (IR_data_repeat != IR_data) {
        IR_repeat = 0;
        IR_code  = 0;
        IR_data_repeat = IR_data;

        switch(IR_data){
            case MI_POWER:
                IR_code = REMOTE_MI_POWER;
                break;
            case MI_UP:
                IR_code = REMOTE_MI_UP;
                break;
            case MI_DOWN:
                IR_code = REMOTE_MI_DOWN;
                break;
            case MI_LEFT:
                IR_code = REMOTE_MI_LEFT;
                break;
            case MI_RIGHT:
                IR_code = REMOTE_MI_RIGHT;
                break;
            case MI_VOLPLUS:
                IR_code = REMOTE_MI_PLUS;
                break;
            case MI_VOLMINUS:
                IR_code = REMOTE_MI_MINUS;
                break;
            case MI_BACK:
                IR_code = REMOTE_MI_BACK;
                break;
            case MI_HOME:
                IR_code = REMOTE_MI_HOME;//focus+
                break;
            case MI_MENU:
                IR_code = REMOTE_MI_MENU;//focus-
                break;
            case MI_ENTER:
                IR_code = REMOTE_MI_OK;
                break;
            default:
                //printf("invalid IR RX code.\r\n");
                break;
        }
        firstFlag = 1;
    } else {
        IR_repeat++;
    }

    printf("MI:%08x,%d\r\n", IR_data, IR_repeat);
    if ((IR_repeat == 3 || firstFlag) && IR_code) {
      IR_repeat = 0;
      firstFlag = 0;
      Drv_SERIAL_Act(SET_CODE(CMD_CODE_MASK_IR, CMD_OP_IR_CODE), IR_code);
    }
  }
  IR_captured = cap;
}

static int Drv_IR_NEC_Decode(uint32_t cap, uint32_t max)
{
  uint32_t time_gap = 0;
  static uint8_t IR_index = 0;
  static uint8_t IR_repeat = 0;
  static uint8_t IR_valid = 0;
  static uint16_t IR_code = 0;
  static uint32_t IR_captured = 0;
  static uint32_t IR_data = 0;
  static uint32_t IR_start = 0;

  //TIM3  APB1 
  //cap:CCR2 is the counter value transferred by the last input capture 1 event (IC1).
  time_gap = TIME_GAP(IR_captured,cap,max);
  
  if (time_gap >= 125 && time_gap <= 140){
    IR_index = 0;
    IR_valid = 0;
    IR_data = 0;
    IR_start = IR_captured;
  }
  else if (time_gap >= 10 && time_gap <= 15){
    IR_data &= ~(1 << IR_index);
    IR_index++;
  }
  else if (time_gap >= 20 && time_gap <= 25){
    IR_data |= (1 << IR_index);
    IR_index++;
  }
  else if (time_gap >= 111 && time_gap <= 115){
    IR_repeat = 1;
    time_gap = TIME_GAP(IR_start,IR_captured,max);
    IR_start = IR_captured;
  }

  //WARNING:The printf cannot be permitted, otherwise it will be too late to deal with the interrupt
  //printf("timeGap=%d,id=%d\r\n", time_gap, IR_index);
  if (IR_index >= 32){
    uint8_t *IR_ptr = (uint8_t*)&IR_data;
    if ((IR_ptr[0] + IR_ptr[1] == 0xff) && (IR_ptr[2] + IR_ptr[3] == 0xff)){
      IR_code = (IR_ptr[0]|(IR_ptr[2]<<8));
      IR_valid = 1;
      IR_index = 0;
      printf("Drv_IR_NEC_Decode[0x%x]\r\n",IR_code);
      if (IR_code) {
        Drv_SERIAL_Act(SET_CODE(CMD_CODE_MASK_IR, CMD_OP_IR_CODE), IR_code);
      }
      return 0;
    }
  }
  else if (IR_repeat){
    IR_repeat = 0;
    if (IR_valid)
    {
       printf("repeat[%d]\r\n",time_gap);
      if (time_gap >= 1050 && time_gap <= 1080)
      {
        Drv_SERIAL_Act(SET_CODE(CMD_CODE_MASK_IR, CMD_OP_IR_CODE), IR_code);
        printf("Drv_IR_NEC_Decode repeat[0x%x]\r\n",IR_code);
        return 0;
      }
    }
  }
  IR_captured = cap;

  return -1;
}

