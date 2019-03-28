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
#include "drv_ir.h"
#include "drv_serial.h"

#define TIME_GAP(s1,s2,max) (s2>=s1?s2-s1:max-s1+s2)

/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;

static uint8_t IR_index = 0;
static uint8_t IR_repeat = 0;
static uint8_t IR_valid = 0;
static uint16_t IR_code;
static uint32_t IR_captured = 0;
static uint32_t IR_data;
static uint32_t IR_start = 0;

/* Private function prototypes -----------------------------------------------*/
static void Drv_IR_Decode(uint32_t cap, uint32_t max);

/**
  * @brief  The drv_ir init.
  */
void Drv_IR_Init(void)
{
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
}

void Drv_IR_Proc(void)
{
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
    Drv_IR_Decode(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4), __HAL_TIM_GET_AUTORELOAD(htim));
  }
}

static void Drv_IR_Decode(uint32_t cap, uint32_t max)
{  
  uint32_t time_gap = 0;
  
  time_gap = TIME_GAP(IR_captured,cap,max); 
  if (time_gap >= 125 && time_gap <= 140){
    IR_index = 0;
    IR_valid = 0;
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
  
  if (IR_index >= 32){
    uint8_t *IR_ptr = (uint8_t*)&IR_data;
    if (
          (IR_ptr[0] + IR_ptr[1] == 0xff)
        &&(IR_ptr[2] + IR_ptr[3] == 0xff)
       ){
      IR_code = (IR_ptr[0]|(IR_ptr[2]<<8));
      IR_valid = 1;
      IR_index = 0;
      Drv_SERIAL_Rpt(SET_CODE(CMD_CODE_MASK_IR, CMD_OP_IR_CODE), IR_code);
      Drv_SERIAL_Act(SET_CODE(CMD_CODE_MASK_IR, CMD_OP_IR_CODE), IR_code);
    }
  }
  else if (IR_repeat){
    IR_repeat = 0;
    if (IR_valid)
    {      
      if (time_gap >= 1060 && time_gap <= 1080)
      {
        Drv_SERIAL_Rpt(SET_CODE(CMD_CODE_MASK_IR, CMD_OP_IR_CODE), IR_code);
        Drv_SERIAL_Act(SET_CODE(CMD_CODE_MASK_IR, CMD_OP_IR_CODE), IR_code);
      }
    }
  }
  IR_captured = cap;
}

