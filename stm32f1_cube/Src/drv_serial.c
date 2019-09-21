
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

static uint32_t          tickstart;

extern uint8_t           single_cmd[CMD_LEN_MAX];


int8_t Drv_IR_CMD_Handler(uint8_t code, uint16_t key);


static cmd_table_t cmd_tbl[] = {
  {CMD_CODE_MASK_IR,    Drv_IR_CMD_Handler},
  //{CMD_CODE_MASK_HDMI,  Drv_HDMI_RCVR_CMD_Handler},
  //{CMD_CODE_MASK_MOTOR, Drv_MOTOR_CMD_Handler},
  //{CMD_CODE_MASK_FAN,   Drv_FAN_CMD_Handler},
};


static struct {
  uint8_t wr_id;
  uint8_t rd_id;
  uint8_t multi_cmd[MULTI_CMD_MAX][CMD_LEN_MAX];
} act_cmd;

void Drv_SERIAL_Init(void)
{

  memset(single_cmd, 0, sizeof(single_cmd));
  memset((uint8_t*)&act_cmd, 0, sizeof(act_cmd));
}

int8_t Drv_CMD_Handler(uint8_t *cmd)
{  
    if(CMD_HEADER_REQ == cmd[0] || CMD_HEADER_REQ_NR == cmd[0])
    {
        uint8_t i;
        uint16_t cmdCode;

        cmdCode = cmd[2] | (cmd[3]<<8);
        for (i = 0; i < ARR_SIZE(cmd_tbl); i ++)
        {
            if (GET_MASK(cmd[1]) == cmd_tbl[i].cmd_mask)
            {
                if (cmd_tbl[i].handler)
                {
                    return cmd_tbl[i].handler(GET_OP(cmd[1]), cmdCode);
                }
            }
        }
    }
    return -1;
}

static uint8_t Drv_SERIAL_Write_Act(uint8_t * pData)
{
    memcpy(act_cmd.multi_cmd[act_cmd.wr_id], pData, CMD_LEN_MAX-1);
    act_cmd.wr_id = ( act_cmd.wr_id + 1 ) % MULTI_CMD_MAX;

    return (uint8_t)HAL_OK;
}

uint8_t Drv_SERIAL_Read_Act(uint8_t * pData)
{
    if (act_cmd.rd_id != act_cmd.wr_id)
    {
        memcpy(pData, act_cmd.multi_cmd[act_cmd.rd_id], CMD_LEN_MAX-1);
        act_cmd.rd_id = ( act_cmd.rd_id + 1 ) % MULTI_CMD_MAX;    
        return (uint8_t)HAL_OK;
    }
    return (uint8_t)HAL_ERROR;
}

uint8_t Drv_SERIAL_Act(uint8_t code, uint16_t param)
{
  uint8_t cmd[CMD_LEN_MAX];
  cmd[0] = CMD_HEADER_REQ;
  cmd[1] = code;
  cmd[2] = ((uint8_t*)&param)[0];
  cmd[3] = ((uint8_t*)&param)[1];
  return Drv_SERIAL_Write_Act(cmd);
}

static void handle_func_MIkeys(uint16_t key)
{
    switch (key)
    {
        case REMOTE_MI_HOME:
            
            break;
        case REMOTE_MI_MENU:
            
            break;
        case REMOTE_MI_POWER:
            break;
        case REMOTE_MI_UP:
        case REMOTE_MI_DOWN:
            SD_ReadFileData();
            break;
        case REMOTE_MI_LEFT:
        case REMOTE_MI_RIGHT:
        case REMOTE_MI_OK:
        case REMOTE_MI_BACK:
        case REMOTE_MI_PLUS:
        case REMOTE_MI_MINUS:

          break;
        default:
          printf("invalid key[0x%x]!\r\n",key);
    }
}

int8_t Drv_IR_CMD_Handler(uint8_t code, uint16_t key)
{
    //printf("Drv_IR_CMD_Handler 0x%x\r\n",key);
    static uint8_t  flag = 0;
    static uint16_t lastKey = 0;

    if (code == CMD_OP_IR_CODE)
    {

        if(key>>8 == 0xff) {// MI controler

            if(lastKey != key) {

                lastKey = key;
                flag = 1;
                tickstart = HAL_GetTick();
            } else {

                if((HAL_GetTick() - tickstart) > ((REMOTE_MI_POWER == key) ? 2000 : 300)) //repeat press
                {
                    tickstart = HAL_GetTick();
                    flag = 1;
                } else {
                    //printf("repeat escap\r\n");
                }
            }

            if(flag) {
                //printf("TX key:%x\r\n",key);
                flag = 0;
                switch (key)
                {
                    case REMOTE_MI_POWER:
                        //handle_power_key();
                        break;
                    default:
                        handle_func_MIkeys(key);
                }
            }
        }
        else
        {//NEC controler
            switch (key>>8)
            {
                case REMOTE_NEC_POWER:
                    //handle_power_key();
                    break;
                default:
                    //handle_func_keys(key);
            }
        }
    }
    return 0;
}

