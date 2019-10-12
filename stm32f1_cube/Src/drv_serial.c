
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
#include "programs.h"

ACT_CMD act_cmd;

static uint32_t          tickstart;

extern uint8_t           single_cmd[CMD_LEN_MAX];
extern uint8_t           photoProgramIdx;
extern uint8_t           filmProgramIdx;
extern uint8_t           filmFrameIdx;
extern uint8_t           runFlag;
#if(PROJECTOR_OSRAM)
extern uint8_t           eplosCfgFlag;
extern uint8_t           eplosSLPxen;
extern uint8_t           currentAdjustment;
#endif
extern PROGRAMS_TYPE_E   programsType;

int8_t Drv_IR_CMD_Handler(uint8_t code, uint16_t key);


static cmd_table_t cmd_tbl[] = {
  {CMD_CODE_MASK_IR,    Drv_IR_CMD_Handler},
  //{CMD_CODE_MASK_HDMI,  Drv_HDMI_RCVR_CMD_Handler},
  //{CMD_CODE_MASK_MOTOR, Drv_MOTOR_CMD_Handler},
  //{CMD_CODE_MASK_FAN,   Drv_FAN_CMD_Handler},
};


void Drv_SERIAL_Init(void)
{

    act_cmd.rd_id = 0;
    act_cmd.wr_id = 0;

    memset(single_cmd, 0, sizeof(single_cmd));

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
#if(PROJECTOR_OSRAM)
            eplosSLPxen = !eplosSLPxen;
            eplosCfgFlag = 1;
#endif
            runFlag = 1;
            break;
            
        case REMOTE_MI_UP:
            if(programsType==PHOTO)
            {
                if(photoProgramIdx<0xff)
                    photoProgramIdx++;
                else
                    photoProgramIdx = 0;

                runFlag = 1;
            }
            else
            {
                if(filmProgramIdx<0xff)
                    filmProgramIdx++;
                else
                    filmProgramIdx = 0;

                filmFrameIdx = 0;//切换影片频道，从片头开始
            }
            break;

        case REMOTE_MI_DOWN:
            if(programsType==PHOTO)
            {
                if(photoProgramIdx>0)
                    photoProgramIdx--;
                else
                    photoProgramIdx=0xff;

                runFlag = 1;
            }
            else
            {
                if(filmProgramIdx>0)
                    filmProgramIdx--;
                else
                    filmProgramIdx=0xff;

                filmFrameIdx = 0;//切换影片频道，从片头开始
            }
            break;
            
        case REMOTE_MI_LEFT:
        case REMOTE_MI_RIGHT:
            break;
        
        case REMOTE_MI_OK:
            runFlag = 1;
            //photoIdx=0;
            filmFrameIdx = 0;
            programsType = (programsType==PHOTO) ? FILM : PHOTO;
            break;
        
        case REMOTE_MI_BACK:
        case REMOTE_MI_PLUS:
#if(PROJECTOR_OSRAM)
            if(currentAdjustment<0x1f)
            {
                currentAdjustment++;
                eplosCfgFlag = 1;
            }
#endif
            break;
            
        case REMOTE_MI_MINUS:
#if(PROJECTOR_OSRAM)
            if(currentAdjustment>0)
            {
                currentAdjustment--;
                eplosCfgFlag = 1;
            }
#endif
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

                if((HAL_GetTick() - tickstart) > ((REMOTE_MI_POWER == key) ? 1000000 : 150000)) //repeat press
                {
                    tickstart = HAL_GetTick();
                    flag = 1;
                } else {
                    //printf("repeat escap\r\n");
                }
            }

            if(flag) {
                //printf("IR key:%x\r\n",key);
                flag = 0;
                handle_func_MIkeys(key);
            }
        }
        else
        {
            //NEC controler
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

