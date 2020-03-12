
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
#include "config.h"
#include "delay.h"

ACT_CMD act_cmd;

static uint32_t          tickstart;

extern TIM_HandleTypeDef htim1,htim2,htim3;
extern uint8_t           single_cmd[CMD_LEN_MAX];
extern uint8_t           photoProgramIdx;
extern uint8_t           lightProgramIdx;
extern uint8_t           filmProgramIdx;
extern uint16_t          filmFrameIdx;
extern uint8_t           runFlag;

#if(PROJECTOR_OSRAM || PROJECTOR_CUBE)
extern FILE_INFO_S       film_file[MAX_FILM_FOLDER];
extern BYTE              photo_filename[MAX_FILE_NUM][FILE_NAME_LEN_SHORT];
extern BYTE              light_filename[MAX_FILE_NUM][FILE_NAME_LEN_SHORT];
extern uint16_t          fileTotalPhoto;
extern uint16_t          fileTotallight;
#endif

#if(PROJECTOR_OSRAM)
extern uint8_t           eplosCfgFlag;
extern uint8_t           eplosSLPxen;
extern uint8_t           currentAdjustment;
extern uint8_t           currentAdjustId;//0 ~ 0x1f
extern uint8_t           currentAdjustList[4];
extern uint8_t           osram_clock[MATRIX_SIZE][MATRIX_SIZE];
extern uint8_t           osram_buff[MATRIX_SIZE][MATRIX_SIZE];

#elif(PROJECTOR_CUBE)
extern BOOL              photoOpenFlag;
extern uint8_t           cubeSoftFrameId;
extern uint8_t           newReqFlag;
extern BOOL              cubeRGBStatus;
extern uint32_t          brightness;//init in appinit
extern BOOL              lightingStatus;
#endif

extern PROGRAMS_TYPE_E   programsType;
extern uint8_t           powerFlag;

//extern BYTE              film_filename[MAX_FILM_FRAME][FILE_NAME_LEN];
//extern BYTE              film_foldername[MAX_FILM_FOLDER][FILE_NAME_LEN];
extern uint16_t          filmTotalProgram;

static int8_t Drv_IR_CMD_Handler(uint8_t code, uint16_t key);
static int8_t Drv_MOTOR_CMD_Handler(uint8_t code, uint16_t param);

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
            #if(PROJECTOR_CUBE)
            runFlag = true;
            cubeRGBStatus = !cubeRGBStatus;
            newReqFlag = true;
            photoOpenFlag = true;
            printf("cubeRGBStatus [%d]\r\n", cubeRGBStatus);
            if(cubeRGBStatus)
            {
                if(lightingStatus)
                {
                    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 0);
                    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, 0);
                }
            }
            else
            {
                OffRGB();
                Delay_ms(100);
                if(lightingStatus)
                {
                    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, brightness);
                    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, brightness);
                }
            }
            
            #elif(PROJECTOR_OSRAM)
            runFlag = true;
            programsType = APP;
            #endif
            break;

        case REMOTE_MI_MENU:
            break;

        case REMOTE_MI_POWER:
            #if(PROJECTOR_OSRAM)
            eplosSLPxen = !powerFlag;
            eplosCfgFlag = true;
            runFlag = true;
            #elif(PROJECTOR_CUBE || PROJECTOR_FOCUS)
            printf("Power:%d\r\n",powerFlag);
            if(powerFlag)
                drv_pwm_on();
            else
                drv_pwm_off(0);
            #endif
            break;

        case REMOTE_MI_UP:
            if(programsType==FILM)
            {
                #if(PROJECTOR_CUBE)
                if(cubeRGBStatus)
                #endif
                {
                    if(filmProgramIdx<0xff)
                        filmProgramIdx++;
                    else
                        filmProgramIdx = 0;

                    filmFrameIdx = 0;//切换影片频道，从片头开始
                    #if(PROJECTOR_CUBE||PROJECTOR_OSRAM)
                    #ifdef LARGE_FILE
                    SD_OpenFilmData();
                    #endif
                    #endif
                }
                #if(PROJECTOR_CUBE||PROJECTOR_OSRAM)
                printf("film [%s]\r\n", film_file[filmProgramIdx%filmTotalProgram].foldername);
                #endif
            }
            else if(programsType==PHOTO)
            {
                #if(PROJECTOR_CUBE)
                if(cubeRGBStatus)
                #endif
                {
                    if(photoProgramIdx<0xff)
                        photoProgramIdx++;
                    else
                        photoProgramIdx = 0;

                    runFlag = true;
                }
                #if(PROJECTOR_CUBE||PROJECTOR_OSRAM)
                printf("photo [%s]\r\n", photo_filename[photoProgramIdx%fileTotalPhoto]);
                #endif
            }
            #if(PROJECTOR_OSRAM)
            else if(programsType==LIGHT)
            {
                if(lightProgramIdx<0xff)
                    lightProgramIdx++;
                else
                    lightProgramIdx = 0;

                runFlag = true;
                printf("photo [%s]\r\n", light_filename[lightProgramIdx%fileTotallight]);
            }
            #endif
            #if(CUBE_MASTER)
            else
            {
                #if(PROJECTOR_CUBE)
                if(cubeRGBStatus)
                #endif
                {
                    newReqFlag = true;
                    if(cubeSoftFrameId<0xff)
                        cubeSoftFrameId++;
                    else
                        cubeSoftFrameId = 0;
                }
                printf("soft [%d]\r\n", cubeSoftFrameId%SOFT_PROGRAMS_MAX);
            }
            #endif
            break;

        case REMOTE_MI_DOWN:
            if(programsType==FILM)
            {
                #if(PROJECTOR_CUBE)
                if(cubeRGBStatus)
                #endif
                {
                    if(filmProgramIdx>0)
                        filmProgramIdx--;
                    else
                        filmProgramIdx=0xff;

                    filmFrameIdx = 0;//切换影片频道，从片头开始
                    #if(PROJECTOR_CUBE||PROJECTOR_OSRAM)
                    #ifdef LARGE_FILE
                    SD_OpenFilmData();
                    #endif
                    #endif
                }
                #if(PROJECTOR_CUBE||PROJECTOR_OSRAM)
                printf("film [%s]\r\n", film_file[filmProgramIdx%filmTotalProgram].foldername);
                #endif
            }
            else if(programsType==PHOTO)
            {
                #if(PROJECTOR_CUBE)
                if(cubeRGBStatus)
                #endif
                {
                    if(photoProgramIdx>0)
                        photoProgramIdx--;
                    else
                        photoProgramIdx=0xff;

                    runFlag = true;
                }
                #if(PROJECTOR_CUBE||PROJECTOR_OSRAM)
                printf("photo [%s]\r\n", photo_filename[photoProgramIdx%fileTotalPhoto]);
                #endif
            }
            #if(CUBE_MASTER)
            else
            {
                #if(PROJECTOR_CUBE)
                if(cubeRGBStatus)
                #endif
                {
                    newReqFlag = true;
                    if(cubeSoftFrameId>0)
                        cubeSoftFrameId--;
                    else
                        cubeSoftFrameId=0xff;
                }
                printf("soft [%d]\r\n", cubeSoftFrameId%SOFT_PROGRAMS_MAX);
            }
            #endif
            break;

        case REMOTE_MI_LEFT:
            #if(PROJECTOR_OSRAM)
            Drv_MOTOR_CMD_Handler(CMD_OP_MOTOR_SET_FORWARD, 4);
            #endif
            break;
        
        case REMOTE_MI_RIGHT:
            #if(PROJECTOR_OSRAM)
            Drv_MOTOR_CMD_Handler(CMD_OP_MOTOR_SET_BACKWARD, 4);
            #endif
            break;

        case REMOTE_MI_OK:
            #if(PROJECTOR_CUBE)
            if(cubeRGBStatus)
            #endif
            {
                runFlag = true;
                #if(PROJECTOR_CUBE)
                newReqFlag = true;
                OffRGB();
                #endif
                //photoOpenFlag = true;
                programsType = (programsType+1)%MAX_PROGRAMS;
                #if(PROJECTOR_CUBE||PROJECTOR_OSRAM)
                if(programsType==FILM)
                {
                    filmFrameIdx = 0;//切换影片频道，从片头开始
                    printf("film [%s]\r\n", film_file[filmProgramIdx%filmTotalProgram].foldername);

                    #ifdef LARGE_FILE
                    SD_OpenFilmData();
                    #endif
                }
                else if(programsType==PHOTO)
                {
                    #if(PROJECTOR_CUBE)
                    photoOpenFlag = true;
                    #endif
                    printf("photo [%s]\r\n", photo_filename[photoProgramIdx%fileTotalPhoto]);
                }
                #if(PROJECTOR_CUBE)
                else if(programsType==AUTO_ALGORITHM)
                {
                    printf("soft [%d]\r\n", cubeSoftFrameId%SOFT_PROGRAMS_MAX);
                }
                #endif
                #if(PROJECTOR_OSRAM)
                else if(programsType==CLOCK)
                {
                    memcpy(osram_buff,osram_clock,MATRIX_SIZE*MATRIX_SIZE);
                }
                #endif
                #endif
            }
            break;

        case REMOTE_MI_BACK:
            break;
        case REMOTE_MI_PLUS:
            #if(PROJECTOR_OSRAM)
            //if(currentAdjustment<0x1f)
            {
                //currentAdjustment++;
                currentAdjustment = currentAdjustList[currentAdjustId++%4];
                printf("currentAdjust %d\r\n",currentAdjustment);
                eplosCfgFlag = true;
            }
            #elif(PROJECTOR_CUBE)
            if(powerFlag)
            {
                drv_pwm_speed_stepup(LIGHTING_STEP);
            }
            #endif
            break;

        case REMOTE_MI_MINUS:
            #if(PROJECTOR_OSRAM)
            runFlag = true;
            programsType = FIXED;
            /*
            if(currentAdjustment>0)
            {
                currentAdjustment--;
                eplosCfgFlag = true;
            }*/
            #elif(PROJECTOR_CUBE)
            if(powerFlag)
            {
                drv_pwm_speed_stepdown(LIGHTING_STEP);
            }
            #endif
            break;

        default:
            printf("invalid key[0x%x]!\r\n",key);
    }
}

#if(PROJECTOR_OSRAM)
static int8_t Drv_MOTOR_CMD_Handler(uint8_t code, uint16_t param)
{
    int8_t rc = HAL_OK;

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
#endif

static int8_t Drv_IR_CMD_Handler(uint8_t code, uint16_t key)
{
    //static uint8_t  repeatflag = 0;
    //static uint16_t lastKey = 0;

    if (code == CMD_OP_IR_CODE)
    {
        // MI controler
        if(key>>8 == 0xff) 
        {
            /*
            if(lastKey != key) 
            {
                lastKey = key;
                repeatflag = 1;
                tickstart = HAL_GetTick();
            }
            else 
            {

                if((HAL_GetTick() - tickstart) > ((REMOTE_MI_POWER == key) ? 1000000 : 150000)) //repeat press
                {
                    tickstart = HAL_GetTick();
                    repeatflag = 1;
                }
                else 
                {
                    //printf("repeat escap\r\n");
                }
            }

            if(repeatflag) */
            {
                //repeatflag = 0;

                #if(!PROJECTOR_CUBE)
                if(powerFlag || (!powerFlag&&key==REMOTE_MI_POWER))
                #endif
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

