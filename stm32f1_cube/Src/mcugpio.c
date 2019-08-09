    /***************************************************************************
    *   @file   mbi5124.c
    *   @version V1.0.0
    *   @brief   MBI5124驱动相关函数
   ***************************************************************************
   *  @description
    *
    *  驱动时序
    * 
    ***************************************************************************
***/

#include "stm32f1xx.h"
#include "delay.h"
#include "gpio.h"
#include "mcugpio.h"

#if(PROJECTOR_MCUGPIO)
uint64_t systime = 0;
uint64_t systime1 = 0;
uint8_t chnFlagPos[16][SECS];//1-8
static uint8_t  currentLine=0;


void MCU_GPIO_SET(SIGNAL_PORT_E signalport, uint8_t ch, PULL_STATUS_E pullup)
{
    if(signalport==COLUMN_CS)
    {
        if(pullup==PULLUP)
        {
            switch(ch)
            {
                case 0:
                    COL_PULL_UP(COLUMN_0)
                    break;
                case 1:
                    COL_PULL_UP(COLUMN_1)
                    break;
                case 2:
                    COL_PULL_UP(COLUMN_2)
                    break;
                case 3:
                    COL_PULL_UP(COLUMN_3)
                    break;
                case 4:
                    COL_PULL_UP(COLUMN_4)
                    break;
                case 5:
                    COL_PULL_UP(COLUMN_5)
                    break;
                case 6:
                    COL_PULL_UP(COLUMN_6)
                    break;
                case 7:
                    COL_PULL_UP(COLUMN_7)
                    break;
                case 8:
                    COL_PULL_UP(COLUMN_8)
                    break;
                case 9:
                    COL_PULL_UP(COLUMN_9)
                    break;
                case 10:
                    COL_PULL_UP(COLUMN_10)
                    break;
                case 11:
                    COL_PULL_UP(COLUMN_11)
                    break;
                case 12:
                    COL_PULL_UP(COLUMN_12)
                    break;
                case 13:
                    COL_PULL_UP(COLUMN_13)
                    break;
                case 14:
                    COL_PULL_UP(COLUMN_14)
                    break;
                case 15:
                    COL_PULL_UP(COLUMN_15)
                    break;
            }
        }
        else
        {
            switch(ch)
            {
                case 0:
                    COL_PULL_DOWN(ROW_0)
                    break;
                case 1:
                    COL_PULL_DOWN(ROW_1)
                    break;
                case 2:
                    COL_PULL_DOWN(ROW_2)
                    break;
                case 3:
                    COL_PULL_DOWN(ROW_3)
                    break;
                case 4:
                    COL_PULL_DOWN(ROW_4)
                    break;
                case 5:
                    COL_PULL_DOWN(ROW_5)
                    break;
                case 6:
                    COL_PULL_DOWN(ROW_6)
                    break;
                case 7:
                    COL_PULL_DOWN(ROW_7)
                    break;
                case 8:
                    COL_PULL_DOWN(ROW_8)
                    break;
                case 9:
                    COL_PULL_DOWN(ROW_9)
                    break;
                case 10:
                    COL_PULL_DOWN(ROW_10)
                    break;
                case 11:
                    COL_PULL_DOWN(ROW_11)
                    break;
                case 12:
                    COL_PULL_DOWN(ROW_12)
                    break;
                case 13:
                    COL_PULL_DOWN(ROW_13)
                    break;
                case 14:
                    COL_PULL_DOWN(ROW_14)
                    break;
                case 15:
                    COL_PULL_DOWN(ROW_15)
                    break;
            }
        }
    }
    else
    {
        if(pullup==PULLUP)
        {
            switch(ch)
            {
                case 0:
                    ROW_PULL_UP(ROW_0)
                    break;
                case 1:
                    ROW_PULL_UP(ROW_1)
                    break;
                case 2:
                    ROW_PULL_UP(ROW_2)
                    break;
                case 3:
                    ROW_PULL_UP(ROW_3)
                    break;
                case 4:
                    ROW_PULL_UP(ROW_4)
                    break;
                case 5:
                    ROW_PULL_UP(ROW_5)
                    break;
                case 6:
                    ROW_PULL_UP(ROW_6)
                    break;
                case 7:
                    ROW_PULL_UP(ROW_7)
                    break;
                case 8:
                    ROW_PULL_UP(ROW_8)
                    break;
                case 9:
                    ROW_PULL_UP(ROW_9)
                    break;
                case 10:
                    ROW_PULL_UP(ROW_10)
                    break;
                case 11:
                    ROW_PULL_UP(ROW_11)
                    break;
                case 12:
                    ROW_PULL_UP(ROW_12)
                    break;
                case 13:
                    ROW_PULL_UP(ROW_13)
                    break;
                case 14:
                    ROW_PULL_UP(ROW_14)
                    break;
                case 15:
                    ROW_PULL_UP(ROW_15)
                    break;
            }
        }
        else
        {
            switch(ch)
            {
                case 0:
                    ROW_PULL_DOWN(ROW_0)
                    break;
                case 1:
                    ROW_PULL_DOWN(ROW_1)
                    break;
                case 2:
                    ROW_PULL_DOWN(ROW_2)
                    break;
                case 3:
                    ROW_PULL_DOWN(ROW_3)
                    break;
                case 4:
                    ROW_PULL_DOWN(ROW_4)
                    break;
                case 5:
                    ROW_PULL_DOWN(ROW_5)
                    break;
                case 6:
                    ROW_PULL_DOWN(ROW_6)
                    break;
                case 7:
                    ROW_PULL_DOWN(ROW_7)
                    break;
                case 8:
                    ROW_PULL_DOWN(ROW_8)
                    break;
                case 9:
                    ROW_PULL_DOWN(ROW_9)
                    break;
                case 10:
                    ROW_PULL_DOWN(ROW_10)
                    break;
                case 11:
                    ROW_PULL_DOWN(ROW_11)
                    break;
                case 12:
                    ROW_PULL_DOWN(ROW_12)
                    break;
                case 13:
                    ROW_PULL_DOWN(ROW_13)
                    break;
                case 14:
                    ROW_PULL_DOWN(ROW_14)
                    break;
                case 15:
                    ROW_PULL_DOWN(ROW_15)
                    break;
            }
        }
    }
}


void ROW_ScanSelect(void)
{
    uint8_t row;
    delay(1);

    //all ROW_X_L

    for(row=0;row<16;row++)
    {
        MCU_GPIO_SET(ROW_CS,row,PULLDOWN);
    }
    MCU_GPIO_SET(ROW_CS,currentLine,PULLUP);
}

uint8_t MCU_SdiInput_null(void)
{
    uint16_t ch=0;

    for(ch = 0; ch < 16; ch++)//所有列信号拉低
    {
        MCU_GPIO_SET(COLUMN_CS,ch,PULLDOWN);
    }
    return 0;
}

uint8_t MCU_SdiInput_X(uint8_t type)
{
    uint8_t ch=0;

    //设置16channel电位
    for(ch = 0; ch < 16; ch++)//每个MBI5153有16个通道
    {
        //printf("L %d  C %d\r\n",currentLine,ch);
        switch(0/*type%2*/)
        {
            case 0:
                //if(((ch==15)||(ch==0)||(line==0)||(line==15)||(line==ch)||(ch+line==15))&&
                if(((currentLine==1)||(currentLine==8))&&(/*(line==2)||*/(ch==type%16)))
                {
                    MCU_GPIO_SET(COLUMN_CS,ch,PULLUP);
                }
                else
                {
                    MCU_GPIO_SET(COLUMN_CS,ch,PULLDOWN);
                }
                break;

            default:
                MCU_GPIO_SET(COLUMN_CS,ch,PULLDOWN);
                break;
        }
        //delay(1);
        if(ch==15 && currentLine==15)
        {
            currentLine = 0;
            delay(1);
            return 1;
        }
        else if(ch==15)
        {
            currentLine++;
        }
    }
    delay(1);
    return 0;
}

uint8_t MCU_SdiInput_Sink()
{
    uint16_t ch,l;

    //写入16通道数据
    for(ch = 0; ch < 16; ch++)//每个MBI5153有16个通道
    {
        MCU_GPIO_SET(COLUMN_CS,ch,PULLDOWN);
        for(l=0;l<SECS;l++)//支持连续的流水灯段
        {
            if((chnFlagPos[ch][l]-1==currentLine)||(chnFlagPos[ch][l]-2==currentLine)||(chnFlagPos[ch][l]==currentLine))
                MCU_GPIO_SET(COLUMN_CS,ch,PULLUP);
        }

        if(ch==15 && currentLine==15)
        {
            currentLine = 0;
            return 1;
        }
        else if(ch==15)
        {
            currentLine++;
        }
    }
    return 0;

}

void cycleScan_X(uint8_t type)
{
    MCU_SdiInput_X(type);
    MCU_SdiInput_null();
    ROW_ScanSelect();
}

void cycleScan_Sink(void)
{
    MCU_SdiInput_Sink();

    ROW_ScanSelect();
}

void MCUGpio_X(void)
{
    static uint8_t key_flag = 0;   //按键标志

    if(HAL_GetTick() - systime>500000)//500ms
    {
        key_flag++;
        systime = HAL_GetTick();
    }
    cycleScan_X(key_flag);
}

void MCUGpio_Sink(void)
{
    int number;

    if(HAL_GetTick() - systime>100000)//100ms
    {
        number = (HAL_GetTick()-systime)%16;//8-15
        systime = HAL_GetTick();

        printf("[MBI5124_Sink]rand chn:%d\r\n", number);
        for(int j=0;j<SECS;j++)
        {
            if(chnFlagPos[number][j]==0)
            {
                chnFlagPos[number][j] = 1;
                break;
            }
        }
    }
    cycleScan_Sink();

    if(HAL_GetTick() - systime1>30000)//30ms
    {
        for(int i=0;i<16;i++)
        {
            for(int j=0;j<SECS;j++)
            {
                if(chnFlagPos[i][j]>0 && chnFlagPos[i][j]<SCAN_LINE+3)
                    chnFlagPos[i][j]++;
                else
                    chnFlagPos[i][j] = 0;
                //printf("Pos[%d]:%d\r\n", i,chnFlagPos[i]);
            }
        }
        systime1 = HAL_GetTick();
    }
}
#endif

