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
#include "mbi5124.h"
#include "config.h"

#if(PROJECTOR_MBI5124)
extern uint8_t              runFlag;
extern char                 promptBuffer[16*6][16];
//extern char                 cartoonBuffer[16*20][16];
extern uint16_t             actType;
extern uint16_t             actTime;

uint64_t                    systime = 0;
uint64_t                    systime1 = 0;
uint8_t                     chnFlagPos[16][SECS];//1-8
static uint8_t              currentLine=0;
char                        refreshBuf[16][16];

char fileBuffer[MAX_FILE_SIZE]={
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
            0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,
            0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,
            0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,
            0,0,0,1,1,1,0,0,1,1,1,0,1,1,1,0,
            0,0,0,1,1,1,0,1,1,1,0,1,1,1,1,0,
            0,0,1,1,1,0,0,1,1,1,0,1,1,1,0,0,
            0,0,1,1,1,0,1,1,1,0,0,1,1,0,0,0,
            0,1,1,1,0,1,1,1,0,0,0,1,1,0,1,0,
            0,1,1,1,0,1,1,0,0,0,0,1,1,1,1,0,
            0,1,1,0,0,0,1,0,0,0,0,0,1,1,1,0,
            0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};   // file copy buffer

void LE(void)
{
    delay(1);
    LE_PIN_H
    delay(1);
    LE_PIN_L
    delay(1);
}

void reg_config(void)
{
    unsigned int sck_cnt;
    unsigned short j;
    unsigned short state_reg;
    unsigned int mask;

    //state_reg=0x7D6B;//red 
    state_reg=0xF16B;//green 
    //state_reg=0xED6B;//blue 
    for(j = 0; j < MBI5124_SIZE; j++)//N片IC级联
    {
        for(sck_cnt = 0;sck_cnt < 16;sck_cnt ++)
        {
            mask = 0x8000 >> sck_cnt;
            if(j == MBI5124_SIZE-1)
                if(sck_cnt == 12){
                    LE_PIN_H
                    delay(1);
                }
            
            if(state_reg & mask)
            {
                SDI_PIN_H
            }
            else
            {
                SDI_PIN_L
            }
            delay(1);
            DCLK_PIN_H
            delay(1);
            DCLK_PIN_L
        }
    }
    LE_PIN_L
    SDI_PIN_L
    delay(1);
}


void MBI_ScanCycle(void)
{
    delay(1);
    if((currentLine)%SCAN_LINE==0)
    {
        AG_DIN_PIN_H
    }
    AG_OE_PIN_H
    delay(1);
    AG_CLK_PIN_H
    delay(1);

    if((currentLine)%SCAN_LINE==0)
    {
        AG_DIN_PIN_L
    }
    AG_OE_PIN_L
    AG_CLK_PIN_L
    delay(1);
}

uint8_t MBI_SdiInput_null(void)
{
    uint16_t ch=0;

    for(ch = 0; ch < 16; ch++)//每个MBI5153有16个通道
    {
        SDI_PIN_L
        DCLK_PIN_H
        delay(1);
        DCLK_PIN_L
        SDI_PIN_L
        delay(1);
    }
    return 0;
}

uint8_t MBI_SdiInput_X(uint8_t type)
{
    uint16_t ch=0,j=0,line=0;

    //写入16*2*16数据
    line = currentLine;
    for(ch = 0; ch < 16; ch++)//每个MBI5153有16个通道
    {
        //printf("L %d  C %d\r\n",currentLine,ch);
        //for(j = 0; j < MBI5124_SIZE; j++)//级联IC数量
        {
            if(actType==1)
            {
                if(((ch==15)||(ch==0)||(line==0)||(line==15)||(line==ch)||(ch+line==15)))
                {
                    SDI_PIN_H
                }
                else
                {
                    SDI_PIN_L
                }
            }
            else if(actType==2)
            {
                if(((line==1)||(line==2)||(line==3)))
                {
                    SDI_PIN_H
                }
                else
                {
                    SDI_PIN_L
                }
            }
            else if(actType==3)
            {
                if((line==type%16)&&(ch==type%16))
                {
                    SDI_PIN_H
                }
                else
                {
                    SDI_PIN_L
                }
            }
            else if(actType==4)
            {
                if((ch==type%16)&&(line==6)&&(type%2==0))
                {
                    SDI_PIN_H
                }
                else if((ch==type%16)&&(line==7)&&(type%2==1))
                {
                    SDI_PIN_H
                }
                else
                {
                    SDI_PIN_L
                }
            }
            else if(actType==5)
            {
                if((ch==type%16))
                {
                    SDI_PIN_H
                }
                else
                {
                    SDI_PIN_L
                }
            }
            else if(actType==6)
            {
                if((line==type%16))
                {
                    SDI_PIN_H
                }
                else
                {
                    SDI_PIN_L
                }
            }
            else if(actType==8)
            {
                SDI_PIN_H
            }
            else if(actType==7)
            {
                if((line==type%16)||(ch==type%16))
                {
                    SDI_PIN_H
                }
                else
                {
                    SDI_PIN_L
                }
            }
            else if(actType==9)
            {
                if((line==type%16)&&((ch==1)||(ch==2)||(ch==3)))
                {
                    SDI_PIN_H
                }
                else
                {
                    SDI_PIN_L
                }
            }
            else if(actType==10)
            {
                SDI_PIN_L
            }

            DCLK_PIN_H
            delay(1);
            DCLK_PIN_L
        }
        SDI_PIN_L
        delay(1);

        if(ch==15 && currentLine==15)
        {
            currentLine = 0;
            return 1;
        }
        else if(ch==15)
        {
            currentLine++;
        }
        /*else
        {
            currentCh++;
        }*/
    }
    //delay(1);
    return 0;
}

uint8_t MBI_SdiInput_Sink()
{
    uint16_t ch,j,l,line;

    //写入16*2*16数据
    line = currentLine;
    for(ch = 0; ch < 16; ch++)//每个MBI5153有16个通道
    {
        //for(j = 0; j < MBI5124_SIZE; j++)//级联IC数量
        {
            for(l=0;l<SECS;l++)//支持连续的流水灯段
            {
                if((chnFlagPos[ch][l]==line+1)||(chnFlagPos[ch][l]==line+2)||(chnFlagPos[ch][l]==line))
                    SDI_PIN_H
            }
            //delay(1);
            DCLK_PIN_H
            delay(1);
            DCLK_PIN_L
        }
        SDI_PIN_L
        delay(1);

        if(ch==15 && currentLine==15)
        {
            currentLine = 0;
            return 1;
        }
        else if(ch==15)
        {
            currentLine++;
        }
        /*else
        {
            currentCh++;
        }*/
    }
    return 0;
}

uint8_t MBI_SdiInput_Play()
{
    uint16_t ch,j,l,line;

    //写入16*2*16数据
    line = currentLine;
    for(ch = 0; ch < 16; ch++)//每个MBI5153有16个通道
    {
        //for(j = 0; j < MBI5124_SIZE; j++)//级联IC数量
        {
            if(refreshBuf[line][ch])
                SDI_PIN_H

            DCLK_PIN_H
            delay(1);
            DCLK_PIN_L
        }
        SDI_PIN_L
        delay(1);

        if(ch==15 && currentLine==15)
        {
            currentLine = 0;
            return 1;
        }
        else if(ch==15)
        {
            currentLine++;
        }
        /*else
        {
            currentCh++;
        }*/
    }
    return 0;
}

void cycleScan_X(uint8_t type)
{
    MBI_ScanCycle();

    MBI_SdiInput_X(type);
    LE();
    MBI_SdiInput_null();
    LE();
    delay(actTime);
}

void cycleScan_Sink(void)
{
    MBI_ScanCycle();

    MBI_SdiInput_Sink();
    LE();
    MBI_SdiInput_null();
    LE();
}

void cycleScan_Play(void)
{
    MBI_ScanCycle();

    MBI_SdiInput_Play();
    LE();
    MBI_SdiInput_null();
    LE();
}

void readFromCaption()
{
    UINT i,j=0;;
    char tmp;

    for(i=0;i<16;i++)
    {
        tmp = fileBuffer[i*16+15];
        for(j=15;j>0;j--)
        {
            fileBuffer[i*16+j] = fileBuffer[i*16+j-1];
        }

        fileBuffer[i*16] = tmp;
    }
    /*
    for(i=0;i<16;i++)
    {
        printf("\r\n");
        for(j=0;j<16;j++)
            printf("%d ",fileBuffer[i*16+j]);
    }*/
}

void MBI5124_X(void)
{
    static uint8_t key_flag = 0;   //按键标志

    if(HAL_GetTick() - systime>500000)//500ms
    {
        key_flag++;
        systime = HAL_GetTick();
    }
    cycleScan_X(key_flag);
}

void MBI5124_Sink(void)
{
    int number;

    if((HAL_GetTick() - systime>100000) && runFlag)//100ms
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

    if((HAL_GetTick() - systime1>30000) && runFlag)//30ms
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

void MBI5124_Play(void)
{
    uint16_t j;

    if((HAL_GetTick() - systime>20000) && runFlag)//100ms
    {
        readFromCaption();
        memcpy(refreshBuf,fileBuffer,256);
        /*for(j=0;j<256;j++)
        {
            refreshBuf[j/16][j%16] = fileBuffer[j];
        }*/
        systime = HAL_GetTick();
    }
    cycleScan_Play();
}

void MBI5124_Prompt(void)
{
    static uint8_t j=0;

    if((HAL_GetTick() - systime>300000) && runFlag)//100ms
    {
        memcpy(refreshBuf,promptBuffer[16*(j%6)],256);
        j++;
        /*for(j=0;j<256;j++)
        {
            refreshBuf[j/16][j%16] = fileBuffer[j];
        }*/
        systime = HAL_GetTick();
    }
    cycleScan_Play();
}
#if 0
void MBI5124_Cartoon(void)
{
    static uint8_t j=0;

    if((HAL_GetTick() - systime>70000) && runFlag)//100ms
    {
        memcpy(refreshBuf,cartoonBuffer[16*(j%20)],256);
        j++;
        /*for(j=0;j<256;j++)
        {
            refreshBuf[j/16][j%16] = fileBuffer[j];
        }*/
        systime = HAL_GetTick();
    }
    cycleScan_Play();
}
#endif
#endif

