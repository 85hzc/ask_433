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

#if(PROJECTOR_MBI5124)
extern uint8_t circuit;
extern volatile uint32_t gclk_pluse;
uint64_t systime = 0;
uint64_t systime1 = 0;
unsigned short sdi_data=0xffff;
uint8_t chnFlagPos[16][SECS];//1-8
static uint8_t  currentLine=0;

void LE(void)
{
    delay(1);
    LE_PIN_H
    delay(1);
    LE_PIN_L
    delay(1);
}

void MBI_ScanDisplay(void)
{
    static uint8_t ii = 0;

    delay(1);
#if(TIMER==0)
    if(ii%8==0)
    {
        AG_DIN_PIN_H
    }
    AG_OE_PIN_H
    delay(1);
    AG_CLK_PIN_H
    delay(1);

    if(ii%8==0)
    {
        AG_DIN_PIN_L
    }
    AG_OE_PIN_L
    AG_CLK_PIN_L
    ii++;
#else
    for(i = 0;i < SCAN_LINE; i++)
    {
        if(i==0)
        {
            AG_DIN_PIN_H
        }
        AG_OE_PIN_H
        //delay_ns(5);
        delay(1);
        AG_CLK_PIN_H
        //delay_ns(5);
        delay(1);

        if(i==0)
        {
            AG_DIN_PIN_L
        }
        AG_OE_PIN_L
        AG_CLK_PIN_L
        //delay(5);
        //delay(100);
#if 1
        if(i==SCAN_LINE-1)
            return;

#if(SEPR_SECTION==1)
        delay(50);
        for(j=0;j<SECTION_PER_SCAN;j++)
        {
            gclk_pluse = 0;
            delay(200);
        }
#else
        //Delay_ms(3);
#endif
#else

#endif
    }
#endif
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
    for(line = currentLine; line < SCAN_LINE; line++)//扫描行数
    {
        for(ch = 0; ch < 16; ch++)//每个MBI5153有16个通道
        {
            //printf("L %d  C %d\r\n",currentLine,ch);
            //for(j = 0; j < MBI5124_SIZE; j++)//级联IC数量
            {
                switch(0/*type%2*/)
                {
                    case 0:
                        //if(((ch==15)||(ch==0)||(line==0)||(line==15)||(line==ch)||(ch+line==15))&&
                        //if(((line==1)||(line==2)||(line==3)))//&&
                            //(/*(line==2)||*/(line==type%16)))
                            //(/*(line==2)||(ch==type%16))*/)
                        if((ch==type%16)&&(line==6)&&(type%2==0))
                        {
                            SDI_PIN_H
                        }
                        else if((ch==type%16)&&(line==7)&&(type%2==1))
                        {
                            SDI_PIN_H
                        }
                        else
                            SDI_PIN_L
                        break;

                    default:
                        SDI_PIN_L
                        break;
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
}

uint8_t MBI_SdiInput_Sink()
{
    uint16_t ch,j,l,line;

    //写入16*2*16数据
    for(line = currentLine; line < SCAN_LINE; line++)//扫描行数
    {
        for(ch = 0; ch < 16; ch++)//每个MBI5153有16个通道
        {
            //for(j = 0; j < MBI5124_SIZE; j++)//级联IC数量
            {
                for(l=0;l<SECS;l++)//支持连续的流水灯段
                {
                    if((chnFlagPos[ch][l]-1==line)||(chnFlagPos[ch][l]-2==line)||(chnFlagPos[ch][l]==line))
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
}

void cycleScan_X(uint8_t type)
{
    static  int ii=0;
    MBI_SdiInput_X(type);
    LE();
    //MBI_ScanCycle();

    MBI_SdiInput_null();
    LE();

    MBI_ScanCycle();
}

void cycleScan_Sink(void)
{
    MBI_SdiInput_Sink();
    LE();
    MBI_ScanCycle();
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

