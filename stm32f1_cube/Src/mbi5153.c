    /***************************************************************************
    *   @file   mbi5153.c
    *   @version V1.0.0
    *   @brief   MBI5153驱动相关函数
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
#include "mbi5153.h"

#if(PROJECTOR_MBI5153)
extern uint8_t circuit;
extern volatile uint32_t gclk_pluse;
uint64_t systime = 0;
uint64_t systime1 = 0;

//unsigned short sdi_data[16]={1<<0,1<<1,1<<2,1<<3,1<<4,1<<5,1<<6,1<<7,\
//                          1<<8,1<<9,1<<10,1<<11,1<<12,1<<13,1<<14,1<<15};

unsigned short sdi_data=0xffff;
uint8_t chnFlagPos[16][SECS];//1-8

void soft_reset(void)
{
    unsigned int sck_cnt;

    //复位
    delay(1);
    LE_PIN_H
    delay(1);
    for(sck_cnt = 10;sck_cnt > 0;sck_cnt--)
    {
        DCLK_PIN_H
        delay(1);
        DCLK_PIN_L
        delay(1);
    }
    LE_PIN_L
    delay(1);
}

void vsync(void)
{
    unsigned int sck_cnt;
/*
#if(TWO_TIMER_PULSE==1)
    while(gclk_num!=GCLKNUM){
        delay(100);
    }
#endif
*/
#if(TWO_TIMER_PULSE==0)
    //TIM1->CR1 &= ~(0x01);
    //GCLK_PIN_L
#endif

    //TIM1->CCR1 = 0;//duty cycle
    delay(10);
    //2个clk拉高LE，发送Vsync
    LE_PIN_H
    for(sck_cnt = 0;sck_cnt < 2;sck_cnt ++)
    {
        delay(1);
        DCLK_PIN_H
        delay(1);
        DCLK_PIN_L
    }
    LE_PIN_L
    delay(1);//LE下降沿与gclk上升沿满足要求
}

void pre_active(void)
{
    unsigned int sck_cnt;
    
    LE_PIN_H
    delay(1);
    for(sck_cnt = 14;sck_cnt > 0;sck_cnt --)
    {
        DCLK_PIN_H
        delay(1);
        DCLK_PIN_L
        delay(1);
    }
    LE_PIN_L
    delay(2);
}

void reg1_config(void)
{
    unsigned int sck_cnt;
    unsigned short j;
    unsigned short state_reg=0xEB | SCAN_LINE_16<<8;//7F 2B  ;//0x00FF
    unsigned int mask;

    for(j = 0; j < MBI5153_SIZE; j++)//N片IC级联
    {
        for(sck_cnt = 0;sck_cnt < 16;sck_cnt ++)
        {
            mask = 0x8000 >> sck_cnt;
            if(j == MBI5153_SIZE-1)
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

void reg2_config(void)
{
    unsigned int sck_cnt;
    unsigned short j;
    unsigned short state_reg=0x0400;
    unsigned int mask;
    
    for(j = 0; j < MBI5153_SIZE; j++)//N片IC级联
    {
        for(sck_cnt = 0;sck_cnt < 16;sck_cnt ++)
        {
            mask = 0x8000 >> sck_cnt;
            if(j == MBI5153_SIZE-1)
                if(sck_cnt == 8){
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

void reg3_config(void)
{
    unsigned int sck_cnt;
    unsigned short j;
    unsigned short state_reg=0x0000;
    unsigned int mask;
    
    for(j = 0; j < MBI5153_SIZE; j++)//N片IC级联
    {
        for(sck_cnt = 0;sck_cnt < 16;sck_cnt ++)
        {
            mask = 0x8000 >> sck_cnt;
            if(j == MBI5153_SIZE-1)
                if(sck_cnt == 0){
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
    delay(2);
}

void MBI_Init(void)
{
    //前置时间
    pre_active();
    //写状态寄存器1
    reg1_config();

    //前置时间
    pre_active();
    //写状态寄存器1
    reg2_config();

/*
    //前置时间
    pre_active();
    //写状态寄存器1
    reg3_config();
*/
    vsync();
}

void MBI_cycleScanRegConfig(void)
{
    //前置时间
    pre_active();
    //写状态寄存器1
    reg1_config();

    //前置时间
    pre_active();
    //写状态寄存器1
    reg2_config();
/*
    //前置时间
    pre_active();
    //写状态寄存器1
    reg3_config();
*/
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
    static uint8_t ii = 0;

    delay(1);
    if(ii%SCAN_LINE==0)
    {
        AG_DIN_PIN_H
    }
    AG_OE_PIN_H
    delay(1);
    AG_CLK_PIN_H
    delay(1);

    if(ii%SCAN_LINE==0)
    {
        AG_DIN_PIN_L
    }
    AG_OE_PIN_L
    AG_CLK_PIN_L
    ii++;

    delay(1);
    gclk_pluse = 0;
}

uint8_t MBI_SdiInput_X(uint8_t type)
{
    uint32_t mask=0;
    uint16_t ch=0,j=0,k=0,line=0;
    static uint8_t  currentLine=1,currentCh=0;

    //写入16*2*16数据
    //for(line = 1; line <= SCAN_LINE; line++)//扫描行数
    for(line = currentLine; line <= SCAN_LINE; line++)//扫描行数
    {
        for(ch = 0; ch < 16; ch++)//每个MBI5153有16个通道
        {
            if(currentLine==line && currentCh==ch)
            {
                //printf("L %d  C %d\r\n",currentLine,currentCh);
                //delay(1);
                for(j = 0; j < MBI5153_SIZE; j++)//级联IC数量
                {
                    for(k = 0; k < 16; k++)
                    {
                        mask = 0x8000 >> k;

                        if(j == MBI5153_SIZE-1)
                        {
                            if(k == 15)
                            {
                                LE_PIN_H
                            }
                        }

                        switch(0/*type%2*/)
                        {
                            case 0:
                                    if((sdi_data & mask)&&
                                        //((ch==15)||(ch==0)||(line==1)||(line==16)||(line-ch==1)||(ch+line==16)))
                                        ((ch==((type>>4)&0xf)%16)&&(line-1==(type&0xf)%16)))
                                        SDI_PIN_H
                                    else
                                        SDI_PIN_L
                                    break;

                            case 1:
                                    SDI_PIN_L
                                    break;

                            default:
                                    SDI_PIN_L
                                    break;
                        }

                        //delay(1);
                        DCLK_PIN_H
                        delay(1);
                        DCLK_PIN_L
                    }
                }
                LE_PIN_L
                SDI_PIN_L
                delay(1);

                if(currentCh==15 && currentLine==16)
                {
                    //vsync();
                    currentCh = 0;
                    currentLine = 1;
                    return 1;
                }
                else if(currentCh==15)
                {
                    currentCh = 0;
                    currentLine++;
                }
                else
                {
                    currentCh++;
                }
                //delay(1);
                return 0;
            }
        }
    }
}

uint8_t MBI_SdiInput_Sink()
{
    uint32_t mask;
    uint16_t ch,j,k,l,line;
    static uint8_t  currentLine=1,currentCh=0;

    //写入16*2*16数据
    //for(line = 1; line <= SCAN_LINE; line++)//扫描行数
    for(line = currentLine; line <= SCAN_LINE; line++)//扫描行数
    {
        for(ch = 0; ch < 16; ch++)//每个MBI5153有16个通道
        {
            if(currentLine==line && currentCh==ch)
            {
                //printf("L %d  C %d\r\n",currentLine,currentCh);
                //delay(1);
                for(j = 0; j < MBI5153_SIZE; j++)//级联IC数量
                {
                    for(k = 0; k < 16; k++)
                    {
                        mask = 0x8000 >> k;
                        if(j == MBI5153_SIZE-1)
                        {
                            if(k == 15)
                            {
                                LE_PIN_H
                            }
                        }

                        for(l=0;l<SECS;l++)//支持连续的流水灯段
                        {
                            if((sdi_data & mask)&&
                                ((chnFlagPos[ch][l]-1==line)||(chnFlagPos[ch][l]-2==line)||(chnFlagPos[ch][l]==line)))
                                SDI_PIN_H
                        }
                        //delay(1);
                        DCLK_PIN_H
                        delay(1);
                        DCLK_PIN_L
                    }
                }
                LE_PIN_L
                SDI_PIN_L
                delay(1);

                if(currentCh==15 && currentLine==16)
                {
                    //vsync();
                    currentCh = 0;
                    currentLine = 1;
                    return 1;
                }
                else if(currentCh==15)
                {
                    currentCh = 0;
                    currentLine++;
                }
                else
                {
                    currentCh++;
                }
                delay(5);
                return 0;
            }
        }
    }
}

void cycleScan_X(uint8_t type)
{
    uint16_t line;
    static uint8_t  isVsync = 1;

    if(isVsync)
        vsync();

    MBI_ScanCycle();
    if(isVsync)
    {
        MBI_cycleScanRegConfig();
        for(line=0;line<SCAN_LINE-1;line++)
        {
            MBI_ScanCycle();
            delay(200);
        }
        isVsync = 0;
    }
    else
    {
        isVsync = MBI_SdiInput_X(type);
    }
}

void cycleScan_Sink(void)
{
    uint16_t i;
    static uint8_t  isVsync = 1;

    if(isVsync)
        vsync();

    MBI_ScanCycle();
    if(isVsync)
    {
        MBI_cycleScanRegConfig();
        for(i=0;i<SCAN_LINE-1;i++)
        {
            MBI_ScanCycle();
            delay(200);
        }
        isVsync = 0;
    }
    else
    {
        isVsync = MBI_SdiInput_Sink();
    }
}

void MBI5153_X(void)
{
    static uint8_t key_flag = 0;   //按键标志

    if(HAL_GetTick() - systime>500000)//500ms
    {
        key_flag++;
        systime = HAL_GetTick();
    }
    cycleScan_X(key_flag);
}

void MBI5153_Sink(void)
{
    int number;

    if(HAL_GetTick() - systime>100000)//100ms
    {
        number = (HAL_GetTick()-systime)%16;//8-15
        systime = HAL_GetTick();

        printf("rand chn:%d\r\n", number);
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

void MBI5153_play()
{
    unsigned short i,j,k,line;
    unsigned int mask;

    //MBI_Init();
    //vsync();
    //pluse_force = 1;

    //写入16*2*16数据
    for(line = 1; line <= SCAN_LINE; line++)//扫描行数
    {

        for(i = 0; i < 16; i++)//每个MBI5153有16个通道
        {
#if(SECTION_PER_SCAN!=0)
            if((i)%4==0) {
                MBI_ScanDisplay();//scan line switch
                delay(4);
                gclk_pluse = 0;
            }
#endif
            delay(1);
            for(j = 0; j < MBI5153_SIZE; j++)//级联IC数量
            {
                for(k = 0; k < 16; k++)
                {
                    mask = 0x8000 >> k;

                    if(j == MBI5153_SIZE-1)
                    {
                        if(k == 15)
                        {
                            LE_PIN_H
                        }
                    }

                    //if((sdi_data & mask)&&(i==15||i==8))// && (circuit%16==i))
#if 1
                    if((sdi_data & mask)&&(i>=7)&&
                        ((i==14)||(i==7)||(i==10&&line==2)||(i==12&&line==5)))// && (circuit%16==i))
#else
                    if(sdi_data & mask)// && (circuit%16==i))
#endif
                        SDI_PIN_H
                    else
                        SDI_PIN_L

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
    }
    //pluse_enable = 0;
    vsync();
}
#endif

