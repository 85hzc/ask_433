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

#include "stm32f4xx.h"
#include "led.h"
#include "delay.h"
#include "key.h"
#include "usart.h"
#include "gpio.h"
#include "mbi5153.h"

extern uint8_t circuit;
extern volatile uint32_t gclk_pluse;
extern volatile uint16_t gclk_time;
extern volatile uint8_t pluse_enable, pluse_force;

#if(TWO_TIMER_PULSE==1)
extern uint8_t gclk_num;
#endif

//unsigned short sdi_data[16]={1<<0,1<<1,1<<2,1<<3,1<<4,1<<5,1<<6,1<<7,\
//                          1<<8,1<<9,1<<10,1<<11,1<<12,1<<13,1<<14,1<<15};

unsigned short sdi_data=0xffff;

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

    //pluse_enable = 0;

    //TIM1->CCR1 = 0;//duty cycle
    delay(100);
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
#if(TWO_TIMER_PULSE==0)
   // TIM1->CR1 |= 0x01;
#elif(TWO_TIMER_PULSE==1)
    gclk_num = 0;
    //Pulse_output(100,129);
    Pulse_output(129,1000);
#endif
    //TIM1->CCR1 = 50;//duty cycle

    pluse_enable = 1;
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
    unsigned short state_reg=0xEB | SCAN_LINE_8<<8;//7F 2B  ;//0x00FF
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
    delay(2);
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
    delay(2);
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

#ifdef SCAN_SUPPORT
void MBI_ScanDisplay(void)
{
    unsigned int sck_cnt;
    uint8_t i,j,din_cnt;
    static uint8_t ii = 0;
    //uint8_t din=0x00, mask;

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
    unsigned int sck_cnt;
    uint8_t i,j,din_cnt;
    static uint8_t ii = 0;
    //uint8_t din=0x00, mask;

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

    delay(3);
    gclk_pluse = 0;
}

uint8_t MBI_SdiInput_X(uint8_t type)
{
    uint32_t sck_cnt,mask;
    uint16_t i,j,k,line;
    static uint8_t  currentLine=1,currentCh=0;

    //写入16*2*16数据
    for(line = 1; line <= SCAN_LINE; line++)//扫描行数
    {

        for(i = 0; i < 16; i++)//每个MBI5153有16个通道
        {

            if(currentLine==line && currentCh==i)
            {
                //printf("L %d  C %d\r\n",currentLine,currentCh);
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

                        switch(type%2)
                        {
                            case 0:
                                if((sdi_data & mask)&&(i>7)&&
                                    ((i==15)||(i==0)||(line==1)||(line==16)||(i-line==0)||(i+line==16)))
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

                        delay(1);
                        DCLK_PIN_H
                        delay(1);
                        DCLK_PIN_L
                    }
                }
                LE_PIN_L
                SDI_PIN_L
                delay(1);

                if(currentCh==15 && currentLine==8)
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
                delay(100);
                return 0;
            }
        }
    }
}

//uint8_t chnFlag[16];//8-15 channel
uint8_t chnFlagPos[16][SECS];//1-8
uint8_t MBI_SdiInput_Sink()
{
    uint32_t sck_cnt,mask;
    uint16_t i,j,k,l,line;
    static uint8_t  currentLine=1,currentCh=0;

    //写入16*2*16数据
    for(line = 1; line <= SCAN_LINE; line++)//扫描行数
    {
        for(i = 0; i < 16; i++)//每个MBI5153有16个通道
        {
            if(currentLine==line && currentCh==i)
            {
                //printf("L %d  C %d\r\n",currentLine,currentCh);
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

                        for(l=0;l<SECS;l++)//支持连续的流水灯段
                        {
                            if((sdi_data & mask)&&
                                ((chnFlagPos[i][l]-1==line)||(chnFlagPos[i][l]-2==line)||(chnFlagPos[i][l]==line)))
                                SDI_PIN_H
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

                if(currentCh==15 && currentLine==8)
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
                delay(100);
                return 0;
            }
        }
    }
}

#endif

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
    pluse_force = 0;

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

void MBI5153()
{
    unsigned int sck_cnt;
    unsigned short i,j,k,line;
    //unsigned short red1,green1,blue1,red2,green2,blue2;
    unsigned int mask;
    //unsigned int bufaddA,bufaddB;

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
    pluse_force = 0;
    vsync();

#ifdef SCAN_SUPPORT
    //开始显示
    //for(i=0;i<125;i++)
    //while(1)
    //{
    //    MBI_ScanDisplay();
    //}

    //soft_reset();
    //MBI_Init();

#endif
    //pluse_enable = 1;
}

void cycleScan_X(uint8_t type)
{
    uint32_t sck_cnt,mask;
    uint16_t i,j,k,line;
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
            delay(135);
        }
        isVsync = 0;
    }
    else
    {
        isVsync = MBI_SdiInput_X(type);
    }
}

void cycleScan_Sink()
{
    uint32_t sck_cnt,mask;
    uint16_t i,j,k,line;
    static uint8_t  isVsync = 1;

    if(isVsync)
        vsync();

    MBI_ScanCycle();
    if(isVsync)
    {
        MBI_cycleScanRegConfig();
        for(i=0;i<7;i++)
        {
            MBI_ScanCycle();
            delay(135);
        }
        isVsync = 0;
    }
    else
    {
        isVsync = MBI_SdiInput_Sink();
    }
}

