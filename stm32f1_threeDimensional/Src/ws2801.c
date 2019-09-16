#include "stm32f1xx.h"
#include "delay.h"
#include "gpio.h"
#include "eplos.h"
#include "config.h"

extern uint8_t                  runFlag;
extern char                     fileBuffer[MAX_FILE_SIZE];
extern USART_TransmiteTYPE      Usart1Tx;

uint8_t                         GData[CHIP_SIZE], RData[CHIP_SIZE], BData[CHIP_SIZE];

static uint64_t                 systime = 0;

void Send_8bits(uint8_t dat)
{
    uint8_t i;

    delay_ns(1);
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            DCLK_PIN_H;
            delay_ns(3);
            DCLK_PIN_L;
            delay_ns(1);
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            DCLK_PIN_H;
            delayns_300();
            DCLK_PIN_L;
            delay_ns(3);
        }
        dat=dat<<1;
    }
    DCLK_PIN_L;
}

void Send_2811_oneString()        //传送16位灰度数据    ,三组相同
{
    uint8_t i=0;

    //__set_FAULTMASK(1); // 关闭调度器
    __disable_irq();
    DCLK_PIN_L;
    /*****************16*3组灰度数据********************************/
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits(GData[i]);
        Send_8bits(RData[i]);
        Send_8bits(BData[i]);
/*
        Send_8bits(0x1f);
        Send_8bits(0x4f);
        Send_8bits(0x7f);*/
    }
    //__set_FAULTMASK(0); //打开调度器
    __enable_irq();
}


void Send_2811_totalPixels(uint8_t GData,uint8_t RData,uint8_t BData)
{
    uint8_t i=0;

    __disable_irq();
    DCLK_PIN_L;
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits(GData);
        Send_8bits(RData);
        Send_8bits(BData);
    }
    __enable_irq();

}

void UartDataHandle(uint8_t *data)
{
    uint8_t i,j;
    uint16_t IR_code = 0;

    printf("data:%s\r\n",data);

    for(i=0;i<IO_SIZE;i++)
    {
        for(j=0;j<CHIP_SIZE;j++)
        {
            GData[j] = data[i*IO_SIZE+j*CHIP_SIZE+0];
            RData[j] = data[i*IO_SIZE+j*CHIP_SIZE+1];
            BData[j] = data[i*IO_SIZE+j*CHIP_SIZE+2];
        }
        Send_2811_oneString();
    }
}

void ColorSchedule()
{
    uint8_t j,rcolor,gcolor,bcolor;
    uint16_t IR_code = 0;
    static uint32_t GRB=0xFFFFFF;
    static RGB_Switch_E rgbsw = RGB_B_minus;

    printf("RGB:%x %x %x\r\n",(GRB>>8)&0xff,(GRB>>16)&0xff,(GRB)&0xff);

    gcolor = (GRB>>16)&0xff;
    rcolor = (GRB>>8)&0xff;
    bcolor = GRB&0xff;

    switch(rgbsw)
    {
        case RGB_B_minus:
            if(bcolor>0)
                bcolor = bcolor-1;
            else
                rgbsw = RGB_G_minus;
            break;

        case RGB_G_minus:
            if(gcolor>0)
                gcolor = gcolor-1;
            else
                rgbsw = RGB_R_minus;
            break;

        case RGB_R_minus:
            if(rcolor>0)
                rcolor = rcolor-1;
            else
                rgbsw = RGB_B_plus;
            break;


        case RGB_B_plus:
            if(bcolor<0xff)
                bcolor = bcolor+1;
            else
                rgbsw = RGB_G_plus;
            break;

        case RGB_G_plus:
            if(gcolor<0xff)
                gcolor = gcolor+1;
            else
                rgbsw = RGB_R_plus;
            break;

        case RGB_R_plus:
            if(rcolor<0xff)
                rcolor = rcolor+1;
            else
                rgbsw = RGB_G1_minus;
            break;

        case RGB_G1_minus:
            if(gcolor>0)
                gcolor = gcolor-1;
            else
                rgbsw = RGB_B1_minus;
            break;

        case RGB_B1_minus:
            if(bcolor>0)
                bcolor = bcolor-1;
            else
                rgbsw = RGB_G1_plus;
            break;

        case RGB_G1_plus:
            if(gcolor<0xff)
                gcolor = gcolor+1;
            else
                rgbsw = RGB_R1_minus;
            break;

        case RGB_R1_minus:
            if(rcolor>0)
                rcolor = rcolor-1;
            else
                rgbsw = RGB_B1_plus;
            break;
        
        case RGB_B1_plus:
            if( bcolor<0XFF)
                bcolor = bcolor+1;
            else
                rgbsw = RGB_R1_plus;
            break;

        case RGB_R1_plus:
            if(rcolor<0xff)
                rcolor = rcolor+1;
            else
                rgbsw = RGB_B_minus;
            break;

    }
    GRB = gcolor<<16 | rcolor<<8 | bcolor;

    for(j=0;j<CHIP_SIZE;j++)
    {
        GData[j] = (GRB>>16)&0xff;
        RData[j] = (GRB>>8)&0xff;
        BData[j] = GRB&0xff;

    }
    Send_2811_oneString();
}


void Compose_RGB(RGB_Type_E RGB, uint8_t index, uint8_t gray)
{
    memset(GData, 0, sizeof(GData));
    memset(RData, 0, sizeof(RData));
    memset(BData, 0, sizeof(BData));

    if(RGB&G)
        GData[index] = gray;
    if(RGB&R)
        RData[index] = gray;
    if(RGB&B)
        BData[index] = gray;
}


void WS2801_play(void)
{
    static uint8_t j=0;

    //if((HAL_GetTick() - systime>20000) && runFlag)//100ms
    {
#if 0
        memcpy(displayMatrix,cartoonBuffer[32*(j%20)],1024);
        j++;
#else
#if 0
        SD_ReadFileData();
        //for(j=0;j<1024;j++)
        //{
        //    displayMatrix[j/32][j%32] = fileBuffer[j];
        //}
        memcpy(Usart1Tx.TX_pData,fileBuffer,TX_LEN);
        Usart1Tx.TX_flag = 1;
#else
        ColorSchedule();
#endif

#endif
        systime = HAL_GetTick();
        //OSRAM_framRefresh();
    }
}


void STRIP_SwitchByPort(uint16_t port)
{
    //rst();
    Send_2811_totalPixels(0xf,0,0);
    Delay_ms(SW_period_200ms);
    Send_2811_totalPixels(0,0xf,0);
    Delay_ms(SW_period_200ms);
    Send_2811_totalPixels(0,0,0xf);
    Delay_ms(SW_period_200ms);
    Send_2811_totalPixels(0xf,0xf,0xf);
    Delay_ms(SW_period_200ms);
    Send_2811_totalPixels(0xf,0xf,0);
    Delay_ms(SW_period_200ms);
    Send_2811_totalPixels(0,0xf,0xf);
    Delay_ms(SW_period_200ms);
    Send_2811_totalPixels(0xf,0,0xf);
    Delay_ms(SW_period_200ms);
    
}

void STRIP_RunningByPort(RGB_Type_E rgb)
{
    uint8_t i,j,color;

    for(i = 0; i < CHIP_SIZE; i++)
    {
#if 1
        Compose_RGB(rgb, i, GRAY);
        Send_2811_oneString();
        Delay_ms(2);
#else
        for(j = 0; j < 25; j++)
        {
            Compose_RGB(rgb, i, 4*j);
            RGB_SDI_Display();
            Delay_us(250);
        }
        for(j = 25; j > 0; j--)
        {
            Compose_RGB(rgb, i, 4*j);
            RGB_SDI_Display();
            Delay_us(250);
        }
#endif
    }

    for(i = CHIP_SIZE; i > 0; i--)
    {
#if 1
        Compose_RGB(rgb, i-1, GRAY);
        Send_2811_oneString();
        Delay_ms(2);
#else
        for(j = 0; j < 25; j++)
        {
            Compose_RGB(rgb, i, 4*j);
            RGB_SDI_Display();
            Delay_us(250);
        }
        for(j = 25; j > 0; j--)
        {
            Compose_RGB(rgb, i, 4*j);
            RGB_SDI_Display();
            Delay_us(250);
        }
#endif
    }
}


