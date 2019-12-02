#include "stm32f1xx.h"
#include "main.h"
#include "gpio.h"
#include "config.h"
#include "programs.h"
#include "delay.h"

#if(PROJECTOR_CUBEPLT)

extern uint8_t                  runFlag;
extern char                     fileBuffer[MAX_FILE_SIZE];
extern uint8_t                  cube_buff_G[IO_SIZE][CHIP_SIZE];
extern uint8_t                  cube_buff_R[IO_SIZE][CHIP_SIZE];
extern uint8_t                  cube_buff_B[IO_SIZE][CHIP_SIZE];

//extern USART_TransmiteTYPE      Usart1Tx;
extern PROGRAMS_TYPE_E          programsType;
extern uint16_t                 filmFrameIdx;
extern UART_HandleTypeDef       huart1;

#if(CUBEPLT_MASTER)
extern uint8_t                  usartTxData[1024];
#endif

uint8_t                         GData1[CHIP_SIZE], RData1[CHIP_SIZE], BData1[CHIP_SIZE];

//static uint64_t                 systime = 0;

void Send_8bits0(uint8_t dat)
{
    uint8_t i;

    delay_ns(1);
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            P0_PIN_H;
            delay_ns(3);
            P0_PIN_L;
            delay_ns(1);
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            P0_PIN_H;
            delayns_300();
            P0_PIN_L;
            delay_ns(3);
        }
        dat=dat<<1;
    }
    P0_PIN_L;
}

void Send_8bits1(uint8_t dat)
{
    uint8_t i;

    delay_ns(1);
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            P1_PIN_H;
            delay_ns(3);
            P1_PIN_L;
            delay_ns(1);
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            P1_PIN_H;
            delayns_300();
            P1_PIN_L;
            delay_ns(3);
        }
        dat=dat<<1;
    }
    P1_PIN_L;
}

void Send_8bits2(uint8_t dat)
{
    uint8_t i;

    delay_ns(1);
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            P2_PIN_H;
            delay_ns(3);
            P2_PIN_L;
            delay_ns(1);
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            P2_PIN_H;
            delayns_300();
            P2_PIN_L;
            delay_ns(3);
        }
        dat=dat<<1;
    }
    P2_PIN_L;
}

void Send_8bits3(uint8_t dat)
{
    uint8_t i;

    delay_ns(1);
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            P3_PIN_H;
            delay_ns(3);
            P3_PIN_L;
            delay_ns(1);
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            P3_PIN_H;
            delayns_300();
            P3_PIN_L;
            delay_ns(3);
        }
        dat=dat<<1;
    }
    P3_PIN_L;
}

void Send_8bits4(uint8_t dat)
{
    uint8_t i;

    delay_ns(1);
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            P4_PIN_H;
            delay_ns(3);
            P4_PIN_L;
            delay_ns(1);
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            P4_PIN_H;
            delayns_300();
            P4_PIN_L;
            delay_ns(3);
        }
        dat=dat<<1;
    }
    P4_PIN_L;
}


void Send_8bits5(uint8_t dat)
{
    uint8_t i;

    delay_ns(1);
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            P5_PIN_H;
            delay_ns(3);
            P5_PIN_L;
            delay_ns(1);
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            P5_PIN_H;
            delayns_300();
            P5_PIN_L;
            delay_ns(3);
        }
        dat=dat<<1;
    }
    P5_PIN_L;
}


void Send_8bits6(uint8_t dat)
{
    uint8_t i;

    delay_ns(1);
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            P6_PIN_H;
            delay_ns(3);
            P6_PIN_L;
            delay_ns(1);
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            P6_PIN_H;
            delayns_300();
            P6_PIN_L;
            delay_ns(3);
        }
        dat=dat<<1;
    }
    P6_PIN_L;
}


void Send_8bits7(uint8_t dat)
{
    uint8_t i;

    delay_ns(1);
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            P7_PIN_H;
            delay_ns(3);
            P7_PIN_L;
            delay_ns(1);
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            P7_PIN_H;
            delayns_300();
            P7_PIN_L;
            delay_ns(3);
        }
        dat=dat<<1;
    }
    P7_PIN_L;
}


void Send_8bits8(uint8_t dat)
{
    uint8_t i;

    delay_ns(1);
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            P8_PIN_H;
            delay_ns(3);
            P8_PIN_L;
            delay_ns(1);
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            P8_PIN_H;
            delayns_300();
            P8_PIN_L;
            delay_ns(3);
        }
        dat=dat<<1;
    }
    P8_PIN_L;
}


void Send_8bits9(uint8_t dat)
{
    uint8_t i;

    delay_ns(1);
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            P9_PIN_H;
            delay_ns(3);
            P9_PIN_L;
            delay_ns(1);
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            P9_PIN_H;
            delayns_300();
            P9_PIN_L;
            delay_ns(3);
        }
        dat=dat<<1;
    }
    P9_PIN_L;
}


void Send_8bits10(uint8_t dat)
{
    uint8_t i;

    delay_ns(1);
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            P10_PIN_H;
            delay_ns(3);
            P10_PIN_L;
            delay_ns(1);
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            P10_PIN_H;
            delayns_300();
            P10_PIN_L;
            delay_ns(3);
        }
        dat=dat<<1;
    }
    P10_PIN_L;
}


void Send_8bits11(uint8_t dat)
{
    uint8_t i;

    delay_ns(1);
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            P11_PIN_H;
            delay_ns(3);
            P11_PIN_L;
            delay_ns(1);
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            P11_PIN_H;
            delayns_300();
            P11_PIN_L;
            delay_ns(3);
        }
        dat=dat<<1;
    }
    P11_PIN_L;
}


void Send_8bits12(uint8_t dat)
{
    uint8_t i;

    delay_ns(1);
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            P12_PIN_H;
            delay_ns(3);
            P12_PIN_L;
            delay_ns(1);
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            P12_PIN_H;
            delayns_300();
            P12_PIN_L;
            delay_ns(3);
        }
        dat=dat<<1;
    }
    P12_PIN_L;
}


void Send_8bits13(uint8_t dat)
{
    uint8_t i;

    delay_ns(1);
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            P13_PIN_H;
            delay_ns(3);
            P13_PIN_L;
            delay_ns(1);
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            P13_PIN_H;
            delayns_300();
            P13_PIN_L;
            delay_ns(3);
        }
        dat=dat<<1;
    }
    P13_PIN_L;
}

void Send_8bits14(uint8_t dat)
{
    uint8_t i;

    delay_ns(1);
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            P14_PIN_H;
            delay_ns(3);
            P14_PIN_L;
            delay_ns(1);
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            P14_PIN_H;
            delayns_300();
            P14_PIN_L;
            delay_ns(3);
        }
        dat=dat<<1;
    }
    P14_PIN_L;
}

void Send_8bits15(uint8_t dat)
{
    uint8_t i;

    delay_ns(1);
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            P15_PIN_H;
            delay_ns(3);
            P15_PIN_L;
            delay_ns(1);
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            P15_PIN_H;
            delayns_300();
            P15_PIN_L;
            delay_ns(3);
        }
        dat=dat<<1;
    }
    P15_PIN_L;
}

void Send_2811_oneString(uint8_t *g,uint8_t *r,uint8_t *b)        //传送16位灰度数据    ,三组相同
{
    uint8_t i=0;

    __disable_irq();
    P1_PIN_L;
    /*****************16*3组灰度数据********************************/
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits1(g[i]);
        Send_8bits1(r[i]);
        Send_8bits1(b[i]);
    }
    __enable_irq();
}

void Send_2811_totalPixels0(uint8_t *g,uint8_t *r,uint8_t *b)
{
    uint8_t i=0;

    __disable_irq();
    //P0_PIN_L;
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits0(g[i]);
        Send_8bits0(r[i]);
        Send_8bits0(b[i]);
    }
    __enable_irq();
}


void Send_2811_totalPixels1(uint8_t *g,uint8_t *r,uint8_t *b)
{
    uint8_t i=0;

    __disable_irq();
    //P1_PIN_L;
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits1(g[i]);
        Send_8bits1(r[i]);
        Send_8bits1(b[i]);
    }
    __enable_irq();
}

void Send_2811_totalPixels2(uint8_t *g,uint8_t *r,uint8_t *b)
{
    uint8_t i=0;

    __disable_irq();
    //P2_PIN_L;
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits2(g[i]);
        Send_8bits2(r[i]);
        Send_8bits2(b[i]);
    }
    __enable_irq();
}


void Send_2811_totalPixels3(uint8_t *g,uint8_t *r,uint8_t *b)
{
    uint8_t i=0;

    __disable_irq();
    //P3_PIN_L;
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits3(g[i]);
        Send_8bits3(r[i]);
        Send_8bits3(b[i]);
    }
    __enable_irq();
}

void Send_2811_totalPixels4(uint8_t *g,uint8_t *r,uint8_t *b)
{
    uint8_t i=0;

    __disable_irq();
    //P4_PIN_L;
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits4(g[i]);
        Send_8bits4(r[i]);
        Send_8bits4(b[i]);
    }
    __enable_irq();
}

void Send_2811_totalPixels5(uint8_t *g,uint8_t *r,uint8_t *b)
{
    uint8_t i=0;

    __disable_irq();
    //P5_PIN_L;
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits5(g[i]);
        Send_8bits5(r[i]);
        Send_8bits5(b[i]);
    }
    __enable_irq();
}

void Send_2811_totalPixels6(uint8_t *g,uint8_t *r,uint8_t *b)
{
    uint8_t i=0;

    __disable_irq();
    //P6_PIN_L;
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits6(g[i]);
        Send_8bits6(r[i]);
        Send_8bits6(b[i]);
    }
    __enable_irq();
}

void Send_2811_totalPixels7(uint8_t *g,uint8_t *r,uint8_t *b)
{
    uint8_t i=0;

    __disable_irq();
    //P7_PIN_L;
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits7(g[i]);
        Send_8bits7(r[i]);
        Send_8bits7(b[i]);
    }
    __enable_irq();
}


void Send_2811_totalPixels8(uint8_t *g,uint8_t *r,uint8_t *b)
{
    uint8_t i=0;

    __disable_irq();
    //P8_PIN_L;
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits8(g[i]);
        Send_8bits8(r[i]);
        Send_8bits8(b[i]);
    }
    __enable_irq();
}

void Send_2811_totalPixels9(uint8_t *g,uint8_t *r,uint8_t *b)
{
    uint8_t i=0;

    __disable_irq();
    //P9_PIN_L;
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits9(g[i]);
        Send_8bits9(r[i]);
        Send_8bits9(b[i]);
    }
    __enable_irq();
}

void Send_2811_totalPixels10(uint8_t *g,uint8_t *r,uint8_t *b)
{
    uint8_t i=0;

    __disable_irq();
    //P10_PIN_L;
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits10(g[i]);
        Send_8bits10(r[i]);
        Send_8bits10(b[i]);
    }
    __enable_irq();
}

void Send_2811_totalPixels11(uint8_t *g,uint8_t *r,uint8_t *b)
{
    uint8_t i=0;

    __disable_irq();
    //P11_PIN_L;
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits11(g[i]);
        Send_8bits11(r[i]);
        Send_8bits11(b[i]);
    }
    __enable_irq();
}

void Send_2811_totalPixels12(uint8_t *g,uint8_t *r,uint8_t *b)
{
    uint8_t i=0;

    __disable_irq();
    //P12_PIN_L;
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits12(g[i]);
        Send_8bits12(r[i]);
        Send_8bits12(b[i]);
    }
    __enable_irq();
}

void Send_2811_totalPixels13(uint8_t *g,uint8_t *r,uint8_t *b)
{
    uint8_t i=0;

    __disable_irq();
    //P13_PIN_L;
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits13(g[i]);
        Send_8bits13(r[i]);
        Send_8bits13(b[i]);
    }
    __enable_irq();
}

void Send_2811_totalPixels14(uint8_t *g,uint8_t *r,uint8_t *b)
{
    uint8_t i=0;

    __disable_irq();
    //P14_PIN_L;
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits14(g[i]);
        Send_8bits14(r[i]);
        Send_8bits14(b[i]);
    }
    __enable_irq();
}

void Send_2811_totalPixels15(uint8_t *g,uint8_t *r,uint8_t *b)
{
    uint8_t i=0;

    __disable_irq();
    //P15_PIN_L;
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits15(g[i]);
        Send_8bits15(r[i]);
        Send_8bits15(b[i]);
    }
    __enable_irq();
}

/*
void UartDataHandle(uint8_t *data)
{
    uint8_t i,j;
    uint16_t IR_code = 0;

    printf("data:%s\r\n",data);

    for(i=0;i<IO_SIZE;i++)
    {
        for(j=0;j<CHIP_SIZE;j++)
        {
            GData1[j] = data[i*IO_SIZE+j*CHIP_SIZE+0];
            RData1[j] = data[i*IO_SIZE+j*CHIP_SIZE+1];
            BData1[j] = data[i*IO_SIZE+j*CHIP_SIZE+2];
        }
        Send_2811_oneString(GData1,RData1,BData1);
    }
}
*/
void ColorSchedule()
{
    uint8_t j,rcolor,gcolor,bcolor;
    uint16_t IR_code = 0;
    static uint32_t GRB=0xFFFFFF;
    static RGB_Switch_E rgbsw = RGB_B_minus;

    //printf("RGB:%x %x %x\r\n",(GRB>>8)&0xff,(GRB>>16)&0xff,(GRB)&0xff);

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
            if(rcolor>0){
                rcolor = rcolor-1;
                gcolor = gcolor+1;
                bcolor = bcolor+1;
            }
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
        GData1[j] = (GRB>>16)&0xff;
        RData1[j] = (GRB>>8)&0xff;
        BData1[j] = GRB&0xff;
    }
    Send_2811_oneString(GData1,RData1,BData1);
}

#if(CUBEPLT_SLAVE)

void CUBE_framRefresh(void)
{

    unsigned short k,fixel;
    uint8_t        row, col;

    Send_2811_totalPixels0(cube_buff_G[0],cube_buff_R[0],cube_buff_B[0]);
    Send_2811_totalPixels1(cube_buff_G[1],cube_buff_R[1],cube_buff_B[1]);
    Send_2811_totalPixels2(cube_buff_G[2],cube_buff_R[2],cube_buff_B[2]);
    Send_2811_totalPixels3(cube_buff_G[3],cube_buff_R[3],cube_buff_B[3]);

    Send_2811_totalPixels4(cube_buff_G[4],cube_buff_R[0],cube_buff_B[0]);
    Send_2811_totalPixels5(cube_buff_G[5],cube_buff_R[0],cube_buff_B[0]);
    Send_2811_totalPixels6(cube_buff_G[6],cube_buff_R[0],cube_buff_B[0]);
    Send_2811_totalPixels7(cube_buff_G[7],cube_buff_R[0],cube_buff_B[0]);

    Send_2811_totalPixels8(cube_buff_G[8],cube_buff_R[0],cube_buff_B[0]);
    Send_2811_totalPixels9(cube_buff_G[9],cube_buff_R[0],cube_buff_B[0]);
    Send_2811_totalPixels10(cube_buff_G[10],cube_buff_R[10],cube_buff_B[10]);
    Send_2811_totalPixels11(cube_buff_G[11],cube_buff_R[11],cube_buff_B[11]);

    Send_2811_totalPixels12(cube_buff_G[12],cube_buff_R[12],cube_buff_B[12]);
    Send_2811_totalPixels13(cube_buff_G[13],cube_buff_R[13],cube_buff_B[13]);
    Send_2811_totalPixels14(cube_buff_G[14],cube_buff_R[14],cube_buff_B[14]);
    Send_2811_totalPixels15(cube_buff_G[15],cube_buff_R[15],cube_buff_B[15]);
    
}

void CUBE_play(void)
{
    FRESULT res = FR_OK;

    if(runFlag)
    {
        runFlag = 0;
        
        CUBE_framRefresh();
        //ColorSchedule();
    }
}
#endif

#if(CUBEPLT_MASTER)
void CUBE_framTransmit(void)
{

    unsigned short k,fixel;
    uint8_t        row, col;

    for(k = 0; k < 16; k++)
    {
        usartTxData[0] = k;
        memcpy(&usartTxData[1], fileBuffer[k*(CHIP_SIZE*IO_SIZE*3)],CHIP_SIZE*IO_SIZE*3);
        HAL_UART_Transmit(&huart1, usartTxData, sizeof(usartTxData), 0xffff);
    }
}


void CUBE_play(void)
{
    FRESULT res = FR_OK;

    if(runFlag || ((programsType==FILM)/* && (HAL_GetTick() - systime>100000)*/))
    {

        runFlag = 0;

        if(programsType==FILM)
        {
            res = SD_ReadFilmData();
            filmFrameIdx++;
        }
        else
        {
            res = SD_ReadPhotoData();
        }
        if(res != FR_OK)
        {
            printf("Read [%s] file failed!\r\n",programsType==PHOTO?"photo":"film");
            return;
        }
        //usartTxFlag = 1;
        CUBE_framTransmit();
    }
}
#endif

void Compose_RGB(RGB_Type_E RGB, uint8_t index, uint8_t gray)
{
    memset(GData1, 0, sizeof(GData1));
    memset(RData1, 0, sizeof(RData1));
    memset(BData1, 0, sizeof(BData1));

    if(RGB&G)
        GData1[index] = gray;
    if(RGB&R)
        RData1[index] = gray;
    if(RGB&B)
        BData1[index] = gray;
}


void STRIP_RunningByPort(RGB_Type_E rgb)
{
    uint8_t i,j,color;

    for(i = 0; i < CHIP_SIZE; i++)
    {
#if 1
        Compose_RGB(rgb, i, GRAY);
        Send_2811_oneString(GData1,RData1,BData1);
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
        Send_2811_oneString(GData1,RData1,BData1);
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

#if CUBEPLT_SLAVE
void AppControlCubePltHandle(uint8_t *key, uint16_t len)
{
    uint8_t *ptr, i, j;

    printf("AppControlCubePltHandle\r\n");

    //crc checkout
    if(len != CUBE_ONE_MODULE_SIZE+1 && MODULE_ID == key[0])
        return;

    ptr = &key[1];

    for( i=0; i<IO_SIZE; i++ )
    {
        for( j=0; j<CHIP_SIZE; j++ )
        {
            cube_buff_G[i][j] = ptr[i*CHIP_SIZE*3+j*3];
            cube_buff_R[i][j] = ptr[i*CHIP_SIZE*3+j*3+1];
            cube_buff_B[i][j] = ptr[i*CHIP_SIZE*3+j*3+2];
        }
    }
}
#endif

#endif

