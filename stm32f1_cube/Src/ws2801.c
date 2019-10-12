#include "stm32f1xx.h"
#include "delay.h"
#include "gpio.h"
#include "eplos.h"
#include "config.h"
#include "programs.h"

#if(PROJECTOR_WS2801)

extern uint8_t                  runFlag;
extern char                     fileBuffer[MAX_FILE_SIZE];
extern uint8_t                  cube_buff_G[CUBE_ROW_SIZE][CUBE_COL_SIZE*CUBE_PAGE_SIZE];
extern uint8_t                  cube_buff_R[CUBE_ROW_SIZE][CUBE_COL_SIZE*CUBE_PAGE_SIZE];
extern uint8_t                  cube_buff_B[CUBE_ROW_SIZE][CUBE_COL_SIZE*CUBE_PAGE_SIZE];

//extern USART_TransmiteTYPE      Usart1Tx;
extern PROGRAMS_TYPE_E          programsType;
extern uint8_t                  filmFrameIdx;

uint8_t                         GData1[CHIP_SIZE], RData1[CHIP_SIZE], BData1[CHIP_SIZE];
uint8_t                         GData2[CHIP_SIZE], RData2[CHIP_SIZE], BData2[CHIP_SIZE];
uint8_t                         GData3[CHIP_SIZE], RData3[CHIP_SIZE], BData3[CHIP_SIZE];
uint8_t                         GData4[CHIP_SIZE], RData4[CHIP_SIZE], BData4[CHIP_SIZE];

//static uint64_t                 systime = 0;

/*
uint8_t displayMatrix1[CUBE_ROW_SIZE][CUBE_COL_SIZE] = {0};
uint8_t displayMatrix2[CUBE_ROW_SIZE][CUBE_COL_SIZE] = {0};
uint8_t displayMatrix3[CUBE_ROW_SIZE][CUBE_COL_SIZE] = {0};
uint8_t displayMatrix4[CUBE_ROW_SIZE][CUBE_COL_SIZE] = {0};
*/
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


void Send_2811_totalPixels1(uint8_t *g,uint8_t *r,uint8_t *b)
{
    uint8_t i=0;

    __disable_irq();
    P1_PIN_L;
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
    P2_PIN_L;
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
    P3_PIN_L;
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
    P4_PIN_L;
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits4(g[i]);
        Send_8bits4(r[i]);
        Send_8bits4(b[i]);
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
            GData1[j] = data[i*IO_SIZE+j*CHIP_SIZE+0];
            RData1[j] = data[i*IO_SIZE+j*CHIP_SIZE+1];
            BData1[j] = data[i*IO_SIZE+j*CHIP_SIZE+2];
        }
        Send_2811_oneString(GData1,RData1,BData1);
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

void WS2801_QuadrantConvert(void)
{
    unsigned short k,fixel;
    uint8_t        row, col;

    //写入 288*24 bits数据
    for(fixel = 0; fixel < CUBE_ROW_SIZE*CUBE_COL_SIZE; fixel++)
    {
        row = fixel/CUBE_ROW_SIZE;
        col = fixel%CUBE_ROW_SIZE;
        
        GData1[fixel] = cube_buff_G[row][col];
        RData1[fixel] = cube_buff_R[row][col];
        BData1[fixel] = cube_buff_B[row][col];

        GData2[fixel] = cube_buff_G[row][col+CUBE_COL_SIZE];
        RData2[fixel] = cube_buff_R[row][col+CUBE_COL_SIZE];
        BData2[fixel] = cube_buff_B[row][col+CUBE_COL_SIZE];

        GData3[fixel] = cube_buff_G[row][col+CUBE_COL_SIZE*2];
        RData3[fixel] = cube_buff_R[row][col+CUBE_COL_SIZE*2];
        BData3[fixel] = cube_buff_B[row][col+CUBE_COL_SIZE*2];

        GData4[fixel] = cube_buff_G[row][col+CUBE_COL_SIZE*3];
        RData4[fixel] = cube_buff_R[row][col+CUBE_COL_SIZE*3];
        BData4[fixel] = cube_buff_B[row][col+CUBE_COL_SIZE*3];
    }
}

void WS2801_framRefresh(void)
{

    Send_2811_totalPixels1(GData1,RData1,BData1);
    Send_2811_totalPixels2(GData2,RData2,BData2);
    Send_2811_totalPixels3(GData3,RData3,BData3);
    Send_2811_totalPixels4(GData4,RData4,BData4);
}


void WS2801_play(void)
{
    FRESULT res = FR_OK;

    if(runFlag || ((programsType==FILM)/* && (HAL_GetTick() - systime>100000)*/))
    {
#if 0
        SD_ReadFileData();
        //for(j=0;j<1024;j++)
        //{
        //    displayMatrix[j/32][j%32] = fileBuffer[j];
        //}
        memcpy(Usart1Tx.TX_pData,fileBuffer,TX_LEN);
        Usart1Tx.TX_flag = 1;
#else
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
        
        WS2801_QuadrantConvert();
        WS2801_framRefresh();
        //ColorSchedule();
#endif
        //systime = HAL_GetTick();
        //OSRAM_framRefresh();
    }
}


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
#endif

