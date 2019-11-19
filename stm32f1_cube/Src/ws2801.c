#include "stm32f1xx.h"
#include "delay.h"
#include "gpio.h"
#include "config.h"
#include "programs.h"

#if(PROJECTOR_CUBE)
extern TIM_HandleTypeDef        htim2;
extern uint8_t                  runFlag;
extern uint16_t                 actType;
extern uint8_t                  usartTxFlag;
extern char                     fileBuffer[MAX_FILE_SIZE];
extern uint8_t                  cube_buff_G[CUBE_ROW_SIZE][CUBE_COL_SIZE];
extern uint8_t                  cube_buff_R[CUBE_ROW_SIZE][CUBE_COL_SIZE];
extern uint8_t                  cube_buff_B[CUBE_ROW_SIZE][CUBE_COL_SIZE];

//extern USART_TransmiteTYPE      Usart1Tx;
extern PROGRAMS_TYPE_E          programsType;
extern uint8_t                  filmFrameIdx;
extern SPI_HandleTypeDef        hspi1;

//static uint8_t                  GData1[CHIP_SIZE], RData1[CHIP_SIZE], BData1[CHIP_SIZE];
//static uint8_t                  GData2[CHIP_SIZE_DOWN], RData2[CHIP_SIZE_DOWN], BData2[CHIP_SIZE_DOWN];
//uint8_t                         GData3[CHIP_SIZE], RData3[CHIP_SIZE], BData3[CHIP_SIZE];
//uint8_t                         GData4[CHIP_SIZE], RData4[CHIP_SIZE], BData4[CHIP_SIZE];

uint8_t                         cubeProgramsType = 0;
//uint8_t                         cubeSoftFrameId = 0;
uint8_t                         newReqFlag = 1;

static uint64_t                 systime = 0;

static uint32_t GRB[32]={0xFFFF00,0xE6FF1E,0xCDFF37,0xB4FF50,0x9BFF69,
                         0x82FF82,0x69FF9B,0x50FFB4,0x37FFCD,0x1EFFE6,

                         0x00FFFF,0x1EE6FF,0x37CDFF,0x50B4FF,0x699BFF,
                         0x8282FF,0x9B69FF,0xB450FF,0xCD37FF,0xE61EFF,

                         0xFF00FF,0xFF1EE6,0xFF37CD,0xFF50B4,0xFF699B,
                         0xFF8282,0xFF9B69,0xFFB450,0xFFCD37,0xFFE61E,

                         0xFFFF00,0xE6FF1E};

/*
void Din_1(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 60);
}
void Din_0(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 20);
}
void rst() 
{
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
    HAL_Delay (1);
}
*/
void SPI_I2S_SendData(SPI_TypeDef* SPIx, uint16_t Data)
{
  /* Check the parameters */
  //assert_param(IS_SPI_ALL_PERIPH(SPIx));
  
  /* Write in the DR register the data to be sent */
  SPIx->DR = Data;
}

#define CODE0 0x80      //0x80 或 0xC0
#define CODE1 0xF0      //0xF8 或 0xFC

void Din_0()
{
    uint8_t TxData;
  /* Wait for SPI1 Tx buffer empty */
 // while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  
  /* Send SPI1 data */

  TxData = CODE0;
  HAL_SPI_Transmit(&hspi1, &TxData, 1, 0xffff);
  //SPI_I2S_SendData(SPI1, TxData);
}

void Din_1()
{
    uint8_t TxData;
  /* Wait for SPI1 Tx buffer empty */
 // while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  
  /* Send SPI1 data */

  TxData = CODE1;
  HAL_SPI_Transmit(&hspi1, &TxData, 1, 0xffff);
  //SPI_I2S_SendData(SPI1, TxData);
}

void rst()
{
    CU_P1_PIN_L;
    delayus(1);
}

#if 0
void Send_8bits1(uint8_t dat)
{
    uint8_t i;
    Din_0();
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;//900 300
        {
            Din_1();
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us //300 900
        {
            Din_0();
        }
        dat=dat<<1;
    }
}
#else
void Send_8bits1(uint8_t dat)
{
    uint8_t i;

    CU_P1_PIN_L;
    delayns_100();
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            CU_P1_PIN_H;
            delayns_900();
            CU_P1_PIN_L;
            __ASM("nop");
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            CU_P1_PIN_H;
            delayns_300();
            CU_P1_PIN_L;
            delayns_600();
        }
        dat=dat<<1;
    }
}
#endif


void Send_8bits2(uint8_t dat)
{
    uint8_t i;

    CU_P1_PIN_L;
    delayns_100();
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            CU_P2_PIN_H;
            delayns_900();
            CU_P2_PIN_L;
            __ASM("nop");
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            CU_P2_PIN_H;
            delayns_300();
            CU_P2_PIN_L;
            delayns_600();
        }
        dat=dat<<1;
    }
}

void Send_8bits3(uint8_t dat)
{
    uint8_t i;

    CU_P1_PIN_L;
    delayns_100();
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            CU_P3_PIN_H;
            delayns_900();
            CU_P3_PIN_L;
            __ASM("nop");
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            CU_P3_PIN_H;
            delayns_300();
            CU_P3_PIN_L;
            delayns_600();
        }
        dat=dat<<1;
    }
}

void Send_8bits4(uint8_t dat)
{
    uint8_t i;

    CU_P1_PIN_L;
    delayns_100();
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            CU_P4_PIN_H;
            delayns_900();
            CU_P4_PIN_L;
            __ASM("nop");
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            CU_P4_PIN_H;
            delayns_300();
            CU_P4_PIN_L;
            delayns_600();
        }
        dat=dat<<1;
    }
}

void Send_2811_oneString(uint8_t *g,uint8_t *r,uint8_t *b)        //传送16位灰度数据    ,三组相同
{
    uint8_t i=0;


    /*****************16*3组灰度数据********************************/
    for(i=0;i<CHIP_SIZE;i++)
    {
        __disable_irq();
        Send_8bits1(g[i]);
        Send_8bits1(r[i]);
        Send_8bits1(b[i]);
        __enable_irq();
    }
}


void Send_2811_totalPixels1()
{
    unsigned short fixel;
    uint8_t        row, col;

    for(col = 0; col < CUBE_PILLAR_SIZE; col++)
    {
        for(row = 0; row < CUBE_ROW_SIZE; row++)
        {
            __disable_irq();
            Send_8bits1(cube_buff_G[row][col]);
            Send_8bits1(cube_buff_R[row][col]);
            Send_8bits1(cube_buff_B[row][col]);
            __enable_irq();
        }
    }
}

void Send_2811_totalPixels2()
{
    unsigned short fixel;
    uint8_t        row, col;

    for(col = 0; col < CUBE_PILLAR_DOWN_SIZE; col++)
    {
        for(row = 0; row < CUBE_ROW_SIZE; row++)
        {
            __disable_irq();
            Send_8bits2(cube_buff_G[row][CUBE_PILLAR_SIZE+col]);
            Send_8bits2(cube_buff_R[row][CUBE_PILLAR_SIZE+col]);
            Send_8bits2(cube_buff_B[row][CUBE_PILLAR_SIZE+col]);
            __enable_irq();
        }
    }
}


void Send_2811_totalPixels3(uint8_t *g,uint8_t *r,uint8_t *b)
{
    uint8_t i=0;

    for(i=0;i<CHIP_SIZE;i++)
    {
        __disable_irq();
        Send_8bits3(g[i]);
        Send_8bits3(r[i]);
        Send_8bits3(b[i]);
        __enable_irq();
    }
}

void Send_2811_totalPixels4(uint8_t *g,uint8_t *r,uint8_t *b)
{
    uint8_t i=0;

    for(i=0;i<CHIP_SIZE;i++)
    {
        __disable_irq();
        Send_8bits4(g[i]);
        Send_8bits4(r[i]);
        Send_8bits4(b[i]);
        __enable_irq();
    }
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

void ScenceGradualChange()
{
    uint32_t GRB;
    uint8_t row,col;
    static uint8_t rcolor = 0xff,gcolor = 0xff,bcolor = 0xff;
    static RGB_Switch_E rgbsw = RGB_B_minus;

    //printf("RGB:%2x%2x%2x\r\n",gcolor,rcolor,bcolor);

    if(newReqFlag)
    {
        newReqFlag = 0;
        rcolor = 0xff;
        gcolor = 0xff;
        bcolor = 0xff;
        rgbsw = RGB_B_minus;
    }

    switch(rgbsw)
    {
        case RGB_B_minus:
            if(bcolor>0)
                bcolor = MINUS(bcolor,GRAY_STEP);
            else
                rgbsw = RGB_G_minus;
            break;

        case RGB_G_minus:
            if(gcolor>0)
                gcolor = MINUS(gcolor,GRAY_STEP);
            else
                rgbsw = RGB_R_minus;
            break;

        case RGB_R_minus:
            if(rcolor>0){
                rcolor = MINUS(rcolor,GRAY_STEP);
                gcolor = PLUS(gcolor,GRAY_STEP);
                bcolor = PLUS(bcolor,GRAY_STEP);
            }
            else
                rgbsw = RGB_B_plus;
            break;


        case RGB_B_plus:
            if(bcolor<0xff)
                bcolor = PLUS(bcolor,GRAY_STEP);
            else
                rgbsw = RGB_G_plus;
            break;

        case RGB_G_plus:
            if(gcolor<0xff)
                gcolor = PLUS(gcolor,GRAY_STEP);
            else
                rgbsw = RGB_R_plus;
            break;

        case RGB_R_plus:
            if(rcolor<0xff)
                rcolor = PLUS(rcolor,GRAY_STEP);
            else
                rgbsw = RGB_G1_minus;
            break;

        case RGB_G1_minus:
            if(gcolor>0)
                gcolor = MINUS(gcolor,GRAY_STEP);
            else
                rgbsw = RGB_B1_minus;
            break;

        case RGB_B1_minus:
            if(bcolor>0)
                bcolor = MINUS(bcolor,GRAY_STEP);
            else
                rgbsw = RGB_G1_plus;
            break;

        case RGB_G1_plus:
            if(gcolor<0xff)
                gcolor = PLUS(gcolor,GRAY_STEP);
            else
                rgbsw = RGB_R1_minus;
            break;

        case RGB_R1_minus:
            if(rcolor>0)
                rcolor = MINUS(rcolor,GRAY_STEP);
            else
                rgbsw = RGB_B1_plus;
            break;
        
        case RGB_B1_plus:
            if( bcolor<0XFF)
                bcolor = PLUS(bcolor,GRAY_STEP);
            else
                rgbsw = RGB_R1_plus;
            break;

        case RGB_R1_plus:
            if(rcolor<0xff)
                rcolor = PLUS(rcolor,GRAY_STEP);
            else
                rgbsw = RGB_B_minus;
            break;
    }
    GRB = gcolor<<16 | rcolor<<8 | bcolor;

    for( row=0; row<CUBE_ROW_SIZE; row++ )
    {
        memset(cube_buff_G[row], gcolor, CUBE_COL_SIZE);
        memset(cube_buff_R[row], rcolor, CUBE_COL_SIZE);
        memset(cube_buff_B[row], bcolor, CUBE_COL_SIZE);
    }
}

void ScenceLayering()
{
    uint8_t     row=0,col=0;
    uint32_t    g[CUBE_COL_SIZE];
    uint8_t     r[CUBE_COL_SIZE];
    uint8_t     b[CUBE_COL_SIZE];

    if(newReqFlag)
    {
        newReqFlag = 0;
        for( row=0; row<CUBE_ROW_SIZE; row++ )
        {
            //初始化颜色组
            memset(cube_buff_G[row], (GRB[row]>>16)&0xff, CUBE_COL_SIZE);
            memset(cube_buff_R[row], (GRB[row]>>8) &0xff, CUBE_COL_SIZE);
            memset(cube_buff_B[row], (GRB[row])    &0xff, CUBE_COL_SIZE);
        }
    }
    else
    {
        memcpy(g, cube_buff_G[0],CUBE_COL_SIZE);
        memcpy(r, cube_buff_R[0],CUBE_COL_SIZE);
        memcpy(b, cube_buff_B[0],CUBE_COL_SIZE);

        //颜色上升
        for( row=0; row<CUBE_ROW_SIZE-1; row++ )
        {
            memcpy(cube_buff_G[row], cube_buff_G[row+1],CUBE_COL_SIZE);
            memcpy(cube_buff_R[row], cube_buff_R[row+1],CUBE_COL_SIZE);
            memcpy(cube_buff_B[row], cube_buff_B[row+1],CUBE_COL_SIZE);
        }

        //新导入的一行颜色
        memcpy(cube_buff_G[CUBE_ROW_SIZE-1], g, CUBE_COL_SIZE);
        memcpy(cube_buff_R[CUBE_ROW_SIZE-1], r, CUBE_COL_SIZE);
        memcpy(cube_buff_B[CUBE_ROW_SIZE-1], b, CUBE_COL_SIZE);
    }
    /*
    for( row=0; row<CUBE_ROW_SIZE; row++ )
    {
        printf("---------------------------\r\n");
        for( col=0; col<CUBE_COL_SIZE; col++ )
        {
            printf("[%d %d]:%x  ",row,col,
                    cube_buff_GRB[row][col]);
        }
        printf("\r\n");
    }*/
}

void ScenceWaving()
{
    uint32_t    GRB = 0xffffff;
    uint8_t     col,row;

    for( col = 0; col < CUBE_COL_SIZE; col++ )
    {
        for( row=0; row<CUBE_ROW_SIZE; row++ )
        {
            //初始化颜色组
            cube_buff_G[row][col] = (GRB>>16)&0xff;
            cube_buff_R[row][col] = (GRB>>8) &0xff;
            cube_buff_B[row][col] = (GRB)    &0xff;
        }
    }
}

void ScenceCycle()
{
    static uint8_t  calc = 0;
    uint8_t         row=0,col=0,base;

    if(newReqFlag)
    {
        newReqFlag = 0;
        calc = 0;
    }

    base = calc%CUBE_COL_SIZE;
    //printf("base=%d\r\n",base);

    for( col = 0; col < CUBE_COL_SIZE; col++ )
    {
        for( row=0; row<CUBE_ROW_SIZE; row++ )
        {
            //初始化颜色组
            if(col+base < CUBE_COL_SIZE)
            {
                cube_buff_G[row][col] = (GRB[col+base]>>16)&0xff;
                cube_buff_R[row][col] = (GRB[col+base]>>8) &0xff;
                cube_buff_B[row][col] = (GRB[col+base])    &0xff;
                //printf("<<< %x\r\n",cube_buff_GRB[row][col]);
            }
            else
            {
                cube_buff_G[row][col] = (GRB[(col+base)-(CUBE_COL_SIZE)]>>16)&0xff;
                cube_buff_R[row][col] = (GRB[(col+base)-(CUBE_COL_SIZE)]>>8) &0xff;
                cube_buff_B[row][col] = (GRB[(col+base)-(CUBE_COL_SIZE)])    &0xff;
                //printf(">>> %x\r\n",cube_buff_GRB[row][col]);
            }
        }
    }
    /*
    for( row=0; row<CUBE_ROW_SIZE; row++ )
    {
        printf("---------------------------\r\n");
        for( col=0; col<CUBE_COL_SIZE; col++ )
        {
            printf("[%d %d]:%x %x %x",row,col,
                    cube_buff_G[row][col],cube_buff_R[row][col],cube_buff_B[row][col]);
        }
        printf("\r\n");
    }*/
    
    calc++;
}


void WS2801_framRefresh(void)
{
    Send_2811_totalPixels1();
    Send_2811_totalPixels2();
}

FRESULT WS2801_softScen()
{
    FRESULT res = FR_TIMEOUT;

    if(cubeProgramsType%4==0)
    {
        //if(HAL_GetTick() - systime>1000)
        {
            ScenceGradualChange();
            //systime = HAL_GetTick();
            res = FR_OK;
        }
    }
    else if(cubeProgramsType%4==1)
    {
        //if(HAL_GetTick() - systime>1000)
        {
            //printf("Layering\r\n");
            ScenceLayering();
            //systime = HAL_GetTick();
            res = FR_OK;
        }
    }
    else if(cubeProgramsType%4==2)
    {
        //if(HAL_GetTick() - systime>50000)
        {
            ScenceCycle();
            //systime = HAL_GetTick();
            res = FR_OK;
        }
    }
    else if(cubeProgramsType%4==3)
    {
        //if(HAL_GetTick() - systime>50000)
        {
            ScenceWaving();
            //systime = HAL_GetTick();
            res = FR_OK;
        }
    }

    return res;
}

void WS2801_play(void)
{
    FRESULT res = FR_OK;
    uint8_t flash_flag = 0;

    if(runFlag || ((programsType!=PHOTO)/* && (HAL_GetTick() - systime>100000)*/))
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
        //printf("cubept=%d,pt=%d\r\n",cubeProgramsType,programsType);
        if(programsType==FILM)
        {
            res = SD_ReadFilmData();
            if(res==FR_OK)
            {
                filmFrameIdx++;
                flash_flag = 1;
            }
        }
        else if(programsType==AUTO_ALGORITHM)
        {
            res = WS2801_softScen();
            if(res==FR_OK)
                flash_flag = 1;
            else
                return;
        }
        else
        {
            res = SD_ReadPhotoData();
            if(res==FR_OK)
                flash_flag = 1;
        }
        if(res != FR_OK)
        {
            printf("Read [%s] file failed!\r\n",programsType==PHOTO?"photo":"film");
            return;
        }

        if(flash_flag)
        {
            //usartTxFlag = 1;
            //WS2801_QuadrantConvert();
            WS2801_framRefresh();
        }
#endif
        //systime = HAL_GetTick();
    }
}

#if 0
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
#endif

