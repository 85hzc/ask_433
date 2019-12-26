#include "stm32f1xx.h"
#include "delay.h"
#include "gpio.h"
#include "config.h"
#include "programs.h"
#include "math.h"

#if(PROJECTOR_CUBE)
#define CU_P1_PIN_H       GPIOB->BSRR = CU_P1_PIN;                        // 输出高电平
#define CU_P1_PIN_L       GPIOB->BSRR = (uint32_t)CU_P1_PIN << 16;        // 输出低电平 
#define CU_P2_PIN_H       GPIOB->BSRR = CU_P2_PIN;                        // 输出高电平
#define CU_P2_PIN_L       GPIOB->BSRR = (uint32_t)CU_P2_PIN << 16;        // 输出低电平 
#define CU_P3_PIN_H       GPIOB->BSRR = CU_P3_PIN;                        // 输出高电平
#define CU_P3_PIN_L       GPIOB->BSRR = (uint32_t)CU_P3_PIN << 16;        // 输出低电平 
#define CU_P4_PIN_H       GPIOB->BSRR = CU_P4_PIN;                        // 输出高电平
#define CU_P4_PIN_L       GPIOB->BSRR = (uint32_t)CU_P4_PIN << 16;        // 输出低电平 

extern TIM_HandleTypeDef        htim2;
extern BOOL                     runFlag;
extern uint16_t                 actType;
extern BOOL                     usartTxFlag;
extern char                     fileBuffer[MAX_FILE_SIZE];
extern uint8_t                  photoProgramIdx;
extern uint16_t                 fileTotalPhoto;  //静态图片数
extern BYTE                     photo_filename[MAX_FILE_NUM][FILE_NAME_LEN_SHORT];
extern uint8_t                  cube_buff_G[CUBE_ROW_SIZE][CUBE_COL_SIZE];
extern uint8_t                  cube_buff_R[CUBE_ROW_SIZE][CUBE_COL_SIZE];
extern uint8_t                  cube_buff_B[CUBE_ROW_SIZE][CUBE_COL_SIZE];
extern uint16_t                 col_up,col_down;
//extern USART_TransmiteTYPE      Usart1Tx;
extern PROGRAMS_TYPE_E          programsType;
extern uint16_t                 filmFrameIdx;
extern SPI_HandleTypeDef        hspi1;
extern uint16_t                 brightness;

//static uint8_t                  GData1[CHIP_SIZE], RData1[CHIP_SIZE], BData1[CHIP_SIZE];
//static uint8_t                  GData2[CHIP_SIZE_DOWN], RData2[CHIP_SIZE_DOWN], BData2[CHIP_SIZE_DOWN];
//uint8_t                         GData3[CHIP_SIZE], RData3[CHIP_SIZE], BData3[CHIP_SIZE];
//uint8_t                         GData4[CHIP_SIZE], RData4[CHIP_SIZE], BData4[CHIP_SIZE];

BOOL                            RGB_white_firsttime_flag = true;
uint8_t                         cubeSoftFrameId = 0;
BOOL                            newReqFlag = true;
BOOL                            cubeRGBStatus = false;
BOOL                            photoOpenFlag = true;

static uint64_t                 systime = 0;
/*
static uint32_t GRB30[30]={//30
                       0xFFFF00,0xE6FF1E,0xCDFF37,0xB4FF50,0x9BFF69,
                       0x82FF82,0x69FF9B,0x50FFB4,0x37FFCD,0x1EFFE6,

                       0x00FFFF,0x1EE6FF,0x37CDFF,0x50B4FF,0x699BFF,
                       0x8282FF,0x9B69FF,0xB450FF,0xCD37FF,0xE61EFF,

                       0xFF00FF,0xFF1EE6,0xFF37CD,0xFF50B4,0xFF699B,
                       0xFF8282,0xFF9B69,0xFFB450,0xFFCD37,0xFFE61E};
*/


static uint32_t GRB48[CUBE_RGB_SIZE]={
                       0xFFFF0F,0xEFFF18,0xDFFF1F,0xCFFF2F,0xBFFF3F,0xAFFF4F,0x9FFF5F,0x8FFF6F,
                       0x7FFF7F,0x6FFF8F,0x5FFF9F,0x4FFFAF,0x3FFFBF,0x2FFFCF,0x1FFFDF,0x18FFEF,
                       
                       0x0FEFFF,0x18DFFF,0x1FCFFF,0x2FBFFF,0x3FAFFF,0x4F9FFF,0x5F8FFF,0x6F7FFF,
                       0x7F6FFF,0x8F5FFF,0x9F4FFF,0xAF3FFF,0xBF2FFF,0xCF1FFF,0xDF18FF,0xEF0FFF,
                       
                       0xFF18EF,0xFF1FDF,0xFF2FCF,0xFF3FBF,0xFF4FAF,0xFF5F9F,0xFF6F8F,0xFF7F7F,
                       0xFF8F6F,0xFF9F5F,0xFFAF4F,0xFFBF3F,0xFFCF2F,0xFFDF1F,0xFFEF18,0xFFFF0F
                       };


static uint32_t GRB[24]={//24
                       0xFFFF0F,0xDFFF1F,0xBFFF3F,0x9FFF5F,
                       0x7FFF7F,0x5FFF9F,0x3FFFBF,0x1FFFDF,
   
                       0x0FDFFF,0x1FBFFF,0x3F9FFF,0x5F7FFF,
                       0x7F5FFF,0x9F3FFF,0xBF1FFF,0xDF0FFF,
   
                       0xFF1FDF,0xFF3FBF,0xFF5F9F,0xFF7F7F,
                       0xFF9F5F,0xFFBF3F,0xFFDF1F,0xFFEF0F};

void WS2801_framRefresh(void);


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

#define IOT_MAX_VAL(a,b)                        (a>b?a:b)
#define OPP_COLOR_CONVERT_PWM_MAX               (255*1.0)
const float sRGB2RGBPWM[3][3] = 
{
    {1.5654, 0.9143, 0.0737},
    {0.1092, 2.0891, -0.00382},
    {0.0113, -0.00677, 0.8857}
};

/******************************************************************************
 Function    : oppColorRGBtoRGBPWM
 Description : OPP sRGB to RGB pwm convert
 Note        : (none)
 Input Para  : (none)
 Output Para : (none)
 Return      : (none)
 Other       : (none)
******************************************************************************/
uint8_t oppColorRGBtoRGBPWM(uint16_t uwPWM[4], uint8_t sRGB[3], uint8_t bri)
{
    float tmp;
    float RGBlinear[3];
    float afPWM[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    /* step 1 */
    tmp = sRGB[0] / 255.0;
    (tmp <= 0.04045) ? (RGBlinear[0] = tmp / 12.92) : (RGBlinear[0] = pow((tmp + 0.055)/1.055, 2.4));
    tmp = sRGB[1] / 255.0;
    (tmp <= 0.04045) ? (RGBlinear[1] = tmp / 12.92) : (RGBlinear[1] = pow((tmp + 0.055)/1.055, 2.4));
    tmp = sRGB[2] / 255.0;
    (tmp <= 0.04045) ? (RGBlinear[2] = tmp / 12.92) : (RGBlinear[2] = pow((tmp + 0.055)/1.055, 2.4));

    /* step 2*/
    afPWM[0] = sRGB2RGBPWM[0][0] * RGBlinear[0] + sRGB2RGBPWM[0][1] * RGBlinear[1] + sRGB2RGBPWM[0][2] * RGBlinear[2];
    afPWM[1] = sRGB2RGBPWM[1][0] * RGBlinear[0] + sRGB2RGBPWM[1][1] * RGBlinear[1] + sRGB2RGBPWM[1][2] * RGBlinear[2];
    afPWM[2] = sRGB2RGBPWM[2][0] * RGBlinear[0] + sRGB2RGBPWM[2][1] * RGBlinear[1] + sRGB2RGBPWM[2][2] * RGBlinear[2];
    afPWM[3] = IOT_MAX_VAL(afPWM[1], afPWM[2]);
    afPWM[3] = IOT_MAX_VAL(afPWM[0], afPWM[3]);
    
    /* step 3*/
    afPWM[0] = afPWM[0] / afPWM[3];
    afPWM[1] = afPWM[1] / afPWM[3];
    afPWM[2] = afPWM[2] / afPWM[3];
    afPWM[3] = 0;

    afPWM[0] = (1.000 < afPWM[0]) ? 1.000 : afPWM[0];
    afPWM[1] = (1.000 < afPWM[1]) ? 1.000 : afPWM[1];
    afPWM[2] = (1.000 < afPWM[2]) ? 1.000 : afPWM[2];
    afPWM[3] = (1.000 < afPWM[3]) ? 1.000 : afPWM[3];

    afPWM[0] = (0.001 > afPWM[0]) ? 0.0 : afPWM[0];
    afPWM[1] = (0.001 > afPWM[1]) ? 0.0 : afPWM[1];
    afPWM[2] = (0.001 > afPWM[2]) ? 0.0 : afPWM[2];
    afPWM[3] = (0.001 > afPWM[3]) ? 0.0 : afPWM[3];

    /*step 4*/
    uwPWM[0] = (uint16_t)((OPP_COLOR_CONVERT_PWM_MAX * afPWM[0]) * (float)bri / 255.0);
    uwPWM[1] = (uint16_t)((OPP_COLOR_CONVERT_PWM_MAX * afPWM[1]) * (float)bri / 255.0);
    uwPWM[2] = (uint16_t)((OPP_COLOR_CONVERT_PWM_MAX * afPWM[2]) * (float)bri / 255.0);
    uwPWM[3] = (uint16_t)((OPP_COLOR_CONVERT_PWM_MAX * afPWM[3]) * (float)bri / 255.0);
    
    return 0;
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

    CU_P2_PIN_L;
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

    CU_P3_PIN_L;
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

    CU_P4_PIN_L;
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

    for(col = 0; col < col_up; col++)
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

    for(col = 0; col < col_down; col++)
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

void calc_RGB_COL(uint8_t *buff_G,uint8_t *buff_R,uint8_t *buff_B)
{
    uint8_t         row=0,col=0,base=0;
    uint16_t        uwPWM[4];
    uint8_t         sRGB[3];
    
    for( col = 0; col < CUBE_RGB_SIZE; col++ )
    {
        //初始化颜色组
        //if(col+base < CUBE_COL_SIZE)
        {
            sRGB[0] = ((GRB48[col+base]>>8) &0xff);
            sRGB[1] = ((GRB48[col+base]>>16)&0xff);
            sRGB[2] = ((GRB48[col+base])    &0xff);
            #if CUBE_LED_ALG
            oppColorRGBtoRGBPWM(uwPWM, sRGB, 255);
            #endif
        }
        /*else
        {
            sRGB[0] = (GRB48[(col+base)]>>8) &0xff;
            sRGB[1] = (GRB48[(col+base)-(CUBE_COL_SIZE)]>>16)&0xff;
            sRGB[2] = (GRB48[(col+base)-(CUBE_COL_SIZE)])    &0xff;
            #if CUBE_LED_ALG
            oppColorRGBtoRGBPWM(uwPWM, sRGB, 255);
            #endif
        }*/
        
        buff_R[col] = uwPWM[0]/RGB_SCALE;
        buff_G[col] = uwPWM[1]/RGB_SCALE;
        buff_B[col] = uwPWM[2]/RGB_SCALE;
    }
}


void calc_RGB_ROW(uint8_t *buff_G,uint8_t *buff_R,uint8_t *buff_B)
{
    uint8_t         row=0,col=0,base;
    uint16_t        uwPWM[4];
    uint8_t         sRGB[3];

    uint32_t    RGB[]= {0xff0000,0xff2900,0xff5200,0xff7B00,
                        0xffA500,0xffB700,0xffC900,0xffDB00,0xffED00,
                        0xffff00,0xCCff00,0x99ff00,0x66ff00,0x33ff00,
                        0x00ff00,0x00E633,0x00CD66,0x00B499,0x009BCC,
                        0x007Fff,0x0066ff,0x004Dff,0x0034ff,0x001Bff,
                        0x0000ff,0x1B00ff,0x3600ff,0x5100ff,0x6C00ff,
                        0x8800ff};
                        /***
                        **赤色 【RGB】255, 0, 0
                        **橙色 【RGB】 255, 165, 0
                        **黄色 【RGB】255, 255, 0 
                        **绿色 【RGB】0, 255, 0 
                        **青色 【RGB】0, 127, 255 
                        **蓝色 【RGB】0, 0, 255 
                        **紫色 【RGB】139, 0, 255 
                        ***/
    
    for( row = 0; row < CUBE_ROW_SIZE; row++ )
    {
        //初始化颜色组

        sRGB[0] = (RGB[row]>>16) &0xff;
        sRGB[1] = (RGB[row]>>8)&0xff;
        sRGB[2] = (RGB[row])    &0xff;
        #if CUBE_LED_ALG
        oppColorRGBtoRGBPWM(uwPWM,  sRGB, 255);
        #endif

        buff_R[row] = uwPWM[0]/RGB_SCALE;
        buff_G[row] = uwPWM[1]/RGB_SCALE;
        buff_B[row] = uwPWM[2]/RGB_SCALE;
    }
}


void calc_RGB_Photo_point()
{
    uint8_t         row=0,col=0;
    uint16_t        uwPWM[4];
    uint8_t         sRGB[3];

    for( row = 0; row < CUBE_ROW_SIZE; row++ )
    {
        for( col = 0; col < CUBE_COL_SIZE; col++ )
        {
            //初始化颜色组

            sRGB[0] = cube_buff_R[row][col];
            sRGB[1] = cube_buff_G[row][col];
            sRGB[2] = cube_buff_B[row][col];
            #if CUBE_LED_ALG
            oppColorRGBtoRGBPWM(uwPWM,  sRGB, 255);
            #endif

            cube_buff_R[row][col] = uwPWM[0];
            cube_buff_G[row][col] = uwPWM[1];
            cube_buff_B[row][col] = uwPWM[2];
        }
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
        memset(cube_buff_G[row], gcolor/RGB_SCALE, CUBE_COL_SIZE);
        memset(cube_buff_R[row], rcolor/RGB_SCALE, CUBE_COL_SIZE);
        memset(cube_buff_B[row], bcolor/RGB_SCALE, CUBE_COL_SIZE);
    }
}

void scenceCandle()
{
    static uint8_t calc=0;
    uint8_t     row=0,col=0;
    uint32_t    g[CUBE_COL_SIZE];
    uint8_t     r[CUBE_COL_SIZE];
    uint8_t     b[CUBE_COL_SIZE];
    uint8_t     h[10]={1,2,3,4,5,5,4,3,2,1};

    if(newReqFlag)
    {
        newReqFlag = 0;
        drv_pwm_speed(500);

        //颜色上升
        for( row=6; row<24; row++ )
        {
            memcpy(cube_buff_G[row], 0xff/RGB_SCALE,CUBE_COL_SIZE);
            memcpy(cube_buff_R[row], 0xff/RGB_SCALE,CUBE_COL_SIZE);
            memcpy(cube_buff_B[row], 0xff/RGB_SCALE,CUBE_COL_SIZE);
        }
    }
    else
    {

        for(col=22;col<CUBE_COL_SIZE;col++)
        {
            //初始化灯头颜色组
            {
                for( row=5; row>5-h[(calc-22)%10]; row-- )
                {
                    memset(cube_buff_G[row], 0x0/RGB_SCALE, CUBE_COL_SIZE);
                    memset(cube_buff_R[row], 0xff/RGB_SCALE, CUBE_COL_SIZE);
                    memset(cube_buff_B[row], 0/RGB_SCALE, CUBE_COL_SIZE);
                }
            }
        }
    }
    
    for( row=0; row<CUBE_ROW_SIZE; row++ )
    {
        printf("---------------------------\r\n");
        for( col=0; col<CUBE_COL_SIZE; col++ )
        {
            printf("[%d %d]:%x %x %x",row,col,
                    cube_buff_G[row][col],cube_buff_R[row][col],cube_buff_B[row][col]);
        }
        printf("\r\n");
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
        for( row=0; row<CUBE_RGB_SIZE; row++ )
        {
            //初始化颜色组
            memset(cube_buff_G[row], ((GRB48[row]>>16)&0xff)/RGB_SCALE, CUBE_COL_SIZE);
            memset(cube_buff_R[row], ((GRB48[row]>>8) &0xff)/RGB_SCALE, CUBE_COL_SIZE);
            memset(cube_buff_B[row], ((GRB48[row])    &0xff)/RGB_SCALE, CUBE_COL_SIZE);
        }
    }
    else
    {
        memcpy(g, cube_buff_G[0],CUBE_COL_SIZE);
        memcpy(r, cube_buff_R[0],CUBE_COL_SIZE);
        memcpy(b, cube_buff_B[0],CUBE_COL_SIZE);

        //颜色上升
        for( row=0; row<CUBE_RGB_SIZE-1; row++ )
        {
            memcpy(cube_buff_G[row], cube_buff_G[row+1],CUBE_COL_SIZE);
            memcpy(cube_buff_R[row], cube_buff_R[row+1],CUBE_COL_SIZE);
            memcpy(cube_buff_B[row], cube_buff_B[row+1],CUBE_COL_SIZE);
        }

        //新导入的一行颜色
        memcpy(cube_buff_G[CUBE_RGB_SIZE-1], g, CUBE_COL_SIZE);
        memcpy(cube_buff_R[CUBE_RGB_SIZE-1], r, CUBE_COL_SIZE);
        memcpy(cube_buff_B[CUBE_RGB_SIZE-1], b, CUBE_COL_SIZE);
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
}

void ScenceCycle()
{
    static uint8_t  calc = 0;
    uint8_t         row=0,col=0,base,idx;
    uint8_t         sRGB[3];
    uint16_t        uwPWM [4];
    static uint8_t  buff_G[CUBE_RGB_SIZE],buff_R[CUBE_RGB_SIZE],buff_B[CUBE_RGB_SIZE];
    
    if(newReqFlag)
    {
        newReqFlag = 0;
        calc = 0;
        calc_RGB_COL(buff_G,buff_R,buff_B);
    }

    if(calc==CUBE_RGB_SIZE)
        calc = 0;

    base = calc%CUBE_RGB_SIZE;

    for( col = 0; col < CUBE_COL_SIZE; col++ )
    {
        idx = (col+base)%CUBE_RGB_SIZE;
        //初始化颜色组
        //if(col+base < CUBE_COL_SIZE)
        {
            sRGB[0] = buff_R[idx]/RGB_SCALE;
            sRGB[1] = buff_G[idx]/RGB_SCALE;
            sRGB[2] = buff_B[idx]/RGB_SCALE;
        }
        //else
        {
        //    sRGB[0] = buff_R[CUBE_COL_SIZE-idx];
        //    sRGB[1] = buff_G[CUBE_COL_SIZE-idx];
        //    sRGB[2] = buff_B[CUBE_COL_SIZE-idx];
        }
        
        for( row=0; row<CUBE_ROW_SIZE; row++ )
        {
            cube_buff_G[row][col] = sRGB[1];
            cube_buff_R[row][col] = sRGB[0];
            cube_buff_B[row][col] = sRGB[2];
            //printf(">>> [%d %d %d] %d %d %d\r\n",uwPWM[1],uwPWM[0],uwPWM[2],sRGB[1],sRGB[0],sRGB[2]);
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

void ScenceSection()
{
    uint8_t     col,row,random;

    random = systime%24;

    if(newReqFlag)
    {
        newReqFlag = 0;
        random = 0;
    }

    for( col = 0; col < CUBE_COL_SIZE; col++ )
    {
        for( row=0; row<CUBE_ROW_SIZE; row++ )
        {
            //初始化颜色组
            cube_buff_G[row][col] = ((GRB48[random+(row/6)*8]>>16)&0xff)/RGB_SCALE;
            cube_buff_R[row][col] = ((GRB48[random+(row/6)*8]>>8) &0xff)/RGB_SCALE;
            cube_buff_B[row][col] = ((GRB48[random+(row/6)*8])    &0xff)/RGB_SCALE;
        }
    }

    /*
    for( row=0; row<CUBE_ROW_SIZE; row++ )
    {
        printf("---------------------------\r\n");
        for( col=0; col<CUBE_COL_SIZE; col++ )
        {
            printf("[%d %d]:%x%x%x",row,col,
                    cube_buff_G[row][col],cube_buff_R[row][col],cube_buff_B[row][col]);
        }
        printf("\r\n");
    }*/
}

void ScenceRainbow()
{
    uint16_t    uwPWM [ 4 ];
    uint8_t     sRGB [ 3 ];
    static uint8_t  buff_G[CUBE_ROW_SIZE],buff_R[CUBE_ROW_SIZE],buff_B[CUBE_ROW_SIZE];
    uint8_t     col,row,r,g,b;

    if(newReqFlag)
    {
        newReqFlag = 0;
        calc_RGB_ROW(buff_G,buff_R,buff_B);
    }

#if 1
    for( row=0; row<CUBE_ROW_SIZE; row++ )
    {
        
        //初始化颜色组
        memset(cube_buff_G[row], buff_G[row], CUBE_COL_SIZE);
        memset(cube_buff_R[row], buff_R[row], CUBE_COL_SIZE);
        memset(cube_buff_B[row], buff_B[row], CUBE_COL_SIZE);
    }
#endif
    /*
    cube_buff_G[0][0]=0xff;
    cube_buff_R[0][0]=0;
    cube_buff_B[0][0]=0;
    cube_buff_G[1][0]=0xff;
    cube_buff_R[1][0]=0;
    cube_buff_B[1][0]=0;       
    cube_buff_G[2][0]=0xff;
    cube_buff_R[2][0]=0;
    cube_buff_B[2][0]=0;

    cube_buff_G[3][0]=0;
    cube_buff_R[3][0]=0xff;
    cube_buff_B[3][0]=0;
    cube_buff_G[4][0]=0;
    cube_buff_R[4][0]=0xff;
    cube_buff_B[4][0]=0;
    cube_buff_G[5][0]=0;
    cube_buff_R[5][0]=0xff;
    cube_buff_B[5][0]=0;
    
    cube_buff_G[6][0]=0;
    cube_buff_R[6][0]=0;
    cube_buff_B[6][0]=0xff;
    cube_buff_G[7][0]=0;
    cube_buff_R[7][0]=0;
    cube_buff_B[7][0]=0xff;
    cube_buff_G[8][0]=0;
    cube_buff_R[8][0]=0;
    cube_buff_B[8][0]=0xff;
    */
    /*
    for( row=0; row<CUBE_ROW_SIZE; row++ )
    {
        printf("---------------------------\r\n");
   
        for( col=0; col<CUBE_COL_SIZE/6; col++ )
        {
            printf("[%3d %3d]:%2x%2x%2x  ",row,col,
                    cube_buff_R[row][col],cube_buff_G[row][col],cube_buff_B[row][col]);
        }
        printf("\r\n");
    }*/
}

void ScenceWaving()
{
    static uint8_t  calc = 0;
    uint8_t     row=0,col=0,idx;

    if(calc==CUBE_RGB_SIZE)
        calc=0;
    
    for( row=0; row<CUBE_ROW_SIZE; row++ )
    {
        //初始化颜色组
        for( col=0; col<CUBE_COL_SIZE; col++ )
        {
            idx = (col*2+row+calc)%CUBE_RGB_SIZE;
            cube_buff_G[row][col] = (GRB48[idx]>>16&0xff)/RGB_SCALE;
            cube_buff_R[row][col] = (GRB48[idx]>>8 &0xff)/RGB_SCALE;
            cube_buff_B[row][col] = (GRB48[idx]    &0xff)/RGB_SCALE;
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

uint8_t PhotoCycle()
{
    uint8_t         row=0,col=0;
    uint8_t         buff_G[24],buff_R[24],buff_B[24];
    
    if(newReqFlag)
    {
        newReqFlag = 0;
        calc_RGB_Photo_point();
    }

    for( row=0; row<CUBE_ROW_SIZE; row++ )
    {
        buff_G[row] = cube_buff_G[row][0];
        buff_R[row] = cube_buff_R[row][0];
        buff_B[row] = cube_buff_B[row][0];
    }

    for( col = 0; col < CUBE_COL_SIZE-1; col++ )
    {
        for( row=0; row<CUBE_ROW_SIZE; row++ )
        {
            cube_buff_G[row][col] = cube_buff_G[row][col+1];
            cube_buff_R[row][col] = cube_buff_R[row][col+1];
            cube_buff_B[row][col] = cube_buff_B[row][col+1];
            //printf("[%d %d %d] ",cube_buff_G[row][col],cube_buff_R[row][col],cube_buff_B[row][col]);
        }
        //printf("\r\n");
    }

    for( row=0; row<CUBE_ROW_SIZE; row++ )
    {
        cube_buff_G[row][CUBE_COL_SIZE-1] = buff_G[row];
        cube_buff_R[row][CUBE_COL_SIZE-1] = buff_R[row];
        cube_buff_B[row][CUBE_COL_SIZE-1] = buff_B[row];
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
    
    return 0;
}

void OffRGB()
{
    uint16_t row,col;

    for( row=0; row<CUBE_RGB_SIZE; row++ )
    {
        memset(cube_buff_G[row], 0, CUBE_COL_SIZE);
        memset(cube_buff_R[row], 0, CUBE_COL_SIZE);
        memset(cube_buff_B[row], 0, CUBE_COL_SIZE);
    }
    WS2801_framRefresh();
}

void WS2801_framRefresh(void)
{
    Send_2811_totalPixels1();
    Send_2811_totalPixels2();
}

void WS2801_framShow(void)
{
    uint16_t row,col;
    
    for( row=0; row<CUBE_ROW_SIZE; row++ )
    {
        printf("--------------------------------------up:%d--------------------------------------\r\n",row);
        for( col=0; col<CUBE_PILLAR_SIZE; col++ )
        {
            printf("%2x%2x%2x ",cube_buff_G[row][col], cube_buff_R[row][col], cube_buff_B[row][col]);
        }
        printf("\r\n");
    }

    for( row=0; row<CUBE_ROW_SIZE; row++ )
    {
        printf("--------------------------------------down:%d--------------------------------------\r\n",row);
        for( col=0; col<CUBE_PILLAR_DOWN_SIZE; col++ )
        {
            printf("%2x%2x%2x ",cube_buff_G[row][CUBE_PILLAR_SIZE+col], cube_buff_R[row][CUBE_PILLAR_SIZE+col], cube_buff_B[row][CUBE_PILLAR_SIZE+col]);
        }
        printf("\r\n");
    }
}

FRESULT WS2801_softScen()
{
    FRESULT res = FR_TIMEOUT;

    if(cubeSoftFrameId%PROGRAM_NUM == 0)
    {
        //if(HAL_GetTick()-systime > 1000)
        {
            ScenceGradualChange();
            //systime = HAL_GetTick();
            res = FR_OK;
        }
    }
    /*
    else if(cubeSoftFrameId%PROGRAM_NUM == 1)
    {
        //printf("scenceCandle\r\n");
        scenceCandle();
        res = FR_OK;
    }*/
    else if(cubeSoftFrameId%PROGRAM_NUM == 1)
    {
        if(HAL_GetTick()-systime > 10000)
        {
            //printf("Layering\r\n");
            ScenceLayering();
            systime = HAL_GetTick();
            res = FR_OK;
        }
    }
    else if(cubeSoftFrameId%PROGRAM_NUM == 2)
    {
        if(HAL_GetTick()-systime > 10000)
        {
            ScenceCycle();
            systime = HAL_GetTick();
            res = FR_OK;
        }
    }
    else if(cubeSoftFrameId%PROGRAM_NUM == 3)
    {
        //if(HAL_GetTick()-systime > 1000)
        {
            //printf("Layering\r\n");
            ScenceWaving();
            //systime = HAL_GetTick();
            res = FR_OK;
        }
    }
    else if(cubeSoftFrameId%PROGRAM_NUM == 4)
    {
        if(newReqFlag)
        {
            ScenceRainbow();
            res = FR_OK;
        }
    }
    else if(cubeSoftFrameId%PROGRAM_NUM >= 5)
    {
        if(HAL_GetTick()-systime > 1000000 || newReqFlag)
        {
            systime = HAL_GetTick();
            ScenceSection();
            res = FR_OK;
        }
    }
    
    return res;
}

void WS2801_play(void)
{
    FRESULT res = FR_NOT_READY;
    BOOL    flash_flag = false;
    uint8_t row;

    //if(runFlag || ((programsType!=PHOTO)/* && (HAL_GetTick() - systime>100000)*/))
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
        //runFlag = 0;

        if(cubeRGBStatus)
        {
            RGB_white_firsttime_flag = true;
            //printf("cubept=%d,pt=%d\r\n",cubeSoftFrameId,programsType);
            /*
            if(programsType==FILM)
            {
                res = SD_ReadFilmData();
                if(res==FR_OK)
                {
                    filmFrameIdx++;
                    flash_flag = true;
                }
            }
            else */
            if(programsType==AUTO_ALGORITHM)
            {
                res = WS2801_softScen();
                if(res==FR_OK)
                    flash_flag = true;
                else
                    return;
            }
            else if(programsType==PHOTO)
            {
                if(photoOpenFlag)
                {
                    photoOpenFlag = false;
                    uint8_t path[FILE_PATH_LEN];

                    memset(path, 0, FILE_PATH_LEN);
                    sprintf(path,"/WS2801/Photo/%s",photo_filename[photoProgramIdx%fileTotalPhoto]);
                    res = SD_ReadPhotoData(path);
                }
                else
                {
                    if(HAL_GetTick()-systime > 30000)
                    {
                        //printf("PhotoCycle\r\n");
                        res = PhotoCycle();
                        systime = HAL_GetTick();
                    }
                    else
                    {
                        return;
                    }
                }
                
                if(res==FR_OK)
                    flash_flag = true;
            }
            if(res != FR_OK)
            {
                printf("Read [%s] file failed!\r\n",programsType==PHOTO?"photo":"film");
                return;
            }
        }
        else
        {
            for( row=0; row<CUBE_ROW_SIZE; row++ )
            {
                /*if(powerFlag)
                {
                    memset(cube_buff_G[row], 0xff, CUBE_COL_SIZE);
                    memset(cube_buff_R[row], 0xff, CUBE_COL_SIZE);
                    memset(cube_buff_B[row], 0xff, CUBE_COL_SIZE);
                }
                else*/
                {
                    memset(cube_buff_G[row], 0, CUBE_COL_SIZE);
                    memset(cube_buff_R[row], 0, CUBE_COL_SIZE);
                    memset(cube_buff_B[row], 0, CUBE_COL_SIZE);
                }
            }
            if(RGB_white_firsttime_flag)
            {
                flash_flag = true;
                RGB_white_firsttime_flag = false;
            }
        }
        
        if(flash_flag)
        {
            //usartTxFlag = true;
            //WS2801_framShow();
            WS2801_framRefresh();
        }
#endif
        //systime = HAL_GetTick();
    }
}

void CUBE_Start()
{
    uint8_t row;
    newReqFlag = true;
    ScenceCycle();

    for(int i = 0; i < 10; i++)
    {
        col_down = i+1;
        WS2801_framRefresh();
        Delay_ms(200);
    }
    Delay_ms(500);

    OffRGB();
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

