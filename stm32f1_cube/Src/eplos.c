    /***************************************************************************
    *   @file   eplos.c
    *   @version V1.0.0
    *   @brief   EPLOS芯片驱动相关函数
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
#include "eplos.h"
#include "config.h"
#include "programs.h"

#if(PROJECTOR_OSRAM)

extern BOOL                 runFlag;
extern char                 fileBuffer[MAX_FILE_SIZE];   // file copy buffer
extern char                 osram_buff[MATRIX_SIZE][MATRIX_SIZE];
extern uint8_t              filmFrameIdx;

uint64_t                    systime = 0;
uint64_t                    systime1 = 0;
uint8_t                     currentAdjustment = 0;//0 ~ 0x1f
BOOL                        eplosCfgFlag;
uint8_t                     eplosSLPxen;
extern PROGRAMS_TYPE_E      programsType;

void i2c_read(unsigned char addr, unsigned char* buf, int len);
void i2c_write(unsigned char addr, unsigned char* buf, int len);
uint8_t I2CReadFromRegister(uint8_t SlaveAddress, uint8_t regaddr, uint8_t *resp, uint8_t NumByteToRead);
uint8_t I2CWriteToRegister(uint8_t SlaveAddress, uint8_t RegisterAddr, uint8_t *command, uint8_t NumByteToWrite);

uint8_t displayMatrix0Q[16][16] = {

    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


uint8_t displayMatrix1Q[16][16] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
    0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,
    0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

uint8_t displayMatrix2Q[16][16] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
    0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
    0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

uint8_t displayMatrix3Q[16][16] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,
    0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,
    0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,
    0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,
    0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

uint8_t UpdateCRC8(uint8_t crcIn, uint8_t byte)
{
    uint8_t i;
    uint8_t crc = crcIn;

    crc^= byte;

    for(i=0;i<8;i++)
    {
        if(crc&0x01)
        {
            crc = (crc>>1)^0x8C;
        }
        else
        {
            crc>>=1;
        }
    }
    return crc;
}
#if 0
void CRC8_calc(const void *inSrc, int inLen, uint8_t *outResult)
{
    uint8_t crc = 0;

    const uint8_t * src = (const uint8_t *) inSrc;
    const uint8_t * srcEnd = src + inLen;

    while( src < srcEnd )
        crc = UpdateCRC8(crc, *src++);

    *outResult = crc&0xffu;
}
#else
//CRC-8 x8+x2+x+1
void CRC8_calc(unsigned char *vptr, int len, uint8_t *outResult)
{
    unsigned int crc = 0;
    int i, j;
    const unsigned char *data = vptr;

    for (j = len; j; j--, data++) {
        
        crc ^= (*data << 8);
        for (i = 8; i; i--) {
            if (crc & 0x8000)
            crc ^= (0x1070 << 3);
            crc <<= 1;
        }
    }
    *outResult = (unsigned char)(crc >> 8);
}
#endif

//0x01
void EPLOS_config(void)
{
    uint8_t QuadrantConf[2];

    QuadrantConf[1] = QuadrantConf[0] = eplosSLPxen<<SLP_MASK_BIT | currentAdjustment;//0x0A;//0000 1010 

    I2CWriteToRegister(I2C_ADDR01, EPLOS_CFG01, QuadrantConf, 2);
    I2CWriteToRegister(I2C_ADDR23, EPLOS_CFG23, QuadrantConf, 2);
}

//0x02
void EPLOS_status_read(void)
{
    uint8_t QuadrantConf[2];

    I2CReadFromRegister(I2C_ADDR01, EPLOS_STATUS01, QuadrantConf, 2);
    I2CReadFromRegister(I2C_ADDR23, EPLOS_STATUS23, QuadrantConf, 2);
}

//0x03
void EPLOS_i2cmon(void)
{
    uint8_t QuadrantConf[2];

    QuadrantConf[0] = 0;
    QuadrantConf[1] = 0;
    I2CWriteToRegister(I2C_ADDR01, EPLOS_I2C_MON01, QuadrantConf, 2);
    I2CWriteToRegister(I2C_ADDR23, EPLOS_I2C_MON23, QuadrantConf, 2);
}


//0x05
void EPLOS_scimon(void)
{
    uint8_t QuadrantConf[2];

    /*  FRE0_SEL : bit 5
    0B CRC-8 calculation frame check is selected.
    1B “One counter” frame check algorithm is selected.
    */
    QuadrantConf[0] = 0x20;
    QuadrantConf[1] = 0x20;
    I2CWriteToRegister(I2C_ADDR01, EPLOS_SCI_MON01, QuadrantConf, 2);
    I2CWriteToRegister(I2C_ADDR23, EPLOS_SCI_MON23, QuadrantConf, 2);
}

void EPLOS_scimon_read(void)
{
    uint8_t QuadrantConf[2];

    I2CReadFromRegister(I2C_ADDR01, EPLOS_SCI_MON01, QuadrantConf, 2);
    I2CReadFromRegister(I2C_ADDR23, EPLOS_SCI_MON23, QuadrantConf, 2);
}

//0x06
void EPLOS_diag(void)
{
    uint8_t QuadrantConf[2];

    QuadrantConf[0] = 0x11;
    QuadrantConf[1] = 0x11;
    I2CWriteToRegister(I2C_ADDR01, EPLOS_DIAG_OUT_SEL01, QuadrantConf, 2);
    I2CWriteToRegister(I2C_ADDR23, EPLOS_DIAG_OUT_SEL23, QuadrantConf, 2);
}


void i2cmsginfo(uint8_t *tx, int txlen, uint8_t *rx, int rxlen)
{
    int i;
    if (tx != NULL) {
        LOG_DEBUG("TX:");
        for(i=0;i<txlen;i++) {
            LOG_DEBUG(" 0x%x", tx[i]);
        }
        
        LOG_DEBUG("\r\n");
    }
    if (rx != NULL) {
        LOG_DEBUG("RX:");
        for(i=0;i<rxlen;i++) {
            LOG_DEBUG(" 0x%x", rx[i]);
        }
        LOG_DEBUG("\r\n");
    }
    LOG_DEBUG("----------\r\n");
}

uint8_t OSRAM_I2C_Write(uint8_t* pBuffer, uint8_t SlaveAddress, uint8_t RegisterAddr, uint16_t NumByteToWrite)
{
    uint8_t iError = 0;
    uint8_t array[8];
    uint8_t stringpos;

    memset(array, 0, sizeof(array));

    array[0] = RegisterAddr;
    for (stringpos = 0; stringpos < NumByteToWrite; stringpos++) {
        array[stringpos + 1] = *(pBuffer + stringpos);
    }
    i2c_write(SlaveAddress, array, NumByteToWrite+1);

    i2cmsginfo(array, NumByteToWrite+1, NULL, 0);

    return iError;
}

uint8_t OSRAM_I2C_Write_Block(uint8_t* pBuffer, uint8_t SlaveAddress, uint8_t RegisterAddr, uint16_t NumByteToWrite)
{
    uint8_t iError = 0;
    uint8_t array[8];
    uint8_t stringpos;

    memset(array, 0, sizeof(array));

    array[0] = RegisterAddr;
    for (stringpos = 0; stringpos < NumByteToWrite; stringpos++) {
        array[stringpos + 1] = *(pBuffer + stringpos);
    }
    i2c_write(SlaveAddress, array, NumByteToWrite+1);

    i2cmsginfo(array, NumByteToWrite+1, NULL, 0);

    return iError;
}

uint8_t OSRAM_I2C_Read(uint8_t* pBuffer, uint8_t SlaveAddress, uint8_t RegisterAddr, uint16_t NumByteToRead)
{
  uint8_t arraytx[8] = {0};

  memset(pBuffer, 0, NumByteToRead);
  arraytx[0] = RegisterAddr;

  i2c_write(SlaveAddress, arraytx, 1);
  i2c_read(SlaveAddress, pBuffer, NumByteToRead);

  i2cmsginfo(arraytx, 1, pBuffer, NumByteToRead);

  return pBuffer[0];
}

uint8_t I2CReadFromRegister(uint8_t SlaveAddress, uint8_t regaddr, uint8_t *resp, uint8_t NumByteToRead)
{
    return OSRAM_I2C_Read(resp, SlaveAddress, regaddr, NumByteToRead);
}

uint8_t I2CWriteToRegister(uint8_t SlaveAddress, uint8_t RegisterAddr, uint8_t *command, uint8_t NumByteToWrite)
{
    return OSRAM_I2C_Write(command, SlaveAddress, RegisterAddr, NumByteToWrite);
}

void OSRAM_QuadrantConvert(void)
{
    uint8_t row, col, tmp, tmpR[16];

    //Column Convert
    for(row = 0; row < 16; row++)
    {
        if(row%2)
        {
            for(col = 0; col < 8; col++)
            {
                tmp = displayMatrix0Q[row][15-col];
                displayMatrix0Q[row][15-col] = displayMatrix0Q[row][col];
                displayMatrix0Q[row][col] = tmp;
            }

            for(col = 0; col < 8; col++)
            {
                tmp = displayMatrix2Q[row][15-col];
                displayMatrix2Q[row][15-col] = displayMatrix2Q[row][col];
                displayMatrix2Q[row][col] = tmp;
            }
             
        }

        if(row%2==0)
        {
            for(col = 0; col < 8; col++)
            {
                tmp = displayMatrix1Q[row][15-col];
                displayMatrix1Q[row][15-col] = displayMatrix1Q[row][col];
                displayMatrix1Q[row][col] = tmp;
            }
        
            for(col = 0; col < 8; col++)
            {
                tmp = displayMatrix3Q[row][15-col];
                displayMatrix3Q[row][15-col] = displayMatrix3Q[row][col];
                displayMatrix3Q[row][col] = tmp;
            }
        }
    }

    //Row Convert
    for(row = 0; row < 8; row++)
    {
        memcpy(tmpR, displayMatrix1Q[row], sizeof(tmpR));
        memcpy(displayMatrix1Q[row], displayMatrix1Q[15-row], sizeof(tmpR));
        memcpy(displayMatrix1Q[15-row], tmpR, sizeof(tmpR));
    }

    //Row Convert
    for(row = 0; row < 8; row++)
    {
        memcpy(tmpR, displayMatrix2Q[row], sizeof(tmpR));
        memcpy(displayMatrix2Q[row], displayMatrix2Q[15-row], sizeof(tmpR));
        memcpy(displayMatrix2Q[15-row], tmpR, sizeof(tmpR));
    }
    
}

void OSRAM_QuadrantShow(void)
{
    uint8_t row, col, tmp;

    printf("\r\nOSRAM_QuadrantShow [Q0]\r\n");
    for(row = 0; row < 16; row++)
    {
        for(col = 0; col < 16; col++)
        {
            printf("%d ",displayMatrix0Q[row][col]);
        }
        printf("\r\n");
    }

    printf("\r\nOSRAM_QuadrantShow [Q1]\r\n");
    for(row = 0; row < 16; row++)
    {
        for(col = 0; col < 16; col++)
        {
            printf("%d ",displayMatrix1Q[row][col]);
        }
        printf("\r\n");
    }

    printf("\r\nOSRAM_QuadrantShow [Q2]\r\n");
    for(row = 0; row < 16; row++)
    {
        for(col = 0; col < 16; col++)
        {
            printf("%d ",displayMatrix2Q[row][col]);
        }
        printf("\r\n");
    }
    printf("\r\nOSRAM_QuadrantShow [Q3]\r\n");
    for(row = 0; row < 16; row++)
    {
        for(col = 0; col < 16; col++)
        {
            printf("%d ",displayMatrix3Q[row][col]);
        }
        printf("\r\n");
    }

}

void OSRAM_framRefresh(void)
{
    unsigned short k,fixel;
    uint8_t        crc0=1,crc1=1,crc2=1,crc3=1,mask, row, col;

/*
    CRC8_calc(displayMatrix1Q, sizeof(displayMatrix1Q), &crc0);
    CRC8_calc(displayMatrix1Q, sizeof(displayMatrix1Q), &crc1);
    CRC8_calc(displayMatrix1Q, sizeof(displayMatrix1Q), &crc2);
    CRC8_calc(displayMatrix1Q, sizeof(displayMatrix1Q), &crc3);
*/

    //写入256bits数据
    //for(fixel = 0; fixel < QT_PIXELS; fixel++)
        //row = fixel/16;
        //col = fixel%16;

    for(row = 0; row < 16; row++)
    {
        for(col = 0; col < 16; col++)
        {
            delay(2);
            QT_CLK_H

#if 0
            if(displayMatrix[fixel/16][fixel%16])
                Q0_SI_H

            if(displayMatrix[fixel/16+16][fixel%16])
                Q1_SI_H

            if(displayMatrix[fixel/16+16][fixel%16+16])
                Q2_SI_H

            if(displayMatrix[fixel/16][fixel%16+16])
                Q3_SI_H
#else
#if 1
            if(displayMatrix0Q[row][col])
            {
                Q0_SI_H
                crc0++;
            }
            
            if(displayMatrix1Q[row][col])
            {
                Q1_SI_H
                crc1++;
            }
            if(displayMatrix2Q[row][col])
            {
                Q2_SI_H
                crc2++;
            }
            if(displayMatrix3Q[row][col])
            {
                Q3_SI_H
                crc3++;
            }
#else
            if(col==0)
            {
                Q0_SI_H
                crc0++;
            }
            /*if(displayMatrix1Q[row][col])
            {
                Q1_SI_H
                crc1++;
            }
            if(displayMatrix2Q[row][col])
            {
                Q2_SI_H
                crc2++;
            }
            if(displayMatrix3Q[row][col])
            {
                Q3_SI_H
                crc3++;
            }*/
#endif
#endif
            delay(1);
            QT_CLK_L
            delay(1);
            Q0_SI_L
            Q1_SI_L
            Q2_SI_L
            Q3_SI_L
        }
    }

    for(k = 0; k < 8; k++)
    {
        delay(2);
        QT_CLK_H

        if(k == 7)
        {
            QT_UPD_H
        }
        mask = 0x80 >> k;
        if(crc0 & mask)
            Q0_SI_H
        if(crc1 & mask)
            Q1_SI_H
        if(crc2 & mask)
            Q2_SI_H
        if(crc3 & mask)
            Q3_SI_H

        delay(1);
        QT_CLK_L
        delay(1);
        Q0_SI_L
        Q1_SI_L
        Q2_SI_L
        Q3_SI_L
    }
    QT_UPD_L
}


void OSRAM_framRefreshNull(void)
{
    unsigned short k;
    uint8_t        crc0=1,crc1=1,crc2=1,crc3=1,mask, row, col;

    for(row = 0; row < 16; row++)
    {
        for(col = 0; col < 16; col++)
        {
            delay(2);
            QT_CLK_H
            delay(1);
            QT_CLK_L
            delay(1);
            Q0_SI_L
            Q1_SI_L
            Q2_SI_L
            Q3_SI_L
        }
    }

    for(k = 0; k < 8; k++)
    {
        delay(2);
        QT_CLK_H

        if(k == 7)
        {
            QT_UPD_H
        }
        mask = 0x80 >> k;
        if(crc0 & mask)
            Q0_SI_H
        if(crc1 & mask)
            Q1_SI_H
        if(crc2 & mask)
            Q2_SI_H
        if(crc3 & mask)
            Q3_SI_H

        delay(1);
        QT_CLK_L
        delay(1);
        Q0_SI_L
        Q1_SI_L
        Q2_SI_L
        Q3_SI_L
    }
    QT_UPD_L
}

void OSRAM_config(void)
{

    if(eplosCfgFlag)
    {
        printf("OSRAM_config [%s]\r\n",eplosSLPxen==ENABLE?"OFF":"ON");
        eplosCfgFlag = false;
    
        EPLOS_config();
        EPLOS_scimon();
        //OSRAM_framRefresh();
        //EPLOS_i2cmon();
        //EPLOS_diag();
        //Delay_ms(100);
        //EPLOS_scimon_read();
        //EPLOS_status_read();
    }
}

void OSRAM_play(void)
{
    uint8_t i;
    FRESULT res = FR_OK;
    static uint8_t j=0;

    //if((HAL_GetTick() - systime>1000000) && runFlag)//100ms
    if(runFlag || ((programsType==FILM)/* && (HAL_GetTick() - systime>100000)*/))
    {
#if 0
        memcpy(displayMatrix,cartoonBuffer[32*(j%20)],1024);
        j++;
#else
        runFlag = 0;

        if(programsType==FILM)
        {
            res = SD_ReadFilmData();
            filmFrameIdx++;
        }
        else if(programsType==PHOTO)
        {
            res = SD_ReadPhotoData();
        }
        /*
        else
        {
            for( i=0; i<MATRIX_SIZE; i++ )
            {
                for( j=0; j<MATRIX_SIZE; j++ )
                {
                    osram_buff[i][j] = fileBuffer[i*(64+2)+j*2]-'0';
                }
            }
        }*/
        if(res != FR_OK)
        {
            printf("Read [%s] file failed!\r\n",programsType==PHOTO?"photo":"film");
            return;
        }
        
        for( i = 0; i < 16; i++ )
        {
            memcpy(displayMatrix0Q[i], &osram_buff[16+i][0], 16);
            memcpy(displayMatrix1Q[i], &osram_buff[i][0], 16);
            memcpy(displayMatrix2Q[i], &osram_buff[i][16], 16);
            memcpy(displayMatrix3Q[i], &osram_buff[16+i][16], 16);
        }

        OSRAM_QuadrantConvert();
        //OSRAM_QuadrantShow();
#endif
        systime = HAL_GetTick();

        OSRAM_framRefresh();
        OSRAM_framRefresh();
       // OSRAM_framRefreshNull();
    }
}

#endif

