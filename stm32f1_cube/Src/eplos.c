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

#if(PROJECTOR_OSRAM)

//uint8_t OSRAM_SlaveAddress = 0;

void i2c_read(unsigned char addr, unsigned char* buf, int len);
void i2c_write(unsigned char addr, unsigned char* buf, int len);
uint8_t I2CReadFromRegister(uint8_t SlaveAddress, uint8_t regaddr, uint8_t *resp);
uint8_t I2CWriteToRegister(uint8_t SlaveAddress, uint8_t RegisterAddr, uint8_t *command, uint8_t NumByteToWrite);

uint8_t displayMatrix[32][32] = {};

uint8_t displayMatrix1Q[256] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0,
    1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0,
    1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0,
    1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0,
    1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0,
    1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

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

void CRC8_calc(const void *inSrc, int inLen, uint8_t *outResult)
{
    uint8_t crc = 0;

    const uint8_t * src = (const uint8_t *) inSrc;
    const uint8_t * srcEnd = src + inLen;

    while( src < srcEnd )
        crc = UpdateCRC8(crc, *src++);

    *outResult = crc&0xffu;
}

void eplos_config(void)
{
    uint8_t QuadrantConf[2];

    QuadrantConf[0] = 0x0A;
    QuadrantConf[1] = 0x0A;
    I2CWriteToRegister(I2C_ADDR01, EPLOS_CFG01, QuadrantConf, 2);
    I2CWriteToRegister(I2C_ADDR23, EPLOS_CFG23, QuadrantConf, 2);
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

uint8_t I2CReadFromRegister(uint8_t SlaveAddress, uint8_t regaddr, uint8_t *resp)
{
    return OSRAM_I2C_Read(resp, SlaveAddress, regaddr, 1);
}

uint8_t I2CWriteToRegister(uint8_t SlaveAddress, uint8_t RegisterAddr, uint8_t *command, uint8_t NumByteToWrite)
{
    return OSRAM_I2C_Write(command, SlaveAddress, RegisterAddr, NumByteToWrite);
}

void OSRAM_play(void)
{
    unsigned short k,fixel;
    uint8_t        crc0,crc1,crc2,crc3,mask;

    printf("OSRAM_play\r\n");

    CRC8_calc(displayMatrix1Q, sizeof(displayMatrix1Q), &crc0);

    //写入256bits数据
    for(fixel = 0; fixel <= QT_PIXELS; fixel++)
    {
        delay(1);
        QT_CLK_H

        if(displayMatrix1Q[fixel])
            Q0_SI_H

        if(displayMatrix1Q[fixel])
            Q1_SI_H

        if(displayMatrix1Q[fixel])
            Q2_SI_H

        if(displayMatrix1Q[fixel])
            Q3_SI_H

        delay(1);
        Q0_SI_L
        Q1_SI_L
        Q2_SI_L
        Q3_SI_L

        QT_CLK_L
    }

    for(k = 0; k < 8; k++)
    {
        delay(1);
        QT_CLK_H

        mask = 0x80 >> k;
        if(crc0 & mask)
            Q0_SI_H
        if(crc0 & mask)
            Q1_SI_H
        if(crc0 & mask)
            Q2_SI_H
        if(crc0 & mask)
            Q3_SI_H

        if(k == 7)
        {
            Q1_UPD_H
        }

        delay(1);
        Q0_SI_L
        Q1_SI_L
        Q2_SI_L
        Q3_SI_L

        QT_CLK_L
    }

    Q1_UPD_L
}
#endif

