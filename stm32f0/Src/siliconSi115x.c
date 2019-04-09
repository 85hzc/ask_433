/**
 ******************************************************************************
 * @file    SILAB.c
 * @author  MEMS Application Team
 * @version V1.2.0
 * @date    28-January-2015
 * @brief   This file provides a set of functions needed to manage the SILAB.
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "siliconSi115x.h"


/* ------------------------------------------------------- */ 
/* Here you should declare the variable that implements    */
/* the internal struct of extended features of SILAB.    */
/* Then you must update the NULL pointer in the variable   */
/* of the extended features below.                         */
/* See the example of LSM6DS3 in lsm6ds3.c                 */
/* ------------------------------------------------------- */
#if 0
SILAB_SENSOR_DrvExtTypeDef SILABDrv_ext = {
    SILAB_SENSOR_SILAB_DIL24_COMPONENT, /* unique ID for SILAB in the SILAB_SENSOR driver class */
    NULL /* pointer to internal struct of extended features of SILAB */
}; 
#endif
uint8_t SILAB_SlaveAddress = SI1153_I2C_ADDR;//SILAB_ADDRESS_LOW;


/*	\Brief: The function is used as I2C bus write
*	\Return : Status of the I2C write
*	\param dev_addr : The device address of the sensor
*	\param reg_addr : Address of the first register, will data is going to be written
*	\param reg_data : It is a value hold in the array,
*		will be used for write the value into the register
*	\param cnt : The no of byte of data to be write
*/
SILAB_SENSOR_StatusTypeDef SILAB_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToWrite)
{
  //mico_i2c_message_t SILAB_i2c_msg = {NULL, NULL, 0, 0, 0, false};
  int iError = 0;
  uint8_t array[8];
  uint8_t stringpos;

  memset(array, 0, sizeof(array));

  array[0] = 0x40 | RegisterAddr;
  for (stringpos = 0; stringpos < NumByteToWrite; stringpos++) {
    array[stringpos + 1] = *(pBuffer + stringpos);
  }
#if(HWI2C)
  SILAB_WRITE(array[0], pBuffer, NumByteToWrite);
#else
  i2c_write(DeviceAddr, array, NumByteToWrite+1);
#endif
#if 0
  iError = MicoI2cBuildTxMessage(&SILAB_i2c_msg, array, NumByteToWrite+1, 3);
#ifndef GPIO_I2C
  iError = MicoI2cTransfer(&SILAB_i2c_device, &SILAB_i2c_msg, 1);
  if(0 != iError){
      SILAB_log("SILAB_SENSOR_ERROR");
    iError = SILAB_SENSOR_ERROR;
  }
#else

#ifdef CTI2C
  CT_Write_Nbyte(DeviceAddr, array[0], NumByteToWrite, &array[1]);
#else
  i2c_write(DeviceAddr, array, NumByteToWrite+1);
#endif

#endif
#endif
  //i2cmsginfo(SILAB_i2c_msg);
  i2cmsginfo(array, NumByteToWrite+1, NULL, 0);
  
  return (SILAB_SENSOR_StatusTypeDef)iError;
}

SILAB_SENSOR_StatusTypeDef SILAB_IO_Write_Block(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToWrite)
{
  //mico_i2c_message_t SILAB_i2c_msg = {NULL, NULL, 0, 0, 0, false};
  int iError = 0;
  uint8_t array[8];
  uint8_t stringpos;

  memset(array, 0, sizeof(array));

  array[0] = RegisterAddr;
  for (stringpos = 0; stringpos < NumByteToWrite; stringpos++) {
    array[stringpos + 1] = *(pBuffer + stringpos);
  }
#if(HWI2C)
  SILAB_WRITE(array[0], pBuffer, NumByteToWrite);
#else
  i2c_write(DeviceAddr, array, NumByteToWrite+1);
#endif
#if 0
  iError = MicoI2cBuildTxMessage(&SILAB_i2c_msg, array, NumByteToWrite+1, 3);
#ifndef GPIO_I2C
  iError = MicoI2cTransfer(&SILAB_i2c_device, &SILAB_i2c_msg, 1);
  if(0 != iError){
      SILAB_log("SILAB_SENSOR_ERROR");
    iError = SILAB_SENSOR_ERROR;
  }
#else

#ifdef CTI2C
  CT_Write_Nbyte(DeviceAddr, array[0], NumByteToWrite, &array[1]);
#else
  i2c_write(DeviceAddr, array, NumByteToWrite+1);
#endif
#endif
#endif
  i2cmsginfo(array, NumByteToWrite+1, NULL, 0);

  return (SILAB_SENSOR_StatusTypeDef)iError;
}
#if 0
SILAB_SENSOR_StatusTypeDef SILAB_IO_Write_single(uint8_t DeviceAddr, uint8_t RegisterAddr)
{
  mico_i2c_message_t SILAB_i2c_msg = {NULL, NULL, 0, 0, 0, false};
  int iError = 0;
  uint8_t array[8];
  uint8_t stringpos;
  //array[0] = RegisterAddr;
  array[0] = 0x40 | RegisterAddr;
  SILAB_log("SILAB_IO_Write_single");
  iError = MicoI2cBuildTxMessage(&SILAB_i2c_msg, array, 1, 3);
#ifndef GPIO_I2C
  iError = MicoI2cTransfer(&SILAB_i2c_device, &SILAB_i2c_msg, 1);
  if(0 != iError){
      SILAB_log("SILAB_SENSOR_ERROR");
    iError = SILAB_SENSOR_ERROR;
  }
#else

#ifdef CTI2C
  CT_Write_Nbyte(DeviceAddr, array[0], 0, &array[1]);
#else
  i2c_write(DeviceAddr, array, 1);
#endif

#endif
  i2cmsginfo(SILAB_i2c_msg);

  return (SILAB_SENSOR_StatusTypeDef)iError;
}
#endif
/*	\Brief: The function is used as I2C bus read
*	\Return : Status of the I2C read
*	\param dev_addr : The device address of the sensor
*	\param reg_addr : Address of the first register, will data is going to be read
*	\param reg_data : This data read from the sensor, which is hold in an array
*	\param cnt : The no of byte of data to be read
*/
uint8_t SILAB_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToRead)
{
  int iError = 0;
  //mico_i2c_message_t SILAB_i2c_msg = {NULL, NULL, 0, 0, 0, false};
  uint8_t arraytx[8] = {0};

  memset(pBuffer, 0, NumByteToRead);
  arraytx[0] = 0x40 | RegisterAddr;
#if(HWI2C)
  iError = SILAB_READ(arraytx[0], pBuffer, NumByteToRead);
#else
  i2c_write(DeviceAddr, arraytx, 1);
  i2c_read(DeviceAddr, pBuffer, NumByteToRead);
#endif
  //Drv_SERIAL_Log("error:%d",iError);

#if 0
  iError = MicoI2cBuildCombinedMessage(&SILAB_i2c_msg, array, pBuffer, 1, NumByteToRead, 3);
  if(0 != iError){
      SILAB_log("SILAB_SENSOR_ERROR");
    return SILAB_SENSOR_ERROR;
  }
#ifndef GPIO_I2C
  iError = MicoI2cTransfer(&SILAB_i2c_device, &SILAB_i2c_msg, 1);
  if(0 != iError){
      SILAB_log("SILAB_SENSOR_ERROR");
    return SILAB_SENSOR_ERROR;
  }
  i2cmsginfo(SILAB_i2c_msg);
#else
#ifdef CTI2C
  if(CT_COM_OK != CT_Read_Nbyte(DeviceAddr, RegisterAddr|0x40, NumByteToRead, pBuffer)) {
      SILAB_log("CT_Read_Nbyte error!");
  } else {
      int i = 0;
      printf("CT_Read_Nbyte:");
      while(NumByteToRead-i) {
          printf("%x",pBuffer[i++]);
      }
      printf("\r\n");
  }
#else
  i2c_read(DeviceAddr, pBuffer, NumByteToRead);
#endif
#endif
#endif
  i2cmsginfo(arraytx, 1, pBuffer, NumByteToRead);

  return (uint8_t)pBuffer[0];
}
#if 0
uint8_t SILAB_IO_Read_Block(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToRead)
{
  int iError = 0;

  mico_i2c_message_t SILAB_i2c_msg = {NULL, NULL, 0, 0, 0, false};

  uint8_t array[8] = {0};
  array[0] = RegisterAddr;
  iError = MicoI2cBuildCombinedMessage(&SILAB_i2c_msg, array, pBuffer, 1, NumByteToRead, 3);
  if(0 != iError){
      SILAB_log("SILAB_SENSOR_ERROR");
    return SILAB_SENSOR_ERROR;
  }
#ifndef GPIO_I2C
  iError = MicoI2cTransfer(&SILAB_i2c_device, &SILAB_i2c_msg, 1);
  if(0 != iError){
      SILAB_log("SILAB_SENSOR_ERROR");
    return SILAB_SENSOR_ERROR;
  }
  i2cmsginfo(SILAB_i2c_msg);
#else
#ifdef CTI2C
  if(CT_COM_OK != CT_Read_Nbyte(DeviceAddr, RegisterAddr, NumByteToRead, pBuffer)) {
      SILAB_log("CT_Read_Nbyte error!");
  } else {
      int i = 0;
      printf("CT_Read_Nbyte:");
      while(NumByteToRead-i) {
          printf("%x",pBuffer[i++]);
      }
      printf("\r\n");
  }
#else
  i2c_read(DeviceAddr, pBuffer, NumByteToRead);
#endif

#endif
  i2cmsginfo(SILAB_i2c_msg);

  return (uint8_t)pBuffer[0];
}

SILAB_SENSOR_StatusTypeDef SILAB_IO_Read_single(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t NumByteToRead)
{
  int iError = 0;
  mico_i2c_message_t SILAB_i2c_msg = {NULL, NULL, 0, 0, 0, false};

  SILAB_log("SILAB_IO_Read_single");
  uint8_t array[8] = {0};
  //array[0] = 0x40 | RegisterAddr;
  //array[0] = 0xff;
  iError = MicoI2cBuildCombinedMessage(&SILAB_i2c_msg, array, pBuffer, 0, NumByteToRead, 3);
  if(0 != iError){
      SILAB_log("SILAB_SENSOR_ERROR");
    return SILAB_SENSOR_ERROR;
  }
#ifndef GPIO_I2C
  iError = MicoI2cTransfer(&SILAB_i2c_device, &SILAB_i2c_msg, 1);
  if(0 != iError){
      SILAB_log("SILAB_SENSOR_ERROR");
    return SILAB_SENSOR_ERROR;
  }
  i2cmsginfo(SILAB_i2c_msg);

#else

#ifdef CTI2C
  if(CT_COM_OK != CT_Read_Nbyte(DeviceAddr, RegisterAddr|0x40, NumByteToRead, pBuffer)) {
      SILAB_log("CT_Read_Nbyte error!");
  } else {
      int i = 0;
      printf("CT_Read_Nbyte:");
      while(NumByteToRead-i) {
          printf("%x",pBuffer[i++]);
      }
      printf("\r\n");
  }
#else
  i2c_read(DeviceAddr, pBuffer, NumByteToRead);
#endif

#endif

  return (SILAB_SENSOR_StatusTypeDef)iError;
}
#endif
/**
 * @brief  Read ID address of SILAB
 * @param  ht_id the pointer where the ID of the device is stored
 * @retval SILAB_SENSOR_OK in case of success, an error code otherwise
 */
SILAB_SENSOR_StatusTypeDef SILAB_ReadID()
{
    uint8_t resp;

    return SILAB_IO_Read(&resp, SILAB_SlaveAddress, SILAB_WHO_AM_I_ADDR, 1);
}
#if 0
/**
 * @brief  Reboot memory content of SILAB
 * @param  None
 * @retval SILAB_SENSOR_OK in case of success, an error code otherwise
 */
static SILAB_SENSOR_StatusTypeDef SILAB_RebootCmd(void)
{
    uint8_t tmpreg;

    /* Read CTRL_REG5 register */
    if(SILAB_IO_Read(&tmpreg, SILAB_SlaveAddress, SILAB_CTRL_REG2_ADDR, 1) != SILAB_SENSOR_OK)
    {
      return SILAB_SENSOR_ERROR;
    }

    /* Enable or Disable the reboot memory */
    tmpreg |= SILAB_RESET_MEMORY;

    /* Write value to MEMS CTRL_REG5 regsister */
    if(SILAB_IO_Write(&tmpreg, SILAB_SlaveAddress, SILAB_CTRL_REG2_ADDR, 1) != SILAB_SENSOR_OK)
    {
      return SILAB_SENSOR_ERROR;
    }
    
    return SILAB_SENSOR_OK;
}


/**
 * @brief  Read SILAB output register, and calculate the raw SILAB_SENSOR
 * @param  raw_press the SILAB_SENSOR raw value
 * @retval SILAB_SENSOR_OK in case of success, an error code otherwise
 */
static SILAB_SENSOR_StatusTypeDef SILAB_I2C_ReadRawSILAB_SENSOR(uint32_t *raw_press)
{
    uint8_t buffer[3], i;
    uint32_t tempVal=0;
    uint8_t tmp = 0x00;
    
    if(SILAB_IO_Read(&tmp, SILAB_SlaveAddress, SILAB_CTRL_REG1_ADDR, 1) != SILAB_SENSOR_OK)
    {
      return SILAB_SENSOR_ERROR;
    }

    /* Output Data Rate selection */
    tmp &= (SILAB_ODR_MASK);
    
    if(tmp == 0x00)
    {
      if(SILAB_IO_Read(&tmp, SILAB_SlaveAddress, SILAB_CTRL_REG2_ADDR, 1) != SILAB_SENSOR_OK)
      {
        return SILAB_SENSOR_ERROR;
      }

      /* Serial Interface Mode selection */
      tmp &= ~(SILAB_ONE_SHOT_MASK);
      tmp |= SILAB_ONE_SHOT_START;

      if(SILAB_IO_Write(&tmp, SILAB_SlaveAddress, SILAB_CTRL_REG2_ADDR, 1) != SILAB_SENSOR_OK)
      {
        return SILAB_SENSOR_ERROR;
      }
    
      do{
      
        if(SILAB_IO_Read(&tmp, SILAB_SlaveAddress, SILAB_STATUS_REG_ADDR, 1) != SILAB_SENSOR_OK)
        {
          return SILAB_SENSOR_ERROR;
        }
       
      }while(!(tmp&&0x01));
    }    
    
    /* Read the register content */

    if(SILAB_IO_Read(buffer, SILAB_SlaveAddress, (SILAB_PRESS_POUT_XL_ADDR | SILAB_I2C_MULTIPLEBYTE_CMD), 3) != SILAB_SENSOR_OK)
    {
      return SILAB_SENSOR_ERROR;
    }

    /* Build the raw data */
    for (i = 0 ; i < 3 ; i++)
        tempVal |= (((uint32_t) buffer[i]) << (8 * i));

    /* convert the 2's complement 24 bit to 2's complement 32 bit */
    if (tempVal & 0x00800000)
        tempVal |= 0xFF000000;

    /* return the built value */
    *raw_press = ((uint32_t) tempVal);
    
    return SILAB_SENSOR_OK;
}

/**
 * @brief  Read SILAB output register, and calculate the SILAB_SENSOR in mbar
 * @param  pfData the SILAB_SENSOR value in mbar
 * @retval SILAB_SENSOR_OK in case of success, an error code otherwise
 */
static SILAB_SENSOR_StatusTypeDef SILAB_GetSILAB_SENSOR(float* pfData)
{
    uint32_t raw_press = 0;
    uint8_t tmp = 0x00;
    
    if(SILAB_IO_Read(&tmp, SILAB_SlaveAddress, SILAB_CTRL_REG1_ADDR, 1) != SILAB_SENSOR_OK)
    {
      return SILAB_SENSOR_ERROR;
    }

    /* Output Data Rate selection */
    tmp &= (SILAB_ODR_MASK);
    
    if(tmp == 0x00)
    {
      if(SILAB_IO_Read(&tmp, SILAB_SlaveAddress, SILAB_CTRL_REG2_ADDR, 1) != SILAB_SENSOR_OK)
      {
        return SILAB_SENSOR_ERROR;
      }

      /* Serial Interface Mode selection */
      tmp &= ~(SILAB_ONE_SHOT_MASK);
      tmp |= SILAB_ONE_SHOT_START;

      if(SILAB_IO_Write(&tmp, SILAB_SlaveAddress, SILAB_CTRL_REG2_ADDR, 1) != SILAB_SENSOR_OK)
      {
        return SILAB_SENSOR_ERROR;
      }
    
      do{
      
        if(SILAB_IO_Read(&tmp, SILAB_SlaveAddress, SILAB_STATUS_REG_ADDR, 1) != SILAB_SENSOR_OK)
        {
          return SILAB_SENSOR_ERROR;
        }
       
      }while(!(tmp&&0x01));
    }    

    if(SILAB_I2C_ReadRawSILAB_SENSOR(&raw_press) != SILAB_SENSOR_OK)
    {
      return SILAB_SENSOR_ERROR;
    }

    *pfData = (float)raw_press /4096.0f;
    
    return SILAB_SENSOR_OK;
}

/**
 * @brief  Exit the shutdown mode for SILAB
 * @param  None
 * @retval SILAB_SENSOR_OK in case of success, an error code otherwise
 */
static SILAB_SENSOR_StatusTypeDef SILAB_PowerOn(void)
{
    uint8_t tmpreg;

    /* Read the register content */
    if(SILAB_IO_Read(&tmpreg, SILAB_SlaveAddress, SILAB_CTRL_REG1_ADDR, 1) != SILAB_SENSOR_OK)
    {
      return SILAB_SENSOR_ERROR;
    }

    /* Set the power down bit */
    tmpreg |= SILAB_MODE_ACTIVE;

    /* Write register */
    if(SILAB_IO_Write(&tmpreg, SILAB_SlaveAddress, SILAB_CTRL_REG1_ADDR, 1) != SILAB_SENSOR_OK)
    {
      return SILAB_SENSOR_ERROR;
    }
    
    return SILAB_SENSOR_OK;
}


/**
 * @brief  Enter the shutdown mode for SILAB
 * @param  None
 * @retval SILAB_SENSOR_OK in case of success, an error code otherwise
 */
static SILAB_SENSOR_StatusTypeDef SILAB_PowerOff(void)
{
    uint8_t tmpreg;

    /* Read the register content */
    if(SILAB_IO_Read(&tmpreg, SILAB_SlaveAddress, SILAB_CTRL_REG1_ADDR, 1) != SILAB_SENSOR_OK)
    {
      return SILAB_SENSOR_ERROR;
    }

    /* Reset the power down bit */
    tmpreg &= ~(SILAB_MODE_ACTIVE);

    /* Write register */
    if(SILAB_IO_Write(&tmpreg, SILAB_SlaveAddress, SILAB_CTRL_REG1_ADDR, 1) != SILAB_SENSOR_OK)
    {
      return SILAB_SENSOR_ERROR;
    }
    
    return SILAB_SENSOR_OK;
}

/**
 * @brief  Set the slave address according to SA0 bit
 * @param  SA0_Bit_Status SILAB_SA0_LOW or SILAB_SA0_HIGH
 * @retval None
 */
static void SILAB_SlaveAddrRemap(uint8_t SA0_Bit_Status)
{
    SILAB_SlaveAddress = (SA0_Bit_Status==SILAB_SA0_LOW?SILAB_ADDRESS_LOW:SILAB_ADDRESS_HIGH);
}

OSStatus SILAB_sensor_init(void)
{
  SILAB_SENSOR_InitTypeDef SILAB;
  SILAB.OutputDataRate = SILAB_ODR_ONE_SHOT;
  SILAB.SILAB_SENSORResolution = SILAB_P_RES_AVG_32;
  SILAB.TemperatureResolution = SILAB_T_RES_AVG_32;
  SILAB.DiffEnable = SILAB_DIFF_ENABLE;
  SILAB.BlockDataUpdate = SILAB_BDU_READ;
  SILAB.SPIMode = SILAB_SPI_SIM_4W;

  if(SILAB_Init(&SILAB) != SILAB_SENSOR_OK){
    return -1;
  }
  return 0;
}

OSStatus SILAB_Read_Data(float *temperature,float *SILAB_SENSOR)
{
  if(SILAB_GetTemperature(temperature) != SILAB_SENSOR_OK){
    return -1;
  }
  if(SILAB_GetSILAB_SENSOR(SILAB_SENSOR) != SILAB_SENSOR_OK){
    return -1;
  }
  return 0;
}
#endif

SILAB_SENSOR_StatusTypeDef Si115xReadFromRegister(uint8_t regaddr)
{
    uint8_t resp;
    return SILAB_IO_Read(&resp, SILAB_SlaveAddress, regaddr, 1);
}

SILAB_SENSOR_StatusTypeDef Si115xWriteToRegister(uint8_t RegisterAddr, uint8_t command, uint8_t NumByteToWrite)
{
    return SILAB_IO_Write(&command, SILAB_SlaveAddress, RegisterAddr, NumByteToWrite);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/     
