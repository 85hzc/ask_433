/**
 ******************************************************************************
 * @file    drv_dlpc.c
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_dlpc source file
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_dlpc.h"
#include "drv.h"

#define DLPC_I2C_ADDR       0x36   //0x3A

#define DLPC_WRITE(s,v,n) \
    HAL_I2C_Mem_Write(&hi2c2, DLPC_I2C_ADDR, s, 1, v, n, HAL_MAX_DELAY)
#define DLPC_READ(s,v,n)  \
    HAL_I2C_Mem_Read(&hi2c2, DLPC_I2C_ADDR, s, 1, v, n, HAL_MAX_DELAY)
#define DLPC_READ_PARAM(s,p,c,v,n)  \
    HAL_I2C_Mem_Read_Param(&hi2c2, DLPC_I2C_ADDR, s, 1, p, c, v, n, HAL_MAX_DELAY)

#define DLPC_PROJ_TOGGLE()  \
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10)
#define DLPC_PROJ_ON()  \
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET)
#define DLPC_PROJ_OFF()  \
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET)

#define INPUT_IMAGE_W     1280
#define INPUT_IMAGE_H     720
#define INPUT_CROP_W      INPUT_IMAGE_W
#define INPUT_CROP_H      INPUT_IMAGE_H

// Write Commands
#define CMD_INPUT_SOURCE_WRITE    0x05
#define CMD_INPUT_FORMAT_WRITE    0x07
#define CMD_TEST_PATTERN_WRITE    0x0B
#define CMD_IMAGE_CROP_WRITE      0x10
#define CMD_DISPLAY_SIZE_WRITE    0x12
#define CMD_IMAGE_ORIENT_WRITE    0x14
#define CMD_IMAGE_CURTAIN_WRITE   0x16
#define CMD_IMAGE_FREEZE_WRITE    0x1A
#define CMD_INPUT_IMAGE_WRITE     0x2E
#define CMD_KEYSTONE_CTRL_WRITE   0x88
#define CMD_KEYSTONE_ANGLE_WRITE  0xBB
#define CMD_LED_CURRENT_WRITE     0x54

// Read Commands
#define CMD_INPUT_SOURCE_READ   (CMD_INPUT_SOURCE_WRITE+1)
#define CMD_TEST_PATTERN_READ   (CMD_TEST_PATTERN_WRITE+1)
#define CMD_IMAGE_CROP_READ     (CMD_IMAGE_CROP_WRITE+1)
#define CMD_DISPLAY_SIZE_READ   (CMD_DISPLAY_SIZE_WRITE+1)
#define CMD_IMAGE_ORIENT_READ   (CMD_IMAGE_ORIENT_WRITE+1)
#define CMD_IMAGE_CURTAIN_READ  (CMD_IMAGE_CURTAIN_WRITE+1)
#define CMD_IMAGE_FREEZE_READ   (CMD_IMAGE_FREEZE_WRITE+1)
#define CMD_INPUT_IMAGE_READ    (CMD_INPUT_IMAGE_WRITE+1)
#define CMD_KEYSTONE_CTRL_READ  (CMD_KEYSTONE_CTRL_WRITE+1)
#define CMD_KEYSTONE_ANGLE_READ (CMD_KEYSTONE_ANGLE_WRITE+1)
#define CMD_LED_CURRENT_READ    (CMD_LED_CURRENT_WRITE+1)

/* Private variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c2;

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  The application entry point.
  */
void Drv_DLPC_Init(void)
{  

}

void Drv_DLPC_CMD_Proc(void)
{
  
}

int8_t drv_dlpc_set_current(uint16_t param)
{
  uint8_t data[6];
  uint16_t r,g,b;

  DLPC_READ(CMD_LED_CURRENT_READ, data, 6);
  r = data[0]|(data[1]<<8);
  g = data[2]|(data[3]<<8);
  b = data[4]|(data[5]<<8);
  if (0==param)
  {
    r-=10;
    g-=10;
    b-=10;
  }
  else
  {
    r+=10;
    g+=10;
    b+=10;
  }

  if (r>500) r=500;
  if (g>500) g=500;
  if (b>500) b=500;

  data[0]=r&0x00ff;
  data[1]=(r&0xff00)>>8;
  data[2]=g&0x00ff;
  data[3]=(g&0xff00)>>8;
  data[4]=b&0x00ff;
  data[5]=(b&0xff00)>>8;
  
  DLPC_WRITE(CMD_LED_CURRENT_WRITE, data, 6);

  return (int8_t)HAL_OK;
}

int8_t drv_dlpc_proj_ctrl(uint16_t param)
{
  if (0==param) 
    DLPC_PROJ_OFF();
  else if (1==param) 
    DLPC_PROJ_ON();
  else 
    DLPC_PROJ_TOGGLE();
  
  return 0;
}

int8_t drv_dlpc_set_orient(void)
{
  uint8_t data;

  DLPC_READ(CMD_IMAGE_ORIENT_READ, &data, 1);
  data = ((data+0x02)&0x06)+(data&~0x06);
  DLPC_WRITE(0x14, &data, 1);

  return (int8_t)HAL_OK;
}

int8_t drv_dlpc_reset_keystone(void)
{
  uint8_t data[5];
  data[0] = 0x00;
  data[1] = 0x33;
  data[2] = 0x01;
  data[3] = 0x00;
  data[4] = 0x01;
  DLPC_WRITE(0x88, data, 5);

  return 0;
}

int8_t drv_dlpc_set_keystone(uint8_t d)
{
  uint8_t data[5];
  
  DLPC_READ(0x89, data, 5);  
  if ((data[0]&0x01)==0x00)
  {
    data[0] = 0x01;
    data[1] = 0x33;
    data[2] = 0x01;
    data[3] = 0x00;
    data[4] = 0x01;
    DLPC_WRITE(0x88, data, 5);
    data[0] = 0x00;
    data[1] = 0x00;
    DLPC_WRITE(0xBB, data, 2);
  }

  DLPC_READ(0xBC, data, 2);  
  data[0] = 0x00;
  data[1] = data[1]+(d?1:-1);
  DLPC_WRITE(0xBB, data, 2);

  return 0;
}

int8_t drv_dlpc_set_input(uint16_t param)
{  
  uint8_t data[8], source, re;
  uint16_t input_w,input_h,crop_w,crop_h;

  if(param>=2)
  {
    DLPC_READ(CMD_INPUT_SOURCE_READ, data, 1);
    source = ((data[0]+1)%0x02)&0x03;
  }
  else
  {
    source = param;
  }

  switch (source)
  {
  case 0x00:// External Video Port
    input_w = INPUT_IMAGE_W;
    input_h = INPUT_IMAGE_H;
    crop_w = INPUT_CROP_W;
    crop_h = INPUT_CROP_H;
    break;
  case 0x01:// Test Pattern Generator
    input_w = 854;
    input_h = 480;
    crop_w = 854;
    crop_h = 480;
    break;
  }

  // Curtain enabled
  data[0] = 0x01;
  re=DLPC_WRITE(CMD_IMAGE_CURTAIN_WRITE, data, 1); 
  // Image freeze enabled
  data[0] = 0x01;
  re|=DLPC_WRITE(CMD_IMAGE_FREEZE_WRITE, data, 1);     
  // Input Image format
  data[0] = 0x43;
  re|=DLPC_WRITE(CMD_INPUT_FORMAT_WRITE, data, 1); 
  // Input Image size
  data[0] = (input_w&0x00FF);
  data[1] = (input_w&0xFF00)>>8;
  data[2] = (input_h&0x00FF);
  data[3] = (input_h&0xFF00)>>8;
  re|=DLPC_WRITE(CMD_INPUT_IMAGE_WRITE, data, 4); 
  // Image Crop (size == Input Image size)
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = (crop_w&0x00FF);
  data[5] = (crop_w&0xFF00)>>8;
  data[6] = (crop_h&0x00FF);
  data[7] = (crop_h&0xFF00)>>8;
  re|=DLPC_WRITE(CMD_IMAGE_CROP_WRITE, data, 8); 
  // DMD Display Size
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = (854&0x00FF);
  data[5] = (854&0xFF00)>>8;
  data[6] = (480&0x00FF);
  data[7] = (480&0xFF00)>>8;
  re|=DLPC_WRITE(CMD_DISPLAY_SIZE_WRITE, data, 8); 
  // Set to External Input
  data[0] = source;
  re|=DLPC_WRITE(CMD_INPUT_SOURCE_WRITE, data, 1); 
  // Unfreeeze
  data[0] = 0x00;
  re|=DLPC_WRITE(CMD_IMAGE_FREEZE_WRITE, data, 1); 
  // Curtain disabled
  data[0] = 0x00;
  re|=DLPC_WRITE(CMD_IMAGE_CURTAIN_WRITE, data, 1); 

  return 0;
}

int8_t drv_dlpc_switch_test_pattern(void)
{
  uint8_t data[8];

  DLPC_READ(CMD_INPUT_SOURCE_READ, data, 1);
  if ((data[0]&0x03)==0x01)
  {
    uint8_t count;
    DLPC_READ(CMD_TEST_PATTERN_READ, data, 6);
    data[0] = ((((data[0]&0x0f)+1)%0x09)&0x0f) + (data[0]&(~0x0f));
    switch(data[0]&0x0f)
    {
    case 0x00:
      data[1]=0x10;
      count=2;
      break;
    case 0x01:
      data[1]=0x70;
      data[2]=0x00;
      data[3]=0xff;
      count=4;
      break;
    case 0x02:
      data[1]=0x70;
      data[2]=0x00;
      data[3]=0xff;
      count=4;
      break;
    case 0x03:      
      data[1]=0x70;
      data[2]=0x01;
      data[3]=0x09;
      count=4;
      break;
    case 0x04:
      data[1]=0x70;
      data[2]=0x1f;
      data[3]=0x1f;
      count=4;
      break;
    case 0x05:
      data[1]=0x70;
      data[2]=0x01;
      data[3]=0x09;
      count=4;
      break;
    case 0x06:
      data[1]=0x70;
      data[2]=0x01;
      data[3]=0x00;      
      data[4]=0x01;
      data[5]=0x00;
      count=6;
      break;
    case 0x07:  
      data[1]=0x70;
      data[2]=0x10;
      data[3]=0x00;      
      data[4]=0x0c;
      data[5]=0x00;
      data[6]=0x00;
      count=7;
      break;
    case 0x08:      
      count=1;
      break;
    }
    DLPC_WRITE(CMD_TEST_PATTERN_WRITE, data, count);
  }

  return 0;
}

int8_t drv_dlpc_sw(void)
{
  uint8_t data[6],param = 0;
  
  if (HAL_OK == DLPC_READ(0xD1, data, 4))
    Drv_SERIAL_Log("System Status Byte1 0x%02X, Byte2 0x%02X, Byte3 0x%02X, Byte4 0x%02X", data[0],data[1],data[2],data[3]);
  
  if (HAL_OK == DLPC_READ(0xD4, data, 1))
    Drv_SERIAL_Log("Controller Device ID 0x%02X", data[0]);

  if (HAL_OK == DLPC_READ_PARAM(0xD5, &param, 1, data, 4))
    Drv_SERIAL_Log("DMD device ID Byte1 0x%02X, Byte2 0x%02X, Byte3 0x%02X, Byte4 0x%02X", data[0],data[1],data[2],data[3]);
  
  if (HAL_OK == DLPC_READ(0xD2, data, 4))
    Drv_SERIAL_Log("sw patch LSB 0x%02X, MSB 0x%02X; sw Minor 0x%02X, Major 0x%02X", data[0],data[1],data[2],data[3]);
  
  if (HAL_OK == DLPC_READ(0xD9, data, 4))
    Drv_SERIAL_Log("flash patch LSB 0x%02X, MSB 0x%02X; flash Minor 0x%02X, Major 0x%02X", data[0],data[1],data[2],data[3]);

  if (HAL_OK == DLPC_READ(0x55, data, 6))
    Drv_SERIAL_Log("RED 0x%02X, 0x%02X; Green 0x%02X, 0x%02X; Blue 0x%02X, 0x%02X", data[0],data[1],data[2],data[3],data[4],data[5]);
  
  return 0;
}

