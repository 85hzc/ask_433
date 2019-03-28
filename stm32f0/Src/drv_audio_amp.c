/**
 ******************************************************************************
 * @file    drv_au_amp.c
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_au_amp source file
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_serial.h"
#include "drv_audio_amp.h"

#define AUDIO_I2C_ADDR       0x30   //0x3A

#define AUDIO_WRITE(s,v,n) \
    HAL_I2C_Mem_Write(&hi2c2, AUDIO_I2C_ADDR, s, 1, v, n, HAL_MAX_DELAY)
#define AUDIO_READ(s,v,n)  \
    HAL_I2C_Mem_Read(&hi2c2, AUDIO_I2C_ADDR, s, 1, v, n, HAL_MAX_DELAY)
    
/* Private variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c2;

/* Private function prototypes -----------------------------------------------*/


/**
  * @brief  The application entry point.
  */
void Drv_AU_AMP_Init(void)
{
  uint8_t data[2];
  uint8_t re;
#if 0
  uint8_t i;
  for (i=0x00,data[0]=0x00;i<0xFE;i+=2)
  {
    re=HAL_I2C_Mem_Write(&hi2c2, i, 0x00, 1, data, 1, HAL_MAX_DELAY);
    if (re==0)
    {
      Drv_SERIAL_Log("audio i2c address found: 0x%02x", i);
    }
    HAL_Delay(10);
  }  
  Drv_SERIAL_Log("audio i2c address finish");
#else
//# Key: w 30 XX YY ==> write to I2C address 0x30, to register 0xXX, data 0xYY
//# # ==> comment delimiter
//#
//# The following list gives an example sequence of items that must be executed in the time
//# between powering the # device up and reading data from the device. Note that there are
//# other valid sequences depending on which features are used.
//# 1. Define starting point:
//# (a) Power up applicable external hardware power supplies
//# (b) Set register page to 0
//#
//w 30 00 00
  data[0]=0x00;
  re=AUDIO_WRITE(0x00,data,1);
  goto LOG;
//#
//# (c) Initiate SW reset (PLL is powered off as part of reset)
//#
//w 30 01 01
  data[0]=0x01;
  re|=AUDIO_WRITE(0x01,data,1);
//#
//# 2. Program clock settings
//# (a) Program PLL clock dividers P, J, D, R (if PLL is used)
//#
//# PLL_clkin = MCLK,codec_clkin = PLL_CLK
//w 30 04 03
  data[0]=0x03;
  re|=AUDIO_WRITE(0x04,data,1);
//# J = 8
//w 30 06 08
  data[0]=0x08;
  re|=AUDIO_WRITE(0x06,data,1);
//# D = 0000, D(13:8) = 0, D(7:0) = 0
//w 30 07 00 00
  data[0]=0x00;
  data[1]=0x00;
  re|=AUDIO_WRITE(0x07,data,2);
//#
//# (b) Power up PLL (if PLL is used)
//# PLL Power up, P = 1, R = 1
//#
//w 30 05 91
  data[0]=0x91;
  re|=AUDIO_WRITE(0x05,data,1);
//#
//# (c) Program and power up NDAC
//#
//# NDAC is powered up and set to 8
//w 30 0B 88
  data[0]=0x88;
  re|=AUDIO_WRITE(0x0B,data,1);
//#
//# (d) Program and power up MDAC
//#
//# MDAC is powered up and set to 2
//w 30 0C 82
  data[0]=0x82;
  re|=AUDIO_WRITE(0x0C,data,1);
//#
//# (e) Program OSR value
//#
//# DOSR = 128, DOSR(9:8) = 0, DOSR(7:0) = 128
//w 30 0D 00 80
  data[0]=0x00;
  data[1]=0x80;
  re|=AUDIO_WRITE(0x0D,data,2);
//#
//# (f) Program I2S word length if required (16, 20, 24, 32 bits)
//# and master mode (BCLK and WCLK are outputs)
//#
//# mode is i2s, wordlength is 16, slave mode
//w 30 1B 00
  data[0]=0x00;
  re|=AUDIO_WRITE(0x1B,data,1);
//# (g) Program the processing block to be used
//#
//# Select Processing Block PRB_P11
//w 30 3C 0B
  data[0]=0x0B;
  re|=AUDIO_WRITE(0x3C,data,1);
//w 30 00 08
  data[0]=0x08;
  re|=AUDIO_WRITE(0x00,data,1);
//w 30 01 04
  data[0]=0x04;
  re|=AUDIO_WRITE(0x01,data,1);
//w 30 00 00
  data[0]=0x00;
  re|=AUDIO_WRITE(0x00,data,1);
//#
//# (h) Miscellaneous page 0 controls
//#
//# DAC => volume control thru pin disable
//w 30 74 00
  data[0]=0x00;
  re|=AUDIO_WRITE(0x74,data,1);
//# 3. Program analog blocks
//#
//# (a) Set register page to 1
//#
//w 30 00 01
  data[0]=0x01;
  re|=AUDIO_WRITE(0x00,data,1);
//#
//# (b) Program common-mode voltage (defalut = 1.35 V)
//#
//w 30 1F 04
  data[0]=0x04;
  re|=AUDIO_WRITE(0x1F,data,1);
//#
//# (c) Program headphone-specific depop settings (in case headphone driver is used)
//#
//# De-pop, Power on = 800 ms, Step time = 4 ms
//w 30 21 4E
  data[0]=0x4E;
  re|=AUDIO_WRITE(0x21,data,1);
//#
//# (d) Program routing of DAC output to the output amplifier (headphone/lineout or speaker)
//#
//# LDAC routed to HPL out, RDAC routed to HPR out
//w 30 23 44
  data[0]=0x44;
  re|=AUDIO_WRITE(0x23,data,1);
//#
//# (e) Unmute and set gain of output driver
//#
//# Unmute HPL, set gain = 0 db
//w 30 28 06
  data[0]=0x06;
  re|=AUDIO_WRITE(0x28,data,1);
//# Unmute HPR, set gain = 0 dB
//w 30 29 06
  data[0]=0x06;
  re|=AUDIO_WRITE(0x29,data,1);
//# Unmute Class-D Left, set gain = 18 dB
//w 30 2A 1C
  data[0]=0x1C;
  re|=AUDIO_WRITE(0x2A,data,1);
//# Unmute Class-D Right, set gain = 18 dB
//w 30 2B 1C
  data[0]=0x1C;
  re|=AUDIO_WRITE(0x2B,data,1);
//#
//# (f) Power up output drivers
//#
//# HPL and HPR powered up
//w 30 1F C2
  data[0]=0xC2;
  re|=AUDIO_WRITE(0x1F,data,1);
//# Power-up Class-D drivers
//w 30 20 C6
  data[0]=0xC6;
  re|=AUDIO_WRITE(0x20,data,1);
//# Enable HPL output analog volume, set = -9 dB
//w 30 24 92
  data[0]=0x92;
  re|=AUDIO_WRITE(0x24,data,1);
//# Enable HPR output analog volume, set = -9 dB
//w 30 25 92
  data[0]=0x92;
  re|=AUDIO_WRITE(0x25,data,1);
//# Enable Class-D Left output analog volume, set = -9 dB
//w 30 26 92
  data[0]=0x92;
  re|=AUDIO_WRITE(0x26,data,1);
//# Enable Class-D Right output analog volume, set = -9 dB
//w 30 27 92
  data[0]=0x92;
  re|=AUDIO_WRITE(0x27,data,1);
//#
//# 4. Apply waiting time determined by the de-pop settings and the soft-stepping settings
//# of the driver gain or poll page 1 / register 63
//#
//# 5. Power up DAC
//# (a) Set register page to 0
//#
//w 30 00 00
  data[0]=0x00;
  re|=AUDIO_WRITE(0x00,data,1);
//#
//# (b) Power up DAC channels and set digital gain
//#
//# Powerup DAC left and right channels (soft step enabled)
//w 30 3F D4
  data[0]=0xD4;
  re|=AUDIO_WRITE(0x3F,data,1);
//#
//# DAC Left gain = -22 dB
//w 30 41 D4
  data[0]=0xD4;
  re|=AUDIO_WRITE(0x41,data,1);
//# DAC Right gain = -22 dB
//w 30 42 D4
  data[0]=0xD4;
  re|=AUDIO_WRITE(0x42,data,1);
//#
//# (c) Unmute digital volume control
//#
//# Unmute DAC left and right channels
//w 30 40 00
  data[0]=0x00;
  re|=AUDIO_WRITE(0x40,data,1);

LOG:
  Drv_SERIAL_Log("AUDIO Init %d",re);
#endif  
}
