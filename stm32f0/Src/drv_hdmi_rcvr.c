/**
 ******************************************************************************
 * @file    drv_i2c_peripheral.c
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_i2c_peripheral source file
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_hdmi_rcvr.h"
#include "drv.h"

#define HDMI_I2C_ADDR    0x90
#define EDID_I2C_ADDR    0xC8
#define EDID_SIZE 256

#define HDMI_WRITE(s,v,n) \
    HAL_I2C_Mem_Write(&hi2c2, HDMI_I2C_ADDR, s, 1, v, n, 1000)
#define HDMI_READ(s,v,n)  \
    HAL_I2C_Mem_Read(&hi2c2, HDMI_I2C_ADDR, s, 1, v, n, 1000)

#define EDID_WRITE(s,v,n) \
    HAL_I2C_Mem_Write(&hi2c2, EDID_I2C_ADDR, s, 1, v, n, 1000)
#define EDID_READ(s,v,n) \
    HAL_I2C_Mem_Read(&hi2c2, EDID_I2C_ADDR, s, 1, v, n, 1000)

/* Private variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c2;

const static uint8_t IT6802_HDMI_INIT_TABLE[][3] = {
  {0x0F,	0x03,	0x00},	//change bank 0
  {0x10,	0xFF,	0x08},	//[3]1: Register reset
  {0x0F,	0x03,	0x00},	//change bank 0
  //{REG_RX_034,	0xFF,	MHL_ADDR+0x01},	//I2C Slave Addresss for MHL block
  {0x10,	0xFF,	0x37},	//[4]Auto Video Reset [2]Int Reset [1]Audio Reset [0]Video Reset
  {0x11,	0xFF,	0x1F},	//Port 0¡G[4]EQ Reset [3]CLKD5 Reset [2]CDR Reset [1]HDCP Reset [0]All logic Reset
  {0x18,	0xFF,	0x1F},	//Port 1¡G[4]EQ Reset [3]CLKD5 Reset [2]CDR Reset [1]HDCP Reset [0]All logic Reset
  {0x12,	0xFF,	0xF8},	//Port 0¡G[7:3] MHL Logic reset
  {0x10,	0xFF,	0x30},	//[4]Auto Video Reset [2]Int Reset [1]Audio Reset [0]Video Reset
  {0x11,	0xFF,	0xA0},	//Port 0¡G[7] Enable Auto Reset when Clock is not stable [5]Enable Auto Reset
  {0x18,	0xFF,	0xA0},	//Port 1¡G[7] Enable Auto Reset when Clock is not stable [5]Enable Auto Reset
  {0x12,	0xFF,	0x00},	//Port 0¡G[7:3] MHL Logic reset
  {0x17,	0xC0,	0x80},	//Port 0¡G[7:6] = 10 invert Port 0 input HCLK , CLKD5I	//2013-0430 Andrew suggestion
  {0x1E,	0xC0,	0x00},	//Port 1¡G[7:6] = 00 invert Port 1 input TMDS , CLKD5I	//2013-0430 Andrew suggestion
  {0x16,	0x08,	0x08},	//Port 0¡G[3]1: Enable CLKD5 auto power down
  {0x1D,	0x08,	0x08},	//Port 1¡G[3]1: Enable CLKD5 auto power down
  {0x2B,	0xFF,	0x07},	//FixTek3D
  //FIX_ID_042 xxxxx //Disable HDCP 1.1 feature to avoid compilance issue from ilegal HDCP 1.1 source device
  {0x31,	0xFF,	0x09},	//[7:4]Enable repeater function [3:0] SCL hold time count & Update Ri sel
  {0x49,	0xFF,	0x09},	// korence marked {0x34 0xFF 0xE1} in programming guide
  //FIX_ID_042 xxxxx
  //FIX_ID_017 xxxxx Disable IPLockChk
  //FIX_ID_001 xxxxx UseIPLock = 0 for avoid clk change
  {0x35,	0x1E,	(0x10+(1<<2))},	//[3:2] RCLKDeltaSel , [1] UseIPLock = 0
  {0x4B,	0x1E,	(0x10+(1<<2))},	//[3:2] RCLKDeltaSel , [1] UseIPLock = 0
  //FIX_ID_001 xxxxx
  //FIX_ID_017 xxxxx
  {0x54,	0xFF,	(1<<4)+1},	//[1:0]RCLK frequency select
  {0x7D, 0xFF, 0x00},   //Disable HW MUTE
  {0x6A,	0xFF,	0x81},			//Decide which kind of packet to be fully recorded on General PKT register
  {0x74,	0xFF,	0xA0},	//[7]Enable i2s and SPDIFoutput [5]Disable false DE output
  {0x50,	0x1F,	0x10|0x02},	//[4]1: Invert output DCLK and DCLK DELAY 2 Step
  {0x65,	0x0C,	0x00},	//[3:2]0=8bits Output color depth
  //	{0x65,	0x0C,	0x04},	//[3:2]1=10bits Output color depth
  //  {0x65,	0x0C,	0x08)},	//[3:2]2=12bits Output color depth
  {0x7A,	0x80,	0x80},	//[7]1: enable audio B Frame Swap Interupt
  //	{REG_RX_02D,	0x03,	0x03},	//[1:0] 11: Enable HDMI/DVI mode over-write
  {0x85,	0x02,	0x02},	//[1]1: gating avmute in video detect module
  //	{REG_RX_051,	0x80,	0x80},	//[7]1: power down color space conversion logic
  //{0xC0,	0x03,	0x00},	//[0]1:Reg_P0DisableShadow  // korence 
  //{0x87,	0xFF,	EDID_I2C_ADDR|0x01},	//[7:1] EDID RAM Slave Adr ,[0]1: Enable access EDID block // korence 
  {0x71,	0x08,	0x00},	//Reg71[3] RegEnPPColMode must clear to 0 for andrew suggestion 2013-0502
  //FIX_ID_030 xxxxx fixed video lost at 640x480 timing
  {0x37,	0xFF,	0xA6},	//Reg37 Reg_P0_WCLKValidNum must set to 0xA6 for andrew suggestion 2014-0403
  {0x4D,	0xFF,	0xA6},	//Reg4D Reg_P1_WCLKValidNum must set to 0xA6 for andrew suggestion 2014-0403
  //FIX_ID_030 xxxxx
  {0x67,	0x80,	0x00},	//Reg67[7] disable HW CSCSel
  {0x7A,  0x50, 0x50},
  //FIX_ID_037 xxxxx //Allion MHL compliance issue debug !!!
  //FIX_ID_018 xxxxx 	modify 1K pull-down to 1.033K ohm HDMI Reg1C0[3:2]=2
  //2014-0526 MHL compliance issue Debug disable ->	{REG_RX_1C0,	0x8C,	0x08},	//[7] PWSB_LV = 0	//2013-0430 Andrew suggestion
  // Reg1C0[3:2] = 00 -> 1.08Kohm	0 %
  // Reg1C0[3:2] = 01 -> 1.18Kohm	+10 %
  // Reg1C0[3:2] = 10 -> 0.98Kohm	-10%
  // Reg1C0[3:2] = 11 -> 0.88Kohm	-20%
  //FIX_ID_018 xxxxx
  {0x77,  0x80, 0x80},	 // IT6801 Audio i2s sck and mclk is common pin
  {0x0F,  0x03, 0x01},	//change bank 1
  {0xC0,  0x8C, 0x88},
  {0x0F,  0x03, 0x00},	//change bank 0
  //FIX_ID_037 xxxxx
  {0x7E,  0x40, 0x40},
  {0x52,  0xFF, 0x20},				//Reg52[5] = 1 for disable Auto video MUTE
  {0x53,  0xFF, 0x00},				//Reg53[7][5] = 01    // for disable B_VDIO_GATTING
  {0x58,  0xFF, 0x33},			// Reg58 for 4Kx2K Video output Driving Strength
  //FIX_ID_053        // Change AIO Strength for some weak connectivity. default value -> 0xAA
  {0x59,  0xFF, 0xAA},			// Reg59 for Audio output Driving Strength
  //RS initial valie
  // 2013/06/06 added by jau-chih.tseng@ite.com.tw
  // Dr. Liu said, reg25/reg3D should set as 0x1F for auto EQ start option.
  {0x25, 0xFF, 0x1F},
  {0x3D, 0xFF, 0x1F},
  //~jau-chih.tseng@ite.com.tw
  {0x27, 0xFF, 0x1F},	// B ch
  {0x28, 0xFF, 0x1F},	// G
  {0x29, 0xFF, 0x1F},	// R
  {0x3F, 0xFF, 0x1F},
  {0x40, 0xFF, 0x1F},
  {0x41, 0xFF, 0x1F},

  {0x0F,	0x03,	0x01},	//change bank 1	//2013-0515 Andrew suggestion	for Auto EQ
  {0xBC,	0xFF,	0x06},	//Reg1BC=0x06		//2013-0515 Andrew suggestion	for Auto EQ
  //FIX_ID_020 xxxxx		//Turn off DEQ for HDMI port 1 with 20m DVI Cable
  {0xCC,	0xFF,	0x00},	//Reg1CC=0x00		for TURN OFF DEQ
  {0xC6,  0x07, 0x03},	// [2:0]Reg_P1_ENHYS = 03 for default enable filter to gating output
  //FIX_ID_020 xxxxx
  {0xB5,	0x03,	0x03},	//Reg1B5[1:0]='11'	for fix Korea K706 MHL pattern Generator	//2013-0515 Andrew suggestion
  //FIX_ID_019	xxxxx modify ENHYS control for MHL mode
  {0xB8,      0x80,      0x00},	// [7] Reg_HWENHYS = 0
  {0xB6,      0x07,      0x03},	// [2:0]Reg_P0_ENHYS = 03 for default enable filter to gating output
  //FIX_ID_019	xxxxx
  //FIX_ID_029	xxxxx fixed Ulta-2000 HDCP fail issue at Receiver mode
  {0x10,      0xFF,      0x00},
  {0x11,      0xFF,      0x00},
  {0x12,      0xFF,      0x00},
  {0x13,      0xFF,      0x00}, // for receiver, BKSV should be zero on inital
  {0x28,      0xFF,      0x00},	// Clear KSV LIST
  {0x29,      0xFF,      0x00},	// Clear KSV LIST
  {0x2A,      0xFF,      0x00},	// Clear KSV LIST
  {0x2B,      0xFF,      0x00},	// Clear KSV LIST
  {0x2C,      0xFF,      0x00},	// Clear KSV LIST
  //FIX_ID_029	xxxxx
  {0x0F,	    0x03,	0x00},	//change bank 0	//2013-0515 Andrew suggestion	for Auto EQ
  //FIX_ID_001 xxxxx Add Auto EQ with Manual EQ
  {0x22,	0xFF,	0x00},	// 07-16 Reg22=0x30	power down auto EQ
  {0x3A,	0xFF,	0x00},	// 07-16 Reg3A=0x30	power down auto EQ
  {0x26,	0xFF,	0x00},	// 07-16 Reg26=0x00 disable Auto Trigger
  {0x3E,	0xFF,	0x00},	// 07-16 Reg3E=0x00 disable Auto Trigger
  //FIX_ID_001 xxxxx
  {0x63,  0xFF, 0x3F},		//for enable interrupt output Pin
  {0x73,  0x08, 0x00},		// for HDCPIntKey = false
  {0x60,  0x40, 0x00},		// disable interrupt mask for NoGenPkt_Rcv
  //FIX_ID_017 xxxxx Disable IPLockChk
  {0x2A, 0x01, 0x00},		// disable PORT 0 EnIPLockChk
  {0x42, 0x01, 0x00},		// disable PORT 1 EnIPLockChk
  //FIX_ID_017 xxxxx
  //FIX_ID_025 xxxxx Audio lock method select for HDMI Repeater / splitter application
  {0x77, 0x0C, 0x08},		// Reg77[3:2] = 01	Audio lock method select
  //FIX_ID_025 xxxxx
  {0xFF, 0xFF, 0xFF},
};
#if 1
const static uint8_t EDID[EDID_SIZE] = {
  0x00,0xff,0xff,0xff,0xff,0xff,0xff,0x00,
  
  0x11,0x90,0x50,0x50,0x01,0x00,0x00,0x00,
  
  0x08,0x13,0x01,0x03,0x80,0x34,0x20,0xa0,
  
  0x22,0xee,0x95,0xa3,0x54,0x4c,0x99,0x26,
  
  0x0f,0x50,0x54,0x21,0x08,0x00,0x01,0x01,
  
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
  
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x1d,
  
  0x00,0x72,0x51,0xd0,0x1e,0x20,0x6e,0x28,
  
  0x55,0x00,0x00,0xd0,0x52,0x00,0x00,0x1e,
  
  0x8c,0x0a,0x56,0x6a,0x30,0xe0,0x5f,0x10,
  
  0x04,0x28,0xff,0x07,0x56,0xe0,0x31,0x00,
  
  0x00,0x1e,0x00,0x00,0x00,0xfc,0x00,0x49,
  
  0x54,0x45,0x36,0x38,0x30,0x31,0x20,0x0a,
  
  0x20,0x20,0x20,0x20,0x00,0x00,0x00,0xfd,
  
  0x00,0x30,0x7a,0x1f,0x41,0x0f,0x00,0x0a,
  
  0x20,0x20,0x20,0x20,0x20,0x20,0x01,0xc0,
  
  0x02,0x03,0x1b,0x41,0x48,0x84,0x02,0x03,
  
  0x11,0x12,0x13,0x2a,0x30,0x23,0x09,0x07,
  
  0x07,0x83,0x01,0x00,0x00,0x65,0x03,0x0c,
  
  0x00,0x10,0x00,0x01,0x1d,0x00,0x72,0x51,
  
  0xd0,0x1e,0x20,0x6e,0x28,0x55,0x00,0x00,
  
  0xd0,0x52,0x00,0x00,0x1e,0x44,0x0c,0x56,
  
  0x36,0x30,0xe0,0x60,0x10,0xea,0x28,0x0f,
  
  0xc3,0x56,0xe0,0x31,0x00,0x00,0x1e,0x47,
  
  0x09,0x80,0x30,0x20,0xe0,0x5f,0x10,0xe4,
  
  0x28,0xff,0xcf,0x80,0xe0,0x21,0x00,0x00,
  
  0x3e,0x9e,0x20,0x00,0x90,0x51,0x20,0x1f,
  
  0x30,0x48,0x80,0x36,0x00,0x00,0x20,0x53,
  
  0x00,0x00,0x1a,0x00,0x00,0x00,0x00,0x00,
  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7c,
};
#else
const static uint8_t EDID[EDID_SIZE] = {
  0x00,0xff,0xff,0xff,0xff,0xff,0xff,0x00,

  0x11,0x90,0x50,0x50,0x11,0x27,0x00,0x00,

  0x04,0x18,0x01,0x04,0xa2,0x34,0x20,0xa0,

  0x26,0x35,0x81,0xa6,0x56,0x48,0x9a,0x24,

  0x12,0x50,0x54,0x00,0x00,0x00,0x01,0x00,

  0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,

  0x01,0x00,0x01,0x01,0x01,0x01,0x01,0x1d,

  0x00,0x72,0x51,0xd0,0x1e,0x20,0x6e,0x28,

  0x55,0x00,0x00,0xd0,0x52,0x00,0x00,0x1e,

  0x01,0x1d,0x00,0x72,0x51,0xd0,0x1e,0x20,

  0x6e,0x28,0x55,0x00,0x00,0xd0,0x52,0x00,

  0x00,0x7f,0x00,0x00,0x00,0xfd,0x00,0x37,

  0x4c,0x1e,0x54,0x11,0x00,0x0a,0x20,0x20,

  0x20,0x20,0x20,0x20,0x00,0x00,0x00,0xfc,

  0x00,0x4d,0x4f,0x4d,0x42,0x36,0x0a,0x20,

  0x20,0x20,0x20,0x20,0x20,0x20,0x00,0x8f,
};
#endif

/* Private function prototypes -----------------------------------------------*/
static uint8_t drv_hdmi_identify_chip(void);
static uint8_t drv_hdmi_init(void);
static uint8_t drv_hdmi_init_edid(void);
static uint8_t drv_hdmi_set_reg(uint8_t reg, uint8_t mask, uint8_t value );
static uint8_t drv_hdmi_get_reg(uint8_t reg, uint8_t mask, uint8_t *value );
static void drv_hdmi_interrupt_handler(void);
static void drv_hdmi_set_hpd(uint8_t value);
//static void drv_hdmi_set_output(uint8_t enable);


/**
  * @brief  The drv_hdmi_rcvr init.
  */
void Drv_HDMI_RCVR_Init(void)
{
  HAL_Delay(100);
  
  if (drv_hdmi_identify_chip())
  {
    Drv_SERIAL_Log("drv_hdmi_identify_chip failed");
    return;
  }
  if (drv_hdmi_init())
  {
    Drv_SERIAL_Log("drv_hdmi_init failed");
    return;
  }
  if (drv_hdmi_init_edid())
  {
    Drv_SERIAL_Log("drv_hdmi_init_edid failed");
    return;
  }
  
  // pull up the HPB signal
  drv_hdmi_set_hpd(1);
}

void Drv_HDMI_RCVR_Proc(void)
{
  drv_hdmi_interrupt_handler();
}

__inline static void change_bank(uint8_t bank)
{
  drv_hdmi_set_reg(0x0F, 0x03, bank&0x03);
}

static void drv_hdmi_set_hpd(uint8_t value)
{
  change_bank(1);
  drv_hdmi_set_reg(0xb0,0x03,value?0x03:0x00);
  change_bank(0);
}
#if 0
static void drv_hdmi_set_output(uint8_t enable)
{  
  if (enable)
  {
    Drv_SERIAL_Log("drv_hdmi_set_output ENALBE");
		drv_hdmi_set_reg(0x64, 0x80, 0x80); // video fifo reset
		drv_hdmi_set_reg(0x64, 0x80, 0x00); // video fifo reset
		HAL_Delay(10);
    
    drv_hdmi_set_reg(0x53,(0x01|0x0E),0x00);
  }
  else
  {    
    Drv_SERIAL_Log("drv_hdmi_set_output DISABLE");
    drv_hdmi_set_reg(0x53,(0x01|0x0E),(0x01|0x0E));
  }
}
#endif

static void drv_hdmi_interrupt_handler(void)
{
#if 0
  unsigned char Reg05h;
  unsigned char Reg06h;
  unsigned char Reg07h;
  unsigned char Reg08h;
  unsigned char Reg09h;
  unsigned char RegD0h;
  unsigned char RegB2h;

  drv_hdmi_set_reg(0x0F, 0x03, 0x00);     // change bank0

  HDMI_READ(0x05, &Reg05h, 1);
  HDMI_READ(0x06, &Reg06h, 1);
  HDMI_READ(0x07, &Reg07h, 1);
  HDMI_READ(0x08, &Reg08h, 1);
  HDMI_READ(0x09, &Reg09h, 1);
  HDMI_READ(0xD0, &RegD0h, 1);
  HDMI_READ(0xB2, &RegB2h, 1);
  
  HDMI_WRITE(0x05, &Reg05h, 1);
  HDMI_WRITE(0x06, &Reg06h, 1);
  HDMI_WRITE(0x07, &Reg07h, 1);
  HDMI_WRITE(0x08, &Reg08h, 1);
  HDMI_WRITE(0x09, &Reg09h, 1);
  HDMI_WRITE(0xD0, &RegD0h, 1);
  HDMI_WRITE(0xB2, &RegB2h, 1);

  if(Reg05h) 
  {
    Drv_SERIAL_Log("Reg05 = 0x%02X",Reg05h);
  }
  if( Reg05h&0x80 ) 
  {
    Drv_SERIAL_Log(("#### Port 0 HDCP Off Detected ###"));
  }
  if( Reg05h&0x40 ) 
  {
    Drv_SERIAL_Log(("#### Port 0 ECC Error 0x%02X ####"));
  }
  if( Reg05h&0x20 ) 
  {
    Drv_SERIAL_Log(("#### Port 0 HDMI/DVI Mode change ####"));
  }
  if( Reg05h&0x08 )
  {
    Drv_SERIAL_Log(("#### Port 0 HDCP Authentication Start ####"));
  }
  if( Reg05h&0x04 )
  {
    Drv_SERIAL_Log(("#### Port 0 Rx Clock Stable Change Detect ####"));
  }
  if( Reg05h&0x02 )
  {
    Drv_SERIAL_Log(("#### Port 0 Rx CKOn Detect ####"));
  }
  if( Reg05h&0x01 ) 
  {
    Drv_SERIAL_Log(("#### Port 0 Power 5V change ####"));
  }
  if(Reg06h)
  {
    Drv_SERIAL_Log("Reg06 = 0x%02X",Reg06h);
  }
  if( Reg06h&0x80 ) 
  {
    Drv_SERIAL_Log(("#### Port 1 HDCP Off Detected ###"));
  }
  if( Reg06h&0x40 ) 
  {
    Drv_SERIAL_Log(("#### Port 1 ECC Error ####"));
  }
  if( Reg06h&0x20 )
  {
    Drv_SERIAL_Log(("#### Port 1 HDMI/DVI Mode change ####"));
  }
  if( Reg06h&0x08 )
  {
    Drv_SERIAL_Log(("#### Port 1 HDCP Authentication Start ####"));
  }
  if( Reg06h&0x10 )
  {
    Drv_SERIAL_Log(("#### Port 1 HDCP Authentication Done ####"));
  }
  if( Reg06h&0x04 )
  {
    Drv_SERIAL_Log(("#### Port 1 Input Clock Change Detect ####"));
  }
  if( Reg06h&0x02 )
  {
    Drv_SERIAL_Log(("#### Port 1 Rx CKOn Detect ####"));
  }
  if( Reg06h&0x01 )
  {
    Drv_SERIAL_Log(("#### Port 1 Power 5V change ####"));
  }
  if( Reg07h)
  {
    Drv_SERIAL_Log("Reg07h = 0x%02X",Reg07h);
  }
  if( Reg07h&0x80 ) 
  {
    Drv_SERIAL_Log(("#### Audio FIFO Error ####"));
  }

  if( Reg07h&0x40 ) 
  {
    Drv_SERIAL_Log(("#### Audio Auto Mute ####"));
  }

  if( Reg07h&0x20 ) 
  {
    Drv_SERIAL_Log(("#### Packet Left Mute ####"));
  }

  if( Reg07h&0x10 ) 
  {
    Drv_SERIAL_Log(("#### Set Mute Packet Received ####"));
  }

  if( Reg07h&0x08 ) 
  {
    Drv_SERIAL_Log(("#### Timer Counter Tntterrupt ####"));
  }

  if( Reg07h&0x04 ) 
  {
    Drv_SERIAL_Log(("#### Video Mode Changed ####"));
  }

  if( Reg07h&0x02 ) 
  {
    Drv_SERIAL_Log("#### Video Stable State Changed ####");
  }

  if( Reg07h&0x01 ) 
  {
    Drv_SERIAL_Log(("#### MHL/HDMI mode changed ####"));      
  }
  if( Reg08h)
  {
    Drv_SERIAL_Log("Reg08h = 0x%02X",Reg08h);
  }
  if( Reg08h&0x80 ) 
  {
    Drv_SERIAL_Log(("#### No General Packet 2 Received ####"));
  }
  if( Reg08h&0x40 ) 
  {
    Drv_SERIAL_Log(("#### No General Packet Received ####"));
  }
  if( Reg08h&0x20 ) 
  {
    Drv_SERIAL_Log(("#### No Audio InfoFrame Received ####"));
  }
  if( Reg08h&0x10) 
  {
    Drv_SERIAL_Log(("#### No AVI InfoFrame Received ####"));
  }
  if( Reg08h&0x08 ) 
  {
    Drv_SERIAL_Log(("#### CD Detect ####"));
  }
  if( Reg08h&0x04 ) 
  {
    Drv_SERIAL_Log(("#### 3D InfoFrame Detect ####"));
  }
  if( Reg08h&0x02 ) 
  {
    Drv_SERIAL_Log(("#### ISRC2 Detect ####"));
  }
  if( Reg08h&0x01 ) 
  {
    Drv_SERIAL_Log(("#### ISRC1 Detect ####"));
  }
  if( Reg09h )
  {
    Drv_SERIAL_Log("Reg09h = 0x%02X",Reg09h);
  }
  if( Reg09h&0x80 )
  {
    Drv_SERIAL_Log(("#### H2V Buffer Skew Fail ####"));
  }
  if( Reg09h&0x40 )
  {
    Drv_SERIAL_Log(("#### Port 1 Deskew Error ####"));
  }
  if( Reg09h&0x20 ) 
  {
    Drv_SERIAL_Log("#### Port 0 Deskew Error ####");
  }
  if( Reg09h&0x10 ) 
  {
    Drv_SERIAL_Log(("#### New Audio Packet Received ####"));
  }
  if( Reg09h&0x08 ) 
  {
    Drv_SERIAL_Log(("#### New ACP Packet Received ####"));
  }
  if( Reg09h&0x04 ) 
  {
    Drv_SERIAL_Log(("#### New SPD Packet Received ####"));
  }
  if( Reg09h&0x02) 
  {
    Drv_SERIAL_Log(("#### New MPEG InfoFrame Received ####"));
  }
  if( Reg09h&0x01) 
  {
    Drv_SERIAL_Log(("#### New AVI InfoFrame Received ####"));
  }
  if( RegD0h )
  {
    Drv_SERIAL_Log("RegD0h = 0x%02X",RegD0h);
  }
  if( RegD0h&0x10 )
  {
    Drv_SERIAL_Log(("#### Port 0 EQ done interrupt ####"));
  }
  if( RegD0h&0x40 )
  {
    Drv_SERIAL_Log(("#### Port 1 EQ done interrupt ####"));
  }
  if( RegD0h&0x20)
  {
    Drv_SERIAL_Log(("#### Port 0 EQ Fail Interrupt ####"));    
  }
  if( RegD0h&0x80)
  {
    Drv_SERIAL_Log(("#### Port 1 EQ Fail Interrupt ####"));
  }
  if( RegB2h )
  {
    Drv_SERIAL_Log("RegB2h = 0x%02X",RegB2h);
  }
#endif  
}

static uint8_t drv_hdmi_identify_chip(void)
{
  uint8_t data[4]={0};
  uint8_t vid[4]={0x54,0x49,0x02,0x68};

  HDMI_READ(0x00,data,4);
  Drv_SERIAL_Log("id %02X,%02X,%02X,%02X",data[0],data[1],data[2],data[3]);
  if (data[0] == vid[0]
      && data[1] == vid[1]
      && data[2] == vid[2]
      && data[3] == vid[3])
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

static uint8_t drv_hdmi_init(void)
{
  uint8_t cnt = 0;
  while (IT6802_HDMI_INIT_TABLE[cnt][0]!= 0xFF)
  {
    if (drv_hdmi_set_reg(
        IT6802_HDMI_INIT_TABLE[cnt][0],
        IT6802_HDMI_INIT_TABLE[cnt][1],
        IT6802_HDMI_INIT_TABLE[cnt][2]))
    {
      Drv_SERIAL_Log("drv_hdmi_init failed at %02X,%02X,%02X", IT6802_HDMI_INIT_TABLE[cnt][0],IT6802_HDMI_INIT_TABLE[cnt][1],IT6802_HDMI_INIT_TABLE[cnt][2]);
      return 1;
    }
    cnt++;
  }
  Drv_SERIAL_Log("drv_hdmi_init %d reg(s) successful", cnt);

  return 0;
}

static uint8_t drv_hdmi_init_edid(void)
{
  uint16_t i;
  uint8_t b, re;

  re=drv_hdmi_set_reg(0xc0,0x03,0x00);
  re|=drv_hdmi_set_reg(0x87,0xff,EDID_I2C_ADDR|0x01);

  for (i=0;i<EDID_SIZE;i++)
  {
    b=EDID[i];
    re|=(uint8_t)EDID_WRITE(i,&b,1);
  }

  re|=drv_hdmi_set_reg(0xc1,0xff,0x99);
  re|=drv_hdmi_set_reg(0xc2,0xff,0x10);
  re|=drv_hdmi_set_reg(0xc3,0xff,0x00);
  re|=drv_hdmi_set_reg(0xc4,0xff,EDID[EDID_SIZE/2-1]);
  re|=drv_hdmi_set_reg(0xc5,0xff,EDID[EDID_SIZE-1]);

  return re;
}

static uint8_t drv_hdmi_get_reg(uint8_t reg, uint8_t mask, uint8_t *value )
{
  if (NULL == value)
  {
    return 1;
  }
  
  if (HAL_OK == HDMI_READ(reg, value, 1))
  {
    uint8_t rsh;
    rsh=(mask%0x80)?((mask%0x40)?((mask%0x20)?((mask%0x10)?((mask%0x08)?((mask%0x04)?((mask%0x02)?0:1):2):3):4):5):6):7;    
    value[0]&=mask;
    value[0]>>=rsh;
    return 0;
  }
  else
  {
    return 1;
  }
}

static uint8_t drv_hdmi_set_reg(uint8_t reg, uint8_t mask, uint8_t value )
{
  uint8_t data;
  if (HAL_OK == HDMI_READ(reg, &data, 1))
  {
    data = (data&((~mask)&0xFF))+(mask&value);
    if (HAL_OK ==  HDMI_WRITE(reg, &data, 1))
    {
      return 0;
    }
  }
  
  return 1;
}

void drv_hdmi_get_p0_status(void)
{  
#if 1
  uint8_t value;

  drv_hdmi_get_reg(0x0A, 0xFF, &value);
  Drv_SERIAL_Log("Input status(0A): %02X", value);
  drv_hdmi_get_reg(0x0B, 0xFF, &value);
  Drv_SERIAL_Log("Input status(0B): %02X", value);
  drv_hdmi_get_reg(0x51, 0xFF, &value);
  Drv_SERIAL_Log("Input port(51): %02X", value);
  drv_hdmi_get_reg(0x71, 0xFF, &value);
  Drv_SERIAL_Log("SCDT_Chg mode(71): %02X", value);
  drv_hdmi_set_reg(0x0F, 0x03, 0x01);  
  drv_hdmi_get_reg(0xB0, 0xFF, &value);
  drv_hdmi_set_reg(0x0F, 0x03, 0x00);
  Drv_SERIAL_Log("HPD(1B0): %02X", value);  
  drv_hdmi_get_reg(0x5B, 0xFF, &value);     // get ENVBUS
  Drv_SERIAL_Log("ENVBUS Ctrl(5B): %02X", value);
  drv_hdmi_get_reg(0x10, 0xFF, &value);
  Drv_SERIAL_Log("ENVBUS(10): %02X", value);
  drv_hdmi_get_reg(0x91, 0xFF, &value);
  Drv_SERIAL_Log("P0_TMDSCLKSpeed(91): %02X", value);  
  change_bank(1);
  HDMI_READ(0xD4, &value, 1);
  change_bank(0);  
  Drv_SERIAL_Log("0xD4: %02X", value);    
  drv_hdmi_get_reg(0x99, 0xFF, &value);
  Drv_SERIAL_Log("0x99: %02X", value);    
  drv_hdmi_get_reg(0x9A, 0xFF, &value);
  Drv_SERIAL_Log("0x9A: %02X", value);  
  Drv_SERIAL_Log("EDID Regs:");
  drv_hdmi_get_reg(0xC0, 0xFF, &value);
  Drv_SERIAL_Log("EDID Regs C0: %02X", value);
  drv_hdmi_get_reg(0xC1, 0xFF, &value);
  Drv_SERIAL_Log("EDID Regs C1: %02X", value);
  drv_hdmi_get_reg(0xC2, 0xFF, &value);
  Drv_SERIAL_Log("EDID Regs C2: %02X", value);
  drv_hdmi_get_reg(0xC3, 0xFF, &value);
  Drv_SERIAL_Log("EDID Regs C3: %02X", value);
  drv_hdmi_get_reg(0xC4, 0xFF, &value);
  Drv_SERIAL_Log("EDID Regs C4: %02X", value);
  drv_hdmi_get_reg(0xC5, 0xFF, &value);
  Drv_SERIAL_Log("EDID Regs C5: %02X", value);
  drv_hdmi_get_reg(0xC6, 0xFF, &value);
  Drv_SERIAL_Log("EDID Regs C6: %02X", value);
  drv_hdmi_get_reg(0xC7, 0xFF, &value);
  Drv_SERIAL_Log("EDID Regs C7: %02X", value);
  drv_hdmi_get_reg(0xC8, 0xFF, &value);
  Drv_SERIAL_Log("EDID Regs C8: %02X", value);  
  drv_hdmi_get_reg(0xC9, 0xFF, &value);
  Drv_SERIAL_Log("EDID Regs C9: %02X", value);
  drv_hdmi_get_reg(0xD1, 0xFF, &value);
  Drv_SERIAL_Log("EDID Regs D1: %02X", value);
// ============== AUDIO ======================  
  drv_hdmi_get_reg(0xAA, 0xFF, &value);  //REG_RX_AUDIO_CH_STAT  
  Drv_SERIAL_Log("0xAA: %02X", value);  
  drv_hdmi_get_reg(0xAB, 0xFF, &value);  // REG_RX_AUD_CHSTAT3
  Drv_SERIAL_Log("0xAB: %02X", value);  
  drv_hdmi_get_reg(0xAC, 0xFF, &value);  //REG_RX_AUDIO_CH_STAT  
  Drv_SERIAL_Log("0xAC: %02X", value);  
  drv_hdmi_get_reg(0xAD, 0xFF, &value);  //REG_RX_AUDIO_CH_STAT  
  Drv_SERIAL_Log("0xAD: %02X", value);  
  drv_hdmi_get_reg(0xAE, 0xFF, &value);  // REG_RX_AUD_CHSTAT3
  Drv_SERIAL_Log("0xAE Audio_CH_Status[31:24]: %02X", value);  
  drv_hdmi_get_reg(0xAF, 0xFF, &value);  // REG_RX_AUD_CHSTAT3
  Drv_SERIAL_Log("0xAF Audio_CH_Status[39:32]: %02X", value);  
  drv_hdmi_get_reg(0x7B, 0xFF, &value);  // REG_RX_AUD_CHSTAT3
#endif
}

