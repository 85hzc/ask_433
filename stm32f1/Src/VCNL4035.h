#ifndef __VCNL4035_H
#define __VCNL4035_H

#include "main.h"

#define VCNL4035_COMM_W        		0xC0        		//VCNL4035的I2C地址,写
#define VCNL4035_COMM_R        		0xC1        		//VCNL4035的I2C地址,读    

#define VCNL4035_ALS_CONF			0x00
#define VCNL4035_ALS_THDH			0x01
#define VCNL4035_ALS_THDL			0x02
#define VCNL4035_PS_CONF1_2			0x03
#define VCNL4035_PS_CONF3_MS		0x04
#define VCNL4035_PS_CANC			0x05
#define VCNL4035_PS_THDL			0x06
#define VCNL4035_PS_THDH			0x07
#define VCNL4035_PS1_DATA			0x08
#define VCNL4035_PS2_DATA			0x09
#define VCNL4035_PS3_DATA			0x0A
#define VCNL4035_ALS_DATA			0x0B
#define VCNL4035_White_DATA			0x0C
#define VCNL4035_INT_FLAG			0x0D
#define VCNL4035_ID					0x0E

#define VCNL4035_PS_CONF1_2_DEF		0xC804		//PS_IT=2T,使能PS,GESTURE_INT_EN=1,GESTURE_MODE=1,PS_HD：16bit
//#define VCNL4035_PS_CONF3_MS_DEF	0x0708		//PS_AF=1,LED电流：200mA
//#define VCNL4035_PS_CONF3_MS_DEF	0x0208		//PS_AF=1,LED电流：100mA
#define VCNL4035_PS_CONF3_MS_DEF	0x0008		//PS_AF=1,LED电流：50mA
#define VCNL4035_PS_TRIG_MASK		0x0004

uint8_t VCNL4035_SendData(uint8_t command, uint16_t val);
uint8_t VCNL4035_ReadData(uint8_t command, uint16_t *pval);
uint8_t VCNL4035_ReadPS(uint16_t *ps);
void VCNL4035_InitGesMode(void);
void VCNL4035_StartGesDetect(void);
void VCNL4035_ClearInt(void);

#endif  //__VCNL4035_H
