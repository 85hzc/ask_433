/**
 ******************************************************************************
 * @file    drv_ir.h
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_ir source file
 ******************************************************************************
 */

#ifndef __DRV_IR_H
#define __DRV_IR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Type prototypes -----------------------------------------------------------*/
#define KEY_CHUP      0x45
#define KEY_CH        0x46
#define KEY_CHDN      0x47
#define KEY_PREV      0x44
#define KEY_NEXT      0x40
#define KEY_PLAY      0x43
#define KEY_VOLDN     0x07
#define KEY_VOLUP     0x15
#define KEY_EQ        0x09
#define KEY_0         0x16
#define KEY_100       0x19
#define KEY_200       0x0D
#define KEY_1         0x0C      
#define KEY_2         0x18
#define KEY_3         0x5E
#define KEY_4         0x08
#define KEY_5         0x1C
#define KEY_6         0x5A
#define KEY_7         0x42
#define KEY_8         0x52
#define KEY_9         0x4A

#define REMOTE_POWER  KEY_1
#define REMOTE_UP     0x02
#define REMOTE_DOWN   0x03
#define REMOTE_LEFT   0x04
#define REMOTE_RIGHT  0x05
#define REMOTE_OK     0x06
#define REMOTE_HOME   0x07
#define REMOTE_BACK   0x08
#define REMOTE_MENU   0x09
#define REMOTE_PLUS   0x0A
#define REMOTE_MINUS  0x0B

/* Private function prototypes -----------------------------------------------*/
void Drv_IR_Init(void);
void Drv_IR_Proc(void);

#endif

