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
//#include "stm32f1xx.h"

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

#define REMOTE_NEC_POWER  KEY_1
#define REMOTE_NEC_UP     KEY_CHUP
#define REMOTE_NEC_DOWN   KEY_CHDN
#define REMOTE_NEC_LEFT   0x04
#define REMOTE_NEC_RIGHT  0x05
#define REMOTE_NEC_OK     KEY_PLAY
#define REMOTE_NEC_HOME   0x07
#define REMOTE_NEC_BACK   KEY_PREV
#define REMOTE_NEC_MENU   0x09
#define REMOTE_NEC_PLUS   KEY_VOLUP
#define REMOTE_NEC_MINUS  KEY_VOLDN


//MI IR controler
#define MI_POWER      0x6AC0001
#define MI_UP         0x4000213
#define MI_DOWN       0x313
#define MI_LEFT       0x1000093
#define MI_RIGHT      0x800413
#define MI_ENTER      0x4800013
#define MI_HOME       0x493
#define MI_BACK       0x1000213
#define MI_MENU       0x613
#define MI_VOLPLUS    0x800113
#define MI_VOLMINUS   0x1800013

#define REMOTE_MI_POWER  0xFF01
#define REMOTE_MI_UP     0xFF02
#define REMOTE_MI_DOWN   0xFF03
#define REMOTE_MI_LEFT   0xFF04
#define REMOTE_MI_RIGHT  0xFF05
#define REMOTE_MI_OK     0xFF06
#define REMOTE_MI_HOME   0xFF07
#define REMOTE_MI_BACK   0xFF08
#define REMOTE_MI_MENU   0xFF09
#define REMOTE_MI_PLUS   0xFF0A
#define REMOTE_MI_MINUS  0xFF0B

/* Private function prototypes -----------------------------------------------*/
void Drv_IR_Init(void);
void Drv_IR_Proc(void);

#endif

