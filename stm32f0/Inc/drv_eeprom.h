/**
 ******************************************************************************
 * @file    drv_eeprom.c
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_i2c_peripheral source file
 ******************************************************************************
 */

#ifndef __DRV_EEPROM_H
#define __DRV_EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Type prototypes -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

void Drv_EEPROM_Init(void);
void Drv_EEPROM_Proc(void);

uint8_t drv_eeprom_read_edid(void);
uint8_t drv_eeprom_write_edid(void);

#endif


