/**
 ******************************************************************************
 * @file    drv_serial.h
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_i2c_peripheral source file
 ******************************************************************************
 */

#ifndef __DRV_SERIAL_H
#define __DRV_SERIAL_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

//#define CMD_LEN_HEADER  1
//#define CMD_LEN_CODE    1
//#define CMD_LEN_PARAM   2
//#define CMD_LEN_CRC     1
#define CMD_LEN_MAX     5

#define CMD_HEADER_REQ        0x4A
#define CMD_HEADER_REQ_NR     0x4B
#define CMD_HEADER_RSP        0x6D
#define CMD_HEADER_RPT        0x55

#define CMD_CODE_MASK         0xF0
#define CMD_CODE_MASK_IR      0x01
#define CMD_CODE_MASK_AU      0x02
#define CMD_CODE_MASK_HDMI    0x03
#define CMD_CODE_MASK_DLPC    0x04
#define CMD_CODE_MASK_ACC     0x05
#define CMD_CODE_MASK_THERM   0x06
#define CMD_CODE_MASK_MOTOR   0x07
#define CMD_CODE_MASK_FAN     0x08

#define SET_CODE(m,c)         ((uint8_t)(m<<4|c))
#define GET_MASK(m)           ((m&CMD_CODE_MASK)>>4)
#define GET_OP(m)             (m&~CMD_CODE_MASK)

typedef enum
{
  CMD_OP_DLPC_SET_INPUT                     = 0x00,
  CMD_OP_DLPC_SET_ORIENT                    = 0x01,
  CMD_OP_DLPC_SET_OUPUT_CTRL                = 0x02,
  CMD_OP_DLPC_SET_PROJ_ON                   = 0x03,
  CMD_OP_DLPC_SET_KEYSTONE_CTRL             = 0x04,
  CMD_OP_DLPC_SET_KEYSTONE_ANGLE            = 0x05,
  CMD_OP_DLPC_DEBUG_SET_LED_CURRENT_UP      = 0x06,
  CMD_OP_DLPC_DEBUG_SET_LED_CURRENT_DN      = 0x07,
  CMD_OP_DLPC_DEBUG_SET_LED_CURRENT_SAVE    = 0x08,
  CMD_OP_DLPC_DEBUG_GET_SHORT_STATUS        = 0x09,
  CMD_OP_DLPC_DEBUG_SWITCH_TEST_PATTERN     = 0x0A,
  CMD_OP_DLPC_DEBUG_SW                      = 0x0B,
  CMD_OP_DLPC_SET_CURRENT                   = 0x0C,
}dlpc_op_t;
typedef enum
{
  CMD_OP_THERM_GET_VALUE                    = 0x00,
}therm_op_t;
typedef enum
{
  CMD_OP_MOTOR_SET_RESET                    = 0x00,
  CMD_OP_MOTOR_SET_FORWARD                  = 0x01,
  CMD_OP_MOTOR_SET_BACKWARD                 = 0x02,
}motor_op_t;
typedef enum
{
  CMD_OP_IR_CODE                            = 0x00
}ir_op_t;
typedef enum
{
  CMD_OP_HDMI_GET_STATUS                    = 0x00,
}hdmi_op_t;
typedef enum
{
  CMD_OP_FAN_SPEED                          = 0x00,
  CMD_OP_FAN_ON                             = 0x01,
  CMD_OP_FAN_OFF                            = 0x02,
  CMD_OP_PWM_RGB                            = 0x03,
}fan_op_t;
typedef enum
{
  CMD_OP_EEPROM_READ_EDID                   = 0x00,
  CMD_OP_EEPROM_WRITE_EDID                  = 0x01,
}eeprom_op_t;
typedef enum
{
  CMD_OP_GYRO_CODE                          = 0x00,
}gyro_op_t;

/* Private function prototypes -----------------------------------------------*/
void Drv_SERIAL_Init(void);
void Drv_SERIAL_Proc(void);
void Drv_SERIAL_Log(const char *format, ...);
uint8_t Drv_SERIAL_Rpt(uint8_t code, uint16_t param);
uint8_t Drv_SERIAL_Act(uint8_t code, uint16_t param);

#endif


