/**
 ******************************************************************************
 * @file    drv_ir.h
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_ir source file
 ******************************************************************************
 */

#ifndef __DRV_SERIAL_H
#define __DRV_SERIAL_H

#define CMD_LEN_MAX             5
#define MULTI_CMD_MAX           20

#define CMD_HEADER_REQ          0x4A
#define CMD_HEADER_REQ_NR       0x4B


#define SET_CODE(m,c)           ((uint8_t)(m<<5|c))
#define GET_MASK(m)             ((m&CMD_CODE_MASK)>>5)
#define GET_OP(m)               (m&~CMD_CODE_MASK)

#define ARR_SIZE(a)             (sizeof(a)/sizeof(a[0]))

#define CMD_CODE_MASK           0xE0
#define CMD_CODE_MASK_IR        0x01
#define CMD_CODE_MASK_MOTOR     0x02

typedef int8_t (*hdlr_func)(uint8_t code, uint16_t param);
typedef struct {
  uint8_t cmd_mask;
  hdlr_func handler;
}cmd_table_t;

typedef enum
{
  CMD_OP_MOTOR_SET_RESET        = 0x00,
  CMD_OP_MOTOR_SET_FORWARD      = 0x01,
  CMD_OP_MOTOR_SET_BACKWARD     = 0x02,
}motor_op_t;

typedef enum
{
  CMD_OP_IR_CODE                = 0x00
}ir_op_t;

typedef struct {
  uint8_t wr_id;
  uint8_t rd_id;
  uint8_t multi_cmd[MULTI_CMD_MAX][CMD_LEN_MAX];
} ACT_CMD;


#endif

