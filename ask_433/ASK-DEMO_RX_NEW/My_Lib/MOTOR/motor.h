#ifndef  _MOTOR_H_
#define  _MOTOR_H_
#include <iostm8s103f3.h>


#define         MOTOR_A_P    PC_ODR_ODR6
#define         MOTOR_A_N    PC_ODR_ODR6

#define         MOTOR_B_P    PC_ODR_ODR6
#define         MOTOR_B_N    PC_ODR_ODR6


typedef enum
{
  CMD_OP_MOTOR_SET_RESET        = 0x00,
  CMD_OP_MOTOR_SET_FORWARD      = 0x01,
  CMD_OP_MOTOR_SET_BACKWARD     = 0x02,
}motor_op_t;

#endif
