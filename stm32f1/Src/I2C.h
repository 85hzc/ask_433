#ifndef __I2C_H
#define __I2C_H

#include "main.h"

void I2C_set_scl_state(uint8_t flag);
uint8_t I2C_get_sda_state(void);
void I2C_set_sda_state(uint8_t flag);
void I2C_init(void);
void I2C_start(void);
void I2C_stop(void);
uint8_t I2C_waitAck(void);
void I2C_sendAck(void);
void I2C_sendNoAck(void);
void I2C_sendByte(uint8_t byte);
uint8_t I2C_recvByte(void);
void I2C_delay_1us(uint16_t delay);

#endif  //__I2C_H
