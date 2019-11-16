#ifndef __ASK_H_
#define __ASK_H_
#include "ALL_Includes.h"

#define ASK PD_ODR_ODR3

#define ASK_SEND_LEN        5

void rf_delay_long();
void rf_delay_short();
void rf_delay_15ms();
void send_one();
void send_zero();
void send_byte(unsigned char da);
void ask_send(unsigned char datt[], unsigned char len);
void Ask_process();
void ReadSelfAddr();
void Write_Coder();

#endif