#ifndef __ASK_H_
#define __ASK_H_
#include "ALL_Includes.h"

#define ASK PD_ODR_ODR3

#if 0
#if 0 //Å·¶¥µÆÍ¸´«
#define ODD
#define ASK_SEND_LEN        4
#define NETID               0xAA
#else //µ÷½¹µÆ
#define TJD
#define ASK_SEND_LEN        3
#define NETID               0xBB
#endif
#else
#define ASK_SEND_LEN        3
#define READ_REALTIME
//#define THERMOELECTRIC_SENSOR
#endif


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
