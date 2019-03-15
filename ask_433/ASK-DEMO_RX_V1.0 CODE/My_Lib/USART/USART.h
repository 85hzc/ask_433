/*************************************
���о����
              stm8s105
               V1.1
             2013.9.8
**************************************/

#ifndef  _USART_H
#define  _USART_H

#include "ALL_Includes.h"

//����UART��TX��RX����
#define  TXPort  PC
#define  TXPin   (1 << 5)
#define  RXPort  PC
#define  RXPin   (1 << 6)

void Uart_Init(u8 SYS_Clk, u32 baud);
void Uart_Sendbyte(unsigned char ch);
void Uart_Senddata(unsigned char* st, unsigned char len);
void UART_IOConfig(void);


#endif