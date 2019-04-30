#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx.h"

#define delay(a) Delay_us(a)
#define Delay_ms(a) Delay_us(a*1000)
//#define delay(a) delay_ns(a)

void Delay_Init(void);				//延时函数初始化
void Delay_us(u32 nTime);	//毫秒延时函数

#endif //__DELAY_H

