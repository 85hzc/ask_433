#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx.h"

#define Delay_ms(a) Delay_us(a*1000)
#define delay(a) Delay_us(a)

void Delay_Init(void);				//��ʱ������ʼ��
void Delay_us(u32 nTime);	//������ʱ����

#endif //__DELAY_H

