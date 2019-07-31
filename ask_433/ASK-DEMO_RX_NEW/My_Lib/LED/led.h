#ifndef  _LED_H_
#define  _LED_H_
#include <iostm8s103f3.h>
//#define  LEDPort  GPIOA
//#define  LEDPin   (1 << 2) 

#define         LED_L1    PC_ODR_ODR6


#define         LIGHT_ON    0x01
#define         LIGHT_OFF   0x02
#define         LIGHT_UP    0x03
#define         LIGHT_DOWN  0x04


void Led_Init(void);
void Led_on(unsigned char m);
void Led_off(unsigned char m);
void Led_on_all();
void Led_off_all();


#endif
