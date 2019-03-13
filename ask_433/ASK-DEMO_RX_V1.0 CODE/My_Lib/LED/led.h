#ifndef  _LED_H_
#define  _LED_H_
#include <iostm8s103f3.h>
//#define  LEDPort  GPIOA
//#define  LEDPin   (1 << 2) 

#define         LED_L1    PC_ODR_ODR3
#define         LED_L2    PA_ODR_ODR1
#define         LED_L3    PA_ODR_ODR2
#define         LED_L4    PA_ODR_ODR3

void Led_Init(void);
void Led_on(unsigned char m);
void Led_off(unsigned char m);
void Led_on_all();
void Led_off_all();


#endif
