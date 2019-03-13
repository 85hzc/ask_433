#ifndef  _LED_H_
#define  _LED_H_
#include <iostm8s103f3.h>
//#define  LEDPort  GPIOA
//#define  LEDPin   (1 << 2) 

//#define DEMO_EVB

#ifdef DEMO_EVB
#define         LED_L1    PA_ODR_ODR3
#define         LED_L2    PB_ODR_ODR4
#define         LED_L3    PB_ODR_ODR5
#define         LED_L4    PC_ODR_ODR3
#else
#define         LED_L5    PD_ODR_ODR2
#define         LED_L6    PC_ODR_ODR7
#endif

void Led_Init(void);
void Led_on(unsigned char m);
void Led_off(unsigned char m);
void Led_on_all();
void Led_off_all();


#endif
