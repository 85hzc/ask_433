#ifndef  _LED_H_
#define  _LED_H_
#include <iostm8s103f3.h>
//#define  LEDPort  GPIOA
//#define  LEDPin   (1 << 2) 

#define         LED_L1    PC_ODR_ODR6


enum
{
    KEY_NULL = 0,
    LIGHT_ON,
    LIGHT_OFF,
    LIGHT_UP,
    LIGHT_DOWN,
    SCENE_1,
    SCENE_2,
    SCENE_3,
    SCENE_4,
};

void Led_Init(void);
void Led_on(unsigned char m);
void Led_off(unsigned char m);
void Led_on_all();
void Led_off_all();


#endif
