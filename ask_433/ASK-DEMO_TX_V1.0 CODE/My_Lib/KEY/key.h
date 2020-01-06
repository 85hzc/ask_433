#ifndef         __KEY_H_
#define         __KEY_H_

#include <iostm8s103f3.h>

#define         SW_K1    PD_IDR_IDR4
#define         SW_K2    PA_IDR_IDR1
#define         SW_K3    PC_IDR_IDR5
#define         SW_K4    PA_IDR_IDR2

#define         SW_K5    PA_IDR_IDR3
#define         SW_K6    PC_IDR_IDR6
#define         SW_K7    PC_IDR_IDR3
#define         SW_K8    PC_IDR_IDR4

#define         SW_SENSOR  SW_K3

#ifdef THERMOELECTRIC_SENSOR
#define         KEY_DELAY 50
#else
#define         KEY_DELAY 20
#endif

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

void Key_Init();
unsigned char key_scan();
#endif
