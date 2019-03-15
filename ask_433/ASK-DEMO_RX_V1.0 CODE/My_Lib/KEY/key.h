#ifndef         __KEY_H_
#define         __KEY_H_

#include "ALL_Includes.h"

#define         SW_K1    PB_IDR_IDR3

void Key_Init();
unsigned char key_scan();
#endif