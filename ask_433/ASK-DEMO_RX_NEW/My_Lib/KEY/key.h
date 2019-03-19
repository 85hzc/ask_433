#ifndef         __KEY_H_
#define         __KEY_H_

#include <iostm8s103f3.h>

#define         SW_K1    PC_IDR_IDR7

void Key_Init();
unsigned char key_scan();
#endif