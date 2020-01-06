#ifndef  _ALL_Includes_H
#define  _ALL_Includes_H

//#include "stm8s.h"
#include <iostm8s103f3.h>
#include "mytype.h"
#include "delay.h"
#include "led.h"
#include "key.h"
#include "USART.h"
#include "time.h"
#include "eeprom.h"
#include <string.h>
#include <intrinsics.h>

///////user////////////
#include "Ask.h"
#ifdef PROJECT_SPOTLIGHT
#include "motor.h"
#endif

#define PWM_SET     100   //16000

#endif

