
#include <stdio.h>
//#include "lcd.h"
#include "Config.h"
#include "FatSpi_Test.h"

//BMP_HEAD bmp;
//BMP_POINT point;
//EXTI_InitTypeDef EXTI_InitStructure;
//GPIO_InitTypeDef GPIO_InitStructure;
//USART_InitTypeDef USART_InitStructure;
//long i;
//u32 y;
//u16 q;

//u32 tx,ty,r_data,g_data,b_data;

 //24位。。变成16位图 
u32 RGB888ToRGB565(u8 r,u8 g,u8 b)
 {return (u32) (r & 0xF8) << 8 | (g & 0xFC) << 3 | (b & 0xF8) >> 3;}    //565



