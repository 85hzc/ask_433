
#include <stdio.h>
//#include "lcd.h"
#include "Config.h"
#include "FatSpi_Test.h"

static vu32 TimingDelay;

BMP_HEAD bmp;

BMP_POINT point;

//EXTI_InitTypeDef EXTI_InitStructure;
//GPIO_InitTypeDef GPIO_InitStructure;
//USART_InitTypeDef USART_InitStructure;
long i;
u32 y;
u16 q;

FATFS fs;            // Work area (file system object) for logical drive
FIL fsrc, fdst;      // file objects
char buffer[1024];   // file copy buffer
FRESULT res,res1;    // FatFs function common result code
UINT br, bw;         // File R/W count
u32 tx,ty,r_data,g_data,b_data; 

FIL file;
ErrorStatus HSEStartUpStatus;

 //24位。。变成16位图 
u32 RGB888ToRGB565(u8 r,u8 g,u8 b)
 {return (u32) (r & 0xF8) << 8 | (g & 0xFC) << 3 | (b & 0xF8) >> 3;}    //565

void test(void)
{
    UINT a = 1;
    
    f_mount(0, &fs);
    res1 = f_open(&fdst, "huangzhicheng.txt", FA_CREATE_ALWAYS | FA_WRITE);
    res = f_open(&fsrc, "opplem.txt", FA_OPEN_EXISTING | FA_READ);
    if((res1 || res) != 0)
    {
        if(res == 0)
        f_close(&fsrc);
        if(res1 == 0)
        f_close(&fdst);

        printf("file system test error\r\n");
        return;
    }

    f_read(&fsrc, buffer,1024, &a);      //从opplem.txt中读取1024个字节
    f_write(&fdst, buffer,1024, &a);     //huangzhicheng.txt中写人1024个字节      
    
    f_close(&fsrc);
    f_close(&fdst);
}


