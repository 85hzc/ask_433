#include "main.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "mbi5153.h"
#include "delay.h"
#include "programs.h"
//#include "FatSpi_Test.h"
#include "config.h"

static FATFS fs;            // Work area (file system object) for logical drive
static FIL fsrc, fdst;      // file objects
static char buffer[1024];   // file copy buffer
static FRESULT res,res1;    // FatFs function common result code
uint8_t currentProgram = 0;

PRO_FILE_S fileName[]= {
    {1,"1.txt"},
    {2,"2.txt"},
    {3,"3.txt"},
    {4,"4.txt"},

};

void readFromTfcard()
{
    //test();

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

    //f_gets();
}


