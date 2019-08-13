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
static FIL fsrc;      // file objects
char fileBuffer[MAX_FILE_SIZE];   // file copy buffer
static FRESULT res;    // FatFs function common result code
uint8_t currentProgram = 0;

PRO_FILE_S fileName[MAX_FILE_NUM]= {
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
    res = f_open(&fsrc, fileName[currentProgram%MAX_FILE_NUM].fileName, FA_CREATE_ALWAYS | FA_WRITE);
    if((res) != 0)
    {
        if(res == 0)
        f_close(&fsrc);
        printf("file system test error\r\n");
        return;
    }
    f_read(&fsrc, fileBuffer, MAX_FILE_SIZE, &a);      //从opplem.txt中读取1024个字节
    f_close(&fsrc);

    //f_gets();
}


