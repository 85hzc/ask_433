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
char fileBuffer[MAX_FILE_SIZE]={
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
            0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,
            0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,
            0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,
            0,0,0,1,1,1,0,0,1,1,1,0,1,1,1,0,
            0,0,0,1,1,1,0,1,1,1,0,1,1,1,1,0,
            0,0,1,1,1,0,0,1,1,1,0,1,1,1,0,0,
            0,0,1,1,1,0,1,1,1,0,0,1,1,0,0,0,
            0,1,1,1,0,1,1,1,0,0,0,1,1,0,1,0,
            0,1,1,1,0,1,1,0,0,0,0,1,1,1,1,0,
            0,1,1,0,0,0,1,0,0,0,0,0,1,1,1,0,
            0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};   // file copy buffer
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
    UINT a = 1,i;

    //printf("readFromTfcard\r\n");

#if 0
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

    printf("File %s return %d bytes\r\n",fileName[currentProgram%MAX_FILE_NUM].fileName,a);
    for(i=0;i<a;i++)
    {
        if(i%16==0)
            printf("\r\n");
        printf("%x ");
    }
    //f_gets();
#else
    char tmp;
    int j=0;
    for(i=0;i<16;i++)
    {
        tmp = fileBuffer[i*16+15];
        for(j=15;j>0;j--)
        {
            fileBuffer[i*16+j] = fileBuffer[i*16+j-1];
        }

        fileBuffer[i*16] = tmp;
    }
    /*
    for(i=0;i<16;i++)
    {
        printf("\r\n");
        for(j=0;j<16;j++)
            printf("%d ",fileBuffer[i*16+j]);
    }*/
#endif
}


