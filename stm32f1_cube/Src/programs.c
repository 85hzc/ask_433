#include "main.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "mbi5153.h"
#include "delay.h"
#include "programs.h"
//#include "FatSpi_Test.h"
#include "config.h"

extern uint16_t                 fileTotalFilm;
extern uint16_t                 fileTotalPhoto;
extern uint8_t                  filmTotalProgram;

static FATFS                    fs;            // Work area (file system object) for logical drive
static FIL                      fsrc, fdst;      // file objects
BYTE                            photo_filename[MAX_FILE_NUM][FILE_NAME_LEN];
BYTE                            film_filename[MAX_FILM_FRAME][FILE_NAME_LEN];
BYTE                            film_foldername[MAX_FILM_FOLDER][FILE_NAME_LEN];

#if(PROJECTOR_OSRAM)
uint8_t                         osram_buff[MATRIX_SIZE][MATRIX_SIZE];
char                            fileBuffer[MAX_FILE_SIZE];
#elif(PROJECTOR_WS2801)
uint8_t                         cube_buff_G[CUBE_ROW_SIZE][CUBE_COL_SIZE*CUBE_PAGE_SIZE];
uint8_t                         cube_buff_R[CUBE_ROW_SIZE][CUBE_COL_SIZE*CUBE_PAGE_SIZE];
uint8_t                         cube_buff_B[CUBE_ROW_SIZE][CUBE_COL_SIZE*CUBE_PAGE_SIZE];
char                            fileBuffer[MAX_FILE_SIZE];
#endif
uint8_t                         photoIdx = 0;
uint8_t                         filmFrameIdx = 0;
uint8_t                         filmProgramIdx = 0;

PROGRAMS_TYPE_E                 programsType;
/*
PRO_FILE_S fileName[MAX_FILE_NUM]= {
    {1,"1.txt"},
    {2,"2.txt"},
    {3,"3.txt"},
    {4,"4.txt"},

};
*/

FRESULT SD_ReadPhotoData()
{
    FRESULT res;
    UINT a = 1,i,j;
    uint8_t path[FILE_PATH_LEN];

    memset(path, 0, sizeof(path));

#if(PROJECTOR_OSRAM)
    sprintf(path,"/OSRAM/Photo/%s",photo_filename[photoIdx%fileTotalPhoto]);
#elif(PROJECTOR_WS2801)
    sprintf(path,"/WS2801/Photo/%s",photo_filename[photoIdx%fileTotalPhoto]);
#endif

    f_mount(0, &fs);
    res = f_open(&fsrc, path, FA_OPEN_EXISTING | FA_READ);
    if(res != 0)
    {
        printf("open [%s] error[%d]!\r\n",photo_filename[photoIdx%fileTotalPhoto],res);
        return res;
    }
    f_read(&fsrc, fileBuffer, MAX_FILE_SIZE, &a);
    f_close(&fsrc);
    printf("[%s] return %d bytes\r\n",photo_filename[photoIdx%fileTotalPhoto],a);

#if(PROJECTOR_OSRAM)
    for( i=0; i<MATRIX_SIZE; i++ )
    {
        for( j=0; j<MATRIX_SIZE; j++ )
        {
            osram_buff[i][j] = fileBuffer[i*(64+2)+j*2]-'0';
        }
    }
#elif(PROJECTOR_WS2801)
    for( i=0; i<CUBE_ROW_SIZE; i++ )
    {
        for( j=0; j<CUBE_COL_SIZE*CUBE_PAGE_SIZE; j++ )
        {
            cube_buff_G[i][j] = fileBuffer[i*36+j*3];
            cube_buff_R[i][j] = fileBuffer[i*36+j*3+1];
            cube_buff_B[i][j] = fileBuffer[i*36+j*3+2];
        }
    }
#endif

    return FR_OK;
}

FRESULT SD_ReadFilmData()
{
    FRESULT res;
    UINT a = 1,i,j;
    uint8_t path[FILE_PATH_LEN];

    memset(path, 0, sizeof(path));

#if(PROJECTOR_OSRAM)
    sprintf(path,"/OSRAM/Film/%s/%s",film_foldername[filmProgramIdx%filmTotalProgram],film_filename[filmFrameIdx%fileTotalFilm]);
#elif(PROJECTOR_WS2801)
    sprintf(path,"/WS2801/Film/%s/%s",film_foldername[filmProgramIdx%filmTotalProgram],film_filename[filmFrameIdx%fileTotalFilm]);
#endif

    printf("path:%s\r\n",path);
    f_mount(0, &fs);
    res = f_open(&fsrc, path, FA_OPEN_EXISTING | FA_READ);
    if(res != 0)
    {
        printf("open [%s/%s] error[%d]!\r\n",
                film_foldername[filmProgramIdx%filmTotalProgram],film_filename[filmFrameIdx%fileTotalFilm],res);
        return res;
    }
    f_read(&fsrc, fileBuffer, MAX_FILE_SIZE, &a);
    f_close(&fsrc);

#if(PROJECTOR_OSRAM)
    for( i=0; i<MATRIX_SIZE; i++ )
    {
        for( j=0; j<MATRIX_SIZE; j++ )
        {
            osram_buff[i][j] = fileBuffer[i*(64+2)+j*2]-'0';
        }
    }
#elif(PROJECTOR_WS2801)
    for( i=0; i<CUBE_ROW_SIZE; i++ )
    {
        for( j=0; j<CUBE_COL_SIZE*CUBE_PAGE_SIZE; j++ )
        {
            cube_buff_G[i][j] = fileBuffer[i*36+j*3];
            cube_buff_R[i][j] = fileBuffer[i*36+j*3+1];
            cube_buff_B[i][j] = fileBuffer[i*36+j*3+2];
        }
    }
#endif

    return FR_OK;
}

/*********************************************************************************************************
** Functoin name:       SD_ReadSDFiles
** Descriptions:        读取SD卡的文件
** input paraments:     无
** output paraments:    无    
** Returned values:     无
*********************************************************************************************************/
void SD_ReadPhotoFileList(char *path)
{
    FILINFO finfo;
    DIR dirs;
    int i_name=0, br;
    FRESULT res;

    /*挂载文件系统*/
    f_mount(0, &fs);
    res =  f_opendir(&dirs, path);
    if (res == FR_OK)
    {
        printf("photo files:\r\n");

        while (f_readdir(&dirs, &finfo) == FR_OK)
        {
            if (finfo.fattrib & AM_ARC)
            {
                if(!finfo.fname[0]){      //文件名不为空，如果为空，则表明该目录下面的文件已经读完了
                    break;
                }
                stringcopy(photo_filename[i_name], (BYTE*)finfo.fname);
                printf("          [%d]:%s\r\n",i_name,photo_filename[i_name]);
                i_name++;
            }
        }
        fileTotalPhoto = i_name;
    }
    else
    {
        printf("open path %s failed.\r\n",path);
    }
}

void SD_ReadFilmFileList(char filmId)
{
    FILINFO finfo;
    DIR dirs;
    int i_name=0, br;
    FRESULT res;
    char path[FILE_PATH_LEN];

    memset(path, 0, sizeof(path));
    sprintf(path,"/OSRAM/Film/%s",film_foldername[filmId]);

    /*挂载文件系统*/
    f_mount(0, &fs);
    res =  f_opendir(&dirs, path);
    if (res == FR_OK)
    {
        printf("film[%s] files:\r\n",film_foldername[filmId]);
    
        while (f_readdir(&dirs, &finfo) == FR_OK)
        {
            if (finfo.fattrib & AM_ARC)
            {
                //文件名不为空，如果为空，则表明该目录下面的文件已经读完了
                if(!finfo.fname[0])
                {
                    break;
                }
                stringcopy(film_filename[i_name], (BYTE*)finfo.fname);
                printf("          [%d]:%s\r\n",i_name,film_filename[i_name]);
                i_name++;
            }
        }
        fileTotalFilm = i_name;
    }
    else
    {
        printf("open path %s failed.\r\n",path);
    }
}

void SD_ReadFilmFolderList(char *path)
{
    FILINFO finfo;
    DIR dirs;
    int i_name=0, br;
    FRESULT res;

    /*挂载文件系统*/
    f_mount(0, &fs);
    res =  f_opendir(&dirs, path);
    if (res == FR_OK)
    {
        printf("film folder:\r\n");
    
        while (f_readdir(&dirs, &finfo) == FR_OK)
        {
            if (finfo.fattrib & AM_DIR)
            {
                //文件名不为空，如果为空，则表明该目录下面的文件已经读完了
                if(!finfo.fname[0])
                {
                    break;
                }
                stringcopy(film_foldername[i_name], (BYTE*)finfo.fname);
                printf("          [%d]:%s\r\n",i_name,film_foldername[i_name]);
                i_name++;
            }
        }
        filmTotalProgram = i_name;
    }
    else
    {
        printf("open path %s failed.\r\n",path);
    }
}

void SD_fileCopy(void)
{
    UINT a = 1;
    char buffer[1024];
    FRESULT res1, res2;   // FatFs function common result code

    memset(buffer,0,sizeof(buffer));

    f_mount(0, &fs);
    res1 = f_open(&fdst, "huangzhi", FA_CREATE_ALWAYS | FA_WRITE);//this fatfs support 8bytes file name
    res2 = f_open(&fsrc, "opple1.txt", FA_OPEN_EXISTING | FA_READ);
    printf("res1:%d,res2:%d\r\n",res1,res2);

    if((res1 || res2) != 0)
    {
        if(!res2)
            f_close(&fsrc);
        if(!res1)
            f_close(&fdst);

        printf("file system test error\r\n");
        return;
    }

    f_read(&fsrc, buffer,256, &a);      //从opplem.txt中读取1024个字节
    printf("read len %d\r\n",a);

    f_write(&fdst, buffer,a, &a);     //huangzhicheng.txt中写人1024个字节      
    f_close(&fsrc);
    f_close(&fdst);
}


