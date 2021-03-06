#include "main.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "mbi5153.h"
#include "delay.h"
#include "programs.h"
#include "config.h"

extern uint16_t                 fileTotalFilm[MAX_FILM_FOLDER];
extern uint16_t                 fileTotalPhoto;
extern uint16_t                 fileTotallight;
extern uint16_t                 filmTotalProgram;

static FATFS                    fs;            // Work area (file system object) for logical drive
static FIL                      fsrc, fdst;      // file objects
static uint32_t                 fileOffset = 0;


#if(PROJECTOR_OSRAM || PROJECTOR_CUBE)
BYTE                            photo_filename[MAX_FILE_NUM][FILE_NAME_LEN_SHORT];
BYTE                            light_filename[MAX_FILE_NUM][FILE_NAME_LEN_SHORT];
//BYTE                            film_filename[MAX_FILM_FRAME][FILE_NAME_LEN];
//BYTE                            film_foldername[MAX_FILM_FOLDER][FILE_NAME_LEN];
FILE_INFO_S                     film_file[MAX_FILM_FOLDER];
char                            fileBuffer[MAX_FILE_SIZE];
#endif

#if(PROJECTOR_OSRAM)
uint8_t                         osram_buff[MATRIX_SIZE][MATRIX_SIZE];

uint8_t                         osram_clock_num_up[10][12][6]={
                                    //0
                                    {
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    1,1,0,0,1,1,
                                    1,1,0,0,1,1,
                                    1,1,0,0,1,1,
                                    1,1,0,0,1,1,
                                    1,1,0,0,1,1,
                                    1,1,0,0,1,1,
                                    1,1,0,0,1,1,
                                    1,1,0,0,1,1,
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    },
                                    {
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    },
                                    {
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    1,1,0,0,0,0,
                                    1,1,0,0,0,0,
                                    1,1,0,0,0,0,
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    },
                                    {
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    },

                                    {
                                    1,1,0,0,1,1,
                                    1,1,0,0,1,1,
                                    1,1,0,0,1,1,
                                    1,1,0,0,1,1,
                                    1,1,0,0,1,1,
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    },
                                    
                                    {
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    1,1,0,0,0,0,
                                    1,1,0,0,0,0,
                                    1,1,0,0,0,0,
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    },                                    
                                    {
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    1,1,0,0,0,0,
                                    1,1,0,0,0,0,
                                    1,1,0,0,0,0,
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    1,1,0,0,1,1,
                                    1,1,0,0,1,1,
                                    1,1,0,0,1,1,
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    },


                                    {
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    },
                                    {
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    1,1,0,0,1,1,
                                    1,1,0,0,1,1,
                                    1,1,0,0,1,1,
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    1,1,0,0,1,1,
                                    1,1,0,0,1,1,
                                    1,1,0,0,1,1,
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    },

                                    {
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    1,1,0,0,1,1,
                                    1,1,0,0,1,1,
                                    1,1,0,0,1,1,
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    0,0,0,0,1,1,
                                    1,1,1,1,1,1,
                                    1,1,1,1,1,1,
                                    },
                                };
                                    
uint8_t                         osram_clock_num_down[10][5][3]={
                                    {
                                    1,1,1,
                                    1,0,1,
                                    1,0,1,
                                    1,0,1,
                                    1,1,1,
                                    },
                                    {
                                    0,0,1,
                                    0,0,1,
                                    0,0,1,
                                    0,0,1,
                                    0,0,1,
                                    },
                                    {
                                    1,1,1,
                                    0,0,1,
                                    1,1,1,
                                    1,0,0,
                                    1,1,1,
                                    },

                                    {
                                    1,1,1,
                                    0,0,1,
                                    1,1,1,
                                    0,0,1,
                                    1,1,1,
                                    },

                                    {
                                    1,0,1,
                                    1,0,1,
                                    1,1,1,
                                    0,0,1,
                                    0,0,1,
                                    },

                                    {
                                    1,1,1,
                                    1,0,0,
                                    1,1,1,
                                    0,0,1,
                                    1,1,1,
                                    },
                                    {
                                    1,1,1,
                                    1,0,0,
                                    1,1,1,
                                    1,0,1,
                                    1,1,1,
                                    },
                                    {
                                    1,1,1,
                                    0,0,1,
                                    0,0,1,
                                    0,0,1,
                                    0,0,1,
                                    },
                                    {
                                    1,1,1,
                                    1,0,1,
                                    1,1,1,
                                    1,0,1,
                                    1,1,1,
                                    },
                                    {
                                    1,1,1,
                                    1,0,1,
                                    1,1,1,
                                    0,0,1,
                                    1,1,1,
                                    },
                                };

uint8_t                         osram_clock[MATRIX_SIZE][MATRIX_SIZE]={
                                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                    1,1,1,1,1,1,0,1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,0,1,1,1,1,1,1,
                                    1,1,1,1,1,1,0,1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,0,1,1,1,1,1,1,
                                    0,0,0,0,1,1,0,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,1,1,0,0,1,1,
                                    0,0,0,0,1,1,0,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,1,1,0,0,1,1,
                                    0,0,0,0,1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,1,1,0,0,1,1,0,1,1,0,0,1,1,
                                    1,1,1,1,1,1,0,1,1,1,1,1,1,0,0,0,0,0,0,1,1,0,0,1,1,0,1,1,0,0,1,1,
                                    1,1,1,1,1,1,0,1,1,1,1,1,1,0,0,0,0,0,0,1,1,0,0,1,1,0,1,1,0,0,1,1,
                                    1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,1,1,0,0,1,1,
                                    1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,1,1,0,0,1,1,
                                    1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,0,1,1,0,1,1,0,0,1,1,
                                    1,1,1,1,1,1,0,1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,0,1,1,1,1,1,1,
                                    1,1,1,1,1,1,0,1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,0,1,1,1,1,1,1,
                                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,

                                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,0,0,1,1,1,0,0,1,1,1,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0,0,0,1,0,0,1,0,1,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,1,1,1,0,0,1,0,1,0,0,1,1,1,0,0,1,0,1,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,0,0,1,1,1,0,0,1,1,1,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,1,1,1,0,0,0,1,0,0,0,0,0,0,1,1,1,0,0,0,1,0,0,0,0,0,0,
                                    0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,
                                    0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,1,1,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,
                                    0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,
                                    0,0,0,0,0,0,1,1,1,0,0,0,1,0,0,0,0,0,0,1,1,1,0,0,0,1,0,0,0,0,0,0,
                                };


#elif(PROJECTOR_CUBE)
uint8_t                         cube_buff_G[CUBE_RGB_SIZE][CUBE_COL_SIZE];
uint8_t                         cube_buff_R[CUBE_RGB_SIZE][CUBE_COL_SIZE];
uint8_t                         cube_buff_B[CUBE_RGB_SIZE][CUBE_COL_SIZE];
#elif(PROJECTOR_MBI5124)
//;
#elif(CUBEPLT_SLAVE)
uint8_t                         cube_buff_G[IO_SIZE][CHIP_SIZE];
uint8_t                         cube_buff_R[IO_SIZE][CHIP_SIZE];
uint8_t                         cube_buff_B[IO_SIZE][CHIP_SIZE];
#elif(CUBEPLT_MASTER)
uint8_t                         cube_buff_RGB[IO_SIZE*CHIP_SIZE*3];
#endif

uint8_t                         photoProgramIdx = 0;
uint8_t                         lightProgramIdx = 0;
uint8_t                         filmProgramIdx = 0;
uint16_t                        filmFrameIdx = 0;
PROGRAMS_TYPE_E                 programsType;

#if(PROJECTOR_OSRAM)
void updateTimeBuf(uint8_t hourH,uint8_t hourL,uint8_t MinH,uint8_t MinL)
{
    uint8_t i;
    /*
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    1,1,1,1,1,1,0,1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,0,1,1,1,1,1,1,
    1,1,1,1,1,1,0,1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,0,1,1,1,1,1,1,
    0,0,0,0,1,1,0,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,1,1,0,0,1,1,
    0,0,0,0,1,1,0,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,1,1,0,0,1,1,
    0,0,0,0,1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,1,1,0,0,1,1,0,1,1,0,0,1,1,
    1,1,1,1,1,1,0,1,1,1,1,1,1,0,0,0,0,0,0,1,1,0,0,1,1,0,1,1,0,0,1,1,
    1,1,1,1,1,1,0,1,1,1,1,1,1,0,0,0,0,0,0,1,1,0,0,1,1,0,1,1,0,0,1,1,
    1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,1,1,0,0,1,1,
    1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,1,1,0,0,1,1,0,0,1,1,0,1,1,0,0,1,1,
    1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,0,1,1,0,1,1,0,0,1,1,
    1,1,1,1,1,1,0,1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,0,1,1,1,1,1,1,
    1,1,1,1,1,1,0,1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,0,1,1,1,1,1,1,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    */


    osram_clock[7][15]=1;
    osram_clock[7][16]=1;
    osram_clock[8][15]=1;
    osram_clock[8][16]=1;
    osram_clock[12][15]=1;
    osram_clock[12][16]=1;
    osram_clock[13][15]=1;
    osram_clock[13][16]=1;

    for(i=0;i<12;i++)
    {
        memcpy(&osram_clock[5+i][0],osram_clock_num_up[hourH][i],6);
    }
    for(i=0;i<12;i++)
    {
        memcpy(&osram_clock[5+i][7],osram_clock_num_up[hourL][i],6);
    }

    for(i=0;i<12;i++)
    {
        memcpy(&osram_clock[5+i][19],osram_clock_num_up[MinH][i],6);
    }
    for(i=0;i<12;i++)
    {
        memcpy(&osram_clock[5+i][26],osram_clock_num_up[MinL][i],6);
    }
}

void updateDateBuf(uint8_t year1,uint8_t year2,uint8_t year3,uint8_t year4,uint8_t monH,uint8_t monL,uint8_t dayH,uint8_t dayL)    
{
    uint8_t i;
    
    /*
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,0,0,1,1,1,0,0,1,1,1,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0,0,0,1,0,0,1,0,1,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,1,1,1,0,0,1,0,1,0,0,1,1,1,0,0,1,0,1,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,0,0,1,1,1,0,0,1,1,1,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,1,1,1,0,0,0,1,0,0,0,0,0,0,1,1,1,0,0,0,1,0,0,0,0,0,0,
    0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,
    0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,1,1,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,
    0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,
    0,0,0,0,0,0,1,1,1,0,0,0,1,0,0,0,0,0,0,1,1,1,0,0,0,1,0,0,0,0,0,0,
    */

    osram_clock[29][15]=1;
    osram_clock[29][16]=1;

    for(i=0;i<5;i++)
    {
        memcpy(&osram_clock[21+i][7],osram_clock_num_down[year1][i],3);
    }
    for(i=0;i<5;i++)
    {
        memcpy(&osram_clock[21+i][12],osram_clock_num_down[year2][i],3);
    }
    for(i=0;i<5;i++)
    {
        memcpy(&osram_clock[21+i][17],osram_clock_num_down[year3][i],3);
    }
    for(i=0;i<5;i++)
    {
        memcpy(&osram_clock[21+i][22],osram_clock_num_down[year4][i],3);
    }


    for(i=0;i<5;i++)
    {
        memcpy(&osram_clock[27+i][6],osram_clock_num_down[monH][i],3);
    }
    for(i=0;i<5;i++)
    {
        memcpy(&osram_clock[27+i][10],osram_clock_num_down[monL][i],3);
    }

    for(i=0;i<5;i++)
    {
        memcpy(&osram_clock[27+i][19],osram_clock_num_down[dayH][i],3);
    }
    for(i=0;i<5;i++)
    {
        memcpy(&osram_clock[27+i][23],osram_clock_num_down[dayL][i],3);
    }
}

void WifiControlEplosHandle(uint8_t *data)
{
    memset(osram_clock, 0, MATRIX_SIZE*MATRIX_SIZE);
    updateTimeBuf(data[8]-'0',data[9]-'0',data[10]-'0',data[11]-'0');
    updateDateBuf(data[0]-'0',data[1]-'0',data[2]-'0',data[3]-'0',data[4]-'0',data[5]-'0',data[6]-'0',data[7]-'0');

    memcpy(osram_buff,osram_clock,MATRIX_SIZE*MATRIX_SIZE);
}

#endif


#if(PROJECTOR_OSRAM || PROJECTOR_CUBE)
FRESULT SD_ReadPhotoData(uint8_t *path)
{
    FRESULT res;
    UINT a = 1,i,j;

    f_mount(0, &fs);
    res = f_open(&fsrc, path, FA_OPEN_EXISTING | FA_READ);
    if(res != 0)
    {
        printf("open [%s] error[%d]!\r\n",path,res);
        return res;
    }
    memset(fileBuffer, 0, MAX_FILE_SIZE);
    f_read(&fsrc, fileBuffer, MAX_FILE_SIZE, &a);
    f_close(&fsrc);
    printf("\r\n[%s] return %d bytes\r\n",path,a);

#if(PROJECTOR_OSRAM)
    for( i=0; i<MATRIX_SIZE; i++ )
    {
        for( j=0; j<MATRIX_SIZE; j++ )
        {
            osram_buff[i][j] = fileBuffer[i*(64+1)+j*2]-'0';
        }
    }
    /*
    for( i=0; i<MATRIX_SIZE; i++ )
    {
        for( j=0; j<MATRIX_SIZE; j++ )
        {
            printf("%d ",osram_buff[i][j]);
        }
        printf("\r\n");
    }*/
    
#elif(PROJECTOR_CUBE)
    for( i=0; i<CUBE_ROW_SIZE; i++ )
    {
        for( j=0; j<CUBE_COL_SIZE; j++ )
        {
            cube_buff_R[i][j] = fileBuffer[i*96+j*3]/RGB_SCALE;
            cube_buff_G[i][j] = fileBuffer[i*96+j*3+1]/RGB_SCALE;
            cube_buff_B[i][j] = fileBuffer[i*96+j*3+2]/RGB_SCALE;
        }
    }
#endif

    return FR_OK;
}

#ifdef LARGE_FILE
FRESULT SD_OpenFilmData()
{
    FRESULT res;
    UINT a = 1,i,j;
    uint8_t path[FILE_PATH_LEN];

    memset(path, 0, sizeof(path));

#if(PROJECTOR_OSRAM)
    sprintf(path,"/OSRAM/Film/%s/%s",film_file[filmProgramIdx%filmTotalProgram].foldername,film_file[filmProgramIdx%filmTotalProgram].filename);
#elif(PROJECTOR_CUBE)
    sprintf(path,"/WS2801/Film/%s/%s",film_file[filmProgramIdx%filmTotalProgram].foldername,film_file[filmProgramIdx%filmTotalProgram].filename);
#endif

    printf("%s\r\n",path);
    f_mount(0, &fs);
    res = f_open(&fsrc, path, FA_OPEN_EXISTING | FA_READ);
    if(res != 0)
    {
        printf("open [%s] error[%d]!\r\n", path, res);
        return res;
    }
    fileOffset = 0;

    return FR_OK;
}

FRESULT SD_ReadFilmData()
{
    FRESULT res;
    UINT a = 1,i,j;

    //printf("%d\r\n",fileOffset);
    res = f_lseek(&fsrc, fileOffset);           //偏移到文件尾部（问题：file_size不为0 导致后续写入变慢）
    if(res != 0)
    {
        printf("f_lseek error[%d]!\r\n", res);
        return res;
    }
    memset(fileBuffer, 0, MAX_FILE_SIZE);
    res = f_read(&fsrc, fileBuffer, MAX_FILE_SIZE, &a);
    if(res != 0)
    {
        printf("f_read error[%d]!\r\n", res);
        return res;
    }
    fileOffset += MAX_FILE_SIZE;
    if(fileOffset >= film_file[filmProgramIdx%filmTotalProgram].filesize)
    {
        fileOffset = 0;
    }

#if(PROJECTOR_OSRAM)
    for( i=0; i<MATRIX_SIZE; i++ )
    {
        for( j=0; j<MATRIX_SIZE; j++ )
        {
            osram_buff[i][j] = fileBuffer[i*(64+1)+j*2]-'0';
        }
    }
#elif(PROJECTOR_CUBE)
    for( i=0; i<CUBE_ROW_SIZE; i++ )
    {
        for( j=0; j<CUBE_COL_SIZE; j++ )
        {
            cube_buff_R[i][j] = fileBuffer[i*96+j*3];
            cube_buff_G[i][j] = fileBuffer[i*96+j*3+1];
            cube_buff_B[i][j] = fileBuffer[i*96+j*3+2];
        }
    }
#endif

    return FR_OK;
}
#else
FRESULT SD_ReadFilmData()
{
    FRESULT res;
    UINT a = 1,i,j;
    uint8_t path[FILE_PATH_LEN];

    memset(path, 0, sizeof(path));

#if(PROJECTOR_OSRAM)
    //sprintf(path,"/OSRAM/Film/%s/%s",film_foldername[filmProgramIdx%filmTotalProgram],film_filename[filmFrameIdx%fileTotalFilm[filmProgramIdx%filmTotalProgram]]);
    sprintf(path,"/OSRAM/Film/%s/%s",film_file[filmProgramIdx%filmTotalProgram].foldername,film_file[filmProgramIdx%filmTotalProgram].filename);
#elif(PROJECTOR_CUBE)
    sprintf(path,"/WS2801/Film/%s/%s",film_file[filmProgramIdx%filmTotalProgram].foldername,film_file[filmProgramIdx%filmTotalProgram].filename);
#endif

    printf("%s\r\n",path);
    f_mount(0, &fs);
    res = f_open(&fsrc, path, FA_OPEN_EXISTING | FA_READ);
    if(res != 0)
    {
        printf("open [%s] error[%d]!\r\n", path, res);
        return res;
    }
    memset(fileBuffer, 0, MAX_FILE_SIZE);
    f_read(&fsrc, fileBuffer, MAX_FILE_SIZE, &a);
    f_close(&fsrc);

#if(PROJECTOR_OSRAM)
    for( i=0; i<MATRIX_SIZE; i++ )
    {
        for( j=0; j<MATRIX_SIZE; j++ )
        {
            osram_buff[i][j] = fileBuffer[i*(64+1)+j*2]-'0';
        }
    }
#elif(PROJECTOR_CUBE)
    for( i=0; i<CUBE_ROW_SIZE; i++ )
    {
        for( j=0; j<CUBE_COL_SIZE; j++ )
        {
            cube_buff_G[i][j] = fileBuffer[i*96+j*3]; //96=32(col)*3(r/g/b)
            cube_buff_R[i][j] = fileBuffer[i*96+j*3+1];
            cube_buff_B[i][j] = fileBuffer[i*96+j*3+2];
        }
    }
#endif

    return FR_OK;
}
#endif

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

    //printf("SD_ReadPhotoFileList:%s\r\n",path);

    /*挂载文件系统*/
    f_mount(0, &fs);
    res =  f_opendir(&dirs, path);
    if (res == FR_OK)
    {
        printf("photo files ");

        while (f_readdir(&dirs, &finfo) == FR_OK)
        {
            if (finfo.fattrib & AM_ARC)
            {
                if(!finfo.fname[0]){      //文件名不为空，如果为空，则表明该目录下面的文件已经读完了
                    break;
                }
                stringcopy(photo_filename[i_name], (BYTE*)finfo.fname);
                //printf("          [%d]:%s\r\n",i_name,photo_filename[i_name]);
                i_name++;
            }
        }
        fileTotalPhoto = i_name;
        printf(" %d\r\n",fileTotalPhoto);
    }
    else
    {
        printf("open path %s failed.\r\n",path);
    }
}

void SD_ReadLightFileList(char *path)
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
        printf("light files ");

        while (f_readdir(&dirs, &finfo) == FR_OK)
        {
            if (finfo.fattrib & AM_ARC)
            {
                if(!finfo.fname[0]){      //文件名不为空，如果为空，则表明该目录下面的文件已经读完了
                    break;
                }
                stringcopy(light_filename[i_name], (BYTE*)finfo.fname);
                //printf("          [%d]:%s\r\n",i_name,light_filename[i_name]);
                i_name++;
            }
        }
        fileTotallight = i_name;
        printf(" %d\r\n",fileTotallight);
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
    int br;
    FRESULT res;
    char path[FILE_PATH_LEN];

    //printf("SD_ReadFilmFileList:%s\r\n",path);

    memset(path, 0, sizeof(path));
    
#if(PROJECTOR_OSRAM)
    sprintf(path,"/OSRAM/Film/%s",film_file[filmId].foldername);
#elif(PROJECTOR_CUBE)
    sprintf(path,"/WS2801/Film/%s",film_file[filmId].foldername);
#elif(CUBEPLT_MASTER)
    sprintf(path,"/CUBE/Film/%s",film_file[filmId].foldername);
#endif
    /*挂载文件系统*/
    f_mount(0, &fs);
    res =  f_opendir(&dirs, path);
    if (res == FR_OK)
    {
        printf("film[%s] files ",film_file[filmId].foldername);
    
        while (f_readdir(&dirs, &finfo) == FR_OK)
        {
            if (finfo.fattrib & AM_ARC)
            {
                //文件名不为空，如果为空，则表明该目录下面的文件已经读完了
                if(!finfo.fname[0])
                {
                    break;
                }
                stringcopy(film_file[filmId].filename, (BYTE*)finfo.fname);
                film_file[filmId].filesize = finfo.fsize;
                //printf("          [%s]size:%lld\r\n",film_file[filmId].filename,film_file[filmId].filesize);
                //i_name++;
            }
        }
        film_file[filmId].framNum = film_file[filmId].filesize/MAX_FILE_SIZE;
        printf("          [%s]size:%ld fnum:%d\r\n",film_file[filmId].filename,film_file[filmId].filesize,film_file[filmId].framNum);
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

    //printf("SD_ReadFilmFolderList:%s\r\n",path);

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
                stringcopy(film_file[i_name].foldername, (BYTE*)finfo.fname);
                printf("          [%d]:%s\r\n",i_name,film_file[i_name].foldername);
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
#endif

