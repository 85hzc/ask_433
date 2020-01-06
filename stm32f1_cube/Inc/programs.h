#ifndef __PROGRAMS_H__
#define __PROGRAMS_H__

#define MINUS(a,b) ((a-b<0)?0:a-b)
#define PLUS(a,b) ((a+b>0xff)?0xff:a+b)

typedef enum
{
    #if PROJECTOR_CUBE
    AUTO_ALGORITHM = 0,
    #endif
    PHOTO,

    #if PROJECTOR_OSRAM
    LIGHT,
    #endif
    MAX_PROGRAMS,
    FILM,
    APP,
    FIXED,

} PROGRAMS_TYPE_E;

typedef enum
{
    WHOLE_SHOW = 0,
    CANDLE_SHOW,
    LAYING_SHOW,
    CYCLE_SHOW,
    WAVING_SHOW,
    RAINBOW_SHOW,
    SECT_SHOW,

    SOFT_PROGRAMS_MAX,
} SOFT_PROGRAMS_E;

#endif
