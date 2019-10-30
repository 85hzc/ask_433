#ifndef __PROGRAMS_H__
#define __PROGRAMS_H__

#define MINUS(a,b) ((a-b<0)?0:a-b)
#define PLUS(a,b) ((a+b>0xff)?0xff:a+b)


typedef enum
{
    AUTO_ALGORITHM = 0,
    PHOTO,
    FILM,

    MAX_PROGRAMS,
} PROGRAMS_TYPE_E;

#endif