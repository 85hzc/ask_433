/*-------------------------------------------*/
/* Integer type definitions for FatFs module */
/*-------------------------------------------*/

#ifndef _INTEGER

#include "stm32f10x.h" 
//#if 0
//#include <windows.h>
//#else

/* These types must be 16-bit, 32-bit or larger integer */
typedef int				INT;
typedef unsigned int	UINT;

/* These types must be 8-bit integer */
typedef signed char		CHAR;
typedef unsigned char	UCHAR;
typedef unsigned char	BYTE;

/* These types must be 16-bit integer */
typedef short			SHORT;
typedef unsigned short	USHORT;
typedef unsigned short	WORD;
typedef unsigned short	WCHAR;

/* These types must be 32-bit integer */
typedef long			LONG;
typedef unsigned long	ULONG;
typedef unsigned long	DWORD;

typedef unsigned char   INT8U;        // �޷���8λ���ͱ���                     
typedef signed   char   INT8S;        // �з���8λ���ͱ���                        
typedef unsigned int    INT16U;       // �޷���16λ���ͱ���                       
typedef signed   int    INT16S;       // �з���16λ���ͱ���                       
typedef unsigned long   INT32U;       // �޷���32λ���ͱ���                       
typedef signed long     INT32S;       // �з���32λ���ͱ���                       
/* Boolean type */
#ifndef __STM32F10x_TYPE_H    
/*����ļ���������typedef enum {FALSE = 0, TRUE = !FALSE} bool;�����������ͻ */
typedef enum { FALSE = 0, TRUE } BOOL;
#else
typedef bool BOOL;
#endif

#define _INTEGER
#endif
