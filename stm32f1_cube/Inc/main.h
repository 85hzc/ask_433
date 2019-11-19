/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "gpio.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/


/* USER CODE BEGIN Private defines */
#define DEBUG_ENABLE        0                   //debug输出开启
//#define SYS_FREQUENCY       32000000            //32MHz主频

//MBI5153方案
#define PROJECTOR_MBI5153   0

//MBI5124 MBI5020方案
#define PROJECTOR_MBI5124   0

//OSRAM eplos芯片投射灯方案
#define PROJECTOR_OSRAM     0

//gpio控制16*16方案
#define PROJECTOR_MCUGPIO   0

//分体系列立体吊灯方案
#define PROJECTOR_CUBE      1
#define CUBE_MASTER         1
#define CUBE_SLAVE          0

#define PROJECTOR_CUBEPLT   0
#define CUBEPLT_MASTER      0
#define CUBEPLT_SLAVE       0


typedef volatile uint32_t   vu32;
typedef volatile uint16_t   vu16;
typedef volatile uint8_t    vu8;
typedef uint32_t            u32;
typedef uint16_t            u16;
typedef uint8_t             u8;

#define UART_RX_LEN         128

typedef struct
{
    uint8_t RX_flag:1;          //IDLE receive flag
    uint16_t RX_Size;           //receive length
    uint8_t RX_pData[UART_RX_LEN];   //DMA receive buffer
}USART_RECEIVETYPE;

#if DEBUG_ENABLE
#define LOG_DEBUG(format, args...)    do { \
                                        printf(format, ##args);\
                                      } while(0)
#else
#define LOG_DEBUG(format, args...)    do {\
                                      } while(0)
#endif  

#define DBG                 printf("%s[%d]\r\n",__FUNCTION__,__LINE__);

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
