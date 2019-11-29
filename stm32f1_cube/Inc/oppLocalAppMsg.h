/******************************************************************************
* Version     : OPP_PLATFORM V100R001C01B001D001                              *
* File        : oppLocalAppMsg.h                                              *
* Description :                                                               *
*               OPPLE业务消息统一定义文件                                     *
* Note        : (none)                                                        *
* Author      : 智能控制研发部                                                *
* Date:       : 2013-08-09                                                    *
* Mod History : (none)                                                        *
******************************************************************************/
#ifndef __OPP_LOCAL_MSG_H__
#define __OPP_LOCAL_MSG_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
*                                Includes                                     *
******************************************************************************/
//#include "oppCommon.h"
/******************************************************************************
*                                Defines                                      *
******************************************************************************/

/******************************************************************************
*                                Typedefs                                     *
******************************************************************************/
#pragma pack(1)
/*业务网络路由层（Opple_Link）协议头结构定义*/
typedef struct
{
    uint16_t srcAddr;
    uint16_t dstAddr;
    uint8_t srcTransNo;
    uint8_t dstTransNo;
    uint8_t msgLen;
    uint16_t groupNo;
} ST_OPP_LC_NET_LAYER;

//////////////////////////////////////////////////业务协议主体结构定义/////////////////////////////////////////
/*(0x030D)APP->LAMP:调频道+请求消息结构定义*/
typedef struct
{
    uint16_t uwMsgType;                      /*消息命令字:*/
}ST_OPP_LC_LAMP_GATEWAY_CALL_SCENE_PLUS_MSG;
#define CALL_SCENE_PLUS_MSG                         0x030D

/*(0x030E)APP->LAMP:调频道-请求消息结构定义*/
typedef struct
{
    uint16_t uwMsgType;                       /*消息命令字:*/
}ST_OPP_LC_LAMP_GATEWAY_CALL_SCENE_MINUS_MSG;
#define CALL_SCENE_MINUS_MSG                        0x030E

/*(0x030F)APP->LAMP:模式切换请求消息结构定义*/
typedef struct
{
    uint16_t uwMsgType;                       /*消息命令字:*/
}ST_OPP_LC_GATEWAY_LAMP_SOURCE_CTRL_SWITCH_MODE_MSG;
#define CTRL_SWITCH_MODE_MSG                        0x030F

/*(0x0310)APP->LAMP:待机请求消息结构定义*/
typedef struct
{
    uint16_t uwMsgType;                       /*消息命令字:*/
}ST_OPP_LC_GATEWAY_LAMP_SOURCE_CTRL_SWITCH_OFF_MSG;
#define CTRL_SWITCH_OFF_MSG                        0x0310

/*(0x0311)APP->LAMP:开关请求消息结构定义*/
typedef struct
{
    uint16_t uwMsgType;                       /*消息命令字:*/
}ST_OPP_LC_GATEWAY_LAMP_SOURCE_CTRL_SWITCH_ON_MSG;
#define CTRL_SWITCH_ON_MSG                        0x0311

/*(0x0312)APP->LAMP:调焦+消息结构定义*/
typedef struct
{
    uint16_t uwMsgType;                       /*消息命令字:*/
}ST_OPP_LC_GATEWAY_LAMP_SOURCE_CTRL_MOTOR_R_MSG;
#define CTRL_MOTOR_R_MSG                        0x0312

/*(0x0313)APP->LAMP:调焦-消息结构定义*/
typedef struct
{
    uint16_t uwMsgType;                       /*消息命令字:*/
}ST_OPP_LC_GATEWAY_LAMP_SOURCE_CTRL_MOTOR_L_MSG;
#define CTRL_MOTOR_L_MSG                        0x0313

/*(0x0314)APP->LAMP:亮度调节+消息结构定义*/
typedef struct
{
    uint16_t uwMsgType;                       /*消息命令字:*/
}ST_OPP_LC_LAMP_GATEWAY_SOURCE_CTRL_BRIGHT_PLUS_MSG;
#define CTRL_BRIGHT_PLUS_MSG                    0x0314

/*(0x0315)APP->LAMP:亮度调节-消息结构定义*/
typedef struct
{
    uint16_t uwMsgType;                       /*消息命令字:*/
}ST_OPP_LC_GATEWAY_LAMP_SOURCE_CTRL_BRIGHT_MINUS_MSG;
#define CTRL_BRIGHT_MINUS_MSG                   0x0315

/*(0x0316)APP->LAMP:擦除显示消息结构定义*/
typedef struct
{
    uint16_t uwMsgType;                       /*消息命令字:*/
}ST_OPP_LC_GATEWAY_LAMP_SOURCE_CTRL_ERASE_MSG;
#define CTRL_ERASE_MSG                          0x0316

/*(0x0317)APP->LAMP:图形信息消息结构定义*/
typedef struct
{
    uint16_t uwMsgType;                       /*消息命令字:*/
    unsigned char msgBody[128];            /*图形信息使用比特位表示*/
}ST_OPP_LC_LAMP_GATEWAY_SOURCE_MATRIX_MSG;
#define CTRL_MATRIX_MSG                         0x0317


#pragma pack()
/******************************************************************************
*                           Extern Declarations                               *
******************************************************************************/

/******************************************************************************
*                              Declarations                                   *
******************************************************************************/


#ifdef __cplusplus
}
#endif


#endif /*__OPP_LOCAL_MSG_H__*/


