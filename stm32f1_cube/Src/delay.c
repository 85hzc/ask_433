/***
    ***************************************************************************
    *   @file   delay.c
    *   @brief   delay接口相关函数
   ***************************************************************************
   *  @description
    *
    *  SysTick定时器配置为1ms中断，实现毫秒延时
    * 
    ***************************************************************************
***/

#include "delay.h"
#include "gpio.h"
#include "mbi5153.h"
#include "mbi5124.h"
#include "mcugpio.h"
#include "main.h"

static uint32_t TimingDelay;  //计数变量
volatile uint32_t gclk_pluse = 0xff;

void delay_ns(uint32_t time)
{
    //u32 tt = time;
    while(time--) {
        __ASM("nop");
        __ASM("nop");
        __ASM("nop");
        __ASM("nop");
    }
}

void delayns_100()
{
    __ASM("nop");
}
void delayns_300()
{
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
    //__ASM("nop");
    //__ASM("nop");
}

void delayns_600()
{
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    /*__ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");*/
}


void delayns_900()
{
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
    __ASM("nop");    __ASM("nop");
}

/*
//  函数：延时初始化
//  说明：配置 SysTick 为1ms中断，并启动定时器
//
void Delay_Init(void)
{
    SysTick_Config(SystemCoreClock / (1000*1000));  //配置SysTick时钟为1ms(1000) 1us(1000*1000)中断
}
*/
//  函数：计时函数
//  说明：在 SysTick 中断服务函数里被调用
void TimingDelay_Decrement(void)
{
    if (TimingDelay != 0)
    {
        TimingDelay--;
    }
}

//  函数：毫秒延时
//  参数：nTime - 延时时间，单位ms
//  说明：每次调用都会重新给TimingDelay赋值，实现 n 毫秒的延时，最大延时 4294967295 ms。
//
void Delay_us(u32 nTime)
{
    TimingDelay = nTime;

    while(TimingDelay != 0);
}

//#define PWM_129 129
#if 0
void gclk(void)
{
#if(TIMER==0)
    if(pluse_force
#if(SEPR_SECTION==1)
    || (gclk_pluse<129&&pluse_enable)
#else
    || pluse_enable
#endif
    )
    {
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        //gclk_pluse = (gclk_pluse==128)?(0):(gclk_pluse+1);
        gclk_pluse++;
        GCLK_PIN_L
    } /*else {
        gclk_pluse++;
        //if(gclk_auto==100)
        {
            //gclk_pluse = 0;
            //gclk_time = 0;
        }
        //gclk_time++;
        //gclk_auto++;
    }*/
#endif
}
#else
void gclk(void)
{
#if(PROJECTOR_MBI5153)
    if(gclk_pluse<129)
    {
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        GCLK_PIN_H
        //gclk_pluse = (gclk_pluse==128)?(0):(gclk_pluse+1);
        gclk_pluse++;
        GCLK_PIN_L
    }
#endif
}
#endif


