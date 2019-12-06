/***
    ***************************************************************************
    *   @file   delay.c
    *   @brief   delay�ӿ���غ���
   ***************************************************************************
   *  @description
    *
    *  SysTick��ʱ������Ϊ1ms�жϣ�ʵ�ֺ�����ʱ
    * 
    ***************************************************************************
***/

#include "delay.h"
#include "gpio.h"
#include "mbi5153.h"
#include "mbi5124.h"
#include "mcugpio.h"
#include "main.h"

static uint32_t TimingDelay;  //��������
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
//  ��������ʱ��ʼ��
//  ˵�������� SysTick Ϊ1ms�жϣ���������ʱ��
//
void Delay_Init(void)
{
    SysTick_Config(SystemCoreClock / (1000*1000));  //����SysTickʱ��Ϊ1ms(1000) 1us(1000*1000)�ж�
}
*/
//  ��������ʱ����
//  ˵������ SysTick �жϷ������ﱻ����
void TimingDelay_Decrement(void)
{
    if (TimingDelay != 0)
    {
        TimingDelay--;
    }
}

//  ������������ʱ
//  ������nTime - ��ʱʱ�䣬��λms
//  ˵����ÿ�ε��ö������¸�TimingDelay��ֵ��ʵ�� n �������ʱ�������ʱ 4294967295 ms��
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


