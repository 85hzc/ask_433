/**********************************************************
* @ File name -> delay.c
* @ Version   -> V1.0
* @ Date      -> 12-26-2013
* @ Brief     -> 系统延时相关的函数

* @ 详细说明请参考《Cortex-M3权威指南(中文)》第133 ~ 134页 第8章 SysTick定时器介绍
**********************************************************/

/*
* @ SysTick定时器 相关控制寄存器说明

@ 1、SysTick控制及状态寄存器（地址：0xE000_E010）复位值为0

	bit16 COUNTFLAG(R）  -> 如果在上次读取本寄存器后，SysTick已经数到了0，则该位为1。如果读取该位，该位将自动清零
	bit2  CLKSOURCE(R/W) -> 0=外部时钟源(STCLK)。1=内核时钟(FCLK) 
	bit1  ENABLE(R/W)    -> SysTick定时器的使能位

@ 2、SysTick重装载数值寄存器（地址：0xE000_E014）复位值为0

	[23:0] RELOAD(R/W) -> 当倒数至零时，将被重装载的值

@ 3、SysTick当前数值寄存器（地址：0xE000_E018） 复位值为0

	[23:0] CURRENT(R/Wc) -> 读取时返回当前倒计数的值，写它则使之清零，同时还会清除在SysTick 控制及状态寄存器中的COUNTFLAG标志

@ 4、SysTick校准数值寄存器（地址：0xE000_E01C）复位值: bit31未知。bit30未知。[23:0]为0

	bit32 NOREF(R)    -> 1=没有外部参考时钟（STCLK不可用）。0=外部参考时钟可用
	bit30 SKEW(R)     -> 1=校准值不是准确的10ms。0=校准值是准确的10ms
	[23:0] TENMS(R/W) -> 10ms的时间内倒计数的格数。芯片设计者应该通过Cortex‐M3的输入信号提供该数值。若该值读回零，则表示无法使用校准功能
	
*/ 

#include "delay.h"

/**********************************************************
                     定义计算变量
**********************************************************/

static uint8_t  fac_us=0;	//us延时倍乘数
static uint16_t fac_ms=0;	//ms延时倍乘数

/**********************************************************
* 函数功能 ---> 初始化延时函数
* 入口参数 ---> SYSCLK：系统工作最高的频率。单位M
* 返回数值 ---> none
* 功能说明 ---> 主要是初始化SysTick寄存器 
**********************************************************/
void delay_init(uint8_t SYSCLK)
{
	SysTick->CTRL &= 0xfffffffb;	//bit2清空，选择外部时钟HCLK/8，停止计数
	fac_us = SYSCLK/8;	//系统时钟的1/8		    
	fac_ms = (uint16_t)fac_us*1000;	//ms需要的SysTick时钟数
}
/**********************************************************
* 函数功能 ---> 延时n个us
* 入口参数 ---> nus：要延时的us数
* 返回数值 ---> none
* 功能说明 ---> none
**********************************************************/		    								   
void delay_us(uint32_t nus)
{		
	uint32_t temp;
		    	 
	SysTick->LOAD = nus*fac_us; //时间加载	  		 
	SysTick->VAL = 0x00;        //清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;      //开始倒数 	 
	do
	{
		temp = SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));	//等待时间到达   
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL = 0x00;	//清空计数器	 
}
/**********************************************************
* 函数功能 ---> 延时n个ms
* 入口参数 ---> nus：要延时的us数
* 返回数值 ---> none
* 功能说明 ---> SysTick->LOAD为24位寄存器,所以,最大延时为:
*               nms <= 0xffffff*8*1000/SYSCLK
*               SYSCLK单位为Hz,nms单位为ms
*               注意nms的范围 0 ~ 1864(72M情况下)
**********************************************************/ 
void delay_ms(uint16_t nms)
{	 		  	  
	uint32_t temp;
			   
	SysTick->LOAD = (uint32_t)nms*fac_ms;	//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL = 0x00;           //清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;      //开始倒数  
	do
	{
		temp = SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));	//等待时间到达   
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL = 0x00;	//清空计数器	  	    
}

