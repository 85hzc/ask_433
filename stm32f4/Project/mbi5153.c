	/***************************************************************************
	*	@file  	mbi5153.c
	*	@version V1.0.0
	*	@brief   MBI5153驱动相关函数
   ***************************************************************************
   *  @description
	*
	*  驱动时序
	* 
	***************************************************************************
***/

#include "stm32f4xx.h"
#include "led.h"
#include "delay.h"
#include "key.h"
#include "usart.h"
#include "gpio.h"
#include "mbi5153.h"

extern uint8_t circuit;

#if(TWO_TIMER_PULSE==1)
extern uint8_t gclk_num;
#endif

//unsigned short sdi_data[16]={1<<0,1<<1,1<<2,1<<3,1<<4,1<<5,1<<6,1<<7,\
//							1<<8,1<<9,1<<10,1<<11,1<<12,1<<13,1<<14,1<<15};

unsigned short sdi_data=0x5ff;

void soft_reset(void)
{
		unsigned int sck_cnt;

		//复位
		delay(1);
		LE_PIN_H
		delay(1);
		for(sck_cnt = 10;sck_cnt > 0;sck_cnt--)
		{
				DCLK_PIN_H
				delay(1);
				DCLK_PIN_L
				delay(1);
		}
		LE_PIN_L
		delay(1);
}

void vsync(void)
{
		unsigned int sck_cnt;

#if(TWO_TIMER_PULSE==1)
		while(gclk_num!=GCLKNUM){
			delay(100);
		}
#endif

#if(TWO_TIMER_PULSE==0)
		TIM1->CR1 &= ~(0x01);
		//GCLK_PIN_L
#endif
		//TIM1->CCR1 = 0;//duty cycle
		delay(10);
		//2个clk拉高LE，发送Vsync
		LE_PIN_H
		for(sck_cnt = 0;sck_cnt < 2;sck_cnt ++)
		{
				delay(1);
				DCLK_PIN_H
				delay(1);
				DCLK_PIN_L
		}
		LE_PIN_L
		delay(10);//LE下降沿与gclk上升沿满足要求
#if(TWO_TIMER_PULSE==0)
		TIM1->CR1 |= 0x01;
#elif(TWO_TIMER_PULSE==1)
		gclk_num = 0;
		//Pulse_output(100,129);
		Pulse_output(100,10);
#endif
		//TIM1->CCR1 = 50;//duty cycle
}

void pre_active(void)
{
		unsigned int sck_cnt;
		
		LE_PIN_H
		delay(1);
		for(sck_cnt = 14;sck_cnt > 0;sck_cnt --)
		{
				DCLK_PIN_H
				delay(1);
				DCLK_PIN_L
				delay(1);
		}
		LE_PIN_L
		delay(2);
}

void reg1_config(void)
{
		unsigned int sck_cnt;
		unsigned short j;
		unsigned short state_reg=0x006B;// | SCAN_LINE_2<<8;
		unsigned int mask;
		
		for(j = 0; j < MBI5153_SIZE; j++)//N片IC级联
		{
				for(sck_cnt = 0;sck_cnt < 16;sck_cnt ++)
				{
						mask = 0x8000 >> sck_cnt;
						if(j == MBI5153_SIZE-1)
							if(sck_cnt == 12){
								LE_PIN_H
								delay(1);
							}
						
						if(state_reg & mask)
						{
							SDI_PIN_H
						}
						else
						{
							SDI_PIN_L
						}
						delay(1);
						DCLK_PIN_H
						delay(1);
						DCLK_PIN_L
				}
		}
		LE_PIN_L
		SDI_PIN_L
		delay(2);
}

void reg2_config(void)
{
		unsigned int sck_cnt;
		unsigned short j;
		unsigned short state_reg=0x0400;
		unsigned int mask;
		
		for(j = 0; j < MBI5153_SIZE; j++)//N片IC级联
		{
				for(sck_cnt = 0;sck_cnt < 16;sck_cnt ++)
				{
						mask = 0x8000 >> sck_cnt;
						if(j == MBI5153_SIZE-1)
							if(sck_cnt == 8){
								LE_PIN_H
								delay(1);
							}

						if(state_reg & mask)
						{
							SDI_PIN_H
						}
						else
						{
							SDI_PIN_L
						}
						delay(1);
						DCLK_PIN_H
						delay(1);
						DCLK_PIN_L
				}
		}
		LE_PIN_L
		SDI_PIN_L
		delay(2);
}

void reg3_config(void)
{
		unsigned int sck_cnt;
		unsigned short j;
		unsigned short state_reg=0x0000;
		unsigned int mask;
		
		for(j = 0; j < MBI5153_SIZE; j++)//N片IC级联
		{
				for(sck_cnt = 0;sck_cnt < 16;sck_cnt ++)
				{
						mask = 0x8000 >> sck_cnt;
						if(j == MBI5153_SIZE-1)
							if(sck_cnt == 0){
								LE_PIN_H
								delay(1);
							}
						
						if(state_reg & mask)
						{
							SDI_PIN_H
						}
						else
						{
							SDI_PIN_L
						}
						delay(1);
						DCLK_PIN_H
						delay(1);
						DCLK_PIN_L
				}
		}
		LE_PIN_L
		SDI_PIN_L
		delay(2);
}

void MBI_Init(void)
{
		//前置时间
		pre_active();
		//写状态寄存器1
		reg1_config();

		//前置时间
		pre_active();
		//写状态寄存器1
		reg2_config();

		//前置时间
		//pre_active();
		//写状态寄存器1
		//reg3_config();

		vsync();
}

void MBI5153()
{
		unsigned int sck_cnt;
		unsigned short i,j,k,m;
		//unsigned short red1,green1,blue1,red2,green2,blue2;
		unsigned int mask;
		//unsigned int bufaddA,bufaddB;

		//vsync();

		//写入16*2*16数据
		//for(m = 0;m < 16;m++)//共有16行
		for(m = 0; m < 1; m++)//共有16行
		{
				for(i = 0; i < 16; i++)//每个MBI5052有16个通道
				{
						delay(10);
						for(j = 0; j < MBI5153_SIZE; j++)//级联IC数量
						{
								for(k = 0; k < 16; k++)
								{
										mask = 0x8000 >> k;

										if(j == MBI5153_SIZE-1)
										{
												if(k == 15)
												{
														LE_PIN_H
												}
										}

									  if((sdi_data & mask) && (circuit%16==i))
										//if((sdi_data & mask) && (i==circuit))
											SDI_PIN_H
										else
											SDI_PIN_L

										delay(1);
										DCLK_PIN_H
										delay(1);
										DCLK_PIN_L
								}
						}
						LE_PIN_L
						SDI_PIN_L
						delay(1);
				}
		}

		vsync();

#if 0
		for(i=0;i<16;i++)
		{
				//源项目作为 消隐
				//for(sck_cnt = 0;sck_cnt < 512;sck_cnt ++)
				for(sck_cnt = 0;sck_cnt < 128;sck_cnt ++)
				{
						GCLK_PIN_H
						delay(1);
						GCLK_PIN_L
						delay(1);
				}
				
				//第513个
				GCLK_PIN_H
				delay(3);
				GCLK_PIN_L
				delay(50);
		}
#endif
		//DCLK_PIN_L
#if 0
		//开始显示
		//for(i = 0;i < 16; i++)
		for(i = 0;i < 1; i++)
		{
				/*
				//设置行电平
				if(i < 8)
						D7258_PORT -> BSRRL = D7258_EN;
				else 
						D7258_PORT -> BSRRH = D7258_EN;
				
				switch(i)
				{
						case 7:
						case 15:
								D7258_PORT_BSRR = (D7258_A | D7258_B | D7258_C)<<16;
								//D7258_PORT -> BSRRH = D7258_A | D7258_B | D7258_C;
								//D7258_PORT -> BSRRL = 0;
								break;
						case 6:
						case 14:
								D7258_PORT_BSRR = D7258_A | ((D7258_B | D7258_C)<<16);
								//D7258_PORT -> BSRRH = D7258_B | D7258_C;
								//D7258_PORT -> BSRRL = D7258_A;
						break;
						case 5:
						case 13:
								D7258_PORT_BSRR = D7258_B | ((D7258_A | D7258_C)<<16);                
								//D7258_PORT -> BSRRH = D7258_A | D7258_C;
								//D7258_PORT -> BSRRL = D7258_B;
								break;
						case 4:
						case 12:
								D7258_PORT_BSRR = D7258_A | D7258_B |((D7258_C)<<16);                
								//D7258_PORT -> BSRRH = D7258_C;
								//D7258_PORT -> BSRRL = D7258_A | D7258_B;                        
						break;
						case 3:
						case 11:
								D7258_PORT_BSRR = D7258_C | ((D7258_A | D7258_B)<<16);
								//D7258_PORT -> BSRRH = D7258_A | D7258_B;
								//D7258_PORT -> BSRRL = D7258_C;
								break;
						case 2:
						case 10:
								D7258_PORT_BSRR = D7258_A | D7258_C | (D7258_B<<16);        
								//D7258_PORT -> BSRRH = D7258_B;
								//D7258_PORT -> BSRRL = D7258_A | D7258_C;
								break;
						case 1:
						case 9:
								D7258_PORT_BSRR = D7258_B | D7258_C | (D7258_A<<16);
								//D7258_PORT -> BSRRH = D7258_A;
								//D7258_PORT -> BSRRL = D7258_B | D7258_C;
								break;
						case 0:
						case 8:
								D7258_PORT_BSRR = D7258_A | D7258_B | D7258_C;
								//D7258_PORT -> BSRRH = 0;
								//D7258_PORT -> BSRRL = D7258_A | D7258_B | D7258_C;
								break;
				}*/
				for(sck_cnt = 0;sck_cnt < 1025;sck_cnt ++)//??1024???,?1024?????? ?????1000/60/16=1.25ms
				{
						JXI5020_PORT ->BSRRL = JXI5020_SCK;
						delay(2);
						if(sck_cnt < 1025-1)
							JXI5020_PORT -> BSRRH = JXI5020_SCK;
						delay(1);
				}
				for(sck_cnt = 0;sck_cnt < 5;sck_cnt ++)//等待50个脉冲的消隐时间
				{
						JXI5020_PORT ->BSRRH  = UR14_SDO | UG14_SDO | UB14_SDO | UR13_SDO | UG13_SDO | UB13_SDO;
						JXI5020_PORT ->BSRRL = JXI5020_SCK;
						delay(1);
						//JXI5020_PORT -> BSRRH = JXI5020_SCK;
				}
				JXI5020_PORT -> BSRRH = JXI5020_SCK;
		}
#endif
}