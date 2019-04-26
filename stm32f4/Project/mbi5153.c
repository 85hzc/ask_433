	/***************************************************************************
	*	@file  	mbi5153.c
	*	@version V1.0.0
	*	@brief   MBI5153������غ���
   ***************************************************************************
   *  @description
	*
	*  ����ʱ��
	* 
	***************************************************************************
***/

#include "stm32f4xx.h"
#include "led.h"
#include "delay.h"
#include "key.h"
#include "usart.h"
#include "gpio.h"

#define MBI5153_SIZE							2

void soft_reset(void)
{
		unsigned int sck_cnt;

		//��λ
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
		unsigned short i,j,k,m;
		unsigned short state_reg=0x006B;
		unsigned int mask;
		
		for(j = 0; j < MBI5153_SIZE; j++)//NƬIC����
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
		//SDI_PIN_L
		delay(2);
}

void reg2_config(void)
{
		unsigned int sck_cnt;
		unsigned short i,j,k,m;
		unsigned short state_reg=0x0400;
		unsigned int mask;
		
		for(j = 0; j < MBI5153_SIZE; j++)//NƬIC����
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
		unsigned short i,j,k,m;
		unsigned short state_reg=0x0000;
		unsigned int mask;
		
		for(j = 0; j < MBI5153_SIZE; j++)//NƬIC����
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

void MBI5153()
{
		unsigned int sck_cnt;
		unsigned short i,j,k,m,sdi_data;
		//unsigned short red1,green1,blue1,red2,green2,blue2;
		unsigned int mask;
		//unsigned int bufaddA,bufaddB;

		sdi_data = 0x1234;
		//sdi_data = 0xffff;

		//ǰ��ʱ��
		pre_active();
		//д״̬�Ĵ���1
		reg1_config();

		//ǰ��ʱ��
		pre_active();
		//д״̬�Ĵ���1
		reg2_config();

		//ǰ��ʱ��
		pre_active();
		//д״̬�Ĵ���1
		reg3_config();

		//д��16*2*16����
		//for(m = 0;m < 16;m++)//����16��
		for(m = 0; m < 1; m++)//����16��
		{
				for(i = 0; i < 16; i++)//ÿ��MBI5052��16��ͨ��
				{
						delay(60);
						for(j = 0; j < MBI5153_SIZE; j++)//����IC����
						{
								for(k = 0; k < 16; k++)
								{
										mask = 0x8000 >> k;

										if(j == MBI5153_SIZE-1)
												if(k == 15){
													LE_PIN_H
												}

										if(sdi_data & mask)
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
		
		delay(5);
		//2��clk����LE������Vsync
		LE_PIN_H
		for(sck_cnt = 0;sck_cnt < 2;sck_cnt ++)
		{
				delay(1);
				DCLK_PIN_H
				delay(1);
				if(sck_cnt < 1)
				DCLK_PIN_L
		}
		LE_PIN_L
		delay(5);//LE�½�����gclk����������Ҫ��

		for(i=0;i<16;i++)
		{
				//Դ��Ŀ��Ϊ ����
				//for(sck_cnt = 0;sck_cnt < 512;sck_cnt ++)
				for(sck_cnt = 0;sck_cnt < 128;sck_cnt ++)
				{
						GCLK_PIN_H
						delay(1);
						GCLK_PIN_L
						delay(1);
				}
				
				//��513��
				GCLK_PIN_H
				delay(3);
				GCLK_PIN_L
				delay(50);
		}
		//DCLK_PIN_L
#if 0
		//��ʼ��ʾ
		//for(i = 0;i < 16; i++)
		for(i = 0;i < 1; i++)
		{
				/*
				//�����е�ƽ
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
				for(sck_cnt = 0;sck_cnt < 5;sck_cnt ++)//�ȴ�50�����������ʱ��
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