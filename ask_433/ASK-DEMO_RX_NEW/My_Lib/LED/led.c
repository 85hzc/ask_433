#include "ALL_Includes.h"

/***********************
�������ܣ���ʼ��LED
�����������
�����������
��    ע��LED_L1, LED_L2, LED_L3, LED_L4�ֱ�ΪPC3, PA1��PA2�� PA3��
***********************/
void Led_Init(void)
{
    PD_DDR_DDR2 =1;
    PD_CR1_C12  =1;
    
    Led_off_all();
}

void Led_on(unsigned char m)
{
  switch(m)
  {
  case 1: LED_L1=1;
    break;
  default:
    break;
  }
}

void Led_off(unsigned char m)
{
  switch(m)
  {
  case 1: LED_L1=0;
    break;
  default:
    break;
  }
}

void Led_on_all()
{
  LED_L1=1;
}

void Led_off_all()
{
  LED_L1=0;
}

