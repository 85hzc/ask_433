//=============================================================================
//�ļ����ƣ�main.c
//���ܸ�Ҫ��STM32F051R8���̡�LED���Գ���
//��Ȩ���У�Դ�ع�����www.vcc-gnd.com
//�Ա����꣺http://vcc-gnd.taobao.com
//����ʱ�䣺2013-11-20
//���Է�ʽ��J-Link OB ARM SWD
//=============================================================================

//ͷ�ļ�
#include "stm32f0xx.h"
#include "BSP_LED.h"			// LED����ģ��ͷ�ļ�

//��������


//��������
//void GPIO_Configuration(void);
//=============================================================================
//�ļ����ƣ�Delay
//���ܸ�Ҫ����ʱ����
//����˵������
//�������أ���
//=============================================================================
void  Delay (uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}


//=============================================================================
//�ļ����ƣ�main
//���ܸ�Ҫ��������
//����˵������
//�������أ�int
//=============================================================================
int main(void)
{
//	unsigned char i;
	 /* LED��ʼ�� */
  LED_Init();

// 	GPIO_SetBits(GPIOA,GPIO_Pin_1);
// 	GPIO_SetBits(GPIOA,GPIO_Pin_2);
// 	 
// 	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
//   for(i=0;i<6;i++) Delay(0xfffff);
// 		GPIO_SetBits(GPIOC,GPIO_Pin_13);
//   for(i=0;i<6;i++) Delay(0xfffff);
	
  while(1)
  {     	
 //  LED1_TURN;
//   LED2_TURN; 
 //  LED3_TURN;
 //  LED4_TURN;
 //  LED5_TURN;
 //  LED6_TURN;
// 		GPIO_SetBits(GPIOC,GPIO_Pin_13);
//    Delay(0xfffff);
// 		GPIO_ResetBits(GPIOC,GPIO_Pin_13);
//    Delay(0xfffff);
		
		GPIO_Write(GPIOA,0XFFFF);
		GPIO_Write(GPIOB,0XFFFF);
		GPIO_Write(GPIOC,0XFFFF);
		GPIO_Write(GPIOD,0XFFFF);
		GPIO_Write(GPIOF,0XFFFF);
		Delay(0x200000);
		
		GPIO_Write(GPIOA,0X0000);
		GPIO_Write(GPIOB,0X0000);
		GPIO_Write(GPIOC,0X0000);
		GPIO_Write(GPIOD,0X0000);
		GPIO_Write(GPIOF,0X0000);
		Delay(0x200000); 
    }
}


/*****END OF FILE****/
