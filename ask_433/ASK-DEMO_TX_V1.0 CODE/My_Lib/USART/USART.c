/*************************************
                ��������
���ڱ��깺����30Ԫ���´ι��򼴿ɷ���5�ǣ�
      ��50Ԫ���´ι��򼴿ɷ���1Ԫ��
     ��100Ԫ���´ι��򼴿ɷ���2Ԫ��
     ��150Ԫ���´ι��򼴿ɷ���4Ԫ��
      ��200Ԫ���´ι��򼴿�����
http://shop106001793.taobao.com/search.htm?spm=a1z10.5.w5002-3375901029.1.l1TXSl&search=y
              stm8s105
               V1.1
             2013.9.8
**************************************/

#include "USART.h"
#include <stdio.h>
#include "ALL_Includes.h"


/**************************
�������ܣ���ʼ��UART
���������SYS_Clk:ϵͳʱ�ӣ�2,4,8,16��
         baud��   ������
�����������
�� �� ֵ����
��    ע����
***************************/
void Uart_Init(u8 SYS_Clk, u32 baud)
{   
    u16 UART_Temp;
  
    UART_IOConfig();//UART IO���ų�ʼ�� 
  
    UART1_CR2 = 0;// ��ֹUART���ͺͽ���
    UART1_CR1 = 0;// b5 = 0,����UART  b2 = 0,��ֹУ��
                                                      
    UART1_CR3 = 0;// b5,b4 = 00,1��ֹͣλ
                            
/************************************************** 
    ���ò����ʣ�����ע�����¼��㣺
    (1) ������дBRR2
    (2) BRR1��ŵ��Ƿ�Ƶϵ���ĵ�11λ����4λ��
    (3) BRR2��ŵ��Ƿ�Ƶϵ���ĵ�15λ����12λ���͵�3λ����0λ
    ������ڲ�����λ9600ʱ����Ƶϵ��=2000000/9600=208
    ��Ӧ��ʮ��������Ϊ00D0��BBR1=0D,BBR2=00
*************************************************/ 
    
    UART_Temp = SYS_Clk*1000000/baud;
    
    UART1_BRR2 = (u8)((UART_Temp&0x000F)|((UART_Temp&0xF000)>>8));
    UART1_BRR1 = (u8)((UART_Temp&0x0FF0)>>4);
                                                                                
    UART1_CR2 = 0x2C; // b3 = 1,������
                       // b2 = 1,�������
                       // b5 = 1,������������ж� 
}

/**************************************
�������ܣ���UART3����һ���ַ�
���������ch -- Ҫ���͵��ַ�
�����������
�� �� ֵ����
��    ע����
***************************************/
void Uart_Sendbyte(unsigned char ch)
{
     while((UART1_SR & 0x80) == 0x00);  // �����ͼĴ������գ���ȴ�
     UART1_DR = ch;                     // ��Ҫ���͵��ַ��͵����ݼĴ���  
}
/////////////////////���ڷ������ֽ�////////////////
void Uart_Senddata(unsigned char* st, unsigned char len)
{
  unsigned char i=0;
  for(i=0; i<len; i++)
  {
    Uart_Sendbyte(st[i]);
  }
}
/***********************************
�������ܣ�UART IO�ڳ�ʼ��
�����������
�����������
��   ע���ɣ������ģʽ�£���ͨ���ãң��Ĵ���
         �����������
***********************************/
void UART_IOConfig(void)
{ 
    PD_DDR |= TXPin;//���ģʽ
    PD_CR1 |= TXPin;//�������   
    
    PD_DDR &=~RXPin;//����ģʽ
    PD_CR1 &=~RXPin;//��������
}

#if 0
/***********************************************
��������: �ض���fputc����
��   ע��ʹ��printf���ض���fputc����������
        �޸���General Options �е�Library Configuration
        ��Library Options
***********************************************/
int fputc(int ch, FILE *f)
{      
    while((UART1->SR&0X40)==0);   
    UART1->DR = (u8) ch;   
    
    return ch;
}
#endif

//#pragma vector=20
#pragma vector = UART1_R_RXNE_vector
__interrupt __root void UART1_Recv_IRQHandler()
{
  unsigned char ch;
  ch=UART1_DR;
  //Uart_Sendbyte(ch);
}







