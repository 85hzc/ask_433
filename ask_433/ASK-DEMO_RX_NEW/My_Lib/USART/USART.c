/*************************************
                今明电子
凡在本店购买满30元，下次购买即可返还5角；
      满50元，下次购买即可返还1元；
     满100元，下次购买即可返还2元；
     满150元，下次购买即可返还4元；
      满200元，下次购买即可免邮
http://shop106001793.taobao.com/search.htm?spm=a1z10.5.w5002-3375901029.1.l1TXSl&search=y
              stm8s105
               V1.1
             2013.9.8
**************************************/

#include "USART.h"
#include <stdio.h>
#include "ALL_Includes.h"


/**************************
函数功能：初始化UART
输入参数：SYS_Clk:系统时钟（2,4,8,16）
         baud：   波特率
输出参数：无
返 回 值：无
备    注：无
***************************/
void Uart_Init(u8 SYS_Clk, u32 baud)
{   
    u16 UART_Temp;
  
    UART_IOConfig();//UART IO引脚初始化 
    
    UART1_CR2 = 0;// 禁止UART发送和接收
    UART1_CR1 = 0;// b5 = 0,允许UART  b2 = 0,禁止校验
    
    UART1_CR3 = 0;// b5,b4 = 00,1个停止位
    
/************************************************** 
    设置波特率，必须注意以下几点：
    (1) 必须先写BRR2
    (2) BRR1存放的是分频系数的第11位到第4位，
    (3) BRR2存放的是分频系数的第15位到第12位，和第3位到第0位
    例如对于波特率位9600时，分频系数=2000000/9600=208
    对应的十六进制数为00D0，BBR1=0D,BBR2=00
*************************************************/ 
    
    UART_Temp = SYS_Clk*1000000/baud;
    
    UART1_BRR2 = (u8)((UART_Temp&0x000F)|((UART_Temp&0xF000)>>8));
    UART1_BRR1 = (u8)((UART_Temp&0x0FF0)>>4);

    /*//rx/tx
    UART1_CR2 = 0x2C; // b3 = 1,允许发送
                       // b2 = 1,允许接收
                       // b5 = 1,允许产生接收中断 
    */
    //tx
    UART1_CR2 = 0x08; // b3 = 1,允许发送
                       // b2 = 1,允许接收
                       // b5 = 1,允许产生接收中断 
}

/**************************************
函数功能：从UART3发送一个字符
输入参数：ch -- 要发送的字符
输出参数：无
返 回 值：无
备    注：无
***************************************/
void Uart_Sendbyte(unsigned char ch)
{
     while((UART1_SR & 0x80) == 0x00);  // 若发送寄存器不空，则等待
     UART1_DR = ch;                     // 将要发送的字符送到数据寄存器  
}
/////////////////////串口发射多个字节////////////////
void Uart_Senddata(unsigned char* st, unsigned char len)
{
  unsigned char i=0;
  for(i=0; i<len; i++)
  {
    Uart_Sendbyte(st[i]);
  }
}
/***********************************
函数功能：UART IO口初始化
输入参数：无
输出参数：无
备   注：ＩＯ在输出模式下，可通过ＣＲ２寄存器
         控制输出速率
***********************************/
void UART_IOConfig(void)
{ 
    PD_DDR |= TXPin;//输出模式
    PD_CR1 |= TXPin;//推挽输出   
    
    PD_DDR &=~RXPin;//输入模式
    PD_CR1 &=~RXPin;//浮空输入
}

#if 0
/***********************************************
函数功能: 重定义fputc函数
备   注：使用printf需重定义fputc函数，并且
        修改在General Options 中的Library Configuration
        和Library Options
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
  Uart_Sendbyte(ch);
}


