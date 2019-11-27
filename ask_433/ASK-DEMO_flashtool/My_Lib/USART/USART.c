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

extern FM24C_Data_S fm24c_data;
extern uint8_t fm24c_store_addr;
unsigned char rxlen=0,rxBuff[64],txBuff[64],uart_flag=0;

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

    ///rx/tx
    UART1_CR2 = 0x2C; // b3 = 1,允许发送
                       // b2 = 1,允许接收
                       // b5 = 1,允许产生接收中断 
    
    //tx
    //UART1_CR2 = 0x08; // b3 = 1,允许发送
                       // b2 = 1,允许接收
                       // b5 = 1,允许产生接收中断 
}


void Uart_compose(unsigned char ch)
{
    //if(rxlen<128)
    //{
        rxBuff[rxlen++] = ch;
        //Uart_Sendbyte(rxlen);
        //Uart_Sendbyte(ch);
        //if(rxlen>=5 && ch=='\0')
        if(rxlen>=24)
        {
            rxlen = 0;
            uart_flag = 1;
        }
    //}
    //else
    //{
    //    rxlen = 0;
    //    uart_flag = 1;
    //}
    
}

void Uart_sendEEpromInfo()
{
    memset(&txBuff, 0, sizeof(txBuff));

    txBuff[0] = fm24c_data.cardType;
    txBuff[1] = fm24c_data.instNum;
    txBuff[2] = fm24c_data.assoAddr[0];
    txBuff[3] = fm24c_data.assoAddr[1];

    txBuff[4] = fm24c_data.dev.devType;
    txBuff[5] = fm24c_data.dev.devAddr;

    txBuff[6] = fm24c_data.dev.key[0].keyType;
    txBuff[7] = fm24c_data.dev.key[0].scene;

    txBuff[8] = fm24c_data.dev.key[1].keyType;
    txBuff[9] = fm24c_data.dev.key[1].scene;

    txBuff[10] = fm24c_data.dev.key[2].keyType;
    txBuff[11] = fm24c_data.dev.key[2].scene;

    txBuff[12] = fm24c_data.dev.key[3].keyType;
    txBuff[13] = fm24c_data.dev.key[3].scene;

    txBuff[14] = fm24c_data.dev.key[4].keyType;
    txBuff[15] = fm24c_data.dev.key[4].scene;

    txBuff[16] = fm24c_data.dev.key[5].keyType;
    txBuff[17] = fm24c_data.dev.key[5].scene;

    txBuff[18] = fm24c_data.dev.key[6].keyType;
    txBuff[19] = fm24c_data.dev.key[6].scene;

    txBuff[20] = fm24c_data.dev.key[7].keyType;
    txBuff[21] = fm24c_data.dev.key[7].scene;

    Uart_Senddata(txBuff, 22);
}

void Uart_SetDevInfo(uint8_t devId)
{
    uint8_t page[FM24C_PAGE_SIZE];

    memset(&fm24c_data, 0, sizeof(FM24C_Data_S));


    fm24c_data.cardType = rxBuff[2];
    fm24c_data.instNum = rxBuff[3];
    fm24c_data.assoAddr[0] = rxBuff[4];
    fm24c_data.assoAddr[1] = rxBuff[5];

    fm24c_data.dev.devType = rxBuff[6];
    fm24c_data.dev.devAddr = rxBuff[7];

    fm24c_data.dev.key[0].keyType = rxBuff[8];
    fm24c_data.dev.key[0].scene = rxBuff[9];

    fm24c_data.dev.key[1].keyType = rxBuff[10];
    fm24c_data.dev.key[1].scene = rxBuff[11];

    fm24c_data.dev.key[2].keyType = rxBuff[12];
    fm24c_data.dev.key[2].scene = rxBuff[13];

    fm24c_data.dev.key[3].keyType = rxBuff[14];
    fm24c_data.dev.key[3].scene = rxBuff[15];

    fm24c_data.dev.key[4].keyType = rxBuff[16];
    fm24c_data.dev.key[4].scene = rxBuff[17];

    fm24c_data.dev.key[5].keyType = rxBuff[18];
    fm24c_data.dev.key[5].scene = rxBuff[19];

    fm24c_data.dev.key[6].keyType = rxBuff[20];
    fm24c_data.dev.key[6].scene = rxBuff[21];

    fm24c_data.dev.key[7].keyType = rxBuff[22];
    fm24c_data.dev.key[7].scene = rxBuff[23];

    delay_ms(20);
    fm24c_store_addr=0;
    memset(&page, 0, sizeof(page));
    page[0] = fm24c_data.cardType;
    page[1] = fm24c_data.instNum;
    page[2] = fm24c_data.assoAddr[0];
    page[3] = fm24c_data.assoAddr[1];
    FM24C_WriteData(page);

    delay_ms(20);
    fm24c_store_addr=0x10;
    memset(&page, 0, sizeof(page));
    page[0] = fm24c_data.dev.devType;
    page[1] = fm24c_data.dev.devAddr;
    FM24C_WriteData(page);
    delay_ms(20);

    fm24c_store_addr=0x20;
    memset(&page, 0, sizeof(page));
    page[0] = fm24c_data.dev.key[0].keyType;
    page[1] = fm24c_data.dev.key[0].scene;
    FM24C_WriteData(page);
    delay_ms(20);

    fm24c_store_addr=0x28;
    memset(&page, 0, sizeof(page));
    page[0] = fm24c_data.dev.key[1].keyType;
    page[1] = fm24c_data.dev.key[1].scene;
    FM24C_WriteData(page);
    delay_ms(20);

    fm24c_store_addr=0x30;
    memset(&page, 0, sizeof(page));
    page[0] = fm24c_data.dev.key[2].keyType;
    page[1] = fm24c_data.dev.key[2].scene;
    FM24C_WriteData(page);
    delay_ms(20);

    fm24c_store_addr=0x38;
    memset(&page, 0, sizeof(page));
    page[0] = fm24c_data.dev.key[3].keyType;
    page[1] = fm24c_data.dev.key[3].scene;
    FM24C_WriteData(page);
    delay_ms(20);

    fm24c_store_addr=0x40;
    memset(&page, 0, sizeof(page));
    page[0] = fm24c_data.dev.key[4].keyType;
    page[1] = fm24c_data.dev.key[4].scene;
    FM24C_WriteData(page);
    delay_ms(20);

    fm24c_store_addr=0x48;
    memset(&page, 0, sizeof(page));
    page[0] = fm24c_data.dev.key[5].keyType;
    page[1] = fm24c_data.dev.key[5].scene;
    FM24C_WriteData(page);
    delay_ms(20);

    fm24c_store_addr=0x50;
    memset(&page, 0, sizeof(page));
    page[0] = fm24c_data.dev.key[6].keyType;
    page[1] = fm24c_data.dev.key[6].scene;
    FM24C_WriteData(page);
    delay_ms(20);

    fm24c_store_addr=0x58;
    memset(&page, 0, sizeof(page));
    page[0] = fm24c_data.dev.key[7].keyType;
    page[1] = fm24c_data.dev.key[7].scene;
    FM24C_WriteData(page);
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

  Uart_compose(ch);
}


