
#include "main.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"

volatile uint16_t I2C_SDA_PIN = SDA_Pin;
volatile uint16_t I2C_SCL_PIN = SCL_Pin;

GPIO_InitTypeDef        GPIO_InitStructure;  
   
#define BYTE uint8_t


void delay_us(void)
{ 
    __ASM("nop");
    //__ASM("nop");
    //__ASM("nop");
    //__ASM("nop");
}

void delayus(unsigned long time)
{
    unsigned int i;
    while(time--)
        for(i=1;i>0;i--)
            delay_us();
}



/**
  * @brief  Set SDA Pin as Output Mode
  * @retval None
  */
void SDA_OUT()  
{  
    memset(&GPIO_InitStructure, 0, sizeof(GPIO_InitTypeDef));

    GPIO_InitStructure.Pin = I2C_SDA_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStructure);
}

/**
  * @brief  Set SDA Pin as Input Mode
  * @retval None
  */
void SDA_IN()  
{

    memset(&GPIO_InitStructure, 0, sizeof(GPIO_InitTypeDef));

    GPIO_InitStructure.Pin = I2C_SDA_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStructure);
}

/**
  * @brief  read input voltage from SDA pin
  * @retval None
  */
GPIO_PinState SDA_READ()
{
  return HAL_GPIO_ReadPin(SDA_GPIO_Port, I2C_SDA_PIN);
}

/**
  * @brief  output high form SDA pin
  * @retval None
  */
void IIC_SDA_1()
{
    HAL_GPIO_WritePin(SDA_GPIO_Port, I2C_SDA_PIN, 1);
}

/**
  * @brief  output low form SDA pin
  * @retval None
  */
void IIC_SDA_0()
{
    HAL_GPIO_WritePin(SDA_GPIO_Port, I2C_SDA_PIN, 0);
}

/**
  * @brief  output high form SCL pin
  * @retval None
  */
void IIC_SCL_1()
{
    HAL_GPIO_WritePin(SCL_GPIO_Port, I2C_SCL_PIN, 1);
}

/**
  * @brief  output LOW form SCL pin
  * @retval None
  */
void IIC_SCL_0()
{
    HAL_GPIO_WritePin(SCL_GPIO_Port, I2C_SCL_PIN, 0);
}


/**
* @brief  Simulate IIC conmunication :Create Start signal
  * @retval None
  */
void IIC_Start(void)
{
    SDA_OUT();     //sda output
    IIC_SDA_1();          
    IIC_SCL_1();
    delayus(4);
    IIC_SDA_0();   //START:when CLK is high,DATA change form high to low 
    delayus(4);
    IIC_SCL_0();   //hold scl line, prepare to transmit data
}

/**
  * @brief  Simulate IIC conmunication : Create Stop signal
  * @retval None
  */
void IIC_Stop(void)
{
    SDA_OUT();    //sda output mode
    delayus(1);
    IIC_SCL_0();
    IIC_SDA_0();  //STOP:when CLK is high DATA change form low to high
    delayus(4);
    IIC_SCL_1();
    IIC_SDA_1();  //indicate transmit over
    delayus(4);
}

/**
* @brief  Simulate IIC conmunication : wait for target device's ACK
* @retval ACK (0) : receive success
* @retval NACK(1) : receive unsuccess
  */
BYTE IIC_Wait_Ack(void)
{
    BYTE ucErrTime = 0;
    IIC_SDA_1();
    delayus(1);
    SDA_IN();      //set as input mode
    delayus(1);
    IIC_SCL_1();
    delayus(1);
    while(SDA_READ())
    {
        ucErrTime++;
        if(ucErrTime > 250)
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_SCL_0(); //release scl line
    return 0;  
} 

/**
  * @brief  Simulate IIC conmunication : make an ACK
  * @retval None
  */
void IIC_Ack(void)
{
    IIC_SCL_0();
    SDA_OUT();
    IIC_SDA_0();
    delayus(4);
    IIC_SCL_1();
    delayus(4);
    IIC_SCL_0();
}

/**
  * @brief  Simulate IIC conmunication : don't make an ACK
  * @retval None
  */
void IIC_NAck(void)
{
    IIC_SCL_0();
    SDA_OUT();
    IIC_SDA_1();
    delayus(4);
    IIC_SCL_1();
    delayus(4);
    IIC_SCL_0();
}   


/**
  * @brief  Simulate IIC conmunication : Transmit one byte Data
  * @param  txd: data to be transmit
  * @retval None
  */
void IIC_Send_Byte(BYTE txd)
{
    BYTE i;   
    SDA_OUT();        
    IIC_SCL_0();//push down scl  to start transmit data
    delayus(2);
    for(i = 0; i < 8; ++i)
    {
        if(txd & 0x80)
        {
            IIC_SDA_1();
        }
        else
        {
            IIC_SDA_0();
        }
        txd <<= 1;
        delayus(2);   
        IIC_SCL_1();
        delayus(2); 
        IIC_SCL_0();
        delayus(2);
    }

    IIC_Wait_Ack();//hzc +
}

/**
  * @brief  Simulate IIC conmunication : Receive one byte Data
  * @param  ack: Whether transmit ACK
  * @retval the data have been receive
  */
BYTE IIC_Read_Byte(unsigned char ack)
{
    unsigned char i, res = 0;
    SDA_IN();               //SDA input mode
    for(i = 0; i < 8; ++i )
    {
        IIC_SCL_0();
        delayus(2);
        IIC_SCL_1();
        res <<= 1;
        if(SDA_READ())
        {
          res++;
        }
        delayus(1);
    }
    if (!ack)
    {
        IIC_NAck();//SDA 1 //make NACK
    }
    else
    {
        IIC_Ack(); //SDA 0 //make ACK
    }
    return res;
}

/*
I2C读操作
addr：目标设备地址
buf：读缓冲区
len：读入字节的长度
*/
void i2c_read(unsigned char addr, unsigned char* buf, int len)
{
    //__disable_irq();

    int i;
    unsigned char t;
    IIC_Start();                        //起始条件，开始数据通信
    //发送地址和数据读写方向
    t = (addr << 1) | 1;                    //低位为1，表示读数据
    IIC_Send_Byte(t);

    //读入数据
    for (i=0; i<len; i++)
        buf[i] = IIC_Read_Byte(1);//send ack 1; nack 0

    IIC_Stop();                     //终止条件，结束数据通信

    //__enable_irq();
}

/*
I2C写操作
addr：目标设备地址
buf：写缓冲区
len：写入字节的长度
*/
void i2c_write(unsigned char addr, unsigned char* buf, int len)
{
    //__disable_irq();

    int i;
    unsigned char t;
    IIC_Start();                        //起始条件，开始数据通信
    //发送地址和数据读写方向
    t = (addr << 1) | 0;                    //低位为0，表示写数据
    IIC_Send_Byte(t);

    //写入数据
    for (i=0; i<len; i++)
        IIC_Send_Byte(buf[i]);

    IIC_Stop();                     //终止条件，结束数据通信

    //__enable_irq();
}




