/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : Software I2C
  ******************************************************************************
  *Author:	JL
  *Date:	2018/11/1
  ******************************************************************************
  **/
  
#include "I2C.h"
#include "stm32f1xx_hal.h"

/*****************************************************************************
*    function:
*        void SHT_set_scl_state(uint8_t flag)
*    description:
*        设置SCL口的输出状态
*    param:
*        int flag :  0 : 输出为0
*                1 : 输出为1
*    return:    
*        void
*****************************************************************************/
void I2C_set_scl_state(uint8_t flag)
{
    if (flag)
        HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_RESET);
}

/*****************************************************************************
*    function:
*        uint8_t SHT_get_sda_state(void);
*    description:
*        读取SDA口的状态值
*    param:
*        void
*    return:    
*        int : IO的状态
*****************************************************************************/
uint8_t I2C_get_sda_state(void)
{
    if (GPIO_PIN_RESET!=HAL_GPIO_ReadPin(SDA_GPIO_Port, SDA_Pin))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/*****************************************************************************
*    function:
*        void SHT_set_sda_state(uint8_t flag)
*    description:
*        设置SDA口的输出状态
*    param:
*        int flag :  0 : 输出为0
*                1 : 输出为1
*    return:    
*        void
*****************************************************************************/
void I2C_set_sda_state(uint8_t flag)
{
    if (flag)
        HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_RESET);
}


//Init I2C pin
void I2C_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;  

    GPIO_InitStruct.Pin = SDA_Pin;    
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct); 

    GPIO_InitStruct.Pin = SCL_Pin;     
    //    GPIO_InitStruct.Pull = GPIO_PULLUP;         //test
    HAL_GPIO_Init(SCL_GPIO_Port, &GPIO_InitStruct);     
//    SHT_set_scl_dir(1);//SCL设置成输出

    I2C_set_sda_state(1);    
    I2C_set_scl_state(1);
    
//    I2C_SCL_OUTPUT();
//    I2C_SCL_DIGIT();
//    I2C_SCL_NOPULL();
//    I2C_SCL_1();
//    I2C_SDA_OUTPUT();
//    I2C_SDA_DIGIT();
//    I2C_SDA_NOPULL();
//    I2C_SDA_1();    
}

/*****************************************************************************
*    function:
*        void SHT_iic_start(void)
*    description:
*        启动IIC 
*    param:
*        void
*    return:    
*        void
*****************************************************************************/
void I2C_start(void)
{
    I2C_set_scl_state(1);//在SCL为高电瓶的时候
    
    I2C_set_sda_state(1);//SDA有一个下降沿
    I2C_delay_1us(2);
    I2C_set_sda_state(0);

    I2C_delay_1us(2);    
    
//    I2C_SDA_OUTPUT();
////    SHT_set_sda_dir(1);
//    I2C_SCL_1();
////    SHT_set_scl_state(1);//在SCL为高电瓶的时候
//    I2C_SDA_1();
////    SHT_set_sda_state(1);//SDA有一个下降沿
//    I2C_delay();
////    SHT_delay_1us(2);
//    I2C_SDA_0();
////    SHT_set_sda_state(0);

//    I2C_delay();
////    SHT_delay_1us(2);    
}

/*****************************************************************************
*    function:
*        void SHT_iic_stop(void)
*    description:
*        停止IIC
*    param:
*        void
*    return:    
*        void
*****************************************************************************/
void I2C_stop(void)
{
	I2C_set_sda_state(0);
    I2C_delay_1us(2);
	
    I2C_set_scl_state(1);//在SCL为高电瓶的时候
    I2C_delay_1us(2);
    I2C_set_sda_state(1);//SDA有一个上升沿

    I2C_delay_1us(2);   
    
//    I2C_SDA_OUTPUT();
////    SHT_set_sda_dir(1);
//    I2C_SDA_0();    //TEST
//    I2C_delay();    //TEST 
//    
//    I2C_SCL_1();
////    SHT_set_scl_state(1);//在SCL为高电瓶的时候
//    
////    I2C_SDA_0();
////    SHT_set_sda_state(0);//SDA有一个上升沿
//    I2C_delay();
////    I2C_delay();    //TEST    
////    SHT_delay_1us(2);
//    I2C_SDA_1();
////    SHT_set_sda_state(1);

//    I2C_delay();
////   SHT_delay_1us(2);    
}

/*****************************************************************************
*    function:
*        void SHT_iic_waitAck(void)
*    description:
*        等待IIC响应
*    param:
*        void
*    return:    
*        int 返回读取
*****************************************************************************/
uint8_t I2C_waitAck(void)
{
    int overTime = 255;
    
//    SHT_set_sda_dir(0);//SDA设置成输入
    I2C_set_sda_state(1);
    I2C_delay_1us(1);
    I2C_set_scl_state(0);
    I2C_delay_1us(1);
    I2C_set_scl_state(1);//上升沿波
    I2C_delay_1us(1);
    while(I2C_get_sda_state())//等待SDA被拉低电平
    {
        //读取总线上的ACK信号
        overTime--;
        if(overTime <= 0)
        {
            I2C_stop();//
            return 0;
        }
        I2C_delay_1us(1);
    }

    I2C_set_scl_state(0);

    return 1;    
    
//    uint8_t overTime = 255;
//    
//    I2C_SDA_INPUT();
//    I2C_SDA_1();
//    I2C_delay();
//    I2C_SCL_0();
//    I2C_delay();
//    I2C_SCL_1();
//    I2C_delay();
//    while (I2C_SDA_VAL())
//    {
//        //读取总线上的ACK信号
//        overTime--;
//        if(overTime <= 0)
//        {
//            I2C_stop();//
//            return 0;
//        }
//        I2C_delay();
//    }

//    I2C_SCL_0();

//    return 1;    
}

/*****************************************************************************
*    function:
*        void SHT_iic_sendAck(void)
*    description:
*        发送响应
*    param:
*        void
*    return:    
*        void
*****************************************************************************/
void I2C_sendAck(void)
{
    I2C_set_sda_state(0);//在SCL边沿，sda输出为低
    I2C_delay_1us(2);
    I2C_set_scl_state(1);
    I2C_delay_1us(2);
    I2C_set_scl_state(0);
    I2C_delay_1us(2);
    I2C_set_sda_state(1);
    
////    SHT_set_sda_dir(1);//SDA设置成输出
//    I2C_SDA_OUTPUT();

//    I2C_SDA_0();
////    SHT_set_sda_state(0);//在SCL边沿，sda输出为底
//    I2C_delay();
////    SHT_delay_1us(2);
//    I2C_SCL_1();
////    SHT_set_scl_state(1);
//    
////    SHT_set_scl_dir(1);
//    I2C_delay();
////    SHT_delay_1us(2);

////    SHT_set_scl_dir(0);
//    I2C_SCL_0();
////    SHT_set_scl_state(0);
//    I2C_delay();
////    SHT_delay_1us(2);
//    I2C_SDA_1();
////    SHT_set_sda_state(1);
}

/*****************************************************************************
*    function:
*        void SHT_iic_sendNoAck(void)
*    description:
*        发送非响应
*    param:
*        void
*    return:    
*        void
*****************************************************************************/
void I2C_sendNoAck(void)
{
    I2C_set_sda_state(1);
    I2C_delay_1us(2);
    I2C_set_scl_state(1);
    I2C_delay_1us(2);
    I2C_set_scl_state(0);
    I2C_delay_1us(2);
    
//    I2C_SDA_OUTPUT();
////    SHT_set_sda_dir(1);//SDA设置成输出

////    SHT_set_sda_state(1);//在SCL边沿，sda输出为底
////    SHT_delay_1us(2);

////    SHT_set_scl_dir(1);
////    SHT_delay_1us(2);

////    SHT_set_scl_dir(0);
//    I2C_SDA_1();
////    SHT_set_sda_state(1);
//    I2C_delay();
////    SHT_delay_1us(2);
//    I2C_SCL_1();
////    SHT_set_scl_state(1);
//    I2C_delay();
////    SHT_delay_1us(2);
//    I2C_SCL_0();
////    SHT_set_scl_state(0);
//    I2C_delay();
////    SHT_delay_1us(2);
}

/*****************************************************************************
*    function:
*        void SHT_iic_sendByte(uint8_t byte)
*    description:
*        发送一个字节
*    param:
*        unsigned char byte : 需要发送的一个字节
*    return:    
*        void
*****************************************************************************/
void I2C_sendByte(uint8_t byte)
{
    int i = 0;

//    SHT_set_sda_dir(1);//设置成输出的方向
    
    //循环一个字节
    for(i = 0; i < 8; i++)
    {
        I2C_set_scl_state(0);
        if(byte&0x80)
        {
            I2C_set_sda_state(1);
        }
        else
        {
            I2C_set_sda_state(0);
        }
        I2C_delay_1us(2);
        I2C_set_scl_state(1);
        I2C_delay_1us(2);

        byte = byte<<1;
    }

    I2C_set_scl_state(0);
    
//    uint8_t i = 0;

//    I2C_SDA_OUTPUT();
////    SHT_set_sda_dir(1);//设置成输出的方向
//    
//    //循环一个字节
//    for(i = 0; i < 8; i++)
//    {
//        I2C_SCL_0();
////        SHT_set_scl_state(0);
//        if(byte&0x80)
//        {
//            I2C_SDA_1();
////            SHT_set_sda_state(1);
//        }
//        else
//        {
//            I2C_SDA_0();
////            SHT_set_sda_state(0);
//        }
//        I2C_delay();
////        SHT_delay_1us(2);
//        I2C_SCL_1();
////        SHT_set_scl_state(1);
//        I2C_delay();
////        SHT_delay_1us(2);
//        byte<<=1;
////        byte = byte<<1;
//    }

//    I2C_SCL_0();
////    SHT_set_scl_state(0);
}

/*****************************************************************************
*    function:
*        uint8_t SHT_iic_recvByte(void)
*    description:
*        接收一个字节
*    param:
*        void
*    return:    
*        unsigned char : 接收到的一个字节
*****************************************************************************/
uint8_t I2C_recvByte(void)
{
    uint8_t i = 0;
    uint8_t data = 0;

//    SHT_set_sda_dir(0);//设置成输入的方向
    I2C_set_sda_state(1);
    
    for(i = 0; i < 8; i++)
    {
        data <<= 1;
        I2C_set_scl_state(0);
        I2C_delay_1us(2);
        I2C_set_scl_state(1);
        if(I2C_get_sda_state())
        {
            data |= 0x01;
        }
        else
        {
            data &= 0xfe;
        }        
    }
    I2C_delay_1us(2);
    I2C_set_scl_state(0);

    return data;
    
//    uint8_t i = 0;
//    uint8_t data = 0;

//    I2C_SDA_INPUT();
////    SHT_set_sda_dir(0);//设置成输入的方向
//    I2C_SDA_1();
////    SHT_set_sda_state(1);
//    
//    for(i = 0; i < 8; i++)
//    {
//        data <<= 1;
//        I2C_SCL_0();
////        SHT_set_scl_state(0);
//        I2C_delay();
////        SHT_delay_1us(2);
//        I2C_SCL_1();
////        SHT_set_scl_state(1);
//        if (I2C_SDA_VAL())
////        if(SHT_get_sda_state())
//        {
//            data |= 0x01;
//        }
//        else
//        {
//            data &= 0xfe;
//        } 
////        I2C_delay();
//    }
//    I2C_delay();
////    SHT_delay_1us(2);
//    I2C_SCL_0();
////    SHT_set_scl_state(0);

//    return data;
}

/*****************************************************************************
*    function:
*        void I2C_delay_1us(uint16_t delay);
*    description:
*        1us的延时函数(32MHz时钟)，1000us量级比较准确，1us量级为大约2us,5us量级约5us
*    param:
*        void
*    return:    
*        void
*****************************************************************************/
void I2C_delay_1us(uint16_t delay)
{
    uint16_t i,j;
    for (i = 0; i < delay; i++)
    {
        for(j=0; j<2; j++)
//        for(j=0; j<1; j++)
        {
            __ASM("NOP");
        }                 
    }			    
}
