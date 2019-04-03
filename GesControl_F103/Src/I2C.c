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
*        ����SCL�ڵ����״̬
*    param:
*        int flag :  0 : ���Ϊ0
*                1 : ���Ϊ1
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
*        ��ȡSDA�ڵ�״ֵ̬
*    param:
*        void
*    return:    
*        int : IO��״̬
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
*        ����SDA�ڵ����״̬
*    param:
*        int flag :  0 : ���Ϊ0
*                1 : ���Ϊ1
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
//    SHT_set_scl_dir(1);//SCL���ó����

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
*        ����IIC 
*    param:
*        void
*    return:    
*        void
*****************************************************************************/
void I2C_start(void)
{
    I2C_set_scl_state(1);//��SCLΪ�ߵ�ƿ��ʱ��
    
    I2C_set_sda_state(1);//SDA��һ���½���
    I2C_delay_1us(2);
    I2C_set_sda_state(0);

    I2C_delay_1us(2);    
    
//    I2C_SDA_OUTPUT();
////    SHT_set_sda_dir(1);
//    I2C_SCL_1();
////    SHT_set_scl_state(1);//��SCLΪ�ߵ�ƿ��ʱ��
//    I2C_SDA_1();
////    SHT_set_sda_state(1);//SDA��һ���½���
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
*        ֹͣIIC
*    param:
*        void
*    return:    
*        void
*****************************************************************************/
void I2C_stop(void)
{
	I2C_set_sda_state(0);
    I2C_delay_1us(2);
	
    I2C_set_scl_state(1);//��SCLΪ�ߵ�ƿ��ʱ��
    I2C_delay_1us(2);
    I2C_set_sda_state(1);//SDA��һ��������

    I2C_delay_1us(2);   
    
//    I2C_SDA_OUTPUT();
////    SHT_set_sda_dir(1);
//    I2C_SDA_0();    //TEST
//    I2C_delay();    //TEST 
//    
//    I2C_SCL_1();
////    SHT_set_scl_state(1);//��SCLΪ�ߵ�ƿ��ʱ��
//    
////    I2C_SDA_0();
////    SHT_set_sda_state(0);//SDA��һ��������
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
*        �ȴ�IIC��Ӧ
*    param:
*        void
*    return:    
*        int ���ض�ȡ
*****************************************************************************/
uint8_t I2C_waitAck(void)
{
    int overTime = 255;
    
//    SHT_set_sda_dir(0);//SDA���ó�����
    I2C_set_sda_state(1);
    I2C_delay_1us(1);
    I2C_set_scl_state(0);
    I2C_delay_1us(1);
    I2C_set_scl_state(1);//�����ز�
    I2C_delay_1us(1);
    while(I2C_get_sda_state())//�ȴ�SDA�����͵�ƽ
    {
        //��ȡ�����ϵ�ACK�ź�
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
//        //��ȡ�����ϵ�ACK�ź�
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
*        ������Ӧ
*    param:
*        void
*    return:    
*        void
*****************************************************************************/
void I2C_sendAck(void)
{
    I2C_set_sda_state(0);//��SCL���أ�sda���Ϊ��
    I2C_delay_1us(2);
    I2C_set_scl_state(1);
    I2C_delay_1us(2);
    I2C_set_scl_state(0);
    I2C_delay_1us(2);
    I2C_set_sda_state(1);
    
////    SHT_set_sda_dir(1);//SDA���ó����
//    I2C_SDA_OUTPUT();

//    I2C_SDA_0();
////    SHT_set_sda_state(0);//��SCL���أ�sda���Ϊ��
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
*        ���ͷ���Ӧ
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
////    SHT_set_sda_dir(1);//SDA���ó����

////    SHT_set_sda_state(1);//��SCL���أ�sda���Ϊ��
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
*        ����һ���ֽ�
*    param:
*        unsigned char byte : ��Ҫ���͵�һ���ֽ�
*    return:    
*        void
*****************************************************************************/
void I2C_sendByte(uint8_t byte)
{
    int i = 0;

//    SHT_set_sda_dir(1);//���ó�����ķ���
    
    //ѭ��һ���ֽ�
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
////    SHT_set_sda_dir(1);//���ó�����ķ���
//    
//    //ѭ��һ���ֽ�
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
*        ����һ���ֽ�
*    param:
*        void
*    return:    
*        unsigned char : ���յ���һ���ֽ�
*****************************************************************************/
uint8_t I2C_recvByte(void)
{
    uint8_t i = 0;
    uint8_t data = 0;

//    SHT_set_sda_dir(0);//���ó�����ķ���
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
////    SHT_set_sda_dir(0);//���ó�����ķ���
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
*        1us����ʱ����(32MHzʱ��)��1000us�����Ƚ�׼ȷ��1us����Ϊ��Լ2us,5us����Լ5us
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
