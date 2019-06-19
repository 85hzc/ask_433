/*********************************************************
//˵����FM24C��������
***********************************************************/
#include "ALL_Includes.h"

/////////////////
#define SCL     PB_ODR_ODR4
#define SDA     PB_ODR_ODR5
#define SDAM    PB_IDR_IDR5
#define SET_SCL_OUT() {PB_DDR_DDR4=1; PB_CR1_C14 = 1; PB_CR2_C24 = 0;}
#define SET_SDA_OUT() {PB_DDR_DDR5=1; PB_CR1_C15 = 1; PB_CR2_C25 = 0;}
#define SET_SDA_IN() {PB_DDR_DDR5=0; PB_CR1_C15 = 0; PB_CR2_C25 = 0;}
//#define I2C_SDA_VAL PB_IDR_IDR5
//--------------------------------------------------------------
// Prototype : void I2C_Start(void)
// Calls : Delay_5us()
// Description : Start Singnal
//--------------------------------------------------------------
void I2C_Init(void)
{
    //I2C_CR1 &= 0xFE; // PE=0, disable I2C module 
    SET_SCL_OUT();
    SET_SDA_OUT();
    SCL=1;
    SDA=1;
}


//--------------------------------------------------------------
// Prototype : void I2C_Start(void)
// Calls : Delay_5us()
// Description : Start Singnal
//--------------------------------------------------------------
void I2C_Start(void)
{
    // SDA 1->0 while SCL High
    SET_SDA_OUT();
    //SET_SCL_OUT();

    SDA = 1;
    SCL = 1;
    delay_us();
    SDA = 0;
    delay_us();
    SCL = 0;
}

//--------------------------------------------------------------
// Prototype : void I2C_Stop(void)
// Calls : delay_us()
// Description : Stop Singnal
//-------------------------------------------------------------- 
void I2C_Stop(void)
{
    // SDA 0->1 while SCL High
    SET_SDA_OUT();
    //SET_SCL_OUT();

    SDA = 0; 
    SCL = 1; 
    delay_us(); 
    SDA = 1; 
    delay_us(); 
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
    uint8_t overTime = 255;

/*
    delay_us(); 
    SET_SDA_IN();

    SCL = 1; 
    asm("nop");
    asm("nop");

    revack = (uint8_t)SDAM;

    delay_us(); 
    SCL = 0; 

    SET_SDA_OUT();

    delay_us(); 
    return revack;*/
    
    delay_us(); 
    SET_SDA_IN();
    //SDA = 1;
    //delay_us();
    //SCL = 0;
    //delay_us();
    SCL=1;
    delay_us();

    while (SDAM)
    {
        //��ȡ�����ϵ�ACK�ź�
        overTime--;
        if(overTime <= 0)
        {
            I2C_Stop();//
            return 0;
        }
        delay_us();
    }
    SCL=0;
    SET_SDA_OUT();

    return 1;    
}

//--------------------------------------------------------------
// Prototype : void I2C_SendACK(uint8_t ack);
// Calls : delay_us()
// Parameters : bit ack:1-noack, 0-ack
// Description : Master device send ACK to slave device.
//--------------------------------------------------------------
void I2C_SendACK(uint8_t ack)
{
    SET_SDA_OUT();
    if(ack == 0)
    {
        SDA = 0; 
    } 
    else
    {
        SDA = 1; 
    }

    SCL = 1; 
    delay_us(); 

    SCL = 0; 
    delay_us(); 
}

//--------------------------------------------------------------
// Prototype : uint8_t I2C_SendByte(uint8_t sendDAT)
// Calls : delay_us()
// Parameters : uint8_t sendDAT---data to be send
// Return Value : CY--slave ack (1---noack��0---ack)
// Description : Send one byte to I2C
//--------------------------------------------------------------
void I2C_SendByte(uint8_t sendDAT)
{
    
    uint8_t i,revack;

    SET_SDA_OUT();

    for (i= 0 ; i< 8; i++) 
    {
        SCL = 0;
        delay_us();
        if (sendDAT & 0x80) // write data
        {
            SDA = 1;
        }
        else
        {
            SDA = 0;
        }
        sendDAT <<= 1;
        SCL = 1;
        delay_us();
    }
    SCL = 0; 

/*
    delay_us(); 
    SET_SDA_IN();

    SCL = 1; 
    asm("nop");
    asm("nop");

    revack = (uint8_t)SDAM;

    delay_us(); 
    SCL = 0; 

    SET_SDA_OUT();

    delay_us(); 
    return revack;*/
}

//--------------------------------------------------------------
// Prototype : uint8_t I2C_RecvByte()
// Calls : delay_us()
// Parameters : none
// Return Value : revDAT- received data
// Description : Receive one byte from I2C
//--------------------------------------------------------------
uint8_t I2C_RecvByte()
{
    uint8_t i;
    uint8_t revDAT = 0;

    SDA = 1; // latch the Data port befor reading

    SET_SDA_IN();

    for (i=0; i<8; i++) 
    { 
        revDAT <<= 1;

        SCL = 1; 

        asm("nop");
        asm("nop");

        if (SDAM)
        {
            revDAT |= 0x01;
        }
        else
        {
            revDAT &= 0xfe;
        }
        delay_us(); 
        SCL = 0; 
        delay_us(); 
    }

    SET_SDA_OUT();

    return revDAT;
}

//--------------------------------------------------------------
// Prototype : void Write_PT2314(uint8_t wdata)
// Calls : I2C_Start(), I2C_Stop(),I2C_SendByte()
// Parameters : RegAddr-- target memory address, 
// wrdata--- data to be writing
// Description : Write one byte to target memory
//--------------------------------------------------------------
void Write_24c02(uint8_t regadd,uint8_t wdata)
{
    I2C_Start();
    I2C_SendByte(FM24C_COMM_W); // Device Addr + Write (operation) 
    I2C_SendByte(regadd);
    I2C_SendByte(wdata);
    I2C_Stop();
}
uint8_t Read_24c02(uint8_t regadd)
{
    uint8_t revdata;

    I2C_Start();
    I2C_SendByte(FM24C_COMM_W); // Device Addr + Write (operation) 
    I2C_SendByte(regadd);

    I2C_Start(); 
    I2C_SendByte(FM24C_COMM_R); // Device Addr + Read (operation) 

    revdata = I2C_RecvByte();
    I2C_SendACK(1);
    I2C_Stop();

    return revdata;
}


////////////////////////


//Init FM24C
void FM24C_Reset()
{
    uint8_t i;
    SCL=0;
    delay_us();
    SDA=1;

    for (i=0;i<9;i++)
    {
        SCL=0;
        delay_us();
        SCL=1;
        delay_us();
    }
}

//Read PCode from FM24C
uint8_t FM24C_ReadData(uint8_t *pbuf)
{
    uint8_t i;

    I2C_Start();
    I2C_SendByte(FM24C_COMM_W);
    
    if(I2C_waitAck())
    {
        I2C_SendByte(FM24C_STORE_ADDR);  
        if(I2C_waitAck())
        {
            I2C_Start();
            I2C_SendByte(FM24C_COMM_R);
            if(I2C_waitAck())
            {
                for (i=0;i<PCODE_NUM-1;i++)
                {
                    pbuf[i]=I2C_RecvByte();
                    I2C_SendACK(0);              //����Ӧ��
//                    i++;
                }
                pbuf[i]=I2C_RecvByte();
                I2C_SendACK(1);//no ack
                I2C_Stop();
                return 1;
            }
        }
    }
     return 0;    
}

//Write PCode to FM24C
uint8_t FM24C_WriteData(uint8_t *pbuf)
{
    uint8_t i;

    I2C_Start();
    I2C_SendByte(FM24C_COMM_W);
    
    if(I2C_waitAck())
    {
        I2C_SendByte(FM24C_STORE_ADDR);
        if(I2C_waitAck())
        {
            i=0;
            do
            {
                I2C_SendByte(pbuf[i]);
                i++;
            } while (I2C_waitAck()&&(i<PCODE_NUM));
            I2C_Stop();
            if (i>=PCODE_NUM)
                return 1;
        }
    }
    return 0;
}
