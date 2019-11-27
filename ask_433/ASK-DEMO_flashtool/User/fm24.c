/*********************************************************
//说明：FM24C操作函数
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

extern FM24C_Data_S fm24c_data;

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

    SCL = 1;
    SDA = 1;
    delay_us();
    SDA = 0;
    delay_us();
    //SCL = 0;
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
    delay_us(); 
    SCL = 1; 
    delay_us(); 
    SDA = 1; 
    delay_us(); 
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
    uint8_t overTime = 255;
    
    SET_SDA_IN();
    SDA=1;
    delay_us(); 
    //SDA = 1;
    //delay_us();
    SCL=0;
    delay_us();
    SCL=1;
    delay_us();

    while (SDAM)
    {
        //读取总线上的ACK信号
        overTime--;
        if(overTime <= 0)
        {
            I2C_Stop();//
            return 0;
        }
        delay_us();
    }
    SCL=0;
    //SET_SDA_OUT();

    return 1;    
}

//--------------------------------------------------------------
// Prototype : void I2C_SendACK(uint8_t ack);
// Calls : delay_us()
// Parameters : bit ack:1-noack, 0-ack
// Description : Master device send ACK to slave device.
//--------------------------------------------------------------
void I2C_SendACK(void)
{
    SET_SDA_OUT();

    SDA = 0; 
    delay_us(); 
    SCL = 1; 
    delay_us(); 

    SCL = 0; 
    delay_us();
    SDA = 1; 
}

void I2C_SendNoACK(void)
{
    SET_SDA_OUT();

    SDA = 1;
    delay_us(); 
    SCL = 1;
    delay_us(); 
    SCL = 0;
    delay_us();
}

//--------------------------------------------------------------
// Prototype : uint8_t I2C_SendByte(uint8_t sendDAT)
// Calls : delay_us()
// Parameters : uint8_t sendDAT---data to be send
// Return Value : CY--slave ack (1---noack，0---ack)
// Description : Send one byte to I2C
//--------------------------------------------------------------
void I2C_SendByte(uint8_t sendDAT)
{
    
    uint8_t i,revack;

    SET_SDA_OUT();

    for (i= 0 ; i< 8; i++) 
    {
        SCL = 0;
        if (sendDAT & 0x80) // write data
        {
            SDA = 1;
        }
        else
        {
            SDA = 0;
        }
        delay_us();
        SCL = 1;
        delay_us();
        sendDAT <<= 1;
    }
    SCL = 0; 
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

    SET_SDA_IN();

    SDA = 1; // latch the Data port befor reading

    for (i=0; i<8; i++) 
    { 
        revDAT <<= 1;
        SCL = 0;
        delay_us();
        SCL = 1;

        if (SDAM)
        {
            revDAT |= 0x01;
        }
        else
        {
            revDAT &= 0xfe;
        }
        delay_us();
    }
    SCL = 0;

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
    I2C_SendNoACK();
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

uint8_t fm24c_store_addr=0;
//Read PCode from FM24C
uint8_t FM24C_ReadData(uint8_t *pbuf)
{
    uint8_t i;

    I2C_Start();
    I2C_SendByte(FM24C_COMM_W);
    
    if(I2C_waitAck())
    {
        I2C_SendByte(fm24c_store_addr);
        if(I2C_waitAck())
        {
            I2C_Start();
            I2C_SendByte(FM24C_COMM_R);
            if(I2C_waitAck())
            {
                for (i=0;i<FM24C_PAGE_SIZE-1;i++)
                {
                    pbuf[i]=I2C_RecvByte();
                    I2C_SendACK();              //发送应答
//                    i++;
                }
                pbuf[i]=I2C_RecvByte();
                I2C_SendNoACK();
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
        I2C_SendByte(fm24c_store_addr);
        if(I2C_waitAck())
        {
            i=0;
            do
            {
                I2C_SendByte(pbuf[i]);
                i++;
            } while (I2C_waitAck()&&(i<FM24C_PAGE_SIZE));
            I2C_Stop();
            if (i>=FM24C_PAGE_SIZE)
                return 1;
        }
    }
    return 0;
}

void FM24C_SetDevInfo(uint8_t devId)
{
  uint8_t page[FM24C_PAGE_SIZE];
  
  memset(&fm24c_data, 0, sizeof(FM24C_Data_S));
  
  delay_ms(20);
  fm24c_data.cardType = 0x11;
  fm24c_data.instNum = 1;
  fm24c_data.assoAddr[0] = 0xA;
  fm24c_data.assoAddr[1] = 0xB;
  fm24c_store_addr=0;
  memset(&page, 0, sizeof(page));
  page[0] = fm24c_data.cardType;
  page[1] = fm24c_data.instNum;
  page[2] = fm24c_data.assoAddr[0];
  page[3] = fm24c_data.assoAddr[1];
  FM24C_WriteData(page);

  delay_ms(20);
  fm24c_data.dev.devType = 0x01;
  fm24c_data.dev.devAddr = 0x28;
  fm24c_store_addr=0x10;
  memset(&page, 0, sizeof(page));
  page[0] = fm24c_data.dev.devType;
  page[1] = fm24c_data.dev.devAddr;
  FM24C_WriteData(page);
  delay_ms(20);
  
  fm24c_data.dev.key[0].keyType = LIGHT_ON;
  fm24c_data.dev.key[1].keyType = LIGHT_OFF;
  fm24c_data.dev.key[2].keyType = LIGHT_UP;
  fm24c_data.dev.key[3].keyType = LIGHT_DOWN;
  fm24c_data.dev.key[4].keyType = SCENE_1;
  fm24c_data.dev.key[5].keyType = SCENE_2;
  fm24c_data.dev.key[6].keyType = SCENE_3;
  fm24c_data.dev.key[7].keyType = SCENE_4;
  
  fm24c_data.dev.key[0].scene = 0;
  fm24c_data.dev.key[1].scene = 0;
  fm24c_data.dev.key[2].scene = 0;
  fm24c_data.dev.key[3].scene = 0;
  fm24c_data.dev.key[4].scene = 30;
  fm24c_data.dev.key[5].scene = 50;
  fm24c_data.dev.key[6].scene = 70;
  fm24c_data.dev.key[7].scene = 90;
  
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
  delay_ms(20);
}

void FM24C_ReadDevInfo(uint8_t devId)
{
  uint8_t page[FM24C_PAGE_SIZE];

  memset(&fm24c_data, 0, sizeof(FM24C_Data_S));
  
  delay_ms(10);
  fm24c_store_addr=0;
  memset(page,0,sizeof(page));
  FM24C_ReadData(page);
  fm24c_data.cardType = page[0];
  fm24c_data.instNum = page[1];
  fm24c_data.assoAddr[0] = page[2];
  fm24c_data.assoAddr[1] = page[3];
  
  delay_ms(20);
  fm24c_store_addr=0x10;
  memset(page,0,sizeof(page));
  FM24C_ReadData(page);
  fm24c_data.dev.devType = page[0];
  fm24c_data.dev.devAddr = page[1];

  delay_ms(20);
  fm24c_store_addr=0x20;
  memset(page,0,sizeof(page));
  FM24C_ReadData(page);
  fm24c_data.dev.key[0].keyType = page[0];
  fm24c_data.dev.key[0].scene = page[1];
  delay_ms(20);

  fm24c_store_addr=0x28;
  memset(page,0,sizeof(page));
  FM24C_ReadData(page);
  fm24c_data.dev.key[1].keyType = page[0];
  fm24c_data.dev.key[1].scene = page[1];
  delay_ms(20);

  fm24c_store_addr=0x30;
  memset(page,0,sizeof(page));
  FM24C_ReadData(page);
  fm24c_data.dev.key[2].keyType = page[0];
  fm24c_data.dev.key[2].scene = page[1];
  delay_ms(20);

  fm24c_store_addr=0x38;
  memset(page,0,sizeof(page));
  FM24C_ReadData(page);
  fm24c_data.dev.key[3].keyType = page[0];
  fm24c_data.dev.key[3].scene = page[1];
  delay_ms(20);

  fm24c_store_addr=0x40;
  memset(page,0,sizeof(page));
  FM24C_ReadData(page);
  fm24c_data.dev.key[4].keyType = page[0];
  fm24c_data.dev.key[4].scene = page[1];
  delay_ms(20);

  fm24c_store_addr=0x48;
  memset(page,0,sizeof(page));
  FM24C_ReadData(page);
  fm24c_data.dev.key[5].keyType = page[0];
  fm24c_data.dev.key[5].scene = page[1];
  delay_ms(20);

  fm24c_store_addr=0x50;
  memset(page,0,sizeof(page));
  FM24C_ReadData(page);
  fm24c_data.dev.key[6].keyType = page[0];
  fm24c_data.dev.key[6].scene = page[1];
  delay_ms(20);

  fm24c_store_addr=0x58;
  memset(page,0,sizeof(page));
  FM24C_ReadData(page);
  fm24c_data.dev.key[7].keyType = page[0];
  fm24c_data.dev.key[7].scene = page[1];
  /*
  for(int i=0;i<8;i++)
  {
    Uart_Sendbyte(fm24c_data.dev.key[i].keyType);
    Uart_Sendbyte(fm24c_data.dev.key[i].scene);
  }*/
}

