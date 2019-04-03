/**
  ******************************************************************************
  * File Name          : VCNL4035.c
  * Description        : Driver for VCNL4035X1
  ******************************************************************************
  *Author:	JL
  *Date:	2019/1/18
  ******************************************************************************
  **/
  
#include "VCNL4035.h"
#include "I2C.h"
#include "stdio.h"

uint8_t VCNL4035_SendData(uint8_t command, uint16_t val)
{
	uint8_t val_H,val_L;
	
	val_H=(uint8_t)((val>>8)&0x00FF);
	val_L=(uint8_t)(val&0x00FF);
	
	I2C_start();
	I2C_sendByte(VCNL4035_COMM_W);	
	if (I2C_waitAck())
	{
		I2C_sendByte(command);  
		if (I2C_waitAck())
		{
			I2C_sendByte(val_L);
			if (I2C_waitAck())
			{
				I2C_sendByte(val_H);
				if (I2C_waitAck())
				{
					I2C_stop();
					return 1;
				}
			}
		}
	}
	return 0;
}

uint8_t VCNL4035_ReadData(uint8_t command, uint16_t *pval)
{
	uint8_t val_H,val_L;
	
	I2C_start();
	I2C_sendByte(VCNL4035_COMM_W);	
	if (I2C_waitAck())
	{
		I2C_sendByte(command);  
		if (I2C_waitAck())
		{
			I2C_start();
			I2C_sendByte(VCNL4035_COMM_R);	
			
			if (I2C_waitAck())
			{
				val_L=I2C_recvByte();
				I2C_sendAck(); 
				val_H=I2C_recvByte();
                I2C_sendNoAck();
                I2C_stop();
				*pval=((((uint16_t)val_H)<<8)&0xFF00)+(uint16_t)val_L;
                return 1;				
			}
		}
	}
	return 0;	
}

//连续读取三通道值
uint8_t VCNL4035_ReadPS(uint16_t *ps)
{
	uint8_t val_L,val_H;
	I2C_start();
	I2C_sendByte(VCNL4035_COMM_R);	
	
	if (I2C_waitAck())
	{	
		//数据1
		val_L=I2C_recvByte();
		I2C_sendAck(); 
		val_H=I2C_recvByte();
		I2C_sendAck();
		ps[0]=((((uint16_t)val_H)<<8)&0xFF00)+(uint16_t)val_L;
		//数据2
		val_L=I2C_recvByte();
		I2C_sendAck(); 
		val_H=I2C_recvByte();
		I2C_sendAck();
		ps[1]=((((uint16_t)val_H)<<8)&0xFF00)+(uint16_t)val_L;	
		//数据3
		val_L=I2C_recvByte();
		I2C_sendAck(); 
		val_H=I2C_recvByte();
		I2C_sendNoAck();
		ps[2]=((((uint16_t)val_H)<<8)&0xFF00)+(uint16_t)val_L;		
		I2C_stop();
		return 1;				
	}	
	return 0;
}

//以手势模式初始化
void VCNL4035_InitGesMode(void)
{
	uint8_t res;
	uint16_t read_val;
	res=VCNL4035_SendData(VCNL4035_PS_CONF1_2,VCNL4035_PS_CONF1_2_DEF);
	res=VCNL4035_ReadData(VCNL4035_PS_CONF1_2,&read_val);
	LOG_DEBUG("Read res: %d, PS CONF_1_2: 0x%04x\n",res,read_val);	
	res=VCNL4035_SendData(VCNL4035_PS_CONF3_MS,VCNL4035_PS_CONF3_MS_DEF);
	res=VCNL4035_ReadData(VCNL4035_PS_CONF3_MS,&read_val);
	LOG_DEBUG("Read res: %d, PS CONF_3_MS: 0x%04x\n",res,read_val);		
	
}

//开始手势检测
void VCNL4035_StartGesDetect(void)
{
	VCNL4035_SendData(VCNL4035_PS_CONF3_MS,(VCNL4035_PS_CONF3_MS_DEF|VCNL4035_PS_TRIG_MASK));
}

//通过读取INT_Flag寄存器清空INT标志
void VCNL4035_ClearInt(void)
{
	uint16_t read_val;
//	uint8_t res;
	VCNL4035_ReadData(VCNL4035_INT_FLAG, &read_val);
//	LOG_DEBUG("Read res: %d, INT_FLAG: 0x%04x\n",res,read_val);	
}
