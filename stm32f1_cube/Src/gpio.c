/***
	***************************************************************************
	*	@file  	gpio.c
	*	@version V1.0.0
	*	@brief   LED�ӿ���غ���
   ***************************************************************************
   *  @description
	*
	*  ��ʼ��GPIO�ڣ�GCLK��DCLK��SDI��LE
	* 	
	***************************************************************************
***/
#include "main.h"
#include "gpio.h"


//LED init
void LED_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /*Configure GPIO pins : LED_LEFT_Pin LED_DOWN_Pin LED_UP_Pin LED_RIGHT_Pin 
                           LED1_Pin */
    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

//spi init
void SPI_GPIO_Soft_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    //Configure PA8 pin: DET pin
    GPIO_InitStruct.Pin = SD_DET_GPIO;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(SD_DET_PORT,&GPIO_InitStruct); 

    //Configure PB12 pin: SD_CS pin
    GPIO_InitStruct.Pin = SD_CS_GPIO;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;    //�������
    HAL_GPIO_Init(SD_CS_PORT,&GPIO_InitStruct);
    HAL_GPIO_WritePin(SD_CS_PORT,SD_CS_GPIO,GPIO_PIN_SET);//disable 


    // SPI_SCK SPI_MOSI  �����������
    GPIO_InitStruct.Pin =  SPI2_SCK | SPI2_MOSI;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(SPI2_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SPI2_PORT,SPI2_SCK|SPI2_MOSI,GPIO_PIN_SET);//disable 

    //SPI_MISO ��������ģʽ
    GPIO_InitStruct.Pin  = SPI2_MISO;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    HAL_GPIO_Init(SPI2_PORT, &GPIO_InitStruct);
}

void SPI_GPIO_Hard_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    //Configure PA8 pin: DET pin
    GPIO_InitStruct.Pin = SD_DET_GPIO;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(SD_DET_PORT,&GPIO_InitStruct); 

    //Configure PB12 pin: SD_CS pin
    GPIO_InitStruct.Pin = SD_CS_GPIO;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;    //�������
    HAL_GPIO_Init(SD_CS_PORT,&GPIO_InitStruct);
    HAL_GPIO_WritePin(SD_CS_PORT,SD_CS_GPIO,GPIO_PIN_SET);//disable 


    // SPI_SCK SPI_MOSI  �����������
    GPIO_InitStruct.Pin =  SPI2_SCK | SPI2_MOSI;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(SPI2_PORT, &GPIO_InitStruct);

    //SPI_MISO ��������ģʽ
    GPIO_InitStruct.Pin  = SPI2_MISO;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_INPUT;
    HAL_GPIO_Init(SPI2_PORT, &GPIO_InitStruct);
}


// ������IO��ʼ��
void MBI_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    //��ʼ������ MBI5153
    /*Configure GPIO pins :  */
    GPIO_InitStruct.Pin = LE_PIN|SDI_PIN|DCLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MBI_PORT, &GPIO_InitStruct);

    //��ʼ������ MBI5153
    /*Configure GPIO pins :  */
    GPIO_InitStruct.Pin = GCLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MBI_GCLK_PORT, &GPIO_InitStruct);

    //��ʼ������ MBI5153
    /*Configure GPIO pins :  */
    GPIO_InitStruct.Pin = AG_CLK_PIN|AG_DIN_PIN|AG_OE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(AG_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(MBI_GCLK_PORT, GCLK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MBI_PORT, LE_PIN|SDI_PIN|DCLK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AG_PORT, AG_CLK_PIN|AG_DIN_PIN|AG_OE_PIN, GPIO_PIN_RESET);
}

void OSRAM_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /*Configure GPIO pins*/
    GPIO_InitStruct.Pin = QUADRANT_EN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(QUADRANT_EN_PORT, &GPIO_InitStruct);

    /*Configure GPIO pins :  */
    GPIO_InitStruct.Pin = QT_CLK|Q0_SI|Q1_SI|Q2_SI|Q3_SI|QT_UPD;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OSRAM_PORT, &GPIO_InitStruct);
}

void MCU_GPIO_Init(void)
{

}

