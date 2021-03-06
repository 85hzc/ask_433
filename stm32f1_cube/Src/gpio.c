/***
	***************************************************************************
	*	@file  	gpio.c
	*	@version V1.0.0
	*	@brief   LED接口相关函数
   ***************************************************************************
   *  @description
	*
	*  初始化GPIO口，GCLK、DCLK、SDI、LE
	* 	
	***************************************************************************
***/
#include "main.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "delay.h"
#include "drv_ir.h"
#include "drv_serial.h"
#include "softspi.h"
#include "programs.h"
#include "config.h"


#if(PROJECTOR_FOCUS)
extern BOOL                     lightingStatus;
#endif

//LED init
void LED_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /*Configure GPIO pins : LED_LEFT_Pin LED_DOWN_Pin LED_UP_Pin LED_RIGHT_Pin 
                           LED1_Pin */
    #if(PROJECTOR_CUBE)
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    #else
    GPIO_InitStruct.Pin = LED_Pin;
    #endif
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);
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
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;    //推挽输出
    HAL_GPIO_Init(SD_CS_PORT,&GPIO_InitStruct);
    HAL_GPIO_WritePin(SD_CS_PORT,SD_CS_GPIO,GPIO_PIN_SET);//disable 


    // SPI_SCK SPI_MOSI  复用推挽输出
    GPIO_InitStruct.Pin =  SPI2_SCK | SPI2_MOSI;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(SPI2_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SPI2_PORT,SPI2_SCK|SPI2_MOSI,GPIO_PIN_SET);//disable 

    //SPI_MISO 上拉输入模式
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
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;    //推挽输出
    HAL_GPIO_Init(SD_CS_PORT,&GPIO_InitStruct);
    HAL_GPIO_WritePin(SD_CS_PORT,SD_CS_GPIO,GPIO_PIN_SET);//disable 


    // SPI_SCK SPI_MOSI  复用推挽输出
    GPIO_InitStruct.Pin =  SPI2_SCK | SPI2_MOSI;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(SPI2_PORT, &GPIO_InitStruct);

    //SPI_MISO 上拉输入模式
    GPIO_InitStruct.Pin  = SPI2_MISO;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_INPUT;
    HAL_GPIO_Init(SPI2_PORT, &GPIO_InitStruct);
}


// 函数：IO初始化
void MBI_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    //初始化引脚 MBI5153
    /*Configure GPIO pins :  */
    GPIO_InitStruct.Pin = LE_PIN|SDI_PIN|DCLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MBI_PORT, &GPIO_InitStruct);

    //初始化引脚 MBI5153
    /*Configure GPIO pins :  */
    GPIO_InitStruct.Pin = GCLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MBI_GCLK_PORT, &GPIO_InitStruct);

    //初始化引脚 MBI5153
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

#if(PROJECTOR_CUBE)
// 函数：IO初始化
void WS2801_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    //初始化引脚
    /*Configure GPIO pins :  */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);
}
#endif

#if PROJECTOR_OSRAM
void OSRAM_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /*Configure GPIO pins*/
    GPIO_InitStruct.Pin = OSRAM_EN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(OSRAM_EN_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(OSRAM_EN_PORT,OSRAM_EN,GPIO_PIN_SET);//enable eplos 

    /*Configure GPIO pins :  */
    GPIO_InitStruct.Pin = QT_CLK;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OSRAM_PORT_CLK, &GPIO_InitStruct);

    /*Configure GPIO pins :  */
    GPIO_InitStruct.Pin = Q0_SI;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OSRAM_PORT_SI0, &GPIO_InitStruct);

    /*Configure GPIO pins :  */
    GPIO_InitStruct.Pin = Q1_SI|Q2_SI|Q3_SI;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OSRAM_PORT_SI1, &GPIO_InitStruct);

    /*Configure GPIO pins :  */
    GPIO_InitStruct.Pin = QT_UPD;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(OSRAM_PORT_UPD, &GPIO_InitStruct);

    //init for motor
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, MOTOR_IN_Pin|MOTOR_BIN1_Pin|MOTOR_AIN2_Pin|MOTOR_BIN2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, MOTOR_AIN1_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = MOTOR_IN_Pin|MOTOR_BIN1_Pin|MOTOR_AIN2_Pin|MOTOR_BIN2_Pin|FAN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MOTOR_AIN1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : MOTOR_OUT_Pin */
    GPIO_InitStruct.Pin = MOTOR_OUT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(MOTOR_OUT_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);    
}
#endif

#if PROJECTOR_FOCUS
void FOCUS_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MOTOR_Focus_BIN1_Pin|MOTOR_Focus_AIN2_Pin|MOTOR_Focus_AIN1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MOTOR_Focus_BIN2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    //init for motor
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); //lighting ON
    lightingStatus = true;

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, MOTOR_Focus_BIN1_Pin|MOTOR_Focus_AIN2_Pin|MOTOR_Focus_AIN1_Pin, GPIO_PIN_RESET);
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, MOTOR_Focus_BIN2_Pin, GPIO_PIN_RESET);
}
#endif

void MCU_GPIO_Init(void)
{

}

#if CUBEPLT_SLAVE
// 函数：IO初始化
void CUBE_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    //初始化引脚
    /*Configure GPIO pins :  */
    GPIO_InitStruct.Pin = P1_PIN|P2_PIN|P3_PIN|P4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(WS_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(WS_PORT, P1_PIN|P2_PIN|P3_PIN|P4_PIN, GPIO_PIN_RESET);
}
#endif
