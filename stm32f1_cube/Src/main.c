/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "mbi5153.h"
#include "delay.h"
#include "drv_ir.h"
#include "drv_serial.h"
#include "softspi.h"
#include "programs.h"
#include "config.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

uint8_t                  scenMode = 0;
uint8_t                  runFlag;
uint8_t                  powerFlag;
uint16_t                 fileTotalPhoto;  //静态图片数
uint16_t                 fileTotalFilm; //每个影片的帧数
uint8_t                  filmTotalProgram;  //Film目录下的影片个数
uint8_t                  single_cmd[CMD_LEN_MAX];
uint16_t                 actType = 1;
uint16_t                 actTime = 100;
uint8_t                  usartTxFlag = 0;

#if(PROJECTOR_CUBE)
uint8_t                  usartTxData[3];
#elif(CUBEPLT_MASTER)
uint8_t                  usartTxData[1024];
#endif

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef       huart1, huart2, huart3;
DMA_HandleTypeDef        hdma_usart1_rx, hdma_usart2_rx, hdma_usart3_rx;
DMA_HandleTypeDef        hdma_usart1_tx, hdma_usart2_tx, hdma_usart3_tx;
USART_RECEIVETYPE        UsartType1, UsartType2, UsartType3;

TIM_HandleTypeDef        htim1,htim2;
TIM_HandleTypeDef        htim3;
SPI_HandleTypeDef        hspi1,hspi2;

extern volatile uint16_t I2C_SDA_PIN;
extern volatile uint16_t I2C_SCL_PIN;

extern uint8_t           photoProgramIdx;
extern uint8_t           filmProgramIdx;
extern uint8_t           filmFrameIdx;
#if(PROJECTOR_OSRAM)
extern uint8_t           eplosSLPxen;
extern uint8_t           eplosCfgFlag;
#endif
extern PROGRAMS_TYPE_E   programsType;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define DEBUG_UART       huart3

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
extern void Drv_FAN_Init(void);
extern void Drv_FAN_Proc(void);
void I2C_init(void);
void UartDebugControlHandle(uint8_t *key);
#if PROJECTOR_CUBE
void AppControlCubeHandle(uint8_t *key);
#endif
#if PROJECTOR_OSRAM
void AppControlEplosHandle(uint8_t *key);
#endif
#if CUBEPLT_SLAVE
void AppControlCubePltHandle(uint8_t *key,uint16_t len);
#endif

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;//主模式
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;//全双工
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;//数据位为8位
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;//CPOL=0,low
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;//CPHA为数据线的第一个变化沿
    hspi1.Init.NSS = SPI_NSS_SOFT;//软件控制NSS
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;//2分频，32M/2=16MHz
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;//最高位先发送
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;//TIMODE模式关闭
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;//CRC关闭
    hspi1.Init.CRCPolynomial = 7;//默认值，无效
    if (HAL_SPI_Init(&hspi1) != HAL_OK)//初始化
    {
        Error_Handler();
    }
}

void MX_SPI2_Init(void)
{
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;//主模式
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;//全双工
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;//数据位为8位
    hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;//CPOL=0,low
    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;//CPHA为数据线的第一个变化沿
    hspi2.Init.NSS = SPI_NSS_SOFT;//软件控制NSS
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;//2分频，32M/2=16MHz
    //hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;//2分频，32M/2=16MHz

    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;//最高位先发送
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;//TIMODE模式关闭
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;//CRC关闭
    hspi2.Init.CRCPolynomial = 7;//默认值，无效
    if (HAL_SPI_Init(&hspi2) != HAL_OK)//初始化
    {
        Error_Handler();
    }
}

/* USER CODE END 0 */

void appInit(void)
{

    runFlag = 1;
    powerFlag = 1;

    //芯片配置相关参数
#if(PROJECTOR_OSRAM)
    eplosCfgFlag = 1;
    eplosSLPxen = DISABLE;
#endif
    //显示相关参数
    programsType = AUTO_ALGORITHM;
    filmProgramIdx = 0;
    filmFrameIdx = 0;
    photoProgramIdx = 0;
    
    Drv_SERIAL_Init();
//#if(PROJECTOR_OSRAM)
    //for fan or led lighting
    Drv_FAN_Init();
//#endif
}

void Drv_SERIAL_Proc(void)
{
    uint8_t *ptr;
    uint8_t key[16];

    //BLE mesh 8258/8269
    if(UsartType1.RX_flag)
    {
        UsartType1.RX_flag = 0;
        ptr = UsartType1.RX_pData;

        memset(key, 0, sizeof(key));
        strncpy(key,ptr,UsartType1.RX_Size);
        printf("rx len=%d\r\n",UsartType1.RX_Size);
        
        //UartDataHandle(rgbdata);
#if PROJECTOR_CUBE
        AppControlCubeHandle(key);
#elif(CUBEPLT_SLAVE)
        AppControlCubePltHandle(key, UsartType1.RX_Size);
#elif PROJECTOR_OSRAM
        AppControlEplosHandle(key);
#endif
        HAL_UART_Transmit(&DEBUG_UART, UsartType1.RX_pData, UsartType1.RX_Size, 0xffff);
    }

    //ble
    /*
    if(UsartType2.RX_flag)
    {
        UsartType2.RX_flag = 0;
        ptr = UsartType2.RX_pData;

        memset(key, 0, sizeof(key));
        strncpy(key,ptr,UsartType2.RX_Size);
        //HAL_UART_Transmit(&DEBUG_UART, UsartType2.RX_pData, UsartType2.RX_Size, 0xffff);
    }
    */
    
    //debug
    if(UsartType3.RX_flag)
    {
        UsartType3.RX_flag = 0;

        ptr = UsartType3.RX_pData;

        memset(key, 0, sizeof(key));
        strncpy(key,ptr,UsartType3.RX_Size);
        UartDebugControlHandle(key);
        /*
        if(key[0]==8)
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        }
        else if(key[0]==9)
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        }
        */
        //HAL_UART_Transmit(&DEBUG_UART, UsartType3.RX_pData, UsartType3.RX_Size, 0xffff);
    }

#if(CUBE_MASTER)
    if(usartTxFlag)
    {
        HAL_UART_Transmit(&huart1, usartTxData, sizeof(usartTxData), 0xffff);
        usartTxFlag = 0;
    }
    /*
    //transmit in CUBE_play
    #elif(CUBEPLT_MASTER)
    if(usartTxFlag)
    {
        HAL_UART_Transmit(&huart1, usartTxData, sizeof(usartTxData), 0xffff);
        usartTxFlag = 0;
    }*/
#endif

#if 0
    /* 1. read the uart buffer... */
    if (HAL_OK == Drv_SERIAL_Read(single_cmd, 0))
    {
        if (Drv_CMD_Handler(single_cmd) > 0)
        {
            if (CMD_HEADER_REQ == single_cmd[0])
            {
                single_cmd[0] = CMD_HEADER_RSP;
                Drv_SERIAL_Write(single_cmd, 0);
            }
        }
    }
#endif
    /* 2. read the action buffer... */
    if (HAL_OK == Drv_SERIAL_Read_Act(single_cmd))
    {
        //printf("Act location:0x%x 0x%x 0x%x 0x%x\r\n",
        //        single_cmd[0],single_cmd[1],single_cmd[2],single_cmd[3]);
        (void)Drv_CMD_Handler(single_cmd);
    }
}

int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();  //systick 1ms

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_TIM1_Init();//for PWM pulse

    //MX_TIM2_Init();

#if(IR_REMOTE)
    //MX_TIM3_Init();//for IR INT
    /* Initialize IR state */
    //Drv_IR_Init();
#endif

#ifdef SUPPORT_FATFS
#ifdef SPI_HARD
    SPI_Configuration();    //SPI初始化
#else
    SPI_GPIO_Soft_Init();
#endif
#endif
    appInit();
    drv_fan_speed(20);

    printf("system start.\r\n");

#if 1
    HAL_UART_Receive_DMA(&huart1, UsartType1.RX_pData, UART_RX_LEN);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    HAL_UART_Receive_DMA(&huart2, UsartType2.RX_pData, UART_RX_LEN);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

    HAL_UART_Receive_DMA(&huart3, UsartType3.RX_pData, UART_RX_LEN);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
#else
    HAL_UART_Receive_IT(&huart1, rxData, 5);
#endif

    printf("system init hclk:%d\r\n",HAL_RCC_GetHCLKFreq());

#ifdef SUPPORT_FATFS
#ifdef SPI_HARD
    SD_Init();
#else
    SDInit();
#endif
    printf("SDInit ok!\r\n");
    //SD_ReadFileList("");
    //SD_ReadFileList("/f1");
    //SD_ReadFileList("/f2");
    //SD_fileCopy();           //测试函数
#endif
#if 0
#if(PROJECTOR_OSRAM)
    I2C_init();
    SD_ReadPhotoFileList("/OSRAM/Photo");
    SD_ReadFilmFolderList("/OSRAM/Film");
    
    for(int i=0;i < filmTotalProgram; i++)
    {
        SD_ReadFilmFileList(i);
    }
#elif(PROJECTOR_CUBE)
    SD_ReadPhotoFileList("/WS2801/Photo");
    SD_ReadFilmFolderList("/WS2801/Film");
    
    for(int i=0;i < filmTotalProgram; i++)
    {
        SD_ReadFilmFileList(i);
    }
#elif(CUBEPLT_MASTER)
    //SD_ReadPhotoFileList("/CUBE/Photo");
    SD_ReadFilmFolderList("/CUBE/Film");
    
    for(int i=0;i < filmTotalProgram; i++)
    {
        SD_ReadFilmFileList(i);
    }
#endif
#endif
    printf("Files Ready!\r\n");

#if(PROJECTOR_MBI5124)
    //reg_config();
#endif
    while (1)
    {
#if(PROJECTOR_OSRAM)
        Drv_FAN_Proc();
#endif

#if(PROJECTOR_MBI5153)

        MBI5153_X();
        //MBI5153_play();
        //MBI5153_Sink();
#elif(PROJECTOR_MBI5124)

        if(scenMode==0)
            MBI5124_X();
        else if(scenMode==1)
            MBI5124_Sink();
        else if(scenMode==2)
            MBI5124_Play();
#elif(PROJECTOR_MCUGPIO)

        MCUGpio_X();
        //MCUGpio_Sink();
#elif(PROJECTOR_OSRAM)

        OSRAM_config();
        OSRAM_play();
        //Delay_ms(200);
        //EPLOS_status_read();
        //Delay_ms(200);
#elif(PROJECTOR_CUBE)

        WS2801_play();
#elif(PROJECTOR_CUBEPLT)

        CUBE_play();
#endif

        //Delay_ms(5);

        Drv_SERIAL_Proc();
/*
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        Delay_ms(100);

        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        Delay_ms(100);
*/
    }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }

    /**Configure the Systick interrupt time 
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/(1000*500));

    /**Configure the Systick 
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USART1 init function */
static void MX_USART2_UART_Init(void)
{

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USART1 init function */
static void MX_USART3_UART_Init(void)
{

    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}

void init_tim3PWMIO()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_InitStruct.Mode=GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pin=GPIO_PIN_6;//PA6
    GPIO_InitStruct.Pull=GPIO_NOPULL;
    GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    __HAL_AFIO_REMAP_PD01_ENABLE();
    __HAL_AFIO_REMAP_TIM1_PARTIAL();

    /*Configure GPIO pin Output Level */
    //HAL_GPIO_WritePin(GPIOA, LED_DCLK_Pin|LED_HOLD_Pin, GPIO_PIN_RESET);
    /*Configure GPIO pin Output Level */
    //HAL_GPIO_WritePin(GPIOB, SCL1_Pin|SDA1_Pin|SCL2_Pin|SDA2_Pin|SCL3_Pin|SDA3_Pin, GPIO_PIN_RESET);

#if 0
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, LED_DOWN_Pin|LED_RIGHT_Pin, GPIO_PIN_SET);
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, LED_LEFT_Pin|LED_UP_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin : INT_Pin */
    GPIO_InitStruct.Pin = INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(INT_GPIO_Port, &GPIO_InitStruct);
#endif

    LED_GPIO_Init();
    
    //init_tim3PWMIO();
#if(PROJECTOR_MBI5153||PROJECTOR_MBI5124)
    MBI_GPIO_Init();//初始化MBI驱动pin
#elif(PROJECTOR_OSRAM)
    OSRAM_GPIO_Init();
#elif(PROJECTOR_MCUGPIO)
    MCU_GPIO_Init();
#elif(PROJECTOR_CUBE)
    WS2801_GPIO_Init();
#elif(CUBEPLT_SLAVE)
    CUBE_GPIO_Init();
#endif

    /* EXTI interrupt init*/
    //HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    //HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    /* DMA1_Channel3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    /* DMA1_Channel4_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    /* DMA1_Channel5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    /* DMA1_Channel6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    /* DMA1_Channel7_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}


/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
#if(PROJECTOR_OSRAM)
  htim1.Init.Prescaler = 640-1;//1KHz
#else
  htim1.Init.Prescaler = 64-1;//10KHz
#endif
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  //htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
#if 1
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  //TIM_OCInitTypeDef TIM_OCInitStructure;

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 81;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  /*
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }*/
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}
#else
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 41;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}
#endif

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 6400-1;//0.1ms/pulse
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.RepetitionCounter = 0;
  //htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}


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
        HAL_GPIO_WritePin(SCL_GPIO_Port, I2C_SCL_PIN, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(SCL_GPIO_Port, I2C_SCL_PIN, GPIO_PIN_RESET);
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
    if (GPIO_PIN_RESET!=HAL_GPIO_ReadPin(SDA_GPIO_Port, I2C_SDA_PIN))
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
        HAL_GPIO_WritePin(SDA_GPIO_Port, I2C_SDA_PIN, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(SDA_GPIO_Port, I2C_SDA_PIN, GPIO_PIN_RESET);
}

//Init I2C pin
void I2C_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = I2C_SDA_PIN | I2C_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct);

    I2C_set_sda_state(1);
    I2C_set_scl_state(1);
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)&ch,1, 0xFFFF);
    return ch;
}

void UsartReceive_IDLE(UART_HandleTypeDef *huart)
{
    uint32_t temp;

    if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET))
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        HAL_UART_DMAStop(huart);
        temp = huart->hdmarx->Instance->CNDTR;

        if(huart->Instance == USART1)
        {
            UsartType1.RX_Size = UART_RX_LEN - temp;
            UsartType1.RX_flag = 1;
            HAL_UART_Receive_DMA(huart, UsartType1.RX_pData, UART_RX_LEN);
        } else if(huart->Instance == USART2)
        {
            UsartType2.RX_Size = UART_RX_LEN - temp;
            UsartType2.RX_flag = 1;
            HAL_UART_Receive_DMA(huart, UsartType2.RX_pData, UART_RX_LEN);
        } else if(huart->Instance == USART3)
        {
            UsartType3.RX_Size = UART_RX_LEN - temp;
            UsartType3.RX_flag = 1;
            HAL_UART_Receive_DMA(huart, UsartType3.RX_pData, UART_RX_LEN);
        }
    }
}

void UartDebugControlHandle(uint8_t *key)
{
    uint16_t ttt;
    uint16_t IR_code = 0;

    printf("UartDebugControlHandle key:%s,len=%d\r\n",key,UsartType3.RX_Size);

    if(!strcmp(key,"stop"))
    {
        //stop
        runFlag = 0;
    }
    else if(!strcmp(key,"start"))
    {
        //play
        runFlag = 1;
    }
    else if(!strcmp(key,"next"))
    {
        IR_code = REMOTE_MI_DOWN;
        photoProgramIdx++;
    }
    else if(!strcmp(key,"pre"))
    {
        if(photoProgramIdx>0)
        {
            photoProgramIdx--;
        }
        IR_code = REMOTE_MI_UP;
    }
    else if(!strcmp(key,"poweron"))
    {
        IR_code = REMOTE_MI_POWER;
        powerFlag = powerFlag==1?0:1;
    }
    else //flash time and display scens
    {
        ttt = atoi(key);
        if(ttt>15)
        {
            actTime = ttt-15;
        }
        else if(ttt<=15 && ttt>10)
        {
            if(ttt==11)
                scenMode = 0;
            else if(ttt==12)
                scenMode = 1;
            else if(ttt==13)
                scenMode = 2;
            else if(ttt==14)
                scenMode = 3;
            else if(ttt==15)
                scenMode = 4;
        }
        else
        {
            actType = ttt;
        }

        drv_fan_speed(ttt);
    }

    if (IR_code) {

        Drv_SERIAL_Act(SET_CODE(CMD_CODE_MASK_IR, CMD_OP_IR_CODE), IR_code);
    }
}

#if PROJECTOR_OSRAM
void AppControlEplosHandle(uint8_t *key)
{
    uint16_t ttt;
    uint16_t IR_code = 0;


    printf("AppControlEplosHandle key:%s\r\n",key);

    if(!strcmp(key,"stop"))
    {
        //stop
        runFlag = 0;
    }
    else if(!strcmp(key,"start"))
    {
        //play
        runFlag = 1;
    }
    else if(!strcmp(key,"next"))
    {
        IR_code = REMOTE_MI_DOWN;
        photoProgramIdx++;
    }
    else if(!strcmp(key,"pre"))
    {
        if(photoProgramIdx>0)
        {
            photoProgramIdx--;
        }
        IR_code = REMOTE_MI_UP;
    }
    else if(!strcmp(key,"poweron"))
    {
        IR_code = REMOTE_MI_POWER;
        powerFlag = powerFlag==1?0:1;
    }
    else //flash time and display scens
    {
        ttt = atoi(key);
        if(ttt>15)
        {
            actTime = ttt-15;
        }
        else if(ttt<=15 && ttt>10)
        {
            if(ttt==11)
                scenMode = 0;
        }
        else
        {
            actType = ttt;
        }
    }

    if (IR_code) {

        Drv_SERIAL_Act(SET_CODE(CMD_CODE_MASK_IR, CMD_OP_IR_CODE), IR_code);
    }
}
#endif

#if PROJECTOR_CUBE
void AppControlCubeHandle(uint8_t *key)
{
    uint16_t ttt;
    uint16_t IR_code = 0;
    uint8_t pId,fId;
    PROGRAMS_TYPE_E type;

    type = *key++;
    pId = *key++;
    fId = *key;

    printf("AppControlCubeHandle type:%d,pid:%d,fid:%d\r\n",type,pId,fId);

    switch(type)
    {
        case PHOTO:
            runFlag = 1;
            photoProgramIdx = pId;
            programsType = PHOTO;
            break;

        case FILM:
            runFlag = 1;
            filmProgramIdx = pId;
            filmFrameIdx = fId;
            programsType = FILM;
            break;

        case AUTO_ALGORITHM:
            programsType = AUTO_ALGORITHM;
            
        default:
            break;
    }
}
#endif


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
