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

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

static uint32_t          tickstart;
uint32_t                 turn_off;
uint8_t                  scenMode=0;
uint8_t                  runFlag;
uint8_t                  powerFlag;
uint8_t                  single_cmd[CMD_LEN_MAX];
uint16_t                 actType = 1;
uint16_t                 actTime = 100;

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef       huart1, huart2, huart3;
DMA_HandleTypeDef        hdma_usart1_rx, hdma_usart2_rx, hdma_usart3_rx;
USART_RECEIVETYPE        UsartType1, UsartType2, UsartType3;

//TIM_HandleTypeDef      htim1;
TIM_HandleTypeDef        htim3,htim2;
SPI_HandleTypeDef        hspi1,hspi2;

extern volatile uint16_t I2C_SDA_PIN;
extern volatile uint16_t I2C_SCL_PIN;
extern uint8_t           currentProgram;

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
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
void Drv_PWM_Init(void);
void Drv_PWM_Proc(void);
void I2C_init(void);
void UartControlHandle(uint8_t *key);

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
    Drv_SERIAL_Init();
}

void Drv_SERIAL_Proc(void)
{
    uint8_t *ptr;
    uint8_t key[16];
    
    //wifi
    if(UsartType1.RX_flag)
    {
        UsartType1.RX_flag = 0;
        HAL_UART_Transmit(&DEBUG_UART, UsartType1.RX_pData, UsartType1.RX_Size, 0xffff);
    }
    //ble
    if(UsartType2.RX_flag)
    {
        UsartType2.RX_flag = 0;

        ptr = UsartType2.RX_pData;

        memset(key, 0, sizeof(key));

        strncpy(key,ptr,UsartType2.RX_Size);
        UartControlHandle(key);
        HAL_UART_Transmit(&DEBUG_UART, UsartType2.RX_pData, UsartType2.RX_Size, 0xffff);
    }
    //debug
    if(UsartType3.RX_flag)
    {
        UsartType3.RX_flag = 0;

        ptr = UsartType3.RX_pData;

        memset(key, 0, sizeof(key));
        strncpy(key,ptr,UsartType3.RX_Size);
        UartControlHandle(key);
        HAL_UART_Transmit(&DEBUG_UART, UsartType3.RX_pData, UsartType3.RX_Size, 0xffff);
    }
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
        printf("Act location:0x%x 0x%x 0x%x 0x%x\r\n",
                single_cmd[0],single_cmd[1],single_cmd[2],single_cmd[3]);
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
    //MX_TIM3_Init();//for IR INT
    
#if(IR_REMOTE)
    MX_TIM2_Init();//for IR INT
    /* Initialize IR state */
    Drv_IR_Init();
#endif

#ifdef SPI_HARD
    SPI_Configuration();    //SPI初始化
#else
    SPI_GPIO_Soft_Init();
#endif
    appInit();

    printf("system start.\r\n");
    //Drv_PWM_Init();

#if 1
    HAL_UART_Receive_DMA(&huart1, UsartType1.RX_pData, RX_LEN);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    HAL_UART_Receive_DMA(&huart2, UsartType2.RX_pData, RX_LEN);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

    HAL_UART_Receive_DMA(&huart3, UsartType3.RX_pData, RX_LEN);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
#else
    HAL_UART_Receive_IT(&huart1, rxData, 5);
#endif

    printf("system init hclk:%d\r\n",HAL_RCC_GetHCLKFreq());

#if 0
    while(1){

        //SPI_CLK_L;
        //SPI_CS_L;
        SPI_MO_L;
        delayus(100);
        //SPI_CLK_H;
        //SPI_CS_H;
        SPI_MO_H;
        delayus(100);
       /*
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        Delay_ms(10);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        Delay_ms(10);*/
    }
#endif
#ifdef SPI_HARD
    SD_Init();
#else
    SDInit();
#endif
    printf("SDInit ok\r\n");
    SD_ReadFileList("");
    SD_ReadFileList("/f1");
    SD_ReadFileList("/f2");

    SD_fileCopy();           //测试函数

#if(PROJECTOR_OSRAM)
    I2C_init();
    EPLOS_config();
    //EPLOS_diag();
#endif

#if(PROJECTOR_MBI5124)
    //reg_config();
#endif
    while (1)
    {
        //Drv_PWM_Proc();
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
        else if(scenMode==3)
            MBI5124_Prompt();
        //else if(scenMode==4)
        //    MBI5124_Cartoon();
#elif(PROJECTOR_OSRAM)
        //OSRAM_play();
        //Delay_ms(500);
#elif(PROJECTOR_MCUGPIO)
        MCUGpio_X();
        //MCUGpio_Sink();
#endif

        Drv_SERIAL_Proc();
        //printf("led\r\n");

        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        Delay_ms(200);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        Delay_ms(200);
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
#endif

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    /* DMA1_Channel6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    /* DMA1_Channel6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

#if 0
/* TIM3 init function */
static void MX_TIM3_Init(void)
{

    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_OC_InitTypeDef sConfigOC;
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 7;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    //htim3.Init.RepetitionCounter = 0;
    //htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
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
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim3, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
}
#else
/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 6400-1;//799
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffffffff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
}

#if(IR_REMOTE)
/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 6400-1;//799
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}
#endif

void Drv_PWM_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    tickstart = HAL_GetTick();
}

void Drv_PWM_Proc(void)
{
    if (turn_off)
    {
        if((HAL_GetTick() - tickstart) > 1000)
        {
            turn_off = 0;
            __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, (uint32_t)100);
        }
    }
}
#endif

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
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;//GPIO_SPEED_fast
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
            UsartType1.RX_Size = RX_LEN - temp;
            UsartType1.RX_flag = 1;
            HAL_UART_Receive_DMA(huart, UsartType1.RX_pData, RX_LEN);
        } else if(huart->Instance == USART2)
        {
            UsartType2.RX_Size = RX_LEN - temp;
            UsartType2.RX_flag = 1;
            HAL_UART_Receive_DMA(huart, UsartType2.RX_pData, RX_LEN);
        } else if(huart->Instance == USART3)
        {
            UsartType3.RX_Size = RX_LEN - temp;
            UsartType3.RX_flag = 1;
            HAL_UART_Receive_DMA(huart, UsartType3.RX_pData, RX_LEN);
        }
    }
}

void UartControlHandle(uint8_t *key)
{
    uint16_t ttt;
    uint16_t IR_code = 0;

/*
    IR_code = REMOTE_MI_DOWN;
    IR_code = REMOTE_MI_LEFT;
    IR_code = REMOTE_MI_RIGHT;
    IR_code = REMOTE_MI_PLUS;
    IR_code = REMOTE_MI_MINUS;
    IR_code = REMOTE_MI_BACK;
    IR_code = REMOTE_MI_HOME;//focus+
    IR_code = REMOTE_MI_MENU;//focus-
    IR_code = REMOTE_MI_OK;
*/
    printf("key:%s\r\n",key);

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
        currentProgram++;
    }
    else if(!strcmp(key,"pre"))
    {
        if(currentProgram>0)
        {
            currentProgram--;
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
    }

    if (IR_code) {

        Drv_SERIAL_Act(SET_CODE(CMD_CODE_MASK_IR, CMD_OP_IR_CODE), IR_code);
    }

}


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
