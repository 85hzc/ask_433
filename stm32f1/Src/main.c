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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "App.h"

/* USER CODE END Includes */

static uint32_t tickstart;
uint32_t        turn_off;

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
//TIM_HandleTypeDef  htim1;

extern volatile uint16_t I2C_SDA_PIN;
extern volatile uint16_t I2C_SCL_PIN;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define DEBUG_UART          huart1

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  //MX_TIM1_Init();//for pwm schedule

  /* USER CODE BEGIN 2 */
  App_Init();
  //Drv_PWM_Init();
  //DEMO_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Drv_SILICON_Proc();
    //Drv_PWM_Proc();

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

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
  huart1.Init.BaudRate = 115200;
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

void init_pwmIO()
{
    GPIO_InitTypeDef GPIO_InitStruct;//GPIO结构�?
    
    GPIO_InitStruct.Mode=GPIO_MODE_AF_PP;//复用推挽输出
    GPIO_InitStruct.Pin=GPIO_PIN_6|GPIO_PIN_7;//PA6
    GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_LOW;//翻转速度=10MHZ
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);//TIM3通道1IO�?
    
    GPIO_InitStruct.Mode=GPIO_MODE_AF_PP;//复用推挽输出
    GPIO_InitStruct.Pin=GPIO_PIN_0|GPIO_PIN_1;//PB0
    GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_LOW;//翻转速度=10MHZ
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);//TIM3通道3IO�?
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_DCLK_Pin|LED_HOLD_Pin|LED_DOWN_Pin|LED_RIGHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_LEFT_Pin|LED_UP_Pin|SCL1_Pin|SDA1_Pin|SCL2_Pin|SDA2_Pin|SCL3_Pin|SDA3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_DCLK_Pin LED_HOLD_Pin SW1_Pin SW2_Pin */
  GPIO_InitStruct.Pin = LED_DCLK_Pin|LED_HOLD_Pin|LED_DOWN_Pin|LED_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_LEFT_Pin LED_DOWN_Pin LED_UP_Pin LED_RIGHT_Pin 
                           LED1_Pin */
  GPIO_InitStruct.Pin = LED_LEFT_Pin|LED_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_Pin */
  GPIO_InitStruct.Pin = INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SCL_Pin SDA_Pin */
  GPIO_InitStruct.Pin = SCL1_Pin|SDA1_Pin|SCL2_Pin|SDA2_Pin|SCL3_Pin|SDA3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  //init_pwmIO();// can not work

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}
#if 0
/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  //htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
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
    Error_Handler(__FILE__, __LINE__);
  }
}


/**
  * @brief  The drv_led init.
  */
void Drv_PWM_Init(void)
{  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  

  tickstart = HAL_GetTick();
}

void Drv_PWM_Proc(void)
{
  if (turn_off)
  {
    if((HAL_GetTick() - tickstart) > 1000)
    {
      turn_off = 0;
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, (uint32_t)0);
    }
  }
}
#endif
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
        HAL_GPIO_WritePin(SCL_GPIO_Port, I2C_SCL_PIN, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(SCL_GPIO_Port, I2C_SCL_PIN, GPIO_PIN_RESET);
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
        HAL_GPIO_WritePin(SDA_GPIO_Port, I2C_SDA_PIN, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(SDA_GPIO_Port, I2C_SDA_PIN, GPIO_PIN_RESET);
}

//Init I2C pin
void I2C_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = I2C_SCL_PIN;
    //GPIO_InitStruct.Pull = GPIO_PULLUP;         //test
    HAL_GPIO_Init(SCL_GPIO_Port, &GPIO_InitStruct);
    //SHT_set_scl_dir(1);//SCL���ó����

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


/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)

{
    HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)&ch,1, 0xFFFF);
    return ch;
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
