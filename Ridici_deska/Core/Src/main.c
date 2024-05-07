/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Rozsah serv 500 - 2500
#define servoMiddle 1500
#define servoLeft 	2400
#define servoRight 	600

#define servo1	TIM1,	1
#define servo2	TIM1,	2
#define servo3	TIM1,	3
#define servo4	TIM1,	4
#define servo5	TIM3,	1
#define servo6	TIM3,	2
#define servo7	TIM3,	3

#define servo7_max	0x3A6
#define servo7_min	0x690

#define fine_change		2
#define rough_change	4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void setServo(TIM_TypeDef *timer, int tim_channel, int duty_cycle);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t pom = 0;
bool lastBtnState = true;		// last state of user button
uint8_t RX_data[3];				// buffer for uart data
uint8_t data_pos = 0;			// buffer data offset
uint8_t receive_complete = 0;	// global flag

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	data_pos++;
	if (data_pos > 2)	// already get 3 bytes
	{
		data_pos = 0;
		HAL_UART_Receive_IT(&huart2, RX_data + data_pos, 1);
		receive_complete = 1;	// set global flag to set servos position
	}
	else
	{
		HAL_UART_Receive_IT(&huart2, RX_data + data_pos, 1);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(PCON_GPIO_Port, PCON_Pin, GPIO_PIN_SET);	// LED signals MCU is ON
	HAL_UART_Receive_IT(&huart2, RX_data, 1);					// START receiving UART data
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	// START ALL PWM channel + set default servos position
	TIM1->CCR1 = servoMiddle;
	TIM1->CCR2 = servoMiddle;
	TIM1->CCR3 = servoMiddle;
	TIM1->CCR4 = servoMiddle;
	TIM3->CCR1 = servoMiddle;
	TIM3->CCR2 = servoMiddle;
	TIM3->CCR3 = servoMiddle;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_Base_Start_IT(&htim2);				// START TIM2 for USER_BTN sampling

	while (1)
	{
		if (receive_complete)
		{
			receive_complete = 0;	// RESET global flag

			// Buttons A,C - servo1, servo2 - main arm UP/DOWN
			// Buttons B,D - servo3 - main arm rotation
			// Buttons E,F - servo4 - elbow UP/DOWN
			// Axis Y - servo5 - wrist UP/DOWN
			// Axis X - servo6 - wrist rotation
			// K button - servo7 - forks clamp
			// E and F buttons at same time - servo7 - forks open


			// Main Arm - servo1/2
			if (RX_data[2] & (1))
			{
				setServo(servo1, TIM1->CCR1 + rough_change);
				setServo(servo2, TIM1->CCR2 + rough_change);
			}
			else if (RX_data[2] & (1 << 2))
			{
				setServo(servo1, TIM1->CCR1 - rough_change);
				setServo(servo2, TIM1->CCR2 - rough_change);
			}

			// Arm rotation - servo3
			if (RX_data[2] & (1 << 1))
				setServo(servo3, TIM1->CCR3 + rough_change);
			else if (RX_data[2] & (1 << 3))
				setServo(servo3, TIM1->CCR3 - rough_change);

			// Forks - servo7
			if (RX_data[2] & (1 << 6))
			{
				if ((TIM3->CCR3 - rough_change) > servo7_max)	// Forks servo has different movement range
					setServo(servo7, TIM3->CCR3 - rough_change);
			}
			else if ((RX_data[2] & (1 << 4)) && (RX_data[2] & (1 << 5)))
			{
				if ((TIM3->CCR3 + rough_change) < servo7_min)
					setServo(servo7, TIM3->CCR3 + rough_change);
			}

			// Elbow - servo4
			if ((RX_data[2] & (1 << 4)) && !(RX_data[2] & (1 << 5)))
				setServo(servo4, TIM1->CCR4 + rough_change);
			else if ((RX_data[2] & (1 << 5)) && !(RX_data[2] & (1 << 4)))
				setServo(servo4, TIM1->CCR4 - rough_change);

			// X-axis - servo6
			if (!((*RX_data > 0x78) && (*RX_data < 0x87)))// Joystick between 120 to 135 (8 bit) --> don't change servo position
			{
				if (*RX_data > 135)
				{
					if ((*RX_data > 135) && (*RX_data < 180))
						setServo(servo5, TIM3->CCR1 - fine_change / 2);
					else if ((*RX_data > 180) && (*RX_data < 210))
						setServo(servo5, TIM3->CCR1 - fine_change);
					else if (*RX_data > 210)
						setServo(servo5, TIM3->CCR1 - rough_change);
				}
				else
				{
					if ((*RX_data > 85) && (*RX_data < 120))
						setServo(servo5, TIM3->CCR1 + fine_change / 2);
					else if ((*RX_data > 55) && (*RX_data < 85))
						setServo(servo5, TIM3->CCR1 + fine_change);
					else if (*RX_data < 55)
						setServo(servo5, TIM3->CCR1 + rough_change);
				}
			}

			// Y-axis - servo5
			if (!((RX_data[1] > 0x78) && (RX_data[1] < 0x87)))// Joystick between 120 to 135 (8 bit) --> don't change servo position
			{

				if (RX_data[1] > 140)
				{
					if ((RX_data[1] > 140) && (RX_data[1] < 180))
						setServo(servo6, TIM3->CCR2 - fine_change / 2);
					if ((RX_data[1] > 180) && (RX_data[1] < 210))
						setServo(servo6, TIM3->CCR2 - fine_change);
					if (RX_data[1] > 210)
						setServo(servo6, TIM3->CCR2 - rough_change);
				}
				else
				{
					if ((RX_data[1] > 85) && (RX_data[1] < 115))
						setServo(servo6, TIM3->CCR2 + fine_change / 2);
					if ((RX_data[1] > 55) && (RX_data[1] < 85))
						setServo(servo6, TIM3->CCR2 + fine_change);
					if (RX_data[1] < 55)
						setServo(servo6, TIM3->CCR2 + rough_change);
				}
			}
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PCON_GPIO_Port, PCON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PCON_Pin */
  GPIO_InitStruct.Pin = PCON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PCON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	// TIM2 IRQ for BTN sampling
{
	bool actual_state = HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin);
	if (lastBtnState != actual_state)
	{
		lastBtnState = actual_state;
		if (actual_state)
		{
			pom = !pom;
		}
	}
}

void setServo(TIM_TypeDef *timer, int tim_channel, int duty_cycle)
{
	if ((duty_cycle > servoLeft) || (duty_cycle < servoRight))	// Servo range control
		return;

	switch (tim_channel)
	{
	case 1:
		timer->CCR1 = duty_cycle;
		break;
	case 2:
		timer->CCR2 = duty_cycle;
		break;
	case 3:
		timer->CCR3 = duty_cycle;
		break;
	case 4:
		timer->CCR4 = duty_cycle;
		break;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
