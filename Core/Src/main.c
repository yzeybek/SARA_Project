/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

#include "pid.h"
#include "sara.h"
#include "state.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

t_pid pid_heave, pid_sway, pid_surge, pid_roll, pid_pitch, pid_yaw;
float pid_values[7];

t_state states[8];
t_range	state_ranges[9];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

	// PID Init Begin

	pid_values[0] = PID_HEAVE_KP;
	pid_values[1] = PID_HEAVE_KI;
	pid_values[2] = PID_HEAVE_KD;
	pid_values[3] = PID_HEAVE_OUT_MIN;
	pid_values[4] = PID_HEAVE_OUT_MAX;
	pid_values[5] = PID_HEAVE_INT_MIN;
	pid_values[6] = PID_HEAVE_INT_MAX;
	pid_init(&pid_heave, pid_values);

	pid_values[0] = PID_SWAY_KP;
	pid_values[1] = PID_SWAY_KI;
	pid_values[2] = PID_SWAY_KD;
	pid_values[3] = PID_SWAY_OUT_MIN;
	pid_values[4] = PID_SWAY_OUT_MAX;
	pid_values[5] = PID_SWAY_INT_MIN;
	pid_values[6] = PID_SWAY_INT_MAX;
	pid_init(&pid_sway, pid_values);

	pid_values[0] = PID_SURGE_KP;
	pid_values[1] = PID_SURGE_KI;
	pid_values[2] = PID_SURGE_KD;
	pid_values[3] = PID_SURGE_OUT_MIN;
	pid_values[4] = PID_SURGE_OUT_MAX;
	pid_values[5] = PID_SURGE_INT_MIN;
	pid_values[6] = PID_SURGE_INT_MAX;
	pid_init(&pid_surge, pid_values);

	pid_values[0] = PID_ROLL_KP;
	pid_values[1] = PID_ROLL_KI;
	pid_values[2] = PID_ROLL_KD;
	pid_values[3] = PID_ROLL_OUT_MIN;
	pid_values[4] = PID_ROLL_OUT_MAX;
	pid_values[5] = PID_ROLL_INT_MIN;
	pid_values[6] = PID_ROLL_INT_MAX;
	pid_init(&pid_roll, pid_values);

	pid_values[0] = PID_PITCH_KP;
	pid_values[1] = PID_PITCH_KI;
	pid_values[2] = PID_PITCH_KD;
	pid_values[3] = PID_PITCH_OUT_MIN;
	pid_values[4] = PID_PITCH_OUT_MAX;
	pid_values[5] = PID_PITCH_INT_MIN;
	pid_values[6] = PID_PITCH_INT_MAX;
	pid_init(&pid_pitch, pid_values);

	pid_values[0] = PID_YAW_KP;
	pid_values[1] = PID_YAW_KI;
	pid_values[2] = PID_YAW_KD;
	pid_values[3] = PID_YAW_OUT_MIN;
	pid_values[4] = PID_YAW_OUT_MAX;
	pid_values[5] = PID_YAW_INT_MIN;
	pid_values[6] = PID_YAW_INT_MAX;
	pid_init(&pid_yaw, pid_values);

	// PID Init End

	// State Init Begin

	state_ranges[0].start = STATE_1_POS_X_START;
	state_ranges[0].end = STATE_1_POS_X_END;
	state_ranges[1].start = STATE_1_POS_Y_START;
	state_ranges[1].end = STATE_1_POS_Y_END;
	state_ranges[2].start = STATE_1_POS_Z_START;
	state_ranges[2].end = STATE_1_POS_Z_END;
	state_ranges[3].start = STATE_1_VEL_X_START;
	state_ranges[3].end = STATE_1_VEL_X_END;
	state_ranges[4].start = STATE_1_VEL_Y_START;
	state_ranges[4].end = STATE_1_VEL_Y_END;
	state_ranges[5].start = STATE_1_VEL_Z_START;
	state_ranges[5].end = STATE_1_VEL_Z_END;
	state_ranges[6].start = STATE_1_EUL_X_START;
	state_ranges[6].end = STATE_1_EUL_X_END;
	state_ranges[7].start = STATE_1_EUL_Y_START;
	state_ranges[7].end = STATE_1_EUL_Y_END;
	state_ranges[8].start = STATE_1_EUL_Z_START;
	state_ranges[8].end = STATE_1_EUL_Z_END;
	state_init(&states[0], state_ranges);

	state_ranges[0].start = STATE_2_POS_X_START;
	state_ranges[0].end = STATE_2_POS_X_END;
	state_ranges[1].start = STATE_2_POS_Y_START;
	state_ranges[1].end = STATE_2_POS_Y_END;
	state_ranges[2].start = STATE_2_POS_Z_START;
	state_ranges[2].end = STATE_2_POS_Z_END;
	state_ranges[3].start = STATE_2_VEL_X_START;
	state_ranges[3].end = STATE_2_VEL_X_END;
	state_ranges[4].start = STATE_2_VEL_Y_START;
	state_ranges[4].end = STATE_2_VEL_Y_END;
	state_ranges[5].start = STATE_2_VEL_Z_START;
	state_ranges[5].end = STATE_2_VEL_Z_END;
	state_ranges[6].start = STATE_2_EUL_X_START;
	state_ranges[6].end = STATE_2_EUL_X_END;
	state_ranges[7].start = STATE_2_EUL_Y_START;
	state_ranges[7].end = STATE_2_EUL_Y_END;
	state_ranges[8].start = STATE_2_EUL_Z_START;
	state_ranges[8].end = STATE_2_EUL_Z_END;
	state_init(&states[1], state_ranges);

	state_ranges[0].start = STATE_3_POS_X_START;
	state_ranges[0].end = STATE_3_POS_X_END;
	state_ranges[1].start = STATE_3_POS_Y_START;
	state_ranges[1].end = STATE_3_POS_Y_END;
	state_ranges[2].start = STATE_3_POS_Z_START;
	state_ranges[2].end = STATE_3_POS_Z_END;
	state_ranges[3].start = STATE_3_VEL_X_START;
	state_ranges[3].end = STATE_3_VEL_X_END;
	state_ranges[4].start = STATE_3_VEL_Y_START;
	state_ranges[4].end = STATE_3_VEL_Y_END;
	state_ranges[5].start = STATE_3_VEL_Z_START;
	state_ranges[5].end = STATE_3_VEL_Z_END;
	state_ranges[6].start = STATE_3_EUL_X_START;
	state_ranges[6].end = STATE_3_EUL_X_END;
	state_ranges[7].start = STATE_3_EUL_Y_START;
	state_ranges[7].end = STATE_3_EUL_Y_END;
	state_ranges[8].start = STATE_3_EUL_Z_START;
	state_ranges[8].end = STATE_3_EUL_Z_END;
	state_init(&states[2], state_ranges);

	state_ranges[0].start = STATE_4_POS_X_START;
	state_ranges[0].end = STATE_4_POS_X_END;
	state_ranges[1].start = STATE_4_POS_Y_START;
	state_ranges[1].end = STATE_4_POS_Y_END;
	state_ranges[2].start = STATE_4_POS_Z_START;
	state_ranges[2].end = STATE_4_POS_Z_END;
	state_ranges[3].start = STATE_4_VEL_X_START;
	state_ranges[3].end = STATE_4_VEL_X_END;
	state_ranges[4].start = STATE_4_VEL_Y_START;
	state_ranges[4].end = STATE_4_VEL_Y_END;
	state_ranges[5].start = STATE_4_VEL_Z_START;
	state_ranges[5].end = STATE_4_VEL_Z_END;
	state_ranges[6].start = STATE_4_EUL_X_START;
	state_ranges[6].end = STATE_4_EUL_X_END;
	state_ranges[7].start = STATE_4_EUL_Y_START;
	state_ranges[7].end = STATE_4_EUL_Y_END;
	state_ranges[8].start = STATE_4_EUL_Z_START;
	state_ranges[8].end = STATE_4_EUL_Z_END;
	state_init(&states[3], state_ranges);

	state_ranges[0].start = STATE_5_POS_X_START;
	state_ranges[0].end = STATE_5_POS_X_END;
	state_ranges[1].start = STATE_5_POS_Y_START;
	state_ranges[1].end = STATE_5_POS_Y_END;
	state_ranges[2].start = STATE_5_POS_Z_START;
	state_ranges[2].end = STATE_5_POS_Z_END;
	state_ranges[3].start = STATE_5_VEL_X_START;
	state_ranges[3].end = STATE_5_VEL_X_END;
	state_ranges[4].start = STATE_5_VEL_Y_START;
	state_ranges[4].end = STATE_5_VEL_Y_END;
	state_ranges[5].start = STATE_5_VEL_Z_START;
	state_ranges[5].end = STATE_5_VEL_Z_END;
	state_ranges[6].start = STATE_5_EUL_X_START;
	state_ranges[6].end = STATE_5_EUL_X_END;
	state_ranges[7].start = STATE_5_EUL_Y_START;
	state_ranges[7].end = STATE_5_EUL_Y_END;
	state_ranges[8].start = STATE_5_EUL_Z_START;
	state_ranges[8].end = STATE_5_EUL_Z_END;
	state_init(&states[4], state_ranges);

	state_ranges[0].start = STATE_6_POS_X_START;
	state_ranges[0].end = STATE_6_POS_X_END;
	state_ranges[1].start = STATE_6_POS_Y_START;
	state_ranges[1].end = STATE_6_POS_Y_END;
	state_ranges[2].start = STATE_6_POS_Z_START;
	state_ranges[2].end = STATE_6_POS_Z_END;
	state_ranges[3].start = STATE_6_VEL_X_START;
	state_ranges[3].end = STATE_6_VEL_X_END;
	state_ranges[4].start = STATE_6_VEL_Y_START;
	state_ranges[4].end = STATE_6_VEL_Y_END;
	state_ranges[5].start = STATE_6_VEL_Z_START;
	state_ranges[5].end = STATE_6_VEL_Z_END;
	state_ranges[6].start = STATE_6_EUL_X_START;
	state_ranges[6].end = STATE_6_EUL_X_END;
	state_ranges[7].start = STATE_6_EUL_Y_START;
	state_ranges[7].end = STATE_6_EUL_Y_END;
	state_ranges[8].start = STATE_6_EUL_Z_START;
	state_ranges[8].end = STATE_6_EUL_Z_END;
	state_init(&states[5], state_ranges);

	state_ranges[0].start = STATE_7_POS_X_START;
	state_ranges[0].end = STATE_7_POS_X_END;
	state_ranges[1].start = STATE_7_POS_Y_START;
	state_ranges[1].end = STATE_7_POS_Y_END;
	state_ranges[2].start = STATE_7_POS_Z_START;
	state_ranges[2].end = STATE_7_POS_Z_END;
	state_ranges[3].start = STATE_7_VEL_X_START;
	state_ranges[3].end = STATE_7_VEL_X_END;
	state_ranges[4].start = STATE_7_VEL_Y_START;
	state_ranges[4].end = STATE_7_VEL_Y_END;
	state_ranges[5].start = STATE_7_VEL_Z_START;
	state_ranges[5].end = STATE_7_VEL_Z_END;
	state_ranges[6].start = STATE_7_EUL_X_START;
	state_ranges[6].end = STATE_7_EUL_X_END;
	state_ranges[7].start = STATE_7_EUL_Y_START;
	state_ranges[7].end = STATE_7_EUL_Y_END;
	state_ranges[8].start = STATE_7_EUL_Z_START;
	state_ranges[8].end = STATE_7_EUL_Z_END;
	state_init(&states[6], state_ranges);

	states[7] = (t_state){0};

	// State Init End

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
	while (1) {
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
