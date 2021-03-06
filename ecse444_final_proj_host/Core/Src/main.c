/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMEOUT 100
#define NUMBER_OF_DIRECTIVES 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char msg_buffer[200];
int timer = 0;

// Booleans
volatile int roundState;
volatile int gameStart = 0;

// Integers
int currentDirective;
int roundNumber = 0;
int p1Score = 0;
int p2Score = 0;

// Strings
char directives[3][15] = {
						"Press it!\r\n",
						"Twist it!\r\n",
						"Say it!\r\n"
						};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
void waitForGameStart(void);
void startNextRound(int dir);
void decideWinner(void);
void updateScoreboard(void);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Setting up LEDs
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);

  // Setting up first round game
  waitForGameStart();
  currentDirective = rand() % NUMBER_OF_DIRECTIVES;
  startNextRound(currentDirective);

  // Starting sleep mode with interrupts
  HAL_SuspendTick();
  HAL_PWR_EnableSleepOnExit();
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

  // Game has ended, we need to restart the clock
  HAL_ResumeTick();

  sprintf(msg_buffer, "\n\n\n==================================================\r\n");
  HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);
  sprintf(msg_buffer, "Game has ended. Thank you for playing!\r\n");
  HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);
  sprintf(msg_buffer, "==================================================\r\n");
  HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|RESET_P1_Pin|RESET_P2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_IN_2_Pin MIC_IN_2_Pin ACC_IN_2_Pin BUTTON_IN_1_Pin
                           MIC_IN_1_Pin ACC_IN_1_Pin */
  GPIO_InitStruct.Pin = BUTTON_IN_2_Pin|MIC_IN_2_Pin|ACC_IN_2_Pin|BUTTON_IN_1_Pin
                          |MIC_IN_1_Pin|ACC_IN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin RESET_P1_Pin RESET_P2_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|RESET_P1_Pin|RESET_P2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*
	 * BUTTON PIN INTERRUPTS
	 * 1st: start the game
	 * 2nd: end the game
	 */
	if (GPIO_Pin == Button_Pin && gameStart == 0){
		gameStart = 1;

	} else if (GPIO_Pin == Button_Pin && gameStart == 1){
		//game is over
		HAL_PWR_DisableSleepOnExit();


	} else {
		/*
		 * Players interrupts
		 * each will trigger a different interrupt
		 *
		 * roundState =
		 * 		0 Player 1 - Press
		 * 		1 Player 1 - Twist
		 * 		2 Player 1 - Say
		 * 		3 Player 2 - Press
		 * 		4 Player 2 - Twist
		 * 		5 Player 2 - Say
		 */

		if (GPIO_Pin == ACC_IN_1_Pin)
		{
			HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
			roundState = 1;
		}
		if (GPIO_Pin == ACC_IN_2_Pin)
		{
			HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
			roundState = 4;
		}

		if (GPIO_Pin == BUTTON_IN_1_Pin){
			HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
			roundState = 0;
		}
		if (GPIO_Pin == BUTTON_IN_2_Pin){
			HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
			roundState = 3;
		}

		if (GPIO_Pin == MIC_IN_1_Pin){
			HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
			roundState = 2;
		}
		if (GPIO_Pin == MIC_IN_2_Pin){
			HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
			roundState = 5;
		}

		HAL_GPIO_TogglePin(RESET_P1_GPIO_Port, RESET_P1_Pin);
		HAL_GPIO_TogglePin(RESET_P2_GPIO_Port, RESET_P2_Pin);

		decideWinner();
//		HAL_Delay(1000);
		while (timer < 5000000) {
			timer++;
		}
		timer = 0;

		updateScoreboard();

//		HAL_Delay(1000);
		while (timer < 5000000) {
			timer++;
		}
		timer = 0;

		sprintf(msg_buffer, "Setting up next round...\r\n");
		HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);

//		HAL_Delay(5000);
		while (timer < 25000000) {
			timer++;
		}
		timer = 0;

		currentDirective = rand() % NUMBER_OF_DIRECTIVES;
		startNextRound(currentDirective);

		//	HAL_Delay(1000);
		//	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
		//    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	}
}

void waitForGameStart(void) {
	sprintf(msg_buffer, "\n\n\n==================================================\r\n");
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);
	sprintf(msg_buffer, "WELCOME TO COMPETITIVE BOP IT\r\n");
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);
	sprintf(msg_buffer, "==================================================\r\n");
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);

	sprintf(msg_buffer, "Here are the rules of the game:\r\n");
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);

	sprintf(msg_buffer, "\t1. When prompted, press the blue button on the host board to start the game.\r\n");
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);

	sprintf(msg_buffer, "\t2. Perform the directives printed to the screen as quickly as possible.\r\n");
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);

	sprintf(msg_buffer, "\t3. First person to complete the action wins the round.\r\n");
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);

	sprintf(msg_buffer, "\t4. If you do the wrong action, you lose a point.\r\n");
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);

	sprintf(msg_buffer, "\n\nPress button when you are ready to start...\r\n");
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);


	while(!gameStart);

	sprintf(msg_buffer, "GAME STARTED! ENJOY!\r\n");
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);
}

void startNextRound(int dir) {
	roundNumber ++;

	sprintf(msg_buffer, "\n\n==================================================\r\n");
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);
	sprintf(msg_buffer, "ROUND #%d\r\n", roundNumber);
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);
	sprintf(msg_buffer, "==================================================\r\n");
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);

	sprintf(msg_buffer, "The next round will begin in:\r\n");
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);
	sprintf(msg_buffer, "3...\r\n");
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);
	while (timer < 5000000) {
		timer++;
	}
	timer = 0;

	sprintf(msg_buffer, "2...\r\n");
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);
	while (timer < 5000000) {
		timer++;
	}
	timer = 0;

	sprintf(msg_buffer, "1...\r\n");
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);
	while (timer < 5000000) {
		timer++;
	}
	timer = 0;

	sprintf(msg_buffer, "%s\r\n", directives[dir]);
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);

}

void decideWinner(void) {
	if(roundState % NUMBER_OF_DIRECTIVES == currentDirective) {
		// Right input
		if(roundState < 3) {
			// P1 wins point
			p1Score++;

			sprintf(msg_buffer, "Player 1 has scored the point!\r\n");
			HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);

		} else {
			// P2 wins point
			p2Score++;

			sprintf(msg_buffer, "Player 2 has scored the point!\r\n");
			HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);
		}

	} else {
		// Wrong input
		if(roundState < 3) {
			// P1 loses point
			p1Score--;

			sprintf(msg_buffer, "Player 1 has done the wrong action. -1 point.\r\n");
			HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);

		} else {
			// P2 loses point
			p2Score--;

			sprintf(msg_buffer, "Player 2 has done the wrong action. -1 point.\r\n");
			HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);
		}
	}
}

void updateScoreboard(void) {
	sprintf(msg_buffer, "Scoreboard:\r\n");
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);

	sprintf(msg_buffer, "\tPlayer 1: %d\r\n", p1Score);
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);

	sprintf(msg_buffer, "\tPlayer 2: %d\r\n", p2Score);
	HAL_UART_Transmit(&huart1, msg_buffer, strlen((char const *) msg_buffer), TIMEOUT);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
