/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static bool system_bg_module_is_powered_on(void)
{
    return HAL_GPIO_ReadPin(BG95_STATUS_GPIO_Port, BG95_STATUS_Pin) != 0;
}

static bool system_bg_module_power_on(void)
{
	// Device power up routine (excerpt from BG documentation):
	// When BG96 is in power off mode, it can be turned on to normal mode by driving the PWRKEY
	// pin to a low level for at least 500ms.

//	if (system_bg_module_is_powered_on())
//	{
//		return true;
//	}

	HAL_GPIO_WritePin(BG95_RESET_N_GPIO_Port, BG95_RESET_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BG95_PWRKEY_GPIO_Port, BG95_PWRKEY_Pin, GPIO_PIN_RESET);

	HAL_Delay(1000);

	HAL_GPIO_WritePin(BG95_RESET_N_GPIO_Port, BG95_RESET_N_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BG95_PWRKEY_GPIO_Port, BG95_PWRKEY_Pin, GPIO_PIN_SET);

	HAL_Delay(50);

	HAL_GPIO_WritePin(BG95_RESET_N_GPIO_Port, BG95_RESET_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BG95_PWRKEY_GPIO_Port, BG95_PWRKEY_Pin, GPIO_PIN_RESET);

	HAL_Delay(750);

	HAL_GPIO_WritePin(BG95_RESET_N_GPIO_Port, BG95_RESET_N_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BG95_PWRKEY_GPIO_Port, BG95_PWRKEY_Pin, GPIO_PIN_SET);

	HAL_Delay(2100);

	bool status = system_bg_module_is_powered_on();
	return status;
}

static bool system_bg_module_power_off(void)
{
	// Device power down routine (excerpt from BG documentation):
	// Driving the PWRKEY pin to a low level voltage for at least 650ms,
	// the module will execute power-down procedure after the PWRKEY is released.

	if (!system_bg_module_is_powered_on())
	{
		return true;
	}

    HAL_GPIO_WritePin(BG95_RESET_N_GPIO_Port, BG95_RESET_N_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BG95_PWRKEY_GPIO_Port, BG95_PWRKEY_Pin, GPIO_PIN_RESET);

    HAL_Delay(1000);

    HAL_GPIO_WritePin(BG95_RESET_N_GPIO_Port, BG95_RESET_N_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BG95_PWRKEY_GPIO_Port, BG95_PWRKEY_Pin, GPIO_PIN_SET);

    HAL_Delay(1000);

    return !system_bg_module_is_powered_on();
}

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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  uint16_t receive_buffer_length = 1024;
  uint8_t receive_buffer[receive_buffer_length];

  char* bg95_uart_echo_cmd = "ATE1\r\0";
  char* bg95_imei_cmd = "AT+QGMR\r\0";
  uint8_t* test_arr = {0xaa, 0xaa, 0xaa, 0xaa};

//  if (system_bg_module_is_powered_on)
//  {
//	  while (!system_bg_module_power_off());
//  }
  while (!system_bg_module_power_on());
//  system_bg_module_power_on();
  HAL_Delay(1000);

  HAL_StatusTypeDef status = HAL_OK;
  uint16_t bytes_read = 0;
  uint16_t cmd_length = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	memset(receive_buffer, '\0', receive_buffer_length);
	cmd_length = strlen(bg95_imei_cmd);
	status = HAL_UART_Transmit(&huart1, bg95_imei_cmd, cmd_length, 1000);
//	status = HAL_UART_Transmit(&huart1, test_arr, 1, 1000);
	status = HAL_UART_Receive(&huart1, receive_buffer, receive_buffer_length, 1000);
	bytes_read = receive_buffer_length - huart1.RxXferCount;
	if (bytes_read > 0)
	{
		for (uint16_t i = 0; i < bytes_read; i++)
		{
			char lol = receive_buffer[i];
		}
	}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BG95_RESET_N_GPIO_Port, BG95_RESET_N_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BG95_AP_READY_Pin|BG95_PON_TRIG_Pin|BG95_DTR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BG95_PWRKEY_GPIO_Port, BG95_PWRKEY_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BG95_NET_STATUS_Pin */
  GPIO_InitStruct.Pin = BG95_NET_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BG95_NET_STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BG95_RESET_N_Pin */
  GPIO_InitStruct.Pin = BG95_RESET_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BG95_RESET_N_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BG95_STATUS_Pin */
  GPIO_InitStruct.Pin = BG95_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BG95_STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BG95_AP_READY_Pin BG95_PON_TRIG_Pin BG95_DTR_Pin BG95_PWRKEY_Pin */
  GPIO_InitStruct.Pin = BG95_AP_READY_Pin|BG95_PON_TRIG_Pin|BG95_DTR_Pin|BG95_PWRKEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LOCK_SWITCH_Pin USART_RI_Pin */
  GPIO_InitStruct.Pin = LOCK_SWITCH_Pin|USART_RI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

