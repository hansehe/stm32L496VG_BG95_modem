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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RX_BUFFER_LEN 1024
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint8_t receive_buffer[RX_BUFFER_LEN];
char msg[80];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void debug_write(char *data){
  HAL_UART_Transmit(&huart5, (uint8_t *) data , strlen(data), 10000);
}

void write_ati(){
  HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t *) "ATI\r", 4 , 100);
  if(status != HAL_OK){
    debug_write("Err Tx\r\n");
  }
}

void check_overflow_flag(){
  if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)){
      debug_write("Overrun\r\n");
      __HAL_UART_CLEAR_OREFLAG(&huart1);
  }
  else{
    debug_write("No overrun\r\n");
  }
}

void receive_write_directly(){
  int rx = 0;
  char msg[80];
  HAL_StatusTypeDef status;
  while(1){
    status = HAL_UART_Receive(&huart1, receive_buffer, 1, 100);
    if(status == HAL_OK){
      rx++;
      HAL_UART_Transmit(&huart5, receive_buffer, 1, 100);
    }
    else if(status == HAL_TIMEOUT){
      sprintf(msg, "RX: %d\r\n", rx);
      debug_write("Timeout\r\n");
      return;
    }
    else{
      debug_write("Err Rx\r\n");
      return;
    }
  }
}

void receive_store_first(){
  int rx = 0;
  char msg[80];
  uint8_t recv[100];
  HAL_StatusTypeDef status;
  while(1){
    status = HAL_UART_Receive(&huart1, receive_buffer, 1, 100);
    if(status == HAL_OK){
      recv[rx] = receive_buffer[0];
      rx++;
    }
    else if(status == HAL_TIMEOUT){
      sprintf(msg, "RX: %d\r\n", rx);
      debug_write(msg);
      debug_write("Timeout\r\n");
      HAL_UART_Transmit(&huart5, recv, rx, 100);
      return;
    }
    else{
      debug_write("Err Rx\r\n");
      return;
    }
  }
}

uint8_t get_bytes_received(){
  return RX_BUFFER_LEN - huart1.hdmarx->Instance->CNDTR;
}

void receive_dma(){
  HAL_StatusTypeDef status;
  status = HAL_UART_Receive_DMA(&huart1, receive_buffer, RX_BUFFER_LEN);
  if(status != HAL_OK){
    debug_write("DMA Err\r\n");
  }
  while(1){
    sprintf(msg, "DMA Rx count: %d\r\n", get_bytes_received());
    debug_write(msg);

    if(get_bytes_received() == 56){
      HAL_UART_Transmit(&huart5, receive_buffer, 56, 100);
      return;
    }
  }
}


static bool system_bg_module_is_powered_on(void){
    return HAL_GPIO_ReadPin(BG95_STATUS_GPIO_Port, BG95_STATUS_Pin) != 0;
}

void bg_reset(){
  HAL_GPIO_WritePin(BG95_RESET_N_GPIO_Port, BG95_RESET_N_Pin, GPIO_PIN_RESET);
  HAL_Delay(300);
  HAL_GPIO_WritePin(BG95_RESET_N_GPIO_Port, BG95_RESET_N_Pin, GPIO_PIN_SET);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uint32_t errorCode = huart->ErrorCode;
  sprintf(msg, "Err CB: %lu\r\n", errorCode);
  check_overflow_flag();
  debug_write(msg);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, GPIO_PIN_RESET);
  debug_write("Rx\r\n");
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  debug_write("Reset\r\n");
  bg_reset();
  while(!system_bg_module_is_powered_on()){
    HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, GPIO_PIN_RESET);
  }

  debug_write("Start\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	memset(receive_buffer, '\0', RX_BUFFER_LEN);
  //status = HAL_UART_Receive_IT(&huart1, receive_buffer, receive_buffer_length);
  //status = HAL_UART_Transmit(&huart1, bg95_uart_echo_cmd, strlen(bg95_uart_echo_cmd), 1000);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(1000);

    debug_write("==Start write_direct==\r\n");
    write_ati();
    receive_write_directly();
    check_overflow_flag();

    debug_write("==Start write_store==\r\n");
    write_ati();
    receive_store_first();
    check_overflow_flag();

    debug_write("==Start write_dma==\r\n");
    write_ati();
    receive_dma();
    check_overflow_flag();

    while(1);
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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BG95_RESET_N_GPIO_Port, BG95_RESET_N_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BG95_AP_READY_Pin|BG95_PON_TRIG_Pin|BG95_DTR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : DEBUG_OUT_Pin */
  GPIO_InitStruct.Pin = DEBUG_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEBUG_OUT_GPIO_Port, &GPIO_InitStruct);

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
