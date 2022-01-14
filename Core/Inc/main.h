/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BG95_NET_STATUS_Pin GPIO_PIN_2
#define BG95_NET_STATUS_GPIO_Port GPIOE
#define BG95_RESET_N_Pin GPIO_PIN_3
#define BG95_RESET_N_GPIO_Port GPIOE
#define BG95_STATUS_Pin GPIO_PIN_1
#define BG95_STATUS_GPIO_Port GPIOC
#define BG95_AP_READY_Pin GPIO_PIN_2
#define BG95_AP_READY_GPIO_Port GPIOC
#define BG95_PON_TRIG_Pin GPIO_PIN_3
#define BG95_PON_TRIG_GPIO_Port GPIOC
#define BG95_DTR_Pin GPIO_PIN_8
#define BG95_DTR_GPIO_Port GPIOC
#define BG95_PWRKEY_Pin GPIO_PIN_9
#define BG95_PWRKEY_GPIO_Port GPIOC
#define LOCK_SWITCH_Pin GPIO_PIN_9
#define LOCK_SWITCH_GPIO_Port GPIOA
#define USART_RI_Pin GPIO_PIN_10
#define USART_RI_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
