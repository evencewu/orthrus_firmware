/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

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
#define ECAT_SYNC0_Pin GPIO_PIN_0
#define ECAT_SYNC0_GPIO_Port GPIOA
#define ECAT_SYNC0_EXTI_IRQn EXTI0_IRQn
#define ECAT_SYNC1_Pin GPIO_PIN_1
#define ECAT_SYNC1_GPIO_Port GPIOA
#define ECAT_SYNC1_EXTI_IRQn EXTI1_IRQn
#define ECAT_RST_Pin GPIO_PIN_2
#define ECAT_RST_GPIO_Port GPIOA
#define ECAT_SPI_NSS_Pin GPIO_PIN_12
#define ECAT_SPI_NSS_GPIO_Port GPIOB
#define ECAT_SPI_IRQ_Pin GPIO_PIN_8
#define ECAT_SPI_IRQ_GPIO_Port GPIOA
#define ECAT_SPI_IRQ_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
