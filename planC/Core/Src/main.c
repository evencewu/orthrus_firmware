/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#define F_LEG 0
#define B_LEG 1

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "unitreeA1_cmd.h"
#include <string.h>
#include <stdio.h>
#include "bsp_can.h"
#include "struct_typedef.h"
#include "RC.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM10_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  remote_control_init();
  CAN_InitArgument();
  MX_TIM4_Init();
  HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  int i = 0;
  while (1)
  {
    uint8_t byte[8];

    if (i == 0)
    {
      Float_to_Byte(data_motor[0][0].T, data_motor[0][0].Pos, byte); // 转换数据
      CAN_send_data(&hcan1, 0x40, byte);                             // 发送数据包A（内含t，w）
      Float_to_Byte(data_motor[0][0].W, data_motor[0][0].Acc, byte);
      CAN_send_data(&hcan1, 0x41, byte);
    }
    else if (i == 1)
    {
      Float_to_Byte(data_motor[0][1].T, data_motor[0][1].Pos, byte); // 转换数据
      CAN_send_data(&hcan1, 0x42, byte);                             // 发送数据包A（内含t，w）
      Float_to_Byte(data_motor[0][1].W, data_motor[0][1].Acc, byte);
      CAN_send_data(&hcan1, 0x43, byte);
    }
    else if (i == 2)
    {
      Float_to_Byte(data_motor[0][2].T, data_motor[0][2].Pos, byte); // 转换数据
      CAN_send_data(&hcan1, 0x44, byte);                             // 发送数据包A（内含t，w）
      Float_to_Byte(data_motor[0][2].W, data_motor[0][2].Acc, byte);
      CAN_send_data(&hcan1, 0x45, byte);
    }
    else if (i == 3)
    {
      Float_to_Byte(data_motor[1][0].T, data_motor[1][0].Pos, byte); // 转换数据
      CAN_send_data(&hcan1, 0x46, byte);                             // 发送数据包A（内含t，w）
      Float_to_Byte(data_motor[1][0].W, data_motor[1][0].Acc, byte);
      CAN_send_data(&hcan1, 0x47, byte);
    }
    else if (i == 4)
    {
      Float_to_Byte(data_motor[1][1].T, data_motor[1][1].Pos, byte); // 转换数据
      CAN_send_data(&hcan1, 0x48, byte);                             // 发送数据包A（内含t，w）
      Float_to_Byte(data_motor[1][1].W, data_motor[1][1].Acc, byte);
      CAN_send_data(&hcan1, 0x49, byte);
    }
    else if (i == 5)
    {
      Float_to_Byte(data_motor[1][2].T, data_motor[1][2].Pos, byte); // 转换数据
      CAN_send_data(&hcan1, 0x4A, byte);                             // 发送数据包A（内含t，w）
      Float_to_Byte(data_motor[1][2].W, data_motor[1][2].Acc, byte);
      CAN_send_data(&hcan1, 0x4B, byte);
    }
    else
    {
      i = -1;
    }

    i++;

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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

#ifdef USE_FULL_ASSERT
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
