/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f1xx_it.h"
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t Rx_data3[500];
extern uint8_t Rx_data1[500];
extern uint32_t ADC_Value[80];
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;

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
#define N1_2_Pin GPIO_PIN_0
#define N1_2_GPIO_Port GPIOC
#define KEY5_Pin GPIO_PIN_1
#define KEY5_GPIO_Port GPIOC
#define BEEP_Pin GPIO_PIN_0
#define BEEP_GPIO_Port GPIOA
#define X_Pin GPIO_PIN_2
#define X_GPIO_Port GPIOA
#define Y_Pin GPIO_PIN_3
#define Y_GPIO_Port GPIOA
#define C_Pin GPIO_PIN_6
#define C_GPIO_Port GPIOA
#define KEY3_Pin GPIO_PIN_4
#define KEY3_GPIO_Port GPIOC
#define KEY4_Pin GPIO_PIN_5
#define KEY4_GPIO_Port GPIOC
#define KEY2_Pin GPIO_PIN_0
#define KEY2_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_2
#define KEY1_GPIO_Port GPIOB
#define N1_1_Pin GPIO_PIN_12
#define N1_1_GPIO_Port GPIOB
#define N5_1_Pin GPIO_PIN_13
#define N5_1_GPIO_Port GPIOB
#define N4_2_Pin GPIO_PIN_14
#define N4_2_GPIO_Port GPIOB
#define N4_1_Pin GPIO_PIN_15
#define N4_1_GPIO_Port GPIOB
#define N3_2_Pin GPIO_PIN_9
#define N3_2_GPIO_Port GPIOC
#define KEY6_Pin GPIO_PIN_10
#define KEY6_GPIO_Port GPIOA
#define N3_1_Pin GPIO_PIN_11
#define N3_1_GPIO_Port GPIOA
#define N5_2_Pin GPIO_PIN_12
#define N5_2_GPIO_Port GPIOA
#define N2_2_Pin GPIO_PIN_8
#define N2_2_GPIO_Port GPIOB
#define N2_1_Pin GPIO_PIN_9
#define N2_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
