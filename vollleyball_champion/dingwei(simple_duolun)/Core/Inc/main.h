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
#include "stm32g4xx_hal.h"

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

/* USER CODE BEGIN Private defines */
#define pi (3.14159265359)
#define TIMESTAMP       uint32_t
#define TIME_SCALAR     1000.0
#define TIMESTAMP2SECOND(tsEnd,tsStart)       (double)(tsEnd-tsStart)/TIME_SCALAR
#define GenerateTimeStamp()										HAL_GetTick()
#define LOCATOR_MIN_TIMEGAP     5.0/TIME_SCALAR
struct GyroResult
{
    TIMESTAMP timeStamp;//角度测量值时间戳
    double rotation;//测量角度
		long loop;
};
struct EncoderResult
{
    double distance[2];//移动距离{0x ,1y}
    TIMESTAMP timeStamp;
};
struct LocatorResult
{
    double x,y,r;
    double posVariance[3];
    double vx,vy,vAng;
    double velVariance[3];
    TIMESTAMP timeStamp;
};

extern uint8_t if_gyro_right;
extern volatile uint8_t gyro_buffer[33];
extern struct LocatorResult lcResult;
void go_jiaozhun(void);
void USART_printf(char *fmt, ...);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
