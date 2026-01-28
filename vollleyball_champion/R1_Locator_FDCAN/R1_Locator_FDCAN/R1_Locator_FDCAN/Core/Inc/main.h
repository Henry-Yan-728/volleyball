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
#define GYRO_SCALAR         1e10
struct GyroResult
{
    TIMESTAMP timeStamp;//�ǶȲ���ֵʱ���
    double rotation;//�����Ƕ�
		long loop;
};
struct EncoderResult
{
    double distance[2];//�ƶ�����{0x ,1y}
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
typedef struct {
    double x;     // λ�� x
    double y;     // λ�� y
    double theta; // �����
} State;

// ״̬Э����
typedef struct {
    double P[3][3]; // 3x3 Э�������
} Covariance;
extern int if_laser_ready;
extern State state ;
extern Covariance cov ; 
extern double process_noise[3][3] ;
extern double measurement_noise[3][3];
extern int if_get_laser;
extern uint8_t Pc_buffer[11];
extern uint8_t if_gyro_right;
extern volatile uint8_t gyro_buffer[33];
void go_jiaozhun(void);
void Merge_init();
void jiguang_restart_position(float x ,float y ,float r,float vx,float vy,float vr);
void update(State *state, Covariance *cov, float z[3], double R[3][3]);
void predict(State *state, Covariance *cov, double imu_x, double imu_y, double imu_theta, double Q[3][3]);
void USART_printf(char *fmt, ...);
void restart_position();
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
