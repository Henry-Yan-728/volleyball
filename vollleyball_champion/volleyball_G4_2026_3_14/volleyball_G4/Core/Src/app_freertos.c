/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fdcan_bsp.h"
#include "cybergear_motor.h"
#include "trajectory_planner.h"
#include "chassis_cybergear.h"

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
/* USER CODE BEGIN Variables */
static uint8_t s_cybergear_boot_ok = 0U;
static uint8_t s_cybergear_chassis_ready = 0U;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for myTask06 */
osThreadId_t myTask06Handle;
const osThreadAttr_t myTask06_attributes = {
  .name = "myTask06",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for myTask07 */
osThreadId_t myTask07Handle;
const osThreadAttr_t myTask07_attributes = {
  .name = "myTask07",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for BinarySem_BallDetect */
osSemaphoreId_t BinarySem_BallDetectHandle;
const osSemaphoreAttr_t BinarySem_BallDetect_attributes = {
  .name = "BinarySem_BallDetect"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);
void StartTask07(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  fdcan_bsp_init();
  cybergear_motors_init();

  fdcan_bsp_start(&hfdcan1);
  fdcan_bsp_start(&hfdcan2);
  fdcan_bsp_start(&hfdcan3);

  if (trajectory_planner_init() == 0)
  {
    s_cybergear_boot_ok = 1U;
  }

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of BinarySem_BallDetect */
  BinarySem_BallDetectHandle = osSemaphoreNew(1, 1, &BinarySem_BallDetect_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(StartTask04, NULL, &myTask04_attributes);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(StartTask05, NULL, &myTask05_attributes);

  /* creation of myTask06 */
  myTask06Handle = osThreadNew(StartTask06, NULL, &myTask06_attributes);

  /* creation of myTask07 */
  myTask07Handle = osThreadNew(StartTask07, NULL, &myTask07_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    if (s_cybergear_boot_ok == 1U && s_cybergear_chassis_ready == 0U)
    {
      if (chassis_cybergear_init(0U) == 0)
      {
        s_cybergear_chassis_ready = 1U;
      }
      else
      {
        osDelay(50);
        continue;
      }
    }

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    chassis_control(0);
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the myTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief Function implementing the myTask07 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask07 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

