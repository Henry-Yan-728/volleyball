/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
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
#include <math.h>
#include <string.h>
#include <stdio.h>

// --- 锟藉级支锟街帮拷 (BSP) 锟斤拷锟接诧拷锟斤拷锟斤拷锟侥ｏ拷锟? ---
#include "fdcan_bsp.h"      // CAN锟斤拷锟斤拷锟斤拷锟斤拷
#include "robot_data.h"     // 全锟街伙拷锟斤拷锟斤拷锟斤拷锟捷结构锟斤拷
#include "chassis_task.h"   // 锟斤拷锟教匡拷锟斤拷锟斤拷锟斤拷
#include "mechanism_task.h" // 锟斤拷械锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟今（伙拷锟斤拷/锟斤拷锟斤拷龋锟?
#include "usart.h" 

// --- 遥锟斤拷锟斤拷协锟介处锟斤拷锟斤拷指锟斤拷锟斤拷锟? ---
#include "Task_command.h"   // 锟结供指锟斤拷锟斤拷锟斤拷锟截猴拷锟斤拷
#include "remote_driver.h"  // 锟结供遥锟斤拷锟斤拷原始锟斤拷锟捷结构锟藉定锟斤拷
#include "queue.h"          // FreeRTOS锟斤拷锟斤拷支锟斤拷
#include "semphr.h"         // FreeRTOS锟脚猴拷锟斤拷/锟斤拷锟斤拷锟斤拷支锟斤拷
#include "chassis_path_task.h" // 路锟斤拷锟芥划锟斤拷锟斤拷
#include "Pan_Tilt_control.h"  // 锟斤拷台锟斤拷锟斤拷

// 锟解部硬锟斤拷锟斤拷锟斤拷锟斤拷锟?
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern UART_HandleTypeDef huart10; 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifndef M_PI
#define M_PI 3.1415926535f  // 圆锟斤拷锟绞讹拷锟斤拷
#endif

// 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟轿?锟斤拷锟斤拷
#define power_add 0.3
// 锟斤拷锟斤拷锟斤拷时锟斤拷锟斤拷锟斤拷ms锟斤拷
#define time_add 1

// --- 锟斤拷锟斤拷锟斤拷锟阶刺?枚锟劫ｏ拷锟街诧拷执锟叫凤拷锟斤拷锟斤拷锟斤拷---
typedef enum {
    ACTION_IDLE = 0, // 锟斤拷锟斤拷状态锟斤拷锟睫讹拷锟斤拷锟斤拷
    ACTION_STEP_1,   // 执锟叫阶讹拷1锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟教э拷锟?
    ACTION_STEP_2    // 执锟叫阶讹拷2锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟轿伙拷锟?
} ActionState_e;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RC_BUFFER_SIZE  64  // 遥锟斤拷锟斤拷锟斤拷锟节斤拷锟秸伙拷锟斤拷锟斤拷锟斤拷小
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
float time_ms = 1;                  // 锟斤拷锟斤拷锟斤拷时锟斤拷锟斤拷值锟斤拷ms锟斤拷
float PITCH = 4.06f;                // Pitch锟结缓锟斤拷锟斤拷锟侥?锟斤拷锟劫讹拷

// --- FreeRTOS 锟节核讹拷锟斤拷锟斤拷 ---
osMutexId_t rc_mutexHandle;            // 锟斤拷锟斤拷全锟斤拷遥锟斤拷锟斤拷锟斤拷锟斤拷 g_remote_data 锟侥伙拷锟斤拷锟斤拷
osMessageQueueId_t remote_queueHandle; // 锟斤拷息锟斤拷锟叫ｏ拷锟斤拷锟节斤拷锟秸达拷锟斤拷锟叫断碉拷原始遥锟斤拷锟斤拷锟斤拷锟斤拷

// 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
const osMutexAttr_t rc_mutex_attributes = {
  .name = "rcMutex"
};

// 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷藕锟斤拷锟斤拷锟斤拷锟解到锟斤拷时锟斤拷锟斤拷锟斤拷
osSemaphoreId_t BinarySem_BallDetectHandle;
const osSemaphoreAttr_t BinarySem_BallDetect_attributes = {
  .name = "BinarySem_BallDetect"
};

// --- 全锟斤拷业锟斤拷锟斤拷锟? ---
remote_engineer_t g_remote_data = {0}; // 全锟斤拷遥锟斤拷锟斤拷锟斤拷锟捷ｏ拷锟斤拷锟斤拷锟斤拷
uint8_t remote_Buffer[RC_BUFFER_SIZE]; // 遥锟斤拷锟斤拷DMA锟斤拷锟斤拷原始锟斤拷锟捷伙拷锟斤拷锟斤拷
uint8_t processsed_command[COMMAND_LENGTH]; // 指锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟捷伙拷锟斤拷锟斤拷

// 锟解部锟斤拷锟斤拷锟斤拷遥锟斤拷锟斤拷原始锟斤拷锟捷结构锟斤拷
extern rc_info_t rc; 
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask06 */
osThreadId_t myTask06Handle;
const osThreadAttr_t myTask06_attributes = {
  .name = "myTask06",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask07 */
osThreadId_t myTask07Handle;
const osThreadAttr_t myTask07_attributes = {
  .name = "myTask07",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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
  // 1. 硬锟斤拷锟斤拷锟斤拷锟斤拷始锟斤拷锟斤拷锟阶诧拷锟斤拷锟借）
  fdcan_bsp_init();     // CAN锟斤拷锟竭底诧拷锟绞硷拷锟?
  Robot_Data_Init();    // 全锟街伙拷锟斤拷锟斤拷锟斤拷锟捷结构锟斤拷锟绞硷拷锟?
  Chassis_Init();       // 锟斤拷锟教匡拷锟狡筹拷始锟斤拷
  Mechanism_Init();     // 锟斤拷械锟斤拷锟斤拷锟斤拷始锟斤拷锟斤拷锟斤拷锟斤拷/锟斤拷锟斤拷龋锟?

  HAL_Delay(100); // 硬锟斤拷锟饺讹拷锟斤拷时

  // 2. 锟斤拷锟斤拷CAN锟斤拷锟斤拷通锟斤拷
  fdcan_bsp_start(&hfdcan1); 
  fdcan_bsp_start(&hfdcan2); 
  fdcan_bsp_start(&hfdcan3); 

  // 3. 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷遥锟斤拷锟斤拷锟斤拷锟捷ｏ拷
  rc_mutexHandle = osMutexNew(&rc_mutex_attributes);

  // 4. 锟斤拷锟斤拷遥锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷息锟斤拷锟叫ｏ拷锟斤拷锟叫筹拷锟斤拷16锟斤拷每锟斤拷元锟斤拷为UartRxMessage_t锟斤拷锟酵ｏ拷
  remote_queueHandle = osMessageQueueNew(16, sizeof(UartRxMessage_t), NULL);
	
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟? */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷藕锟斤拷锟斤拷锟斤拷锟绞贾?1锟斤拷锟斤拷锟斤拷锟斤拷1锟斤拷
  BinarySem_BallDetectHandle = osSemaphoreNew(1, 1, &BinarySem_BallDetect_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* 锟斤拷时锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟睫ｏ拷 */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* 锟斤拷锟叫达拷锟斤拷锟斤拷锟? */
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
  /* 锟斤拷锟今创斤拷锟斤拷锟? */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* 锟铰硷拷锟斤拷志锟介（锟斤拷锟睫ｏ拷 */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  默锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷拢锟?1ms锟斤拷锟节ｏ拷
  * @param  argument: 未使锟斤拷
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* 锟斤拷锟斤拷循锟斤拷 */
  for(;;)
  {
    Update_Virtual_Axis();
    Chassis_Task_Loop();
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief  锟斤拷锟侥匡拷锟斤拷锟斤拷锟今：碉拷锟斤拷+锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷疲锟?10ms锟斤拷锟节ｏ拷
* @param  argument: 未使锟斤拷
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms执锟斤拷锟斤拷锟节ｏ拷100Hz锟斤拷
  xLastWakeTime = osKernelGetTickCount();

  // --- 1. 锟街诧拷锟斤拷锟斤拷锟斤拷始锟斤拷 ---
  remote_engineer_t local_rc = {0}; // 锟斤拷锟斤拷遥锟斤拷锟斤拷锟斤拷锟捷ｏ拷锟斤拷锟斤拷频锟斤拷锟斤拷锟斤拷全锟街憋拷锟斤拷锟斤拷
  float target_vx = 0, target_vy = 0, target_vr = 0; // 锟斤拷锟斤拷目锟斤拷锟劫度ｏ拷x/y锟斤拷平锟狡ｏ拷z锟斤拷锟斤拷转锟斤拷
  
  // --- 2. 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟? ---
  const float CUSHION_READY_DEG = 47.0f;   // 锟斤拷锟斤拷锟斤拷锟斤拷锟绞硷拷嵌龋锟斤拷悖?
  const float CUSHION_ACTION_DEG = 114.0f;// 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷嵌龋锟斤拷悖?
  const float SERVE_READY_RAD    = 30.0f; // 锟斤拷锟斤拷锟斤拷锟斤拷锟绞硷拷嵌龋锟斤拷悖?
  const float SERVE_ACTION_RAD   = 210.0f;// 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷嵌龋锟斤拷悖?
  float PITCH_ANGLE = 0.0f;               // Pitch锟斤拷目锟斤拷嵌锟?

  // 锟斤拷锟斤拷状态锟斤拷锟斤拷锟斤拷
  ActionState_e serve_state = ACTION_IDLE;
  uint32_t serve_start_tick = 0;          // 锟斤拷锟斤拷锟斤拷锟斤拷始时锟斤拷锟?
  
  // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷时锟斤拷锟斤拷锟斤拷锟叫碉拷锟斤拷锟斤拷蟠シ锟斤拷锟?
  #define DEBOUNCE_DELAY_MS 10

  // 锟斤拷锟斤拷状态锟斤拷史锟斤拷锟斤拷录锟斤拷一帧状态锟斤拷锟斤拷锟斤拷锟截达拷锟斤拷锟斤拷
  static uint32_t last_time_btn5 = 0; // 锟斤拷锟斤拷5锟较次达拷锟斤拷时锟斤拷
  static uint32_t last_time_btn2 = 0; // 锟斤拷锟斤拷2锟较次达拷锟斤拷时锟斤拷
  static uint32_t last_time_btn4 = 0; // 锟斤拷锟斤拷4锟较次达拷锟斤拷时锟斤拷
  static uint32_t last_time_btn1 = 0; // 锟斤拷锟斤拷1锟较次达拷锟斤拷时锟斤拷
  static uint32_t last_time_btn3 = 0; // 锟斤拷锟斤拷3锟较次达拷锟斤拷时锟斤拷

  // 锟斤拷锟斤拷锟斤拷一帧状态
  static uint8_t last_button5 = 0;
  static uint8_t last_button2 = 0;
  static uint8_t last_button4 = 0;
  static uint8_t last_button1 = 0;
  static uint8_t last_button3 = 0;
  
  // 锟皆讹拷模式锟斤拷锟斤拷状态
  static uint8_t last_button_front = 0;
  static uint8_t last_button_back = 0;

  /* 锟斤拷锟斤拷循锟斤拷 */
  for(;;)
  {
      uint32_t current_tick = HAL_GetTick(); // 锟斤拷取系统锟斤拷前时锟斤拷锟斤拷锟絤s锟斤拷

      // ============================================================
      // 1. 锟竭程帮拷全锟斤拷取全锟斤拷遥锟斤拷锟斤拷锟斤拷锟捷ｏ拷锟接伙拷锟斤拷锟斤拷锟斤拷
      // ============================================================
      if (osMutexAcquire(rc_mutexHandle, 10) == osOK) {
          local_rc = g_remote_data; // 锟斤拷锟斤拷全锟斤拷锟斤拷锟捷碉拷锟斤拷锟斤拷
          osMutexRelease(rc_mutexHandle); // 锟酵放伙拷锟斤拷锟斤拷
      }
      printf("%f\r\n",local_rc.vx);
			osDelay(200);
      // ============================================================
      // 2. 锟斤拷锟斤拷模式锟叫伙拷锟竭硷拷锟斤拷锟街讹拷/锟皆讹拷/锟斤拷锟斤拷位锟斤拷
      // ============================================================
      if (local_rc.mode == CHASSIS_MODE_MANUAL) 
      {
          /* --- 锟街讹拷模式锟斤拷遥锟斤拷锟斤拷直锟接匡拷锟狡碉拷锟斤拷 --- */
          target_vx = local_rc.vx / 100.0f;  // X锟斤拷锟劫度ｏ拷锟斤拷一锟斤拷锟斤拷-1~1锟斤拷
          target_vy = local_rc.vy / 100.0f;  // Y锟斤拷锟劫度ｏ拷锟斤拷一锟斤拷锟斤拷-1~1锟斤拷
          target_vr = -local_rc.vw * 3.0f;   // 锟斤拷转锟劫度ｏ拷锟脚达拷系锟斤拷3锟斤拷
          // --- 锟斤拷锟斤拷5锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟截达拷锟斤拷+锟斤拷锟斤拷锟斤拷---
          if (local_rc.button5 == 1 && last_button5 == 0) {
              if ((current_tick - last_time_btn5) > DEBOUNCE_DELAY_MS) {
                  if (serve_state == ACTION_IDLE) { // 锟斤拷锟斤拷锟斤拷时锟斤拷锟斤拷
                      serve_state = ACTION_STEP_1;
                      serve_start_tick = current_tick;
                  }
                  last_time_btn5 = current_tick; // 锟斤拷锟斤拷锟较次达拷锟斤拷时锟斤拷
              }
          }
          last_button5 = local_rc.button5; // 锟斤拷锟铰帮拷锟斤拷状态

          // --- 锟斤拷锟斤拷2/4锟斤拷Pitch锟斤拷嵌却值锟斤拷锟斤拷锟?5锟姐）---
          if (local_rc.button2 == 1 && last_button2 == 0) {
              if ((current_tick - last_time_btn2) > DEBOUNCE_DELAY_MS) {
                  PITCH_ANGLE += 5.0f;
                  Mechanism_Dian_Pitch_SetAngle(PITCH_ANGLE); // 锟斤拷锟斤拷Pitch锟角讹拷
                  last_time_btn2 = current_tick;
              }
          }
          last_button2 = local_rc.button2; 

          if (local_rc.button4 == 1 && last_button4 == 0) {
              if ((current_tick - last_time_btn4) > DEBOUNCE_DELAY_MS) {
                  PITCH_ANGLE -= 5.0f;
                  Mechanism_Dian_Pitch_SetAngle(PITCH_ANGLE);
                  last_time_btn4 = current_tick;
              }
          }
          last_button4 = local_rc.button4; 

          // --- 锟斤拷锟斤拷1/3锟斤拷Pitch锟斤拷锟劫讹拷微锟斤拷锟斤拷锟斤拷0.3锟斤拷---
          if (local_rc.button1 == 1 && last_button1 == 0) {
              if ((current_tick - last_time_btn1) > DEBOUNCE_DELAY_MS) {
                  PITCH += power_add;
                  last_time_btn1 = current_tick;
              }
          }
          last_button1 = local_rc.button1; 

          if (local_rc.button3 == 1 && last_button3 == 0) {
              if ((current_tick - last_time_btn3) > DEBOUNCE_DELAY_MS) {
                  PITCH -= power_add;
                  last_time_btn3 = current_tick;
              }
          }
          last_button3 = local_rc.button3; 

          // 锟斤拷锟铰碉拷锟斤拷锟劫讹拷指锟斤拷
          Chassis_Update(target_vx, target_vy, target_vr);
      }
      else if (local_rc.mode == CHASSIS_MODE_AUTO)
      {
          /* --- 锟皆讹拷模式锟斤拷路锟斤拷锟芥划锟斤拷锟狡碉拷锟斤拷 --- */
          float cur_x, cur_y, cur_yaw;    // 锟斤拷锟斤拷锟剿碉拷前位锟剿ｏ拷x/y锟斤拷锟疥，偏锟斤拷锟角ｏ拷
          uint32_t last_update;           // 位锟斤拷锟斤拷锟斤拷锟斤拷时锟斤拷
          
          // 锟劫斤拷锟斤拷锟斤拷取全锟斤拷位锟剿ｏ拷锟斤拷锟斤拷锟斤拷叱坛锟酵伙拷锟?
          taskENTER_CRITICAL();
          cur_x = g_robot_pose.x;
          cur_y = g_robot_pose.y;
          cur_yaw = g_robot_pose.angle;
          last_update = g_robot_pose.update_tick;
          taskEXIT_CRITICAL();

          // 位锟斤拷锟斤拷锟捷筹拷时锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷1s未锟斤拷锟斤拷锟斤拷停止锟斤拷锟教ｏ拷
          if (HAL_GetTick() - last_update > 1000) {
               target_vx = 0; target_vy = 0; target_vr = 0;
          }
          else 
          {
              // 锟斤拷锟斤拷目锟斤拷位锟斤拷偏锟斤拷
              float err_x = g_robot_target.target_x - cur_x;
              float err_y = g_robot_target.target_y - cur_y;
              
              // 路锟斤拷锟芥划锟斤拷始锟斤拷锟斤拷锟斤拷锟阶达拷执锟叫ｏ拷
              static uint8_t initialized = 0;
              if (!initialized) {
                  Planner_SetTarget(cur_x, cur_y, err_x, err_y, 0.0f); 
                  initialized = 1;
              }

              // 锟斤拷锟斤拷路锟斤拷锟芥划锟斤拷锟斤拷取锟劫讹拷指锟斤拷
              TrajVel_t cmd = Planner_Update(cur_x, cur_y, cur_yaw);
              
              // 锟斤拷锟斤拷浠伙拷锟斤拷锟斤拷锟斤拷锟斤拷锟较? 锟斤拷 锟斤拷锟斤拷锟剿憋拷锟斤拷锟斤拷锟斤拷系
              float theta = cur_yaw * (M_PI / 180.0f); // 锟角讹拷转锟斤拷锟斤拷
              target_vx =  cmd.vx * cosf(theta) + cmd.vy * sinf(theta);
              target_vy = -cmd.vx * sinf(theta) + cmd.vy * cosf(theta);
              target_vr = cmd.vr;
              
              // 锟斤拷锟斤拷目锟斤拷位锟剿猴拷锟斤拷锟矫规划锟斤拷
              if (Planner_IsArrived(cur_x, cur_y, cur_yaw)) {
                  initialized = 0;
              }
          }
          Chassis_Update(target_vx, target_vy, target_vr);
      }
      else if (local_rc.mode == CHASSIS_MODE_SERVE)
      {
          /* --- 锟斤拷锟斤拷位模式锟斤拷锟斤拷锟斤拷锟狡讹拷锟斤拷指锟斤拷位锟矫猴拷锟斤拷 --- */
          float cur_x, cur_y, cur_yaw, target_x = 0.0f, target_y = 0.0f;
          uint32_t last_update;
          
          // 锟劫斤拷锟斤拷锟斤拷取锟斤拷前位锟斤拷
          taskENTER_CRITICAL();
          cur_x = g_robot_pose.x;
          cur_y = g_robot_pose.y;
          cur_yaw = g_robot_pose.angle;
          last_update = g_robot_pose.update_tick;
          taskEXIT_CRITICAL();
				
          // 锟斤拷锟斤拷5锟斤拷锟斤拷位锟斤拷原锟姐（0,0锟斤拷
          if (local_rc.button5 == 1 && last_button5 == 0) {
              target_x = 0.0f;
              target_y = 0.0f;
          }
          last_button5 = local_rc.button5;
				
          // 锟斤拷锟斤拷2锟斤拷前锟斤拷前锟斤拷位锟矫ｏ拷800,500锟斤拷锟斤拷锟斤拷锟斤拷4锟斤拷前锟斤拷锟斤拷位锟矫ｏ拷1000,1000锟斤拷
          if (local_rc.button2 == 1 && last_button_front == 0) {
              target_x = 800.0f;
              target_y = 500.0f;
          }else if (local_rc.button4 == 1 && last_button_back == 0) {
              target_x = 1000.0f;
              target_y = 1000.0f;
          }
          last_button_front = local_rc.button2;
          last_button_back = local_rc.button4;

          // 位锟斤拷锟斤拷锟捷筹拷时锟斤拷锟斤拷
          if (HAL_GetTick() - last_update > 1000) {
               target_vx = 0; target_vy = 0; target_vr = 0;
          }
          else 
          {
              // 锟斤拷锟斤拷目锟斤拷偏锟斤拷
              float err_x = target_x - cur_x;
              float err_y = target_y - cur_y;
              
              // 路锟斤拷锟芥划锟斤拷始锟斤拷
              static uint8_t initialized = 0;
              if (!initialized) {
                  Planner_SetTarget(cur_x, cur_y, err_x, err_y, 0.0f);
                  initialized = 1;
              }

              // 锟斤拷锟斤拷路锟斤拷锟芥划
              TrajVel_t cmd = Planner_Update(cur_x, cur_y, cur_yaw);
              
              // 锟斤拷锟斤拷浠?
              float theta = cur_yaw * (M_PI / 180.0f);
              target_vx =  cmd.vx * cosf(theta) + cmd.vy * sinf(theta);
              target_vy = -cmd.vx * sinf(theta) + cmd.vy * cosf(theta);
              target_vr = cmd.vr;
              
              // 锟斤拷锟铰碉拷锟斤拷锟劫讹拷
              Chassis_Update(target_vx, target_vy, target_vr);

              // 锟斤拷锟斤拷目锟斤拷位锟矫猴拷锟皆讹拷锟斤拷锟斤拷锟斤拷锟斤拷
              if (Planner_IsArrived(cur_x, cur_y, cur_yaw)) {
                  if (serve_state == ACTION_IDLE) {
                      serve_state = ACTION_STEP_1;
                      serve_start_tick = current_tick;
                  }
              }
          }
      }

      // ============================================================
      // 3. 锟斤拷锟斤拷锟斤拷锟阶刺?锟斤拷执锟叫ｏ拷锟街诧拷锟斤拷锟斤拷锟斤拷
      // ============================================================
      switch (serve_state) {
          case ACTION_STEP_1:
              // 锟阶讹拷1锟斤拷锟斤拷锟斤拷锟斤拷锟教э拷锟? + 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟?
              Mechanism_Cushion_SetAngle(CUSHION_ACTION_DEG, PITCH);
              Mechanism_Serve_SetAngle(SERVE_ACTION_RAD); 

              // 锟斤拷时300ms锟斤拷锟斤拷锟阶讹拷2
              if (current_tick - serve_start_tick > 300) {
                  serve_state = ACTION_STEP_2;
              }
              break;

          case ACTION_STEP_2:
              // 锟阶讹拷2锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟轿伙拷锟斤拷锟斤拷伲锟?+ 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟?
              Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, 0.3f);
              Mechanism_Serve_SetAngle(SERVE_ACTION_RAD);

              // 锟斤拷锟斤拷时800ms锟斤拷位锟斤拷锟斤拷锟斤拷状态
              if (current_tick - serve_start_tick > 800) {
                  Mechanism_Serve_SetAngle(SERVE_READY_RAD); // 锟斤拷锟斤拷锟斤拷锟斤拷锟轿?
                  serve_state = ACTION_IDLE;
              }
              break;

          default: break;
      }

      // 锟斤拷准锟斤拷时锟斤拷锟斤拷证10ms锟斤拷锟节ｏ拷
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief  CAN2锟斤拷锟斤拷锟斤拷锟今：伙拷锟斤拷锟斤拷位锟斤拷锟较憋拷锟斤拷锟斤拷频锟斤拷
* @param  argument: 未使锟斤拷
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  FDCAN_TxHeaderTypeDef TxHeader; // CAN锟斤拷锟斤拷帧头
  uint8_t TxData[12];             // CAN锟斤拷锟斤拷锟斤拷锟捷ｏ拷12锟街节ｏ拷
  
  // 锟斤拷始锟斤拷CAN锟斤拷锟斤拷帧头
  TxHeader.Identifier = 0x101;                // 帧ID锟斤拷0x101
  TxHeader.IdType = FDCAN_STANDARD_ID;        // 锟斤拷准ID锟斤拷11位锟斤拷
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;    // 锟斤拷锟斤拷帧
  TxHeader.DataLength = FDCAN_DLC_BYTES_12;   // 锟斤拷锟捷筹拷锟饺ｏ拷12锟街斤拷
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // 锟斤拷锟斤拷状态锟斤拷锟斤拷锟斤拷
  TxHeader.BitRateSwitch = FDCAN_BRS_ON;      // 锟斤拷锟斤拷锟斤拷锟叫伙拷锟斤拷锟斤拷锟斤拷
  TxHeader.FDFormat = FDCAN_FD_CAN;           // FD CAN锟斤拷式
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟铰硷拷
  TxHeader.MessageMarker = 0;                 // 锟斤拷息锟斤拷牵锟?0

  /* 锟斤拷锟斤拷循锟斤拷 */
  for(;;)
  {
      float temp_x, temp_y, temp_angle;
      
      // 锟劫斤拷锟斤拷锟斤拷取位锟剿ｏ拷锟斤拷锟斤拷锟斤拷叱坛锟酵伙拷锟?
      taskENTER_CRITICAL();
      temp_x = g_robot_pose.x;
      temp_y = g_robot_pose.y;
      temp_angle = g_robot_pose.angle;
      taskEXIT_CRITICAL();

      // 锟斤拷锟捷达拷锟斤拷锟紽loat 锟斤拷 Byte锟斤拷
      memcpy(&TxData[0], &temp_x, 4);     // 0-3锟街节ｏ拷X锟斤拷锟疥（float锟斤拷
      memcpy(&TxData[4], &temp_y, 4);     // 4-7锟街节ｏ拷Y锟斤拷锟疥（float锟斤拷
      memcpy(&TxData[8], &temp_angle, 4); // 8-11锟街节ｏ拷偏锟斤拷锟角ｏ拷float锟斤拷

      // 锟斤拷锟酵碉拷CAN2锟斤拷锟斤拷
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData);
      osDelay(1); // 约1000Hz锟斤拷锟斤拷频锟斤拷
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief  遥锟斤拷锟斤拷锟斤拷锟捷斤拷锟斤拷锟斤拷锟今：达拷锟斤拷锟斤拷DMA锟斤拷锟秸碉拷原始锟斤拷锟斤拷
* @param  argument: 未使锟斤拷
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  // 锟斤拷始锟斤拷UART2 DMA循锟斤拷锟斤拷锟秸ｏ拷ReceiveToIdle模式锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟叫断ｏ拷
  HAL_UARTEx_ReceiveToIdle_DMA(&huart10, remote_Buffer, RC_BUFFER_SIZE);
  // 锟斤拷锟斤拷DMA锟诫传锟斤拷锟叫断ｏ拷锟斤拷锟斤拷锟斤拷一帧锟斤拷锟斤拷锟斤拷锟捷ｏ拷
  __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT); 

  UartRxMessage_t rx_msg;        // 锟斤拷锟节斤拷锟斤拷锟斤拷息锟结构锟斤拷
  remote_engineer_t temp_rc;     // 锟斤拷时遥锟斤拷锟斤拷锟斤拷锟斤拷

  /* 锟斤拷锟斤拷循锟斤拷 */
  for(;;)
  {
      // 锟斤拷锟斤拷锟饺达拷锟斤拷息锟斤拷锟叫ｏ拷锟斤拷锟皆达拷锟斤拷锟叫断碉拷原始锟斤拷锟捷ｏ拷
      if (osMessageQueueGet(remote_queueHandle, &rx_msg, NULL, osWaitForever) == osOK)
      {
          // 锟斤拷锟斤拷锟斤拷锟斤拷原始锟斤拷锟斤拷为指锟斤拷锟绞?
          Command_Write(rx_msg.data, rx_msg.size);
          
          // 循锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷指锟斤拷
          while (Command_GetCommand(processsed_command) != 0)
          {
              code_unzipread(processsed_command);   // 锟斤拷锟揭ｏ拷锟斤拷锟叫?锟斤拷
              Remote_Data_Convert(&rc, &temp_rc);   // 转锟斤拷为锟斤拷准锟斤拷锟捷革拷式
              
              // 锟竭程帮拷全锟斤拷锟斤拷全锟斤拷遥锟斤拷锟斤拷锟斤拷锟斤拷
              if (osMutexAcquire(rc_mutexHandle, 10) == osOK) 
              {
                  g_remote_data = temp_rc;
                  osMutexRelease(rc_mutexHandle);
              }
          }
      }
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief  锟斤拷锟斤拷锟斤拷锟今：硷拷獾斤拷锟襟触凤拷锟斤拷锟斤拷锟斤拷
* @param  argument: 未使锟斤拷
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟?
  #define CUSHION_READY_DEG   47.0f    // 锟斤拷始锟角讹拷
  #define CUSHION_SPEED      2.06f     // 锟斤拷始锟劫讹拷
  #define ACTION_HOLD_MS      800      // 锟斤拷锟斤拷锟斤拷锟斤拷时锟戒（ms锟斤拷
  const float CUSHION_ACTION_DEG = 114.0f; // 锟斤拷锟斤拷锟角讹拷

  osDelay(1000); // 系统锟斤拷锟斤拷锟斤拷时锟斤拷锟饺达拷硬锟斤拷锟饺讹拷锟斤拷
  
  // 锟斤拷始锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟绞嘉伙拷茫锟剿?锟斤拷锟斤拷锟斤拷确锟斤拷锟斤拷位锟斤拷
  Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, CUSHION_SPEED);
  osDelay(500);
  Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, CUSHION_SPEED);
  printf("System Ready. Angle reset.\r\n"); // 系统锟斤拷锟斤拷锟斤拷示

  /* 锟斤拷锟斤拷循锟斤拷 */
  for(;;)
  {
    // 1. 锟斤拷取锟斤拷锟斤拷锟斤拷锟斤拷锟脚号ｏ拷锟酵碉拷平锟斤拷示锟斤拷獾斤拷锟?
    if (HAL_GPIO_ReadPin(PHOTO_GATE_GPIO_Port, PHOTO_GATE_Pin) == GPIO_PIN_RESET)
    {
        osDelay(5); // 硬锟斤拷锟斤拷锟斤拷锟斤拷5ms锟斤拷
        if (HAL_GPIO_ReadPin(PHOTO_GATE_GPIO_Port, PHOTO_GATE_Pin) == GPIO_PIN_RESET)
        {
            // 锟斤拷时锟斤拷锟斤拷锟斤拷锟斤拷锟解负锟斤拷锟斤拷
            if(time_ms < 0)
            {
                time_ms = 0;
            }
            osDelay(time_ms); // 锟斤拷锟斤拷锟斤拷时锟斤拷锟斤拷锟斤拷锟矫ｏ拷
            
            // === 锟斤拷獾斤拷锟街达拷蟹锟斤拷锟斤拷锟? ===
            printf("Ball detected! Action!\r\n");

            // 锟斤拷锟斤拷锟斤拷锟教э拷锟?
            Mechanism_Cushion_SetAngle(CUSHION_ACTION_DEG, PITCH);
            osDelay(ACTION_HOLD_MS); // 锟斤拷锟街讹拷锟斤拷确锟斤拷锟斤拷锟斤拷锟斤拷锟?

            // 锟斤拷锟斤拷锟斤拷锟斤拷锟轿?
            Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, 0.3f);
            
            // 锟饺达拷锟斤拷锟诫开锟斤拷锟斤拷牛锟斤拷锟斤拷锟斤拷馗锟斤拷锟斤拷锟斤拷锟?
            while (HAL_GPIO_ReadPin(PHOTO_GATE_GPIO_Port, PHOTO_GATE_Pin) == GPIO_PIN_RESET)
            {
                osDelay(10);
            }
            
            osDelay(500); // 锟斤拷位锟斤拷锟饺讹拷锟斤拷时
            printf("Ready.\r\n"); // 锟斤拷锟斤拷锟斤拷示
        }
    }
    // 锟斤拷锟斤拷锟斤拷冢锟?500ms
    osDelay(500);
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief  锟斤拷械锟斤拷锟斤拷循锟斤拷锟斤拷锟斤拷1ms锟斤拷锟节革拷锟铰伙拷锟斤拷状态
* @param  argument: 未使锟斤拷
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* 锟斤拷锟斤拷循锟斤拷 */
  for(;;)
  {
    Mechanism_Loop_1ms(); // 锟斤拷械锟斤拷锟斤拷1ms锟斤拷锟节达拷锟斤拷锟秸伙拷锟斤拷锟斤拷/状态锟斤拷锟铰ｏ拷
    osDelay(1);           // 1ms锟斤拷时
  }
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief  CAN3锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷台锟角讹拷锟较憋拷锟斤拷1ms锟斤拷锟节ｏ拷
* @param  argument: 未使锟斤拷
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
  FDCAN_TxHeaderTypeDef TxHeader; // CAN锟斤拷锟斤拷帧头
  uint8_t TxData[12];             // 锟斤拷锟斤拷锟斤拷锟捷伙拷锟斤拷锟斤拷
  
  // 锟斤拷始锟斤拷CAN锟斤拷锟斤拷帧头
  TxHeader.Identifier = CAN_ID_PC_FEEDBACK; // 帧ID锟斤拷0x300锟斤拷PC锟斤拷锟斤拷专锟矫ｏ拷
  TxHeader.IdType = FDCAN_STANDARD_ID;      // 锟斤拷准ID
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;  // 锟斤拷锟斤拷帧
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;  // 锟斤拷锟捷筹拷锟饺ｏ拷8锟街斤拷
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;   // 锟截闭诧拷锟斤拷锟斤拷锟叫伙拷锟斤拷锟斤拷统CAN锟斤拷
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;    // 锟斤拷统CAN锟斤拷式
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
  
  // 锟斤拷准锟斤拷时锟斤拷始锟斤拷锟斤拷1ms锟斤拷锟节ｏ拷
  TickType_t xLastWakeTime = osKernelGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms执锟斤拷锟斤拷锟斤拷

  /* 锟斤拷锟斤拷循锟斤拷 */
  for(;;)
  {
      uint32_t current_timestamp = HAL_GetTick(); // 锟斤拷前时锟斤拷锟斤拷锟絤s锟斤拷
      float yaw_angle = 0.0f;                     // 锟斤拷台Yaw锟斤拷嵌锟?
      float pitch_angle = 0.0f;                   // 锟斤拷台Pitch锟斤拷嵌锟?
      
      // 锟斤拷取锟斤拷台实时锟角讹拷
      gimbal_get_angles(&yaw_angle, &pitch_angle);

      // 锟斤拷锟斤拷压锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷*100转为int16_t锟斤拷锟斤拷省锟斤拷锟斤拷锟斤拷锟斤拷2位小锟斤拷锟斤拷
      int16_t yaw_send   = (int16_t)(yaw_angle * 100.0f);
      int16_t pitch_send = (int16_t)(pitch_angle * 100.0f);

      // 锟斤拷锟捷达拷锟?
      memcpy(&TxData[0], &current_timestamp, 4); // 0-3锟街节ｏ拷时锟斤拷锟斤拷锟絬int32_t锟斤拷
      memcpy(&TxData[4], &yaw_send, 2);          // 4-5锟街节ｏ拷Yaw锟角度ｏ拷int16_t锟斤拷
      memcpy(&TxData[6], &pitch_send, 2);        // 6-7锟街节ｏ拷Pitch锟角度ｏ拷int16_t锟斤拷

      // 锟斤拷锟酵碉拷CAN3锟斤拷锟斤拷
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData);

      // 锟斤拷准锟斤拷时锟斤拷锟斤拷证1ms锟斤拷锟节ｏ拷
      vTaskDelayUntil(&xLastWakeTime, xFrequency);  
  }
  /* USER CODE END StartTask07 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* -------------------------------------------------------------------------
// 锟斤拷锟斤拷DMA锟斤拷锟斤拷锟斤拷锟斤拷卸匣氐锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟揭ｏ拷锟斤拷锟皆?始锟斤拷锟捷ｏ拷
// ------------------------------------------------------------------------- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    
    if (huart->Instance == USART10) { // 锟斤拷锟斤拷锟斤拷USART10锟斤拷遥锟斤拷锟斤拷锟斤拷锟节ｏ拷
        UartRxMessage_t rx_msg;
        // 锟斤拷锟捷筹拷锟饺憋拷锟斤拷锟斤拷锟斤拷锟解缓锟斤拷锟斤拷锟斤拷锟斤拷锟?
        uint16_t copy_size = (Size < sizeof(rx_msg.data)) ? Size : sizeof(rx_msg.data);
        
        // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟捷碉拷锟斤拷息锟结构锟斤拷
        memcpy(rx_msg.data, remote_Buffer, copy_size);
        rx_msg.size = copy_size;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // 锟叫讹拷锟叫凤拷锟斤拷锟斤拷息锟斤拷锟斤拷锟叫ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷
        xQueueSendFromISR(remote_queueHandle, &rx_msg, &xHigherPriorityTaskWoken);
        
        // 锟斤拷锟斤拷DMA锟斤拷锟秸ｏ拷循锟斤拷锟斤拷锟秸ｏ拷
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_Buffer, RC_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); 

        // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷龋锟斤拷锟斤拷锟斤拷要锟斤拷
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
  * @brief  锟斤拷锟节达拷锟斤拷氐锟斤拷锟斤拷锟斤拷锟斤拷荽锟斤拷锟?
  * @param  huart: 锟斤拷锟节撅拷锟?
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART10) {
        // 锟斤拷锟斤拷锟斤拷锟斤拷志锟斤拷锟斤拷偶校锟斤拷/帧锟斤拷锟斤拷/锟斤拷锟?/锟斤拷锟斤拷锟斤拷
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_ORE | UART_FLAG_NE);
        // 锟斤拷锟斤拷DMA锟斤拷锟秸ｏ拷锟街革拷通锟脚ｏ拷
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_Buffer, RC_BUFFER_SIZE);
    }
}
/* USER CODE END Application */


