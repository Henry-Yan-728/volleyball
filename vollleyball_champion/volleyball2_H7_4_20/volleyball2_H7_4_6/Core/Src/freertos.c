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
#include <stdlib.h>

// --- 閿熻棄绾ф敮閿熻甯嫹 (BSP) 閿熸枻鎷烽敓鎺ヨ鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓渚ワ綇鎷烽敓? ---
#include "fdcan_bsp.h"      // CAN閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
#include "robot_data.h"     // 鍏ㄩ敓琛椾紮鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸嵎缁撴瀯閿熸枻鎷?
#include "chassis_task.h"   // 閿熸枻鎷烽敓鏁欏尅鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
#include "mechanism_task.h" // 閿熸枻鎷锋閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓浠婏紙浼欐嫹閿熸枻鎷?閿熸枻鎷烽敓鏂ゆ嫹榫嬮敓?
#include "usart.h" 

// --- 閬ラ敓鏂ゆ嫹閿熸枻鎷峰崗閿熶粙澶勯敓鏂ゆ嫹閿熸枻鎷锋寚閿熸枻鎷烽敓鏂ゆ嫹閿? ---
#include "Task_command.h"   // 閿熺粨渚涙寚閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鎴尨鎷烽敓鏂ゆ嫹
#include "planner_tuning_protocol.h"
#include "remote_driver.h"  // 閿熺粨渚涢仴閿熸枻鎷烽敓鏂ゆ嫹鍘熷閿熸枻鎷烽敓鎹风粨鏋勯敓钘夊畾閿熸枻鎷?
#include "queue.h"          // FreeRTOS閿熸枻鎷烽敓鏂ゆ嫹鏀敓鏂ゆ嫹
#include "semphr.h"         // FreeRTOS閿熻剼鐚存嫹閿熸枻鎷?閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷锋敮閿熸枻鎷?
#include "chassis_path_task.h" // 璺敓鏂ゆ嫹閿熻姤鍒掗敓鏂ゆ嫹閿熸枻鎷?
#include "Pan_Tilt_control.h"  // 閿熸枻鎷峰彴閿熸枻鎷烽敓鏂ゆ嫹

// 閿熻В閮ㄧ‖閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿?
#include "chassis_cybergear.h"
#include "cybergear_motor.h"
#include "trajectory_planner.h"
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern UART_HandleTypeDef huart10; 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifndef M_PI
#define M_PI 3.1415926535f  // 鍦嗛敓鏂ゆ嫹閿熺粸璁规嫹閿熸枻鎷?
#endif
#define DEG_TO_RAD (M_PI / 180.0f)
#define TASK03_CAN_PERIOD_MS 20U
#define CYBERGEAR_FEEDBACK_PERIOD_MS 20U
#define MANUAL_ATTITUDE_TARGET_YAW_RAD 0.0f
#define MANUAL_ATTITUDE_POSE_TIMEOUT_MS 1000U
#define MANUAL_ATTITUDE_MOVE_DEADBAND 1.0f
#define PLANNER_TUNING_MAX_SPEED       10000.0f
#define PLANNER_TUNING_MAX_DIST        30000.0f
#define PLANNER_TUNING_MAX_POS_TOL     5000.0f
#define PLANNER_TUNING_MAX_ANGLE_KP    20.0f
#define PLANNER_TUNING_MAX_VR          20.0f
#define PLANNER_TUNING_MAX_VR_SLEW     10.0f
#define PLANNER_TUNING_MAX_SCALE       10.0f
static uint8_t s_cybergear_boot_ok = 0U;
static uint8_t s_cybergear_chassis_ready = 0U;
static float s_manual_attitude_last_vr = 0.0f;
static const PlannerConfig_t s_default_planner_cfg = {
  // ================= 1. 速度包络组 (Speed Envelope) =================
  .max_spd = 800.0f,       // [800] 全局最大平移速度 -> 更快，但更容易冲过头
  .start_spd = 120.0f,     // [120] 起步初始速度 -> 起步更猛，太大会显得突兀
  .stop_spd = 80.0f,       // [80] 末端保底速度 -> 到点更干脆，太大容易震荡
  .up_dist = 300.0f,       // [300] 加速段距离 -> 起步更柔和
  .down_dist = 300.0f,     // [300] 减速段距离 -> 刹车更早、更稳

  // ================= 2. 转向控制组 (Steering Control) =================
  .angle_kp = 1.5f,        // [1.5] 航向比例环增益 -> 转头更凶，太大容易甩头
  .max_vr = 1.2f,          // [1.2] 最大自转角速度 -> 允许更快调头
  .vr_slew_step = 0.08f,   // [0.08] 角速度斜率限制 -> 转向发力更快，太大会更突兀
  .yaw_deadzone = 0.03f,   // [0.03] 航向软死区 -> 更能压住小角度抽动

  // ================= 3. 远近场耦合组 (Far/Near Field Coupling) =================
  .far_near_dist = 500.0f,    // [500] 远近场切换阈值 -> 更早进入赶路模式
  .far_weight_min = 0.40f,    // [0.40] 远场平移权重下限 -> 被撞偏时也更愿意往前走
  .far_max_vr_scale = 0.70f,  // [0.70] 远场最大角速度折减 -> 远距离画龙更少
  .far_vr_slew_scale = 1.50f, // [1.50] 远场角速度斜率放宽 -> 远距离姿态调整更干脆

  // ================= 4. 到点判定组 (Arrival Logic) =================
  .pos_tolerance = 20.0f,  // [20] 到点位置容差 -> 更容易触发“已到达”
  .yaw_tolerance = 0.05f,  // [0.05] 到点角度容差 -> 到点后更少原地微调
  .ignore_yaw = 0U         // [0] 是否忽略最终朝向 -> 1 时只看位置不看姿态
};

// --- 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓闃惰揪鎷?鏋氶敓鍔綇鎷烽敓琛楄鎷锋墽閿熷彨鍑ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷?--


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RC_BUFFER_SIZE  64  // 閬ラ敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熻妭鏂ゆ嫹閿熺Ц浼欐嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷峰皬
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
float PITCH = 5.86f;                // Pitch閿熺粨缂撻敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹?閿熸枻鎷烽敓鍔鎷?
volatile uint32_t g_can3_tx_drop_task03 = 0U;
volatile uint32_t g_can3_tx_drop_task07 = 0U;
volatile uint32_t g_planner_tuning_reject_count = 0U;
volatile uint32_t g_planner_tuning_clamp_count = 0U;

// --- FreeRTOS 閿熻妭鏍歌鎷烽敓鏂ゆ嫹閿熸枻鎷?---
osMutexId_t rc_mutexHandle;            // 閿熸枻鎷烽敓鏂ゆ嫹鍏ㄩ敓鏂ゆ嫹閬ラ敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷?g_remote_data 閿熶茎浼欐嫹閿熸枻鎷烽敓鏂ゆ嫹
osMessageQueueId_t remote_queueHandle; // 閿熸枻鎷锋伅閿熸枻鎷烽敓鍙綇鎷烽敓鏂ゆ嫹閿熻妭鏂ゆ嫹閿熺Ц杈炬嫹閿熸枻鎷烽敓鍙柇纰夋嫹鍘熷閬ラ敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷?

// 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷?
const osMutexAttr_t rc_mutex_attributes = {
  .name = "rcMutex"
};
// 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷疯棔閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓瑙ｅ埌閿熸枻鎷锋椂閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷?
osSemaphoreId_t BinarySem_BallDetectHandle;
const osSemaphoreAttr_t BinarySem_BallDetect_attributes = {
  .name = "BinarySem_BallDetect"
};

// --- 鍏ㄩ敓鏂ゆ嫹涓氶敓鏂ゆ嫹閿熸枻鎷烽敓? ---
remote_engineer_t g_remote_data = {0}; // 鍏ㄩ敓鏂ゆ嫹閬ラ敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸嵎锝忔嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷?
uint8_t remote_Buffer[RC_BUFFER_SIZE]; // 閬ラ敓鏂ゆ嫹閿熸枻鎷稤MA閿熸枻鎷烽敓鏂ゆ嫹鍘熷閿熸枻鎷烽敓鎹蜂紮鎷烽敓鏂ゆ嫹閿熸枻鎷?
uint8_t processsed_command[COMMAND_LENGTH]; // 鎸囬敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸嵎浼欐嫹閿熸枻鎷烽敓鏂ゆ嫹

// 閿熻В閮ㄩ敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閬ラ敓鏂ゆ嫹閿熸枻鎷峰師濮嬮敓鏂ゆ嫹閿熸嵎缁撴瀯閿熸枻鎷?
extern rc_info_t rc; 
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask06 */
osThreadId_t myTask06Handle;
const osThreadAttr_t myTask06_attributes = {
  .name = "myTask06",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask07 */
osThreadId_t myTask07Handle;
const osThreadAttr_t myTask07_attributes = {
  .name = "myTask07",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void freertos_require_handle(const void *handle, const char *name);
static void planner_tuning_send_value(uint8_t cmd_type, uint8_t param_id, float value);
static uint8_t planner_tuning_try_read_value(uint8_t param_id, float *value_out);
static uint8_t planner_tuning_try_write_value(uint8_t param_id, float value);
static uint8_t planner_tuning_clamp_value(uint8_t param_id, float value, float *value_out);
static void planner_tuning_process_frame(const PlannerTuningFrame_t *frame);
static float manual_attitude_clampf(float value, float min_value, float max_value);
static float manual_attitude_normalize_angle(float angle_rad);
static float manual_attitude_apply_soft_deadzone(float value, float deadzone);
static float manual_attitude_update_vr(float now_yaw_rad);
static void manual_attitude_reset(void);

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
  // 1. 纭敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷峰閿熸枻鎷烽敓鏂ゆ嫹閿熼樁璇ф嫹閿熸枻鎷烽敓鍊燂級
  fdcan_bsp_init();     // CAN閿熸枻鎷烽敓绔簳璇ф嫹閿熺粸纭锋嫹閿?
  Robot_Data_Init();    // 鍏ㄩ敓琛椾紮鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸嵎缁撴瀯閿熸枻鎷烽敓缁炵》鎷烽敓?
  Chassis_Init();       // 閿熸枻鎷烽敓鏁欏尅鎷烽敓鐙＄鎷峰閿熸枻鎷?
  Mechanism_Init();     // 閿熸枻鎷锋閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷峰閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹/閿熸枻鎷烽敓鏂ゆ嫹榫嬮敓?

  // 2. 閿熸枻鎷烽敓鏂ゆ嫹CAN閿熸枻鎷烽敓鏂ゆ嫹閫氶敓鏂ゆ嫹
  s_cybergear_boot_ok = 0U;
  s_cybergear_chassis_ready = 0U;
  cybergear_motors_init();
  fdcan_bsp_register_all_dispatches();
  if (trajectory_planner_init() == 0)
  {
    s_cybergear_boot_ok = 1U;
  }
  fdcan_bsp_start(&hfdcan1); 
  fdcan_bsp_start(&hfdcan2); 
  fdcan_bsp_start(&hfdcan3); 

  // 3. 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閬ラ敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸嵎锝忔嫹
  rc_mutexHandle = osMutexNew(&rc_mutex_attributes);
  freertos_require_handle(rc_mutexHandle, "rc_mutexHandle");

  // 4. 閿熸枻鎷烽敓鏂ゆ嫹閬ラ敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹鎭敓鏂ゆ嫹閿熷彨锝忔嫹閿熸枻鎷烽敓鍙鎷烽敓鏂ゆ嫹16閿熸枻鎷锋瘡閿熸枻鎷峰厓閿熸枻鎷蜂负UartRxMessage_t閿熸枻鎷烽敓閰碉綇鎷?
  remote_queueHandle = osMessageQueueNew(16, sizeof(UartRxMessage_t), NULL);
  freertos_require_handle(remote_queueHandle, "remote_queueHandle");
  PlannerTuning_Reset();
	
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿? */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  // 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷疯棔閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓缁炵》鎷?1閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹1閿熸枻鎷?
  BinarySem_BallDetectHandle = osSemaphoreNew(1, 1, &BinarySem_BallDetect_attributes);
  freertos_require_handle(BinarySem_BallDetectHandle, "BinarySem_BallDetectHandle");
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* 閿熸枻鎷锋椂閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鐫綇鎷?*/
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* 閿熸枻鎷烽敓鍙揪鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓? */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  freertos_require_handle(defaultTaskHandle, "defaultTaskHandle");

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);
  freertos_require_handle(myTask02Handle, "myTask02Handle");

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);
  freertos_require_handle(myTask03Handle, "myTask03Handle");

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(StartTask04, NULL, &myTask04_attributes);
  freertos_require_handle(myTask04Handle, "myTask04Handle");

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(StartTask05, NULL, &myTask05_attributes);
  freertos_require_handle(myTask05Handle, "myTask05Handle");
  printf("[RTOS] myTask05 created\r\n");

  /* creation of myTask06 */
  myTask06Handle = osThreadNew(StartTask06, NULL, &myTask06_attributes);
  freertos_require_handle(myTask06Handle, "myTask06Handle");

  /* creation of myTask07 */
  myTask07Handle = osThreadNew(StartTask07, NULL, &myTask07_attributes);
  freertos_require_handle(myTask07Handle, "myTask07Handle");
  printf("[RTOS] all tasks created\r\n");

  /* USER CODE BEGIN RTOS_THREADS */
  /* 閿熸枻鎷烽敓浠婂垱鏂ゆ嫹閿熸枻鎷烽敓? */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* 閿熼摪纭锋嫹閿熸枻鎷峰織閿熶粙锛堥敓鏂ゆ嫹閿熺潾锝忔嫹 */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  榛橀敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹鎷㈤敓?1ms閿熸枻鎷烽敓鑺傦綇鎷?
  * @param  argument: 鏈娇閿熸枻鎷?
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  uint32_t last_feedback_tick = 0U;
  /* USER CODE BEGIN StartDefaultTask */
  /* 閿熸枻鎷烽敓鏂ゆ嫹寰敓鏂ゆ嫹 */
  for(;;)
  {
    if ((s_cybergear_boot_ok == 1U) && (s_cybergear_chassis_ready == 0U))
    {
      if (chassis_cybergear_init(0U) == 0)
      {
        s_cybergear_chassis_ready = 1U;
        last_feedback_tick = HAL_GetTick();
      }
    }
    if ((s_cybergear_chassis_ready == 1U) &&
        ((HAL_GetTick() - last_feedback_tick) >= CYBERGEAR_FEEDBACK_PERIOD_MS))
    {
      chassis_request_angle_feedback();
      last_feedback_tick = HAL_GetTick();
    }
    Update_Virtual_Axis(); // 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸嵎锝忔嫹閬ラ敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷锋槧閿熸垝锛?
    osDelay(1);            // 閿熸枻鎷锋椂1ms閿熸枻鎷风害1000Hz鎵ч敓鏂ゆ嫹棰戦敓缁烇綇鎷?
  }
  /* USER CODE END StartDefaultTask */
}

static void freertos_require_handle(const void *handle, const char *name)
{
  if (handle == NULL) {
    printf("[RTOS] create failed: %s\r\n", name);
    configASSERT(handle != NULL);
  }
}

static void planner_tuning_send_value(uint8_t cmd_type, uint8_t param_id, float value)
{
  uint8_t tx_frame[PLANNER_TUNING_FRAME_LENGTH];

  if (PlannerTuning_BuildFrame(cmd_type, param_id, value, tx_frame) == 0U) {
    return;
  }

  (void)HAL_UART_Transmit(&huart10, tx_frame, sizeof(tx_frame), 20U);
}

static uint8_t planner_tuning_try_read_value(uint8_t param_id, float *value_out)
{
  PlannerConfig_t cfg;

  if (value_out == NULL) {
    return 0U;
  }

  Planner_GetConfig(&cfg);

  switch (param_id) {
    case PID_MAX_SPD:            *value_out = cfg.max_spd; break;
    case PID_START_SPD:          *value_out = cfg.start_spd; break;
    case PID_STOP_SPD:           *value_out = cfg.stop_spd; break;
    case PID_UP_DIST:            *value_out = cfg.up_dist; break;
    case PID_DOWN_DIST:          *value_out = cfg.down_dist; break;
    case PID_ANGLE_KP:           *value_out = cfg.angle_kp; break;
    case PID_MAX_VR:             *value_out = cfg.max_vr; break;
    case PID_VR_SLEW_STEP:       *value_out = cfg.vr_slew_step; break;
    case PID_YAW_DEADZONE:       *value_out = cfg.yaw_deadzone; break;
    case PID_FAR_NEAR_DIST:      *value_out = cfg.far_near_dist; break;
    case PID_FAR_WEIGHT_MIN:     *value_out = cfg.far_weight_min; break;
    case PID_FAR_MAX_VR_SCALE:   *value_out = cfg.far_max_vr_scale; break;
    case PID_FAR_VR_SLEW_SCALE:  *value_out = cfg.far_vr_slew_scale; break;
    case PID_POS_TOLERANCE:      *value_out = cfg.pos_tolerance; break;
    case PID_YAW_TOLERANCE:      *value_out = cfg.yaw_tolerance; break;
    default:
      return 0U;
  }

  return 1U;
}

static uint8_t planner_tuning_clamp_value(uint8_t param_id, float value, float *value_out)
{
  float min_value = 0.0f;
  float max_value = 0.0f;

  if ((value_out == NULL) || (isfinite(value) == 0)) {
    g_planner_tuning_reject_count++;
    return 0U;
  }

  switch (param_id) {
    case PID_MAX_SPD:
    case PID_START_SPD:
    case PID_STOP_SPD:
      min_value = 0.0f;
      max_value = PLANNER_TUNING_MAX_SPEED;
      break;

    case PID_UP_DIST:
    case PID_DOWN_DIST:
    case PID_FAR_NEAR_DIST:
      min_value = 1.0f;
      max_value = PLANNER_TUNING_MAX_DIST;
      break;

    case PID_ANGLE_KP:
      min_value = 0.0f;
      max_value = PLANNER_TUNING_MAX_ANGLE_KP;
      break;

    case PID_MAX_VR:
      min_value = 0.0f;
      max_value = PLANNER_TUNING_MAX_VR;
      break;

    case PID_VR_SLEW_STEP:
      min_value = 0.0f;
      max_value = PLANNER_TUNING_MAX_VR_SLEW;
      break;

    case PID_YAW_DEADZONE:
    case PID_YAW_TOLERANCE:
      min_value = 0.0f;
      max_value = (float)M_PI;
      break;

    case PID_FAR_WEIGHT_MIN:
      min_value = 0.0f;
      max_value = 1.0f;
      break;

    case PID_FAR_MAX_VR_SCALE:
    case PID_FAR_VR_SLEW_SCALE:
      min_value = 0.1f;
      max_value = PLANNER_TUNING_MAX_SCALE;
      break;

    case PID_POS_TOLERANCE:
      min_value = 1.0f;
      max_value = PLANNER_TUNING_MAX_POS_TOL;
      break;

    default:
      g_planner_tuning_reject_count++;
      return 0U;
  }

  if (value < min_value) {
    value = min_value;
    g_planner_tuning_clamp_count++;
  } else if (value > max_value) {
    value = max_value;
    g_planner_tuning_clamp_count++;
  }

  *value_out = value;
  return 1U;
}

static uint8_t planner_tuning_try_write_value(uint8_t param_id, float value)
{
  PlannerConfig_t cfg;
  float safe_value;

  if (planner_tuning_clamp_value(param_id, value, &safe_value) == 0U) {
    return 0U;
  }

  Planner_GetConfig(&cfg);

  switch (param_id) {
    case PID_MAX_SPD:            cfg.max_spd = safe_value; break;
    case PID_START_SPD:          cfg.start_spd = safe_value; break;
    case PID_STOP_SPD:           cfg.stop_spd = safe_value; break;
    case PID_UP_DIST:            cfg.up_dist = safe_value; break;
    case PID_DOWN_DIST:          cfg.down_dist = safe_value; break;
    case PID_ANGLE_KP:           cfg.angle_kp = safe_value; break;
    case PID_MAX_VR:             cfg.max_vr = safe_value; break;
    case PID_VR_SLEW_STEP:       cfg.vr_slew_step = safe_value; break;
    case PID_YAW_DEADZONE:       cfg.yaw_deadzone = safe_value; break;
    case PID_FAR_NEAR_DIST:      cfg.far_near_dist = safe_value; break;
    case PID_FAR_WEIGHT_MIN:     cfg.far_weight_min = safe_value; break;
    case PID_FAR_MAX_VR_SCALE:   cfg.far_max_vr_scale = safe_value; break;
    case PID_FAR_VR_SLEW_SCALE:  cfg.far_vr_slew_scale = safe_value; break;
    case PID_POS_TOLERANCE:      cfg.pos_tolerance = safe_value; break;
    case PID_YAW_TOLERANCE:      cfg.yaw_tolerance = safe_value; break;
    default:
      return 0U;
  }

  Planner_SetConfig(&cfg);
  return 1U;
}

static void planner_tuning_process_frame(const PlannerTuningFrame_t *frame)
{
  float current_value = 0.0f;

  if (frame == NULL) {
    return;
  }

  if (frame->cmd_type == PLANNER_TUNING_CMD_WRITE) {
    if (frame->param_id == SYS_RESET_DEFAULT) {
      Planner_Init(s_default_planner_cfg);
      planner_tuning_send_value(frame->cmd_type, frame->param_id, 1.0f);
      printf("[planner] reset default\r\n");
      return;
    }

    if (frame->param_id == SYS_SAVE_TO_FLASH) {
      planner_tuning_send_value(frame->cmd_type, frame->param_id, 0.0f);
      printf("[planner] save to flash not implemented yet\r\n");
      return;
    }

    if ((planner_tuning_try_write_value(frame->param_id, frame->value) != 0U) &&
        (planner_tuning_try_read_value(frame->param_id, &current_value) != 0U)) {
      planner_tuning_send_value(frame->cmd_type, frame->param_id, current_value);
      printf("[planner] set 0x%02X = %.3f\r\n", frame->param_id, (double)current_value);
      return;
    }
  } else if (frame->cmd_type == PLANNER_TUNING_CMD_READ) {
    if (planner_tuning_try_read_value(frame->param_id, &current_value) != 0U) {
      planner_tuning_send_value(frame->cmd_type, frame->param_id, current_value);
      return;
    }
  }

  planner_tuning_send_value(frame->cmd_type, frame->param_id, -1.0f);
  printf("[planner] unsupported frame cmd=0x%02X id=0x%02X\r\n", frame->cmd_type, frame->param_id);
}

static float manual_attitude_clampf(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static float manual_attitude_normalize_angle(float angle_rad)
{
  while (angle_rad > M_PI)
  {
    angle_rad -= 2.0f * M_PI;
  }
  while (angle_rad < -M_PI)
  {
    angle_rad += 2.0f * M_PI;
  }
  return angle_rad;
}

static float manual_attitude_apply_soft_deadzone(float value, float deadzone)
{
  float abs_value = fabsf(value);

  if (deadzone <= 0.0f)
  {
    return value;
  }
  if (abs_value <= deadzone)
  {
    return 0.0f;
  }
  if (value > 0.0f)
  {
    return value - deadzone;
  }
  return value + deadzone;
}

static float manual_attitude_update_vr(float now_yaw_rad)
{
  PlannerConfig_t cfg;
  float yaw_err;
  float yaw_ctrl_err;
  float target_vr;
  float delta_vr;

  Planner_GetConfig(&cfg);

  yaw_err = manual_attitude_normalize_angle(MANUAL_ATTITUDE_TARGET_YAW_RAD - now_yaw_rad);
  yaw_ctrl_err = manual_attitude_apply_soft_deadzone(yaw_err, cfg.yaw_deadzone);
  target_vr = yaw_ctrl_err * cfg.angle_kp;
  target_vr = manual_attitude_clampf(target_vr, -cfg.max_vr, cfg.max_vr);

  delta_vr = target_vr - s_manual_attitude_last_vr;
  if (cfg.vr_slew_step > 0.0f)
  {
    delta_vr = manual_attitude_clampf(delta_vr, -cfg.vr_slew_step, cfg.vr_slew_step);
  }

  s_manual_attitude_last_vr += delta_vr;
  return s_manual_attitude_last_vr;
}

static void manual_attitude_reset(void)
{
  s_manual_attitude_last_vr = 0.0f;
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief  閿熸枻鎷烽敓渚ュ尅鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓浠婏細纰夋嫹閿熸枻鎷?閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷风柌閿?10ms閿熸枻鎷烽敓鑺傦綇鎷?
* @param  argument: 鏈娇閿熸枻鎷?
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10);
  remote_engineer_t local_rc = {0};
  float target_vx = 0.0f, target_vy = 0.0f, target_vr = 0.0f;

  float PITCH_ANGLE = 0.0f;
  MechanismServeController_t serve_controller;
  chassis_mode_e last_mode = CHASSIS_MODE_STANDBY;
  float serve_target_x = 0.0f;
  float serve_target_y = 0.0f;
  float auto_target_x = 0.0f;
  float auto_target_y = 0.0f;
  float auto_last_target_x = 0.0f;
  float auto_last_target_y = 0.0f;
  float serve_last_target_x = 0.0f;
  float serve_last_target_y = 0.0f;
  uint8_t auto_planner_active = 0U;
  uint8_t serve_planner_active = 0U;

  #define DEBOUNCE_DELAY_MS 10

  static uint32_t last_time_btn5 = 0U;
  static uint32_t last_time_btn2 = 0U;
  static uint32_t last_time_btn4 = 0U;
  static uint32_t last_time_btn1 = 0U;
  static uint32_t last_time_btn3 = 0U;

  static uint8_t last_button5 = 0U;
  static uint8_t last_button2 = 0U;
  static uint8_t last_button4 = 0U;
  static uint8_t last_button1 = 0U;
  static uint8_t last_button3 = 0U;
  static uint8_t last_button_front = 0U;
  static uint8_t last_button_back = 0U;
  static uint8_t last_button_home = 0U;

  Mechanism_ServeController_Init(&serve_controller, HAL_GetTick());
  Planner_Init(s_default_planner_cfg);

  xLastWakeTime = osKernelGetTickCount();

  for (;;)
  {
      uint32_t current_tick = HAL_GetTick();
      PC_state system_state;

      if (osMutexAcquire(rc_mutexHandle, 10) == osOK) {
          local_rc = g_remote_data;
          osMutexRelease(rc_mutexHandle);
      }

      if (local_rc.mode != last_mode)
      {
          auto_planner_active = 0U;
          serve_planner_active = 0U;
          manual_attitude_reset();
          Mechanism_ServeController_Cancel(&serve_controller);
          if (local_rc.mode == CHASSIS_MODE_STANDBY)
          {
              Chassis_Stop();
          }
          last_mode = local_rc.mode;
      }

      if (local_rc.button5 == 1U && last_button5 == 0U) {
          if ((current_tick - last_time_btn5) > DEBOUNCE_DELAY_MS) {
              if ((local_rc.mode != CHASSIS_MODE_SERVE) &&
                  (Get_System_Doit_State() == START) &&
                  Mechanism_ServeController_IsIdle(&serve_controller) &&
                  Mechanism_ServeController_IsReady(&serve_controller)) {
                  Mechanism_ServeController_Request(&serve_controller);
              }
              last_time_btn5 = current_tick;
          }
      }
      last_button5 = local_rc.button5;

      if (local_rc.button1 == 1U && last_button1 == 0U) {
          if ((current_tick - last_time_btn1) > DEBOUNCE_DELAY_MS) {
              if (Get_System_Doit_State() == START) {
                  Set_System_Doit_State(OVER);
              }
              last_time_btn1 = current_tick;
          }
      }
      last_button1 = local_rc.button1;

      if (local_rc.button3 == 1U && last_button3 == 0U) {
          if ((current_tick - last_time_btn3) > DEBOUNCE_DELAY_MS) {
              system_state = Get_System_Doit_State();
              if ((system_state == OVER) ||
                  (system_state == OVER_BUSY) ||
                  (system_state == SYS_ERROR_SENSOR_JAM) ||
                  (system_state == SYS_ERROR_MOTOR_COMMS)) {
                  Set_System_Doit_State(START);
              }
              last_time_btn3 = current_tick;
          }
      }
      last_button3 = local_rc.button3;

      system_state = Get_System_Doit_State();
      if ((system_state == SYS_ERROR_SENSOR_JAM) ||
          (system_state == SYS_ERROR_MOTOR_COMMS))
      {
          target_vx = 0.0f;
          target_vy = 0.0f;
          target_vr = 0.0f;
          auto_planner_active = 0U;
          serve_planner_active = 0U;
          Mechanism_ServeController_Cancel(&serve_controller);
          Chassis_Stop();
          vTaskDelayUntil(&xLastWakeTime, xFrequency);
          continue;
      }

      if (local_rc.mode == CHASSIS_MODE_MANUAL)
      {
          float cur_yaw = 0.0f;
          uint32_t pose_update_tick = 0U;
          uint8_t pose_fresh = 0U;
          uint8_t manual_is_moving = 0U;

          target_vx = local_rc.vx / 25.0f;
          target_vy = local_rc.vy / 25.0f;
          target_vr = -local_rc.vw * 10.0f;
          auto_planner_active = 0U;
          serve_planner_active = 0U;

          taskENTER_CRITICAL();
          cur_yaw = g_robot_pose.angle;
          pose_update_tick = g_robot_pose.update_tick;
          taskEXIT_CRITICAL();

          pose_fresh = (uint8_t)((current_tick - pose_update_tick) <= MANUAL_ATTITUDE_POSE_TIMEOUT_MS);
          manual_is_moving = (uint8_t)((fabsf(target_vx) > MANUAL_ATTITUDE_MOVE_DEADBAND) ||
                                       (fabsf(target_vy) > MANUAL_ATTITUDE_MOVE_DEADBAND));

          if ((manual_is_moving != 0U) &&
              (pose_fresh != 0U))
          {
              target_vr = manual_attitude_update_vr(cur_yaw * DEG_TO_RAD);
          }
          else
          {
              manual_attitude_reset();
          }

          if (local_rc.button2 == 1U && last_button2 == 0U) {
              if ((current_tick - last_time_btn2) > DEBOUNCE_DELAY_MS) {
                  PITCH_ANGLE += 5.0f;
                  Mechanism_Dian_Pitch_SetAngle(PITCH_ANGLE);
                  last_time_btn2 = current_tick;
              }
          }
          last_button2 = local_rc.button2;

          if (local_rc.button4 == 1U && last_button4 == 0U) {
              if ((current_tick - last_time_btn4) > DEBOUNCE_DELAY_MS) {
                  PITCH_ANGLE -= 5.0f;
                  Mechanism_Dian_Pitch_SetAngle(PITCH_ANGLE);
                  last_time_btn4 = current_tick;
              }
          }
          last_button4 = local_rc.button4;

          if (Mechanism_ServeController_IsIdle(&serve_controller)) {
              Chassis_Update(target_vx, target_vy, target_vr);
          }
      }
      else if (local_rc.mode == CHASSIS_MODE_AUTO)
      {
          float cur_x;
          float cur_y;
          float cur_yaw;
          float cur_vx;
          float cur_vy;
          uint32_t last_update;

          taskENTER_CRITICAL();
          cur_x = g_robot_pose.x;
          cur_y = g_robot_pose.y;
          cur_yaw = g_robot_pose.angle;
          cur_vx = g_robot_pose.vx;
          cur_vy = g_robot_pose.vy;
          last_update = g_robot_pose.update_tick;
          auto_target_x = g_robot_target.target_x;
          auto_target_y = g_robot_target.target_y;
          taskEXIT_CRITICAL();
          cur_yaw *= DEG_TO_RAD;

          if (HAL_GetTick() - last_update > 1000U) {
              target_vx = 0.0f;
              target_vy = 0.0f;
              target_vr = 0.0f;
              auto_planner_active = 0U;
          }
          else
          {
              if ((auto_planner_active == 0U) ||
                  (fabsf(auto_target_x - auto_last_target_x) > 1.0f) ||
                  (fabsf(auto_target_y - auto_last_target_y) > 1.0f)) {
                  Planner_SetTarget(cur_x, cur_y, auto_target_x, auto_target_y, 0.0f);
                  auto_last_target_x = auto_target_x;
                  auto_last_target_y = auto_target_y;
                  auto_planner_active = 1U;
              }

              TrajVel_t cmd = Planner_Update(cur_x, cur_y, cur_yaw, cur_vx, cur_vy);
              float theta = cur_yaw;
              target_vx = cmd.vx * cosf(theta) + cmd.vy * sinf(theta);
              target_vy = -cmd.vx * sinf(theta) + cmd.vy * cosf(theta);
              target_vr = cmd.vr;

              if (Planner_IsArrived(cur_x, cur_y, cur_yaw)) {
                  auto_planner_active = 0U;
                  target_vx = 0.0f;
                  target_vy = 0.0f;
                  target_vr = 0.0f;
              }
          }

          serve_planner_active = 0U;
          Chassis_Update(-target_vy, target_vx, target_vr);
      }
      else if (local_rc.mode == CHASSIS_MODE_SERVE)
      {
          float cur_x;
          float cur_y;
          float cur_yaw;
          float cur_vx;
          float cur_vy;
          uint32_t last_update;

          taskENTER_CRITICAL();
          cur_x = g_robot_pose.x;
          cur_y = g_robot_pose.y;
          cur_yaw = g_robot_pose.angle;
          cur_vx = g_robot_pose.vx;
          cur_vy = g_robot_pose.vy;
          last_update = g_robot_pose.update_tick;
          taskEXIT_CRITICAL();
          cur_yaw *= DEG_TO_RAD;

          if (local_rc.button5 == 1U && last_button_home == 0U) {
              serve_target_x = 0.0f;
              serve_target_y = 0.0f;
              serve_planner_active = 0U;
          }
          last_button_home = local_rc.button5;

          if (local_rc.button2 == 1U && last_button_front == 0U) {
              serve_target_x = 800.0f;
              serve_target_y = 500.0f;
              serve_planner_active = 0U;
          } else if (local_rc.button4 == 1U && last_button_back == 0U) {
              serve_target_x = 1000.0f;
              serve_target_y = 1000.0f;
              serve_planner_active = 0U;
          }
          last_button_front = local_rc.button2;
          last_button_back = local_rc.button4;

          if (HAL_GetTick() - last_update > 1000U) {
              target_vx = 0.0f;
              target_vy = 0.0f;
              target_vr = 0.0f;
              serve_planner_active = 0U;
          }
          else
          {
              if ((serve_planner_active == 0U) ||
                  (fabsf(serve_target_x - serve_last_target_x) > 1.0f) ||
                  (fabsf(serve_target_y - serve_last_target_y) > 1.0f)) {
                  Planner_SetTarget(cur_x, cur_y, serve_target_x, serve_target_y, 0.0f);
                  serve_last_target_x = serve_target_x;
                  serve_last_target_y = serve_target_y;
                  serve_planner_active = 1U;
              }

              TrajVel_t cmd = Planner_Update(cur_x, cur_y, cur_yaw, cur_vx, cur_vy);
              float theta = cur_yaw;
              target_vx = cmd.vx * cosf(theta) + cmd.vy * sinf(theta);
              target_vy = -cmd.vx * sinf(theta) + cmd.vy * cosf(theta);
              target_vr = cmd.vr;

              if (Planner_IsArrived(cur_x, cur_y, cur_yaw)) {
                  serve_planner_active = 0U;
                  target_vx = 0.0f;
                  target_vy = 0.0f;
                  target_vr = 0.0f;
                  if ((Get_System_Doit_State() == START) &&
                      Mechanism_ServeController_IsIdle(&serve_controller) &&
                      Mechanism_ServeController_IsReady(&serve_controller)) {
                      Mechanism_ServeController_Request(&serve_controller);
                  }
              }
          }

          auto_planner_active = 0U;
          Chassis_Update(target_vx, target_vy, target_vr);
      }
      else
      {
          target_vx = 0.0f;
          target_vy = 0.0f;
          target_vr = 0.0f;
          auto_planner_active = 0U;
          serve_planner_active = 0U;
          Chassis_Stop();
      }

      Mechanism_ServeController_Process(&serve_controller, current_tick, PITCH);
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief  CAN2閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓浠婏細浼欐嫹閿熸枻鎷烽敓鏂ゆ嫹浣嶉敓鏂ゆ嫹閿熻緝鎲嬫嫹閿熸枻鎷烽敓鏂ゆ嫹棰戦敓鏂ゆ嫹
* @param  argument: 鏈娇閿熸枻鎷?
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
    /* USER CODE BEGIN StartTask03 */
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(TASK03_CAN_PERIOD_MS);

    TxHeader.Identifier = 0x400;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    for(;;)
    {
        uint8_t auto_state = 0;
        PC_state current_state = Get_System_Doit_State();
        if ((current_state == OVER) || (current_state == OVER_BUSY))
        {
            auto_state = 0;
        }else if ((current_state == START) || (current_state == START_BUSY))
        {
            auto_state = 1;
        }
        TxData[0] = auto_state;
        memset(&TxData[1], 0, sizeof(TxData) - 1);

        if (fdcan_bsp_send(&hfdcan3, &TxHeader, TxData) != 0U)
        {
            g_can3_tx_drop_task03++;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    /* USER CODE END StartTask03 */
}
/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief  閬ラ敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸嵎鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熶粖锛氳揪鎷烽敓鏂ゆ嫹閿熸枻鎷稤MA閿熸枻鎷烽敓绉哥鎷峰師濮嬮敓鏂ゆ嫹閿熸枻鎷?
* @param  argument: 鏈娇閿熸枻鎷?
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart10, remote_Buffer, RC_BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);

  UartRxMessage_t rx_msg;
  PlannerTuningFrame_t tuning_frame;
  remote_engineer_t temp_rc = {0};

  for (;;)
  {
      if (osMessageQueueGet(remote_queueHandle, &rx_msg, NULL, osWaitForever) == osOK)
      {
          if (PlannerTuning_Write(rx_msg.data, (uint8_t)rx_msg.size) == 0U)
          {
              PlannerTuning_Reset();
              (void)PlannerTuning_Write(rx_msg.data, (uint8_t)rx_msg.size);
          }

          while (PlannerTuning_GetFrame(&tuning_frame) != 0U)
          {
              planner_tuning_process_frame(&tuning_frame);
          }

          if (Command_Write(rx_msg.data, (uint8_t)rx_msg.size) == 0U)
          {
              Command_Reset();
              (void)Command_Write(rx_msg.data, (uint8_t)rx_msg.size);
          }

          while (Command_GetCommand(processsed_command) != 0U)
          {
              code_unzipread(processsed_command);
              Remote_Data_Convert(&rc, &temp_rc);

              if (osMutexAcquire(rc_mutexHandle, 10) == osOK)
              {
                  g_remote_data = temp_rc;
                  remote_engineer = temp_rc;
                  osMutexRelease(rc_mutexHandle);
              }
          }
      }
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief  閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓浠婏細纭锋嫹鐛炬枻鎷烽敓瑗熻Е鍑ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷?
* @param  argument: 鏈娇閿熸枻鎷?
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  #define CUSHION_READY_DEG   54.0f
  #define CUSHION_SPEED       2.66f
  #define BALL_DETECT_TASK_PERIOD_MS  2U

  printf("[RTOS] StartTask05 entered\r\n");
  osDelay(1000);
  Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, CUSHION_SPEED);
  osDelay(500);
  Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, CUSHION_SPEED);
  printf("System Ready. Angle reset.\r\n");

  for(;;)
  {
    Mechanism_BallDetect_Process(PITCH);
    osDelay(BALL_DETECT_TASK_PERIOD_MS);
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief  閿熸枻鎷锋閿熸枻鎷烽敓鏂ゆ嫹寰敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹1ms閿熸枻鎷烽敓鑺傞潻鎷烽敓閾颁紮鎷烽敓鏂ゆ嫹鐘舵€?
* @param  argument: 鏈娇閿熸枻鎷?
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(1);

  xLastWakeTime = xTaskGetTickCount();
  for(;;)
  {
    Mechanism_Loop_1ms();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief  CAN3閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷峰彴閿熻璁规嫹閿熻緝鎲嬫嫹閿熸枻鎷?ms閿熸枻鎷烽敓鑺傦綇鎷?
* @param  argument: 鏈娇閿熸枻鎷?
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
  FDCAN_TxHeaderTypeDef TxHeader; // CAN閿熸枻鎷烽敓鏂ゆ嫹甯уご
  uint8_t TxData[8];             // 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鎹蜂紮鎷烽敓鏂ゆ嫹閿熸枻鎷?
  
  // 閿熸枻鎷峰閿熸枻鎷稢AN閿熸枻鎷烽敓鏂ゆ嫹甯уご
  TxHeader.Identifier = CAN_ID_PC_FEEDBACK; // 甯D閿熸枻鎷?x300閿熸枻鎷稰C閿熸枻鎷烽敓鏂ゆ嫹涓撻敓鐭綇鎷?
  TxHeader.IdType = FDCAN_STANDARD_ID;      // 閿熸枻鎷峰噯ID
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;  // 閿熸枻鎷烽敓鏂ゆ嫹甯?
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;  // 閿熸枻鎷烽敓鎹风鎷烽敓楗猴綇鎷?閿熻鏂ゆ嫹
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;   // 閿熸埅闂鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鍙紮鎷烽敓鏂ゆ嫹閿熸枻鎷风粺CAN閿熸枻鎷?
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;    // 閿熸枻鎷风粺CAN閿熸枻鎷峰紡
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
  
  // 閿熸枻鎷峰噯閿熸枻鎷锋椂閿熸枻鎷峰閿熸枻鎷烽敓鏂ゆ嫹1ms閿熸枻鎷烽敓鑺傦綇鎷?
  TickType_t xLastWakeTime = osKernelGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms鎵ч敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹

  /* 閿熸枻鎷烽敓鏂ゆ嫹寰敓鏂ゆ嫹 */
  for(;;)
  {
      uint32_t current_timestamp = HAL_GetTick(); // 閿熸枻鎷峰墠鏃堕敓鏂ゆ嫹閿熸枻鎷烽敓绲閿熸枻鎷?
      float yaw_angle = 0.0f;                     // 閿熸枻鎷峰彴Yaw閿熸枻鎷峰祵閿?
      float pitch_angle = 0.0f;                   // 閿熸枻鎷峰彴Pitch閿熸枻鎷峰祵閿?
      
      // 閿熸枻鎷峰彇閿熸枻鎷峰彴瀹炴椂閿熻璁规嫹
      gimbal_get_angles(&yaw_angle, &pitch_angle);

      // 閿熸枻鎷烽敓鏂ゆ嫹鍘嬮敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹*100杞负int16_t閿熸枻鎷烽敓鏂ゆ嫹鐪侀敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷?浣嶅皬閿熸枻鎷烽敓鏂ゆ嫹
      int16_t yaw_send   = (int16_t)(yaw_angle * 100.0f);
      int16_t pitch_send = (int16_t)(pitch_angle * 100.0f);

      // 閿熸枻鎷烽敓鎹疯揪鎷烽敓?
      memcpy(&TxData[0], &current_timestamp, 4); // 0-3閿熻鑺傦綇鎷锋椂閿熸枻鎷烽敓鏂ゆ嫹閿熺惮int32_t閿熸枻鎷?
      memcpy(&TxData[4], &yaw_send, 2);          // 4-5閿熻鑺傦綇鎷穀aw閿熻搴︼綇鎷穒nt16_t閿熸枻鎷?
      memcpy(&TxData[6], &pitch_send, 2);        // 6-7閿熻鑺傦綇鎷稰itch閿熻搴︼綇鎷穒nt16_t閿熸枻鎷?

      // 閿熸枻鎷烽敓閰电鎷稢AN3閿熸枻鎷烽敓鏂ゆ嫹
      if (fdcan_bsp_send(&hfdcan3, &TxHeader, TxData) != 0U)
      {
          g_can3_tx_drop_task07++;
      }

      // 閿熸枻鎷峰噯閿熸枻鎷锋椂閿熸枻鎷烽敓鏂ゆ嫹璇?ms閿熸枻鎷烽敓鑺傦綇鎷?
      vTaskDelayUntil(&xLastWakeTime, xFrequency);  
  }
  /* USER CODE END StartTask07 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* -------------------------------------------------------------------------
// 閿熸枻鎷烽敓鏂ゆ嫹DMA閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹鍗稿專姘愰敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸彮锝忔嫹閿熸枻鎷烽敓鏂ゆ嫹?濮嬮敓鏂ゆ嫹閿熸嵎锝忔嫹
// ------------------------------------------------------------------------- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    
    if (huart->Instance == USART10) {
        UartRxMessage_t rx_msg;
        uint16_t copy_size = (Size < sizeof(rx_msg.data)) ? Size : sizeof(rx_msg.data);

        if (copy_size > 0U)
        {
            memcpy(rx_msg.data, remote_Buffer, copy_size);
            rx_msg.size = copy_size;

            if (remote_queueHandle != NULL)
            {
                (void)osMessageQueuePut(remote_queueHandle, &rx_msg, 0U, 0U);
            }
        }

        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_Buffer, RC_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

/**
  * @brief  閿熸枻鎷烽敓鑺傝揪鎷烽敓鏂ゆ嫹姘愰敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷疯嵔閿熸枻鎷烽敓?
  * @param  huart: 閿熸枻鎷烽敓鑺傛拝鎷烽敓?
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART10) {
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_ORE | UART_FLAG_NE);
        Command_Reset();
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_Buffer, RC_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}
/* USER CODE END Application */


