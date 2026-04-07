/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : FreeRTOS 应用层代码，包含任务定义、初始化和业务逻辑
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

// --- 板级支持包 (BSP) 与硬件驱动模块 ---
#include "fdcan_bsp.h"      // CAN 总线驱动
#include "robot_data.h"     // 全局机器人数据结构体
#include "chassis_task.h"   // 底盘控制任务
#include "mechanism_task.h" // 执行机构任务（摩擦轮/拨叉）
#include "usart.h" 

// --- 遥控器协议与指令解析 ---
#include "Task_command.h"   // 提供指令处理函数
#include "remote_driver.h"  // 提供遥控器原始数据结构定义
#include "queue.h"          // FreeRTOS 队列支持
#include "semphr.h"         // FreeRTOS 信号量/互斥量支持
#include "chassis_path_task.h" // 路径规划任务
#include "Pan_Tilt_control.h"  // 云台控制

// 外部硬件句柄声明
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern UART_HandleTypeDef huart10; 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifndef M_PI
#define M_PI 3.1415926535f  // 圆周率常量
#endif

// 发球力度增加步长
#define power_add 0.3
// 发球延迟时间步长 (ms)
#define time_add 1

// --- 执行机构状态枚举，分步执行发球动作 ---
typedef enum {
    ACTION_IDLE = 0, // 空闲状态，无动作
    ACTION_STEP_1,   // 执行阶段1：摩擦轮加速与拨叉抬起
    ACTION_STEP_2    // 执行阶段2：拨叉复位与摩擦轮减速
} ActionState_e;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RC_BUFFER_SIZE  64  // 遥控器 DMA 接收缓冲区大小
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
float time_ms = 1;                  // 发球延迟时间变量 (ms)
float PITCH = 4.06f;                // Pitch 摩擦轮初始速度

// --- FreeRTOS 内核对象定义 ---
osMutexId_t rc_mutexHandle;            // 保护全局遥控器数据 g_remote_data 的互斥量
osMessageQueueId_t remote_queueHandle; // 消息队列，用于接收串口中断的原始遥控器数据

// 互斥量属性配置
const osMutexAttr_t rc_mutex_attributes = {
  .name = "rcMutex"
};

// 检测到球的二进制信号量（目前代码中未实际使用）
osSemaphoreId_t BinarySem_BallDetectHandle;
const osSemaphoreAttr_t BinarySem_BallDetect_attributes = {
  .name = "BinarySem_BallDetect"
};

// --- 全局业务变量 ---
remote_engineer_t g_remote_data = {0}; // 全局遥控器数据，线程安全
uint8_t remote_Buffer[RC_BUFFER_SIZE]; // 遥控器 DMA 原始数据缓冲区
uint8_t processsed_command[COMMAND_LENGTH]; // 指令解析后数据缓冲区

// 外部声明的遥控器原始数据结构
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
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 1024 * 4,
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
static void RTOS_CheckThreadCreated(const char *name, osThreadId_t handle);


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
  * @brief  FreeRTOS 初始化
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  // 1. 硬件初始化（底层驱动）
  fdcan_bsp_init();     // CAN 总线底层初始化
  Robot_Data_Init();    // 全局机器人数据结构体初始化
  Chassis_Init();       // 底盘控制初始化
  Mechanism_Init();     // 执行机构初始化（摩擦轮/拨叉）

  HAL_Delay(100); // 硬件稳定延时

  // 2. 启动 CAN 总线通信
  fdcan_bsp_start(&hfdcan1); 
  fdcan_bsp_start(&hfdcan2); 
  fdcan_bsp_start(&hfdcan3); 

  // 3. 创建互斥量，用于保护遥控器数据
  rc_mutexHandle = osMutexNew(&rc_mutex_attributes);

  // 4. 创建遥控器数据消息队列，容量16，每个元素为 UartRxMessage_t 类型
  remote_queueHandle = osMessageQueueNew(16, sizeof(UartRxMessage_t), NULL);
	
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* 其他互斥量初始化（此处无） */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  // 创建检测球的二进制信号量，初始值1（目前代码逻辑中未使用）
  BinarySem_BallDetectHandle = osSemaphoreNew(1, 1, &BinarySem_BallDetect_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* 软件定时器初始化（此处无） */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* 其他队列初始化（此处无） */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  RTOS_CheckThreadCreated("defaultTask", defaultTaskHandle);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);
  RTOS_CheckThreadCreated("myTask02", myTask02Handle);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);
  RTOS_CheckThreadCreated("myTask03", myTask03Handle);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(StartTask04, NULL, &myTask04_attributes);
  RTOS_CheckThreadCreated("myTask04", myTask04Handle);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(StartTask05, NULL, &myTask05_attributes);
  RTOS_CheckThreadCreated("myTask05", myTask05Handle);

  /* creation of myTask06 */
  myTask06Handle = osThreadNew(StartTask06, NULL, &myTask06_attributes);
  RTOS_CheckThreadCreated("myTask06", myTask06Handle);

  /* creation of myTask07 */
  myTask07Handle = osThreadNew(StartTask07, NULL, &myTask07_attributes);
  RTOS_CheckThreadCreated("myTask07", myTask07Handle);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 线程创建后代码（此处无） */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* 事件标志组（此处无） */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  默认任务，高频率：1ms 周期
  * @param  argument: 未使用
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* 无限循环 */
  for(;;)
  {
    Update_Virtual_Axis(); // 更新虚拟轴数据（遥控器映射）
    osDelay(1);            // 延时1ms，约1000Hz执行频率
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief  核心业务任务：底盘+机构控制，10ms 周期
* @param  argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10);
  xLastWakeTime = osKernelGetTickCount();

  remote_engineer_t local_rc = {0};
  float target_vx = 0, target_vy = 0, target_vr = 0;

  const float CUSHION_READY_DEG = 47.0f;
  const float CUSHION_ACTION_DEG = 114.0f;
  const float SERVE_READY_RAD    = 30.0f;
  const float SERVE_ACTION_RAD   = 210.0f;
  float PITCH_ANGLE = 0.0f;

  ActionState_e serve_state = ACTION_IDLE;
  uint32_t serve_start_tick = 0;

  #define DEBOUNCE_DELAY_MS 10
  static uint32_t last_time_btn5 = 0;
  static uint32_t last_time_btn2 = 0;
  static uint32_t last_time_btn4 = 0;
  static uint32_t last_time_btn1 = 0;
  static uint32_t last_time_btn3 = 0;

  static uint8_t last_button5 = 0;
  static uint8_t last_button2 = 0;
  static uint8_t last_button4 = 0;
  static uint8_t last_button1 = 0;
  static uint8_t last_button3 = 0;

  static uint8_t last_button_front = 0;
  static uint8_t last_button_back = 0;

  static chassis_mode_e last_mode = CHASSIS_MODE_STANDBY;
  static uint8_t auto_planner_initialized = 0;
  static uint8_t serve_planner_initialized = 0;
  static uint8_t serve_arrived_latched = 0;
  static float serve_target_x = 0.0f;
  static float serve_target_y = 0.0f;
  static uint32_t last_debug_tick = 0;

  printf("[task2] started\r\n");

  for(;;)
  {
      uint32_t current_tick = HAL_GetTick();

      // 获取互斥量，读取最新的遥控器数据
      if (osMutexAcquire(rc_mutexHandle, 10) == osOK) {
          local_rc = g_remote_data;
          osMutexRelease(rc_mutexHandle);
      }

      // 每200ms打印一次调试信息
      if ((current_tick - last_debug_tick) >= 200U) {
          long vx_milli = (long)(local_rc.vx * 1000.0f);
          printf("vx_milli=%ld\r\n", vx_milli);
          last_debug_tick = current_tick;
      }

      // 手动模式：遥控器直接控制底盘与机构
      if (local_rc.mode == CHASSIS_MODE_MANUAL)
      {
          // 遥控器摇杆映射到底盘速度
          target_vx = local_rc.vx / 100.0f;
          target_vy = local_rc.vy / 100.0f;
          target_vr = -local_rc.vw * 3.0f;

          // 按键5：触发发球动作状态机
          if (local_rc.button5 == 1 && last_button5 == 0) {
              if ((current_tick - last_time_btn5) > DEBOUNCE_DELAY_MS) {
                  if (serve_state == ACTION_IDLE) {
                      serve_state = ACTION_STEP_1;
                      serve_start_tick = current_tick;
                  }
                  last_time_btn5 = current_tick;
              }
          }
          last_button5 = local_rc.button5;

          // 按键2：Pitch角度增加5度
          if (local_rc.button2 == 1 && last_button2 == 0) {
              if ((current_tick - last_time_btn2) > DEBOUNCE_DELAY_MS) {
                  PITCH_ANGLE += 5.0f;
                  Mechanism_Dian_Pitch_SetAngle(PITCH_ANGLE);
                  last_time_btn2 = current_tick;
              }
          }
          last_button2 = local_rc.button2;

          // 按键4：Pitch角度减少5度
          if (local_rc.button4 == 1 && last_button4 == 0) {
              if ((current_tick - last_time_btn4) > DEBOUNCE_DELAY_MS) {
                  PITCH_ANGLE -= 5.0f;
                  Mechanism_Dian_Pitch_SetAngle(PITCH_ANGLE);
                  last_time_btn4 = current_tick;
              }
          }
          last_button4 = local_rc.button4;

          // 按键1：发球力度增加
          if (local_rc.button1 == 1 && last_button1 == 0) {
              if ((current_tick - last_time_btn1) > DEBOUNCE_DELAY_MS) {
                  PITCH += power_add;
                  last_time_btn1 = current_tick;
              }
          }
          last_button1 = local_rc.button1;

          // 按键3：发球力度减小
          if (local_rc.button3 == 1 && last_button3 == 0) {
              if ((current_tick - last_time_btn3) > DEBOUNCE_DELAY_MS) {
                  PITCH -= power_add;
                  last_time_btn3 = current_tick;
              }
          }
          last_button3 = local_rc.button3;

          // 更新底盘控制输出
          Chassis_Update(target_vx, target_vy, target_vr);
      }
      // 自动模式：路径规划自动导航
      else if (local_rc.mode == CHASSIS_MODE_AUTO)
      {
          float cur_x, cur_y, cur_yaw;
          float target_x, target_y;
          uint8_t target_updated;
          uint32_t last_update;

          // 进入临界区读取全局位姿与目标点
          taskENTER_CRITICAL();
          cur_x = g_robot_pose.x;
          cur_y = g_robot_pose.y;
          cur_yaw = g_robot_pose.angle;
          last_update = g_robot_pose.update_tick;
          target_x = g_robot_target.target_x;
          target_y = g_robot_target.target_y;
          target_updated = g_robot_target.is_updated;
          g_robot_target.is_updated = 0;
          taskEXIT_CRITICAL();

          // 如果模式切换、首次初始化或目标点更新，则重新设置规划器目标
          if ((local_rc.mode != last_mode) || !auto_planner_initialized || target_updated) {
              Planner_SetTarget(cur_x, cur_y, target_x, target_y, 0.0f);
              auto_planner_initialized = 1;
          }

          // 如果位姿长时间未更新（超过1秒），停止运动
          if (HAL_GetTick() - last_update > 1000U) {
              target_vx = 0.0f;
              target_vy = 0.0f;
              target_vr = 0.0f;
          }
          else
          {
              // 调用规划器计算速度指令
              TrajVel_t cmd = Planner_Update(cur_x, cur_y, cur_yaw);
              // 将世界坐标系速度转换为机器人坐标系
              float theta = cur_yaw * (M_PI / 180.0f);
              target_vx =  cmd.vx * cosf(theta) + cmd.vy * sinf(theta);
              target_vy = -cmd.vx * sinf(theta) + cmd.vy * cosf(theta);
              target_vr = cmd.vr;

              // 如果到达目标点，清除初始化标志
              if (Planner_IsArrived(cur_x, cur_y, cur_yaw)) {
                  auto_planner_initialized = 0;
              }
          }

          // 更新底盘控制输出
          Chassis_Update(target_vx, target_vy, target_vr);
      }
      // 发球模式：自动导航到指定点并自动发球
      else if (local_rc.mode == CHASSIS_MODE_SERVE)
      {
          float cur_x, cur_y, cur_yaw;
          uint32_t last_update;

          // 进入临界区读取全局位姿
          taskENTER_CRITICAL();
          cur_x = g_robot_pose.x;
          cur_y = g_robot_pose.y;
          cur_yaw = g_robot_pose.angle;
          last_update = g_robot_pose.update_tick;
          taskEXIT_CRITICAL();

          // 按键5：设置目标点为原点 (0, 0)
          if (local_rc.button5 == 1 && last_button5 == 0) {
              serve_target_x = 0.0f;
              serve_target_y = 0.0f;
              serve_planner_initialized = 0;
              serve_arrived_latched = 0;
          }
          last_button5 = local_rc.button5;

          // 按键2：设置目标点为 (800, 500)
          if (local_rc.button2 == 1 && last_button_front == 0) {
              serve_target_x = 800.0f;
              serve_target_y = 500.0f;
              serve_planner_initialized = 0;
              serve_arrived_latched = 0;
          } 
          // 按键4：设置目标点为 (1000, 1000)
          else if (local_rc.button4 == 1 && last_button_back == 0) {
              serve_target_x = 1000.0f;
              serve_target_y = 1000.0f;
              serve_planner_initialized = 0;
              serve_arrived_latched = 0;
          }
          last_button_front = local_rc.button2;
          last_button_back = local_rc.button4;

          // 如果模式切换或首次初始化，设置规划器目标
          if ((local_rc.mode != last_mode) || !serve_planner_initialized) {
              Planner_SetTarget(cur_x, cur_y, serve_target_x, serve_target_y, 0.0f);
              serve_planner_initialized = 1;
          }

          // 如果位姿长时间未更新，停止运动
          if (HAL_GetTick() - last_update > 1000U) {
              target_vx = 0.0f;
              target_vy = 0.0f;
              target_vr = 0.0f;
          }
          else
          {
              // 调用规划器计算速度
              TrajVel_t cmd = Planner_Update(cur_x, cur_y, cur_yaw);
              // 坐标系转换
              float theta = cur_yaw * (M_PI / 180.0f);
              target_vx =  cmd.vx * cosf(theta) + cmd.vy * sinf(theta);
              target_vy = -cmd.vx * sinf(theta) + cmd.vy * cosf(theta);
              target_vr = cmd.vr;

              // 如果到达目标点，触发自动发球
              if (Planner_IsArrived(cur_x, cur_y, cur_yaw)) {
                  if (!serve_arrived_latched && serve_state == ACTION_IDLE) {
                      serve_state = ACTION_STEP_1;
                      serve_start_tick = current_tick;
                      serve_arrived_latched = 1;
                  }
              } else {
                  serve_arrived_latched = 0;
              }
          }

          // 更新底盘控制
          Chassis_Update(target_vx, target_vy, target_vr);
      }
      // 其他模式（待机）：停止运动
      else
      {
          target_vx = 0.0f;
          target_vy = 0.0f;
          target_vr = 0.0f;
          Chassis_Update(target_vx, target_vy, target_vr);
          auto_planner_initialized = 0;
          serve_planner_initialized = 0;
          serve_arrived_latched = 0;
      }

      last_mode = local_rc.mode;

      // 发球动作状态机处理
      switch (serve_state) {
          case ACTION_STEP_1:
              // 阶段1：拨叉到发球角度，摩擦轮加速
              Mechanism_Cushion_SetAngle(CUSHION_ACTION_DEG, PITCH);
              Mechanism_Serve_SetAngle(SERVE_ACTION_RAD);
              if (current_tick - serve_start_tick > 300U) {
                  serve_state = ACTION_STEP_2;
              }
              break;

          case ACTION_STEP_2:
              // 阶段2：拨叉复位，摩擦轮减速，一段时间后发球轮复位
              Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, 0.3f);
              Mechanism_Serve_SetAngle(SERVE_ACTION_RAD);
              if (current_tick - serve_start_tick > 800U) {
                  Mechanism_Serve_SetAngle(SERVE_READY_RAD);
                  serve_state = ACTION_IDLE;
              }
              break;

          default:
              break;
      }

      // 精确延时，保证10ms周期
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief  CAN2 发送任务：机器人位姿上报，高频率
* @param  argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  FDCAN_TxHeaderTypeDef TxHeader; // CAN 发送帧头
  uint8_t TxData[12];             // CAN 发送数据，12字节
  
  // 初始化 CAN 发送帧头
  TxHeader.Identifier = 0x101;                // 帧ID：0x101
  TxHeader.IdType = FDCAN_STANDARD_ID;        // 标准ID（11位）
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;    // 数据帧
  TxHeader.DataLength = FDCAN_DLC_BYTES_12;   // 数据长度：12字节
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // 错误状态指示
  TxHeader.BitRateSwitch = FDCAN_BRS_ON;      // 波特率切换开启
  TxHeader.FDFormat = FDCAN_FD_CAN;           // FD CAN 格式
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 不保存发送事件
  TxHeader.MessageMarker = 0;                 // 消息标记：0

  /* 无限循环 */
  for(;;)
  {
      float temp_x, temp_y, temp_angle;
      
      // 进入临界区读取位姿，防止多线程冲突
      taskENTER_CRITICAL();
      temp_x = g_robot_pose.x;
      temp_y = g_robot_pose.y;
      temp_angle = g_robot_pose.angle;
      taskEXIT_CRITICAL();

      // 数据打包：Float 转 Byte
      memcpy(&TxData[0], &temp_x, 4);     // 0-3字节：X坐标 (float)
      memcpy(&TxData[4], &temp_y, 4);     // 4-7字节：Y坐标 (float)
      memcpy(&TxData[8], &temp_angle, 4); // 8-11字节：偏航角 (float)

      // 发送到 CAN2 总线
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
      osDelay(1); // 约1000Hz发送频率
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief  遥控器数据解析任务：处理从 DMA 接收到的原始数据
* @param  argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  // 初始化 UART10 DMA 循环接收，ReceiveToIdle 模式，等待空闲中断
  HAL_UARTEx_ReceiveToIdle_DMA(&huart10, remote_Buffer, RC_BUFFER_SIZE);
  // 禁用 DMA 半满中断，只需要一帧数据结束后的中断
  __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT); 

  UartRxMessage_t rx_msg;        // 用于接收消息结构体
  remote_engineer_t temp_rc;     // 临时遥控器数据

  // 检查队列是否创建成功
  if (remote_queueHandle == NULL)
  {
      printf("[RTOS] remote_queue create failed\r\n");
      taskDISABLE_INTERRUPTS();
      for (;;)
      {
      }
  }

  /* 无限循环 */
  for(;;)
  {
      // 阻塞等待消息队列，数据来自串口中断
      if (osMessageQueueGet(remote_queueHandle, &rx_msg, NULL, osWaitForever) == osOK)
      {
          // 将原始数据写入指令解析器
          Command_Write(rx_msg.data, rx_msg.size);
          
          // 循环解析出所有有效指令
          while (Command_GetCommand(processsed_command) != 0)
          {
              code_unzipread(processsed_command);   // 解包遥控器按键
              Remote_Data_Convert(&rc, &temp_rc);   // 转换为标准数据格式
              
              // 线程安全地更新全局遥控器数据
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
* @brief  发球任务：检测到球触发发射机构
* @param  argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  // 发球机构参数定义
  #define CUSHION_READY_DEG   47.0f    // 初始角度
  #define CUSHION_SPEED      2.06f     // 初始速度
  #define ACTION_HOLD_MS      800      // 动作保持时间 (ms)
  const float CUSHION_ACTION_DEG = 114.0f; // 发射角度

  printf("[task5] started\r\n");

  osDelay(1000); // 系统启动延时，等待硬件稳定
  
  // 初始化发球机构，回到初始位置，重复两次确保到位
  Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, CUSHION_SPEED);
  osDelay(500);
  Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, CUSHION_SPEED);
  printf("System Ready. Angle reset.\r\n"); // 系统就绪提示

  /* 无限循环 */
  for(;;)
  {
    // 1. 读取光电门信号，低电平表示检测到球
    if (HAL_GPIO_ReadPin(PHOTO_GATE_GPIO_Port, PHOTO_GATE_Pin) == GPIO_PIN_RESET)
    {
        osDelay(5); // 硬件消抖5ms
        if (HAL_GPIO_ReadPin(PHOTO_GATE_GPIO_Port, PHOTO_GATE_Pin) == GPIO_PIN_RESET)
        {
            // 限制延时时间不为负
            if(time_ms < 0)
            {
                time_ms = 0;
            }
            osDelay((uint32_t)time_ms); // 发球延时时间控制
            
            // === 检测到球，执行发射动作 ===
            printf("Ball detected! Action!\r\n");

            // 拨叉发射动作
            Mechanism_Cushion_SetAngle(CUSHION_ACTION_DEG, PITCH);
            osDelay(ACTION_HOLD_MS); // 保持一段时间确保球发出

            // 拨叉复位
            Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, 0.3f);
            
            // 等待球离开光电门，防止重复触发
            while (HAL_GPIO_ReadPin(PHOTO_GATE_GPIO_Port, PHOTO_GATE_Pin) == GPIO_PIN_RESET)
            {
                osDelay(10);
            }
            
            osDelay(500); // 复位稳定延时
            printf("Ready.\r\n"); // 就绪提示
        }
    }
    // 循环检测，间隔500ms
    osDelay(500);
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief  执行机构循环任务：1ms 周期更新电机状态
* @param  argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* 无限循环 */
  for(;;)
  {
    Mechanism_Loop_1ms(); // 执行机构1ms周期处理（接收/状态更新）
    osDelay(1);           // 1ms 延时
  }
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief  CAN3 发送任务：云台角度上报，1ms 周期
* @param  argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
  FDCAN_TxHeaderTypeDef TxHeader; // CAN 发送帧头
  uint8_t TxData[12];             // 发送数据缓冲区
  
  // 初始化 CAN 发送帧头
  TxHeader.Identifier = CAN_ID_PC_FEEDBACK; // 帧ID：0x300（PC反馈专用）
  TxHeader.IdType = FDCAN_STANDARD_ID;      // 标准ID
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;  // 数据帧
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;  // 数据长度：8字节
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;   // 关闭波特率切换，使用经典CAN
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;    // 经典CAN格式
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
  
  // 准备精确延时开始时间，1ms周期
  TickType_t xLastWakeTime = osKernelGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms执行一次

  /* 无限循环 */
  for(;;)
  {
      uint32_t current_timestamp = HAL_GetTick(); // 当前时间戳 (ms)
      float yaw_angle = 0.0f;                     // 云台 Yaw 角度
      float pitch_angle = 0.0f;                   // 云台 Pitch 角度
      
      // 获取云台实时角度
      gimbal_get_angles(&yaw_angle, &pitch_angle);

      // 数据压缩：角度*100转为int16_t，保留两位小数
      int16_t yaw_send   = (int16_t)(yaw_angle * 100.0f);
      int16_t pitch_send = (int16_t)(pitch_angle * 100.0f);

      // 数据打包
      memcpy(&TxData[0], &current_timestamp, 4); // 0-3字节：时间戳 (uint32_t)
      memcpy(&TxData[4], &yaw_send, 2);          // 4-5字节：Yaw角度 (int16_t)
      memcpy(&TxData[6], &pitch_send, 2);        // 6-7字节：Pitch角度 (int16_t)

      // 发送到 CAN3 总线
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData);

      // 精确延时，保证1ms周期
      vTaskDelayUntil(&xLastWakeTime, xFrequency);  
  }
  /* USER CODE END StartTask07 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
  * @brief  检查线程是否创建成功，失败则死循环
  * @param  name: 线程名称
  * @param  handle: 线程句柄
  * @retval None
  */
static void RTOS_CheckThreadCreated(const char *name, osThreadId_t handle)
{
    if (handle == NULL)
    {
        printf("[RTOS] create %s failed, free_heap=%lu\r\n", name, (unsigned long)xPortGetFreeHeapSize());
        taskDISABLE_INTERRUPTS();
        for (;;)
        {
        }
    }
}

/**
  * @brief  内存分配失败钩子函数
  * @param  None
  * @retval None
  */
#if (configUSE_MALLOC_FAILED_HOOK == 1)
void vApplicationMallocFailedHook(void)
{
    printf("[RTOS] malloc failed, free_heap=%lu\r\n", (unsigned long)xPortGetFreeHeapSize());
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}
#endif

/**
  * @brief  栈溢出钩子函数
  * @param  xTask: 任务句柄
  * @param  pcTaskName: 任务名称
  * @retval None
  */
#if (configCHECK_FOR_STACK_OVERFLOW > 0)
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
    (void)xTask;
    printf("[RTOS] stack overflow: %s\r\n", (pcTaskName != NULL) ? (char *)pcTaskName : "unknown");
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}
#endif

/* -------------------------------------------------------------------------
// 串口 DMA 接收空闲中断回调函数，用于接收遥控器原始数据
// ------------------------------------------------------------------------- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    
    if (huart->Instance == USART10) { // 判断是否为 USART10（遥控器串口）
        UartRxMessage_t rx_msg;
        // 数据长度保护，防止缓冲区溢出
        uint16_t copy_size = (Size < sizeof(rx_msg.data)) ? Size : sizeof(rx_msg.data);
        
        // 复制接收到的数据到消息结构体
        memcpy(rx_msg.data, remote_Buffer, copy_size);
        rx_msg.size = copy_size;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // 在中断中发送消息到队列
        xQueueSendFromISR(remote_queueHandle, &rx_msg, &xHigherPriorityTaskWoken);
        
        // 重新开启 DMA 接收，循环接收
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_Buffer, RC_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); 

        // 触发任务切换（如果需要）
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
  * @brief  串口错误回调函数，用于数据接收出错时恢复
  * @param  huart: 串口句柄
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART10) {
        // 清除错误标志（奇偶校验/帧错误/溢出/噪声）
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_ORE | UART_FLAG_NE);
        // 重新开启 DMA 接收，恢复通信
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_Buffer, RC_BUFFER_SIZE);
    }
}
/* USER CODE END Application */