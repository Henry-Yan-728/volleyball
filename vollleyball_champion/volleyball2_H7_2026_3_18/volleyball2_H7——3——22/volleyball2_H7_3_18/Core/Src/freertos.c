/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    freertos.c
  * @brief   FreeRTOS应用层代码，包含任务定义、初始化及业务逻辑
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

// --- 板级支持包(BSP)与业务模块头文件 ---
#include "fdcan_bsp.h"      // CAN通信驱动
#include "robot_data.h"     // 全局机器人数据结构定义
#include "chassis_task.h"   // 底盘运动控制任务
#include "mechanism_task.h" // 机构控制任务(拨球/发球)
#include "usart.h" 

// --- 遥控器协议解析与指令处理 ---
#include "Task_command.h"   // 指令解析接口
#include "remote_driver.h"  // 遥控器原始数据结构定义
#include "queue.h"          // FreeRTOS队列支持
#include "semphr.h"         // FreeRTOS互斥量/信号量支持
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
// 发球延迟时间增加步长(ms)
#define time_add 1

// --- 发球动作状态机枚举 ---
typedef enum {
    ACTION_IDLE = 0, // 空闲状态，等待触发
    ACTION_STEP_1,   // 执行阶段1：拨球+发球机构蓄力
    ACTION_STEP_2    // 执行阶段2：复位+完成发球
} ActionState_e;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RC_BUFFER_SIZE  64  // 遥控器DMA接收缓冲区大小
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
float time_ms = 1;                  // 发球延迟时间变量(ms)
float PITCH = 4.06f;                // Pitch轴缓冲力度变量

// --- FreeRTOS内核对象句柄 ---
osMutexId_t rc_mutexHandle;            // 保护全局遥控器数据g_remote_data的互斥量
osMessageQueueId_t remote_queueHandle; // 消息队列：传递UART中断接收到的原始遥控器数据

// 互斥量属性配置
const osMutexAttr_t rc_mutex_attributes = {
  .name = "rcMutex"
};

// 检测到球的二值信号量
osSemaphoreId_t BinarySem_BallDetectHandle;
const osSemaphoreAttr_t BinarySem_BallDetect_attributes = {
  .name = "BinarySem_BallDetect"
};

// --- 全局业务变量 ---
remote_engineer_t g_remote_data = {0}; // 全局遥控器数据结构体
uint8_t remote_Buffer[RC_BUFFER_SIZE]; // 遥控器DMA接收原始数据缓冲区
uint8_t processsed_command[COMMAND_LENGTH]; // 解析后的指令数据缓冲区

// 外部声明：遥控器原始数据结构
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
  * @brief  FreeRTOS初始化函数
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  // 1. 硬件与业务模块初始化(顺序不可随意调换)
  fdcan_bsp_init();     // CAN总线底层初始化
  Robot_Data_Init();    // 全局机器人数据结构初始化
  Chassis_Init();       // 底盘运动控制初始化
  Mechanism_Init();     // 机构控制初始化(拨球/发球)

  HAL_Delay(100); // 硬件初始化稳定延时

  // 2. 启动CAN总线通信
  fdcan_bsp_start(&hfdcan1); 
  fdcan_bsp_start(&hfdcan2); 
  fdcan_bsp_start(&hfdcan3); 

  // 3. 创建保护全局遥控器数据的互斥量
  rc_mutexHandle = osMutexNew(&rc_mutex_attributes);

  // 4. 创建遥控器数据消息队列：容量16，每个元素为UartRxMessage_t类型
  remote_queueHandle = osMessageQueueNew(16, sizeof(UartRxMessage_t), NULL);
	
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* 在此处添加互斥量 */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  // 创建二值信号量：初始计数1，最大计数1
  BinarySem_BallDetectHandle = osSemaphoreNew(1, 1, &BinarySem_BallDetect_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* 在此处添加定时器 */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* 在此处添加队列 */
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
  /* 在此处创建线程 */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* 在此处添加事件 */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  默认任务：1ms周期，更新虚拟轴(映射遥控器输入)
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
    Update_Virtual_Axis(); // 更新虚拟轴数据(遥控器输入映射)
    osDelay(1);            // 延时1ms，约1000Hz执行频率
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief  核心运动任务：底盘控制+机构控制+发球逻辑，10ms周期
* @param  argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms执行周期，100Hz
  xLastWakeTime = osKernelGetTickCount();

  // --- 1. 本地变量初始化 ---
  remote_engineer_t local_rc = {0}; // 本地遥控器数据副本，减少互斥锁持有时间
  float target_vx = 0, target_vy = 0, target_vr = 0; // 底盘目标速度：x/y平移，z旋转
  
  // --- 2. 机构控制参数定义 ---
  const float CUSHION_READY_DEG = 47.0f;   // 拨球机构准备角度
  const float CUSHION_ACTION_DEG = 114.0f;// 拨球机构发球角度
  const float SERVE_READY_RAD    = 30.0f; // 发球机构准备角度
  const float SERVE_ACTION_RAD   = 210.0f;// 发球机构发球角度
	float wheel_r=0.046f;
  float PITCH_ANGLE = 0.0f;               // Pitch轴目标角度

  // 发球状态机变量
  ActionState_e serve_state = ACTION_IDLE;
  uint32_t serve_start_tick = 0;          // 发球动作开始时间戳
  
  // 按键消抖延时定义
  #define DEBOUNCE_DELAY_MS 10

  // 按键历史记录：记录上一帧状态，用于检测边沿触发
  static uint32_t last_time_btn5 = 0; // 按键5上次触发时间
  static uint32_t last_time_btn2 = 0; // 按键2上次触发时间
  static uint32_t last_time_btn4 = 0; // 按键4上次触发时间
  static uint32_t last_time_btn1 = 0; // 按键1上次触发时间
  static uint32_t last_time_btn3 = 0; // 按键3上次触发时间

  // 按键上一帧状态
  static uint8_t last_button5 = 0;
  static uint8_t last_button2 = 0;
  static uint8_t last_button4 = 0;
  static uint8_t last_button1 = 0;
  static uint8_t last_button3 = 0;
  
  // 发球模式按键状态
  static uint8_t last_button_front = 0;
  static uint8_t last_button_back = 0;

  /* 无限循环 */
  for(;;)
  {
      uint32_t current_tick = HAL_GetTick(); // 获取系统当前时间(ms)

      // ============================================================
      // 1. 互斥锁安全获取全局遥控器数据
      // ============================================================
      if (osMutexAcquire(rc_mutexHandle, 10) == osOK) {
          local_rc = g_remote_data; // 拷贝全局数据到本地
          osMutexRelease(rc_mutexHandle); // 释放互斥锁
      }
      // ============================================================
      // 2. 底盘模式分支：手动/自动/发球定位
      // ============================================================
      if (local_rc.mode == CHASSIS_MODE_MANUAL) 
      {
          /* --- 手动模式：遥控器直接控制速度 --- */
          target_vx = local_rc.vx / 10.0f*wheel_r;  // X轴速度(归一化-1~1)
          target_vy = local_rc.vy / 10.0f*wheel_r;  // Y轴速度(归一化-1~1)
          target_vr = -local_rc.vw * 3.0f;   // 旋转速度(系数3)
          // --- 按键5：触发发球动作(边沿检测+消抖) ---
          if (local_rc.button5 == 1 && last_button5 == 0) {
              if ((current_tick - last_time_btn5) > DEBOUNCE_DELAY_MS) {
                  if (serve_state == ACTION_IDLE) { // 仅在空闲时触发
                      serve_state = ACTION_STEP_1;
                      serve_start_tick = current_tick;
                  }
                  last_time_btn5 = current_tick; // 更新上次触发时间
              }
          }
          last_button5 = local_rc.button5; // 更新按键状态

          // --- 按键2/4：Pitch轴角度粗调(±5度) ---
          if (local_rc.button2 == 1 && last_button2 == 0) {
              if ((current_tick - last_time_btn2) > DEBOUNCE_DELAY_MS) {
                  PITCH_ANGLE += 5.0f;
                  Mechanism_Dian_Pitch_SetAngle(PITCH_ANGLE); // 设置Pitch角度
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

          // --- 按键1/3：Pitch轴力度微调(±0.3) ---
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
          // 更新底盘速度指令
          Chassis_Update(target_vx, target_vy, target_vr);
      }
      else if (local_rc.mode == CHASSIS_MODE_AUTO)
      {
          /* --- 自动模式：路径规划控制速度 --- */
          float cur_x, cur_y, cur_yaw;    // 底盘当前位置：x/y坐标，偏航角
          uint32_t last_update;           // 位置更新时间戳
          
          // 临界区安全获取全局位置
          taskENTER_CRITICAL();
          cur_x = g_robot_pose.x;
          cur_y = g_robot_pose.y;
          cur_yaw = g_robot_pose.angle;
          last_update = g_robot_pose.update_tick;
          taskEXIT_CRITICAL();

          // 位置数据超时检测：超过1s未更新则停止底盘
          if (HAL_GetTick() - last_update > 1000) {
               target_vx = 0; target_vy = 0; target_vr = 0;
          }
          else 
          {
              // 计算目标位置偏差
              float err_x = g_robot_target.target_x - cur_x;
              float err_y = g_robot_target.target_y - cur_y;
              
              // 路径规划初始化
              static uint8_t initialized = 0;
              if (!initialized) {
                  Planner_SetTarget(cur_x, cur_y, err_x, err_y, 0.0f); 
                  initialized = 1;
              }

              // 路径规划更新，获取速度指令
              TrajVel_t cmd = Planner_Update(cur_x, cur_y, cur_yaw);
              
              // 速度坐标系转换：世界坐标系 -> 机器人坐标系
              float theta = cur_yaw * (M_PI / 180.0f); // 角度转弧度
              target_vx =  cmd.vx * cosf(theta) + cmd.vy * sinf(theta);
              target_vy = -cmd.vx * sinf(theta) + cmd.vy * cosf(theta);
              target_vr = cmd.vr;
              
              // 到达目标位置后，重置规划器
              if (Planner_IsArrived(cur_x, cur_y, cur_yaw)) {
                  initialized = 0;
              }
          }
          Chassis_Update(target_vx, target_vy, target_vr);
      }
      else if (local_rc.mode == CHASSIS_MODE_SERVE)
      {
          /* --- 发球定位模式：自动导航到指定位姿 --- */
          float cur_x, cur_y, cur_yaw, target_x = 0.0f, target_y = 0.0f;
          uint32_t last_update;
          
          // 临界区获取当前位置
          taskENTER_CRITICAL();
          cur_x = g_robot_pose.x;
          cur_y = g_robot_pose.y;
          cur_yaw = g_robot_pose.angle;
          last_update = g_robot_pose.update_tick;
          taskEXIT_CRITICAL();
				
          // 按键5：定位到原点(0,0)
          if (local_rc.button5 == 1 && last_button5 == 0) {
              target_x = 0.0f;
              target_y = 0.0f;
          }
          last_button5 = local_rc.button5;
				
          // 按键2：定位到(800,500)，按键4：定位到(1000,1000)
          if (local_rc.button2 == 1 && last_button_front == 0) {
              target_x = 800.0f;
              target_y = 500.0f;
          }else if (local_rc.button4 == 1 && last_button_back == 0) {
              target_x = 1000.0f;
              target_y = 1000.0f;
          }
          last_button_front = local_rc.button2;
          last_button_back = local_rc.button4;

          // 位置数据超时检测
          if (HAL_GetTick() - last_update > 1000) {
               target_vx = 0; target_vy = 0; target_vr = 0;
          }
          else 
          {
              // 计算目标偏差
              float err_x = target_x - cur_x;
              float err_y = target_y - cur_y;
              
              // 路径规划初始化
              static uint8_t initialized = 0;
              if (!initialized) {
                  Planner_SetTarget(cur_x, cur_y, err_x, err_y, 0.0f);
                  initialized = 1;
              }

              // 路径规划更新
              TrajVel_t cmd = Planner_Update(cur_x, cur_y, cur_yaw);
              
              // 坐标系转换
              float theta = cur_yaw * (M_PI / 180.0f);
              target_vx =  cmd.vx * cosf(theta) + cmd.vy * sinf(theta);
              target_vy = -cmd.vx * sinf(theta) + cmd.vy * cosf(theta);
              target_vr = cmd.vr;
              
              // 更新底盘速度
              Chassis_Update(target_vx, target_vy, target_vr);

              // 到达目标位置后自动触发发球
              if (Planner_IsArrived(cur_x, cur_y, cur_yaw)) {
                  if (serve_state == ACTION_IDLE) {
                      serve_state = ACTION_STEP_1;
                      serve_start_tick = current_tick;
                  }
              }
          }
      }

      // ============================================================
      // 3. 发球动作状态机执行
      // ============================================================
      switch (serve_state) {
          case ACTION_STEP_1:
              // 阶段1：拨球到发球位 + 发球机构蓄力
              Mechanism_Cushion_SetAngle(CUSHION_ACTION_DEG, PITCH);
					osDelay(1);
              Mechanism_Serve_SetAngle(SERVE_ACTION_RAD); 
							Chassis_Update(1, 0, 0);
							osDelay(500);
              // 延时300ms后进入阶段2
              if (current_tick - serve_start_tick > 300) {
                  serve_state = ACTION_STEP_2;
              }
              break;

          case ACTION_STEP_2:
              // 阶段2：拨球复位 + 保持发球机构
              Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, 0.3f);
										osDelay(1);
              Mechanism_Serve_SetAngle(SERVE_ACTION_RAD);

              // 延时500ms后复位发球机构，回到空闲状态
              if (current_tick - serve_start_tick > 500) {
                  Mechanism_Serve_SetAngle(SERVE_READY_RAD); // 发球机构复位
                  serve_state = ACTION_IDLE;
              }
              break;

          default: break;
      }

      // 精确延时，保证10ms周期
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief  CAN2发送任务：发送底盘位置数据，高频循环
* @param  argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  FDCAN_TxHeaderTypeDef TxHeader; // CAN发送帧头
  uint8_t TxData[12];             // CAN发送数据缓冲区，12字节
  
  // 初始化CAN发送帧头
  TxHeader.Identifier = 0x101;                // 帧ID：0x101
  TxHeader.IdType = FDCAN_STANDARD_ID;        // 标准ID(11位)
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;    // 数据帧
  TxHeader.DataLength = FDCAN_DLC_BYTES_12;   // 数据长度：12字节
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // 错误状态指示
  TxHeader.BitRateSwitch = FDCAN_BRS_ON;      // 开启波特率切换
  TxHeader.FDFormat = FDCAN_FD_CAN;           // FD CAN模式
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 无发送事件
  TxHeader.MessageMarker = 0;                 // 消息标记：0

  /* 无限循环 */
  for(;;)
  {
      float temp_x, temp_y, temp_angle;
      
      // 临界区安全获取位置数据
      taskENTER_CRITICAL();
      temp_x = g_robot_pose.x;
      temp_y = g_robot_pose.y;
      temp_angle = g_robot_pose.angle;
      taskEXIT_CRITICAL();

      // 浮点数打包为字节流
      memcpy(&TxData[0], &temp_x, 4);     // 0-3字节：X坐标(float)
      memcpy(&TxData[4], &temp_y, 4);     // 4-7字节：Y坐标(float)
      memcpy(&TxData[8], &temp_angle, 4); // 8-11字节：偏航角(float)

      // 通过CAN2发送数据
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
      osDelay(1); // 约1000Hz发送频率
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief  遥控器数据解析任务：处理UART DMA接收到的原始数据
* @param  argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  // 启动UART10 DMA空闲中断接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart10, remote_Buffer, RC_BUFFER_SIZE);
  // 关闭DMA半满中断，仅使用空闲中断
  __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT); 

  UartRxMessage_t rx_msg;        // 接收到的消息结构体
  remote_engineer_t temp_rc;     // 临时遥控器数据

  /* 无限循环 */
  for(;;)
  {
      // 阻塞等待消息队列数据
      if (osMessageQueueGet(remote_queueHandle, &rx_msg, NULL, osWaitForever) == osOK)
      {
          // 将原始数据写入指令解析器
          Command_Write(rx_msg.data, rx_msg.size);
          
          // 循环解析所有可用指令
          while (Command_GetCommand(processsed_command) != 0)
          {
              code_unzipread(processsed_command);   // 解压缩指令
              Remote_Data_Convert(&rc, &temp_rc);   // 转换为标准数据格式
              
              // 互斥锁安全更新全局遥控器数据
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
* @brief  发球检测任务：检测光电门触发并执行发球
* @param  argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  // 机构控制参数定义
  #define CUSHION_READY_DEG   47.0f    // 初始角度
  #define CUSHION_SPEED      2.06f     // 初始速度
  #define ACTION_HOLD_MS      800      // 动作保持时间(ms)
  const float CUSHION_ACTION_DEG = 114.0f; // 发球角度

  osDelay(1000); // 系统启动稳定延时，等待硬件就绪
  
  // 初始化拨球机构到初始位置，确保复位正确
  Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, CUSHION_SPEED);
  osDelay(500);
  Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, CUSHION_SPEED);
  printf("System Ready. Angle reset.\r\n"); // 系统就绪提示

  /* 无限循环 */
  for(;;)
  {
    // 1. 检测光电门信号，低电平表示检测到球
    if (HAL_GPIO_ReadPin(PHOTO_GATE_GPIO_Port, PHOTO_GATE_Pin) == GPIO_PIN_RESET)
    {
        osDelay(5); // 硬件消抖5ms
        if (HAL_GPIO_ReadPin(PHOTO_GATE_GPIO_Port, PHOTO_GATE_Pin) == GPIO_PIN_RESET)
        {
            // 确保延时时间不为负
            if(time_ms < 0)
            {
                time_ms = 0;
            }
            osDelay(time_ms); // 延时等待球到位
            
            // === 检测到球，执行发球动作 ===
            printf("Ball detected! Action!\r\n");

            // 拨球机构动作
            Mechanism_Cushion_SetAngle(CUSHION_ACTION_DEG, PITCH);
            osDelay(ACTION_HOLD_MS); // 保持动作确保发球完成

            // 拨球机构复位
            Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, 0.3f);
            
            // 等待光电门信号恢复(球离开)
            while (HAL_GPIO_ReadPin(PHOTO_GATE_GPIO_Port, PHOTO_GATE_Pin) == GPIO_PIN_RESET)
            {
                osDelay(10);
            }
            
            osDelay(500); // 复位稳定延时
            printf("Ready.\r\n"); // 就绪提示
        }
    }
    // 循环检测间隔：500ms
    osDelay(500);
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief  机构循环任务：1ms周期更新机构指令与状态
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
    Mechanism_Loop_1ms(); // 机构1ms周期循环：更新指令/状态
    osDelay(1);           // 1ms延时
  }
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief  CAN3发送任务：发送云台角度数据，1ms周期
* @param  argument: 未使用
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
  FDCAN_TxHeaderTypeDef TxHeader; // CAN发送帧头
  uint8_t TxData[12];             // 发送数据缓冲区
  
  // 初始化CAN发送帧头
  TxHeader.Identifier = CAN_ID_PC_FEEDBACK; // 帧ID：0x300(PC反馈专用)
  TxHeader.IdType = FDCAN_STANDARD_ID;      // 标准ID
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;  // 数据帧
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;  // 数据长度：8字节
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;   // 关闭波特率切换(经典CAN)
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;    // 经典CAN模式
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
  
  // 精确延时初始化，1ms周期
  TickType_t xLastWakeTime = osKernelGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms执行周期

  /* 无限循环 */
  for(;;)
  {
      uint32_t current_timestamp = HAL_GetTick(); // 当前时间戳(ms)
      float yaw_angle = 0.0f;                     // 云台Yaw角度
      float pitch_angle = 0.0f;                   // 云台Pitch角度
      
      // 获取云台实时角度
      gimbal_get_angles(&yaw_angle, &pitch_angle);

      // 角度压缩：乘以100转为int16_t，保留2位小数
      int16_t yaw_send   = (int16_t)(yaw_angle * 100.0f);
      int16_t pitch_send = (int16_t)(pitch_angle * 100.0f);

      // 数据打包
      memcpy(&TxData[0], &current_timestamp, 4); // 0-3字节：时间戳(int32_t)
      memcpy(&TxData[4], &yaw_send, 2);          // 4-5字节：Yaw角度(int16_t)
      memcpy(&TxData[6], &pitch_send, 2);        // 6-7字节：Pitch角度(int16_t)

      // 通过CAN3发送数据
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData);

      // 精确延时，保证1ms周期
      vTaskDelayUntil(&xLastWakeTime, xFrequency);  
  }
  /* USER CODE END StartTask07 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* -------------------------------------------------------------------------
// UART DMA空闲中断回调：接收一帧数据后发送到消息队列
// ------------------------------------------------------------------------- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    
    if (huart->Instance == USART10) { // 仅处理USART10(遥控器)的中断
        UartRxMessage_t rx_msg;
        // 数据长度限制：防止溢出
        uint16_t copy_size = (Size < sizeof(rx_msg.data)) ? Size : sizeof(rx_msg.data);
        
        // 拷贝接收到的数据到消息结构体
        memcpy(rx_msg.data, remote_Buffer, copy_size);
        rx_msg.size = copy_size;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // 中断安全方式发送数据到消息队列
        xQueueSendFromISR(remote_queueHandle, &rx_msg, &xHigherPriorityTaskWoken);
        
        // 重新启动DMA接收，循环接收
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_Buffer, RC_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); 

        // 触发任务切换(如果需要)
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
  * @brief  UART错误中断回调：处理接收错误并恢复通信
  * @param  huart: UART句柄
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART10) {
        // 清除所有错误标志：奇偶校验/帧错误/溢出/噪声
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_ORE | UART_FLAG_NE);
        // 重新启动DMA接收，恢复通信
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_Buffer, RC_BUFFER_SIZE);
    }
}
/* USER CODE END Application */