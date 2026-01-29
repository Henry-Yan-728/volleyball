/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * 文件名          : app_freertos.c
  * 描述            : FreeRTOS 应用程序代码，包含任务调度与逻辑控制
  ******************************************************************************
  */
/* USER CODE END Header */

/* 包含头文件 ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* 私有包含 ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdio.h>

// --- 板级支持包 (BSP) 和 机器人功能模块 ---
#include "fdcan_bsp.h"      // CAN驱动
#include "robot_data.h"     // 全局机器人数据结构
#include "chassis_task.h"   // 底盘底层控制
#include "mechanism_task.h" // 机构（拨杆、发球等）控制
#include "usart.h" 

// --- 遥控器协议处理与指令解析 ---
#include "Task_command.h"   // 提供指令写入与获取接口
#include "remote_driver.h"  // 提供遥控器原始解算数据结构
#include "queue.h"          // 队列支持
#include "semphr.h"         // 信号量/互斥量支持
#include "chassis_path_task.h" // 路径规划相关

// 外部引用硬件句柄
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern UART_HandleTypeDef huart2; 
/* USER CODE END Includes */

/* 私有类型定义 -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define M_PI 3.1415926535f

// --- 动作序列状态机枚举：用于处理需要延时配合的动作 ---
typedef enum {
    ACTION_IDLE = 0, // 空闲状态
    ACTION_STEP_1,   // 动作执行阶段 1 (如：机构伸出)
    ACTION_STEP_2    // 动作执行阶段 2 (如：机构复位)
} ActionState_e;
/* USER CODE END PTD */

/* 私有宏定义 ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RC_BUFFER_SIZE  64  // 串口接收缓冲区大小
/* USER CODE END PD */

/* 私有变量 ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// --- FreeRTOS 操作系统对象 ---
osMutexId_t rc_mutexHandle;            // 保护全局遥控器变量 g_remote_data 的互斥锁
osMessageQueueId_t remote_queueHandle; // 消息队列：用于串口中断将原始数据传给解析任务

const osMutexAttr_t rc_mutex_attributes = {
  .name = "rcMutex"
};

// --- 全局业务变量 ---
remote_engineer_t g_remote_data = {0}; // 最终解算出的遥控器结构体
uint8_t remote_Buffer[RC_BUFFER_SIZE]; // 串口 DMA 接收原始数据的内存空间
uint8_t processsed_command[COMMAND_LENGTH]; // 指令解析临时缓冲区
float pitch = 0.0f;
// 外部引用原始遥控器数据结构
extern rc_info_t rc; 

/* 任务句柄定义 */
osThreadId_t defaultTaskHandle;
osThreadId_t myTask02Handle; // 主控任务：底盘 + 动作逻辑
osThreadId_t myTask03Handle; // 通讯任务：位置上报
osThreadId_t myTask04Handle; // 解析任务：遥控器协议解算
osThreadId_t myTask05Handle; // 自动逻辑任务：光电门触发

/* 任务属性配置 */
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask", .priority = (osPriority_t) osPriorityNormal, .stack_size = 256 * 4 };
const osThreadAttr_t myTask02_attributes = { .name = "myTask02", .priority = (osPriority_t) osPriorityLow, .stack_size = 256 * 4 };
const osThreadAttr_t myTask03_attributes = { .name = "myTask03", .priority = (osPriority_t) osPriorityLow, .stack_size = 256 * 4 };
const osThreadAttr_t myTask04_attributes = { .name = "myTask04", .priority = (osPriority_t) osPriorityLow, .stack_size = 256 * 4 };
const osThreadAttr_t myTask05_attributes = { .name = "myTask05", .priority = (osPriority_t) osPriorityLow, .stack_size = 256 * 4 };

/* USER CODE END Variables */

/* 私有函数原型 ---------------------------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);

void MX_FREERTOS_Init(void); 
/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS 初始化
  * @retval 无
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  
  // 1. 初始化硬件控制相关数据与底层驱动
  fdcan_bsp_init();     // CAN总线初始化
  Robot_Data_Init();    // 全局数据结构初始化
  Chassis_Init();       // 底盘参数初始化
  Mechanism_Init();     // 机构参数初始化

  osDelay(100);

  // 2. 启动所有 CAN 通讯口
  fdcan_bsp_start(&hfdcan1); 
  fdcan_bsp_start(&hfdcan2); 
  fdcan_bsp_start(&hfdcan3); 

  // 3. 创建互斥锁 (用于线程安全)
  rc_mutexHandle = osMutexNew(&rc_mutex_attributes);

  // 4. 创建指令消息队列
  remote_queueHandle = osMessageQueueNew(16, sizeof(UartRxMessage_t), NULL);
	
  // 5. 开启 UART2 DMA 循环接收，检测空闲中断 (ReceiveToIdle)
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, remote_Buffer, RC_BUFFER_SIZE);
  // 禁用半传输中断，只处理一帧完成后的空闲中断
  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT); 

  /* USER CODE END Init */

  /* 创建任务线程 */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);
  myTask04Handle = osThreadNew(StartTask04, NULL, &myTask04_attributes);
  myTask05Handle = osThreadNew(StartTask05, NULL, &myTask05_attributes);
}

/**
* @brief  Task02: 底盘运动与机构逻辑控制
* 循环周期：10ms
*/
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms 控制周期
  xLastWakeTime = osKernelGetTickCount();

  // --- 1. 局部控制变量初始化 ---
  remote_engineer_t local_rc = {0}; 
  float target_vx = 0, target_vy = 0, target_vr = 0;
  
  // --- 2. 机构动作参数配置 (角度/速度) ---
  const float CUSHION_ORIGIN_DEG = 57.0f;   // 垫球机构复位角度 (deg)
  const float CUSHION_ACTION_DEG = 114.0f;  // 垫球机构动作角度 (deg)
  const float SERVE_READY_RAD    = 30.0f;   // 发球电机复位转速
  const float SERVE_ACTION_RAD   = 210.0f;  // 发球电机动作转速
  float PITCH_ANGLE = 0.0f;                 // Pitch 轴角度控制变量

  // 发球动作状态机相关变量
  ActionState_e serve_state = ACTION_IDLE;
  uint32_t serve_start_tick = 0;
  uint8_t last_button5 = 0;

  // 垫球机构调节按键历史记录 (用于边沿检测)
  uint8_t last_button_front = 0;
  uint8_t last_button_back = 0;

  /* 任务主循环 */
  for(;;)
  {
      uint32_t current_tick = HAL_GetTick(); // 获取当前系统时间

      // ============================================================
      // 1. 安全地从全局变量获取最新的遥控器数据 (加锁保护)
      // ============================================================
      if (osMutexAcquire(rc_mutexHandle, 10) == osOK) {
          local_rc = g_remote_data; 
          osMutexRelease(rc_mutexHandle);
      }

      // ============================================================
      // 2. 底盘运动控制逻辑 (分模式处理)
      // ============================================================
      if (local_rc.mode == CHASSIS_MODE_MANUAL) 
      {
          /* --- 手动模式 --- */
          target_vx = local_rc.vx / 100.0f;  // 左右平移
          target_vy = local_rc.vy / 100.0f;  // 前后平移
          target_vr = -local_rc.vw * 5.0f;   // 自旋

          // 手动触发发球状态机 (按键5上升沿)
          if (local_rc.button5 == 1 && last_button5 == 0) {
              if (serve_state == ACTION_IDLE) {
                  serve_state = ACTION_STEP_1;
                  serve_start_tick = current_tick;
              }
          }
          last_button5 = local_rc.button5;

          // 垫球机构 Pitch 轴角度手动调节 (按键2加，按键4减)
          if (local_rc.button2 == 1 && last_button_front == 0) {
              PITCH_ANGLE += 5.0f;
              Mechanism_Pitch_SetAngle(PITCH_ANGLE);
          } else if (local_rc.button4 == 1 && last_button_back == 0) {
              PITCH_ANGLE -= 5.0f;
              Mechanism_Pitch_SetAngle(PITCH_ANGLE);
          }
          last_button_front = local_rc.button2;
          last_button_back = local_rc.button4;
					
					if (local_rc.button1 == 1 && last_button_front == 0) {
              pitch += 0.1f;
          } else if (local_rc.button3 == 1 && last_button_back == 0) {
              pitch -= 0.1f;
          }
          last_button_front = local_rc.button1;
          last_button_back = local_rc.button3;



          // 执行底盘驱动更新
          Chassis_Update(target_vx, target_vy, target_vr);
      }
      else if (local_rc.mode == CHASSIS_MODE_AUTO)
      {
          /* --- 全自动模式 (路径规划) --- */
          float cur_x, cur_y, cur_yaw;
          uint32_t last_update;
          
          // 进入临界区，读取全局定位坐标，防止数据被中断修改导致不一致
          taskENTER_CRITICAL();
          cur_x = g_robot_pose.x;
          cur_y = g_robot_pose.y;
          cur_yaw = g_robot_pose.angle;
          last_update = g_robot_pose.update_tick;
          taskEXIT_CRITICAL();

          // 超时保护：如果定位坐标长时间不更新，停止运动
          if (HAL_GetTick() - last_update > 1000) {
               target_vx = 0; target_vy = 0; target_vr = 0;
          }
          else 
          {
              float err_x = g_robot_target.target_x - cur_x;
              float err_y = g_robot_target.target_y - cur_y;
              
              static uint8_t initialized = 0;
              if (!initialized) {
                  // 初始化规划器目标
                  Planner_SetTarget(cur_x, cur_y, err_x, err_y, 0.0f); 
                  initialized = 1;
              }

              // 获取规划好的世界系速度
              TrajVel_t cmd = Planner_Update(cur_x, cur_y, cur_yaw);
              
              // 世界坐标系速度转为车体系速度
              float theta = cur_yaw * (M_PI / 180.0f);
              target_vx =  cmd.vx * cosf(theta) + cmd.vy * sinf(theta);
              target_vy = -cmd.vx * sinf(theta) + cmd.vy * cosf(theta);
              target_vr = cmd.vr;
              if (Planner_IsArrived(cur_x, cur_y, cur_yaw)) {
								initialized = 0;//重启定位函数
							}
						}
          Chassis_Update(target_vx, target_vy, target_vr);
      }
      else if (local_rc.mode == CHASSIS_MODE_SERVE)
      {
          /* --- 自动对位发球模式 --- */
          float cur_x, cur_y, cur_yaw, target_x, target_y;
          uint32_t last_update;
          
          taskENTER_CRITICAL();
          cur_x = g_robot_pose.x;
          cur_y = g_robot_pose.y;
          cur_yaw = g_robot_pose.angle;
          last_update = g_robot_pose.update_tick;
          taskEXIT_CRITICAL();
				
				//复位按键
          if (local_rc.button5 == 1 && last_button5 == 0) {
						target_x = 0.0f;
						target_y = 0.0f;
          }
          last_button5 = local_rc.button5;
				
					//4键后场前发球，2键后场后发球
          if (local_rc.button2 == 1 && last_button_front == 0) {
						target_x = 800.0f;
						target_y = 500.0f;
					}else if (local_rc.button4 == 1 && last_button_back == 0) {
						target_x = 1000.0f;
						target_y = 1000.0f;
					}
          last_button_front = local_rc.button2;
          last_button_back = local_rc.button4;
					
          if (HAL_GetTick() - last_update > 1000) {
               target_vx = 0; target_vy = 0; target_vr = 0;
          }
          else 
          {
              float err_x = target_x - cur_x;
              float err_y = target_y - cur_y;
              
              static uint8_t initialized = 0;
              if (!initialized) {
                  Planner_SetTarget(cur_x, cur_y, err_x, err_y, 0.0f);
                  initialized = 1;
              }

              TrajVel_t cmd = Planner_Update(cur_x, cur_y, cur_yaw);
              
              float theta = cur_yaw * (M_PI / 180.0f);
              target_vx =  cmd.vx * cosf(theta) + cmd.vy * sinf(theta);
              target_vy = -cmd.vx * sinf(theta) + cmd.vy * cosf(theta);
              target_vr = cmd.vr;
              
              Chassis_Update(target_vx, target_vy, target_vr);

              // 如果到达预定发球点，触发发球机构
              if (Planner_IsArrived(cur_x, cur_y, cur_yaw)) {
                  if (serve_state == ACTION_IDLE) {
                      serve_state = ACTION_STEP_1;
                      serve_start_tick = current_tick;
                  }
              }
          }
      }

      // ============================================================
      // 3. 机构控制逻辑：发球状态机执行 (异步非阻塞)
      // ============================================================
      switch (serve_state) {
          case ACTION_STEP_1:
              // 动作：垫球机构配合抬起，发球电机转动
              Mechanism_Cushion_SetAngle(CUSHION_ACTION_DEG, 1.56f);
              Mechanism_Serve_SetAngle(SERVE_ACTION_RAD); 

              // 延时 300ms 后切换到复位阶段
              if (current_tick - serve_start_tick > 300) {
                  serve_state = ACTION_STEP_2;
              }
              break;

          case ACTION_STEP_2:
              // 动作：垫球机构复位，发球电机保持转动确保球射出
              Mechanism_Cushion_SetAngle(CUSHION_ORIGIN_DEG, 0.3f);
              Mechanism_Serve_SetAngle(SERVE_ACTION_RAD);

              // 延时总计 800ms 后动作结束，状态机回到空闲
              if (current_tick - serve_start_tick > 800) {
                  Mechanism_Serve_SetAngle(SERVE_READY_RAD); // 电机转速恢复待机
                  serve_state = ACTION_IDLE;
              }
              break;

          default: break;
      }

      // 任务频率控制
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartTask02 */
}

/**
* @brief Task03: 数据上报任务 (CAN-FD)
* 功能：将机器人当前的世界坐标系位姿通过 CAN 发送给其他节点
*/
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  FDCAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[12];
  
  // 初始化 FDCAN 发送帧头
  TxHeader.Identifier = 0x101;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_12; 
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_ON;   
  TxHeader.FDFormat = FDCAN_FD_CAN;        
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  for(;;)
  {
      float temp_x, temp_y, temp_angle;
      
      // 使用临界区读取位姿数据，确保 x,y,angle 的同步性
      taskENTER_CRITICAL();
      temp_x = g_robot_pose.x;
      temp_y = g_robot_pose.y;
      temp_angle = g_robot_pose.angle;
      taskEXIT_CRITICAL();

      // 数据打包 (Float -> Byte)
      memcpy(&TxData[0], &temp_x, 4);
      memcpy(&TxData[4], &temp_y, 4);
      memcpy(&TxData[8], &temp_angle, 4);

      // 发送到 CAN2
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
      osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/**
* @brief Task04: 遥控器指令解析任务
* 功能：从队列接收串口原始字节流，解算为可用的遥控器结构体
*/
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  UartRxMessage_t rx_msg;
  remote_engineer_t temp_rc; 

  for(;;)
  {
      // 阻塞等待消息队列中的数据包 (来自串口 ISR)
      if (osMessageQueueGet(remote_queueHandle, &rx_msg, NULL, osWaitForever) == osOK)
      {
          // 写入解析器缓冲区
          Command_Write(rx_msg.data, rx_msg.size);
          // 循环解析直到缓冲区内没有完整的指令包
          while (Command_GetCommand(processsed_command) != 0)
          {
              code_unzipread(processsed_command);   // 解码协议   
              Remote_Data_Convert(&rc, &temp_rc);   // 转换为通用工程数据格式             
              
              // 写入全局变量，加锁防止主控任务正在读取时数据发生篡改
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

/**
* @brief Task05: 自动接发球触发逻辑 (传感器驱动)
* 功能：通过光电门传感器检测球是否到位，自动执行垫球击球动作
*/
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
    #define CUSHION_READY_DEG   57.0f 
    #define CUSHION_SPEED       1.86f
    #define ACTION_HOLD_MS      800
    const float CUSHION_ACTION_DEG = 114.0f;

    osDelay(1000); // 上电延时，等待传感器稳定
    // 初始化击球板位置
    Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, CUSHION_SPEED);
    printf("System Ready. Angle reset.\r\n");

  for(;;)
  {
    // 1. 读取光电门引脚电平 (GPIOB 11)
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET)
    {
        osDelay(5); // 软件消抖
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET)
        {
            // === 检测到球！执行一次性击球任务 ===
            printf("Ball detected! Action!\r\n");
//					if(pitch>0)
//					{
//						pitch = pitch;
//					}else{
//					pitch = 50.0f;
//					}
//					osDelay(pitch);

            // 击球动作开始
            Mechanism_Cushion_SetAngle(CUSHION_ACTION_DEG, CUSHION_SPEED+pitch);
            osDelay(ACTION_HOLD_MS); // 保持一段时间确保球被打出

            // 复位动作
            Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, 0.3f);
            
            // 阻塞等待球离开光电门感应范围，防止连续触发
            while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET)
            {
                osDelay(10);
            }
            
            osDelay(500); // 动作冷却期
            printf("Ready.\r\n");
        }
    }
    
    // 空闲时的任务延时
    osDelay(5);
  }
  /* USER CODE END StartTask05 */
}

/* -------------------------------------------------------------------------
// 串口空闲中断/DMA 完成中断回调函数
// ------------------------------------------------------------------------- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    
    if (huart->Instance == USART2) { 
        UartRxMessage_t rx_msg;
        // 限制拷贝大小，防止内存越界
        uint16_t copy_size = (Size < sizeof(rx_msg.data)) ? Size : sizeof(rx_msg.data);
        
        memcpy(rx_msg.data, remote_Buffer, copy_size);
        rx_msg.size = copy_size;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // 通过队列将原始字节流发送给解析任务
        xQueueSendFromISR(remote_queueHandle, &rx_msg, &xHigherPriorityTaskWoken);
        
        // 重新开启下一次 DMA 接收
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_Buffer, RC_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); 

        // 如果发送导致了高优先级任务就绪，进行上下文切换
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// 串口硬件错误回调
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        // 清除溢出、奇偶校验等错误标志，防止串口卡死
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_ORE | UART_FLAG_NE);
        // 重新尝试开启 DMA
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_Buffer, RC_BUFFER_SIZE);
    }
}

/* 默认守护任务实现 */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  for(;;)
  {
      osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}