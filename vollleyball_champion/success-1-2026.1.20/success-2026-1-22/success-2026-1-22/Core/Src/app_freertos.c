/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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

// --- BSP 和 机器人模块 ---
#include "fdcan_bsp.h"
#include "robot_data.h"
#include "chassis_task.h"
#include "mechanism_task.h"
#include "usart.h" 

// --- 遥控器与指令 ---
#include "Task_command.h"   // 提供 Command_Write, Command_GetCommand
#include "remote_driver.h"  // 提供 remote_engineer_t, code_unzipread, Remote_Data_Convert
#include "queue.h"
#include "semphr.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern UART_HandleTypeDef huart2; // 已确认为 UART2
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define M_PI 3.1415926535f
// --- 动作序列状态机枚举 ---
typedef enum {
    ACTION_IDLE = 0, // 空闲
    ACTION_STEP_1,   // 阶段1 (动作/出)
    ACTION_STEP_2    // 阶段2 (复位/回)
} ActionState_e;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RC_BUFFER_SIZE  64  // [修改] 增大缓冲区，确保能容纳最大的遥控器数据帧
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// --- FreeRTOS 对象 ---
osMutexId_t rc_mutexHandle;            // 保护 g_remote_data
osMessageQueueId_t remote_queueHandle; // 串口ISR -> 解析任务

const osMutexAttr_t rc_mutex_attributes = {
  .name = "rcMutex"
};

// --- 全局变量 ---
remote_engineer_t g_remote_data = {0}; // [修改] 初始化为0
uint8_t remote_Buffer[RC_BUFFER_SIZE]; // [修改] 使用宏定义的大小
uint8_t processsed_command[COMMAND_LENGTH]; 

// 如果 rc 在 remote_driver.c 中定义，这里 extern 引用
extern rc_info_t rc; 

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
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);

void MX_FREERTOS_Init(void); 
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  
  // 1. 硬件与数据初始化
  fdcan_bsp_init();
  Robot_Data_Init();
  Chassis_Init(); 
  Mechanism_Init();

  osDelay(100);

  // 2. 开启 CAN
  fdcan_bsp_start(&hfdcan1); 
  fdcan_bsp_start(&hfdcan2); 
  fdcan_bsp_start(&hfdcan3); 

  // 3. 创建互斥锁
  rc_mutexHandle = osMutexNew(&rc_mutex_attributes);

  // 4. 创建消息队列
  remote_queueHandle = osMessageQueueNew(16, sizeof(UartRxMessage_t), NULL);
	
  // 5. 开启 UART2 DMA 接收
  // [修改] 使用 RC_BUFFER_SIZE
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, remote_Buffer, RC_BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT); 

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  for(;;)
  {
      osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief  底盘控制任务 (状态机：手动/自动) + 机构动作序列控制
*/
/* USER CODE END Header_StartTask02 */
__weak void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms 控制周期
  xLastWakeTime = osKernelGetTickCount();

  // --- 1. 底盘控制变量初始化 ---
  remote_engineer_t local_rc = {0}; 
  float target_vx = 0, target_vy = 0, target_vr = 0;
  
  // 自动模式参数
  float pos_kp = 1.5f; 
  float max_speed_auto = 1.0f; 

  // --- 2. 机构控制变量初始化 (接球/发球) ---
  // 动作参数配置 (根据机械结构修改这里)
  const float CUSHION_ORIGIN_DEG = 37.0f;   // 垫球机构：复位角度 (度)
  const float CUSHION_ACTION_DEG = 114.0f; // 垫球机构：动作角度 (度，注意正负方向)
  const float SERVE_READY_RAD   = 30.0f;  // 发球电机：复位角度 (rad/s)
  const float SERVE_ACTION_RAD   = 210.0f;  // 发球电机：动作角度 (rad/s)
  float PITCH_ANGLE = 0.0f;  // 垫球角度电机：旋转角度 (rad/s)

  // 发球逻辑变量 (Button 1)
  ActionState_e serve_state = ACTION_IDLE;
  uint32_t serve_start_tick = 0;
  uint8_t last_button5 = 0;

  //垫球角度改变量
  uint8_t last_button_front = 0;
  uint8_t last_button_back = 0;

  /* Infinite loop */
  for(;;)
  {
      uint32_t current_tick = HAL_GetTick(); // 获取当前系统时间 (ms)

      // ============================================================
      // 1. 安全获取遥控器数据 (Mutex保护)
      // ============================================================
      if (osMutexAcquire(rc_mutexHandle, 10) == osOK) {
          local_rc = g_remote_data; 
          osMutexRelease(rc_mutexHandle);
      } else {
          // 获取锁超时，保持上一帧数据或做急停处理
      }
      // ============================================================
      // 2. 底盘运动控制逻辑 (保持原有逻辑不变)
      // ============================================================
      if (local_rc.mode == CHASSIS_MODE_MANUAL) 
      {
          // --- 手动模式 ---
          target_vx = local_rc.vx/100;
          target_vy = local_rc.vy/100;
          target_vr = -local_rc.vw*5;
      }
      else 
      {
          // --- 自动/待机模式 ---
          float cur_x, cur_y, cur_angle;
          uint32_t last_update;
          
          taskENTER_CRITICAL();
          cur_x = g_robot_pose.x;
          cur_y = g_robot_pose.y;
          cur_angle = g_robot_pose.angle;
          last_update = g_robot_pose.update_tick;
          taskEXIT_CRITICAL();

          // 超时停车保护
          if (HAL_GetTick() - last_update > 1000) {
             target_vx = 0; target_vy = 0; target_vr = 0;
          }
          else 
          {
              float err_x = g_robot_target.target_x - cur_x;
              float err_y = g_robot_target.target_y - cur_y;
              
              float theta = cur_angle * (M_PI / 180.0f);
              float vx_body =  err_x * cosf(theta) + err_y * sinf(theta);
              float vy_body = -err_x * sinf(theta) + err_y * cosf(theta);

              if (sqrtf(err_x*err_x + err_y*err_y) > 0.05f) {
                  target_vx = vx_body * pos_kp;
                  target_vy = vy_body * pos_kp;
                  
                  // 矢量限幅
                  float speed_mod = sqrtf(target_vx*target_vx + target_vy*target_vy);
                  if (speed_mod > max_speed_auto) {
                      float ratio = max_speed_auto / speed_mod;
                      target_vx *= ratio;
                      target_vy *= ratio;
                  }
              } else {
                  target_vx = 0; target_vy = 0;
              }
              target_vr = 0; 
          }
      }
//			printf("%f %f %f\r\n",target_vx,target_vy,target_vr);
      // 执行底盘解算
      Chassis_Update(target_vx, target_vy, target_vr);
      // ============================================================
      // 3. 机构控制逻辑 (接球 & 发球) - 新增部分
      // ============================================================

      // --- [逻辑 A] 接球 (Button 1) ---
      // 1. 检测上升沿 (当前是1，上次是0)
//      if (local_rc.button1 == 1 && last_button1 == 0) {
//          if (catch_state == ACTION_IDLE) { // 防重入：只有空闲时才触发
//              catch_state = ACTION_STEP_1;
//              catch_start_tick = current_tick;
//          }
//      }
//      last_button1 = local_rc.button1; // 更新历史状态

//      // 2. 接球状态机
//      switch (catch_state) {
//          case ACTION_STEP_1:
//              // 动作：垫球机构 转到 接球角度
//              Mechanism_Cushion_SetAngle(CUSHION_ACTION_DEG, 0.6f); 
//              
//              // 延时：保持 300ms
//              if (current_tick - catch_start_tick > 300) {
//                  catch_state = ACTION_STEP_2;
//              }
//              break;

//          case ACTION_STEP_2:
//              // 动作：垫球机构 复位
//              Mechanism_Cushion_SetAngle(CUSHION_ORIGIN_DEG, 0.6f);
//              
//              // 延时：给 300ms 让它归位，然后结束
//              if (current_tick - catch_start_tick > 600) {
//                  catch_state = ACTION_IDLE;
//              }
//              break;
//              
//          case ACTION_IDLE:
//          default:
//              // 这里不做处理，留给下面的默认逻辑或发球逻辑接管
//              break;
//      }


      // --- [逻辑 B] 发球 (Button 2) ---
      // 1. 检测上升沿
      if (local_rc.button5 == 1 && last_button5 == 0) {
          if (serve_state == ACTION_IDLE) {
              serve_state = ACTION_STEP_1;
              serve_start_tick = current_tick;
          }
      }
      last_button5 = local_rc.button5;

      // 2. 发球状态机
      switch (serve_state) {
          case ACTION_STEP_1:
              // 垫球机构：配合动作 (比如稍微抬起把球送进摩擦轮)
            Mechanism_Cushion_SetAngle(CUSHION_ACTION_DEG, 0.6f);
              
              // 发球电机：开始旋转
              Mechanism_Serve_SetAngle(SERVE_ACTION_RAD); 

              // 延时：300ms 后垫球机构先归位
              if (current_tick - serve_start_tick > 300) {
                  serve_state = ACTION_STEP_2;
              }
              break;

          case ACTION_STEP_2:
              // 垫球机构：复位
              Mechanism_Cushion_SetAngle(CUSHION_ORIGIN_DEG, 0.3f);
              
              // 发球电机：继续保持旋转，凑够总时长
              Mechanism_Serve_SetAngle(SERVE_ACTION_RAD);

              // 延时：总共 800ms 后停止发球电机 (0.8秒 * 15rad/s ≈ 2圈)
              if (current_tick - serve_start_tick > 800) {
                  Mechanism_Serve_SetAngle(SERVE_READY_RAD); // 停转
                  serve_state = ACTION_IDLE;
              }
              break;

          case ACTION_IDLE:
          default:
              break;
      }

      // --- [逻辑 C] 默认保活逻辑 ---
      // 如果两个功能都在空闲状态，强制让垫球机构保持在原点
      // 这是一个很重要的“默认行为”，防止电机没力气软掉
//      if (catch_state == ACTION_IDLE && serve_state == ACTION_IDLE) {
//          Mechanism_Cushion_SetAngle(CUSHION_ORIGIN_DEG, 0.3f); // 给一个小一点的 KP 保持位置
//      }
//4.垫球机角度改变
      if (local_rc.button2 == 1 && last_button_front == 0) {
        PITCH_ANGLE += 15.0f;
        Mechanism_Pitch_SetAngle(PITCH_ANGLE);
      }else if (local_rc.button4 == 1 && last_button_back == 0) {
        PITCH_ANGLE -= 15.0f;
        Mechanism_Pitch_SetAngle(PITCH_ANGLE);
      }
      last_button_front = local_rc.button2;
        last_button_back = local_rc.button4;

      // ============================================================
      // 4. 任务调度延时
      // ============================================================
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
// 数据上报任务
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  FDCAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[12];
  
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
      // [新增] 临界区保护，确保发送的数据是一组完整的
      float temp_x, temp_y, temp_angle;
      
      taskENTER_CRITICAL();
      temp_x = g_robot_pose.x;
      temp_y = g_robot_pose.y;
      temp_angle = g_robot_pose.angle;
      taskEXIT_CRITICAL();

      memcpy(&TxData[0], &temp_x, 4);
      memcpy(&TxData[4], &temp_y, 4);
      memcpy(&TxData[8], &temp_angle, 4);

      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
      osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief 指令解析任务
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  UartRxMessage_t rx_msg;
  remote_engineer_t temp_rc; 

  /* Infinite loop */
  for(;;)
  {
      if (osMessageQueueGet(remote_queueHandle, &rx_msg, NULL, osWaitForever) == osOK)
      {
          Command_Write(rx_msg.data, rx_msg.size);
          while (Command_GetCommand(processsed_command) != 0)
          {
              code_unzipread(processsed_command);      
              Remote_Data_Convert(&rc, &temp_rc);      			
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
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
    #define CUSHION_READY_DEG   37.0f 
    #define CUSHION_SPEED       15.0f
    #define ACTION_HOLD_MS      500
    const float CUSHION_ACTION_DEG = 114.0f;

    // === 新增变量：记录上次发送“保持指令”的时间 ===
	osDelay(1000);
    // 上电初始化，击球板放在最低位。
    Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, CUSHION_SPEED);
    printf("System Ready. Angle reset.\r\n");

  for(;;)
  {
    // 1. 检测光电门
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET)
    {
        osDelay(10); // 消抖
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET)
        {
            // === 有球！执行击球任务 ===
            printf("Ball detected! Action!\r\n");

            // 击球
            Mechanism_Cushion_SetAngle(CUSHION_ACTION_DEG, CUSHION_SPEED);
            osDelay(ACTION_HOLD_MS);

            // 复位（注意：这里建议用正常速度 CUSHION_SPEED，保证回位有力）
            Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, 0.3f);
            
            // 等待球离开
            while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET)
            {
                osDelay(10);
            }
            
            osDelay(500); // 冷却
            printf("Ready.\r\n");

       
        }
    }
    
    // 空闲延时
    osDelay(5);
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// -------------------------------------------------------------------------
// 串口中断回调
// -------------------------------------------------------------------------
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    
    if (huart->Instance == USART2) { 
        UartRxMessage_t rx_msg;
        // 限制拷贝大小，防止 msg 结构体溢出
        uint16_t copy_size = (Size < sizeof(rx_msg.data)) ? Size : sizeof(rx_msg.data);
        
        memcpy(rx_msg.data, remote_Buffer, copy_size);
        rx_msg.size = copy_size;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        xQueueSendFromISR(remote_queueHandle, &rx_msg, &xHigherPriorityTaskWoken);
        // [修改] 重新开启 DMA，使用定义好的 Buffer 大小
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_Buffer, RC_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); 

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// 串口错误回调：防死机
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_ORE | UART_FLAG_NE);
        // [修改] 重新开启 DMA，使用定义好的 Buffer 大小
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_Buffer, RC_BUFFER_SIZE);
    }
}
/* USER CODE END Application */

