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

// --- �弶֧�ְ� (BSP) ���Ӳ������ģ��? ---
#include "fdcan_bsp.h"      // CAN��������
#include "robot_data.h"     // ȫ�ֻ��������ݽṹ��
#include "chassis_task.h"   // ���̿�������
#include "mechanism_task.h" // ��е�����������񣨻���/����ȣ�?
#include "usart.h" 

// --- ң����Э�鴦����ָ�����? ---
#include "Task_command.h"   // �ṩָ�������غ���
#include "remote_driver.h"  // �ṩң����ԭʼ���ݽṹ�嶨��
#include "queue.h"          // FreeRTOS����֧��
#include "semphr.h"         // FreeRTOS�ź���/������֧��
#include "chassis_path_task.h" // ·���滮����
#include "Pan_Tilt_control.h"  // ��̨����

// �ⲿӲ���������?
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern UART_HandleTypeDef huart10; 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifndef M_PI
#define M_PI 3.1415926535f  // Բ���ʶ���
#endif

// ����������������������?����
#define power_add 0.3
// ������ʱ������ms��
#define time_add 1

// --- �������״�?ö�٣��ֲ�ִ�з�������---
typedef enum {
    ACTION_IDLE = 0, // ����״̬���޶�����
    ACTION_STEP_1,   // ִ�н׶�1���������������̧��?
    ACTION_STEP_2,    // ִ�н׶�2�����������������λ��?
	    ACTION_STEP_3    // ִ�н׶�2�����������������λ��?
} ActionState_e;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RC_BUFFER_SIZE  64  // ң�������ڽ��ջ�������С
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
float time_ms = 1;                  // ������ʱ����ֵ��ms��
float PITCH = 4.06f;                // Pitch�Ỻ������?���ٶ�

// --- FreeRTOS �ں˶����� ---
osMutexId_t rc_mutexHandle;            // ����ȫ��ң�������� g_remote_data �Ļ�����
osMessageQueueId_t remote_queueHandle; // ��Ϣ���У����ڽ��մ����жϵ�ԭʼң��������

// ��������������
const osMutexAttr_t rc_mutex_attributes = {
  .name = "rcMutex"
};

// ����������ź�������⵽��ʱ������
osSemaphoreId_t BinarySem_BallDetectHandle;
const osSemaphoreAttr_t BinarySem_BallDetect_attributes = {
  .name = "BinarySem_BallDetect"
};

// --- ȫ��ҵ�����? ---
remote_engineer_t g_remote_data = {0}; // ȫ��ң�������ݣ�������
uint8_t remote_Buffer[RC_BUFFER_SIZE]; // ң����DMA����ԭʼ���ݻ�����
uint8_t processsed_command[COMMAND_LENGTH]; // ָ�����������ݻ�����

// �ⲿ������ң����ԭʼ���ݽṹ��
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
  // 1. Ӳ��������ʼ�����ײ����裩
  fdcan_bsp_init();     // CAN���ߵײ��ʼ��?
  Robot_Data_Init();    // ȫ�ֻ��������ݽṹ���ʼ��?
  Chassis_Init();       // ���̿��Ƴ�ʼ��
  Mechanism_Init();     // ��е������ʼ��������/����ȣ�?

  HAL_Delay(100); // Ӳ���ȶ���ʱ

  // 2. ����CAN����ͨ��
  fdcan_bsp_start(&hfdcan1); 
  fdcan_bsp_start(&hfdcan2); 
  fdcan_bsp_start(&hfdcan3); 

  // 3. ����������������ң�������ݣ�
  rc_mutexHandle = osMutexNew(&rc_mutex_attributes);

  // 4. ����ң����������Ϣ���У����г���16��ÿ��Ԫ��ΪUartRxMessage_t���ͣ�
  remote_queueHandle = osMessageQueueNew(16, sizeof(UartRxMessage_t), NULL);
	
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* �������������? */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  // ��������������ź�������ʼ�?1��������1��
  BinarySem_BallDetectHandle = osSemaphoreNew(1, 1, &BinarySem_BallDetect_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* ��ʱ�����������ޣ� */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* ���д������? */
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
  /* ���񴴽����? */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* �¼���־�飨���ޣ� */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Ĭ��������������£�?1ms���ڣ�
  * @param  argument: δʹ��
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* ����ѭ�� */
  for(;;)
  {
    Update_Virtual_Axis(); // �������������ݣ�ң��������ӳ�䣩
    osDelay(1);            // ��ʱ1ms��Լ1000Hzִ��Ƶ�ʣ�
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief  ���Ŀ������񣺵���+����������ƣ�?10ms���ڣ�
* @param  argument: δʹ��
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10msִ�����ڣ�100Hz��
  xLastWakeTime = osKernelGetTickCount();

  // --- 1. �ֲ�������ʼ�� ---
  remote_engineer_t local_rc = {0}; // ����ң�������ݣ�����Ƶ������ȫ�ֱ�����
  float target_vx = 0, target_vy = 0, target_vr = 0; // ����Ŀ���ٶȣ�x/y��ƽ�ƣ�z����ת��
  
  // --- 2. ���������������? ---
  const float CUSHION_READY_DEG = 51.0f;   // ���������ʼ�Ƕȣ���?
  const float CUSHION_ACTION_DEG = 124.0f;// ������������Ƕȣ���?
  const float SERVE_READY_RAD    = 23.0f; // ���������ʼ�Ƕȣ���?
  const float SERVE_ACTION_RAD   = 210.0f;// ������������Ƕȣ���?
	float wheel_r=0.046f;
  float PITCH_ANGLE = 0.0f;               // Pitch��Ŀ��Ƕ�?
int bang_flag=1;
	
  // ����״̬������
  ActionState_e serve_state = ACTION_IDLE;
  uint32_t serve_start_tick = 0;          // ��������ʼʱ���?
  
  // ����������ʱ�������е�����󴥷���?
  #define DEBOUNCE_DELAY_MS 10

  // ����״̬��ʷ����¼��һ֡״̬�������ش�����
  static uint32_t last_time_btn5 = 0; // ����5�ϴδ���ʱ��
  static uint32_t last_time_btn2 = 0; // ����2�ϴδ���ʱ��
  static uint32_t last_time_btn4 = 0; // ����4�ϴδ���ʱ��
  static uint32_t last_time_btn1 = 0; // ����1�ϴδ���ʱ��
  static uint32_t last_time_btn3 = 0; // ����3�ϴδ���ʱ��

  // ������һ֡״̬
  static uint8_t last_button5 = 0;
  static uint8_t last_button2 = 0;
  static uint8_t last_button4 = 0;
  static uint8_t last_button1 = 0;
  static uint8_t last_button3 = 0;
  
  // �Զ�ģʽ����״̬
  static uint8_t last_button_front = 0;
  static uint8_t last_button_back = 0;
  Mechanism_Serve_SetAngle(SERVE_READY_RAD); 
	osDelay(200);
  Mechanism_Serve_SetAngle(SERVE_READY_RAD); 
  /* ����ѭ�� */
  for(;;)
  {
      uint32_t current_tick = HAL_GetTick(); // ��ȡϵͳ��ǰʱ�����ms��

      // ============================================================
      // 1. �̰߳�ȫ��ȡȫ��ң�������ݣ��ӻ�������
      // ============================================================
      if (osMutexAcquire(rc_mutexHandle, 10) == osOK) {
          local_rc = g_remote_data; // ����ȫ�����ݵ�����
          osMutexRelease(rc_mutexHandle); // �ͷŻ�����
      }
      // ============================================================
      // 2. ����ģʽ�л��߼����ֶ�/�Զ�/����λ��
      // ============================================================
      if (local_rc.mode == CHASSIS_MODE_MANUAL) 
      {
          /* --- �ֶ�ģʽ��ң����ֱ�ӿ��Ƶ��� --- */
          target_vx = local_rc.vx / 50.0f*wheel_r;  // X���ٶȣ���һ����-1~1��
          target_vy = local_rc.vy / 50.0f*wheel_r;  // Y���ٶȣ���һ����-1~1��
          target_vr = -local_rc.vw * 3.0f;   // ��ת�ٶȣ��Ŵ�ϵ��3��
          // --- ����5�������������������ش���+������---
          if (local_rc.button5 == 1 && last_button5 == 0) {
              if ((current_tick - last_time_btn5) > DEBOUNCE_DELAY_MS) {
                  if (serve_state == ACTION_IDLE) { // ������ʱ����
                      serve_state = ACTION_STEP_1;
										bang_flag =0;
                      serve_start_tick = current_tick;
                  }
                  last_time_btn5 = current_tick; // �����ϴδ���ʱ��
              }
          }
          last_button5 = local_rc.button5; // ���°���״̬

          // --- ����2/4��Pitch��Ƕȴֵ�����?5�㣩---
          if (local_rc.button2 == 1 && last_button2 == 0) {
              if ((current_tick - last_time_btn2) > DEBOUNCE_DELAY_MS) {
                  PITCH_ANGLE += 5.0f;
                  Mechanism_Dian_Pitch_SetAngle(PITCH_ANGLE); // ����Pitch�Ƕ�
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

          // --- ����1/3��Pitch���ٶ�΢������0.3��---
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
          // ���µ����ٶ�ָ��
					if(bang_flag)
					{
          Chassis_Update(target_vx, target_vy, target_vr);
      }
		}
      else if (local_rc.mode == CHASSIS_MODE_AUTO)
      {
          /* --- �Զ�ģʽ��·���滮���Ƶ��� --- */
          float cur_x, cur_y, cur_yaw;    // �����˵�ǰλ�ˣ�x/y���꣬ƫ���ǣ�
          uint32_t last_update;           // λ��������ʱ��
          
          // �ٽ�����ȡȫ��λ�ˣ�������̳߳�ͻ��?
          taskENTER_CRITICAL();
          cur_x = g_robot_pose.x;
          cur_y = g_robot_pose.y;
          cur_yaw = g_robot_pose.angle;
          last_update = g_robot_pose.update_tick;
          taskEXIT_CRITICAL();

          // λ�����ݳ�ʱ����������1sδ������ֹͣ���̣�
          if (HAL_GetTick() - last_update > 1000) {
               target_vx = 0; target_vy = 0; target_vr = 0;
          }
          else 
          {
              // ����Ŀ��λ��ƫ��
              float err_x = g_robot_target.target_x - cur_x;
              float err_y = g_robot_target.target_y - cur_y;
              
              // ·���滮��ʼ�������״�ִ�У�
              static uint8_t initialized = 0;
              if (!initialized) {
                  Planner_SetTarget(cur_x, cur_y, err_x, err_y, 0.0f); 
                  initialized = 1;
              }

              // ����·���滮����ȡ�ٶ�ָ��
              TrajVel_t cmd = Planner_Update(cur_x, cur_y, cur_yaw);
              
              // ����任�����������? �� �����˱�������ϵ
              float theta = cur_yaw * (M_PI / 180.0f); // �Ƕ�ת����
              target_vx =  cmd.vx * cosf(theta) + cmd.vy * sinf(theta);
              target_vy = -cmd.vx * sinf(theta) + cmd.vy * cosf(theta);
              target_vr = cmd.vr;
              
              // ����Ŀ��λ�˺����ù滮��
              if (Planner_IsArrived(cur_x, cur_y, cur_yaw)) {
                  initialized = 0;
              }
          }
          Chassis_Update(target_vx, target_vy, target_vr);
      }
      else if (local_rc.mode == CHASSIS_MODE_SERVE)
      {
          /* --- ����λģʽ�������ƶ���ָ��λ�ú��� --- */
          float cur_x, cur_y, cur_yaw, target_x = 0.0f, target_y = 0.0f;
          uint32_t last_update;
          
          // �ٽ�����ȡ��ǰλ��
          taskENTER_CRITICAL();
          cur_x = g_robot_pose.x;
          cur_y = g_robot_pose.y;
          cur_yaw = g_robot_pose.angle;
          last_update = g_robot_pose.update_tick;
          taskEXIT_CRITICAL();
				
          // ����5����λ��ԭ�㣨0,0��
          if (local_rc.button5 == 1 && last_button5 == 0) {
              target_x = 0.0f;
              target_y = 0.0f;
          }
          last_button5 = local_rc.button5;
				
          // ����2��ǰ��ǰ��λ�ã�800,500��������4��ǰ����λ�ã�1000,1000��
          if (local_rc.button2 == 1 && last_button_front == 0) {
              target_x = 800.0f;
              target_y = 500.0f;
          }else if (local_rc.button4 == 1 && last_button_back == 0) {
              target_x = 1000.0f;
              target_y = 1000.0f;
          }
          last_button_front = local_rc.button2;
          last_button_back = local_rc.button4;

          // λ�����ݳ�ʱ����
          if (HAL_GetTick() - last_update > 1000) {
               target_vx = 0; target_vy = 0; target_vr = 0;
          }
          else 
          {
              // ����Ŀ��ƫ��
              float err_x = target_x - cur_x;
              float err_y = target_y - cur_y;
              
              // ·���滮��ʼ��
              static uint8_t initialized = 0;
              if (!initialized) {
                  Planner_SetTarget(cur_x, cur_y, err_x, err_y, 0.0f);
                  initialized = 1;
              }

              // ����·���滮
              TrajVel_t cmd = Planner_Update(cur_x, cur_y, cur_yaw);
              
              // �����?
              float theta = cur_yaw * (M_PI / 180.0f);
              target_vx =  cmd.vx * cosf(theta) + cmd.vy * sinf(theta);
              target_vy = -cmd.vx * sinf(theta) + cmd.vy * cosf(theta);
              target_vr = cmd.vr;
              
              // ���µ����ٶ�
              Chassis_Update(target_vx, target_vy, target_vr);

              // ����Ŀ��λ�ú��Զ���������
              if (Planner_IsArrived(cur_x, cur_y, cur_yaw)) {
                  if (serve_state == ACTION_IDLE) {
                      serve_state = ACTION_STEP_1;
                      serve_start_tick = current_tick;
                  }
              }
          }
      }

      // ============================================================
      // 3. �������״�?��ִ�У��ֲ�������
      // ============================================================
      switch (serve_state) {
          case ACTION_STEP_1:
              // �׶�1���������̧��? + �����������?
              Mechanism_Cushion_SetAngle(CUSHION_ACTION_DEG, PITCH);
              // ��ʱ300ms�����׶�2
              if (current_tick - serve_start_tick > 100) {
                  serve_state = ACTION_STEP_2;
              }
              break;

          case ACTION_STEP_2:
											if(bang_flag==0){
					Chassis_Update(0,7.0f,0);
           if (current_tick - serve_start_tick > 518) {
              Mechanism_Serve_SetAngle(SERVE_ACTION_RAD); 
						 serve_state=ACTION_STEP_3;
						}
					}
					 break;
						
					case ACTION_STEP_3:
						if(bang_flag==0){
							if(current_tick-serve_start_tick>700)
							{
					Chassis_Update(0,0,0);
						}
					}
              // �׶�2�����������λ�����٣�?+ �����������?
              Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, 0.3f);
										osDelay(5);

              // ����ʱ800ms��λ������״̬
              if (current_tick - serve_start_tick > 1300) {
                  Mechanism_Serve_SetAngle_back(SERVE_READY_RAD); // ����������?
                bang_flag=1;  
								serve_state = ACTION_IDLE;
              }
              break;

          default: break;
      }

      // ��׼��ʱ����֤10ms���ڣ�
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief  CAN2�������񣺻�����λ���ϱ�����Ƶ��
* @param  argument: δʹ��
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  FDCAN_TxHeaderTypeDef TxHeader; // CAN����֡ͷ
  uint8_t TxData[12];             // CAN�������ݣ�12�ֽڣ�
  
  // ��ʼ��CAN����֡ͷ
  TxHeader.Identifier = 0x101;                // ֡ID��0x101
  TxHeader.IdType = FDCAN_STANDARD_ID;        // ��׼ID��11λ��
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;    // ����֡
  TxHeader.DataLength = FDCAN_DLC_BYTES_12;   // ���ݳ��ȣ�12�ֽ�
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // ����״̬������
  TxHeader.BitRateSwitch = FDCAN_BRS_ON;      // �������л�������
  TxHeader.FDFormat = FDCAN_FD_CAN;           // FD CAN��ʽ
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // �����������¼�
  TxHeader.MessageMarker = 0;                 // ��Ϣ��ǣ�?0

  /* ����ѭ�� */
  for(;;)
  {
      float temp_x, temp_y, temp_angle;
      
      // �ٽ�����ȡλ�ˣ�������̳߳�ͻ��?
      taskENTER_CRITICAL();
      temp_x = g_robot_pose.x;
      temp_y = g_robot_pose.y;
      temp_angle = g_robot_pose.angle;
      taskEXIT_CRITICAL();

      // ���ݴ����Float �� Byte��
      memcpy(&TxData[0], &temp_x, 4);     // 0-3�ֽڣ�X���꣨float��
      memcpy(&TxData[4], &temp_y, 4);     // 4-7�ֽڣ�Y���꣨float��
      memcpy(&TxData[8], &temp_angle, 4); // 8-11�ֽڣ�ƫ���ǣ�float��

      // ���͵�CAN2����
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
      osDelay(1); // Լ1000Hz����Ƶ��
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief  ң�������ݽ������񣺴�����DMA���յ�ԭʼ����
* @param  argument: δʹ��
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  // ��ʼ��UART2 DMAѭ�����գ�ReceiveToIdleģʽ�������������жϣ�
  HAL_UARTEx_ReceiveToIdle_DMA(&huart10, remote_Buffer, RC_BUFFER_SIZE);
  // ����DMA�봫���жϣ�������һ֡�������ݣ�
  __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT); 

  UartRxMessage_t rx_msg;        // ���ڽ�����Ϣ�ṹ��
  remote_engineer_t temp_rc;     // ��ʱң��������

  /* ����ѭ�� */
  for(;;)
  {
      // �����ȴ���Ϣ���У����Դ����жϵ�ԭʼ���ݣ�
      if (osMessageQueueGet(remote_queueHandle, &rx_msg, NULL, osWaitForever) == osOK)
      {
          // ��������ԭʼ����Ϊָ����?
          Command_Write(rx_msg.data, rx_msg.size);
          
          // ѭ����������ָ��
          while (Command_GetCommand(processsed_command) != 0)
          {
              code_unzipread(processsed_command);   // ���ң�����?��
              Remote_Data_Convert(&rc, &temp_rc);   // ת��Ϊ��׼���ݸ�ʽ
              
              // �̰߳�ȫ����ȫ��ң��������
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
* @brief  �������񣺼�⵽��󴥷�������
* @param  argument: δʹ��
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  // ���������������?
  #define CUSHION_READY_DEG   51.0f    // ��ʼ�Ƕ�
  #define CUSHION_SPEED      2.06f     // ��ʼ�ٶ�
  #define ACTION_HOLD_MS      800      // ��������ʱ�䣨ms��
  const float CUSHION_ACTION_DEG = 124.0f; // �����Ƕ�

  osDelay(1000); // ϵͳ������ʱ���ȴ�Ӳ���ȶ���
  
  // ��ʼ�������������ʼλ�ã��?������ȷ����λ��
  Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, CUSHION_SPEED);
  osDelay(500);
  Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, CUSHION_SPEED);
  printf("System Ready. Angle reset.\r\n"); // ϵͳ������ʾ

  /* ����ѭ�� */
  for(;;)
  {
    // 1. ��ȡ���������źţ��͵�ƽ��ʾ��⵽��?
    if (HAL_GPIO_ReadPin(PHOTO_GATE_GPIO_Port, PHOTO_GATE_Pin) == GPIO_PIN_RESET)
    {
        osDelay(5); // Ӳ��������5ms��
        if (HAL_GPIO_ReadPin(PHOTO_GATE_GPIO_Port, PHOTO_GATE_Pin) == GPIO_PIN_RESET)
        {
            // ��ʱ���������⸺����
            if(time_ms < 0)
            {
                time_ms = 0;
            }
            osDelay(time_ms); // ������ʱ�������ã�
            
            // === ��⵽��ִ�з�����? ===
            printf("Ball detected! Action!\r\n");

            // �������̧��?
            Mechanism_Cushion_SetAngle(CUSHION_ACTION_DEG, PITCH);
            osDelay(ACTION_HOLD_MS); // ���ֶ���ȷ���������?

            // ����������?
            Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, 0.3f);
            
            // �ȴ����뿪����ţ������ظ�������?
            while (HAL_GPIO_ReadPin(PHOTO_GATE_GPIO_Port, PHOTO_GATE_Pin) == GPIO_PIN_RESET)
            {
                osDelay(10);
            }
            
            osDelay(500); // ��λ���ȶ���ʱ
            printf("Ready.\r\n"); // ������ʾ
        }
    }
    // ������ڣ�?500ms
    osDelay(500);
  }
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief  ��е����ѭ������1ms���ڸ��»���״̬
* @param  argument: δʹ��
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* ����ѭ�� */
  for(;;)
  {
    Mechanism_Loop_1ms(); // ��е����1ms���ڴ����ջ�����/״̬���£�
    osDelay(1);           // 1ms��ʱ
  }
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief  CAN3����������̨�Ƕ��ϱ���1ms���ڣ�
* @param  argument: δʹ��
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
  FDCAN_TxHeaderTypeDef TxHeader; // CAN����֡ͷ
  uint8_t TxData[12];             // �������ݻ�����
  
  // ��ʼ��CAN����֡ͷ
  TxHeader.Identifier = CAN_ID_PC_FEEDBACK; // ֡ID��0x300��PC����ר�ã�
  TxHeader.IdType = FDCAN_STANDARD_ID;      // ��׼ID
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;  // ����֡
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;  // ���ݳ��ȣ�8�ֽ�
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;   // �رղ������л�����ͳCAN��
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;    // ��ͳCAN��ʽ
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
  
  // ��׼��ʱ��ʼ����1ms���ڣ�
  TickType_t xLastWakeTime = osKernelGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1msִ������

  /* ����ѭ�� */
  for(;;)
  {
      uint32_t current_timestamp = HAL_GetTick(); // ��ǰʱ�����ms��
      float yaw_angle = 0.0f;                     // ��̨Yaw��Ƕ�?
      float pitch_angle = 0.0f;                   // ��̨Pitch��Ƕ�?
      
      // ��ȡ��̨ʵʱ�Ƕ�
      gimbal_get_angles(&yaw_angle, &pitch_angle);

      // ����ѹ����������*100תΪint16_t����ʡ��������2λС����
      int16_t yaw_send   = (int16_t)(yaw_angle * 100.0f);
      int16_t pitch_send = (int16_t)(pitch_angle * 100.0f);

      // ���ݴ��?
      memcpy(&TxData[0], &current_timestamp, 4); // 0-3�ֽڣ�ʱ�����uint32_t��
      memcpy(&TxData[4], &yaw_send, 2);          // 4-5�ֽڣ�Yaw�Ƕȣ�int16_t��
      memcpy(&TxData[6], &pitch_send, 2);        // 6-7�ֽڣ�Pitch�Ƕȣ�int16_t��

      // ���͵�CAN3����
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData);

      // ��׼��ʱ����֤1ms���ڣ�
      vTaskDelayUntil(&xLastWakeTime, xFrequency);  
  }
  /* USER CODE END StartTask07 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* -------------------------------------------------------------------------
// ����DMA��������жϻص�����������ң�����?ʼ���ݣ�
// ------------------------------------------------------------------------- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    
    if (huart->Instance == USART10) { // ������USART10��ң�������ڣ�
        UartRxMessage_t rx_msg;
        // ���ݳ��ȱ��������⻺���������?
        uint16_t copy_size = (Size < sizeof(rx_msg.data)) ? Size : sizeof(rx_msg.data);
        
        // �����������ݵ���Ϣ�ṹ��
        memcpy(rx_msg.data, remote_Buffer, copy_size);
        rx_msg.size = copy_size;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // �ж��з�����Ϣ�����У���������
        xQueueSendFromISR(remote_queueHandle, &rx_msg, &xHigherPriorityTaskWoken);
        
        // ����DMA���գ�ѭ�����գ�
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_Buffer, RC_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); 

        // ����������ȣ������Ҫ��
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
  * @brief  ���ڴ���ص��������ݴ���?
  * @param  huart: ���ھ��?
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART10) {
        // ��������־����żУ��/֡����/���?/������
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_ORE | UART_FLAG_NE);
        // ����DMA���գ��ָ�ͨ�ţ�
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_Buffer, RC_BUFFER_SIZE);
    }
}
/* USER CODE END Application */

