/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * 鏂囦欢鍚?         : app_freertos.c
  * 鎻忚堪            : FreeRTOS 搴旂敤绋嬪簭浠ｇ爜锛屽寘鍚换鍔¤皟搴︿笌閫昏緫鎺у埗
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

// --- 鏉跨骇鏀寔鍖?(BSP) 鍜?鏈哄櫒浜哄姛鑳芥ā鍧?---
#include "fdcan_bsp.h"      // CAN椹卞姩
#include "robot_data.h"     // 鍏ㄥ眬鏈哄櫒浜烘暟鎹粨鏋?
#include "chassis_task.h"   // 搴曠洏搴曞眰鎺у埗
#include "mechanism_task.h" // 鏈烘瀯锛堟嫧鏉嗐€佸彂鐞冪瓑锛夋帶鍒?
#include "usart.h" 

// --- 閬ユ帶鍣ㄥ崗璁鐞嗕笌鎸囦护瑙ｆ瀽 ---
#include "Task_command.h"   // 鎻愪緵鎸囦护鍐欏叆涓庤幏鍙栨帴鍙?
#include "remote_driver.h"  // 鎻愪緵閬ユ帶鍣ㄥ師濮嬭В绠楁暟鎹粨鏋?
#include "queue.h"          // 闃熷垪鏀寔
#include "semphr.h"         // 淇″彿閲?浜掓枼閲忔敮鎸?
#include "chassis_path_task.h" // 璺緞瑙勫垝鐩稿叧
#include "Pan_Tilt_control.h"
#include "cybergear_motor.h"
#include "trajectory_planner.h"
#include "chassis_cybergear.h"
// 澶栭儴寮曠敤纭欢鍙ユ焺
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern UART_HandleTypeDef huart2; 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifndef M_PI
#define M_PI 3.1415926535f
#endif
#define power_add 0.3
#define time_add 1
#define begin 1
#define finish 0
#define TASK03_CAN_PERIOD_MS 20U
#define TASK07_CAN_PERIOD_MS 5U
static uint8_t s_cybergear_boot_ok = 0U;
static uint8_t s_cybergear_chassis_ready = 0U;

uint64_t auto_state = 0;

	float time_ms = 1;
  float PITCH = 4.06f;                 // Pitch 杞村姏搴︽帶鍒跺彉閲?

// --- 鍔ㄤ綔搴忓垪鐘舵€佹満鏋氫妇锛氱敤浜庡鐞嗛渶瑕佸欢鏃堕厤鍚堢殑鍔ㄤ綔 ---
typedef enum {
    ACTION_IDLE = 0, // 绌洪棽鐘舵€?
    ACTION_STEP_1,   // 鍔ㄤ綔鎵ц闃舵 1 (濡傦細鏈烘瀯浼稿嚭)
    ACTION_STEP_2    // 鍔ㄤ綔鎵ц闃舵 2 (濡傦細鏈烘瀯澶嶄綅)
} ActionState_e;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RC_BUFFER_SIZE  64  // 涓插彛鎺ユ敹缂撳啿鍖哄ぇ灏?
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// --- FreeRTOS 鎿嶄綔绯荤粺瀵硅薄 ---
typedef struct
{
  uint32_t sample_tick;
  uint16_t defaultTask_min_words;
  uint16_t task02_min_words;
  uint16_t task03_min_words;
  uint16_t task04_min_words;
  uint16_t task05_min_words;
  uint16_t task06_min_words;
  uint16_t task07_min_words;
} FreertosStackTelemetry_t;

osMutexId_t rc_mutexHandle;            // 淇濇姢鍏ㄥ眬閬ユ帶鍣ㄥ彉閲?g_remote_data 鐨勪簰鏂ラ攣
osMessageQueueId_t remote_queueHandle; // 娑堟伅闃熷垪锛氱敤浜庝覆鍙ｄ腑鏂皢鍘熷鏁版嵁浼犵粰瑙ｆ瀽浠诲姟

const osMutexAttr_t rc_mutex_attributes = {
  .name = "rcMutex"
};

// --- 鍏ㄥ眬涓氬姟鍙橀噺 ---
remote_engineer_t g_remote_data = {0}; // 鏈€缁堣В绠楀嚭鐨勯仴鎺у櫒缁撴瀯浣?
uint8_t remote_Buffer[RC_BUFFER_SIZE]; // 涓插彛 DMA 鎺ユ敹鍘熷鏁版嵁鐨勫唴瀛樼┖闂?
uint8_t processsed_command[COMMAND_LENGTH]; // 鎸囦护瑙ｆ瀽涓存椂缂撳啿鍖?
volatile FreertosStackTelemetry_t g_freertos_stack_telemetry = {0};
volatile const char* g_freertos_fault_task = NULL;
volatile uint32_t g_freertos_fault_free_heap = 0U;
volatile uint32_t g_can3_tx_drop_task03 = 0U;
volatile uint32_t g_can3_tx_drop_task07 = 0U;

// 澶栭儴寮曠敤鍘熷閬ユ帶鍣ㄦ暟鎹粨鏋?
extern rc_info_t rc; 

/* 浠诲姟鍙ユ焺瀹氫箟 */
osThreadId_t defaultTaskHandle;
osThreadId_t myTask02Handle; // 涓绘帶浠诲姟锛氬簳鐩?+ 鍔ㄤ綔閫昏緫
osThreadId_t myTask03Handle; // 閫氳浠诲姟锛氫綅缃笂鎶?
osThreadId_t myTask04Handle; // 瑙ｆ瀽浠诲姟锛氶仴鎺у櫒鍗忚瑙ｇ畻
osThreadId_t myTask05Handle; // 鑷姩閫昏緫浠诲姟锛氬厜鐢甸棬瑙﹀彂

/* 浠诲姟灞炴€ч厤缃?*/

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 768 * 4
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 384 * 4
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 768 * 4
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 768 * 4
};
/* Definitions for myTask06 */
osThreadId_t myTask06Handle;
const osThreadAttr_t myTask06_attributes = {
  .name = "myTask06",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for myTask07 */
osThreadId_t myTask07Handle;
const osThreadAttr_t myTask07_attributes = {
  .name = "myTask07",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for BinarySem_BallDetect */
osSemaphoreId_t BinarySem_BallDetectHandle;
const osSemaphoreAttr_t BinarySem_BallDetect_attributes = {
  .name = "BinarySem_BallDetect"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
static uint16_t prv_stack_words_clamp(UBaseType_t words);
static void prv_update_stack_telemetry(void);

void MX_FREERTOS_Init(void); 
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
  
  // 1. 鍒濆鍖栫‖浠舵帶鍒剁浉鍏虫暟鎹笌搴曞眰椹卞姩
  fdcan_bsp_init();     // CAN鎬荤嚎鍒濆鍖?
    cybergear_motors_init();
   Robot_Data_Init();    // 鍏ㄥ眬鏁版嵁缁撴瀯鍒濆鍖?
   Chassis_Init();       // 搴曠洏鍙傛暟鍒濆鍖?
   Mechanism_Init();     // 鏈烘瀯鍙傛暟鍒濆鍖?

osDelay(100);

  // 2. 鍚姩鎵€鏈?CAN 閫氳鍙?
  fdcan_bsp_start(&hfdcan1); 
  fdcan_bsp_start(&hfdcan2); 
  fdcan_bsp_start(&hfdcan3);

    if (trajectory_planner_init() == 0)
    {
        s_cybergear_boot_ok = 1U;
    }

  // 3. 鍒涘缓浜掓枼閿?(鐢ㄤ簬绾跨▼瀹夊叏)
  rc_mutexHandle = osMutexNew(&rc_mutex_attributes);

  // 4. 鍒涘缓鎸囦护娑堟伅闃熷垪
  remote_queueHandle = osMessageQueueNew(16, sizeof(UartRxMessage_t), NULL);
	
  // 5. 寮€鍚?UART2 DMA 寰幆鎺ユ敹锛屾娴嬬┖闂蹭腑鏂?(ReceiveToIdle)
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, remote_Buffer, RC_BUFFER_SIZE);
  // 绂佺敤鍗婁紶杈撲腑鏂紝鍙鐞嗕竴甯у畬鎴愬悗鐨勭┖闂蹭腑鏂?
  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT); 

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
  uint32_t last_stack_check_tick = HAL_GetTick();

  for(;;)
  {
      if (s_cybergear_boot_ok == 1U && s_cybergear_chassis_ready == 0U)
      {
          if (chassis_cybergear_init(0U) == 0)
          {
              s_cybergear_chassis_ready = 1U;
          }
      }
    if ((HAL_GetTick() - last_stack_check_tick) >= 1000U)
    {
      prv_update_stack_telemetry();
      last_stack_check_tick = HAL_GetTick();
    }

    Update_Virtual_Axis();
//		unitree_update();
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
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms 鎺у埗鍛ㄦ湡
  xLastWakeTime = osKernelGetTickCount();

  // --- 1. 灞€閮ㄦ帶鍒跺彉閲忓垵濮嬪寲 ---
  remote_engineer_t local_rc = {0}; 
  float target_vx = 0, target_vy = 0, target_vr = 0;
  
  // --- 2. 鏈烘瀯鍔ㄤ綔鍙傛暟閰嶇疆 (瑙掑害/閫熷害) ---
  const float CUSHION_READY_DEG = 47.0f;   // 鍨悆鏈烘瀯澶嶄綅瑙掑害 (deg)
  const float CUSHION_ACTION_DEG = 114.0f;  // 鍨悆鏈烘瀯鍔ㄤ綔瑙掑害 (deg)
  const float SERVE_READY_RAD    = 30.0f;   // 鍙戠悆鐢垫満澶嶄綅杞€?
  const float SERVE_ACTION_RAD   = 210.0f;  // 鍙戠悆鐢垫満鍔ㄤ綔杞€?
  float PITCH_ANGLE = 0.0f;                 // Pitch 杞磋搴︽帶鍒跺彉閲?
	
	// 瀹氫箟娑堟姈鏃堕棿闃堝€?(閫氬父 20ms - 50ms)
#define DEBOUNCE_DELAY_MS 10

// 瀹氫箟闈欐€佸彉閲忎繚瀛樹笂涓€娆¤Е鍙戠殑鏃堕棿 (涔熷彲浠ユ斁鍦ㄥ叏灞€缁撴瀯浣撲腑)
static uint32_t last_time_btn5 = 0;
static uint32_t last_time_btn2 = 0;
static uint32_t last_time_btn4 = 0;
static uint32_t last_time_btn1 = 0;
static uint32_t last_time_btn3 = 0;

// 瀹氫箟鐙珛鐨勬寜閿姸鎬佸巻鍙插彉閲?(淇鍘熶唬鐮佸彉閲忓鐢ㄧ殑Bug)
static uint8_t last_button5 = 0;
static uint8_t last_button2 = 0;
static uint8_t last_button4 = 0;
static uint8_t last_button1 = 0;
static uint8_t last_button3 = 0;


  // 鍙戠悆鍔ㄤ綔鐘舵€佹満鐩稿叧鍙橀噺
  ActionState_e serve_state = ACTION_IDLE;
  uint32_t serve_start_tick = 0;

  // 鍨悆鏈烘瀯璋冭妭鎸夐敭鍘嗗彶璁板綍 (鐢ㄤ簬杈规部妫€娴?
  uint8_t last_button_front = 0;
  uint8_t last_button_back = 0;

  /* 浠诲姟涓诲惊鐜?*/
  for(;;)
  {
      uint32_t current_tick = HAL_GetTick(); // 鑾峰彇褰撳墠绯荤粺鏃堕棿

      // ============================================================
      // 1. 瀹夊叏鍦颁粠鍏ㄥ眬鍙橀噺鑾峰彇鏈€鏂扮殑閬ユ帶鍣ㄦ暟鎹?(鍔犻攣淇濇姢)
      // ============================================================
      if (osMutexAcquire(rc_mutexHandle, 10) == osOK) {
          local_rc = g_remote_data; 
          osMutexRelease(rc_mutexHandle);
      }

      // ============================================================
      // 2. 搴曠洏杩愬姩鎺у埗閫昏緫 (鍒嗘ā寮忓鐞?
      // ============================================================
      if (local_rc.mode == CHASSIS_MODE_MANUAL) 
      {
          /* --- 鎵嬪姩妯″紡 --- */
          target_vx = local_rc.vx / 100.0f;  // 宸﹀彸骞崇Щ
          target_vy = local_rc.vy / 100.0f;  // 鍓嶅悗骞崇Щ
          target_vr = -local_rc.vw * 3.0f;   // 鑷棆


// --- Button 5 澶勭悊 ---
if (local_rc.button5 == 1 && last_button5 == 0) {
    // 妫€鏌ュ綋鍓嶆椂闂翠笌涓婃瑙﹀彂鏃堕棿鐨勫樊鍊兼槸鍚﹀ぇ浜庢秷鎶栭槇鍊?
    if ((current_tick - last_time_btn5) > DEBOUNCE_DELAY_MS) {
        if (serve_state == ACTION_IDLE) {
            serve_state = ACTION_STEP_1;
            serve_start_tick = current_tick;
        }
        last_time_btn5 = current_tick; // 鏇存柊瑙﹀彂鏃堕棿
    }
}
last_button5 = local_rc.button5;

// --- Button 2 & 4 澶勭悊 (Pitch 瑙掑害璋冭妭) ---
// Button 2
if (local_rc.button2 == 1 && last_button2 == 0) {
    if ((current_tick - last_time_btn2) > DEBOUNCE_DELAY_MS) {
        PITCH_ANGLE += 5.0f;
        Mechanism_Dian_Pitch_SetAngle(PITCH_ANGLE);
        last_time_btn2 = current_tick;
    }
}
last_button2 = local_rc.button2; // 鐙珛淇濆瓨鐘舵€?

// Button 4
if (local_rc.button4 == 1 && last_button4 == 0) {
    if ((current_tick - last_time_btn4) > DEBOUNCE_DELAY_MS) {
        PITCH_ANGLE -= 5.0f;
        Mechanism_Dian_Pitch_SetAngle(PITCH_ANGLE);
        last_time_btn4 = current_tick;
    }
}
last_button4 = local_rc.button4; // 鐙珛淇濆瓨鐘舵€?

// --- Button 1 & 3 澶勭悊 (PITCH 寰皟) ---
// Button 1
if (local_rc.button1 == 1 && last_button1 == 0) {
    if ((current_tick - last_time_btn1) > DEBOUNCE_DELAY_MS) {
        PITCH += power_add;
//        time_ms += time_add;
        last_time_btn1 = current_tick;
    }
}
last_button1 = local_rc.button1; // 鐙珛淇濆瓨鐘舵€?

// Button 3
if (local_rc.button3 == 1 && last_button3 == 0) {
    if ((current_tick - last_time_btn3) > DEBOUNCE_DELAY_MS) {
        PITCH -= power_add;
//        time_ms -= time_add;
        last_time_btn3 = current_tick;
    }
}
last_button3 = local_rc.button3; // 鐙珛淇濆瓨鐘舵€?

          // 鎵ц搴曠洏椹卞姩鏇存柊
           Chassis_Update(target_vx, target_vy, target_vr);
      }
      else if (local_rc.mode == CHASSIS_MODE_AUTO)
      {
				
					if (local_rc.button5 == 1 && last_button5 == 0) {
				// 妫€鏌ュ綋鍓嶆椂闂翠笌涓婃瑙﹀彂鏃堕棿鐨勫樊鍊兼槸鍚﹀ぇ浜庢秷鎶栭槇鍊?
				if ((current_tick - last_time_btn5) > DEBOUNCE_DELAY_MS) {
					if(auto_state ==finish)
					{
						auto_state = begin;
					}else if (auto_state==begin)
					{
						auto_state = finish;
					}
							last_time_btn5 = current_tick; // 鏇存柊瑙﹀彂鏃堕棿
			}
}
last_button5 = local_rc.button5;

          /* --- 鍏ㄨ嚜鍔ㄦā寮?(璺緞瑙勫垝) --- */
          float cur_x, cur_y, cur_yaw;
          uint32_t last_update;
          
          // 杩涘叆涓寸晫鍖猴紝璇诲彇鍏ㄥ眬瀹氫綅鍧愭爣锛岄槻姝㈡暟鎹涓柇淇敼瀵艰嚧涓嶄竴鑷?
          taskENTER_CRITICAL();
          cur_x = g_robot_pose.x;
          cur_y = g_robot_pose.y;
          cur_yaw = g_robot_pose.angle;
          last_update = g_robot_pose.update_tick;
          taskEXIT_CRITICAL();

          // 瓒呮椂淇濇姢锛氬鏋滃畾浣嶅潗鏍囬暱鏃堕棿涓嶆洿鏂帮紝鍋滄杩愬姩
          if (HAL_GetTick() - last_update > 1000) {
               target_vx = 0; target_vy = 0; target_vr = 0;
          }
          else 
          {
              float err_x = g_robot_target.target_x - cur_x;
              float err_y = g_robot_target.target_y - cur_y;
              
              static uint8_t initialized = 0;
              if (!initialized) {
                  // 鍒濆鍖栬鍒掑櫒鐩爣
                  Planner_SetTarget(cur_x, cur_y, err_x, err_y, 0.0f); 
                  initialized = 1;
              }

              // 鑾峰彇瑙勫垝濂界殑涓栫晫绯婚€熷害
              TrajVel_t cmd = Planner_Update(cur_x, cur_y, cur_yaw);
              
              // 涓栫晫鍧愭爣绯婚€熷害杞负杞︿綋绯婚€熷害
              float theta = cur_yaw * (M_PI / 180.0f);
              target_vx =  cmd.vx * cosf(theta) + cmd.vy * sinf(theta);
              target_vy = -cmd.vx * sinf(theta) + cmd.vy * cosf(theta);
              target_vr = cmd.vr;
              if (Planner_IsArrived(cur_x, cur_y, cur_yaw)) {
								initialized = 0;//閲嶅惎瀹氫綅鍑芥暟
							}
						}
          Chassis_Update(target_vx, target_vy, target_vr);
      }
      else if (local_rc.mode == CHASSIS_MODE_SERVE)
      {
          /* --- 鑷姩瀵逛綅鍙戠悆妯″紡 --- */
          float cur_x, cur_y, cur_yaw, target_x = 0.0f, target_y = 0.0f;
          uint32_t last_update;
          
          taskENTER_CRITICAL();
          cur_x = g_robot_pose.x;
          cur_y = g_robot_pose.y;
          cur_yaw = g_robot_pose.angle;
          last_update = g_robot_pose.update_tick;
          taskEXIT_CRITICAL();
				
				//澶嶄綅鎸夐敭
          if (local_rc.button5 == 1 && last_button5 == 0) {
						target_x = 0.0f;
						target_y = 0.0f;
          }
          last_button5 = local_rc.button5;
				
					//4閿悗鍦哄墠鍙戠悆锛?閿悗鍦哄悗鍙戠悆
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

              // 濡傛灉鍒拌揪棰勫畾鍙戠悆鐐癸紝瑙﹀彂鍙戠悆鏈烘瀯
              if (Planner_IsArrived(cur_x, cur_y, cur_yaw)) {
                  if (serve_state == ACTION_IDLE) {
                      serve_state = ACTION_STEP_1;
                      serve_start_tick = current_tick;
                  }
              }
          }
      }

      // ============================================================
      // 3. 鏈烘瀯鎺у埗閫昏緫锛氬彂鐞冪姸鎬佹満鎵ц (寮傛闈為樆濉?
      // ============================================================
      switch (serve_state) {
          case ACTION_STEP_1:
              // 鍔ㄤ綔锛氬灚鐞冩満鏋勯厤鍚堟姮璧凤紝鍙戠悆鐢垫満杞姩
								Mechanism_Cushion_SetAngle(CUSHION_ACTION_DEG, PITCH);
								Mechanism_Serve_SetAngle(SERVE_ACTION_RAD); 

              // 寤舵椂 300ms 鍚庡垏鎹㈠埌澶嶄綅闃舵
              if (current_tick - serve_start_tick > 300) {
                  serve_state = ACTION_STEP_2;
              }
              break;

          case ACTION_STEP_2:
              // 鍔ㄤ綔锛氬灚鐞冩満鏋勫浣嶏紝鍙戠悆鐢垫満淇濇寔杞姩纭繚鐞冨皠鍑?
								Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, 0.3f);
								Mechanism_Serve_SetAngle(SERVE_ACTION_RAD);

              // 寤舵椂鎬昏 800ms 鍚庡姩浣滅粨鏉燂紝鐘舵€佹満鍥炲埌绌洪棽
              if (current_tick - serve_start_tick > 800) {
                  Mechanism_Serve_SetAngle(SERVE_READY_RAD); // 鐢垫満杞€熸仮澶嶅緟鏈?
                  serve_state = ACTION_IDLE;
              }
              break;

          default: break;
      }

      // 浠诲姟棰戠巼鎺у埗
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
      TxData[0] = (uint8_t)auto_state;
      memset(&TxData[1], 0, sizeof(TxData) - 1);

      if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3) > 0U)
      {
          HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData);
      }
      else
      {
          g_can3_tx_drop_task03++;
      }

      vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
  UartRxMessage_t rx_msg;
  remote_engineer_t temp_rc; 

  for(;;)
  {
      // 闃诲绛夊緟娑堟伅闃熷垪涓殑鏁版嵁鍖?(鏉ヨ嚜涓插彛 ISR)
      if (osMessageQueueGet(remote_queueHandle, &rx_msg, NULL, osWaitForever) == osOK)
      {
          // 鍐欏叆瑙ｆ瀽鍣ㄧ紦鍐插尯
          Command_Write(rx_msg.data, rx_msg.size);
          // 寰幆瑙ｆ瀽鐩村埌缂撳啿鍖哄唴娌℃湁瀹屾暣鐨勬寚浠ゅ寘
          while (Command_GetCommand(processsed_command) != 0)
          {
              code_unzipread(processsed_command);   // 瑙ｇ爜鍗忚   
              Remote_Data_Convert(&rc, &temp_rc);   // 杞崲涓洪€氱敤宸ョ▼鏁版嵁鏍煎紡             
              
              // 鍐欏叆鍏ㄥ眬鍙橀噺锛屽姞閿侀槻姝富鎺т换鍔℃鍦ㄨ鍙栨椂鏁版嵁鍙戠敓绡℃敼
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
    #define CUSHION_READY_DEG   47.0f 
    #define CUSHION_SPEED      2.06f
    #define ACTION_HOLD_MS      800
    const float CUSHION_ACTION_DEG = 114.0f;

    osDelay(1000); // 涓婄數寤舵椂锛岀瓑寰呬紶鎰熷櫒绋冲畾
    // 鍒濆鍖栧嚮鐞冩澘浣嶇疆
    Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, CUSHION_SPEED);
    printf("System Ready. Angle reset.\r\n");

  for(;;)
  {
    // 1. 璇诲彇鍏夌數闂ㄥ紩鑴氱數骞?(GPIOB 11)
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET)
    {
        osDelay(5); // 杞欢娑堟姈
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET)
        {
					if(time_ms<0)
					{
						time_ms=0;
					}
						osDelay(time_ms);
            // === 妫€娴嬪埌鐞冿紒鎵ц涓€娆℃€у嚮鐞冧换鍔?===
            printf("Ball detected! Action!\r\n");

            // 鍑荤悆鍔ㄤ綔寮€濮?
            Mechanism_Cushion_SetAngle(CUSHION_ACTION_DEG, PITCH);
            osDelay(ACTION_HOLD_MS); // 淇濇寔涓€娈垫椂闂寸‘淇濈悆琚墦鍑?

            // 澶嶄綅鍔ㄤ綔
            Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, 0.3f);
            
            // 闃诲绛夊緟鐞冪寮€鍏夌數闂ㄦ劅搴旇寖鍥达紝闃叉杩炵画瑙﹀彂
            while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET)
            {
                osDelay(10);
            }
            
            osDelay(500); // 鍔ㄤ綔鍐峰嵈鏈?
            printf("Ready.\r\n");
        }
   }
    // 绌洪棽鏃剁殑浠诲姟寤舵椂
    osDelay(5);
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
    Mechanism_Loop_1ms();
    osDelay(2000);

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
  FDCAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8];
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(TASK07_CAN_PERIOD_MS);

  TxHeader.Identifier = CAN_ID_PC_FEEDBACK;
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
      uint32_t current_timestamp = HAL_GetTick();
      float yaw_angle = 0.0f;
      float pitch_angle = 0.0f;
      int16_t yaw_send;
      int16_t pitch_send;

      gimbal_get_angles(&yaw_angle, &pitch_angle);

      yaw_send = (int16_t)(yaw_angle * 100.0f);
      pitch_send = -(int16_t)(pitch_angle * 100.0f);

      memcpy(&TxData[0], &current_timestamp, 4);
      memcpy(&TxData[4], &yaw_send, 2);
      memcpy(&TxData[6], &pitch_send, 2);

      if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3) > 0U)
      {
          HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData);
      }
      else
      {
          g_can3_tx_drop_task07++;
      }

      vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartTask07 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static uint16_t prv_stack_words_clamp(UBaseType_t words)
{
    if (words > 0xFFFFU)
    {
        return 0xFFFFU;
    }
    return (uint16_t)words;
}

static void prv_update_stack_min(volatile uint16_t *min_words, UBaseType_t current_words)
{
    uint16_t clamped = prv_stack_words_clamp(current_words);
    if ((*min_words == 0U) || (clamped < *min_words))
    {
        *min_words = clamped;
    }
}

static void prv_update_stack_telemetry(void)
{
    g_freertos_stack_telemetry.sample_tick = HAL_GetTick();

    if (defaultTaskHandle != NULL)
        prv_update_stack_min(&g_freertos_stack_telemetry.defaultTask_min_words,
                             uxTaskGetStackHighWaterMark((TaskHandle_t)defaultTaskHandle));
    if (myTask02Handle != NULL)
        prv_update_stack_min(&g_freertos_stack_telemetry.task02_min_words,
                             uxTaskGetStackHighWaterMark((TaskHandle_t)myTask02Handle));
    if (myTask03Handle != NULL)
        prv_update_stack_min(&g_freertos_stack_telemetry.task03_min_words,
                             uxTaskGetStackHighWaterMark((TaskHandle_t)myTask03Handle));
    if (myTask04Handle != NULL)
        prv_update_stack_min(&g_freertos_stack_telemetry.task04_min_words,
                             uxTaskGetStackHighWaterMark((TaskHandle_t)myTask04Handle));
    if (myTask05Handle != NULL)
        prv_update_stack_min(&g_freertos_stack_telemetry.task05_min_words,
                             uxTaskGetStackHighWaterMark((TaskHandle_t)myTask05Handle));
    if (myTask06Handle != NULL)
        prv_update_stack_min(&g_freertos_stack_telemetry.task06_min_words,
                             uxTaskGetStackHighWaterMark((TaskHandle_t)myTask06Handle));
    if (myTask07Handle != NULL)
        prv_update_stack_min(&g_freertos_stack_telemetry.task07_min_words,
                             uxTaskGetStackHighWaterMark((TaskHandle_t)myTask07Handle));
}

/* -------------------------------------------------------------------------
// 涓插彛绌洪棽涓柇/DMA 瀹屾垚涓柇鍥炶皟鍑芥暟
// ------------------------------------------------------------------------- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    
    if (huart->Instance == USART2) { 
        UartRxMessage_t rx_msg;
        // 闄愬埗鎷疯礉澶у皬锛岄槻姝㈠唴瀛樿秺鐣?
        uint16_t copy_size = (Size < sizeof(rx_msg.data)) ? Size : sizeof(rx_msg.data);
        
        memcpy(rx_msg.data, remote_Buffer, copy_size);
        rx_msg.size = copy_size;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // 閫氳繃闃熷垪灏嗗師濮嬪瓧鑺傛祦鍙戦€佺粰瑙ｆ瀽浠诲姟
        xQueueSendFromISR(remote_queueHandle, &rx_msg, &xHigherPriorityTaskWoken);
        
        // 閲嶆柊寮€鍚笅涓€娆?DMA 鎺ユ敹
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_Buffer, RC_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); 

        // 濡傛灉鍙戦€佸鑷翠簡楂樹紭鍏堢骇浠诲姟灏辩华锛岃繘琛屼笂涓嬫枃鍒囨崲
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// 涓插彛纭欢閿欒鍥炶皟
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        // 娓呴櫎婧㈠嚭銆佸鍋舵牎楠岀瓑閿欒鏍囧織锛岄槻姝覆鍙ｅ崱姝?
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_ORE | UART_FLAG_NE);
        // 閲嶆柊灏濊瘯寮€鍚?DMA
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_Buffer, RC_BUFFER_SIZE);
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    g_freertos_fault_task = (pcTaskName != NULL) ? pcTaskName : "unknown";
    g_freertos_fault_free_heap = xPortGetFreeHeapSize();
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}

void vApplicationMallocFailedHook(void)
{
    g_freertos_fault_task = "malloc_failed";
    g_freertos_fault_free_heap = xPortGetFreeHeapSize();
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}

/* USER CODE END Application */




