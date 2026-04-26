/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * 閺傚洣娆㈤崥?         : app_freertos.c
  * 閹诲繗锟?           : FreeRTOS 鎼存梻鏁ょ粙瀣碍娴狅絿鐖滈敍灞藉瘶閸氼偂鎹㈤崝陇鐨熸惔锔跨瑢闁槒绶幒褍锟?
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

// --- 閺夎法楠囬弨顖涘瘮锟?(BSP) 锟?閺堝搫娅掓禍鍝勫閼宠姤膩锟?---
#include "fdcan_bsp.h"      // CAN妞瑰崬锟?
#include "robot_data.h"     // 閸忋劌鐪張鍝勬珤娴滅儤鏆熼幑顔剧波锟?
#include "chassis_task.h"   // 鎼存洜娲忔惔鏇炵湴閹貉冨煑
#include "mechanism_task.h" // 閺堢儤鐎敍鍫熷閺夊棎鈧礁褰傞悶鍐搼閿涘甯堕崚?
#include "usart.h" 

// --- 闁儲甯堕崳銊ュ礂鐠侇喖顦╅悶鍡曠瑢閹稿洣鎶ょ憴锝嗭拷?---
#include "Task_command.h"   // 閹绘劒绶甸幐鍥︽姢閸愭瑥鍙嗘稉搴ゅ箯閸欐牗甯撮崣?
#include "remote_driver.h"  // 閹绘劒绶甸柆銉﹀付閸ｃ劌甯慨瀣掔粻妤佹殶閹诡喚绮ㄩ弸?
#include "queue.h"          // 闂冪喎鍨弨顖涘瘮
#include "semphr.h"         // 娣団€冲娇锟?娴滄帗鏋奸柌蹇旀暜锟?
#include "chassis_path_task.h" // 鐠侯垰绶炵憴鍕灊閻╃锟?
#include "Pan_Tilt_control.h"
#include "cybergear_motor.h"
#include "trajectory_planner.h"
#include "chassis_cybergear.h"
// 婢舵牠鍎村鏇犳暏绾兛娆㈤崣銉︾労
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
#define MM_TO_M 0.001f
#define AUTO_TARGET_YAW_RAD 0.0f
#define TARGET_DATA_TIMEOUT_MS 300U
#define POSE_DATA_TIMEOUT_MS 1000U
#define LOCK_YAW_KP 4.5f
#define LOCK_YAW_MAX_VR 6.0f
#define LOCK_YAW_DEADBAND_RAD 0.020f
static uint8_t s_cybergear_boot_ok = 0U;
static uint8_t s_cybergear_chassis_ready = 0U;
static const PlannerConfig_t s_auto_planner_config = {
    .max_spd = 8500.0f,
    .start_spd = 2200.0f,
    .stop_spd = 100.0f,
    .up_dist = 220.0f,
    .down_dist = 500.0f,
    .angle_kp = 1.8f,
};

uint64_t auto_state = 0;

	float time_ms = 1;
  float PITCH = 4.06f;                 // Pitch 鏉炴潙濮忔惔锔藉付閸掕泛褰夐柌?

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RC_BUFFER_SIZE  64  // 娑撴彃褰涢幒銉︽暪缂傛挸鍟块崠鍝勩亣锟?
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// --- FreeRTOS 閹垮秳缍旂化鑽ょ埠鐎电锟?---
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

osMutexId_t rc_mutexHandle;            // 娣囨繃濮㈤崗銊ョ湰闁儲甯堕崳銊ュ綁锟?g_remote_data 閻ㄥ嫪绨伴弬銉╂敚
osMessageQueueId_t remote_queueHandle; // 濞戝牊浼呴梼鐔峰灙閿涙氨鏁ゆ禍搴濊閸欙絼鑵戦弬顓炵殺閸樼喎顫愰弫鐗堝祦娴肩姷绮扮憴锝嗙€芥禒璇插

const osMutexAttr_t rc_mutex_attributes = {
  .name = "rcMutex"
};

// --- 閸忋劌鐪稉姘閸欐﹢锟?---
remote_engineer_t g_remote_data = {0}; // 閺堚偓缂佸牐袙缁犳鍤惃鍕淮閹貉冩珤缂佹挻鐎担?
uint8_t remote_Buffer[RC_BUFFER_SIZE]; // 娑撴彃锟?DMA 閹恒儲鏁归崢鐔奉潗閺佺増宓侀惃鍕敶鐎涙鈹栭梻?
uint8_t processsed_command[COMMAND_LENGTH]; // 閹稿洣鎶ょ憴锝嗙€芥稉瀛樻缂傛挸鍟块崠?
volatile FreertosStackTelemetry_t g_freertos_stack_telemetry = {0};
volatile const char* g_freertos_fault_task = NULL;
volatile uint32_t g_freertos_fault_free_heap = 0U;
volatile uint32_t g_can3_tx_drop_task03 = 0U;
volatile uint32_t g_can3_tx_drop_task07 = 0U;

// 婢舵牠鍎村鏇犳暏閸樼喎顫愰柆銉﹀付閸ｃ劍鏆熼幑顔剧波锟?
extern rc_info_t rc; 

/* 娴犺濮熼崣銉︾労鐎规矮锟?*/
osThreadId_t defaultTaskHandle;
osThreadId_t myTask02Handle; // 娑撶粯甯舵禒璇插閿涙艾绨抽惄?+ 閸斻劋缍旈柅鏄忕帆
osThreadId_t myTask03Handle; // 闁俺顔嗘禒璇插閿涙矮缍呯純顔荤瑐锟?
osThreadId_t myTask04Handle; // 鐟欙絾鐎芥禒璇插閿涙岸浠撮幒褍娅掗崡蹇氼唴鐟欙絿锟?
osThreadId_t myTask05Handle; // 閼奉亜濮╅柅鏄忕帆娴犺濮熼敍姘帨閻㈢敻妫憴锕€锟?

/* 娴犺濮熺仦鐐粹偓褔鍘ょ純?*/

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
static float prv_normalize_angle_rad(float angle);
static float prv_clampf(float value, float min_value, float max_value);
static void prv_lock_yaw_update(uint32_t current_tick,
                                float cmd_vx_field,
                                float cmd_vy_field,
                                float *target_vx,
                                float *target_vy,
                                float *target_vr);

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
  
  // 1. 閸掓繂顫愰崠鏍€栨禒鑸靛付閸掑墎娴夐崗铏殶閹诡喕绗屾惔鏇炵湴妞瑰崬锟?
  fdcan_bsp_init();     // CAN閹崵鍤庨崚婵嗩潗锟?
    cybergear_motors_init();
   Robot_Data_Init();    // 閸忋劌鐪弫鐗堝祦缂佹挻鐎崚婵嗩潗锟?
   Chassis_Init();       // 鎼存洜娲忛崣鍌涙殶閸掓繂顫愰崠?
   Mechanism_Init();     // 閺堢儤鐎崣鍌涙殶閸掓繂顫愰崠?


  // 2. 閸氼垰濮╅幍鈧張?CAN 闁俺顔嗛崣?
  Planner_Init(s_auto_planner_config);
  fdcan_bsp_start(&hfdcan1); 
  fdcan_bsp_start(&hfdcan2); 
  fdcan_bsp_start(&hfdcan3);

    if (trajectory_planner_init() == 0)
    {
        s_cybergear_boot_ok = 1U;
    }

  // 3. 閸掓稑缂撴禍鎺撴灱锟?(閻劋绨痪璺ㄢ柤鐎瑰锟?
  rc_mutexHandle = osMutexNew(&rc_mutex_attributes);

  // 4. 閸掓稑缂撻幐鍥︽姢濞戝牊浼呴梼鐔峰灙
  remote_queueHandle = osMessageQueueNew(16, sizeof(UartRxMessage_t), NULL);
	
  // 5. 瀵偓锟?UART2 DMA 瀵邦亞骞嗛幒銉︽暪閿涘本顥呭ù瀣敄闂傝弓鑵戦弬?(ReceiveToIdle)
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, remote_Buffer, RC_BUFFER_SIZE);
  // 缁備胶鏁ら崡濠佺炊鏉堟挷鑵戦弬顓ㄧ礉閸欘亜顦╅悶鍡曠鐢冪暚閹存劕鎮楅惃鍕敄闂傝弓鑵戦弬?
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

static float prv_normalize_angle_rad(float angle)
{
  while (angle > M_PI)
  {
    angle -= 2.0f * M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2.0f * M_PI;
  }
  return angle;
}

static float prv_clampf(float value, float min_value, float max_value)
{
  if (value > max_value)
  {
    return max_value;
  }
  if (value < min_value)
  {
    return min_value;
  }
  return value;
}

static void prv_lock_yaw_update(uint32_t current_tick,
                                float cmd_vx_field,
                                float cmd_vy_field,
                                float *target_vx,
                                float *target_vy,
                                float *target_vr)
{
  float current_yaw = 0.0f;
  uint32_t pose_update_tick = 0U;

  if ((target_vx == NULL) || (target_vy == NULL) || (target_vr == NULL))
  {
    return;
  }

  taskENTER_CRITICAL();
  current_yaw = g_robot_pose.angle;
  pose_update_tick = g_robot_pose.update_tick;
  taskEXIT_CRITICAL();

  *target_vx = cmd_vx_field;
  *target_vy = cmd_vy_field;
  *target_vr = 0.0f;

  if ((current_tick - pose_update_tick) <= POSE_DATA_TIMEOUT_MS)
  {
    float yaw_err = prv_normalize_angle_rad(AUTO_TARGET_YAW_RAD - current_yaw);
    float cos_yaw = cosf(current_yaw);
    float sin_yaw = sinf(current_yaw);

    *target_vx = (cmd_vx_field * cos_yaw) + (cmd_vy_field * sin_yaw);
    *target_vy = (-cmd_vx_field * sin_yaw) + (cmd_vy_field * cos_yaw);

    if (fabsf(yaw_err) > LOCK_YAW_DEADBAND_RAD)
    {
      *target_vr = prv_clampf(-yaw_err * LOCK_YAW_KP, -LOCK_YAW_MAX_VR, LOCK_YAW_MAX_VR);
    }
  }
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
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms 閹貉冨煑閸涖劍锟?
  xLastWakeTime = osKernelGetTickCount();

  // --- 1. 鐏炩偓闁劍甯堕崚璺哄綁闁插繐鍨垫慨瀣 ---
  remote_engineer_t local_rc = {0}; 
  float target_vx = 0, target_vy = 0, target_vr = 0;
  uint8_t auto_initialized = 0U;
  uint8_t last_mode = 0xFFU;
  
  // --- 2. 閺堢儤鐎崝銊ょ稊閸欏倹鏆熼柊宥囩枂 (鐟欐帒锟?闁喎锟? ---
  float PITCH_ANGLE = 0.0f;                 // Pitch 鏉炵顫楁惔锔藉付閸掕泛褰夐柌?
	
	// 鐎规矮绠熷☉鍫熷閺冨爼妫块梼鍫濃偓?(闁艾锟?20ms - 50ms)
#define DEBOUNCE_DELAY_MS 10

// 鐎规矮绠熼棃娆愨偓浣稿綁闁插繋绻氱€涙ü绗傛稉鈧▎陇袝閸欐垹娈戦弮鍫曟？ (娑旂喎褰叉禒銉︽杹閸︺劌鍙忕仦鈧紒鎾寸€担鎾茶厬)
static uint32_t last_time_btn2 = 0;
static uint32_t last_time_btn4 = 0;
static uint32_t last_time_btn1 = 0;
static uint32_t last_time_btn3 = 0;

// 鐎规矮绠熼悪顒傜彌閻ㄥ嫭瀵滈柨顔惧Ц閹礁宸婚崣鎻掑綁锟?(娣囶喖顦查崢鐔跺敩閻礁褰夐柌蹇擃槻閻劎娈態ug)
static uint8_t last_button5 = 0;
static uint8_t last_button2 = 0;
static uint8_t last_button4 = 0;
static uint8_t last_button1 = 0;
static uint8_t last_button3 = 0;


  // 閸欐垹鎮嗛崝銊ょ稊閻樿埖鈧焦婧€閻╃鍙ч崣姗€锟?

  // 閸偆鎮嗛張鐑樼€拫鍐Ν閹稿鏁崢鍡楀蕉鐠佹澘锟?(閻劋绨潏瑙勯儴濡偓锟?
  uint8_t last_button_front = 0;
  uint8_t last_button_back = 0;

  /* 娴犺濮熸稉璇叉儕锟?*/
  for(;;)
  {
      uint32_t current_tick = HAL_GetTick(); // 閼惧嘲褰囪ぐ鎾冲缁崵绮洪弮鍫曟？

      // ============================================================
      // 1. 鐎瑰鍙忛崷棰佺矤閸忋劌鐪崣姗€鍣洪懢宄板絿閺堚偓閺傛壆娈戦柆銉﹀付閸ｃ劍鏆熼幑?(閸旂娀鏀ｆ穱婵囧Б)
      // ============================================================
      if (osMutexAcquire(rc_mutexHandle, 10) == osOK) {
          local_rc = g_remote_data; 
          osMutexRelease(rc_mutexHandle);
      }

      if (local_rc.mode != last_mode) {
          auto_initialized = 0U;
          auto_state = finish;
          taskENTER_CRITICAL();
          g_robot_target.target_x = 0.0f;
          g_robot_target.target_y = 0.0f;
          g_robot_target.update_tick = 0U;
          g_robot_target.is_valid = 0U;
          g_robot_target.is_updated = 0U;
          taskEXIT_CRITICAL();
          last_mode = local_rc.mode;
      }

      // ============================================================
      // 2. 鎼存洜娲忔潻鎰З閹貉冨煑闁槒锟?(閸掑棙膩瀵繐顦╅悶?
      // ============================================================
      if ((local_rc.mode == CHASSIS_MODE_MANUAL) || (local_rc.mode == CHASSIS_MODE_LOCK_YAW)) 
      {
          /* --- 閹靛濮╁Ο鈥崇础 --- */
          if (local_rc.mode == CHASSIS_MODE_MANUAL) {
              target_vx = local_rc.vx / 100.0f;  // 瀹革箑褰搁獮宕囷拷?
              target_vy = local_rc.vy / 100.0f;  // 閸撳秴鎮楅獮宕囷拷?
              target_vr = -local_rc.vw * 3.0f;
          } else {
              prv_lock_yaw_update(current_tick,
                                  local_rc.vx / 100.0f,
                                  local_rc.vy / 100.0f,
                                  &target_vx,
                                  &target_vy,
                                  &target_vr);
          }



// --- Button 2 & 4 婢跺嫮锟?(Pitch 鐟欐帒瀹崇拫鍐Ν) ---
// Button 2
if (local_rc.button2 == 1 && last_button2 == 0) {
    if ((current_tick - last_time_btn2) > DEBOUNCE_DELAY_MS) {
        PITCH_ANGLE += 5.0f;
        Mechanism_Dian_Pitch_SetAngle(PITCH_ANGLE);
        last_time_btn2 = current_tick;
    }
}
last_button2 = local_rc.button2; // 閻欘剛鐝涙穱婵嗙摠閻樿埖锟?

// Button 4
if (local_rc.button4 == 1 && last_button4 == 0) {
    if ((current_tick - last_time_btn4) > DEBOUNCE_DELAY_MS) {
        PITCH_ANGLE -= 5.0f;
        Mechanism_Dian_Pitch_SetAngle(PITCH_ANGLE);
        last_time_btn4 = current_tick;
    }
}
last_button4 = local_rc.button4; // 閻欘剛鐝涙穱婵嗙摠閻樿埖锟?

// --- Button 1 & 3 婢跺嫮锟?(PITCH 瀵邦喛锟? ---
// Button 1
if (local_rc.button1 == 1 && last_button1 == 0) {
    if ((current_tick - last_time_btn1) > DEBOUNCE_DELAY_MS) {
        PITCH += power_add;
//        time_ms += time_add;
        last_time_btn1 = current_tick;
    }
}
last_button1 = local_rc.button1; // 閻欘剛鐝涙穱婵嗙摠閻樿埖锟?

// Button 3
if (local_rc.button3 == 1 && last_button3 == 0) {
    if ((current_tick - last_time_btn3) > DEBOUNCE_DELAY_MS) {
        PITCH -= power_add;
//        time_ms -= time_add;
        last_time_btn3 = current_tick;
    }
}
last_button3 = local_rc.button3; // 閻欘剛鐝涙穱婵嗙摠閻樿埖锟?

          // 閹笛嗩攽鎼存洜娲忔す鍗炲З閺囧瓨锟?
           Chassis_Update(target_vx, target_vy, target_vr);
      }
      else if (local_rc.mode == CHASSIS_MODE_AUTO)
      {
          if (local_rc.button1 == 1 && last_button1 == 0) {
              if ((current_tick - last_time_btn1) > DEBOUNCE_DELAY_MS) {
                  taskENTER_CRITICAL();
                  auto_state = finish;
                  g_robot_target.target_x = 0.0f;
                  g_robot_target.target_y = 0.0f;
                  g_robot_target.update_tick = 0U;
                  g_robot_target.is_valid = 0U;
                  g_robot_target.is_updated = 0U;
                  taskEXIT_CRITICAL();
                  auto_initialized = 0U;
                  last_time_btn1 = current_tick;
              }
          }
          last_button1 = local_rc.button1;

          if (local_rc.button3 == 1 && last_button3 == 0) {
              if ((current_tick - last_time_btn3) > DEBOUNCE_DELAY_MS) {
                  taskENTER_CRITICAL();
                  auto_state = begin;
                  g_robot_target.target_x = 0.0f;
                  g_robot_target.target_y = 0.0f;
                  g_robot_target.update_tick = 0U;
                  g_robot_target.is_valid = 0U;
                  g_robot_target.is_updated = 0U;
                  taskEXIT_CRITICAL();
                  auto_initialized = 0U;
                  last_time_btn3 = current_tick;
              }
          }
          last_button3 = local_rc.button3;

          /* --- 闂佺绻堥崝蹇涘吹濠婂牆绀夐柕濞㈢繝绀侀?(闁荤姳璀﹂崹鎵閻愬灚鍠嗛柛鏇ㄥ亜锟? --- */
          float cur_x, cur_y, cur_yaw;
          float target_x, target_y;
          uint32_t pose_update_tick;
          uint32_t target_update_tick;
          uint8_t target_valid;
          uint8_t target_updated;
          
          // 闁哄鏅滅粙鎴﹀矗閸℃鈻旈悗闈涙啞濞呮洟鏌涢弽顐㈢殤缂佽鲸绻勯幏鐘垫嫚閹绘帞鎮奸梺绋跨箞閸斿矂鎯囬鍌楀亾鐟欏嫮鐓紓宥呮嚇瀹曟悂骞囬鐘茬伇闂佹寧绋戦惌鍌毼ｇ拠宸桨闁靛繆鍓濆▓鍫曟煙鐠団€虫灕妞ゎ偓闄勭粙澶愵敇閻斿摜鍔锋繛锝呮祩閸犳寮懖鈹惧亾娴ｅ啫顥嶉柛銇卞嫮鈻旂€广儱瀚閬嶆煠?
          taskENTER_CRITICAL();
          cur_x = g_robot_pose.x;
          cur_y = g_robot_pose.y;
          cur_yaw = g_robot_pose.angle;
          pose_update_tick = g_robot_pose.update_tick;
          target_x = g_robot_target.target_x;
          target_y = g_robot_target.target_y;
          target_update_tick = g_robot_target.update_tick;
          target_valid = g_robot_target.is_valid;
          target_updated = g_robot_target.is_updated;
          g_robot_target.is_updated = 0U;
          taskEXIT_CRITICAL();

          // 闁烩剝甯掗幊蹇擃渻閸屾稓鈹嶆繝闈涙搐琚橀梺鎸庣⊕閼归箖銆呰瀵顭ㄩ崘顏呮瘞婵炶揪绲界粔鏉戭焽濡ゅ懎鍐€闁搞儺鍓氬В鎰版煛閸愩劎鍩ｆ俊顐ｅ缁嬪顓奸崱妯煎嚱闂佸搫鍊告惔婊呮濠靛纾绘繝濠傚閸撻箖寮堕埡鍌氬锟?
          if (auto_state != begin) {
               auto_initialized = 0U;
               target_vx = 0.0f; target_vy = 0.0f; target_vr = 0.0f;
          }
          else if ((HAL_GetTick() - pose_update_tick > 1000U) ||
                   (target_valid == 0U) ||
                   (HAL_GetTick() - target_update_tick > TARGET_DATA_TIMEOUT_MS)) {
               auto_initialized = 0U;
               target_vx = 0.0f; target_vy = 0.0f; target_vr = 0.0f;
          }
          else 
          {
              if ((!auto_initialized) || (target_updated != 0U)) {
                  // 闂佸憡甯楃换鍌烇綖閹版澘绀岄柡宥庡墻濞兼劙鏌涢幒鎾崇瑨婵炲懏甯￠幆鍕敊閻ｅ苯锟?
                  Planner_SetTarget(cur_x, cur_y, target_x, target_y, AUTO_TARGET_YAW_RAD); 
                  auto_initialized = 1U;
              }

              // 闂佸吋鍎抽崲鑼躲亹閸モ晜鍠嗛柛鏇ㄥ亜閻忓﹤鈹戦崒婊勬珪婵炲牊鍨剁粙澶愬冀椤愶絾鐝紓渚囧灠椤兘鍩€椤掑倸鏋庨悗?
              TrajVel_t cmd = Planner_Update(cur_x, cur_y, cur_yaw);
              
              if (Planner_IsArrived(cur_x, cur_y, cur_yaw)) {
                  target_vx = 0.0f;
                  target_vy = 0.0f;
                  target_vr = 0.0f;
                  auto_initialized = 0U;//闂備焦褰冪粔鎾箚鎼达絺鍋撶憴鍕叝缂傚秴鎳樺畷娆撴嚍閵夛附锟?
              } else {
                  // 婵炴垶鎸婚悧婊堝疾椤愶箑閿ら柟閭﹀幘閸ㄨ偐绱掗姘肩吋闁逞屽墰閸犲海鈧娅曞顏堫敄閼愁垳顦伴柡澶屽剳缂傛氨绱炵€ｎ剙瀵插┑鐘冲焹閸嬫捇鎮㈠畡閭﹀敽
                  float theta = cur_yaw;
                  target_vx = ( cmd.vx * cosf(theta) + cmd.vy * sinf(theta)) * MM_TO_M;
                  target_vy = (-cmd.vx * sinf(theta) + cmd.vy * cosf(theta)) * MM_TO_M;
                  target_vr = cmd.vr;
              }
						}
          Chassis_Update(-target_vy, target_vx, -target_vr);
      }
      else if (local_rc.mode == CHASSIS_MODE_SERVE)
      {
          /* --- 閼奉亜濮╃€甸€涚秴閸欐垹鎮嗗Ο鈥崇础 --- */
          float cur_x, cur_y, cur_yaw, target_x = 0.0f, target_y = 0.0f;
          uint32_t last_update;
          
          taskENTER_CRITICAL();
          cur_x = g_robot_pose.x;
          cur_y = g_robot_pose.y;
          cur_yaw = g_robot_pose.angle;
          last_update = g_robot_pose.update_tick;
          taskEXIT_CRITICAL();
				
				//婢跺秳缍呴幐澶愭暛
          if (local_rc.button5 == 1 && last_button5 == 0) {
						target_x = 0.0f;
						target_y = 0.0f;
          }
          last_button5 = local_rc.button5;
				
					//4闁款喖鎮楅崷鍝勫閸欐垹鎮嗛敍?闁款喖鎮楅崷鍝勬倵閸欐垹锟?
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
              static uint8_t initialized = 0;
              if (!initialized) {
                  Planner_SetTarget(cur_x, cur_y, target_x, target_y, cur_yaw);
                  initialized = 1;
              }

              TrajVel_t cmd = Planner_Update(cur_x, cur_y, cur_yaw);

              if (Planner_IsArrived(cur_x, cur_y, cur_yaw)) {
                  target_vx = 0.0f;
                  target_vy = 0.0f;
                  target_vr = 0.0f;
              } else {
                  float theta = cur_yaw;
                  target_vx = ( cmd.vx * cosf(theta) + cmd.vy * sinf(theta)) * MM_TO_M;
                  target_vy = (-cmd.vx * sinf(theta) + cmd.vy * cosf(theta)) * MM_TO_M;
                  target_vr = cmd.vr;
              }
              
              Chassis_Update(target_vx, target_vy, target_vr);
          }
      }

      // ============================================================
      // 3. 閺堢儤鐎幒褍鍩楅柅鏄忕帆閿涙艾褰傞悶鍐Ц閹焦婧€閹笛嗩攽 (瀵倹顒為棃鐐烘▎锟?
      // ============================================================

      // 娴犺濮熸０鎴犲芳閹貉冨煑
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
      // 闂冭顢ｇ粵澶婄窡濞戝牊浼呴梼鐔峰灙娑擃厾娈戦弫鐗堝祦锟?(閺夈儴鍤滄稉鎻掑經 ISR)
      if (osMessageQueueGet(remote_queueHandle, &rx_msg, NULL, osWaitForever) == osOK)
      {
          // 閸愭瑥鍙嗙憴锝嗙€介崳銊х处閸愭彃锟?
          Command_Write(rx_msg.data, rx_msg.size);
          // 瀵邦亞骞嗙憴锝嗙€介惄鏉戝煂缂傛挸鍟块崠鍝勫敶濞屸剝婀佺€瑰本鏆ｉ惃鍕瘹娴犮倕锟?
          while (Command_GetCommand(processsed_command) != 0)
          {
              code_unzipread(processsed_command);   // 鐟欙絿鐖滈崡蹇氼唴   
              Remote_Data_Convert(&rc, &temp_rc);   // 鏉烆剚宕叉稉娲偓姘辨暏瀹搞儳鈻奸弫鐗堝祦閺嶇厧锟?            
              
              // 閸愭瑥鍙嗛崗銊ョ湰閸欐﹢鍣洪敍灞藉闁夸線妲诲顫瘜閹貉傛崲閸斺剝顒滈崷銊嚢閸欐牗妞傞弫鐗堝祦閸欐垹鏁撶弧鈩冩暭
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

    osDelay(1000); // 娑撳﹦鏁稿鑸垫閿涘瞼鐡戝鍛炊閹扮喎娅掔粙鍐茬暰
    // 閸掓繂顫愰崠鏍у毊閻炲啯婢樻担宥囩枂
    Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, CUSHION_SPEED);
    printf("System Ready. Angle reset.\r\n");

  for(;;)
  {
    // 1. 鐠囪褰囬崗澶屾暩闂傘劌绱╅懘姘辨暩锟?(GPIOB 11)
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET)
    {
        osDelay(5); // 鏉烆垯娆㈠☉鍫熷
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET)
        {
					if(time_ms<0)
					{
						time_ms=0;
					}
						osDelay(time_ms);
            // === 濡偓濞村鍩岄悶鍐跨磼閹笛嗩攽娑撯偓濞嗏剝鈧冨毊閻炲啩鎹㈤崝?===
            printf("Ball detected! Action!\r\n");

            // 閸戣崵鎮嗛崝銊ょ稊瀵偓锟?
            Mechanism_Cushion_SetAngle(CUSHION_ACTION_DEG, PITCH);
            osDelay(ACTION_HOLD_MS); // 娣囨繃瀵旀稉鈧▓鍨闂傚鈥樻穱婵堟倖鐞氼偅澧﹂崙?

            // 婢跺秳缍呴崝銊ょ稊
            Mechanism_Cushion_SetAngle(CUSHION_READY_DEG, 0.3f);
            
            // 闂冭顢ｇ粵澶婄窡閻炲啰顬囧鈧崗澶屾暩闂傘劍鍔呮惔鏃囧瘱閸ヨ揪绱濋梼鍙夘剾鏉╃偟鐢荤憴锕€锟?
            while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET)
            {
                osDelay(10);
            }
            
            osDelay(500); // 閸斻劋缍旈崘宄板祱锟?
            printf("Ready.\r\n");
        }
   }
    // 缁屾椽妫介弮鍓佹畱娴犺濮熷鑸垫
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
      pitch_send = (int16_t)(pitch_angle * 100.0f);

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
// 娑撴彃褰涚粚娲＝娑擃厽锟?DMA 鐎瑰本鍨氭稉顓熸焽閸ョ偠鐨熼崙鑺ユ殶
// ------------------------------------------------------------------------- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    
    if (huart->Instance == USART2) { 
        UartRxMessage_t rx_msg;
        // 闂勬劕鍩楅幏鐤婢堆冪毈閿涘矂妲诲銏犲敶鐎涙绉洪悾?
        uint16_t copy_size = (Size < sizeof(rx_msg.data)) ? Size : sizeof(rx_msg.data);
        
        memcpy(rx_msg.data, remote_Buffer, copy_size);
        rx_msg.size = copy_size;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // 闁俺绻冮梼鐔峰灙鐏忓棗甯慨瀣摟閼哄倹绁﹂崣鎴︹偓浣虹舶鐟欙絾鐎芥禒璇插
        xQueueSendFromISR(remote_queueHandle, &rx_msg, &xHigherPriorityTaskWoken);
        
        // 闁插秵鏌婂鈧崥顖欑瑓娑撯偓锟?DMA 閹恒儲锟?
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_Buffer, RC_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); 

        // 婵″倹鐏夐崣鎴︹偓浣割嚤閼风繝绨℃妯圭喘閸忓牏楠囨禒璇插鐏忚京鍗庨敍宀冪箻鐞涘奔绗傛稉瀣瀮閸掑洦锟?
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// 娑撴彃褰涚涵顑挎闁挎瑨顕ら崶鐐剁殶
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        // 濞撳懘娅庡┃銏犲毉閵嗕礁顨岄崑鑸电墡妤犲瞼鐡戦柨娆掝嚖閺嶅洤绻旈敍宀勬Щ濮濐澀瑕嗛崣锝呭幢锟?
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_ORE | UART_FLAG_NE);
        // 闁插秵鏌婄亸婵婄槸瀵偓锟?DMA
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
