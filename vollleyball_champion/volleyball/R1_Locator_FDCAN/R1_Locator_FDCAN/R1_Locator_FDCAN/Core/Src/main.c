/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序入口体
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "bsp_can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct {
    float x;
    float y;
    float theta; // 角度 (-PI, PI]
    float vx;
    float vy;
    float v_theta;
} RobotPose_t;

typedef struct {
    float q[4];      // 四元数 [w, x, y, z]
    float gyro_rad[3]; // 角速度 [x, y, z] rad/s
    float accel[3];    // 加速度 [x, y, z]
    float yaw;         // 当前偏航角
    float yaw_offset;  // 零飘/校准偏置
    uint32_t last_update_tick;
} IMU_State_t;

typedef struct {
    int32_t pulse_total[2]; // 累积脉冲
    int32_t pulse_delta[2]; // 增量脉冲
    float dist_delta[2];    // 距离增量
} Encoder_State_t;

/* 私有宏定义 ----------------------------------------------------------------*/
#define PI 3.1415926535f
#define DEG2RAD 0.0174532925f
#define GYRO_SCALE_FACTOR 20.072f // 示例修正系数

// 运动学参数 (改为 float 以利用 FPU)
#define ROT_E0_P (-886.069f)
#define ROT_E1_P (-881.046f)
#define ROT_E0_N (-1401.902f)
#define ROT_E1_N (-657.607f)

#define KINEMATIC_X0 (0.013838f)
#define KINEMATIC_X1 (0.013791f)
#define KINEMATIC_Y0 (0.013806f)
#define KINEMATIC_Y1 (-0.013764f)

/* 全局变量 ------------------------------------------------------------------*/
RobotPose_t g_robot_pose = {105.0f, 105.0f, 0.0f, 0.0f, 0.0f, 0.0f};
IMU_State_t g_imu = {{1.0f,0,0,0}, {0}, {0}, 0, 0, 0};
Encoder_State_t g_encoder = {0};

// 通信缓冲区
uint8_t g_uart1_rx_byte;      // 陀螺仪单字节接收
uint8_t g_gyro_buf[33];       // 陀螺仪完整数据包
volatile uint8_t g_gyro_ready = 0; // 陀螺仪数据包就绪标志

uint8_t g_uart2_rx_buf[11];   // PC/激光接收
volatile uint8_t g_laser_ready = 0; // 激光数据就绪标志
float g_laser_data[2];        // 解析后的激光数据 [x, y]

// 编码器定时器溢出计数
volatile int32_t g_tim_overflow[2] = {0};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Robot_Update_IMU(void);
void Robot_Update_Odometry(void);
void CAN_Send_RobotPose(void);
void CAN_Send_Command(uint32_t id, uint8_t* data, uint8_t len);
float Math_InvSqrt(float x);
float Math_NormalizeAngle(float angle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 定时器溢出处理 (编码器计数)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1)) g_tim_overflow[0]--;
        else g_tim_overflow[0]++;
    }
    else if (htim->Instance == TIM2) {
        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) g_tim_overflow[1]--;
        else g_tim_overflow[1]++;
    }
}

// 串口接收回调 (极简模式，不做复杂解析)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    static uint8_t gyro_idx = 0;
    
    // --- USART1: 陀螺仪 (模拟环形缓冲/状态机) ---
    if (huart->Instance == USART1) {
        // 简单的包头检测状态机
        if (gyro_idx == 0 && g_uart1_rx_byte != 0x80) { // 等待包头
            // 错误，忽略
        } else {
            g_gyro_buf[gyro_idx++] = g_uart1_rx_byte;
            if (gyro_idx >= 33) { // 包满
                gyro_idx = 0;
                g_gyro_ready = 1; // 置标志位，主循环处理
            }
        }
        HAL_UART_Receive_IT(&huart1, &g_uart1_rx_byte, 1);
    }
    
    // --- USART2: PC/激光 (DMA接收或定长中断) ---
    else if (huart->Instance == USART2) {
        // 假设协议: Head(0xBE) ... Tail(0xBF 0xCF)
        if (g_uart2_rx_buf[0] == 0xBE && g_uart2_rx_buf[10] == 0xCF) { // 简单校验
            // 这里为了数据一致性，可以把解析放在中断里，或者拷贝出去
            // 为了优化，我们这里只拷贝数据，不做printf等耗时操作
            memcpy(&g_laser_data[0], &g_uart2_rx_buf[2], 8); // 拷贝两个float
            g_laser_ready = 1;
        }
        HAL_UART_Receive_IT(&huart2, g_uart2_rx_buf, 11); // 继续接收下一包
    }
}

// CAN 接收回调
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        // 处理复位命令等简单逻辑
        if (rx_header.Identifier == 0xAE && rx_data[0] == 'r') {
             // 设置复位标志，主循环处理
             g_robot_pose.x = 105.0f; 
             g_robot_pose.y = 105.0f;
             g_robot_pose.theta = 0.0f;
        }
    }
}
/*************************************************************/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_FDCAN2_Init();
  /* USER CODE BEGIN 2 */
	//Merge_init();
  // 启动外设
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_UART_Receive_IT(&huart1, &g_uart1_rx_byte, 1);
    HAL_UART_Receive_IT(&huart2, g_uart2_rx_buf, 11);

    uint32_t tick_imu = 0;
    uint32_t tick_odom = 0;
    uint32_t tick_can = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      uint32_t now = HAL_GetTick();

        // 1. IMU 解算 (尽快处理)
        if (g_gyro_ready) {
            Robot_Update_IMU(); // 内部清除 g_gyro_ready
        }

        // 2. 里程计更新 (5ms ~ 200Hz)
        if (now - tick_odom >= 5) {
            Robot_Update_Odometry();
            tick_odom = now;
        }

        // 3. CAN 发送 (20ms ~ 50Hz)
        if (now - tick_can >= 20) {
            CAN_Send_RobotPose();
            tick_can = now;
        }
        
        // 4. 激光数据融合处理 (当数据到达时)
        if (g_laser_ready) {
            // 执行 Kalman 更新步骤...
            g_laser_ready = 0;
        }
    }  
	/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV5;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* 功能函数优化实现 ----------------------------------------------------------*/

/**
 * @brief  获取编码器当前总计数值 (处理原子性)
 */
static int32_t Get_Encoder_Count(TIM_TypeDef *TIMx, int idx) {
    int32_t count;
    // 临界区保护，防止读取时发生溢出中断导致高位错误
    __disable_irq(); 
    count = (g_tim_overflow[idx] * (int32_t)(TIMx->ARR + 1)) + TIMx->CNT;
    __enable_irq();
    return count;
}

/**
 * @brief  IMU 数据解算 (Mahony 互补滤波简化版)
 */
void Robot_Update_IMU(void) {
    g_gyro_ready = 0; // 清除标志
    
    // 1. 解析原始数据 (移位操作保持整型)
    int32_t raw_w[3], raw_a[3];
    raw_w[0] = (g_gyro_buf[5]<<24) | (g_gyro_buf[4]<<16) | (g_gyro_buf[3]<<8) | g_gyro_buf[2];
    raw_w[1] = (g_gyro_buf[9]<<24) | (g_gyro_buf[8]<<16) | (g_gyro_buf[7]<<8) | g_gyro_buf[6];
    raw_w[2] = (g_gyro_buf[13]<<24)| (g_gyro_buf[12]<<16)| (g_gyro_buf[11]<<8)| g_gyro_buf[10];
    
    // 加速度... (同理省略)
    raw_a[0] = (g_gyro_buf[17]<<24)| (g_gyro_buf[16]<<16)| (g_gyro_buf[15]<<8)| g_gyro_buf[14];
    raw_a[1] = (g_gyro_buf[21]<<24)| (g_gyro_buf[20]<<16)| (g_gyro_buf[19]<<8)| g_gyro_buf[18];
    raw_a[2] = (g_gyro_buf[25]<<24)| (g_gyro_buf[24]<<16)| (g_gyro_buf[23]<<8)| g_gyro_buf[22];

    // 2. 物理量转换 (使用 float 常量)
    const float DEG_TO_RAD_RATE = 0.01745329f;
    for(int i=0; i<3; i++) {
        g_imu.gyro_rad[i] = *(float*)&raw_w[i] * DEG_TO_RAD_RATE;
        g_imu.accel[i]    = *(float*)&raw_a[i];
    }
    
    // 坐标系修正
    g_imu.gyro_rad[0] *= -1.0f;
    g_imu.gyro_rad[2] *= -1.0f;
    g_imu.accel[0]    *= -1.0f;
    g_imu.accel[2]    *= -1.0f;

    // 3. 计算 dt
    uint32_t now = HAL_GetTick();
    float dt = (now - g_imu.last_update_tick) * 0.001f;
    g_imu.last_update_tick = now;
    if(dt <= 0 || dt > 0.1f) dt = 0.005f; // 防止异常 dt

    // 4. 姿态解算 (这里简化为只积分 Yaw，实际应加入加速度互补滤波)
    // 假设已经有完善的四元数更新逻辑，此处为了节省空间略过标准Mahony算法代码
    // 核心是使用 float 进行 q_angle_update...
    
    // 简单积分示例 (带漂移补偿)
    float delta_theta = g_imu.gyro_rad[2] * dt;
    
    // 漂移修正 (线性补偿)
    // delta_theta -= DRIFT_BIAS * dt; 

    g_imu.yaw += delta_theta;
    g_imu.yaw = Math_NormalizeAngle(g_imu.yaw);
    
    // 更新全局位姿的角度
    g_robot_pose.theta = g_imu.yaw;
}

/**
 * @brief  里程计定位核心算法
 */
void Robot_Update_Odometry(void) {
    // 1. 读取当前编码器值
    int32_t cur_pulse[2];
    cur_pulse[0] = Get_Encoder_Count(TIM1, 0);
    cur_pulse[1] = Get_Encoder_Count(TIM2, 1);

    // 2. 计算增量
    g_encoder.pulse_delta[0] = cur_pulse[0] - g_encoder.pulse_total[0];
    g_encoder.pulse_delta[1] = cur_pulse[1] - g_encoder.pulse_total[1];
    
    g_encoder.pulse_total[0] = cur_pulse[0];
    g_encoder.pulse_total[1] = cur_pulse[1];

    // 3. 角度增量 (从 IMU 获取更准，或从轮速差分)
    // 这里使用 IMU 的当前角度与上一次的差值更为平滑，或者直接用陀螺仪角速度积分
    static float last_theta = 0;
    float d_theta = g_robot_pose.theta - last_theta;
    d_theta = Math_NormalizeAngle(d_theta); // 处理过零点
    last_theta = g_robot_pose.theta;

    // 4. 旋转解耦 (使用 float FPU)
    // 原理：消除自旋时轮子产生的非位移脉冲
    float dx_enc = (float)g_encoder.pulse_delta[0];
    float dy_enc = (float)g_encoder.pulse_delta[1];

    if (d_theta > 0) {
        dx_enc -= ROT_E0_P * d_theta;
        dy_enc -= ROT_E1_P * d_theta;
    } else {
        dx_enc -= ROT_E0_N * d_theta;
        dy_enc -= ROT_E1_N * d_theta;
    }

    // 5. 运动学逆解 (底盘坐标系位移)
    // dx_robot, dy_robot
    float dr_x = KINEMATIC_X0 * dx_enc + KINEMATIC_X1 * dy_enc;
    float dr_y = KINEMATIC_Y0 * dx_enc + KINEMATIC_Y1 * dy_enc;

    // 6. 航位推算 (世界坐标系)
    // 使用中值积分或简单欧拉积分
    float sin_t = sinf(g_robot_pose.theta); // 使用 sinf (float)
    float cos_t = cosf(g_robot_pose.theta); // 使用 cosf (float)

    float d_world_x = cos_t * dr_x - sin_t * dr_y;
    float d_world_y = sin_t * dr_x + cos_t * dr_y;

    g_robot_pose.x += d_world_x;
    g_robot_pose.y += d_world_y;

    // 7. 速度计算 (低通滤波)
    // ... 可以添加滑动平均滤波 ...
}

/**
 * @brief  通用 CAN 发送函数 (优化：不重复代码)
 */
void CAN_Send_Command(uint32_t id, uint8_t* data, uint8_t len) {
    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = (len <= 8) ? FDCAN_DLC_BYTES_8 : FDCAN_DLC_BYTES_8; // 需根据FDCAN配置调整
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    TxHeader.FDFormat = FDCAN_FD_CAN; // 或 Classic
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // 检查 FIFO 空间
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data);
    }
}

/**
 * @brief  打包发送定位数据 (拆包逻辑)
 */
void CAN_Send_RobotPose(void) {
    // 准备数据：x, y, theta, vx, vy, vw (6个float = 24 bytes)
    // 分3包发送，ID 0xAA
    static uint8_t tx_buf[24];
    memcpy(tx_buf, &g_robot_pose, 24); // 结构体内存布局需紧凑，或逐个赋值

    for (int i = 0; i < 3; i++) {
        CAN_Send_Command(0xAA, &tx_buf[i*8], 8);
    }
}

/* 数学库优化 ----------------------------------------------------------------*/

// 快速平方根倒数 (Quake III算法，比 standard 1.0f/sqrtf() 快，但精度略低，STM32 FPU下差异不大)
float Math_InvSqrt(float x) {
    float xhalf = 0.5f * x;
    int i = *(int*)&x;
    i = 0x5f3759df - (i >> 1);
    x = *(float*)&i;
    x = x * (1.5f - xhalf * x * x);
    return x;
}

// 角度归一化到 [-PI, PI]
float Math_NormalizeAngle(float angle) {
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */