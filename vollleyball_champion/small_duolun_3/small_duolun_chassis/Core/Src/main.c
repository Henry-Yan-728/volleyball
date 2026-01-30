/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fdcan_bsp.h"
#include "chassis_task.h"
#include "control.h"    // 包含 STEERING_GEAR_RATIO
#include "dji_motor.h" 
#include "vesc.h"       
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ----------------- 1. 定义轨迹控制结构体 ----------------- */
typedef struct {
    float current_angle;    // 虚拟轴当前的实时角度 (舵轮输出轴角度，度)
    float target_angle;     // 最终目标角度
    float velocity_deg_ms;  // 速度：度/毫秒
    uint32_t last_tick;     // 上次更新时间
} Virtual_Axis_t;

// 初始化：起始0度，目标0度，速度 0.05度/ms (50度/秒)
Virtual_Axis_t v_axis = {0.0f, 0.0f, 0.05f, 0}; 

/* ----------------- 2. 轨迹更新与同步分发函数 ----------------- */
// 请在主循环中高频调用 (例如 1ms 一次)
void Update_Virtual_Axis(void)
{
    uint32_t now = HAL_GetTick();
    
    // 简单限频，防止调用过快
    if (now - v_axis.last_tick < 1) return; 
    v_axis.last_tick = now;

    // --- A. 轨迹插补 (Ramp) ---
    float error = v_axis.target_angle - v_axis.current_angle;
    
    if (fabs(error) <= v_axis.velocity_deg_ms) {
        v_axis.current_angle = v_axis.target_angle;
    }
    else {
        if (error > 0) v_axis.current_angle += v_axis.velocity_deg_ms;
        else           v_axis.current_angle -= v_axis.velocity_deg_ms;
    }

    // --- B. 同步分发给 3 个舵向电机 ---
    DJI_Motor_Instance* m0 = dji_motor_get_instance(0); // Wheel 1
    DJI_Motor_Instance* m1 = dji_motor_get_instance(1); // Wheel 2
    DJI_Motor_Instance* m2 = dji_motor_get_instance(2); // Wheel 3

    if (m0 && m1 && m2)
    {
        // 1. 考虑减速比：电机轴角度 = 输出轴角度 * 减速比
        // STEERING_GEAR_RATIO 在 control.h 中定义 (例如 36.0f)
        float motor_shaft_angle = v_axis.current_angle * STEERING_GEAR_RATIO;

        // 2. 转换为编码器数值
        int32_t encoder_pos = dji_degree2encoder(motor_shaft_angle);

        // 3. 同步设置 (正向同步)
        // 假设所有电机安装方向一致，如果某个电机反了，请在对应行加负号
        dji_motor_set_location(m0, encoder_pos);
        dji_motor_set_location(m1, encoder_pos);
        dji_motor_set_location(m2, encoder_pos);
    }
}/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint32_t time[4]={0};
float i = 0.0f;
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart4,(uint8_t *)&ch,1,HAL_MAX_DELAY );
	return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_LPUART1_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
// 1. 初始化 BSP 和 协议层
  fdcan_bsp_init();
  dji_motors_init(); // 初始化 3 个 DJI 电机
  Vesc_init();
  Chassis_Init(); 
  
  // 2. 开启 CAN 硬件
  fdcan_bsp_start(&hfdcan1); 
  fdcan_bsp_start(&hfdcan2); 
  fdcan_bsp_start(&hfdcan3); 

  uint32_t test_timer = 0;
  int state = 0;
  time[0] = HAL_GetTick();
	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		time[0] = HAL_GetTick();
    
    // 1. 高频执行轨迹更新 (模拟 1kHz 任务)
    Update_Virtual_Axis();

    // 2. 业务逻辑：每 5 秒切换一次目标角度，测试同步效果
    if (HAL_GetTick() - test_timer > 5000)
    {
        test_timer = HAL_GetTick();
        
        // 状态机：0度 -> 90度 -> 0度 -> -45度 ...
        switch(state) {
            case 0: v_axis.target_angle = 90.0f;  break; // 统一转到 90度
            case 1: v_axis.target_angle = 0.0f;   break; // 回零
            case 2: v_axis.target_angle = -90.0f; break; // 统一转到 -90度
            case 3: v_axis.target_angle = 0.0f;   break; // 回零
        }
        state = (state + 1) % 4;
        
        printf("State Changed: Target = %.1f\r\n", v_axis.target_angle);
    }
    
    // 可选：周期性打印调试信息
    if(time[0] - time[2] > 200) {
        // printf("Current: %.1f\r\n", v_axis.current_angle);
        time[2] = time[0];
    }
		/* USER CODE END 3 */
}
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
#ifdef USE_FULL_ASSERT
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
