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
#include <string.h>
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
/* USER CODE END PV */

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
int fgetc(FILE *f)
{
    uint8_t ch = 0;
    HAL_UART_Receive(&huart4, &ch, 1, HAL_MAX_DELAY);
    return ch;
}

extern float vx, vy, vr; // 加上 volatile 保护

// 串口接收相关的全局变量
#define RX_BUF_SIZE 32
uint8_t rx_buffer[RX_BUF_SIZE];   // DMA 接收缓存
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
  
  // 2. 开启 CAN 硬件
  fdcan_bsp_start(&hfdcan1); 
  fdcan_bsp_start(&hfdcan2); 
  fdcan_bsp_start(&hfdcan3); 
  dji_motors_init(); // 初始化 3 个 DJI 电机
  Vesc_init();
  Chassis_Init(); 

  // 清除错误标志，防止上电死锁
  __HAL_UART_CLEAR_IT(&huart4, UART_CLEAR_OREF | UART_CLEAR_NEF);

  // 开启 DMA 空闲中断接收 (核心修改)
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_buffer, RX_BUF_SIZE);
  // 必须关闭 DMA 过半中断(Half Transfer)，否则数据收到一半也会乱触发解析
  __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT); 

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

    // 1ms 周期执行底盘任务
    if(time[0] - time[1] >= 1) {
      Chassis_Task_Loop();
      time[1] = time[0];
    }

    // 每 200ms 自动打印一次当前速度，用于观测 (观测点)
    if(time[0] - time[2] >= 200) {
      // 使用 "%.2f" 格式可以直观看到 float 数值
      printf(">> Target [VX:%.2f | VY:%.2f | VR:%.2f]\r\n", vx, vy, vr);
      time[2] = time[0];
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
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart4) 
    {
        // 1. 安全封口
        if (Size < RX_BUF_SIZE) {
            rx_buffer[Size] = '\0';
        } else {
            rx_buffer[RX_BUF_SIZE - 1] = '\0';
        }
        
        // 2. 优化解析逻辑：使用 strstr 独立查找，支持单帧多指令 (例如: "X=1.5 Y=-0.2 R=0")
        char *ptr;
        if ((ptr = strstr((char*)rx_buffer, "X=")) != NULL) {
            sscanf(ptr, "X=%f", &vx);
        }
        if ((ptr = strstr((char*)rx_buffer, "Y=")) != NULL) {
            sscanf(ptr, "Y=%f", &vy);
        }
        if ((ptr = strstr((char*)rx_buffer, "R=")) != NULL) {
            sscanf(ptr, "R=%f", &vr);
        }
        
        // 刷新串口心跳时间
        time[3] = HAL_GetTick();

        // 3. 重新开启 DMA 空闲中断接收并关闭过半中断
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_buffer, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT); 
    }
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
