/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CRC.h"
#include "tjc_handle.h"
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
#define message_Length 7
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t time[4]={0};

uint8_t code_zip[10] = {0};
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(150);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_GPIO_WritePin (GPIOC , GPIO_PIN_15 ,GPIO_PIN_RESET );
	HAL_Delay(200);
  HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Rx_data3, 500);//开启串口1IDLE中断+DMA接收
	__HAL_DMA_DISABLE_IT (&hdma_usart3_rx ,DMA_IT_HT );//关闭接收过半中断
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Rx_data1, 500);//开启串口2IDLE中断+DMA接收
	__HAL_DMA_DISABLE_IT (&hdma_usart1_rx ,DMA_IT_HT );//关闭接收过半中断
////	OPEN_adjust();//开机校验
////	ADC_adjust();
	test=1;//开机校验
	send_flag=1;
	V_Flag_1=0;
	HAL_TIM_Base_Start_IT (&htim7);//以中断开启定时器7
	HAL_TIM_Base_Start_IT (&htim2);//以中断开启定时器2
	HAL_TIM_Base_Start_IT (&htim4);//以中断开启定时器4
	time[0]=HAL_GetTick();
	time[1]=time[0];time[2]=time[0];time[3]=time[0];
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
/* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 1. 不断调用解析程序处理串口缓冲区的数据
    TJC_Parse_Process();
    
    time[0]=HAL_GetTick();
    if(time[0]-time[1]>40) // 40ms 发送周期
    {
      // --- 处理屏幕发送的 start 和 end 指令 ---
      if (TJC_Status.cmd_start) {
          screen_start_cmd = 1;
          TJC_Status.cmd_start = 0;
      }
      if (TJC_Status.cmd_end) {
          screen_start_cmd = 0;
          TJC_Status.cmd_end = 0;
      }
      
      // 将模式状态映射到 TX_buffer[13]（接收端将其解析为 sw1）
      TX_buffer[13] = screen_start_cmd;

      // --- 原有的物理按键校验和计算 ---
      TX_buffer[34]=0;
      for(i=1;i<=23;i++)
      {
        TX_buffer[34] = TX_buffer[34] + TX_buffer[i];
      }
      
      uint16_t x_temp, y_temp, c_temp;
      
      // 2. 默认状态下，解析物理摇杆的数据
      x_temp = (TX_buffer[1])*1000 + (TX_buffer[2])*100 + (TX_buffer[3])*10 + (TX_buffer[4]);
      y_temp = (TX_buffer[5])*1000 + (TX_buffer[6])*100 + (TX_buffer[7])*10 + (TX_buffer[8]);
      c_temp = (TX_buffer[9])*1000 + (TX_buffer[10])*100 + (TX_buffer[11])*10 + (TX_buffer[12]);
      
      // 3. 【核心修改】如果处于屏幕控制模式，直接用屏幕坐标覆盖需要压缩的变量
      if (screen_start_cmd == 1) {
          x_temp = (uint16_t)TJC_Status.target_x;
          y_temp = (uint16_t)TJC_Status.target_y;
          
          // 安全限幅保护 (非常重要！)
          // 遥控器的压缩算法分配了 11 bit (最大2047) 来存储坐标。
          // 这里进行限幅，防止异常数据导致位移溢出，破坏其他通道数据。
          if (x_temp > 2000) x_temp = 2000;
          if (x_temp < 1000) x_temp = 1000; // 假设最小值为1000
          if (y_temp > 2000) y_temp = 2000;
          if (y_temp < 1000) y_temp = 1000;
      }

      // 4. 压缩打包逻辑 (直接对最终的 temp 变量进行位移压缩)
      code_zip[0] = 0x61;
      code_zip[1] = x_temp>>3;
      code_zip[2] = (x_temp & 0x07)<<5 | y_temp>>6;
      code_zip[3] = (y_temp & 0x3F)<<2 | c_temp>>9;
      code_zip[4] = (c_temp>>1) & 0xFF;
      code_zip[5] = (c_temp & 0x01)<<7;
      
      code_zip[5] |= ((TX_buffer[13]) & 0x03)<<5; // 整合了串口屏的启动标志
      code_zip[5] |= ((TX_buffer[14]) & 0x03)<<3;
      
      if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET){
        TX_buffer[15]=1;
      }
      else{
        TX_buffer[15]=0;
      }
      
      code_zip[5] |= ((TX_buffer[15]) & 0x01)<<1;
      code_zip[5] |= ((TX_buffer[16])>>1) & 0x01;

      code_zip[6] = ((TX_buffer[16]) & 0x01)<<7;
      code_zip[6] |= ((TX_buffer[17]) & 0x03)<<5;
      
      code_zip[6] |= ((TX_buffer[18]) & 0x01)<<4;
      code_zip[6] |= ((TX_buffer[19]) & 0x01)<<3;
      code_zip[6] |= ((TX_buffer[20]) & 0x01)<<2;
      code_zip[6] |= ((TX_buffer[21]) & 0x01)<<1;
      code_zip[6] |= ((TX_buffer[22]) & 0x01);
      
      code_zip[7] = ((TX_buffer[23]) & 0x01)<<7;
      
      uint16_t calculated_crc = CRC_16(code_zip , 8);			
      code_zip[8] |= (calculated_crc>>8);
      code_zip[9] |= calculated_crc;			
      
      HAL_UART_Transmit(&huart3, code_zip, sizeof(code_zip), 0xFF);
      memset(code_zip, 0, 10);
      
      time[1] = time[0];
    }
    /* USER CODE END WHILE */
		
    /* USER CODE BEGIN 3 */
////		HAL_Delay(1500);
////	HAL_GPIO_WritePin (BEEP_GPIO_Port , BEEP_Pin ,GPIO_PIN_SET );
////	HAL_Delay(500);
////  HAL_GPIO_WritePin (BEEP_GPIO_Port , BEEP_Pin ,GPIO_PIN_RESET );
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
