/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "fdcan.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "usb_otg.h"
#include "gpio.h"
#include "unitree_motor.h"
#include <stdio.h>

UART_HandleTypeDef *huart_print = &huart1;

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(huart_print, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

void SystemClock_Config(void);
static void MPU_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MPU_Config();

    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();

    printf("\r\n================================\r\n");
    printf("STM32H723 Position Control\r\n");
    printf("================================\r\n");

    // 初始化位置控制
    MotorPositionControl_Init(1);
    printf("[Init] 完成\r\n");

    uint32_t switch_timer = HAL_GetTick();
    uint8_t state = 0;

    while (1)
    {
        uint32_t current_tick = HAL_GetTick();

        // 每3秒切换一次位置（0度 <-> 90度）
        if (current_tick - switch_timer >= 3000) {
            switch_timer = current_tick;
            state = !state;
            float target = state ? 90.0f : 0.0f;
            printf("[Cmd] 目标: %d度\r\n", (int)target);
            MotorPositionControl_SetTarget(1, target, 0.5f, 0.1f);  // Kp=0.5, Kd=0.1
        }

        // 更新控制
        MotorPositionControl_Update(1);

        HAL_Delay(1);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 68;
    RCC_OscInitStruct.PLL.PLLP = 1;
    RCC_OscInitStruct.PLL.PLLQ = 5;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
        Error_Handler();
    }
}

void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};
    HAL_MPU_Disable();
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x0;
    MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
