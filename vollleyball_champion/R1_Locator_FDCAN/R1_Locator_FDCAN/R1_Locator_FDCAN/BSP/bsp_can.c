#include "bsp_can.h"
#include "main.h"
#include <stdio.h>
//#include "dji_3508_2006_motor.h"
//#include "debug.h"
/**************内部宏定义与重命名begin**************/

/**************内部宏定义与重命名end**************/

/**************内部变量与函数begin**************/

/**************内部变量与函数end**************/

/**************外部接口begin**************/

/**************外部接口end**************/

FDCAN_TxHeaderTypeDef TxHeader1;
FDCAN_RxHeaderTypeDef RxHeader1;
FDCAN_TxHeaderTypeDef TxHeader2;
FDCAN_RxHeaderTypeDef RxHeader2;
void FDCAN1_RxFilter_Config(void)
{
	FDCAN_FilterTypeDef sFilterConfig;
	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x00000000;
	sFilterConfig.FilterID2 = 0X1FFFFFFF;
	if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	
	if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE) != HAL_OK)//设置全局配置
	{
		Error_Handler();//进入硬件错误
	}
	
	if(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)//启动FIFO0中断
	{
		Error_Handler();//进入硬件错误
	}
	
	HAL_FDCAN_Start(&hfdcan1);
}

void FDCAN2_RxFilter_Config(void)
{
	FDCAN_FilterTypeDef sFilterConfig;
	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 1;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	sFilterConfig.FilterID1 = 0x00000000;
	sFilterConfig.FilterID2 = 0X1FFFFFFF;
	if(HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	
	if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_ACCEPT_IN_RX_FIFO1,FDCAN_ACCEPT_IN_RX_FIFO1,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE) != HAL_OK)//设置全局配置
	{
		Error_Handler();//进入硬件错误
	}
	
	if(HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)//启动FIFO1中断
	{
		Error_Handler();//进入硬件错误
	}
	
	HAL_FDCAN_Start(&hfdcan2);
}

//void FDCAN3_RxFilter_Config(void)
//{
//	FDCAN_FilterTypeDef sFilterConfig;
//	/* Configure Rx filter */
//	sFilterConfig.IdType = FDCAN_STANDARD_ID;
//	sFilterConfig.FilterIndex = 0;
//	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
//	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
//	sFilterConfig.FilterID1 = 0x00000000;
//	sFilterConfig.FilterID2 = 0X1FFFFFFF;
//	if(HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK)
//	{
//		Error_Handler();
//	}
//	
//	if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE) != HAL_OK)//设置全局配置
//	{
//		Error_Handler();//进入硬件错误
//	}
//	
//	if(HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)//启动FIFO0中断
//	{
//		Error_Handler();//进入硬件错误
//	}
//	
//	HAL_FDCAN_Start(&hfdcan3);
//}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  //__HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_FLAG_BUS_OFF);
    if(hfdcan->Instance == FDCAN1)
    {
        MX_FDCAN1_Init();
    } 
    else if(hfdcan->Instance == FDCAN2)
    {
        MX_FDCAN2_Init();
    }  
    else 
    {
        
    }
}




