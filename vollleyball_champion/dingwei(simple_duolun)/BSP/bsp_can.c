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
uint8_t can_data_num_g=0;//结构体数组can_database_g的大小，在Hash_table_init(void)中更新
Can_Data can_database_g[]={//通信表，odrive和vesc的依赖项。新加ID时，在ID_NUMDEF中定义相应ID的意义
//    //Data_type            Data_ID             *Data_ptr                                   		Data_length  	*MenuFunc   Channel		Fifo_num
//		{WRITE_ONLY,      vesc_motor1,         (uint8_t*)(&vesc_content_transform[1].u8_data),       4,          NULL,        2,		FDCAN_RX_FIFO0},
//		{WRITE_ONLY,      vesc_motor2,         (uint8_t*)(&vesc_content_transform[2].u8_data),       4,          NULL,        2,		FDCAN_RX_FIFO0},
//        {WRITE_ONLY,      vesc_motor3,         (uint8_t*)(&vesc_content_transform[3].u8_data),       4,          NULL,        2,		FDCAN_RX_FIFO0}
};
uint16_t hash_table[1000]={999};
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
	sFilterConfig.FilterIndex = 0;
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

/*
1.函数功能：初始化通信hash表
2.入参：none
3.返回值：none
4.用法及调用要求：在使用vesc和odrive之前调用此函数
5.其它：
*/
void Hash_table_init(void){ 
	int i;
	can_data_num_g = sizeof(can_database_g) / sizeof(can_database_g[0]);
	for(i=0;i<1000;i++){
		hash_table[i] = 999;
	}
	for(i=0;i<can_data_num_g;i++){
		hash_table[can_database_g[i].Data_ID] = i;
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)//dji电机
{
//    if(hfdcan==&hfdcan3)
//	{
//		USART_printf("3");
//		static int cnt[8]={0};
//		FDCAN_RxHeaderTypeDef rx_header;
//		uint8_t rx_data[8];
//		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
//		switch (rx_header.Identifier){
//			case CAN_3508_M1_ID:	
//			case CAN_3508_M2_ID:
//			case CAN_3508_M3_ID:
//			case CAN_3508_M4_ID:{
//                Dji_3508_first_four_motor_control(rx_header.Identifier-CAN_3508_M1_ID,rx_data);
//				break;
//            }
//            default:
//                break;
//        }
        /*以下为接收vesc传回5065电机相关报文的语句*/
//        switch (rx_header.Identifier)
//		{
//            case CAN_5065_M1_ID:
//            case CAN_5065_M2_ID:
//            case CAN_5065_M3_ID:
//			{//之后测试成功后可以考虑将vesc报文的发送函数也放到此处，现为测试对于发回报文的解析是否正确，暂只将报文中速度的解析函数放在此处///////////////////////////////////////////////////////
//                get_vesc_speed(rx_header.Identifier-CAN_5065_M1_ID+1,rx_data);
//                break;
//            }
//            default:
//                break;
//        }
//    }
    if(hfdcan==&hfdcan1)
	{
//		USART_printf("1");
		static int cnt[8]={0};
		FDCAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
//        switch (rx_header.Identifier){
//			case CAN_3508_M1_ID:	
//			case CAN_3508_M2_ID:
//			case CAN_3508_M3_ID:
//			case CAN_3508_M4_ID:{
//					Dji_3508_first_motor_control(rx_header.Identifier-CAN_3508_M1_ID,rx_data);
//					break;
//			}
//			default:
//				break;
//        }
//        switch (rx_header.Identifier)
//		{
//            case CAN_5065_M1_ID:
//            case CAN_5065_M2_ID:
//            case CAN_5065_M3_ID:
//			{//之后测试成功后可以考虑将vesc报文的发送函数也放到此处，现为测试对于发回报文的解析是否正确，暂只将报文中速度的解析函数放在此处///////////////////////////////////////////////////////
//                get_vesc_speed(rx_header.Identifier-CAN_5065_M1_ID+1,rx_data);
//                break;
//            }
//            default:
//                break;
//        }
	}
}


//void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
//{
//	if(hfdcan==&hfdcan2)
//	{
//		USART_printf("2");
//		FDCAN_RxHeaderTypeDef rx_header;
//    	uint8_t rx_data[8];
//    	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data);
////				switch (rx_header.Identifier){
////			case CAN_3508_M1_ID:	
////			case CAN_3508_M2_ID:
////			case CAN_3508_M3_ID:
////			case CAN_3508_M4_ID:{
////                Dji_3508_first_motor_control(rx_header.Identifier-CAN_3508_M1_ID,rx_data);
////				break;
////            }
////            default:
////                break;
////        }
////        if(rx_header.StdId==CAN_3508_M8_ID)
////        {
////            Dji_3508_last_motor_control(rx_header.StdId-CAN_3508_M1_ID,rx_data);
////        }
////        if(rx_header.StdId==CAN_3508_M6_ID)//用6号id代替9号电机
////        {
////            Dji_3508_last_motor_control(rx_header.StdId-CAN_3508_M1_ID+3,rx_data);
////        }
////        if(rx_header.StdId==CAN_3508_M4_ID)
////        {
////            Dji_3508_first_motor_control(rx_header.StdId-CAN_3508_M1_ID,rx_data);
////        }
////        switch (rx_header.Identifier)
////		{
////            case CAN_5065_M1_ID:
////            case CAN_5065_M2_ID:
////            case CAN_5065_M3_ID:
////			{//之后测试成功后可以考虑将vesc报文的发送函数也放到此处，现为测试对于发回报文的解析是否正确，暂只将报文中速度的解析函数放在此处///////////////////////////////////////////////////////
////                get_vesc_speed(rx_header.Identifier-CAN_5065_M1_ID+1,rx_data);
////                break;
////            }
////            default:
////                break;
////        }
//	}
//}
