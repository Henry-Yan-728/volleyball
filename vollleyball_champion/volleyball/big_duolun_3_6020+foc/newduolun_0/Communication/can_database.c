/*
对于vesc和odrive以及通信用CAN信号
1.
2.
3.
*/
#include "fdcan.h"
#include "can_database.h"
#include "dji_6020_motor.h"
#include "vesc.h"
#include "wheel_control.h"
#include "rc_foc_motor.h"
#include "main.h"
/**************内部宏定义与重命名begin**************/

/**************内部宏定义与重命名end**************/

/**************内部变量与函数begin**************/
static void Can_filter_init(void);
/**************内部变量与函数end**************/
int32_t ReceiveVel;
int32_t ReceiveAngle;
int32_t ReceiveFlag;
/**************外部接口begin**************/
//VESC
uint8_t can_data_num_g_vesc=0;//结构体数组can_database_g_vesc的大小，在Hash_table_init(void)中更新
Can_Data_vesc can_database_g_vesc[]={//通信表，odrive和vesc的依赖项。新加ID时，在ID_NUMDEF中定义相应ID的意义
    //Data_type            Data_ID             *Data_ptr                                   Data_length   *MenuFunc   Channel       Fifo_num  
		{WRITE_ONLY,      vesc_motor1,         (uint8_t*)(&vesc_content_transform[1].u8_data),       4,          NULL,        1,      FDCAN_FILTER_TO_RXFIFO0 },
		//{WRITE_ONLY,      vesc_motor2,         (uint8_t*)(&vesc_content_transform[2].u8_data),       4,          NULL,        2,      CAN_FILTER_FIFO0},   
};
uint16_t hash_table[1000]={999};
//RC_FOC

void Hash_table_init(void);
void Can_start_work(void);


/**************外部接口end**************/

/*
1.函数功能：初始化通信hash表
2.入参：none
3.返回值：none
4.用法及调用要求：在使用vesc和odrive之前调用此函数
5.其它：
*/
void Hash_table_init(void){ 
	int i;
	can_data_num_g_vesc = sizeof(can_database_g_vesc) / sizeof(can_database_g_vesc[0]);//具体反应有几个vesc
	for(i=0;i<1000;i++){
		hash_table[i] = 999;
	}
	for(i=0;i<can_data_num_g_vesc;i++){
		hash_table[can_database_g_vesc[i].Data_ID] = i;
	}
}

/*
1.函数功能：初始化FD_CAN过滤器
2.入参：none
3.返回值：none
4.用法及调用要求：
5.其它：FDCAN通信，内容皆为FDCAN
*/
static void Can_filter_init(void){
	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x00000000;
	sFilterConfig.FilterID2 = 0X1FFFFFFF;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
	if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE) != HAL_OK)//设置全局配置
	{
		Error_Handler();//进入硬件错误
	}
	//严格来讲这个是FD_CAN的初始化
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	sFilterConfig.FilterID1 = 0x00000000;
	sFilterConfig.FilterID2 = 0X1FFFFFFF;
  HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);
	if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_ACCEPT_IN_RX_FIFO1,FDCAN_ACCEPT_IN_RX_FIFO1,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE) != HAL_OK)//设置全局配置
	{
		Error_Handler();//进入硬件错误
	}
	
}	

/*
1.函数功能：开启CAN并关联接收FIFO
2.入参：none
3.返回值：none
4.用法及调用要求：
5.其它：
*/
void Can_start_work(void){
		Can_filter_init();
	  HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
		HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
}
//can1与6020电机通信
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,uint32_t RxFifo0ITs)//dji电机
{
	if(hfdcan==&hfdcan1){
		static int cnt[8]={0};
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
		switch (rx_header.Identifier){
			case CAN_6020_M1_ID:	
			{
					Dji_6020_motor_control(rx_header,rx_data);//can1与6020通信
					break;
			}
			default:
				break;
	  }
	}
}
//can2与主控板间通信，主要获取目标的角度值与速度值
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,uint32_t RxFifo1ITs)
{
	if(hfdcan==&hfdcan2){
		FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data);
		if(rx_header.Identifier==0x11)
		{
			ReceiveVel = (rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3];
			vel= *((float*)&ReceiveVel);//指针强制转化为float类型
			ReceiveAngle = (rx_data[4] << 24) | (rx_data[5] << 16) | (rx_data[6] << 8) | rx_data[7];
			target_angle_pos= *((float*)&ReceiveAngle);
		}
	}
}

void HAL_CAN_ErrorCallback(FDCAN_HandleTypeDef *hcan)
{
     //hcan->ErrorCode  即可获取相应的ESR寄存器的值   进入这里就表示一定产生错误中断了，而且是BUSOFF中断
}

