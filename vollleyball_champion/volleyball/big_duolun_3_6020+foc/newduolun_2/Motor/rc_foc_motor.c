#include "rc_foc_motor.h"
#include "main.h"
#include "pid.h"
#include "fdcan.h"
#include "basic.h"
#include "can_database.h"

float_to_u8 blazer_content_transform[Motor_Num];

CAN_Data_TypeDef CAN_Data[Motor_Num]=
{
	{0x00, (uint8_t*)(&blazer_content_transform[0].u8_data)},
	{0x01, (uint8_t*)(&blazer_content_transform[1].u8_data)},
	{0x02, (uint8_t*)(&blazer_content_transform[2].u8_data)},
	{0x03, (uint8_t*)(&blazer_content_transform[3].u8_data)},
	{0x04, (uint8_t*)(&blazer_content_transform[4].u8_data)},
	{0x05, (uint8_t*)(&blazer_content_transform[5].u8_data)},
	{0x06, (uint8_t*)(&blazer_content_transform[6].u8_data)},
	{0x07, (uint8_t*)(&blazer_content_transform[7].u8_data)},
};

uint32_t FloatToIntBit(float x)
{
	uint32_t *pInt;
	pInt = (uint32_t*)(&x);
	return *pInt;
}

float IntBitToFloat(uint32_t x)
{
	float * pFloat;
	pFloat = (float *)(&x);
	return *pFloat;
}

/**
   * @brief  CAN1过滤器初始化函数
   * @param  无
   * @retval 无
   */
void CAN_Filter_Init(void)
{

	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x00000000;
	sFilterConfig.FilterID2 = 0X1FFFFFFF;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
}

/**
   * @brief  CAN发送报文
   * @param  ID 由节点ID和参数ID构成的复合ID
   * @retval 无
   */
void CAN_SendMessage(uint32_t ID)
{
	FDCAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint8_t send_num = 0;
	uint32_t pTxMailbox = 0;
	
	CAN_TxHeaderStruct.Identifier			  = ID;
	CAN_TxHeaderStruct.IdType			  = FDCAN_EXTENDED_ID ;
	CAN_TxHeaderStruct.TxFrameType 			= FDCAN_DATA_FRAME;		/* 数据帧 */
	CAN_TxHeaderStruct.DataLength 			= 0X04;					/* 设置数据长度 */
	CAN_TxHeaderStruct.ErrorStateIndicator 	= FDCAN_ESI_ACTIVE;		/* 设置错误状态指 */
	CAN_TxHeaderStruct.BitRateSwitch 		= FDCAN_BRS_OFF;		/* 关闭可变波特率 */
	CAN_TxHeaderStruct.FDFormat 				= FDCAN_CLASSIC_CAN;			/* FDCAN格式 */
	CAN_TxHeaderStruct.TxEventFifoControl 	= FDCAN_NO_TX_EVENTS;	/* 用于发送事件FIFO控制, 无发送事件*/
	CAN_TxHeaderStruct.MessageMarker 		= 0;					/* 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0-0xFF */
	
	while(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CAN_TxHeaderStruct, CAN_Data[ID >> 8].data_ptr) != HAL_OK)
	{
		delay_us(25);
		/*发送阻滞*/
		if(++ send_num == 10)
			break;
	}
}

/**
   * @brief  设定运行模式
   * @param  node_id   驱动节点ID 
			 mode      模式 
			 0:闲置状态 	1:电流模式 2:速度模式 3:位置模式 4:校准电阻电感磁链 
		     5:校准编码器偏移 6:恢复默认配置 7:保存当前配置 8:清除错误
   * @retval 无
   */
void set_blazer_mode(uint32_t node_id, float mode)
{
	uint32_t blazer_mode_id = node_id << 8 | CAN_SET_MODE;
	uint32_t mode_u32;
	
	mode_u32 = FloatToIntBit(mode);
	
	blazer_content_transform[node_id].u8_data[0] = mode_u32 >> 24;
	blazer_content_transform[node_id].u8_data[1] = mode_u32 >> 16;
	blazer_content_transform[node_id].u8_data[2] = mode_u32 >> 8;
	blazer_content_transform[node_id].u8_data[3] = mode_u32;
	
	CAN_SendMessage(blazer_mode_id);
}

/**
   * @brief  设定电机转速
			 转速为机械转速
   * @param  node_id 驱动节点ID 
             speed   设定速度(r/s)
   * @retval 无
   */
void set_blazer_speed(uint32_t node_id, float speed)
{
	uint32_t blazer_speed_id = node_id << 8 | CAN_SET_SPEED;
	uint32_t speed_u32;
	
	speed_u32 = FloatToIntBit(speed);
	
	blazer_content_transform[node_id].u8_data[0] = speed_u32 >> 24;
	blazer_content_transform[node_id].u8_data[1] = speed_u32 >> 16;
	blazer_content_transform[node_id].u8_data[2] = speed_u32 >> 8;
	blazer_content_transform[node_id].u8_data[3] = speed_u32;
	
	CAN_SendMessage(blazer_speed_id);
}

/**
   * @brief  设定电机加速度
   * @param  node_id   驱动节点ID 
             speed_acc 设定加速度(r/(s*s))
   * @retval 无
   */
void set_blazer_speed_acc(uint32_t node_id, float speed_acc)
{
	uint32_t blazer_speed_acc_id = node_id << 8 | CAN_SET_ACC;
	uint32_t speed_acc_u32;
	
	speed_acc_u32 = FloatToIntBit(speed_acc);
	
	blazer_content_transform[node_id].u8_data[0] = speed_acc_u32 >> 24;
	blazer_content_transform[node_id].u8_data[1] = speed_acc_u32 >> 16;
	blazer_content_transform[node_id].u8_data[2] = speed_acc_u32 >> 8;
	blazer_content_transform[node_id].u8_data[3] = speed_acc_u32;
	
	CAN_SendMessage(blazer_speed_acc_id);
}

/**
   * @brief  设定电机减速度
   * @param  node_id   驱动节点ID 
             speed_dec 设定减速度(r/(s*s))
   * @retval 无
   */
void set_blazer_speed_dec(uint32_t node_id, float speed_dec)
{
	uint32_t blazer_speed_dec_id = node_id << 8 | CAN_SET_DEC;
	uint32_t speed_dec_u32;
	
	speed_dec_u32 = FloatToIntBit(speed_dec);
	
	blazer_content_transform[node_id].u8_data[0] = speed_dec_u32 >> 24;
	blazer_content_transform[node_id].u8_data[1] = speed_dec_u32 >> 16;
	blazer_content_transform[node_id].u8_data[2] = speed_dec_u32 >> 8;
	blazer_content_transform[node_id].u8_data[3] = speed_dec_u32;
	
	CAN_SendMessage(blazer_speed_dec_id);
}


/**
   * @brief  设定速度环kp
   * @param  node_id 驱动节点ID 
             kp 	 速度环kp
   * @retval 无
   */
void set_blazer_speed_kp(uint32_t node_id, float speed_kp)
{
	uint32_t blazer_speed_kp_id = node_id << 8 | CAN_SET_SPEED_KP;
	uint32_t speed_kp_u32;
	
	speed_kp_u32 = FloatToIntBit(speed_kp);
	
	blazer_content_transform[node_id].u8_data[0] = speed_kp_u32 >> 24;
	blazer_content_transform[node_id].u8_data[1] = speed_kp_u32 >> 16;
	blazer_content_transform[node_id].u8_data[2] = speed_kp_u32 >> 8;
	blazer_content_transform[node_id].u8_data[3] = speed_kp_u32;
	
	CAN_SendMessage(blazer_speed_kp_id);
}

/**
   * @brief  设定速度环ki
   * @param  node_id 驱动节点ID 
             kp 	 速度环ki
   * @retval 无
   */
void set_blazer_speed_ki(uint32_t node_id, float speed_ki)
{
	uint32_t blazer_speed_ki_id = node_id << 8 | CAN_SET_SPEED_KI;
	uint32_t speed_ki_u32;
	
	speed_ki_u32 = FloatToIntBit(speed_ki);
	
	blazer_content_transform[node_id].u8_data[0] = speed_ki_u32 >> 24;
	blazer_content_transform[node_id].u8_data[1] = speed_ki_u32 >> 16;
	blazer_content_transform[node_id].u8_data[2] = speed_ki_u32 >> 8;
	blazer_content_transform[node_id].u8_data[3] = speed_ki_u32;
	
	CAN_SendMessage(blazer_speed_ki_id);
}
