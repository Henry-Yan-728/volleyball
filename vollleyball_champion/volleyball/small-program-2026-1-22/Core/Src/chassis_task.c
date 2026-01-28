#include "chassis_task.h"
#include "fdcan_bsp.h" 
#include <math.h>
#include <stdio.h>
// 引用 CAN1 句柄
extern FDCAN_HandleTypeDef hfdcan1;

// 辅助联合体：用于 float 转 字节
typedef union {
    float f;
    uint8_t b[4];
} FloatByte_t;

// =============================================================
//  内部函数：计算并发送指令
// =============================================================

uint8_t FDCAN1_Transmit(uint8_t *TxData, uint32_t id, uint32_t len, uint8_t EXTflag)
{
	FDCAN_TxHeaderTypeDef TxMessage;

	TxMessage.Identifier 			= id;					/* 设置发送帧ID */
	TxMessage.IdType				= EXTflag ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
	TxMessage.TxFrameType 			= FDCAN_DATA_FRAME;		/* 数据帧 */
	TxMessage.DataLength 			= len;					/* CAN FD数据长度（DLC宏） */
	TxMessage.ErrorStateIndicator 	= FDCAN_ESI_ACTIVE;	
	TxMessage.BitRateSwitch 		= FDCAN_BRS_ON;			/* CAN FD必须开启BRS */
	TxMessage.FDFormat 				= FDCAN_FD_CAN;			/* 启用CAN FD格式（支持>8字节） */
	TxMessage.TxEventFifoControl 	= FDCAN_NO_TX_EVENTS;	
	TxMessage.MessageMarker 		= 0;

	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxMessage, TxData) != HAL_OK)
	{
		return 1;
	}

	return 0;
}
void Chassis_Update(float vx, float vy, float vr)
{
	uint8_t TxMessage[12];
FloatByte_t float_buf; 
		uint32_t v_x= *(uint32_t*)(&vx);
		uint32_t v_y= *(uint32_t*)(&vy);
		uint32_t v_r= *(uint32_t*)(&vr);
		TxMessage[0]  = (v_x >> 24) & 0xFF;  // 提取24~31位
		TxMessage[1]  = (v_x >> 16) & 0xFF;  // 提取16~23位
		TxMessage[2] = (v_x >> 8)  & 0xFF;  // 提取8~15位
		TxMessage[3] = v_x & 0xFF;
		TxMessage[4]  = (v_y >> 24) & 0xFF;  // 提取24~31位
		TxMessage[5]  = (v_y >> 16) & 0xFF;  // 提取16~23位
		TxMessage[6] = (v_y >> 8)  & 0xFF;  // 提取8~15位
		TxMessage[7] = v_y & 0xFF;
		TxMessage[8]  = (v_r >> 24) & 0xFF;  // 提取24~31位
		TxMessage[9]  = (v_r >> 16) & 0xFF;  // 提取16~23位
		TxMessage[10] = (v_r >> 8)  & 0xFF;  // 提取8~15位
		TxMessage[11] = v_r & 0xFF;
		FDCAN1_Transmit(TxMessage, 0x11, 12, 1);
		
}