#include "vesc.h"
#include "fdcan_bsp.h" // 必须包含，用于获取 hfdcan 句柄和定义

/************** 配置区域 **************/

// 定义每个 VESC 电机挂载在哪个 FDCAN 总线上
// 下标 0-3 对应 motor_id 0-3
// 请根据实际硬件连接修改此处！
static FDCAN_HandleTypeDef* vesc_hfdcan_map[vesc_motor_nums] = {
    &hfdcan2, // Motor 0 对应 CAN2
    &hfdcan2, // Motor 1 对应 CAN2
    &hfdcan2, // Motor 2 对应 CAN2
    &hfdcan2  // Motor 3 对应 CAN2
};

/************** 内部变量与函数 **************/
union s32_to_u8 vesc_content_transform[vesc_motor_nums] = {0};
static int vesc_motor_poles_s[vesc_motor_nums] = {12, 12, 12, 12}; // 极对数，需根据电机修改
static int motor_speed_s[vesc_motor_nums] = {0};

// 内部发送函数声明
static void Vesc_Send_Msg(uint8_t motor_id, uint32_t frame_id, uint8_t* data);

/************** 外部接口实现 **************/

/**
 * @brief 初始化函数 (预留)
 */
void Vesc_init(void) {
    Vesc_speed_control_init();
}

/**
 * @brief 更改 VESC 目标速度缓存
 */
void Change_vesc_speed(int motor_id, int target_spd){
    if(motor_id >= vesc_motor_nums) return;
    motor_speed_s[motor_id] = target_spd;
	Com2vesc(motor_id);
}

/**
 * @brief 填充并发送 VESC 速度控制报文
 * @param motor_id 电机 ID (0 ~ vesc_motor_nums-1)
 */
void Com2vesc(uint32_t motor_id){
    if(motor_id >= vesc_motor_nums) return;

    // VESC ID 生成规则: Controller_ID | (Command << 8)
    // 注意：VESC CAN ID 是扩展帧 (29-bit)
    uint32_t vesc_speed_id = motor_id | (CAN_PACKET_SET_RPM << 8);
    
    // 计算电转速 ERPM = 机械转速 RPM * 极对数
    int32_t Erpm = motor_speed_s[motor_id] * vesc_motor_poles_s[motor_id];

    // 大端模式填充 (VESC 协议要求)
    vesc_content_transform[motor_id].u8_data[0] = (uint8_t)(Erpm >> 24);
    vesc_content_transform[motor_id].u8_data[1] = (uint8_t)(Erpm >> 16);
    vesc_content_transform[motor_id].u8_data[2] = (uint8_t)(Erpm >> 8);
    vesc_content_transform[motor_id].u8_data[3] = (uint8_t)(Erpm);

    // 调用底层发送
    Vesc_Send_Msg(motor_id, vesc_speed_id, vesc_content_transform[motor_id].u8_data);
}

/**
 * @brief 所有电调初始速度设为0
 */
void Vesc_speed_control_init(void){
    for (int i = 0; i < vesc_motor_nums; i++){
        motor_speed_s[i] = 0;
    }
}

/************** 内部函数实现 **************/

/**
 * @brief VESC 底层发送函数
 * @param motor_id 用于查找对应的 CAN 句柄
 * @param frame_id 完整的 VESC CAN ID (扩展帧)
 * @param data 4字节数据指针
 */
static void Vesc_Send_Msg(uint8_t motor_id, uint32_t frame_id, uint8_t* data)
{
    FDCAN_TxHeaderTypeDef TxMessage;
    
    // 1. 获取对应的 FDCAN 句柄
    if(motor_id >= vesc_motor_nums) return;
    FDCAN_HandleTypeDef* hfdcan = vesc_hfdcan_map[motor_id];
    
    if(hfdcan == NULL) return;

    // 2. 配置发送头 (VESC 必须使用扩展帧 FDCAN_EXTENDED_ID)
    TxMessage.Identifier = frame_id;
    TxMessage.IdType = FDCAN_EXTENDED_ID;     // [修正] VESC 使用扩展帧
    TxMessage.TxFrameType = FDCAN_DATA_FRAME;
    TxMessage.DataLength = FDCAN_DLC_BYTES_4; // VESC 设置 RPM 是 4 字节数据
    TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxMessage.BitRateSwitch = FDCAN_BRS_OFF;  // 通常不开启波特率切换，视情况而定
    TxMessage.FDFormat = FDCAN_CLASSIC_CAN;   // VESC 通常使用经典 CAN 2.0B
    TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxMessage.MessageMarker = 0;

    // 3. 发送数据 (尝试发送，带有简单的重试机制或直接发送)
    // 这里可以直接调用 HAL 库，也可以封装在 fdcan_bsp_send 中
    // 由于 fdcan_bsp_send 是通用封装，这里直接使用 HAL 库以确保对 Extended ID 的完全控制
    
    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data);
    }
    // else: FIFO 已满，丢包处理 (实时控制中通常丢弃旧包优于阻塞等待)
}

#if 0

void set_vesc_position(uint32_t ID,int32_t location){
	uint32_t vesc_position_id=ID+(CAN_PACKET_SET_POS<<8);
	location*=1000000;
	vesc_content_transform[ID].u8_data[0]=location>>24;
	vesc_content_transform[ID].u8_data[1]=location>>16;
	vesc_content_transform[ID].u8_data[2]=location>>8;
	vesc_content_transform[ID].u8_data[3]=location;
	Write_database_vesc_extid(vesc_position_id);
}

void set_vesc_current(uint32_t ID,int32_t current){
	uint32_t vesc_current_id=ID+(CAN_PACKET_SET_CURRENT<<8);
	current*=1000;
	vesc_content_transform[ID].u8_data[0]=current>>24;
	vesc_content_transform[ID].u8_data[1]=current>>16;
	vesc_content_transform[ID].u8_data[2]=current>>8;
	vesc_content_transform[ID].u8_data[3]=current;
	Write_database_vesc_extid(vesc_current_id);
}

#endif

