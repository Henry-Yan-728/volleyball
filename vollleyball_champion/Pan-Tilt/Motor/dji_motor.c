/**
  ******************************************************************************
  * @file           : dji_motor.c
  * @brief          : DJI电机驱动 (独立闭环版本)
  ******************************************************************************
  */

#include "dji_motor.h"
#include "fdcan_bsp.h" 
#include <stdbool.h>
#include <string.h>

/* ------------------------- Private Defines ------------------------- */
#define CAN_ID_FIRST_FOUR_MOTORS  0x200
#define CAN_ID_LAST_FOUR_MOTORS   0x1FF
#define DJI_ENCODER_RESOLUTION    8192

/* ------------------------- Static Variables ------------------------- */
static DJI_Motor_Instance dji_motors[DJI_MOTOR_COUNT];
static bool dji_motor_control_enabled = true;

/* ------------------------- Function Prototypes ------------------------- */
static void get_motor_measure(DJI_Motor_Instance* motor, uint8_t rx_data[8]);
static void get_total_angle(DJI_Motor_Instance* motor);
static void dji_motor_send_commands(DJI_Motor_Instance* trigger_motor);
static void dji_motor_configure(uint8_t index, const uint16_t id, FDCAN_HandleTypeDef* hfdcan, 
                                float loc_kp, float loc_ki, float loc_kd,
                                float spd_kp, float spd_ki, float spd_kd,
                                float loc_out_limit_up, float loc_out_limit_down,
                                float spd_out_limit_up, float spd_out_limit_down);

/* ------------------------- Public Functions ------------------------- */

int32_t dji_degree2encoder(float degree)
{
    float ratio = (float)DJI_ENCODER_RESOLUTION / 360.0f;
    return (int32_t)(degree * ratio);
}

void dji_motors_init(void)
{
    // === 1. 配置 Yaw 轴 (GM6020) ===
    // 假设：使用 FDCAN1，电机物理ID设为5 (Rx: 0x205)，对应控制帧 0x1FF
    dji_motor_configure(GIMBAL_MOTOR_YAW_ID, CAN_ID_M5, &hfdcan1, 
                        // 位置环 (6020直驱需要较大的P，较小的I防止震荡)
                        0.5f,  0.0f, 0.8f,     
                        // 速度环 (力矩响应)
                        40.0f, 0.8f, 0.0f,     
                        5000, -5000, 10000, -10000); // 6020最大电流约30000

    // === 2. 配置 Pitch 轴 (M2006) ===
    // 假设：使用 FDCAN1，电机物理ID设为1 (Rx: 0x202)，对应控制帧 0x200
    dji_motor_configure(GIMBAL_MOTOR_PITCH_ID, CAN_ID_M2, &hfdcan1, 
                        // 位置环 (2006减速电机，位置环P可以小一点，依靠速度环)
                        0.15f, 0.0f, 0.8f,     // 位置环 PID
                        10.0f, 0.1f, 0.0f,     // 速度环 PID
                        3000, -3000, 6000, -6000); // 输出限幅
}

DJI_Motor_Instance* dji_motor_get_instance(uint8_t motor_index)
{
    if (motor_index >= DJI_MOTOR_COUNT) return NULL;
    return &dji_motors[motor_index];
}

void dji_motor_set_location(DJI_Motor_Instance* motor, int32_t location)
{
    if (motor == NULL) return;
    motor->control_mode = MOTOR_MODE_POSITION;
    motor->target_loc = location;
}

void dji_motor_set_speed(DJI_Motor_Instance* motor, int16_t speed)
{
    if (motor == NULL) return;
    motor->control_mode = MOTOR_MODE_SPEED;
    motor->target_speed = speed;
}

/* ------------------------- Callback / Handler ------------------------- */

// 统一的消息处理回调 (核心修改：独立 PID 计算)
void dji_motor_message_handler(void* instance, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[8])
{
    if (instance == NULL) return;
    DJI_Motor_Instance* motor = (DJI_Motor_Instance*)instance;

    // 1. 解析数据
    get_motor_measure(motor, rx_data);
    
    // 2. 上电初始化锁定 (防止疯转)
    if (motor->is_online == 0)
    {
        get_total_angle(motor); 
        motor->target_loc = motor->measure.total_angle; // 锁定当前位置
        motor->is_online = 1;
        motor->last_msg_time = HAL_GetTick();
        return;
    }
    
    motor->is_online = 1;
    motor->last_msg_time = HAL_GetTick();
    get_total_angle(motor);

    // 3. 执行独立 PID 计算
	if (motor->control_mode == MOTOR_MODE_POSITION) {
    // 1. 位置环：输入角度误差，输出目标转速
    // 注意：若 Pid_incremental_cal 内部没有累加，这里需要处理
    float speed_target = Pid_incremental_cal(&motor->loc_pid, (float)motor->measure.total_angle, (float)motor->target_loc);
    
    // 2. 速度环：输入转速误差，输出电流强度
    Pid_incremental_cal(&motor->spd_pid, (float)motor->measure.speed_rpm, speed_target);
	}
	else // SPEED MODE
    {
        Pid_incremental_cal(&motor->spd_pid, motor->measure.speed_rpm, (float)motor->target_speed);
    }

    // 4. 发送指令 (无需等待其他电机)
    dji_motor_send_commands(motor);
}

/* ------------------------- Private Helper Functions ------------------------- */

static void dji_motor_configure(uint8_t index, const uint16_t id, FDCAN_HandleTypeDef* hfdcan, 
                                float loc_kp, float loc_ki, float loc_kd,
                                float spd_kp, float spd_ki, float spd_kd,
                                float loc_out_limit_up, float loc_out_limit_down,
                                float spd_out_limit_up, float spd_out_limit_down)
{
    if (index >= DJI_MOTOR_COUNT) return;
    DJI_Motor_Instance* motor = &dji_motors[index];

    motor->can_id = id;
    motor->hfdcan = hfdcan;
    motor->control_mode = MOTOR_MODE_POSITION;

    motor->loc_pid.Kp = loc_kp; motor->loc_pid.Ki = loc_ki; motor->loc_pid.Kd = loc_kd;
    motor->loc_pid.out_limit_up = loc_out_limit_up; motor->loc_pid.out_limit_down = loc_out_limit_down;

    motor->spd_pid.Kp = spd_kp; motor->spd_pid.Ki = spd_ki; motor->spd_pid.Kd = spd_kd;
    motor->spd_pid.out_limit_up = spd_out_limit_up; motor->spd_pid.out_limit_down = spd_out_limit_down;

    FDCAN_Dispatch_t dispatch_item;
    dispatch_item.id_type = FDCAN_STANDARD_ID;
    dispatch_item.id = motor->can_id;
    dispatch_item.instance_ptr = motor;
    dispatch_item.handler = dji_motor_message_handler;

    fdcan_bsp_register(&dispatch_item, hfdcan);
}

static void get_motor_measure(DJI_Motor_Instance* motor, uint8_t rx_data[8])
{
    motor->measure.last_angle = motor->measure.angle;
    motor->measure.angle = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
    motor->measure.speed_rpm = (int16_t)(rx_data[2] << 8 | rx_data[3]);
    motor->measure.given_current = (int16_t)(rx_data[4] << 8 | rx_data[5]);
    motor->measure.temperate = rx_data[6];
}

static void get_total_angle(DJI_Motor_Instance* motor)
{
    int32_t delta = motor->measure.angle - motor->measure.last_angle;
    if (delta > (DJI_ENCODER_RESOLUTION / 2)) delta -= DJI_ENCODER_RESOLUTION;
    else if (delta < -(DJI_ENCODER_RESOLUTION / 2)) delta += DJI_ENCODER_RESOLUTION;
    motor->measure.total_angle += delta;
}

/**
 * @brief 发送控制指令 (核心修改)
 * @details 自动判断需要发送 0x200 还是 0x1FF，或者都发送
 */
static void dji_motor_send_commands(DJI_Motor_Instance* trigger_motor)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    int16_t out_group1[4] = {0}; // 对应 0x200 (ID 1-4)
    int16_t out_group2[4] = {0}; // 对应 0x1FF (ID 5-8)
    
    bool need_send_group1 = false;
    bool need_send_group2 = false;

    // 遍历所有电机，填充数据到对应的组
    for(int i=0; i < DJI_MOTOR_COUNT; i++) {
        if (!dji_motors[i].is_online) continue; // 掉线不发送

        int16_t current = dji_motor_control_enabled ? (int16_t)dji_motors[i].spd_pid.now_out : 0;
        
        switch (dji_motors[i].can_id)
        {
            case CAN_ID_M1: out_group1[0] = current; need_send_group1 = true; break;
            case CAN_ID_M2: out_group1[1] = current; need_send_group1 = true; break;
            case CAN_ID_M3: out_group1[2] = current; need_send_group1 = true; break;
            case CAN_ID_M4: out_group1[3] = current; need_send_group1 = true; break;
            
            case CAN_ID_M5: out_group2[0] = current; need_send_group2 = true; break;
            case CAN_ID_M6: out_group2[1] = current; need_send_group2 = true; break;
            case CAN_ID_M7: out_group2[2] = current; need_send_group2 = true; break;
            case CAN_ID_M8: out_group2[3] = current; need_send_group2 = true; break;
        }
    }

    // 配置通用头信息
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    // 发送 Group 1 (0x200) - 适用于 2006 (ID 1)
    if (need_send_group1) {
        tx_header.Identifier = CAN_ID_FIRST_FOUR_MOTORS;
        tx_data[0] = (uint8_t)(out_group1[0] >> 8); tx_data[1] = (uint8_t)out_group1[0];
        tx_data[2] = (uint8_t)(out_group1[1] >> 8); tx_data[3] = (uint8_t)out_group1[1];
        tx_data[4] = (uint8_t)(out_group1[2] >> 8); tx_data[5] = (uint8_t)out_group1[2];
        tx_data[6] = (uint8_t)(out_group1[3] >> 8); tx_data[7] = (uint8_t)out_group1[3];
        // 假设所有云台电机都在同一个CAN总线上，这里取第一个电机的句柄
        if(dji_motors[0].hfdcan) HAL_FDCAN_AddMessageToTxFifoQ(dji_motors[0].hfdcan, &tx_header, tx_data);
    }

    // 发送 Group 2 (0x1FF) - 适用于 6020 (ID 5)
    if (need_send_group2) {
        tx_header.Identifier = CAN_ID_LAST_FOUR_MOTORS;
        tx_data[0] = (uint8_t)(out_group2[0] >> 8); tx_data[1] = (uint8_t)out_group2[0];
        tx_data[2] = (uint8_t)(out_group2[1] >> 8); tx_data[3] = (uint8_t)out_group2[1];
        tx_data[4] = (uint8_t)(out_group2[2] >> 8); tx_data[5] = (uint8_t)out_group2[2];
        tx_data[6] = (uint8_t)(out_group2[3] >> 8); tx_data[7] = (uint8_t)out_group2[3];
        if(dji_motors[0].hfdcan) HAL_FDCAN_AddMessageToTxFifoQ(dji_motors[0].hfdcan, &tx_header, tx_data);
    }
}