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
    // === 配置三个舵向电机 (ID 1, 2, 3) ===
    // 假设它们都接在 FDCAN2 上，且 ID 分别为 0x201, 0x202, 0x203
    
    // Motor 0 (Wheel 1)
    dji_motor_configure(0, CAN_3508_2006_M1_ID, &hfdcan1, 
                       0.384706f,	0.012333f,	0.004745f,     // 位置环 PID
                       26.7f,	0.98f,	0.024314f,     // 速度环 PID
                        12000, -12000, 22000, -22000); // 输出限幅

    // Motor 1 (Wheel 2)
    dji_motor_configure(1, CAN_3508_2006_M2_ID, &hfdcan1, 
                       0.384706f,	0.012333f,	0.004745f,     // 位置环 PID
                       26.7f,	0.98f,	0.024314f,     // 速度环 PID
                        12000, -12000, 22000, -22000); // 输出限幅

    // Motor 2 (Wheel 3)
    dji_motor_configure(2, CAN_3508_2006_M3_ID, &hfdcan1, 
                       0.384706f,	0.012333f,	0.004745f,     // 位置环 PID
                       26.7f,	0.98f,	0.024314f,     // 速度环 PID
                        12000, -12000, 22000, -22000); // 输出限幅
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
// 3. 执行独立 PID 计算
    if (motor->control_mode == MOTOR_MODE_POSITION)
    {
        // 1. 计算当前绝对误差
        float err = (float)(motor->target_loc - motor->measure.total_angle);
        
        // 2. 计算误差的变化率 (D项，核心刹车组件)
        float d_err = err - motor->loc_pid.err_last;
        motor->loc_pid.err_last = err; // 记录本次误差，供下次使用

        // 3. 位置式 PD 计算：P负责瞬间起步，D负责快到终点时反向刹车
        float speed_target = (motor->loc_pid.Kp * err) + (motor->loc_pid.Kd * d_err);
        
        // 4. 目标速度强力限幅 (保持你原来的设定)
        if (speed_target > motor->loc_pid.out_limit_up) speed_target = motor->loc_pid.out_limit_up;
        if (speed_target < motor->loc_pid.out_limit_down) speed_target = motor->loc_pid.out_limit_down;

        // 5. 将计算出的目标速度喂给内环（速度环依然用增量式，没问题）
        Pid_incremental_cal(&motor->spd_pid, motor->measure.speed_rpm, speed_target);
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

static void dji_motor_send_commands(DJI_Motor_Instance* trigger_motor)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    int16_t out[DJI_MOTOR_COUNT];

    for(int i=0; i < DJI_MOTOR_COUNT; i++) {
        out[i] = dji_motor_control_enabled ? (int16_t)dji_motors[i].spd_pid.now_out : 0;
    }

    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;
    
    // 发送 ID 0x200 (控制 1-4 号电机)
    // 简单判断：只要是前4个电机触发的，或者是 NULL (全部发送)，就发这一帧
    if (trigger_motor == NULL || (trigger_motor >= &dji_motors[0] && trigger_motor <= &dji_motors[3]))
    {
        tx_header.Identifier = CAN_ID_FIRST_FOUR_MOTORS;
        tx_data[0] = (uint8_t)(out[0] >> 8); tx_data[1] = (uint8_t)out[0];
        tx_data[2] = (uint8_t)(out[1] >> 8); tx_data[3] = (uint8_t)out[1];
        tx_data[4] = (uint8_t)(out[2] >> 8); tx_data[5] = (uint8_t)out[2];
        tx_data[6] = (uint8_t)(out[3] >> 8); tx_data[7] = (uint8_t)out[3];

        FDCAN_HandleTypeDef* hcan = (trigger_motor) ? trigger_motor->hfdcan : dji_motors[0].hfdcan;
        if(hcan) HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, tx_data);
    }
    
    // 如需控制 5-8 号电机，可在此处添加 CAN_ID_LAST_FOUR_MOTORS (0x1FF) 的逻辑
}