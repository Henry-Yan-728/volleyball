#include "dji_motor.h"
#include "fdcan_bsp.h" 
#include <stdbool.h>
#include <string.h>
#ifdef USE_FREERTOS
    #include "FreeRTOS.h"
    #include "task.h" 
#endif

/* ------------------------- Private Defines ------------------------- */
#define CAN_ID_FIRST_FOUR_MOTORS  0x200
#define CAN_ID_LAST_FOUR_MOTORS   0x1FF

/* ------------------------- Static Variables ------------------------- */
static DJI_Motor_Instance dji_motors[DJI_MOTOR_COUNT];
static volatile bool dji_motor_control_enabled = true;

/* ------------------------- Private Function Prototypes ------------------------- */
static void get_motor_measure(DJI_Motor_Instance* motor, uint8_t rx_data[8]);
static void get_total_angle(DJI_Motor_Instance* motor);
static void dji_motor_send_commands(void);
static void dji_motor_configure(uint8_t index, const uint16_t id, FDCAN_HandleTypeDef* hfdcan, 
                                float loc_kp, float loc_ki, float loc_kd,
                                float spd_kp, float spd_ki, float spd_kd,
                                float loc_out_limit_up, float loc_out_limit_down,
                                float spd_out_limit_up, float spd_out_limit_down);
static void dji_critical_section_enter(void);
static void dji_critical_section_exit(void);

/* ------------------------- Standard Conversion Functions ------------------------- */

int32_t dji_angle_to_encoder(float angle_deg, float reduction_ratio)
{
    // 公式: (角度 / 360) * (编码器单圈分辨率 * 减速比)
    float total_counts_per_round = DJI_ENCODER_RESOLUTION * reduction_ratio;
    return (int32_t)((angle_deg / 360.0f) * total_counts_per_round);
}

float dji_encoder_to_angle(int32_t encoder_val, float reduction_ratio)
{
    // 公式: (编码器值 / (编码器单圈分辨率 * 减速比)) * 360
    float total_counts_per_round = DJI_ENCODER_RESOLUTION * reduction_ratio;
    return ((float)encoder_val / total_counts_per_round) * 360.0f;
}

/* ------------------------- Public Function Implementations ------------------------- */

// 旧接口保留兼容性，默认 1:1
int32_t dji_degree2encoder(float degree)
{
    return dji_angle_to_encoder(degree, 1.0f);
}

void dji_motors_init(void)
{
    // 示例电机配置 (FDCAN2)
    memset(dji_motors, 0, sizeof(dji_motors));
    dji_motor_control_enabled = true;

    dji_motor_configure(DJI_INDEX_CHASSIS_STEER_0, CAN_3508_2006_M1_ID, &hfdcan2, 
    0.384706f, 0.006333f, 0.004745f,
    26.7f, 0.98f, 0.024314f,
    12000.0f, -12000.0f, 22000.0f, -22000.0f);
    dji_motor_configure(DJI_INDEX_CHASSIS_STEER_1, CAN_3508_2006_M2_ID, &hfdcan2, 
    0.384706f, 0.006333f, 0.004745f,
                            26.7f, 0.98f, 0.024314f,
                            12000.0f, -12000.0f, 22000.0f, -22000.0f);
    dji_motor_configure(DJI_INDEX_CHASSIS_STEER_2, CAN_3508_2006_M3_ID, &hfdcan2,
    0.384706f, 0.006333f, 0.004745f,
                            26.7f, 0.98f, 0.024314f,
                            12000.0f, -12000.0f, 22000.0f, -22000.0f);
    // Yaw轴 (GM6020)
    dji_motor_configure(GIMBAL_MOTOR_YAW_ID, CAN_3508_2006_M5_ID, &hfdcan2, 
                        0.5f,  0.0f, 0.8f,     
                        40.0f, 0.8f, 0.0f,     
                        5000, -5000, 10000, -10000);

    // Pitch轴 (M2006)
    dji_motor_configure(GIMBAL_MOTOR_PITCH_ID, CAN_3508_2006_M6_ID, &hfdcan2, 
                        0.15f, 0.0f, 0.8f, 
                        20.0f, 0.1f, 0.0f, 
                        12000, -12000, 16000, -16000);

    dji_motor_configure(DJI_INDEX_AXIS_RIGHT, CAN_3508_2006_M7_ID, &hfdcan2,
                        0.2f, 0.001f, 0.035f,
                        24.0f, 1.2f, 0.03f,
                        1200, -1200, 12000, -12000);
    dji_motor_configure(DJI_INDEX_AXIS_LEFT, CAN_3508_2006_M8_ID, &hfdcan2,
                        0.2f, 0.001f, 0.035f,
                        24.0f, 1.2f, 0.03f,
                        1200, -1200, 12000, -12000);
}

void dji_motors_register_dispatches(void)
{
    for (uint8_t i = 0; i < DJI_MOTOR_COUNT; i++)
    {
        DJI_Motor_Instance* motor = &dji_motors[i];
        FDCAN_Dispatch_t dispatch_item;

        if ((motor->hfdcan == NULL) || (motor->can_id == 0U))
        {
            continue;
        }

        dispatch_item.id_type = FDCAN_STANDARD_ID;
        dispatch_item.id = motor->can_id;
        dispatch_item.mask = 0U;
        dispatch_item.instance_ptr = motor;
        dispatch_item.handler = dji_motor_message_handler;

        fdcan_bsp_register(&dispatch_item, motor->hfdcan);
    }
}

DJI_Motor_Instance* dji_motor_get_instance(uint8_t motor_index)
{
    if (motor_index >= DJI_MOTOR_COUNT) return NULL;
    return &dji_motors[motor_index];
}

void dji_motor_set_location(DJI_Motor_Instance* motor, int32_t location)
{
    if (motor == NULL) return;
    dji_critical_section_enter();
    motor->control_mode = MOTOR_MODE_POSITION;
    motor->target_loc = location;
    dji_critical_section_exit();
}

void dji_motor_set_speed(DJI_Motor_Instance* motor, int16_t speed)
{
    if (motor == NULL) return;
    dji_critical_section_enter();
    motor->control_mode = MOTOR_MODE_SPEED;
    motor->target_speed = speed;
    dji_critical_section_exit();
}

void dji_motor_stop_all(void)
{
    dji_critical_section_enter();
    dji_motor_control_enabled = false;
    dji_critical_section_exit();
}

void dji_motor_resume_all(void)
{
    dji_critical_section_enter();
    dji_motor_control_enabled = true;
    dji_critical_section_exit();
}

/* ------------------------- Internal/Callback Function Implementations ------------------------- */

void dji_motor_message_handler(void* instance, FDCAN_RxHeaderTypeDef* rx_header, uint8_t rx_data[8])
{
    if (instance == NULL) return;
    (void)rx_header;
    DJI_Motor_Instance* motor = (DJI_Motor_Instance*)instance;

    get_motor_measure(motor, rx_data);

    // -------------------------------------------------------------
    // FIX START: 修复上电第一帧重复计算 total_angle 的问题
    // -------------------------------------------------------------
    
    // 1. 每一帧都必须计算多圈角度，且只计算一次
    get_total_angle(motor); 

    // 2. 上电保护：如果是第一帧，将目标位置重置为当前位置，防止疯转
    if (motor->is_online == 0)
    {
        // 此时 motor->measure.total_angle 已经是正确的值了
        motor->target_loc = motor->measure.total_angle; 
        motor->is_online = 1;
    }
    
    // FIX END: 原代码在这里又调用了一次 get_total_angle(motor)，已删除
    // -------------------------------------------------------------
    
    motor->last_msg_time = HAL_GetTick();
}

void DJI_Motor_Control_Loop_1ms(void)
{
    for (uint8_t i = 0U; i < DJI_MOTOR_COUNT; ++i)
    {
        DJI_Motor_Instance* motor = &dji_motors[i];

        if ((motor->hfdcan == NULL) || (motor->can_id == 0U))
        {
            continue;
        }

        if (motor->control_mode == MOTOR_MODE_POSITION)
        {
            float speed_target = Pid_incremental_cal(&motor->loc_pid,
                                                     (float)motor->measure.total_angle,
                                                     (float)motor->target_loc);
            Pid_incremental_cal(&motor->spd_pid, (float)motor->measure.speed_rpm, speed_target);
        }
        else if (motor->control_mode == MOTOR_MODE_SPEED)
        {
            Pid_incremental_cal(&motor->spd_pid,
                                (float)motor->measure.speed_rpm,
                                (float)motor->target_speed);
        }
    }

    dji_motor_send_commands();
}

/* ------------------------- Private Helper Implementations ------------------------- */

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

    if (delta > (8192 / 2))
        delta -= 8192;
    else if (delta < -(8192 / 2))
        delta += 8192;

    motor->measure.total_angle += delta;
}

static void dji_motor_send_commands(void)
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
    
    if (dji_motors[0].hfdcan != NULL)
    {
        tx_header.Identifier = CAN_ID_FIRST_FOUR_MOTORS;
        tx_data[0] = (uint8_t)(out[0] >> 8); tx_data[1] = (uint8_t)out[0];
        tx_data[2] = (uint8_t)(out[1] >> 8); tx_data[3] = (uint8_t)out[1];
        tx_data[4] = (uint8_t)(out[2] >> 8); tx_data[5] = (uint8_t)out[2];
        tx_data[6] = (uint8_t)(out[3] >> 8); tx_data[7] = (uint8_t)out[3];
        (void)fdcan_bsp_send(dji_motors[0].hfdcan, &tx_header, tx_data);
    }
    
    if (dji_motors[4].hfdcan != NULL)
    {
        tx_header.Identifier = CAN_ID_LAST_FOUR_MOTORS;
        tx_data[0] = (uint8_t)(out[4] >> 8); tx_data[1] = (uint8_t)out[4];
        tx_data[2] = (uint8_t)(out[5] >> 8); tx_data[3] = (uint8_t)out[5];
        tx_data[4] = (uint8_t)(out[6] >> 8); tx_data[5] = (uint8_t)out[6];
        tx_data[6] = (uint8_t)(out[7] >> 8); tx_data[7] = (uint8_t)out[7];
        (void)fdcan_bsp_send(dji_motors[4].hfdcan, &tx_header, tx_data);
    }
}

static void dji_critical_section_enter(void) {
#ifdef USE_FREERTOS
    taskENTER_CRITICAL();
#else
    __disable_irq();
#endif
}

static void dji_critical_section_exit(void) {
#ifdef USE_FREERTOS
    taskEXIT_CRITICAL();
#else
    __enable_irq();
#endif
}
