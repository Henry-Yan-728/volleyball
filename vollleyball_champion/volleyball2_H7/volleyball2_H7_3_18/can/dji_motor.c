#include "dji_motor.h"

#include "fdcan_bsp.h"

#include <stdbool.h>

#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

#define CAN_ID_FIRST_FOUR_MOTORS 0x200U
#define CAN_ID_LAST_FOUR_MOTORS  0x1FFU

static DJI_Motor_Instance dji_motors[DJI_MOTOR_COUNT];
static bool dji_motor_control_enabled = true;

static void get_motor_measure(DJI_Motor_Instance *motor, uint8_t rx_data[8]);
static void get_total_angle(DJI_Motor_Instance *motor);
static void dji_motor_send_commands(DJI_Motor_Instance *trigger_motor);
static void dji_motor_configure(uint8_t index,
                                uint16_t id,
                                FDCAN_HandleTypeDef *hfdcan,
                                float loc_kp,
                                float loc_ki,
                                float loc_kd,
                                float spd_kp,
                                float spd_ki,
                                float spd_kd,
                                float loc_out_limit_up,
                                float loc_out_limit_down,
                                float spd_out_limit_up,
                                float spd_out_limit_down);
static void dji_critical_section_enter(void);
static void dji_critical_section_exit(void);

int32_t dji_angle_to_encoder(float angle_deg, float reduction_ratio)
{
    float total_counts_per_round = DJI_ENCODER_RESOLUTION * reduction_ratio;
    return (int32_t)((angle_deg / 360.0f) * total_counts_per_round);
}

float dji_encoder_to_angle(int32_t encoder_val, float reduction_ratio)
{
    float total_counts_per_round = DJI_ENCODER_RESOLUTION * reduction_ratio;
    return ((float)encoder_val / total_counts_per_round) * 360.0f;
}

int32_t dji_degree2encoder(float degree)
{
    return dji_angle_to_encoder(degree, 1.0f);
}

void dji_motors_init(void)
{
    /* ID1,2: 3508 on CAN2 */
    dji_motor_configure(0U, CAN_3508_2006_M1_ID, &hfdcan2,
                        0.20f, 0.001f, 0.035f,
                        24.0f, 1.20f, 0.030f,
                        1200.0f, -1200.0f, 12000.0f, -12000.0f);

    dji_motor_configure(1U, CAN_3508_2006_M2_ID, &hfdcan2,
                        0.20f, 0.001f, 0.035f,
                        24.0f, 1.20f, 0.030f,
                        1200.0f, -1200.0f, 12000.0f, -12000.0f);

    /* ID3,4,5: steering 2006 on CAN2 */
    dji_motor_configure(2U, CAN_3508_2006_M3_ID, &hfdcan2,
                        0.364706f, 0.007333f, 0.004745f,
                        16.7f, 0.98f, 0.024314f,
                        12000.0f, -12000.0f, 22000.0f, -22000.0f);

    dji_motor_configure(3U, CAN_3508_2006_M4_ID, &hfdcan2,
                        0.364706f, 0.007333f, 0.004745f,
                        16.7f, 0.98f, 0.024314f,
                        12000.0f, -12000.0f, 22000.0f, -22000.0f);

    dji_motor_configure(4U, CAN_3508_2006_M5_ID, &hfdcan2,
                        0.364706f, 0.007333f, 0.004745f,
                        16.7f, 0.98f, 0.024314f,
                        12000.0f, -12000.0f, 22000.0f, -22000.0f);

    /* ID6: 6020 on CAN2 */
    dji_motor_configure(GIMBAL_MOTOR_YAW_ID, CAN_3508_2006_M6_ID, &hfdcan2,
                        0.67f, 0.006f, 0.005f,
                        40.0f, 3.0f, 0.0f,
                        180.0f, -180.0f, 28000.0f, -28000.0f);
}

DJI_Motor_Instance *dji_motor_get_instance(uint8_t motor_index)
{
    if (motor_index >= DJI_MOTOR_COUNT) return NULL;
    return &dji_motors[motor_index];
}

void dji_motor_set_location(DJI_Motor_Instance *motor, int32_t location)
{
    if (motor == NULL) return;
    dji_critical_section_enter();
    motor->control_mode = MOTOR_MODE_POSITION;
    motor->target_loc = location;
    dji_critical_section_exit();
}

void dji_motor_set_speed(DJI_Motor_Instance *motor, int16_t speed)
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

void dji_motor_message_handler(void *instance, FDCAN_RxHeaderTypeDef *rx_header, uint8_t rx_data[8])
{
    (void)rx_header;

    if (instance == NULL) return;

    DJI_Motor_Instance *motor = (DJI_Motor_Instance *)instance;

    get_motor_measure(motor, rx_data);
    get_total_angle(motor);

    if (motor->is_online == 0U) {
        motor->target_loc = motor->measure.total_angle;
        motor->is_online = 1U;
    }

    motor->last_msg_time = HAL_GetTick();

    if (motor->control_mode == MOTOR_MODE_POSITION) {
        float speed_target = Pid_incremental_cal(&motor->loc_pid,
                                                 (float)motor->measure.total_angle,
                                                 (float)motor->target_loc);
        (void)Pid_incremental_cal(&motor->spd_pid,
                                  (float)motor->measure.speed_rpm,
                                  speed_target);
    } else if (motor->control_mode == MOTOR_MODE_SPEED) {
        (void)Pid_incremental_cal(&motor->spd_pid,
                                  (float)motor->measure.speed_rpm,
                                  (float)motor->target_speed);
    }

    dji_motor_send_commands(motor);
}

static void dji_motor_configure(uint8_t index,
                                uint16_t id,
                                FDCAN_HandleTypeDef *hfdcan,
                                float loc_kp,
                                float loc_ki,
                                float loc_kd,
                                float spd_kp,
                                float spd_ki,
                                float spd_kd,
                                float loc_out_limit_up,
                                float loc_out_limit_down,
                                float spd_out_limit_up,
                                float spd_out_limit_down)
{
    if (index >= DJI_MOTOR_COUNT) return;

    DJI_Motor_Instance *motor = &dji_motors[index];

    motor->can_id = id;
    motor->hfdcan = hfdcan;
    motor->control_mode = MOTOR_MODE_POSITION;

    motor->loc_pid.Kp = loc_kp;
    motor->loc_pid.Ki = loc_ki;
    motor->loc_pid.Kd = loc_kd;
    motor->loc_pid.out_limit_up = loc_out_limit_up;
    motor->loc_pid.out_limit_down = loc_out_limit_down;

    motor->spd_pid.Kp = spd_kp;
    motor->spd_pid.Ki = spd_ki;
    motor->spd_pid.Kd = spd_kd;
    motor->spd_pid.out_limit_up = spd_out_limit_up;
    motor->spd_pid.out_limit_down = spd_out_limit_down;

    FDCAN_Dispatch_t dispatch_item;
    dispatch_item.id_type = FDCAN_STANDARD_ID;
    dispatch_item.id = motor->can_id;
    dispatch_item.instance_ptr = motor;
    dispatch_item.handler = dji_motor_message_handler;

    fdcan_bsp_register(&dispatch_item, hfdcan);
}

static void get_motor_measure(DJI_Motor_Instance *motor, uint8_t rx_data[8])
{
    motor->measure.last_angle = motor->measure.angle;
    motor->measure.angle = (uint16_t)((uint16_t)rx_data[0] << 8 | (uint16_t)rx_data[1]);
    motor->measure.speed_rpm = (int16_t)((uint16_t)rx_data[2] << 8 | (uint16_t)rx_data[3]);
    motor->measure.given_current = (int16_t)((uint16_t)rx_data[4] << 8 | (uint16_t)rx_data[5]);
    motor->measure.temperate = rx_data[6];
}

static void get_total_angle(DJI_Motor_Instance *motor)
{
    int32_t delta = (int32_t)motor->measure.angle - (int32_t)motor->measure.last_angle;

    if (delta > (8192 / 2)) delta -= 8192;
    else if (delta < -(8192 / 2)) delta += 8192;

    motor->measure.total_angle += delta;
}

static void dji_motor_send_commands(DJI_Motor_Instance *trigger_motor)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    int16_t out[DJI_MOTOR_COUNT];

    for (uint8_t i = 0U; i < DJI_MOTOR_COUNT; i++) {
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

    if ((trigger_motor == NULL) || ((trigger_motor >= &dji_motors[0]) && (trigger_motor <= &dji_motors[3]))) {
        tx_header.Identifier = CAN_ID_FIRST_FOUR_MOTORS;
        tx_data[0] = (uint8_t)(out[0] >> 8); tx_data[1] = (uint8_t)out[0];
        tx_data[2] = (uint8_t)(out[1] >> 8); tx_data[3] = (uint8_t)out[1];
        tx_data[4] = (uint8_t)(out[2] >> 8); tx_data[5] = (uint8_t)out[2];
        tx_data[6] = (uint8_t)(out[3] >> 8); tx_data[7] = (uint8_t)out[3];

        FDCAN_HandleTypeDef *hcan = (trigger_motor != NULL) ? trigger_motor->hfdcan : dji_motors[0].hfdcan;
        if (hcan != NULL) {
            (void)HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, tx_data);
        }
    }

    if ((trigger_motor == NULL) || ((trigger_motor >= &dji_motors[4]) && (trigger_motor <= &dji_motors[7]))) {
        tx_header.Identifier = CAN_ID_LAST_FOUR_MOTORS;
        tx_data[0] = (uint8_t)(out[4] >> 8); tx_data[1] = (uint8_t)out[4];
        tx_data[2] = (uint8_t)(out[5] >> 8); tx_data[3] = (uint8_t)out[5];
        tx_data[4] = (uint8_t)(out[6] >> 8); tx_data[5] = (uint8_t)out[6];
        tx_data[6] = (uint8_t)(out[7] >> 8); tx_data[7] = (uint8_t)out[7];

        FDCAN_HandleTypeDef *hcan = (trigger_motor != NULL) ? trigger_motor->hfdcan : dji_motors[4].hfdcan;
        if (hcan != NULL) {
            (void)HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, tx_data);
        }
    }
}

static void dji_critical_section_enter(void)
{
#ifdef USE_FREERTOS
    taskENTER_CRITICAL();
#else
    __disable_irq();
#endif
}

static void dji_critical_section_exit(void)
{
#ifdef USE_FREERTOS
    taskEXIT_CRITICAL();
#else
    __enable_irq();
#endif
}
