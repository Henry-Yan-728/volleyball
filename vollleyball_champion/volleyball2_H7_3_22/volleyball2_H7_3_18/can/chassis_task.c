#include "chassis_task.h"

#include "dji_motor.h"

#include <math.h>

extern FDCAN_HandleTypeDef hfdcan2;

#define PI 3.1415926535f

#define STEERING_GEAR_RATIO   108.0f
#define WHEEL_RADIUS_M        0.023f
#define DRIVE_GEAR_RATIO      1.0f

#define CHASSIS_AUTO_CENTER_LINEAR_THRESHOLD   0.02f
#define CHASSIS_AUTO_CENTER_ANGULAR_THRESHOLD  0.02f
#define CHASSIS_AUTO_CENTER_ANGLE_DEG          0.0f
#define CHASSIS_SPIN_MIN_WHEEL_SPEED           0.06f

#define SPEED_DEADBAND                         0.05f

#define DRIVE_RAMP_STEP_UP_RPM                120
#define DRIVE_RAMP_STEP_DOWN_RPM              60

#define VESC_CAN_PACKET_SET_RPM               3U
#define VESC_MOTOR_POLE_PAIRS                 12

static const uint8_t s_steer_motor_indices[CHASSIS_STEERING_WHEEL_COUNT] = {
    CHASSIS_STEER_MOTOR_IDX_W1,
    CHASSIS_STEER_MOTOR_IDX_W2,
    CHASSIS_STEER_MOTOR_IDX_W3
};

static const float s_steer_angle_offsets[CHASSIS_STEERING_WHEEL_COUNT] = {
    0.0f,
    -60.0f,
    60.0f
};

static const uint8_t s_drive_vesc_ids[CHASSIS_STEERING_WHEEL_COUNT] = {
    CHASSIS_DRIVE_VESC_ID_W1,
    CHASSIS_DRIVE_VESC_ID_W2,
    CHASSIS_DRIVE_VESC_ID_W3
};

static int32_t s_drive_rpm_cmd[CHASSIS_STEERING_WHEEL_COUNT] = {0};

static float normalize_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static float calc_steering_angle(float x, float y)
{
    if ((fabsf(x) < 0.001f) && (fabsf(y) < 0.001f)) {
        return 0.0f;
    }

    {
        float theta = atan2f(y, x);
        float angle_deg = 90.0f - theta * 180.0f / PI;
        return normalize_angle(angle_deg);
    }
}

static int32_t speed_to_motor_rpm(float speed_ms)
{
    float wheel_rpm = (speed_ms / (2.0f * PI * WHEEL_RADIUS_M)) * 60.0f;
    float motor_rpm = wheel_rpm * DRIVE_GEAR_RATIO;
    return (int32_t)motor_rpm;
}

static int32_t ramp_drive_rpm(uint8_t wheel_index, int32_t target_rpm)
{
    int32_t current_rpm = s_drive_rpm_cmd[wheel_index];
    int32_t delta = target_rpm - current_rpm;

    if (delta > 0) {
        int32_t step = (current_rpm < 0) ? DRIVE_RAMP_STEP_DOWN_RPM : DRIVE_RAMP_STEP_UP_RPM;
        if (delta > step) {
            current_rpm += step;
        } else {
            current_rpm = target_rpm;
        }
    } else if (delta < 0) {
        int32_t step = (current_rpm > 0) ? DRIVE_RAMP_STEP_DOWN_RPM : DRIVE_RAMP_STEP_UP_RPM;
        if (-delta > step) {
            current_rpm -= step;
        } else {
            current_rpm = target_rpm;
        }
    }

    s_drive_rpm_cmd[wheel_index] = current_rpm;
    return current_rpm;
}

static void vesc_send_rpm(uint8_t controller_id, int32_t mechanical_rpm)
{
    uint8_t tx_data[4];
    FDCAN_TxHeaderTypeDef tx_header;
    int32_t erpm = mechanical_rpm * VESC_MOTOR_POLE_PAIRS;
    uint32_t vesc_id = ((uint32_t)VESC_CAN_PACKET_SET_RPM << 8) | (uint32_t)controller_id;

    tx_data[0] = (uint8_t)(erpm >> 24);
    tx_data[1] = (uint8_t)(erpm >> 16);
    tx_data[2] = (uint8_t)(erpm >> 8);
    tx_data[3] = (uint8_t)(erpm);

    tx_header.Identifier = vesc_id;
    tx_header.IdType = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_4;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0U) {
        (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_header, tx_data);
    }
}

static void steering_wheel_autocenter(uint8_t wheel_index, float center_angle_deg)
{
    uint8_t steer_idx = s_steer_motor_indices[wheel_index];
    DJI_Motor_Instance *steering_motor = dji_motor_get_instance(steer_idx);

    if (steering_motor == NULL) return;

    {
        float current_angle_deg = (float)steering_motor->measure.total_angle / 8192.0f / STEERING_GEAR_RATIO * 360.0f;
        float current_heading = normalize_angle(fmodf(current_angle_deg, 360.0f));
        float delta_angle = normalize_angle(center_angle_deg - current_heading);
        float final_target_deg = current_angle_deg + delta_angle;
        int32_t target_encoder = (int32_t)(final_target_deg / 360.0f * STEERING_GEAR_RATIO * 8192.0f);

        dji_motor_set_location(steering_motor, target_encoder);
    }

    s_drive_rpm_cmd[wheel_index] = 0;
    vesc_send_rpm(s_drive_vesc_ids[wheel_index], 0);
}

static void steering_wheel_control(uint8_t wheel_index, float target_speed_ms, float target_angle_deg)
{
    uint8_t steer_idx = s_steer_motor_indices[wheel_index];
    DJI_Motor_Instance *steering_motor = dji_motor_get_instance(steer_idx);
    int direction = 1;

    if (steering_motor == NULL) return;

    {
        float current_angle_deg = (float)steering_motor->measure.total_angle / 8192.0f / STEERING_GEAR_RATIO * 360.0f;
        float current_heading = normalize_angle(fmodf(current_angle_deg, 360.0f));
        float delta_angle = normalize_angle(target_angle_deg - current_heading);
        float final_target_deg;
        int32_t target_encoder;

        if (fabsf(delta_angle) > 95.0f) {
            if (delta_angle > 0.0f) delta_angle -= 180.0f;
            else                    delta_angle += 180.0f;
            direction = -1;
        }

        final_target_deg = current_angle_deg + delta_angle;
        target_encoder = (int32_t)(final_target_deg / 360.0f * STEERING_GEAR_RATIO * 8192.0f);
        dji_motor_set_location(steering_motor, target_encoder);
    }

    {
        float drive_target_speed = (fabsf(target_speed_ms) >= SPEED_DEADBAND) ? (target_speed_ms * (float)direction) : 0.0f;
        int32_t target_rpm = speed_to_motor_rpm(drive_target_speed);
        int32_t smooth_rpm = ramp_drive_rpm(wheel_index, target_rpm);

        vesc_send_rpm(s_drive_vesc_ids[wheel_index], smooth_rpm);
    }
}

void Chassis_Init(void)
{
    uint8_t i;
    for (i = 0U; i < CHASSIS_STEERING_WHEEL_COUNT; i++) {
        s_drive_rpm_cmd[i] = 0;
    }
    Chassis_Stop();
}

void Chassis_Update(float vx, float vy, float vr)
{
    float vx_w[CHASSIS_STEERING_WHEEL_COUNT];
    float vy_w[CHASSIS_STEERING_WHEEL_COUNT];
    float spd[CHASSIS_STEERING_WHEEL_COUNT];
    float ang[CHASSIS_STEERING_WHEEL_COUNT];
    uint8_t i;

    if ((fabsf(vx) < CHASSIS_AUTO_CENTER_LINEAR_THRESHOLD) &&
        (fabsf(vy) < CHASSIS_AUTO_CENTER_LINEAR_THRESHOLD) &&
        (fabsf(vr) < CHASSIS_AUTO_CENTER_ANGULAR_THRESHOLD)) {
        for (i = 0U; i < CHASSIS_STEERING_WHEEL_COUNT; i++) {
            steering_wheel_autocenter(i, CHASSIS_AUTO_CENTER_ANGLE_DEG);
        }
        return;
    }

    {
        float vr_r = vr * CHASSIS_RADIUS;
        vx_w[0] = vx - vr_r;
        vy_w[0] = vy;

        vx_w[1] = vx + 0.5f * vr_r;
        vy_w[1] = vy + 0.866f * vr_r;

        vx_w[2] = vx + 0.5f * vr_r;
        vy_w[2] = vy - 0.866f * vr_r;
    }

    for (i = 0U; i < CHASSIS_STEERING_WHEEL_COUNT; i++) {
        spd[i] = sqrtf(vx_w[i] * vx_w[i] + vy_w[i] * vy_w[i]);
    }

    if ((fabsf(vx) < CHASSIS_AUTO_CENTER_LINEAR_THRESHOLD) &&
        (fabsf(vy) < CHASSIS_AUTO_CENTER_LINEAR_THRESHOLD) &&
        (fabsf(vr) >= CHASSIS_AUTO_CENTER_ANGULAR_THRESHOLD)) {
        for (i = 0U; i < CHASSIS_STEERING_WHEEL_COUNT; i++) {
            if (fabsf(spd[i]) < CHASSIS_SPIN_MIN_WHEEL_SPEED) {
                spd[i] = CHASSIS_SPIN_MIN_WHEEL_SPEED;
            }
        }
    }

    for (i = 0U; i < CHASSIS_STEERING_WHEEL_COUNT; i++) {
        ang[i] = -calc_steering_angle(vx_w[i], vy_w[i]) + s_steer_angle_offsets[i];
        steering_wheel_control(i, spd[i], ang[i]);
    }
}

void Chassis_Stop(void)
{
    uint8_t i;
    for (i = 0U; i < CHASSIS_STEERING_WHEEL_COUNT; i++) {
        steering_wheel_autocenter(i, CHASSIS_AUTO_CENTER_ANGLE_DEG);
    }
}
