#include "control.h"

#include "dji_motor.h"
#include "vesc.h"

#include <math.h>

#define PI_F 3.1415926535f

/* User-required mapping:
 * 3508: ID1,2
 * 2006 steering: ID3,4,5
 * 6020: ID6
 */
static const uint8_t s_steering_motor_index_map[STEERING_WHEEL_COUNT] = {2U, 3U, 4U};
static const uint8_t s_vesc_motor_id_map[STEERING_WHEEL_COUNT] = {0U, 1U, 2U};

static float normalize_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/* Keep the original small_duolun conversion behavior. */
static int32_t speed_to_vesc_cmd(float speed_ms)
{
    float wheel_rpm = (speed_ms / (2.0f * PI_F * WHEEL_RADIUS)) * 60.0f;
    float motor_rpm = wheel_rpm * DRIVE_GEAR_RATIO;
    return (int32_t)(motor_rpm * VESC_POLE_PAIRS);
}

static DJI_Motor_Instance *get_steering_motor(uint8_t wheel_index)
{
    if (wheel_index >= STEERING_WHEEL_COUNT) return NULL;
    return dji_motor_get_instance(s_steering_motor_index_map[wheel_index]);
}

static uint8_t get_vesc_id(uint8_t wheel_index)
{
    if (wheel_index >= STEERING_WHEEL_COUNT) return 0U;
    return s_vesc_motor_id_map[wheel_index];
}

void Steering_Wheel_Control(uint8_t wheel_index, float target_speed, float target_angle_deg)
{
    DJI_Motor_Instance *steering_motor = get_steering_motor(wheel_index);
    if (steering_motor == NULL) return;

    uint8_t drive_enable = (fabsf(target_speed) >= SPEED_DEADBAND);
    double current_angle_deg =
        (double)steering_motor->measure.total_angle / 8192.0 / (double)STEERING_GEAR_RATIO * 360.0;

    float current_heading = fmodf((float)current_angle_deg, 360.0f);
    current_heading = normalize_angle(current_heading);

    float delta_angle = normalize_angle(target_angle_deg - current_heading);
    int direction = 1;

    /* shortest steering path + invert drive direction */
    if (fabsf(delta_angle) > 95.0f) {
        if (delta_angle > 0.0f) delta_angle -= 180.0f;
        else delta_angle += 180.0f;
        direction = -1;
    }

    double final_target_deg = current_angle_deg + (double)delta_angle;
    int32_t target_encoder =
        (int32_t)(final_target_deg / 360.0 * (double)STEERING_GEAR_RATIO * 8192.0);

    dji_motor_set_location(steering_motor, target_encoder);

    int32_t target_vesc = drive_enable ? (speed_to_vesc_cmd(target_speed) * direction) : 0;
    uint8_t vesc_id = get_vesc_id(wheel_index);
    Change_vesc_speed(vesc_id, target_vesc);
    Com2vesc(vesc_id);
}

void Steering_Wheel_AutoCenter(uint8_t wheel_index, float center_angle_deg)
{
    DJI_Motor_Instance *steering_motor = get_steering_motor(wheel_index);
    if (steering_motor == NULL) return;

    double current_angle_deg =
        (double)steering_motor->measure.total_angle / 8192.0 / (double)STEERING_GEAR_RATIO * 360.0;

    float current_heading = fmodf((float)current_angle_deg, 360.0f);
    current_heading = normalize_angle(current_heading);

    float delta_angle = normalize_angle(center_angle_deg - current_heading);
    double final_target_deg = current_angle_deg + (double)delta_angle;
    int32_t target_encoder =
        (int32_t)(final_target_deg / 360.0 * (double)STEERING_GEAR_RATIO * 8192.0);

    dji_motor_set_location(steering_motor, target_encoder);

    uint8_t vesc_id = get_vesc_id(wheel_index);
    Change_vesc_speed(vesc_id, 0);
    Com2vesc(vesc_id);
}
