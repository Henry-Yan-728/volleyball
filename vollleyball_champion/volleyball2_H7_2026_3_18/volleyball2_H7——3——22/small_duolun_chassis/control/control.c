#include "control.h"
#include "dji_motor.h"
#include "vesc.h"
#include <math.h>

#define PI 3.1415926535f

static float normalize_angle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static int32_t Speed_to_ERPM(float speed_ms) {
    float wheel_rpm = (speed_ms / (2.0f * PI * WHEEL_RADIUS)) * 60.0f;
    float motor_rpm = wheel_rpm * DRIVE_GEAR_RATIO;
    return (int32_t)(motor_rpm * VESC_POLE_PAIRS);
}

void Steering_Wheel_Control(uint8_t wheel_index, float target_spd, float target_ang)
{
    DJI_Motor_Instance* steering_motor = dji_motor_get_instance(wheel_index);
    if (steering_motor == NULL) return;
    uint8_t drive_enable = (fabsf(target_spd) >= SPEED_DEADBAND);


    // 用 double 降低多圈累计后精度损失
    double current_angle_deg = (double)steering_motor->measure.total_angle / 8192.0 / (double)STEERING_GEAR_RATIO * 360.0;

    float current_heading = fmodf((float)current_angle_deg, 360.0f);
    current_heading = normalize_angle(current_heading);

    float delta_angle = normalize_angle(target_ang - current_heading);
    int direction = 1;

    // 最短路径 + 动力轮反转
    if (fabsf(delta_angle) > 95.0f) {
        if (delta_angle > 0) delta_angle -= 180.0f;
        else                 delta_angle += 180.0f;
        direction = -1;
    }

    double final_target_deg = current_angle_deg + (double)delta_angle;
    int32_t target_encoder = (int32_t)(final_target_deg / 360.0 * (double)STEERING_GEAR_RATIO * 8192.0);

    dji_motor_set_location(steering_motor, target_encoder);

    int32_t target_erpm = drive_enable ? (Speed_to_ERPM(target_spd) * direction) : 0;
    Change_vesc_speed(wheel_index, target_erpm);
    Com2vesc(wheel_index);
}

void Steering_Wheel_AutoCenter(uint8_t wheel_index, float center_angle_deg)
{
    DJI_Motor_Instance* steering_motor = dji_motor_get_instance(wheel_index);
    if (steering_motor == NULL) return;

    double current_angle_deg = (double)steering_motor->measure.total_angle / 8192.0 / (double)STEERING_GEAR_RATIO * 360.0;

    float current_heading = fmodf((float)current_angle_deg, 360.0f);
    current_heading = normalize_angle(current_heading);

    float delta_angle = normalize_angle(center_angle_deg - current_heading);
    double final_target_deg = current_angle_deg + (double)delta_angle;
    int32_t target_encoder = (int32_t)(final_target_deg / 360.0 * (double)STEERING_GEAR_RATIO * 8192.0);

    dji_motor_set_location(steering_motor, target_encoder);

    Change_vesc_speed(wheel_index, 0);
    Com2vesc(wheel_index);
}
