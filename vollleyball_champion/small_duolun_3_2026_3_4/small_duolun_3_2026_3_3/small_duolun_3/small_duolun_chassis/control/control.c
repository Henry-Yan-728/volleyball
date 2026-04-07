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

    // 【修改点1】速度死区依然要维持当前位置的闭环
    if (fabsf(target_spd) < SPEED_DEADBAND)
    {
        Change_vesc_speed(wheel_index, 0);
        Com2vesc(wheel_index); 
        // 关键：持续下发当前的位置作为目标，防止舵机卸力！
        dji_motor_set_location(steering_motor, steering_motor->measure.total_angle);
        return; 
    }

    // 【修改点2】强制使用 double 防止无休止旋转后 float 精度丢失导致跑偏
    double current_angle_deg = (double)steering_motor->measure.total_angle / 8192.0 / (double)STEERING_GEAR_RATIO * 360.0;
    
    // 取余化简后再用 float 计算偏差，确保绝对精确
    float current_heading = fmod((float)current_angle_deg, 360.0f);
    current_heading = normalize_angle(current_heading);

    float delta_angle = normalize_angle(target_ang - current_heading);
    int direction = 1; 

    // 最短路径 + 反转动力逻辑
    if (fabsf(delta_angle) > 95.0f) {
        if (delta_angle > 0) delta_angle -= 180.0f;
        else                 delta_angle += 180.0f;
        direction = -1;
    }

    // 计算增量目标
    double final_target_deg = current_angle_deg + (double)delta_angle;
    int32_t target_encoder = (int32_t)(final_target_deg / 360.0 * (double)STEERING_GEAR_RATIO * 8192.0);
    
    dji_motor_set_location(steering_motor, target_encoder);

    int32_t target_erpm = Speed_to_ERPM(target_spd) * direction;
    Change_vesc_speed(wheel_index, target_erpm);
    Com2vesc(wheel_index);
}