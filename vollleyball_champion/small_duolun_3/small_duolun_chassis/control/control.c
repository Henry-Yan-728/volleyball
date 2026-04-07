#include "control.h"
#include "dji_motor.h"
#include "vesc.h"
#include <math.h>
#include <stdio.h>

#define PI 3.1415926535f

// ================= 关键调试参数 =================
// 如果电机给速度后疯狂旋转，请将此值从 1.0f 改为 -1.0f
#define STEERING_POLARITY         1.0f  
// 限制单次增量，建议先调小（如 0.5f）观察，稳定后再加大
#define STEER_RAMP_DEG_PER_CALL   0.5f  

// ================= 辅助函数 =================

// 1. 将速度 (m/s) 转换为 VESC 需要的 ERPM
static int32_t Speed_to_ERPM(float speed_ms) {
    // 轮子转速 (RPM) = 速度 / 轮子周长 * 60
    float wheel_rpm = (speed_ms / (2.0f * PI * WHEEL_RADIUS)) * 60.0f;
    // 电机转速 (RPM) = 轮子转速 * 减速比
    float motor_rpm = wheel_rpm * DRIVE_GEAR_RATIO;
    // ERPM = 电机转速 * 极对数
    return (int32_t)(motor_rpm * VESC_POLE_PAIRS);
}

// 2. 将角度标准化到 -180 到 180 度之间
static float normalize_angle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

// ================= 核心控制逻辑 =================

void Steering_Wheel_Control(uint8_t wheel_index, float target_spd, float target_ang)
{
    static float last_target_ang[4]   = {0.0f}; 
    static double current_ramped_deg[4] = {0.0};   
    static uint8_t init_flag[4]       = {0};             

    DJI_Motor_Instance* steering_motor = dji_motor_get_instance(wheel_index);
    if (steering_motor == NULL || wheel_index >= 3) return; // 优化：共3个轮子，边界检查改为3

    // 1. 获取当前电机的实际连续角度
    double current_actual_angle_deg = STEERING_POLARITY * (double)steering_motor->measure.total_angle / 8192.0 / (double)STEERING_GEAR_RATIO * 360.0;

    // 2. 初始化：将虚拟轴对齐到物理轴
    if (!init_flag[wheel_index]) {
        current_ramped_deg[wheel_index] = current_actual_angle_deg;
        init_flag[wheel_index] = 1;
    }

    // 3. 速度死区处理
    if (fabsf(target_spd) < SPEED_DEADBAND) {
        target_ang = last_target_ang[wheel_index];
        target_spd = 0.0f;
    } else {
        last_target_ang[wheel_index] = target_ang;
    }

    // 4. 最短路径算法 (核心修复：基于虚拟规划位置 current_ramped_deg 进行查偏，防止机械滞后引起反馈振荡)
    float current_heading = fmod((float)current_ramped_deg[wheel_index], 360.0f);
    current_heading = normalize_angle(current_heading);

    float delta_angle = normalize_angle(target_ang - current_heading);
    int direction = 1; 

    if (fabsf(delta_angle) > 90.0f) {
        delta_angle = normalize_angle(delta_angle + 180.0f);
        direction = -1;
    }

    // 计算本次期望达到的目标绝对位置 (基于 Ramp，而不是 Actual)
    double final_target_deg = current_ramped_deg[wheel_index] + (double)delta_angle;

    // 5. 轨迹平滑插补 (Ramp)
    double error = final_target_deg - current_ramped_deg[wheel_index];
    if (fabs(error) <= STEER_RAMP_DEG_PER_CALL) {
        current_ramped_deg[wheel_index] = final_target_deg;
    } else {
        current_ramped_deg[wheel_index] += (error > 0 ? STEER_RAMP_DEG_PER_CALL : -STEER_RAMP_DEG_PER_CALL);
    }

    // 6. 映射回编码器值并下发
    int32_t target_encoder = (int32_t)((current_ramped_deg[wheel_index] * STEERING_POLARITY) / 360.0 * (double)STEERING_GEAR_RATIO * 8192.0);
    dji_motor_set_location(steering_motor, target_encoder);

    // 7. 动力电机输出 (引入余弦衰减)
    float speed_scale = cosf(delta_angle * PI / 180.0f);
    if (speed_scale < 0.0f) speed_scale = 0.0f;

    int32_t target_erpm = Speed_to_ERPM(target_spd * speed_scale) * direction;
    Change_vesc_speed(wheel_index, target_erpm);
    Com2vesc(wheel_index);
}