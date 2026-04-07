#include "control.h"
#include "dji_motor.h"
#include "vesc.h"
#include <math.h>

#define PI 3.1415926535f

// ================= 提取自 wheel_control.c 的核心逻辑 =================

elem_type get_abs(elem_type x){
    x = (x >= 0) ? x : -x;
    return x;
}

// 核心解算逻辑：计算最短路径并决定动力电机极性
elem_type get_delta_angle(float target_angle, float now_angle, int* direction){
    elem_type delta_angle = 0;
    
    delta_angle = get_abs(target_angle - now_angle);
    if(delta_angle <= 90){
        *direction = 1;
        return target_angle - now_angle;
    }
    else if (get_abs(target_angle - now_angle - 360) <= 90) {
        *direction = 1;
        return target_angle - now_angle - 360;
    }
    else if(get_abs(target_angle - now_angle + 360) <= 90){
        *direction = 1;
        return target_angle - now_angle + 360;
    }
    
    delta_angle = target_angle - now_angle + 180;
    if(delta_angle > 90) delta_angle -= 360;
    
    *direction = -1;
    return delta_angle;
}

// ================= 内部辅助函数 =================

// 将速度 (m/s) 转换为 VESC 需要的 ERPM
static int32_t Speed_to_ERPM(float speed_ms) {
    float wheel_rpm = (speed_ms / (2.0f * PI * WHEEL_RADIUS)) * 60.0f;
    float motor_rpm = wheel_rpm * DRIVE_GEAR_RATIO;
    return (int32_t)(motor_rpm * VESC_POLE_PAIRS);
}

void control_init(void) {
    // 预留初始化接口
}

// ================= 底盘 2006+VESC 控制执行 =================

void Steering_Wheel_Control(uint8_t wheel_index, float target_spd, float target_angle_deg)
{
    // 安全检查：防止越界 (只有3个轮子)
    if (wheel_index >= 3) return;
    
    DJI_Motor_Instance* steering_motor = dji_motor_get_instance(wheel_index);
    if (steering_motor == NULL) return;

    // 1. 获取当前电机的多圈累计连续角度 (单位：度)
    double continuous_deg = STEERING_POLARITY * (double)steering_motor->measure.total_angle / 8192.0 / (double)STEERING_GEAR_RATIO * 360.0;

    // 2. 将连续角度折算到 -180 ~ 180 度的单圈范围 (适配 get_delta_angle 的 now_angle 入参)
    float now_angle_pos = fmod((float)continuous_deg, 360.0f);
    if (now_angle_pos > 180.0f) {
        now_angle_pos -= 360.0f;
    } else if (now_angle_pos <= -180.0f) {
        now_angle_pos += 360.0f;
    }

    // 3. 调用 wheel_control.c 的核心逻辑获取偏差与极性
    int direction = 1;
    float delta_angle = get_delta_angle(target_angle_deg, now_angle_pos, &direction);

    // 4. 将算出的偏差直接叠加回连续目标角度上 (多圈映射机制)
    double final_target_deg = continuous_deg + (double)delta_angle;

    // 5. 映射回编码器值并下发给 DJI 位置环
    int32_t target_encoder = (int32_t)((final_target_deg * STEERING_POLARITY) / 360.0 * (double)STEERING_GEAR_RATIO * 8192.0);
    dji_motor_set_location(steering_motor, target_encoder);

    // 6. 速度下发 VESC (直接使用计算出的 direction)
    int32_t target_erpm = Speed_to_ERPM(target_spd) * direction;
    Change_vesc_speed(wheel_index, target_erpm);
    Com2vesc(wheel_index);
}