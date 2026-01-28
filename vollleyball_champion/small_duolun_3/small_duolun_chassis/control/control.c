#include "control.h"
#include "dji_motor.h"
#include "vesc.h"
#include <math.h>

// 辅助函数：角度标准化到 -180 ~ 180
static float normalize_angle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 舵轮控制核心逻辑 (Swerve Optimization)
 * @param wheel_index: 0, 1, 2 对应三个舵轮
 * @param target_spd: 目标线速度
 * @param target_ang: 目标绝对角度 (-180 到 180)
 */
void Steering_Wheel_Control(uint8_t wheel_index, float target_spd, float target_ang)
{
    // 1. 获取对应的 DJI 舵向电机实例
    // 假设: dji_motors[0-2] 对应轮子 0-2 的舵向电机
    DJI_Motor_Instance* steering_motor = dji_motor_get_instance(wheel_index);
    if (steering_motor == NULL) return;

    // 2. 获取当前舵向角度 (考虑减速比)
    // dji_motor 驱动维护了 total_angle (编码器值)，需转换为度
    // 公式: (Total_Encoder / 8192) * 360 / 减速比
    float current_angle_deg = (float)steering_motor->measure.total_angle / 8192.0f * 360.0f / STEERING_GEAR_RATIO;
    
    // 3. 优化路径计算 (最短路径 + 反转逻辑)
    // 将当前累计角度模 360，得到当前的朝向 (-180 ~ 180)
    float current_heading = fmodf(current_angle_deg, 360.0f);
    current_heading = normalize_angle(current_heading);

    // 计算角度差
    float delta_angle = normalize_angle(target_ang - current_heading);
    
    int direction = 1; // VESC 电机方向 (1: 正转, -1: 反转)

    // 如果角度差绝对值大于 90 度，说明转反向更近 (例如从 0 度去 170 度，不如转到 -10 度并倒车)
    if (fabsf(delta_angle) > 90.0f)
    {
        if (delta_angle > 0) delta_angle -= 180.0f;
        else                 delta_angle += 180.0f;
        
        direction = -1; // 倒车
    }

    // 4. 计算最终目标位置
    // 目标位置 = 当前累计位置 + 优化后的角度差
    float final_target_deg = current_angle_deg + delta_angle;

    // 转换为编码器值并下发给 DJI 电机
    int32_t target_encoder = (int32_t)(final_target_deg / 360.0f * 8192.0f * STEERING_GEAR_RATIO);
    dji_motor_set_location(steering_motor, target_encoder);

    // 5. 控制 VESC 航向电机
    // 假设 target_spd 是 m/s，需要转换成 ERPM
    // 这里简单做一个比例转换，你需要根据轮径和 Kv值 调整系数
    // 假设: 1 m/s ~= 2000 ERPM (需校准)
    const float SPEED_TO_ERPM_RATIO = 2000.0f; 
    
    int32_t target_erpm = (int32_t)(target_spd * SPEED_TO_ERPM_RATIO * direction);

    // 映射 Wheel Index 到 VESC ID
    // 假设 Wheel 0 -> VESC 0, Wheel 1 -> VESC 1...
    Change_vesc_speed(wheel_index, target_erpm);
    
    // 立即发送 CAN 报文 (如果系统负载允许，也可以放在定时器中断里统一发)
    Com2vesc(wheel_index);
}

void control_init(void)
{
    // 初始化代码
}