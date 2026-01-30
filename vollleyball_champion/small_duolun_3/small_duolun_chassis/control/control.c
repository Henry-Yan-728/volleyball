#include "control.h"
#include "dji_motor.h"
#include "vesc.h"
#include <math.h>

#define PI 3.1415926535f

// 辅助函数：角度标准化到 -180 ~ 180
static float normalize_angle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 将物理速度 (m/s) 转换为 VESC 的 ERPM
 * 公式: Speed = (RPM / 60) * 2 * PI * R / Ratio
 * ERPM = RPM * PolePairs
 */
static int32_t Speed_to_ERPM(float speed_ms)
{
    // 1. 计算物理转速 RPM (Revolutions Per Minute)
    // RPM = (Speed / (2 * PI * R)) * 60 * Ratio
    float wheel_rpm = (speed_ms / (2.0f * PI * WHEEL_RADIUS)) * 60.0f;
    float motor_rpm = wheel_rpm * DRIVE_GEAR_RATIO;

    // 2. 计算电转速 ERPM (Electrical RPM)
    int32_t erpm = (int32_t)(motor_rpm * VESC_POLE_PAIRS);
    
    return erpm;
}

/**
 * @brief 舵轮控制核心逻辑 (Swerve Optimization)
 */
void Steering_Wheel_Control(uint8_t wheel_index, float target_spd, float target_ang)
{
    // 1. 获取 DJI 舵向电机实例
    DJI_Motor_Instance* steering_motor = dji_motor_get_instance(wheel_index);
    if (steering_motor == NULL) return;

    // ============================================================
    // 【修改点 1】速度死区保护
    // 如果目标速度极小，只需让动力电机停转，不要转动舵机（防止抖动）
    // ============================================================
    if (fabsf(target_spd) < SPEED_DEADBAND)
    {
        // 停止 VESC
        Change_vesc_speed(wheel_index, 0);
        Com2vesc(wheel_index); 
        
        // 舵机保持当前位置不动 (不发送新的 set_location 指令，或者设为当前测量值)
        // 这样可以避免摇杆归中时轮子乱转
        return; 
    }

    // 2. 获取当前物理角度 (Degree)
    // 公式: (Total_Encoder / 8192) / Ratio * 360
    float current_angle_deg = (float)steering_motor->measure.total_angle / 8192.0f / STEERING_GEAR_RATIO * 360.0f;
    
    // 3. 优化路径计算 (最短路径 + 反转逻辑)
    float current_heading = fmodf(current_angle_deg, 360.0f);
    current_heading = normalize_angle(current_heading);

    float delta_angle = normalize_angle(target_ang - current_heading);
    
    int direction = 1; // 动力电机方向系数

    // 舵轮逻辑核心：如果转角超过 90 度，则转相反方向并反转动力电机
    if (fabsf(delta_angle) > 90.0f)
    {
        if (delta_angle > 0) delta_angle -= 180.0f;
        else                 delta_angle += 180.0f;
        
        direction = -1; // 倒车
    }

    // 4. 计算最终目标 Encoder 值
    // 目标 = 当前累计 + 偏差
    float final_target_deg = current_angle_deg + delta_angle;

    // 转换为编码器值 (注意乘除顺序保持精度)
    int32_t target_encoder = (int32_t)(final_target_deg / 360.0f * STEERING_GEAR_RATIO * 8192.0f);
    
    // 下发舵向指令
    dji_motor_set_location(steering_motor, target_encoder);

    // 5. 控制 VESC 动力电机
    // 【修改点 2】使用物理参数计算 ERPM
    int32_t target_erpm = Speed_to_ERPM(target_spd) * direction;

    Change_vesc_speed(wheel_index, target_erpm);
    
    // 发送 CAN (注意：确保这里调用的频率不要过高，建议 10ms-20ms 一次)
    Com2vesc(wheel_index);
}

void control_init(void)
{
    // 如果有初始化的参数可以写在这里
}