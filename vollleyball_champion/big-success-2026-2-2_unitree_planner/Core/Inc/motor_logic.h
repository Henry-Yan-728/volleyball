#ifndef MOTOR_LOGIC_H
#define MOTOR_LOGIC_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// --- 结构体定义 (原 motor_profile 的内容) ---
typedef struct {
    float start_position;     // 起始位置 (rad)
    float target_position;    // 目标位置 (rad)
    float max_speed;          // 最大速度 (rad/s)
    float acceleration;       // 加速度 (rad/s^2)
    float deceleration;       // 减速度 (rad/s^2)

    float t_accel;            // 加速时间
    float t_const;            // 匀速时间
    float t_total;            // 总时间

    float peak_speed;         // 峰值速度
    float total_distance;     // 总距离
    float direction;          // 方向
    
    bool is_finished;         // 是否完成
} TrapezoidProfile;

// --- 对外功能接口  ---

/**
 * @brief 初始化并执行回零 (阻塞式)
 */
void Motor_InitZeroing(uint32_t motorId);

/**
 * @brief 发送运动指令
 * @param target_pos 目标位置 (相对零点)
 * @param max_v 最大速度
 * @param max_a 加速度
 */
void Motor_MoveTo(float target_pos, float max_v, float max_a);

/**
 * @brief 控制循环更新 (需在高频任务中调用)
 * @param elapsed_ms 任务周期 (ms)
 */
void Motor_UpdateLoop(uint32_t elapsed_ms);

/**
 * @brief 获取当前运动状态
 */
bool Motor_IsMoving(void);

#endif // MOTOR_LOGIC_H