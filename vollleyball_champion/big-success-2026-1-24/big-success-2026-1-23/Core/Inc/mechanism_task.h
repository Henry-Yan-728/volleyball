#ifndef MECHANISM_TASK_H
#define MECHANISM_TASK_H

#include "main.h"
#include <stdbool.h>

// =============================================================
//  电机 ID 定义 (CAN3)
// =============================================================

// 宇树电机 ID (0, 1, 2 用于垫球，3 用于发球)
#define UNITREE_ID_CUSHION_1  1
#define UNITREE_ID_CUSHION_2  2
#define UNITREE_ID_CUSHION_3  3
#define UNITREE_ID_SERVE      0

// 大疆电机索引 (在 dji_motors 数组中的位置)
// 定义两个大疆电机的索引 (假设是 4 和 5)
#define DJI_INDEX_AXIS_MASTER  4  // 正转电机 (俯仰电机)
#define DJI_INDEX_AXIS_SLAVE   5  // 反转电机 (辅助电机)

// 机构角度定义 (角度制)
#define CUSHION_ANGLE_START   37.0f   // 起始位置
#define CUSHION_ANGLE_HIT     97.0f   // 击球位置 (最大速度点)
#define CUSHION_ANGLE_END     114.0f  // 停止位置

// 减速比
#define REDUCTION_RATIO       6.33f

typedef struct {
    // 状态标志
    bool is_moving;
    bool is_returning; // 是否处于复位阶段
    
    // 轨迹参数 (单位: 输出轴弧度 rad)
    float start_pos;
    float hit_pos;     // 速度峰值点
    float end_pos;
    
    // 运动学参数
    float max_vel;     // 设定的峰值速度 (rad/s)
    float accel;       // 加速度 (rad/s^2)
    float decel;       // 减速度 (rad/s^2)
    
    // 运行时变量
    float current_time;
    float total_time_acc; // 加速段时间
    float total_time_dec; // 减速段时间
    
    // 输出给电机的指令
    float out_pos_rad;   // 输出轴目标弧度
    float out_vel_rad;   // 输出轴目标角速度
    float out_torque;    // 前馈力矩

} Cushion_Profile_t;

typedef struct {
    float current_angle;    // 虚拟轴当前的实时角度 (度)
    float target_angle;     // 最终想要到达的角度 (度)
    float velocity_deg_ms;  // 速度：每毫秒转多少度 (0.1度/ms = 100度/秒)
    uint32_t last_tick;     // 上次更新的时间戳
} Virtual_Axis_t;

// =============================================================
//  函数声明
// =============================================================

// 初始化所有上层机构
void Mechanism_Init(void);

/**
 * @brief 控制垫球机构 (3个宇树电机同步运动)
 * @param angle_rad: 目标角度 (弧度)
 * @param kp: 刚度系数 (建议 0.05 ~ 0.5)
 */
void Mechanism_Cushion_Trigger(float peak_speed_deg_s);

// 复位动作 (慢速回到起点)
void Mechanism_Cushion_Return(void);

// 轨迹更新函数 (需放入 1ms 高频任务)
void Mechanism_Cushion_Update_Loop(uint32_t dt_ms);

/**
 * @brief 控制发球机构 (1个宇树电机)
 * @param speed_rad_s: 目标转速 (rad/s)
 * @note  宇树电机转速较高，注意安全
 */
void Mechanism_Serve_SetAngle(float speed_rad_s);

/**
 * @brief 控制俯仰机构 (1个大疆 M3508)
 * @param angle_deg: 目标角度 (角度制，例如 45.0 度)
 */
void Mechanism_Pitch_SetAngle(float angle_deg);

void Mechanism_Zero();

void Update_Virtual_Axis(void);

#endif // MECHANISM_TASK_H