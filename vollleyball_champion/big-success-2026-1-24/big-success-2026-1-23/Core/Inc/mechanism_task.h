#ifndef MECHANISM_TASK_H
#define MECHANISM_TASK_H

#include "main.h"

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
void Mechanism_Cushion_SetAngle(float angle_rad, float kp);

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