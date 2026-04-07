#ifndef MECHANISM_TASK_H
#define MECHANISM_TASK_H

#include "main.h"

// =============================================================
//  核心机械参数配置
// =============================================================
// 宇树电机减速比 (原代码中的 6.33)
#define UNITREE_REDUCTION_RATIO  6.33f  

// 外部机械臂/滑轨传动比 
// 含义：电机(M3508 19.2:1) 
#define EXTERNAL_MECHANISM_RATIO (19.2032f*3)

// =============================================================
//  电机 ID 定义 (CAN3)
// =============================================================
#define UNITREE_ID_CUSHION_1  1
#define UNITREE_ID_CUSHION_2  2
#define UNITREE_ID_CUSHION_3  3
#define UNITREE_ID_SERVE      0

#define DJI_INDEX_AXIS_MASTER  4
#define DJI_INDEX_AXIS_SLAVE   5

// 全局速度目标值 (单位: RPM)
extern float current_yaw_speed;
extern float current_pitch_speed;

typedef struct {
    float current_angle;    // 虚拟轴当前的实时角度 (度)
    float target_angle;     // 最终想要到达的角度 (度)
    float velocity_deg_ms;  // 速度：每毫秒转多少度 
    uint32_t last_tick;     // 上次更新的时间戳
} Virtual_Axis_t;

// =============================================================
//  函数声明
// =============================================================

void Mechanism_Init(void);

/**
 * @brief 控制垫球机构 (3个宇树电机同步运动)
 * @param angle_deg: 目标角度 (度)
 * @param kp: 刚度系数 
 */
void Mechanism_Cushion_SetAngle(float angle_deg, float kp);

/**
 * @brief 控制发球机构 (1个宇树电机)
 * @param angle_deg: 目标值 (度制，根据模式不同可能为位置或速度)
 */
void Mechanism_Serve_SetAngle(float angle_deg);

/**
 * @brief 控制俯仰机构 
 */
void Mechanism_Dian_Pitch_SetAngle(float angle_deg);

void Mechanism_Zero(void);

void Update_Virtual_Axis(void);

void Mechanism_Loop_1ms(void); 

#endif // MECHANISM_TASK_H