#ifndef CHASSIS_PATH_TASK
#define CHASSIS_PATH_TASK

#include "main.h"

// 速度规划配置结构体
typedef struct {
    float max_spd;      // 最大平移速度 (mm/s)
    float start_spd;    // 起始速度
    float stop_spd;     // 终点速度
    float up_dist;      // 加速距离 (mm)
    float down_dist;    // 减速距离 (mm)
    float angle_kp;     // 角度环比例系数
} PlannerConfig_t;

// 速度输出结构体
typedef struct {
    float vx; // 世界坐标系 x 速度
    float vy; // 世界坐标系 y 速度
    float vr; // 角速度 (rad/s 或 对应控制量)
} TrajVel_t;

/* 外部接口 */
void Planner_Init(PlannerConfig_t config);
void Planner_SetTarget(float start_x, float start_y, float target_x, float target_y, float target_yaw);
TrajVel_t Planner_Update(float now_x, float now_y, float now_yaw);
uint8_t Planner_IsArrived(float now_x, float now_y, float now_yaw);

#endif