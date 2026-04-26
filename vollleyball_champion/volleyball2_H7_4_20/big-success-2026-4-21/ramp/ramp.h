#ifndef RAMP_H
#define RAMP_H

#include <stdint.h>
#include <math.h>

typedef struct {
    float current_pos;  // 当前规划出的位置 (SetPoint)
    float current_vel;  // 当前规划出的速度
    float target_pos;   // 最终想要到达的目标位置
    
    float max_vel;      // 最大速度 (单位/秒)
    float max_accel;    // 最大加速度 (单位/秒^2)
    float max_decel;    // 最大减速度 (单位/秒^2)
    
} Ramp_Handle_t;

// 初始化
void ramp_init(Ramp_Handle_t* ramp, float max_vel, float max_accel, float max_decel);

// 设置新目标
void ramp_set_target(Ramp_Handle_t* ramp, float target);

// 核心计算函数 (需周期性调用，例如 1ms 一次)
// dt: 调用周期，单位秒 (例如 0.001f)
// 返回值: 这一时刻应该发送给电机的平滑位置
float ramp_calc(Ramp_Handle_t* ramp, float dt);

#endif