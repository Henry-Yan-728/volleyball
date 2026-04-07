#ifndef CONTROL_H
#define CONTROL_H

#include "main.h"

typedef float elem_type;

// ================= 机械与控制参数 =================

// 1. 舵向参数 (DJI 2006)
// 2006电机自带减速比 36:1，外加 3:1 减速，总减速比 = 108.0f
#define STEERING_GEAR_RATIO      108.0f  
#define STEERING_POLARITY        1.0f    // 极性翻转：如原地打转请改为 -1.0f

// 2. 动力参数 (VESC)
#define WHEEL_RADIUS             0.05f   // 轮子半径 (米)
#define DRIVE_GEAR_RATIO         1.0f    // 动力轮减速比
#define VESC_POLE_PAIRS          12      // VESC 极对数

// ================= 函数声明 =================

void control_init(void);

// 提取自 wheel_control 的偏差计算函数暴露给外部（可选）
elem_type get_delta_angle(float target_angle, float now_angle, int* direction);

/**
 * @brief 2006 + VESC 舵轮底层控制函数
 * @param wheel_index 轮子编号 (0-2)
 * @param target_speed 目标平移速度 (m/s)
 * @param target_angle_deg 目标舵向角度 (度) (-180 ~ 180)
 */
void Steering_Wheel_Control(uint8_t wheel_index, float target_speed, float target_angle_deg);

#endif // CONTROL_H