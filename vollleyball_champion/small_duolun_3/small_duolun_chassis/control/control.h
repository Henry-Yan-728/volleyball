#ifndef CONTROL_H
#define CONTROL_H

#include "main.h"

// 机械参数定义
// DJI 2006电机减速比为 36:1
// 如果你的舵轮是直接连接电机轴，或者是其他减速比，请修改此处
#define STEERING_GEAR_RATIO  36.0f 

// VESC电机极对数 (根据实际电机修改)
// 用于将 RPM 转换为 ERPM
#define VESC_POLE_PAIRS      7 

void control_init(void);

/**
 * @brief 舵轮底层控制函数
 * @param wheel_index 轮子编号 (0-2)
 * @param target_speed 目标速度 (m/s) (会转换为VESC的ERPM)
 * @param target_angle_deg 目标角度 (度) (-180 ~ 180)
 */
void Steering_Wheel_Control(uint8_t wheel_index, float target_speed, float target_angle_deg);

#endif