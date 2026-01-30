#ifndef CONTROL_H
#define CONTROL_H

#include "main.h"

// ================= 机械参数 (需根据实际测量修改) =================

// 1. 舵向参数
// DJI 2006电机减速比为 36:1 (36圈电机轴 = 1圈输出轴)
#define STEERING_GEAR_RATIO  36.0f 

// 2. 动力参数 (VESC)
#define WHEEL_RADIUS         0.05f   // 轮子半径 (单位: 米) (例如直径10cm的轮子，半径0.05)
#define DRIVE_GEAR_RATIO     1.0f    // 动力电机减速比 (如果是直驱则为 1.0)
#define VESC_POLE_PAIRS      12       // 电机极对数 (一般轮毂电机是 15，航模电机是 7，需查手册)

// 3. 控制阈值
#define SPEED_DEADBAND       0.05f   // 速度死区 (m/s)，小于此速度不转动舵机

// ===============================================================

void control_init(void);

/**
 * @brief 舵轮底层控制函数
 * @param wheel_index 轮子编号 (0-2)
 * @param target_speed 目标速度 (m/s)
 * @param target_angle_deg 目标角度 (度) (-180 ~ 180)
 */
void Steering_Wheel_Control(uint8_t wheel_index, float target_speed, float target_angle_deg);

#endif // CONTROL_H