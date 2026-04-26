#ifndef PAN_TILT_CONTROL_H
#define PAN_TILT_CONTROL_H

#include "main.h"

// 初始化云台电机
void gimbal_control_init(void);

// 设置云台速度 (单位: RPM)
void gimbal_set_speed(float yaw_speed_cmd, float pitch_target_deg);

// 获取云台当前角度 (单位: 度)
void gimbal_get_angles(float* yaw_angle_deg, float* pitch_angle_deg);

#endif // PAN_TILT_CONTROL_H
