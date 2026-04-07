/**
  ******************************************************************************
  * @file    : chassis_cybergear.h
  * @brief   : 小米CyberGear底盘控制封装接口
  ******************************************************************************
  * @note 使用流程
  *
  *   1. 在 FreeRTOS 任务体中（调度器启动后）调用一次 chassis_cybergear_init()。
  *   2. 之后在任意任务中调用 chassis_control(Angle) 设置目标角度。
  *   3. 如需紧急制动，调用 chassis_cybergear_stop()。
  *
  * @note chassis_control 的 Angle 单位为【度(°)】，内部自动转换为弧度后交给
  *   trajectory_planner，规划器以速控方式平滑驱动电机到达目标位置。
  ******************************************************************************
  */

#ifndef CHASSIS_CYBERGEAR_H
#define CHASSIS_CYBERGEAR_H

#include "cybergear_motor.h"

/**
 * @brief 底盘初始化
 * @note  必须在 FreeRTOS 调度器启动后、fdcan_bsp 已初始化完成后调用
 * @param motor_index  电机在全局数组中的索引（通常为 0）
 * @retval  0  成功
 * @retval -1  失败（电机实例为空或规划器启动失败）
 */
int8_t chassis_cybergear_init(uint8_t motor_index);

/**
 * @brief 底盘角度控制
 * @param angle_deg  目标角度，单位：度(°)
 */
void chassis_control(float angle_deg);

/**
 * @brief 底盘速度直接控制（跳过位置规划，适合遥控跟速场景）
 * @param speed_rad_s  目标速度，单位：rad/s，范围 [-30, 30]
 */
void chassis_control_speed(float speed_rad_s);

/**
 * @brief 紧急停止，立即制动电机
 */
void chassis_cybergear_stop(void);

/**
 * @brief 获取底盘当前角度
 * @retval 当前电机角度，单位：rad；未初始化时返回 0.0f
 */
float chassis_get_angle_rad(void);

#endif // CHASSIS_CYBERGEAR_H
