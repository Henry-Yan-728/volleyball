/**
  ******************************************************************************
  * @file    : cybergear_speed_planner.h
  * @brief   : 小米 CyberGear 电机速控规划器接口
  ******************************************************************************
  * @note 功能说明
  *
  *   本模块将小米CyberGear电机置于【速度控制模式】，并提供带加速度限幅的
  *   速度平滑规划。用户只需设置目标速度（rad/s），规划器内部会以设定的最大
  *   加速度斜率平滑地将当前速度爬升/降低到目标速度，再周期性下发给电机。
  *
  * @note 典型使用流程
  *
  *   1. 在 FreeRTOS 任务启动前调用 cybergear_speed_planner_init() 完成初始化。
  *   2. 获取电机实例后调用 cybergear_speed_planner_start(motor) 使能电机并启动规划任务。
  *   3. 在任何任务中调用 cybergear_speed_planner_set_target_speed(speed) 设置目标速度。
  *   4. 需要停止时调用 cybergear_speed_planner_emergency_stop() 立即清零速度并释放电机。
  *
  * @note 参数调整
  *
  *   修改本文件中的宏：
  *     SPEED_PLANNER_MAX_SPEED       最大允许速度 (rad/s)
  *     SPEED_PLANNER_MAX_ACCEL       最大加速度   (rad/s^2)
  *     SPEED_PLANNER_CONTROL_PERIOD  控制周期     (ms)
  ******************************************************************************
  */

#ifndef CYBERGEAR_SPEED_PLANNER_H
#define CYBERGEAR_SPEED_PLANNER_H

#include "cybergear_motor.h"
#include <stdint.h>

/* ======================== 用户调参区 ======================== */

/** 最大允许速度 (rad/s)，超出此值的指令会被截断 */
#define SPEED_PLANNER_MAX_SPEED        (10.0f)

/** 最大加速度 / 减速度 (rad/s^2)，越大响应越快、冲击越大 */
#define SPEED_PLANNER_MAX_ACCEL        (5.0f)

/** 控制任务周期 (ms)，决定速度更新频率 */
#define SPEED_PLANNER_CONTROL_PERIOD   (10U)

/* ======================== 公共接口 ======================== */

/**
 * @brief  初始化速控规划器（创建 FreeRTOS 任务与互斥锁）
 * @note   必须在 FreeRTOS 调度器启动前调用，或在初始化任务中调用一次。
 * @retval  0  成功
 * @retval -1  失败（资源不足）
 */
int8_t cybergear_speed_planner_init(void);

/**
 * @brief  绑定电机实例并启动规划任务
 * @note   电机将被自动切换到速度模式并使能。
 *         初始目标速度为 0，电机静止。
 * @param  motor  由 cybergear_motor_register() 返回的电机实例指针
 * @retval  0  成功
 * @retval -1  参数无效或初始化未完成
 */
int8_t cybergear_speed_planner_start(CyberGear_Motor_Instance *motor);

/**
 * @brief  设置电机目标速度
 * @note   线程安全，可在任意 FreeRTOS 任务中调用。
 *         速度会被限幅在 [-SPEED_PLANNER_MAX_SPEED, +SPEED_PLANNER_MAX_SPEED]。
 *         实际输出速度受最大加速度约束平滑过渡。
 * @param  target_speed_rad_s  目标速度，单位 rad/s（正方向取决于电机安装方向）
 */
void cybergear_speed_planner_set_target_speed(float target_speed_rad_s);

/**
 * @brief  获取当前规划器输出的实时速度
 * @retval 当前规划输出速度 (rad/s)
 */
float cybergear_speed_planner_get_current_speed(void);

/**
 * @brief  紧急停止：立即将目标速度和输出速度清零，并停止电机
 * @note   不同于 set_target_speed(0)（后者会平滑减速），
 *         本函数立即切断指令、停止电机使能。
 */
void cybergear_speed_planner_emergency_stop(void);

#endif /* CYBERGEAR_SPEED_PLANNER_H */
