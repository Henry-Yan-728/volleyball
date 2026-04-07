#ifndef MOTOR_RAMP_H
#define MOTOR_RAMP_H

#include <stdint.h>
#include "main.h"

// 斜坡控制器结构体，每个电机实例化一个
typedef struct {
    int32_t target_val;   // 目标速度 (RPM)
    int32_t current_val;  // 当前平滑后的速度 (RPM)
    int32_t accel_step;   // 每周期允许的最大加速步长 (RPM/周期)
    int32_t decel_step;   // 每周期允许的最大减速步长 (RPM/周期)
} RampController_t;

/**************** 外部接口 ****************/

/**
 * @brief 初始化斜坡控制器参数
 * @param ramp 控制器实例指针
 * @param accel 每周期加速步长
 * @param decel 每周期减速步长
 */
void Motor_Ramp_Init(RampController_t *ramp, int32_t accel, int32_t decel);

/**
 * @brief 设置新的目标速度
 * @param ramp 控制器实例指针
 * @param target 目标速度值
 */
void Motor_Ramp_SetTarget(RampController_t *ramp, int32_t target);

/**
 * @brief 执行单次斜坡计算 (需按固定周期调用，如 10ms)
 * @param ramp 控制器实例指针
 * @return int32_t 计算后应该下发的当前速度
 */
int32_t Motor_Ramp_Calc(RampController_t *ramp);

/**
 * @brief 紧急停止 (瞬间将目标和当前速度清零)
 * @param ramp 控制器实例指针
 */
void Motor_Ramp_EStop(RampController_t *ramp);

#endif /* MOTOR_RAMP_H */