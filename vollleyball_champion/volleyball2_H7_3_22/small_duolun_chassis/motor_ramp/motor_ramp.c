#include "motor_ramp.h"

void Motor_Ramp_Init(RampController_t *ramp, int32_t accel, int32_t decel) {
    if (ramp == NULL) return;
    
    ramp->target_val = 0;
    ramp->current_val = 0;
    // 确保步长为正数，简化后续逻辑
    ramp->accel_step = (accel > 0) ? accel : -accel; 
    ramp->decel_step = (decel > 0) ? decel : -decel;
}

void Motor_Ramp_SetTarget(RampController_t *ramp, int32_t target) {
    if (ramp == NULL) return;
    ramp->target_val = target;
}

int32_t Motor_Ramp_Calc(RampController_t *ramp) {
    if (ramp == NULL) return 0;

    int32_t target = ramp->target_val;
    int32_t current = ramp->current_val;
    int32_t step = 0;

    if (current < target) {
        // 当前速度小于目标速度，需要增加
        if (current >= 0) {
            step = ramp->accel_step; // 正向加速
        } else {
            step = ramp->decel_step; // 反向减速 (刹车，向0靠近)
        }
        
        current += step;
        if (current > target) {
            current = target; // 防止超调
        }
    } 
    else if (current > target) {
        // 当前速度大于目标速度，需要减小
        if (current <= 0) {
            step = ramp->accel_step; // 反向加速 (向更小的负数靠近)
        } else {
            step = ramp->decel_step; // 正向减速 (刹车，向0靠近)
        }
        
        current -= step;
        if (current < target) {
            current = target; // 防止超调
        }
    }

    ramp->current_val = current;
    return current;
}

void Motor_Ramp_EStop(RampController_t *ramp) {
    if (ramp == NULL) return;
    ramp->target_val = 0;
    ramp->current_val = 0;
}