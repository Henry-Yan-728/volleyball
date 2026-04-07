#include "ramp.h"

// 辅助函数：符号函数
static float sign(float x) {
    if (x > 0) return 1.0f;
    if (x < 0) return -1.0f;
    return 0.0f;
}

void ramp_init(Ramp_Handle_t* ramp, float max_vel, float max_accel, float max_decel) {
    ramp->current_pos = 0.0f;
    ramp->current_vel = 0.0f;
    ramp->target_pos = 0.0f;
    ramp->max_vel = max_vel;
    ramp->max_accel = max_accel;
    ramp->max_decel = max_decel;
}

void ramp_set_target(Ramp_Handle_t* ramp, float target) {
    ramp->target_pos = target;
}

float ramp_calc(Ramp_Handle_t* ramp, float dt) {
    float pos_err = ramp->target_pos - ramp->current_pos;
    
    // 计算当前速度减速到0所需的刹车距离: v^2 / 2a
    float stop_dist = (ramp->current_vel * ramp->current_vel) / (2.0f * ramp->max_decel);
    
    float target_vel = 0.0f;

    // --- 核心修复开始 ---
    
    // 情况1: 我们已经在刹车距离内了 (或者误差很小)，需要减速或微调
    if (fabsf(pos_err) <= stop_dist) {
        
        float small_err_threshold = 5.0f; // 死区阈值
        
        if (fabsf(pos_err) < small_err_threshold) {
            // 微小误差：P控制或直接停止
            target_vel = 2.0f * pos_err; 
        } 
        else {
            // 正常刹车：平方根曲线
            float v_limit = sqrtf(2.0f * ramp->max_decel * fabsf(pos_err));
            target_vel = v_limit * sign(pos_err);
        }
    }
    // 情况2: 距离还很远，全速前进！ (你漏掉了这个 else 分支)
    else {
        target_vel = ramp->max_vel * sign(pos_err);
    }

    // --- 核心修复结束 ---

    // 下面是原本的速度斜坡(S-Curve/Trapezoid)生成逻辑，保持不变
    float vel_err = target_vel - ramp->current_vel;
    float max_vel_change = ramp->max_accel * dt;

    if (fabsf(vel_err) > max_vel_change) {
        ramp->current_vel += max_vel_change * sign(vel_err);
    } else {
        ramp->current_vel = target_vel;
    }

    ramp->current_pos += ramp->current_vel * dt;

    return ramp->current_pos;
}