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
    float pos_err_abs = fabsf(pos_err);
    
    // 计算当前速度减速到0所需的刹车距离: v^2 / 2a
    float stop_dist = (ramp->current_vel * ramp->current_vel) / (2.0f * ramp->max_decel);
    
    float target_vel = 0.0f;

    // --- 核心修复与优化 ---
    
    // 阈值：决定何时从“曲线减速”切换到“线性靠近”
    // 建议稍微大一点，例如 20~50，保证由于系统延迟导致的超调能被线性区吸收
    float small_err_threshold = 20.0f; 

    // 情况1: 在刹车距离内，或者误差已经进入微调范围
    if (pos_err_abs <= stop_dist || pos_err_abs < small_err_threshold) {
        
        if (pos_err_abs < 1.0f) {
            // [极快响应] 误差极小时直接锁死，防止反复震荡
            target_vel = 0.0f;
            ramp->current_pos = ramp->target_pos; // 强制拉入目标
            ramp->current_vel = 0.0f;
            return ramp->current_pos;
        } 
        else if (pos_err_abs < small_err_threshold) {
            // [关键修复] 线性区 (P控制)
            // 这里的增益 k 必须与 sqrt 曲线在阈值处的切线斜率匹配，保持速度连续
            // 公式: V_boundary = sqrt(2 * a * threshold)
            // Gain = V_boundary / threshold
            // 简化后: Gain = sqrt(2 * a / threshold)
            
            float transition_gain = sqrtf(2.0f * ramp->max_decel / small_err_threshold);
            
            // 为了更快收敛，可以稍微在这个理论增益上乘以 1.2 倍
            target_vel = transition_gain * pos_err; 
        } 
        else {
            // 正常刹车：平方根曲线 v = sqrt(2as)
            float v_limit = sqrtf(2.0f * ramp->max_decel * pos_err_abs);
            target_vel = v_limit * sign(pos_err);
        }
    }
    // 情况2: 距离还很远，全速前进
    else {
        target_vel = ramp->max_vel * sign(pos_err);
    }

    // --- 速度斜坡逻辑 ---

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