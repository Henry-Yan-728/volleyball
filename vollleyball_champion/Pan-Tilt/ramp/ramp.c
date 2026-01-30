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
    float stop_dist = (ramp->current_vel * ramp->current_vel) / (2.0f * ramp->max_decel);
    
    float target_vel = 0.0f;

    // 逻辑：如果当前距离足以刹车，且还有剩余距离，则允许加速或保持最大速度
    // 否则，必须开始减速
    
    if (fabsf(pos_err) <= stop_dist) {
        // 接近目标，需要减速
        // 期望速度方向指向目标，大小随距离减小
        // 这里使用平方根曲线平滑逼近： v = sqrt(2 * a * s) * sign(err)
        // 但为了防止震荡，在极小误差时直接用 P 控制或直接归零
        float v_limit = sqrtf(2.0f * ramp->max_decel * fabsf(pos_err));
        target_vel = v_limit * sign(pos_err);
    } else {
        // 距离很远，全速前进
        target_vel = ramp->max_vel * sign(pos_err);
    }

    // 速度斜坡处理 (限制加速度)
    float vel_err = target_vel - ramp->current_vel;
    float max_vel_change = ramp->max_accel * dt;

    if (fabsf(vel_err) > max_vel_change) {
        ramp->current_vel += max_vel_change * sign(vel_err);
    } else {
        ramp->current_vel = target_vel;
    }

    // 积分得到位置
    ramp->current_pos += ramp->current_vel * dt;

    return ramp->current_pos;
}