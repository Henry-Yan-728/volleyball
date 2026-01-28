#include "chassis_path_task.h"
#include <math.h>

static PlannerConfig_t _cfg;
static float _t_x, _t_y, _t_yaw; // 目标值
static float _s_x, _s_y;         // 起点值
static uint8_t _is_running = 0;

// 内部函数：计算两点距离
static float _get_dist(float x1, float y1, float x2, float y2) {
    return sqrtf(powf(x2 - x1, 2) + powf(y2 - y1, 2));
}

// 内部函数：角度归一化 (-PI to PI)
static float _normalize_angle(float angle) {
    while (angle > M_PI)  angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

void Planner_Init(PlannerConfig_t config) {
    _cfg = config;
    _is_running = 0;
}

void Planner_SetTarget(float start_x, float start_y, float target_x, float target_y, float target_yaw) {
    _s_x = start_x; _s_y = start_y;
    _t_x = target_x; _t_y = target_y;
    _t_yaw = target_yaw;
    _is_running = 1;
}

TrajVel_t Planner_Update(float now_x, float now_y, float now_yaw) {
    TrajVel_t out = {0, 0, 0};
    if (!_is_running) return out;

    float dist_to_target = _get_dist(now_x, now_y, _t_x, _t_y);
    float dist_from_start = _get_dist(now_x, now_y, _s_x, _s_y);

    // 1. 标量速度规划 (梯形曲线)
    float speed = _cfg.max_spd;
    // 加速段
    if (dist_from_start < _cfg.up_dist) {
        speed = _cfg.start_spd + (_cfg.max_spd - _cfg.start_spd) * (dist_from_start / _cfg.up_dist);
    }
    // 减速段
    if (dist_to_target < _cfg.down_dist) {
        float decel_speed = _cfg.stop_spd + (_cfg.max_spd - _cfg.stop_spd) * (dist_to_target / _cfg.down_dist);
        if (decel_speed < speed) speed = decel_speed;
    }
    if (speed < _cfg.stop_spd) speed = _cfg.stop_spd;

    // 2. 计算方向向量 (Vx, Vy)
    float angle_to_target = atan2f(_t_y - _s_y, _t_x - _s_x);
    out.vx = speed * cosf(angle_to_target);
    out.vy = speed * sinf(angle_to_target);

    // 3. 计算角速度 (Vr) - 简单的比例控制
    float yaw_err = _normalize_angle(_t_yaw - now_yaw);
    out.vr = yaw_err * _cfg.angle_kp;

    return out;
}

uint8_t Planner_IsArrived(float now_x, float now_y, float now_yaw) {
    if (_get_dist(now_x, now_y, _t_x, _t_y) < 20.0f && 
        fabsf(_normalize_angle(_t_yaw - now_yaw)) < 0.05f) {
        _is_running = 0;
        return 1;
    }
    return 0;
}