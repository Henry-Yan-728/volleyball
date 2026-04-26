#include "chassis_path_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.1415926535f
#endif
#define PLANNER_MIN_DIST_EPSILON 1e-3f

static PlannerConfig_t _cfg;
static float _t_x, _t_y, _t_yaw; // 目标值
static float _s_x, _s_y;         // 起点值
static uint8_t _is_running = 0;
static float _last_vr = 0.0f;

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

static float _clampf(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float _apply_soft_deadzone(float value, float deadzone) {
    float abs_value = fabsf(value);

    if (deadzone <= 0.0f) {
        return value;
    }

    if (abs_value <= deadzone) {
        return 0.0f;
    }

    if (value > 0.0f) {
        return value - deadzone;
    }

    return value + deadzone;
}

static PlannerConfig_t _sanitize_config(PlannerConfig_t config) {
    if (config.max_spd < 0.0f) {
        config.max_spd = 0.0f;
    }
    if (config.start_spd < 0.0f) {
        config.start_spd = 0.0f;
    }
    if (config.stop_spd < 0.0f) {
        config.stop_spd = 0.0f;
    }
    if (config.max_vr < 0.0f) {
        config.max_vr = 0.0f;
    }
    if (config.vr_slew_step < 0.0f) {
        config.vr_slew_step = 0.0f;
    }
    if (config.far_near_dist <= PLANNER_MIN_DIST_EPSILON) {
        config.far_near_dist = 500.0f;
    }
    config.far_weight_min = _clampf(config.far_weight_min, 0.0f, 1.0f);
    if (config.far_max_vr_scale <= 0.0f) {
        config.far_max_vr_scale = 1.0f;
    }
    if (config.far_vr_slew_scale <= 0.0f) {
        config.far_vr_slew_scale = 1.0f;
    }
    if (config.pos_tolerance <= 0.0f) {
        config.pos_tolerance = 20.0f;
    }
    if (config.yaw_tolerance <= 0.0f) {
        config.yaw_tolerance = 0.05f;
    }
    if (config.yaw_deadzone < 0.0f) {
        config.yaw_deadzone = 0.0f;
    }

    return config;
}

static PlannerConfig_t _get_config_snapshot(void) {
    PlannerConfig_t config;

    taskENTER_CRITICAL();
    config = _cfg;
    taskEXIT_CRITICAL();

    return config;
}

static uint8_t _pos_arrived(float now_x, float now_y, const PlannerConfig_t *cfg) {
    return (uint8_t)(_get_dist(now_x, now_y, _t_x, _t_y) <= cfg->pos_tolerance);
}

static uint8_t _yaw_arrived(float now_yaw, const PlannerConfig_t *cfg) {
    if (cfg->ignore_yaw != 0U) {
        return 1U;
    }

    return (uint8_t)(fabsf(_normalize_angle(_t_yaw - now_yaw)) <= cfg->yaw_tolerance);
}

static float _compute_dynamic_down_dist(float current_speed, const PlannerConfig_t *cfg) {
    float down_dist = cfg->down_dist;

    if (down_dist <= PLANNER_MIN_DIST_EPSILON) {
        return PLANNER_MIN_DIST_EPSILON;
    }

    if ((cfg->max_spd <= cfg->stop_spd) || (current_speed <= cfg->stop_spd)) {
        return down_dist;
    }

    {
        float decel = ((cfg->max_spd * cfg->max_spd) - (cfg->stop_spd * cfg->stop_spd)) / (2.0f * down_dist);

        if (decel <= PLANNER_MIN_DIST_EPSILON) {
            return down_dist;
        }

        {
            float braking_dist = ((current_speed * current_speed) - (cfg->stop_spd * cfg->stop_spd)) / (2.0f * decel);
            if (braking_dist < 0.0f) {
                braking_dist = 0.0f;
            }
            if (braking_dist > down_dist) {
                down_dist = braking_dist;
            }
        }
    }

    return down_dist;
}

static float _apply_vr_slew(float target_vr, float slew_step) {
    float delta_vr = target_vr - _last_vr;

    if (slew_step > 0.0f) {
        delta_vr = _clampf(delta_vr, -slew_step, slew_step);
    }

    _last_vr += delta_vr;
    return _last_vr;
}

void Planner_Init(PlannerConfig_t config) {
    Planner_SetConfig(&config);

    _is_running = 0;
    _last_vr = 0.0f;
}

void Planner_SetConfig(const PlannerConfig_t *config) {
    if (config == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    _cfg = _sanitize_config(*config);
    taskEXIT_CRITICAL();
}

void Planner_GetConfig(PlannerConfig_t *config_out) {
    if (config_out == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *config_out = _cfg;
    taskEXIT_CRITICAL();
}

void Planner_SetTarget(float start_x, float start_y, float target_x, float target_y, float target_yaw) {
    _s_x = start_x; _s_y = start_y;
    _t_x = target_x; _t_y = target_y;
    _t_yaw = target_yaw;
    _is_running = 1;
    _last_vr = 0.0f;
}

TrajVel_t Planner_Update(float now_x, float now_y, float now_yaw, float now_vx, float now_vy) {
    PlannerConfig_t cfg = _get_config_snapshot();
    TrajVel_t out = {0, 0, 0};
    if (!_is_running) {
        _last_vr = 0.0f;
        return out;
    }

    float dist_to_target = _get_dist(now_x, now_y, _t_x, _t_y);
    float dist_from_start = _get_dist(now_x, now_y, _s_x, _s_y);
    float yaw_err = _normalize_angle(_t_yaw - now_yaw);
    float yaw_ctrl_err = _apply_soft_deadzone(yaw_err, cfg.yaw_deadzone);
    float abs_yaw_ctrl_err = fabsf(yaw_ctrl_err);
    float translation_weight = 1.0f;
    float current_speed = sqrtf((now_vx * now_vx) + (now_vy * now_vy));
    float dynamic_down_dist = _compute_dynamic_down_dist(current_speed, &cfg);
    float far_mode_ratio = 0.0f;
    float current_max_vr = cfg.max_vr;
    float current_vr_slew_step = cfg.vr_slew_step;

    if (cfg.ignore_yaw == 0U) {
        float strict_weight;
        float far_weight;

        far_mode_ratio = _clampf(dist_to_target / cfg.far_near_dist, 0.0f, 1.0f);

        strict_weight = cosf(abs_yaw_ctrl_err);
        if (strict_weight < 0.0f) {
            strict_weight = 0.0f;
        }

        far_weight = cfg.far_weight_min + ((1.0f - cfg.far_weight_min) * strict_weight);
        translation_weight = strict_weight + (far_mode_ratio * (far_weight - strict_weight));
        current_max_vr *= 1.0f + (far_mode_ratio * (cfg.far_max_vr_scale - 1.0f));
        current_vr_slew_step *= 1.0f + (far_mode_ratio * (cfg.far_vr_slew_scale - 1.0f));
    }

    if (_pos_arrived(now_x, now_y, &cfg) != 0U) {
        if (_yaw_arrived(now_yaw, &cfg) == 0U) {
            float target_vr = yaw_ctrl_err * cfg.angle_kp;
            target_vr = _clampf(target_vr, -current_max_vr, current_max_vr);
            out.vr = _apply_vr_slew(target_vr, current_vr_slew_step);
        } else {
            _last_vr = 0.0f;
        }
        return out;
    }

    // 1. 标量速度规划 (梯形曲线)
    float speed = cfg.max_spd;
    // 加速段
    if ((cfg.up_dist > PLANNER_MIN_DIST_EPSILON) && (dist_from_start < cfg.up_dist)) {
        speed = cfg.start_spd + (cfg.max_spd - cfg.start_spd) * (dist_from_start / cfg.up_dist);
    }
    // 减速段
    if ((dynamic_down_dist > PLANNER_MIN_DIST_EPSILON) && (dist_to_target < dynamic_down_dist)) {
        float decel_speed = cfg.stop_spd + (cfg.max_spd - cfg.stop_spd) * (dist_to_target / dynamic_down_dist);
        if (decel_speed < speed) speed = decel_speed;
    }
    speed = _clampf(speed, cfg.stop_spd, cfg.max_spd);

    // 2. 闭环计算方向向量，始终从当前位置指向目标点
    float angle_to_target = atan2f(_t_y - now_y, _t_x - now_x);
    out.vx = speed * cosf(angle_to_target) * translation_weight;
    out.vy = speed * sinf(angle_to_target) * translation_weight;

    // 3. 计算角速度 (Vr) - 简单的比例控制
    if (cfg.ignore_yaw == 0U) {
        float target_vr = yaw_ctrl_err * cfg.angle_kp;
        target_vr = _clampf(target_vr, -current_max_vr, current_max_vr);
        out.vr = _apply_vr_slew(target_vr, current_vr_slew_step);
    } else {
        _last_vr = 0.0f;
    }

    return out;
}

uint8_t Planner_IsArrived(float now_x, float now_y, float now_yaw) {
    PlannerConfig_t cfg = _get_config_snapshot();

    if ((_pos_arrived(now_x, now_y, &cfg) != 0U) &&
        (_yaw_arrived(now_yaw, &cfg) != 0U)) {
        _is_running = 0;
        _last_vr = 0.0f;
        return 1U;
    }
    return 0U;
}
