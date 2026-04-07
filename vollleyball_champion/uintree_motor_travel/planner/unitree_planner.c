#include "unitree_planner.h"
#include <math.h>
#include <string.h>

#define DEG_TO_RAD(x) ((x) * (PI / 180.0f))

// --- 调试参数 ---
// 如果仍然超调，将此值改小 (例如 0.5f)。设为 0.0f 则完全关闭速度前馈（纯位置控制）
#define VEL_FF_GAIN  1.8f 

void Unitree_Init(UnitreePlanner_t *planner, uint32_t motor_id) {
    memset(planner, 0, sizeof(UnitreePlanner_t));
    planner->motor_id = motor_id;
    planner->is_moving = false;
    planner->last_pos_target = 0.0f; // 注意：实际使用前最好同步一次当前电机位置
}

void Unitree_SetTrajectory(UnitreePlanner_t *planner, 
                           float start_deg_out, 
                           float end_deg_out, 
                           float speed_deg_s,
                           float kp_out, 
                           float kd_out) 
{
    if (speed_deg_s <= 0.0f) return;

    // 1. 刚度转换：除以减速比平方 (Key Point)
    // 务必确保传入的 kd_out 足够大 (例如 > 40.0)
    planner->cmd_kp_rotor = kp_out / UNITREE_GEAR_RATIO_SQ;
    planner->cmd_kd_rotor = kd_out / UNITREE_GEAR_RATIO_SQ;

    // 2. 坐标转换
    planner->start_pos_rad = DEG_TO_RAD(start_deg_out) * UNITREE_GEAR_RATIO;
    planner->end_pos_rad   = DEG_TO_RAD(end_deg_out) * UNITREE_GEAR_RATIO;
    planner->total_dist_rad = planner->end_pos_rad - planner->start_pos_rad;

    // 3. 时间计算
    float speed_rad_s = DEG_TO_RAD(speed_deg_s) * UNITREE_GEAR_RATIO;
    planner->total_duration = fabsf(planner->total_dist_rad) / speed_rad_s;

    // 4. 状态重置
    planner->current_time = 0.0f;
    planner->last_pos_target = planner->start_pos_rad;
    planner->is_moving = true;
}

void Unitree_UpdateAndSend(UnitreePlanner_t *planner, float dt) {
    float pos_cmd = 0.0f;
    float vel_cmd = 0.0f;
    float torque_ff = 8.0f;

    if (planner->is_moving) {
        planner->current_time += dt;

        if (planner->current_time >= planner->total_duration) {
            // --- 运动结束 ---
            pos_cmd = planner->end_pos_rad;
            vel_cmd = 0.0f; // 强制速度归零，通过 Kd 刹车
            
            planner->last_pos_target = pos_cmd;
            planner->is_moving = false;
        } 
        else {
            // --- 五次多项式插值 ---
            float t = planner->current_time / planner->total_duration;
            
            // 幂次预计算
            float t2 = t * t;
            float t3 = t2 * t;
            float t4 = t3 * t;
            float t5 = t4 * t;

            // 位置: 10t^3 - 15t^4 + 6t^5
            float pos_scale = (10.0f * t3) - (15.0f * t4) + (6.0f * t5);
            pos_cmd = planner->start_pos_rad + (planner->total_dist_rad * pos_scale);

            // 速度: 30t^2 - 60t^3 + 30t^4
            float vel_scale = (30.0f * t2) - (60.0f * t3) + (30.0f * t4);
            float raw_vel = (planner->total_dist_rad * vel_scale) / planner->total_duration;
            
            // 应用前馈增益：适当减小前馈速度可以减少超调
            vel_cmd = raw_vel * VEL_FF_GAIN;
            
            planner->last_pos_target = pos_cmd;
        }
    } else {
        // --- 保持模式 ---
        pos_cmd = planner->last_pos_target;
        vel_cmd = 0.0f;
    }

    // --- 发送 ---
    Unitree_Send_Cmd(
        planner->motor_id,
        torque_ff,
        vel_cmd,             
        pos_cmd,             
        planner->cmd_kp_rotor,
        planner->cmd_kd_rotor 
    );
}