#include "unitree_planner.h"
#include "unitree_driver.h"
#include <math.h>
#include <string.h>

// 辅助宏：角度转弧度
#define DEG_TO_RAD(x) ((x) * (PI / 180.0f))

void Unitree_Init(UnitreePlanner_t *planner, uint32_t motor_id) {
    memset(planner, 0, sizeof(UnitreePlanner_t));
    planner->motor_id = motor_id;
    planner->is_moving = false;
    
    // 默认保持 0 位置，刚度为 0 (安全起见)
    planner->last_pos_target = 0.0f;
    planner->cmd_kp_rotor = 0.0f;
    planner->cmd_kd_rotor = 0.0f;
}

void Unitree_SetTrajectory(UnitreePlanner_t *planner, 
                           float start_deg_out, 
                           float end_deg_out, 
                           float speed_deg_s,
                           float kp_out, 
                           float kd_out) 
{
    if (speed_deg_s <= 0.0f) return;

    // 1. 参数预处理：将输出端参数转换为转子端参数
    // 刚度/阻尼除以减速比平方 (Key Point)
    planner->cmd_kp_rotor = kp_out / UNITREE_GEAR_RATIO_SQ;
    planner->cmd_kd_rotor = kd_out / UNITREE_GEAR_RATIO_SQ;

    // 位置转换为转子弧度：输出角度 * (PI/180) * 6.33
    planner->start_pos_rad = DEG_TO_RAD(start_deg_out) * UNITREE_GEAR_RATIO;
    planner->end_pos_rad   = DEG_TO_RAD(end_deg_out) * UNITREE_GEAR_RATIO;
    
    // 速度转换为转子弧度/秒
    planner->speed_rad_s   = DEG_TO_RAD(speed_deg_s) * UNITREE_GEAR_RATIO;

    // 2. 计算时间参数
    float distance = planner->end_pos_rad - planner->start_pos_rad;
    planner->direction = (distance >= 0.0f) ? 1.0f : -1.0f;
    planner->total_duration = fabsf(distance) / planner->speed_rad_s;

    // 3. 重置状态
    planner->current_time = 0.0f;
    planner->last_pos_target = planner->start_pos_rad;
    planner->is_moving = true;
}

void Unitree_UpdateAndSend(UnitreePlanner_t *planner, float dt) {
    float pos_cmd = 0.0f;
    float vel_cmd = 0.0f;
    float torque_ff = 0.0f; // 位置模式通常无力矩前馈

    if (planner->is_moving) {
        // --- 运动插值 ---
        planner->current_time += dt;

        if (planner->current_time >= planner->total_duration) {
            // 到达终点
            pos_cmd = planner->end_pos_rad;
            vel_cmd = 0.0f;
            
            planner->last_pos_target = pos_cmd; // 更新保持位
            planner->is_moving = false;         // 标记结束
        } else {
            // 梯形/线性过程
            // S = S0 + V * t
            float current_dist = planner->speed_rad_s * planner->current_time;
            pos_cmd = planner->start_pos_rad + (current_dist * planner->direction);
            
            // 发送前馈速度 (V_ref)，这对宇树电机的跟踪性能至关重要
            vel_cmd = planner->speed_rad_s * planner->direction;
            
            planner->last_pos_target = pos_cmd;
        }
    } else {
        // --- 保持模式 (Servo Lock) ---
        // 即使不动，也必须持续发送指令以维持刚度
        pos_cmd = planner->last_pos_target;
        vel_cmd = 0.0f;
    }

    // --- 直接发送 CAN 指令 ---
    // 注意：这里的 pos, vel, kp, kd 均已在 SetTrajectory 和 上文中转换为 Rotor Frame
    Unitree_Send_Cmd(
        planner->motor_id,   // ID
        torque_ff,           // T_ff (0)
        vel_cmd,             // Omega (Rotor rad/s)
        pos_cmd,             // Pos (Rotor rad)
        planner->cmd_kp_rotor, // Kp (Corrected)
        planner->cmd_kd_rotor  // Kd (Corrected)
    );
}