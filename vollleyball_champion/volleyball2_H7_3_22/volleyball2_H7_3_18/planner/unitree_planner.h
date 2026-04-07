#ifndef UNITREE_PLANNER_H
#define UNITREE_PLANNER_H

#include <stdint.h>
#include <stdbool.h>

// --- 宇树电机物理参数 ---
#define UNITREE_GEAR_RATIO      6.33f
#define UNITREE_GEAR_RATIO_SQ   (UNITREE_GEAR_RATIO * UNITREE_GEAR_RATIO)
#define PI                      3.1415926535f

// --- 外部发送函数 ---
extern void Unitree_Send_Cmd(uint32_t motorId, float torque, float speed, float position, float Kpos, float Kspd);

// --- 规划器对象结构体 ---
typedef struct {
    // 静态配置
    uint32_t motor_id;

    // 预计算的控制参数 (Rotor Frame)
    float cmd_kp_rotor;      
    float cmd_kd_rotor;      
    
    // 轨迹参数 (Rotor Frame)
    float start_pos_rad;     // 起始位置
    float end_pos_rad;       // 目标位置
    float total_dist_rad;    // 总距离 (end - start)，可为负
    
    // 运行时状态
    float current_time;      // 当前耗时
    float total_duration;    // 总耗时
    bool  is_moving;         // 是否运动中
    
    // 缓存最后的指令 (用于保持位置)
    float last_pos_target;   
} UnitreePlanner_t;

/**
 * @brief 初始化规划器
 */
void Unitree_Init(UnitreePlanner_t *planner, uint32_t motor_id);

/**
 * @brief 设置轨迹 (使用五次多项式平滑规划)
 * 注意: speed_deg_s 此时代表“平均速度”。峰值速度约为平均速度的 1.8 倍。
 */
void Unitree_SetTrajectory(UnitreePlanner_t *planner, 
                           float start_deg_out, 
                           float end_deg_out, 
                           float speed_deg_s,
                           float kp_out, 
                           float kd_out);

/**
 * @brief 核心更新函数 (建议 1kHz 调用)
 */
void Unitree_UpdateAndSend(UnitreePlanner_t *planner, float dt);

#endif // UNITREE_PLANNER_H