#ifndef UNITREE_PLANNER_H
#define UNITREE_PLANNER_H

#include <stdint.h>
#include <stdbool.h>

// --- 宇树电机物理参数 ---
#define UNITREE_GEAR_RATIO      6.33f
#define UNITREE_GEAR_RATIO_SQ   (UNITREE_GEAR_RATIO * UNITREE_GEAR_RATIO)
#define PI                      3.1415926535f

// --- 外部提供的发送函数声明 ---
// 确保项目中有此函数的具体实现
extern void Unitree_Send_Cmd(uint32_t motorId, float torque, float speed, float position, float Kpos, float Kspd);

// --- 规划器对象结构体 ---
typedef struct {
    // 静态配置
    uint32_t motor_id;

    // 预计算的控制参数 (Rotor Frame)
    float cmd_kp_rotor;      // 已经除以 6.33^2 的 Kp
    float cmd_kd_rotor;      // 已经除以 6.33^2 的 Kd
    
    // 轨迹参数 (Rotor Frame)
    float start_pos_rad;     // 起始位置 (转子端弧度)
    float end_pos_rad;       // 目标位置 (转子端弧度)
    float speed_rad_s;       // 巡航速度 (转子端弧度/秒)
    
    // 运行时状态
    float current_time;      // 当前运动耗时
    float total_duration;    // 总耗时
    float direction;         // 1.0 或 -1.0
    bool  is_moving;         // 是否在运动中
    
    // 缓存最后的指令 (用于保持位置)
    float last_pos_target;   
} UnitreePlanner_t;

/**
 * @brief 初始化规划器
 * @param planner 指针
 * @param motor_id 电机CAN ID
 */
void Unitree_Init(UnitreePlanner_t *planner, uint32_t motor_id);

/**
 * @brief 设置新的运动轨迹（并设置刚度）
 * @param start_deg_out 输出端起始角度 (度)
 * @param end_deg_out   输出端目标角度 (度)
 * @param speed_deg_s   输出端速度 (度/秒)
 * @param kp_out        输出端期望刚度 (会自动 / 40.07)
 * @param kd_out        输出端期望阻尼 (会自动 / 40.07)
 */
void Unitree_SetTrajectory(UnitreePlanner_t *planner, 
                           float start_deg_out, 
                           float end_deg_out, 
                           float speed_deg_s,
                           float kp_out, 
                           float kd_out);

/**
 * @brief 核心更新函数
 * 放在你的定时器中断或 FreeRTOS 任务循环中调用
 * 内部会直接调用 Unitree_Send_Cmd
 * * @param dt 当前循环的时间间隔 (秒), e.g. 1kHz = 0.001f
 */
void Unitree_UpdateAndSend(UnitreePlanner_t *planner, float dt);

#endif // UNITREE_PLANNER_H