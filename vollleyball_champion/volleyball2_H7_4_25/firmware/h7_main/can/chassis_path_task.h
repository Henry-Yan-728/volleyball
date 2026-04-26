#ifndef CHASSIS_PATH_TASK
#define CHASSIS_PATH_TASK

#include "main.h"

/**
 * @brief 底盘自动规划参数集
 * @note  注释说明：参数作用 -> 调大后的现象
 * @note  为了兼容现有调用面，暂时沿用 PlannerConfig_t 名字
 */
typedef struct {
    // ================= 1. 速度包络组 (Speed Envelope) =================
    float max_spd;      // 最大平移速度 (mm/s) -> 更快，但更容易冲过头
    float start_spd;    // 起步速度 (mm/s) -> 起步更猛，太大会显得突兀
    float stop_spd;     // 末端保底速度 (mm/s) -> 收尾更干脆，太大容易到点抖动
    float up_dist;      // 加速距离 (mm) -> 起步加速更柔和
    float down_dist;    // 标称减速距离 (mm) -> 更早刹车、更稳

    // ================= 2. 转向控制组 (Steering Control) =================
    float angle_kp;     // 航向比例系数 -> 转向更积极，过大易左右摆
    float max_vr;       // 最大角速度输出 (rad/s) -> 允许转得更快
    float vr_slew_step; // 单周期角速度变化限幅 (rad/s) -> 转向响应更快，太大会更突兀
    float yaw_deadzone; // 航向软死区 (rad) -> 更能抑制小角度来回抖动

    // ================= 3. 远近场耦合组 (Far/Near Field Coupling) =================
    float far_near_dist;    // 远近场过渡距离 (mm) -> 更早进入“先赶路”模式
    float far_weight_min;   // 远场平移权重下限 [0,1] -> 被撞偏时也更愿意继续走
    float far_max_vr_scale; // 远场角速度上限比例 -> 越小越少“边跑边画龙”
    float far_vr_slew_scale;// 远场角速度变化限幅比例 -> 越大远距离姿态调整越干脆

    // ================= 4. 到点判定组 (Arrival Logic) =================
    float pos_tolerance; // 位置到点阈值 (mm) -> 更容易判到点
    float yaw_tolerance; // 航向到点阈值 (rad) -> 更容易判姿态到位
    uint8_t ignore_yaw;  // 置 1 时仅按位置判定到点 -> 忽略最终朝向
} PlannerConfig_t;

// 速度输出结构体
typedef struct {
    float vx; // 世界坐标系 x 速度
    float vy; // 世界坐标系 y 速度
    float vr; // 角速度 (rad/s 或 对应控制量)
} TrajVel_t;

/* 外部接口 */
void Planner_Init(PlannerConfig_t config);
void Planner_SetConfig(const PlannerConfig_t *config);
void Planner_GetConfig(PlannerConfig_t *config_out);
void Planner_SetTarget(float start_x, float start_y, float target_x, float target_y, float target_yaw);
TrajVel_t Planner_Update(float now_x, float now_y, float now_yaw, float now_vx, float now_vy);
uint8_t Planner_IsArrived(float now_x, float now_y, float now_yaw);

#endif
