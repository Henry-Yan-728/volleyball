#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "main.h"

// =============================================================
//  舵轮 CAN ID 定义 (CAN1)
//  请确保你的 4 个舵轮驱动板分别刷入了对应的接收 ID
// =============================================================

#define CHASSIS_RADIUS  0.25f // 底盘半径，单位米

extern float vx , vy , vr ;


// =============================================================
//  函数声明
// =============================================================

// 初始化底盘任务
void Chassis_Init(void);

// 更新底盘运动 (周期性调用，建议 1ms 或 2ms 一次)
// vx, vy: m/s,  vr: rad/s
void Chassis_Update(float vx, float vy, float vr);

// 紧急停止 (所有轮子归零)
void Chassis_Stop(void);

// 新增：底盘周期任务，需在 main.c 中定时调用
void Chassis_Task_Loop(void);

#endif // CHASSIS_TASK_H