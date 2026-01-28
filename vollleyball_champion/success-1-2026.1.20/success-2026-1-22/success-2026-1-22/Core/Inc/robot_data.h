#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H

#include "main.h"

// =============================================================
//  CAN ID 定义 (根据你之前的描述)
// =============================================================
// CAN2: 定位系统
#define CAN_ID_POSITION_DATA     0xAA   // 定位板发来的数据 (24 Bytes)

// CAN2: 上位机 (PC)
#define CAN_ID_PC_SET_TARGET     0x100  // PC发来的目标点 (8 Bytes)

// =============================================================
//  数据结构定义
// =============================================================

/**
 * @brief 机器人全场定位信息 (来自定位板 0xAA)
 */
typedef struct {
    float x;            // X轴坐标 (m)
    float y;            // Y轴坐标 (m)
    float angle;        // 当前角度 (度)
    float vx;           // X轴速度 (m/s)
    float vy;           // Y轴速度 (m/s)
    float vr;           // 角速度
    uint32_t update_tick; // 最后一次更新的时间戳 (用于判断是否掉线)
} Robot_Pose_t;

/**
 * @brief 目标指令信息 (来自上位机 0x100)
 */
typedef struct {
    float target_x;     // 目标 X 坐标
    float target_y;     // 目标 Y 坐标
    uint8_t is_updated; // 收到新数据的标志位
} Robot_Target_t;

// =============================================================
//  全局变量声明 (在其他文件里可以直接读取这些变量)
// =============================================================
extern Robot_Pose_t   g_robot_pose;   // 全局变量：当前姿态
extern Robot_Target_t g_robot_target; // 全局变量：当前目标

// =============================================================
//  函数声明
// =============================================================
void Robot_Data_Init(void); // 初始化并注册回调

#endif // ROBOT_DATA_H