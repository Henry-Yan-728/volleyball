#ifndef UNITREE_DRIVER_H
#define UNITREE_DRIVER_H

#include "main.h"

// 宇树电机参数限制
#define UNITREE_P_MIN -12.5f
#define UNITREE_P_MAX 12.5f
#define UNITREE_V_MIN -30.0f
#define UNITREE_V_MAX 30.0f
#define UNITREE_T_MIN -10.0f
#define UNITREE_T_MAX 10.0f

#define UNITREE_KP_MIN 0.0f
#define UNITREE_KP_MAX 500.0f
#define UNITREE_KD_MIN 0.0f
#define UNITREE_KD_MAX 5.0f

// --- 函数声明 ---

/**
 * @brief 发送宇树电机控制指令 (混合控制模式 Mode 10)
 * @note 此函数会自动应用“零点偏移”，用户只需传入逻辑坐标
 * @param motorId: 电机ID (0-3)
 * @param torque: 前馈力矩 (Nm)
 * @param speed: 目标角速度 (rad/s)
 * @param position: 目标角度 (rad, 逻辑坐标)
 * @param Kpos: 位置刚度
 * @param Kspd: 速度刚度
 */
void Unitree_Send_Cmd(uint32_t motorId, float torque, float speed, float position, float Kpos, float Kspd);

/**
 * @brief 发送CAN命令设置电机的位置和速度增益 (辅助设置)
 */
HAL_StatusTypeDef sendCANSetMotorKK(uint32_t moduleId, uint32_t motorId, float Kpos, float Kspd);

void get_Unitree_pos(uint32_t motorId);

//void Unitree_SetCurrentPosAsZero(uint32_t motorId);
#endif