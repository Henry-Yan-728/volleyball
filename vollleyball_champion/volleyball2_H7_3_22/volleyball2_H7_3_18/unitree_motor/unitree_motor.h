/**
 * @file unitree_motor.h
 * @brief 宇树电机控制接口 - 纯底层驱动封装 (USART3 RS485)
 */

#ifndef __UNITREE_MOTOR_H
#define __UNITREE_MOTOR_H

#include "main.h"
#include "usart.h"
#include "gom_protocol.h"

// USART3 用于 RS485 控制宇树电机
#define UNITREE_UART huart3

// --- 1. 底层通讯接口 ---
void UnitreeMotor_Init(void);
void UnitreeMotor_SetDirection(uint8_t tx_enable);
void UnitreeMotor_SendCmd(MotorCmd_t *cmd);
int  UnitreeMotor_ReceiveData(MotorData_t *data, uint32_t timeout_ms);

// --- 2. 基础控制接口（无状态、纯指令下发） ---
void UnitreeMotor_Control(uint8_t motor_id, uint8_t mode, float torque, float speed, float position, float kp, float kd);
void UnitreeMotor_Stop(uint8_t motor_id);
void UnitreeMotor_SetPosition(uint8_t motor_id, float position, float kp, float kd);
void UnitreeMotor_SetSpeed(uint8_t motor_id, float speed, float kd);
void UnitreeMotor_SetTorque(uint8_t motor_id, float torque);

#endif