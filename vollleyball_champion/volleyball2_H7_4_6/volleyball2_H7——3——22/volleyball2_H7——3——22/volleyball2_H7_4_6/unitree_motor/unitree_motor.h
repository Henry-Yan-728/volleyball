/**
 * @file unitree_motor.h
 * @brief 宇树电机控制接口 - 支持多路 RS485 硬件流控 (USART2, USART3)
 */

#ifndef __UNITREE_MOTOR_H
#define __UNITREE_MOTOR_H

#include "main.h"
#include "usart.h"
#include "gom_protocol.h" // 假设包含 MotorCmd_t, MotorData_t, RIS_ControlData_t 等定义



// --- 1. 底层通讯接口 ---
void UnitreeMotor_Init(void);
void UnitreeMotor_SendCmd(UART_HandleTypeDef *huart, MotorCmd_t *cmd);
int  UnitreeMotor_ReceiveData(UART_HandleTypeDef *huart, MotorData_t *data, uint32_t timeout_ms);

// --- 2. 基础控制接口（动态绑定串口，无状态纯指令下发） ---
void UnitreeMotor_Control(UART_HandleTypeDef *huart, uint8_t motor_id, uint8_t mode, float torque, float speed, float position, float kp, float kd);
void UnitreeMotor_Stop(UART_HandleTypeDef *huart, uint8_t motor_id);
void UnitreeMotor_SetPosition(UART_HandleTypeDef *huart, uint8_t motor_id, float position, float kp, float kd);
void UnitreeMotor_SetSpeed(UART_HandleTypeDef *huart, uint8_t motor_id, float speed, float kd);
void UnitreeMotor_SetTorque(UART_HandleTypeDef *huart, uint8_t motor_id, float torque);

#endif // __UNITREE_MOTOR_H
