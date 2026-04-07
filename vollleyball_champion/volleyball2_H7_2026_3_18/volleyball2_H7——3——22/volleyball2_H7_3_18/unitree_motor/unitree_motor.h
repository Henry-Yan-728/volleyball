/**
 * @file unitree_motor.h
 * @brief Unitree motor RS485 driver helpers for USART2/USART3.
 */

#ifndef __UNITREE_MOTOR_H
#define __UNITREE_MOTOR_H

#include "main.h"
#include "usart.h"
#include "gom_protocol.h"

typedef enum
{
    UNITREE_BUS_USART2 = 0,
    UNITREE_BUS_USART3 = 1,
    UNITREE_BUS_COUNT
} UnitreeBusId_t;

void UnitreeMotor_Init(void);

HAL_StatusTypeDef UnitreeMotor_SendCmdOnBus(UnitreeBusId_t bus_id, MotorCmd_t *cmd);
int UnitreeMotor_ReceiveDataOnBus(UnitreeBusId_t bus_id, MotorData_t *data, uint32_t timeout_ms);

void UnitreeMotor_SendCmd(MotorCmd_t *cmd);
int UnitreeMotor_ReceiveData(MotorData_t *data, uint32_t timeout_ms);

void UnitreeMotor_ControlOnBus(UnitreeBusId_t bus_id, uint8_t motor_id, uint8_t mode, float torque, float speed, float position, float kp, float kd);
void UnitreeMotor_StopOnBus(UnitreeBusId_t bus_id, uint8_t motor_id);
void UnitreeMotor_SetPositionOnBus(UnitreeBusId_t bus_id, uint8_t motor_id, float position, float kp, float kd);
void UnitreeMotor_SetSpeedOnBus(UnitreeBusId_t bus_id, uint8_t motor_id, float speed, float kd);
void UnitreeMotor_SetTorqueOnBus(UnitreeBusId_t bus_id, uint8_t motor_id, float torque);

void UnitreeMotor_Control(uint8_t motor_id, uint8_t mode, float torque, float speed, float position, float kp, float kd);
void UnitreeMotor_Stop(uint8_t motor_id);
void UnitreeMotor_SetPosition(uint8_t motor_id, float position, float kp, float kd);
void UnitreeMotor_SetSpeed(uint8_t motor_id, float speed, float kd);
void UnitreeMotor_SetTorque(uint8_t motor_id, float torque);

void Unitree_Send_CmdOnBus(UnitreeBusId_t bus_id, uint32_t motor_id, float torque, float speed, float position, float kp, float kd);
void Unitree_Send_Cmd(uint32_t motor_id, float torque, float speed, float position, float kp, float kd);

#endif
