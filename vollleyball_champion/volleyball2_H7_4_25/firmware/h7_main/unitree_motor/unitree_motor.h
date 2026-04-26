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
typedef enum {
    UNITREE_MOTOR_LINK_OK = 0,
    UNITREE_MOTOR_LINK_INVALID_ARG = -1,
    UNITREE_MOTOR_LINK_TIMEOUT = -2,
    UNITREE_MOTOR_LINK_BAD_HEADER = -3,
    UNITREE_MOTOR_LINK_BAD_CRC = -4,
    UNITREE_MOTOR_LINK_HAL_ERROR = -5,
    UNITREE_MOTOR_LINK_ID_MISMATCH = -6,
    UNITREE_MOTOR_LINK_MUTEX_TIMEOUT = -7
} UnitreeMotorLinkResult_t;

typedef struct {
    uint32_t tx_count;
    uint32_t rx_ok_count;
    uint32_t mutex_timeout_count;
    uint32_t timeout_count;
    uint32_t bad_header_count;
    uint32_t bad_crc_count;
    uint32_t hal_error_count;
    uint32_t id_mismatch_count;
    uint32_t last_tx_tick;
    uint32_t last_rx_ok_tick;
    uint8_t last_tx_motor_id;
    uint8_t last_rx_motor_id;
    uint8_t last_motor_error;
    int32_t last_result;
} UnitreeMotorLinkState_t;

void UnitreeMotor_Init(void);
UnitreeMotorLinkResult_t UnitreeMotor_SendCmd(UART_HandleTypeDef *huart, MotorCmd_t *cmd);
int  UnitreeMotor_ReceiveData(UART_HandleTypeDef *huart, MotorData_t *data, uint32_t timeout_ms);
UnitreeMotorLinkResult_t UnitreeMotor_Probe(UART_HandleTypeDef *huart, uint8_t motor_id,
                                            uint32_t timeout_ms, MotorData_t *data);
const UnitreeMotorLinkState_t *UnitreeMotor_GetLinkState(UART_HandleTypeDef *huart);
void UnitreeMotor_ResetLinkState(UART_HandleTypeDef *huart);

// --- 2. 基础控制接口（动态绑定串口，无状态纯指令下发） ---
UnitreeMotorLinkResult_t UnitreeMotor_Control(UART_HandleTypeDef *huart, uint8_t motor_id, uint8_t mode,
                                              float torque, float speed, float position, float kp, float kd);
UnitreeMotorLinkResult_t UnitreeMotor_Stop(UART_HandleTypeDef *huart, uint8_t motor_id);
UnitreeMotorLinkResult_t UnitreeMotor_StopNoWait(UART_HandleTypeDef *huart, uint8_t motor_id);
UnitreeMotorLinkResult_t UnitreeMotor_SetPosition(UART_HandleTypeDef *huart, uint8_t motor_id,
                                                  float position, float kp, float kd);
UnitreeMotorLinkResult_t UnitreeMotor_SetSpeed(UART_HandleTypeDef *huart, uint8_t motor_id, float speed,
                                               float kd);
UnitreeMotorLinkResult_t UnitreeMotor_SetTorque(UART_HandleTypeDef *huart, uint8_t motor_id, float torque);

#endif // __UNITREE_MOTOR_H
