/**
 * @file unitree_motor.c
 * @brief Unitree motor transport layer over RS485 UART
 */

#include "unitree_motor.h"

#include <stdio.h>
#include <string.h>

void UnitreeMotor_Init(void)
{
    printf("[Unitree] motor control ready on USART2/USART3 RS485\r\n");
}

void UnitreeMotor_SendCmd(UART_HandleTypeDef *huart, MotorCmd_t *cmd)
{
    if (huart == NULL || cmd == NULL) {
        return;
    }

    modify_data(cmd);
    HAL_UART_Transmit(huart, (uint8_t *)&cmd->motor_send_data, sizeof(RIS_ControlData_t), 10);
}

int UnitreeMotor_ReceiveData(UART_HandleTypeDef *huart, MotorData_t *data, uint32_t timeout_ms)
{
    uint8_t rx_buffer[sizeof(RIS_MotorData_t)];
    HAL_StatusTypeDef status;

    if (huart == NULL || data == NULL) {
        return 0;
    }

    status = HAL_UART_Receive(huart, rx_buffer, sizeof(RIS_MotorData_t), timeout_ms);
    if (status != HAL_OK) {
        return 0;
    }

    memcpy(&data->motor_recv_data, rx_buffer, sizeof(RIS_MotorData_t));
    extract_data(data);
    return data->correct;
}

void UnitreeMotor_Control(UART_HandleTypeDef *huart, uint8_t motor_id, uint8_t mode, float torque,
                          float speed, float position, float kp, float kd)
{
    MotorCmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.id = motor_id;
    cmd.mode = mode;
    cmd.T = torque;
    cmd.W = speed;
    cmd.Pos = position;
    cmd.K_P = kp;
    cmd.K_W = kd;

    UnitreeMotor_SendCmd(huart, &cmd);
}

void UnitreeMotor_Stop(UART_HandleTypeDef *huart, uint8_t motor_id)
{
    UnitreeMotor_Control(huart, motor_id, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void UnitreeMotor_SetPosition(UART_HandleTypeDef *huart, uint8_t motor_id, float position, float kp, float kd)
{
    UnitreeMotor_Control(huart, motor_id, 1, 0.0f, 0.0f, position, kp, kd);
}

void UnitreeMotor_SetSpeed(UART_HandleTypeDef *huart, uint8_t motor_id, float speed, float kd)
{
    UnitreeMotor_Control(huart, motor_id, 1, 0.0f, speed, 0.0f, 0.0f, kd);
}

void UnitreeMotor_SetTorque(UART_HandleTypeDef *huart, uint8_t motor_id, float torque)
{
    UnitreeMotor_Control(huart, motor_id, 1, torque, 0.0f, 0.0f, 0.0f, 0.0f);
}
