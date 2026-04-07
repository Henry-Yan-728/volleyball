#include "unitree_motor.h"

#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *UnitreeMotor_GetHandle(UnitreeBusId_t bus_id)
{
    switch (bus_id)
    {
    case UNITREE_BUS_USART2:
        return &huart2;
    case UNITREE_BUS_USART3:
        return &huart3;
    default:
        return NULL;
    }
}

static const char *UnitreeMotor_GetBusName(UnitreeBusId_t bus_id)
{
    switch (bus_id)
    {
    case UNITREE_BUS_USART2:
        return "USART2";
    case UNITREE_BUS_USART3:
        return "USART3";
    default:
        return "UNKNOWN";
    }
}

void UnitreeMotor_Init(void)
{
    printf("[Unitree] RS485 buses ready: %s + %s\r\n",
           UnitreeMotor_GetBusName(UNITREE_BUS_USART2),
           UnitreeMotor_GetBusName(UNITREE_BUS_USART3));
}

HAL_StatusTypeDef UnitreeMotor_SendCmdOnBus(UnitreeBusId_t bus_id, MotorCmd_t *cmd)
{
    UART_HandleTypeDef *uart = UnitreeMotor_GetHandle(bus_id);

    if ((uart == NULL) || (cmd == NULL))
    {
        return HAL_ERROR;
    }

    modify_data(cmd);
    return HAL_UART_Transmit(uart,
                             (uint8_t *)&cmd->motor_send_data,
                             sizeof(RIS_ControlData_t),
                             10);
}

int UnitreeMotor_ReceiveDataOnBus(UnitreeBusId_t bus_id, MotorData_t *data, uint32_t timeout_ms)
{
    UART_HandleTypeDef *uart = UnitreeMotor_GetHandle(bus_id);
    uint8_t rx_buffer[sizeof(RIS_MotorData_t)];

    if ((uart == NULL) || (data == NULL))
    {
        return 0;
    }

    if (HAL_UART_Receive(uart, rx_buffer, sizeof(RIS_MotorData_t), timeout_ms) != HAL_OK)
    {
        return 0;
    }

    memcpy(&data->motor_recv_data, rx_buffer, sizeof(RIS_MotorData_t));
    extract_data(data);
    return data->correct;
}

void UnitreeMotor_SendCmd(MotorCmd_t *cmd)
{
    (void)UnitreeMotor_SendCmdOnBus(UNITREE_BUS_USART3, cmd);
}

int UnitreeMotor_ReceiveData(MotorData_t *data, uint32_t timeout_ms)
{
    return UnitreeMotor_ReceiveDataOnBus(UNITREE_BUS_USART3, data, timeout_ms);
}

void UnitreeMotor_ControlOnBus(UnitreeBusId_t bus_id,
                               uint8_t motor_id,
                               uint8_t mode,
                               float torque,
                               float speed,
                               float position,
                               float kp,
                               float kd)
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

    (void)UnitreeMotor_SendCmdOnBus(bus_id, &cmd);
}

void UnitreeMotor_StopOnBus(UnitreeBusId_t bus_id, uint8_t motor_id)
{
    UnitreeMotor_ControlOnBus(bus_id, motor_id, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void UnitreeMotor_SetPositionOnBus(UnitreeBusId_t bus_id, uint8_t motor_id, float position, float kp, float kd)
{
    UnitreeMotor_ControlOnBus(bus_id, motor_id, 1, 0.0f, 0.0f, position, kp, kd);
}

void UnitreeMotor_SetSpeedOnBus(UnitreeBusId_t bus_id, uint8_t motor_id, float speed, float kd)
{
    UnitreeMotor_ControlOnBus(bus_id, motor_id, 1, 0.0f, speed, 0.0f, 0.0f, kd);
}

void UnitreeMotor_SetTorqueOnBus(UnitreeBusId_t bus_id, uint8_t motor_id, float torque)
{
    UnitreeMotor_ControlOnBus(bus_id, motor_id, 1, torque, 0.0f, 0.0f, 0.0f, 0.0f);
}

void UnitreeMotor_Control(uint8_t motor_id, uint8_t mode, float torque, float speed, float position, float kp, float kd)
{
    UnitreeMotor_ControlOnBus(UNITREE_BUS_USART3, motor_id, mode, torque, speed, position, kp, kd);
}

void UnitreeMotor_Stop(uint8_t motor_id)
{
    UnitreeMotor_StopOnBus(UNITREE_BUS_USART3, motor_id);
}

void UnitreeMotor_SetPosition(uint8_t motor_id, float position, float kp, float kd)
{
    UnitreeMotor_SetPositionOnBus(UNITREE_BUS_USART3, motor_id, position, kp, kd);
}

void UnitreeMotor_SetSpeed(uint8_t motor_id, float speed, float kd)
{
    UnitreeMotor_SetSpeedOnBus(UNITREE_BUS_USART3, motor_id, speed, kd);
}

void UnitreeMotor_SetTorque(uint8_t motor_id, float torque)
{
    UnitreeMotor_SetTorqueOnBus(UNITREE_BUS_USART3, motor_id, torque);
}

void Unitree_Send_CmdOnBus(UnitreeBusId_t bus_id, uint32_t motor_id, float torque, float speed, float position, float kp, float kd)
{
    UnitreeMotor_ControlOnBus(bus_id, (uint8_t)motor_id, 1, torque, speed, position, kp, kd);
}

void Unitree_Send_Cmd(uint32_t motor_id, float torque, float speed, float position, float kp, float kd)
{
    Unitree_Send_CmdOnBus(UNITREE_BUS_USART3, motor_id, torque, speed, position, kp, kd);
}
