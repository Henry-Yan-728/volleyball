/**
 * @file unitree_motor.c
 * @brief 宇树电机控制接口实现 - 纯底层驱动封装
 */

#include "unitree_motor.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

extern UART_HandleTypeDef huart3;

/**
 * @brief 设置 RS485 方向
 */
void UnitreeMotor_SetDirection(uint8_t tx_enable)
{
    if (tx_enable) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   // 发送模式
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // 接收模式
    }
}

/**
 * @brief 初始化宇树电机控制
 */
void UnitreeMotor_Init(void)
{
    printf("[Unitree] 电机控制初始化 - USART3 RS485\r\n");
    UnitreeMotor_SetDirection(1); // 默认置为接收态
}

/**
 * @brief 发送电机控制命令
 */
void UnitreeMotor_SendCmd(MotorCmd_t *cmd)
{
    modify_data(cmd);
    
    UnitreeMotor_SetDirection(1); // 切换发送
    HAL_UART_Transmit(&UNITREE_UART, (uint8_t *)&cmd->motor_send_data, sizeof(RIS_ControlData_t), 10);
//    UnitreeMotor_SetDirection(0); // 切换接收
}

/**
 * @brief 接收电机反馈数据
 */
int UnitreeMotor_ReceiveData(MotorData_t *data, uint32_t timeout_ms)
{
    uint8_t rx_buffer[sizeof(RIS_MotorData_t)];
    
//    UnitreeMotor_SetDirection(0); // 确保接收态
    
    HAL_StatusTypeDef status = HAL_UART_Receive(&UNITREE_UART, rx_buffer, sizeof(RIS_MotorData_t), timeout_ms);
    
    if (status == HAL_OK) {
        memcpy(&data->motor_recv_data, rx_buffer, sizeof(RIS_MotorData_t));
        extract_data(data);
        return data->correct;
    }
    
    return 0;
}

/**
 * @brief 核心控制下发接口
 */
void UnitreeMotor_Control(uint8_t motor_id, uint8_t mode, float torque, 
                          float speed, float position, float kp, float kd)
{
    MotorCmd_t cmd;
    cmd.id = motor_id;
    cmd.mode = mode;
    cmd.T = torque;
    cmd.W = speed;
    cmd.Pos = position;
    cmd.K_P = kp;
    cmd.K_W = kd;
    
    UnitreeMotor_SendCmd(&cmd);
}

void UnitreeMotor_Stop(uint8_t motor_id) {
    UnitreeMotor_Control(motor_id, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void UnitreeMotor_SetPosition(uint8_t motor_id, float position, float kp, float kd) {
    UnitreeMotor_Control(motor_id, 1, 0.0f, 0.0f, position, kp, kd);
}

void UnitreeMotor_SetSpeed(uint8_t motor_id, float speed, float kd) {
    UnitreeMotor_Control(motor_id, 1, 0.0f, speed, 0.0f, 0.0f, kd);
}

void UnitreeMotor_SetTorque(uint8_t motor_id, float torque) {
    UnitreeMotor_Control(motor_id, 1, torque, 0.0f, 0.0f, 0.0f, 0.0f);
}