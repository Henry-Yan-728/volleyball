/**
 * @file unitree_motor.h
 * @brief Unitree motor RS485 driver API (USART3)
 */

#ifndef __UNITREE_MOTOR_H
#define __UNITREE_MOTOR_H

#include "gom_protocol.h"
#include "main.h"
#include "usart.h"

#define UNITREE_UART huart3

#ifndef UNITREE_RS485_DE_GPIO_Port
#define UNITREE_RS485_DE_GPIO_Port GPIOB
#endif

#ifndef UNITREE_RS485_DE_Pin
#define UNITREE_RS485_DE_Pin GPIO_PIN_14
#endif

extern volatile uint32_t g_unitree_tx_ok_count;
extern volatile uint32_t g_unitree_tx_fail_count;
extern volatile uint32_t g_unitree_tx_busy_count;

void UnitreeMotor_Init(void);
void UnitreeMotor_SetDirection(uint8_t tx_enable);
void UnitreeMotor_SendCmd(MotorCmd_t *cmd);
int UnitreeMotor_ReceiveData(MotorData_t *data, uint32_t timeout_ms);

void UnitreeMotor_Control(uint8_t motor_id, uint8_t mode, float torque, float speed, float position, float kp, float kd);
void UnitreeMotor_Stop(uint8_t motor_id);
void UnitreeMotor_SetPosition(uint8_t motor_id, float position, float kp, float kd);
void UnitreeMotor_SetSpeed(uint8_t motor_id, float speed, float kd);
void UnitreeMotor_SetTorque(uint8_t motor_id, float torque);

void UnitreeMotor_SetPositionMulti(const uint8_t *motor_ids,
                                   const float *positions,
                                   const float *kps,
                                   const float *kds,
                                   uint8_t count);

#endif
