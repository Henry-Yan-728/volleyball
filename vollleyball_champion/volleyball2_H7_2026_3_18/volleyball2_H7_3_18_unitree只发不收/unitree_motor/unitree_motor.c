/**
 * @file unitree_motor.c
 * @brief Unitree motor RS485 driver implementation
 */

#include "unitree_motor.h"

#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"

#define UNITREE_UART_TIMEOUT_MS 1U

volatile uint32_t g_unitree_tx_ok_count = 0U;
volatile uint32_t g_unitree_tx_fail_count = 0U;
volatile uint32_t g_unitree_tx_busy_count = 0U;

static osMutexId_t s_unitree_bus_mutex = NULL;
static const osMutexAttr_t s_unitree_bus_mutex_attr = {
    .name = "unitreeBusMutex"
};

static void UnitreeMotor_EnsureMutex(void)
{
    if (s_unitree_bus_mutex != NULL) {
        return;
    }

    if (osKernelGetState() == osKernelInactive) {
        return;
    }

    s_unitree_bus_mutex = osMutexNew(&s_unitree_bus_mutex_attr);
}

static void UnitreeMotor_BusLock(void)
{
    UnitreeMotor_EnsureMutex();

    if (s_unitree_bus_mutex != NULL) {
        (void)osMutexAcquire(s_unitree_bus_mutex, osWaitForever);
    }
}

static void UnitreeMotor_BusUnlock(void)
{
    if (s_unitree_bus_mutex != NULL) {
        (void)osMutexRelease(s_unitree_bus_mutex);
    }
}

void UnitreeMotor_SetDirection(uint8_t tx_enable)
{
    (void)tx_enable;
    // USART3 is initialized by HAL_RS485Ex_Init, so hardware DE handles the bus direction.
}

void UnitreeMotor_Init(void)
{
    UnitreeMotor_EnsureMutex();
    printf("[Unitree] Motor driver init - USART3 RS485\r\n");
}

void UnitreeMotor_SendCmd(MotorCmd_t *cmd)
{
    HAL_StatusTypeDef tx_status;

    if (cmd == NULL) {
        return;
    }

    modify_data(cmd);

    UnitreeMotor_BusLock();

    if (HAL_UART_GetState(&UNITREE_UART) != HAL_UART_STATE_READY) {
        g_unitree_tx_busy_count++;
        UnitreeMotor_BusUnlock();
        return;
    }

    tx_status = HAL_UART_Transmit(&UNITREE_UART,
                                  (uint8_t *)&cmd->motor_send_data,
                                  sizeof(RIS_ControlData_t),
                                  UNITREE_UART_TIMEOUT_MS);
    if (tx_status == HAL_OK) {
        g_unitree_tx_ok_count++;
    } else {
        g_unitree_tx_fail_count++;
    }
    UnitreeMotor_BusUnlock();
}

int UnitreeMotor_ReceiveData(MotorData_t *data, uint32_t timeout_ms)
{
    uint8_t rx_buffer[sizeof(RIS_MotorData_t)];
    HAL_StatusTypeDef status;

    if (data == NULL) {
        return 0;
    }

    UnitreeMotor_BusLock();
    status = HAL_UART_Receive(&UNITREE_UART, rx_buffer, sizeof(RIS_MotorData_t), timeout_ms);
    UnitreeMotor_BusUnlock();

    if (status == HAL_OK) {
        memcpy(&data->motor_recv_data, rx_buffer, sizeof(RIS_MotorData_t));
        extract_data(data);
        return data->correct;
    }

    return 0;
}

void UnitreeMotor_Control(uint8_t motor_id, uint8_t mode, float torque,
                          float speed, float position, float kp, float kd)
{
    MotorCmd_t cmd = {0};

    cmd.id = motor_id;
    cmd.mode = mode;
    cmd.T = torque;
    cmd.W = speed;
    cmd.Pos = position;
    cmd.K_P = kp;
    cmd.K_W = kd;

    UnitreeMotor_SendCmd(&cmd);
}

void UnitreeMotor_Stop(uint8_t motor_id)
{
    UnitreeMotor_Control(motor_id, 0U, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void UnitreeMotor_SetPosition(uint8_t motor_id, float position, float kp, float kd)
{
    UnitreeMotor_Control(motor_id, 1U, 0.0f, 0.0f, position, kp, kd);
}

void UnitreeMotor_SetSpeed(uint8_t motor_id, float speed, float kd)
{
    UnitreeMotor_Control(motor_id, 1U, 0.0f, speed, 0.0f, 0.0f, kd);
}

void UnitreeMotor_SetTorque(uint8_t motor_id, float torque)
{
    UnitreeMotor_Control(motor_id, 1U, torque, 0.0f, 0.0f, 0.0f, 0.0f);
}

void UnitreeMotor_SetPositionMulti(const uint8_t *motor_ids,
                                   const float *positions,
                                   const float *kps,
                                   const float *kds,
                                   uint8_t count)
{
    if ((motor_ids == NULL) || (positions == NULL) || (kps == NULL) || (kds == NULL) || (count == 0U)) {
        return;
    }

    UnitreeMotor_BusLock();

    for (uint8_t i = 0; i < count; i++) {
        MotorCmd_t cmd = {0};

        cmd.id = motor_ids[i];
        cmd.mode = 1U;
        cmd.T = 0.0f;
        cmd.W = 0.0f;
        cmd.Pos = positions[i];
        cmd.K_P = kps[i];
        cmd.K_W = kds[i];

        modify_data(&cmd);

        if (HAL_UART_Transmit(&UNITREE_UART,
                              (uint8_t *)&cmd.motor_send_data,
                              sizeof(RIS_ControlData_t),
                              UNITREE_UART_TIMEOUT_MS) != HAL_OK) {
            break;
        }
    }

    UnitreeMotor_BusUnlock();
}
