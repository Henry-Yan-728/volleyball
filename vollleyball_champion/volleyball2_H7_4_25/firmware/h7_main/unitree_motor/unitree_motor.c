/**
 * @file unitree_motor.c
 * @brief Unitree motor transport layer over RS485 UART
 */

#include "unitree_motor.h"

#include "cmsis_os2.h"

#include <stdio.h>
#include <string.h>

#define UNITREE_MOTOR_MUTEX_TIMEOUT_MS 10U

static UnitreeMotorLinkState_t s_unitree_link_usart2;
static UnitreeMotorLinkState_t s_unitree_link_usart3;
static osMutexId_t s_unitree_mutex_usart2;
static osMutexId_t s_unitree_mutex_usart3;
static const osMutexAttr_t s_unitree_mutex_usart2_attr = {
    .name = "unitreeU2"
};
static const osMutexAttr_t s_unitree_mutex_usart3_attr = {
    .name = "unitreeU3"
};

static UnitreeMotorLinkResult_t unitree_lock_uart_with_timeout(UART_HandleTypeDef *huart,
                                                               uint32_t timeout_ms);
static HAL_StatusTypeDef unitree_send_cmd_locked_with_timeout(UART_HandleTypeDef *huart, MotorCmd_t *cmd,
                                                              uint32_t timeout_ms);

static UnitreeMotorLinkState_t *unitree_get_link_state_mutable(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        return &s_unitree_link_usart2;
    }
    if (huart == &huart3) {
        return &s_unitree_link_usart3;
    }
    return NULL;
}

static osMutexId_t unitree_get_uart_mutex(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        return s_unitree_mutex_usart2;
    }
    if (huart == &huart3) {
        return s_unitree_mutex_usart3;
    }
    return NULL;
}

static UnitreeMotorLinkResult_t unitree_lock_uart(UART_HandleTypeDef *huart)
{
    return unitree_lock_uart_with_timeout(huart, UNITREE_MOTOR_MUTEX_TIMEOUT_MS);
}

static UnitreeMotorLinkResult_t unitree_lock_uart_with_timeout(UART_HandleTypeDef *huart, uint32_t timeout_ms)
{
    osMutexId_t mutex = unitree_get_uart_mutex(huart);
    UnitreeMotorLinkState_t *link_state = unitree_get_link_state_mutable(huart);

    if (mutex == NULL) {
        return UNITREE_MOTOR_LINK_OK;
    }

    if (osMutexAcquire(mutex, timeout_ms) == osOK) {
        return UNITREE_MOTOR_LINK_OK;
    }

    if (link_state != NULL) {
        link_state->mutex_timeout_count++;
        link_state->last_result = UNITREE_MOTOR_LINK_MUTEX_TIMEOUT;
    }
    return UNITREE_MOTOR_LINK_MUTEX_TIMEOUT;
}

static void unitree_unlock_uart(UART_HandleTypeDef *huart)
{
    osMutexId_t mutex = unitree_get_uart_mutex(huart);

    if (mutex != NULL) {
        (void)osMutexRelease(mutex);
    }
}

static HAL_StatusTypeDef unitree_send_cmd_locked(UART_HandleTypeDef *huart, MotorCmd_t *cmd)
{
    return unitree_send_cmd_locked_with_timeout(huart, cmd, 10);
}

static HAL_StatusTypeDef unitree_send_cmd_locked_with_timeout(UART_HandleTypeDef *huart, MotorCmd_t *cmd,
                                                              uint32_t timeout_ms)
{
    UnitreeMotorLinkState_t *link_state;

    if (huart == NULL || cmd == NULL) {
        return HAL_ERROR;
    }

    modify_data(cmd);
    link_state = unitree_get_link_state_mutable(huart);
    if (link_state != NULL) {
        link_state->tx_count++;
        link_state->last_tx_tick = HAL_GetTick();
        link_state->last_tx_motor_id = (uint8_t)cmd->id;
    }

    return HAL_UART_Transmit(huart, (uint8_t *)&cmd->motor_send_data, sizeof(RIS_ControlData_t), timeout_ms);
}

static UnitreeMotorLinkResult_t unitree_receive_frame_locked(UART_HandleTypeDef *huart, MotorData_t *data,
                                                             uint32_t timeout_ms)
{
    uint8_t rx_buffer[sizeof(RIS_MotorData_t)];
    HAL_StatusTypeDef status;
    UnitreeMotorLinkState_t *link_state;

    if (huart == NULL || data == NULL) {
        return UNITREE_MOTOR_LINK_INVALID_ARG;
    }

    memset(data, 0, sizeof(*data));
    link_state = unitree_get_link_state_mutable(huart);

    status = HAL_UART_Receive(huart, rx_buffer, sizeof(RIS_MotorData_t), timeout_ms);
    if (status != HAL_OK) {
        if (status == HAL_TIMEOUT) {
            data->timeout = 1U;
            if (link_state != NULL) {
                link_state->timeout_count++;
                link_state->last_result = UNITREE_MOTOR_LINK_TIMEOUT;
            }
            return UNITREE_MOTOR_LINK_TIMEOUT;
        }

        if (link_state != NULL) {
            link_state->hal_error_count++;
            link_state->last_result = UNITREE_MOTOR_LINK_HAL_ERROR;
        }
        return UNITREE_MOTOR_LINK_HAL_ERROR;
    }

    memcpy(&data->motor_recv_data, rx_buffer, sizeof(RIS_MotorData_t));
    extract_data(data);
    if (data->correct == 0) {
        if (data->bad_msg != 0U) {
            if (link_state != NULL) {
                link_state->bad_crc_count++;
                link_state->last_result = UNITREE_MOTOR_LINK_BAD_CRC;
            }
            return UNITREE_MOTOR_LINK_BAD_CRC;
        }

        if (link_state != NULL) {
            link_state->bad_header_count++;
            link_state->last_result = UNITREE_MOTOR_LINK_BAD_HEADER;
        }
        return UNITREE_MOTOR_LINK_BAD_HEADER;
    }

    if (link_state != NULL) {
        link_state->rx_ok_count++;
        link_state->last_rx_ok_tick = HAL_GetTick();
        link_state->last_rx_motor_id = data->motor_id;
        link_state->last_motor_error = (uint8_t)data->MError;
        link_state->last_result = UNITREE_MOTOR_LINK_OK;
    }

    return UNITREE_MOTOR_LINK_OK;
}

void UnitreeMotor_Init(void)
{
    memset(&s_unitree_link_usart2, 0, sizeof(s_unitree_link_usart2));
    memset(&s_unitree_link_usart3, 0, sizeof(s_unitree_link_usart3));

    if (s_unitree_mutex_usart2 == NULL) {
        s_unitree_mutex_usart2 = osMutexNew(&s_unitree_mutex_usart2_attr);
    }
    if (s_unitree_mutex_usart3 == NULL) {
        s_unitree_mutex_usart3 = osMutexNew(&s_unitree_mutex_usart3_attr);
    }
    if ((s_unitree_mutex_usart2 == NULL) || (s_unitree_mutex_usart3 == NULL)) {
        printf("[Unitree] RS485 mutex init failed\r\n");
        Error_Handler();
    }

    printf("[Unitree] motor control ready on USART2/USART3 RS485\r\n");
}

UnitreeMotorLinkResult_t UnitreeMotor_SendCmd(UART_HandleTypeDef *huart, MotorCmd_t *cmd)
{
    UnitreeMotorLinkResult_t lock_result;
    HAL_StatusTypeDef tx_status;
    UnitreeMotorLinkState_t *link_state;

    if (huart == NULL || cmd == NULL) {
        return UNITREE_MOTOR_LINK_INVALID_ARG;
    }

    link_state = unitree_get_link_state_mutable(huart);
    lock_result = unitree_lock_uart(huart);
    if (lock_result != UNITREE_MOTOR_LINK_OK) {
        return lock_result;
    }

    tx_status = unitree_send_cmd_locked(huart, cmd);
    if ((tx_status != HAL_OK) && (link_state != NULL)) {
        link_state->hal_error_count++;
        link_state->last_result = UNITREE_MOTOR_LINK_HAL_ERROR;
    }

    unitree_unlock_uart(huart);
    if (tx_status != HAL_OK) {
        return UNITREE_MOTOR_LINK_HAL_ERROR;
    }

    if (link_state != NULL) {
        link_state->last_result = UNITREE_MOTOR_LINK_OK;
    }
    return UNITREE_MOTOR_LINK_OK;
}

static UnitreeMotorLinkResult_t unitree_send_cmd_no_wait(UART_HandleTypeDef *huart, MotorCmd_t *cmd)
{
    UnitreeMotorLinkResult_t lock_result;
    HAL_StatusTypeDef tx_status;
    UnitreeMotorLinkState_t *link_state;

    if (huart == NULL || cmd == NULL) {
        return UNITREE_MOTOR_LINK_INVALID_ARG;
    }

    link_state = unitree_get_link_state_mutable(huart);
    lock_result = unitree_lock_uart_with_timeout(huart, 0U);
    if (lock_result != UNITREE_MOTOR_LINK_OK) {
        return lock_result;
    }

    tx_status = unitree_send_cmd_locked_with_timeout(huart, cmd, 0U);
    if ((tx_status != HAL_OK) && (link_state != NULL)) {
        link_state->hal_error_count++;
        link_state->last_result = UNITREE_MOTOR_LINK_HAL_ERROR;
    }

    unitree_unlock_uart(huart);
    if (tx_status != HAL_OK) {
        return UNITREE_MOTOR_LINK_HAL_ERROR;
    }

    if (link_state != NULL) {
        link_state->last_result = UNITREE_MOTOR_LINK_OK;
    }
    return UNITREE_MOTOR_LINK_OK;
}

int UnitreeMotor_ReceiveData(UART_HandleTypeDef *huart, MotorData_t *data, uint32_t timeout_ms)
{
    UnitreeMotorLinkResult_t lock_result;
    UnitreeMotorLinkResult_t rx_result;

    lock_result = unitree_lock_uart(huart);
    if (lock_result != UNITREE_MOTOR_LINK_OK) {
        return 0;
    }

    rx_result = unitree_receive_frame_locked(huart, data, timeout_ms);
    unitree_unlock_uart(huart);

    return (rx_result == UNITREE_MOTOR_LINK_OK) ? 1 : 0;
}

UnitreeMotorLinkResult_t UnitreeMotor_Probe(UART_HandleTypeDef *huart, uint8_t motor_id,
                                            uint32_t timeout_ms, MotorData_t *data)
{
    MotorData_t local_data;
    MotorCmd_t cmd;
    MotorData_t *rx_data;
    HAL_StatusTypeDef tx_status;
    UnitreeMotorLinkResult_t result;
    UnitreeMotorLinkState_t *link_state;

    if (huart == NULL) {
        return UNITREE_MOTOR_LINK_INVALID_ARG;
    }

    rx_data = (data != NULL) ? data : &local_data;
    result = unitree_lock_uart(huart);
    if (result != UNITREE_MOTOR_LINK_OK) {
        return result;
    }

    memset(&cmd, 0, sizeof(cmd));
    cmd.id = motor_id;
    cmd.mode = 0U;
    tx_status = unitree_send_cmd_locked(huart, &cmd);
    if (tx_status != HAL_OK) {
        link_state = unitree_get_link_state_mutable(huart);
        if (link_state != NULL) {
            link_state->hal_error_count++;
            link_state->last_result = UNITREE_MOTOR_LINK_HAL_ERROR;
        }
        unitree_unlock_uart(huart);
        return UNITREE_MOTOR_LINK_HAL_ERROR;
    }

    result = unitree_receive_frame_locked(huart, rx_data, timeout_ms);
    if (result != UNITREE_MOTOR_LINK_OK) {
        unitree_unlock_uart(huart);
        return result;
    }

    if (rx_data->motor_id != motor_id) {
        link_state = unitree_get_link_state_mutable(huart);
        if (link_state != NULL) {
            link_state->id_mismatch_count++;
            link_state->last_result = UNITREE_MOTOR_LINK_ID_MISMATCH;
        }
        unitree_unlock_uart(huart);
        return UNITREE_MOTOR_LINK_ID_MISMATCH;
    }

    unitree_unlock_uart(huart);
    return UNITREE_MOTOR_LINK_OK;
}

const UnitreeMotorLinkState_t *UnitreeMotor_GetLinkState(UART_HandleTypeDef *huart)
{
    return unitree_get_link_state_mutable(huart);
}

void UnitreeMotor_ResetLinkState(UART_HandleTypeDef *huart)
{
    UnitreeMotorLinkState_t *link_state = unitree_get_link_state_mutable(huart);

    if (link_state != NULL) {
        memset(link_state, 0, sizeof(*link_state));
    }
}

UnitreeMotorLinkResult_t UnitreeMotor_Control(UART_HandleTypeDef *huart, uint8_t motor_id, uint8_t mode,
                                              float torque, float speed, float position, float kp, float kd)
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

    return UnitreeMotor_SendCmd(huart, &cmd);
}

UnitreeMotorLinkResult_t UnitreeMotor_Stop(UART_HandleTypeDef *huart, uint8_t motor_id)
{
    return UnitreeMotor_Control(huart, motor_id, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

UnitreeMotorLinkResult_t UnitreeMotor_StopNoWait(UART_HandleTypeDef *huart, uint8_t motor_id)
{
    MotorCmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.id = motor_id;
    cmd.mode = 0U;

    return unitree_send_cmd_no_wait(huart, &cmd);
}

UnitreeMotorLinkResult_t UnitreeMotor_SetPosition(UART_HandleTypeDef *huart, uint8_t motor_id, float position,
                                                  float kp, float kd)
{
    return UnitreeMotor_Control(huart, motor_id, 1, 0.0f, 0.0f, position, kp, kd);
}

UnitreeMotorLinkResult_t UnitreeMotor_SetSpeed(UART_HandleTypeDef *huart, uint8_t motor_id, float speed, float kd)
{
    return UnitreeMotor_Control(huart, motor_id, 1, 0.0f, speed, 0.0f, 0.0f, kd);
}

UnitreeMotorLinkResult_t UnitreeMotor_SetTorque(UART_HandleTypeDef *huart, uint8_t motor_id, float torque)
{
    return UnitreeMotor_Control(huart, motor_id, 1, torque, 0.0f, 0.0f, 0.0f, 0.0f);
}
