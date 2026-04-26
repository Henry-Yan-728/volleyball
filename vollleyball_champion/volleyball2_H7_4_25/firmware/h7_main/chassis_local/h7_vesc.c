#include "h7_vesc.h"

#include "fdcan_bsp.h"
#include "h7_vesc_rx.h"

#include <string.h>

static H7VescMotor_t h7_vesc_motors[H7_VESC_MOTOR_COUNT];
static uint32_t h7_vesc_startup_tick = 0U;

static FDCAN_HandleTypeDef * const h7_vesc_default_buses[H7_VESC_MOTOR_COUNT] = {
    &hfdcan1,
    &hfdcan1,
    &hfdcan1,
    &hfdcan1,
};

static const uint8_t h7_vesc_default_controller_ids[H7_VESC_MOTOR_COUNT] = {
    0U,
    1U,
    2U,
    3U,
};

static const uint8_t h7_vesc_default_motor_poles[H7_VESC_MOTOR_COUNT] = {
    12U,
    12U,
    12U,
    12U,
};

static void H7Vesc_ConfigureInstance(uint8_t index,
                                     FDCAN_HandleTypeDef *hfdcan,
                                     uint8_t controller_id,
                                     uint8_t motor_poles);

void H7Vesc_Init(void)
{
    h7_vesc_startup_tick = HAL_GetTick();
    memset(h7_vesc_motors, 0, sizeof(h7_vesc_motors));

    for (uint8_t i = 0U; i < H7_VESC_MOTOR_COUNT; ++i) {
        H7Vesc_ConfigureInstance(i,
                                 h7_vesc_default_buses[i],
                                 h7_vesc_default_controller_ids[i],
                                 h7_vesc_default_motor_poles[i]);
    }
}

H7VescMotor_t *H7Vesc_GetInstance(uint8_t motor_index)
{
    if (motor_index >= H7_VESC_MOTOR_COUNT) {
        return NULL;
    }

    return &h7_vesc_motors[motor_index];
}

H7VescMotor_t *H7Vesc_GetInstanceByCan(FDCAN_HandleTypeDef *hfdcan, uint8_t controller_id)
{
    for (uint8_t i = 0U; i < H7_VESC_MOTOR_COUNT; ++i) {
        if ((h7_vesc_motors[i].hfdcan == hfdcan) &&
            (h7_vesc_motors[i].controller_id == controller_id)) {
            return &h7_vesc_motors[i];
        }
    }

    return NULL;
}

void H7Vesc_SetTargetRpm(H7VescMotor_t *motor, int32_t target_rpm)
{
    if (motor == NULL) {
        return;
    }

    motor->target_rpm = target_rpm;
}

uint8_t H7Vesc_SendTarget(H7VescMotor_t *motor)
{
    FDCAN_TxHeaderTypeDef tx_header;
    int32_t target_erpm;

    if ((motor == NULL) || (motor->hfdcan == NULL)) {
        return 1U;
    }
    if (H7Vesc_IsTxReady() == 0U) {
        return 0U;
    }

    target_erpm = motor->target_rpm * (int32_t)motor->motor_poles;

    motor->tx_data[0] = (uint8_t)(((uint32_t)target_erpm >> 24) & 0xFFU);
    motor->tx_data[1] = (uint8_t)(((uint32_t)target_erpm >> 16) & 0xFFU);
    motor->tx_data[2] = (uint8_t)(((uint32_t)target_erpm >> 8) & 0xFFU);
    motor->tx_data[3] = (uint8_t)((uint32_t)target_erpm & 0xFFU);

    tx_header.Identifier = ((uint32_t)H7_CAN_PACKET_SET_RPM << 8) | motor->controller_id;
    tx_header.IdType = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_4;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    return fdcan_bsp_send(motor->hfdcan, &tx_header, motor->tx_data);
}

uint8_t H7Vesc_IsTxReady(void)
{
    return ((HAL_GetTick() - h7_vesc_startup_tick) >= H7_VESC_STARTUP_DELAY_MS) ? 1U : 0U;
}

void H7Vesc_UpdateFeedback(H7VescMotor_t *motor, int32_t feedback_erpm)
{
    if (motor == NULL) {
        return;
    }

    motor->last_feedback_erpm = feedback_erpm;
    if (motor->motor_poles != 0U) {
        motor->last_feedback_rpm = feedback_erpm / (int32_t)motor->motor_poles;
    } else {
        motor->last_feedback_rpm = 0;
    }
    motor->last_feedback_time = HAL_GetTick();
}

int32_t H7Vesc_GetFeedbackRpm(const H7VescMotor_t *motor)
{
    if (motor == NULL) {
        return 0;
    }

    return motor->last_feedback_rpm;
}

static void H7Vesc_ConfigureInstance(uint8_t index,
                                     FDCAN_HandleTypeDef *hfdcan,
                                     uint8_t controller_id,
                                     uint8_t motor_poles)
{
    H7VescMotor_t *motor;
    FDCAN_Dispatch_t dispatch_item;

    if (index >= H7_VESC_MOTOR_COUNT) {
        return;
    }

    motor = &h7_vesc_motors[index];
    memset(motor, 0, sizeof(*motor));

    motor->hfdcan = hfdcan;
    motor->controller_id = controller_id;
    motor->motor_poles = motor_poles;
    motor->target_rpm = 0;

    dispatch_item.id_type = FDCAN_EXTENDED_ID;
    dispatch_item.id = ((uint32_t)H7_VESC_STATUS_PACKET_ID << 8) | controller_id;
    dispatch_item.mask = 0x1FFFFFFFU;
    dispatch_item.instance_ptr = NULL;
    dispatch_item.handler = H7Vesc_RxMessageHandler;

    fdcan_bsp_register(&dispatch_item, hfdcan);
}
