#include "vesc.h"

#include "fdcan_bsp.h"

/* User-required mapping: VESC control is on CAN1. */
static FDCAN_HandleTypeDef *vesc_hfdcan_map[vesc_motor_nums] = {
    &hfdcan1,
    &hfdcan1,
    &hfdcan1,
    &hfdcan1
};

union s32_to_u8 vesc_content_transform[vesc_motor_nums] = {0};
static int vesc_motor_poles_s[vesc_motor_nums] = {12, 12, 12, 12};
static int motor_speed_s[vesc_motor_nums] = {0};

static void Vesc_Send_Msg(uint8_t motor_id, uint32_t frame_id, uint8_t *data);

void Vesc_init(void)
{
    Vesc_speed_control_init();
}

void Change_vesc_speed(int motor_id, int target_spd)
{
    if ((motor_id < 0) || (motor_id >= vesc_motor_nums)) return;
    motor_speed_s[motor_id] = target_spd;
}

void Com2vesc(uint32_t motor_id)
{
    if (motor_id >= vesc_motor_nums) return;

    uint32_t vesc_speed_id = motor_id | (CAN_PACKET_SET_RPM << 8);
    int32_t erpm = motor_speed_s[motor_id] * vesc_motor_poles_s[motor_id];

    vesc_content_transform[motor_id].u8_data[0] = (uint8_t)(erpm >> 24);
    vesc_content_transform[motor_id].u8_data[1] = (uint8_t)(erpm >> 16);
    vesc_content_transform[motor_id].u8_data[2] = (uint8_t)(erpm >> 8);
    vesc_content_transform[motor_id].u8_data[3] = (uint8_t)(erpm);

    Vesc_Send_Msg((uint8_t)motor_id, vesc_speed_id, vesc_content_transform[motor_id].u8_data);
}

void Vesc_speed_control_init(void)
{
    for (int i = 0; i < vesc_motor_nums; i++) {
        motor_speed_s[i] = 0;
    }
}

static void Vesc_Send_Msg(uint8_t motor_id, uint32_t frame_id, uint8_t *data)
{
    if (motor_id >= vesc_motor_nums) return;

    FDCAN_HandleTypeDef *hfdcan = vesc_hfdcan_map[motor_id];
    if (hfdcan == NULL) return;

    FDCAN_TxHeaderTypeDef tx_message;
    tx_message.Identifier = frame_id;
    tx_message.IdType = FDCAN_EXTENDED_ID;
    tx_message.TxFrameType = FDCAN_DATA_FRAME;
    tx_message.DataLength = FDCAN_DLC_BYTES_4;
    tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_message.BitRateSwitch = FDCAN_BRS_OFF;
    tx_message.FDFormat = FDCAN_CLASSIC_CAN;
    tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_message.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0) {
        (void)HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_message, data);
    }
}
