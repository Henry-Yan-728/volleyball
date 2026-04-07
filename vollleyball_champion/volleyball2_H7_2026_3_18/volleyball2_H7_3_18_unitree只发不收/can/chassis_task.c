#include "chassis_task.h"
#include "fdcan_bsp.h"

#include <stdint.h>

extern FDCAN_HandleTypeDef hfdcan1;

#define CHASSIS_CMD_DLC FDCAN_DLC_BYTES_8

static int16_t float_to_scaled_i16(float value)
{
    float scaled = value * CHASSIS_CMD_VEL_SCALE;
    if (scaled > 32767.0f) return INT16_MAX;
    if (scaled < -32768.0f) return INT16_MIN;
    return (int16_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static void pack_i16_be(uint8_t *dst, int16_t value)
{
    dst[0] = (uint8_t)((uint16_t)value >> 8);
    dst[1] = (uint8_t)((uint16_t)value & 0xFFU);
}

static uint8_t calc_checksum(const uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; ++i) {
        sum = (uint8_t)(sum + data[i]);
    }
    return sum;
}

static uint8_t FDCAN1_Transmit(const uint8_t *tx_data, uint32_t id)
{
    FDCAN_TxHeaderTypeDef tx_header;

    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = CHASSIS_CMD_DLC;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, (uint8_t *)tx_data) != HAL_OK) {
        return 1;
    }

    return 0;
}

void Chassis_Init(void)
{
    Chassis_Stop();
}

void Chassis_Update(float vx, float vy, float vr)
{
    uint8_t tx_message[8];

    pack_i16_be(&tx_message[0], float_to_scaled_i16(vx));
    pack_i16_be(&tx_message[2], float_to_scaled_i16(vy));
    pack_i16_be(&tx_message[4], float_to_scaled_i16(vr));

    tx_message[6] = 0; // reserved
    tx_message[7] = calc_checksum(tx_message, 7);

    (void)FDCAN1_Transmit(tx_message, CHASSIS_CMD_CAN_ID);
}

void Chassis_Stop(void)
{
    Chassis_Update(0.0f, 0.0f, 0.0f);
}
