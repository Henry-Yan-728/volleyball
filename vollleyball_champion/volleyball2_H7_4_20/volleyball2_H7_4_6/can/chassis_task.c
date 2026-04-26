#include "chassis_task.h"
#include "fdcan_bsp.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define CHASSIS_CMD_DLC FDCAN_DLC_BYTES_8
#define CHASSIS_WHEEL_COUNT 3U
#define CHASSIS_STEERING_EPSILON 0.001f
#define CHASSIS_SQRT3_OVER_2 0.8660254f

typedef struct
{
    uint8_t tx_message[8];
} ChassisMessage_t;

volatile uint32_t g_chassis_fdcan1_tx_ok_count = 0U;
volatile uint32_t g_chassis_fdcan1_tx_wait_count = 0U;
volatile uint32_t g_chassis_fdcan1_tx_drop_count = 0U;

static uint8_t FDCAN1_Transmit(const uint8_t *tx_data, uint32_t id)
{
    FDCAN_TxHeaderTypeDef tx_header;

    if (tx_data == NULL)
    {
        g_chassis_fdcan1_tx_drop_count++;
        return 1U;
    }

    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = CHASSIS_CMD_DLC;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    for (uint8_t retry = 0U; retry < 3U; retry++)
    {
        if (fdcan_bsp_send(&hfdcan1, &tx_header, (uint8_t *)tx_data) == 0U)
        {
            if (retry != 0U)
            {
                g_chassis_fdcan1_tx_wait_count++;
            }

            g_chassis_fdcan1_tx_ok_count++;
            return 0U;
        }
    }

    g_chassis_fdcan1_tx_drop_count++;
    return 1U;
}

static float Calc_Steering_Angle(float x, float y)
{
    if ((fabsf(x) < CHASSIS_STEERING_EPSILON) &&
        (fabsf(y) < CHASSIS_STEERING_EPSILON))
    {
        return 0.0f;
    }

    float angle_deg = 90.0f - (atan2f(y, x) * 180.0f / 3.1415926f);

    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

static uint32_t prv_float_to_u32(float value)
{
    uint32_t bits = 0U;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static void prv_pack_u32_be(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)((value >> 24) & 0xFFU);
    dst[1] = (uint8_t)((value >> 16) & 0xFFU);
    dst[2] = (uint8_t)((value >> 8) & 0xFFU);
    dst[3] = (uint8_t)(value & 0xFFU);
}

void Chassis_Init(void)
{
    Chassis_Stop();
}

void Chassis_Update(float vx, float vy, float vr)
{
    ChassisMessage_t tx_data[CHASSIS_WHEEL_COUNT];
    const uint32_t wheel_ids[CHASSIS_WHEEL_COUNT] = {
        CHASSIS_WHEEL_1_CAN_ID,
        CHASSIS_WHEEL_2_CAN_ID,
        CHASSIS_WHEEL_3_CAN_ID
    };
    const float vr_r = vr * CHASSIS_RADIUS;

    const float vx_1 = vx - vr_r;
    const float vy_1 = vy;

    const float vx_2 = vx + 0.5f * vr_r;
    const float vy_2 = vy + CHASSIS_SQRT3_OVER_2 * vr_r;

    const float vx_3 = vx + 0.5f * vr_r;
    const float vy_3 = vy - CHASSIS_SQRT3_OVER_2 * vr_r;

    const float speeds[CHASSIS_WHEEL_COUNT] = {
        sqrtf((vx_1 * vx_1) + (vy_1 * vy_1)),
        sqrtf((vx_2 * vx_2) + (vy_2 * vy_2)),
        sqrtf((vx_3 * vx_3) + (vy_3 * vy_3))
    };
    const float angles[CHASSIS_WHEEL_COUNT] = {
        Calc_Steering_Angle(vx_1, vy_1),
        Calc_Steering_Angle(vx_2, vy_2),
        Calc_Steering_Angle(vx_3, vy_3)
    };

    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        prv_pack_u32_be(&tx_data[i].tx_message[0], prv_float_to_u32(speeds[i]));
        prv_pack_u32_be(&tx_data[i].tx_message[4], prv_float_to_u32(angles[i]));
    }

    static uint8_t next_tx_index = 0U;

    for (uint8_t sent = 0U; sent < CHASSIS_WHEEL_COUNT; sent++)
    {
        uint8_t index = (uint8_t)((next_tx_index + sent) % CHASSIS_WHEEL_COUNT);

        if (FDCAN1_Transmit(tx_data[index].tx_message, wheel_ids[index]) != 0U)
        {
            break;
        }

        next_tx_index = (uint8_t)((index + 1U) % CHASSIS_WHEEL_COUNT);
    }
}

void Chassis_Stop(void)
{
    Chassis_Update(0.0f, 0.0f, 0.0f);
}
