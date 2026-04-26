#include "unitree_driver.h"
#include "fdcan.h"
#include "fdcan_bsp.h"

#include <math.h>
#include <stdio.h>

extern FDCAN_HandleTypeDef hfdcan3;

#define UNITREE_MODULE_ID     3U
#define UNITREE_MOTOR_COUNT   4U
#define UNITREE_BROADCAST_ID  15U
#define UNITREE_RX_ID_MASK    0x00000F00UL
#define UNITREE_FEEDBACK_VALID_MS 10U

typedef struct
{
    uint8_t motor_id;
    volatile uint32_t update_counter;
    volatile uint32_t update_tick;
    volatile int8_t temperature;
    volatile float torque;
    volatile float speed;
    volatile float position;
} UnitreeFeedback_t;

int8_t motor_temperature[3];
float actual_torque[3], actual_speed[3], actual_position[3];

static uint8_t g_unitree_driver_initialized = 0U;
static FDCAN_Dispatch_t g_unitree_dispatch[UNITREE_MOTOR_COUNT];
static UnitreeFeedback_t g_unitree_feedback[UNITREE_MOTOR_COUNT];

static UnitreeFeedback_t *unitree_feedback_get(uint32_t motorId);
static void unitree_feedback_handler(void *instance_ptr,
                                     FDCAN_RxHeaderTypeDef *rx_header,
                                     uint8_t rx_data[64]);
static void unitree_driver_init(void);
static HAL_StatusTypeDef unitree_send_control_raw(uint32_t moduleId,
                                                  uint32_t motorId,
                                                  float torque,
                                                  float speed,
                                                  float position);

static UnitreeFeedback_t *unitree_feedback_get(uint32_t motorId)
{
    if (motorId >= UNITREE_MOTOR_COUNT)
    {
        return NULL;
    }

    return &g_unitree_feedback[motorId];
}

static void unitree_feedback_handler(void *instance_ptr,
                                     FDCAN_RxHeaderTypeDef *rx_header,
                                     uint8_t rx_data[64])
{
    UnitreeFeedback_t *feedback = (UnitreeFeedback_t *)instance_ptr;
    uint16_t temp_torque;
    uint16_t temp_speed;
    uint32_t temp_position;

    if ((feedback == NULL) || (rx_header == NULL) || (rx_data == NULL))
    {
        return;
    }

    if (rx_header->DataLength < FDCAN_DLC_BYTES_8)
    {
        return;
    }

    temp_torque = (uint16_t)(rx_data[6] | (rx_data[7] << 8));
    temp_speed = (uint16_t)(rx_data[4] | (rx_data[5] << 8));
    temp_position = (uint32_t)(rx_data[0] |
                               (rx_data[1] << 8) |
                               (rx_data[2] << 16) |
                               (rx_data[3] << 24));

    feedback->temperature = (int8_t)(rx_header->Identifier & 0xFFU);
    feedback->torque = (float)temp_torque / 256.0f;
    feedback->speed = (float)temp_speed / 256.0f * (2.0f * (float)M_PI);
    feedback->position = (float)temp_position / 32768.0f * (2.0f * (float)M_PI);
    feedback->update_tick = HAL_GetTick();
    feedback->update_counter++;
}

static void unitree_driver_init(void)
{
    uint32_t i;

    if (g_unitree_driver_initialized != 0U)
    {
        return;
    }

    __disable_irq();
    if (g_unitree_driver_initialized != 0U)
    {
        __enable_irq();
        return;
    }

    for (i = 0U; i < UNITREE_MOTOR_COUNT; ++i)
    {
        g_unitree_feedback[i].motor_id = (uint8_t)i;
        g_unitree_feedback[i].update_counter = 0U;
        g_unitree_feedback[i].update_tick = 0U;
        g_unitree_feedback[i].temperature = 0;
        g_unitree_feedback[i].torque = 0.0f;
        g_unitree_feedback[i].speed = 0.0f;
        g_unitree_feedback[i].position = 0.0f;

        g_unitree_dispatch[i].id_type = FDCAN_EXTENDED_ID;
        g_unitree_dispatch[i].id = ((uint32_t)i & 0x0FU) << 8;
        g_unitree_dispatch[i].mask = UNITREE_RX_ID_MASK;
        g_unitree_dispatch[i].instance_ptr = &g_unitree_feedback[i];
        g_unitree_dispatch[i].handler = unitree_feedback_handler;

        fdcan_bsp_register(&g_unitree_dispatch[i], &hfdcan3);
    }

    g_unitree_driver_initialized = 1U;
    __enable_irq();
}

static HAL_StatusTypeDef unitree_send_control_raw(uint32_t moduleId,
                                                  uint32_t motorId,
                                                  float torque,
                                                  float speed,
                                                  float position)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    int32_t Pset;
    int16_t Wset;
    int16_t Tset;

    TxHeader.Identifier = ((moduleId & 0x3U) << 27) |
                          (0U << 26) |
                          (0U << 24) |
                          (10U << 16) |
                          ((motorId & 0xFU) << 8) |
                          (1U << 12) |
                          (0U << 15) |
                          0U;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    Pset = (int32_t)(position * 32768.0f / (2.0f * (float)M_PI));
    Wset = (int16_t)(speed * 256.0f / (2.0f * (float)M_PI));
    Tset = (int16_t)(torque * 256.0f);

    TxData[0] = (uint8_t)(Pset & 0xFF);
    TxData[1] = (uint8_t)((Pset >> 8) & 0xFF);
    TxData[2] = (uint8_t)((Pset >> 16) & 0xFF);
    TxData[3] = (uint8_t)((Pset >> 24) & 0xFF);
    TxData[4] = (uint8_t)(Wset & 0xFF);
    TxData[5] = (uint8_t)((Wset >> 8) & 0xFF);
    TxData[6] = (uint8_t)(Tset & 0xFF);
    TxData[7] = (uint8_t)((Tset >> 8) & 0xFF);

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3) == 0U)
    {
        return HAL_BUSY;
    }

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData);
}

HAL_StatusTypeDef sendCANControlMotor(uint32_t moduleId,
                                      uint32_t motorId,
                                      float torque,
                                      float speed,
                                      float position,
                                      int8_t *motor_temperature_out,
                                      float *actual_torque_out,
                                      float *actual_speed_out,
                                      float *actual_position_out)
{
    HAL_StatusTypeDef status;
    UnitreeFeedback_t *feedback;

    unitree_driver_init();

    feedback = unitree_feedback_get(motorId);
    if ((feedback == NULL) ||
        (motor_temperature_out == NULL) ||
        (actual_torque_out == NULL) ||
        (actual_speed_out == NULL) ||
        (actual_position_out == NULL))
    {
        return HAL_ERROR;
    }

    *motor_temperature_out = feedback->temperature;
    *actual_torque_out = feedback->torque;
    *actual_speed_out = feedback->speed;
    *actual_position_out = feedback->position;

    status = unitree_send_control_raw(moduleId, motorId, torque, speed, position);
    if (status != HAL_OK)
    {
        return status;
    }

    if ((feedback->update_tick != 0U) &&
        ((HAL_GetTick() - feedback->update_tick) <= UNITREE_FEEDBACK_VALID_MS))
    {
        *motor_temperature_out = feedback->temperature;
        *actual_torque_out = feedback->torque;
        *actual_speed_out = feedback->speed;
        *actual_position_out = feedback->position;
    }

    return HAL_OK;
}

HAL_StatusTypeDef sendCANSetMotorKK(uint32_t moduleId, uint32_t motorId, float Kpos, float Kspd)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8] = {0};
    uint16_t KspdSet;
    uint16_t KposSet;

    unitree_driver_init();

    TxHeader.Identifier = ((moduleId & 0x3U) << 27) |
                          (0U << 26) |
                          (0U << 24) |
                          (11U << 16) |
                          ((motorId & 0xFU) << 8) |
                          0U;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    KspdSet = (uint16_t)(Kspd * 1280.0f);
    KposSet = (uint16_t)(Kpos * 1280.0f);

    TxData[0] = (uint8_t)(KspdSet & 0xFF);
    TxData[1] = (uint8_t)((KspdSet >> 8) & 0xFF);
    TxData[2] = (uint8_t)(KposSet & 0xFF);
    TxData[3] = (uint8_t)((KposSet >> 8) & 0xFF);

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3) == 0U)
    {
        return HAL_BUSY;
    }

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, TxData);
}

void Unitree_Send_Cmd(uint32_t motorId, float torque, float speed, float position, float Kpos, float Kspd)
{
    int8_t motor_temperature0;
    float actual_torque0;
    float actual_speed0;
    float actual_position0;
    HAL_StatusTypeDef status;
    static float last_Kpos[UNITREE_MOTOR_COUNT] = {-1.0f, -1.0f, -1.0f, -1.0f};
    static float last_Kspd[UNITREE_MOTOR_COUNT] = {-1.0f, -1.0f, -1.0f, -1.0f};

    if (motorId >= UNITREE_MOTOR_COUNT)
    {
        return;
    }

    if ((last_Kpos[motorId] != Kpos) || (last_Kspd[motorId] != Kspd))
    {
        (void)sendCANSetMotorKK(UNITREE_MODULE_ID, motorId, Kpos, Kspd);
        last_Kpos[motorId] = Kpos;
        last_Kspd[motorId] = Kspd;
    }

    status = sendCANControlMotor(UNITREE_MODULE_ID,
                                 motorId,
                                 torque,
                                 speed,
                                 position,
                                 &motor_temperature0,
                                 &actual_torque0,
                                 &actual_speed0,
                                 &actual_position0);

    if ((status == HAL_OK) && (motorId < 3U))
    {
        motor_temperature[motorId] = motor_temperature0;
        actual_torque[motorId] = actual_torque0;
        actual_speed[motorId] = actual_speed0;
        actual_position[motorId] = actual_position0;
    }
}

HAL_StatusTypeDef Unitree_Send_Broadcast_Cmd(float torque, float speed, float position, float Kpos, float Kspd)
{
    static float last_Kpos = -1.0f;
    static float last_Kspd = -1.0f;

    unitree_driver_init();

    if ((last_Kpos != Kpos) || (last_Kspd != Kspd))
    {
        HAL_StatusTypeDef kk_status = sendCANSetMotorKK(UNITREE_MODULE_ID, UNITREE_BROADCAST_ID, Kpos, Kspd);
        if (kk_status != HAL_OK)
        {
            return kk_status;
        }

        last_Kpos = Kpos;
        last_Kspd = Kspd;
    }

    return unitree_send_control_raw(UNITREE_MODULE_ID,
                                    UNITREE_BROADCAST_ID,
                                    torque,
                                    speed,
                                    position);
}

void get_Unitree_pos(uint32_t motorId)
{
    int8_t motor_temperature0;
    float actual_torque0;
    float actual_speed0;
    float actual_position0;
    HAL_StatusTypeDef status;

    status = sendCANControlMotor(UNITREE_MODULE_ID,
                                 motorId,
                                 0.0f,
                                 0.0f,
                                 0.0f,
                                 &motor_temperature0,
                                 &actual_torque0,
                                 &actual_speed0,
                                 &actual_position0);

    if ((status == HAL_OK) && (motorId < 3U))
    {
        motor_temperature[motorId] = motor_temperature0;
        actual_torque[motorId] = actual_torque0;
        actual_speed[motorId] = actual_speed0;
        actual_position[motorId] = actual_position0;
    }
}
