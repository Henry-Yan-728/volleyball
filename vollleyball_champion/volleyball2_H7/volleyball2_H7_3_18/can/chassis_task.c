#include "chassis_task.h"

#include "control.h"
#include "fdcan_bsp.h"
#include "vesc.h"

#include <math.h>
#include <stdint.h>

volatile float vx = 0.0f;
volatile float vy = 0.0f;
volatile float vr = 0.0f;

extern FDCAN_HandleTypeDef hfdcan3;

static uint32_t last_cmd_time = 0U;

static uint8_t Chassis_Should_AutoCenter(float target_vx, float target_vy, float target_vr)
{
    return (fabsf(target_vx) < CHASSIS_AUTO_CENTER_LINEAR_THRESHOLD) &&
           (fabsf(target_vy) < CHASSIS_AUTO_CENTER_LINEAR_THRESHOLD) &&
           (fabsf(target_vr) < CHASSIS_AUTO_CENTER_ANGULAR_THRESHOLD);
}

static float Calc_Steering_Angle(float x, float y)
{
    if ((fabsf(x) < 0.001f) && (fabsf(y) < 0.001f)) return 0.0f;

    float theta = atan2f(y, x);
    float angle_deg = 90.0f - theta * 180.0f / 3.1415926f;

    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;

    return angle_deg;
}

static float apply_min_abs(float value, float min_abs)
{
    if (fabsf(value) >= min_abs) return value;
    return (value >= 0.0f) ? min_abs : -min_abs;
}

static int16_t unpack_i16_be(const uint8_t *src)
{
    uint16_t raw = ((uint16_t)src[0] << 8) | (uint16_t)src[1];
    return (int16_t)raw;
}

static uint8_t calc_checksum(const uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; ++i) {
        sum = (uint8_t)(sum + data[i]);
    }
    return sum;
}

void Chassis_Stop(void)
{
    vx = 0.0f;
    vy = 0.0f;
    vr = 0.0f;
    Chassis_Update(0.0f, 0.0f, 0.0f);
}

void Chassis_Update(float target_vx, float target_vy, float target_vr)
{
    if (Chassis_Should_AutoCenter(target_vx, target_vy, target_vr)) {
        Steering_Wheel_AutoCenter(0U, CHASSIS_AUTO_CENTER_ANGLE_DEG);
        Steering_Wheel_AutoCenter(1U, CHASSIS_AUTO_CENTER_ANGLE_DEG);
        Steering_Wheel_AutoCenter(2U, CHASSIS_AUTO_CENTER_ANGLE_DEG);
        return;
    }

    float vr_r = target_vr * CHASSIS_RADIUS;

    float vx_1 = target_vx - vr_r;
    float vy_1 = target_vy;

    float vx_2 = target_vx + 0.5f * vr_r;
    float vy_2 = target_vy + 0.866f * vr_r;

    float vx_3 = target_vx + 0.5f * vr_r;
    float vy_3 = target_vy - 0.866f * vr_r;

    float spd_1 = sqrtf(vx_1 * vx_1 + vy_1 * vy_1);
    float spd_2 = sqrtf(vx_2 * vx_2 + vy_2 * vy_2);
    float spd_3 = sqrtf(vx_3 * vx_3 + vy_3 * vy_3);

    if ((fabsf(target_vx) < CHASSIS_AUTO_CENTER_LINEAR_THRESHOLD) &&
        (fabsf(target_vy) < CHASSIS_AUTO_CENTER_LINEAR_THRESHOLD) &&
        (fabsf(target_vr) >= CHASSIS_AUTO_CENTER_ANGULAR_THRESHOLD)) {
        spd_1 = apply_min_abs(spd_1, CHASSIS_SPIN_MIN_WHEEL_SPEED);
        spd_2 = apply_min_abs(spd_2, CHASSIS_SPIN_MIN_WHEEL_SPEED);
        spd_3 = apply_min_abs(spd_3, CHASSIS_SPIN_MIN_WHEEL_SPEED);
    }

    float ang_1 = -Calc_Steering_Angle(vx_1, vy_1);
    float ang_2 = -Calc_Steering_Angle(vx_2, vy_2);
    float ang_3 = -Calc_Steering_Angle(vx_3, vy_3);

    /* keep original 60-degree mechanical offsets */
    Steering_Wheel_Control(0U, spd_1, ang_1);
    Steering_Wheel_Control(1U, spd_2, ang_2 - 60.0f);
    Steering_Wheel_Control(2U, spd_3, ang_3 + 60.0f);
}

void Chassis_Task_Loop(void)
{
    uint32_t current_time = HAL_GetTick();

    if ((current_time - last_cmd_time) > CHASSIS_CMD_TIMEOUT_MS) {
        Chassis_Stop();
        return;
    }

    Chassis_Update(vx, vy, vr);
}

static void Chassis_Command_Callback(void *instance_ptr, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    (void)instance_ptr;

    if ((rx_header == NULL) || (rx_data == NULL)) return;
    if (rx_header->DataLength != FDCAN_DLC_BYTES_8) return;
    if (rx_data[7] != calc_checksum(rx_data, 7U)) return;

    vx = -(float)unpack_i16_be(&rx_data[0]) / CHASSIS_CMD_VEL_SCALE;
    vy = -(float)unpack_i16_be(&rx_data[2]) / CHASSIS_CMD_VEL_SCALE;
    vr = -(float)unpack_i16_be(&rx_data[4]) / CHASSIS_CMD_VEL_SCALE;

    last_cmd_time = HAL_GetTick();
}

void Chassis_Init(void)
{
    FDCAN_Dispatch_t chassis;

    chassis.id_type = FDCAN_STANDARD_ID;
    chassis.id = CHASSIS_CMD_CAN_ID;
    chassis.instance_ptr = NULL;
    chassis.handler = Chassis_Command_Callback;

    fdcan_bsp_register(&chassis, &hfdcan3);

    Vesc_init();
    last_cmd_time = HAL_GetTick();
    vx = 0.0f;
    vy = 0.0f;
    vr = 0.0f;
}
