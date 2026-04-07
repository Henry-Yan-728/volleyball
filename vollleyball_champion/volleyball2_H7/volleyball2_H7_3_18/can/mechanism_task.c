#include "mechanism_task.h"

#include <math.h>

#include "Pan_Tilt_control.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "dji_motor.h"
#include "task.h"
#include "unitree_motor.h"

#ifndef M_PI
#define M_PI 3.1415926535f
#endif

#define DEG_TO_RAD_FACTOR (M_PI / 180.0f)
#define UNITREE_TARGET_COUNT 4U

typedef struct {
    float position;
    float kp;
    float kd;
} UnitreeTarget_t;

Virtual_Axis_t v_axis = {0.0f, 0.0f, 0.1f, 0U};

float current_yaw_speed = 0.0f;
float current_pitch_speed = 0.0f;

static const uint8_t s_unitree_motor_ids[UNITREE_TARGET_COUNT] = {
    UNITREE_ID_CUSHION_1,
    UNITREE_ID_CUSHION_2,
    UNITREE_ID_CUSHION_3,
    UNITREE_ID_SERVE
};

static UnitreeTarget_t s_unitree_targets[UNITREE_TARGET_COUNT];
static uint8_t s_unitree_targets_ready = 0U;

static float convert_deg_to_unitree_rad(float deg)
{
    return deg * DEG_TO_RAD_FACTOR * UNITREE_REDUCTION_RATIO;
}

static void mechanism_set_unitree_defaults(void)
{
    for (uint8_t i = 0U; i < UNITREE_TARGET_COUNT; i++) {
        s_unitree_targets[i].position = 0.0f;
        s_unitree_targets[i].kp = 0.0f;
        s_unitree_targets[i].kd = 0.1f;
    }
}

static void mechanism_push_unitree_targets(void)
{
    float positions[UNITREE_TARGET_COUNT];
    float kps[UNITREE_TARGET_COUNT];
    float kds[UNITREE_TARGET_COUNT];

    if (s_unitree_targets_ready == 0U) {
        return;
    }

    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < UNITREE_TARGET_COUNT; i++) {
        positions[i] = s_unitree_targets[i].position;
        kps[i] = s_unitree_targets[i].kp;
        kds[i] = s_unitree_targets[i].kd;
    }
    taskEXIT_CRITICAL();

    UnitreeMotor_SetPositionMulti(s_unitree_motor_ids,
                                  positions,
                                  kps,
                                  kds,
                                  UNITREE_TARGET_COUNT);
}

void Mechanism_Init(void)
{
    dji_motors_init();
    gimbal_control_init();
    UnitreeMotor_Init();

    mechanism_set_unitree_defaults();
    s_unitree_targets_ready = 1U;

    HAL_Delay(100);
}

void Mechanism_Cushion_SetAngle(float angle_deg, float kp)
{
    const float target_rad = convert_deg_to_unitree_rad(angle_deg);
    const float kd_default = 0.2f;

    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < 3U; i++) {
        s_unitree_targets[i].position = target_rad;
        s_unitree_targets[i].kp = kp;
        s_unitree_targets[i].kd = kd_default;
    }
    taskEXIT_CRITICAL();
}

void Mechanism_Zero(void)
{
    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < UNITREE_TARGET_COUNT; i++) {
        s_unitree_targets[i].position = 0.0f;
        s_unitree_targets[i].kp = 0.0f;
        s_unitree_targets[i].kd = 0.1f;
    }
    taskEXIT_CRITICAL();

    mechanism_push_unitree_targets();
}

void Mechanism_Serve_SetAngle(float angle_deg)
{
    const float target_rad = convert_deg_to_unitree_rad(angle_deg);

    taskENTER_CRITICAL();
    s_unitree_targets[3].position = target_rad;
    s_unitree_targets[3].kp = 1.0f;
    s_unitree_targets[3].kd = 0.1f;
    taskEXIT_CRITICAL();
}

void Mechanism_Dian_Pitch_SetAngle(float angle_deg)
{
    v_axis.target_angle = angle_deg;
}

void Update_Virtual_Axis(void)
{
    const uint32_t now = HAL_GetTick();

    if (now - v_axis.last_tick < 1U) {
        return;
    }
    v_axis.last_tick = now;

    const float error = v_axis.target_angle - v_axis.current_angle;

    if (fabsf(error) <= v_axis.velocity_deg_ms) {
        v_axis.current_angle = v_axis.target_angle;
    } else {
        if (error > 0.0f) {
            v_axis.current_angle += v_axis.velocity_deg_ms;
        } else {
            v_axis.current_angle -= v_axis.velocity_deg_ms;
        }
    }

    DJI_Motor_Instance *m_right = dji_motor_get_instance(0U);
    DJI_Motor_Instance *m_left = dji_motor_get_instance(1U);

    if ((m_right != NULL) && (m_left != NULL)) {
        const int32_t encoder_pos = dji_angle_to_encoder(v_axis.current_angle, EXTERNAL_MECHANISM_RATIO);

        dji_motor_set_location(m_left, encoder_pos);
        dji_motor_set_location(m_right, -encoder_pos);
    }
}

void Mechanism_Loop_1ms(void)
{
    mechanism_push_unitree_targets();
    gimbal_set_speed(current_yaw_speed, current_pitch_speed);
}
