/**
  ******************************************************************************
  * @file    : chassis_cybergear.c
  * @brief   : Xiaomi CyberGear chassis control wrapper
  ******************************************************************************
  */

#include "chassis_cybergear.h"
#include "trajectory_planner.h"
#include "FreeRTOS.h"
#include "task.h"

#define INIT_DELAY_MS               (10U)
#define FEEDBACK_RETRY_INTERVAL_MS  (5U)
#define FEEDBACK_TIMEOUT_MS         (120U)

static CyberGear_Motor_Instance *s_motor = NULL;
static CyberGear_Control_Mode_e s_runtime_mode = MOTOR_CONTROL_MODE_UNSET;
static uint8_t s_runtime_enabled = 0U;

static int8_t chassis_wait_mech_pos_feedback(uint32_t timeout_ms)
{
    uint32_t start_tick;
    uint32_t last_msg_tick;

    if (s_motor == NULL)
    {
        return -1;
    }

    start_tick = HAL_GetTick();
    last_msg_tick = s_motor->last_msg_time;

    while ((HAL_GetTick() - start_tick) < timeout_ms)
    {
        cybergear_motor_request_parameter(s_motor, PARAM_MECH_POS);
        vTaskDelay(pdMS_TO_TICKS(FEEDBACK_RETRY_INTERVAL_MS));

        if ((s_motor->last_msg_time != last_msg_tick) && (s_motor->is_online != 0U))
        {
            return 0;
        }
    }

    return -1;
}

static int8_t chassis_activate_mode(CyberGear_Control_Mode_e mode)
{
    if (s_motor == NULL)
    {
        return -1;
    }

    if ((s_runtime_mode == mode) && (s_runtime_enabled != 0U))
    {
        return 0;
    }

    cybergear_motor_set_mode(s_motor, mode);
    vTaskDelay(pdMS_TO_TICKS(INIT_DELAY_MS));

    cybergear_motor_enable(s_motor);
    vTaskDelay(pdMS_TO_TICKS(INIT_DELAY_MS));

    if (mode == MOTOR_CONTROL_MODE_POSITION)
    {
        (void)chassis_wait_mech_pos_feedback(FEEDBACK_TIMEOUT_MS);

        if (trajectory_planner_start(s_motor) != 0)
        {
            cybergear_motor_stop(s_motor);
            s_runtime_mode = MOTOR_CONTROL_MODE_UNSET;
            s_runtime_enabled = 0U;
            return -1;
        }
    }

    s_runtime_mode = mode;
    s_runtime_enabled = 1U;
    return 0;
}

int8_t chassis_cybergear_init(uint8_t motor_index)
{
    s_motor = cybergear_motor_get_instance(motor_index);
    if (s_motor == NULL)
    {
        return -1;
    }

    s_runtime_mode = MOTOR_CONTROL_MODE_UNSET;
    s_runtime_enabled = 0U;

    cybergear_motor_stop(s_motor);
    vTaskDelay(pdMS_TO_TICKS(INIT_DELAY_MS));

    (void)chassis_wait_mech_pos_feedback(FEEDBACK_TIMEOUT_MS);

    cybergear_motor_set_zero_position(s_motor);
    vTaskDelay(pdMS_TO_TICKS(INIT_DELAY_MS));

    (void)chassis_wait_mech_pos_feedback(FEEDBACK_TIMEOUT_MS);

    cybergear_motor_stop(s_motor);
    s_runtime_mode = MOTOR_CONTROL_MODE_UNSET;
    s_runtime_enabled = 0U;

    return 0;
}

void chassis_control(float angle_deg)
{
    float target_rad;

    if (s_motor == NULL)
    {
        return;
    }

    if (chassis_activate_mode(MOTOR_CONTROL_MODE_POSITION) != 0)
    {
        return;
    }

    target_rad = cybergear_motor_degree2rad(angle_deg);
    trajectory_planner_set_target(target_rad);
}

void chassis_control_speed(float speed_rad_s)
{
    if (s_motor == NULL)
    {
        return;
    }

    if (chassis_activate_mode(MOTOR_CONTROL_MODE_SPEED) != 0)
    {
        return;
    }

    if (speed_rad_s > 30.0f) speed_rad_s = 30.0f;
    if (speed_rad_s < -30.0f) speed_rad_s = -30.0f;

    cybergear_motor_set_speed(s_motor, speed_rad_s);
}

void chassis_cybergear_stop(void)
{
    if (s_motor == NULL)
    {
        return;
    }

    if (s_runtime_mode == MOTOR_CONTROL_MODE_POSITION)
    {
        trajectory_planner_set_target(s_motor->measure.angle);
        vTaskDelay(pdMS_TO_TICKS(INIT_DELAY_MS));
    }

    cybergear_motor_stop(s_motor);
    s_runtime_mode = MOTOR_CONTROL_MODE_UNSET;
    s_runtime_enabled = 0U;
}

float chassis_get_angle_rad(void)
{
    if (s_motor == NULL)
    {
        return 0.0f;
    }

    return s_motor->measure.angle;
}
