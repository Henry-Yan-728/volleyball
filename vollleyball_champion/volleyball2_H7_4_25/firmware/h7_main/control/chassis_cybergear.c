/**
  ******************************************************************************
  * @file    : chassis_cybergear.c
  * @brief   : CyberGear control wrapper
  ******************************************************************************
  */

#include "chassis_cybergear.h"
#include "trajectory_planner.h"
#include "FreeRTOS.h"
#include "task.h"

#define INIT_DELAY_MS   (10U)

static CyberGear_Motor_Instance *s_motor = NULL;

static float prv_clamp_output_angle_deg(float output_angle_deg)
{
    if (output_angle_deg > CHASSIS_CYBERGEAR_MAX_ANGLE_DEG)
    {
        return CHASSIS_CYBERGEAR_MAX_ANGLE_DEG;
    }
    if (output_angle_deg < CHASSIS_CYBERGEAR_MIN_ANGLE_DEG)
    {
        return CHASSIS_CYBERGEAR_MIN_ANGLE_DEG;
    }
    return output_angle_deg;
}

int8_t chassis_cybergear_init(uint8_t motor_index)
{
    s_motor = cybergear_motor_get_instance(motor_index);
    if (s_motor == NULL)
    {
        return -1;
    }

    cybergear_motor_stop(s_motor);
    vTaskDelay(pdMS_TO_TICKS(INIT_DELAY_MS));

    cybergear_motor_set_mode(s_motor, MOTOR_CONTROL_MODE_POSITION);
    vTaskDelay(pdMS_TO_TICKS(INIT_DELAY_MS));

    cybergear_motor_set_zero_position(s_motor);
    s_motor->measure.angle = 0.0f;
    vTaskDelay(pdMS_TO_TICKS(INIT_DELAY_MS));

    cybergear_motor_enable(s_motor);
    vTaskDelay(pdMS_TO_TICKS(INIT_DELAY_MS));

    if (trajectory_planner_start(s_motor) != 0)
    {
        cybergear_motor_stop(s_motor);
        s_motor = NULL;
        return -1;
    }

    return 0;
}

int8_t chassis_cybergear_attach_passive(uint8_t motor_index)
{
    s_motor = cybergear_motor_get_instance(motor_index);
    if (s_motor == NULL)
    {
        return -1;
    }

    cybergear_motor_stop(s_motor);
    vTaskDelay(pdMS_TO_TICKS(INIT_DELAY_MS));
    return 0;
}

void chassis_control(float angle_deg)
{
    float target_rad;

    if (s_motor == NULL)
    {
        return;
    }

    angle_deg = prv_clamp_output_angle_deg(angle_deg);
    target_rad = cybergear_motor_degree2rad(angle_deg * CHASSIS_CYBERGEAR_DIRECTION_SIGN);
    trajectory_planner_set_target(target_rad);
}

void chassis_control_speed(float speed_rad_s)
{
    if (s_motor == NULL)
    {
        return;
    }

    if (speed_rad_s > 30.0f)
    {
        speed_rad_s = 30.0f;
    }
    if (speed_rad_s < -30.0f)
    {
        speed_rad_s = -30.0f;
    }

    cybergear_motor_set_speed(s_motor, speed_rad_s * CHASSIS_CYBERGEAR_DIRECTION_SIGN);
}

void chassis_cybergear_stop(void)
{
    if (s_motor == NULL)
    {
        return;
    }

    trajectory_planner_set_target(s_motor->measure.angle);
    vTaskDelay(pdMS_TO_TICKS(INIT_DELAY_MS));
    cybergear_motor_stop(s_motor);
}

float chassis_get_angle_rad(void)
{
    if (s_motor == NULL)
    {
        return 0.0f;
    }
    return s_motor->measure.angle * CHASSIS_CYBERGEAR_DIRECTION_SIGN;
}

void chassis_request_angle_feedback(void)
{
    if (s_motor == NULL)
    {
        return;
    }

    cybergear_motor_request_parameter(s_motor, PARAM_MECH_POS);
}
