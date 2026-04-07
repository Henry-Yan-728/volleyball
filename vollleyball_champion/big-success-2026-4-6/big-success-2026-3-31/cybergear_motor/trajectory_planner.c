/**
  ******************************************************************************
  * @file           : trajectory_planner.c
  * @brief          : CyberGear position trajectory planner
  ******************************************************************************
  */

#include "trajectory_planner.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cybergear_motor.h"

#define PLANNER_CONTROL_PERIOD_MS   (10)
#define PLANNER_TASK_STACK_SIZE     (256)
#define PLANNER_TASK_PRIORITY       (tskIDLE_PRIORITY + 32)

#define PLANNER_MAX_VELOCITY        (V_MAX)
#define PLANNER_MAX_ACCELERATION    (30.00f)
#define MOTOR_INTERNAL_SPD_LIMIT    (V_MAX)

static TaskHandle_t xPlannerTaskHandle = NULL;
static SemaphoreHandle_t g_target_mutex = NULL;

static float g_final_target_pos = 0.0f;
static CyberGear_Motor_Instance* g_motor = NULL;

static float g_current_ramped_pos = 0.0f;
static float g_current_ramped_vel = 0.0f;

static void prv_planner_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(PLANNER_CONTROL_PERIOD_MS);
    const float dt = (float)PLANNER_CONTROL_PERIOD_MS / 1000.0f;
    const float accel_step = PLANNER_MAX_ACCELERATION * dt;
    const float max_vel = PLANNER_MAX_VELOCITY;

    (void)pvParameters;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        float local_target;
        float pos_error;
        float vel_sign;
        float stop_dist;
        float target_vel_sign;

        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        xSemaphoreTake(g_target_mutex, portMAX_DELAY);
        local_target = g_final_target_pos;
        xSemaphoreGive(g_target_mutex);

        pos_error = local_target - g_current_ramped_pos;
        vel_sign = (g_current_ramped_vel > 0.0f) ? 1.0f : -1.0f;
        stop_dist = (g_current_ramped_vel * g_current_ramped_vel) / (2.0f * PLANNER_MAX_ACCELERATION);

        if ((fabsf(pos_error) <= stop_dist) && ((pos_error * g_current_ramped_vel) >= 0.0f))
        {
            g_current_ramped_vel -= vel_sign * accel_step;

            if ((g_current_ramped_vel * vel_sign) < 0.0f)
            {
                g_current_ramped_vel = 0.0f;
            }
        }
        else
        {
            target_vel_sign = (pos_error > 0.0f) ? 1.0f : -1.0f;
            g_current_ramped_vel += target_vel_sign * accel_step;
        }

        if (g_current_ramped_vel > max_vel)
        {
            g_current_ramped_vel = max_vel;
        }
        else if (g_current_ramped_vel < -max_vel)
        {
            g_current_ramped_vel = -max_vel;
        }

        if ((g_current_ramped_vel != 0.0f) || (fabsf(pos_error) > 1e-4f))
        {
            g_current_ramped_pos += g_current_ramped_vel * dt;
        }

        cybergear_motor_set_position(g_motor, g_current_ramped_pos, MOTOR_INTERNAL_SPD_LIMIT);
    }
}

int8_t trajectory_planner_init(void)
{
    BaseType_t xReturned;

    g_target_mutex = xSemaphoreCreateMutex();
    if (g_target_mutex == NULL)
    {
        return -1;
    }

    xReturned = xTaskCreate(prv_planner_task,
                            "PlannerTask",
                            PLANNER_TASK_STACK_SIZE,
                            NULL,
                            PLANNER_TASK_PRIORITY,
                            &xPlannerTaskHandle);
    if (xReturned != pdPASS)
    {
        return -1;
    }

    if (xPlannerTaskHandle == NULL)
    {
        return -1;
    }

    vTaskSuspend(xPlannerTaskHandle);
    return 0;
}

int8_t trajectory_planner_start(CyberGear_Motor_Instance* motor)
{
    if ((motor == NULL) || (xPlannerTaskHandle == NULL) || (g_target_mutex == NULL))
    {
        return -1;
    }

    g_motor = motor;
    g_current_ramped_pos = g_motor->measure.angle;
    g_current_ramped_vel = 0.0f;

    xSemaphoreTake(g_target_mutex, portMAX_DELAY);
    g_final_target_pos = g_current_ramped_pos;
    xSemaphoreGive(g_target_mutex);

    vTaskResume(xPlannerTaskHandle);
    return 0;
}

void trajectory_planner_set_target(float target_position)
{
    if (g_target_mutex == NULL)
    {
        return;
    }

    xSemaphoreTake(g_target_mutex, portMAX_DELAY);
    g_final_target_pos = target_position;
    xSemaphoreGive(g_target_mutex);
}

